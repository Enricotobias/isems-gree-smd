#include "ota.h"
#include "config.h"

#include "data.h"
#include "rtos_common.h"

// --- FIX: PINDAHKAN INCLUDE INI KE ATAS (GLOBAL SCOPE) ---
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
// ---------------------------------------------------------

#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>

static bool otaPendingValidation = false;
static unsigned long otaPendingSinceMs = 0;
static unsigned long otaLastValidationLogMs = 0;
static const unsigned long OTA_HEALTH_STABLE_MS = 60000UL;
static const uint32_t OTA_HEALTH_MIN_FREE_HEAP = 50000U;
static const unsigned long OTA_DOWNLOAD_INACTIVITY_MS = 15000UL;
static const unsigned long OTA_DOWNLOAD_TOTAL_TIMEOUT_MS = 10UL * 60UL * 1000UL;
static const unsigned long OTA_PROGRESS_PERSIST_INTERVAL_MS = 10000UL;

static bool otaPendingAckLoaded = false;
static bool otaPendingAckValid = false;
static String otaPendingAckStatus;
static String otaPendingAckDetail;
static unsigned long otaLastProgressPersistMs = 0;
static unsigned long otaLastPendingAckAttemptMs = 0;

static bool isValidMd5(const String& value) {
    if (value.length() != 32) return false;
    for (size_t i = 0; i < value.length(); ++i) {
        const char c = value[i];
        if (!((c >= '0' && c <= '9') ||
              (c >= 'a' && c <= 'f') ||
              (c >= 'A' && c <= 'F'))) {
            return false;
        }
    }
    return true;
}

static void loadPendingAck() {
    if (otaPendingAckLoaded) return;
    otaPendingAckLoaded = true;

    Preferences prefs;
    if (!prefs.begin("ota_guard", true)) return;
    otaPendingAckValid = prefs.getBool("ack_valid", false);
    otaPendingAckStatus = prefs.getString("ack_status", "");
    otaPendingAckDetail = prefs.getString("ack_detail", "");
    prefs.end();
}

void otaQueuePendingAck(const char* status, const String& detail, bool forceWrite) {
    const bool isProgress = strcmp(status, "progress") == 0;
    if (isProgress && !forceWrite &&
        millis() - otaLastProgressPersistMs < OTA_PROGRESS_PERSIST_INTERVAL_MS) {
        return;
    }

    otaPendingAckLoaded = true;
    otaPendingAckValid = true;
    otaPendingAckStatus = status;
    otaPendingAckDetail = detail;
    if (isProgress) otaLastProgressPersistMs = millis();

    Preferences prefs;
    if (!prefs.begin("ota_guard", false)) {
        Serial.println("[OTA] Failed to open pending ACK storage");
        return;
    }
    prefs.putString("ack_status", otaPendingAckStatus);
    prefs.putString("ack_detail", otaPendingAckDetail);
    prefs.putBool("ack_valid", true);
    prefs.end();
}

void otaClearPendingAck() {
    otaPendingAckLoaded = true;
    otaPendingAckValid = false;
    otaPendingAckStatus = "";
    otaPendingAckDetail = "";

    Preferences prefs;
    if (!prefs.begin("ota_guard", false)) return;
    prefs.remove("ack_status");
    prefs.remove("ack_detail");
    prefs.putBool("ack_valid", false);
    prefs.end();
}

static String normalizeVersion(String version) {
    version.toLowerCase();
    version.replace("isems_v", "");
    if (version.startsWith("v")) version.remove(0, 1);
    version.trim();
    return version;
}

static void setAwaitingHealthValidation(bool awaiting,
                                        const String& targetVersion = "") {
    Preferences prefs;
    if (!prefs.begin("ota_guard", false)) return;
    prefs.putBool("await_health", awaiting);
    if (awaiting) {
        prefs.putString("target_ver", targetVersion);
    } else {
        prefs.remove("target_ver");
    }
    prefs.end();
}

static bool isAwaitingHealthValidation() {
    Preferences prefs;
    if (!prefs.begin("ota_guard", true)) return false;
    const bool awaiting = prefs.getBool("await_health", false);
    prefs.end();
    return awaiting;
}

static String getAwaitingTargetVersion() {
    Preferences prefs;
    if (!prefs.begin("ota_guard", true)) return "";
    const String targetVersion = prefs.getString("target_ver", "");
    prefs.end();
    return targetVersion;
}


// ==========================================
// RAII WRAPPER - AUTO CLEANUP GUARANTEED
// ==========================================
class HTTPClientRAII {
private:
    // Keep transport objects alive longer than HTTPClient. Members are
    // destroyed in reverse declaration order, so HTTPClient is destroyed
    // first and can never retain a pointer to an already-freed transport.
    WiFiClientSecure clientSecure;
    WiFiClient client;
    HTTPClient http;
    bool secure;

public:
    HTTPClientRAII() : secure(false) {}
    
    ~HTTPClientRAII() {
        http.end();
        if (secure) {
            clientSecure.stop();
        } else {
            client.stop();
        }
    }
    
    bool begin(const String& url) {
        secure = url.startsWith("https://");
        if (secure) {
            Serial.printf("[OTA] HTTPS insecure transport heap=%u largest=%u time=%lld\n",
                          ESP.getFreeHeap(),
                          static_cast<unsigned>(
                              heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                          static_cast<long long>(time(nullptr)));
            clientSecure.setInsecure();
            clientSecure.setTimeout(10000);
            clientSecure.setHandshakeTimeout(30);
            return http.begin(clientSecure, url);
        }
        if (url.startsWith("http://")) {
            Serial.println("[OTA] HTTP transport (checksum validation remains enabled)");
            client.setTimeout(10000);
            return http.begin(client, url);
        }
        Serial.println("[OTA] Unsupported URL scheme");
        return false;
    }
    
    // Accessor methods
    HTTPClient* operator->() { return &http; }
    HTTPClient* get() { return &http; }

    String transportError(int httpCode) {
        String detail;
        switch (httpCode) {
            case HTTPC_ERROR_CONNECTION_REFUSED:
                detail = "connection setup failed; check DNS, server, port, firewall, or TLS certificate";
                break;
            case HTTPC_ERROR_SEND_HEADER_FAILED:
                detail = "connected but failed to send HTTP headers";
                break;
            case HTTPC_ERROR_SEND_PAYLOAD_FAILED:
                detail = "connected but failed to send HTTP payload";
                break;
            case HTTPC_ERROR_NOT_CONNECTED:
                detail = "network client disconnected before request";
                break;
            case HTTPC_ERROR_CONNECTION_LOST:
                detail = "connection lost while receiving response";
                break;
            case HTTPC_ERROR_NO_STREAM:
                detail = "HTTP response stream unavailable";
                break;
            case HTTPC_ERROR_NO_HTTP_SERVER:
                detail = "target did not respond as an HTTP server";
                break;
            case HTTPC_ERROR_TOO_LESS_RAM:
                detail = "insufficient heap for HTTP/TLS";
                break;
            case HTTPC_ERROR_ENCODING:
                detail = "unsupported Transfer-Encoding";
                break;
            case HTTPC_ERROR_STREAM_WRITE:
                detail = "failed writing HTTP stream";
                break;
            case HTTPC_ERROR_READ_TIMEOUT:
                detail = "server response timed out";
                break;
            default:
                detail = HTTPClient::errorToString(httpCode);
                if (detail.isEmpty()) detail = "unknown transport error";
                break;
        }

        if (secure) {
            char tlsError[160] = {};
            int tlsCode = clientSecure.lastError(tlsError, sizeof(tlsError));
            if (tlsCode != 0 || tlsError[0] != '\0') {
                detail += " | TLS code=";
                detail += String(tlsCode);
                detail += " ";
                detail += tlsError;
            }
        }
        return detail;
    }
};

static String describeHttpStatus(int code) {
    switch (code) {
        case 301:
        case 302:
        case 307:
        case 308:
            return "redirect was not resolved";
        case 400:
            return "server rejected the request";
        case 401:
            return "authentication required";
        case 403:
            return "download forbidden; check link permission or anti-bot protection";
        case 404:
            return "file or endpoint not found";
        case 408:
            return "server request timeout";
        case 429:
            return "server rate limit reached";
        case 500:
        case 502:
        case 503:
        case 504:
            return "firmware server or upstream service unavailable";
        default:
            return "";
    }
}
// ==========================================
// HELPER: VERSION COMPARISON
// ==========================================
bool isNewer(String current, String newer) {
    // Fungsi lambda untuk membersihkan string versi menjadi angka murni
    auto parse = [](String v) -> long {
        String cleanVer = "";
        
        // 1. Hapus prefix umum jika ada
        v.replace("isems_v", "");
        v.replace("v", "");
        
        // 2. Ambil HANYA angka
        for (unsigned int i = 0; i < v.length(); i++) {
            if (isDigit(v[i])) {
                cleanVer += v[i];
            }
        }
        
        // Contoh: "isems_v1.1.2" -> "112" -> 112 (int)
        return cleanVer.toInt();
    };

    long currentInt = parse(current);
    long newerInt = parse(newer);

    // Debugging (Opsional, akan muncul di Serial Monitor)
    // Serial.printf("[OTA Debug] Compare: %ld vs %ld\n", currentInt, newerInt);

    return newerInt > currentInt;
}

// ==========================================
// FUNGSI INTI: DOWNLOAD BINARY DENGAN MD5
// ==========================================
bool otaDownloadAndFlash(const String& url, const String& expectedMD5,
                         String& outDetail, const String& targetVersion) {
    Serial.println("[OTA] Starting binary download...");
    dataPublishAck("update", "progress", "Downloading firmware...");
    outDetail = "";

    if (!url.startsWith("http://") && !url.startsWith("https://")) {
        outDetail = "OTA URL must use HTTP or HTTPS";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }
    if (!isValidMd5(expectedMD5)) {
        outDetail = "Firmware MD5 is required and must be 32 hexadecimal characters";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }
    
    // === RAII PATTERN - AUTO CLEANUP ===
    HTTPClientRAII httpClient;
    
    if (!httpClient.begin(url)) {
        Serial.println("[OTA] Connection init failed");
        outDetail = "Firmware HTTPS client init failed; URL invalid or TLS client unavailable";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    httpClient->setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpClient->setTimeout(60000);

    esp_task_wdt_reset();
    int code = httpClient->GET();
    
    if (code != HTTP_CODE_OK) {
        outDetail = "Firmware HTTP " + String(code);
        if (code < 0) {
            outDetail += ": " + httpClient.transportError(code);
        } else {
            String reason = describeHttpStatus(code);
            if (!reason.isEmpty()) outDetail += ": " + reason;
        }
        Serial.printf("[OTA] %s\n", outDetail.c_str());
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    int contentLength = httpClient->getSize();
    Serial.printf("[OTA] Size: %d bytes\n", contentLength);
    
    if (contentLength <= 0) {
        outDetail = "Firmware response has no Content-Length; chunked/dynamic download is not supported";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    // Check space
    if (!Update.begin(contentLength)) {
        outDetail = "Not enough OTA partition space";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    Serial.printf("[OTA] Setting MD5: %s\n", expectedMD5.c_str());
    Update.setMD5(expectedMD5.c_str());

    WiFiClient *stream = httpClient->getStreamPtr();
    size_t written = 0;
    uint8_t buff[1024];
    unsigned long lastAck = 0;
    unsigned long downloadStartedMs = millis();
    unsigned long lastDataMs = downloadStartedMs;
    bool writeError = false;
    bool timedOut = false;

    while ((httpClient->connected() || stream->available()) &&
           (written < contentLength) && !writeError) {
        esp_task_wdt_reset();

        const unsigned long now = millis();
        if (now - lastDataMs >= OTA_DOWNLOAD_INACTIVITY_MS) {
            outDetail = "Download stalled: no data for 15 seconds";
            timedOut = true;
            break;
        }
        if (now - downloadStartedMs >= OTA_DOWNLOAD_TOTAL_TIMEOUT_MS) {
            outDetail = "Download exceeded 10 minute safety limit";
            timedOut = true;
            break;
        }
        
        size_t avail = stream->available();
        if (avail) {
            int c = stream->readBytes(buff, min(avail, sizeof(buff)));
            if (c <= 0) {
                delay(1);
                continue;
            }
            
            if (Update.write(buff, c) != c) {
                Serial.println("[OTA] Write error detected");
                dataPublishAck("update", "failed", "Flash write error");
                writeError = true;
                break; // ✅ AMAN - Destructor akan cleanup
            }
            
            written += c;
            lastDataMs = millis();
            
            // Progress ACK
            if (millis() - lastAck > 2000) {
                int pct = (written * 100) / contentLength;
                String msg = "Flashing: " + String(pct) + "%";
                Serial.println("[OTA] " + msg);
                dataPublishAck("update", "progress", msg);
                lastAck = millis();
            }
        }
        delay(1);
    }

    // Check result
    if (timedOut) {
        Serial.println("[OTA] " + outDetail);
        Update.abort();
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    if (writeError) {
        Serial.println("[OTA] Aborted due to write error");
        Update.abort();
        outDetail = "Flash write error";
        return false;
    }

    if (written != contentLength) {
        Serial.printf("[OTA] Download incomplete: %d/%d bytes\n", written, contentLength);
        outDetail = "Download interrupted (" + String(written) + "/" +
                    String(contentLength) + " bytes)";
        Update.abort();
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    Serial.println("[OTA] Download complete. Verifying MD5...");
    dataPublishAck("update", "progress", "Verifying MD5...");
    
    if (Update.end()) {
        Serial.println("[OTA] SUCCESS! MD5 Match.");
        setAwaitingHealthValidation(true, targetVersion);
        outDetail = "Firmware flashed; rebooting for health validation";
        otaQueuePendingAck("processing", outDetail, true);
        
        delay(250);
        
        // Reboot (destructor tetap dipanggil sebelum restart)
        ESP.restart();
    } else {
        String reason = Update.errorString();
        Serial.printf("[OTA] Verification failed: %s\n", reason.c_str());
        outDetail = "Firmware verification failed: " + reason;
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    return true;
}

static String readIsemsVersionFromPartition(const esp_partition_t* partition) {
    if (partition == nullptr) return "";

    static const char marker[] = "ISEMS_VER:";
    static const size_t MARKER_LEN = sizeof(marker) - 1;
    static const size_t MAX_VERSION_LEN = 31;
    static const size_t CHUNK_SIZE = 512;

    uint8_t buffer[CHUNK_SIZE + MARKER_LEN + MAX_VERSION_LEN] = {};
    size_t carry = 0;

    for (size_t offset = 0; offset < partition->size; offset += CHUNK_SIZE) {
        const size_t remaining = static_cast<size_t>(partition->size) - offset;
        const size_t toRead = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
        if (esp_partition_read(partition, offset, buffer + carry, toRead) != ESP_OK) {
            return "";
        }

        const size_t available = carry + toRead;
        for (size_t i = 0; i + MARKER_LEN < available; ++i) {
            if (memcmp(buffer + i, marker, MARKER_LEN) != 0) continue;

            String version;
            size_t pos = i + MARKER_LEN;
            while (pos < available && version.length() < MAX_VERSION_LEN) {
                const char c = static_cast<char>(buffer[pos++]);
                if (!((c >= '0' && c <= '9') || c == '.' ||
                      c == '-' || c == '_' || c == 'v' || c == 'V')) {
                    break;
                }
                version += c;
            }
            version.trim();
            if (!version.isEmpty()) return version;
        }

        carry = min(available, MARKER_LEN + MAX_VERSION_LEN);
        memmove(buffer, buffer + available - carry, carry);
    }

    return "";
}

bool otaRollbackToPrevious(String& outDetail) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* previous = esp_ota_get_next_update_partition(running);

    if (running == nullptr || previous == nullptr) {
        outDetail = "Previous OTA partition not found";
        return false;
    }

    esp_app_desc_t runningDesc = {};
    esp_app_desc_t previousDesc = {};
    if (esp_ota_get_partition_description(running, &runningDesc) != ESP_OK) {
        outDetail = "Cannot read running firmware metadata";
        return false;
    }
    if (esp_ota_get_partition_description(previous, &previousDesc) != ESP_OK ||
        previousDesc.version[0] == '\0') {
        outDetail = "Previous OTA slot is empty or invalid";
        return false;
    }

    const String runningVersion = String(FW_VERSION);
    String previousVersion = readIsemsVersionFromPartition(previous);
    const bool previousAppVersionKnown = !previousVersion.isEmpty();
    if (!previousAppVersionKnown) {
        previousVersion = "unknown (build " + String(previousDesc.version) + ")";
    }

    if (previousAppVersionKnown &&
        normalizeVersion(runningVersion) == normalizeVersion(previousVersion)) {
        outDetail = "Previous OTA slot contains the same version: " + previousVersion;
        return false;
    }

    Serial.printf("[OTA] Manual rollback: app %s -> %s (partition=%s, build=%s)\n",
                  runningVersion.c_str(), previousVersion.c_str(),
                  previous->label, previousDesc.version);

    esp_err_t result = esp_ota_set_boot_partition(previous);
    if (result != ESP_OK) {
        outDetail = "Failed to select previous firmware, error=" + String((int)result);
        return false;
    }

    outDetail = "Rollback scheduled: " + runningVersion + " -> " + previousVersion;
    return true;
}

// ==========================================
// FUNGSI UTAMA: CEK MANIFEST JSON
// ==========================================
bool otaProcessManifest(const String& manifestUrl, String& outDetail) {
    Serial.println("[OTA] Checking manifest...");
    dataPublishAck("update", "processing", "Checking version info...");
    outDetail = "";

    if (!manifestUrl.startsWith("http://") && !manifestUrl.startsWith("https://")) {
        outDetail = "Manifest URL must use HTTP or HTTPS";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    String binUrl;
    String md5;
    String newVer;
    String curVer = String(FW_VERSION);
    String json;

    static const uint8_t MANIFEST_MAX_ATTEMPTS = 3;
    for (uint8_t attempt = 1; attempt <= MANIFEST_MAX_ATTEMPTS; ++attempt) {
        HTTPClientRAII httpClient;
        
        if (!httpClient.begin(manifestUrl)) {
            outDetail = "Manifest HTTPS client init failed; URL invalid or TLS client unavailable";
            Serial.printf("[OTA] Manifest attempt %u/%u: %s\n",
                          attempt, MANIFEST_MAX_ATTEMPTS, outDetail.c_str());
        } else {
            httpClient->setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
            httpClient->setTimeout(20000);
            httpClient->setConnectTimeout(15000);
            httpClient->setReuse(false);

            esp_task_wdt_reset();
            yield();

            Serial.printf("[OTA] Sending GET request to manifest (attempt %u/%u)...\n",
                          attempt, MANIFEST_MAX_ATTEMPTS);
            int code = httpClient->GET();

            Serial.printf("[OTA] Manifest HTTP Code: %d\n", code);

            if (code == HTTP_CODE_OK) {
                json = httpClient->getString();
                outDetail = "";
            } else {
                outDetail = "Manifest HTTP " + String(code);
                if (code < 0) {
                    outDetail += ": " + httpClient.transportError(code);
                } else {
                    String reason = describeHttpStatus(code);
                    if (!reason.isEmpty()) outDetail += ": " + reason;
                }
                Serial.printf("[OTA] Manifest attempt %u/%u failed: %s\n",
                              attempt, MANIFEST_MAX_ATTEMPTS, outDetail.c_str());
            }
        }

        if (!json.isEmpty()) break;
        if (attempt < MANIFEST_MAX_ATTEMPTS) {
            unsigned long retryDelayMs = 1000UL * attempt;
            Serial.printf("[OTA] Retrying manifest with fresh TLS session in %lu ms...\n",
                          retryDelayMs);
            unsigned long waitStart = millis();
            while (millis() - waitStart < retryDelayMs) {
                esp_task_wdt_reset();
                delay(50);
            }
        }
    }

    if (json.isEmpty()) {
        if (outDetail.isEmpty()) outDetail = "Manifest download failed";
        Serial.printf("[OTA] Manifest failed after %u attempts: %s\n",
                      MANIFEST_MAX_ATTEMPTS, outDetail.c_str());
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    {
        Serial.printf("[OTA] Manifest JSON (%u bytes): %s\n",
                      static_cast<unsigned>(json.length()), json.c_str());

        if (json.length() == 0 || json.length() >= 512) {
            Serial.printf("[OTA] Manifest length invalid: %u\n",
                          static_cast<unsigned>(json.length()));
            outDetail = "Manifest payload too large or empty";
            dataPublishAck("update", "failed", outDetail);
            return false;
        }

        Serial.printf("[OTA] Before manifest parse: heap=%u largest=%u integrity=%s stack=%u\n",
                      ESP.getFreeHeap(),
                      static_cast<unsigned>(
                          heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                      heap_caps_check_integrity_all(false) ? "OK" : "CORRUPT",
                      static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));

        // Parse from a bounded mutable buffer. This avoids ArduinoJson's
        // Reader<String> path, which is where firmware 2.1.1 repeatedly
        // panicked immediately after printing a valid manifest.
        char manifestBuffer[512];
        memcpy(manifestBuffer, json.c_str(), json.length() + 1);

        StaticJsonDocument<512> doc;
        DeserializationError err =
            deserializeJson(doc, manifestBuffer, json.length());
        
        if (err) {
            Serial.printf("[OTA] JSON Parse Error: %s\n", err.c_str());
            outDetail = "Invalid JSON Manifest";
            dataPublishAck("update", "failed", outDetail);
            return false;
        }

        const char* parsedVersion = doc["version"] | "";
        const char* parsedUrl = doc["url"] | "";
        const char* parsedMd5 = doc["md5"] | "";
        newVer = parsedVersion;
        binUrl = parsedUrl;
        md5 = parsedMd5;

        Serial.printf("[OTA] Manifest parsed: version=%s url_len=%u md5_len=%u\n",
                      newVer.c_str(), static_cast<unsigned>(binUrl.length()),
                      static_cast<unsigned>(md5.length()));
        Serial.printf("[OTA] Before manifest HTTP cleanup: heap=%u largest=%u integrity=%s stack=%u\n",
                      ESP.getFreeHeap(),
                      static_cast<unsigned>(
                          heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                      heap_caps_check_integrity_all(false) ? "OK" : "CORRUPT",
                      static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    }
    // ═══════════════════════════════════════════════════════════════
    // httpClient is NOW fully destroyed — TLS session freed.
    // ═══════════════════════════════════════════════════════════════

    // Allow heap to settle after TLS teardown
    delay(500);
    esp_task_wdt_reset();

    Serial.printf("[OTA] Manifest HTTP cleanup complete: heap=%u largest=%u integrity=%s stack=%u\n",
                  ESP.getFreeHeap(),
                  static_cast<unsigned>(
                      heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                  heap_caps_check_integrity_all(false) ? "OK" : "CORRUPT",
                  static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    Serial.printf("[OTA] Current: %s | New: %s\n", curVer.c_str(), newVer.c_str());

    if (newVer.length() == 0 || binUrl.length() == 0) {
        outDetail = "Invalid manifest data";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }
    if (!binUrl.startsWith("http://") && !binUrl.startsWith("https://")) {
        outDetail = "Manifest binary URL must use HTTP or HTTPS";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }
    if (!isValidMd5(md5)) {
        outDetail = "Manifest MD5 missing or invalid";
        dataPublishAck("update", "failed", outDetail);
        return false;
    }

    if (!isNewer(curVer, newVer)) {
        outDetail = "Already up-to-date (" + curVer + ")";
        dataPublishAck("update", "success", outDetail);
        return true;
    }

    // Found new version
    String msg = "New version found: " + newVer;
    dataPublishAck("update", "processing", msg);
    
    // Heap check before HTTPS download — TLS needs ~40-50KB free heap
    if (binUrl.startsWith("https://")) {
        uint32_t freeHeap = ESP.getFreeHeap();
        Serial.printf("[OTA] Free heap before HTTPS download: %u bytes\n", freeHeap);
        if (freeHeap < 50000) {
            Serial.printf("[OTA] Insufficient heap for HTTPS OTA (%u < 50000)\n", freeHeap);
            outDetail = "Low memory for HTTPS (" + String(freeHeap) + " bytes)";
            dataPublishAck("update", "failed", outDetail);
            return false;
        }
    }

    // Start firmware download (creates its own HTTPClientRAII internally)
    return otaDownloadAndFlash(binUrl, md5, outDetail, newVer);
}

// ==========================================
// PROBE URL (Fungsi Lama / Utility)
// ==========================================
int otaProbeURL(const String& url) {
    esp_task_wdt_reset();

    Serial.println("[OTA] ====== CONNECTION TEST ======");
    Serial.printf("[OTA] Target URL: %s\n", url.c_str());
    
    // === RAII PATTERN ===
    HTTPClientRAII httpClient;
    
    if (!httpClient.begin(url)) {
        Serial.println("[OTA] Connection failed to init");
        return -1; // Auto cleanup
    }
    
    esp_task_wdt_reset();
    
    httpClient->setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpClient->setConnectTimeout(10000);
    httpClient->setUserAgent("ESP32-OTA-Client/1.0");
    
    const char* headerKeys[] = {"Content-Type", "Content-Length"};
    httpClient->collectHeaders(headerKeys, 2);
    
    Serial.println("[OTA] Sending HEAD request...");
    int httpCode = httpClient->sendRequest("HEAD");
    
    esp_task_wdt_reset();
    
    // Fallback to GET if HEAD fails
    if (httpCode == 405 || httpCode < 0) {
        Serial.println("[OTA] HEAD failed, trying GET...");
        httpCode = httpClient->GET();
    }
    
    Serial.printf("[OTA] HTTP Response Code: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) {
        String contentType = httpClient->header("Content-Type");
        Serial.printf("[OTA] Content-Type: %s\n", contentType.c_str());
        
        // Validate content type
        if (contentType.indexOf("text/html") >= 0 || 
            contentType.indexOf("application/json") >= 0) {
            Serial.println("[OTA] Error: Link is webpage, not binary!");
            return 415; // Auto cleanup
        }
        
        // Validate file size
        String contentLenStr = httpClient->header("Content-Length");
        if (contentLenStr.length() > 0) {
            long fileSize = contentLenStr.toInt();
            long freeSpace = ESP.getFreeSketchSpace();
            
            Serial.printf("[OTA] File: %ld bytes, Free: %ld bytes\n", 
                      fileSize, freeSpace);
            
            if (fileSize > freeSpace) {
                Serial.println("[OTA] Error: File too large!");
                return 413; // Auto cleanup
            }
            
            if (fileSize < 10000) {
                Serial.println("[OTA] Error: File too small!");
                return 204; // Auto cleanup
            }
        }
    }

    Serial.println("[OTA] ====== TEST COMPLETE ======");
    return httpCode; // Auto cleanup
}

// ==========================================
// ROLLBACK PROTECTION
// ==========================================
static bool otaTasksHealthy() {
    TaskHandle_t handles[] = {
        g_taskNetwork,
        g_taskSensor,
        g_taskAutomation,
        g_taskControl,
        g_taskHealth
    };

    for (TaskHandle_t handle : handles) {
        if (handle == nullptr || uxTaskGetStackHighWaterMark(handle) == 0) {
            return false;
        }
    }
    return true;
}

void otaPendingAckService() {
    loadPendingAck();
    if (!otaPendingAckValid || !dataIsConnected()) return;

    const unsigned long now = millis();
    if (now - otaLastPendingAckAttemptMs < 3000UL) return;
    otaLastPendingAckAttemptMs = now;

    const String status = otaPendingAckStatus;
    const String detail = otaPendingAckDetail;
    if (dataPublishAck("update", status.c_str(), detail)) {
        otaClearPendingAck();
        Serial.println("[OTA] Pending ACK delivered");
    }
}

void otaValidationService() {
    if (!otaPendingValidation) return;

    const unsigned long now = millis();
    const bool stableLongEnough = now - otaPendingSinceMs >= OTA_HEALTH_STABLE_MS;
    const bool tasksHealthy = otaTasksHealthy();
    const uint32_t freeHeap = ESP.getFreeHeap();
    const bool heapHealthy = freeHeap >= OTA_HEALTH_MIN_FREE_HEAP;

    if (!stableLongEnough || !tasksHealthy || !heapHealthy) {
        if (now - otaLastValidationLogMs >= 10000UL) {
            otaLastValidationLogMs = now;
            Serial.printf("[OTA] Pending validation: stable=%d tasks=%d heap=%u/%u mqtt=%d (MQTT not required)\n",
                          stableLongEnough, tasksHealthy, freeHeap,
                          OTA_HEALTH_MIN_FREE_HEAP, dataIsConnected());
        }
        return;
    }

    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        otaPendingValidation = false;
        setAwaitingHealthValidation(false);
        Serial.println("[OTA] Firmware marked valid after health check.");
        dataPublishAck("update", "success", "Firmware boot health check passed and rollback cancelled");
    } else {
        Serial.println("[OTA] Failed to mark firmware valid; rollback protection remains active.");
    }
}

void otaValidateFirmware() {
    otaValidationService();
}

void otaInit() {
    Serial.println("[OTA] Init complete");
    loadPendingAck();

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (running != nullptr &&
        esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        otaPendingValidation = true;
        otaPendingSinceMs = millis();
        otaLastValidationLogMs = 0;
        Serial.printf("[OTA] New firmware pending verification. Health gate: %lus, heap >= %u, tasks alive.\n",
                      OTA_HEALTH_STABLE_MS / 1000UL, OTA_HEALTH_MIN_FREE_HEAP);
    } else {
        otaPendingValidation = false;
        if (isAwaitingHealthValidation()) {
            const String targetVersion = getAwaitingTargetVersion();
            const String runningVersion = String(FW_VERSION);
            const bool targetKnown = !targetVersion.isEmpty();
            const bool targetBooted =
                targetKnown &&
                normalizeVersion(targetVersion) == normalizeVersion(runningVersion);

            setAwaitingHealthValidation(false);
            if (!targetKnown || targetBooted) {
                String detail = targetKnown
                    ? "Target firmware booted successfully and is already valid: " + runningVersion
                    : "Firmware booted successfully; direct OTA target version was not provided";
                otaQueuePendingAck("success", detail, true);
                Serial.printf("[OTA] OTA boot accepted: target=%s running=%s state=%d\n",
                              targetVersion.c_str(), runningVersion.c_str(), (int)state);
            } else {
                otaQueuePendingAck(
                    "failed",
                    "Bootloader restored previous firmware; target=" + targetVersion +
                        " running=" + runningVersion,
                    true);
                Serial.printf("[OTA] Detected OTA rollback: target=%s running=%s state=%d\n",
                              targetVersion.c_str(), runningVersion.c_str(), (int)state);
            }
        }
        Serial.println("[OTA] Firmware already validated (no pending state).");
    }
}
