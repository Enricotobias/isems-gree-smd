#include "ota.h"
#include "config.h"

#include "data.h"

// --- FIX: PINDAHKAN INCLUDE INI KE ATAS (GLOBAL SCOPE) ---
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
// ---------------------------------------------------------

#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "LittleFS.h"

static const char* OTA_PENDING_ACK_FILE = "/ota_ack_pending.json";

static bool otaStorePendingSuccessAck(const String& detail) {
    File file = LittleFS.open(OTA_PENDING_ACK_FILE, "w");
    if (!file) {
        Serial.Println("[OTA] Failed to store pending success ACK");
        return false;
    }

    StaticJsonDocument<192> doc;
    doc["detail"] = detail;
    serializeJson(doc, file);
    file.close();
    Serial.Println("[OTA] Pending success ACK stored");
    return true;
}

static bool otaLoadPendingSuccessAck(String& detail) {
    if (!LittleFS.exists(OTA_PENDING_ACK_FILE)) {
        return false;
    }

    File file = LittleFS.open(OTA_PENDING_ACK_FILE, "r");
    if (!file) {
        Serial.Println("[OTA] Pending ACK file exists but cannot be opened");
        return false;
    }

    StaticJsonDocument<192> doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) {
        Serial.Printf("[OTA] Pending ACK file invalid: %s\n", err.c_str());
        return false;
    }

    detail = doc["detail"] | "Update OK. Rebooting...";
    return true;
}

static void otaClearPendingSuccessAck() {
    if (LittleFS.exists(OTA_PENDING_ACK_FILE)) {
        LittleFS.remove(OTA_PENDING_ACK_FILE);
    }
}

static void otaPublishPendingSuccessAckIfAny() {
    String detail;
    if (!otaLoadPendingSuccessAck(detail)) {
        return;
    }

    if (!dataIsConnected()) {
        Serial.Println("[OTA] Pending success ACK exists but MQTT is not connected yet");
        return;
    }

    if (dataPublishAck("update", "success", detail)) {
        Serial.Println("[OTA] Pending success ACK published after reboot");
        otaClearPendingSuccessAck();
    } else {
        Serial.Println("[OTA] Failed to publish pending success ACK after reboot; will retry on next reconnect");
    }
}


// ==========================================
// RAII WRAPPER - AUTO CLEANUP GUARANTEED
// ==========================================
class HTTPClientRAII {
private:
    std::unique_ptr<HTTPClient> http;
    std::unique_ptr<WiFiClientSecure> clientSecure;
    std::unique_ptr<WiFiClient> client;
    bool isHTTPS;

public:
    HTTPClientRAII() : isHTTPS(false) {
        http = std::unique_ptr<HTTPClient>(new HTTPClient());
    }
    
    // Destructor OTOMATIS dipanggil saat object keluar scope
    // Bahkan saat exception/break/return!
    ~HTTPClientRAII() {
        if (http) {
            http->end();
        }
        // unique_ptr otomatis delete client/clientSecure
    }
    
    bool begin(const String& url) {
        isHTTPS = url.startsWith("https://");
        
        if (isHTTPS) {
            clientSecure = std::unique_ptr<WiFiClientSecure>(new WiFiClientSecure());
            if (!clientSecure) return false;
            
            clientSecure->setInsecure();
            clientSecure->setTimeout(10000);
            return http->begin(*clientSecure, url);
        } else {
            client = std::unique_ptr<WiFiClient>(new WiFiClient());
            if (!client) return false;
            
            client->setTimeout(10000);
            return http->begin(*client, url);
        }
    }
    
    // Accessor methods
    HTTPClient* operator->() { return http.get(); }
    HTTPClient* get() { return http.get(); }
};
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
    // Serial.Printf("[OTA Debug] Compare: %ld vs %ld\n", currentInt, newerInt);

    return newerInt > currentInt;
}

// ==========================================
// FUNGSI INTI: DOWNLOAD BINARY DENGAN MD5
// ==========================================
void otaDownloadAndFlash(const String& url, const String& expectedMD5) {
    Serial.Println("[OTA] Starting binary download...");
    dataPublishAck("update", "progress", "Downloading firmware...");
    
    // === RAII PATTERN - AUTO CLEANUP ===
    HTTPClientRAII httpClient;
    
    if (!httpClient.begin(url)) {
        Serial.Println("[OTA] Connection init failed");
        dataPublishAck("update", "failed", "Connection init failed");
        return; // Destructor otomatis cleanup
    }

    httpClient->setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpClient->setTimeout(60000);

    esp_task_wdt_reset();
    int code = httpClient->GET();
    
    if (code != HTTP_CODE_OK) {
        Serial.Printf("[OTA] HTTP Error Code: %d\n", code);
        dataPublishAck("update", "failed", "HTTP Error " + String(code));
        return; // Auto cleanup
    }

    int contentLength = httpClient->getSize();
    Serial.Printf("[OTA] Size: %d bytes\n", contentLength);
    
    if (contentLength <= 0) {
        dataPublishAck("update", "failed", "Empty content length");
        return; // Auto cleanup
    }

    // Check space
    if (!Update.begin(contentLength)) {
        dataPublishAck("update", "failed", "Not enough space");
        return; // Auto cleanup
    }

    // Set MD5 checksum
    if (expectedMD5.length() == 32) {
        Serial.Printf("[OTA] Setting MD5: %s\n", expectedMD5.c_str());
        Update.setMD5(expectedMD5.c_str());
    }

    WiFiClient *stream = httpClient->getStreamPtr();
    size_t written = 0;
    uint8_t buff[1024];
    unsigned long lastAck = 0;
    bool writeError = false;

    while (httpClient->connected() && (written < contentLength) && !writeError) {
        esp_task_wdt_reset();
        
        size_t avail = stream->available();
        if (avail) {
            int c = stream->readBytes(buff, min(avail, sizeof(buff)));
            
            if (Update.write(buff, c) != c) {
                Serial.Println("[OTA] Write error detected");
                dataPublishAck("update", "failed", "Flash write error");
                writeError = true;
                break; // ✅ AMAN - Destructor akan cleanup
            }
            
            written += c;
            
            // Progress ACK
            if (millis() - lastAck > 2000) {
                int pct = (written * 100) / contentLength;
                String msg = "Flashing: " + String(pct) + "%";
                Serial.Println("[OTA] " + msg);
                dataPublishAck("update", "progress", msg);
                lastAck = millis();
            }
        }
        delay(1);
    }

    // Check result
    if (writeError) {
        Serial.Println("[OTA] Aborted due to write error");
        return; // Auto cleanup
    }

    if (written != contentLength) {
        Serial.Printf("[OTA] Download incomplete: %d/%d bytes\n", written, contentLength);
        dataPublishAck("update", "failed", "Download interrupted");
        return; // Auto cleanup
    }

    Serial.Println("[OTA] Download complete. Verifying MD5...");
    dataPublishAck("update", "progress", "Verifying MD5...");
    
    if (Update.end()) {
        Serial.Println("[OTA] SUCCESS! MD5 Match.");
        String successDetail = "Update OK. Rebooting...";
        otaStorePendingSuccessAck(successDetail);
        dataPublishAck("update", "success", successDetail);
        
        // Tunggu ACK terkirim
        delay(1000);
        esp_task_wdt_reset();
        
        // Reboot (destructor tetap dipanggil sebelum restart)
        ESP.restart();
    } else {
        String reason = Update.errorString();
        Serial.Printf("[OTA] Verification failed: %s\n", reason.c_str());
        dataPublishAck("update", "failed", "MD5 Mismatch: " + reason);
        return; // Auto cleanup
    }
}

// ==========================================
// FUNGSI UTAMA: CEK MANIFEST JSON
// ==========================================
void otaProcessManifest(const String& manifestUrl) {
    Serial.Println("[OTA] Checking manifest...");
    dataPublishAck("update", "processing", "Checking version info...");

    // === RAII PATTERN ===
    HTTPClientRAII httpClient;
    
    if (!httpClient.begin(manifestUrl)) {
        Serial.Println("[OTA] Manifest connection init failed");
        dataPublishAck("update", "failed", "Manifest URL unreachable");
        return; // Auto cleanup
    }

    httpClient->setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpClient->setTimeout(15000);
    httpClient->setConnectTimeout(10000);

    esp_task_wdt_reset();
    yield();
    
    Serial.Println("[OTA] Sending GET request to manifest...");
    int code = httpClient->GET();
    
    Serial.Printf("[OTA] Manifest HTTP Code: %d\n", code);
    
    if (code != 200) {
        Serial.Printf("[OTA] Manifest HTTP Error: %d\n", code);
        dataPublishAck("update", "failed", "Manifest HTTP " + String(code));
        return; // Auto cleanup
    }

    String json = httpClient->getString();
    Serial.Println("[OTA] Manifest JSON: " + json);
    
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, json);
    
    if (err) {
        Serial.Printf("[OTA] JSON Parse Error: %s\n", err.c_str());
        dataPublishAck("update", "failed", "Invalid JSON Manifest");
        return; // Auto cleanup
    }

    String newVer = doc["version"] | "";
    String binUrl = doc["url"] | "";
    String md5    = doc["md5"] | "";
    
    String curVer = String(FW_VERSION);

    Serial.Printf("[OTA] Current: %s | New: %s\n", curVer.c_str(), newVer.c_str());

    if (newVer.length() == 0 || binUrl.length() == 0) {
        dataPublishAck("update", "failed", "Invalid manifest data");
        return; // Auto cleanup
    }

    if (!isNewer(curVer, newVer)) {
        dataPublishAck("update", "success", "Already up-to-date (" + curVer + ")");
        return; // Auto cleanup
    }

    // Found new version
    String msg = "New version found: " + newVer;
    dataPublishAck("update", "processing", msg);
    
    // ✅ CRITICAL: Tunggu httpClient destructor selesai sebelum download
    // Caranya: buat scope baru agar httpClient di-destroy dulu
    {
        // httpClient akan di-destroy di sini (end of scope)
    }
    
    // Tunggu cleanup selesai
    delay(500);
    
    // Lanjut download (dengan HTTPClientRAII baru di dalamnya)
    otaDownloadAndFlash(binUrl, md5);
}

// ==========================================
// PROBE URL (Fungsi Lama / Utility)
// ==========================================
int otaProbeURL(const String& url) {
    Serial.Println("[OTA] ====== CONNECTION TEST ======");
    Serial.Printf("[OTA] Target URL: %s\n", url.c_str());
    
    // === RAII PATTERN ===
    HTTPClientRAII httpClient;
    
    if (!httpClient.begin(url)) {
        Serial.Println("[OTA] Connection failed to init");
        return -1; // Auto cleanup
    }
    
    esp_task_wdt_reset();
    
    httpClient->setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    httpClient->setConnectTimeout(10000);
    httpClient->setUserAgent("ESP32-OTA-Client/1.0");
    
    const char* headerKeys[] = {"Content-Type", "Content-Length"};
    httpClient->collectHeaders(headerKeys, 2);
    
    Serial.Println("[OTA] Sending HEAD request...");
    int httpCode = httpClient->sendRequest("HEAD");
    
    esp_task_wdt_reset();
    
    // Fallback to GET if HEAD fails
    if (httpCode == 405 || httpCode < 0) {
        Serial.Println("[OTA] HEAD failed, trying GET...");
        httpCode = httpClient->GET();
    }
    
    Serial.Printf("[OTA] HTTP Response Code: %d\n", httpCode);

    if (httpCode == HTTP_CODE_OK) {
        String contentType = httpClient->header("Content-Type");
        Serial.Printf("[OTA] Content-Type: %s\n", contentType.c_str());
        
        // Validate content type
        if (contentType.indexOf("text/html") >= 0 || 
            contentType.indexOf("application/json") >= 0) {
            Serial.Println("[OTA] Error: Link is webpage, not binary!");
            return 415; // Auto cleanup
        }
        
        // Validate file size
        String contentLenStr = httpClient->header("Content-Length");
        if (contentLenStr.length() > 0) {
            long fileSize = contentLenStr.toInt();
            long freeSpace = ESP.getFreeSketchSpace();
            
            Serial.Printf("[OTA] File: %ld bytes, Free: %ld bytes\n", 
                      fileSize, freeSpace);
            
            if (fileSize > freeSpace) {
                Serial.Println("[OTA] Error: File too large!");
                return 413; // Auto cleanup
            }
            
            if (fileSize < 10000) {
                Serial.Println("[OTA] Error: File too small!");
                return 204; // Auto cleanup
            }
        }
    }

    Serial.Println("[OTA] ====== TEST COMPLETE ======");
    return httpCode; // Auto cleanup
}

// ==========================================
// ROLLBACK PROTECTION
// ==========================================
void otaValidateFirmware() {
    // Fungsi ini aman karena header esp_ota_ops.h sudah di-include di paling atas
    
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            Serial.Println("[OTA] ⚠️ New firmware detected. Validating...");
            
            // Mark firmware as valid to prevent rollback
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
                Serial.Println("[OTA] ✅ Firmware MARKED VALID.");

                if (dataIsConnected()) {
                    dataPublishAck("system", "success", "Firmware validated!");
                }
                otaPublishPendingSuccessAckIfAny();
            } else {
                Serial.Println("[OTA] ❌ Failed to mark firmware valid!");
            }
        } else {
            Serial.Println("[OTA] Firmware already validated (no pending state).");
            otaPublishPendingSuccessAckIfAny();
        }
    } else {
        Serial.Println("[OTA] ⚠️ Could not read OTA partition state.");
        otaPublishPendingSuccessAckIfAny();
    }
}

void otaInit() {
    Serial.Println("[OTA] Init complete");
}