#include "data.h"
#include "config.h"
#include "controll.h"
#include "logger.h"
#include "crash_monitor.h"
#include "automation.h"
#include "ir_health.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "esp_task_wdt.h"
#include "ota.h"
#include "time.h"
#include "activity.h"

// ============= MQTT Topics =============
static String topicStatus;      // isems/devices/{id}/status
static String topicData;        // isems/devices/{id}/data
static String topicSetting;     // isems/devices/{id}/setting
static String topicLogs;        // isems/devices/{id}/logs

static String cmdSetting;       // isems/command/{id}/setting
static String cmdKoneksi;       // isems/command/{id}/koneksi
static String cmdSchedule;      // isems/command/{id}/schedule
static String cmdRemote;        // isems/command/{id}/remote
static String cmdReboot;        // isems/command/{id}/reboot
static String cmdInfo;          // isems/command/{id}/info
static String cmdLogs;          // isems/command/{id}/logs

String topicCmdUpdate = "";
static String cmdDiagnostics = "";
String topicInfo = "";

String topicAck = ""; 
DisconnectReason lastDisconnectReason = REASON_POWER_ON; // Default saat boot
unsigned long lastDisconnectTime = 0;
    
unsigned long mqttReconnectCount = 0;
unsigned long wifiDisconnectCount = 0;
bool isManualDisconnect = false; // <--- TAMBAHKAN INI (Flag penanda)

static bool pendingSaveRequest = false;
static unsigned long saveRequestTime = 0;

extern bool forceScheduleCheck;
extern volatile unsigned long wifiDisconnectEpoch;
extern bool rtcOK;

static BPFrame lastKnownState = {false, false, 0, "", "", 0, "", "", "", ""};



enum ActionSource {
  ACTION_NONE,
  ACTION_AUTOMATION,
  ACTION_MQTT_REMOTE
};

String getDisconnectReasonStr(DisconnectReason reason) {
    switch(reason) {
        case REASON_POWER_ON: return "POWER_ON_RESET";
        case REASON_POWER_LOSS: return "POWER_LOSS";
        case REASON_WIFI_LOST: return "WIFI_LOST";
        case REASON_MQTT_LOST: return "MQTT_TIMEOUT";
        case REASON_WDT_RESET: return "WATCHDOG_CRASH";
        case REASON_CRASH:     return "SOFTWARE_CRASH";
        case REASON_MANUAL:    return "MANUAL_SAVE";
        default: return "UNKNOWN";
    }
}

static ActionSource lastActionSource = ACTION_NONE;
static unsigned long lastActionMs = 0;

// ============= MQTT Client =============
static WiFiClientSecure espClient;  // Gunakan WiFiClientSecure untuk SSL/TLS
static PubSubClient mqtt(espClient);
static String mqttUser = "";
static String mqttPass = "";
// Last Will & Testament (LWT) - Dikirim oleh broker jika device disconnect mendadak
// Dibangun dinamis sebelum setiap mqtt.connect() agar last_online tersimpan.
static String lwtPayload;

// Timestamp saat MQTT terakhir berhasil connect (millis).
// Digunakan untuk cooldown guard agar publish setting tidak langsung setelah reconnect
// (SSL buffer masih penuh dari burst status + subscribe).
static unsigned long lastMqttConnectMs = 0;
static const unsigned long MQTT_POST_CONNECT_PUBLISH_COOLDOWN_MS = 3000UL;

// Counter publish gagal berturut-turut. Jika >= 3 saat mqtt.connected(),
// berarti zombie connection (SSL rusak) — force reconnect di dataLoop().
static uint8_t consecutivePublishFails = 0;

// Recovery bertingkat untuk RC -2 (TCP/TLS gagal).
// Tahap 1: reset WiFi radio + TLS stack.
// Tahap 2: reboot ESP32 jika tetap macet cukup lama.
static uint8_t mqttRc2ConsecutiveFails = 0;
static unsigned long mqttRc2FirstFailMs = 0;
static unsigned long lastMqttRc2RecoveryMs = 0;
static bool mqttRc2WifiRecoveryAttempted = false;
static const uint8_t MQTT_RC2_WIFI_RESET_THRESHOLD = 5;
static const unsigned long MQTT_RC2_RESTART_AFTER_MS = 300000UL;
static const unsigned long MQTT_RC2_RECOVERY_COOLDOWN_MS = 30000UL;

static const uint8_t MAX_NEARBY_WIFI_NETWORKS = 5;
static const unsigned long WIFI_SCAN_REFRESH_MS = 180000UL;
static const unsigned long WIFI_SCAN_RETRY_MS = 15000UL;

struct NearbyWifiEntry {
  String ssid;
  String bssid;
  int32_t rssi;
  uint8_t channel;
  wifi_auth_mode_t authMode;
  bool hidden;
};

static NearbyWifiEntry nearbyWifiCache[MAX_NEARBY_WIFI_NETWORKS];
static uint8_t nearbyWifiCount = 0;
static bool nearbyWifiScanInProgress = false;
static unsigned long lastNearbyWifiScanStartMs = 0;
static unsigned long lastNearbyWifiScanCompleteMs = 0;

// ============= Command Processing =============
static bool isProcessingCommand = false;

struct PendingMqttCommand {
  bool used;
  char topic[128];
  char payload[2049];
  unsigned int length;
};

static PendingMqttCommand pendingCommands[4] = {};
static uint8_t pendingCommandHead = 0;
static uint8_t pendingCommandTail = 0;
static uint8_t pendingCommandCount = 0;
static unsigned long lastDeferredPublishLogMs = 0;

// ============= Forward Declarations =============
static void ensureMqttConnected();
static void handleMqttMessage(char* topic, byte* payload, unsigned int length);
static void processQueuedMqttCommand();
static void routeMqttCommand(const String& topicStr, const char* payload, unsigned int length);
static void handleCmdSetting(const char* payload, unsigned int length);
static void handleCmdKoneksi(const char* payload, unsigned int length);
static void handleCmdSchedule(const char* payload, unsigned int length);
static void handleCmdRemote(const char* payload, unsigned int length);
static void handleCmdLogs(const char* payload, unsigned int length);
static void publishLogs(const String& message);
static void serviceNearbyWifiScan();

// ============= Build Topics =============
void dataBuildTopics(const String& device_id) {
  // Publish topics
  topicStatus  = "isems/devices/" + device_id + "/status";
  topicData    = "isems/devices/" + device_id + "/data";
  topicSetting = "isems/devices/" + device_id + "/setting";
  topicInfo = "isems/devices/" + device_id + "/info";
  topicAck = "isems/devices/" + device_id + "/ack";
  topicLogs = "isems/devices/" + device_id + "/logs";
  activityBuildTopic(device_id);
  
  // Command topics
  cmdSetting  = "isems/command/" + device_id + "/setting";
  cmdKoneksi  = "isems/command/" + device_id + "/koneksi";
  cmdSchedule = "isems/command/" + device_id + "/schedule";
  cmdRemote   = "isems/command/" + device_id + "/remote";
  topicCmdUpdate = "isems/command/" + device_id + "/update";
  cmdReboot = "isems/command/" + device_id + "/reboot";
  cmdDiagnostics = "isems/command/" + device_id + "/diagnostics";
  cmdInfo = "isems/command/" + device_id + "/info";
  cmdLogs = "isems/command/" + device_id + "/logs";

  logPrintln(" - Update: " + topicCmdUpdate);
  logPrintln(" - Diagnostics: " + cmdDiagnostics);
  logPrintln(" - Info: " + cmdInfo);
  logPrintln(" - Reboot: " + cmdReboot);
  logPrintln(" - Logs: " + cmdLogs);
}

void dataSetServer(const char* host, uint16_t port, const String& user, const String& pass) {
  mqtt.setServer(host, port);
  mqttUser = user;
  mqttPass = pass;
}

// Helper untuk menerjemahkan kode reset menjadi teks yang mudah dibaca
String getResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON: return "Power On";
    case ESP_RST_EXT: return "External Pin Reset";
    case ESP_RST_SW: return "Software/System Restart"; // Biasanya karena OTA atau ESP.restart()
    case ESP_RST_PANIC: return "Crash/Exception"; // Ini bahaya (kodingan error)
    case ESP_RST_INT_WDT: return "Watchdog (Internal)";
    case ESP_RST_TASK_WDT: return "Watchdog (Task)"; // Hang/Looping macet
    case ESP_RST_BROWNOUT: return "Brownout (Low Voltage)"; // Kabel power jelek
    case ESP_RST_SDIO: return "SDIO Reset";
    default: return "Unknown";
  }
}

bool dataIsPublishReady() {
  return mqtt.connected() && (millis() - lastMqttConnectMs >= MQTT_POST_CONNECT_PUBLISH_COOLDOWN_MS);
}

static const char* getWifiAuthModeLabel(wifi_auth_mode_t authMode) {
  switch (authMode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
    case WIFI_AUTH_OWE: return "OWE";
    default: return "UNKNOWN";
  }
}

static void clearNearbyWifiCache() {
  nearbyWifiCount = 0;
  for (uint8_t i = 0; i < MAX_NEARBY_WIFI_NETWORKS; ++i) {
    nearbyWifiCache[i].ssid = "";
    nearbyWifiCache[i].bssid = "";
    nearbyWifiCache[i].rssi = 0;
    nearbyWifiCache[i].channel = 0;
    nearbyWifiCache[i].authMode = WIFI_AUTH_OPEN;
    nearbyWifiCache[i].hidden = false;
  }
}

static void updateNearbyWifiCache(int16_t networkCount) {
  clearNearbyWifiCache();
  for (int16_t i = 0; i < networkCount && nearbyWifiCount < MAX_NEARBY_WIFI_NETWORKS; ++i) {
    String ssid = WiFi.SSID(i);
    bool hidden = (ssid.length() == 0);
    if (hidden) {
      ssid = "<hidden>";
    }

    nearbyWifiCache[nearbyWifiCount].ssid = ssid;
  nearbyWifiCache[nearbyWifiCount].bssid = WiFi.BSSIDstr(i);
    nearbyWifiCache[nearbyWifiCount].rssi = WiFi.RSSI(i);
    nearbyWifiCache[nearbyWifiCount].channel = (uint8_t)WiFi.channel(i);
    nearbyWifiCache[nearbyWifiCount].authMode = (wifi_auth_mode_t)WiFi.encryptionType(i);
    nearbyWifiCache[nearbyWifiCount].hidden = hidden;
    nearbyWifiCount++;
  }
}

static void appendNearbyWifiNetworks(JsonArray networks) {
  for (uint8_t i = 0; i < nearbyWifiCount; ++i) {
    JsonObject network = networks.createNestedObject();
    network["ssid"] = nearbyWifiCache[i].ssid;
    network["bssid"] = nearbyWifiCache[i].bssid;
    network["rssi"] = nearbyWifiCache[i].rssi;
    network["channel"] = nearbyWifiCache[i].channel;
    network["auth"] = getWifiAuthModeLabel(nearbyWifiCache[i].authMode);
    network["hidden"] = nearbyWifiCache[i].hidden;
  }
}

static void startNearbyWifiScan() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  int16_t existingScan = WiFi.scanComplete();
  if (existingScan == WIFI_SCAN_RUNNING) {
    nearbyWifiScanInProgress = true;
    return;
  }
  if (existingScan >= 0) {
    WiFi.scanDelete();
  }

  lastNearbyWifiScanStartMs = millis();
  int16_t rc = WiFi.scanNetworks(true, true);
  if (rc == WIFI_SCAN_RUNNING) {
    nearbyWifiScanInProgress = true;
    return;
  }

  nearbyWifiScanInProgress = false;
  if (rc >= 0) {
    updateNearbyWifiCache(rc);
    lastNearbyWifiScanCompleteMs = millis();
    WiFi.scanDelete();
  }
}

static void serviceNearbyWifiScan() {
  if (WiFi.status() != WL_CONNECTED) {
    nearbyWifiScanInProgress = false;
    return;
  }

  int16_t scanState = WiFi.scanComplete();
  if (scanState == WIFI_SCAN_RUNNING) {
    nearbyWifiScanInProgress = true;
    return;
  }

  if (scanState >= 0) {
    updateNearbyWifiCache(scanState);
    lastNearbyWifiScanCompleteMs = millis();
    nearbyWifiScanInProgress = false;
    WiFi.scanDelete();
    return;
  }

  unsigned long now = millis();
  bool needsRefresh = (lastNearbyWifiScanCompleteMs == 0) ||
                      (now - lastNearbyWifiScanCompleteMs >= WIFI_SCAN_REFRESH_MS);
  bool retryWindowPassed = (lastNearbyWifiScanStartMs == 0) ||
                           (now - lastNearbyWifiScanStartMs >= WIFI_SCAN_RETRY_MS);
  if (needsRefresh && retryWindowPassed) {
    startNearbyWifiScan();
  }
}

static int32_t measureNetworkPingMs() {
  if (WiFi.status() != WL_CONNECTED) {
    return -1;
  }

  // "Ping" lightweight: measure TCP connect RTT to Google DNS.
  // This avoids extra dependencies (ICMP ping library).
  WiFiClient probe;
  probe.setTimeout(800);
  IPAddress target(8, 8, 8, 8);

  yield();
  unsigned long start = millis();
  bool ok = probe.connect(target, 53);
  unsigned long elapsed = millis() - start;
  probe.stop();

  return ok ? static_cast<int32_t>(elapsed) : -1;
}

// ============= Publish Setting =============
void dataPublishSetting() {
  // Pastikan MQTT connected
  if (!mqtt.connected()) {
    logPrintln("[Data] MQTT not connected, cannot publish setting");
    return;
  }

  if (!dataIsPublishReady()) {
    unsigned long now = millis();
    if (now - lastDeferredPublishLogMs >= 1000UL) {
      lastDeferredPublishLogMs = now;
      logPrintf("[Data] Deferring setting publish until MQTT cooldown passes (%lu ms remaining)\n",
                MQTT_POST_CONNECT_PUBLISH_COOLDOWN_MS - (now - lastMqttConnectMs));
    }
    return;
  }
  
  // Cek apakah topic sudah di-inisialisasi
  if (topicSetting.length() == 0) {
    logPrintln("[Data] ERROR: topicSetting not initialized");
    return;
  }
  StaticJsonDocument<2048> doc;
  doc["automation"] = cfg.setting.automation;
  doc["max_temp"] = cfg.setting.max_temp;
  doc["min_temp"] = cfg.setting.min_temp;
  doc["set_temp"] = cfg.setting.set_temp;
  doc["set_mode"] = cfg.setting.set_mode;
  doc["default_fan"] = cfg.setting.default_fan;
  doc["temp_cooldown"] = cfg.setting.temp_control_cooldown_sec;
  doc["auto_recovery"]   = cfg.setting.auto_recovery_sec;

// 2. === LOGIKA STATUS AUTOMATION (PINDAHAN DARI TOPIC DATA) ===
  String sysMode = "AUTO";
  String pauseReason = "-";
  int remainingSec = 0;

  if (cfg.setting.manualModeActive) {
    sysMode = "MANUAL_RECOVERY";
    pauseReason = "MANUAL_OVERRIDE";
    
    // Hitung sisa waktu snapshot saat ini
    unsigned long elapsed = millis() - cfg.setting.lastManualControlMs;
    // Asumsi 3 menit (180000ms)
    long rem = 180000UL - elapsed;
    if (rem < 0) rem = 0;
    remainingSec = rem / 1000;
    
  } else if (!cfg.setting.automation) {
    sysMode = "OFF";
    pauseReason = "USER_DISABLED";
  } else {
    sysMode = "AUTO";
  }
  // Masukkan ke JSON
  doc["system_mode"] = sysMode;
  doc["pause_reason"] = pauseReason;
  // Kita kirim sisa waktu saat status ini berubah. 
  // Dashboard bisa menggunakannya untuk countdown statis atau sekedar info.
  doc["manual_remaining_s"] = remainingSec; 

  
  // ============================================================
  JsonArray daily = doc.createNestedArray("daily");
  for (int i = 0; i < 7; ++i) {
    JsonObject dayObj = daily.createNestedObject();
    dayObj["day"] = DAY_NAMES[i];
    if (cfg.setting.daily[i].type == DAY_PART_TIME) {
      dayObj["type"] = "part_time";
      JsonObject w = dayObj.createNestedObject("waktu");
      w["mulai"] = cfg.setting.daily[i].waktu.mulai;
      w["selesai"] = cfg.setting.daily[i].waktu.selesai;
    } else if (cfg.setting.daily[i].type == DAY_LIBUR) {
      dayObj["type"] = "libur";
    } else {
      dayObj["type"] = "full_time";
    }
  }

  String payload;
  payload.reserve(1500);
  serializeJson(doc, payload);

  // Flush pending SSL data sebelum publish
  mqtt.loop();
  yield();
  esp_task_wdt_reset();
  
  bool published = mqtt.publish(topicSetting.c_str(), payload.c_str(), true);
  if (!published) {
    // Retry 1: flush + short delay
    mqtt.loop();
    delay(100);
    published = mqtt.publish(topicSetting.c_str(), payload.c_str(), true);
  }
  if (!published) {
    // Retry 2: longer delay
    mqtt.loop();
    delay(200);
    published = mqtt.publish(topicSetting.c_str(), payload.c_str(), true);
  }
  
  if (published) {
    consecutivePublishFails = 0;
    logPrintf("[Data] ✓ Published setting successfully to: %s\n", topicSetting.c_str());
  } else {
    consecutivePublishFails++;
    logPrintf("[Data] ✗ Failed to publish setting to: %s (%d bytes, state=%d, fails=%d)\n", 
              topicSetting.c_str(), payload.length(), mqtt.state(), consecutivePublishFails);
  }
}

void syncTime() {
  if (WiFi.status() != WL_CONNECTED) {
    logPrintln("[Time] Skip NTP sync - WiFi not connected");
    return;
  }

  logPrintln("[Time] Requesting confirmed NTP sync...");
  cfgSyncTime(5000);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 250)) {
    logPrintf("[Time] Current after sync attempt: %04d-%02d-%02d %02d:%02d:%02d\n",
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    logPrintln("[Time] Failed to read current time after sync attempt");
  }

  esp_task_wdt_reset();
  yield();
}

// Helper: format seconds ke format "Xh Ym Zs"
static String formatDuration(unsigned long totalSeconds) {
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  char buf[32];
  snprintf(buf, sizeof(buf), "%luh %lum %lus", hours, minutes, seconds);
  return String(buf);
}

// Helper: waktu WIB saat ini sebagai string
static String getWibTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "unknown";
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d WIB",
           timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
           timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  return String(buf);
}

// Waktu WIB terakhir saat device benar-benar online (MQTT connected).
// Diupdate setiap heartbeat dan sebelum graceful disconnect.
static String lastOnlineWibStr = "unknown";

// Disconnect timestamp & offline duration — di-reset setiap kali disconnect terjadi
// agar reconnect berikutnya (dalam boot yang sama) reload dari file terbaru.
static unsigned long bootDisconnectTimestamp = 0;
static bool bootTsLoaded = false;
static String cachedOfflineDuration = "";

static void buildLwtPayload() {
  StaticJsonDocument<256> doc;
  doc["status"] = "offline";
  doc["mqtt_log_enabled"] = logGetMqttEnabled();
  lwtPayload = "";
  serializeJson(doc, lwtPayload);
}

// Fungsi helper untuk publish retained status payload
static bool publishStatusRetainedPayload(const String& payload) {
  if (!mqtt.connected()) {
    logPrintln("[Data] Cannot publish status - MQTT not connected");
    return false;
  }
  if (topicStatus.length() == 0) {
    logPrintln("[Data] Cannot publish status - topicStatus is empty");
    return false;
  }

  bool published = mqtt.publish(topicStatus.c_str(), payload.c_str(), true);
  if (!published) {
    consecutivePublishFails++;
    logPrintf("[Data] Failed to publish retained status to: %s (fails=%d)\n", topicStatus.c_str(), consecutivePublishFails);
    return false;
  }

  consecutivePublishFails = 0;
  mqtt.loop();
  delay(50);
  return true;
}

static bool publishDataRetainedPayload(const String& payload) {
  if (!mqtt.connected()) {
    logPrintln("[Data] Cannot publish data - MQTT not connected");
    return false;
  }
  if (topicData.length() == 0) {
    logPrintln("[Data] Cannot publish data - topicData is empty");
    return false;
  }

  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(payload.c_str());
  const size_t payloadLength = payload.length();

  bool ok = mqtt.beginPublish(topicData.c_str(), payloadLength, true);
  if (ok) {
    size_t written = mqtt.write(bytes, payloadLength);
    ok = (written == payloadLength) && (mqtt.endPublish() == 1);
  }
  if (!ok) {
    consecutivePublishFails++;
    logPrintf("[Data] Failed to publish retained data to: %s (len=%u, fails=%d)\n",
              topicData.c_str(), static_cast<unsigned>(payloadLength), consecutivePublishFails);
    return false;
  }

  consecutivePublishFails = 0;
  mqtt.loop();
  delay(50);
  return true;
}

// Fungsi helper untuk publish status
void publishStatus(const String& status) {
  if (!mqtt.connected()) return;

  if (status == "online") {
      StaticJsonDocument<512> doc;
      
      doc["status"] = "online";
      doc["disconnect_reason"] = getDisconnectReasonStr(lastDisconnectReason);
      
      // Load disconnect timestamp sekali per boot (jika sudah di-cache, pakai nilai lama)
      if (!bootTsLoaded) {
        bootDisconnectTimestamp = cfgLoadDisconnectTimestamp();
        bootTsLoaded = true;
      }
      unsigned long disconnectTimestamp = bootDisconnectTimestamp;
      if (disconnectTimestamp > 1600000000UL) { // Timestamp valid (NTP synced)
        // Konversi ke WIB string untuk last_disconnect
        time_t ts = (time_t)disconnectTimestamp;
        struct tm* tmInfo = localtime(&ts);
        if (tmInfo) {
          char wibBuf[32];
          snprintf(wibBuf, sizeof(wibBuf), "%04d-%02d-%02d %02d:%02d:%02d WIB",
                   tmInfo->tm_year + 1900, tmInfo->tm_mon + 1, tmInfo->tm_mday,
                   tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
          doc["last_disconnect"] = wibBuf;
        } else {
          doc["last_disconnect"] = "unknown";
        }
        // Hitung durasi offline - di-cache agar tidak terus bertambah tiap heartbeat
        if (cachedOfflineDuration.isEmpty()) {
          unsigned long currentTimestamp = (unsigned long)time(nullptr);
          if (currentTimestamp > 1600000000UL && currentTimestamp > disconnectTimestamp) {
            unsigned long offlineSec = currentTimestamp - disconnectTimestamp;
            cachedOfflineDuration = formatDuration(offlineSec);
            logPrintf("[Data] Offline duration: %s (%lu seconds)\n", cachedOfflineDuration.c_str(), offlineSec);
          } else {
            cachedOfflineDuration = "0h 0m 0s";
            logPrintln("[Data] Time not synced yet or invalid, offline duration = 0");
          }
        }
        doc["offline_duration"] = cachedOfflineDuration;
      } else {
        doc["last_disconnect"] = "unknown";
        doc["offline_duration"] = "0h 0m 0s";
        if (disconnectTimestamp > 0) {
          logPrintln("[Data] Invalid disconnect timestamp (pre-NTP), ignoring");
        }
      }
      
      doc["mqtt_log_enabled"] = logGetMqttEnabled();

      String payload;
      serializeJson(doc, payload);
      publishStatusRetainedPayload(payload);
      
      logPrintf("[Data] Status published: %s (Reason: %s)\n", 
                status.c_str(), getDisconnectReasonStr(lastDisconnectReason).c_str());
  } 
  else if (status == "offline") {
      StaticJsonDocument<256> doc;
      doc["status"] = "offline";
      doc["reason"] = getDisconnectReasonStr(lastDisconnectReason);
      doc["mqtt_log_enabled"] = logGetMqttEnabled();
      
      String payload;
      serializeJson(doc, payload);
      publishStatusRetainedPayload(payload);
      
      // Simpan disconnect timestamp untuk perhitungan offline_duration berikutnya
      cfgSaveDisconnectTimestamp();
      
      logPrintln("[Data] Offline status published");
  }
  else {
      // Status lain (rebooting, updating, etc.)
      publishStatusRetainedPayload(status);
  }
}

// ============= MQTT Connection =============

static unsigned long lastMqttRetryMs = 0;
static uint8_t mqttRetryAttempt = 0;
const uint8_t MAX_RETRY_ATTEMPTS = 6;
static bool mqttInitialized = false;

// Exponential backoff: 10s, 20s, 40s, 80s, 160s, 300s (max)
const unsigned long RETRY_INTERVALS_MS[] = {
  10000,   // 10 detik
  20000,   // 20 detik
  40000,   // 40 detik
  80000,   // 1.3 menit
  160000,  // 2.6 menit
  300000   // 5 menit (max)
};

// Tambahkan random jitter (0-5 detik) untuk hindari thundering herd
unsigned long getRetryInterval() {
  // FIX: Cast ke same type sebelum min()
  uint8_t idx = (mqttRetryAttempt < MAX_RETRY_ATTEMPTS) ? mqttRetryAttempt : (MAX_RETRY_ATTEMPTS - 1);
  unsigned long baseInterval = RETRY_INTERVALS_MS[idx];
  
  // Random jitter ±2.5 detik (5000ms range)
  long jitter = (long)(random(5000)) - 2500;
  
  return baseInterval + jitter;
}

static void resetMqttRc2RecoveryState(const char* reason = nullptr) {
  if (mqttRc2FirstFailMs != 0 && reason && reason[0] != '\0') {
    logPrintf("[MQTT] RC -2 recovery reset: %s\n", reason);
  }
  mqttRc2ConsecutiveFails = 0;
  mqttRc2FirstFailMs = 0;
  lastMqttRc2RecoveryMs = 0;
  mqttRc2WifiRecoveryAttempted = false;
}

static void noteMqttRc2Failure() {
  unsigned long now = millis();
  if (mqttRc2FirstFailMs == 0) {
    mqttRc2FirstFailMs = now;
  }
  if (mqttRc2ConsecutiveFails < 255) {
    mqttRc2ConsecutiveFails++;
  }
  logPrintf("[MQTT] RC -2 tracker: fails=%u, stuck=%lu sec, wifi_recovery=%s\n",
            mqttRc2ConsecutiveFails,
            (now - mqttRc2FirstFailMs) / 1000UL,
            mqttRc2WifiRecoveryAttempted ? "done" : "pending");
}

static void performMqttRc2WifiRecovery() {
  lastMqttRc2RecoveryMs = millis();
  mqttRc2WifiRecoveryAttempted = true;
  lastDisconnectReason = REASON_MQTT_LOST;

  logPrintln("[MQTT] RC -2 persisted - resetting WiFi radio + TLS stack...");
  dataResetConnection();

  cfg.koneksi.wifi_ssid.trim();
  cfg.koneksi.wifi_pass.trim();

  WiFi.disconnect();
  delay(100);
  esp_task_wdt_reset();
  yield();

  WiFi.mode(WIFI_OFF);
  delay(300);
  esp_task_wdt_reset();
  yield();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(cfg.koneksi.wifi_ssid.c_str(), cfg.koneksi.wifi_pass.c_str());

  lastMqttRetryMs = 0;
  mqttRetryAttempt = 0;
  logPrintln("[MQTT] RC -2 WiFi/TLS recovery triggered");
}

static void maybeEscalateMqttRc2Failure() {
  if (mqttRc2FirstFailMs == 0 || WiFi.status() != WL_CONNECTED) {
    return;
  }

  unsigned long now = millis();
  if (!mqttRc2WifiRecoveryAttempted &&
      mqttRc2ConsecutiveFails >= MQTT_RC2_WIFI_RESET_THRESHOLD &&
      (lastMqttRc2RecoveryMs == 0 || now - lastMqttRc2RecoveryMs >= MQTT_RC2_RECOVERY_COOLDOWN_MS)) {
    performMqttRc2WifiRecovery();
    return;
  }

  if (mqttRc2WifiRecoveryAttempted &&
      now - mqttRc2FirstFailMs >= MQTT_RC2_RESTART_AFTER_MS &&
      now - lastMqttRc2RecoveryMs >= MQTT_RC2_RECOVERY_COOLDOWN_MS) {
    logPrintf("[MQTT] RC -2 persisted for %lu sec after WiFi/TLS recovery - restarting ESP32...\n",
              (now - mqttRc2FirstFailMs) / 1000UL);
    lastDisconnectReason = REASON_MQTT_LOST;
    delay(200);
    esp_task_wdt_reset();
    yield();
    ESP.restart();
  }
}

static void ensureMqttConnected() {
  // Flag untuk memastikan publish offline hanya sekali per periode disconnect
  static bool offlineNotifyDone = false;

  if (WiFi.status() != WL_CONNECTED) {
    logPrintln("[MQTT] WiFi not connected, skip MQTT connection");
    return;
  }
  WiFi.setSleep(false);

  if (cfg.device_id.length() == 0) {
    logPrintln("[MQTT] ERROR: device_id is empty - cannot build topics/connect");
    return;
  }
  
  if (mqtt.connected()) {
    mqttRetryAttempt = 0; // Reset counter saat berhasil
    return;
  }
  
  // Pastikan topics diinisialisasi
  if (topicStatus.length() == 0 || topicData.length() == 0) {
    logPrintln("[MQTT] Topics not initialized, initializing now...");
    dataBuildTopics(cfg.device_id);
    dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port, 
                  cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
  }
  
  // === EXPONENTIAL BACKOFF CHECK ===
  unsigned long now = millis();
  unsigned long retryInterval = getRetryInterval();
  
  if (now - lastMqttRetryMs < retryInterval) {
    // Log setiap 30 detik agar tidak spam
    static unsigned long lastLogMs = 0;
    if (now - lastLogMs >= 30000) {
      unsigned long remainingMs = retryInterval - (now - lastMqttRetryMs);
      logPrintf("[MQTT] Waiting %lu seconds before retry (attempt %d/%d)\n", 
                remainingMs / 1000, mqttRetryAttempt + 1, MAX_RETRY_ATTEMPTS);
      lastLogMs = now;
    }
    return;
  }
  
  lastMqttRetryMs = now;
  mqttRetryAttempt++;
  
  // Reset SSL client
  espClient.stop();
  delay(100);
  yield();
  
  logPrintln("[MQTT] mencoba konek ke server...");
  logPrintf("[MQTT] Server: %s:%d (SSL)\n", cfg.koneksi.mqtt_host.c_str(), 
            cfg.koneksi.mqtt_port);
  logPrintf("[MQTT] Client ID: %s\n", cfg.device_id.c_str());
  logPrintf("[MQTT] Retry attempt: %d/%d\n", mqttRetryAttempt, MAX_RETRY_ATTEMPTS);
  
  esp_task_wdt_reset();
  yield();
  
  // Single connection attempt
  bool ok;
  const char* clientId = cfg.device_id.c_str();
  unsigned long connectStart = millis();
  
  // Restore lastOnlineWibStr dari epoch tersimpan di LittleFS (sekali saat boot).
  // Tanpa ini, LWT dan corrected offline akan punya last_online = "unknown" atau
  // waktu saat connect (bukan waktu terakhir device benar-benar online).
  if (!bootTsLoaded) {
    bootDisconnectTimestamp = cfgLoadDisconnectTimestamp();
    bootTsLoaded = true;
    if (bootDisconnectTimestamp > 1600000000UL) {
      time_t t = (time_t)bootDisconnectTimestamp;
      struct tm* tmInfo = localtime(&t);
      if (tmInfo) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d WIB",
                 tmInfo->tm_year + 1900, tmInfo->tm_mon + 1, tmInfo->tm_mday,
                 tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
        lastOnlineWibStr = buf;
      }
    }
  }
  
  buildLwtPayload();
  if (mqttUser.length()) {
    ok = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                      topicStatus.c_str(), 1, true, lwtPayload.c_str());
  } else {
    ok = mqtt.connect(clientId, topicStatus.c_str(), 1, true, lwtPayload.c_str());
  }
  
  unsigned long connectDuration = millis() - connectStart;
  logPrintf("[MQTT] Connection attempt took %lu ms\n", connectDuration);

  // Deteksi SSL heap exhaustion: jika connect() return < 5ms,
  // berarti tidak ada koneksi yang benar-benar dicoba (mbedTLS gagal alokasi heap).
  if (!ok && connectDuration < 5) {
    logPrintf("[MQTT] ✗ SSL instant-fail (0ms) - Free heap: %u bytes\n", ESP.getFreeHeap());
    logPrintln("[MQTT] Kemungkinan heap exhausted. Auto-restart dalam 3 detik...");
    delay(3000);
    ESP.restart();
  }
  
  esp_task_wdt_reset();
  yield();
  
  if (ok) {
    // SUCCESS - Reset counter
    mqttRetryAttempt = 0;
    resetMqttRc2RecoveryState("mqtt connected");
    
    if (isManualDisconnect) {
      isManualDisconnect = false;
      logPrintln("[MQTT] Reconnected after config save (Counter ignored)");
    } else {
      mqttReconnectCount++;
      logPrintf("[MQTT] Reconnect Count: %lu\n", mqttReconnectCount);
    }
    
    logPrintln("[MQTT] ✓ Connected successfully!");
    lastMqttConnectMs = millis(); // Catat waktu connect untuk cooldown guard
    offlineNotifyDone = false; // Reset agar bisa notify kembali jika nanti disconnect lagi

    // Koreksi retained offline status sebelum go online.
    // LWT broker menyimpan static "CONNECTION_LOST" - timpa dengan reason sesungguhnya.
    {
      StaticJsonDocument<256> offDoc;
      offDoc["status"] = "offline";
      offDoc["reason"] = getDisconnectReasonStr(lastDisconnectReason);
      offDoc["mqtt_log_enabled"] = logGetMqttEnabled();
      String offPayload;
      serializeJson(offDoc, offPayload);
      publishStatusRetainedPayload(offPayload);
      mqtt.loop();
    }

    publishStatus("online");
    
    esp_task_wdt_reset();
    yield();
    
    // Subscribe to all command topics (log failures so it isn't silent)
    if (!mqtt.subscribe(cmdSetting.c_str()))     logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdSetting.c_str());
    if (!mqtt.subscribe(cmdKoneksi.c_str()))     logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdKoneksi.c_str());
    if (!mqtt.subscribe(cmdSchedule.c_str()))    logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdSchedule.c_str());
    if (!mqtt.subscribe(cmdRemote.c_str()))      logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdRemote.c_str());
    if (topicCmdUpdate.length() > 0) {
      if (!mqtt.subscribe(topicCmdUpdate.c_str())) logPrintf("[MQTT] ✗ Subscribe failed: %s\n", topicCmdUpdate.c_str());
    } else {
      logPrintln("[MQTT] Skip subscribe: topicCmdUpdate empty");
    }
    if (!mqtt.subscribe(cmdDiagnostics.c_str())) logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdDiagnostics.c_str());
    if (!mqtt.subscribe(cmdInfo.c_str()))        logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdInfo.c_str());
    if (!mqtt.subscribe(cmdReboot.c_str()))      logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdReboot.c_str());
    if (!mqtt.subscribe(cmdLogs.c_str()))        logPrintf("[MQTT] ✗ Subscribe failed: %s\n", cmdLogs.c_str());
    logPrintln("[MQTT] Subscribed to command topics");
    
    esp_task_wdt_reset();
    yield();
    
    // Publish setting ditunda sampai post-connect cooldown terlewati.
  } else {
    int rc = mqtt.state();
    logPrintf("[MQTT] ✗ Gagal konek, rc=%d (", rc);
    switch(rc) {
      case -4: Serial.print("Timeout - server tidak merespons"); break;
      case -3: Serial.print("Connection lost"); break;
      case -2: Serial.print("TCP/TLS gagal - cek host/port/sertifikat"); break;
      case -1: Serial.print("Disconnected"); break;
      case 1: Serial.print("Bad protocol"); break;
      case 2: Serial.print("Bad client ID"); break;
      case 3: Serial.print("Server unavailable"); break;
      case 4: Serial.print("Bad credentials - cek username/password"); break;
      case 5: Serial.print("Unauthorized"); break;
      default: Serial.print("Unknown error");
    }
    logPrintln(")");

    if (rc == -2) {
      noteMqttRc2Failure();
      maybeEscalateMqttRc2Failure();
    } else {
      resetMqttRc2RecoveryState("non RC-2 failure");
    }
    
    // Log next retry time
    unsigned long nextRetryMs = getRetryInterval();
    logPrintf("[MQTT] Next retry in %lu seconds (attempt %d/%d)\n", 
              nextRetryMs / 1000, mqttRetryAttempt + 1, MAX_RETRY_ATTEMPTS);

    // Setelah 3 retry gagal & WiFi masih terhubung, publish retained offline sekali
    // agar dashboard langsung tahu device offline tanpa menunggu LWT broker.
    if (mqttRetryAttempt >= 3 && !offlineNotifyDone && WiFi.status() == WL_CONNECTED) {
      logPrintln("[MQTT] 3 retry gagal - mencoba publish status offline...");
      lastDisconnectReason = REASON_MQTT_LOST;
      espClient.stop();
      yield();
      bool notifyOk;
      buildLwtPayload();
      if (mqttUser.length()) {
        notifyOk = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                                topicStatus.c_str(), 1, true, lwtPayload.c_str());
      } else {
        notifyOk = mqtt.connect(clientId, topicStatus.c_str(), 1, true, lwtPayload.c_str());
      }
      if (notifyOk) {
        publishStatus("offline");
        delay(100);
        mqtt.disconnect();
        espClient.stop();
        logPrintln("[MQTT] ✓ Status offline berhasil dipublish setelah 3 retry gagal");
      } else {
        logPrintln("[MQTT] ✗ Offline notify gagal - broker tidak terjangkau, LWT akan aktif");
      }
      offlineNotifyDone = true;
    }
  }
}

// ==========================================
// FUNGSI INIT DATA (SECURE MQTT)
// ==========================================
void dataInit() {
  // 1. Sync Waktu dulu (CRITICAL STEP)
  // Tanpa ini, setCACert akan menolak koneksi karena "Certificate Not Yet Valid"
  syncTime();

  // 2. Konfigurasi SSL - Gunakan Root CA
  // HAPUS atau KOMENTARI baris ini: espClient.setInsecure();
  
  if (PROD_ROOT_CA != NULL) {
      logPrintln("[Data] Setting Secure Root CA Certificate...");
      espClient.setCACert(PROD_ROOT_CA); // Menggunakan sertifikat dari config.h
  } else {
      logPrintln("[Data] WARNING: Root CA is NULL! Fallback to Insecure.");
      espClient.setInsecure();
  }
  
  // 3. Konfigurasi MQTT Client
  mqtt.setBufferSize(2048);    // Buffer cukup untuk payload JSON, hemat heap untuk SSL
  mqtt.setKeepAlive(30);       // 30 detik keepalive
  mqtt.setSocketTimeout(15);   // 15 detik timeout socket
  mqtt.setCallback(handleMqttMessage);
  
  // Reset flags
  mqttInitialized = false;
  isProcessingCommand = false;
  resetMqttRc2RecoveryState();

  // 4. Cek penyebab restart
  // ESP32 brownout detector memberi sinyal ESP_RST_BROWNOUT saat tegangan drop
  // (power loss sesungguhnya). ESP_RST_POWERON = power cycle biasa (tidak bisa
  // dibedakan antara sengaja atau tidak karena CPU tidak sempat tulis apapun).
  esp_reset_reason_t r = esp_reset_reason();
  if (r == ESP_RST_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT) {
      lastDisconnectReason = REASON_WDT_RESET;
      logPrintln("[Data] Disconnect reason: WATCHDOG RESET");
  } else if (r == ESP_RST_BROWNOUT) {
      lastDisconnectReason = REASON_POWER_LOSS;
      logPrintln("[Data] Disconnect reason: POWER LOSS (brownout detected)");
  } else if (r == ESP_RST_PANIC) {
      lastDisconnectReason = REASON_CRASH;
      logPrintln("[Data] Disconnect reason: SOFTWARE CRASH (panic)");
  } else if (r == ESP_RST_POWERON) {
      lastDisconnectReason = REASON_POWER_ON;
      logPrintln("[Data] Disconnect reason: POWER ON / POWER CYCLE");
  } else if (r == ESP_RST_SW) {
      lastDisconnectReason = REASON_POWER_ON;
      logPrintln("[Data] Disconnect reason: SOFTWARE RESET (OTA/command)");
  } else {
      lastDisconnectReason = REASON_POWER_ON;
      logPrintf("[Data] Disconnect reason: UNKNOWN (esp_reset_reason=%d)\n", (int)r);
  }
  
  // Reset retry timer agar koneksi bisa langsung dicoba
  lastMqttRetryMs = 0;
  
  // 5. Build Topics & Server Config
  if (cfg.device_id.length() > 0) {
    dataBuildTopics(cfg.device_id);
    dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port, 
                  cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
                  
    logPrintf("[Data] MQTT topics built for device: %s\n", cfg.device_id.c_str());
  }
  
  // 6. Setup logger MQTT publisher
  logSetMqttPublisher(publishLogs);
  logPrintln("[Data] Logger MQTT publisher configured");
  
  logPrintln("[Data] MQTT Config: Buffer=4096, KeepAlive=30s, Timeout=15s");
  logPrintf("[Data] MQTT Server: %s:%d (SECURE)\n", cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port);

  extern void activityBuildTopic(const String&);
  activityBuildTopic(cfg.device_id);
  
  esp_task_wdt_reset();
  yield();
}

void dataKeepAlive() {
  // Lightweight: hanya mqtt.loop() untuk process incoming messages & keepalive.
  // Tidak melakukan connection management, heartbeat, atau reconnect.
  if (mqtt.connected()) {
    mqtt.loop();
  }
}

void dataLoop() {
  static bool wasMqttConnected = false;
  static unsigned long lastHeartbeatMs = 0;

  if (WiFi.status() != WL_CONNECTED) {
    resetMqttRc2RecoveryState("wifi disconnected");
    // Gunakan epoch dari WiFi event handler jika tersedia (lebih akurat).
    unsigned long disconnectEpoch = wifiDisconnectEpoch;
    if (disconnectEpoch > 1600000000UL) {
      wifiDisconnectEpoch = 0; // Konsumsi flag
      // Update lastOnlineWibStr dengan waktu tepat saat WiFi disconnect
      time_t t = (time_t)disconnectEpoch;
      struct tm* tmInfo = localtime(&t);
      if (tmInfo) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d WIB",
                 tmInfo->tm_year + 1900, tmInfo->tm_mon + 1, tmInfo->tm_mday,
                 tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
        lastOnlineWibStr = buf;
      }
    }
    if (mqtt.connected()) {
      // Do NOT send MQTT DISCONNECT here.
      // Forcing socket close is more reliable for LWT/offline propagation.
      cfgSaveDisconnectTimestamp(disconnectEpoch > 1600000000UL ? disconnectEpoch : 0);
      espClient.stop();
      // Reset cache agar reconnect berikutnya reload timestamp terbaru
      bootTsLoaded = false;
      cachedOfflineDuration = "";
    } else if (wasMqttConnected) {
      // mqtt.connected() sudah false (TCP RST), tapi kita belum sempat save
      cfgSaveDisconnectTimestamp(disconnectEpoch > 1600000000UL ? disconnectEpoch : 0);
      bootTsLoaded = false;
      cachedOfflineDuration = "";
    }
    wasMqttConnected = false;
    return;
  }

  serviceNearbyWifiScan();
  
  if (!mqtt.connected()) {
    // Simpan disconnect timestamp sekali saat MQTT terdeteksi offline
    if (wasMqttConnected) {
      if (!isManualDisconnect) {
        // Hanya simpan & reset cache untuk disconnect sesungguhnya
        cfgSaveDisconnectTimestamp();
        bootTsLoaded = false;
        cachedOfflineDuration = "";
      }
      wasMqttConnected = false;
    }
    ensureMqttConnected();
    return;
  }
  
  wasMqttConnected = true;
  mqtt.loop();

  if (!mqttInitialized && dataIsPublishReady()) {
    yield();
    esp_task_wdt_reset();
    dataPublishSetting();
    if (mqtt.connected()) {
      mqttInitialized = true;
    }
  }

  processQueuedMqttCommand();

  // Zombie connection detection: MQTT reports connected tapi publish terus gagal.
  // Ini terjadi saat SSL layer rusak tapi TCP masih "connected".
  if (consecutivePublishFails >= 3) {
    logPrintln("[MQTT] \u2717 Zombie connection detected (3+ publish fails) \u2014 forcing clean reconnect");
    consecutivePublishFails = 0;
    mqtt.disconnect();
    espClient.stop();
    return;
  }

  // Immediate error code change detection — tidak menunggu siklus publish data.
  {
    static String lastMonitoredErrors = "";
    static bool errorMonitorInit = false;
    BPFrame frame = sensorGetLastFrame();
    if (frame.valid) {
      if (!errorMonitorInit) {
        lastMonitoredErrors = frame.errors;
        errorMonitorInit = true;
      } else if (frame.errors != lastMonitoredErrors) {
        activityTrackError(lastMonitoredErrors, frame.errors);
        lastMonitoredErrors = frame.errors;
      }
    }
  }

  // Heartbeat: simpan timestamp periodik setiap 30 detik selama connected.
  // Jika device mati mendadak (power loss), timestamp terakhir tetap tersimpan
  // sehingga offline_duration bisa dihitung saat device hidup kembali.
  unsigned long now = millis();

  if (now - lastHeartbeatMs >= 120000UL) { // 2 menit
    lastHeartbeatMs = now;
    cfgSaveLastAlive(); // checkpoint power loss - file terpisah, tidak timpa disconnect_ts
    publishStatus("online"); // refresh retained online payload

    // Refresh LWT payload setiap 4 heartbeat (2 menit).
    // Cukup update retained status dan LWT payload di memory.
    static uint8_t lwtRefreshCounter = 0;
    lwtRefreshCounter++;
    if (lwtRefreshCounter >= 6) {
      lwtRefreshCounter = 0;
      lastOnlineWibStr = getWibTimeString();
      buildLwtPayload(); // Update LWT payload di memory untuk reconnect berikutnya
      publishStatus("online"); // Refresh retained status di broker
      logPrintln("[MQTT] LWT payload updated + status refreshed");
    }
  }
  // Selalu update waktu terakhir online selama connected,
  // agar publishStatus("offline") menampilkan waktu yang akurat.
  lastOnlineWibStr = getWibTimeString();
}

void dataDisconnect() {
  if (mqtt.connected()) {
    // For graceful disconnect, send offline status manually
    logPrintln("[MQTT] Sending offline status before graceful disconnect...");
    lastOnlineWibStr = getWibTimeString(); // Update tepat sebelum disconnect
    publishStatus("offline");
    delay(100); // Give time for message to be sent
    mqtt.disconnect();
    espClient.stop(); // Stop SSL client
    
    // Disconnect timestamp sudah disimpan oleh publishStatus("offline")
    // Create graceful shutdown flag
    if (LittleFS.begin()) {
      File f = LittleFS.open("/shutdown.flag", "w");
      if (f) {
        f.println("graceful");
        f.close();
        logPrintln("[MQTT] Graceful shutdown flag created");
      }
      LittleFS.remove("/running.flag");
    }
    
    logPrintln("[MQTT] Disconnected gracefully");
  }
}

bool dataIsConnected() {
  return mqtt.connected();
}

void dataNotifyOffline() {
  // Publish retained offline status saat WiFi masih ada tapi MQTT tidak terjangkau.
  // Dipanggil setelah 3x WiFi retry atau 3x MQTT retry gagal.
  if (WiFi.status() != WL_CONNECTED) return;
  if (topicStatus.length() == 0) return;
  if (mqtt.connected()) return; // Masih connected, tidak perlu notify
  logPrintln("[MQTT] Mempublish status offline (retry gagal 3x)...");
  espClient.stop();
  yield();
  bool ok;
  const char* clientId = cfg.device_id.c_str();
  buildLwtPayload();
  if (mqttUser.length()) {
    ok = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                      topicStatus.c_str(), 1, true, lwtPayload.c_str());
  } else {
    ok = mqtt.connect(clientId, topicStatus.c_str(), 1, true, lwtPayload.c_str());
  }
  if (ok) {
    publishStatus("offline");
    delay(100);
    mqtt.disconnect();
    espClient.stop();
    logPrintln("[MQTT] \u2713 Status offline berhasil dipublish");
  } else {
    logPrintln("[MQTT] \u2717 Offline notify gagal - broker tidak terjangkau");
  }
}

void dataResetConnection() {
  // Reset SSL client dan disconnect MQTT untuk memaksa fresh connection
  logPrintln("[MQTT] Resetting connection (WiFi reconnected)...");
  
  if (mqtt.connected()) {
    // Jangan publish "offline" di sini — cfgSaveDisconnectTimestamp() di dalam
    // publishStatus("offline") akan menimpa timestamp disconnect asli dengan waktu
    // reconnect, membuat offline_duration menjadi hanya ~3 detik.
    // LWT (broker-side) sudah menangani retained offline status.
    mqtt.disconnect();
  }
  
  espClient.stop();
  delay(200); // Beri waktu untuk cleanup
  yield();
  
  // Reinitialize SSL client - keep CA validation policy consistent with dataInit()
  if (PROD_ROOT_CA != NULL) {
    espClient.setCACert(PROD_ROOT_CA);
  } else {
    espClient.setInsecure();
  }
  
  // Reset retry timer agar bisa langsung connect
  lastMqttRetryMs = 0;
  mqttInitialized = false;
  
  logPrintln("[MQTT] Connection reset complete, ready for fresh connection");
}

void dataForceConnect() {
  // Cek WiFi terlebih dahulu
  if (WiFi.status() != WL_CONNECTED) {
    logPrintln("[MQTT] WiFi not connected, cannot force connect");
    return;
  }
  
  if (mqtt.connected()) {
    logPrintln("[MQTT] Already connected");
    return;
  }
  
  // Pastikan topics sudah diinisialisasi
  if (topicStatus.length() == 0) {
    logPrintln("[MQTT] Initializing topics before connection...");
    dataBuildTopics(cfg.device_id);
    dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port, 
                  cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
  }
  
  // Reset SSL client untuk memastikan koneksi bersih
  espClient.stop();
  delay(100);
  yield();
  
  // Bypass cooldown untuk force connection
  logPrintln("[MQTT] Force connecting to server...");
  logPrintf("[MQTT] Server: %s:%d (SSL)\n", cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port);
  logPrintf("[MQTT] Client ID: %s\n", cfg.device_id.c_str());
  esp_task_wdt_reset();
  yield();
  
  // Single connection attempt with timeout tracking
  bool ok;
  const char* clientId = cfg.device_id.c_str();
  unsigned long connectStart = millis();
  
  // Gunakan LWT (Last Will and Testament) agar status offline otomatis terdeteksi broker
  buildLwtPayload();
  if (mqttUser.length()) {
    ok = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                      topicStatus.c_str(), 1, true, lwtPayload.c_str());
  } else {
    ok = mqtt.connect(clientId, topicStatus.c_str(), 1, true, lwtPayload.c_str());
  }
  
  unsigned long connectDuration = millis() - connectStart;
  logPrintf("[MQTT] Connection attempt took %lu ms\n", connectDuration);

  // Deteksi SSL heap exhaustion: jika connect() return < 5ms,
  // berarti tidak ada koneksi yang benar-benar dicoba (mbedTLS gagal alokasi heap).
  if (!ok && connectDuration < 5) {
    logPrintf("[MQTT] ✗ SSL instant-fail (0ms) - Free heap: %u bytes\n", ESP.getFreeHeap());
    logPrintln("[MQTT] Kemungkinan heap exhausted. Auto-restart dalam 3 detik...");
    delay(3000);
    ESP.restart();
  }
  
  esp_task_wdt_reset();
  yield();
  
  if (ok) {
    resetMqttRc2RecoveryState("force connect success");
    // === [MODIFIKASI 1] Logika Counter Cerdas ===
    if (isManualDisconnect) {
      // Jika reconnect setelah save config, reset flag & jangan hitung sebagai error
      isManualDisconnect = false;
      logPrintln("[MQTT] Reconnected after config save (Counter ignored)");
    } else {
      // Murni putus jaringan -> Tambah counter
      mqttReconnectCount++;
      logPrintf("[MQTT] Reconnect Count: %lu\n", mqttReconnectCount);
    }
    // ============================================

    logPrintln("[MQTT] Force connect successful!");
    lastMqttConnectMs = millis(); // Catat waktu connect untuk cooldown guard
    publishStatus("online");
    
    // === [MODIFIKASI 2] Kirim Telemetry System Info ===
    // Kirim data IP, Uptime, dan Error Count ke topik .../info
    dataPublishInfo(); 
    // ==================================================
    
    esp_task_wdt_reset();
    yield();
    
    // Subscribe to all command topics
    mqtt.subscribe(cmdSetting.c_str());
    mqtt.subscribe(cmdKoneksi.c_str());
    mqtt.subscribe(cmdSchedule.c_str());
    mqtt.subscribe(cmdRemote.c_str());
    mqtt.subscribe(cmdReboot.c_str());
    mqtt.subscribe(cmdDiagnostics.c_str());
    mqtt.subscribe(cmdInfo.c_str());
    
    // === [MODIFIKASI 3] Wajib Subscribe Topik OTA ===
    if (topicCmdUpdate.length() > 0) {
       mqtt.subscribe(topicCmdUpdate.c_str());
    }
    // ================================================
    
    logPrintln("[MQTT] Subscribed to command topics");
    
    esp_task_wdt_reset();
    yield();
    
    // Update retry timer setelah berhasil
    lastMqttRetryMs = millis();
    
    // Publish setting hanya jika belum pernah di-init
    if (!mqttInitialized) {
      yield(); // Give time before publishing
      esp_task_wdt_reset();
      dataPublishSetting();
      mqttInitialized = true;
    }
  } else {
    int rc = mqtt.state();
    logPrintf("[MQTT] ✗ Force connect failed, rc=%d (", rc);
    switch(rc) {
      case -4: Serial.print("Timeout - server tidak merespons"); break;
      case -3: Serial.print("Connection lost"); break;
      case -2: Serial.print("TCP/TLS gagal - cek host/port/sertifikat"); break;
      case -1: Serial.print("Disconnected"); break;
      case 1: Serial.print("Bad protocol"); break;
      case 2: Serial.print("Bad client ID"); break;
      case 3: Serial.print("Server unavailable"); break;
      case 4: Serial.print("Bad credentials - cek username/password"); break;
      case 5: Serial.print("Unauthorized"); break;
      default: Serial.print("Unknown error");
    }
    logPrintln(")");
    if (rc == -2) {
      noteMqttRc2Failure();
      maybeEscalateMqttRc2Failure();
    } else {
      resetMqttRc2RecoveryState("force connect non RC-2 failure");
    }
    // Update timer even on failure untuk menghindari spam
    lastMqttRetryMs = millis();
  }
}

static String buildPublishedErrorHex(const BPFrame& frame) {
  if (!frame.valid || frame.error_val16.length() == 0) return "0000";

  unsigned int rawError = 0;
  if (sscanf(frame.error_val16.c_str(), "0x%x", &rawError) != 1 &&
      sscanf(frame.error_val16.c_str(), "%x", &rawError) != 1) {
    return "0000";
  }

  uint8_t frame4  = (uint8_t)(rawError & 0xFF);          // byte 4  = low byte
  uint8_t frame13 = (uint8_t)((rawError >> 8) & 0xFF);  // byte 13 = high byte

  char buffer[8];
  sprintf(buffer, "%02X%02X", frame4, frame13);  // e.g. "0180"
  return String(buffer);
}

// ============= Publish Suhu & Status AC =============
void dataPublishSuhu(const BPFrame& d, bool read_ok) {
  // Naikkan ukuran buffer JSON karena ada penambahan data string waktu & status
  StaticJsonDocument<2048> doc;
  bool setTempValid = sensorIsDisplayHealthy();
  
  // 1. Data Sensor AC
  doc["read_mainboard"] = read_ok;
  doc["read_display"] = setTempValid;
  doc["read_rtc"] = rtcOK;
  IRHealthStats irStats = irHealthGetStats();
  if (!irHealthIsEnabled() || (irStats.totalCommands == 0)) {
    doc["read_ir"] = "-";
  } else {
    doc["read_ir"] = irStats.lastCommandSuccess;
  }
  doc["mac"] = WiFi.macAddress();
  doc["wifi_ssid"] = WiFi.SSID();
  doc["wifi_bssid"] = WiFi.BSSIDstr();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["net_ping_ms"] = measureNetworkPingMs();
  doc["set_temp"] = setTempValid ? (int)sensorGetSetTemp() : 0;
  if (read_ok) {
    doc["power"] = d.power;
    doc["mode"] = d.mode;
    doc["fan_speed"] = d.fan;
    doc["room_temp"] = (int)d.current_temp;
    doc["frame_hex"] = d.frameHex;
    doc["error_hex"] = buildPublishedErrorHex(d);
    doc["error_code"] = d.errors;
  } else {
    doc["power"] = false;
    doc["mode"] = "-";
    doc["fan_speed"] = "-";
    doc["room_temp"] = 0;
    doc["frame_hex"] = "";
    doc["error_hex"] = "0000";
    doc["error_code"] = "-";
  }
  
  // 2. Data Koneksi

  // 3. Data Waktu (device time dari NTP/RTC)
  struct tm timeinfo;
  bool timeValid = getLocalTime(&timeinfo) && (timeinfo.tm_year + 1900) > 2020;
  if (timeValid) {
    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
    doc["device_time"] = timeStr;
  } else {
    doc["device_time"] = "UNSYNCED";
  }
  // Simpan state hanya jika frame masih fresh.
  if (read_ok) {
    lastKnownState = d;
  }
  
  String payload;
  payload.reserve(1024); 
  serializeJson(doc, payload);

  if (!mqtt.connected() || topicData.length() == 0) {
    logPrintln("[Data] Skip publish (MQTT not connected / topicData empty)");
    return;
  }

  bool ok = publishDataRetainedPayload(payload);
  if (!ok) {
    // Retry 1: flush + short delay
    mqtt.loop();
    delay(50);
    ok = publishDataRetainedPayload(payload);
  }
  if (!ok) {
    // Retry 2: longer delay
    mqtt.loop();
    delay(100);
    ok = publishDataRetainedPayload(payload);
  }
  if (ok) {
    logPrintln("[Data] Published suhu + status sistem ke MQTT");
  } else {
    logPrintf("[Data] ✗ Publish failed (state=%d)\n", mqtt.state());
  }
}

// ============================================================
// BONUS: Update handleCmdSetting() untuk clear manual mode
// jika user manually enable automation via MQTT
// ============================================================
static void handleCmdSetting(const char* payload, unsigned int length) {
  logPrintln("[CMD Setting] ========== START ==========");
  if (length > 2048) {
    logPrintln("[CMD Setting] Payload too large");
    return;
  }
  
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("[CMD Setting] JSON error: ");
    dataPublishAck("setting", "failed", "Invalid JSON format");
    logPrintln(err.c_str());
    return;
  }

  bool changed = false;

  // Immutable keys - reject any attempt to modify
  if (doc.containsKey("device_id") || doc.containsKey("ac_brand")) {
    dataPublishAck("setting", "failed", "Immutable field (device_id/ac_brand) cannot be changed");
    logPrintln("[CMD Setting] ERROR: Attempt to modify immutable field");
    return;
  }
  
  // Validasi automation - hanya boolean
  if (doc.containsKey("automation")) {
    JsonVariant autoValue = doc["automation"];

    if (!autoValue.is<bool>()) {
      logPrintln("[CMD Setting] ERROR: automation must be boolean (true/false)");
      return;
    }

    bool newAutoValue = autoValue.as<bool>();

    if (newAutoValue != cfg.setting.automation) {

      cfg.setting.automation = newAutoValue;

      if (newAutoValue) {
        // === USER MENGHIDUPKAN AUTOMATION ===
        logPrintln("[CMD Setting] Automation enabled by user");

        cfg.setting.automationPausedByUser = false;

        // Clear manual recovery jika ada
        if (cfg.setting.manualModeActive) {
          logPrintln("[CMD Setting] Clearing manual mode (user override)");
          cfg.setting.manualModeActive = false;
          cfg.setting.lastManualControlMs = millis();
        }

      } else {
        // === USER MEMATIKAN AUTOMATION ===
        logPrintln("[CMD Setting] Automation disabled by user");

        cfg.setting.automationPausedByUser = true;
        cfg.setting.manualModeActive = false;
      }

      logPrintf(
        "[CMD Setting] Automation changed to: %s\n",
        cfg.setting.automation ? "true" : "false"
      );

      changed = true;
    }
  }
  // ... rest of validation code (max_temp, min_temp, set_temp, set_mode) ...
  // [COPY DARI KODE ASLI ANDA]
  
  if (doc.containsKey("max_temp")) {
    JsonVariant maxTempValue = doc["max_temp"];
    if (maxTempValue.is<int>()) {
      int maxTemp = maxTempValue.as<int>();
      if (maxTemp <= 30) {
        cfg.setting.max_temp = maxTemp;
        logPrintf("[CMD Setting] Max temp changed to: %d\n", cfg.setting.max_temp);
        changed = true;
      } else {
        logPrintf("[CMD Setting] ERROR: max_temp must be <= 30, got: %d\n", maxTemp);
      }
    } else {
      logPrintln("[CMD Setting] ERROR: max_temp must be a number");
    }
  }
  
  if (doc.containsKey("min_temp")) {
    JsonVariant minTempValue = doc["min_temp"];
    if (minTempValue.is<int>()) {
      int minTemp = minTempValue.as<int>();
      if (minTemp >= 16) {
        cfg.setting.min_temp = minTemp;
        logPrintf("[CMD Setting] Min temp changed to: %d\n", cfg.setting.min_temp);
        changed = true;
      } else {
        logPrintf("[CMD Setting] ERROR: min_temp must be >= 16, got: %d\n", minTemp);
      }
    } else {
      logPrintln("[CMD Setting] ERROR: min_temp must be a number");
    }
  }
  
  if (doc.containsKey("set_temp")) {
    JsonVariant setTempValue = doc["set_temp"];
    if (setTempValue.is<int>()) {
      int setTemp = setTempValue.as<int>();
      if (setTemp >= 16 && setTemp <= 30) {
        cfg.setting.set_temp = setTemp;
        logPrintf("[CMD Setting] Set temp changed to: %d\n", cfg.setting.set_temp);
        changed = true;
      } else {
        logPrintf("[CMD Setting] ERROR: set_temp must be 16-30, got: %d\n", setTemp);
      }
    } else {
      logPrintln("[CMD Setting] ERROR: set_temp must be a number");
    }
  }
  
  if (doc.containsKey("set_mode")) {
    JsonVariant setModeValue = doc["set_mode"];
    if (setModeValue.is<const char*>()) {
      String setMode = setModeValue.as<String>();
      setMode.toUpperCase();
      
      if (setMode == "COOL" || setMode == "FAN" || setMode == "HEAT" || 
          setMode == "DRY" || setMode == "AUTO") {
        cfg.setting.set_mode = setMode;
        logPrintf("[CMD Setting] Set mode changed to: %s\n", cfg.setting.set_mode.c_str());
        changed = true;
      } else {
        logPrintf("[CMD Setting] ERROR: set_mode must be COOL/FAN/HEAT/DRY/AUTO, got: %s\n", 
                      setMode.c_str());
      }
    } else {
      logPrintln("[CMD Setting] ERROR: set_mode must be a string");
    }
  }

  // === [BARU] Setting Temp Control Cooldown (Min 60 detik) ===
  if (doc.containsKey("temp_cooldown")) {
    int val = doc["temp_cooldown"].as<int>();
    if (val >= 60) {
      if (cfg.setting.temp_control_cooldown_sec != val) {
        cfg.setting.temp_control_cooldown_sec = val;
        logPrintf("[CMD Setting] Temp Cooldown changed to: %d sec\n", val);
        changed = true;
      }
    } else {
      logPrintln("[CMD Setting] ERROR: temp_cooldown min 60 seconds");
    }
  }

  if (doc.containsKey("auto_recovery")) {
    int val = doc["auto_recovery"].as<int>();
    // Kita batasi minimal 60 detik (1 menit) agar tidak bentrok terus
    if (val >= 60) {
      if (cfg.setting.auto_recovery_sec != val) {
        cfg.setting.auto_recovery_sec = val;
        logPrintf("[CMD Setting] Auto Recovery changed to: %d sec\n", val);
        changed = true;
      }
    } else {
      logPrintln("[CMD Setting] ERROR: auto_recovery min 60 seconds");
    }
  }

  if (doc.containsKey("default_fan")) {
    JsonVariant defaultFanValue = doc["default_fan"];
    if (defaultFanValue.is<const char*>()) {
      String defaultFan = defaultFanValue.as<String>();
      defaultFan.toUpperCase();
      defaultFan.trim();
      
      // Validasi nilai yang diperbolehkan
      if (defaultFan == "AUTO" || 
          defaultFan == "LOW" || defaultFan == "MIN" ||
          defaultFan == "MED" || defaultFan == "MEDIUM" || 
          defaultFan == "HIGH" || defaultFan == "MAX" || 
          defaultFan == "TURBO") {
        
        // Normalisasi ke standard value
        if (defaultFan == "MIN") defaultFan = "LOW";
        if (defaultFan == "MEDIUM") defaultFan = "MED";
        if (defaultFan == "MAX") defaultFan = "HIGH";
        
        cfg.setting.default_fan = defaultFan;
        logPrintf("[CMD Setting] Default fan changed to: %s\n", cfg.setting.default_fan.c_str());
        changed = true;
      } else {
        logPrintf("[CMD Setting] ERROR: default_fan must be AUTO/LOW/MED/HIGH/TURBO, got: %s\n", 
                      defaultFan.c_str());
        dataPublishAck("setting", "failed", "Invalid default_fan value: " + defaultFan);
        return;
      }
    } else {
      logPrintln("[CMD Setting] ERROR: default_fan must be a string");
      dataPublishAck("setting", "failed", "default_fan must be string");
      return;
    }
  }
  
  if (changed) {
    yield();
    
    logPrintln("[CMD Setting] Saving config...");
    
    yield();
    
    if (!cfgSave()) {
      logPrintln("[CMD Setting] ERROR: Failed to save config");
      dataPublishAck("setting", "failed", "Storage write error");
      return; 
    }
    
    logPrintln("[CMD Setting] Config saved successfully");
    yield();
    
    cfgApply();
    yield();
    
    if (mqtt.connected()) {
      logPrintln("[CMD Setting] Publishing updated setting to MQTT...");
      dataPublishSetting();
    } else {
      logPrintln("[CMD Setting] WARNING: MQTT not connected, cannot publish");
    }
    logPrintln("[CMD Setting] ========== SUCCESS! ==========");
  } else {
    logPrintln("[CMD Setting] No valid changes detected");
    dataPublishAck("setting", "success", "No changes detected");
    logPrintln("[CMD Setting] ========== END (No Change) ==========");
  }
  if (changed) {
  logPrintln("[CMD Setting] Setting berubah, force automation re-evaluate");
  forceScheduleCheck = true;
  dataPublishAck("setting", "success", "Configuration updated");
  }
}

static void handleCmdUpdate(const char* payload, unsigned int length) {
  logPrintln("[CMD Update] ========== START ==========");

  // 1. Cek Sinyal & Parsing
  if (WiFi.RSSI() < -85) {
    dataPublishAck("update", "failed", "Signal weak (< -85dBm)");
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);

  if (err) {
    dataPublishAck("update", "failed", "Invalid JSON");
    return;
  }

  // Bisa menerima "manifest_url" atau "url" (untuk backward compatibility)
  String url = "";
  if (doc.containsKey("manifest_url")) {
      url = doc["manifest_url"].as<String>();
  } else if (doc.containsKey("url")) {
      url = doc["url"].as<String>();
  }

  if (url.length() > 0) {
      // 2. Kirim ACK bahwa perintah diterima
      dataPublishAck("update", "processing", "Request received, starting engine...");
      
      // Update status dashboard
      publishStatus("updating");

      // 3. Panggil proses Manifest (Logika baru)
      // Fungsi ini sudah mengandung banyak dataPublishAck() didalamnya
      otaProcessManifest(url);

      // Jika otaProcessManifest selesai tanpa restart (berarti gagal update/versi sama),
      // kita kembalikan status ke online
      publishStatus("online");

  } else {
      dataPublishAck("update", "failed", "URL missing in payload");
  }

  logPrintln("[CMD Update] ========== END ==========");
}

// Handler: Update Koneksi (WiFi & MQTT)
static void handleCmdKoneksi(const char* payload, unsigned int length) {
  logPrintln("[CMD Koneksi] ========== START ==========");
  
  if (length > 1024) {
    logPrintln("[CMD Koneksi] Payload too large");
    return;
  }
  
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    logPrintln("[CMD Koneksi] JSON parse error: ");
    dataPublishAck("koneksi", "failed", "Invalid JSON format");
    logPrintln(err.c_str());
    return;
  }

  logPrintln("[CMD Koneksi] JSON parsed OK");

  // Immutable keys - reject any attempt to modify
  if (doc.containsKey("device_id") || doc.containsKey("ac_brand")) {
    dataPublishAck("koneksi", "failed", "Immutable field (device_id/ac_brand) cannot be changed");
    logPrintln("[CMD Koneksi] ERROR: Attempt to modify immutable field");
    return;
  }
  
  bool wifiChanged = false;
  bool mqttChanged = false;
  
  esp_task_wdt_reset();
  yield();
  
  // Check WiFi changes
  if (doc.containsKey("wifi_ssid")) {
    String newSsid = doc["wifi_ssid"].as<String>();
    logPrintf("[CMD Koneksi] wifi_ssid: '%s' -> '%s'\n", 
                  cfg.koneksi.wifi_ssid.c_str(), newSsid.c_str());
    if (newSsid.length() > 0 && newSsid != cfg.koneksi.wifi_ssid) {
      cfg.koneksi.wifi_ssid = newSsid;
      wifiChanged = true;
    }
  }
  
  if (doc.containsKey("wifi_pass")) {
    String newPass = doc["wifi_pass"].as<String>();
    logPrintln("[CMD Koneksi] wifi_pass: updated");
    if (newPass.length() > 0 && newPass != cfg.koneksi.wifi_pass) {
      cfg.koneksi.wifi_pass = newPass;
      wifiChanged = true;
    }
  }
  
  // Check MQTT changes
  if (doc.containsKey("mqtt_host")) {
    String newHost = doc["mqtt_host"].as<String>();
    logPrintf("[CMD Koneksi] mqtt_host: '%s' -> '%s'\n", 
                  cfg.koneksi.mqtt_host.c_str(), newHost.c_str());
    if (newHost.length() > 0 && newHost != cfg.koneksi.mqtt_host) {
      cfg.koneksi.mqtt_host = newHost;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_port")) {
    uint16_t newPort = doc["mqtt_port"].as<uint16_t>();
    logPrintf("[CMD Koneksi] mqtt_port: %d -> %d\n", 
                  cfg.koneksi.mqtt_port, newPort);
    if (newPort > 0 && newPort != cfg.koneksi.mqtt_port) {
      cfg.koneksi.mqtt_port = newPort;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_user")) {
    String newUser = doc["mqtt_user"].as<String>();
    logPrintf("[CMD Koneksi] mqtt_user: '%s' -> '%s'\n", 
                  cfg.koneksi.mqtt_user.c_str(), newUser.c_str());
    if (newUser.length() > 0 && newUser != cfg.koneksi.mqtt_user) {
      cfg.koneksi.mqtt_user = newUser;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_pass")) {
    String newPass = doc["mqtt_pass"].as<String>();
    logPrintln("[CMD Koneksi] mqtt_pass: updated");
    if (newPass.length() > 0 && newPass != cfg.koneksi.mqtt_pass) {
      cfg.koneksi.mqtt_pass = newPass;
      mqttChanged = true;
    }
  }

  // Check if any changes detected
  if (!wifiChanged && !mqttChanged) {
    logPrintln("[CMD Koneksi] No changes detected");
    dataPublishAck("koneksi", "success", "No changes detected");
    logPrintln("[CMD Koneksi] ========== END (No Change) ==========");
    return;
  }
  
  yield();
  esp_task_wdt_reset();
  
  // Save configuration
  logPrintln("[CMD Koneksi] Changes detected, saving config...");
  if (wifiChanged) {
    logPrintf("[CMD Koneksi] WiFi will change to SSID: %s\n", cfg.koneksi.wifi_ssid.c_str());
  }
  if (mqttChanged) {
    logPrintf("[CMD Koneksi] MQTT will change to: %s:%d\n", 
                  cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port);
  }
  
  yield();
  esp_task_wdt_reset();
  
  if (!cfgSave()) {
    logPrintln("[CMD Koneksi] ERROR: Failed to save config!");
    dataPublishAck("koneksi", "failed", "Storage write error");
    logPrintln("[CMD Koneksi] ========== END (Save Failed) ==========");
    return;
  }
  
  logPrintln("[CMD Koneksi] Config saved successfully to LittleFS");
  yield();
  esp_task_wdt_reset();
  
  // === PERBAIKAN UTAMA: TRIGGER IMMEDIATE RECONNECT ===
  
  // 1. Set reason untuk monitoring
  if (wifiChanged) {
    lastDisconnectReason = REASON_MANUAL;
    lastDisconnectTime = millis();
  }

  // 2. Disconnect MQTT dulu (PENTING: Harus graceful!)
  if (mqtt.connected()) {
    logPrintln("[CMD Koneksi] Disconnecting MQTT gracefully...");
    isManualDisconnect = true; // Flag agar counter tidak bertambah
    publishStatus("offline");
    delay(200); // Beri waktu pesan terkirim
    mqtt.disconnect();
    espClient.stop();
  }
  
  delay(300);
  yield();
  esp_task_wdt_reset();
  
  // 3. Apply config ke variabel internal
  cfgApply();
  
  // 4. JIKA WIFI BERUBAH: Force disconnect & reconnect
  if (wifiChanged) {
    logPrintln("[CMD Koneksi] WiFi changed, forcing reconnect...");
    
    // Kirim ACK sebelum disconnect (ini pesan terakhir!)
    dataPublishAck("koneksi", "success", "WiFi changed, reconnecting now...");
    delay(500); // Beri waktu pesan terkirim
    
    // Disconnect WiFi total (flag wifiOK di main loop akan trigger reconnect)
    WiFi.disconnect(true); // true = wipe credentials
    WiFi.mode(WIFI_OFF);
    delay(500);
    
    // === KUNCI: RESET TIMER AGAR RECONNECT LANGSUNG! ===
    dataScheduleImmediateWiFiRetry();
    
    logPrintln("[CMD Koneksi] WiFi disconnected, ready for immediate retry");
  } 
  // 5. JIKA HANYA MQTT YANG BERUBAH: Reconnect MQTT saja
  else if (mqttChanged) {
    logPrintln("[CMD Koneksi] Only MQTT changed, reconnecting broker...");
    
    dataPublishAck("koneksi", "success", "MQTT settings updated, reconnecting...");
    delay(200);
    
    // Reset MQTT connection
    dataResetConnection();
    lastMqttRetryMs = 0; // Trigger immediate MQTT reconnect
    
    logPrintln("[CMD Koneksi] MQTT ready for reconnect");
  }
  
  logPrintln("[CMD Koneksi] ========== SUCCESS! ==========");
}

// Handler: Update Schedule (jadwal harian)
static void handleCmdSchedule(const char* payload, unsigned int length) {
  logPrintln("[CMD Schedule] ========== START ==========");
  
  if (length > 2048) {
    logPrintln("[CMD Schedule] Payload too large");
    return;
  }
  
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("[CMD Schedule] JSON parse error: ");
    dataPublishAck("schedule", "failed", "Invalid JSON format");
    logPrintln(err.c_str());
    return;
  }

  logPrintln("[CMD Schedule] JSON parsed OK");
  bool changed = false;
  
  if (!doc.containsKey("daily")) {
    logPrintln("[CMD Schedule] ERROR: Missing 'daily' array");
    return;
  }
  
  JsonArray daily = doc["daily"].as<JsonArray>();
  if (daily.isNull()) {
    logPrintln("[CMD Schedule] ERROR: 'daily' is not a valid array");
    return;
  }
  
  // Valid day names (case insensitive)
  const char* validDays[] = {"senin", "selasa", "rabu", "kamis", "jumat", "sabtu", "minggu"};
  const char* validTypes[] = {"part_time", "full_time", "libur"};
  
  for (JsonObject dayObj : daily) {
    // Validasi field 'day'
    if (!dayObj.containsKey("day")) {
      logPrintln("[CMD Schedule] SKIP: Missing 'day' field");
      continue;
    }
    
    const char* dayName = dayObj["day"] | "";
    if (!dayName || strlen(dayName) == 0) {
      logPrintln("[CMD Schedule] SKIP: Empty day name");
      continue;
    }
    
    // Find day index - case insensitive comparison
    int idx = -1;
    for (int i = 0; i < 7; ++i) {
      if (strcasecmp(dayName, validDays[i]) == 0) {
        idx = i;
        break;
      }
    }
    
    if (idx < 0) {
      logPrintf("[CMD Schedule] SKIP: Invalid day name '%s' (valid: senin-minggu)\n", dayName);
      continue;
    }
    
    // Validasi field 'type'
    if (!dayObj.containsKey("type")) {
      logPrintf("[CMD Schedule] SKIP: Missing 'type' field for day '%s'\n", dayName);
      continue;
    }
    
    const char* type = dayObj["type"] | "";
    if (!type || strlen(type) == 0) {
      logPrintf("[CMD Schedule] SKIP: Empty type for day '%s'\n", dayName);
      continue;
    }
    
    // Validasi apakah type valid
    bool validType = false;
    for (int i = 0; i < 3; i++) {
      if (strcasecmp(type, validTypes[i]) == 0) {
        validType = true;
        break;
      }
    }
    
    if (!validType) {
      logPrintf("[CMD Schedule] SKIP: Invalid type '%s' for day '%s' (valid: part_time,full_time,libur)\n", 
                    type, dayName);
      continue;
    }
    
    // Process berdasarkan type
    if (strcasecmp(type, "part_time") == 0) {
      cfg.setting.daily[idx].type = DAY_PART_TIME;
      
      // Handle waktu untuk part_time
      if (dayObj.containsKey("waktu")) {
        JsonObject waktu = dayObj["waktu"].as<JsonObject>();
        if (!waktu.isNull()) {
          // Ambil waktu mulai
          const char* mulai = waktu["mulai"] | "";
          if (mulai && strlen(mulai) > 0) {
            cfg.setting.daily[idx].waktu.mulai = String(mulai);
            logPrintf("[CMD Schedule] Updated %s mulai: %s\n", dayName, mulai);
          } else {
            logPrintf("[CMD Schedule] %s mulai: keeping old value (%s)\n", 
                          dayName, cfg.setting.daily[idx].waktu.mulai.c_str());
          }
          
          // Ambil waktu selesai
          const char* selesai = waktu["selesai"] | "";
          if (selesai && strlen(selesai) > 0) {
            cfg.setting.daily[idx].waktu.selesai = String(selesai);
            logPrintf("[CMD Schedule] Updated %s selesai: %s\n", dayName, selesai);
          } else {
            logPrintf("[CMD Schedule] %s selesai: keeping old value (%s)\n", 
                          dayName, cfg.setting.daily[idx].waktu.selesai.c_str());
          }
        } else {
          logPrintf("[CMD Schedule] %s: invalid waktu object, keeping old values (mulai:%s, selesai:%s)\n", 
                        dayName, cfg.setting.daily[idx].waktu.mulai.c_str(), 
                        cfg.setting.daily[idx].waktu.selesai.c_str());
        }
      } else {
        logPrintf("[CMD Schedule] %s: no waktu specified, keeping old values (mulai:%s, selesai:%s)\n", 
                      dayName, cfg.setting.daily[idx].waktu.mulai.c_str(), 
                      cfg.setting.daily[idx].waktu.selesai.c_str());
      }
      
      logPrintf("[CMD Schedule] ✓ Updated %s: part_time (%s-%s)\n", 
                    dayName, cfg.setting.daily[idx].waktu.mulai.c_str(), 
                    cfg.setting.daily[idx].waktu.selesai.c_str());
    } 
    else if (strcasecmp(type, "libur") == 0) {
      cfg.setting.daily[idx].type = DAY_LIBUR;
      cfg.setting.daily[idx].waktu.mulai = "";
      cfg.setting.daily[idx].waktu.selesai = "";
      logPrintf("[CMD Schedule] ✓ Updated %s: libur\n", dayName);
    } 
    else { // full_time
      cfg.setting.daily[idx].type = DAY_FULL;
      cfg.setting.daily[idx].waktu.mulai = "";
      cfg.setting.daily[idx].waktu.selesai = "";
      logPrintf("[CMD Schedule] ✓ Updated %s: full_time\n", dayName);
    }
    
    changed = true;
    yield(); // Prevent watchdog timeout during loop
  }

  if (changed) {
    yield();
    esp_task_wdt_reset();
    
    logPrintln("[CMD Schedule] Saving schedule...");
    yield();
    
    if (!cfgSave()) {
      logPrintln("[CMD Schedule] ERROR: Failed to save schedule");
      dataPublishAck("schedule", "failed", "Storage write error");
      logPrintln("[CMD Schedule] ========== FAILED! ==========");
      return;
    }
    
    logPrintln("[CMD Schedule] Schedule saved successfully");
    yield();
    esp_task_wdt_reset();
    
    // Apply config changes
    cfgApply();
    yield();
    
    if (mqtt.connected()) {
      logPrintln("[CMD Schedule] Publishing updated setting to MQTT...");
      dataPublishSetting();
    } else {
      logPrintln("[CMD Schedule] WARNING: MQTT not connected, cannot publish");
    }
    dataPublishAck("schedule", "success", "Schedule updated");
    logPrintln("[CMD Schedule] ========== SUCCESS! ==========");
  } else {
    logPrintln("[CMD Schedule] No valid changes made");
    logPrintln("[CMD Schedule] ========== END (No Change) ==========");
    dataPublishAck("schedule", "success", "No changes made");
  }
}

static void handleCmdRemote(const char* payload, unsigned int length) {
  logPrintln("[CMD Remote] ========== START ==========");
  
  if (length > 512) {
    logPrintln("[CMD Remote] Payload too large");
    dataPublishAck("remote", "failed", "Payload too large");
    return;
  }

  esp_task_wdt_reset();
  yield();
  
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    logPrintln("[CMD Remote] JSON parse error: ");
    dataPublishAck("remote", "failed", "Invalid JSON format");
    logPrintln(err.c_str());
    return;
  }
  
  logPrintln("[CMD Remote] JSON parsed OK");
  
  // Validasi: Tolak jika ada nested "remote" object
  if (doc.containsKey("remote")) {
    logPrintln("[CMD Remote] ERROR: Invalid format - nested 'remote' object detected");
    logPrintln("[CMD Remote] Expected format: {\"power\":true,\"temp\":23,\"mode\":\"COOL\"}");
    logPrintln("[CMD Remote] Received format: {\"remote\":{...}} <- INVALID!");
    dataPublishAck("remote", "failed", "Invalid format: remove nested 'remote' wrapper");
    logPrintln("[CMD Remote] ========== REJECTED ==========");
    return;
  }
  
  // Validasi: Minimal harus ada salah satu key yang valid
  if (!doc.containsKey("power") && !doc.containsKey("temp") && 
      !doc.containsKey("mode") && !doc.containsKey("fan") &&
      !doc.containsKey("swing") && !doc.containsKey("sleep") && 
      !doc.containsKey("turbo") && !doc.containsKey("lights") &&
      !doc.containsKey("health")) {
    logPrintln("[CMD Remote] ERROR: No valid command keys found");
    logPrintln("[CMD Remote] Valid keys: power, temp, mode, fan, swing, sleep, turbo, lights, health");
    dataPublishAck("remote", "failed", "No valid command keys");
    logPrintln("[CMD Remote] ========== REJECTED ==========");
    return;
  }
  
  esp_task_wdt_reset();
  yield();
  
  logPrintln("[CMD Remote] Sending IR command...");
  
  // ═══════════════════════════════════════════════════════════════════
  // [UPDATED] MQTT REMOTE
  // MQTT remote TIDAK trigger manual recovery.
  // Manual recovery hanya di-trigger oleh data dari AC (physical remote)
  // yang terdeteksi di monitorExternalChange().
  // Jika AC tidak merespons IR -> state tidak berubah -> tidak ada trigger.
  // ═══════════════════════════════════════════════════════════════════
  
  // Baseline state untuk default expected (hindari default "power=true" ketika user hanya kirim temp/mode/fan)
  BPFrame baselineFrame = sensorGetLastFrame();
  if (!baselineFrame.valid) {
    baselineFrame = activityGetLastKnownState();
  }

  // Extract expected state dari command untuk state matching
  bool expectedPower = doc.containsKey("power") ? doc["power"].as<bool>() : (baselineFrame.valid ? baselineFrame.power : true);
  String expectedMode = doc.containsKey("mode") ? doc["mode"].as<String>() : (baselineFrame.valid ? baselineFrame.mode : cfg.setting.set_mode);
  String expectedFan = doc.containsKey("fan") ? doc["fan"].as<String>() : (baselineFrame.valid ? baselineFrame.fan : cfg.setting.default_fan);
  uint8_t expectedTemp = doc.containsKey("temp") ? doc["temp"].as<uint8_t>() : cfg.setting.set_temp;
  
  // Set expected state di automation module
  automationSetExpectedStateFromMqtt(expectedPower, expectedMode, expectedFan, expectedTemp);
  
  esp_task_wdt_reset();
  yield();
  
  String cmdStr;
  cmdStr.reserve(512);
  serializeJson(doc, cmdStr);
  
  logPrintf("[CMD Remote] Command: %s\n", cmdStr.c_str());
  logPrintf("[CMD Remote] Expected: power=%s mode=%s fan=%s set_temp=%d\n",
            expectedPower ? "ON" : "OFF", expectedMode.c_str(), 
            expectedFan.c_str(), expectedTemp);
  
  esp_task_wdt_reset();
  yield();

  lastActionSource = ACTION_MQTT_REMOTE;
  lastActionMs = millis();
  
  // Get old state before executing command (prefer sensor frame; fallback to activity)
  BPFrame oldFrame = sensorGetLastFrame();
  if (!sensorIsHealthy()) {
    oldFrame = activityGetLastKnownState();
  }

  int oldSetTemp = -1;
  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    oldSetTemp = (int)disp.set_temp;
  }

  if (!doc.containsKey("temp") && oldSetTemp > 0) {
    expectedTemp = (uint8_t)oldSetTemp;
  }
  
  // Eksekusi perintah IR + retry jika display tidak merespon
  // Requirement: bila display/state belum berubah dalam 2 detik, kirim ulang (maks 3x).
  bool anyActivityTracked = false;
  static const uint8_t MAX_REMOTE_SEND_ATTEMPTS = 5;
  static const unsigned long REMOTE_DISPLAY_WAIT_MS = 1000UL;

  // EXPERIMENT: set true to make 1 MQTT command trigger exactly 1 IR send.
  // Keep the retry logic intact so it can be re-enabled later.
  static const bool REMOTE_SINGLE_SEND_EXPERIMENT = true;

  const bool wantsPower = doc.containsKey("power");
  const bool wantsMode  = doc.containsKey("mode");
  const bool wantsFan   = doc.containsKey("fan");
  const bool wantsTemp  = doc.containsKey("temp");
  const bool hasVerifiableKeys = wantsPower || wantsMode || wantsFan || wantsTemp;
  const uint8_t maxSendAttempts = REMOTE_SINGLE_SEND_EXPERIMENT ? 1 : MAX_REMOTE_SEND_ATTEMPTS;

  if (REMOTE_SINGLE_SEND_EXPERIMENT) {
    logPrintln("[CMD Remote] Single-send experiment: retry disabled (1 MQTT = 1 IR send)");
  }

  auto normalizeModeLabel = [](String v) -> String {
    v.toUpperCase();
    v.trim();
    if (v == "COLD") v = "COOL";
    return v;
  };

  auto normalizeFanBucket = [](String v) -> String {
    v.toUpperCase();
    v.trim();
    if (v == "MEDIUM") v = "MED";
    if (v == "MIN") v = "LOW";
    if (v == "MAX" || v == "TURBO") v = "HIGH";
    return v;
  };

  for (uint8_t sendAttempt = 0; sendAttempt < maxSendAttempts && !anyActivityTracked; ++sendAttempt) {
    logPrintf("[CMD Remote] IR send attempt %u/%u\n", (unsigned)(sendAttempt + 1), (unsigned)maxSendAttempts);
    controllHandleCommand(cmdStr.c_str(), cmdStr.length());

    unsigned long t0 = millis();
    bool shouldAccept = false;
    bool shouldTrackTemp = false;
    bool shouldTrackMainboard = false;
    int newSetTemp = -1;
    bool hasFreshSetTemp = false;
    BPFrame newFrame;
    bool hasFreshNewFrame = false;

    while ((millis() - t0) < REMOTE_DISPLAY_WAIT_MS) {
      sensorPoll();
      delay(50);
      esp_task_wdt_reset();
      yield();

      if (sensorIsDisplayHealthy()) {
        DisplayFrame disp = sensorGetLastDisplayFrame();
        newSetTemp = (int)disp.set_temp;
        hasFreshSetTemp = (newSetTemp > 0);
      }

      newFrame = sensorGetLastFrame();
      hasFreshNewFrame = (sensorIsHealthy() && newFrame.valid);

      if (hasVerifiableKeys) {
        bool match = true;
        if (wantsPower) {
          if (!hasFreshNewFrame) match = false;
          else match &= (newFrame.power == expectedPower);
        }
        if (wantsMode && expectedPower) {
          if (!hasFreshNewFrame) match = false;
          else match &= (normalizeModeLabel(newFrame.mode) == normalizeModeLabel(expectedMode));
        }
        if (wantsFan && expectedPower) {
          if (!hasFreshNewFrame) match = false;
          else match &= (normalizeFanBucket(newFrame.fan) == normalizeFanBucket(expectedFan));
        }
        if (wantsTemp && expectedPower) {
          if (!hasFreshSetTemp || newSetTemp <= 0) match = false;
          else match &= ((uint8_t)newSetTemp == expectedTemp);
        }
        shouldAccept = match;
        if (shouldAccept) {
          shouldTrackTemp = (oldSetTemp > 0 && newSetTemp > 0 && newSetTemp != oldSetTemp);
          shouldTrackMainboard = hasFreshNewFrame;
        }
      } else {
        bool changed = false;
        if (oldSetTemp > 0 && newSetTemp > 0 && newSetTemp != oldSetTemp) {
          changed = true;
          shouldTrackTemp = true;
        }
        if (hasFreshNewFrame) {
          bool stateChanged = (!oldFrame.valid) ||
                             (newFrame.power != oldFrame.power) ||
                             (newFrame.mode != oldFrame.mode) ||
                             (newFrame.fan != oldFrame.fan);
          if (stateChanged) {
            changed = true;
            shouldTrackMainboard = true;
          }
        }
        shouldAccept = changed;
      }

      if (shouldAccept) {
        break; // Early exit: AC has responded!
      }
    }

    if (shouldAccept) {
      if (shouldTrackTemp && oldSetTemp > 0 && newSetTemp > 0) {
        activityTrackTemp(oldSetTemp, newSetTemp, SOURCE_MQTT_REMOTE);
        oldSetTemp = newSetTemp;
      }

      if (shouldTrackMainboard && hasFreshNewFrame) {
        if (oldFrame.valid) {
          bool stateChanged = (newFrame.power != oldFrame.power) ||
                             (newFrame.mode != oldFrame.mode) ||
                             (newFrame.fan != oldFrame.fan);
          if (stateChanged) {
            activityTrackChange(oldFrame, newFrame, SOURCE_MQTT_REMOTE);
          }
        } else {
          activityTrackPower(oldFrame.power, newFrame.power, SOURCE_MQTT_REMOTE);
          if (newFrame.power) {
            activityTrackMode(oldFrame.mode, newFrame.mode, SOURCE_MQTT_REMOTE);
            activityTrackFan(oldFrame.fan, newFrame.fan, SOURCE_MQTT_REMOTE);
          }
        }
        oldFrame = newFrame;
      }

      anyActivityTracked = true;
    }

    if (!anyActivityTracked && (sendAttempt + 1) < maxSendAttempts) {
      logPrintf("[CMD Remote] No matching display response within %lu ms, retrying...\n", (unsigned long)REMOTE_DISPLAY_WAIT_MS);
    }
  }
  
  if (!anyActivityTracked) {
    logPrintln("[CMD Remote] AC did not respond - activity not tracked");
    dataPublishAck("remote", "no_response", "No fresh state change detected after command");
    logPrintln("[CMD Remote] ========== NO RESPONSE ==========");
    return;
  }
  
  esp_task_wdt_reset();
  yield();
  
  // ═══════════════════════════════════════════════════════════════════
  // MQTT remote tidak trigger manual recovery.
  // Manual recovery hanya dari data AC (monitorExternalChange).
  // ═══════════════════════════════════════════════════════════════════
  
  dataPublishAck("remote", "success", "Command executed");
  logPrintln("[CMD Remote] ========== SUCCESS! ==========");
}

// ==========================================
// FUNGSI PUBLISH LOGS KE MQTT
// ==========================================
static void publishLogs(const String& message) {
  if (!mqtt.connected() || topicLogs.length() == 0) {
    return;
  }
  
  // Publish log message ke topic logs
  mqtt.publish(topicLogs.c_str(), message.c_str(), false);
}

// ==========================================
// HANDLER COMMAND LOGS
// ==========================================
static void handleCmdLogs(const char* payload, unsigned int length) {
  logPrintln("╔════════════════════════════════════════╗");
  logPrintln("║  📝 LOGS CONTROL COMMAND RECEIVED     ║");
  logPrintln("╚════════════════════════════════════════╝");
  
  if (length > 256) {
    logPrintln("[CMD Logs] Payload too large");
    dataPublishAck("logs", "failed", "Payload too large");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  
  if (err) {
    logPrintln("[CMD Logs] JSON parse error");
    dataPublishAck("logs", "failed", "Invalid JSON format");
    return;
  }
  
  // Cek apakah ada field "enabled"
  if (!doc.containsKey("enabled")) {
    logPrintln("[CMD Logs] Missing 'enabled' field");
    dataPublishAck("logs", "failed", "Missing 'enabled' field. Use: {\"enabled\": true} or {\"enabled\": false}");
    return;
  }
  
  bool enable = doc["enabled"].as<bool>();
  
  // Set logging status
  logSetMqttEnabled(enable);
  
  // Save to config for persistence
  cfg.setting.mqtt_log_enabled = enable;
  cfgSave();
  
  String status = enable ? "enabled" : "disabled";
  logPrintf("[CMD Logs] MQTT logging %s\n", status.c_str());
  
  // Send acknowledgment
  String ackMsg = "MQTT logging " + status;
  dataPublishAck("logs", "success", ackMsg);
  
  // Jika enabled, kirim log konfirmasi ke topic logs
  if (enable) {
    String confirmMsg = "[System] MQTT logging enabled\n";
    publishLogs(confirmMsg);
  }
  
  // Publish updated status immediately so dashboard sees the change
  publishStatus("online");
  dataPublishInfo();
  
  logPrintln("[CMD Logs] ========== END ==========");
}

// ============= MQTT Message Router =============
static void processQueuedMqttCommand() {
  if (isProcessingCommand || pendingCommandCount == 0) {
    return;
  }

  PendingMqttCommand command = pendingCommands[pendingCommandHead];
  pendingCommands[pendingCommandHead].used = false;
  pendingCommandHead = (pendingCommandHead + 1) % 4;
  pendingCommandCount--;

  isProcessingCommand = true;
  routeMqttCommand(String(command.topic), command.payload, command.length);
  isProcessingCommand = false;
}

static void routeMqttCommand(const String& topicStr, const char* payload, unsigned int length) {
  logPrintf("[DEBUG] Recv Topic: '%s' (Len: %d)\n", topicStr.c_str(), topicStr.length());
  logPrintf("[MQTT] Payload: %s\n", payload);

  if (topicStr.equals(cmdRemote)) {
    logPrintln("[MQTT] → handleCmdRemote");
    handleCmdRemote(payload, length);
  }
  else if (topicStr.equals(cmdSetting)) {
    logPrintln("[MQTT] → handleCmdSetting");
    handleCmdSetting(payload, length);
  }
  else if (topicStr.equals(cmdKoneksi)) {
    logPrintln("[MQTT] → handleCmdKoneksi");
    handleCmdKoneksi(payload, length);
  }
  else if (topicStr.equals(cmdSchedule)) {
    logPrintln("[MQTT] → handleCmdSchedule");
    handleCmdSchedule(payload, length);
  }
  else if (topicStr.equals(topicCmdUpdate)) {
    logPrintln("[MQTT] → handleCmdUpdate MATCHED!");
    handleCmdUpdate(payload, length);
  }
  else if (topicStr.equals(cmdDiagnostics)) {
    logPrintln("╔════════════════════════════════════════╗");
    logPrintln("║  🔍 DIAGNOSTICS REQUEST RECEIVED      ║");
    logPrintln("╚════════════════════════════════════════╝");
    dataPublishAck("diagnostics", "processing", "Generating diagnostics...");
    delay(500);
    esp_task_wdt_reset();
    dataPublishDiagnostics();
    dataPublishAck("diagnostics", "success", "Diagnostics published to /diagnostics topic");
    logPrintln("[MQTT] Diagnostics request completed");
  }
  else if (topicStr.equals(cmdLogs)) {
    logPrintln("[MQTT] → handleCmdLogs");
    handleCmdLogs(payload, length);
  }
  else if (topicStr.equals(cmdInfo)) {
    logPrintln("╔════════════════════════════════════════╗");
    logPrintln("║  📊 INFO REQUEST RECEIVED             ║");
    logPrintln("╚════════════════════════════════════════╝");
    dataPublishAck("info", "processing", "Generating system info...");
    delay(300);
    esp_task_wdt_reset();
    dataPublishInfo();
    dataPublishAck("info", "success", "System info published to /info topic");
    logPrintln("[MQTT] Info request completed");
  }
  else if (topicStr.equals(cmdReboot)) {
    logPrintln("╔════════════════════════════════════════╗");
    logPrintln("║  🔄 REMOTE REBOOT COMMANDED           ║");
    logPrintln("╚════════════════════════════════════════╝");
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    int delaySeconds = 3;
    if (!err && doc.containsKey("delay_seconds")) {
      delaySeconds = doc["delay_seconds"].as<int>();
      if (delaySeconds < 1) delaySeconds = 1;
      if (delaySeconds > 60) delaySeconds = 60;
    }
    logPrintf("[Reboot] Rebooting in %d seconds...\n", delaySeconds);
    String ackMsg = "Rebooting in " + String(delaySeconds) + " seconds...";
    dataPublishAck("reboot", "success", ackMsg);
    publishStatus("rebooting");
    cfgSaveDisconnectTimestamp();
    if (LittleFS.begin()) {
      File f = LittleFS.open("/shutdown.flag", "w");
      if (f) {
        f.println("graceful_reboot");
        f.close();
        logPrintln("[Reboot] Graceful shutdown flag created");
      }
      LittleFS.remove("/running.flag");
    }
    unsigned long startMs = millis();
    while (millis() - startMs < (delaySeconds * 1000)) {
      delay(500);
      esp_task_wdt_reset();
      mqtt.loop();
    }
    logPrintln("[Reboot] Restarting now...");
    delay(500);
    ESP.restart();
  }
  else {
    logPrintf("[MQTT] Unknown/Ignored topic: %s\n", topicStr.c_str());
    if (topicStr.indexOf("update") >= 0) {
      logPrintln("[DEBUG] WARNING: Topik mengandung kata 'update' tapi tidak cocok persis.");
      logPrintf("[DEBUG] Harapan: '%s'\n", topicCmdUpdate.c_str());
      logPrintf("[DEBUG] Diterima: '%s'\n", topicStr.c_str());
    }
  }
}

static void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length > 2048) {
    logPrintln("[MQTT] Payload too large, rejected");
    return;
  }
  if (pendingCommandCount >= 4) {
    logPrintln("[MQTT] Command queue full, dropping incoming command");
    return;
  }

  PendingMqttCommand& slot = pendingCommands[pendingCommandTail];
  slot.used = true;
  strncpy(slot.topic, topic, sizeof(slot.topic) - 1);
  slot.topic[sizeof(slot.topic) - 1] = '\0';
  unsigned int copyLen = length;
  if (copyLen >= sizeof(slot.payload)) {
    copyLen = sizeof(slot.payload) - 1;
  }
  memcpy(slot.payload, payload, copyLen);
  slot.payload[copyLen] = '\0';
  slot.length = copyLen;
  pendingCommandTail = (pendingCommandTail + 1) % 4;
  pendingCommandCount++;
  logPrintf("[MQTT] Queued command topic='%s' len=%u depth=%u\n", slot.topic, slot.length, pendingCommandCount);
}

void dataCheckPendingSave() {
  if (!pendingSaveRequest) return;
  
  // Tunggu minimal 2 detik setelah request untuk memastikan tidak ada operasi baca/tulis lain
  if (millis() - saveRequestTime < 2000) return;
  
  logPrintln("[Data] Processing pending config save...");
  
  // Kita tidak perlu disconnect MQTT jika RAM ESP32 Anda cukup (>4MB flash/PSRAM).
  // Namun jika sering crash karena OOM (Out of Memory), biarkan disconnect ini.
  bool wasMqttConnected = mqtt.connected();
  if (wasMqttConnected) {
    isManualDisconnect = true;
    lastDisconnectReason = REASON_MANUAL;
    // Best-effort: publish retained offline before graceful disconnect.
    publishStatus("offline");
    delay(200);
    mqtt.disconnect();
    delay(100);
    espClient.stop();  // Tutup SSL socket agar tidak leak FD
  }

  // --- BAGIAN INI DIHAPUS/DISEDERHANAKAN ---
  // Jangan LittleFS.end() disini. 
  // Biarkan cfgSave() yang menangani pengecekan FS.
  
  // Save
  if (cfgSave()) {
    logPrintln("[Data] ✓ Pending config saved successfully");
    
    // Apply config baru (terutama jika ada perubahan logic automation)
    cfgApply();
    
    // Reconnect MQTT Logic
    if (wasMqttConnected) {
        // Kita biarkan loop utama yang melakukan reconnect 
        // agar logic-nya terpusat di dataLoop/ensureMqttConnected
        logPrintln("[Data] Config saved, MQTT will reconnect in main loop");
    }
  } else {
    logPrintln("[Data] ✗ Failed to save pending config");
  }
  
  pendingSaveRequest = false;
}

bool dataPublishRaw(const String& topic, const String& payload) {
  if (!mqtt.connected()) return false;
  return mqtt.publish(topic.c_str(), payload.c_str(), false);
}

bool dataPublishInfo() {
  if (!mqtt.connected()) {
    logPrintln("[Info] Cannot publish - MQTT not connected");
    return false;
  }

  StaticJsonDocument<128> doc;
  doc["fw_version"] = FW_VERSION;
  doc["ac_brand"] = cfg.ac_brand;
  doc["ip"] = WiFi.localIP().toString();

  // PUBLISH
  String payload;
  serializeJson(doc, payload);

  // Gunakan retain = true agar saat dashboard dibuka, status langsung muncul
  if (mqtt.publish(topicInfo.c_str(), payload.c_str(), true)) {
    logPrintf("[Info] ✓ Summary published (fw_version=%s)\n", FW_VERSION);
    return true;
  } else {
    logPrintln("[Info] ✗ Failed to publish summary");
    return false;
  }
}

bool dataPublishAck(const char* command, const char* status, const String& detail) {
    if (!mqtt.connected()) {
      logPrintf("[ACK] Skip publish, MQTT disconnected: %s -> %s (%s)\n", command, status, detail.c_str());
      return false;
    }
    if (topicAck.length() == 0) {
      logPrintf("[ACK] Skip publish, topicAck empty: %s -> %s (%s)\n", command, status, detail.c_str());
      return false;
    }

    StaticJsonDocument<256> doc;
    doc["cmd"] = command;
    doc["status"] = status;
    doc["detail"] = detail;
    doc["ts"] = millis() / 1000;

    String payload;
    serializeJson(doc, payload);

    mqtt.loop();
    yield();
    esp_task_wdt_reset();

    bool published = mqtt.publish(topicAck.c_str(), payload.c_str(), false);
    if (!published) {
      mqtt.loop();
      delay(75);
      esp_task_wdt_reset();
      published = mqtt.publish(topicAck.c_str(), payload.c_str(), false);
    }
    if (!published) {
      mqtt.loop();
      delay(150);
      esp_task_wdt_reset();
      published = mqtt.publish(topicAck.c_str(), payload.c_str(), false);
    }

    if (published) {
      consecutivePublishFails = 0;
      mqtt.loop();
      delay(40);
      esp_task_wdt_reset();
      logPrintf("[ACK] %s -> %s (%s)\n", command, status, detail.c_str());
      return true;
    }

    consecutivePublishFails++;
    logPrintf("[ACK] Publish failed: %s -> %s (%s), state=%d, fails=%d\n",
              command, status, detail.c_str(), mqtt.state(), consecutivePublishFails);
    return false;
}

void dataPublishDiagnostics() {
  if (!mqtt.connected()) {
    logPrintln("[Diagnostics] Cannot publish - MQTT not connected");
    return;
  }
  
  logPrintln("╔════════════════════════════════════════╗");
  logPrintln("║   🔍 GENERATING FULL DIAGNOSTICS       ║");
  logPrintln("╚════════════════════════════════════════╝");

  StaticJsonDocument<4096> doc; // Buffer besar

  // ═══════════════════════════════════════════════
  // 1. FIRMWARE & BUILD
  // ═══════════════════════════════════════════════
  JsonObject fw = doc.createNestedObject("firmware");
  fw["version"] = FW_VERSION;
  fw["built_at"] = String(__DATE__) + " " + String(__TIME__);
  fw["reset_reason"] = getResetReason(); // Pindahan dari Info

  // ═══════════════════════════════════════════════
  // 2. NETWORK DETAILED
  // ═══════════════════════════════════════════════
  JsonObject net = doc.createNestedObject("network");
  net["ip"] = WiFi.localIP().toString();
  net["mac"] = WiFi.macAddress(); // Pindahan dari Info
  net["ssid"] = WiFi.SSID();      // Pindahan dari Info
  net["rssi"] = WiFi.RSSI();
  net["gateway"] = WiFi.gatewayIP().toString();
  net["dns"] = WiFi.dnsIP(0).toString();

  doc["wifi_scan_count"] = nearbyWifiCount;
  doc["wifi_scan_in_progress"] = nearbyWifiScanInProgress;
  doc["wifi_scan_age_ms"] = lastNearbyWifiScanCompleteMs == 0 ? 0UL : (millis() - lastNearbyWifiScanCompleteMs);
  JsonArray wifiNearby = doc.createNestedArray("wifi_nearby");
  appendNearbyWifiNetworks(wifiNearby);

  // ═══════════════════════════════════════════════
  // 3. MEMORY HEALTH (Complete)
  // ═══════════════════════════════════════════════
  JsonObject mem = doc.createNestedObject("memory");
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxAlloc = ESP.getMaxAllocHeap();
  
  mem["heap_free"] = freeHeap;
  mem["heap_min"] = ESP.getMinFreeHeap();      // Pindahan dari Info
  mem["heap_max_alloc"] = maxAlloc;            // Pindahan dari Info
  
  // Hitung Fragmentasi
  float fragmentation = 0;
  if (freeHeap > 0) {
    fragmentation = 100.0 - ((float)maxAlloc * 100.0 / (float)freeHeap);
  }
  mem["fragmentation_pct"] = round(fragmentation * 10) / 10.0;
  
  if (psramFound()) {
    mem["psram_free"] = ESP.getFreePsram();
  }

  // ═══════════════════════════════════════════════
  // 4. STATISTIK KONEKSI & RELIABILITY
  // ═══════════════════════════════════════════════
  JsonObject conn = doc.createNestedObject("stats");
  conn["uptime_sec"] = millis() / 1000;
  conn["mqtt_reconnects"] = mqttReconnectCount;  // Pindahan dari Info
  conn["wifi_disconnects"] = wifiDisconnectCount; // Pindahan dari Info
  
  // Crash Stats Summary
  CrashStats cStats = getCrashStats();
  JsonObject crash = doc.createNestedObject("crash_stats"); // Pindahan dari Info
  crash["total_boots"] = cStats.totalBoots;
  crash["total_crashes"] = cStats.totalCrashes;
  crash["crash_rate"] = cStats.crashRate;
  crash["brownouts"] = cStats.brownoutResets;

  // ═══════════════════════════════════════════════
  // 5. SENSOR HEALTH DETAILED
  // ═══════════════════════════════════════════════
  JsonObject sens = doc.createNestedObject("sensor");
  sens["healthy"] = sensorIsHealthy();
  sens["uart_errors"] = uartErrorCount;           // Pindahan dari Info
  
  // Fix: Handle case when no valid frame exists (avoid 4294967295)
  if (sensorHasValidFrame()) {
    unsigned long frameAge = sensorTimeSinceLastFrame();
    // Additional check: if frameAge is abnormally large (> 1 hour), treat as invalid
    if (frameAge < 3600000UL) { // 3600000ms = 1 hour
      sens["last_frame_age"] = frameAge / 1000;
      sens["last_rx_age_sec"] = frameAge / 1000;
    } else {
      sens["last_frame_age"] = -1; // Invalid/too old
      sens["last_rx_age_sec"] = -1;
    }
  } else {
    sens["last_frame_age"] = -1; // No valid frame
    sens["last_rx_age_sec"] = -1;
  }
  
  sens["last_rx_ms"] = sensorGetLastValidFrameMs();  // Timestamp last valid frame (milliseconds since boot)
  sens["comm_error_inferred"] = sensorHasInferredCommError();

  // ═══════════════════════════════════════════════
  // 6. AUTOMATION STATS
  // ═══════════════════════════════════════════════
  JsonObject autoStats = doc.createNestedObject("automation"); // Pindahan dari Info
  autoStats["active"] = cfg.setting.automation;
  autoStats["manual_mode"] = cfg.setting.manualModeActive;
  autoStats["auto_on"] = autoOnCount;
  autoStats["auto_off"] = autoOffCount;
  autoStats["manual_override"] = manualOverrideCount;
  autoStats["ir_sent"] = irSendCount;

  // ═══════════════════════════════════════════════
  // 7. IR LED HEALTH (NEW)
  // ═══════════════════════════════════════════════
  IRHealthStats irStats = irHealthGetStats();
  JsonObject irHealth = doc.createNestedObject("ir_led_health");
  irHealth["enabled"] = irHealthIsEnabled();
  irHealth["total_commands"] = irStats.totalCommands;
  irHealth["success_count"] = irStats.successCount;
  irHealth["no_response_count"] = irStats.noResponseCount;
  irHealth["mismatch_count"] = irStats.mismatchCount;
  irHealth["success_rate"] = round(irStats.getSuccessRate() * 10) / 10.0;
  irHealth["health_status"] = irStats.getHealthStatus();
  
  // Last command details
  if (irStats.lastCommandMs > 0) {
    JsonObject lastCmd = irHealth.createNestedObject("last_command");
    lastCmd["success"] = irStats.lastCommandSuccess;
    lastCmd["type"] = irStats.lastCommandType;
    lastCmd["expected"] = irStats.lastExpected;
    lastCmd["actual"] = irStats.lastActual;
    lastCmd["age_sec"] = (millis() - irStats.lastCommandMs) / 1000;
  }

  // ═══════════════════════════════════════════════
  // 8. FILESYSTEM
  // ═══════════════════════════════════════════════
  JsonObject fs = doc.createNestedObject("filesystem");
  fs["used_pct"] = (LittleFS.usedBytes() * 100.0) / LittleFS.totalBytes();

  // ═══════════════════════════════════════════════
  // 9. HEALTH SCORE CALCULATION (Logic Pindahan dari Info)
  // ═══════════════════════════════════════════════
  int healthScore = 100;
  JsonArray alerts = doc.createNestedArray("health_alerts"); // Pindahan dari Info

  // --- Penilaian Memory ---
  if (freeHeap < 15000) {
    healthScore -= 40;
    alerts.add("CRITICAL: Low memory");
  } else if (freeHeap < 25000) {
    healthScore -= 20;
    alerts.add("WARNING: Memory pressure");
  }
  
  if (fragmentation > 40) {
    healthScore -= 15;
    alerts.add("WARNING: High fragmentation");
  }

  // --- Penilaian WiFi ---
  int rssi = WiFi.RSSI();
  if (rssi < -85) {
    healthScore -= 20;
    alerts.add("CRITICAL: Weak Signal");
  } else if (rssi < -75) {
    healthScore -= 10;
  }

  // --- Penilaian Sensor ---
  if (!sensorIsHealthy()) {
    healthScore -= 25;
    alerts.add("CRITICAL: Sensor Disconnected");
  } else if (uartErrorCount > 50) {
    healthScore -= 10;
    alerts.add("WARNING: High UART Errors");
  }
  
  // --- Penilaian IR LED Health ---
  float irSuccessRate = irStats.getSuccessRate();
  if (irStats.totalCommands > 10) { // Only evaluate after sufficient commands
    if (irSuccessRate < 70.0) {
      healthScore -= 30;
      alerts.add("CRITICAL: IR LED Failing");
    } else if (irSuccessRate < 85.0) {
      healthScore -= 15;
      alerts.add("WARNING: IR LED Unreliable");
    } else if (irSuccessRate < 95.0) {
      healthScore -= 5;
      alerts.add("INFO: IR LED Occasional Issues");
    }
    
    if (irStats.noResponseCount > 5) {
      alerts.add("WARNING: IR LED No Response Detected");
    }
    if (irStats.mismatchCount > 3) {
      alerts.add("WARNING: IR LED Mismatch Detected");
    }
  }

  // --- Penilaian Stabilitas ---
  if (cStats.crashRate > 10.0) {
    healthScore -= 20;
    alerts.add("CRITICAL: High Crash Rate");
  }
  if (cStats.brownoutResets > 2) {
    healthScore -= 15;
    alerts.add("WARNING: Unstable Power (Brownout)");
  }

  // Clamp Score
  if (healthScore < 0) healthScore = 0;
  
  // Tentukan Status Text
  String healthStatus = "HEALTHY";
  if (healthScore < 50) healthStatus = "CRITICAL";
  else if (healthScore < 75) healthStatus = "WARNING";

  // Masukkan ke JSON
  doc["health_score"] = healthScore;
  doc["health_status"] = healthStatus;

  doc["timestamp"] = millis() / 1000;

  // ═══════════════════════════════════════════════
  // 9. PUBLISH
  // ═══════════════════════════════════════════════
  String payload;
  payload.reserve(4096);
  size_t len = serializeJson(doc, payload);
  
  // Sesuaikan topic ID
  String topic = "isems/devices/" + cfg.device_id + "/diagnostics"; // Pastikan cfg.device.deviceId sesuai struct Anda
  
  if (mqtt.publish(topic.c_str(), payload.c_str(), true)) {
    logPrintln("╔════════════════════════════════════════╗");
    logPrintln("║  ✅ DIAGNOSTICS PUBLISHED              ║");
    logPrintln("╚════════════════════════════════════════╝");
    logPrintf("Size: %d bytes | Score: %d (%s)\n", len, healthScore, healthStatus.c_str());
  } else {
    logPrintln("[Diagnostics] ✗ Failed to publish");
  }
}