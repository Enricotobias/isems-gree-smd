#include "data.h"
#include "config.h"
#include "controll.h"

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

static String cmdSetting;       // isems/command/{id}/setting
static String cmdKoneksi;       // isems/command/{id}/koneksi
static String cmdSchedule;      // isems/command/{id}/schedule
static String cmdRemote;        // isems/command/{id}/remote
static String cmdReboot;        // isems/command/{id}/reboot
static String cmdInfo;          // isems/command/{id}/info

String topicCmdUpdate = "";
String topicInfo = "";

String topicAck = ""; 
DisconnectReason lastDisconnectReason = REASON_POWER_ON; // Default saat boot
unsigned long lastDisconnectTime = 0;
    
unsigned long mqttReconnectCount = 0;
unsigned long wifiDisconnectCount = 0;
bool isManualDisconnect = false; // <--- TAMBAHKAN INI (Flag penanda)

static bool pendingSaveRequest = false;
static unsigned long saveRequestTime = 0;
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
static void serviceNearbyWifiScan();

// ============= Build Topics =============
void dataBuildTopics(const String& device_id) {
  // Publish topics
  topicStatus  = "isems/devices/" + device_id + "/status";
  topicData    = "isems/devices/" + device_id + "/data";
  topicSetting = "isems/devices/" + device_id + "/setting";
  topicInfo = "isems/devices/" + device_id + "/info";
  topicAck = "isems/devices/" + device_id + "/ack";
  activityBuildTopic(device_id);
  
  // Command topics
  cmdSetting  = "isems/command/" + device_id + "/setting";
  cmdKoneksi  = "isems/command/" + device_id + "/koneksi";
  cmdSchedule = "isems/command/" + device_id + "/schedule";
  cmdRemote   = "isems/command/" + device_id + "/remote";
  topicCmdUpdate = "isems/command/" + device_id + "/update";
  cmdReboot = "isems/command/" + device_id + "/reboot";
  cmdInfo = "isems/command/" + device_id + "/info";

  Serial.Println(" - Update: " + topicCmdUpdate);
  Serial.Println(" - Info: " + cmdInfo);
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
    Serial.Println("[Data] MQTT not connected, cannot publish setting");
    return;
  }

  if (!dataIsPublishReady()) {
    unsigned long now = millis();
    if (now - lastDeferredPublishLogMs >= 1000UL) {
      lastDeferredPublishLogMs = now;
      Serial.Printf("[Data] Deferring setting publish until MQTT cooldown passes (%lu ms remaining)\n",
                MQTT_POST_CONNECT_PUBLISH_COOLDOWN_MS - (now - lastMqttConnectMs));
    }
    return;
  }
  
  // Cek apakah topic sudah di-inisialisasi
  if (topicSetting.length() == 0) {
    Serial.Println("[Data] ERROR: topicSetting not initialized");
    return;
  }
  StaticJsonDocument<4096> doc;
  doc["automation"] = cfg.setting.automation;

  
  // ============================================================
  JsonArray daily = doc.createNestedArray("schedule");
  for (int i = 0; i < 7; ++i) {
    JsonObject dayObj = daily.createNestedObject();
    dayObj["day"] = DAY_IDX_TO_NUM(i);
    JsonArray data = dayObj.createNestedArray("data");
    DaySchedule& d = cfg.setting.daily[i];
    if (d.type == DAY_OFFTIME) {
      dayObj["type"] = "offtime";
    } else if (d.type == DAY_FULLTIME) {
      dayObj["type"] = "fulltime";
      JsonObject p = data.createNestedObject();
      p["mode"] = d.periods[0].mode;
      p["fan"] = d.periods[0].fan;
      p["set_temp"] = d.periods[0].set_temp;
      p["min_temp"] = d.periods[0].min_temp;
      p["max_temp"] = d.periods[0].max_temp;
    } else {
      dayObj["type"] = "parttime";
      uint8_t cnt = d.periodCount;
      if (cnt > MAX_PERIODS_PER_DAY) cnt = MAX_PERIODS_PER_DAY;
      for (uint8_t pIdx = 0; pIdx < cnt; pIdx++) {
        JsonObject p = data.createNestedObject();
        p["start"] = d.periods[pIdx].start;
        p["end"] = d.periods[pIdx].end;
        p["mode"] = d.periods[pIdx].mode;
        p["fan"] = d.periods[pIdx].fan;
        p["set_temp"] = d.periods[pIdx].set_temp;
        p["min_temp"] = d.periods[pIdx].min_temp;
        p["max_temp"] = d.periods[pIdx].max_temp;
      }
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
    Serial.Printf("[Data] ✓ Published setting successfully to: %s\n", topicSetting.c_str());
  } else {
    consecutivePublishFails++;
    Serial.Printf("[Data] ✗ Failed to publish setting to: %s (%d bytes, state=%d, fails=%d)\n", 
              topicSetting.c_str(), payload.length(), mqtt.state(), consecutivePublishFails);
  }
}

void syncTime() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.Println("[Time] Skip NTP sync - WiFi not connected");
    return;
  }

  Serial.Println("[Time] Requesting confirmed NTP sync...");
  cfgSyncTime(5000);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 250)) {
    Serial.Printf("[Time] Current after sync attempt: %04d-%02d-%02d %02d:%02d:%02d\n",
              timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
              timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.Println("[Time] Failed to read current time after sync attempt");
  }

  esp_task_wdt_reset();
  yield();
}

static void buildLwtPayload() {
  StaticJsonDocument<256> doc;
  doc["status"] = "offline";
  lwtPayload = "";
  serializeJson(doc, lwtPayload);
}

// Fungsi helper untuk publish retained status payload
static bool publishStatusRetainedPayload(const String& payload) {
  if (!mqtt.connected()) {
    Serial.Println("[Data] Cannot publish status - MQTT not connected");
    return false;
  }
  if (topicStatus.length() == 0) {
    Serial.Println("[Data] Cannot publish status - topicStatus is empty");
    return false;
  }

  bool published = mqtt.publish(topicStatus.c_str(), payload.c_str(), true);
  if (!published) {
    consecutivePublishFails++;
    Serial.Printf("[Data] Failed to publish retained status to: %s (fails=%d)\n", topicStatus.c_str(), consecutivePublishFails);
    return false;
  }

  consecutivePublishFails = 0;
  mqtt.loop();
  delay(50);
  return true;
}

static bool publishDataRetainedPayload(const String& payload) {
  if (!mqtt.connected()) {
    Serial.Println("[Data] Cannot publish data - MQTT not connected");
    return false;
  }
  if (topicData.length() == 0) {
    Serial.Println("[Data] Cannot publish data - topicData is empty");
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
    Serial.Printf("[Data] Failed to publish retained data to: %s (len=%u, fails=%d)\n",
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
      StaticJsonDocument<256> doc;

      doc["status"] = "online";
      doc["disconnect_reason"] = getDisconnectReasonStr(lastDisconnectReason);

      String payload;
      serializeJson(doc, payload);
      publishStatusRetainedPayload(payload);

      Serial.Printf("[Data] Status published: %s (Reason: %s)\n",
                status.c_str(), getDisconnectReasonStr(lastDisconnectReason).c_str());
  } 
  else if (status == "offline") {
      StaticJsonDocument<256> doc;
      doc["status"] = "offline";
      doc["reason"] = getDisconnectReasonStr(lastDisconnectReason);
      
      String payload;
      serializeJson(doc, payload);
      publishStatusRetainedPayload(payload);
      
      Serial.Println("[Data] Offline status published");
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
    Serial.Printf("[MQTT] RC -2 recovery reset: %s\n", reason);
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
  Serial.Printf("[MQTT] RC -2 tracker: fails=%u, stuck=%lu sec, wifi_recovery=%s\n",
            mqttRc2ConsecutiveFails,
            (now - mqttRc2FirstFailMs) / 1000UL,
            mqttRc2WifiRecoveryAttempted ? "done" : "pending");
}

static void performMqttRc2WifiRecovery() {
  lastMqttRc2RecoveryMs = millis();
  mqttRc2WifiRecoveryAttempted = true;
  lastDisconnectReason = REASON_MQTT_LOST;

  Serial.Println("[MQTT] RC -2 persisted - resetting WiFi radio + TLS stack...");
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
  Serial.Println("[MQTT] RC -2 WiFi/TLS recovery triggered");
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
    Serial.Printf("[MQTT] RC -2 persisted for %lu sec after WiFi/TLS recovery - restarting ESP32...\n",
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
    Serial.Println("[MQTT] WiFi not connected, skip MQTT connection");
    return;
  }
  WiFi.setSleep(false);

  if (cfg.device_id.length() == 0) {
    Serial.Println("[MQTT] ERROR: device_id is empty - cannot build topics/connect");
    return;
  }
  
  if (mqtt.connected()) {
    mqttRetryAttempt = 0; // Reset counter saat berhasil
    return;
  }
  
  // Pastikan topics diinisialisasi
  if (topicStatus.length() == 0 || topicData.length() == 0) {
    Serial.Println("[MQTT] Topics not initialized, initializing now...");
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
      Serial.Printf("[MQTT] Waiting %lu seconds before retry (attempt %d/%d)\n", 
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
  
  Serial.Println("[MQTT] mencoba konek ke server...");
  Serial.Printf("[MQTT] Server: %s:%d (SSL)\n", cfg.koneksi.mqtt_host.c_str(), 
            cfg.koneksi.mqtt_port);
  Serial.Printf("[MQTT] Client ID: %s\n", cfg.device_id.c_str());
  Serial.Printf("[MQTT] Retry attempt: %d/%d\n", mqttRetryAttempt, MAX_RETRY_ATTEMPTS);
  
  esp_task_wdt_reset();
  yield();
  
  // Single connection attempt
  bool ok;
  const char* clientId = cfg.device_id.c_str();
  unsigned long connectStart = millis();
  
  buildLwtPayload();
  if (mqttUser.length()) {
    ok = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                      topicStatus.c_str(), 1, true, lwtPayload.c_str());
  } else {
    ok = mqtt.connect(clientId, topicStatus.c_str(), 1, true, lwtPayload.c_str());
  }
  
  unsigned long connectDuration = millis() - connectStart;
  Serial.Printf("[MQTT] Connection attempt took %lu ms\n", connectDuration);

  // Deteksi SSL heap exhaustion: jika connect() return < 5ms,
  // berarti tidak ada koneksi yang benar-benar dicoba (mbedTLS gagal alokasi heap).
  if (!ok && connectDuration < 5) {
    Serial.Printf("[MQTT] ✗ SSL instant-fail (0ms) - Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.Println("[MQTT] Kemungkinan heap exhausted. Auto-restart dalam 3 detik...");
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
      Serial.Println("[MQTT] Reconnected after config save (Counter ignored)");
    } else {
      mqttReconnectCount++;
      Serial.Printf("[MQTT] Reconnect Count: %lu\n", mqttReconnectCount);
    }
    
    Serial.Println("[MQTT] ✓ Connected successfully!");
    lastMqttConnectMs = millis(); // Catat waktu connect untuk cooldown guard
    offlineNotifyDone = false; // Reset agar bisa notify kembali jika nanti disconnect lagi

    // Koreksi retained offline status sebelum go online.
    // LWT broker menyimpan static "CONNECTION_LOST" - timpa dengan reason sesungguhnya.
    {
      StaticJsonDocument<256> offDoc;
      offDoc["status"] = "offline";
      offDoc["reason"] = getDisconnectReasonStr(lastDisconnectReason);
      String offPayload;
      serializeJson(offDoc, offPayload);
      publishStatusRetainedPayload(offPayload);
      mqtt.loop();
    }

    publishStatus("online");
    
    esp_task_wdt_reset();
    yield();
    
    // Subscribe to all command topics (log failures so it isn't silent)
    if (!mqtt.subscribe(cmdSetting.c_str()))     Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdSetting.c_str());
    if (!mqtt.subscribe(cmdKoneksi.c_str()))     Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdKoneksi.c_str());
    if (!mqtt.subscribe(cmdSchedule.c_str()))    Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdSchedule.c_str());
    if (!mqtt.subscribe(cmdRemote.c_str()))      Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdRemote.c_str());
    if (topicCmdUpdate.length() > 0) {
      if (!mqtt.subscribe(topicCmdUpdate.c_str())) Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", topicCmdUpdate.c_str());
    } else {
      Serial.Println("[MQTT] Skip subscribe: topicCmdUpdate empty");
    }

    if (!mqtt.subscribe(cmdInfo.c_str()))        Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdInfo.c_str());
    if (!mqtt.subscribe(cmdReboot.c_str()))      Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdReboot.c_str());
    Serial.Println("[MQTT] Subscribed to command topics");
    
    esp_task_wdt_reset();
    yield();
    
    // Publish setting ditunda sampai post-connect cooldown terlewati.
  } else {
    int rc = mqtt.state();
    Serial.Printf("[MQTT] ✗ Gagal konek, rc=%d (", rc);
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
    Serial.Println(")");

    if (rc == -2) {
      noteMqttRc2Failure();
      maybeEscalateMqttRc2Failure();
    } else {
      resetMqttRc2RecoveryState("non RC-2 failure");
    }
    
    // Log next retry time
    unsigned long nextRetryMs = getRetryInterval();
    Serial.Printf("[MQTT] Next retry in %lu seconds (attempt %d/%d)\n", 
              nextRetryMs / 1000, mqttRetryAttempt + 1, MAX_RETRY_ATTEMPTS);

    // Setelah 3 retry gagal & WiFi masih terhubung, publish retained offline sekali
    // agar dashboard langsung tahu device offline tanpa menunggu LWT broker.
    if (mqttRetryAttempt >= 3 && !offlineNotifyDone && WiFi.status() == WL_CONNECTED) {
      Serial.Println("[MQTT] 3 retry gagal - mencoba publish status offline...");
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
        Serial.Println("[MQTT] ✓ Status offline berhasil dipublish setelah 3 retry gagal");
      } else {
        Serial.Println("[MQTT] ✗ Offline notify gagal - broker tidak terjangkau, LWT akan aktif");
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
      Serial.Println("[Data] Setting Secure Root CA Certificate...");
      espClient.setCACert(PROD_ROOT_CA); // Menggunakan sertifikat dari config.h
  } else {
      Serial.Println("[Data] WARNING: Root CA is NULL! Fallback to Insecure.");
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
      Serial.Println("[Data] Disconnect reason: WATCHDOG RESET");
  } else if (r == ESP_RST_BROWNOUT) {
      lastDisconnectReason = REASON_POWER_LOSS;
      Serial.Println("[Data] Disconnect reason: POWER LOSS (brownout detected)");
  } else if (r == ESP_RST_PANIC) {
      lastDisconnectReason = REASON_CRASH;
      Serial.Println("[Data] Disconnect reason: SOFTWARE CRASH (panic)");
  } else if (r == ESP_RST_POWERON) {
      lastDisconnectReason = REASON_POWER_ON;
      Serial.Println("[Data] Disconnect reason: POWER ON / POWER CYCLE");
  } else if (r == ESP_RST_SW) {
      lastDisconnectReason = REASON_POWER_ON;
      Serial.Println("[Data] Disconnect reason: SOFTWARE RESET (OTA/command)");
  } else {
      lastDisconnectReason = REASON_POWER_ON;
      Serial.Printf("[Data] Disconnect reason: UNKNOWN (esp_reset_reason=%d)\n", (int)r);
  }
  
  // Reset retry timer agar koneksi bisa langsung dicoba
  lastMqttRetryMs = 0;
  
  // 5. Build Topics & Server Config
  if (cfg.device_id.length() > 0) {
    dataBuildTopics(cfg.device_id);
    dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port, 
                  cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
                  
    Serial.Printf("[Data] MQTT topics built for device: %s\n", cfg.device_id.c_str());
  }
  
  // 6. Setup logger MQTT publisher
  
  Serial.Println("[Data] Logger MQTT publisher configured");
  
  Serial.Println("[Data] MQTT Config: Buffer=4096, KeepAlive=30s, Timeout=15s");
  Serial.Printf("[Data] MQTT Server: %s:%d (SECURE)\n", cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port);

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
    if (mqtt.connected()) {
      // Do NOT send MQTT DISCONNECT here.
      // Forcing socket close is more reliable for LWT/offline propagation.
      espClient.stop();
    } else if (wasMqttConnected) {
      // mqtt.connected() sudah false (TCP RST), tapi kita belum sempat save
    }
    wasMqttConnected = false;
    return;
  }

  serviceNearbyWifiScan();
  
  if (!mqtt.connected()) {
    if (wasMqttConnected) {
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
    Serial.Println("[MQTT] \u2717 Zombie connection detected (3+ publish fails) \u2014 forcing clean reconnect");
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

  // Heartbeat
  unsigned long now = millis();

  if (now - lastHeartbeatMs >= 120000UL) { // 2 menit
    lastHeartbeatMs = now;
    publishStatus("online"); // refresh retained online payload

    // Refresh LWT payload setiap 4 heartbeat (2 menit).
    // Cukup update retained status dan LWT payload di memory.
    static uint8_t lwtRefreshCounter = 0;
    lwtRefreshCounter++;
    if (lwtRefreshCounter >= 6) {
      lwtRefreshCounter = 0;
      buildLwtPayload(); // Update LWT payload di memory untuk reconnect berikutnya
      publishStatus("online"); // Refresh retained status di broker
      Serial.Println("[MQTT] LWT payload updated + status refreshed");
    }
  }
}

void dataDisconnect() {
  if (mqtt.connected()) {
    // For graceful disconnect, send offline status manually
    Serial.Println("[MQTT] Sending offline status before graceful disconnect...");
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
        Serial.Println("[MQTT] Graceful shutdown flag created");
      }
      LittleFS.remove("/running.flag");
    }
    
    Serial.Println("[MQTT] Disconnected gracefully");
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
  Serial.Println("[MQTT] Mempublish status offline (retry gagal 3x)...");
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
    Serial.Println("[MQTT] \u2713 Status offline berhasil dipublish");
  } else {
    Serial.Println("[MQTT] \u2717 Offline notify gagal - broker tidak terjangkau");
  }
}

void dataResetConnection() {
  // Reset SSL client dan disconnect MQTT untuk memaksa fresh connection
  Serial.Println("[MQTT] Resetting connection (WiFi reconnected)...");
  
  if (mqtt.connected()) {
    // Jangan publish "offline" di sini — LWT (broker-side) sudah menangani retained offline status.
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
  
  Serial.Println("[MQTT] Connection reset complete, ready for fresh connection");
}

void dataForceConnect() {
  // Cek WiFi terlebih dahulu
  if (WiFi.status() != WL_CONNECTED) {
    Serial.Println("[MQTT] WiFi not connected, cannot force connect");
    return;
  }
  
  if (mqtt.connected()) {
    Serial.Println("[MQTT] Already connected");
    return;
  }
  
  // Pastikan topics sudah diinisialisasi
  if (topicStatus.length() == 0) {
    Serial.Println("[MQTT] Initializing topics before connection...");
    dataBuildTopics(cfg.device_id);
    dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port, 
                  cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
  }
  
  // Reset SSL client untuk memastikan koneksi bersih
  espClient.stop();
  delay(100);
  yield();
  
  // Bypass cooldown untuk force connection
  Serial.Println("[MQTT] Force connecting to server...");
  Serial.Printf("[MQTT] Server: %s:%d (SSL)\n", cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port);
  Serial.Printf("[MQTT] Client ID: %s\n", cfg.device_id.c_str());
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
  Serial.Printf("[MQTT] Connection attempt took %lu ms\n", connectDuration);

  // Deteksi SSL heap exhaustion: jika connect() return < 5ms,
  // berarti tidak ada koneksi yang benar-benar dicoba (mbedTLS gagal alokasi heap).
  if (!ok && connectDuration < 5) {
    Serial.Printf("[MQTT] ✗ SSL instant-fail (0ms) - Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.Println("[MQTT] Kemungkinan heap exhausted. Auto-restart dalam 3 detik...");
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
      Serial.Println("[MQTT] Reconnected after config save (Counter ignored)");
    } else {
      // Murni putus jaringan -> Tambah counter
      mqttReconnectCount++;
      Serial.Printf("[MQTT] Reconnect Count: %lu\n", mqttReconnectCount);
    }
    // ============================================

    Serial.Println("[MQTT] Force connect successful!");
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

    mqtt.subscribe(cmdInfo.c_str());
    
    // === [MODIFIKASI 3] Wajib Subscribe Topik OTA ===
    if (topicCmdUpdate.length() > 0) {
       mqtt.subscribe(topicCmdUpdate.c_str());
    }
    // ================================================
    
    Serial.Println("[MQTT] Subscribed to command topics");
    
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
    Serial.Printf("[MQTT] ✗ Force connect failed, rc=%d (", rc);
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
    Serial.Println(")");
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
    Serial.Println("[Data] Skip publish (MQTT not connected / topicData empty)");
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
    Serial.Println("[Data] Published suhu + status sistem ke MQTT");
  } else {
    Serial.Printf("[Data] ✗ Publish failed (state=%d)\n", mqtt.state());
  }
}

// ============================================================
// BONUS: Update handleCmdSetting() untuk clear manual mode
// jika user manually enable automation via MQTT
// ============================================================
static void handleCmdSetting(const char* payload, unsigned int length) {
  Serial.Println("[CMD Setting] ========== START ==========");
  if (length > 2048) {
    Serial.Println("[CMD Setting] Payload too large");
    return;
  }
  
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("[CMD Setting] JSON error: ");
    dataPublishAck("setting", "failed", "Invalid JSON format");
    Serial.Println(err.c_str());
    return;
  }

  bool changed = false;

  // Immutable keys - reject any attempt to modify
  if (doc.containsKey("device_id") || doc.containsKey("ac_brand")) {
    dataPublishAck("setting", "failed", "Immutable field (device_id/ac_brand) cannot be changed");
    Serial.Println("[CMD Setting] ERROR: Attempt to modify immutable field");
    return;
  }
  
  // Validasi automation - hanya boolean
  if (doc.containsKey("automation")) {
    JsonVariant autoValue = doc["automation"];

    if (!autoValue.is<bool>()) {
      Serial.Println("[CMD Setting] ERROR: automation must be boolean (true/false)");
      return;
    }

    bool newAutoValue = autoValue.as<bool>();

    if (newAutoValue != cfg.setting.automation) {

      cfg.setting.automation = newAutoValue;

      if (newAutoValue) {
        // === USER MENGHIDUPKAN AUTOMATION ===
        Serial.Println("[CMD Setting] Automation enabled by user");

        cfg.setting.automationPausedByUser = false;

        // Clear manual recovery jika ada
        if (cfg.setting.manualModeActive) {
          Serial.Println("[CMD Setting] Clearing manual mode (user override)");
          cfg.setting.manualModeActive = false;
          cfg.setting.lastManualControlMs = millis();
        }

      } else {
        // === USER MEMATIKAN AUTOMATION ===
        Serial.Println("[CMD Setting] Automation disabled by user");

        cfg.setting.automationPausedByUser = true;
        cfg.setting.manualModeActive = false;
      }

      Serial.Printf(
        "[CMD Setting] Automation changed to: %s\n",
        cfg.setting.automation ? "true" : "false"
      );

      changed = true;
    }
  }

  // global_set_temp, global_min_temp, global_max_temp, global_mode, global_fan,
  // temp_cooldown, auto_recovery — removed from MQTT command.
  // All AC parameters now come from schedule periods.

  if (changed) {
    yield();
    
    Serial.Println("[CMD Setting] Saving config...");
    
    yield();
    
    if (!cfgSave()) {
      Serial.Println("[CMD Setting] ERROR: Failed to save config");
      dataPublishAck("setting", "failed", "Storage write error");
      return; 
    }
    
    Serial.Println("[CMD Setting] Config saved successfully");
    yield();
    
    cfgApply();
    yield();
    
    if (mqtt.connected()) {
      Serial.Println("[CMD Setting] Publishing updated setting to MQTT...");
      dataPublishSetting();
    } else {
      Serial.Println("[CMD Setting] WARNING: MQTT not connected, cannot publish");
    }
    Serial.Println("[CMD Setting] ========== SUCCESS! ==========");
  } else {
    Serial.Println("[CMD Setting] No valid changes detected");
    dataPublishAck("setting", "success", "No changes detected");
    Serial.Println("[CMD Setting] ========== END (No Change) ==========");
  }
  if (changed) {
  Serial.Println("[CMD Setting] Setting berubah, force automation re-evaluate");
  forceScheduleCheck = true;
  dataPublishAck("setting", "success", "Configuration updated");
  }
}

static void handleCmdUpdate(const char* payload, unsigned int length) {
  Serial.Println("[CMD Update] ========== START ==========");

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

  Serial.Println("[CMD Update] ========== END ==========");
}

// Handler: Update Koneksi (WiFi & MQTT)
static void handleCmdKoneksi(const char* payload, unsigned int length) {
  Serial.Println("[CMD Koneksi] ========== START ==========");
  
  if (length > 1024) {
    Serial.Println("[CMD Koneksi] Payload too large");
    return;
  }
  
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.Println("[CMD Koneksi] JSON parse error: ");
    dataPublishAck("koneksi", "failed", "Invalid JSON format");
    Serial.Println(err.c_str());
    return;
  }

  Serial.Println("[CMD Koneksi] JSON parsed OK");

  // Immutable keys - reject any attempt to modify
  if (doc.containsKey("device_id") || doc.containsKey("ac_brand")) {
    dataPublishAck("koneksi", "failed", "Immutable field (device_id/ac_brand) cannot be changed");
    Serial.Println("[CMD Koneksi] ERROR: Attempt to modify immutable field");
    return;
  }
  
  bool wifiChanged = false;
  bool mqttChanged = false;
  
  esp_task_wdt_reset();
  yield();
  
  // Check WiFi changes
  if (doc.containsKey("wifi_ssid")) {
    String newSsid = doc["wifi_ssid"].as<String>();
    Serial.Printf("[CMD Koneksi] wifi_ssid: '%s' -> '%s'\n", 
                  cfg.koneksi.wifi_ssid.c_str(), newSsid.c_str());
    if (newSsid.length() > 0 && newSsid != cfg.koneksi.wifi_ssid) {
      nvs_setWiFiSSID(newSsid);
      cfg.koneksi.wifi_ssid = newSsid;
      wifiChanged = true;
    }
  }
  
  if (doc.containsKey("wifi_pass")) {
    String newPass = doc["wifi_pass"].as<String>();
    Serial.Println("[CMD Koneksi] wifi_pass: updated");
    if (newPass.length() > 0 && newPass != cfg.koneksi.wifi_pass) {
      nvs_setWiFiPass(newPass);
      cfg.koneksi.wifi_pass = newPass;
      wifiChanged = true;
    }
  }
  
  // Check MQTT changes
  if (doc.containsKey("mqtt_host")) {
    String newHost = doc["mqtt_host"].as<String>();
    Serial.Printf("[CMD Koneksi] mqtt_host: '%s' -> '%s'\n", 
                  cfg.koneksi.mqtt_host.c_str(), newHost.c_str());
    if (newHost.length() > 0 && newHost != cfg.koneksi.mqtt_host) {
      cfg.koneksi.mqtt_host = newHost;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_port")) {
    uint16_t newPort = doc["mqtt_port"].as<uint16_t>();
    Serial.Printf("[CMD Koneksi] mqtt_port: %d -> %d\n", 
                  cfg.koneksi.mqtt_port, newPort);
    if (newPort > 0 && newPort != cfg.koneksi.mqtt_port) {
      cfg.koneksi.mqtt_port = newPort;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_user")) {
    String newUser = doc["mqtt_user"].as<String>();
    Serial.Printf("[CMD Koneksi] mqtt_user: '%s' -> '%s'\n", 
                  cfg.koneksi.mqtt_user.c_str(), newUser.c_str());
    if (newUser.length() > 0 && newUser != cfg.koneksi.mqtt_user) {
      cfg.koneksi.mqtt_user = newUser;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_pass")) {
    String newPass = doc["mqtt_pass"].as<String>();
    Serial.Println("[CMD Koneksi] mqtt_pass: updated");
    if (newPass.length() > 0 && newPass != cfg.koneksi.mqtt_pass) {
      cfg.koneksi.mqtt_pass = newPass;
      mqttChanged = true;
    }
  }

  // Check if any changes detected
  if (!wifiChanged && !mqttChanged) {
    Serial.Println("[CMD Koneksi] No changes detected");
    dataPublishAck("koneksi", "success", "No changes detected");
    Serial.Println("[CMD Koneksi] ========== END (No Change) ==========");
    return;
  }
  
  yield();
  esp_task_wdt_reset();
  
  // Save configuration
  Serial.Println("[CMD Koneksi] Changes detected, saving config...");
  if (wifiChanged) {
    Serial.Printf("[CMD Koneksi] WiFi will change to SSID: %s\n", cfg.koneksi.wifi_ssid.c_str());
  }
  if (mqttChanged) {
    Serial.Printf("[CMD Koneksi] MQTT will change to: %s:%d\n", 
                  cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port);
  }
  
  yield();
  esp_task_wdt_reset();
  
  if (!cfgSave()) {
    Serial.Println("[CMD Koneksi] ERROR: Failed to save config!");
    dataPublishAck("koneksi", "failed", "Storage write error");
    Serial.Println("[CMD Koneksi] ========== END (Save Failed) ==========");
    return;
  }
  
  Serial.Println("[CMD Koneksi] Config saved successfully to LittleFS");
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
    Serial.Println("[CMD Koneksi] Disconnecting MQTT gracefully...");
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
    Serial.Println("[CMD Koneksi] WiFi changed, forcing reconnect...");
    
    // Kirim ACK sebelum disconnect (ini pesan terakhir!)
    dataPublishAck("koneksi", "success", "WiFi changed, reconnecting now...");
    delay(500); // Beri waktu pesan terkirim
    
    // Disconnect WiFi total (flag wifiOK di main loop akan trigger reconnect)
    WiFi.disconnect(true); // true = wipe credentials
    WiFi.mode(WIFI_OFF);
    delay(500);
    
    // === KUNCI: RESET TIMER AGAR RECONNECT LANGSUNG! ===
    dataScheduleImmediateWiFiRetry();
    
    Serial.Println("[CMD Koneksi] WiFi disconnected, ready for immediate retry");
  } 
  // 5. JIKA HANYA MQTT YANG BERUBAH: Reconnect MQTT saja
  else if (mqttChanged) {
    Serial.Println("[CMD Koneksi] Only MQTT changed, reconnecting broker...");
    
    dataPublishAck("koneksi", "success", "MQTT settings updated, reconnecting...");
    delay(200);
    
    // Reset MQTT connection
    dataResetConnection();
    lastMqttRetryMs = 0; // Trigger immediate MQTT reconnect
    
    Serial.Println("[CMD Koneksi] MQTT ready for reconnect");
  }
  
  Serial.Println("[CMD Koneksi] ========== SUCCESS! ==========");
}

// Handler: Update Schedule (jadwal harian)
static void handleCmdSchedule(const char* payload, unsigned int length) {
  Serial.Println("[CMD Schedule] ========== START ==========");
  
  if (length > 2048) {
    Serial.Println("[CMD Schedule] Payload too large");
    return;
  }
  
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("[CMD Schedule] JSON parse error: ");
    dataPublishAck("schedule", "failed", "Invalid JSON format");
    Serial.Println(err.c_str());
    return;
  }

  Serial.Println("[CMD Schedule] JSON parsed OK");
  bool changed = false;

  JsonArray daily = doc.as<JsonArray>();
  if (daily.isNull()) {
    dataPublishAck("schedule", "failed", "Payload must be root array");
    return;
  }

  auto isValidHHMM = [](const char* s) -> bool {
    if (!s || strlen(s) != 5 || s[2] != ':') return false;
    if (!isdigit(s[0]) || !isdigit(s[1]) || !isdigit(s[3]) || !isdigit(s[4])) return false;
    int h = (s[0] - '0') * 10 + (s[1] - '0');
    int m = (s[3] - '0') * 10 + (s[4] - '0');
    return (h >= 0 && h <= 23 && m >= 0 && m <= 59);
  };

  for (JsonVariant v : daily) {
    JsonObject dayObj = v.as<JsonObject>();
    if (dayObj.isNull()) continue;

    if (!dayObj.containsKey("day") || !dayObj["day"].is<int>()) {
      Serial.Println("[CMD Schedule] SKIP: 'day' must be numeric (0..6)");
      continue;
    }
    int numericDay = dayObj["day"].as<int>();
    if (numericDay < 0 || numericDay > 6) {
      Serial.Printf("[CMD Schedule] SKIP: invalid day=%d (valid: 0..6)\n", numericDay);
      continue;
    }
    int idx = DAY_NUM_TO_IDX(numericDay);

    const char* type = dayObj["type"] | "";
    if (!type || !*type) {
      Serial.Printf("[CMD Schedule] SKIP: missing type for day=%d\n", numericDay);
      continue;
    }

    DaySchedule& d = cfg.setting.daily[idx];
    memset(d.periods, 0, sizeof(d.periods));
    d.periodCount = 0;

    if (strcasecmp(type, "offtime") == 0 || strcasecmp(type, "libur") == 0) {
      d.type = DAY_OFFTIME;
      changed = true;
      continue;
    }

    if (strcasecmp(type, "fulltime") == 0 || strcasecmp(type, "full_time") == 0) {
      d.type = DAY_FULLTIME;
      d.periodCount = 1;
      strncpy(d.periods[0].start, "00:00", sizeof(d.periods[0].start) - 1);
      strncpy(d.periods[0].end, "23:59", sizeof(d.periods[0].end) - 1);
      d.periods[0].set_temp = cfg.setting.global_set_temp;
      d.periods[0].min_temp = cfg.setting.global_min_temp;
      d.periods[0].max_temp = cfg.setting.global_max_temp;
      strncpy(d.periods[0].mode, cfg.setting.global_mode.c_str(), sizeof(d.periods[0].mode) - 1);
      strncpy(d.periods[0].fan, cfg.setting.global_fan.c_str(), sizeof(d.periods[0].fan) - 1);
      changed = true;
      continue;
    }

    if (strcasecmp(type, "parttime") != 0 && strcasecmp(type, "part_time") != 0) {
      Serial.Printf("[CMD Schedule] SKIP: invalid type='%s' day=%d\n", type, numericDay);
      continue;
    }

    JsonArray arr = dayObj["data"].as<JsonArray>();
    if (arr.isNull() || arr.size() == 0) {
      Serial.Printf("[CMD Schedule] SKIP: day=%d parttime requires data[]\n", numericDay);
      continue;
    }

    d.type = DAY_PARTTIME;
    uint8_t cnt = 0;
    for (JsonVariant pv : arr) {
      if (cnt >= MAX_PERIODS_PER_DAY) break;
      JsonObject p = pv.as<JsonObject>();
      if (p.isNull()) continue;

      const char* start = p["start"] | "";
      const char* end = p["end"] | "";
      if (!isValidHHMM(start) || !isValidHHMM(end)) continue;

      strncpy(d.periods[cnt].start, start, sizeof(d.periods[cnt].start) - 1);
      strncpy(d.periods[cnt].end, end, sizeof(d.periods[cnt].end) - 1);
      d.periods[cnt].set_temp = p["set_temp"] | cfg.setting.global_set_temp;
      d.periods[cnt].min_temp = p["min_temp"] | cfg.setting.global_min_temp;
      d.periods[cnt].max_temp = p["max_temp"] | cfg.setting.global_max_temp;
      const char* mode = p["mode"] | cfg.setting.global_mode.c_str();
      const char* fan = p["fan"] | cfg.setting.global_fan.c_str();
      strncpy(d.periods[cnt].mode, mode, sizeof(d.periods[cnt].mode) - 1);
      strncpy(d.periods[cnt].fan, fan, sizeof(d.periods[cnt].fan) - 1);
      cnt++;
    }

    if (cnt == 0) {
      Serial.Printf("[CMD Schedule] SKIP: day=%d has no valid periods\n", numericDay);
      d.type = DAY_OFFTIME;
      continue;
    }

    d.periodCount = cnt;
    changed = true;
    yield();
  }

  if (changed) {
    yield();
    esp_task_wdt_reset();
    
    Serial.Println("[CMD Schedule] Saving schedule...");
    yield();
    
    if (!cfgSave()) {
      Serial.Println("[CMD Schedule] ERROR: Failed to save schedule");
      dataPublishAck("schedule", "failed", "Storage write error");
      Serial.Println("[CMD Schedule] ========== FAILED! ==========");
      return;
    }
    
    Serial.Println("[CMD Schedule] Schedule saved successfully");
    yield();
    esp_task_wdt_reset();
    
    // Apply config changes
    cfgApply();
    yield();
    
    if (mqtt.connected()) {
      Serial.Println("[CMD Schedule] Publishing updated setting to MQTT...");
      dataPublishSetting();
    } else {
      Serial.Println("[CMD Schedule] WARNING: MQTT not connected, cannot publish");
    }
    dataPublishAck("schedule", "success", "Schedule updated");
    Serial.Println("[CMD Schedule] ========== SUCCESS! ==========");
  } else {
    Serial.Println("[CMD Schedule] No valid changes made");
    Serial.Println("[CMD Schedule] ========== END (No Change) ==========");
    dataPublishAck("schedule", "success", "No changes made");
  }
}

static void handleCmdRemote(const char* payload, unsigned int length) {
  Serial.Println("[CMD Remote] ========== START ==========");
  
  if (length > 512) {
    Serial.Println("[CMD Remote] Payload too large");
    dataPublishAck("remote", "failed", "Payload too large");
    return;
  }

  esp_task_wdt_reset();
  yield();
  
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.Println("[CMD Remote] JSON parse error: ");
    dataPublishAck("remote", "failed", "Invalid JSON format");
    Serial.Println(err.c_str());
    return;
  }
  
  Serial.Println("[CMD Remote] JSON parsed OK");
  
  // Validasi: Tolak jika ada nested "remote" object
  if (doc.containsKey("remote")) {
    Serial.Println("[CMD Remote] ERROR: Invalid format - nested 'remote' object detected");
    Serial.Println("[CMD Remote] Expected format: {\"power\":true,\"temp\":23,\"mode\":\"COOL\"}");
    Serial.Println("[CMD Remote] Received format: {\"remote\":{...}} <- INVALID!");
    dataPublishAck("remote", "failed", "Invalid format: remove nested 'remote' wrapper");
    Serial.Println("[CMD Remote] ========== REJECTED ==========");
    return;
  }
  
  // Validasi: Minimal harus ada salah satu key yang valid
  if (!doc.containsKey("power") && !doc.containsKey("temp") && 
      !doc.containsKey("mode") && !doc.containsKey("fan") &&
      !doc.containsKey("swing") && !doc.containsKey("sleep") && 
      !doc.containsKey("turbo") && !doc.containsKey("lights") &&
      !doc.containsKey("health")) {
    Serial.Println("[CMD Remote] ERROR: No valid command keys found");
    Serial.Println("[CMD Remote] Valid keys: power, temp, mode, fan, swing, sleep, turbo, lights, health");
    dataPublishAck("remote", "failed", "No valid command keys");
    Serial.Println("[CMD Remote] ========== REJECTED ==========");
    return;
  }
  
  esp_task_wdt_reset();
  yield();
  
  Serial.Println("[CMD Remote] Sending IR command...");
  
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
  String expectedMode = doc.containsKey("mode") ? doc["mode"].as<String>() : (baselineFrame.valid ? baselineFrame.mode : cfg.setting.global_mode);
  String expectedFan = doc.containsKey("fan") ? doc["fan"].as<String>() : (baselineFrame.valid ? baselineFrame.fan : cfg.setting.global_fan);
  uint8_t expectedTemp = doc.containsKey("temp") ? doc["temp"].as<uint8_t>() : cfg.setting.global_set_temp;
  
  // Set expected state di automation module
  automationSetExpectedStateFromMqtt(expectedPower, expectedMode, expectedFan, expectedTemp);
  
  esp_task_wdt_reset();
  yield();
  
  String cmdStr;
  cmdStr.reserve(512);
  serializeJson(doc, cmdStr);
  
  Serial.Printf("[CMD Remote] Command: %s\n", cmdStr.c_str());
  Serial.Printf("[CMD Remote] Expected: power=%s mode=%s fan=%s set_temp=%d\n",
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
    Serial.Println("[CMD Remote] Single-send experiment: retry disabled (1 MQTT = 1 IR send)");
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
    Serial.Printf("[CMD Remote] IR send attempt %u/%u\n", (unsigned)(sendAttempt + 1), (unsigned)maxSendAttempts);
    controllEnqueueCommand(cmdStr.c_str(), cmdStr.length());

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
      Serial.Printf("[CMD Remote] No matching display response within %lu ms, retrying...\n", (unsigned long)REMOTE_DISPLAY_WAIT_MS);
    }
  }
  
  if (!anyActivityTracked) {
    Serial.Println("[CMD Remote] AC did not respond - activity not tracked");
    dataPublishAck("remote", "no_response", "No fresh state change detected after command");
    Serial.Println("[CMD Remote] ========== NO RESPONSE ==========");
    return;
  }
  
  esp_task_wdt_reset();
  yield();
  
  // ═══════════════════════════════════════════════════════════════════
  // MQTT remote tidak trigger manual recovery.
  // Manual recovery hanya dari data AC (monitorExternalChange).
  // ═══════════════════════════════════════════════════════════════════
  
  dataPublishAck("remote", "success", "Command executed");
  Serial.Println("[CMD Remote] ========== SUCCESS! ==========");
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
  Serial.Printf("[DEBUG] Recv Topic: '%s' (Len: %d)\n", topicStr.c_str(), topicStr.length());
  Serial.Printf("[MQTT] Payload: %s\n", payload);

  if (topicStr.equals(cmdRemote)) {
    Serial.Println("[MQTT] → handleCmdRemote");
    handleCmdRemote(payload, length);
  }
  else if (topicStr.equals(cmdSetting)) {
    Serial.Println("[MQTT] → handleCmdSetting");
    handleCmdSetting(payload, length);
  }
  else if (topicStr.equals(cmdKoneksi)) {
    Serial.Println("[MQTT] → handleCmdKoneksi");
    handleCmdKoneksi(payload, length);
  }
  else if (topicStr.equals(cmdSchedule)) {
    Serial.Println("[MQTT] → handleCmdSchedule");
    handleCmdSchedule(payload, length);
  }
  else if (topicStr.equals(topicCmdUpdate)) {
    Serial.Println("[MQTT] → handleCmdUpdate MATCHED!");
    handleCmdUpdate(payload, length);
  }

  else if (topicStr.equals(cmdInfo)) {
    Serial.Println("╔════════════════════════════════════════╗");
    Serial.Println("║  📊 INFO REQUEST RECEIVED             ║");
    Serial.Println("╚════════════════════════════════════════╝");
    dataPublishAck("info", "processing", "Generating system info...");
    delay(300);
    esp_task_wdt_reset();
    dataPublishInfo();
    dataPublishAck("info", "success", "System info published to /info topic");
    Serial.Println("[MQTT] Info request completed");
  }
  else if (topicStr.equals(cmdReboot)) {
    Serial.Println("╔════════════════════════════════════════╗");
    Serial.Println("║  🔄 REMOTE REBOOT COMMANDED           ║");
    Serial.Println("╚════════════════════════════════════════╝");
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    int delaySeconds = 3;
    if (!err && doc.containsKey("delay_seconds")) {
      delaySeconds = doc["delay_seconds"].as<int>();
      if (delaySeconds < 1) delaySeconds = 1;
      if (delaySeconds > 60) delaySeconds = 60;
    }
    Serial.Printf("[Reboot] Rebooting in %d seconds...\n", delaySeconds);
    String ackMsg = "Rebooting in " + String(delaySeconds) + " seconds...";
    dataPublishAck("reboot", "success", ackMsg);
    publishStatus("rebooting");
    if (LittleFS.begin()) {
      File f = LittleFS.open("/shutdown.flag", "w");
      if (f) {
        f.println("graceful_reboot");
        f.close();
        Serial.Println("[Reboot] Graceful shutdown flag created");
      }
      LittleFS.remove("/running.flag");
    }
    unsigned long startMs = millis();
    while (millis() - startMs < (delaySeconds * 1000)) {
      delay(500);
      esp_task_wdt_reset();
      mqtt.loop();
    }
    Serial.Println("[Reboot] Restarting now...");
    delay(500);
    ESP.restart();
  }
  else {
    Serial.Printf("[MQTT] Unknown/Ignored topic: %s\n", topicStr.c_str());
    if (topicStr.indexOf("update") >= 0) {
      Serial.Println("[DEBUG] WARNING: Topik mengandung kata 'update' tapi tidak cocok persis.");
      Serial.Printf("[DEBUG] Harapan: '%s'\n", topicCmdUpdate.c_str());
      Serial.Printf("[DEBUG] Diterima: '%s'\n", topicStr.c_str());
    }
  }
}

static void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length > 2048) {
    Serial.Println("[MQTT] Payload too large, rejected");
    return;
  }
  if (pendingCommandCount >= 4) {
    Serial.Println("[MQTT] Command queue full, dropping incoming command");
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
  Serial.Printf("[MQTT] Queued command topic='%s' len=%u depth=%u\n", slot.topic, slot.length, pendingCommandCount);
}

void dataCheckPendingSave() {
  if (!pendingSaveRequest) return;
  
  // Tunggu minimal 2 detik setelah request untuk memastikan tidak ada operasi baca/tulis lain
  if (millis() - saveRequestTime < 2000) return;
  
  Serial.Println("[Data] Processing pending config save...");
  
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
    Serial.Println("[Data] ✓ Pending config saved successfully");
    
    // Apply config baru (terutama jika ada perubahan logic automation)
    cfgApply();
    
    // Reconnect MQTT Logic
    if (wasMqttConnected) {
        // Kita biarkan loop utama yang melakukan reconnect 
        // agar logic-nya terpusat di dataLoop/ensureMqttConnected
        Serial.Println("[Data] Config saved, MQTT will reconnect in main loop");
    }
  } else {
    Serial.Println("[Data] ✗ Failed to save pending config");
  }
  
  pendingSaveRequest = false;
}

bool dataPublishRaw(const String& topic, const String& payload) {
  if (!mqtt.connected()) return false;
  return mqtt.publish(topic.c_str(), payload.c_str(), false);
}

bool dataPublishInfo() {
  if (!mqtt.connected()) {
    Serial.Println("[Info] Cannot publish - MQTT not connected");
    return false;
  }

  StaticJsonDocument<4096> doc;

  // ═══════════════════════════════════════════════
  // 1. FIRMWARE & BUILD
  // ═══════════════════════════════════════════════
  JsonObject fw = doc.createNestedObject("firmware");
  fw["version"] = FW_VERSION;
  fw["built_at"] = String(__DATE__) + " " + String(__TIME__);
  fw["reset_reason"] = getResetReason();
  doc["ac_brand"] = cfg.ac_brand;

  // ═══════════════════════════════════════════════
  // 2. NETWORK DETAILED
  // ═══════════════════════════════════════════════
  JsonObject net = doc.createNestedObject("network");
  net["ip"] = WiFi.localIP().toString();
  net["mac"] = WiFi.macAddress();
  net["ssid"] = WiFi.SSID();
  net["rssi"] = WiFi.RSSI();
  net["gateway"] = WiFi.gatewayIP().toString();
  net["dns"] = WiFi.dnsIP(0).toString();

  // ═══════════════════════════════════════════════
  // 3. MEMORY HEALTH
  // ═══════════════════════════════════════════════
  JsonObject mem = doc.createNestedObject("memory");
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxAlloc = ESP.getMaxAllocHeap();
  mem["heap_free"] = freeHeap;
  mem["heap_min"] = ESP.getMinFreeHeap();
  mem["heap_max_alloc"] = maxAlloc;
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
  conn["mqtt_reconnects"] = mqttReconnectCount;
  conn["wifi_disconnects"] = wifiDisconnectCount;

  CrashStats cStats = getCrashStats();
  JsonObject crash = doc.createNestedObject("crash_stats");
  crash["total_boots"] = cStats.totalBoots;
  crash["total_crashes"] = cStats.totalCrashes;
  crash["crash_rate"] = cStats.crashRate;
  crash["brownouts"] = cStats.brownoutResets;

  // ═══════════════════════════════════════════════
  // 5. SENSOR HEALTH
  // ═══════════════════════════════════════════════
  JsonObject sens = doc.createNestedObject("sensor");
  sens["healthy"] = sensorIsHealthy();
  sens["uart_errors"] = uartErrorCount;
  if (sensorHasValidFrame()) {
    unsigned long frameAge = sensorTimeSinceLastFrame();
    if (frameAge < 3600000UL) {
      sens["last_frame_age"] = frameAge / 1000;
      sens["last_rx_age_sec"] = frameAge / 1000;
    } else {
      sens["last_frame_age"] = -1;
      sens["last_rx_age_sec"] = -1;
    }
  } else {
    sens["last_frame_age"] = -1;
    sens["last_rx_age_sec"] = -1;
  }
  sens["last_rx_ms"] = sensorGetLastValidFrameMs();
  sens["comm_error_inferred"] = sensorHasInferredCommError();

  // ═══════════════════════════════════════════════
  // 6. FILESYSTEM
  // ═══════════════════════════════════════════════
  JsonObject fs = doc.createNestedObject("filesystem");
  fs["used_pct"] = (LittleFS.usedBytes() * 100.0) / LittleFS.totalBytes();

  // ═══════════════════════════════════════════════
  // 7. WIFI NEARBY & TIMESTAMP
  // ═══════════════════════════════════════════════
  JsonArray networks = doc.createNestedArray("wifi_nearby");
  appendNearbyWifiNetworks(networks);

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    doc["timestamp"] = buf;
  } else {
    doc["timestamp"] = "UNSYNCED";
  }

  // ═══════════════════════════════════════════════
  // PUBLISH
  // ═══════════════════════════════════════════════
  String payload;
  payload.reserve(3072);
  size_t len = serializeJson(doc, payload);

  mqtt.loop();
  yield();
  esp_task_wdt_reset();

  if (mqtt.publish(topicInfo.c_str(), payload.c_str(), true)) {
    Serial.Printf("[Info] ✓ Published %d bytes (fw=%s)\n", len, FW_VERSION);
    return true;
  } else {
    Serial.Println("[Info] ✗ Failed to publish");
    return false;
  }
}

bool dataPublishAck(const char* command, const char* status, const String& detail) {
    if (!mqtt.connected()) {
      Serial.Printf("[ACK] Skip publish, MQTT disconnected: %s -> %s (%s)\n", command, status, detail.c_str());
      return false;
    }
    if (topicAck.length() == 0) {
      Serial.Printf("[ACK] Skip publish, topicAck empty: %s -> %s (%s)\n", command, status, detail.c_str());
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
      Serial.Printf("[ACK] %s -> %s (%s)\n", command, status, detail.c_str());
      return true;
    }

    consecutivePublishFails++;
    Serial.Printf("[ACK] Publish failed: %s -> %s (%s), state=%d, fails=%d\n",
              command, status, detail.c_str(), mqtt.state(), consecutivePublishFails);
    return false;
}

// dataPublishDiagnostics() — REMOVED.
// All diagnostic data merged into dataPublishInfo() above.
// The /diagnostics topic is no longer published.