#include "data.h"
#include "config.h"
#include "controll.h"

#include "crash_monitor.h"
#include "automation.h"
#include "ir_health.h"
#include "offline_log.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "ota.h"
#include "time.h"
#include "activity.h"
#include "rtos_common.h"

// ============= MQTT Topics =============
static String topicStatus;      // isems/devices/{id}/status
static String topicData;        // isems/devices/{id}/data
static String topicSetting;     // isems/devices/{id}/setting
static String topicSettingDayPrefix; // isems/devices/{id}/setting/day/

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

static BPFrame lastKnownState = {};



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
        case REASON_USB_RESET: return "USB_RESET";
        case REASON_JTAG_RESET: return "JTAG_RESET";
        case REASON_EFUSE_RESET: return "EFUSE_RESET";
        case REASON_POWER_GLITCH: return "POWER_GLITCH";
        case REASON_CPU_LOCKUP: return "CPU_LOCKUP";
        default: return "UNKNOWN";
    }
}

static ActionSource lastActionSource = ACTION_NONE;
static unsigned long lastActionMs = 0;

// ============= MQTT Client =============
static WiFiClientSecure espClient;  // Gunakan WiFiClientSecure untuk SSL/TLS
static PubSubClient mqtt(espClient);

// WiFiClientSecure::connect() menjalankan TCP lalu TLS handshake secara blocking.
// Tanpa batas ini, packet loss atau captive portal dapat menahan NetTask puluhan
// detik sebelum mqtt.connect() mengembalikan hasil.
static const uint16_t MQTT_TLS_HANDSHAKE_TIMEOUT_SEC = 12;
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
static bool pendingSettingPublish = false;
static uint8_t pendingSettingDayMask = 0;
static unsigned long lastSettingPublishAttemptMs = 0;
static unsigned long lastSettingDayPublishAttemptMs = 0;
static const unsigned long SETTING_PUBLISH_RETRY_MS = 1000UL;

static bool immediateWiFiRetryRequested = false;
static bool wifiCredentialTrialActive = false;
static bool wifiCredentialRollbackAckPending = false;
static KoneksiCfg wifiCredentialTrialPrevious;
static KoneksiCfg wifiCredentialTrialTarget;
static unsigned long wifiCredentialTrialStartedMs = 0;
static const unsigned long WIFI_CREDENTIAL_TRIAL_TIMEOUT_MS = 9UL * 60UL * 1000UL;

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
static void serviceDeferredRemoteCheck();
static void serviceOfflineLogFlush();
static void rollbackWiFiCredentialTrial(const char* reason);

// ============= Helper: Stop SSL client + restore callback =============
// WAJIB dipakai setiap kali espClient.stop() dipanggil sebelum reconnect.
// PubSubClient menyimpan pointer callback secara internal — espClient.stop()
// tidak menghapusnya, tapi mqtt.disconnect() + espClient.stop() dalam urutan
// tertentu bisa membuat state internal PubSubClient tidak konsisten.
// Memanggil setCallback() ulang setelah stop adalah cara paling aman.
static void stopAndResetClient() {
  if (mqtt.connected()) {
    mqtt.disconnect();
  }
  espClient.stop();
  // Restore SSL policy
  if (PROD_ROOT_CA != NULL) {
    espClient.setCACert(PROD_ROOT_CA);
  } else {
    espClient.setInsecure();
  }
  espClient.setHandshakeTimeout(MQTT_TLS_HANDSHAKE_TIMEOUT_SEC);
  // Restore callback — ini yang paling kritis
  mqtt.setCallback(handleMqttMessage);
}

// ============= Build Topics =============
void dataBuildTopics(const String& device_id) {
  // Publish topics
  topicStatus  = "isems/devices/" + device_id + "/status";
  topicData    = "isems/devices/" + device_id + "/data";
  topicSetting = "isems/devices/" + device_id + "/setting";
  topicSettingDayPrefix = topicSetting + "/day/";
  pendingSettingDayMask = 0x7F;
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
    case ESP_RST_USB: return "USB Peripheral Reset";
    case ESP_RST_JTAG: return "JTAG Reset";
    case ESP_RST_EFUSE: return "eFuse Error Reset";
    case ESP_RST_PWR_GLITCH: return "Power Glitch Reset";
    case ESP_RST_CPU_LOCKUP: return "CPU Lockup Reset";
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

struct RtcBatteryReading {
  uint16_t raw;
  uint16_t adcMv;
  float voltage;
  int percent;
};

static int estimateCr1220Percent(float voltage) {
  if (voltage >= 3.00f) return 100;
  if (voltage <= 2.00f) return 0;

  // CR1220 has a fairly flat discharge curve; treat this as a health estimate.
  if (voltage >= 2.85f) {
    return 80 + (int)((voltage - 2.85f) * (20.0f / 0.15f) + 0.5f);
  }
  if (voltage >= 2.70f) {
    return 50 + (int)((voltage - 2.70f) * (30.0f / 0.15f) + 0.5f);
  }
  if (voltage >= 2.50f) {
    return 20 + (int)((voltage - 2.50f) * (30.0f / 0.20f) + 0.5f);
  }
  return (int)((voltage - 2.00f) * (20.0f / 0.50f) + 0.5f);
}

static RtcBatteryReading readRtcBattery() {
  const uint8_t samples = 12;
  uint32_t rawSum = 0;
  uint32_t mvSum = 0;

  for (uint8_t i = 0; i < samples; ++i) {
    rawSum += analogRead(BAT_ADC_PIN);
    mvSum += analogReadMilliVolts(BAT_ADC_PIN);
    delayMicroseconds(250);
  }

  RtcBatteryReading reading;
  reading.raw = rawSum / samples;
  reading.adcMv = mvSum / samples;
  float dividerRatio = (BAT_ADC_R_TOP_OHM + BAT_ADC_R_BOT_OHM) / BAT_ADC_R_BOT_OHM;
  reading.voltage = (reading.adcMv / 1000.0f) * dividerRatio;
  reading.percent = constrain(estimateCr1220Percent(reading.voltage), 0, 100);
  return reading;
}

// ============= Publish Setting =============
bool dataPublishSettingDay(uint8_t day) {
  if (day > 6) return false;
  const uint8_t dayBit = static_cast<uint8_t>(1U << day);

  if (cfgIsRuntimeDefaultDueLoadFailure()) {
    pendingSettingDayMask &= static_cast<uint8_t>(~dayBit);
    Serial.Printf("[Data] Skip setting day %u: runtime default config from load failure\n", day);
    return false;
  }

  if (!mqtt.connected() || !dataIsPublishReady() || topicSettingDayPrefix.length() == 0) {
    pendingSettingDayMask |= dayBit;
    return false;
  }

  DaySchedule snapshot = {};
  bool locked = false;
  if (g_cfgMutex) {
    if (CFG_LOCK() != pdTRUE) {
      pendingSettingDayMask |= dayBit;
      return false;
    }
    locked = true;
  }
  snapshot = cfg.setting.daily[day];
  if (locked) CFG_UNLOCK();

  StaticJsonDocument<3072> doc;
  doc["day"] = DAY_IDX_TO_NUM(day);
  if (snapshot.type == DAY_OFFTIME) {
    doc["type"] = "offtime";
  } else if (snapshot.type == DAY_FULLTIME) {
    doc["type"] = "fulltime";
  } else {
    doc["type"] = "parttime";
  }

  JsonArray periods = doc.createNestedArray("data");
  uint8_t count = snapshot.periodCount;
  if (count > MAX_PERIODS_PER_DAY) count = MAX_PERIODS_PER_DAY;
  for (uint8_t index = 0; index < count; ++index) {
    const TimePeriod& period = snapshot.periods[index];
    JsonObject item = periods.createNestedObject();
    item["start"] = period.start;
    item["end"] = period.end;
    item["mode"] = period.mode;
    item["fan"] = period.fan;
    item["set_temp"] = period.set_temp;
  }

  if (doc.overflowed()) {
    pendingSettingDayMask |= dayBit;
    Serial.Printf("[Data] Setting day %u JSON overflow\n", day);
    return false;
  }

  String payload;
  payload.reserve(1536);
  serializeJson(doc, payload);
  const String topic = topicSettingDayPrefix + String(day);
  lastSettingDayPublishAttemptMs = millis();

  mqtt.loop();
  yield();
  esp_task_wdt_reset();
  const bool published = mqtt.publish(topic.c_str(), payload.c_str(), true);
  if (published) {
    pendingSettingDayMask &= static_cast<uint8_t>(~dayBit);
    consecutivePublishFails = 0;
    Serial.Printf("[Data] Published setting day %u (%u bytes)\n",
                  day,
                  static_cast<unsigned>(payload.length()));
  } else {
    pendingSettingDayMask |= dayBit;
    ++consecutivePublishFails;
    Serial.Printf("[Data] Failed setting day %u publish (state=%d)\n", day, mqtt.state());
  }
  return published;
}

void dataPublishSettingDays(uint8_t dayMask) {
  dayMask &= 0x7F;
  pendingSettingDayMask |= dayMask;
  if (!mqtt.connected() || !dataIsPublishReady()) return;

  for (uint8_t day = 0; day < 7; ++day) {
    const uint8_t dayBit = static_cast<uint8_t>(1U << day);
    if ((dayMask & dayBit) == 0) continue;
    if (!dataPublishSettingDay(day) && !mqtt.connected()) return;
  }
}

void dataPublishSetting() {
  if (cfgIsRuntimeDefaultDueLoadFailure()) {
    pendingSettingPublish = false;
    Serial.Println("[Data] Skip setting publish: runtime default config from load failure");
    return;
  }

  // Pastikan MQTT connected
  if (!mqtt.connected()) {
    Serial.Println("[Data] MQTT not connected, cannot publish setting");
    pendingSettingPublish = true;
    return;
  }

  if (!dataIsPublishReady()) {
    unsigned long now = millis();
    if (now - lastDeferredPublishLogMs >= 1000UL) {
      lastDeferredPublishLogMs = now;
      Serial.Printf("[Data] Deferring setting publish until MQTT cooldown passes (%lu ms remaining)\n",
                MQTT_POST_CONNECT_PUBLISH_COOLDOWN_MS - (now - lastMqttConnectMs));
    }
    pendingSettingPublish = true;
    return;
  }
  
  // Cek apakah topic sudah di-inisialisasi
  if (topicSetting.length() == 0) {
    Serial.Println("[Data] ERROR: topicSetting not initialized");
    pendingSettingPublish = true;
    return;
  }
  lastSettingPublishAttemptMs = millis();

  // The weekly schedule is published only through retained setting/day/{0..6}
  // topics. Keeping it out of the base setting payload avoids duplicating up to
  // 70 periods and prevents consumers from treating a partial update as a
  // complete weekly replacement.
  StaticJsonDocument<128> doc;
  doc["automation"] = cfg.setting.automation;

  String payload;
  payload.reserve(128);
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
    pendingSettingPublish = false;
    Serial.Printf("[Data] ✓ Published setting successfully to: %s\n", topicSetting.c_str());
  } else {
    consecutivePublishFails++;
    pendingSettingPublish = true;
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

static bool publishDataPayload(const String& payload, bool retained) {
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

  bool ok = mqtt.beginPublish(topicData.c_str(), payloadLength, retained);
  if (ok) {
    size_t written = mqtt.write(bytes, payloadLength);
    ok = (written == payloadLength) && (mqtt.endPublish() == 1);
  }
  if (!ok) {
    consecutivePublishFails++;
    Serial.Printf("[Data] Failed to publish %s data to: %s (len=%u, fails=%d)\n",
                  retained ? "retained" : "non-retained",
                  topicData.c_str(), static_cast<unsigned>(payloadLength), consecutivePublishFails);
    return false;
  }

  consecutivePublishFails = 0;
  mqtt.loop();
  delay(50);
  return true;
}

static bool publishDataRetainedPayload(const String& payload) {
  return publishDataPayload(payload, true);
}

static bool getCurrentEpoch(uint32_t& epoch) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 10) || (timeinfo.tm_year + 1900) <= 2020) {
    return false;
  }

  time_t now = time(nullptr);
  if (now < 1609459200) {
    return false;
  }

  epoch = static_cast<uint32_t>(now);
  return true;
}

static bool formatEpochDeviceTime(uint32_t epoch, char* out, size_t outLen) {
  if (out == nullptr || outLen == 0) return false;
  time_t t = static_cast<time_t>(epoch);
  struct tm timeinfo;
  localtime_r(&t, &timeinfo);
  return strftime(out, outLen, "%Y-%m-%d %H:%M:%S", &timeinfo) > 0;
}

static void storeOfflineTelemetry(const BPFrame& d, bool read_ok, const char* reason) {
  if (!read_ok || !d.valid) {
    Serial.Printf("[OfflineLog] Skip store: invalid sensor frame (%s)\n", reason ? reason : "-");
    return;
  }

  uint32_t epoch = 0;
  if (!getCurrentEpoch(epoch)) {
    Serial.Println("[OfflineLog] Skip store: device time not valid");
    return;
  }

  bool ok = offlineLogAppend(epoch, d.power, static_cast<int16_t>(d.current_temp));
  if (ok) {
    Serial.Printf("[OfflineLog] Stored telemetry (%s), count=%lu/%lu file=%u fs_free=%u fs_used=%u/%u\n",
                  reason ? reason : "-",
                  static_cast<unsigned long>(offlineLogCount()),
                  static_cast<unsigned long>(offlineLogCapacity()),
                  static_cast<unsigned>(offlineLogFileBytes()),
                  static_cast<unsigned>(offlineLogFsFreeBytes()),
                  static_cast<unsigned>(offlineLogFsUsedBytes()),
                  static_cast<unsigned>(offlineLogFsTotalBytes()));
  } else {
    Serial.Printf("[OfflineLog] Failed to store telemetry (%s)\n", reason ? reason : "-");
  }
}

static bool buildOfflinePayload(const OfflineLogRecord& record, String& payload) {
  char timeStr[32];
  if (!formatEpochDeviceTime(record.epoch, timeStr, sizeof(timeStr))) {
    return false;
  }

  StaticJsonDocument<256> doc;
  doc["mac"] = WiFi.macAddress();
  doc["offline"] = true;
  doc["device_time"] = timeStr;
  doc["power"] = record.power != 0;
  doc["room_temp"] = static_cast<int>(record.room_temp);

  payload = "";
  serializeJson(doc, payload);
  return !payload.isEmpty();
}

static void serviceOfflineLogFlush() {
  static unsigned long lastFlushMs = 0;
  static const unsigned long FLUSH_INTERVAL_MS = 3000UL;

  if (!mqtt.connected() || topicData.length() == 0 || !dataIsPublishReady()) return;
  if (offlineLogCount() == 0) return;

  unsigned long now = millis();
  if (now - lastFlushMs < FLUSH_INTERVAL_MS) return;
  lastFlushMs = now;

  OfflineLogRecord record = {};
  if (!offlineLogPeek(record)) {
    Serial.Println("[OfflineLog] Dropping unreadable/corrupt record");
    offlineLogPop();
    return;
  }

  String payload;
  payload.reserve(160);
  if (!buildOfflinePayload(record, payload)) {
    Serial.Println("[OfflineLog] Dropping record with invalid timestamp");
    offlineLogPop();
    return;
  }

  if (publishDataPayload(payload, false)) {
    offlineLogPop();
    Serial.Printf("[OfflineLog] Flushed 1 record, remaining=%lu file=%u fs_free=%u fs_used=%u/%u\n",
                  static_cast<unsigned long>(offlineLogCount()),
                  static_cast<unsigned>(offlineLogFileBytes()),
                  static_cast<unsigned>(offlineLogFsFreeBytes()),
                  static_cast<unsigned>(offlineLogFsUsedBytes()),
                  static_cast<unsigned>(offlineLogFsTotalBytes()));
  } else {
    Serial.Printf("[OfflineLog] Flush publish failed (state=%d), remaining=%lu file=%u fs_free=%u fs_used=%u/%u\n",
                  mqtt.state(), static_cast<unsigned long>(offlineLogCount()),
                  static_cast<unsigned>(offlineLogFileBytes()),
                  static_cast<unsigned>(offlineLogFsFreeBytes()),
                  static_cast<unsigned>(offlineLogFsUsedBytes()),
                  static_cast<unsigned>(offlineLogFsTotalBytes()));
  }
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
  
  // Reset SSL client + restore callback
  stopAndResetClient();
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
                      topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
  } else {
    ok = mqtt.connect(clientId, nullptr, nullptr,
                      topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
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
    if (!mqtt.subscribe(cmdSetting.c_str(), 1))  Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdSetting.c_str());
    if (!mqtt.subscribe(cmdKoneksi.c_str()))     Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdKoneksi.c_str());
    if (!mqtt.subscribe(cmdSchedule.c_str(), 1)) Serial.Printf("[MQTT] ✗ Subscribe failed: %s\n", cmdSchedule.c_str());
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
      stopAndResetClient();
      yield();
      bool notifyOk;
      buildLwtPayload();
      if (mqttUser.length()) {
        notifyOk = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                                topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
      } else {
        notifyOk = mqtt.connect(clientId, nullptr, nullptr,
                                topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
      }
      if (notifyOk) {
        publishStatus("offline");
        delay(100);
        stopAndResetClient();
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
  pinMode(BAT_ADC_PIN, INPUT);
  analogReadResolution(12);
  #if defined(ADC_11db)
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
  #endif

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
  espClient.setHandshakeTimeout(MQTT_TLS_HANDSHAKE_TIMEOUT_SEC);
  
  // 3. Konfigurasi MQTT Client
  mqtt.setBufferSize(8192);    // Full retained setting can include many schedule periods
  mqtt.setKeepAlive(30);       // 30 detik keepalive
  mqtt.setSocketTimeout(15);   // 15 detik timeout MQTT socket
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
  } else if (r == ESP_RST_PWR_GLITCH) {
      lastDisconnectReason = REASON_POWER_GLITCH;
      Serial.Println("[Data] Disconnect reason: POWER GLITCH RESET");
  } else if (r == ESP_RST_PANIC) {
      lastDisconnectReason = REASON_CRASH;
      Serial.Println("[Data] Disconnect reason: SOFTWARE CRASH (panic)");
  } else if (r == ESP_RST_CPU_LOCKUP) {
      lastDisconnectReason = REASON_CPU_LOCKUP;
      Serial.Println("[Data] Disconnect reason: CPU LOCKUP / DOUBLE EXCEPTION");
  } else if (r == ESP_RST_POWERON) {
      lastDisconnectReason = REASON_POWER_ON;
      Serial.Println("[Data] Disconnect reason: POWER ON / POWER CYCLE");
  } else if (r == ESP_RST_SW) {
      lastDisconnectReason = REASON_POWER_ON;
      Serial.Println("[Data] Disconnect reason: SOFTWARE RESET (OTA/command)");
  } else if (r == ESP_RST_USB) {
      lastDisconnectReason = REASON_USB_RESET;
      Serial.Println("[Data] Disconnect reason: USB PERIPHERAL RESET");
  } else if (r == ESP_RST_JTAG) {
      lastDisconnectReason = REASON_JTAG_RESET;
      Serial.Println("[Data] Disconnect reason: JTAG RESET");
  } else if (r == ESP_RST_EFUSE) {
      lastDisconnectReason = REASON_EFUSE_RESET;
      Serial.Println("[Data] Disconnect reason: EFUSE ERROR RESET");
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
  offlineLogInit();
  
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
    if (wifiCredentialTrialActive &&
        millis() - wifiCredentialTrialStartedMs >= WIFI_CREDENTIAL_TRIAL_TIMEOUT_MS) {
      rollbackWiFiCredentialTrial("MQTT did not connect within validation window");
    }
    return;
  }
  
  wasMqttConnected = true;
  mqtt.loop();
  dataOnNetworkConnected();

  if (!mqttInitialized && dataIsPublishReady()) {
    yield();
    esp_task_wdt_reset();
    dataPublishSetting();
    if (mqtt.connected()) {
      mqttInitialized = true;
    }
  }

  if (pendingSettingPublish && dataIsPublishReady() &&
      (millis() - lastSettingPublishAttemptMs >= SETTING_PUBLISH_RETRY_MS)) {
    Serial.Println("[Data] Retrying pending setting publish...");
    yield();
    esp_task_wdt_reset();
    dataPublishSetting();
  }

  if (pendingSettingDayMask != 0 && dataIsPublishReady() &&
      (millis() - lastSettingDayPublishAttemptMs >= SETTING_PUBLISH_RETRY_MS)) {
    for (uint8_t day = 0; day < 7; ++day) {
      const uint8_t dayBit = static_cast<uint8_t>(1U << day);
      if ((pendingSettingDayMask & dayBit) != 0) {
        dataPublishSettingDay(day);
        break;
      }
    }
  }

  processQueuedMqttCommand();
  serviceDeferredRemoteCheck();
  serviceOfflineLogFlush();

  // Zombie connection detection: MQTT reports connected tapi publish terus gagal.
  // Ini terjadi saat SSL layer rusak tapi TCP masih "connected".
  if (consecutivePublishFails >= 3) {
    Serial.Println("[MQTT] \u2717 Zombie connection detected (3+ publish fails) \u2014 forcing clean reconnect");
    consecutivePublishFails = 0;
    stopAndResetClient();
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

void dataPauseMqttForOta() {
  Serial.Println("[OTA] Pausing MQTT TLS before HTTP(S) transfer...");
  isManualDisconnect = true;
  if (mqtt.connected()) {
    mqtt.loop();
    delay(100);
  }
  stopAndResetClient();
  mqttInitialized = false;
  resetMqttRc2RecoveryState();
  delay(250);
  esp_task_wdt_reset();
  Serial.Printf("[OTA] MQTT TLS released, heap=%u largest=%u\n",
                ESP.getFreeHeap(),
                static_cast<unsigned>(
                    heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
}

void dataResumeMqttAfterOta() {
  Serial.Println("[OTA] OTA returned without reboot; reconnecting MQTT...");
  isManualDisconnect = false;
  mqttRetryAttempt = 0;
  lastMqttRetryMs = 0;
  mqttInitialized = false;
  dataForceConnect();
}

bool dataIsConnected() {
  return mqtt.connected();
}

void dataScheduleImmediateWiFiRetry() {
  immediateWiFiRetryRequested = true;
}

bool dataConsumeImmediateWiFiRetryRequest() {
  bool requested = immediateWiFiRetryRequested;
  immediateWiFiRetryRequested = false;
  return requested;
}

bool dataIsWiFiCredentialTrialActive() {
  return wifiCredentialTrialActive;
}

bool dataIsWiFiCredentialTrialExpired() {
  return wifiCredentialTrialActive &&
         (millis() - wifiCredentialTrialStartedMs >= WIFI_CREDENTIAL_TRIAL_TIMEOUT_MS);
}

static void rollbackWiFiCredentialTrial(const char* reason) {
  if (!wifiCredentialTrialActive) return;

  Serial.Printf("[WiFi Guard] Trial failed: %s\n", reason ? reason : "unknown");
  Serial.Printf("[WiFi Guard] Rolling back to previous SSID: %s\n",
                wifiCredentialTrialPrevious.wifi_ssid.c_str());

  wifiCredentialTrialActive = false;
  wifiCredentialRollbackAckPending = true;
  cfg.koneksi = wifiCredentialTrialPrevious;
  cfgApply();

  stopAndResetClient();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  dataScheduleImmediateWiFiRetry();
}

static void beginWiFiCredentialTrial(const KoneksiCfg& previous, const KoneksiCfg& target) {
  wifiCredentialTrialPrevious = previous;
  wifiCredentialTrialTarget = target;
  wifiCredentialTrialStartedMs = millis();
  wifiCredentialTrialActive = true;
  wifiCredentialRollbackAckPending = false;

  cfg.koneksi = target;
  cfgApply();

  Serial.Printf("[WiFi Guard] Testing new WiFi SSID: %s\n", target.wifi_ssid.c_str());
  dataPublishAck("koneksi", "processing",
                 "Testing new WiFi credentials; saving only after WiFi and MQTT connect");

  isManualDisconnect = true;
  lastDisconnectReason = REASON_MANUAL;
  lastDisconnectTime = millis();
  publishStatus("offline");
  delay(200);
  stopAndResetClient();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(300);
  dataScheduleImmediateWiFiRetry();
}

static void commitWiFiCredentialTrial() {
  if (!wifiCredentialTrialActive || !mqtt.connected()) return;

  Serial.Printf("[WiFi Guard] WiFi + MQTT validated, committing SSID to NVS: %s\n",
                wifiCredentialTrialTarget.wifi_ssid.c_str());

  if (!cfgSave()) {
    rollbackWiFiCredentialTrial("config save failed after MQTT validation");
    dataPublishAck("koneksi", "failed", "Validation passed but config save failed; rolled back WiFi credentials");
    return;
  }

  nvs_setWiFiSSID(wifiCredentialTrialTarget.wifi_ssid);
  nvs_setWiFiPass(wifiCredentialTrialTarget.wifi_pass);
  wifiCredentialTrialActive = false;
  wifiCredentialRollbackAckPending = false;

  dataPublishAck("koneksi", "success", "WiFi credentials validated and saved");
  dataPublishSetting();
}

void dataOnWiFiCredentialTrialFailedAfterRetries() {
  rollbackWiFiCredentialTrial("WiFi failed for 3 retry intervals");
}

void dataOnNetworkConnected() {
  if (wifiCredentialTrialActive) {
    commitWiFiCredentialTrial();
  } else if (wifiCredentialRollbackAckPending && mqtt.connected()) {
    wifiCredentialRollbackAckPending = false;
    dataPublishAck("koneksi", "failed", "New WiFi credentials failed; restored previous WiFi credentials");
  }
  otaPendingAckService();
}

void dataNotifyOffline() {
  // Publish retained offline status saat WiFi masih ada tapi MQTT tidak terjangkau.
  // Dipanggil setelah 3x WiFi retry atau 3x MQTT retry gagal.
  if (WiFi.status() != WL_CONNECTED) return;
  if (topicStatus.length() == 0) return;
  if (mqtt.connected()) return; // Masih connected, tidak perlu notify
  Serial.Println("[MQTT] Mempublish status offline (retry gagal 3x)...");
  stopAndResetClient();
  yield();
  bool ok;
  const char* clientId = cfg.device_id.c_str();
  buildLwtPayload();
  if (mqttUser.length()) {
    ok = mqtt.connect(clientId, mqttUser.c_str(), mqttPass.c_str(),
                      topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
  } else {
    ok = mqtt.connect(clientId, nullptr, nullptr,
                      topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
  }
  if (ok) {
    publishStatus("offline");
    delay(100);
    stopAndResetClient();
    Serial.Println("[MQTT] \u2713 Status offline berhasil dipublish");
  } else {
    Serial.Println("[MQTT] \u2717 Offline notify gagal - broker tidak terjangkau");
  }
}

void dataResetConnection() {
  // Reset SSL client dan disconnect MQTT untuk memaksa fresh connection
  Serial.Println("[MQTT] Resetting connection (WiFi reconnected)...");
  
  stopAndResetClient();
  delay(200); // Beri waktu untuk cleanup
  yield();
  
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
  stopAndResetClient();
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
                      topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
  } else {
    ok = mqtt.connect(clientId, nullptr, nullptr,
                      topicStatus.c_str(), 1, true, lwtPayload.c_str(), false);
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
    
    esp_task_wdt_reset();
    yield();
    
    // Subscribe to all command topics WITH RETRY
    // Sama dengan ensureMqttConnected() — subscribe gagal → force disconnect → retry bersih
    int subscribeFailCount = 0;
    auto trySubscribe = [&](const char* topic, uint8_t qos) -> bool {
      for (int attempt = 0; attempt < 3; attempt++) {
        if (mqtt.subscribe(topic, qos)) return true;
        mqtt.loop();
        delay(100);
        esp_task_wdt_reset();
      }
      subscribeFailCount++;
      Serial.Printf("[MQTT] ✗ Subscribe failed after 3 retries: %s\n", topic);
      return false;
    };

    trySubscribe(cmdSetting.c_str(), 1);
    trySubscribe(cmdKoneksi.c_str(), 0);
    trySubscribe(cmdSchedule.c_str(), 1);
    trySubscribe(cmdRemote.c_str(), 0);
    trySubscribe(cmdReboot.c_str(), 0);
    trySubscribe(cmdInfo.c_str(), 0);
    if (topicCmdUpdate.length() > 0) {
      trySubscribe(topicCmdUpdate.c_str(), 0);
    }

    if (subscribeFailCount > 0) {
      Serial.Printf("[MQTT] ⚠ %d subscribe(s) gagal — force disconnect untuk reconnect bersih\n", subscribeFailCount);
      stopAndResetClient();
      lastMqttRetryMs = 0;
      return;
    }

    Serial.Println("[MQTT] ✓ All command topics subscribed successfully");
    
    esp_task_wdt_reset();
    yield();
    
    // Update retry timer setelah berhasil
    lastMqttRetryMs = millis();

    // Publish info + setting setelah cooldown (ditunda ke dataLoop via mqttInitialized flag)
    // dataPublishInfo() dipindah ke sini tapi dijaga cooldown agar tidak overflow SSL buffer
    if (dataIsPublishReady()) {
      dataPublishInfo();
    }
    
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
  StaticJsonDocument<3072> doc;
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
  RtcBatteryReading rtcBattery = readRtcBattery();
  doc["rtc_battery_adc"] = rtcBattery.raw;
  doc["rtc_battery_adc_mv"] = rtcBattery.adcMv;
  doc["rtc_battery_voltage"] = serialized(String(rtcBattery.voltage, 2));
  doc["rtc_battery_percent"] = rtcBattery.percent;
  doc["set_temp"] = setTempValid ? (int)sensorGetSetTemp() : 0;
  bool outdoorEvapValid = sensorHasValidEvapTemp();
  float outdoorEvapTemp = sensorGetEvapTemp();
  if (read_ok) {
    doc["power"] = d.power;
    doc["mode"] = d.mode;
    doc["fan_speed"] = d.fan;
    doc["room_temp"] = (int)d.current_temp;
    if (outdoorEvapValid) {
      doc["evap_temp"] = serialized(String(outdoorEvapTemp, 2));
    } else {
      doc["evap_temp"] = nullptr;
    }
    doc["frame_hex"] = d.frameHex;
    doc["outdoor_frame_hex"] = sensorGetLastOutdoorFrameHex();
    doc["error_hex"] = buildPublishedErrorHex(d);
    doc["error_code"] = d.errors;
  } else {
    doc["power"] = false;
    doc["mode"] = "-";
    doc["fan_speed"] = "-";
    doc["room_temp"] = 0;
    if (outdoorEvapValid) {
      doc["evap_temp"] = serialized(String(outdoorEvapTemp, 2));
    } else {
      doc["evap_temp"] = nullptr;
    }
    doc["frame_hex"] = "";
    doc["outdoor_frame_hex"] = sensorGetLastOutdoorFrameHex();
    doc["error_hex"] = "0000";
    doc["error_code"] = "-";
  }

  unsigned long mainboardByteCount = sensorGetMainboardByteCount();
  unsigned long displayByteCount = sensorGetDisplayByteCount();
  unsigned long mainboardUartAge = sensorTimeSinceLastMainboardByte();
  unsigned long displayUartAge = sensorTimeSinceLastDisplayByte();
  unsigned long mainboardFrameAge = sensorTimeSinceLastFrame();
  unsigned long displayFrameAge = sensorTimeSinceLastDisplayFrame();
  unsigned long outdoorByteCount = sensorGetOutdoorByteCount();
  unsigned long outdoorUartAge = sensorTimeSinceLastOutdoorByte();
  unsigned long outdoorFrameAge = sensorTimeSinceLastOutdoorFrame();

  doc["mainboard_uart_seen"] = mainboardByteCount > 0;
  doc["mainboard_uart_bytes"] = mainboardByteCount;
  doc["mainboard_uart_batch_len"] = sensorGetMainboardUartBatchLen();
  {
    String sampleHex = sensorGetMainboardUartSampleHex();
    doc["mainboard_uart_sample_hex"] = sampleHex.length() ? sampleHex : "-";
  }
  if (mainboardByteCount > 0) doc["mainboard_uart_age_ms"] = mainboardUartAge;
  else doc["mainboard_uart_age_ms"] = nullptr;
  if (sensorHasValidFrame()) doc["mainboard_frame_age_ms"] = mainboardFrameAge;
  else doc["mainboard_frame_age_ms"] = nullptr;

  doc["display_uart_seen"] = displayByteCount > 0;
  doc["display_uart_bytes"] = displayByteCount;
  {
    String sampleHex = sensorGetDisplayUartSampleHex();
    doc["display_uart_sample_hex"] = sampleHex.length() ? sampleHex : "-";
  }
  if (displayByteCount > 0) doc["display_uart_age_ms"] = displayUartAge;
  else doc["display_uart_age_ms"] = nullptr;
  if (sensorHasValidDisplayFrame()) doc["display_frame_age_ms"] = displayFrameAge;
  else doc["display_frame_age_ms"] = nullptr;

  doc["outdoor_uart_seen"] = outdoorByteCount > 0;
  doc["outdoor_uart_bytes"] = outdoorByteCount;
  doc["outdoor_uart_batch_len"] = sensorGetOutdoorUartBatchLen();
  {
    String outdoorSampleHex = sensorGetOutdoorUartSampleHex();
    doc["outdoor_uart_sample_hex"] = outdoorSampleHex.length() ? outdoorSampleHex : "-";
  }
  if (outdoorByteCount > 0) doc["outdoor_uart_age_ms"] = outdoorUartAge;
  else doc["outdoor_uart_age_ms"] = nullptr;
  if (sensorHasValidEvapTemp()) doc["outdoor_frame_age_ms"] = outdoorFrameAge;
  else doc["outdoor_frame_age_ms"] = nullptr;

  doc["uart_error_count"] = uartErrorCount;
  
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
    storeOfflineTelemetry(d, read_ok, "mqtt disconnected or topic empty");
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
    storeOfflineTelemetry(d, read_ok, "live publish failed");
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
      pendingSettingPublish = true;
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

  StaticJsonDocument<768> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    dataPublishAck("update", "failed", "Invalid JSON");
    return;
  }

  String action = doc["action"] | "manifest";
  action.toLowerCase();

  if (action == "rollback") {
    if (!(doc["confirm"] | false)) {
      dataPublishAck("update", "failed", "Rollback requires confirm=true");
      return;
    }

    dataPublishAck("update", "processing", "Selecting previous OTA firmware...");
    String rollbackDetail;
    if (!otaRollbackToPrevious(rollbackDetail)) {
      dataPublishAck("update", "failed", rollbackDetail);
      Serial.Println("[CMD Update] ========== END (ROLLBACK FAILED) ==========");
      return;
    }

    dataPublishAck("update", "success", rollbackDetail + "; rebooting");
    mqtt.loop();
    delay(500);
    ESP.restart();
    return;
  }

  if (action != "manifest" && action != "direct") {
    dataPublishAck("update", "failed", "Unknown action; use manifest, direct, or rollback");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    dataPublishAck("update", "failed", "WiFi not connected");
    return;
  }
  if (WiFi.RSSI() < -85) {
    dataPublishAck("update", "failed", "Signal weak (< -85dBm)");
    return;
  }

  String url;
  String expectedMD5;
  if (action == "direct") {
    if (doc.containsKey("binary_url")) url = doc["binary_url"].as<String>();
    else if (doc.containsKey("download_url")) url = doc["download_url"].as<String>();
    else if (doc.containsKey("url")) url = doc["url"].as<String>();
    expectedMD5 = doc["md5"] | "";
    expectedMD5.toLowerCase();

    bool md5Valid = expectedMD5.length() == 32;
    for (size_t i = 0; md5Valid && i < expectedMD5.length(); ++i) {
      const char c = expectedMD5[i];
      md5Valid = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    }
    if (!md5Valid) {
      dataPublishAck("update", "failed", "Direct OTA requires a 32-character hexadecimal md5");
      return;
    }
  } else if (doc.containsKey("manifest_url")) url = doc["manifest_url"].as<String>();
  else if (doc.containsKey("url")) url = doc["url"].as<String>();

  if (!url.startsWith("http://") && !url.startsWith("https://")) {
    dataPublishAck("update", "failed", "OTA URL must use HTTP or HTTPS");
    return;
  }

  dataPublishAck("update", "processing", "Request received, starting engine...");
  publishStatus("updating");
  dataPublishAck("update", "processing", "Pausing MQTT to free TLS heap...");
  mqtt.loop();
  delay(150);
  dataPauseMqttForOta();

  String otaDetail;
  bool otaOk = action == "direct"
      ? otaDownloadAndFlash(url, expectedMD5, otaDetail)
      : otaProcessManifest(url, otaDetail);

  dataResumeMqttAfterOta();
  publishStatus("online");
  if (!otaDetail.isEmpty()) dataPublishAck("update", otaOk ? "success" : "failed", otaDetail);

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
  KoneksiCfg previousKoneksi = cfg.koneksi;
  KoneksiCfg nextKoneksi = cfg.koneksi;
  
  esp_task_wdt_reset();
  yield();
  
  // Check WiFi changes
  if (doc.containsKey("wifi_ssid")) {
    String newSsid = doc["wifi_ssid"].as<String>();
    Serial.Printf("[CMD Koneksi] wifi_ssid: '%s' -> '%s'\n", 
                  cfg.koneksi.wifi_ssid.c_str(), newSsid.c_str());
    if (newSsid.length() > 0 && newSsid != nextKoneksi.wifi_ssid) {
      nextKoneksi.wifi_ssid = newSsid;
      wifiChanged = true;
    }
  }
  
  if (doc.containsKey("wifi_pass")) {
    String newPass = doc["wifi_pass"].as<String>();
    Serial.Println("[CMD Koneksi] wifi_pass: updated");
    if (newPass.length() > 0 && newPass != nextKoneksi.wifi_pass) {
      nextKoneksi.wifi_pass = newPass;
      wifiChanged = true;
    }
  }
  
  // Check MQTT changes
  if (doc.containsKey("mqtt_host")) {
    String newHost = doc["mqtt_host"].as<String>();
    Serial.Printf("[CMD Koneksi] mqtt_host: '%s' -> '%s'\n", 
                  cfg.koneksi.mqtt_host.c_str(), newHost.c_str());
    if (newHost.length() > 0 && newHost != nextKoneksi.mqtt_host) {
      nextKoneksi.mqtt_host = newHost;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_port")) {
    uint16_t newPort = doc["mqtt_port"].as<uint16_t>();
    Serial.Printf("[CMD Koneksi] mqtt_port: %d -> %d\n", 
                  cfg.koneksi.mqtt_port, newPort);
    if (newPort > 0 && newPort != nextKoneksi.mqtt_port) {
      nextKoneksi.mqtt_port = newPort;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_user")) {
    String newUser = doc["mqtt_user"].as<String>();
    Serial.Printf("[CMD Koneksi] mqtt_user: '%s' -> '%s'\n", 
                  cfg.koneksi.mqtt_user.c_str(), newUser.c_str());
    if (newUser.length() > 0 && newUser != nextKoneksi.mqtt_user) {
      nextKoneksi.mqtt_user = newUser;
      mqttChanged = true;
    }
  }
  
  if (doc.containsKey("mqtt_pass")) {
    String newPass = doc["mqtt_pass"].as<String>();
    Serial.Println("[CMD Koneksi] mqtt_pass: updated");
    if (newPass.length() > 0 && newPass != nextKoneksi.mqtt_pass) {
      nextKoneksi.mqtt_pass = newPass;
      mqttChanged = true;
    }
  }

  // Check timezone changes
  bool tzChanged = false;
  if (doc.containsKey("tz_offset")) {
    int8_t newTz = doc["tz_offset"].as<int8_t>();
    Serial.Printf("[CMD Koneksi] tz_offset: %d -> %d\n", cfg.koneksi.tz_offset, newTz);
    if (newTz >= -12 && newTz <= 14 && newTz != nextKoneksi.tz_offset) {
      nextKoneksi.tz_offset = newTz;
      tzChanged = true;
    }
  }

  // Check if any changes detected
  if (!wifiChanged && !mqttChanged && !tzChanged) {
    Serial.Println("[CMD Koneksi] No changes detected");
    dataPublishAck("koneksi", "success", "No changes detected");
    Serial.Println("[CMD Koneksi] ========== END (No Change) ==========");
    return;
  }
  
  yield();
  esp_task_wdt_reset();

  if (wifiChanged) {
    Serial.Printf("[CMD Koneksi] WiFi credential trial started for SSID: %s\n",
                  nextKoneksi.wifi_ssid.c_str());
    beginWiFiCredentialTrial(previousKoneksi, nextKoneksi);
    Serial.Println("[CMD Koneksi] ========== WIFI TRIAL ==========");
    return;
  }

  cfg.koneksi = nextKoneksi;
  
  // Save configuration
  Serial.Println("[CMD Koneksi] Changes detected, saving config...");
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

  cfgApply();

  if (mqttChanged) {
    Serial.Println("[CMD Koneksi] Only MQTT changed, reconnecting broker...");
    
    dataPublishAck("koneksi", "success", "MQTT settings updated, reconnecting...");
    delay(200);
    
    // Reset MQTT connection
    dataResetConnection();
    lastMqttRetryMs = 0; // Trigger immediate MQTT reconnect
    
    Serial.Println("[CMD Koneksi] MQTT ready for reconnect");
  } else {
    dataPublishAck("koneksi", "success", "Connection settings updated");
  }
  
  Serial.Println("[CMD Koneksi] ========== SUCCESS! ==========");
}

// Handler: Update Schedule (jadwal harian)
static void handleCmdSchedule(const char* payload, unsigned int length) {
  Serial.Println("[CMD Schedule] ========== START ==========");

  if (!payload || length == 0) {
    dataPublishAck("schedule", "failed", "Empty payload");
    return;
  }
  if (length > 2048) {
    dataPublishAck("schedule", "failed", "Payload too large; maximum is 2048 bytes");
    return;
  }

  const size_t jsonCapacity = static_cast<size_t>(length) * 2U + 1024U;
  const size_t largestBlock =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (largestBlock < jsonCapacity) {
    String detail = "Insufficient parse memory; need=" + String(jsonCapacity) +
                    " largest=" + String(largestBlock);
    dataPublishAck("schedule", "failed", detail);
    return;
  }

  DynamicJsonDocument doc(jsonCapacity);
  const DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    dataPublishAck("schedule",
                   "failed",
                   err == DeserializationError::NoMemory
                       ? "JSON document too large"
                       : "Invalid JSON format");
    return;
  }

  JsonArray daily = doc.as<JsonArray>();
  if (daily.isNull() || daily.size() == 0) {
    dataPublishAck("schedule", "failed", "Payload must be a non-empty root array");
    return;
  }

  static DaySchedule original[7];
  static DaySchedule staging[7];
  int defaultSetTemp = 24;
  int defaultMinTemp = 20;
  int defaultMaxTemp = 26;
  char defaultMode[6] = {};
  char defaultFan[8] = {};

  bool locked = false;
  if (g_cfgMutex) {
    if (CFG_LOCK() != pdTRUE) {
      dataPublishAck("schedule", "failed", "Configuration busy; retry command");
      return;
    }
    locked = true;
  }
  memcpy(original, cfg.setting.daily, sizeof(original));
  memcpy(staging, cfg.setting.daily, sizeof(staging));
  defaultSetTemp = cfg.setting.global_set_temp;
  defaultMinTemp = cfg.setting.global_min_temp;
  defaultMaxTemp = cfg.setting.global_max_temp;
  strncpy(defaultMode, cfg.setting.global_mode.c_str(), sizeof(defaultMode) - 1);
  strncpy(defaultFan, cfg.setting.global_fan.c_str(), sizeof(defaultFan) - 1);
  if (locked) CFG_UNLOCK();

  auto copyUpper = [](char* destination, size_t destinationSize, const char* source) {
    memset(destination, 0, destinationSize);
    if (!source) return;
    strncpy(destination, source, destinationSize - 1);
    for (size_t i = 0; destination[i] != '\0'; ++i) {
      destination[i] = static_cast<char>(toupper(static_cast<unsigned char>(destination[i])));
    }
  };

  auto normalizeMode = [&](char* destination, const char* source) -> bool {
    copyUpper(destination, sizeof(((TimePeriod*)nullptr)->mode), source);
    if (strcmp(destination, "COLD") == 0) {
      copyUpper(destination, sizeof(((TimePeriod*)nullptr)->mode), "COOL");
    }
    return strcmp(destination, "COOL") == 0 || strcmp(destination, "DRY") == 0 ||
           strcmp(destination, "FAN") == 0 || strcmp(destination, "AUTO") == 0 ||
           strcmp(destination, "HEAT") == 0;
  };

  auto normalizeFan = [&](char* destination, const char* source) -> bool {
    copyUpper(destination, sizeof(((TimePeriod*)nullptr)->fan), source);
    if (strcmp(destination, "MIN") == 0) {
      copyUpper(destination, sizeof(((TimePeriod*)nullptr)->fan), "LOW");
    }
    if (strcmp(destination, "MED") == 0) {
      copyUpper(destination, sizeof(((TimePeriod*)nullptr)->fan), "MEDIUM");
    }
    if (strcmp(destination, "MAX") == 0) {
      copyUpper(destination, sizeof(((TimePeriod*)nullptr)->fan), "HIGH");
    }
    return strcmp(destination, "LOW") == 0 || strcmp(destination, "MEDIUM") == 0 ||
           strcmp(destination, "HIGH") == 0 || strcmp(destination, "AUTO") == 0 ||
           strcmp(destination, "QUIET") == 0 || strcmp(destination, "TURBO") == 0;
  };

  auto parsePeriod = [&](JsonObject object,
                         TimePeriod& period,
                         int day,
                         int index,
                         String& failure) -> bool {
    if (object.isNull()) {
      failure = "Period must be an object: day=" + String(day) + " index=" + String(index);
      return false;
    }

    const char* start = object["start"] | "";
    const char* end = object["end"] | "";
    if (scheduleParseMinute(start) < 0 || scheduleParseMinute(end) < 0) {
      failure = "Invalid HH:MM: day=" + String(day) + " index=" + String(index);
      return false;
    }

    int setTemp = defaultSetTemp;
    if (object.containsKey("set_temp")) {
      if (!object["set_temp"].is<int>()) {
        failure = "Invalid set_temp type: day=" + String(day) + " index=" + String(index);
        return false;
      }
      setTemp = object["set_temp"].as<int>();
    }
    if (setTemp < 16 || setTemp > 30) {
      failure = "Invalid set_temp 16..30: day=" + String(day) + " index=" + String(index);
      return false;
    }

    memset(&period, 0, sizeof(period));
    strncpy(period.start, start, sizeof(period.start) - 1);
    strncpy(period.end, end, sizeof(period.end) - 1);
    period.set_temp = static_cast<uint8_t>(setTemp);
    period.min_temp = static_cast<uint8_t>(defaultMinTemp);
    period.max_temp = static_cast<uint8_t>(defaultMaxTemp);

    const char* mode = object["mode"] | defaultMode;
    const char* fan = object["fan"] | defaultFan;
    if (!normalizeMode(period.mode, mode)) {
      failure = "Invalid mode: day=" + String(day) + " index=" + String(index);
      return false;
    }
    if (!normalizeFan(period.fan, fan)) {
      failure = "Invalid fan: day=" + String(day) + " index=" + String(index);
      return false;
    }
    return true;
  };

  auto resultDetail = [&](const ScheduleResult& result) -> String {
    switch (result.code) {
      case SCHEDULE_RESULT_INVALID_TIME:
        return "Invalid schedule time: day=" + String(result.day) +
               " period=" + String(result.period);
      case SCHEDULE_RESULT_INVALID_DAY_TYPE:
        return "Invalid schedule day type: day=" + String(result.day);
      case SCHEDULE_RESULT_TOO_MANY_PERIODS:
        return "Too many periods: day=" + String(result.day) +
               " maximum=" + String(MAX_PERIODS_PER_DAY);
      case SCHEDULE_RESULT_FULLTIME_CONFLICT:
        return "Cannot insert into fulltime schedule";
      case SCHEDULE_RESULT_OVERLAP: {
        if (result.day >= 0 && result.day <= 6 && result.conflictDay >= 0 &&
            result.conflictDay <= 6 && result.period >= 0 &&
            result.period < staging[result.day].periodCount && result.conflictPeriod >= 0 &&
            result.conflictPeriod < staging[result.conflictDay].periodCount) {
          const TimePeriod& period = staging[result.day].periods[result.period];
          const TimePeriod& conflict =
              staging[result.conflictDay].periods[result.conflictPeriod];
          return "Schedule overlap: day=" + String(result.day) + " " + period.start + "-" +
                 period.end + " conflicts with day=" + String(result.conflictDay) + " " +
                 conflict.start + "-" + conflict.end;
        }
        return "Schedule overlap: day=" + String(result.day) +
               " period=" + String(result.period) +
               " conflicts_day=" + String(result.conflictDay) +
               " conflicts_period=" + String(result.conflictPeriod);
      }
      default:
        return "Unknown schedule validation error";
    }
  };

  uint8_t seenDayMask = 0;
  uint8_t affectedDayMask = 0;
  uint16_t totalInserted = 0;
  bool hasInsert = false;
  bool hasReplace = false;

  for (JsonVariant variant : daily) {
    JsonObject dayObject = variant.as<JsonObject>();
    if (dayObject.isNull()) {
      dataPublishAck("schedule", "failed", "Every schedule entry must be an object");
      return;
    }
    if (!dayObject.containsKey("day") || !dayObject["day"].is<int>()) {
      dataPublishAck("schedule", "failed", "Day must be numeric 0..6");
      return;
    }

    const int numericDay = dayObject["day"].as<int>();
    if (numericDay < 0 || numericDay > 6) {
      dataPublishAck("schedule", "failed", "Invalid day; expected 0..6");
      return;
    }
    const uint8_t dayBit = static_cast<uint8_t>(1U << numericDay);
    if ((seenDayMask & dayBit) != 0) {
      dataPublishAck("schedule", "failed", "Duplicate day in one command: day=" + String(numericDay));
      return;
    }
    seenDayMask |= dayBit;

    const char* operation = dayObject["operation"] | "replace";
    const bool insertOperation = strcasecmp(operation, "insert") == 0;
    const bool replaceOperation = strcasecmp(operation, "replace") == 0;
    if (!insertOperation && !replaceOperation) {
      dataPublishAck("schedule", "failed", "Invalid operation; expected replace or insert");
      return;
    }

    const char* type = dayObject["type"] | "";
    if (!type || type[0] == '\0') {
      dataPublishAck("schedule", "failed", "Missing schedule type: day=" + String(numericDay));
      return;
    }

    if (insertOperation) {
      hasInsert = true;
      if (strcasecmp(type, "parttime") != 0 && strcasecmp(type, "part_time") != 0) {
        dataPublishAck("schedule", "failed", "Insert requires parttime type");
        return;
      }

      JsonArray periodsJson = dayObject["data"].as<JsonArray>();
      if (periodsJson.isNull() || periodsJson.size() == 0 ||
          periodsJson.size() > MAX_PERIODS_PER_DAY) {
        dataPublishAck("schedule",
                       "failed",
                       "Insert data must contain 1..10 periods: day=" + String(numericDay));
        return;
      }

      TimePeriod incoming[MAX_PERIODS_PER_DAY] = {};
      uint8_t incomingCount = 0;
      String failure;
      for (JsonVariant periodVariant : periodsJson) {
        if (!parsePeriod(periodVariant.as<JsonObject>(),
                         incoming[incomingCount],
                         numericDay,
                         incomingCount,
                         failure)) {
          dataPublishAck("schedule", "failed", failure);
          return;
        }
        ++incomingCount;
      }

      uint8_t added = 0;
      ScheduleResult insertResult =
          scheduleInsertPeriods(staging[numericDay], incoming, incomingCount, &added);
      if (insertResult.code != SCHEDULE_RESULT_OK) {
        insertResult.day = numericDay;
        if (insertResult.conflictDay < 0) insertResult.conflictDay = numericDay;
        if (insertResult.code == SCHEDULE_RESULT_OVERLAP && insertResult.period >= 0 &&
            insertResult.period < incomingCount) {
          const TimePeriod& rejected = incoming[insertResult.period];
          dataPublishAck("schedule",
                         "failed",
                         "Schedule overlap: day=" + String(numericDay) + " " +
                             rejected.start + "-" + rejected.end +
                             " conflicts with an existing or inserted period");
        } else {
          dataPublishAck("schedule", "failed", resultDetail(insertResult));
        }
        return;
      }
      totalInserted += added;
      affectedDayMask |= dayBit;
      continue;
    }

    DaySchedule replacement = {};
    hasReplace = true;
    if (strcasecmp(type, "offtime") == 0 || strcasecmp(type, "libur") == 0) {
      replacement.type = DAY_OFFTIME;
    } else if (strcasecmp(type, "fulltime") == 0 ||
               strcasecmp(type, "full_time") == 0) {
      replacement.type = DAY_FULLTIME;
      replacement.periodCount = 1;
      strncpy(replacement.periods[0].start, "00:00", sizeof(replacement.periods[0].start) - 1);
      strncpy(replacement.periods[0].end, "23:59", sizeof(replacement.periods[0].end) - 1);
      replacement.periods[0].set_temp = static_cast<uint8_t>(defaultSetTemp);
      replacement.periods[0].min_temp = static_cast<uint8_t>(defaultMinTemp);
      replacement.periods[0].max_temp = static_cast<uint8_t>(defaultMaxTemp);
      if (!normalizeMode(replacement.periods[0].mode, defaultMode) ||
          !normalizeFan(replacement.periods[0].fan, defaultFan)) {
        dataPublishAck("schedule", "failed", "Invalid global mode or fan for fulltime");
        return;
      }
    } else if (strcasecmp(type, "parttime") == 0 ||
               strcasecmp(type, "part_time") == 0) {
      JsonArray periodsJson = dayObject["data"].as<JsonArray>();
      if (periodsJson.isNull() || periodsJson.size() == 0 ||
          periodsJson.size() > MAX_PERIODS_PER_DAY) {
        dataPublishAck("schedule",
                       "failed",
                       "Parttime data must contain 1..10 periods: day=" + String(numericDay));
        return;
      }

      replacement.type = DAY_PARTTIME;
      String failure;
      for (JsonVariant periodVariant : periodsJson) {
        if (!parsePeriod(periodVariant.as<JsonObject>(),
                         replacement.periods[replacement.periodCount],
                         numericDay,
                         replacement.periodCount,
                         failure)) {
          dataPublishAck("schedule", "failed", failure);
          return;
        }
        ++replacement.periodCount;
      }
      scheduleSortPeriods(replacement);
    } else {
      dataPublishAck("schedule", "failed", "Invalid schedule type: day=" + String(numericDay));
      return;
    }

    staging[numericDay] = replacement;
    affectedDayMask |= dayBit;
  }

  ScheduleResult validation = scheduleValidateWeek(staging);
  if (validation.code != SCHEDULE_RESULT_OK) {
    dataPublishAck("schedule", "failed", resultDetail(validation));
    return;
  }

  bool changed = false;
  for (uint8_t day = 0; day < 7; ++day) {
    if ((affectedDayMask & (1U << day)) != 0 &&
        memcmp(&original[day], &staging[day], sizeof(DaySchedule)) != 0) {
      changed = true;
      break;
    }
  }
  if (!changed) {
    const String detail = hasInsert
                              ? "Insert already applied; no duplicate created"
                              : "Schedule already matches requested state";
    dataPublishAck("schedule", "success", detail);
    return;
  }

  locked = false;
  if (g_cfgMutex) {
    if (CFG_LOCK() != pdTRUE) {
      dataPublishAck("schedule", "failed", "Configuration busy; retry command");
      return;
    }
    locked = true;
  }

  memcpy(cfg.setting.daily, staging, sizeof(staging));
  if (!cfgSaveSchedule()) {
    memcpy(cfg.setting.daily, original, sizeof(original));
    if (locked) CFG_UNLOCK();
    dataPublishAck("schedule", "failed", "Storage write error; previous schedule retained");
    return;
  }
  if (locked) CFG_UNLOCK();

  forceScheduleCheck = true;
  if (g_taskAutomation) xTaskNotifyGive(g_taskAutomation);

  dataPublishSettingDays(affectedDayMask);

  if (mqtt.connected()) {
    dataPublishSetting();
  } else {
    pendingSettingPublish = true;
  }

  uint8_t affectedCount = 0;
  int singleDay = -1;
  for (uint8_t day = 0; day < 7; ++day) {
    if ((affectedDayMask & (1U << day)) == 0) continue;
    ++affectedCount;
    singleDay = day;
  }

  String detail;
  if (affectedCount == 1 && hasInsert && !hasReplace) {
    detail = "Inserted schedule day=" + String(singleDay) +
             " added=" + String(totalInserted) +
             " total=" + String(staging[singleDay].periodCount);
  } else if (affectedCount == 1 && hasReplace && !hasInsert) {
    detail = "Replaced schedule day=" + String(singleDay) +
             " total=" + String(staging[singleDay].periodCount);
  } else {
    detail = "Updated schedule days=" + String(affectedCount) +
             " inserted=" + String(totalInserted);
  }
  dataPublishAck("schedule", "success", detail);
  Serial.Println("[CMD Schedule] ========== SUCCESS ==========");
}

struct DeferredRemoteCheck {
  bool pending;
  unsigned long enqueueMs;
  BPFrame oldFrame;
  int oldSetTemp;
  uint8_t checkCount;
} g_deferredRemoteCheck = { false, 0, {}, -1, 0 };

static void serviceDeferredRemoteCheck() {
  if (!g_deferredRemoteCheck.pending) return;

  unsigned long elapsed = millis() - g_deferredRemoteCheck.enqueueMs;
  if (elapsed < 1500UL) return;

  g_deferredRemoteCheck.checkCount++;
  bool tracked = false;

  if (g_deferredRemoteCheck.oldSetTemp > 0 && sensorIsDisplayHealthy()) {
    uint8_t newSetTemp = sensorGetSetTemp();
    if (newSetTemp > 0 && newSetTemp != g_deferredRemoteCheck.oldSetTemp) {
      activityTrackTemp(g_deferredRemoteCheck.oldSetTemp, newSetTemp, SOURCE_MQTT_REMOTE);
      tracked = true;
    }
  }

  if (sensorIsHealthy() && g_deferredRemoteCheck.oldFrame.valid) {
    BPFrame newFrame = sensorGetLastFrame();
    bool stateChanged = (newFrame.power != g_deferredRemoteCheck.oldFrame.power) ||
                       (newFrame.mode != g_deferredRemoteCheck.oldFrame.mode) ||
                       (newFrame.fan != g_deferredRemoteCheck.oldFrame.fan);
    if (stateChanged) {
      activityTrackChange(g_deferredRemoteCheck.oldFrame, newFrame, SOURCE_MQTT_REMOTE);
      tracked = true;
    }
  }

  if (tracked || g_deferredRemoteCheck.checkCount >= 3) {
    g_deferredRemoteCheck.pending = false;
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

  auto rejectRemoteValue = [](const char* field, const char* reason) {
    Serial.Printf("[CMD Remote] REJECT: invalid %s (%s)\n", field, reason);
    String msg = "Invalid ";
    msg += field;
    msg += ": ";
    msg += reason;
    dataPublishAck("remote", "failed", msg);
    Serial.Println("[CMD Remote] ========== REJECTED ==========");
  };

  auto isValidBoolLike = [](JsonVariant v) -> bool {
    if (v.is<bool>()) return true;
    if (v.is<int>()) {
      int n = v.as<int>();
      return n == 0 || n == 1;
    }
    if (v.is<const char*>()) {
      String s = v.as<const char*>();
      s.toUpperCase();
      s.trim();
      return s == "ON" || s == "OFF" || s == "TRUE" || s == "FALSE" || s == "1" || s == "0";
    }
    return false;
  };

  auto isValidMode = [](JsonVariant v) -> bool {
    if (!v.is<const char*>()) return false;
    String s = v.as<const char*>();
    s.toUpperCase();
    s.trim();
    if (s == "COLD") s = "COOL";
    return s == "COOL" || s == "DRY" || s == "FAN" || s == "AUTO" || s == "HEAT";
  };

  auto isValidFan = [](JsonVariant v) -> bool {
    if (!v.is<const char*>()) return false;
    String s = v.as<const char*>();
    s.toUpperCase();
    s.trim();
    return s == "LOW" || s == "MIN" || s == "MED" || s == "MEDIUM" ||
           s == "HIGH" || s == "MAX" || s == "AUTO" || s == "QUIET" || s == "TURBO";
  };

  if (doc.containsKey("power") && !isValidBoolLike(doc["power"])) {
    rejectRemoteValue("power", "expected boolean, 0/1, ON/OFF, or TRUE/FALSE");
    return;
  }

  if (doc.containsKey("mode") && !isValidMode(doc["mode"])) {
    rejectRemoteValue("mode", "expected COOL/DRY/FAN/AUTO/HEAT");
    return;
  }

  if (doc.containsKey("fan") && !isValidFan(doc["fan"])) {
    rejectRemoteValue("fan", "expected LOW/MEDIUM/HIGH/AUTO/QUIET/TURBO");
    return;
  }

  if (doc.containsKey("temp")) {
    if (!doc["temp"].is<int>()) {
      rejectRemoteValue("temp", "expected number 16..30");
      return;
    }
    int t = doc["temp"].as<int>();
    if (t < 16 || t > 30) {
      rejectRemoteValue("temp", "expected number 16..30");
      return;
    }
  }

  if (doc.containsKey("swing")) {
    JsonVariant sv = doc["swing"];
    if (sv.is<const char*>()) {
      String s = String(sv.as<const char*>());
      s.toUpperCase();
      s.trim();
      if (s != "AUTO" && s != "ON" && s != "OFF" &&
          s != "HIGHEST" && s != "UPPER_MIDDLE" && s != "MIDDLE" &&
          s != "LOWER_MIDDLE" && s != "LOWEST") {
        rejectRemoteValue("swing", "expected AUTO/ON/OFF/HIGHEST/UPPER_MIDDLE/MIDDLE/LOWER_MIDDLE/LOWEST");
        return;
      }
    } else if (!sv.is<bool>() && !sv.is<JsonObject>()) {
      rejectRemoteValue("swing", "expected bool, string, or object {vertical, horizontal}");
      return;
    }
  }
  
  esp_task_wdt_reset();
  yield();

  auto readBoolLike = [](JsonVariant v) -> bool {
    if (v.is<bool>()) return v.as<bool>();
    if (v.is<int>()) return v.as<int>() != 0;
    if (v.is<const char*>()) {
      String s = v.as<const char*>();
      s.toUpperCase();
      s.trim();
      return s == "ON" || s == "TRUE" || s == "1";
    }
    return false;
  };
  
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
  bool expectedPower = doc.containsKey("power") ? readBoolLike(doc["power"]) : (baselineFrame.valid ? baselineFrame.power : true);
  String expectedMode = doc.containsKey("mode") ? doc["mode"].as<String>() : (baselineFrame.valid ? baselineFrame.mode : cfg.setting.global_mode);
  String expectedFan = doc.containsKey("fan") ? doc["fan"].as<String>() : (baselineFrame.valid ? baselineFrame.fan : cfg.setting.global_fan);
  uint8_t expectedTemp = doc.containsKey("temp") ? doc["temp"].as<uint8_t>() : cfg.setting.global_set_temp;
  expectedMode.toUpperCase();
  expectedFan.toUpperCase();
  if (doc.containsKey("power") && !expectedPower) {
    expectedMode = "";
    expectedFan = "";
    expectedTemp = 0;
  }
  
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

  // Enqueue IR command (non-blocking; TaskControl handles transmission).
  bool enqueued = controllEnqueueCommand(cmdStr.c_str(), cmdStr.length());
  if (!enqueued) {
    Serial.println("[CMD Remote] IR enqueue failed");
    automationClearExpectedStateFromMqtt();
    dataPublishAck("remote", "failed", "IR queue full or unavailable");
    return;
  }

  g_deferredRemoteCheck.pending = true;
  g_deferredRemoteCheck.enqueueMs = millis();
  g_deferredRemoteCheck.oldFrame = oldFrame;
  g_deferredRemoteCheck.oldSetTemp = oldSetTemp;
  g_deferredRemoteCheck.checkCount = 0;

  // Response validation ACK is handled by ir_health.
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
    stopAndResetClient();
    delay(100);
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
