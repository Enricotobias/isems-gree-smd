#include "config.h"

#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <Wire.h>
#include "data.h"
#include "esp_task_wdt.h"
#include "esp_sntp.h"

RTC_PCF8563 rtc;

const char* CONFIG_PATH = "/config.json";

const char* DEF_WIFI_SSID = "Atop Lt2";
const char* DEF_WIFI_PASS = "Atop88ls";
const char* DEF_MQTT_HOST = "mqtt.nicourbanindonesia.com";
const uint16_t DEF_MQTT_PORT = 8883;  // Port 8883 untuk MQTT over SSL/TLS
const char* DEF_MQTT_USER = "isems";
const char* DEF_MQTT_PASS = "Atop88ls";

const bool   DEF_AUTOMATION = false;
const int    DEF_GLOBAL_SET_TEMP = 24;
const int    DEF_GLOBAL_MIN_TEMP = 20;
const int    DEF_GLOBAL_MAX_TEMP = 26;
const char*  DEF_GLOBAL_MODE     = "COOL";
const char*  DEF_GLOBAL_FAN      = "AUTO";
const int    DEF_TEMP_COOLDOWN   = 300;
const int    DEF_AUTO_RECOVERY   = 300;

static void feedWatchdogAndYield() {
  esp_task_wdt_reset();
  yield();
}

Config cfg;

static Preferences _wifiPrefs;

String nvs_getWiFiSSID() {
  _wifiPrefs.begin("wifi", true);
  String v = _wifiPrefs.getString("ssid", DEF_WIFI_SSID);
  _wifiPrefs.end();
  return v;
}

String nvs_getWiFiPass() {
  _wifiPrefs.begin("wifi", true);
  String v = _wifiPrefs.getString("pass", DEF_WIFI_PASS);
  _wifiPrefs.end();
  return v;
}

void nvs_setWiFiSSID(const String& ssid) {
  _wifiPrefs.begin("wifi", false);
  _wifiPrefs.putString("ssid", ssid);
  _wifiPrefs.end();
}

void nvs_setWiFiPass(const String& pass) {
  _wifiPrefs.begin("wifi", false);
  _wifiPrefs.putString("pass", pass);
  _wifiPrefs.end();
}

const char* PROD_ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

static void initDayOfftime(DaySchedule& d) {
  d.type = DAY_OFFTIME;
  d.periodCount = 0;
  memset(d.periods, 0, sizeof(d.periods));
}

static void initDayFulltime(DaySchedule& d,
                            uint8_t setTemp = DEF_GLOBAL_SET_TEMP,
                            uint8_t minTemp = DEF_GLOBAL_MIN_TEMP,
                            uint8_t maxTemp = DEF_GLOBAL_MAX_TEMP,
                            const char* mode = DEF_GLOBAL_MODE,
                            const char* fan = DEF_GLOBAL_FAN) {
  d.type = DAY_FULLTIME;
  d.periodCount = 1;
  memset(&d.periods[0], 0, sizeof(TimePeriod));
  strncpy(d.periods[0].mode, mode, sizeof(d.periods[0].mode) - 1);
  strncpy(d.periods[0].fan, fan, sizeof(d.periods[0].fan) - 1);
  d.periods[0].set_temp = setTemp;
  d.periods[0].min_temp = minTemp;
  d.periods[0].max_temp = maxTemp;
}

static void initDayParttime(DaySchedule& d,
                            const char* start, const char* end,
                            uint8_t setTemp = DEF_GLOBAL_SET_TEMP,
                            uint8_t minTemp = DEF_GLOBAL_MIN_TEMP,
                            uint8_t maxTemp = DEF_GLOBAL_MAX_TEMP,
                            const char* mode = DEF_GLOBAL_MODE,
                            const char* fan = DEF_GLOBAL_FAN) {
  d.type = DAY_PARTTIME;
  d.periodCount = 1;
  memset(&d.periods[0], 0, sizeof(TimePeriod));
  strncpy(d.periods[0].start, start, sizeof(d.periods[0].start) - 1);
  strncpy(d.periods[0].end, end, sizeof(d.periods[0].end) - 1);
  strncpy(d.periods[0].mode, mode, sizeof(d.periods[0].mode) - 1);
  strncpy(d.periods[0].fan, fan, sizeof(d.periods[0].fan) - 1);
  d.periods[0].set_temp = setTemp;
  d.periods[0].min_temp = minTemp;
  d.periods[0].max_temp = maxTemp;
}

static void parsePeriodFromJson(TimePeriod& p, JsonObject pObj, bool isFulltime = false) {
  memset(&p, 0, sizeof(TimePeriod));
  if (!isFulltime) {
    strncpy(p.start, pObj["start"] | "08:00", sizeof(p.start) - 1);
    strncpy(p.end, pObj["end"] | "17:00", sizeof(p.end) - 1);
  }
  strncpy(p.mode, pObj["mode"] | "", sizeof(p.mode) - 1);
  strncpy(p.fan, pObj["fan"] | "", sizeof(p.fan) - 1);
  for (int i = 0; p.mode[i]; i++) p.mode[i] = toupper(p.mode[i]);
  for (int i = 0; p.fan[i]; i++) p.fan[i] = toupper(p.fan[i]);
  p.set_temp = (uint8_t)(pObj["set_temp"] | 0);
  p.min_temp = (uint8_t)(pObj["min_temp"] | 0);
  p.max_temp = (uint8_t)(pObj["max_temp"] | 0);
}

// Load default configuration into memory
static void loadDefaultConfig() {
  cfg.ac_brand = AC_BRAND;
  cfg.setting.automationPausedByUser = false;
  
  cfg.koneksi.wifi_ssid = nvs_getWiFiSSID();
  cfg.koneksi.wifi_pass = nvs_getWiFiPass();
  cfg.koneksi.mqtt_host = DEF_MQTT_HOST;
  cfg.koneksi.mqtt_port = DEF_MQTT_PORT;
  cfg.koneksi.mqtt_user = DEF_MQTT_USER;
  cfg.koneksi.mqtt_pass = DEF_MQTT_PASS;

  cfg.setting.automation = DEF_AUTOMATION;
  cfg.setting.global_set_temp = DEF_GLOBAL_SET_TEMP;
  cfg.setting.global_min_temp = DEF_GLOBAL_MIN_TEMP;
  cfg.setting.global_max_temp = DEF_GLOBAL_MAX_TEMP;
  cfg.setting.global_mode = DEF_GLOBAL_MODE;
  cfg.setting.global_fan = DEF_GLOBAL_FAN;
  cfg.setting.temp_cooldown_sec = DEF_TEMP_COOLDOWN;
  cfg.setting.auto_recovery_sec = 300; // 5 Menit (Standar aman kompresor)

  for (int i = 0; i < 5; i++) {
    initDayParttime(cfg.setting.daily[i], "07:55", "17:00");
  }
  initDayParttime(cfg.setting.daily[5], "07:55", "13:00");
  initDayOfftime(cfg.setting.daily[6]);

  cfg.setting.manualModeActive = false;
  cfg.setting.lastManualControlMs = 0;
}

bool cfgSave() {
  // Mount Check
  if (!LittleFS.begin()) {
    Serial.Println("[Config] WARNING: LittleFS not mounted, trying to mount...");
    if (!LittleFS.begin(true)) {
      Serial.Println("[Config] ERROR: LittleFS mount failed completely");
      return false;
    }
  }

  // === DIAGNOSTIK PARTISI ===
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  Serial.Printf("[Config] Info: Total Space: %d bytes, Used: %d bytes\n", totalBytes, usedBytes);

  if (totalBytes == 0) {
    Serial.Println("[Config] FATAL ERROR: Partition size is 0! Check 'Tools > Partition Scheme' in Arduino IDE.");
    return false;
  }
  // ==========================

  StaticJsonDocument<6144> doc;

  // Immutable metadata: always write from firmware constant
  cfg.ac_brand = AC_BRAND;
  doc["ac_brand"] = cfg.ac_brand;

  JsonObject koneksi = doc.createNestedObject("koneksi");
  koneksi["mqtt_host"] = cfg.koneksi.mqtt_host;
  koneksi["mqtt_port"] = cfg.koneksi.mqtt_port;
  koneksi["mqtt_user"] = cfg.koneksi.mqtt_user;
  koneksi["mqtt_pass"] = cfg.koneksi.mqtt_pass;

  JsonObject setting = doc.createNestedObject("setting");
  setting["automation"] = cfg.setting.automation;
  
  setting["automationPausedByUser"] = cfg.setting.automationPausedByUser;
  setting["manualModeActive"] = cfg.setting.manualModeActive;
  setting["lastManualControlMs"] = (unsigned long)cfg.setting.lastManualControlMs;
  
  JsonArray schedule = setting.createNestedArray("schedule");
  for (int i = 0; i < 7; ++i) {
    JsonObject dayObj = schedule.createNestedObject();
    dayObj["day"] = DAY_IDX_TO_NUM(i);
    DaySchedule& d = cfg.setting.daily[i];
    JsonArray dataArr = dayObj.createNestedArray("data");
    if (d.type == DAY_OFFTIME) {
      dayObj["type"] = "offtime";
      continue;
    }
    dayObj["type"] = (d.type == DAY_FULLTIME) ? "fulltime" : "parttime";
    uint8_t cnt = d.periodCount;
    if (cnt > MAX_PERIODS_PER_DAY) cnt = MAX_PERIODS_PER_DAY;
    for (uint8_t p = 0; p < cnt; p++) {
      JsonObject po = dataArr.createNestedObject();
      if (d.type == DAY_PARTTIME) {
        po["start"] = d.periods[p].start;
        po["end"] = d.periods[p].end;
      }
      po["mode"] = d.periods[p].mode;
      po["fan"] = d.periods[p].fan;
      po["set_temp"] = d.periods[p].set_temp;
      po["min_temp"] = d.periods[p].min_temp;
      po["max_temp"] = d.periods[p].max_temp;
    }
  }

  // Hapus file lama
  if (LittleFS.exists(CONFIG_PATH)) {
    LittleFS.remove(CONFIG_PATH);
  }

  File f = LittleFS.open(CONFIG_PATH, "w");
  
  // SELF-HEALING BLOCK
  if (!f) {
    Serial.Println("[Config] Error: File creation failed. Filesystem corrupt?");
    Serial.Println("[Config] ACTION: Formatting LittleFS...");
    
    LittleFS.end();
    // Format
    if (LittleFS.format()) {
      Serial.Println("[Config] Format success. Waiting to stabilize...");
      delay(100); // Beri jeda sedikit
      
      // Remount dengan formatOnFail = true
      if (LittleFS.begin(true)) {
        Serial.Println("[Config] Remounted. Retrying open...");
        // Coba buka file sekali lagi
        f = LittleFS.open(CONFIG_PATH, "w");
      } else {
        Serial.Println("[Config] Remount failed after format.");
      }
    } else {
      Serial.Println("[Config] Format failed.");
    }
  }

  if (!f) {
    Serial.Println("[Config] Error: Cannot open config for write (Persistent failure)");
    return false;
  }

  size_t bytesWritten = serializeJsonPretty(doc, f);
  f.close();
  
  if (bytesWritten == 0) {
    Serial.Println("[Config] Error: Failed to write config (0 bytes written)");
    return false;
  }

  Serial.Printf("[Config] Successfully saved %d bytes to %s\n", bytesWritten, CONFIG_PATH);
  return true;
} 

bool cfgLoad() {
  if (!LittleFS.begin()) {
    Serial.Println("WARNING: LittleFS not mounted, using default config");
    loadDefaultConfig();
    return false;
  }

  if (!LittleFS.exists(CONFIG_PATH)) {
    Serial.Println("WARNING: Config file not found, creating default config");
    loadDefaultConfig();
    return cfgSave();
  }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    Serial.Println("ERROR: Cannot open config for read");
    loadDefaultConfig();
    return false;
  }

  size_t size = f.size();
  
  if (size > 8192) {
    Serial.Println("ERROR: Config file too large (>8KB), possibly corrupt");
    f.close();
    loadDefaultConfig();
    return false;
  }
  
  std::unique_ptr<char[]> buf(new char[size + 1]);
  if (!buf) {
    Serial.Println("ERROR: Failed to allocate memory for config");
    f.close();
    loadDefaultConfig();
    return false;
  }
  
  f.readBytes(buf.get(), size);
  buf[size] = '\0';
  f.close();

  StaticJsonDocument<4096> doc;
  DeserializationError err = deserializeJson(doc, buf.get());
  if (err) {
    Serial.print("ERROR: Parse config error: ");
    Serial.Println(err.c_str());
    loadDefaultConfig();
    return false;
  }

  // ═══════════════════════════════════════════════════════════════════
  // [FIXED] Load config dari file - TIDAK overwrite dengan DEF_*
  // DEF_* hanya digunakan sebagai fallback jika key tidak ada di file
  // ═══════════════════════════════════════════════════════════════════
  
  // Immutable metadata: always forced to firmware constant
  cfg.ac_brand = AC_BRAND;

  JsonObject k = doc["koneksi"].as<JsonObject>();
  if (!k.isNull()) {
    cfg.koneksi.wifi_ssid = nvs_getWiFiSSID();
    cfg.koneksi.wifi_pass = nvs_getWiFiPass();
    cfg.koneksi.mqtt_host = k["mqtt_host"] | DEF_MQTT_HOST;
    cfg.koneksi.mqtt_port = k["mqtt_port"] | DEF_MQTT_PORT;
    cfg.koneksi.mqtt_user = k["mqtt_user"] | DEF_MQTT_USER;
    cfg.koneksi.mqtt_pass = k["mqtt_pass"] | DEF_MQTT_PASS;
  } else {
    cfg.koneksi.wifi_ssid = nvs_getWiFiSSID();
    cfg.koneksi.wifi_pass = nvs_getWiFiPass();
    cfg.koneksi.mqtt_host = DEF_MQTT_HOST;
    cfg.koneksi.mqtt_port = DEF_MQTT_PORT;
    cfg.koneksi.mqtt_user = DEF_MQTT_USER;
    cfg.koneksi.mqtt_pass = DEF_MQTT_PASS;
  }
  
  // Trim whitespace dari WiFi credentials
  cfg.koneksi.wifi_ssid.trim();
  cfg.koneksi.wifi_pass.trim();

  JsonObject s = doc["setting"].as<JsonObject>();
  if (!s.isNull()) {
    if (s.containsKey("automation")) {
      JsonVariant autoValue = s["automation"];
      if (autoValue.is<bool>()) {
        cfg.setting.automation = autoValue.as<bool>();
      } else if (autoValue.is<const char*>()) {
        const char* strVal = autoValue.as<const char*>();
        cfg.setting.automation = (strcasecmp(strVal, "true") == 0 || strcmp(strVal, "1") == 0);
      } else {
        cfg.setting.automation = autoValue.as<bool>();
      }
    } else {
      cfg.setting.automation = DEF_AUTOMATION;
    }
    
    
    // global_set_temp, global_min/max_temp, global_mode, global_fan,
    // temp_cooldown, auto_recovery — no longer persisted in config.
    // Hardcoded runtime defaults for automation.cpp fallback:
    cfg.setting.global_set_temp = DEF_GLOBAL_SET_TEMP;
    cfg.setting.global_min_temp = DEF_GLOBAL_MIN_TEMP;
    cfg.setting.global_max_temp = DEF_GLOBAL_MAX_TEMP;
    cfg.setting.global_mode = DEF_GLOBAL_MODE;
    cfg.setting.global_fan = DEF_GLOBAL_FAN;
    cfg.setting.temp_cooldown_sec = DEF_TEMP_COOLDOWN;
    cfg.setting.auto_recovery_sec = DEF_AUTO_RECOVERY;

    // Load auto-recovery runtime fields
    cfg.setting.manualModeActive = s["manualModeActive"] | false;
    cfg.setting.lastManualControlMs = s["lastManualControlMs"] | (unsigned long)0;
    cfg.setting.automationPausedByUser = s["automationPausedByUser"] | false;

    // Validate loaded values
    if (cfg.setting.lastManualControlMs == 0) {
      cfg.setting.lastManualControlMs = millis();
    }

    for (int i = 0; i < 7; ++i) {
      initDayOfftime(cfg.setting.daily[i]);
    }

    if (s.containsKey("schedule")) {
      JsonArray schedule = s["schedule"].as<JsonArray>();
      for (JsonObject dayObj : schedule) {
        if (!dayObj.containsKey("day") || !dayObj["day"].is<int>()) continue;
        int dayNum = dayObj["day"].as<int>();
        int idx = -1;
        if (dayNum >= 0 && dayNum <= 6) {
          // New standard: Senin=0..Minggu=6
          idx = DAY_NUM_TO_IDX(dayNum);
        } else if (dayNum >= 1 && dayNum <= 7) {
          // Legacy (old firmware): Senin=1..Minggu=7
          idx = dayNum - 1;
        } else {
          continue;
        }
        if (idx < 0 || idx > 6) continue;
        const char* type = dayObj["type"] | "offtime";

        if (strcasecmp(type, "offtime") == 0) {
          initDayOfftime(cfg.setting.daily[idx]);
          continue;
        }

        JsonArray data = dayObj["data"].as<JsonArray>();
        if (strcasecmp(type, "fulltime") == 0) {
          initDayFulltime(cfg.setting.daily[idx]);
          if (!data.isNull() && data.size() > 0) {
            parsePeriodFromJson(cfg.setting.daily[idx].periods[0], data[0].as<JsonObject>(), true);
          }
          continue;
        }

        initDayParttime(cfg.setting.daily[idx], "08:00", "17:00");
        cfg.setting.daily[idx].periodCount = 0;
        uint8_t count = 0;
        for (JsonObject pObj : data) {
          if (count >= MAX_PERIODS_PER_DAY) break;
          parsePeriodFromJson(cfg.setting.daily[idx].periods[count], pObj, false);
          count++;
        }
        cfg.setting.daily[idx].periodCount = count;
        if (cfg.setting.daily[idx].periodCount == 0) {
          initDayOfftime(cfg.setting.daily[idx]);
        }
      }
    }
  } else {
    cfg.setting.automation = DEF_AUTOMATION;
    cfg.setting.global_set_temp = DEF_GLOBAL_SET_TEMP;
    cfg.setting.global_min_temp = DEF_GLOBAL_MIN_TEMP;
    cfg.setting.global_max_temp = DEF_GLOBAL_MAX_TEMP;
    cfg.setting.global_mode = DEF_GLOBAL_MODE;
    cfg.setting.global_fan = DEF_GLOBAL_FAN;
    cfg.setting.temp_cooldown_sec = DEF_TEMP_COOLDOWN;
    cfg.setting.auto_recovery_sec = DEF_AUTO_RECOVERY;

    for (int i = 0; i < 5; i++) initDayParttime(cfg.setting.daily[i], "07:55", "17:00");
    initDayParttime(cfg.setting.daily[5], "07:55", "13:00");
    initDayOfftime(cfg.setting.daily[6]);
    cfg.setting.automationPausedByUser = false;
  }

  Serial.Printf("[Config] Loaded - automation: %s\n",
                cfg.setting.automation ? "true" : "false");

  return true;
}

void cfgApply() {
  dataBuildTopics(cfg.device_id);
  dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port,
                cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
}

// Helper function to get WiFi status description
static const char* getWiFiStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:     return "Idle";
    case WL_NO_SSID_AVAIL:   return "SSID not found";
    case WL_SCAN_COMPLETED:  return "Scan completed";
    case WL_CONNECTED:       return "Connected";
    case WL_CONNECT_FAILED:  return "Connection failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED:    return "Disconnected";
    default:                 return "Unknown";
  }
}

void cfgApplyWiFi() {
  Serial.Println("[Config] Applying WiFi changes");
  cfgApply();
  Serial.Println("[Config] WiFi will reconnect in main loop");
}

void cfgStartWiFiConnect() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    delay(100);
    feedWatchdogAndYield();
  }
  
  WiFi.mode(WIFI_OFF);
  delay(200);
  feedWatchdogAndYield();
  
  cfg.koneksi.wifi_ssid.trim();
  cfg.koneksi.wifi_pass.trim();
  
  Serial.Printf("WiFi: connecting to [%s]...\n", cfg.koneksi.wifi_ssid.c_str());
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  
  WiFi.begin(cfg.koneksi.wifi_ssid.c_str(), cfg.koneksi.wifi_pass.c_str());
  feedWatchdogAndYield();
}

// Blocking version with retry and timeout handling
bool cfgConnectWiFiBlocking() {
  const int MAX_RETRIES = 3;
  const int TIMEOUT_MS = 10000; // 10 seconds timeout per attempt
  
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    // Disconnect first if already connected
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect();
      delay(1000);
      feedWatchdogAndYield();
    }
    
    // Clear any previous WiFi config
    WiFi.mode(WIFI_OFF);
    delay(500);
    feedWatchdogAndYield();
    
    Serial.Printf("WiFi: Attempt %d/%d - connecting to %s ...\n", 
                  retry + 1, MAX_RETRIES, cfg.koneksi.wifi_ssid.c_str());
    
    // Set WiFi mode and begin connection
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);
    
    // Start connection
    WiFi.begin(cfg.koneksi.wifi_ssid.c_str(), cfg.koneksi.wifi_pass.c_str());
    
    // Wait for connection with timeout
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
      
      feedWatchdogAndYield();
      
      // Check for specific failure states
      if (WiFi.status() == WL_CONNECT_FAILED || 
          WiFi.status() == WL_NO_SSID_AVAIL ||
          WiFi.status() == WL_CONNECTION_LOST) {
        Serial.Printf("\nWiFi: Connection failed - %s\n", getWiFiStatusText(WiFi.status()));
        break;
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.Println();
      Serial.Printf("WiFi: Connected successfully!\n");
      Serial.Printf("WiFi: IP address: %s\n", WiFi.localIP().toString().c_str());
      Serial.Printf("WiFi: Signal strength: %d dBm\n", WiFi.RSSI());
      return true; // Success
    } else {
      Serial.Println();
      Serial.Printf("WiFi: Failed to connect - %s\n", getWiFiStatusText(WiFi.status()));
      
      if (retry < MAX_RETRIES - 1) {
        Serial.Printf("WiFi: Retrying in 2 seconds...\n");
        WiFi.disconnect();
        delay(2000);
        feedWatchdogAndYield();
      }
    }
  }
  
  // All retries failed
  Serial.Printf("WiFi: Failed to connect after %d attempts\n", MAX_RETRIES);
  Serial.Println("WiFi: Please check your WiFi credentials and network availability");
  return false;
}

bool cfgCheckRTC() {
  Wire.beginTransmission(0x51); // PCF8563T I2C address
  return (Wire.endTransmission() == 0);
}

bool cfgInitRTC() {
  #if defined(RTC_SDA_PIN) && defined(RTC_SCL_PIN)
  // I2C bus recovery: toggle SCL 9x untuk release slave yang hold SDA (bus stuck saat boot)
  pinMode(RTC_SCL_PIN, OUTPUT);
  pinMode(RTC_SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(RTC_SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(RTC_SCL_PIN, LOW);  delayMicroseconds(10);
  }
  // STOP condition
  pinMode(RTC_SDA_PIN, OUTPUT);
  digitalWrite(RTC_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(RTC_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(RTC_SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(RTC_SDA_PIN, INPUT);
  pinMode(RTC_SCL_PIN, INPUT);
  
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  #endif
  
  // Beri waktu I2C bus untuk stabil
  delay(200);
  esp_task_wdt_reset();
  yield();
  
  // Cukup probe kehadiran chip - validasi tahun ada di cfgSyncFromRTC()
  unsigned long startTime = millis();
  const unsigned long RTC_TIMEOUT = 5000;
  
  while (millis() - startTime < RTC_TIMEOUT) {
    esp_task_wdt_reset();
    yield();
    
    Wire.beginTransmission(0x51); // PCF8563T address
    if (Wire.endTransmission() == 0) {
      rtc.begin(); // inisialisasi library PCF8563T
      Serial.Println("[RTC] PCF8563T ditemukan di I2C 0x51");
      return true;
    }
    
    delay(200);
    yield();
    esp_task_wdt_reset();
  }
  
  Serial.Println("ERROR: RTC PCF8563T tidak ditemukan (timeout 5s)");
  return false;
}

bool cfgSyncFromRTC() {
  esp_task_wdt_reset();
  yield();
  
  if (!rtc.begin()) {
    Serial.Println("ERROR: Failed to access PCF8563T RTC");
    return false;
  }
  
  DateTime now = rtc.now();
  
  esp_task_wdt_reset();
  yield();
  
  int fullYear = now.year();
  
  if (fullYear < 2020 || fullYear > 2100) {
    Serial.Printf("WARNING: RTC time invalid (year=%d)\n", fullYear);
    return false;
  }
  
  // Set timezone untuk WIB (GMT+7) terlebih dahulu
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  
  // Buat struktur waktu dari RTC (anggap RTC sudah menyimpan waktu WIB)
  struct tm t;
  t.tm_year = fullYear - 1900;
  t.tm_mon = now.month() - 1;
  t.tm_mday = now.day();
  t.tm_hour = now.hour();
  t.tm_min = now.minute();
  t.tm_sec = now.second();
  t.tm_isdst = -1;

  esp_task_wdt_reset();
  yield();

  time_t epoch = mktime(&t);
  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, NULL);

  Serial.Printf("[RTC] Time synced from PCF8563T: %04d-%02d-%02d %02d:%02d:%02d WIB\n",
               fullYear, now.month(), now.day(), now.hour(), now.minute(), now.second());

  return true;
}

void cfgSyncTime(unsigned long timeoutMs) {
  bool syncedFromNTP = false;
  time_t now;

  if (WiFi.status() == WL_CONNECTED) {
    Serial.Println("[NTP] Starting time sync...");
    esp_task_wdt_reset();
    yield();

    // Jangan percaya waktu sistem yang sudah ada sebelum SNTP benar-benar complete.
    if (esp_sntp_enabled()) {
      esp_sntp_stop();
    }
    
    // Set timezone untuk WIB (GMT+7) sebelum sinkronisasi NTP
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    
    unsigned long start = millis();
    now = time(nullptr);
    
    while ((millis() - start) < timeoutMs) {
      delay(250);
      yield();
      esp_task_wdt_reset();

      sntp_sync_status_t syncStatus = sntp_get_sync_status();
      now = time(nullptr);

      if (syncStatus == SNTP_SYNC_STATUS_COMPLETED && now >= 100000) {
        syncedFromNTP = true;
        break;
      }

      if (((millis() - start) % 2000) < 250) {
        Serial.print(".");
        yield();
        esp_task_wdt_reset();
      }
    }

    esp_task_wdt_reset();
    yield();
    
    if (syncedFromNTP) {
      struct tm *timeinfo = localtime(&now);
      
      Serial.Printf("\n[NTP] Got time from server: %04d-%02d-%02d %02d:%02d:%02d WIB\n",
                   timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                   timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
      
      // Update RTC PCF8563T dengan waktu WIB
      rtc.adjust(DateTime(
        timeinfo->tm_year + 1900,
        timeinfo->tm_mon + 1,
        timeinfo->tm_mday,
        timeinfo->tm_hour,
        timeinfo->tm_min,
        timeinfo->tm_sec
      ));
      
      Serial.Printf("[NTP] RTC updated with WIB time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                   timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                   
      // Verifikasi waktu yang tersimpan di RTC
      delay(100);
      DateTime rtcNow = rtc.now();
      
      Serial.Printf("[NTP] RTC verification: %04d-%02d-%02d %02d:%02d:%02d\n",
                   rtcNow.year(), rtcNow.month(), rtcNow.day(),
                   rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    } else {
      Serial.Println("\n[NTP] Timeout, no confirmed SNTP sync. Fallback to RTC");
    }
  }

  if (!syncedFromNTP) {
    cfgSyncFromRTC();
  }
  
  esp_task_wdt_reset();
  yield();
}

bool cfgFormatLittleFS() {
  Serial.Println("[FORMAT] Formatting LittleFS...");
  Serial.Println("[FORMAT] WARNING: All data will be erased!");
  
  LittleFS.end();
  delay(100);
  
  if (!LittleFS.begin(true)) {  // true = force format
    Serial.Println("[FORMAT] FAILED to format LittleFS!");
    return false;
  }
  
  Serial.Println("[FORMAT] LittleFS successfully formatted");
  Serial.Println("[FORMAT] Creating default config...");
  
  // Load default config
  loadDefaultConfig();
  
  // Save to file
  if (cfgSave()) {
    Serial.Println("[FORMAT] Default config successfully saved");
    return true;
  } else {
    Serial.Println("[FORMAT] FAILED to save default config");
    return false;
  }
}
