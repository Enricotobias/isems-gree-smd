#include "config.h"
#include "logger.h"
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

const char* DEF_DEVICE_ID = "002";

const char* DEF_WIFI_SSID = "Atop Lt2";
const char* DEF_WIFI_PASS = "Atop88ls";
const char* DEF_MQTT_HOST = "mqtt.nicourbanindonesia.com";
const uint16_t DEF_MQTT_PORT = 8883;  // Port 8883 untuk MQTT over SSL/TLS
const char* DEF_MQTT_USER = "isems";
const char* DEF_MQTT_PASS = "Atop88ls";

const bool   DEF_AUTOMATION = false;
const int    DEF_MAX_TEMP = 25;
const int    DEF_MIN_TEMP = 23;
const int    DEF_SET_TEMP = 24;
const char* DEF_SET_MODE = "COOL";
const char* DEF_DEFAULT_FAN = "LOW";  // Default fan speed

const char* DEF_SENIN_BUKA = "07:55";
const char* DEF_SENIN_TUTUP = "17:00";
const char* DEF_SELASA_BUKA = "07:55";
const char* DEF_SELASA_TUTUP = "17:00";
const char* DEF_RABU_BUKA = "07:55";
const char* DEF_RABU_TUTUP = "17:00";
const char* DEF_KAMIS_BUKA = "07:55";
const char* DEF_KAMIS_TUTUP = "17:00";
const char* DEF_JUMAT_BUKA = "07:55";
const char* DEF_JUMAT_TUTUP = "17:00";

static void feedWatchdogAndYield() {
  esp_task_wdt_reset();
  yield();
}
const char* DEF_SABTU_BUKA = "07:55";
const char* DEF_SABTU_TUTUP = "13:00";

Config cfg;

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

static void setPartTime(DaySchedule &d, const char* buka, const char* tutup) {
  d.type = DAY_PART_TIME;
  d.waktu.mulai = String(buka);
  d.waktu.selesai = String(tutup);
}

static void setFullTime(DaySchedule &d) {
  d.type = DAY_FULL;
  d.waktu.mulai = "";
  d.waktu.selesai = "";
}

static void setLibur(DaySchedule &d) {
  d.type = DAY_LIBUR;
  d.waktu.mulai = "";
  d.waktu.selesai = "";
}

// Load default configuration into memory
static void loadDefaultConfig() {
  cfg.device_id = DEF_DEVICE_ID;
  cfg.ac_brand = AC_BRAND;
  cfg.setting.automationPausedByUser = false;
  
  cfg.koneksi.wifi_ssid = DEF_WIFI_SSID;
  cfg.koneksi.wifi_pass = DEF_WIFI_PASS;
  cfg.koneksi.mqtt_host = DEF_MQTT_HOST;
  cfg.koneksi.mqtt_port = DEF_MQTT_PORT;
  cfg.koneksi.mqtt_user = DEF_MQTT_USER;
  cfg.koneksi.mqtt_pass = DEF_MQTT_PASS;

  cfg.setting.automation = DEF_AUTOMATION;
  cfg.setting.max_temp = DEF_MAX_TEMP;
  cfg.setting.min_temp = DEF_MIN_TEMP;
  cfg.setting.set_temp = DEF_SET_TEMP;
  cfg.setting.set_mode = String(DEF_SET_MODE);
  cfg.setting.default_fan = String(DEF_DEFAULT_FAN);
  cfg.setting.temp_control_cooldown_sec = 300;
  cfg.setting.auto_recovery_sec = 300; // 5 Menit (Standar aman kompresor)
  cfg.setting.mqtt_log_enabled = false; // Default: MQTT logging disabled

  // Initialize all days as full_time first
  for (int i = 0; i < 7; ++i) {
    setFullTime(cfg.setting.daily[i]);
  }

  // Set specific schedules
  setPartTime(cfg.setting.daily[0], DEF_SENIN_BUKA, DEF_SENIN_TUTUP);
  setPartTime(cfg.setting.daily[1], DEF_SELASA_BUKA, DEF_SELASA_TUTUP);
  setPartTime(cfg.setting.daily[2], DEF_RABU_BUKA, DEF_RABU_TUTUP);
  setPartTime(cfg.setting.daily[3], DEF_KAMIS_BUKA, DEF_KAMIS_TUTUP);
  setPartTime(cfg.setting.daily[4], DEF_JUMAT_BUKA, DEF_JUMAT_TUTUP);
  setPartTime(cfg.setting.daily[5], DEF_SABTU_BUKA, DEF_SABTU_TUTUP);
  setLibur(cfg.setting.daily[6]); // Minggu = libur
}

bool cfgSave() {
  // Mount Check
  if (!LittleFS.begin()) {
    logPrintln("[Config] WARNING: LittleFS not mounted, trying to mount...");
    if (!LittleFS.begin(true)) {
      logPrintln("[Config] ERROR: LittleFS mount failed completely");
      return false;
    }
  }

  // === DIAGNOSTIK PARTISI ===
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  logPrintf("[Config] Info: Total Space: %d bytes, Used: %d bytes\n", totalBytes, usedBytes);

  if (totalBytes == 0) {
    logPrintln("[Config] ⚠ FATAL ERROR: Partition size is 0! Check 'Tools > Partition Scheme' in Arduino IDE.");
    return false;
  }
  // ==========================

  StaticJsonDocument<4096> doc;

  doc["device_id"] = cfg.device_id;
  // Immutable metadata: always write from firmware constant
  cfg.ac_brand = AC_BRAND;
  doc["ac_brand"] = cfg.ac_brand;

  JsonObject koneksi = doc.createNestedObject("koneksi");
  koneksi["wifi_ssid"] = cfg.koneksi.wifi_ssid;
  koneksi["wifi_pass"] = cfg.koneksi.wifi_pass;
  koneksi["mqtt_host"] = cfg.koneksi.mqtt_host;
  koneksi["mqtt_port"] = cfg.koneksi.mqtt_port;
  koneksi["mqtt_user"] = cfg.koneksi.mqtt_user;
  koneksi["mqtt_pass"] = cfg.koneksi.mqtt_pass;

  JsonObject setting = doc.createNestedObject("setting");
  setting["automation"] = cfg.setting.automation;
  setting["max_temp"] = cfg.setting.max_temp;
  setting["min_temp"] = cfg.setting.min_temp;
  setting["set_temp"] = cfg.setting.set_temp;
  setting["set_mode"] = cfg.setting.set_mode;
  setting["default_fan"] = cfg.setting.default_fan;
  setting["temp_cooldown"]   = cfg.setting.temp_control_cooldown_sec;
  setting["auto_recovery"]   = cfg.setting.auto_recovery_sec;
  setting["mqtt_log_enabled"] = cfg.setting.mqtt_log_enabled;
  
  setting["automationPausedByUser"] = cfg.setting.automationPausedByUser;
  setting["manualModeActive"] = cfg.setting.manualModeActive;
  setting["lastManualControlMs"] = (unsigned long)cfg.setting.lastManualControlMs;
  
  JsonArray daily = setting.createNestedArray("daily");
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

  // Hapus file lama
  if (LittleFS.exists(CONFIG_PATH)) {
    LittleFS.remove(CONFIG_PATH);
  }

  File f = LittleFS.open(CONFIG_PATH, "w");
  
  // SELF-HEALING BLOCK
  if (!f) {
    logPrintln("[Config] ⚠ Error: File creation failed. Filesystem corrupt?");
    logPrintln("[Config] ⚠ ACTION: Formatting LittleFS...");
    
    LittleFS.end();
    // Format
    if (LittleFS.format()) {
      logPrintln("[Config] ✓ Format success. Waiting to stabilize...");
      delay(100); // Beri jeda sedikit
      
      // Remount dengan formatOnFail = true
      if (LittleFS.begin(true)) {
        logPrintln("[Config] ✓ Remounted. Retrying open...");
        // Coba buka file sekali lagi
        f = LittleFS.open(CONFIG_PATH, "w");
      } else {
        logPrintln("[Config] ✗ Remount failed after format.");
      }
    } else {
      logPrintln("[Config] ✗ Format failed.");
    }
  }

  if (!f) {
    logPrintln("[Config] ✗ Error: Cannot open config for write (Persistent failure)");
    return false;
  }

  size_t bytesWritten = serializeJsonPretty(doc, f);
  f.close();
  
  if (bytesWritten == 0) {
    logPrintln("[Config] Error: Failed to write config (0 bytes written)");
    return false;
  }

  logPrintf("[Config] Successfully saved %d bytes to %s\n", bytesWritten, CONFIG_PATH);
  return true;
} 

bool cfgLoad() {
  if (!LittleFS.begin()) {
    logPrintln("WARNING: LittleFS not mounted, using default config");
    loadDefaultConfig();
    return false;
  }

  if (!LittleFS.exists(CONFIG_PATH)) {
    logPrintln("WARNING: Config file not found, creating default config");
    loadDefaultConfig();
    return cfgSave();
  }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    logPrintln("ERROR: Cannot open config for read");
    loadDefaultConfig();
    return false;
  }

  size_t size = f.size();
  
  if (size > 8192) {
    logPrintln("ERROR: Config file too large (>8KB), possibly corrupt");
    f.close();
    loadDefaultConfig();
    return false;
  }
  
  std::unique_ptr<char[]> buf(new char[size + 1]);
  if (!buf) {
    logPrintln("ERROR: Failed to allocate memory for config");
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
    logPrintln(err.c_str());
    loadDefaultConfig();
    return false;
  }

  // ═══════════════════════════════════════════════════════════════════
  // [FIXED] Load config dari file - TIDAK overwrite dengan DEF_*
  // DEF_* hanya digunakan sebagai fallback jika key tidak ada di file
  // ═══════════════════════════════════════════════════════════════════
  
  cfg.device_id = doc["device_id"] | DEF_DEVICE_ID;
  // Immutable metadata: always forced to firmware constant
  cfg.ac_brand = AC_BRAND;

  JsonObject k = doc["koneksi"].as<JsonObject>();
  if (!k.isNull()) {
    cfg.koneksi.wifi_ssid = k["wifi_ssid"] | DEF_WIFI_SSID;
    cfg.koneksi.wifi_pass = k["wifi_pass"] | DEF_WIFI_PASS;
    cfg.koneksi.mqtt_host = k["mqtt_host"] | DEF_MQTT_HOST;
    cfg.koneksi.mqtt_port = k["mqtt_port"] | DEF_MQTT_PORT;
    cfg.koneksi.mqtt_user = k["mqtt_user"] | DEF_MQTT_USER;
    cfg.koneksi.mqtt_pass = k["mqtt_pass"] | DEF_MQTT_PASS;
  } else {
    cfg.koneksi.wifi_ssid = DEF_WIFI_SSID;
    cfg.koneksi.wifi_pass = DEF_WIFI_PASS;
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
    
    cfg.setting.max_temp = s["max_temp"] | DEF_MAX_TEMP;
    cfg.setting.min_temp = s["min_temp"] | DEF_MIN_TEMP;
    cfg.setting.set_temp = s["set_temp"] | DEF_SET_TEMP;
    cfg.setting.set_mode = String((const char*)(s["set_mode"] | DEF_SET_MODE));
    cfg.setting.default_fan = String((const char*)(s["default_fan"] | DEF_DEFAULT_FAN));
    
    // PERBAIKAN: Load auto-recovery fields
    cfg.setting.manualModeActive = s["manualModeActive"] | false;
    cfg.setting.lastManualControlMs = s["lastManualControlMs"] | (unsigned long)0;
    cfg.setting.temp_control_cooldown_sec = s["temp_cooldown"] | 300;
    cfg.setting.auto_recovery_sec = s["auto_recovery"] | 300;
    cfg.setting.mqtt_log_enabled = s["mqtt_log_enabled"] | false;
    cfg.setting.automationPausedByUser = s["automationPausedByUser"] | false;

    // Validate loaded values
    if (cfg.setting.lastManualControlMs == 0) {
      cfg.setting.lastManualControlMs = millis();
    }

    for (int i = 0; i < 7; ++i) {
      setFullTime(cfg.setting.daily[i]);
    }

    if (s.containsKey("daily")) {
      JsonArray daily = s["daily"].as<JsonArray>();
      for (JsonObject dayObj : daily) {
        const char* dayName = dayObj["day"] | "";
        if (!dayName || strlen(dayName) == 0) continue;
        
        int idx = -1;
        for (int i = 0; i < 7; ++i) {
          if (strcasecmp(dayName, DAY_NAMES[i]) == 0) {
            idx = i;
            break;
          }
        }
        if (idx < 0) continue;
        
        const char* t = dayObj["type"] | "full_time";
        if (strcasecmp(t, "part_time") == 0) {
          cfg.setting.daily[idx].type = DAY_PART_TIME;
          JsonObject w = dayObj["waktu"].as<JsonObject>();
          if (!w.isNull()) {
            const char* mulai = w["mulai"] | w["buka"] | "";
            const char* selesai = w["selesai"] | w["tutup"] | "";
            cfg.setting.daily[idx].waktu.mulai = String(mulai);
            cfg.setting.daily[idx].waktu.selesai = String(selesai);
          }
        } else if (strcasecmp(t, "libur") == 0) {
          setLibur(cfg.setting.daily[idx]);
        } else {
          setFullTime(cfg.setting.daily[idx]);
        }
      }
    }
  } else {
    cfg.setting.automation = DEF_AUTOMATION;
    cfg.setting.max_temp = DEF_MAX_TEMP;
    cfg.setting.min_temp = DEF_MIN_TEMP;
    cfg.setting.set_temp = DEF_SET_TEMP;
    cfg.setting.set_mode = String(DEF_SET_MODE);

    for (int i = 0; i < 7; ++i) setFullTime(cfg.setting.daily[i]);
    setPartTime(cfg.setting.daily[0], DEF_SENIN_BUKA, DEF_SENIN_TUTUP);
    setPartTime(cfg.setting.daily[1], DEF_SELASA_BUKA, DEF_SELASA_TUTUP);
    setPartTime(cfg.setting.daily[2], DEF_RABU_BUKA, DEF_RABU_TUTUP);
    setPartTime(cfg.setting.daily[3], DEF_KAMIS_BUKA, DEF_KAMIS_TUTUP);
    setPartTime(cfg.setting.daily[4], DEF_JUMAT_BUKA, DEF_JUMAT_TUTUP);
    setPartTime(cfg.setting.daily[5], DEF_SABTU_BUKA, DEF_SABTU_TUTUP);
    setLibur(cfg.setting.daily[6]);
    cfg.setting.automationPausedByUser = false;
  }

  logPrintf("[Config] Loaded - automation: %s, max_temp: %d, min_temp: %d\n",
                cfg.setting.automation ? "true" : "false",
                cfg.setting.max_temp,
                cfg.setting.min_temp);

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
  logPrintln("[Config] Applying WiFi changes");
  cfgApply();
  logPrintln("[Config] WiFi will reconnect in main loop");
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
  
  logPrintf("WiFi: connecting to [%s]...\n", cfg.koneksi.wifi_ssid.c_str());
  
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
    
    logPrintf("WiFi: Attempt %d/%d - connecting to %s ...\n", 
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
        logPrintf("\nWiFi: Connection failed - %s\n", getWiFiStatusText(WiFi.status()));
        break;
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      logPrintln();
      logPrintf("WiFi: Connected successfully!\n");
      logPrintf("WiFi: IP address: %s\n", WiFi.localIP().toString().c_str());
      logPrintf("WiFi: Signal strength: %d dBm\n", WiFi.RSSI());
      return true; // Success
    } else {
      logPrintln();
      logPrintf("WiFi: Failed to connect - %s\n", getWiFiStatusText(WiFi.status()));
      
      if (retry < MAX_RETRIES - 1) {
        logPrintf("WiFi: Retrying in 2 seconds...\n");
        WiFi.disconnect();
        delay(2000);
        feedWatchdogAndYield();
      }
    }
  }
  
  // All retries failed
  logPrintf("WiFi: Failed to connect after %d attempts\n", MAX_RETRIES);
  logPrintln("WiFi: Please check your WiFi credentials and network availability");
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
      logPrintln("[RTC] PCF8563T ditemukan di I2C 0x51");
      return true;
    }
    
    delay(200);
    yield();
    esp_task_wdt_reset();
  }
  
  logPrintln("ERROR: RTC PCF8563T tidak ditemukan (timeout 5s)");
  return false;
}

bool cfgSyncFromRTC() {
  esp_task_wdt_reset();
  yield();
  
  if (!rtc.begin()) {
    logPrintln("ERROR: Failed to access PCF8563T RTC");
    return false;
  }
  
  DateTime now = rtc.now();
  
  esp_task_wdt_reset();
  yield();
  
  int fullYear = now.year();
  
  if (fullYear < 2020 || fullYear > 2100) {
    logPrintf("WARNING: RTC time invalid (year=%d)\n", fullYear);
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

  logPrintf("[RTC] Time synced from PCF8563T: %04d-%02d-%02d %02d:%02d:%02d WIB\n",
               fullYear, now.month(), now.day(), now.hour(), now.minute(), now.second());

  return true;
}

void cfgSyncTime(unsigned long timeoutMs) {
  bool syncedFromNTP = false;
  time_t now;

  if (WiFi.status() == WL_CONNECTED) {
    logPrintln("[NTP] Starting time sync...");
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
      
      logPrintf("\n[NTP] Got time from server: %04d-%02d-%02d %02d:%02d:%02d WIB\n",
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
      
      logPrintf("[NTP] RTC updated with WIB time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                   timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                   
      // Verifikasi waktu yang tersimpan di RTC
      delay(100);
      DateTime rtcNow = rtc.now();
      
      logPrintf("[NTP] RTC verification: %04d-%02d-%02d %02d:%02d:%02d\n",
                   rtcNow.year(), rtcNow.month(), rtcNow.day(),
                   rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    } else {
      logPrintln("\n[NTP] Timeout, no confirmed SNTP sync. Fallback to RTC");
    }
  }

  if (!syncedFromNTP) {
    cfgSyncFromRTC();
  }
  
  esp_task_wdt_reset();
  yield();
}

bool cfgFormatLittleFS() {
  logPrintln("[FORMAT] Formatting LittleFS...");
  logPrintln("[FORMAT] WARNING: All data will be erased!");
  
  LittleFS.end();
  delay(100);
  
  if (!LittleFS.begin(true)) {  // true = force format
    logPrintln("[FORMAT] FAILED to format LittleFS!");
    return false;
  }
  
  logPrintln("[FORMAT] LittleFS successfully formatted");
  logPrintln("[FORMAT] Creating default config...");
  
  // Load default config
  loadDefaultConfig();
  
  // Save to file
  if (cfgSave()) {
    logPrintln("[FORMAT] Default config successfully saved");
    return true;
  } else {
    logPrintln("[FORMAT] FAILED to save default config");
    return false;
  }
}

// ==========================================
// DISCONNECT TIMESTAMP MANAGEMENT
// ==========================================
static const char* DISCONNECT_TIMESTAMP_FILE = "/disconnect_ts.txt";
static const char* LAST_ALIVE_FILE = "/last_alive.txt";

void cfgSaveDisconnectTimestamp(unsigned long epochOverride) {
  unsigned long ts = (epochOverride > 0) ? epochOverride : (unsigned long)time(nullptr);
  
  // Validasi NTP: pastikan waktu sudah sync (epoch > Sept 2020)
  if (ts < 1600000000UL) {
    logPrintf("[Config] Skip saving timestamp - NTP not synced (ts=%lu)\n", ts);
    return;
  }
  
  if (!LittleFS.begin()) {
    logPrintln("[Config] ERROR: LittleFS not mounted for saving timestamp");
    return;
  }
  
  File f = LittleFS.open(DISCONNECT_TIMESTAMP_FILE, "w");
  if (f) {
    f.println(ts);
    f.close();
    // Konversi ke WIB untuk log
    time_t t = (time_t)ts;
    struct tm* tmInfo = localtime(&t);
    if (tmInfo) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d WIB",
               tmInfo->tm_year + 1900, tmInfo->tm_mon + 1, tmInfo->tm_mday,
               tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
      logPrintf("[Config] Disconnect timestamp saved: %s (epoch=%lu)\n", buf, ts);
    } else {
      logPrintf("[Config] Disconnect timestamp saved: epoch=%lu\n", ts);
    }
  } else {
    logPrintln("[Config] ERROR: Failed to save disconnect timestamp");
  }
}

unsigned long cfgLoadDisconnectTimestamp() {
  if (!LittleFS.begin()) {
    logPrintln("[Config] ERROR: LittleFS not mounted for loading timestamp");
    return 0;
  }

  // Prioritas utama: last_alive.txt (ditulis heartbeat tiap 30s, tidak pernah ditimpa
  // oleh path reconnect/offline notify sehingga nilainya selalu akurat).
  if (LittleFS.exists(LAST_ALIVE_FILE)) {
    File fa = LittleFS.open(LAST_ALIVE_FILE, "r");
    if (fa) {
      String laLine = fa.readStringUntil('\n');
      fa.close();
      unsigned long laTs = laLine.toInt();
      if (laTs > 1600000000UL) {
        logPrintf("[Config] Last alive timestamp loaded: %lu\n", laTs);
        // Bersihkan disconnect_ts jika ada (tidak terpakai)
        if (LittleFS.exists(DISCONNECT_TIMESTAMP_FILE)) {
          LittleFS.remove(DISCONNECT_TIMESTAMP_FILE);
        }
        return laTs;
      }
    }
  }

  // Fallback: disconnect_ts.txt (untuk kasus pertama kali boot / last_alive belum ada)
  if (!LittleFS.exists(DISCONNECT_TIMESTAMP_FILE)) {
    logPrintln("[Config] No disconnect timestamp file found");
    return 0;
  }

  File f = LittleFS.open(DISCONNECT_TIMESTAMP_FILE, "r");
  if (!f) {
    logPrintln("[Config] ERROR: Failed to open disconnect timestamp file");
    return 0;
  }

  String line = f.readStringUntil('\n');
  f.close();

  unsigned long ts = line.toInt();
  if (ts > 0) {
    LittleFS.remove(DISCONNECT_TIMESTAMP_FILE);
    logPrintf("[Config] Disconnect timestamp loaded (fallback) and removed: %lu\n", ts);
  } else {
    logPrintln("[Config] WARNING: Invalid timestamp in file");
  }

  return ts;
}

// Simpan "last alive" checkpoint untuk proteksi power loss.
// Menggunakan file TERPISAH (/last_alive.txt) sehingga tidak menimpa
// /disconnect_ts.txt yang menyimpan waktu disconnect sesungguhnya.
void cfgSaveLastAlive() {
  unsigned long ts = (unsigned long)time(nullptr);
  if (ts < 1600000000UL) return; // NTP belum sync
  
  if (!LittleFS.begin()) return;
  
  File f = LittleFS.open(LAST_ALIVE_FILE, "w");
  if (f) {
    f.println(ts);
    f.close();
  }
}
