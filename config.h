#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>

// ========= PCF8563T RTC Configuration (ESP32-C3) =========
// Pin I2C untuk ESP32-C3:
// SDA = IO5 (Pin 19)
// SCL = IO6 (Pin 20)
#define RTC_SDA_PIN 5
#define RTC_SCL_PIN 6

#define FW_VERSION "1.6.5"
// Merk AC disimpan sebagai konstanta agar ikut tertanam di .bin
#define AC_BRAND "GREE"

extern const char* FIRMWARE_TAG;
extern const char* AC_BRAND_TAG;
extern RTC_PCF8563 rtc;
extern const char* PROD_ROOT_CA;


// Nama hari urut: 0=senin ... 6=minggu
static const char* DAY_NAMES[7] = {
  "senin", "selasa", "rabu", "kamis", "jumat", "sabtu", "minggu"
};

struct Waktu {
  // untuk part_time: mulai/selesai dalam format "HH:MM"
  String mulai;
  String selesai;
};

enum DayType : uint8_t {
  DAY_FULL = 0,
  DAY_PART_TIME = 1,
  DAY_LIBUR = 2
};

struct DaySchedule {
  DayType type;    // DAY_FULL, DAY_PART_TIME, or DAY_LIBUR
  Waktu waktu;     // only meaningful when type == DAY_PART_TIME
};

struct KoneksiCfg {
  String wifi_ssid;
  String wifi_pass;
  String mqtt_host;
  uint16_t mqtt_port;
  String mqtt_user;
  String mqtt_pass;
};

struct SettingCfg {
  bool automation;
  int max_temp;
  int min_temp;
  int set_temp;
  String set_mode;
  String default_fan;
  DaySchedule daily[7];
  
  // === AUTO-RECOVERY SYSTEM ===
  unsigned long lastManualControlMs;  // Timestamp remote control terakhir
  bool manualModeActive;              // Flag manual mode aktif
  bool automationPausedByUser;

  // === [BARU] DYNAMIC COOLDOWNS (Dalam Detik) ===
  int temp_control_cooldown_sec; // Default: 300 detik (5 menit) 
  int auto_recovery_sec;
  
  // === MQTT LOGGING ===
  bool mqtt_log_enabled;        // Status MQTT logging (default: false)
};

struct Config {
  String device_id;
  // Immutable device metadata (forced from firmware constants)
  String ac_brand;
  KoneksiCfg koneksi;
  SettingCfg setting;
};

extern Config cfg;

bool cfgLoad();
bool cfgSave();
void cfgApply(); // Apply MQTT config only (tanpa disconnect WiFi)
void cfgApplyWiFi(); // Apply WiFi config (disconnect & reconnect WiFi)
void cfgStartWiFiConnect(); // Non-blocking WiFi connection start
bool cfgConnectWiFiBlocking(); // Blocking WiFi connection with retry
bool cfgInitRTC();
bool cfgCheckRTC(); // Quick non-blocking I2C probe
bool cfgSyncFromRTC();
void cfgSyncTime(unsigned long timeoutMs = 5000);  // Reduced timeout untuk hindari WDT
bool cfgFormatLittleFS(); // Format ulang LittleFS (hapus semua data)

// Disconnect timestamp management
void cfgSaveDisconnectTimestamp(unsigned long epochOverride = 0);
unsigned long cfgLoadDisconnectTimestamp();
void cfgSaveLastAlive(); // Heartbeat checkpoint - file terpisah dari disconnect_ts
