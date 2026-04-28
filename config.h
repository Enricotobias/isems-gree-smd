#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <Preferences.h>

// Compatibility shim: some files still use Serial.Println/Printf (capital P),
// but HWCDC/Arduino uses println/printf.
#ifndef ISEMS_SERIAL_CASE_COMPAT
#define ISEMS_SERIAL_CASE_COMPAT
#define Println println
#define Printf printf
#endif

// ========= PCF8563T RTC Configuration (ESP32-C3) =========
// Pin I2C untuk ESP32-C3:
// SDA = IO5 (Pin 19)
// SCL = IO6 (Pin 20)
#define RTC_SDA_PIN 5
#define RTC_SCL_PIN 6

#define FW_VERSION "1.6.7"
// Merk AC disimpan sebagai konstanta agar ikut tertanam di .bin
#define AC_BRAND "GREE"

extern const char* FIRMWARE_TAG;
extern const char* AC_BRAND_TAG;
extern RTC_PCF8563 rtc;
extern const char* PROD_ROOT_CA;


enum DayType : uint8_t {
  DAY_OFFTIME  = 0,
  DAY_FULLTIME = 1,
  DAY_PARTTIME = 2
};

#define DAY_FULL DAY_FULLTIME
#define DAY_PART_TIME DAY_PARTTIME
#define DAY_LIBUR DAY_OFFTIME

static const uint8_t MAX_PERIODS_PER_DAY = 5;

struct TimePeriod {
  char start[6];
  char end[6];
  char mode[6];
  char fan[8];
  uint8_t set_temp;
  uint8_t min_temp;
  uint8_t max_temp;
};

struct DaySchedule {
  DayType type;
  uint8_t periodCount;
  TimePeriod periods[MAX_PERIODS_PER_DAY];
};

// Day numbering standard: Senin=0, Selasa=1, ..., Minggu=6
#define DAY_NUM_TO_IDX(n) ((int)(n))
#define DAY_IDX_TO_NUM(i) ((int)(i))

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
  int global_set_temp;
  int global_min_temp;
  int global_max_temp;
  String global_mode;
  String global_fan;
  DaySchedule daily[7];
  
  // === AUTO-RECOVERY SYSTEM ===
  unsigned long lastManualControlMs;  // Timestamp remote control terakhir
  bool manualModeActive;              // Flag manual mode aktif
  bool automationPausedByUser;

  int temp_cooldown_sec;
  int auto_recovery_sec;
};

struct Config {
  String device_id;
  // Immutable device metadata (forced from firmware constants)
  String ac_brand;
  KoneksiCfg koneksi;
  SettingCfg setting;
};

extern Config cfg;

String nvs_getWiFiSSID();
String nvs_getWiFiPass();
void   nvs_setWiFiSSID(const String& ssid);
void   nvs_setWiFiPass(const String& pass);

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

