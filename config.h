#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <Preferences.h>
#include "schedule_logic.h"

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

// ========= CR1220 RTC Backup Battery Monitor =========
// BAT_ADC = divider VBAT_CR1220 -> GPIO4 / ADC1_CH4
#define BAT_ADC_PIN 4
#define BAT_ADC_R_TOP_OHM 10000000.0f
#define BAT_ADC_R_BOT_OHM 4700000.0f

#define FW_VERSION "2.5.0"
// Merk AC disimpan sebagai konstanta agar ikut tertanam di .bin
#define AC_BRAND "GREE"

extern const char* FIRMWARE_TAG;
extern const char* AC_BRAND_TAG;
extern RTC_PCF8563 rtc;
extern const char* PROD_ROOT_CA;


// Day numbering standard: Minggu=0, Senin=1, ..., Sabtu=6
#define DAY_NUM_TO_IDX(n) ((int)(n))
#define DAY_IDX_TO_NUM(i) ((int)(i))

struct KoneksiCfg {
  String wifi_ssid;
  String wifi_pass;
  String mqtt_host;
  uint16_t mqtt_port;
  String mqtt_user;
  String mqtt_pass;
  int8_t tz_offset;  // Timezone offset dari UTC (jam). WIB=7, WITA=8, WIT=9
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

String getStableDeviceMac();

String nvs_getWiFiSSID();
String nvs_getWiFiPass();
void   nvs_setWiFiSSID(const String& ssid);
void   nvs_setWiFiPass(const String& pass);

bool cfgLoad();
bool cfgSave();
bool cfgSaveSchedule();
bool cfgIsRuntimeDefaultDueLoadFailure();
void cfgStartDeferredDefaultSave();
void cfgApply(); // Apply MQTT config only (tanpa disconnect WiFi)
void cfgApplyWiFi(); // Apply WiFi config (disconnect & reconnect WiFi)
void cfgStartWiFiConnect(); // Non-blocking WiFi connection start
bool cfgConnectWiFiBlocking(); // Blocking WiFi connection with retry
bool cfgInitRTC();
bool cfgCheckRTC(); // Quick non-blocking I2C probe
bool cfgSyncFromRTC();
void cfgSyncTime(unsigned long timeoutMs = 5000);  // Reduced timeout untuk hindari WDT
bool cfgFormatLittleFS(); // Format ulang LittleFS (hapus semua data)

