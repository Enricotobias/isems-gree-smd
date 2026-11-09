#include "automation.h"
#include "config.h"
#include "controll.h"
#include "crash_monitor.h"
#include "sensor.h"
#include "data.h"
#include "rtos_common.h"
#include <time.h>
#include <ArduinoJson.h>

#include "activity.h"
#include <WiFi.h>

static String normalizeFanSpeed(const String& fan) {
  String normalized = fan;
  normalized.toUpperCase();
  normalized.trim();
  
  if (normalized == "MED") normalized = "MEDIUM";
  if (normalized == "MIN") normalized = "LOW";
  if (normalized == "MAX") normalized = "HIGH";
  
  return normalized;
}

static unsigned long lastScheduleCheckMs = 0;
const unsigned long SCHEDULE_CHECK_INTERVAL_MS = 10000UL;

static unsigned long lastTempControlMs = 0;
static unsigned long lastIRCommandMs = 0;
const unsigned long IR_SETTLE_TIME_MS = 5000;
static bool waitingForACResponse = false;
static bool expectedACState = false;

// IR verification state (Gree: uses display frame when available)
static unsigned long irVerifyStartMs = 0;
static uint8_t irVerifyRetryCount = 0;
// Timeout harus >= IR_SETTLE_TIME_MS * (max_retries + 2) = 5000 * 5 = 25000ms
// agar setiap retry mendapat kesempatan settle penuh sebelum timeout.
static const unsigned long IR_VERIFY_TIMEOUT_MS = 25000UL;

// Global throttle untuk mencegah spam IR command (jadwal/resync/mode-sync).
// Aman: sedikit di atas IR settle time agar display/state sempat update.

// Legacy counters retained for binary/config compatibility; temperature maintenance is disabled.
static int coolingAttemptCount = 0;

static bool expectedPowerValid = false;
static bool expectedModeValid  = false;
static bool expectedFanValid   = false;
static bool expectedSetTempValid = false;

static bool expectedPower;
static String expectedMode;
static String expectedFan;
static uint8_t expectedSetTemp;

volatile bool forceScheduleCheck = false;

static unsigned long lastSystemActionMs = 0;
static const unsigned long SYSTEM_ACTION_WINDOW_MS = 15000;
static bool firstRunMonitor = true;

static bool lastLoggedShouldBeOn = false;
static bool logStateInit = false;
static bool lastDryModeWindowActive = false;

static bool lastKnownPower = false;
static uint8_t lastKnownSetTemp = 0;
static String lastKnownMode = "";
static String lastKnownFan = "";

static int lastKnownCurrentTemp = 0;

// Jika user mengubah set_temp via remote fisik (dalam range), kita anggap "allowed".
// Override ini dipakai agar koreksi fan tidak ikut memaksa set_temp kembali.
// Namun override akan dibersihkan otomatis saat target automation (cfg.setting.global_set_temp) berubah.
static bool allowedSetTempOverrideActive = false;
static uint8_t allowedSetTempOverride = 0;
static int lastConfigSetTemp = -1;

// BP-frame fallback monitoring (khusus POWER) saat display frame tidak valid.
static bool firstRunBpMonitor = true;
static bool lastKnownBpPower = false;
static String lastKnownBpMode = "";
static String lastKnownBpFan = "";

static bool lastSchedulePower = false;
static bool scheduleStateInit = false;
static bool lastScheduleTargetValid = false;
static char lastScheduleTargetMode[6] = {};
static char lastScheduleTargetFan[8] = {};
static int lastScheduleTargetTemp = -1;

enum ActionSource {
  ACTION_NONE,
  ACTION_AUTOMATION,
  ACTION_MQTT_REMOTE
};

static ActionSource lastActionSource = ACTION_NONE;
static unsigned long lastActionMs = 0;
#define ACTION_WINDOW_MS 8000UL

static unsigned long expectedStateSetMs = 0;
static const unsigned long EXPECTED_STATE_WINDOW_MS = 10000UL;

static bool isExpectedStateValid() {
  if (!expectedPowerValid && !expectedModeValid && !expectedFanValid && !expectedSetTempValid) {
    return false;
  }
  return (millis() - expectedStateSetMs) < EXPECTED_STATE_WINDOW_MS;
}

static void clearExpectedState() {
  expectedPowerValid = false;
  expectedModeValid = false;
  expectedFanValid = false;
  expectedSetTempValid = false;
}

static void setExpectedState(bool power, const String& mode, const String& fan, uint8_t setTemp) {
  expectedPower = power;
  expectedPowerValid = true;

  expectedMode = mode;
  expectedModeValid = (mode.length() > 0);

  expectedFan = normalizeFanSpeed(fan);
  expectedFanValid = (fan.length() > 0);

  expectedSetTemp = setTemp;
  expectedSetTempValid = (setTemp > 0);
  expectedStateSetMs = millis();

  Serial.Printf(
    "[Expected] Set: power=%s mode=%s fan=%s set_temp=%d (valid: p=%d m=%d f=%d t=%d)\n",
    power ? "ON" : "OFF",
    expectedModeValid ? expectedMode.c_str() : "-",
    expectedFanValid ? expectedFan.c_str() : "-",
    expectedSetTempValid ? (int)expectedSetTemp : 0,
    expectedPowerValid ? 1 : 0,
    expectedModeValid ? 1 : 0,
    expectedFanValid ? 1 : 0,
    expectedSetTempValid ? 1 : 0
  );
}

static bool matchesExpectedState(const DisplayFrame& disp) {
  if (!isExpectedStateValid()) return false;
  
  if (expectedPowerValid && disp.power != expectedPower) {
    Serial.Printf("[Expected] Power mismatch: expected=%s got=%s\n",
              expectedPower ? "ON" : "OFF", disp.power ? "ON" : "OFF");
    return false;
  }
  
  if (expectedModeValid && disp.power) {
    String dispMode = disp.mode;
    String expMode = expectedMode;
    dispMode.toUpperCase();
    expMode.toUpperCase();
    if (dispMode != expMode) {
      Serial.Printf("[Expected] Mode mismatch: expected=%s got=%s\n",
                expMode.c_str(), dispMode.c_str());
      return false;
    }
  }
  
  if (expectedFanValid && disp.power) {
    String dispFan = normalizeFanSpeed(disp.fan);
    String expFan = normalizeFanSpeed(expectedFan);
    if (dispFan != expFan) {
      Serial.Printf("[Expected] Fan mismatch: expected=%s got=%s\n",
                expFan.c_str(), dispFan.c_str());
      return false;
    }
  }
  
  if (expectedSetTempValid && disp.power) {
    if (disp.set_temp != expectedSetTemp) {
      Serial.Printf("[Expected] SetTemp mismatch: expected=%d got=%d\n",
                expectedSetTemp, disp.set_temp);
      return false;
    }
  }
  
  return true;
}

static bool getACPowerStatus() {
  // Mainboard reflects the actual indoor-unit power state. Prefer it so an
  // ambiguous display frame cannot make automation resend a redundant OFF IR command.
  BPFrame frame = sensorGetLastFrame();
  if (sensorIsHealthy()) {
    return frame.power;
  }

  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    return disp.power;
  }

  return false;
}

static int getDayOfWeek() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  // tm_wday is 0 for Sunday, 1 for Monday, etc.
  return timeinfo.tm_wday;
}

static void getCurrentHourMinute(int &hour, int &minute) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    hour = -1; minute = -1; return;
  }
  hour = timeinfo.tm_hour;
  minute = timeinfo.tm_min;
}

static const TimePeriod* getActiveTimePeriod() {
  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) return nullptr;

  int curH, curM;
  getCurrentHourMinute(curH, curM);
  if (curH < 0) return nullptr;
  return scheduleFindActivePeriod(cfg.setting.daily, dayIdx, curH * 60 + curM);
}

static int parseMinuteOfDay(const char* hhmm) {
  return scheduleParseMinute(hhmm);
}

struct MinuteInterval {
  int start;
  int end;
};

static void addInterval(MinuteInterval intervals[], uint8_t& count, int startMin, int endMin) {
  if (startMin < 0 || endMin < 0 || count >= (MAX_PERIODS_PER_DAY * 2)) return;

  if (endMin < startMin) {
    intervals[count++] = {startMin, 1440};
    if (count < (MAX_PERIODS_PER_DAY * 2)) intervals[count++] = {0, endMin + 1};
  } else {
    intervals[count++] = {startMin, endMin + 1};
  }
}

static bool scheduleCoversFullDay(const DaySchedule& day) {
  if (day.type == DAY_FULLTIME) return true;
  if (day.type != DAY_PARTTIME) return false;

  MinuteInterval intervals[MAX_PERIODS_PER_DAY * 2];
  uint8_t count = 0;
  uint8_t periodCount = day.periodCount;
  if (periodCount > MAX_PERIODS_PER_DAY) periodCount = MAX_PERIODS_PER_DAY;

  for (uint8_t i = 0; i < periodCount; i++) {
    int startMin = parseMinuteOfDay(day.periods[i].start);
    int endMin = parseMinuteOfDay(day.periods[i].end);
    addInterval(intervals, count, startMin, endMin);
  }
  if (count == 0) return false;

  for (uint8_t i = 1; i < count; i++) {
    MinuteInterval key = intervals[i];
    int j = i - 1;
    while (j >= 0 && intervals[j].start > key.start) {
      intervals[j + 1] = intervals[j];
      j--;
    }
    intervals[j + 1] = key;
  }

  const int maxGapMin = 1;
  if (intervals[0].start > maxGapMin) return false;

  int coveredUntil = intervals[0].end;
  for (uint8_t i = 1; i < count; i++) {
    if (intervals[i].start > coveredUntil + maxGapMin) return false;
    if (intervals[i].end > coveredUntil) coveredUntil = intervals[i].end;
  }

  return coveredUntil >= (1440 - maxGapMin);
}

static int firstPeriodStartMinute(const DaySchedule& day) {
  int firstStart = -1;
  uint8_t periodCount = day.periodCount;
  if (periodCount > MAX_PERIODS_PER_DAY) periodCount = MAX_PERIODS_PER_DAY;

  for (uint8_t i = 0; i < periodCount; i++) {
    int startMin = parseMinuteOfDay(day.periods[i].start);
    if (startMin >= 0 && (firstStart < 0 || startMin < firstStart)) {
      firstStart = startMin;
    }
  }
  return firstStart;
}

static bool isWithinMinuteWindow(int currentMin, int startMin, int durationMin) {
  if (currentMin < 0 || startMin < 0 || durationMin <= 0) return false;
  int endMin = startMin + durationMin;
  if (endMin <= 1440) return currentMin >= startMin && currentMin < endMin;
  return currentMin >= startMin || currentMin < (endMin - 1440);
}

static bool isDryModeWindowActive(const TimePeriod* activePeriod) {
  if (!activePeriod) return false;

  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) return false;

  DaySchedule& day = cfg.setting.daily[dayIdx];
  if (day.type == DAY_OFFTIME) return false;

  int hour, minute;
  getCurrentHourMinute(hour, minute);
  if (hour < 0) return false;

  int startMin = scheduleCoversFullDay(day) ? (6 * 60) : firstPeriodStartMinute(day);
  return isWithinMinuteWindow((hour * 60) + minute, startMin, 15);
}

static inline int resolveSetTemp(const TimePeriod* tp) {
  return (tp && tp->set_temp > 0) ? tp->set_temp : cfg.setting.global_set_temp; // DEF 24
}

static inline int resolveMinTemp(const TimePeriod* tp) {
  return (tp && tp->min_temp > 0) ? tp->min_temp : cfg.setting.global_min_temp; // DEF 20
}

static inline int resolveMaxTemp(const TimePeriod* tp) {
  return (tp && tp->max_temp > 0) ? tp->max_temp : cfg.setting.global_max_temp; // DEF 26
}

static inline String resolveMode(const TimePeriod* tp) {
  String fallback = cfg.setting.global_mode;
  if (fallback.length() == 0) fallback = "COOL";
  return (tp && tp->mode[0] != '\0') ? String(tp->mode) : fallback;
}

static inline String resolveFan(const TimePeriod* tp) {
  String fallback = cfg.setting.global_fan;
  if (fallback.length() == 0) fallback = "AUTO";
  String fan = (tp && tp->fan[0] != '\0') ? String(tp->fan) : fallback;
  // AUTO tidak bisa divalidasi IR (display tidak mengirimkan nilai AUTO).
  // Konversi ke LOW di sini agar semua path otomatis aman.
  String f = fan;
  f.toUpperCase();
  f.trim();
  if (f == "AUTO") return String("LOW");
  return normalizeFanSpeed(fan);
}

static bool scheduleTargetChanged(const String& mode, const String& fan, int setTemp) {
  return !lastScheduleTargetValid || strcmp(lastScheduleTargetMode, mode.c_str()) != 0 ||
         strcmp(lastScheduleTargetFan, fan.c_str()) != 0 ||
         lastScheduleTargetTemp != setTemp;
}

static void rememberScheduleTarget(const String& mode, const String& fan, int setTemp) {
  memset(lastScheduleTargetMode, 0, sizeof(lastScheduleTargetMode));
  memset(lastScheduleTargetFan, 0, sizeof(lastScheduleTargetFan));
  strncpy(lastScheduleTargetMode, mode.c_str(), sizeof(lastScheduleTargetMode) - 1);
  strncpy(lastScheduleTargetFan, fan.c_str(), sizeof(lastScheduleTargetFan) - 1);
  lastScheduleTargetTemp = setTemp;
  lastScheduleTargetValid = true;
}

// sanitizeFan tetap ada sebagai safety net untuk input dari luar resolveFan.
static inline String sanitizeFan(const String& fan) {
  String f = fan;
  f.toUpperCase();
  f.trim();
  if (f == "AUTO") return String("LOW");
  return normalizeFanSpeed(fan);
}

static void turnACOn() {
  Serial.Println("[Auto] Perintahkan AC ON");
  autoOnCount++;
  
  BPFrame currentFrame = sensorGetLastFrame();
  BPFrame newState = currentFrame;
  newState.power = true;
  
  if (sensorIsHealthy() && !currentFrame.power) {
    activityTrackPower(currentFrame.power, true, SOURCE_AUTOMATION);
  }
  
  lastActionMs = millis();
  lastActionSource = ACTION_AUTOMATION;

  // Kita hanya mengirim command POWER saja di sini.
  // Jangan set expected mode/fan/temp agar monitor tidak salah menganggap mismatch sebagai physical remote.
  setExpectedState(true, "", "", 0);
  
  asm volatile("" ::: "memory");
  
  StaticJsonDocument<256> doc;
  doc["power"] = true;
  String jsonStr; 
  serializeJson(doc, jsonStr);
  controllEnqueueCommand(jsonStr.c_str(), jsonStr.length());
  
  lastActionMs = millis();
}

static void turnACOff() {
  Serial.Println("[Auto] Perintahkan AC OFF");
  autoOffCount++;

  BPFrame currentFrame = sensorGetLastFrame();
  if (sensorIsHealthy()) {
    BPFrame newState = currentFrame;
    newState.power = false;
    activityTrackChange(currentFrame, newState, SOURCE_AUTOMATION);
  }

  lastActionMs = millis();
  lastActionSource = ACTION_AUTOMATION;
  
  setExpectedState(false, "", "", 0);
  
  StaticJsonDocument<256> doc;
  doc["power"] = false;
  String jsonStr; serializeJson(doc, jsonStr);
  controllEnqueueCommand(jsonStr.c_str(), jsonStr.length());
}

static void applyTempModeFan(int temp, const String& mode, const String& fan) {
  // AUTO tidak dikenali oleh display frame â€” konversi ke LOW agar IR verification bekerja.
  const String safeFan = sanitizeFan(fan);
  Serial.Printf("[Auto] Set %dÂ°C, mode=%s, fan=%s\n", temp, mode.c_str(), safeFan.c_str());

  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    if ((int)disp.set_temp != temp) {
      activityTrackTemp((int)disp.set_temp, temp, SOURCE_AUTOMATION);
    }
  }

  lastActionMs = millis();
  lastActionSource = ACTION_AUTOMATION;
  
  setExpectedState(true, mode, safeFan, (uint8_t)temp);
  
  StaticJsonDocument<256> doc;
  doc["power"] = true; doc["temp"] = temp; doc["mode"] = mode; doc["fan"] = safeFan;
  String jsonStr; serializeJson(doc, jsonStr);
  controllEnqueueCommand(jsonStr.c_str(), jsonStr.length());
}

static void applyModeOnly(const String& mode) {
  Serial.Printf("[Auto] Set mode=%s\n", mode.c_str());
  
  BPFrame currentFrame = sensorGetLastFrame();
  if (sensorIsHealthy() && currentFrame.mode != mode) {
    activityTrackMode(currentFrame.mode, mode, SOURCE_AUTOMATION);
  }
  
  lastActionMs = millis();
  lastActionSource = ACTION_AUTOMATION;

  // Command ini hanya mengirim MODE, jadi expected cukup mode saja.
  setExpectedState(true, mode, "", 0);
  
  StaticJsonDocument<256> doc;
  doc["mode"] = mode;
  String jsonStr; 
  serializeJson(doc, jsonStr);
  controllEnqueueCommand(jsonStr.c_str(), jsonStr.length());
}

static void controlAC(bool power, String mode, int temp, String fanSpeed = "") {
  if (power) {
    if (fanSpeed.length() == 0) {
      String rFan = resolveFan(getActiveTimePeriod());
      if (rFan.length() > 0) {
        fanSpeed = rFan;
        Serial.Printf("[Auto] Using period target fan: %s\n", fanSpeed.c_str());
      } else {
        fanSpeed = "LOW";
        Serial.Println("[Auto] No target fan found, using LOW");
      }
    }
    
    applyTempModeFan(temp, mode, fanSpeed);
  } else {
    turnACOff();
  }
}

static bool computeAutomationShouldBeOnNow(bool& shouldBeOn) {
  shouldBeOn = lastSchedulePower;

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  struct tm timeinfo;
  bool rtcOk = getLocalTime(&timeinfo);

  if (!wifiOk && !rtcOk) {
    Serial.Printf("[Auto] Immediate resync fallback to last schedule state: %s\n",
              shouldBeOn ? "ON" : "OFF");
    return false;
  }

  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) {
    Serial.Printf("[Auto] Immediate resync fallback to last schedule state: %s\n",
              shouldBeOn ? "ON" : "OFF");
    return false;
  }

  const TimePeriod* tp = getActiveTimePeriod();
  shouldBeOn = (tp != nullptr);

  return true;
}

// =================================================================
// TEMPERATURE MAINTENANCE
// Alur:
// 1. Suhu dalam range (min_temp <= suhu <= max_temp) â†’ No action
// 2. Suhu > max_temp â†’ Set temp, fan MEDIUM, 3x retry lalu temp-2
// 3. Suhu < min_temp â†’ Set temp, fan LOW
// =================================================================
static void checkAndMaintainTemperature() {
  if (!getACPowerStatus()) {
    // Reset counter jika AC OFF
    coolingAttemptCount = 0;
    return;
  }

  BPFrame frame = sensorGetLastFrame();
  if (!frame.valid) return;

  unsigned long now = millis();

  // Check temp_cooldown
  unsigned long tempCdMs = (unsigned long)cfg.setting.temp_cooldown_sec * 1000UL;
  if (now - lastTempControlMs < tempCdMs) {
    return;
  }
  
  const TimePeriod* tp = getActiveTimePeriod();
  if (!tp) {
    if (getACPowerStatus()) {
      turnACOff();
    }
    return;
  }
  if (isDryModeWindowActive(tp)) {
    coolingAttemptCount = 0;
    return;
  }

  int currentTemp = frame.current_temp;
  int setTemp = resolveSetTemp(tp);
  int maxTemp = resolveMaxTemp(tp);
  int minTemp = resolveMinTemp(tp);
  String targetMode = resolveMode(tp);
  String targetFan = resolveFan(tp);

  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  // CASE 1: SUHU DALAM RANGE â†’ CHECK FAN SPEED
  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  if (currentTemp >= minTemp && currentTemp <= maxTemp) {
    // Reset counter jika suhu kembali normal
    if (coolingAttemptCount > 0) {
      Serial.Println("[Auto] Temperature back to normal range, resetting counter");
      coolingAttemptCount = 0;
    }
    
    // Check if fan speed matches default_fan setting
    String currentFan = frame.fan;
    String defaultFan = targetFan;
    currentFan = normalizeFanSpeed(currentFan);
    defaultFan = normalizeFanSpeed(defaultFan);
    
    if (currentFan != defaultFan) {
      Serial.Printf("[Auto] Temperature OK but fan mismatch: %s != %s, adjusting...\n", 
                currentFan.c_str(), defaultFan.c_str());
      
      lastTempControlMs = now;

      int desiredTempForFan = setTemp;
      if (allowedSetTempOverrideActive) {
        desiredTempForFan = (int)allowedSetTempOverride;
      }
      
      if (frame.fan != targetFan) {
        activityTrackFan(frame.fan, targetFan, SOURCE_AUTOMATION);
      }
      applyTempModeFan(desiredTempForFan, targetMode, targetFan);
    }
    
    return;
  }

  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  // CASE 2: SUHU > MAX_TEMP â†’ COOLING MODE
  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  if (currentTemp > maxTemp) {
    coolingAttemptCount++;
    Serial.Printf("[Auto] Too hot: %dÂ°C > %dÂ°C (attempt %d/3)\n", 
              currentTemp, maxTemp, coolingAttemptCount);
    
    lastTempControlMs = now;
    
    // Cek apakah sudah 3x attempt
    if (coolingAttemptCount >= 3) {
      int newSetTemp = setTemp - 2;
      if (newSetTemp < 16) newSetTemp = 16; // Min AC limit
      
      Serial.Printf("[Auto] 3x cooling failed, lowering setpoint: %dÂ°C â†’ %dÂ°C\n", 
                setTemp, newSetTemp);
      
      if (frame.fan != "MEDIUM") {
        activityTrackFan(frame.fan, "MEDIUM", SOURCE_AUTOMATION);
      }
      applyTempModeFan(newSetTemp, targetMode, "MEDIUM");
      
      // Reset counter setelah menurunkan setpoint
      coolingAttemptCount = 0;
    } else {
      // Normal cooling: set_temp, fan MEDIUM
      Serial.Printf("[Auto] Cooling action: %dÂ°C, fan=MEDIUM\n", setTemp);
      
      if (frame.fan != "MEDIUM") {
        activityTrackFan(frame.fan, "MEDIUM", SOURCE_AUTOMATION);
      }
      applyTempModeFan(setTemp, targetMode, "MEDIUM");
    }
    
    return;
  }

  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  // CASE 3: SUHU < MIN_TEMP â†’ WARMING MODE
  // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
  if (currentTemp < minTemp) {
    // Reset cooling counter
    coolingAttemptCount = 0;
    
    Serial.Printf("[Auto] Too cold: %dÂ°C < %dÂ°C\n", currentTemp, minTemp);
    
    lastTempControlMs = now;
    
    // Set ke set_temp dengan fan LOW
    Serial.Printf("[Auto] Warming action: %dÂ°C, fan=LOW\n", setTemp);
    
    if (frame.fan != "LOW") {
      activityTrackFan(frame.fan, "LOW", SOURCE_AUTOMATION);
    }
    applyTempModeFan(setTemp, targetMode, "LOW");
    
    return;
  }
}

void monitorExternalChange() {
  BPFrame mainboard = sensorGetLastFrame();

  // Jika display frame tidak valid, tetap monitor perubahan POWER dari BP frame.
  // Tujuan: jika user mematikan/menyalakan AC dari remote/panel (power change),
  // kita tetap bisa masuk manual recovery agar automation tidak langsung melawan.
  if (!sensorHasValidDisplayFrame()) {
    // Reset baseline monitor display agar tidak false-detect ketika display kembali valid.
    firstRunMonitor = true;

    if (!mainboard.valid) {
      return;
    }

    if (firstRunBpMonitor) {
      lastKnownBpPower = mainboard.power;
      lastKnownBpMode = mainboard.mode;
      lastKnownBpFan = mainboard.fan;
      firstRunBpMonitor = false;
      return;
    }

    bool bpPowerChanged = (mainboard.power != lastKnownBpPower);
    bool bpModeChanged = (mainboard.mode != lastKnownBpMode);
    bool bpFanChanged  = (normalizeFanSpeed(mainboard.fan) != normalizeFanSpeed(lastKnownBpFan));

    if (!bpPowerChanged && !bpModeChanged && !bpFanChanged) {
      return;
    }

    unsigned long now = millis();

    if (bpPowerChanged) {
      Serial.Printf("[Monitor-BP] Power Change: %s â†’ %s (Display invalid)\n",
                lastKnownBpPower ? "ON" : "OFF", mainboard.power ? "ON" : "OFF");
    } else {
      Serial.Printf("[Monitor-BP] Non-power change (Display invalid): mode=%sâ†’%s fan=%sâ†’%s\n",
                lastKnownBpMode.c_str(), mainboard.mode.c_str(),
                lastKnownBpFan.c_str(), mainboard.fan.c_str());
    }

    // Jika perubahan ini sesuai expected state (hasil Automation/MQTT), jangan trigger physical-remote path.
    if (isExpectedStateValid()) {
      bool ok = true;
      if (expectedPowerValid && (mainboard.power != expectedPower)) ok = false;

      if (expectedModeValid && mainboard.power) {
        String mbMode = mainboard.mode;
        String expMode = expectedMode;
        mbMode.toUpperCase();
        expMode.toUpperCase();
        if (mbMode != expMode) ok = false;
      }

      if (expectedFanValid && mainboard.power) {
        String mbFan = normalizeFanSpeed(mainboard.fan);
        String expFan = normalizeFanSpeed(expectedFan);
        if (mbFan != expFan) ok = false;
      }

      if (ok) {
        Serial.Println("[Monitor-BP] âœ“ Change matches expected state (MQTT/Automation)");
        clearExpectedState();
        lastActionSource = ACTION_NONE;
        lastKnownBpPower = mainboard.power;
        lastKnownBpMode = mainboard.mode;
        lastKnownBpFan = mainboard.fan;
        return;
      }
    }

    // Jika baru saja ada aksi sistem, jangan langsung trigger physical-remote path.
    unsigned long timeSinceAction = now - lastActionMs;
    if (timeSinceAction <= ACTION_WINDOW_MS && lastActionSource != ACTION_NONE) {
      Serial.Printf("[Monitor-BP] Recent system action (%lums) - ignore power change\n", timeSinceAction);
      lastKnownBpPower = mainboard.power;
      lastKnownBpMode = mainboard.mode;
      lastKnownBpFan = mainboard.fan;
      return;
    }

    // Jika automation sedang OFF, hanya update baseline.
    if (!cfg.setting.automation) {
      Serial.Println("[Monitor-BP] Automation OFF -> No action (but still track activity)");

      // Tetap track perubahan walaupun automation OFF
      if (bpPowerChanged) {
        activityTrackPower(lastKnownBpPower, mainboard.power, SOURCE_PHYSICAL_REMOTE);
      }
      if (bpModeChanged && mainboard.power) {
        activityTrackMode(lastKnownBpMode, mainboard.mode, SOURCE_PHYSICAL_REMOTE);
      }
      if (bpFanChanged && mainboard.power) {
        activityTrackFan(lastKnownBpFan, mainboard.fan, SOURCE_PHYSICAL_REMOTE);
      }

      lastKnownBpPower = mainboard.power;
      lastKnownBpMode = mainboard.mode;
      lastKnownBpFan = mainboard.fan;
      return;
    }

    // Non-power change: resync ke state automation (set temp tetap dari config, bukan dari BP).
    if (!bpPowerChanged) {
      Serial.Println("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
      Serial.Println("â•‘ âš ï¸ PHYSICAL REMOTE CHANGE DETECTED    â•‘");
      Serial.Println("â•‘ Source: IR Remote / Panel Button      â•‘");
      Serial.Println("â•‘ (Display invalid) -> RESYNC AUTOMATIONâ•‘");
      if (bpModeChanged) {
        Serial.Printf("â•‘ Mode(BP): %s â†’ %s                 â•‘\n",
                  lastKnownBpMode.c_str(), mainboard.mode.c_str());
      }
      if (bpFanChanged) {
        Serial.Printf("â•‘ Fan(BP): %s â†’ %s                  â•‘\n",
                  lastKnownBpFan.c_str(), mainboard.fan.c_str());
      }
      Serial.Println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

      if (bpModeChanged && mainboard.power) {
        activityTrackMode(lastKnownBpMode, mainboard.mode, SOURCE_PHYSICAL_REMOTE);
      }
      if (bpFanChanged && mainboard.power) {
        activityTrackFan(lastKnownBpFan, mainboard.fan, SOURCE_PHYSICAL_REMOTE);
      }

      clearExpectedState();

      const TimePeriod* tp = getActiveTimePeriod();
      String targetMode = resolveMode(tp);
      int targetSetTemp = resolveSetTemp(tp);
      String resyncFan = resolveFan(tp);

      lastKnownBpPower = mainboard.power;
      lastKnownBpMode = mainboard.mode;
      lastKnownBpFan = mainboard.fan;

      if (mainboard.power) {
        lastActionMs = millis();
        lastActionSource = ACTION_AUTOMATION;
        applyTempModeFan(targetSetTemp, targetMode, resyncFan);
      } else {
        forceScheduleCheck = true;
      }
      return;
    }

    // Physical remote power change detected -> masuk manual recovery.
    Serial.Println("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
    Serial.Println("â•‘ âš ï¸ PHYSICAL REMOTE POWER DETECTED     â•‘");
    Serial.Println("â•‘ Source: IR Remote / Panel Button      â•‘");
    Serial.printf("â•‘ Power: %s â†’ %s                    â•‘\n",
            lastKnownBpPower ? "ON" : "OFF", mainboard.power ? "ON" : "OFF");
    Serial.Println("â•‘ -> Manual Recovery ACTIVE             â•‘");
    Serial.Println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

    manualOverrideCount++;
    Serial.Printf("[Auto] Manual Override Count: %lu\n", manualOverrideCount);
    activityTrackPower(lastKnownBpPower, mainboard.power, SOURCE_PHYSICAL_REMOTE);

    cfg.setting.automation = false;

    if (cfgSave()) {
      cfgApply();
      if (dataIsConnected()) dataPublishSetting();
    }

    clearExpectedState();
    lastActionSource = ACTION_NONE;

    lastKnownBpPower = mainboard.power;
    lastKnownBpMode = mainboard.mode;
    lastKnownBpFan = mainboard.fan;
    return;
  }

  DisplayFrame disp = sensorGetLastDisplayFrame();
  
  if (firstRunMonitor) {
    lastKnownPower = disp.power;
    lastKnownSetTemp = disp.set_temp;
    lastKnownMode = disp.mode;
    lastKnownFan = disp.fan;
    if (mainboard.valid) {
      lastKnownCurrentTemp = mainboard.current_temp;
    }
    firstRunMonitor = false;

    if (mainboard.valid) {
      lastKnownBpPower = mainboard.power;
      firstRunBpMonitor = false;
    }
    return;
  }

  bool powerChanged = disp.power != lastKnownPower;
  bool setTempChanged = disp.set_temp != lastKnownSetTemp;
  bool modeChanged = disp.mode != lastKnownMode;
  bool fanChanged = disp.fan != lastKnownFan;

  // Semua perubahan set temperature dari remote fisik harus masuk ke jalur
  // resync automation, termasuk nilai yang dahulu dianggap masih dalam range.
  // Legacy in-range override path is intentionally disabled. All set_temp
  // changes must continue to the common automation resync path below.
  if (false && setTempChanged && !powerChanged && !modeChanged && !fanChanged) {
    const TimePeriod* tp = getActiveTimePeriod();
    uint8_t minTemp = resolveMinTemp(tp);
    uint8_t maxTemp = resolveMaxTemp(tp);
    uint8_t newSetTemp = disp.set_temp;
    
    if (newSetTemp >= minTemp && newSetTemp <= maxTemp) {
      Serial.Printf("[Monitor] Set temp changed: %d â†’ %d (Within range [%d-%d] - Allowed)\n", 
                lastKnownSetTemp, newSetTemp, minTemp, maxTemp);
      Serial.Println("[Monitor] â†’ No manual recovery triggered");

      // Simpan override set_temp (allowed) supaya koreksi fan tidak memaksa balik.
      allowedSetTempOverrideActive = true;
      allowedSetTempOverride = newSetTemp;

      // Jika perubahan ini sebenarnya hasil aksi sistem (Automation/MQTT),
      // bersihkan expected state agar tidak menggantung.
      if (matchesExpectedState(disp)) {
        clearExpectedState();
        lastActionSource = ACTION_NONE;
      }

      // Set temp change dalam range dianggap allowed (tidak di-resync),
      // baik automation ON maupun OFF.
      lastKnownSetTemp = disp.set_temp;
      if (mainboard.valid) {
        lastKnownCurrentTemp = mainboard.current_temp;
      }
      return;
    } else {
      Serial.Printf("[Monitor] Set temp changed: %d â†’ %d (Outside range [%d-%d])\n", 
                lastKnownSetTemp, newSetTemp, minTemp, maxTemp);
    }
  }

  if (!powerChanged && !setTempChanged && !modeChanged && !fanChanged) {
    if (mainboard.valid) {
      lastKnownCurrentTemp = mainboard.current_temp;
    }
    return;
  }

  unsigned long now = millis();
  
  Serial.Printf("[Monitor] Display Change: power=%d->%d, set_temp=%d->%d, mode=%s->%s, fan=%s->%s\n",
            lastKnownPower, disp.power,
            lastKnownSetTemp, disp.set_temp,
            lastKnownMode.c_str(), disp.mode.c_str(),
            lastKnownFan.c_str(), disp.fan.c_str());

  if (matchesExpectedState(disp)) {
    Serial.Println("[Monitor] âœ“ Change matches expected state (MQTT/Automation)");
    Serial.Println("[Monitor] â†’ No manual recovery triggered");
    clearExpectedState();
    lastActionSource = ACTION_NONE;
  }
  else if (isExpectedStateValid()) {
    Serial.Println("[Monitor] âš  Change does NOT match expected state");
    
    unsigned long timeSinceAction = now - lastActionMs;
    if (timeSinceAction <= ACTION_WINDOW_MS && lastActionSource != ACTION_NONE) {
      Serial.Printf("[Monitor] Within action window (%lums) but state mismatch\n", timeSinceAction);
      
      if (lastActionSource == ACTION_MQTT_REMOTE) {
        if (cfg.setting.manualModeActive) {
          Serial.Println("[Monitor] MQTT Remote action - keep attribution (manual recovery active)");
        } else {
          Serial.Println("[Monitor] MQTT Remote action - keep attribution");
        }
      }
      else if (lastActionSource == ACTION_AUTOMATION) {
        Serial.Println("[Monitor] Automation action - keep attribution");
      }
    }
    else {
      goto physical_remote_detected;
    }
  }
  else {
    unsigned long timeSinceAction = now - lastActionMs;
    if (timeSinceAction <= ACTION_WINDOW_MS && lastActionSource != ACTION_NONE) {
      if (lastActionSource == ACTION_MQTT_REMOTE) {
        if (cfg.setting.manualModeActive) {
          Serial.Println("[Monitor] Recent MQTT action detected - keep attribution (manual recovery active)");
        } else {
          Serial.Println("[Monitor] Recent MQTT action detected - keep attribution");
        }
      }
      else if (lastActionSource == ACTION_AUTOMATION) {
        Serial.Println("[Monitor] Recent Automation action detected - keep attribution");
      }
    }
    else {
      goto physical_remote_detected;
    }
  }

  lastKnownPower = disp.power;
  lastKnownSetTemp = disp.set_temp;
  lastKnownMode = disp.mode;
  lastKnownFan = disp.fan;
  if (mainboard.valid) {
    lastKnownCurrentTemp = mainboard.current_temp;
  }
  return;

physical_remote_detected:
  if (!cfg.setting.automation) {
    Serial.Println("[Monitor] Physical remote change detected. Automation OFF -> No action (but still track activity).");

    // Tetap track perubahan walaupun automation OFF
    if (powerChanged) {
      activityTrackPower(lastKnownPower, disp.power, SOURCE_PHYSICAL_REMOTE);
    }
    if (setTempChanged) {
      activityTrackTemp((int)lastKnownSetTemp, (int)disp.set_temp, SOURCE_PHYSICAL_REMOTE);
    }
    if (modeChanged) {
      activityTrackMode(lastKnownMode, disp.mode, SOURCE_PHYSICAL_REMOTE);
    }
    if (fanChanged) {
      activityTrackFan(lastKnownFan, disp.fan, SOURCE_PHYSICAL_REMOTE);
    }

    lastKnownPower = disp.power;
    lastKnownSetTemp = disp.set_temp;
    lastKnownMode = disp.mode;
    lastKnownFan = disp.fan;
    if (mainboard.valid) {
      lastKnownCurrentTemp = mainboard.current_temp;
    }
    return;
  }

  // Perubahan POWER tetap dianggap user override dan masuk manual mode recovery.
  if (!powerChanged) {
    // Jika hanya set_temp berubah dan nilainya di luar range, lakukan RESYNC
    // ke target automation (temp+mode+fan) agar konsisten.
    // Legacy range-only path is intentionally disabled; set_temp changes are
    // handled by the common non-power resync path below.
    if (false && setTempChanged && !modeChanged && !fanChanged) {
      const TimePeriod* tp = getActiveTimePeriod();
      uint8_t minTemp = resolveMinTemp(tp);
      uint8_t maxTemp = resolveMaxTemp(tp);
      uint8_t newSetTemp = disp.set_temp;

      if (newSetTemp < minTemp || newSetTemp > maxTemp) {
        Serial.Println("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
        Serial.Println("â•‘ âš ï¸ PHYSICAL REMOTE CHANGE DETECTED    â•‘");
        Serial.Println("â•‘ Source: IR Remote / Panel Button      â•‘");
        Serial.Println("â•‘ -> Set temp out-of-range: RESYNC AUTO â•‘");
        Serial.Printf("â•‘ Set Temp: %d â†’ %d (Range: %d-%d)    â•‘\n",
                  lastKnownSetTemp, disp.set_temp,
                  minTemp, maxTemp);
        Serial.Printf("â•‘ Restoring to: %dÂ°C                 â•‘\n", resolveSetTemp(tp));
        Serial.Println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

        // Track perubahan dari physical remote
        activityTrackTemp((int)lastKnownSetTemp, (int)disp.set_temp, SOURCE_PHYSICAL_REMOTE);

        // Update baseline ke state display saat ini.
        lastKnownPower = disp.power;
        lastKnownSetTemp = disp.set_temp;
        lastKnownMode = disp.mode;
        lastKnownFan = disp.fan;
        if (mainboard.valid) {
          lastKnownCurrentTemp = mainboard.current_temp;
        }

        // Kirim IR command untuk restore ke target automation.
        // Jangan paksa ON jika display menunjukkan AC OFF.
        if (disp.power) {
          String targetMode = resolveMode(tp);
          int targetSetTemp = resolveSetTemp(tp);
          String resyncFan = resolveFan(tp);
          lastActionMs = millis();
          lastActionSource = ACTION_AUTOMATION;
          applyTempModeFan(targetSetTemp, targetMode, resyncFan);
        } else {
          forceScheduleCheck = true;
        }
        return;
      }
    }

    Serial.Println("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
    Serial.Println("â•‘ âš ï¸ PHYSICAL REMOTE CHANGE DETECTED    â•‘");
    Serial.Println("â•‘ Source: IR Remote / Panel Button      â•‘");
    Serial.Println("â•‘ -> Non-power change: RESYNC AUTOMATIONâ•‘");
    
    const TimePeriod* tp = getActiveTimePeriod();
    if (setTempChanged) {
      Serial.Printf("â•‘ Set Temp: %d â†’ %d (Range: %d-%d)    â•‘\n",
                lastKnownSetTemp, disp.set_temp,
                resolveMinTemp(tp), resolveMaxTemp(tp));
    }
    if (modeChanged) {
      Serial.Printf("â•‘ Mode: %s â†’ %s                     â•‘\n",
                lastKnownMode.c_str(), disp.mode.c_str());
    }
    if (fanChanged) {
      Serial.Printf("â•‘ Fan: %s â†’ %s                      â•‘\n",
                lastKnownFan.c_str(), disp.fan.c_str());
    }
    Serial.Println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

    // Track perubahan dari physical remote (tanpa masuk manual mode)
    if (setTempChanged) {
      activityTrackTemp((int)lastKnownSetTemp, (int)disp.set_temp, SOURCE_PHYSICAL_REMOTE);
    }
    if (modeChanged && disp.power) {
      activityTrackMode(lastKnownMode, disp.mode, SOURCE_PHYSICAL_REMOTE);
    }
    if (fanChanged && disp.power) {
      activityTrackFan(lastKnownFan, disp.fan, SOURCE_PHYSICAL_REMOTE);
    }

    // Langsung resync ke target automation.
    String targetMode = resolveMode(tp);
    int targetSetTemp = resolveSetTemp(tp);
    String resyncFan = resolveFan(tp);

    // Update baseline agar tidak memicu berulang sebelum resync ter-apply
    lastKnownPower = disp.power;
    lastKnownSetTemp = disp.set_temp;
    lastKnownMode = disp.mode;
    lastKnownFan = disp.fan;
    if (mainboard.valid) {
      lastKnownCurrentTemp = mainboard.current_temp;
    }

    // Jangan paksa ON jika display menunjukkan OFF.
    // Jika OFF, biarkan automationCheck yang menentukan sesuai jadwal.
    if (disp.power) {
      lastActionMs = millis();
      lastActionSource = ACTION_AUTOMATION;
      applyTempModeFan(targetSetTemp, targetMode, resyncFan);
    } else {
      forceScheduleCheck = true;
    }

    return;
  }
  
  Serial.Println("â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—");
  Serial.Println("â•‘ âš ï¸ PHYSICAL REMOTE CHANGE DETECTED    â•‘");
  Serial.Println("â•‘ Source: IR Remote / Panel Button      â•‘");
  
  const TimePeriod* tp = getActiveTimePeriod();

  if (powerChanged) {
    Serial.Printf("â•‘ Power: %s â†’ %s                    â•‘\n", 
              lastKnownPower ? "ON" : "OFF", disp.power ? "ON" : "OFF");
  }
  if (setTempChanged) {
    Serial.Printf("â•‘ Set Temp: %d â†’ %d (Range: %d-%d)    â•‘\n", 
              lastKnownSetTemp, disp.set_temp, 
              resolveMinTemp(tp), resolveMaxTemp(tp));
  }
  if (modeChanged) {
    Serial.Printf("â•‘ Mode: %s â†’ %s                     â•‘\n", 
              lastKnownMode.c_str(), disp.mode.c_str());
  }
  if (fanChanged) {
    Serial.Printf("â•‘ Fan: %s â†’ %s                      â•‘\n", 
              lastKnownFan.c_str(), disp.fan.c_str());
  }
  
  Serial.Println("â•‘ -> Manual Recovery ACTIVE             â•‘");
  Serial.Println("â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•");

  // Track specific changes from physical remote
  if (powerChanged) {
    activityTrackPower(lastKnownPower, disp.power, SOURCE_PHYSICAL_REMOTE);
  }
  if (setTempChanged) {
    activityTrackTemp((int)lastKnownSetTemp, (int)disp.set_temp, SOURCE_PHYSICAL_REMOTE);
  }
  if (modeChanged && disp.power) {
    activityTrackMode(lastKnownMode, disp.mode, SOURCE_PHYSICAL_REMOTE);
  }
  if (fanChanged && disp.power) {
    activityTrackFan(lastKnownFan, disp.fan, SOURCE_PHYSICAL_REMOTE);
  }

  clearExpectedState();

  lastKnownPower = disp.power;
  lastKnownSetTemp = disp.set_temp;
  lastKnownMode = disp.mode;
  lastKnownFan = disp.fan;
  if (mainboard.valid) {
    lastKnownCurrentTemp = mainboard.current_temp;
  }

  bool shouldBeOn = false;
  computeAutomationShouldBeOnNow(shouldBeOn);

  if (shouldBeOn) {
    String targetMode = resolveMode(tp);
    int targetSetTemp = resolveSetTemp(tp);
    String resyncFan = resolveFan(tp);

    if (!disp.power) {
      activityTrackPower(false, true, SOURCE_AUTOMATION);
    }

    lastActionMs = millis();
    lastActionSource = ACTION_AUTOMATION;
    applyTempModeFan(targetSetTemp, targetMode, resyncFan);
  } else {
    if (disp.power) {
      activityTrackPower(true, false, SOURCE_AUTOMATION);
    }

    String targetMode = resolveMode(tp);
    int targetSetTemp = resolveSetTemp(tp);

    lastActionMs = millis();
    lastActionSource = ACTION_AUTOMATION;
    controlAC(false, targetMode, targetSetTemp);
  }

  forceScheduleCheck = true;
}

void automationInit() {
  Serial.Printf("[Auto] Init State: %s\n", cfg.setting.automation ? "ON" : "OFF");
  
  if (cfg.setting.lastManualControlMs == 0) {
    cfg.setting.lastManualControlMs = millis();
    cfg.setting.manualModeActive = false;
  }
  
  lastScheduleCheckMs = millis(); 
  forceScheduleCheck = true; 
}

void automationCheck() {
  unsigned long now = millis();

  // Jika target set_temp automation berubah, hapus override set_temp dari physical remote
  // agar target baru bisa diterapkan.
  int currentTargetTemp = resolveSetTemp(getActiveTimePeriod());
  if (lastConfigSetTemp < 0) {
    lastConfigSetTemp = currentTargetTemp;
  } else if (currentTargetTemp != lastConfigSetTemp) {
    allowedSetTempOverrideActive = false;
    allowedSetTempOverride = 0;
    lastConfigSetTemp = currentTargetTemp;
  }

  if (waitingForACResponse) {
    if (now - lastIRCommandMs < IR_SETTLE_TIME_MS) {
      return;
    }

    if (irVerifyStartMs > 0 && (now - irVerifyStartMs) > IR_VERIFY_TIMEOUT_MS) {
      Serial.Println("[Auto] âš ï¸ IR verification timeout");
      waitingForACResponse = false;
      irVerifyRetryCount = 0;
      irVerifyStartMs = 0;
      return;
    }
    
    bool actualState = getACPowerStatus();
    if (actualState == expectedACState) {
      Serial.Println("[Auto] AC state verified successfully");
      waitingForACResponse = false;
      irVerifyRetryCount = 0;
      irVerifyStartMs = 0;
    } else {
      if (irVerifyRetryCount < 3) {
        irVerifyRetryCount++;
        Serial.Printf("[Auto] State mismatch, retry %d/3\n", irVerifyRetryCount);
        lastIRCommandMs = now;
        return;
      } else {
        Serial.Println("[Auto] âš ï¸ AC not responding after 3 retries");
        waitingForACResponse = false;
        irVerifyRetryCount = 0;
        irVerifyStartMs = 0;
      }
    }
  }

  if (cfg.setting.manualModeActive) {
    unsigned long recoveryMs = (unsigned long)cfg.setting.auto_recovery_sec * 1000UL;
    if (now - cfg.setting.lastManualControlMs >= recoveryMs) {
      // Jika user secara eksplisit mematikan automation via MQTT,
      // jangan re-enable otomatis â€” hanya clear manual mode.
      if (cfg.setting.automationPausedByUser) {
        Serial.Println("[Auto] Recovery timer habis, tapi automation di-disable oleh user â€” tetap OFF");
        cfg.setting.manualModeActive = false;
        if (cfgSave()) {
          cfgApply();
          if (dataIsConnected()) dataPublishSetting();
        }
        return;
      }

      Serial.Println("[Auto] Manual recovery selesai, automation aktif kembali");
      
      activityTrackAutoRecovery();
      
      cfg.setting.manualModeActive = false;
      cfg.setting.automation = true;
      cfg.setting.automationPausedByUser = false;

      lastScheduleCheckMs = 0;
      scheduleStateInit = false;
      lastTempControlMs = 0;

      Serial.Println("[Auto] Restoring automation target state with optimal fan");
      
      const TimePeriod* tp = getActiveTimePeriod();
      String optimalFan = resolveFan(tp);
      int targetSetTemp = resolveSetTemp(tp);
      String targetMode = resolveMode(tp);
      
      BPFrame currentFrame = sensorGetLastFrame();
      
      if (currentFrame.valid) {
        if (!currentFrame.power) {
          activityTrackPower(false, true, SOURCE_AUTOMATION);
        }
        if (currentFrame.mode != targetMode) {
          activityTrackMode(currentFrame.mode, targetMode, SOURCE_AUTOMATION);
        }
        if (currentFrame.fan != optimalFan) {
          activityTrackFan(currentFrame.fan, optimalFan, SOURCE_AUTOMATION);
        }
      }
      
      controlAC(true, targetMode, targetSetTemp, optimalFan);
      
      waitingForACResponse = true;
      expectedACState = true;
      lastIRCommandMs = now;
      irVerifyStartMs = now;
      irVerifyRetryCount = 0;
      
      lastTempControlMs = 0;

      if (cfgSave()) {
        cfgApply();
        if (dataIsConnected()) {
          dataPublishSetting();
        }
      }

      Serial.Printf("[Auto] automation=%d manual=%d pausedByUser=%d\n",
        cfg.setting.automation, cfg.setting.manualModeActive, 
        cfg.setting.automationPausedByUser);
      
      forceScheduleCheck = true;
    }
    return;
  }
  bool isForceCheck = forceScheduleCheck;
  if (isForceCheck) {
    static unsigned long lastForceScheduleLogMs = 0;
    if (lastForceScheduleLogMs == 0 || (now - lastForceScheduleLogMs) >= 10000UL) {
      Serial.println("[Auto] Force schedule evaluation triggered");
      lastForceScheduleLogMs = now;
    }
    // Reset schedule check timer so it re-evaluates immediately,
    // but do NOT reset lastTempControlMs â€” temperature cooldown must be preserved.
    lastScheduleCheckMs = 0;
    scheduleStateInit = false;
  }

  if (!isForceCheck && (now - lastScheduleCheckMs < SCHEDULE_CHECK_INTERVAL_MS)) {
    return;
  }
  lastScheduleCheckMs = now;

  if (!cfg.setting.automation) {
    if (forceScheduleCheck) forceScheduleCheck = false;
    return;
  }

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  struct tm timeinfo;
  bool rtcOk = getLocalTime(&timeinfo);
  
  if (!wifiOk && !rtcOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 30000) {
      Serial.Println("[Auto] âš ï¸ WiFi dan RTC tidak tersedia - automation dimatikan");
      lastWarnMs = now;
    }
    if (forceScheduleCheck) forceScheduleCheck = false;
    return;
  } else if (!rtcOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 60000) {
      Serial.Println("[Auto] âš ï¸ RTC tidak tersedia, menggunakan NTP dari WiFi");
      lastWarnMs = now;
    }
  } else if (!wifiOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 60000) {
      Serial.Println("[Auto] âš ï¸ WiFi tidak tersedia, menggunakan RTC");
      lastWarnMs = now;
    }
  }

  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 30000) {
      Serial.Println("[Auto] âš ï¸ Gagal mendapatkan waktu - skip cycle");
      lastWarnMs = now;
    }
    return;
  }

  const TimePeriod* activePeriod = getActiveTimePeriod();
  DaySchedule &d = cfg.setting.daily[dayIdx];
  bool shouldBeOn = (activePeriod != nullptr);
  bool dryModeActive = shouldBeOn && isDryModeWindowActive(activePeriod);
  String scheduleTargetMode = dryModeActive ? String("DRY") : resolveMode(activePeriod);
  String scheduleTargetFan = dryModeActive ? String("") : resolveFan(activePeriod);
  int scheduleTargetTemp = dryModeActive ? -1 : resolveSetTemp(activePeriod);
  bool targetChanged = shouldBeOn &&
                       scheduleTargetChanged(scheduleTargetMode,
                                             scheduleTargetFan,
                                             scheduleTargetTemp);
  if (dryModeActive != lastDryModeWindowActive) {
    Serial.Printf("[Auto] DRY startup window: %s\n", dryModeActive ? "ACTIVE" : "INACTIVE");
    lastDryModeWindowActive = dryModeActive;
  }
  
  if (!logStateInit || shouldBeOn != lastLoggedShouldBeOn) {
    Serial.Printf("[Auto] DayIdx=%d (Day=%d), Type=%d, ShouldBeOn=%s\n",
      dayIdx, DAY_IDX_TO_NUM(dayIdx), d.type, shouldBeOn ? "YES" : "NO");
    lastLoggedShouldBeOn = shouldBeOn;
    logStateInit = true;
  }

  if (shouldBeOn) {
    BPFrame currentFrame = sensorGetLastFrame();
    
    if (!currentFrame.valid) {
      static unsigned long lastWaitValidFrameLogMs = 0;
      if (lastWaitValidFrameLogMs == 0 || (now - lastWaitValidFrameLogMs) >= 10000UL) {
        Serial.println("[Auto] â³ Waiting for valid frame... No IR command sent");
        lastWaitValidFrameLogMs = now;
      }
      return;
    }
    
    if (!getACPowerStatus()) {
       Serial.Println("[Auto] Schedule ON -> Turning AC ON with intelligent fan");
       
       // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       // INCREMENT COUNTER untuk automation ON
       // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       autoOnCount++;
       Serial.Printf("[Auto] Auto ON Count: %lu\n", autoOnCount);
       
       // Track activity before executing
       if (currentFrame.valid && !currentFrame.power) {
         activityTrackPower(false, true, SOURCE_AUTOMATION);
       }
       
      if (dryModeActive) {
        applyModeOnly("DRY");
      } else {
        controlAC(true, scheduleTargetMode, scheduleTargetTemp, scheduleTargetFan);
      }
       rememberScheduleTarget(scheduleTargetMode, scheduleTargetFan, scheduleTargetTemp);
       lastSchedulePower = true;
       lastActionMs = millis();
       lastActionSource = ACTION_AUTOMATION;
       
       waitingForACResponse = true;
       expectedACState = true;
       lastIRCommandMs = now;
      irVerifyStartMs = now;
      irVerifyRetryCount = 0;
    }
    else {
       BPFrame lastFrame = sensorGetLastFrame();
      String currentMode = lastFrame.mode;
       bool modeMismatch = (currentMode != scheduleTargetMode);
       
       if (isForceCheck || modeMismatch || targetChanged) {
           Serial.Println("[Auto] Syncing State (Mode/Temp config changed)...");
           Serial.Printf("[Auto] Target: mode=%s fan=%s temp=%d\n",
                         scheduleTargetMode.c_str(),
                         scheduleTargetFan.c_str(),
                         scheduleTargetTemp);
           
           if (dryModeActive) {
             applyModeOnly("DRY");
           } else {
             controlAC(true,
                       scheduleTargetMode,
                       scheduleTargetTemp,
                       scheduleTargetFan);
           }
           
           waitingForACResponse = true;
           expectedACState = true;
           lastIRCommandMs = now;
           irVerifyStartMs = now;
           irVerifyRetryCount = 0;
       }
       rememberScheduleTarget(scheduleTargetMode, scheduleTargetFan, scheduleTargetTemp);
       lastSchedulePower = true;
    }
  } 
  else {
    BPFrame currentFrame = sensorGetLastFrame();
    
    if (!currentFrame.valid) {
      static unsigned long lastWaitValidFrameLogMs = 0;
      if (lastWaitValidFrameLogMs == 0 || (now - lastWaitValidFrameLogMs) >= 10000UL) {
        Serial.println("[Auto] â³ Waiting for valid frame... No IR command sent");
        lastWaitValidFrameLogMs = now;
      }
      return;
    }
    
    if (getACPowerStatus()) {
       Serial.Println("[Auto] Schedule OFF -> Turning AC OFF");
       
       // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       // INCREMENT COUNTER untuk automation OFF
       // â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
       autoOffCount++;
       Serial.Printf("[Auto] Auto OFF Count: %lu\n", autoOffCount);
       
       // Track activity before executing
       if (currentFrame.valid && currentFrame.power) {
         activityTrackPower(true, false, SOURCE_AUTOMATION);
       }
       
       const TimePeriod* tp = getActiveTimePeriod();
       controlAC(false, resolveMode(tp), resolveSetTemp(tp));
       
       waitingForACResponse = true;
       expectedACState = false;
       lastIRCommandMs = now;
      irVerifyStartMs = now;
      irVerifyRetryCount = 0;
       lastActionMs = millis();
       lastActionSource = ACTION_AUTOMATION;
    }
    lastScheduleTargetValid = false;
    lastSchedulePower = false;
  }

  // Temperature/fan maintenance is intentionally disabled. Automation follows
  // the active schedule only and must not alter set_temp or fan speed from
  // room temperature measurements.

  if (!scheduleStateInit) scheduleStateInit = true;
  if (forceScheduleCheck) forceScheduleCheck = false;
}

void automationSetExpectedStateFromMqtt(bool power, const String& mode, const String& fan, uint8_t setTemp) {
  // Guard with mutex â€” this may be called from NetworkTask while AutoTask reads the same state.
  // String assignments are NOT atomic and can corrupt without synchronization.
  if (g_cfgMutex && CFG_LOCK() == pdTRUE) {
    lastActionMs = millis();
    lastActionSource = ACTION_MQTT_REMOTE;
    setExpectedState(power, mode, fan, setTemp);
    CFG_UNLOCK();
  }
}

void automationClearExpectedStateFromMqtt() {
  if (g_cfgMutex && CFG_LOCK() == pdTRUE) {
    clearExpectedState();
    CFG_UNLOCK();
  }
}
