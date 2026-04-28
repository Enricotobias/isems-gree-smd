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

static bool lastAutomationState = false;

static String normalizeFanSpeed(const String& fan) {
  String normalized = fan;
  normalized.toUpperCase();
  normalized.trim();
  
  if (normalized == "MED") normalized = "MEDIUM";
  if (normalized == "MIN") normalized = "LOW";
  if (normalized == "MAX") normalized = "HIGH";
  
  return normalized;
}

static unsigned long lastRecoveryCheckMs = 0;
const unsigned long RECOVERY_CHECK_INTERVAL_MS = 5000UL;
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

// Counter untuk tracking cooling attempts
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
static bool isSystemAction = false;
static bool firstRunMonitor = true;
static bool systemActionFromAutomation = false;
static bool systemActionFromMqtt = false;

static bool lastLoggedShouldBeOn = false;
static bool logStateInit = false;

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

static bool isRecentSystemAction() {
  return (lastActionSource != ACTION_NONE) &&
         (millis() - lastActionMs < 1500);
}

static bool getACPowerStatus() {
  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    return disp.power;
  }
  BPFrame frame = sensorGetLastFrame();
  return sensorIsHealthy() ? frame.power : false;
}

static int getDayOfWeek() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return -1;
  int day = timeinfo.tm_wday - 1;
  if (day < 0) day = 6;
  return day;
}

static void getCurrentHourMinute(int &hour, int &minute) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    hour = -1; minute = -1; return;
  }
  hour = timeinfo.tm_hour;
  minute = timeinfo.tm_min;
}

static bool parseTime(const String& timeStr, int &hour, int &minute) {
  if (timeStr.length() != 5 || timeStr.charAt(2) != ':') return false;
  hour = timeStr.substring(0, 2).toInt();
  minute = timeStr.substring(3, 5).toInt();
  return (hour >= 0 && hour < 24 && minute >= 0 && minute < 60);
}

static bool isWithinOperatingHours(const String& mulai, const String& selesai) {
  int curH, curM;
  getCurrentHourMinute(curH, curM);
  if (curH < 0) return false;
  
  int sH, sM, eH, eM;
  if (!parseTime(mulai, sH, sM)) return false;
  if (!parseTime(selesai, eH, eM)) return false;
  
  int curMin = curH * 60 + curM;
  int sMin = sH * 60 + sM;
  int eMin = eH * 60 + eM;
  
  if (eMin < sMin) return (curMin >= sMin || curMin <= eMin);
  return (curMin >= sMin && curMin < eMin);
}

static const TimePeriod* getActiveTimePeriod() {
  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) return nullptr;

  DaySchedule& d = cfg.setting.daily[dayIdx];
  if (d.type == DAY_OFFTIME) return nullptr;
  if (d.type == DAY_FULLTIME) {
    return (d.periodCount > 0) ? &d.periods[0] : nullptr;
  }

  int curH, curM;
  getCurrentHourMinute(curH, curM);
  if (curH < 0) return nullptr;
  int curMin = curH * 60 + curM;

  uint8_t cnt = d.periodCount;
  if (cnt > MAX_PERIODS_PER_DAY) cnt = MAX_PERIODS_PER_DAY;
  for (uint8_t p = 0; p < cnt; p++) {
    if (d.periods[p].start[0] == '\0' || d.periods[p].end[0] == '\0') continue;
    int sH = ((d.periods[p].start[0] - '0') * 10) + (d.periods[p].start[1] - '0');
    int sM = ((d.periods[p].start[3] - '0') * 10) + (d.periods[p].start[4] - '0');
    int eH = ((d.periods[p].end[0] - '0') * 10) + (d.periods[p].end[1] - '0');
    int eM = ((d.periods[p].end[3] - '0') * 10) + (d.periods[p].end[4] - '0');
    int sMin = sH * 60 + sM;
    int eMin = eH * 60 + eM;
    bool inRange = (eMin < sMin) ? (curMin >= sMin || curMin < eMin) : (curMin >= sMin && curMin < eMin);
    if (inRange) return &d.periods[p];
  }

  return nullptr;
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
  return (tp && tp->fan[0] != '\0') ? String(tp->fan) : fallback;
}

static String selectOptimalFanSpeed(int currentTemp, int targetTemp) {
  int tempDiff = currentTemp - targetTemp;
  
  Serial.Printf("[Auto] Temperature Analysis: Current=%d, Target=%d, Diff=%d\n", 
            currentTemp, targetTemp, tempDiff);
  
  String fanSpeed;
  
  if (tempDiff < -1) {
    fanSpeed = "LOW";
    Serial.Printf("[Auto] Temp %d°C below target → Fan: LOW (ultra silent)\n", abs(tempDiff));
  }
  else if (tempDiff <= 0) {
    fanSpeed = "LOW";
    Serial.Println("[Auto] Temp at/near target → Fan: LOW (energy saving)");
  }
  else if (tempDiff == 1) {
    fanSpeed = "HIGH";
    Serial.Println("[Auto] Temp 1°C above target → Fan: HIGH (aggressive cooling)");
  }
  else {
    fanSpeed = "TURBO";
    Serial.Printf("[Auto] Temp %d°C above target → Fan: TURBO (maximum cooling)\n", tempDiff);
  }
  
  return fanSpeed;
}

static bool isIntelligentFanChange(const String& oldFan, const String& newFan, 
                                   int currentTemp, int targetTemp) {
  if (resolveFan(getActiveTimePeriod()) != "AUTO") {
    return false;
  }
  
  String expectedFan = selectOptimalFanSpeed(currentTemp, targetTemp);
  
  // Normalize untuk comparison
  String normalizedNew = newFan;
  normalizedNew.toUpperCase();
  normalizedNew.trim();
  
  String normalizedExpected = expectedFan;
  normalizedExpected.toUpperCase();
  normalizedExpected.trim();
  
  return (normalizedNew == normalizedExpected);
}

static void trackAndExecute(
  std::function<void()> executeFunc,
  const BPFrame& expectedNewState,
  ActivitySource source = SOURCE_AUTOMATION
) {
  BPFrame oldState = sensorGetLastFrame();
  bool oldStateFresh = sensorIsHealthy();
  
  executeFunc();
  
  delay(500);
  
  if (oldStateFresh) {
    activityTrackChange(oldState, expectedNewState, source);
  }
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

static void applyTempAndMode(int temp, const String& mode) {
  Serial.Printf("[Auto] Set %d°C, mode=%s\n", temp, mode.c_str());

  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    if ((int)disp.set_temp != temp) {
      activityTrackTemp((int)disp.set_temp, temp, SOURCE_AUTOMATION);
    }
  }
  
  BPFrame currentFrame = sensorGetLastFrame();
  if (sensorIsHealthy() && currentFrame.mode != mode) {
    activityTrackMode(currentFrame.mode, mode, SOURCE_AUTOMATION);
  }

  lastActionMs = millis();
  lastActionSource = ACTION_AUTOMATION;

  // Command ini tidak mengirim fan, jadi jangan memvalidasi fan.
  setExpectedState(true, mode, "", (uint8_t)temp);
  
  StaticJsonDocument<256> doc;
  doc["power"] = true; doc["temp"] = temp; doc["mode"] = mode;
  String jsonStr; serializeJson(doc, jsonStr);
  controllEnqueueCommand(jsonStr.c_str(), jsonStr.length());
}

static void applyTempModeFan(int temp, const String& mode, const String& fan) {
  Serial.Printf("[Auto] Set %d°C, mode=%s, fan=%s\n", temp, mode.c_str(), fan.c_str());

  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    if ((int)disp.set_temp != temp) {
      activityTrackTemp((int)disp.set_temp, temp, SOURCE_AUTOMATION);
    }
  }

  lastActionMs = millis();
  lastActionSource = ACTION_AUTOMATION;
  
  setExpectedState(true, mode, fan, (uint8_t)temp);
  
  StaticJsonDocument<256> doc;
  doc["power"] = true; doc["temp"] = temp; doc["mode"] = mode; doc["fan"] = fan;
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
        fanSpeed = "AUTO";
        Serial.Println("[Auto] No target fan found, using AUTO");
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

static void checkAndFixMode() {
  BPFrame frame = sensorGetLastFrame();
  if (!frame.valid) return;
  
  const TimePeriod* tp = getActiveTimePeriod();
  if (!tp) return;
  
  String cMode = frame.mode; 
  String kMode = resolveMode(tp);
  cMode.toUpperCase(); kMode.toUpperCase();
  
  if (cMode != kMode) applyModeOnly(kMode);
}

// =================================================================
// [NEW] TEMPERATURE MAINTENANCE - SIMPLIFIED LOGIC
// Alur:
// 1. Suhu dalam range (min_temp <= suhu <= max_temp) → No action
// 2. Suhu > max_temp → Set temp, fan HIGH, 3x retry lalu temp-2
// 3. Suhu < min_temp → Set temp, fan LOW
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

  int currentTemp = frame.current_temp;
  int setTemp = resolveSetTemp(tp);
  int maxTemp = resolveMaxTemp(tp);
  int minTemp = resolveMinTemp(tp);
  String targetMode = resolveMode(tp);
  String targetFan = resolveFan(tp);

  // ═════════════════════════════════════════════════════════════
  // CASE 1: SUHU DALAM RANGE → CHECK FAN SPEED
  // ═════════════════════════════════════════════════════════════
  if (currentTemp >= minTemp && currentTemp <= maxTemp) {
    // Reset counter jika suhu kembali normal
    if (coolingAttemptCount > 0) {
      Serial.Println("[Auto] Temperature back to normal range, resetting counter");
      coolingAttemptCount = 0;
    }
    
    // Check if fan speed matches default_fan setting
    String currentFan = frame.fan;
    String defaultFan = targetFan;
    currentFan.toUpperCase();
    defaultFan.toUpperCase();
    currentFan.trim();
    defaultFan.trim();
    
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

  // ═════════════════════════════════════════════════════════════
  // CASE 2: SUHU > MAX_TEMP → COOLING MODE
  // ═════════════════════════════════════════════════════════════
  if (currentTemp > maxTemp) {
    coolingAttemptCount++;
    Serial.Printf("[Auto] Too hot: %d°C > %d°C (attempt %d/3)\n", 
              currentTemp, maxTemp, coolingAttemptCount);
    
    lastTempControlMs = now;
    
    // Cek apakah sudah 3x attempt
    if (coolingAttemptCount >= 3) {
      int newSetTemp = setTemp - 2;
      if (newSetTemp < 16) newSetTemp = 16; // Min AC limit
      
      Serial.Printf("[Auto] 3x cooling failed, lowering setpoint: %d°C → %d°C\n", 
                setTemp, newSetTemp);
      
      if (frame.fan != "MEDIUM") {
        activityTrackFan(frame.fan, "MEDIUM", SOURCE_AUTOMATION);
      }
      applyTempModeFan(newSetTemp, targetMode, "MEDIUM");
      
      // Reset counter setelah menurunkan setpoint
      coolingAttemptCount = 0;
    } else {
      // Normal cooling: set_temp, fan HIGH
      Serial.Printf("[Auto] Cooling action: %d°C, fan=HIGH\n", setTemp);
      
      if (frame.fan != "MEDIUM") {
        activityTrackFan(frame.fan, "MEDIUM", SOURCE_AUTOMATION);
      }
      applyTempModeFan(setTemp, targetMode, "MEDIUM");
    }
    
    return;
  }

  // ═════════════════════════════════════════════════════════════
  // CASE 3: SUHU < MIN_TEMP → WARMING MODE
  // ═════════════════════════════════════════════════════════════
  if (currentTemp < minTemp) {
    // Reset cooling counter
    coolingAttemptCount = 0;
    
    Serial.Printf("[Auto] Too cold: %d°C < %d°C\n", currentTemp, minTemp);
    
    lastTempControlMs = now;
    
    // Set ke set_temp dengan fan LOW
    Serial.Printf("[Auto] Warming action: %d°C, fan=LOW\n", setTemp);
    
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
      Serial.Printf("[Monitor-BP] Power Change: %s → %s (Display invalid)\n",
                lastKnownBpPower ? "ON" : "OFF", mainboard.power ? "ON" : "OFF");
    } else {
      Serial.Printf("[Monitor-BP] Non-power change (Display invalid): mode=%s→%s fan=%s→%s\n",
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
        Serial.Println("[Monitor-BP] ✓ Change matches expected state (MQTT/Automation)");
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
      Serial.Println("╔═══════════════════════════════════════╗");
      Serial.Println("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
      Serial.Println("║ Source: IR Remote / Panel Button      ║");
      Serial.Println("║ (Display invalid) -> RESYNC AUTOMATION║");
      if (bpModeChanged) {
        Serial.Printf("║ Mode(BP): %s → %s                 ║\n",
                  lastKnownBpMode.c_str(), mainboard.mode.c_str());
      }
      if (bpFanChanged) {
        Serial.Printf("║ Fan(BP): %s → %s                  ║\n",
                  lastKnownBpFan.c_str(), mainboard.fan.c_str());
      }
      Serial.Println("╚═══════════════════════════════════════╝");

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

      if (resyncFan == "AUTO" && mainboard.valid) {
        resyncFan = selectOptimalFanSpeed(mainboard.current_temp, targetSetTemp);
      }

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
    Serial.Println("╔═══════════════════════════════════════╗");
    Serial.Println("║ ⚠️ PHYSICAL REMOTE POWER DETECTED     ║");
    Serial.Println("║ Source: IR Remote / Panel Button      ║");
    Serial.printf("║ Power: %s → %s                    ║\n",
            lastKnownBpPower ? "ON" : "OFF", mainboard.power ? "ON" : "OFF");
    Serial.Println("║ -> Manual Recovery ACTIVE             ║");
    Serial.Println("╚═══════════════════════════════════════╝");

    manualOverrideCount++;
    Serial.Printf("[Auto] Manual Override Count: %lu\n", manualOverrideCount);
    activityTrackPower(lastKnownBpPower, mainboard.power, SOURCE_PHYSICAL_REMOTE);

    cfg.setting.automation = false;
    cfg.setting.manualModeActive = true;
    cfg.setting.lastManualControlMs = now;

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

  if (fanChanged && !powerChanged && !modeChanged && !setTempChanged) {
    // Check if this fan change matches intelligent control expectation
    int currentTemp = mainboard.valid ? mainboard.current_temp : lastKnownCurrentTemp;
    int targetTemp = resolveSetTemp(getActiveTimePeriod());
    bool isIntelligentChange = isIntelligentFanChange(
      lastKnownFan, 
      disp.fan, 
      currentTemp, 
      targetTemp
    );
    
    if (isIntelligentChange) {
      Serial.Printf("[Monitor] Fan changed: %s → %s (Intelligent Control - Ignored)\n", 
                lastKnownFan.c_str(), disp.fan.c_str());
      
      lastKnownFan = disp.fan;
      fanChanged = false;
    }
  }

  if (setTempChanged && !powerChanged && !modeChanged && !fanChanged) {
    const TimePeriod* tp = getActiveTimePeriod();
    uint8_t minTemp = resolveMinTemp(tp);
    uint8_t maxTemp = resolveMaxTemp(tp);
    uint8_t newSetTemp = disp.set_temp;
    
    if (newSetTemp >= minTemp && newSetTemp <= maxTemp) {
      Serial.Printf("[Monitor] Set temp changed: %d → %d (Within range [%d-%d] - Allowed)\n", 
                lastKnownSetTemp, newSetTemp, minTemp, maxTemp);
      Serial.Println("[Monitor] → No manual recovery triggered");

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
      Serial.Printf("[Monitor] Set temp changed: %d → %d (Outside range [%d-%d])\n", 
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
    Serial.Println("[Monitor] ✓ Change matches expected state (MQTT/Automation)");
    Serial.Println("[Monitor] → No manual recovery triggered");
    clearExpectedState();
    lastActionSource = ACTION_NONE;
  }
  else if (isExpectedStateValid()) {
    Serial.Println("[Monitor] ⚠ Change does NOT match expected state");
    
    unsigned long timeSinceAction = now - lastActionMs;
    if (timeSinceAction <= ACTION_WINDOW_MS && lastActionSource != ACTION_NONE) {
      Serial.Printf("[Monitor] Within action window (%lums) but state mismatch\n", timeSinceAction);
      
      if (lastActionSource == ACTION_MQTT_REMOTE) {
        if (cfg.setting.manualModeActive) {
          Serial.Println("[Monitor] ✓ MQTT Remote action (Manual recovery active)");
        } else {
          Serial.Println("[Monitor] ✓ MQTT Remote action (No manual recovery)");
        }
        clearExpectedState();
        lastActionSource = ACTION_NONE;
      }
      else if (lastActionSource == ACTION_AUTOMATION) {
        Serial.Println("[Monitor] ✓ Automation action - No manual recovery");
        clearExpectedState();
        lastActionSource = ACTION_NONE;
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
          Serial.Println("[Monitor] ✓ Recent MQTT action detected (Manual recovery active)");
        } else {
          Serial.Println("[Monitor] ✓ Recent MQTT action detected (No manual recovery)");
        }
        lastActionSource = ACTION_NONE;
      }
      else if (lastActionSource == ACTION_AUTOMATION) {
        Serial.Println("[Monitor] ✓ Recent Automation action detected - No manual recovery");
        lastActionSource = ACTION_NONE;
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
    if (setTempChanged && !modeChanged && !fanChanged) {
      const TimePeriod* tp = getActiveTimePeriod();
      uint8_t minTemp = resolveMinTemp(tp);
      uint8_t maxTemp = resolveMaxTemp(tp);
      uint8_t newSetTemp = disp.set_temp;

      if (newSetTemp < minTemp || newSetTemp > maxTemp) {
        Serial.Println("╔═══════════════════════════════════════╗");
        Serial.Println("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
        Serial.Println("║ Source: IR Remote / Panel Button      ║");
        Serial.Println("║ -> Set temp out-of-range: RESYNC AUTO ║");
        Serial.Printf("║ Set Temp: %d → %d (Range: %d-%d)    ║\n",
                  lastKnownSetTemp, disp.set_temp,
                  minTemp, maxTemp);
        Serial.Printf("║ Restoring to: %d°C                 ║\n", resolveSetTemp(tp));
        Serial.Println("╚═══════════════════════════════════════╝");

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

          if (resyncFan == "AUTO" && mainboard.valid) {
            resyncFan = selectOptimalFanSpeed(mainboard.current_temp, targetSetTemp);
          }
          lastActionMs = millis();
          lastActionSource = ACTION_AUTOMATION;
          applyTempModeFan(targetSetTemp, targetMode, resyncFan);
        } else {
          forceScheduleCheck = true;
        }
        return;
      }
    }

    Serial.Println("╔═══════════════════════════════════════╗");
    Serial.Println("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
    Serial.Println("║ Source: IR Remote / Panel Button      ║");
    Serial.Println("║ -> Non-power change: RESYNC AUTOMATION║");
    
    const TimePeriod* tp = getActiveTimePeriod();
    if (setTempChanged) {
      Serial.Printf("║ Set Temp: %d → %d (Range: %d-%d)    ║\n",
                lastKnownSetTemp, disp.set_temp,
                resolveMinTemp(tp), resolveMaxTemp(tp));
    }
    if (modeChanged) {
      Serial.Printf("║ Mode: %s → %s                     ║\n",
                lastKnownMode.c_str(), disp.mode.c_str());
    }
    if (fanChanged) {
      Serial.Printf("║ Fan: %s → %s                      ║\n",
                lastKnownFan.c_str(), disp.fan.c_str());
    }
    Serial.Println("╚═══════════════════════════════════════╝");

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
    
    if (resyncFan == "AUTO" && mainboard.valid) {
      resyncFan = selectOptimalFanSpeed(mainboard.current_temp, targetSetTemp);
    }

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
  
  Serial.Println("╔═══════════════════════════════════════╗");
  Serial.Println("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
  Serial.Println("║ Source: IR Remote / Panel Button      ║");
  
  const TimePeriod* tp = getActiveTimePeriod();

  if (powerChanged) {
    Serial.Printf("║ Power: %s → %s                    ║\n", 
              lastKnownPower ? "ON" : "OFF", disp.power ? "ON" : "OFF");
  }
  if (setTempChanged) {
    Serial.Printf("║ Set Temp: %d → %d (Range: %d-%d)    ║\n", 
              lastKnownSetTemp, disp.set_temp, 
              resolveMinTemp(tp), resolveMaxTemp(tp));
  }
  if (modeChanged) {
    Serial.Printf("║ Mode: %s → %s                     ║\n", 
              lastKnownMode.c_str(), disp.mode.c_str());
  }
  if (fanChanged) {
    Serial.Printf("║ Fan: %s → %s                      ║\n", 
              lastKnownFan.c_str(), disp.fan.c_str());
  }
  
  Serial.Println("║ -> Manual Recovery ACTIVE             ║");
  Serial.Println("╚═══════════════════════════════════════╝");

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
    
    if (resyncFan == "AUTO" && mainboard.valid) {
      resyncFan = selectOptimalFanSpeed(mainboard.current_temp, targetSetTemp);
    }

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

static void checkAutoRecovery() {
  if (!cfg.setting.manualModeActive) return;

  unsigned long now = millis();
  if (now - lastRecoveryCheckMs < RECOVERY_CHECK_INTERVAL_MS) return;
  lastRecoveryCheckMs = now;

  unsigned long recoveryMs = (unsigned long)cfg.setting.auto_recovery_sec * 1000UL; 

  unsigned long elapsedMs = now - cfg.setting.lastManualControlMs;

  if (elapsedMs >= recoveryMs) return;
  
  long remMs = recoveryMs - elapsedMs;
  if (remMs < 0) remMs = 0;

  unsigned long rSec = remMs / 1000UL;

  static unsigned long lastLogMs = 0;
  if (now - lastLogMs >= 10000 || rSec <= 10) {
    Serial.Printf("[Auto-Recovery] ⏳ Manual mode - Recovery in %lu:%02lu\n", rSec / 60, rSec % 60);
    lastLogMs = now;
  }
}

void automationInit() {
  Serial.Printf("[Auto] Init State: %s\n", cfg.setting.automation ? "ON" : "OFF");
  
  if (cfg.setting.lastManualControlMs == 0) {
    cfg.setting.lastManualControlMs = millis();
    cfg.setting.manualModeActive = false;
  }
  
  lastAutomationState = cfg.setting.automation;
  lastRecoveryCheckMs = millis();
  lastScheduleCheckMs = millis(); 
  forceScheduleCheck = true; 
}

void automationCheck() {
  checkAutoRecovery();
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
      Serial.Println("[Auto] ⚠️ IR verification timeout");
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
        Serial.Println("[Auto] ⚠️ AC not responding after 3 retries");
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
      // jangan re-enable otomatis — hanya clear manual mode.
      if (cfg.setting.automationPausedByUser) {
        Serial.Println("[Auto] Recovery timer habis, tapi automation di-disable oleh user — tetap OFF");
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
      
      if (optimalFan == "AUTO" && currentFrame.valid) {
        optimalFan = selectOptimalFanSpeed(currentFrame.current_temp, targetSetTemp);
      }
      
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
    lastTempControlMs = 0;
    lastScheduleCheckMs = 0;
    scheduleStateInit = false;
  }

  if (!isForceCheck && (now - lastScheduleCheckMs < SCHEDULE_CHECK_INTERVAL_MS)) {
    return;
  }
  lastScheduleCheckMs = now;

  if (!cfg.setting.automation) {
    lastTempControlMs = 0;
    if (forceScheduleCheck) forceScheduleCheck = false;
    return;
  }

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  struct tm timeinfo;
  bool rtcOk = getLocalTime(&timeinfo);
  
  if (!wifiOk && !rtcOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 30000) {
      Serial.Println("[Auto] ⚠️ WiFi dan RTC tidak tersedia - automation dimatikan");
      lastWarnMs = now;
    }
    lastTempControlMs = 0;
    if (forceScheduleCheck) forceScheduleCheck = false;
    return;
  } else if (!rtcOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 60000) {
      Serial.Println("[Auto] ⚠️ RTC tidak tersedia, menggunakan NTP dari WiFi");
      lastWarnMs = now;
    }
  } else if (!wifiOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 60000) {
      Serial.Println("[Auto] ⚠️ WiFi tidak tersedia, menggunakan RTC");
      lastWarnMs = now;
    }
  }

  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 30000) {
      Serial.Println("[Auto] ⚠️ Gagal mendapatkan waktu - skip cycle");
      lastWarnMs = now;
    }
    return;
  }

  const TimePeriod* activePeriod = getActiveTimePeriod();
  DaySchedule &d = cfg.setting.daily[dayIdx];
  bool shouldBeOn = (activePeriod != nullptr);
  
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
        Serial.println("[Auto] ⏳ Waiting for valid frame... No IR command sent");
        lastWaitValidFrameLogMs = now;
      }
      return;
    }
    
    if (!getACPowerStatus()) {
       Serial.Println("[Auto] Schedule ON -> Turning AC ON with intelligent fan");
       
       // ═══════════════════════════════════════════════
       // INCREMENT COUNTER untuk automation ON
       // ═══════════════════════════════════════════════
       autoOnCount++;
       Serial.Printf("[Auto] Auto ON Count: %lu\n", autoOnCount);
       
       String startupFan = resolveFan(activePeriod);
       String startupMode = resolveMode(activePeriod);
       int startupSetTemp = resolveSetTemp(activePeriod);

       if (startupFan == "AUTO" && currentFrame.valid) {
         startupFan = selectOptimalFanSpeed(currentFrame.current_temp, startupSetTemp);
       }
       
       // Track activity before executing
       if (currentFrame.valid && !currentFrame.power) {
         activityTrackPower(false, true, SOURCE_AUTOMATION);
       }
       
      controlAC(true, startupMode, startupSetTemp, startupFan);
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
      String targetMode = resolveMode(activePeriod);
       
       bool modeMismatch = (currentMode != targetMode);
       
       if (isForceCheck || modeMismatch) {
           Serial.Println("[Auto] Syncing State (Mode/Temp config changed)...");
           Serial.Printf("[Auto] Mode: %s -> %s\n", currentMode.c_str(), targetMode.c_str());
           
           String syncFan = resolveFan(activePeriod);
           int syncSetTemp = resolveSetTemp(activePeriod);
           
           if (syncFan == "AUTO" && lastFrame.valid) {
             syncFan = selectOptimalFanSpeed(lastFrame.current_temp, syncSetTemp);
           }
           
           controlAC(true, targetMode, syncSetTemp, syncFan);
           
           waitingForACResponse = true;
           expectedACState = true;
           lastIRCommandMs = now;
           irVerifyStartMs = now;
           irVerifyRetryCount = 0;
       }
       lastSchedulePower = true;
    }
  } 
  else {
    BPFrame currentFrame = sensorGetLastFrame();
    
    if (!currentFrame.valid) {
      static unsigned long lastWaitValidFrameLogMs = 0;
      if (lastWaitValidFrameLogMs == 0 || (now - lastWaitValidFrameLogMs) >= 10000UL) {
        Serial.println("[Auto] ⏳ Waiting for valid frame... No IR command sent");
        lastWaitValidFrameLogMs = now;
      }
      return;
    }
    
    if (getACPowerStatus()) {
       Serial.Println("[Auto] Schedule OFF -> Turning AC OFF");
       
       // ═══════════════════════════════════════════════
       // INCREMENT COUNTER untuk automation OFF
       // ═══════════════════════════════════════════════
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
    lastSchedulePower = false;
  }

  if (shouldBeOn && !waitingForACResponse) {
    checkAndMaintainTemperature();
  }

  if (!scheduleStateInit) scheduleStateInit = true;
  if (forceScheduleCheck) forceScheduleCheck = false;
}

void automationSetExpectedStateFromMqtt(bool power, const String& mode, const String& fan, uint8_t setTemp) {
  // Guard with mutex — this may be called from NetworkTask while AutoTask reads the same state.
  // String assignments are NOT atomic and can corrupt without synchronization.
  if (g_cfgMutex && CFG_LOCK() == pdTRUE) {
    lastActionMs = millis();
    lastActionSource = ACTION_MQTT_REMOTE;
    setExpectedState(power, mode, fan, setTemp);
    CFG_UNLOCK();
  } else {
    // Fallback without lock
    lastActionMs = millis();
    lastActionSource = ACTION_MQTT_REMOTE;
    setExpectedState(power, mode, fan, setTemp);
  }
}