#include "automation.h"
#include "config.h"
#include "controll.h"
#include "crash_monitor.h"
#include "sensor.h"
#include "data.h"
#include <time.h>
#include <ArduinoJson.h>
#include "logger.h"
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

bool forceScheduleCheck = false;

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
// Namun override akan dibersihkan otomatis saat target automation (cfg.setting.set_temp) berubah.
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

  logPrintf(
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
    logPrintf("[Expected] Power mismatch: expected=%s got=%s\n",
              expectedPower ? "ON" : "OFF", disp.power ? "ON" : "OFF");
    return false;
  }
  
  if (expectedModeValid && disp.power) {
    String dispMode = disp.mode;
    String expMode = expectedMode;
    dispMode.toUpperCase();
    expMode.toUpperCase();
    if (dispMode != expMode) {
      logPrintf("[Expected] Mode mismatch: expected=%s got=%s\n",
                expMode.c_str(), dispMode.c_str());
      return false;
    }
  }
  
  if (expectedFanValid && disp.power) {
    String dispFan = normalizeFanSpeed(disp.fan);
    String expFan = normalizeFanSpeed(expectedFan);
    if (dispFan != expFan) {
      logPrintf("[Expected] Fan mismatch: expected=%s got=%s\n",
                expFan.c_str(), dispFan.c_str());
      return false;
    }
  }
  
  if (expectedSetTempValid && disp.power) {
    if (disp.set_temp != expectedSetTemp) {
      logPrintf("[Expected] SetTemp mismatch: expected=%d got=%d\n",
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

static String selectOptimalFanSpeed(int currentTemp, int targetTemp) {
  int tempDiff = currentTemp - targetTemp;
  
  logPrintf("[Auto] Temperature Analysis: Current=%d, Target=%d, Diff=%d\n", 
            currentTemp, targetTemp, tempDiff);
  
  String fanSpeed;
  
  if (tempDiff < -1) {
    fanSpeed = "LOW";
    logPrintf("[Auto] Temp %d°C below target → Fan: LOW (ultra silent)\n", abs(tempDiff));
  }
  else if (tempDiff <= 0) {
    fanSpeed = "LOW";
    logPrintln("[Auto] Temp at/near target → Fan: LOW (energy saving)");
  }
  else if (tempDiff == 1) {
    fanSpeed = "HIGH";
    logPrintln("[Auto] Temp 1°C above target → Fan: HIGH (aggressive cooling)");
  }
  else {
    fanSpeed = "TURBO";
    logPrintf("[Auto] Temp %d°C above target → Fan: TURBO (maximum cooling)\n", tempDiff);
  }
  
  return fanSpeed;
}

static bool isIntelligentFanChange(const String& oldFan, const String& newFan, 
                                   int currentTemp, int targetTemp) {
  if (cfg.setting.default_fan != "AUTO") {
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
  logPrintln("[Auto] Perintahkan AC ON");
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
  controllHandleCommand(jsonStr.c_str(), jsonStr.length());
  
  lastActionMs = millis();
}

static void turnACOff() {
  logPrintln("[Auto] Perintahkan AC OFF");
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
  controllHandleCommand(jsonStr.c_str(), jsonStr.length());
}

static void applyTempAndMode(int temp, const String& mode) {
  logPrintf("[Auto] Set %d°C, mode=%s\n", temp, mode.c_str());

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
  controllHandleCommand(jsonStr.c_str(), jsonStr.length());
}

static void applyTempModeFan(int temp, const String& mode, const String& fan) {
  logPrintf("[Auto] Set %d°C, mode=%s, fan=%s\n", temp, mode.c_str(), fan.c_str());

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
  controllHandleCommand(jsonStr.c_str(), jsonStr.length());
}

static void applyModeOnly(const String& mode) {
  logPrintf("[Auto] Set mode=%s\n", mode.c_str());
  
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
  controllHandleCommand(jsonStr.c_str(), jsonStr.length());
}

static void controlAC(bool power, String mode, int temp, String fanSpeed = "") {
  if (power) {
    if (fanSpeed.length() == 0) {
      if (cfg.setting.default_fan.length() > 0) {
        fanSpeed = cfg.setting.default_fan;
        logPrintf("[Auto] Using config default fan: %s\n", fanSpeed.c_str());
      } else {
        fanSpeed = "AUTO";
        logPrintln("[Auto] No default fan configured, using AUTO");
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
    logPrintf("[Auto] Immediate resync fallback to last schedule state: %s\n",
              shouldBeOn ? "ON" : "OFF");
    return false;
  }

  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) {
    logPrintf("[Auto] Immediate resync fallback to last schedule state: %s\n",
              shouldBeOn ? "ON" : "OFF");
    return false;
  }

  DaySchedule &daySchedule = cfg.setting.daily[dayIdx];
  if (daySchedule.type == DAY_FULL) {
    shouldBeOn = true;
  } else if (daySchedule.type == DAY_PART_TIME) {
    shouldBeOn = isWithinOperatingHours(daySchedule.waktu.mulai, daySchedule.waktu.selesai);
  } else {
    shouldBeOn = false;
  }

  return true;
}

static void checkAndFixMode() {
  BPFrame frame = sensorGetLastFrame();
  if (!frame.valid) return;
  
  String cMode = frame.mode; String kMode = cfg.setting.set_mode;
  cMode.toUpperCase(); kMode.toUpperCase();
  
  if (cMode != kMode) applyModeOnly(cfg.setting.set_mode);
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
  unsigned long tempCdMs = (unsigned long)cfg.setting.temp_control_cooldown_sec * 1000UL;
  if (now - lastTempControlMs < tempCdMs) {
    return;
  }
  
  int currentTemp = frame.current_temp;
  int setTemp = cfg.setting.set_temp;
  int maxTemp = cfg.setting.max_temp;
  int minTemp = cfg.setting.min_temp;

  // ═════════════════════════════════════════════════════════════
  // CASE 1: SUHU DALAM RANGE → CHECK FAN SPEED
  // ═════════════════════════════════════════════════════════════
  if (currentTemp >= minTemp && currentTemp <= maxTemp) {
    // Reset counter jika suhu kembali normal
    if (coolingAttemptCount > 0) {
      logPrintln("[Auto] Temperature back to normal range, resetting counter");
      coolingAttemptCount = 0;
    }
    
    // Check if fan speed matches default_fan setting
    String currentFan = frame.fan;
    String defaultFan = cfg.setting.default_fan;
    currentFan.toUpperCase();
    defaultFan.toUpperCase();
    currentFan.trim();
    defaultFan.trim();
    
    if (currentFan != defaultFan) {
      logPrintf("[Auto] Temperature OK but fan mismatch: %s != %s, adjusting...\n", 
                currentFan.c_str(), defaultFan.c_str());
      
      lastTempControlMs = now;

      int desiredTempForFan = setTemp;
      if (allowedSetTempOverrideActive) {
        desiredTempForFan = (int)allowedSetTempOverride;
      }
      
      if (frame.fan != cfg.setting.default_fan) {
        activityTrackFan(frame.fan, cfg.setting.default_fan, SOURCE_AUTOMATION);
      }
      applyTempModeFan(desiredTempForFan, cfg.setting.set_mode, cfg.setting.default_fan);
    }
    
    return;
  }

  // ═════════════════════════════════════════════════════════════
  // CASE 2: SUHU > MAX_TEMP → COOLING MODE
  // ═════════════════════════════════════════════════════════════
  if (currentTemp > maxTemp) {
    coolingAttemptCount++;
    logPrintf("[Auto] Too hot: %d°C > %d°C (attempt %d/3)\n", 
              currentTemp, maxTemp, coolingAttemptCount);
    
    lastTempControlMs = now;
    
    // Cek apakah sudah 3x attempt
    if (coolingAttemptCount >= 3) {
      int newSetTemp = setTemp - 2;
      if (newSetTemp < 16) newSetTemp = 16; // Min AC limit
      
      logPrintf("[Auto] 3x cooling failed, lowering setpoint: %d°C → %d°C\n", 
                setTemp, newSetTemp);
      
      if (frame.fan != "HIGH") {
        activityTrackFan(frame.fan, "HIGH", SOURCE_AUTOMATION);
      }
      applyTempModeFan(newSetTemp, cfg.setting.set_mode, "HIGH");
      
      // Reset counter setelah menurunkan setpoint
      coolingAttemptCount = 0;
    } else {
      // Normal cooling: set_temp, fan HIGH
      logPrintf("[Auto] Cooling action: %d°C, fan=HIGH\n", setTemp);
      
      if (frame.fan != "HIGH") {
        activityTrackFan(frame.fan, "HIGH", SOURCE_AUTOMATION);
      }
      applyTempModeFan(setTemp, cfg.setting.set_mode, "HIGH");
    }
    
    return;
  }

  // ═════════════════════════════════════════════════════════════
  // CASE 3: SUHU < MIN_TEMP → WARMING MODE
  // ═════════════════════════════════════════════════════════════
  if (currentTemp < minTemp) {
    // Reset cooling counter
    coolingAttemptCount = 0;
    
    logPrintf("[Auto] Too cold: %d°C < %d°C\n", currentTemp, minTemp);
    
    lastTempControlMs = now;
    
    // Set ke set_temp dengan fan LOW
    logPrintf("[Auto] Warming action: %d°C, fan=LOW\n", setTemp);
    
    if (frame.fan != "LOW") {
      activityTrackFan(frame.fan, "LOW", SOURCE_AUTOMATION);
    }
    applyTempModeFan(setTemp, cfg.setting.set_mode, "LOW");
    
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
      logPrintf("[Monitor-BP] Power Change: %s → %s (Display invalid)\n",
                lastKnownBpPower ? "ON" : "OFF", mainboard.power ? "ON" : "OFF");
    } else {
      logPrintf("[Monitor-BP] Non-power change (Display invalid): mode=%s→%s fan=%s→%s\n",
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
        logPrintln("[Monitor-BP] ✓ Change matches expected state (MQTT/Automation)");
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
      logPrintf("[Monitor-BP] Recent system action (%lums) - ignore power change\n", timeSinceAction);
      lastKnownBpPower = mainboard.power;
      lastKnownBpMode = mainboard.mode;
      lastKnownBpFan = mainboard.fan;
      return;
    }

    // Jika automation sedang OFF, hanya update baseline.
    if (!cfg.setting.automation) {
      logPrintln("[Monitor-BP] Automation OFF -> No action (but still track activity)");

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
      logPrintln("╔═══════════════════════════════════════╗");
      logPrintln("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
      logPrintln("║ Source: IR Remote / Panel Button      ║");
      logPrintln("║ (Display invalid) -> RESYNC AUTOMATION║");
      if (bpModeChanged) {
        logPrintf("║ Mode(BP): %s → %s                 ║\n",
                  lastKnownBpMode.c_str(), mainboard.mode.c_str());
      }
      if (bpFanChanged) {
        logPrintf("║ Fan(BP): %s → %s                  ║\n",
                  lastKnownBpFan.c_str(), mainboard.fan.c_str());
      }
      logPrintln("╚═══════════════════════════════════════╝");

      if (bpModeChanged && mainboard.power) {
        activityTrackMode(lastKnownBpMode, mainboard.mode, SOURCE_PHYSICAL_REMOTE);
      }
      if (bpFanChanged && mainboard.power) {
        activityTrackFan(lastKnownBpFan, mainboard.fan, SOURCE_PHYSICAL_REMOTE);
      }

      clearExpectedState();

      String resyncFan = cfg.setting.default_fan;
      if (cfg.setting.default_fan == "AUTO" && mainboard.valid) {
        resyncFan = selectOptimalFanSpeed(mainboard.current_temp, cfg.setting.set_temp);
      }

      lastKnownBpPower = mainboard.power;
      lastKnownBpMode = mainboard.mode;
      lastKnownBpFan = mainboard.fan;

      if (mainboard.power) {
        lastActionMs = millis();
        lastActionSource = ACTION_AUTOMATION;
        applyTempModeFan(cfg.setting.set_temp, cfg.setting.set_mode, resyncFan);
      } else {
        forceScheduleCheck = true;
      }
      return;
    }

    // Physical remote power change detected -> masuk manual recovery.
    logPrintln("╔═══════════════════════════════════════╗");
    logPrintln("║ ⚠️ PHYSICAL REMOTE POWER DETECTED     ║");
    logPrintln("║ Source: IR Remote / Panel Button      ║");
    logPrintf ("║ Power: %s → %s                    ║\n",
               lastKnownBpPower ? "ON" : "OFF", mainboard.power ? "ON" : "OFF");
    logPrintln("║ -> Manual Recovery ACTIVE             ║");
    logPrintln("╚═══════════════════════════════════════╝");

    manualOverrideCount++;
    logPrintf("[Auto] Manual Override Count: %lu\n", manualOverrideCount);
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
    bool isIntelligentChange = isIntelligentFanChange(
      lastKnownFan, 
      disp.fan, 
      currentTemp, 
      cfg.setting.set_temp
    );
    
    if (isIntelligentChange) {
      logPrintf("[Monitor] Fan changed: %s → %s (Intelligent Control - Ignored)\n", 
                lastKnownFan.c_str(), disp.fan.c_str());
      
      lastKnownFan = disp.fan;
      fanChanged = false;
    }
  }

  if (setTempChanged && !powerChanged && !modeChanged && !fanChanged) {
    uint8_t minTemp = cfg.setting.min_temp;
    uint8_t maxTemp = cfg.setting.max_temp;
    uint8_t newSetTemp = disp.set_temp;
    
    if (newSetTemp >= minTemp && newSetTemp <= maxTemp) {
      logPrintf("[Monitor] Set temp changed: %d → %d (Within range [%d-%d] - Allowed)\n", 
                lastKnownSetTemp, newSetTemp, minTemp, maxTemp);
      logPrintln("[Monitor] → No manual recovery triggered");

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
      logPrintf("[Monitor] Set temp changed: %d → %d (Outside range [%d-%d])\n", 
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
  
  logPrintf("[Monitor] Display Change: power=%d->%d, set_temp=%d->%d, mode=%s->%s, fan=%s->%s\n",
            lastKnownPower, disp.power,
            lastKnownSetTemp, disp.set_temp,
            lastKnownMode.c_str(), disp.mode.c_str(),
            lastKnownFan.c_str(), disp.fan.c_str());

  if (matchesExpectedState(disp)) {
    logPrintln("[Monitor] ✓ Change matches expected state (MQTT/Automation)");
    logPrintln("[Monitor] → No manual recovery triggered");
    clearExpectedState();
    lastActionSource = ACTION_NONE;
  }
  else if (isExpectedStateValid()) {
    logPrintln("[Monitor] ⚠ Change does NOT match expected state");
    
    unsigned long timeSinceAction = now - lastActionMs;
    if (timeSinceAction <= ACTION_WINDOW_MS && lastActionSource != ACTION_NONE) {
      logPrintf("[Monitor] Within action window (%lums) but state mismatch\n", timeSinceAction);
      
      if (lastActionSource == ACTION_MQTT_REMOTE) {
        if (cfg.setting.manualModeActive) {
          logPrintln("[Monitor] ✓ MQTT Remote action (Manual recovery active)");
        } else {
          logPrintln("[Monitor] ✓ MQTT Remote action (No manual recovery)");
        }
        clearExpectedState();
        lastActionSource = ACTION_NONE;
      }
      else if (lastActionSource == ACTION_AUTOMATION) {
        logPrintln("[Monitor] ✓ Automation action - No manual recovery");
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
          logPrintln("[Monitor] ✓ Recent MQTT action detected (Manual recovery active)");
        } else {
          logPrintln("[Monitor] ✓ Recent MQTT action detected (No manual recovery)");
        }
        lastActionSource = ACTION_NONE;
      }
      else if (lastActionSource == ACTION_AUTOMATION) {
        logPrintln("[Monitor] ✓ Recent Automation action detected - No manual recovery");
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
    logPrintln("[Monitor] Physical remote change detected. Automation OFF -> No action (but still track activity).");

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
      uint8_t minTemp = cfg.setting.min_temp;
      uint8_t maxTemp = cfg.setting.max_temp;
      uint8_t newSetTemp = disp.set_temp;

      if (newSetTemp < minTemp || newSetTemp > maxTemp) {
        logPrintln("╔═══════════════════════════════════════╗");
        logPrintln("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
        logPrintln("║ Source: IR Remote / Panel Button      ║");
        logPrintln("║ -> Set temp out-of-range: RESYNC AUTO ║");
        logPrintf("║ Set Temp: %d → %d (Range: %d-%d)    ║\n",
                  lastKnownSetTemp, disp.set_temp,
                  cfg.setting.min_temp, cfg.setting.max_temp);
        logPrintf("║ Restoring to: %d°C                 ║\n", cfg.setting.set_temp);
        logPrintln("╚═══════════════════════════════════════╝");

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
          String resyncFan = cfg.setting.default_fan;
          if (cfg.setting.default_fan == "AUTO" && mainboard.valid) {
            resyncFan = selectOptimalFanSpeed(mainboard.current_temp, cfg.setting.set_temp);
          }
          lastActionMs = millis();
          lastActionSource = ACTION_AUTOMATION;
          applyTempModeFan(cfg.setting.set_temp, cfg.setting.set_mode, resyncFan);
        } else {
          forceScheduleCheck = true;
        }
        return;
      }
    }

    logPrintln("╔═══════════════════════════════════════╗");
    logPrintln("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
    logPrintln("║ Source: IR Remote / Panel Button      ║");
    logPrintln("║ -> Non-power change: RESYNC AUTOMATION║");
    if (setTempChanged) {
      logPrintf("║ Set Temp: %d → %d (Range: %d-%d)    ║\n",
                lastKnownSetTemp, disp.set_temp,
                cfg.setting.min_temp, cfg.setting.max_temp);
    }
    if (modeChanged) {
      logPrintf("║ Mode: %s → %s                     ║\n",
                lastKnownMode.c_str(), disp.mode.c_str());
    }
    if (fanChanged) {
      logPrintf("║ Fan: %s → %s                      ║\n",
                lastKnownFan.c_str(), disp.fan.c_str());
    }
    logPrintln("╚═══════════════════════════════════════╝");

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
    String resyncFan = cfg.setting.default_fan;
    if (cfg.setting.default_fan == "AUTO" && mainboard.valid) {
      resyncFan = selectOptimalFanSpeed(mainboard.current_temp, cfg.setting.set_temp);
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
      applyTempModeFan(cfg.setting.set_temp, cfg.setting.set_mode, resyncFan);
    } else {
      forceScheduleCheck = true;
    }

    return;
  }
  
  logPrintln("╔═══════════════════════════════════════╗");
  logPrintln("║ ⚠️ PHYSICAL REMOTE CHANGE DETECTED    ║");
  logPrintln("║ Source: IR Remote / Panel Button      ║");
  
  if (powerChanged) {
    logPrintf("║ Power: %s → %s                    ║\n", 
              lastKnownPower ? "ON" : "OFF", disp.power ? "ON" : "OFF");
  }
  if (setTempChanged) {
    logPrintf("║ Set Temp: %d → %d (Range: %d-%d)    ║\n", 
              lastKnownSetTemp, disp.set_temp, 
              cfg.setting.min_temp, cfg.setting.max_temp);
  }
  if (modeChanged) {
    logPrintf("║ Mode: %s → %s                     ║\n", 
              lastKnownMode.c_str(), disp.mode.c_str());
  }
  if (fanChanged) {
    logPrintf("║ Fan: %s → %s                      ║\n", 
              lastKnownFan.c_str(), disp.fan.c_str());
  }
  
  logPrintln("║ -> Manual Recovery ACTIVE             ║");
  logPrintln("╚═══════════════════════════════════════╝");

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
    String resyncFan = cfg.setting.default_fan;
    if (cfg.setting.default_fan == "AUTO" && mainboard.valid) {
      resyncFan = selectOptimalFanSpeed(mainboard.current_temp, cfg.setting.set_temp);
    }

    if (!disp.power) {
      activityTrackPower(false, true, SOURCE_AUTOMATION);
    }

    lastActionMs = millis();
    lastActionSource = ACTION_AUTOMATION;
    applyTempModeFan(cfg.setting.set_temp, cfg.setting.set_mode, resyncFan);
  } else {
    if (disp.power) {
      activityTrackPower(true, false, SOURCE_AUTOMATION);
    }

    lastActionMs = millis();
    lastActionSource = ACTION_AUTOMATION;
    controlAC(false, cfg.setting.set_mode, cfg.setting.set_temp);
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
    logPrintf("[Auto-Recovery] ⏳ Manual mode - Recovery in %lu:%02lu\n", rSec / 60, rSec % 60);
    lastLogMs = now;
  }
}

void automationInit() {
  logPrintf("[Auto] Init State: %s\n", cfg.setting.automation ? "ON" : "OFF");
  
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
  if (lastConfigSetTemp < 0) {
    lastConfigSetTemp = cfg.setting.set_temp;
  } else if (cfg.setting.set_temp != lastConfigSetTemp) {
    allowedSetTempOverrideActive = false;
    allowedSetTempOverride = 0;
    lastConfigSetTemp = cfg.setting.set_temp;
  }

  if (waitingForACResponse) {
    if (now - lastIRCommandMs < IR_SETTLE_TIME_MS) {
      return;
    }

    if (irVerifyStartMs > 0 && (now - irVerifyStartMs) > IR_VERIFY_TIMEOUT_MS) {
      logPrintln("[Auto] ⚠️ IR verification timeout");
      waitingForACResponse = false;
      irVerifyRetryCount = 0;
      irVerifyStartMs = 0;
      return;
    }
    
    bool actualState = getACPowerStatus();
    if (actualState == expectedACState) {
      logPrintln("[Auto] AC state verified successfully");
      waitingForACResponse = false;
      irVerifyRetryCount = 0;
      irVerifyStartMs = 0;
    } else {
      if (irVerifyRetryCount < 3) {
        irVerifyRetryCount++;
        logPrintf("[Auto] State mismatch, retry %d/3\n", irVerifyRetryCount);
        lastIRCommandMs = now;
        return;
      } else {
        logPrintln("[Auto] ⚠️ AC not responding after 3 retries");
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
        logPrintln("[Auto] Recovery timer habis, tapi automation di-disable oleh user — tetap OFF");
        cfg.setting.manualModeActive = false;
        if (cfgSave()) {
          cfgApply();
          if (dataIsConnected()) dataPublishSetting();
        }
        return;
      }

      logPrintln("[Auto] Manual recovery selesai, automation aktif kembali");
      
      activityTrackAutoRecovery();
      
      cfg.setting.manualModeActive = false;
      cfg.setting.automation = true;
      cfg.setting.automationPausedByUser = false;

      lastScheduleCheckMs = 0;
      scheduleStateInit = false;
      lastTempControlMs = 0;

      logPrintln("[Auto] Restoring automation target state with optimal fan");
      
      BPFrame currentFrame = sensorGetLastFrame();
      String optimalFan = cfg.setting.default_fan;
      
      if (cfg.setting.default_fan == "AUTO" && currentFrame.valid) {
        optimalFan = selectOptimalFanSpeed(currentFrame.current_temp, cfg.setting.set_temp);
      }
      
      if (currentFrame.valid) {
        if (!currentFrame.power) {
          activityTrackPower(false, true, SOURCE_AUTOMATION);
        }
        if (currentFrame.mode != cfg.setting.set_mode) {
          activityTrackMode(currentFrame.mode, cfg.setting.set_mode, SOURCE_AUTOMATION);
        }
        if (currentFrame.fan != optimalFan) {
          activityTrackFan(currentFrame.fan, optimalFan, SOURCE_AUTOMATION);
        }
      }
      
      controlAC(true, cfg.setting.set_mode, cfg.setting.set_temp, optimalFan);
      
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

      logPrintf("[Auto] automation=%d manual=%d pausedByUser=%d\n",
        cfg.setting.automation, cfg.setting.manualModeActive, 
        cfg.setting.automationPausedByUser);
      
      forceScheduleCheck = true;
    }
    return;
  }
  bool isForceCheck = forceScheduleCheck;
  if (isForceCheck) {
    logPrintln("[Auto] Force schedule evaluation triggered");
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
      logPrintln("[Auto] ⚠️ WiFi dan RTC tidak tersedia - automation dimatikan");
      lastWarnMs = now;
    }
    lastTempControlMs = 0;
    if (forceScheduleCheck) forceScheduleCheck = false;
    return;
  } else if (!rtcOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 60000) {
      logPrintln("[Auto] ⚠️ RTC tidak tersedia, menggunakan NTP dari WiFi");
      lastWarnMs = now;
    }
  } else if (!wifiOk) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 60000) {
      logPrintln("[Auto] ⚠️ WiFi tidak tersedia, menggunakan RTC");
      lastWarnMs = now;
    }
  }

  int dayIdx = getDayOfWeek();
  if (dayIdx < 0) {
    static unsigned long lastWarnMs = 0;
    if (now - lastWarnMs > 30000) {
      logPrintln("[Auto] ⚠️ Gagal mendapatkan waktu - skip cycle");
      lastWarnMs = now;
    }
    return;
  }

  DaySchedule &d = cfg.setting.daily[dayIdx];
  bool shouldBeOn = false;

  if (d.type == DAY_FULL) {
    shouldBeOn = true;
  }
  else if (d.type == DAY_PART_TIME) {
    shouldBeOn = isWithinOperatingHours(d.waktu.mulai, d.waktu.selesai);
  }
  
  if (!logStateInit || shouldBeOn != lastLoggedShouldBeOn) {
    logPrintf("[Auto] Day=%d (%s), Type=%d, ShouldBeOn=%s\n", 
      dayIdx, DAY_NAMES[dayIdx], d.type, shouldBeOn ? "YES" : "NO");
    lastLoggedShouldBeOn = shouldBeOn;
    logStateInit = true;
  }

  if (shouldBeOn) {
    BPFrame currentFrame = sensorGetLastFrame();
    
    if (!currentFrame.valid) {
      logPrintln("[Auto] ⏳ Waiting for valid frame... No IR command sent");
      return;
    }
    
    if (!getACPowerStatus()) {
       logPrintln("[Auto] Schedule ON -> Turning AC ON with intelligent fan");
       
       // ═══════════════════════════════════════════════
       // INCREMENT COUNTER untuk automation ON
       // ═══════════════════════════════════════════════
       autoOnCount++;
       logPrintf("[Auto] Auto ON Count: %lu\n", autoOnCount);
       
       String startupFan = cfg.setting.default_fan;
       
       if (cfg.setting.default_fan == "AUTO" && currentFrame.valid) {
         startupFan = selectOptimalFanSpeed(currentFrame.current_temp, cfg.setting.set_temp);
       }
       
       // Track activity before executing
       if (currentFrame.valid && !currentFrame.power) {
         activityTrackPower(false, true, SOURCE_AUTOMATION);
       }
       
       controlAC(true, cfg.setting.set_mode, cfg.setting.set_temp, startupFan);
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
       String targetMode = String(cfg.setting.set_mode);
       
       bool modeMismatch = (currentMode != targetMode);
       
       if (isForceCheck || modeMismatch) {
           logPrintln("[Auto] Syncing State (Mode/Temp config changed)...");
           logPrintf("[Auto] Mode: %s -> %s\n", currentMode.c_str(), targetMode.c_str());
           
           String syncFan = cfg.setting.default_fan;
           
           if (cfg.setting.default_fan == "AUTO" && lastFrame.valid) {
             syncFan = selectOptimalFanSpeed(lastFrame.current_temp, cfg.setting.set_temp);
           }
           
           controlAC(true, cfg.setting.set_mode, cfg.setting.set_temp, syncFan);
           
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
      logPrintln("[Auto] ⏳ Waiting for valid frame... No IR command sent");
      return;
    }
    
    if (getACPowerStatus()) {
       logPrintln("[Auto] Schedule OFF -> Turning AC OFF");
       
       // ═══════════════════════════════════════════════
       // INCREMENT COUNTER untuk automation OFF
       // ═══════════════════════════════════════════════
       autoOffCount++;
       logPrintf("[Auto] Auto OFF Count: %lu\n", autoOffCount);
       
       // Track activity before executing
       if (currentFrame.valid && currentFrame.power) {
         activityTrackPower(true, false, SOURCE_AUTOMATION);
       }
       
       controlAC(false, cfg.setting.set_mode, cfg.setting.set_temp);
       
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
  lastActionMs = millis();
  lastActionSource = ACTION_MQTT_REMOTE;
  setExpectedState(power, mode, fan, setTemp);

  // MQTT remote hanya set expected state — TIDAK trigger manual recovery.
  // Manual recovery hanya boleh di-trigger oleh perubahan data dari AC
  // (physical remote / panel button) yang terdeteksi di monitorExternalChange().
  // Jika AC tidak merespons IR (no response), state tidak berubah → tidak ada trigger.
  logPrintf("[MQTT] Expected state set: power=%s mode=%s fan=%s temp=%d (no manual recovery)\n",
            power ? "ON" : "OFF", mode.c_str(), fan.c_str(), setTemp);
}