#include "ir_health.h"
#include "sensor.h"

#include "data.h"
#include "config.h"
#include <Arduino.h>

static IRHealthStats stats = {0, 0, 0, 0, false, "", "", "", 0};
static bool healthEnabled = false;
static bool expectingResponse = false;

// Gate IR validation until we have at least one valid DisplayFrame since boot.
// This avoids counting false NO_RESPONSE / mismatches when the UART Display stream
// is not yet synchronized or wiring is not ready.
static bool waitingFirstValidDisplayFrame = true;

static void autoEnableIfDisplayReady() {
  if (!waitingFirstValidDisplayFrame) return;
  if (!sensorHasValidDisplayFrame()) return;

  waitingFirstValidDisplayFrame = false;
  healthEnabled = true;
  Serial.Println("[IR Health] Display frame detected; IR validation enabled");
}

static String normalizeFanSpeedLabel(const String& fan) {
  String normalized = fan;
  normalized.toUpperCase();
  normalized.trim();

  if (normalized == "MED") normalized = "MEDIUM";
  if (normalized == "MIN") normalized = "LOW";
  if (normalized == "MAX") normalized = "HIGH";

  return normalized;
}

// Expected state from IR command
static bool expectedHasPower = false;
static bool expectedPower = false;
static bool expectedHasMode = false;
static String expectedMode = "";
static bool expectedHasFan = false;
static String expectedFan = "";
static bool expectedHasTemp = false;
static uint8_t expectedSetTemp = 0;
static unsigned long commandSentMs = 0;

static bool observedRelevantChange = false;

// Previous display state (before command)
static bool prevPower = false;
static String prevMode = "";
static String prevFan = "";
static uint8_t prevSetTemp = 0;

// Throttling untuk warning log
static unsigned long lastNoFrameWarningMs = 0;
const unsigned long NO_FRAME_WARNING_INTERVAL_MS = 60000UL; // 1 menit
static bool hasReceivedValidFrame = false; // Track jika pernah dapat frame valid

static const unsigned long IR_HEALTH_MIN_DELAY_MS = 5000UL;
static const unsigned long IR_HEALTH_TIMEOUT_MS = 8000UL;
static const unsigned long POWER_OFF_TRANSITION_FRAME_MS = 2000UL;

static String buildExpectedSummary() {
  String s = "power=";
  s += expectedHasPower ? (expectedPower ? "ON" : "OFF") : "-";
  s += " mode=";
  s += expectedHasMode ? expectedMode : "-";
  s += " fan=";
  s += expectedHasFan ? normalizeFanSpeedLabel(expectedFan) : "-";
  s += " set_temp=";
  s += expectedHasTemp ? String(expectedSetTemp) : "-";
  return s;
}

static String buildActualSummary(const DisplayFrame& disp) {
  return String("power=") + (disp.power ? "ON" : "OFF") +
         " mode=" + disp.mode +
         " fan=" + normalizeFanSpeedLabel(disp.fan) +
         " set_temp=" + String(disp.set_temp);
}

static bool isPowerOffOnlyExpected() {
  return expectedHasPower && !expectedPower &&
         !expectedHasMode && !expectedHasFan && !expectedHasTemp;
}

static bool isFanAutoOnlyExpected() {
  return !expectedHasPower && !expectedHasMode && expectedHasFan &&
         normalizeFanSpeedLabel(expectedFan) == "AUTO" && !expectedHasTemp;
}

static bool isFrameFreshAfterCommand(unsigned long frameMs) {
  return frameMs != 0 && (long)(frameMs - commandSentMs) >= 0;
}

static void reportValidationSuccess(const String& actual) {
  stats.totalCommands++;
  stats.lastCommandMs = commandSentMs;
  stats.successCount++;
  stats.lastCommandSuccess = true;
  stats.lastCommandType = "SUCCESS";
  stats.lastExpected = buildExpectedSummary();
  stats.lastActual = actual;

  String ackMsg = "IR command verified: " + stats.lastActual +
                 " | Rate: " + String(stats.getSuccessRate(), 1) + "%";
  dataPublishAck("ir_validation", "success", ackMsg);

  expectingResponse = false;
}

static bool validatePowerOffFromMainboard() {
  if (!isPowerOffOnlyExpected()) return false;

  BPFrame frame = {};
  unsigned long frameMs = 0;
  if (!sensorGetMainboardSnapshot(frame, frameMs)) return false;
  if (!isFrameFreshAfterCommand(frameMs)) return false;
  if (frame.power) return false;

  reportValidationSuccess("power=OFF source=MAINBOARD");
  return true;
}

static bool hasRelevantFreshMainboardFrame() {
  BPFrame frame = {};
  unsigned long frameMs = 0;
  if (!sensorGetMainboardSnapshot(frame, frameMs)) return false;
  if (!isFrameFreshAfterCommand(frameMs)) return false;

  if (!frame.power) {
    return true;
  }

  unsigned long frameDelayMs = frameMs - commandSentMs;
  if (frameDelayMs <= POWER_OFF_TRANSITION_FRAME_MS) {
    Serial.Printf("[IR Health] Ignoring transitional MAINBOARD ON frame at +%lums\n",
                  frameDelayMs);
    return false;
  }

  return true;
}

static bool handlePowerOffNoFreshFrameTimeout(unsigned long elapsed) {
  if (!isPowerOffOnlyExpected()) return false;
  if (elapsed < IR_HEALTH_TIMEOUT_MS) return false;
  if (isFrameFreshAfterCommand(sensorGetLastValidDisplayFrameMs())) return false;
  if (hasRelevantFreshMainboardFrame()) return false;

  reportValidationSuccess("power=OFF source=NO_FRESH_RELEVANT_FRAME");
  return true;
}

void irHealthInit() {
  stats.totalCommands = 0;
  stats.successCount = 0;
  stats.noResponseCount = 0;
  stats.mismatchCount = 0;
  stats.lastCommandSuccess = false;
  stats.lastCommandType = "";
  stats.lastExpected = "";
  stats.lastActual = "";
  stats.lastCommandMs = 0;
  // Default: keep disabled until we have a valid display frame.
  healthEnabled = false;
  expectingResponse = false;
  waitingFirstValidDisplayFrame = true;
  expectedHasPower = false;
  expectedHasMode = false;
  expectedHasFan = false;
  expectedHasTemp = false;
  observedRelevantChange = false;
  
  Serial.Println("[IR Health] Initialized");
}

void irHealthSetExpectedState(bool hasPower, bool power,
                              bool hasMode, const String& mode,
                              bool hasFan, const String& fan,
                              bool hasTemp, uint8_t setTemp) {
  autoEnableIfDisplayReady();
  if (!healthEnabled) return;

  if (hasPower && !power) {
    hasMode = false;
    hasFan = false;
    hasTemp = false;
  }

  const bool anyExpected = hasPower || hasMode || hasFan || hasTemp;
  if (!anyExpected) {
    // Do not track commands that have no verifiable keys.
    expectingResponse = false;
    return;
  }
  
  // Capture previous state from display frame
  if (sensorHasValidDisplayFrame()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    prevPower = disp.power;
    prevMode = disp.mode;
    prevFan = disp.fan;
    prevSetTemp = disp.set_temp;
  }
 
  expectedHasPower = hasPower;
  expectedHasMode = hasMode;
  expectedHasFan = hasFan;
  expectedHasTemp = hasTemp;
  expectedPower = power;
  expectedMode = mode;
  expectedFan = normalizeFanSpeedLabel(fan);
  expectedSetTemp = setTemp;
  commandSentMs = millis();
  expectingResponse = true;
  observedRelevantChange = false;
  
  Serial.Printf("[IR Health] Expected: %s\n", buildExpectedSummary().c_str());
}

void irHealthCheckResponse() {
  // Even if we're not expecting a response, we use this polling point to
  // auto-enable IR validation once a display frame exists.
  autoEnableIfDisplayReady();

  if (!healthEnabled || !expectingResponse) return;
  
  // Wait at least 5 seconds for AC to respond, but timeout after 8 seconds.
  unsigned long elapsed = millis() - commandSentMs;
  if (elapsed < IR_HEALTH_MIN_DELAY_MS) return;

  if (isFanAutoOnlyExpected()) {
    Serial.Println("[IR Health] Fan AUTO command assumed successful; display does not emit a validation frame for AUTO");
    reportValidationSuccess("fan=AUTO source=ASSUMED_AUTO");
    return;
  }

  if (validatePowerOffFromMainboard()) return;
  
  // Get current display frame
  if (!sensorHasValidDisplayFrame()) {
    if (handlePowerOffNoFreshFrameTimeout(elapsed)) return;

    // Hanya log warning jika sebelumnya pernah dapat frame valid
    if (hasReceivedValidFrame) {
      // Throttle warning log - hanya print setiap 1 menit
      unsigned long now = millis();
      if (now - lastNoFrameWarningMs >= NO_FRAME_WARNING_INTERVAL_MS) {
        Serial.Println("[IR Health] ⚠ No valid display frame for health check");
        lastNoFrameWarningMs = now;
      }
    }
    if (elapsed >= IR_HEALTH_TIMEOUT_MS) {
      stats.totalCommands++;
      stats.lastCommandMs = commandSentMs;
      stats.noResponseCount++;
      stats.lastCommandSuccess = false;
      stats.lastCommandType = "NO_RESPONSE";
      stats.lastExpected = buildExpectedSummary();
      stats.lastActual = "NO_FRAME";

      String ackMsg = "No valid display frame within timeout";
      dataPublishAck("ir_validation", "no_response", ackMsg);

      expectingResponse = false;
    }
    return;
  }
  
  // Mark bahwa kita sudah pernah dapat frame valid
  hasReceivedValidFrame = true;

  bool freshDisplayFrame = isFrameFreshAfterCommand(sensorGetLastValidDisplayFrameMs());
  if (isPowerOffOnlyExpected() && !freshDisplayFrame) {
    if (handlePowerOffNoFreshFrameTimeout(elapsed)) return;
    return;
  }
  
  DisplayFrame disp = sensorGetLastDisplayFrame();

  // Track whether any relevant field changed at all.
  bool relevantChanged = false;
  if (expectedHasPower) relevantChanged |= (disp.power != prevPower);
  if (expectedHasMode)  relevantChanged |= (disp.mode != prevMode);
  if (expectedHasFan)   relevantChanged |= (normalizeFanSpeedLabel(disp.fan) != normalizeFanSpeedLabel(prevFan));
  if (expectedHasTemp)  relevantChanged |= (disp.set_temp != prevSetTemp);
  if (relevantChanged) observedRelevantChange = true;

  const bool requiresOnForSettings = expectedHasMode || expectedHasFan || expectedHasTemp;

  bool powerMatch = !expectedHasPower || (disp.power == expectedPower);
  bool modeMatch = !expectedHasMode;
  bool fanMatch = !expectedHasFan;
  bool tempMatch = !expectedHasTemp;

  if (!requiresOnForSettings || disp.power) {
    if (expectedHasMode) {
      String dispMode = disp.mode;
      String expMode = expectedMode;
      dispMode.toUpperCase();
      expMode.toUpperCase();
      modeMatch = (dispMode == expMode);
    }
    if (expectedHasFan) {
      String dispFan = normalizeFanSpeedLabel(disp.fan);
      String expFan = normalizeFanSpeedLabel(expectedFan);
      fanMatch = (dispFan == expFan);
    }
    if (expectedHasTemp) {
      tempMatch = (disp.set_temp == expectedSetTemp);
    }
  }

  bool fullMatch = powerMatch && modeMatch && fanMatch && tempMatch;

  if (fullMatch) {
    reportValidationSuccess(buildActualSummary(disp));
    return;
  }

  if (elapsed < IR_HEALTH_TIMEOUT_MS) return;

  stats.totalCommands++;
  stats.lastCommandMs = commandSentMs;
  stats.lastExpected = buildExpectedSummary();
  stats.lastActual = buildActualSummary(disp);

  if (observedRelevantChange) {
    stats.mismatchCount++;
    stats.lastCommandSuccess = false;
    stats.lastCommandType = "MISMATCH";

    String ackMsg = "Display MISMATCH - Expected: " + stats.lastExpected +
                   " | Actual: " + stats.lastActual +
                   " | Rate: " + String(stats.getSuccessRate(), 1) + "%";
    dataPublishAck("ir_validation", "mismatch", ackMsg);
  } else {
    stats.noResponseCount++;
    stats.lastCommandSuccess = false;
    stats.lastCommandType = "NO_RESPONSE";

    String ackMsg = "Display NO RESPONSE - Expected: " + stats.lastExpected +
                   " | Rate: " + String(stats.getSuccessRate(), 1) + "%" +
                   " | Total: " + String(stats.totalCommands);
    dataPublishAck("ir_validation", "no_response", ackMsg);
  }

  expectingResponse = false;
}

IRHealthStats irHealthGetStats() {
  return stats;
}

void irHealthReset() {
  stats.totalCommands = 0;
  stats.successCount = 0;
  stats.noResponseCount = 0;
  stats.mismatchCount = 0;
  stats.lastCommandSuccess = false;
  stats.lastCommandType = "";
  stats.lastExpected = "";
  stats.lastActual = "";
  stats.lastCommandMs = 0;
  expectingResponse = false;
  
  Serial.Println("[IR Health] Statistics reset");
}

void irHealthSetEnabled(bool enabled) {
  healthEnabled = enabled;
  Serial.Printf("[IR Health] Monitoring %s\n", enabled ? "enabled" : "disabled");
}

bool irHealthIsEnabled() {
  return healthEnabled;
}
