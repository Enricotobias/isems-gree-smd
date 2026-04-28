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
  expectedFan = fan;
  expectedSetTemp = setTemp;
  commandSentMs = millis();
  expectingResponse = true;
  observedRelevantChange = false;
  
  Serial.Printf("[IR Health] Expected: power=%s mode=%s fan=%s set_temp=%d\n",
            power ? "ON" : "OFF", mode.c_str(), fan.c_str(), setTemp);
}

void irHealthCheckResponse() {
  // Even if we're not expecting a response, we use this polling point to
  // auto-enable IR validation once a display frame exists.
  autoEnableIfDisplayReady();

  if (!healthEnabled || !expectingResponse) return;
  
  // Wait at least 5 seconds for AC to respond, but timeout after 8 seconds.
  unsigned long elapsed = millis() - commandSentMs;
  if (elapsed < IR_HEALTH_MIN_DELAY_MS) return;
  
  // Timeout: stop waiting even if no frame arrives
  if (elapsed > IR_HEALTH_TIMEOUT_MS) {
    stats.totalCommands++;
    stats.lastCommandMs = commandSentMs;
    stats.noResponseCount++;
    stats.lastCommandSuccess = false;
    stats.lastCommandType = "NO_RESPONSE";
    stats.lastExpected = String("power=") + (expectedPower ? "ON" : "OFF") +
                        " mode=" + expectedMode +
                        " fan=" + expectedFan +
                        " set_temp=" + String(expectedSetTemp);
    stats.lastActual = "TIMEOUT";

    Serial.Println("╔═══════════════════════════════════════╗");
    Serial.Println("║ ⚠️ IR LED HEALTH: TIMEOUT             ║");
    Serial.Println("╚═══════════════════════════════════════╝");
    Serial.Printf("[IR Health] Expected: %s\n", stats.lastExpected.c_str());
    Serial.Println("[IR Health] Actual: Timeout waiting for display response");
    Serial.Printf("[IR Health] Success Rate: %.1f%% (%s)\n",
              stats.getSuccessRate(), stats.getHealthStatus().c_str());

    String ackMsg = "Display TIMEOUT - Expected: " + stats.lastExpected +
                   " | Rate: " + String(stats.getSuccessRate(), 1) + "%" +
                   " | Total: " + String(stats.totalCommands);
    dataPublishAck("ir_validation", "no_response", ackMsg);

    expectingResponse = false;
    return;
  }

  // Get current display frame
  if (!sensorHasValidDisplayFrame()) {
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
      stats.lastExpected = String("power=") + (expectedPower ? "ON" : "OFF") +
                          " mode=" + expectedMode +
                          " fan=" + expectedFan +
                          " set_temp=" + String(expectedSetTemp);
      stats.lastActual = "NO_FRAME";

      String ackMsg = "No valid display frame within timeout";
      dataPublishAck("ir_validation", "no_response", ackMsg);

      expectingResponse = false;
    }
    return;
  }
  
  // Mark bahwa kita sudah pernah dapat frame valid
  hasReceivedValidFrame = true;
  
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
    stats.totalCommands++;
    stats.lastCommandMs = commandSentMs;
    stats.successCount++;
    stats.lastCommandSuccess = true;
    stats.lastCommandType = "SUCCESS";
    stats.lastExpected = String("power=") + (expectedPower ? "ON" : "OFF") +
                        " mode=" + expectedMode +
                        " fan=" + expectedFan +
                        " set_temp=" + String(expectedSetTemp);
    stats.lastActual = String("power=") + (disp.power ? "ON" : "OFF") +
                      " mode=" + disp.mode +
                      " fan=" + disp.fan +
                      " set_temp=" + String(disp.set_temp);

    String ackMsg = "IR command verified: " + stats.lastActual +
                   " | Rate: " + String(stats.getSuccessRate(), 1) + "%";
    dataPublishAck("ir_validation", "success", ackMsg);

    expectingResponse = false;
    return;
  }

  if (elapsed < IR_HEALTH_TIMEOUT_MS) return;

  stats.totalCommands++;
  stats.lastCommandMs = commandSentMs;
  stats.lastExpected = String("power=") + (expectedPower ? "ON" : "OFF") +
                      " mode=" + expectedMode +
                      " fan=" + expectedFan +
                      " set_temp=" + String(expectedSetTemp);
  stats.lastActual = String("power=") + (disp.power ? "ON" : "OFF") +
                    " mode=" + disp.mode +
                    " fan=" + disp.fan +
                    " set_temp=" + String(disp.set_temp);

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
