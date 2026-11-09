#include "tasks.h"
#include "rtos_common.h"
#include "controll.h"
#include "sensor.h"
#include "ir_health.h"
#include "esp_task_wdt.h"
#include "runtime_health.h"

namespace {
constexpr unsigned long STARTUP_IR_DELAY_MS = 5000;
constexpr unsigned long STARTUP_IR_RESULT_TIMEOUT_MS = 20000;
constexpr unsigned long STARTUP_IR_RETRY_DELAY_MS = 3000;
constexpr uint8_t STARTUP_IR_MAX_ATTEMPTS = 2;

bool startupIrValidationService(bool commandJustHandled) {
  static const unsigned long taskStartedMs = millis();
  static unsigned long nextAttemptMs = taskStartedMs + STARTUP_IR_DELAY_MS;
  static unsigned long commandSentMs = 0;
  static unsigned long baselineTotal = 0;
  static uint8_t attempts = 0;
  static bool waitingResult = false;
  static bool finished = false;

  if (finished) return true;

  IRHealthStats stats = irHealthGetStats();
  if (commandJustHandled) {
    // Give the real command's validation priority. If it cannot produce a
    // result, startup validation remains eligible after this cooldown.
    nextAttemptMs = millis() + STARTUP_IR_RESULT_TIMEOUT_MS;
    return false;
  }
  if (!waitingResult && stats.totalCommands > 0) {
    finished = true;
    return true;
  }

  const unsigned long now = millis();
  if (waitingResult) {
    if (stats.totalCommands > baselineTotal) {
      waitingResult = false;
      if (stats.lastCommandSuccess || attempts >= STARTUP_IR_MAX_ATTEMPTS) {
        finished = true;
        Serial.printf("[Startup IR] Validation finished: %s after %u attempt(s)\n",
                      stats.lastCommandSuccess ? "success" : "failed", attempts);
        return true;
      }
      nextAttemptMs = now + STARTUP_IR_RETRY_DELAY_MS;
    } else if (now - commandSentMs < STARTUP_IR_RESULT_TIMEOUT_MS) {
      return false;
    } else {
      waitingResult = false;
      if (attempts >= STARTUP_IR_MAX_ATTEMPTS) {
        finished = true;
        Serial.println("[Startup IR] Validation result timeout after 2 attempts");
        return true;
      }
      nextAttemptMs = now + STARTUP_IR_RETRY_DELAY_MS;
    }
  }

  if ((long)(now - nextAttemptMs) < 0 || !sensorIsHealthy()) return false;

  BPFrame frame = sensorGetLastFrame();
  if (!frame.valid) return false;

  char payload[40];
  if (!frame.power) {
    snprintf(payload, sizeof(payload), "{\"power\":false}");
  } else {
    if (!sensorHasValidDisplayFrame() ||
        sensorTimeSinceLastDisplayFrame() >= FRAME_TIMEOUT_MS) {
      return false;
    }
    const uint8_t setTemp = sensorGetSetTemp();
    if (setTemp < 16 || setTemp > 30) return false;
    snprintf(payload, sizeof(payload), "{\"temp\":%u}", setTemp);
  }

  irHealthSetEnabled(true);
  baselineTotal = stats.totalCommands;
  attempts++;
  waitingResult = true;
  commandSentMs = now;
  Serial.printf("[Startup IR] Attempt %u/%u using current AC state: %s\n",
                attempts, STARTUP_IR_MAX_ATTEMPTS, payload);
  controllHandleCommand(payload, strlen(payload));
  return false;
}
}  // namespace

void TaskControl(void* param) {
  if (!runtimeRegisterCurrentTask(RUNTIME_CONTROL, "CtrlTask")) vTaskDelete(NULL);

  IrCmdMsg msg;
  for (;;) {
    bool commandJustHandled = false;
    if (g_irCmdQueue && xQueueReceive(g_irCmdQueue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
      commandJustHandled = true;
      controllHandleCommand(msg.payload, msg.len);
    }
    startupIrValidationService(commandJustHandled);
    esp_task_wdt_reset();
    runtimeHeartbeat(RUNTIME_CONTROL);
  }
}
