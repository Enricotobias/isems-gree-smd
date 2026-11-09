#include "runtime_health.h"
#include "rtos_common.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include <string.h>

RTC_DATA_ATTR static char runtimeFaultReason[64] = {};

struct RuntimeEntry { TaskHandle_t handle; uint32_t lastBeat; uint8_t strikes; const char* name; };
static RuntimeEntry entries[RUNTIME_TASK_COUNT] = {};
static const uint32_t staleMs[RUNTIME_TASK_COUNT] = { 90000UL, 3000UL, 3000UL, 3000UL, 15000UL };

bool runtimeRegisterCurrentTask(RuntimeTaskId id, const char* name) {
  if (id >= RUNTIME_TASK_COUNT) return false;
  const esp_err_t result = esp_task_wdt_add(NULL);
  if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
    runtimeRecordFault("task_wdt_add");
    return false;
  }
  entries[id] = { xTaskGetCurrentTaskHandle(), millis(), 0, name };
  return true;
}
void runtimeHeartbeat(RuntimeTaskId id) { if (id < RUNTIME_TASK_COUNT) { entries[id].lastBeat = millis(); entries[id].strikes = 0; } }
bool runtimeCreateTaskChecked(BaseType_t result, const char* name) { if (result == pdPASS) return true; runtimeRecordFault(name); return false; }
void runtimeRecordFault(const char* reason) { strncpy(runtimeFaultReason, reason ? reason : "unknown", sizeof(runtimeFaultReason) - 1); runtimeFaultReason[sizeof(runtimeFaultReason) - 1] = '\0'; Serial.printf("[RUNTIME] fault=%s heap=%u\n", runtimeFaultReason, ESP.getFreeHeap()); delay(20); ESP.restart(); }
bool runtimeHealthCheck() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < RUNTIME_TASK_COUNT; ++i) {
    if (!entries[i].handle) continue;
    const UBaseType_t stackMin = uxTaskGetStackHighWaterMark(entries[i].handle);
    if (stackMin < 128) Serial.printf("[RUNTIME] low stack task=%s words=%u\n", entries[i].name, (unsigned)stackMin);
    if (now - entries[i].lastBeat > staleMs[i]) {
      if (++entries[i].strikes >= RUNTIME_HEALTH_REQUIRED_STRIKES) { runtimeRecordFault(entries[i].name); return false; }
    } else entries[i].strikes = 0;
  }
  return true;
}
