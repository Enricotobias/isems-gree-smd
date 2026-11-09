#include "tasks.h"
#include "rtos_common.h"
#include "automation.h"
#include "activity.h"
#include "esp_task_wdt.h"
#include "runtime_health.h"

void TaskAutomation(void* param) {
  if (!runtimeRegisterCurrentTask(RUNTIME_AUTO, "AutoTask")) vTaskDelete(NULL);

  automationInit();
  activityInit();

  for (;;) {
    // Wake on: (1) MQTT command notification, (2) sensor frame change, or (3) 200ms timeout.
    // 200ms ensures external changes are detected promptly even without explicit notification.
    uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

    // Always monitor for external changes (physical remote / panel button).
    // This MUST run frequently to detect AC state changes from IR remote.
    monitorExternalChange();

    // Run schedule/automation logic
    if (g_cfgMutex && CFG_LOCK() == pdTRUE) {
      automationCheck();
      CFG_UNLOCK();
    }
    runtimeHeartbeat(RUNTIME_AUTO);

    esp_task_wdt_reset();
  }
}
