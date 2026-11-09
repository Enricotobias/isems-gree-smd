#include "tasks.h"
#include "rtos_common.h"
#include "esp_task_wdt.h"
#include "runtime_health.h"

void TaskHealth(void* param) {
  static_assert(RUNTIME_HEALTH_REQUIRED_STRIKES == 2, "health supervisor requires two consecutive stale samples");
  if (!runtimeRegisterCurrentTask(RUNTIME_HEALTH, "Health")) vTaskDelete(NULL);

  for (;;) {
    esp_task_wdt_reset();
    runtimeHeartbeat(RUNTIME_HEALTH);
    runtimeHealthCheck();

    static unsigned long lastHealthLog = 0;
    if (millis() - lastHealthLog >= 300000UL) {
      lastHealthLog = millis();
      Serial.printf("[HEALTH] Heap=%u Min=%u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
