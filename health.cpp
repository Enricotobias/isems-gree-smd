#include "tasks.h"
#include "rtos_common.h"
#include "config.h"
#include "esp_task_wdt.h"

void TaskHealth(void* param) {
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
