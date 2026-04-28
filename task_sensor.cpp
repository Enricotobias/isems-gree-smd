#include "tasks.h"
#include "rtos_common.h"
#include "sensor.h"
#include "esp_task_wdt.h"

void TaskSensor(void* param) {
  esp_task_wdt_add(NULL);

  for (;;) {
    sensorPoll();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
