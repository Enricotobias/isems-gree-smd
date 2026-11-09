#include "tasks.h"
#include "rtos_common.h"
#include "sensor.h"
#include "esp_task_wdt.h"
#include "runtime_health.h"

void TaskSensor(void* param) {
  if (!runtimeRegisterCurrentTask(RUNTIME_SENSOR, "SnsTask")) vTaskDelete(NULL);

  for (;;) {
    sensorPoll();
    runtimeHeartbeat(RUNTIME_SENSOR);
    esp_task_wdt_reset();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
  }
}
