#include "tasks.h"
#include "rtos_common.h"
#include "controll.h"
#include "esp_task_wdt.h"

void TaskControl(void* param) {
  esp_task_wdt_add(NULL);

  IrCmdMsg msg;

  for (;;) {
    if (g_irCmdQueue && xQueueReceive(g_irCmdQueue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
      controllHandleCommand(msg.payload, msg.len);
    }
    esp_task_wdt_reset();
  }
}
