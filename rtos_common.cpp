#include "rtos_common.h"
QueueHandle_t g_irCmdQueue = nullptr;
SemaphoreHandle_t g_cfgMutex = nullptr;
SemaphoreHandle_t g_sensorMutex = nullptr;
TaskHandle_t g_taskNetwork = nullptr;
TaskHandle_t g_taskSensor = nullptr;
TaskHandle_t g_taskAutomation = nullptr;
TaskHandle_t g_taskControl = nullptr;
TaskHandle_t g_taskHealth = nullptr;

// Report stack high-water marks for tasks to aid debugging of stack overflows
void reportStackHighWaterMarks() {
  UBaseType_t mark;
  Serial.println("[RTOS] Stack high-water marks (bytes left):");
  mark = (g_taskNetwork != nullptr) ? uxTaskGetStackHighWaterMark(g_taskNetwork) : 0;
  Serial.printf(" - NetTask: %u\n", (unsigned)mark);
  mark = (g_taskSensor != nullptr) ? uxTaskGetStackHighWaterMark(g_taskSensor) : 0;
  Serial.printf(" - SnsTask: %u\n", (unsigned)mark);
  mark = (g_taskAutomation != nullptr) ? uxTaskGetStackHighWaterMark(g_taskAutomation) : 0;
  Serial.printf(" - AutoTask: %u\n", (unsigned)mark);
  mark = (g_taskControl != nullptr) ? uxTaskGetStackHighWaterMark(g_taskControl) : 0;
  Serial.printf(" - CtrlTask: %u\n", (unsigned)mark);
  mark = (g_taskHealth != nullptr) ? uxTaskGetStackHighWaterMark(g_taskHealth) : 0;
  Serial.printf(" - Health: %u\n", (unsigned)mark);
  Serial.printf(" - FreeHeap: %u bytes\n", ESP.getFreeHeap());
}
