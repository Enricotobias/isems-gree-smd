#include "rtos_common.h"
#include "config.h"

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
	// Serial monitor output disabled.
}
