#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define IR_CMD_QUEUE_LEN 8
#define IR_CMD_STR_MAX 512

struct IrCmdMsg {
  char payload[IR_CMD_STR_MAX];
  size_t len;
};

extern QueueHandle_t g_irCmdQueue;
extern SemaphoreHandle_t g_cfgMutex;
extern SemaphoreHandle_t g_sensorMutex;

#define CFG_LOCK()   xSemaphoreTake(g_cfgMutex, pdMS_TO_TICKS(200))
#define CFG_UNLOCK() xSemaphoreGive(g_cfgMutex)

#define SENSOR_LOCK()   xSemaphoreTake(g_sensorMutex, pdMS_TO_TICKS(100))
#define SENSOR_UNLOCK() xSemaphoreGive(g_sensorMutex)

extern TaskHandle_t g_taskNetwork;
extern TaskHandle_t g_taskSensor;
extern TaskHandle_t g_taskAutomation;
extern TaskHandle_t g_taskControl;
extern TaskHandle_t g_taskHealth;

// Diagnostics: report stack high-water marks for created tasks
void reportStackHighWaterMarks();
