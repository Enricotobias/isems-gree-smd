#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_err.h"

enum RuntimeTaskId : uint8_t { RUNTIME_NET = 0, RUNTIME_SENSOR, RUNTIME_AUTO, RUNTIME_CONTROL, RUNTIME_HEALTH, RUNTIME_TASK_COUNT };
constexpr uint8_t RUNTIME_HEALTH_REQUIRED_STRIKES = 2;
bool runtimeRegisterCurrentTask(RuntimeTaskId id, const char* name);
void runtimeHeartbeat(RuntimeTaskId id);
bool runtimeCreateTaskChecked(BaseType_t result, const char* name);
bool runtimeHealthCheck();
void runtimeRecordFault(const char* reason);
