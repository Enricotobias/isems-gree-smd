#pragma once
#include <Arduino.h>

void controllInit();
void controllHandleCommand(const char* payload, size_t length);
bool controllEnqueueCommand(const char* payload, size_t len);

// Fungsi helper untuk custom fan control
void update_fan_custom(const char* fanSpeed);