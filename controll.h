#pragma once
#include <Arduino.h>

void controllInit();
void controllHandleCommand(const char* payload, size_t length);

// Fungsi helper untuk custom fan control
void update_fan_custom(const char* fanSpeed);