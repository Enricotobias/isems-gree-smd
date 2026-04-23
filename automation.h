#pragma once
#include <Arduino.h>
 
extern bool forceScheduleCheck;

void automationInit();
void automationCheck();
void monitorExternalChange();
void automationSetTempCheckInterval(unsigned long intervalMs);

// Set expected state dari MQTT command.
// NOTE: Jika MQTT mematikan AC (power=false), masuk manual recovery agar automation
// tidak langsung melawan/resync.
void automationSetExpectedStateFromMqtt(bool power, const String& mode, const String& fan, uint8_t setTemp);

