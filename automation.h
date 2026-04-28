#pragma once
#include <Arduino.h>
 
extern volatile bool forceScheduleCheck;

void automationInit();
void automationCheck();
void monitorExternalChange();

// Set expected state dari MQTT command.
// NOTE: Jika MQTT mematikan AC (power=false), masuk manual recovery agar automation
// tidak langsung melawan/resync.
void automationSetExpectedStateFromMqtt(bool power, const String& mode, const String& fan, uint8_t setTemp);
