#pragma once
#include <Arduino.h>

void logInit();
void logPrint(const String& s);
void logPrintln(const String& s);
void logPrintln(const char* s);
void logPrintln(); 
void logPrintf(const char* format, ...);
void logLoop();

// MQTT Logging Control
void logSetMqttEnabled(bool enabled);
bool logGetMqttEnabled();
void logSetMqttPublisher(void (*publisher)(const String&));