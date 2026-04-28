#pragma once

void TaskNetwork_MQTT(void* param);
void TaskSensor(void* param);
void TaskAutomation(void* param);
void TaskControl(void* param);
void TaskHealth(void* param);

extern bool tryConnectWiFi();
extern void checkMemoryHealth();
extern void updateLEDStatus();
extern void checkNetworkHealth();
