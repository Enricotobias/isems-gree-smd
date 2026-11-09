#pragma once

#include <Arduino.h>

struct OfflineLogRecord {
  uint32_t epoch;
  int16_t room_temp;
  uint8_t power;
  uint8_t flags;
};

bool offlineLogInit();
bool offlineLogAppend(uint32_t epoch, bool power, int16_t roomTemp);
bool offlineLogPeek(OfflineLogRecord& record);
bool offlineLogPop();
uint32_t offlineLogCount();
uint32_t offlineLogCapacity();
bool offlineLogReady();
size_t offlineLogFileBytes();
size_t offlineLogFsUsedBytes();
size_t offlineLogFsTotalBytes();
size_t offlineLogFsFreeBytes();
