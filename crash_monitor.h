#ifndef CRASH_MONITOR_H
#define CRASH_MONITOR_H

#include <Arduino.h>
#include "config.h"

// === RTC Memory (Bertahan saat crash, hilang saat power off) ===
extern uint32_t bootCount;
extern uint32_t crashCount;

// === Global Counters (untuk runtime tracking) ===
extern uint32_t irSendCount;
extern uint32_t autoOnCount;
extern uint32_t autoOffCount;
extern uint32_t manualOverrideCount;

struct CrashStats {
  uint32_t totalBoots;
  uint32_t totalCrashes;
  uint32_t wdtResets;
  uint32_t panicResets;
  uint32_t brownoutResets;
  uint32_t lastCrashTimestamp;
  float crashRate;
};

// === Public Functions ===
void crashMonitorInit();
CrashStats getCrashStats();
void publishCrashDiagnostics();
String getDetailedResetReason();

#endif