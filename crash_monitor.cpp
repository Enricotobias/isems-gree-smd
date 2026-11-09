#include "crash_monitor.h"
#include "data.h"
#include "config.h"

#include <ArduinoJson.h>
#include "LittleFS.h"

RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t crashCount = 0;

// === Counter Definitions (akan di-increment dari modul lain) ===
uint32_t irSendCount = 0;
uint32_t autoOnCount = 0;
uint32_t autoOffCount = 0;
uint32_t manualOverrideCount = 0;

#define CRASH_STATS_FILE "/crash_stats.json"

static void loadCrashStats(uint32_t& totalBoots, uint32_t& totalCrashes,
                            uint32_t& wdtResets, uint32_t& panicResets,
                            uint32_t& brownoutResets, uint32_t& lastCrashTs) {
  totalBoots = totalCrashes = wdtResets = panicResets = brownoutResets = lastCrashTs = 0;
  if (!LittleFS.exists(CRASH_STATS_FILE)) return;
  File f = LittleFS.open(CRASH_STATS_FILE, "r");
  if (!f) return;
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, f) == DeserializationError::Ok) {
    totalBoots     = doc["tb"] | 0;
    totalCrashes   = doc["tc"] | 0;
    wdtResets      = doc["wr"] | 0;
    panicResets    = doc["pr"] | 0;
    brownoutResets = doc["bo"] | 0;
    lastCrashTs    = doc["lc"] | 0;
  }
  f.close();
}

static void saveCrashStats(uint32_t totalBoots, uint32_t totalCrashes,
                            uint32_t wdtResets, uint32_t panicResets,
                            uint32_t brownoutResets, uint32_t lastCrashTs) {
  File f = LittleFS.open(CRASH_STATS_FILE, "w");
  if (!f) return;
  StaticJsonDocument<256> doc;
  doc["tb"] = totalBoots;
  doc["tc"] = totalCrashes;
  doc["wr"] = wdtResets;
  doc["pr"] = panicResets;
  doc["bo"] = brownoutResets;
  doc["lc"] = lastCrashTs;
  serializeJson(doc, f);
  f.close();
}

String getDetailedResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_POWERON: return "Power On / Cold Boot";
    case ESP_RST_EXT: return "External Reset Pin";
    case ESP_RST_SW: return "Software Restart (ESP.restart)";
    case ESP_RST_PANIC: return "Exception/Panic (Code Error)";
    case ESP_RST_INT_WDT: return "Interrupt Watchdog Timeout";
    case ESP_RST_TASK_WDT: return "Task Watchdog Timeout";
    case ESP_RST_WDT: return "Watchdog Reset";
    case ESP_RST_DEEPSLEEP: return "Wake from Deep Sleep";
    case ESP_RST_BROWNOUT: return "Brownout Detector (Low Voltage)";
    case ESP_RST_SDIO: return "SDIO Reset";
    case ESP_RST_USB: return "USB Peripheral Reset";
    case ESP_RST_JTAG: return "JTAG Reset";
    case ESP_RST_EFUSE: return "eFuse Error Reset";
    case ESP_RST_PWR_GLITCH: return "Power Glitch Reset";
    case ESP_RST_CPU_LOCKUP: return "CPU Lockup / Double Exception";
    default: return "Unknown Reset (" + String((int)reason) + ")";
  }
}

void crashMonitorInit() {
  bootCount++;
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║     CRASH MONITOR INIT                ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  uint32_t totalBoots, totalCrashes, wdtResets, panicResets, brownoutResets, lastCrashTs;
  loadCrashStats(totalBoots, totalCrashes, wdtResets, panicResets, brownoutResets, lastCrashTs);
  
  esp_reset_reason_t reason = esp_reset_reason();
  bool wasCrash = false;
  String resetReason = getDetailedResetReason();
  
  Serial.printf("Reset Reason: %s\n", resetReason.c_str());
  Serial.printf("Boot Count (RTC): %d\n", bootCount);
  
  switch(reason) {
    case ESP_RST_PANIC:
    case ESP_RST_CPU_LOCKUP:
      crashCount++;
      totalCrashes++;
      panicResets++;
      wasCrash = true;
      Serial.println("⚠️ CRASH TYPE: Exception/Panic");
      break;
      
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      crashCount++;
      totalCrashes++;
      wdtResets++;
      wasCrash = true;
      Serial.println("⚠️ CRASH TYPE: Watchdog Timeout");
      break;
      
    case ESP_RST_BROWNOUT:
      brownoutResets++;
      Serial.println("⚠️ BROWNOUT RESET - Check Power Supply!");
      break;
      
    case ESP_RST_POWERON:
      Serial.println("✅ Clean Power On Boot");
      break;
      
    case ESP_RST_SW:
      Serial.println("✅ Software Restart (Normal)");
      break;
      
    default:
      Serial.println("ℹ️ Other Reset Type");
  }
  
  if (wasCrash) {
    lastCrashTs = (uint32_t)millis();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║      🚨 CRASH DETECTED 🚨             ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("Total Crashes: %d\n", totalCrashes);
    Serial.printf("WDT Resets: %d\n", wdtResets);
    Serial.printf("Panic Resets: %d\n", panicResets);
  }
  
  totalBoots++;
  saveCrashStats(totalBoots, totalCrashes, wdtResets, panicResets, brownoutResets, lastCrashTs);
  
  float crashRate = (totalBoots > 0) ? (totalCrashes * 100.0 / totalBoots) : 0;
  
  Serial.println("────────────────────────────────────────");
  Serial.printf("📊 STATISTICS:\n");
  Serial.printf("   Total Boots:    %d\n", totalBoots);
  Serial.printf("   Total Crashes:  %d\n", totalCrashes);
  Serial.printf("   Crash Rate:     %.2f%%\n", crashRate);
  Serial.printf("   Brownouts:      %d\n", brownoutResets);
  Serial.println("────────────────────────────────────────");
  
  if (crashRate > 15.0 && totalBoots > 10) {
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║   🚨 CRITICAL CRASH RATE 🚨           ║");
    Serial.println("║   Preserving config, resetting stats  ║");
    Serial.println("╚════════════════════════════════════════╝");
    delay(2000);
    saveCrashStats(0, 0, 0, 0, 0, 0);
    ESP.restart();
  }
}

CrashStats getCrashStats() {
  CrashStats stats = {0};
  uint32_t lastCrashTs;
  loadCrashStats(stats.totalBoots, stats.totalCrashes, stats.wdtResets,
                 stats.panicResets, stats.brownoutResets, lastCrashTs);
  stats.lastCrashTimestamp = lastCrashTs;
  if (stats.totalBoots > 0) {
    stats.crashRate = stats.totalCrashes * 100.0 / stats.totalBoots;
  }
  return stats;
}
