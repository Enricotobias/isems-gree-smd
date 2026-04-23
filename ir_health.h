#pragma once
#include <Arduino.h>

// ==========================================
// IR HEALTH MONITORING SYSTEM
// ==========================================
// System ini melakukan validasi otomatis setiap IR command yang dikirim
// dengan membandingkan expected state vs actual display frame response.
//
// HASIL VALIDASI:
// 1. SUCCESS      - Display merespon sesuai command (ACK: ir_validation/success)
// 2. MISMATCH     - Display merespon tapi tidak sesuai (ACK: ir_validation/mismatch)
// 3. NO_RESPONSE  - Display tidak merespon sama sekali (ACK: ir_validation/no_response)
//
// ACK dikirim ke MQTT topic: isems/iot/{device_id}/ack
// Format: {"cmd":"ir_validation", "status":"success/mismatch/no_response", "detail":"..."}
// ==========================================

// IR Health Status
struct IRHealthStats {
  unsigned long totalCommands;
  unsigned long successCount;
  unsigned long noResponseCount;
  unsigned long mismatchCount;
  
  // Last command tracking
  bool lastCommandSuccess;
  String lastCommandType;      // "power", "temp", "mode", "fan"
  String lastExpected;
  String lastActual;
  unsigned long lastCommandMs;
  
  // Success rate
  float getSuccessRate() const {
    if (totalCommands == 0) return 100.0f;
    return (float)successCount * 100.0f / (float)totalCommands;
  }
  
  // Health status
  String getHealthStatus() const {
    float rate = getSuccessRate();
    if (rate >= 95.0f) return "EXCELLENT";
    if (rate >= 85.0f) return "GOOD";
    if (rate >= 70.0f) return "FAIR";
    return "POOR";
  }
};

// Initialize IR health monitoring
void irHealthInit();

// Set expected state before sending IR command
void irHealthSetExpectedState(bool hasPower, bool power,
                              bool hasMode, const String& mode,
                              bool hasFan, const String& fan,
                              bool hasTemp, uint8_t setTemp);

// Check response after IR command (call after 2-3 seconds)
void irHealthCheckResponse();

// Get health statistics
IRHealthStats irHealthGetStats();

// Reset statistics
void irHealthReset();

// Enable/disable health monitoring
void irHealthSetEnabled(bool enabled);
bool irHealthIsEnabled();
