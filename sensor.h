#pragma once
#include <Arduino.h>

// Uncomment untuk enable debug output
// #define DEBUG_SENSOR_RAW      // Raw data dari serial
// #define DEBUG_SENSOR_CLEAN    // Data setelah DLE strip
// #define DEBUG_SENSOR_FRAMES   // Frame scanning info

#define G_MB_RX_PIN    20       // IO20 UART0 RX <- G_MB_RX (mainboard via R10/R11 divider)
#define G_D_TX_PIN     3        // IO3  UART1 RX <- G_D_TX  (display via R12/R13 divider)
#define GREE_BAUD      2400
#define FRAME_LEN      24
#define FRAME_GAP_MS   150
#define FRAME_TIMEOUT_MS 30000  // 30 detik timeout jika tidak ada frame

// Frame dari Mainboard (TX) - header 0x1E 0x14
struct BPFrame {
  bool   valid;
  bool   power;
  uint8_t current_temp; // room temperature
  String mode;
  String fan;
  uint8_t status;
  String statusHex;
  String code_hex;
  String error_val16;
  String errors;   // comma-separated list, "-" jika tidak ada
  String frameHex; // hex dump (trimmed)
};

// Frame dari Display (RX) - header 0x68 0x14
struct DisplayFrame {
  bool    valid;
  bool    power;
  uint8_t set_temp;     // set temperature (target)
  String  mode;
  String  fan;
  String  frameHex;
};

void sensorInit();
void sensorPoll();

// Mainboard frame (TX) - room temp, errors, etc
BPFrame sensorGetLastFrame();
bool sensorHasValidFrame();
bool sensorHasInferredCommError();
unsigned long sensorInferredSinceMs();
unsigned long sensorTimeSinceLastFrame();
bool sensorIsHealthy();
unsigned long sensorGetLastValidFrameMs();  // Get timestamp of last valid frame
unsigned long sensorGetMainboardByteCount();

// Display frame (RX) - set temp, user settings
DisplayFrame sensorGetLastDisplayFrame();
bool sensorHasValidDisplayFrame();
unsigned long sensorTimeSinceLastDisplayFrame();
bool sensorIsDisplayHealthy();
unsigned long sensorGetLastValidDisplayFrameMs();
uint8_t sensorGetSetTemp();  // Quick getter for set temperature
unsigned long sensorGetDisplayByteCount();

extern unsigned long uartErrorCount;
