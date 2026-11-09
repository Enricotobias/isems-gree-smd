#pragma once
#include <Arduino.h>

// Uncomment untuk enable debug output
// #define DEBUG_SENSOR_RAW      // Raw data dari serial
// #define DEBUG_SENSOR_CLEAN    // Data setelah DLE strip
// #define DEBUG_SENSOR_FRAMES   // Frame scanning info

#define G_MB_RX_PIN    20       // IO20 UART0 RX <- G_MB_RX (mainboard via R10/R11 divider)
#define G_D_TX_PIN     3        // IO3  UART1 RX <- G_D_TX  (display via R12/R13 divider)
#define OD_OPTO_RX_PIN 7       // IO21 UART2 RX <- OD_OPTO_RX (outdoor-indoor comm)
#define GREE_BAUD      2400
#define OD_OPTO_BAUD   1200
#define FRAME_LEN      24
#define FRAME_GAP_MS   150
#define FRAME_TIMEOUT_MS      30000   // 30 detik — komunikasi dianggap putus
#define FRAME_TIMEOUT_OFF_MS  300000  // 5 menit — toleransi saat AC OFF

// Frame dari Mainboard (TX) - header 0x1E 0x14`
struct BPFrame {
  bool   valid;
  bool   power;
  uint8_t current_temp; // room temperature
  float  evap_temp;     // evaporator temperature (from OD_OPTO_RX)
  bool   evap_temp_valid; // whether evap_temp is valid
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
bool sensorRefreshMainboardFrameHeader(uint32_t waitMs);

// Mainboard frame (TX) - room temp, errors, etc
BPFrame sensorGetLastFrame();
bool sensorGetMainboardSnapshot(BPFrame& frame, unsigned long& frameMs);
bool sensorHasValidFrame();
bool sensorHasInferredCommError();
unsigned long sensorInferredSinceMs();
unsigned long sensorTimeSinceLastFrame();
bool sensorIsHealthy();
unsigned long sensorGetLastValidFrameMs();  // Get timestamp of last valid frame
unsigned long sensorGetMainboardByteCount();
unsigned long sensorTimeSinceLastMainboardByte();
unsigned long sensorGetMainboardUartBatchLen();
String sensorGetMainboardUartSampleHex();
String sensorGetLastOutdoorFrameHex();
unsigned long sensorTimeSinceLastOutdoorFrame();
unsigned long sensorGetOutdoorByteCount();
unsigned long sensorTimeSinceLastOutdoorByte();
unsigned long sensorGetOutdoorUartBatchLen();
String sensorGetOutdoorUartSampleHex();
bool sensorHasValidEvapTemp();
float sensorGetEvapTemp();

// Display frame (RX) - set temp, user settings
DisplayFrame sensorGetLastDisplayFrame();
bool sensorHasValidDisplayFrame();
unsigned long sensorTimeSinceLastDisplayFrame();
bool sensorIsDisplayHealthy();
unsigned long sensorGetLastValidDisplayFrameMs();
uint8_t sensorGetSetTemp();  // Quick getter for set temperature
unsigned long sensorGetDisplayByteCount();
unsigned long sensorTimeSinceLastDisplayByte();
String sensorGetDisplayUartSampleHex();

extern unsigned long uartErrorCount;
