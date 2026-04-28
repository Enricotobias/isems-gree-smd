#include "sensor.h"
#include "config.h"
#include "rtos_common.h"


// GREE BP parser - ported from Python reference implementation
// Handles DLE pairs, error decoding, and proper frame parsing
// Now supports both Mainboard TX (0x1E 0x14) and Display RX (0x68 0x14) frames

static String describeBPInfo(const BPFrame& d);
static String describeDisplayInfo(const DisplayFrame& d);

static BPFrame lastLoggedFrame;
static bool lastLoggedFrameValid = false;

// Display frame tracking
static DisplayFrame lastLoggedDisplayFrame;
static bool lastLoggedDisplayFrameValid = false;

// Mainboard (TX) buffers - IO20 (G_MB_RX via UART0)
static uint8_t rxBuf[512];
static uint8_t cleanBuf[512];
static size_t rxLen = 0;
static uint32_t lastByteMs = 0;
static uint32_t lastValidFrameMs = 0;
static unsigned long mainboardByteCount = 0;

// Display (RX) buffers - IO3 (G_D_TX via UART1)
static uint8_t dispRxBuf[512];
static size_t dispRxLen = 0;
static uint32_t dispLastByteMs = 0;
static uint32_t dispLastValidFrameMs = 0;
static unsigned long displayByteCount = 0;

static BPFrame lastFrame = {false,false,0,"","",0,"","","",""};
static DisplayFrame lastDisplayFrame = {false,false,0,"","",""};
static bool haveFrame = false;
static bool haveDisplayFrame = false;
static bool lastPrintedPower = false;
static bool everHadValidFrame = false;
static bool inferredCommError = false;
static unsigned long inferredSinceMs = 0;
static uint8_t lastPrintedTemp = 0;
static String lastPrintedMode = "";
static String lastPrintedFan = "";
static bool firstFrameReceived = false;
static uint32_t lastSerialResetMs = 0;
unsigned long uartErrorCount = 0;
static bool uartErrorFlagActive = false;  // Flag to prevent spam counting errors in same invalid period 
static bool mainboardStartupDebugLogged = false;

// Header sesuai Python: HDR_BP = b"\x1E\x14"
static const uint8_t HDR_BP[2] = {0x1E, 0x14};
static const uint8_t POWER_MASK_INFERRED = 0x30;

// MODE_MAP sesuai Python implementation
static const char* MODE_MAP_LOOKUP(uint8_t core) {
  switch (core) {
    case 0x1: return "FAN";
    case 0x2: return "COOL";
    case 0x3: return "DRY";
    case 0x4: return "HEAT";
    case 0x5: return "AUTO";
    case 0xA: return "COOL";  // 0xA juga map ke COOL seperti Python
    default:  return "?";
  }
}

static bool isBPFrameChanged(const BPFrame& a, const BPFrame& b) {
  if (!b.valid) return true;
  if (!a.valid) return true;

  if (a.power != b.power) return true;
  if (a.current_temp != b.current_temp) return true;
  if (a.mode != b.mode) return true;
  if (a.fan != b.fan) return true;
  if (a.errors != b.errors) return true;

  return false;
}

// strip DLE pairs sesuai implementasi Python
size_t stripDLE(const uint8_t* in, size_t n, uint8_t* out) {
  size_t i = 0, o = 0;
  while (i < n) {
    uint8_t b = in[i];
    if (b == 0x10) {
      if (i + 1 < n && in[i+1] == 0x10) { 
        out[o++] = 0x10; 
        i += 2; 
      } else { 
        i++; 
      }
    } else {
      out[o++] = b; 
      i++;
    }
  }
  return o;
}

static String hexdump_line(const uint8_t* b, size_t len, size_t maxlen = 24) {
  size_t upto = (len < maxlen) ? len : maxlen;
  String s;
  s.reserve(upto * 3 + 8);
  for (size_t i = 0; i < upto; ++i) {
    if (i) s += " ";
    char buf[4];
    sprintf(buf, "%02X", b[i]);
    s += String(buf);
  }
  if (len > maxlen) s += " ...";
  return s;
}

// ============================================================================
// DECODE ERROR BITS (UPDATED MAPPING)
// ============================================================================
static void decode_error_bits_from_frame(const uint8_t* frame, uint16_t &val16, uint8_t &low, uint8_t &high, String &outErrors) {
  if (!frame) { 
    val16 = 0; low = 0; high = 0; outErrors = "-"; 
    return; 
  }
  
  low = frame[4];    // Byte 4 (Error Low)
  high = frame[13];  // Byte 13 (Error High)
  val16 = ((uint16_t)high << 8) | (uint16_t)low;
  
  // === VALIDATED MAPPING (LOW BYTE) ===
  const char* LOW_BIT_MAP[8] = {
    "F1",     // Bit 0: Indoor Ambient Sensor (Open/Short)
    "F2",     // Bit 1: Indoor Evap Sensor (Open/Short)
    "F3",     // Bit 2: Outdoor Ambient Sensor
    "F4",     // Bit 3: Outdoor Condenser Sensor
    "F5",     // Bit 4: Outdoor Discharge Sensor
    "F6",     // Bit 5: Overheat / Frequency Limit
    "F8",     // Bit 6: Indoor Fan Feedback Error
    "E1,E3"   // Bit 7: High/Low Pressure Protection (Shared)
  };
  
  // === VALIDATED MAPPING (HIGH BYTE) ===
  const char* HIGH_BIT_MAP[8] = {
    "C5",     // Bit 0: Jumper Cap Missing/Fault
    "H6",     // Bit 1: Indoor Fan Motor Stuck/No Feedback
    "U8",     // Bit 2: PG Motor Zero-Crossing Error
    "P7",     // Bit 3: IPM Temperature Protection
    "E6",     // Bit 4: Communication Error (Primary)
    "E6",     // Bit 5: Communication Error (Secondary/Validated)
    "P8",     // Bit 6: Outdoor Module Over-Temp
    "PU"      // Bit 7: Capacitor Charge Loop Error
  };

  bool first = true;
  String acc;
  acc.reserve(64);
  
  // Check Low Byte
  for (int bit = 0; bit < 8; ++bit) {
    if (low & (1 << bit)) {
      if (!first) acc += ",";
      acc += String(LOW_BIT_MAP[bit]);
      first = false;
    }
  }
  
  // Check High Byte
  for (int bit = 0; bit < 8; ++bit) {
    if (high & (1 << bit)) {
      if (!first) acc += ",";
      acc += String(HIGH_BIT_MAP[bit]);
      first = false;
    }
  }
  
  if (acc.length() == 0) acc = "-";
  outErrors = acc;
}

// fan decode - FIXED: Based on real binary data analysis (2026-01-26)
// Validated mappings:
// - flag_mode8=false, nibble=1: LOW ✓
// - flag_mode8=false, nibble=2: HIGH ✓
// - flag_mode8=false, nibble=5: LOW ✓ (was incorrectly mapped to AUTO)
// - flag_mode8=true, nibble=5: AUTO ✓
static String _decode_fan_label(const String& mode_label, uint8_t mode_raw, uint8_t fan_nib) {
  bool flag_mode8 = (mode_raw & 0x8) != 0;
  
  if (mode_label == "DRY") return String("LOW");
  
  if (mode_label == "FAN") {
    if (fan_nib == 0) return String("AUTO");
    else if (fan_nib == 1) return String(flag_mode8 ? "MED" : "LOW");
    else return String("HIGH");
  }
  
  if (flag_mode8) {
    // Mode 0xA (COOL with mode8 flag)
    switch (fan_nib) {
      case 0: return String("LOW");       // Unmapped guess
      case 1: return String("MED");       // ✓ Validated
      case 2: return String("TURBO");     // ✓ Validated
      case 3: return String("LOW");       // Unmapped guess
      case 4: return String("LOW");       // Unmapped guess
      case 5: return String("MED");      // ✓ Validated (HEX: 5A with mode=0xA)
      case 6: return String("TURBO");     // Unmapped guess
      default: { char buf[8]; sprintf(buf, "0x%X", fan_nib); return String(buf); }
    }
  } else {
    // Mode 0x2 (Normal COOL/HEAT)
    switch (fan_nib) {
      case 0: return String("LOW");       // Unmapped guess
      case 1: return String("LOW");       // ✓ Validated (HEX: 12)
      case 2: return String("HIGH");      // ✓ Validated (HEX: 22)
      case 3: return String("LOW");       // Unmapped guess
      case 4: return String("MED");       // Unmapped guess
      case 5: return String("LOW");       // ✓ FIXED! Was AUTO, now LOW (HEX: 52)
      case 6: return String("HIGH");      // Unmapped guess
      default: { char buf[8]; sprintf(buf, "0x%X", fan_nib); return String(buf); }
    }
  }
}

// parse_bp
static BPFrame parse_bp(const uint8_t* frame, size_t n) {
  BPFrame d;
  d.valid = false;
  d.power = false;
  d.current_temp = 0;
  d.mode = "?";
  d.fan = "?";
  d.status = 0;
  d.statusHex = "";
  d.code_hex = "";
  d.error_val16 = "";
  d.errors = "-";
  d.frameHex = "";

  if (n < FRAME_LEN) return d;
  if (!(frame[0] == HDR_BP[0] && frame[1] == HDR_BP[1])) return d;

  uint8_t room = frame[5];
  uint8_t status = frame[8];
  uint8_t b9 = frame[9];

  uint16_t code = ((uint16_t)status << 8) | (uint16_t)b9;
  bool power = (status & POWER_MASK_INFERRED) != 0;
  
  uint8_t mode_raw = status & 0x0F;
  uint8_t mode_core = mode_raw & 0x7;
  const char* mode_label_c = MODE_MAP_LOOKUP(mode_core);
  String mode_label = mode_label_c ? String(mode_label_c) : String("?");
  
  uint8_t fan_nib = (status >> 4) & 0x0F;
  String fan_label = _decode_fan_label(mode_label, mode_raw, fan_nib);

  uint16_t val16; 
  uint8_t low, high; 
  String active_errors;
  decode_error_bits_from_frame(frame, val16, low, high, active_errors);

  char code_hex_buf[8]; sprintf(code_hex_buf, "0x%04X", code);
  char err_hex_buf[8]; sprintf(err_hex_buf, "0x%04X", val16);

  d.valid = true;
  d.power = power;
  d.current_temp = room;
  d.mode = mode_label;
  d.fan = fan_label;
  d.status = status;
  { char stx[6]; sprintf(stx, "0x%02X", status); d.statusHex = String(stx); }
  d.code_hex = String(code_hex_buf);
  d.error_val16 = String(err_hex_buf);
  d.errors = active_errors;
  d.frameHex = hexdump_line(frame, n, 24);
  
  return d;
}

// ============================================================================
// DISPLAY FRAME PARSING (Header 0x68 0x14)
// ============================================================================
static const uint8_t HDR_DISPLAY[2] = {0x68, 0x14};

static bool isDisplayFrameChanged(const DisplayFrame& a, const DisplayFrame& b) {
  if (!b.valid) return true;
  if (!a.valid) return true;
  if (a.power != b.power) return true;
  if (a.set_temp != b.set_temp) return true;
  if (a.mode != b.mode) return true;
  if (a.fan != b.fan) return true;
  return false;
}

static DisplayFrame parse_display_frame(const uint8_t* frame, size_t n) {
  DisplayFrame d;
  d.valid = false;
  d.power = false;
  d.set_temp = 0;
  d.mode = "?";
  d.fan = "?";
  d.frameHex = "";

  if (n < FRAME_LEN) return d;
  if (!(frame[0] == HDR_DISPLAY[0] && frame[1] == HDR_DISPLAY[1])) return d;

  // Byte[2]: MODE (0x00=OFF, 0x08=AUTO, 0x09=COOL, 0x0A=DRY, 0x0B=FAN/HEAT)
  uint8_t mode_byte = frame[2];
  
  // Power detection: OFF if Byte[2]=0x00 or Byte[7]=0x80
  bool power = (mode_byte != 0x00) && (frame[7] != 0x80);
  
  // Mode decoding
  String mode_label = "?";
  if (!power || mode_byte == 0x00) {
    mode_label = "OFF";
  } else {
    switch (mode_byte) {
      case 0x08: mode_label = "AUTO"; break;
      case 0x09: mode_label = "COOL"; break;
      case 0x0A: mode_label = "DRY"; break;
      case 0x0B: 
        // FAN or HEAT - check Byte[16] high nibble
        if ((frame[16] >> 4) == 0x0C) {
          mode_label = "HEAT";
        } else {
          mode_label = "FAN";
        }
        break;
      default: 
        char buf[8]; 
        sprintf(buf, "0x%02X", mode_byte); 
        mode_label = String(buf);
        break;
    }
  }

  // Byte[3]: High nibble = set_temp-16, Low nibble = fan
  uint8_t set_temp = ((frame[3] >> 4) & 0x0F) + 16;
  uint8_t fan_nibble = frame[3] & 0x0F;
  bool turbo_flag = (frame[5] & 0x01) != 0;  // Byte[5] bit 0 = TURBO
  
  // Fan speed decoding
  String fan_label;
  if (fan_nibble == 0) {
    fan_label = "LOW";
  } else if (fan_nibble == 1) {
    fan_label = "LOW";
  } else if (fan_nibble == 2) {
    fan_label = "MED";
  } else if (fan_nibble == 3) {
    if (turbo_flag) {
      fan_label = "TURBO";
    } else {
      fan_label = "HIGH";
    }
  } else if (fan_nibble == 4) {
    // Observed on real units when fan is effectively LOW.
    // Mapping this avoids noisy "0x4" labels that break monitor logic.
    fan_label = "LOW";
  } else {
    char buf[8]; 
    sprintf(buf, "0x%X", fan_nibble); 
    fan_label = String(buf);
  }

  d.valid = true;
  d.power = power;
  d.set_temp = set_temp;
  d.mode = mode_label;
  d.fan = fan_label;
  d.frameHex = hexdump_line(frame, n, 24);
  
  return d;
}

static String describeDisplayInfo(const DisplayFrame& d) {
  String s;
  s.reserve(80);
  s += "power="; s += d.power ? "ON" : "OFF";
  s += " set_temp=" + String(d.set_temp);
  s += " mode=" + d.mode;
  s += " fan=" + d.fan;
  return s;
}

void sensorInit() {
  // Initialize UART0 for Mainboard RX (IO20 - G_MB_RX)
  Serial0.begin(GREE_BAUD, SERIAL_8E1, G_MB_RX_PIN);
  while (Serial0.available()) Serial0.read();
  rxLen = 0;
  lastByteMs = millis();
  lastValidFrameMs = 0;
  mainboardByteCount = 0;
  haveFrame = false;
  lastFrame = {false,false,0,"","",0,"","","",""};
  firstFrameReceived = false;
  lastPrintedPower = false;
  lastPrintedTemp = 0;
  lastPrintedMode = "";
  lastPrintedFan = "";
  lastSerialResetMs = millis();
  
  // Initialize UART1 for Display RX (IO3 - G_D_TX)
  Serial1.begin(GREE_BAUD, SERIAL_8N1, G_D_TX_PIN);
  while (Serial1.available()) Serial1.read();
  dispRxLen = 0;
  dispLastByteMs = millis();
  dispLastValidFrameMs = 0;
  displayByteCount = 0;
  haveDisplayFrame = false;
  lastDisplayFrame = {false,false,0,"","",""};
  
  Serial.Println("[Sensor] Init done (MB=IO20/UART0, DSP=IO3/UART1)");
}

// ============================================================================
// HELPER: GET ERROR MEANING (DESKRIPSI LENGKAP)
// ============================================================================
static String get_error_meaning(String code) {
  // --- Low Byte Errors ---
  if (code == "F1") return "Sensor Ambien Indoor Rusak/Lepas"; // [cite: 15]
  if (code == "F2") return "Sensor Evaporator Indoor Rusak/Lepas"; // [cite: 15]
  if (code == "F3") return "Sensor Ambien Outdoor Rusak/Lepas"; // [cite: 15]
  if (code == "F4") return "Sensor Kondensor Outdoor Rusak/Lepas"; // [cite: 15]
  if (code == "F5") return "Sensor Discharge Outdoor Rusak/Lepas"; // [cite: 15]
  if (code == "F6") return "Overheat Protection / Limit"; // (Tidak ada di PDF, tapi tervalidasi fisik)
  if (code == "F8") return "Umpan Balik Fan Indoor Error"; 
  if (code == "E1,E3") return "Proteksi Tekanan Kompresor (Tinggi/Rendah)"; // [cite: 9]

  // --- High Byte Errors ---
  if (code == "C5") return "Jumper Cap Mainboard Hilang/Rusak"; // [cite: 4]
  if (code == "H6") return "Fan Motor Indoor Macet/Tidak Respon"; // [cite: 4]
  if (code == "U8") return "Malfungsi Sinyal Zero-Crossing"; // [cite: 28]
  if (code == "P7") return "Proteksi Overheat Modul IPM"; // 
  if (code == "E6") return "Komunikasi Indoor-Outdoor Putus"; // 
  if (code == "P8") return "Suhu Modul Outdoor Terlalu Tinggi"; // 
  if (code == "PU") return "Malfungsi Charging Kapasitor"; // 

  return "";
} 

// ============================================================================
// DESCRIBE INFO (UPDATED: SHOW CODE + MEANING)
// ============================================================================
static String describeBPInfo(const BPFrame& d) {
  String s;
  s.reserve(160);

  s += "power="; s += d.power ? "ON" : "OFF";
  s += " room_temp=" + String(d.current_temp);
  s += " mode=" + d.mode;
  s += " fan=" + d.fan;

  // Jika ada error, parse satu per satu untuk menampilkan artinya
  if (d.errors != "-" && d.errors.length() > 0) {
    s += " err=";
    
    // Parsing manual string error (contoh: "F1,E6")
    int start = 0;
    int commaIdx = d.errors.indexOf(',');
    bool first = true;

    while (commaIdx != -1) {
      if (!first) s += ", ";
      String code = d.errors.substring(start, commaIdx);
      String mean = get_error_meaning(code);
      s += code;
      if (mean.length() > 0) s += "(" + mean + ")";
      
      start = commaIdx + 1;
      commaIdx = d.errors.indexOf(',', start);
      first = false;
    }
    
    // Sisa terakhir
    if (!first) s += ", ";
    String lastCode = d.errors.substring(start);
    String lastMean = get_error_meaning(lastCode);
    s += lastCode;
    if (lastMean.length() > 0) s += "(" + lastMean + ")";
  }

  return s;
}

void sensorPoll() {
  // ============================================================================
  // MAINBOARD TX POLLING (UART0 - IO20)
  // ============================================================================
  
  // Serial recovery
  if (haveFrame && !sensorIsHealthy() && (millis() - lastSerialResetMs) > (FRAME_TIMEOUT_MS * 2)) {
    Serial0.end(); delay(100);
    Serial0.begin(GREE_BAUD, SERIAL_8E1, G_MB_RX_PIN);
    while (Serial0.available()) Serial0.read();
    rxLen = 0; lastSerialResetMs = millis();
  }
  
  // Read serial
  while (Serial0.available()) {
    if (rxLen >= sizeof(rxBuf) - 50) break;
    rxBuf[rxLen++] = (uint8_t)Serial0.read();
    mainboardByteCount++;
    lastByteMs = millis();
  }

  bool shouldProcess = (rxLen > 0 && (millis() - lastByteMs) > FRAME_GAP_MS);
  bool bufferNearlyFull = (rxLen > sizeof(rxBuf) / 2);
  
  if (shouldProcess || bufferNearlyFull) {
    size_t rawLen = rxLen;
    size_t cleanLen = stripDLE(rxBuf, rawLen, cleanBuf);
    rxLen = 0;
    bool validFrameFoundThisPass = false;
    int headerOffset = -1;
    
    size_t i = 0;
    while (i + FRAME_LEN <= cleanLen) {
      if (cleanBuf[i] == HDR_BP[0] && cleanBuf[i+1] == HDR_BP[1]) {
        headerOffset = (int)i;

        BPFrame d = parse_bp(&cleanBuf[i], FRAME_LEN);
        if (d.valid) {
          validFrameFoundThisPass = true;
          lastFrame = d;
          haveFrame = true;
          everHadValidFrame = true;

          // LOG JIKA BERUBAH
          if (!lastLoggedFrameValid || isBPFrameChanged(d, lastLoggedFrame)) {

            String hex = hexdump_line(&cleanBuf[i], FRAME_LEN);
            String info = describeBPInfo(d);

            uint8_t errLow  = cleanBuf[i + 4];
            uint8_t errHigh = cleanBuf[i + 13];
            uint16_t errRaw = ((uint16_t)errHigh << 8) | errLow;

            if (errRaw == 0) {
              Serial.Println("[MAINBOARD] BP_FRAME");
            } else {
              Serial.Println("[MAINBOARD] BP_FRAME_WITH_ERROR_BITS");
            }

            Serial.Printf("HEX   : %s\n", hex.c_str());
            Serial.Printf("INFO  : %s\n", info.c_str());
            Serial.Printf("ERROR : raw=0x%04X (byte4=%02X byte13=%02X)\n", errRaw, errLow, errHigh);
            
            lastLoggedFrame = d;
            lastLoggedFrameValid = true;
          }

          if (inferredCommError) {
            inferredCommError = false;
            Serial.Println("[MAINBOARD] BP_RECOVERED");
            Serial.Println("INFO  : BPFrame stream recovered");
          }

          // ═══════════════════════════════════════════════════════════════
          // Reset UART error flag ketika frame valid diterima
          // Siap untuk detect & count error berikutnya
          // ═══════════════════════════════════════════════════════════════
          if (uartErrorFlagActive) {
            uartErrorFlagActive = false;
            Serial.Println("[MAINBOARD] UART error flag reset (valid frame received)");
          }

          lastValidFrameMs = millis();
          
          bool hasChanged = !firstFrameReceived ||
                           (d.power != lastPrintedPower) ||
                           (d.current_temp != lastPrintedTemp) ||
                           (d.mode != lastPrintedMode) ||
                           (d.fan != lastPrintedFan);

          if (hasChanged) {
            lastPrintedPower = d.power;
            lastPrintedTemp = d.current_temp;
            lastPrintedMode = d.mode;
            lastPrintedFan = d.fan;
            firstFrameReceived = true;
            
            // Wake automation task immediately for fast external change detection
            if (g_taskAutomation) {
              xTaskNotifyGive(g_taskAutomation);
            }
          }
        }
        i += FRAME_LEN;
      } else {
        // ═══════════════════════════════════════════════════════════════
        // UART ERROR: Header found but frame invalid
        // Hanya increment 1x per periode sampai dapat valid frame lagi
        // Tidak count error sebelum pernah dapat frame valid (boot awal)
        // ═══════════════════════════════════════════════════════════════
        if (everHadValidFrame && !uartErrorFlagActive) {
          uartErrorCount++;
          uartErrorFlagActive = true;  // Set flag, prevent spam increment
          Serial.Printf("[MAINBOARD] Error: Header found but frame invalid (UART Error Count: %lu)\n", uartErrorCount);
        }
        i++;
      }
    }

    if (!everHadValidFrame && !validFrameFoundThisPass && rawLen > 0 && !mainboardStartupDebugLogged) {
      for (size_t j = 0; j + 1 < cleanLen; ++j) {
        if (cleanBuf[j] == HDR_BP[0] && cleanBuf[j + 1] == HDR_BP[1]) {
          headerOffset = (int)j;
          break;
        }
      }

      Serial.Println("[MAINBOARD] STARTUP RAW DIAGNOSTIC");
      Serial.Printf("RAWLEN: %u CLEANLEN: %u HEADER_OFFSET: %d\n",
                (unsigned)rawLen, (unsigned)cleanLen, headerOffset);
      Serial.Printf("RAW   : %s\n", hexdump_line(rxBuf, rawLen, 48).c_str());
      Serial.Printf("CLEAN : %s\n", hexdump_line(cleanBuf, cleanLen, 48).c_str());

      if (headerOffset >= 0 && cleanLen < FRAME_LEN) {
        Serial.Println("INFO  : Header 1E 14 terlihat, tetapi panjang frame kurang dari 24 byte");
      } else if (headerOffset < 0) {
        Serial.Println("INFO  : Tidak ada header 1E 14 di data mainboard yang diterima");
      }

      mainboardStartupDebugLogged = true;
    }
  }

  // INFERRED COMM ERROR (E6)
  if (everHadValidFrame) {
    unsigned long age = millis() - lastValidFrameMs;
    if (age > FRAME_TIMEOUT_MS) {
      if (!inferredCommError) {
        inferredCommError = true;
        inferredSinceMs = millis();
        
        // ═══════════════════════════════════════════════════════════════
        // Increment error hanya jika belum increment di periode ini
        // Prevent double counting (sudah count dari parse error)
        // ═══════════════════════════════════════════════════════════════
        if (!uartErrorFlagActive) {
          uartErrorCount++;
          uartErrorFlagActive = true;
          Serial.Printf("[MAINBOARD] BP_INFERRED_COMM_TIMEOUT (UART Error Count: %lu)\n", uartErrorCount);
        } else {
          Serial.Println("[MAINBOARD] BP_INFERRED_COMM_TIMEOUT (already counted in this period)");
        }
        Serial.Println("INFO  : no valid BPFrame received (err=E6(Comm Error))");
      }
    }
  }
  
  // ============================================================================
  // DISPLAY RX POLLING (Serial1 - IO3)
  // ============================================================================
  
  // Read display serial
  while (Serial1.available()) {
    if (dispRxLen >= sizeof(dispRxBuf) - 50) break;
    dispRxBuf[dispRxLen++] = (uint8_t)Serial1.read();
    displayByteCount++;
    dispLastByteMs = millis();
  }

  bool dispShouldProcess = (dispRxLen > 0 && (millis() - dispLastByteMs) > FRAME_GAP_MS);
  bool dispBufferNearlyFull = (dispRxLen > sizeof(dispRxBuf) / 2);
  
  if (dispShouldProcess || dispBufferNearlyFull) {
    // Display frames don't use DLE encoding, process directly
    size_t i = 0;
    while (i + FRAME_LEN <= dispRxLen) {
      if (dispRxBuf[i] == HDR_DISPLAY[0] && dispRxBuf[i+1] == HDR_DISPLAY[1]) {
        
        DisplayFrame d = parse_display_frame(&dispRxBuf[i], FRAME_LEN);
        if (d.valid) {
          lastDisplayFrame = d;
          haveDisplayFrame = true;

          // LOG JIKA BERUBAH
          if (!lastLoggedDisplayFrameValid || isDisplayFrameChanged(d, lastLoggedDisplayFrame)) {

            String hex = hexdump_line(&dispRxBuf[i], FRAME_LEN);
            String info = describeDisplayInfo(d);

            Serial.Println("[DISPLAY] FRAME");
            Serial.Printf("HEX   : %s\n", hex.c_str());
            Serial.Printf("INFO  : %s\n", info.c_str());
            
            lastLoggedDisplayFrame = d;
            lastLoggedDisplayFrameValid = true;
          }

          dispLastValidFrameMs = millis();
        }
        i += FRAME_LEN;
      } else {
        i++;
      }
    }
    // Shift remaining bytes to prevent dropping partial frames
    if (i < dispRxLen) {
      size_t remaining = dispRxLen - i;
      memmove(dispRxBuf, &dispRxBuf[i], remaining);
      dispRxLen = remaining;
    } else {
      dispRxLen = 0;
    }
    
    // If we're processing due to a gap timeout, any remaining partial frame is dead
    if (dispShouldProcess && dispRxLen < FRAME_LEN) {
      dispRxLen = 0;
    }
  }
}

BPFrame sensorGetLastFrame() { return lastFrame; }
bool sensorHasValidFrame() { return haveFrame && lastFrame.valid; }
unsigned long sensorTimeSinceLastFrame() { if (!haveFrame) return 0xFFFFFFFF; return millis() - lastValidFrameMs; }
bool sensorIsHealthy() { if (!haveFrame) return false; return (millis() - lastValidFrameMs) < FRAME_TIMEOUT_MS; }
bool sensorHasInferredCommError() { return inferredCommError; }
unsigned long sensorInferredSinceMs() { if (!inferredCommError) return 0; return millis() - inferredSinceMs; }
unsigned long sensorGetLastValidFrameMs() { return lastValidFrameMs; }  // Return timestamp of last valid frame
unsigned long sensorGetMainboardByteCount() { return mainboardByteCount; }

// Display frame getters
DisplayFrame sensorGetLastDisplayFrame() { return lastDisplayFrame; }
bool sensorHasValidDisplayFrame() { return haveDisplayFrame && lastDisplayFrame.valid; }
unsigned long sensorTimeSinceLastDisplayFrame() {
  if (!haveDisplayFrame) return 0xFFFFFFFF;
  return millis() - dispLastValidFrameMs;
}

bool sensorIsDisplayHealthy() {
  if (!haveDisplayFrame) return false;
  return (millis() - dispLastValidFrameMs) < FRAME_TIMEOUT_MS;
}

unsigned long sensorGetLastValidDisplayFrameMs() { return dispLastValidFrameMs; }
unsigned long sensorGetDisplayByteCount() { return displayByteCount; }

uint8_t sensorGetSetTemp() { 
  if (haveDisplayFrame && lastDisplayFrame.valid) {
    return lastDisplayFrame.set_temp;
  }
  return 0; 
}