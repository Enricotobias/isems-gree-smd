#include "controll.h"
#include "config.h"

#include "crash_monitor.h"
#include "data.h"
#include "ir_health.h"
#include <ArduinoJson.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Kelvinator.h>
#include "esp_task_wdt.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp_err.h"
#include "esp_pm.h"
#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif

#include <cstring>
#include "rtos_common.h"

const uint16_t IR_LED_PIN = 10;  // IO10 — ESP32-C3
const bool INVERTED = false;
IRsend irsend(IR_LED_PIN, INVERTED, true);
IRKelvinatorAC ac(IR_LED_PIN);

uint8_t cur[16] = {0x61,0x08,0x20,0x50,0x11,0xC0,0x00,0x00,0x61,0x08,0x20,0x70,0x00,0x00,0x30,0x60};

// IR repeat untuk meningkatkan peluang diterima AC.
// Total burst = 1x send utama + REPEAT_COUNT x repeat.
static const uint8_t REPEAT_COUNT = 1;
static const uint16_t GAP_US = 20000;

#if defined(ARDUINO_ARCH_ESP32)
static bool s_ir_rmt_ready = false;
static rmt_channel_handle_t s_ir_rmt_tx_chan = nullptr;
static rmt_encoder_handle_t s_ir_rmt_copy_encoder = nullptr;
static SemaphoreHandle_t s_ir_rmt_init_mutex = nullptr;
static bool s_ir_rmt_disabled = false;
static bool s_ir_rmt_logged_failure = false;

static constexpr uint32_t kIrRmtResolutionHz = 1000000;  // 1 tick = 1us
static constexpr size_t kIrRmtChannelMemSymbols = 64;  // must be even
static constexpr size_t kIrRmtMaxFrameSymbols = 512;
static constexpr size_t kIrRmtQueueDepth = 4;

static void irRmtAppendMarkSpace(rmt_symbol_word_t* symbols, size_t& count, size_t maxSymbols, uint32_t mark_us, uint32_t space_us) {
  if (count >= maxSymbols) return;

  const uint32_t s1 = (space_us > 32767U) ? 32767U : space_us;
  symbols[count].level0 = 1;
  symbols[count].duration0 = (uint16_t)mark_us;
  symbols[count].level1 = 0;
  symbols[count].duration1 = (uint16_t)s1;
  count++;

  space_us -= s1;

  while (space_us > 0 && count < maxSymbols) {
    uint32_t chunk0 = (space_us > 32767U) ? 32767U : space_us;
    space_us -= chunk0;
    
    uint32_t chunk1 = (space_us > 32767U) ? 32767U : space_us;
    space_us -= chunk1;

    // Prevent duration0 == 0 or duration1 == 0 (EOF marker in RMT)
    if (chunk1 == 0 && chunk0 > 1) {
        chunk1 = 1;
        chunk0 -= 1;
    } else if (chunk1 == 0 && chunk0 == 1 && count > 0) {
        // Steal 1us from the previous symbol's space to satisfy non-zero requirement
        if (symbols[count-1].duration1 > 1) {
            symbols[count-1].duration1 -= 1;
            chunk0 = 1;
            chunk1 = 1;
        } else {
            // fallback, should be very rare
            chunk1 = 1; // Adds 1us error, negligible
        }
    }

    symbols[count].level0 = 0;
    symbols[count].duration0 = (uint16_t)chunk0;
    symbols[count].level1 = 0;
    symbols[count].duration1 = (uint16_t)chunk1;
    count++;
  }
}

static void irRmtAppendByteLsbFirst(rmt_symbol_word_t* symbols, size_t& count, size_t maxSymbols, uint8_t b) {
  const uint32_t kBitMark = 680;
  const uint32_t kOneSpace = 1530;
  const uint32_t kZeroSpace = 510;
  for (uint8_t bit = 0; bit < 8; bit++) {
    irRmtAppendMarkSpace(symbols, count, maxSymbols, kBitMark, (b & (1U << bit)) ? kOneSpace : kZeroSpace);
  }
}

static void irRmtAppendKelvinatorBits(rmt_symbol_word_t* symbols, size_t& count, size_t maxSymbols, uint64_t data, uint16_t nbits) {
  const uint32_t kBitMark = 680;
  const uint32_t kOneSpace = 1530;
  const uint32_t kZeroSpace = 510;
  for (uint16_t i = 0; i < nbits; i++) {
    irRmtAppendMarkSpace(symbols, count, maxSymbols, kBitMark, (data & (1ULL << i)) ? kOneSpace : kZeroSpace);
  }
}

static bool irRmtInitIfPossible() {
  if (s_ir_rmt_ready) return true;
  if (s_ir_rmt_disabled) return false;

  if (!s_ir_rmt_init_mutex) {
    s_ir_rmt_init_mutex = xSemaphoreCreateMutex();
    if (!s_ir_rmt_init_mutex) return false;
  }

  if (xSemaphoreTake(s_ir_rmt_init_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
    return false;
  }

  if (s_ir_rmt_ready) {
    xSemaphoreGive(s_ir_rmt_init_mutex);
    return true;
  }

  if (s_ir_rmt_copy_encoder) {
    (void)rmt_del_encoder(s_ir_rmt_copy_encoder);
    s_ir_rmt_copy_encoder = nullptr;
  }
  if (s_ir_rmt_tx_chan) {
    (void)rmt_disable(s_ir_rmt_tx_chan);
    (void)rmt_del_channel(s_ir_rmt_tx_chan);
    s_ir_rmt_tx_chan = nullptr;
  }

  rmt_tx_channel_config_t tx_config = {};
  tx_config.gpio_num = (gpio_num_t)IR_LED_PIN;
  tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_config.resolution_hz = kIrRmtResolutionHz;
  tx_config.mem_block_symbols = kIrRmtChannelMemSymbols;
  tx_config.trans_queue_depth = kIrRmtQueueDepth;
  tx_config.intr_priority = 0;
  tx_config.flags.invert_out = INVERTED ? 1 : 0;
  tx_config.flags.with_dma = 0;
  tx_config.flags.io_loop_back = 0;
  tx_config.flags.io_od_mode = 0;
  tx_config.flags.allow_pd = 0;
  tx_config.flags.init_level = 0;

  esp_err_t err = rmt_new_tx_channel(&tx_config, &s_ir_rmt_tx_chan);
  if (err != ESP_OK) {
    s_ir_rmt_tx_chan = nullptr;
    if (!s_ir_rmt_logged_failure) {
      s_ir_rmt_logged_failure = true;
      Serial.Printf("[Controll] IR RMT TX init failed: rmt_new_tx_channel err=%s (%d)\n", esp_err_to_name(err), (int)err);
    }
    if (err == ESP_ERR_NOT_FOUND) {
      s_ir_rmt_disabled = true;
    }
    xSemaphoreGive(s_ir_rmt_init_mutex);
    return false;
  }

  rmt_copy_encoder_config_t copy_config = {};
  err = rmt_new_copy_encoder(&copy_config, &s_ir_rmt_copy_encoder);
  if (err != ESP_OK) {
    (void)rmt_del_channel(s_ir_rmt_tx_chan);
    s_ir_rmt_tx_chan = nullptr;
    s_ir_rmt_copy_encoder = nullptr;
    if (!s_ir_rmt_logged_failure) {
      s_ir_rmt_logged_failure = true;
      Serial.Printf("[Controll] IR RMT TX init failed: rmt_new_copy_encoder err=%s (%d)\n", esp_err_to_name(err), (int)err);
    }
    xSemaphoreGive(s_ir_rmt_init_mutex);
    return false;
  }

  rmt_carrier_config_t carrier_config = {};
  carrier_config.frequency_hz = 38000;
  carrier_config.duty_cycle = 0.5f;
  carrier_config.flags.polarity_active_low = 0;
  carrier_config.flags.always_on = 0;
  err = rmt_apply_carrier(s_ir_rmt_tx_chan, &carrier_config);
  if (err != ESP_OK) {
    (void)rmt_del_encoder(s_ir_rmt_copy_encoder);
    (void)rmt_del_channel(s_ir_rmt_tx_chan);
    s_ir_rmt_tx_chan = nullptr;
    s_ir_rmt_copy_encoder = nullptr;
    if (!s_ir_rmt_logged_failure) {
      s_ir_rmt_logged_failure = true;
      Serial.Printf("[Controll] IR RMT TX init failed: rmt_apply_carrier err=%s (%d)\n", esp_err_to_name(err), (int)err);
    }
    xSemaphoreGive(s_ir_rmt_init_mutex);
    return false;
  }

  err = rmt_enable(s_ir_rmt_tx_chan);
  if (err != ESP_OK) {
    (void)rmt_del_encoder(s_ir_rmt_copy_encoder);
    (void)rmt_del_channel(s_ir_rmt_tx_chan);
    s_ir_rmt_tx_chan = nullptr;
    s_ir_rmt_copy_encoder = nullptr;
    if (!s_ir_rmt_logged_failure) {
      s_ir_rmt_logged_failure = true;
      Serial.Printf("[Controll] IR RMT TX init failed: rmt_enable err=%s (%d)\n", esp_err_to_name(err), (int)err);
    }
    xSemaphoreGive(s_ir_rmt_init_mutex);
    return false;
  }

  s_ir_rmt_ready = true;
  xSemaphoreGive(s_ir_rmt_init_mutex);
  return true;
}

static bool irRmtSendKelvinatorState(const uint8_t* state, uint16_t repeat) {
  if (!state) return false;
  if (!irRmtInitIfPossible()) return false;

  const uint32_t kHdrMark = 9010;
  const uint32_t kHdrSpace = 4505;
  const uint32_t kBitMark = 680;
  const uint32_t kGapSpace = 19975;
  
  static rmt_symbol_word_t symbols[kIrRmtMaxFrameSymbols];
  size_t count = 0;

  for (uint16_t r = 0; r <= repeat; r++) {
    Serial.Printf("[IR-RMT] Building frame repeat %d/%d (count before: %u)\n", (int)r, (int)repeat, (unsigned)count);
    
    // Command Block #1
    irRmtAppendMarkSpace(symbols, count, kIrRmtMaxFrameSymbols, kHdrMark, kHdrSpace);
    for (size_t i = 0; i < 4; i++) {
      irRmtAppendByteLsbFirst(symbols, count, kIrRmtMaxFrameSymbols, state[i]);
    }
    irRmtAppendKelvinatorBits(symbols, count, kIrRmtMaxFrameSymbols, 2, 3); // 3-bit footer b010
    irRmtAppendMarkSpace(symbols, count, kIrRmtMaxFrameSymbols, kBitMark, kGapSpace);
    
    // Data Block #1
    for (size_t i = 4; i < 8; i++) {
      irRmtAppendByteLsbFirst(symbols, count, kIrRmtMaxFrameSymbols, state[i]);
    }
    irRmtAppendMarkSpace(symbols, count, kIrRmtMaxFrameSymbols, kBitMark, kGapSpace * 2);

    // Command Block #2
    irRmtAppendMarkSpace(symbols, count, kIrRmtMaxFrameSymbols, kHdrMark, kHdrSpace);
    for (size_t i = 8; i < 12; i++) {
      irRmtAppendByteLsbFirst(symbols, count, kIrRmtMaxFrameSymbols, state[i]);
    }
    irRmtAppendKelvinatorBits(symbols, count, kIrRmtMaxFrameSymbols, 2, 3); // 3-bit footer b010
    irRmtAppendMarkSpace(symbols, count, kIrRmtMaxFrameSymbols, kBitMark, kGapSpace);

    // Data Block #2
    for (size_t i = 12; i < 16; i++) {
      irRmtAppendByteLsbFirst(symbols, count, kIrRmtMaxFrameSymbols, state[i]);
    }
    irRmtAppendMarkSpace(symbols, count, kIrRmtMaxFrameSymbols, kBitMark, kGapSpace * 2);
    
    Serial.Printf("[IR-RMT] Built frame repeat %d/%d (count after: %u)\n", (int)r, (int)repeat, (unsigned)count);
  }

  Serial.Printf("[IR-RMT] Total symbols built: %u (max: %u)\n", (unsigned)count, (unsigned)kIrRmtMaxFrameSymbols);
  
  if (count == 0 || count >= kIrRmtMaxFrameSymbols) {
    Serial.Printf("[IR-RMT] ERROR: count overflow or empty (%u)\n", (unsigned)count);
    return false;
  }

  rmt_transmit_config_t tx_config = {};
  tx_config.loop_count = 0;
  tx_config.flags.eot_level = 0;
  tx_config.flags.queue_nonblocking = 0;
  
  Serial.Printf("[IR-RMT] Transmitting %u symbols (%u bytes)...\n", (unsigned)count, (unsigned)(count * sizeof(rmt_symbol_word_t)));
  esp_err_t err = rmt_transmit(s_ir_rmt_tx_chan, s_ir_rmt_copy_encoder, symbols, count * sizeof(rmt_symbol_word_t), &tx_config);
  if (err != ESP_OK) {
    Serial.Printf("[IR-RMT] rmt_transmit failed: %s (%d)\n", esp_err_to_name(err), (int)err);
    return false;
  }
  
  Serial.Printf("[IR-RMT] Waiting for transmit to complete...\n");
  (void)rmt_tx_wait_all_done(s_ir_rmt_tx_chan, -1);
  Serial.Println("[IR-RMT] Transmit complete!");

  return true;
}
#endif

#if defined(ARDUINO_ARCH_ESP32)
static esp_pm_lock_handle_t s_ir_pm_lock_no_sleep = nullptr;
static esp_pm_lock_handle_t s_ir_pm_lock_cpu_max = nullptr;

static inline void irTimingLockAcquire() {
  if (s_ir_pm_lock_cpu_max) esp_pm_lock_acquire(s_ir_pm_lock_cpu_max);
  if (s_ir_pm_lock_no_sleep) esp_pm_lock_acquire(s_ir_pm_lock_no_sleep);
}

static inline void irTimingLockRelease() {
  if (s_ir_pm_lock_no_sleep) esp_pm_lock_release(s_ir_pm_lock_no_sleep);
  if (s_ir_pm_lock_cpu_max) esp_pm_lock_release(s_ir_pm_lock_cpu_max);
}

static inline void irSendCriticalBegin() {
  irTimingLockAcquire();
}

static inline void irSendCriticalEnd() {
  irTimingLockRelease();
}
#else
static inline void irSendCriticalBegin() {}
static inline void irSendCriticalEnd() {}
#endif

void send_state(const uint8_t st[16]) {
  Serial.Println("[IR] Sending command...");
  
  // Reset WDT sebelum IR send
  esp_task_wdt_reset();
  yield();

  irSendCount++;
  
  bool sentViaRmt = false;
#if defined(ARDUINO_ARCH_ESP32)
  if (irRmtInitIfPossible()) {
    Serial.Printf("[IR] RMT available, attempting to send with REPEAT_COUNT=%d...\n", (int)REPEAT_COUNT);
    irSendCriticalBegin();
    sentViaRmt = irRmtSendKelvinatorState(st, REPEAT_COUNT);
    irSendCriticalEnd();
    if (!sentViaRmt) {
      // RMT init berhasil tapi send gagal — ini tidak seharusnya terjadi.
      // Reboot untuk reset state RMT.
      Serial.Println("[IR] FATAL: RMT send failed after successful init — rebooting");
      delay(200);
      ESP.restart();
    }
    Serial.Println("[IR] RMT send succeeded!");
  } else {
    // RMT tidak tersedia — seharusnya sudah di-handle di controllInit().
    // Jika sampai sini berarti ada bug, reboot.
    Serial.Println("[IR] FATAL: RMT not available at send time — rebooting");
    delay(200);
    ESP.restart();
  }
#endif
  
  // Reset WDT setelah selesai
  esp_task_wdt_reset();
  yield();
  
  Serial.Printf("[IR] Command sent via RMT (REPEAT_COUNT=%d)\n", (int)REPEAT_COUNT);
}

static void ensurePowerOn() {
  ac.setRaw(cur);
  ac.setPower(true);
  memcpy(cur, ac.getRaw(), 16);
}

static void applyPowerFromLibrary(bool powerOn) {
  ac.setRaw(cur);
  ac.setPower(powerOn);
  memcpy(cur, ac.getRaw(), 16);
  send_state(cur);
}

void setFullState(uint8_t temp, uint8_t mode, uint8_t fan) {
  Serial.Printf(
    "[IR] Set FULL state: %dC, mode=%d, fan=%d\n",
    temp, mode, fan
  );

  ac.setRaw(cur);
  ac.setPower(true);
  ac.setMode(mode);
  ac.setTemp(temp);
  ac.setFan(fan);

  memcpy(cur, ac.getRaw(), 16);
  send_state(cur);
}

void update_mode(uint8_t mode) {
  ensurePowerOn();
  ac.setRaw(cur);
  ac.setMode(mode);
  memcpy(cur, ac.getRaw(), 16);
  send_state(cur);
}

// === FUNGSI: SET FAN MENGGUNAKAN HYBRID (LIBRARY + CUSTOM BYTE) ===
// MEDIUM menggunakan custom byte replacement, yang lain pakai library
void update_fan(const char* fanSpeed) {
  Serial.Printf("[IR] Set fan speed: %s\n", fanSpeed);
  
  // Ensure power is ON
  ensurePowerOn();
  
  // Load current state ke library (preserve suhu, mode, dll)
  ac.setRaw(cur);
  ac.setPower(true);

  // LOW/MIN: use custom byte patch
  // After the MEDIUM custom patch, the library may keep byte0 unchanged.
  // LOW must force byte0 back to the LOW code.
  if (strcasecmp(fanSpeed, "LOW") == 0 || strcasecmp(fanSpeed, "MIN") == 0) {
    Serial.Println("[IR] Using custom byte patch for LOW");

    ac.setQuiet(false);
    ac.setTurbo(false);
    ac.setFan(kKelvinatorFanMin);

    uint8_t patched[16];
    memcpy(patched, ac.getRaw(), 16);

    // Force LOW fan identifier: keep low-nibble (power/options), set hi-nibble to 0x1.
    patched[0] = (patched[0] & 0x0F) | 0x10;
    // Force normal (non-turbo)
    patched[2] = (patched[2] & 0x0F) | 0x20;

    ac.setRaw(patched);
    memcpy(cur, ac.getRaw(), 16);
    send_state(cur);

    Serial.Printf("[IR] LOW fan sent (custom byte0=0x%02X)\n", cur[0]);
    return;
  }
  
  // ═══════════════════════════════════════════════════════════════
  // MEDIUM: Gunakan Custom Byte Replacement
  // Hanya ubah byte 0 (fan identifier) tanpa mengubah byte lain
  // ═══════════════════════════════════════════════════════════════
  if (strcasecmp(fanSpeed, "MED") == 0 || strcasecmp(fanSpeed, "MEDIUM") == 0) {
    Serial.Println("[IR] Using custom byte replacement for MEDIUM");
    
    uint8_t patched[16];
    memcpy(patched, ac.getRaw(), 16);

    // Force MEDIUM fan identifier: keep low-nibble (power/options), set hi-nibble to 0x2.
    // Byte 2 tetap normal (non-turbo)
    patched[0] = (patched[0] & 0x0F) | 0x20;
    patched[2] = (patched[2] & 0x0F) | 0x20;

    // Re-apply ke library agar checksum/dup-block konsisten
    ac.setRaw(patched);
    memcpy(cur, ac.getRaw(), 16);
    send_state(cur);
    
    Serial.Printf("[IR] MEDIUM fan sent (custom byte0=0x%02X)\n", cur[0]);
    return;
  }

  // ═══════════════════════════════════════════════════════════════
  // TURBO: Gunakan Custom Byte Patch
  // Force turbo nibble di byte 2 (0x30xx) agar AC benar-benar TURBO.
  // Tetap preserve suhu/mode karena basisnya dari cur.
  // ═══════════════════════════════════════════════════════════════
  if (strcasecmp(fanSpeed, "TURBO") == 0) {
    Serial.Println("[IR] Using custom byte patch for TURBO");

    // Start from current state, set fan max via library first.
    ac.setQuiet(false);
    ac.setTurbo(false);
    ac.setFan(kKelvinatorFanMax);

    uint8_t patched[16];
    memcpy(patched, ac.getRaw(), 16);

    // Fan HIGH identifier (per capture IR receiver)
    patched[0] = 0x39;

    // Force TURBO flag/high-nibble.
    patched[2] = (patched[2] & 0x0F) | 0x30;

    ac.setRaw(patched);
    memcpy(cur, ac.getRaw(), 16);
    send_state(cur);

    Serial.Printf("[IR] TURBO fan sent (custom byte2=0x%02X)\n", cur[2]);
    return;
  }
  
  // ═══════════════════════════════════════════════════════════════
  // Untuk fan speed lainnya: Gunakan Library
  // ═══════════════════════════════════════════════════════════════
  if (strcasecmp(fanSpeed, "QUIET") == 0) {
    ac.setQuiet(true);
    ac.setTurbo(false);
    ac.setFan(kKelvinatorFanMin);  // Fan speed 1 (minimum)
  }
  else if (strcasecmp(fanSpeed, "HIGH") == 0 || strcasecmp(fanSpeed, "MAX") == 0) {
    ac.setQuiet(false);
    ac.setTurbo(false);
    ac.setFan(kKelvinatorFanMax);  // Fan speed 5 (maximum)
  }
  else if (strcasecmp(fanSpeed, "TURBO") == 0) {
    ac.setQuiet(false);
    ac.setTurbo(true);
    ac.setFan(kKelvinatorFanMax);  // Fan speed 5 + turbo flag
  }
  else if (strcasecmp(fanSpeed, "AUTO") == 0) {
    ac.setQuiet(false);
    ac.setTurbo(false);
    ac.setFan(kKelvinatorFanAuto);  // Fan speed 0 (auto)
  }
  else {
    Serial.Printf("[IR] Unknown fan speed: %s\n", fanSpeed);
    return;
  }
  
  // Update cur state dan kirim
  memcpy(cur, ac.getRaw(), 16);
  send_state(cur);
  
  Serial.Printf("[IR] Fan speed %s sent successfully\n", fanSpeed);
}

void set_temp(uint8_t t) {
  if (t < 16) t = 16;
  if (t > 30) t = 30;

  ensurePowerOn();
  ac.setRaw(cur);
  ac.setTemp(t);
  memcpy(cur, ac.getRaw(), 16);
  send_state(cur);
}

void controllInit() {
  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, LOW);
  irsend.begin();
  irHealthInit();  // Initialize IR health monitoring

#if defined(ARDUINO_ARCH_ESP32)
  if (!s_ir_pm_lock_no_sleep) {
    (void)esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ir_send", &s_ir_pm_lock_no_sleep);
  }
  if (!s_ir_pm_lock_cpu_max) {
    (void)esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "ir_cpu", &s_ir_pm_lock_cpu_max);
  }

  // RMT adalah satu-satunya jalur IR yang diizinkan pada ESP32-C3.
  // Software bit-bang tidak diperbolehkan karena single-core dan timing kritis.
  // Jika RMT gagal init, reboot sampai berhasil.
  static uint8_t rmtRetryCount = 0;
  const uint8_t RMT_MAX_RETRIES = 5;

  while (!irRmtInitIfPossible()) {
    rmtRetryCount++;
    Serial.Printf("[Controll] IR RMT init failed (attempt %d/%d), rebooting...\n",
              (int)rmtRetryCount, (int)RMT_MAX_RETRIES);
    delay(500);
    ESP.restart();
  }

  Serial.Printf("[Controll] IR RMT TX: enabled (res %luHz, mem %u symbols)\n",
            (unsigned long)kIrRmtResolutionHz,
            (unsigned)kIrRmtChannelMemSymbols);
#endif

  Serial.Println("[Controll] IR initialized on IO10");
}

void controllHandleCommand(const char* payload, size_t length) {
  if (!payload || length == 0 || length > 1024) {
    Serial.Println("[Controll] Invalid payload");
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("[Controll] JSON error: ");
    Serial.Println(error.c_str());
    return;
  }

  bool needSend = false;
  bool powerHandled = false;

  // Load current AC state
  ac.setRaw(cur);
  
  // ═══════════════════════════════════════════════════════════════════
  // [NEW] Extract expected state for IR health monitoring
  // ═══════════════════════════════════════════════════════════════════
  const bool hasPower = doc.containsKey("power");
  const bool hasMode = doc.containsKey("mode");
  const bool hasFan = doc.containsKey("fan");
  const bool hasTemp = doc.containsKey("temp");

  bool expectedPower = false;
  if (hasPower) {
    JsonVariant powerValue = doc["power"];
    if (powerValue.is<bool>()) {
      expectedPower = powerValue.as<bool>();
    }
    else if (powerValue.is<const char*>()) {
      String pwrStr = String(powerValue.as<const char*>());
      pwrStr.toUpperCase();
      pwrStr.trim();
      expectedPower = (pwrStr == "ON" || pwrStr == "TRUE" || pwrStr == "1");
    }
    else if (powerValue.is<int>()) {
      expectedPower = (powerValue.as<int>() != 0);
    }
  }

  String expectedMode = hasMode ? doc["mode"].as<String>() : "";
  String expectedFan = hasFan ? doc["fan"].as<String>() : "";
  uint8_t expectedTemp = hasTemp ? doc["temp"].as<uint8_t>() : 0;
  
  const bool validateStateFields = !(hasPower && !expectedPower);

  // Validate only fields explicitly present in the command. A fan/temp/mode
  // command without "power" must not inherit the default false value as OFF.
  irHealthSetExpectedState(hasPower, expectedPower,
                           validateStateFields && hasMode, expectedMode,
                           validateStateFields && hasFan, expectedFan,
                           validateStateFields && hasTemp, expectedTemp);

  // ═══════════════════════════════════════════════════════════════════
  // [FIX] Cek apakah command berisi setting lain selain power
  // Jika ada setting lain, SKIP power command terpisah untuk menghindari
  // double IR transmission yang membuat temp/setting tidak ter-apply
  // ═══════════════════════════════════════════════════════════════════
  bool hasOtherSettings = doc.containsKey("mode") || doc.containsKey("temp") || doc.containsKey("fan") || doc.containsKey("swing");
  bool isPowerOnlyCommand = doc.containsKey("power") && !hasOtherSettings;

  // Log jika power command ada tapi di-skip karena ada settings lain
  if (doc.containsKey("power") && !isPowerOnlyCommand) {
    bool powerOn = doc["power"].as<bool>();
    Serial.Printf("[Controll] Power=%s (included in full state, skip separate command)\n", 
              powerOn ? "ON" : "OFF");
  }

  // =================================================
  // POWER COMMAND (KIRIM HANYA JIKA COMMAND POWER SAJA)
  // =================================================
  if (doc.containsKey("power") && isPowerOnlyCommand) {
    bool powerOn = false;
    JsonVariant powerValue = doc["power"];

    if (powerValue.is<bool>()) {
      powerOn = powerValue.as<bool>();
    }
    else if (powerValue.is<const char*>()) {
      String pwrStr = String(powerValue.as<const char*>());
      pwrStr.toUpperCase();
      pwrStr.trim();
      powerOn = (pwrStr == "ON" || pwrStr == "TRUE" || pwrStr == "1");
    }
    else if (powerValue.is<int>()) {
      powerOn = (powerValue.as<int>() != 0);
    }
    else {
      Serial.Println("[Controll] Unsupported power value type");
      return;
    }

    Serial.Printf(
      "[Controll] Executing: AC %s\n",
      powerOn ? "ON" : "OFF"
    );

    applyPowerFromLibrary(powerOn);

    powerHandled = true;
  }

  // =================================================
  // MODE COMMAND
  // =================================================
  if (doc.containsKey("mode")) {
    JsonVariant modeValue = doc["mode"];
    if (!modeValue.is<const char*>()) {
      Serial.Println("[Controll] Invalid mode type: expected string");
      return;
    }

    String modeStr = String(modeValue.as<const char*>());
    modeStr.toUpperCase();
    modeStr.trim();

    Serial.Printf("[Controll] Mode command: %s\n", modeStr.c_str());

    ac.setPower(true);   // ensure ON

    if (modeStr == "COLD" || modeStr == "COOL") {
      ac.setMode(kKelvinatorCool);
      needSend = true;
    }
    else if (modeStr == "DRY") {
      ac.setMode(kKelvinatorDry);
      needSend = true;
    }
    else if (modeStr == "FAN") {
      ac.setMode(kKelvinatorFan);
      needSend = true;
    }
    else if (modeStr == "AUTO") {
      ac.setMode(kKelvinatorAuto);
      needSend = true;
    }
    else if (modeStr == "HEAT") {
      ac.setMode(kKelvinatorHeat);
      needSend = true;
    }
    else {
      Serial.Printf("[Controll] Invalid mode: %s\n", modeStr.c_str());
    }
  }

  // =================================================
  // TEMPERATURE COMMAND
  // =================================================
  if (doc.containsKey("temp")) {
    int t = doc["temp"].as<int>();
    Serial.Printf("[Controll] Set temp command: %dC\n", t);

    if (t >= 16 && t <= 30) {
      ac.setPower(true);          // ensure ON
      ac.setTemp((uint8_t)t);
      needSend = true;
    } else {
      Serial.Printf("[Controll] Temp out of range: %d (valid: 16-30)\n", t);
    }
  }

  // =================================================
  // FAN COMMAND - MENGGUNAKAN LIBRARY/CUSTOM HYBRID
  // =================================================
  String fanStr = "";
  bool hasFanCommand = false;
  
  if (doc.containsKey("fan")) {
    JsonVariant fanValue = doc["fan"];
    if (!fanValue.is<const char*>()) {
      Serial.Println("[Controll] Invalid fan type: expected string");
      return;
    }

    fanStr = String(fanValue.as<const char*>());
    fanStr.toUpperCase();
    fanStr.trim();
    hasFanCommand = true;

    Serial.Printf("[Controll] Fan command: %s\n", fanStr.c_str());
  }

  // =================================================
  // SWING COMMAND (Kelvinator: vertical + horizontal)
  // =================================================
  bool hasSwingCommand = false;

  if (doc.containsKey("swing")) {
    JsonVariant swingValue = doc["swing"];
    hasSwingCommand = true;

    if (doc.containsKey("power")) {
      ac.setPower(expectedPower);
    } else {
      ac.setPower(true);
    }

    if (swingValue.is<bool>()) {
      bool swingOn = swingValue.as<bool>();
      if (swingOn) {
        ac.setSwingVertical(true, kKelvinatorSwingVAuto);
        Serial.Printf("[Controll] Swing vertical: AUTO\n");
      } else {
        ac.setSwingVertical(false, kKelvinatorSwingVOff);
        ac.setSwingHorizontal(false);
        Serial.Printf("[Controll] Swing: OFF (vertical + horizontal)\n");
      }
      needSend = true;
    }
    else if (swingValue.is<const char*>()) {
      String swingStr = String(swingValue.as<const char*>());
      swingStr.toUpperCase();
      swingStr.trim();
      Serial.Printf("[Controll] Swing command: %s\n", swingStr.c_str());

      if (swingStr == "AUTO" || swingStr == "ON") {
        ac.setSwingVertical(true, kKelvinatorSwingVAuto);
        needSend = true;
      } else if (swingStr == "OFF") {
        ac.setSwingVertical(false, kKelvinatorSwingVOff);
        ac.setSwingHorizontal(false);
        needSend = true;
      } else if (swingStr == "HIGHEST") {
        ac.setSwingVertical(false, kKelvinatorSwingVHighest);
        needSend = true;
      } else if (swingStr == "UPPER_MIDDLE") {
        ac.setSwingVertical(false, kKelvinatorSwingVUpperMiddle);
        needSend = true;
      } else if (swingStr == "MIDDLE") {
        ac.setSwingVertical(false, kKelvinatorSwingVMiddle);
        needSend = true;
      } else if (swingStr == "LOWER_MIDDLE") {
        ac.setSwingVertical(false, kKelvinatorSwingVLowerMiddle);
        needSend = true;
      } else if (swingStr == "LOWEST") {
        ac.setSwingVertical(false, kKelvinatorSwingVLowest);
        needSend = true;
      } else {
        Serial.Printf("[Controll] Invalid swing value: %s\n", swingStr.c_str());
        hasSwingCommand = false;
      }
    }
    else if (swingValue.is<JsonObject>()) {
      JsonObject swingObj = swingValue.as<JsonObject>();

      if (swingObj.containsKey("vertical") && swingObj["vertical"].is<const char*>()) {
        String vStr = String(swingObj["vertical"].as<const char*>());
        vStr.toUpperCase();
        vStr.trim();

        if (vStr == "AUTO" || vStr == "ON") {
          ac.setSwingVertical(true, kKelvinatorSwingVAuto);
        } else if (vStr == "OFF") {
          ac.setSwingVertical(false, kKelvinatorSwingVOff);
        } else if (vStr == "HIGHEST") {
          ac.setSwingVertical(false, kKelvinatorSwingVHighest);
        } else if (vStr == "UPPER_MIDDLE") {
          ac.setSwingVertical(false, kKelvinatorSwingVUpperMiddle);
        } else if (vStr == "MIDDLE") {
          ac.setSwingVertical(false, kKelvinatorSwingVMiddle);
        } else if (vStr == "LOWER_MIDDLE") {
          ac.setSwingVertical(false, kKelvinatorSwingVLowerMiddle);
        } else if (vStr == "LOWEST") {
          ac.setSwingVertical(false, kKelvinatorSwingVLowest);
        } else {
          Serial.Printf("[Controll] Invalid swing vertical: %s\n", vStr.c_str());
        }
        Serial.Printf("[Controll] Swing vertical: %s\n", vStr.c_str());
        needSend = true;
      }

      if (swingObj.containsKey("horizontal")) {
        bool hOn = false;
        JsonVariant hVal = swingObj["horizontal"];
        if (hVal.is<bool>()) {
          hOn = hVal.as<bool>();
        } else if (hVal.is<const char*>()) {
          String hStr = String(hVal.as<const char*>());
          hStr.toUpperCase();
          hStr.trim();
          hOn = (hStr == "ON" || hStr == "TRUE" || hStr == "1");
        }
        ac.setSwingHorizontal(hOn);
        Serial.Printf("[Controll] Swing horizontal: %s\n", hOn ? "ON" : "OFF");
        needSend = true;
      }
    }
    else {
      Serial.Println("[Controll] Invalid swing type");
      hasSwingCommand = false;
    }
  }

  // =================================================
  // FINAL IR SEND (SINGLE, CORRECT FRAME)
  // =================================================
  if (needSend) {
    // Jika ada fan command, set fan dulu sebelum getRaw()
    if (hasFanCommand) {
      // Check apakah MEDIUM (perlu custom byte replacement)
      if (fanStr == "MED" || fanStr == "MEDIUM") {
        // Untuk MEDIUM, patch byte saja agar set temp tidak berubah
        uint8_t patched[16];
        memcpy(patched, ac.getRaw(), 16);

        // Force MEDIUM identifier: keep low-nibble (power/options), set hi-nibble to 0x2.
        patched[0] = (patched[0] & 0x0F) | 0x20;
        // Force normal (non-turbo)
        patched[2] = (patched[2] & 0x0F) | 0x20;

        // Re-apply ke library agar checksum/dup-block konsisten
        ac.setRaw(patched);
        memcpy(cur, ac.getRaw(), 16);
        Serial.Printf("[Controll] MEDIUM fan applied (custom byte0=0x%02X)\n", cur[0]);
      } else if (fanStr == "LOW" || fanStr == "MIN") {
        // LOW: Force LOW identifier supaya transisi dari MEDIUM tidak nyangkut.
        ac.setQuiet(false);
        ac.setTurbo(false);
        ac.setFan(kKelvinatorFanMin);

        uint8_t patched[16];
        memcpy(patched, ac.getRaw(), 16);

        patched[0] = (patched[0] & 0x0F) | 0x10;
        patched[2] = (patched[2] & 0x0F) | 0x20;

        ac.setRaw(patched);
        memcpy(cur, ac.getRaw(), 16);
        Serial.Printf("[Controll] LOW fan applied (custom byte0=0x%02X)\n", cur[0]);
      } else if (fanStr == "TURBO") {
        // TURBO: patch byte agar AC benar-benar TURBO (bukan HIGH)
        ac.setQuiet(false);
        ac.setTurbo(false);
        ac.setFan(kKelvinatorFanMax);

        uint8_t patched[16];
        memcpy(patched, ac.getRaw(), 16);

        // Fan HIGH identifier + TURBO flag (per capture IR receiver)
        patched[0] = 0x39;
        patched[2] = (patched[2] & 0x0F) | 0x30;

        ac.setRaw(patched);
        memcpy(cur, ac.getRaw(), 16);
        Serial.Printf("[Controll] TURBO fan applied (custom byte0=0x39 byte2=0x%02X)\n", cur[2]);
      } else {
        // Untuk fan speed lain, gunakan library
        if (fanStr == "QUIET") {
          ac.setQuiet(true);
          ac.setTurbo(false);
          ac.setFan(kKelvinatorFanMin);
        }
        else if (fanStr == "HIGH" || fanStr == "MAX") {
          ac.setQuiet(false);
          ac.setTurbo(false);
          ac.setFan(kKelvinatorFanMax);
        }
        else if (fanStr == "AUTO") {
          ac.setQuiet(false);
          ac.setTurbo(false);
          ac.setFan(kKelvinatorFanAuto);
        }
        
        memcpy(cur, ac.getRaw(), 16);
        Serial.Printf("[Controll] Fan %s applied via library\n", fanStr.c_str());
      }
    } else {
      // Tidak ada fan command, ambil state normal
      memcpy(cur, ac.getRaw(), 16);
    }
    
    send_state(cur);
    powerHandled = true;
  }
  
  // =================================================
  // FALLBACK: Jika hanya fan command tanpa mode/temp
  // =================================================
  else if (hasFanCommand && !needSend) {
    // Panggil update_fan untuk handle fan-only command
    update_fan(fanStr.c_str());
    powerHandled = true;
  }

  if (!powerHandled) {
    Serial.Println("[Controll] No valid command found in payload");
  }
}

bool controllEnqueueCommand(const char* payload, size_t len) {
  if (!payload || len == 0) return false;

  if (!g_irCmdQueue) {
    // Ensure IR commands are executed only by CtrlTask.
    // If the queue isn't ready yet, create it and enqueue; do NOT send IR inline.
    g_irCmdQueue = xQueueCreate(IR_CMD_QUEUE_LEN, sizeof(IrCmdMsg));
    if (!g_irCmdQueue) {
      Serial.println("[IR] Failed to create command queue");
      return false;
    }
  }

  IrCmdMsg msg;
  if (len >= IR_CMD_STR_MAX) len = IR_CMD_STR_MAX - 1;
  memcpy(msg.payload, payload, len);
  msg.payload[len] = '\0';
  msg.len = len;

  bool sent = (xQueueSend(g_irCmdQueue, &msg, pdMS_TO_TICKS(100)) == pdTRUE);
  if (!sent) Serial.Println("[IR] Queue full, command dropped");
  return sent;
}
