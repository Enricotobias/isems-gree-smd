#include "controll.h"
#include "config.h"
#include "logger.h"
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

const uint16_t IR_LED_PIN = 10;  // IO10 — ESP32-C3
const bool INVERTED = false;
IRsend irsend(IR_LED_PIN, INVERTED, true);
IRKelvinatorAC ac(IR_LED_PIN);

uint8_t ST_ON[16]  = {0x29,0x08,0x20,0x50,0x00,0xC0,0x00,0x70,0x29,0x08,0x20,0x70,0x00,0x00,0x30,0xE0};
uint8_t ST_OFF[16] = {0x21,0x08,0x20,0x50,0x00,0xC0,0x00,0xF0,0x21,0x08,0x20,0x70,0x00,0x00,0x30,0x60};
uint8_t cur[16] = {0x21,0x08,0x20,0x50,0x00,0xC0,0x00,0xF0,0x21,0x08,0x20,0x70,0x00,0x00,0x30,0x60};

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
      logPrintf("[Controll] IR RMT TX init failed: rmt_new_tx_channel err=%s (%d)\n", esp_err_to_name(err), (int)err);
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
      logPrintf("[Controll] IR RMT TX init failed: rmt_new_copy_encoder err=%s (%d)\n", esp_err_to_name(err), (int)err);
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
      logPrintf("[Controll] IR RMT TX init failed: rmt_apply_carrier err=%s (%d)\n", esp_err_to_name(err), (int)err);
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
      logPrintf("[Controll] IR RMT TX init failed: rmt_enable err=%s (%d)\n", esp_err_to_name(err), (int)err);
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
    logPrintf("[IR-RMT] Building frame repeat %d/%d (count before: %u)\n", (int)r, (int)repeat, (unsigned)count);
    
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
    
    logPrintf("[IR-RMT] Built frame repeat %d/%d (count after: %u)\n", (int)r, (int)repeat, (unsigned)count);
  }

  logPrintf("[IR-RMT] Total symbols built: %u (max: %u)\n", (unsigned)count, (unsigned)kIrRmtMaxFrameSymbols);
  
  if (count == 0 || count >= kIrRmtMaxFrameSymbols) {
    logPrintf("[IR-RMT] ERROR: count overflow or empty (%u)\n", (unsigned)count);
    return false;
  }

  rmt_transmit_config_t tx_config = {};
  tx_config.loop_count = 0;
  tx_config.flags.eot_level = 0;
  tx_config.flags.queue_nonblocking = 0;
  
  logPrintf("[IR-RMT] Transmitting %u symbols (%u bytes)...\n", (unsigned)count, (unsigned)(count * sizeof(rmt_symbol_word_t)));
  esp_err_t err = rmt_transmit(s_ir_rmt_tx_chan, s_ir_rmt_copy_encoder, symbols, count * sizeof(rmt_symbol_word_t), &tx_config);
  if (err != ESP_OK) {
    logPrintf("[IR-RMT] rmt_transmit failed: %s (%d)\n", esp_err_to_name(err), (int)err);
    return false;
  }
  
  logPrintf("[IR-RMT] Waiting for transmit to complete...\n");
  (void)rmt_tx_wait_all_done(s_ir_rmt_tx_chan, -1);
  logPrintln("[IR-RMT] Transmit complete!");

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
  logPrintln("[IR] Sending command...");
  
  // Reset WDT sebelum IR send
  esp_task_wdt_reset();
  yield();

  irSendCount++;
  
  bool sentViaRmt = false;
#if defined(ARDUINO_ARCH_ESP32)
  if (irRmtInitIfPossible()) {
    logPrintf("[IR] RMT available, attempting to send with REPEAT_COUNT=%d...\n", (int)REPEAT_COUNT);
    irSendCriticalBegin();
    sentViaRmt = irRmtSendKelvinatorState(st, REPEAT_COUNT);
    irSendCriticalEnd();
    if (!sentViaRmt) {
      logPrintln("[IR] RMT send failed, falling back to library send");
    } else {
      logPrintln("[IR] RMT send succeeded!");
    }
  } else {
    logPrintln("[IR] RMT not available, will use library send");
  }
#endif

  if (!sentViaRmt) {
    logPrintf("[IR] Using library send (1 main + %d repeats = %d total sends)\n", (int)REPEAT_COUNT, (int)(1 + REPEAT_COUNT));
    
    irSendCriticalBegin();
    irsend.sendKelvinator(st, 16);
    irSendCriticalEnd();
    logPrintln("[IR] Library send #1 (main) done");

    for (uint8_t i = 0; i < REPEAT_COUNT; i++) {
      logPrintf("[IR] Starting repeat %d/%d\n", (int)(i + 1), (int)REPEAT_COUNT);
      
      if (GAP_US >= 1000) delay(GAP_US / 1000);
      const uint32_t gapRem = GAP_US % 1000;
      if (gapRem) delayMicroseconds(gapRem);

      esp_task_wdt_reset();
      irSendCriticalBegin();
      irsend.sendKelvinator(st, 16);
      irSendCriticalEnd();
      
      logPrintf("[IR] Library send #%d (repeat) done\n", (int)(i + 2));
    }
  }
  
  // Reset WDT setelah selesai
  esp_task_wdt_reset();
  yield();
  
  logPrintf("[IR] Command sent successfully (%s path, REPEAT_COUNT=%d)\n", sentViaRmt ? "RMT" : "LIB", (int)REPEAT_COUNT);
  logPrintf("[IR] TX path: %s\n", sentViaRmt ? "RMT" : "LIB");
}

static void ensurePowerOn() {
  ac.setRaw(cur);
  ac.setPower(true);
  memcpy(cur, ac.getRaw(), 16);
}

void setFullState(uint8_t temp, uint8_t mode, uint8_t fan) {
  logPrintf(
    "[IR] Set FULL state: %d°C, mode=%d, fan=%d\n",
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
  logPrintf("[IR] Set fan speed: %s\n", fanSpeed);
  
  // Ensure power is ON
  ensurePowerOn();
  
  // Load current state ke library (preserve suhu, mode, dll)
  ac.setRaw(cur);
  ac.setPower(true);

  // ═══════════════════════════════════════════════════════════════
  // LOW/MIN: Gunakan Custom Byte Patch
  // Alasan: Setelah MEDIUM custom patch, library kadang tidak mengubah
  // byte0 sehingga transisi MEDIUM → LOW bisa "nyangkut" di MEDIUM.
  // Jadi LOW harus memaksa byte0 kembali ke kode LOW.
  // ═══════════════════════════════════════════════════════════════
  if (strcasecmp(fanSpeed, "LOW") == 0 || strcasecmp(fanSpeed, "MIN") == 0) {
    logPrintln("[IR] Using custom byte patch for LOW");

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

    logPrintf("[IR] LOW fan sent (custom byte0=0x%02X)\n", cur[0]);
    return;
  }
  
  // ═══════════════════════════════════════════════════════════════
  // MEDIUM: Gunakan Custom Byte Replacement
  // Hanya ubah byte 0 (fan identifier) tanpa mengubah byte lain
  // ═══════════════════════════════════════════════════════════════
  if (strcasecmp(fanSpeed, "MED") == 0 || strcasecmp(fanSpeed, "MEDIUM") == 0) {
    logPrintln("[IR] Using custom byte replacement for MEDIUM");
    
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
    
    logPrintf("[IR] MEDIUM fan sent (custom byte0=0x%02X)\n", cur[0]);
    return;
  }

  // ═══════════════════════════════════════════════════════════════
  // TURBO: Gunakan Custom Byte Patch
  // Force turbo nibble di byte 2 (0x30xx) agar AC benar-benar TURBO.
  // Tetap preserve suhu/mode karena basisnya dari cur.
  // ═══════════════════════════════════════════════════════════════
  if (strcasecmp(fanSpeed, "TURBO") == 0) {
    logPrintln("[IR] Using custom byte patch for TURBO");

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

    logPrintf("[IR] TURBO fan sent (custom byte2=0x%02X)\n", cur[2]);
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
    logPrintf("[IR] Unknown fan speed: %s\n", fanSpeed);
    return;
  }
  
  // Update cur state dan kirim
  memcpy(cur, ac.getRaw(), 16);
  send_state(cur);
  
  logPrintf("[IR] Fan speed %s sent successfully\n", fanSpeed);
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

  if (irRmtInitIfPossible()) {
    logPrintf("[Controll] IR RMT TX: enabled (res %luHz, mem %u symbols)\n",
              (unsigned long)kIrRmtResolutionHz,
              (unsigned)kIrRmtChannelMemSymbols);
  } else {
    logPrintf("[Controll] IR RMT TX: not available, using software timing\n");
  }
#endif

  logPrintln("[Controll] IR initialized on IO10");
}

void controllHandleCommand(const char* payload, size_t length) {
  if (!payload || length == 0 || length > 1024) {
    logPrintln("[Controll] Invalid payload");
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("[Controll] JSON error: ");
    logPrintln(error.c_str());
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
  
  // Set expected state before sending IR
  irHealthSetExpectedState(hasPower, expectedPower, hasMode, expectedMode, hasFan, expectedFan, hasTemp, expectedTemp);

  // ═══════════════════════════════════════════════════════════════════
  // [FIX] Cek apakah command berisi setting lain selain power
  // Jika ada setting lain, SKIP power command terpisah untuk menghindari
  // double IR transmission yang membuat temp/setting tidak ter-apply
  // ═══════════════════════════════════════════════════════════════════
  bool hasOtherSettings = doc.containsKey("mode") || doc.containsKey("temp") || doc.containsKey("fan");
  bool isPowerOnlyCommand = doc.containsKey("power") && !hasOtherSettings;

  // Log jika power command ada tapi di-skip karena ada settings lain
  if (doc.containsKey("power") && !isPowerOnlyCommand) {
    bool powerOn = doc["power"].as<bool>();
    logPrintf("[Controll] Power=%s (included in full state, skip separate command)\n", 
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
      logPrintln("[Controll] Unsupported power value type");
      return;
    }

    logPrintf(
      "[Controll] ✓ Executing: AC %s\n",
      powerOn ? "ON" : "OFF"
    );

    send_state(powerOn ? ST_ON : ST_OFF);
    memcpy(cur, powerOn ? ST_ON : ST_OFF, 16);

    powerHandled = true;
  }

  // =================================================
  // MODE COMMAND
  // =================================================
  if (doc.containsKey("mode")) {
    String modeStr = String(doc["mode"].as<const char*>());
    modeStr.toUpperCase();

    logPrintf("[Controll] Mode command: %s\n", modeStr.c_str());

    ac.setPower(true);   // 🔑 WAJIB: pastikan ON

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
      logPrintf("[Controll] Invalid mode: %s\n", modeStr.c_str());
    }
  }

  // =================================================
  // TEMPERATURE COMMAND
  // =================================================
  if (doc.containsKey("temp")) {
    int t = doc["temp"].as<int>();
    logPrintf("[Controll] Set temp command: %d°C\n", t);

    if (t >= 16 && t <= 30) {
      ac.setPower(true);          // 🔑 WAJIB
      ac.setTemp((uint8_t)t);
      needSend = true;
    } else {
      logPrintf("[Controll] Temp out of range: %d (valid: 16-30)\n", t);
    }
  }

  // =================================================
  // FAN COMMAND - MENGGUNAKAN LIBRARY/CUSTOM HYBRID
  // =================================================
  String fanStr = "";
  bool hasFanCommand = false;
  
  if (doc.containsKey("fan")) {
    fanStr = String(doc["fan"].as<const char*>());
    fanStr.toUpperCase();
    fanStr.trim();
    hasFanCommand = true;

    logPrintf("[Controll] Fan command: %s\n", fanStr.c_str());
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
        logPrintf("[Controll] MEDIUM fan applied (custom byte0=0x%02X)\n", cur[0]);
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
        logPrintf("[Controll] LOW fan applied (custom byte0=0x%02X)\n", cur[0]);
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
        logPrintf("[Controll] TURBO fan applied (custom byte0=0x39 byte2=0x%02X)\n", cur[2]);
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
        logPrintf("[Controll] Fan %s applied via library\n", fanStr.c_str());
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
    logPrintln("[Controll] No valid command found in payload");
  }
}