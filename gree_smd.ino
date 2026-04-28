#include <Arduino.h>
#include "LittleFS.h"
#include <WiFi.h>
#include "config.h"
#include "sensor.h"
#include "crash_monitor.h"
#include "data.h"
#include "controll.h"
#include "automation.h"
#include "ir_health.h"
#include "esp_task_wdt.h"

#include "ota.h"
#include "activity.h"
#include "rtos_common.h"
#include "tasks.h"

#if CONFIG_FREERTOS_UNICORE
  #define APP_CPU 0
  #define PRO_CPU 0
  #define xTaskCreate_RTOS(fn, name, stack, param, prio, handle) \
    xTaskCreate(fn, name, stack, param, prio, handle)
  #define xTaskCreateNetwork(fn, name, stack, param, prio, handle) \
    xTaskCreate(fn, name, stack, param, prio, handle)
#else
  #define APP_CPU 1
  #define PRO_CPU 0
  #define xTaskCreate_RTOS(fn, name, stack, param, prio, handle) \
    xTaskCreatePinnedToCore(fn, name, stack, param, prio, handle, APP_CPU)
  #define xTaskCreateNetwork(fn, name, stack, param, prio, handle) \
    xTaskCreatePinnedToCore(fn, name, stack, param, prio, handle, PRO_CPU)
#endif

#define WDT_TIMEOUT 120  // 120 seconds (match gree, tolerate long SSL ops)

// LED Status Pins (ESP32-C3 GPIO)
#define LED_RED_PIN 0     // IO0 - WiFi disconnected - blink red
#define LED_BLUE_PIN 2    // IO2 - MQTT disconnected - blink blue  
#define LED_GREEN_PIN 1   // IO1 - All connected - solid green
#define GREEN_BLINK_INTERVAL_MS 1000

// Note: AUTOMATION_INTERVAL_MS, PUBLISH_INTERVAL_MS, RETRY_INTERVAL_MS
// are now defined locally in their respective RTOS tasks.
// These remain because tryConnectWiFi/tryConnectMQTT still use them:
#define MAX_RETRY_ATTEMPTS 6
#define WIFI_RETRY_DELAY_MS 5000

const char* FIRMWARE_TAG __attribute__((used)) = "ISEMS_VER:" FW_VERSION;
const char* AC_BRAND_TAG __attribute__((used)) = "ISEMS_AC_BRAND:" AC_BRAND;

unsigned long lastConnectionTime = 0;
const unsigned long REBOOT_TIMEOUT = 15 * 60 * 1000UL; // 15 Menit

// Note: lastAutomationMs, lastPublishMs, lastWifiRetryMs, lastMqttRetryMs, lastReadUnitState
// are now local variables in TaskNetwork_MQTT.

// Stub: WiFi reconnect is now handled autonomously by TaskNetwork_MQTT.
// data.cpp still calls this; keeping as no-op to avoid compilation errors.
void dataScheduleImmediateWiFiRetry() { /* no-op */ }

volatile bool wifiOK = false;
volatile bool mqttOK = false;
bool rtcOK = true;
static bool taskWdtReady = false;

static void ensureTaskWatchdogReady() {
  if (taskWdtReady) {
    return;
  }

  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };

  esp_err_t status = esp_task_wdt_status(NULL);
  if (status == ESP_ERR_INVALID_STATE) {
    esp_err_t initResult = esp_task_wdt_init(&wdtConfig);
    if (initResult != ESP_OK && initResult != ESP_ERR_INVALID_STATE) {
      Serial.Printf("[WDT] Init failed: %d\n", (int)initResult);
    }
  } else {
    esp_err_t reconfigResult = esp_task_wdt_reconfigure(&wdtConfig);
    if (reconfigResult != ESP_OK && reconfigResult != ESP_ERR_INVALID_STATE) {
      Serial.Printf("[WDT] Reconfigure failed: %d\n", (int)reconfigResult);
    }
  }

  status = esp_task_wdt_status(NULL);
  if (status == ESP_ERR_NOT_FOUND) {
    esp_err_t addResult = esp_task_wdt_add(NULL);
    if (addResult != ESP_OK) {
      Serial.Printf("[WDT] Add current task failed: %d\n", (int)addResult);
    }
  }

  taskWdtReady = (esp_task_wdt_status(NULL) == ESP_OK);
  if (taskWdtReady) {
    esp_task_wdt_reset();
  }
}

String getWiFiStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS: return "Menunggu koneksi";
    case WL_NO_SSID_AVAIL: return "SSID tidak ditemukan";
    case WL_SCAN_COMPLETED: return "Scan selesai";
    case WL_CONNECTED: return "Terhubung";
    case WL_CONNECT_FAILED: return "Koneksi gagal";
    case WL_CONNECTION_LOST: return "Koneksi terputus";
    case WL_DISCONNECTED: return "Terputus";
    default: return "Status tidak dikenal";
  }
}

static void feedWatchdogAndYield() {
  ensureTaskWatchdogReady();
  if (taskWdtReady) {
    esp_task_wdt_reset();
  }
  yield();
}

bool tryConnectWiFi() {
  for (int i = 1; i <= MAX_RETRY_ATTEMPTS; i++) {
    Serial.Printf("[WiFi] Percobaan %d/%d\n", i, MAX_RETRY_ATTEMPTS);
    feedWatchdogAndYield();
    
    cfgStartWiFiConnect();
    uint32_t t0 = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 10000) {
      delay(200);
      feedWatchdogAndYield();
      
      wl_status_t status = WiFi.status();
      if (status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED) {
        Serial.Printf("Gagal: %s\n", getWiFiStatusText(status).c_str());
        break;
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setSleep(false);
      Serial.Printf("Terhubung ke: %s\n", cfg.koneksi.wifi_ssid.c_str());
      Serial.Printf("IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    
    if (i < MAX_RETRY_ATTEMPTS) {
      Serial.Println("Retry dalam 2 detik...");
      WiFi.disconnect(true);
      delay(2000);
      feedWatchdogAndYield();
    }
  }
  
  Serial.Println("WiFi gagal terhubung");
  return false;
}

bool tryConnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  for (int i = 1; i <= MAX_RETRY_ATTEMPTS; i++) {
    Serial.Printf("[MQTT] Percobaan %d/%d\n", i, MAX_RETRY_ATTEMPTS);
    feedWatchdogAndYield();
    
    // Force immediate connection attempt
    dataForceConnect();
    
    // Check connection after short delay
    delay(1000);
    feedWatchdogAndYield();
    
    if (dataIsConnected()) {
      Serial.Println("[MQTT] Terhubung!");
      return true;
    }
    
    if (i < MAX_RETRY_ATTEMPTS) {
      Serial.Println("[MQTT] Retry dalam 2 detik...");
      delay(2000);
      feedWatchdogAndYield();
    }
  }
  
  return false;
}

// Contoh: RED/BULE/GREEN dim values
const int BRIGHT_DIM = 50;   // redup (coba 20..80)
const int BRIGHT_MED = 120;  // medium (opsional)
const int BRIGHT_FULL = 255; // penuh

// Setup PWM (panggil sekali di init) — ESP32-C3 LEDC API
void initLED() {
  const int freq = 5000;
  const int resolution = 8; // 8-bit (0..255)
  ledcAttach(LED_RED_PIN, freq, resolution);
  ledcAttach(LED_BLUE_PIN, freq, resolution);
  ledcAttach(LED_GREEN_PIN, freq, resolution);

  // set default (semua off)
  ledcWrite(LED_RED_PIN, 0);
  ledcWrite(LED_BLUE_PIN, 0);
  ledcWrite(LED_GREEN_PIN, 0);
}

void checkMemoryHealth() {
  uint32_t freeHeap = ESP.getFreeHeap();
  
  // Warning threshold
  if (freeHeap < 20000) {
    // Publish to MQTT if connected
    if (dataIsConnected()) {
      String alert = "Low memory: " + String(freeHeap) + " bytes free";
      dataPublishAck("memory", "warning", alert);
    }
  }
  
  // Critical threshold - emergency reboot
  if (freeHeap < 10000) {
    if (dataIsConnected()) {
      publishStatus("emergency_reboot_oom");
      dataPublishAck("system", "critical", "Out of memory - rebooting");
      delay(1000);
    }
    
    ESP.restart();
  }
}

// update status simple: pakai nilai redup (BRIGHT_DIM) bukan HIGH
void updateLEDStatus() {
  static unsigned long lastBlinkTime = 0;
  static unsigned long lastFaultBlinkTime = 0;
  static bool blinkState = false;
  static bool faultBlinkState = false;

  unsigned long now = millis();

  // Base blink (RED & BLUE)
  if (now - lastBlinkTime >= 500) {
    lastBlinkTime = now;
    blinkState = !blinkState;
  }

  // Fault blink (faster) for critical warnings (RTC / frame invalid)
  if (now - lastFaultBlinkTime >= 200) {
    lastFaultBlinkTime = now;
    faultBlinkState = !faultBlinkState;
  }

  // Priority order:
  // 1) RTC missing -> PURPLE blink
  // 2) WiFi down -> RED blink
  // 3) MQTT down -> BLUE blink  (moved UP: more important than frame status)
  // 4) Frame invalid/timeout (display or mainboard) -> YELLOW blink
  // 5) All OK -> GREEN solid

  if (!rtcOK) {
    // 🟣 RTC not found
    ledcWrite(LED_RED_PIN, faultBlinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_BLUE_PIN, faultBlinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_GREEN_PIN, 0);
    return;
  }

  if (!wifiOK) {
    // 🔴 WiFi down
    ledcWrite(LED_RED_PIN, blinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_BLUE_PIN, 0);
    ledcWrite(LED_GREEN_PIN, 0);
    return;
  }

  if (!mqttOK) {
    // 🔵 WiFi connected but MQTT not connected
    ledcWrite(LED_RED_PIN, 0);
    ledcWrite(LED_BLUE_PIN, blinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_GREEN_PIN, 0);
    return;
  }

  // Frame health (hanya relevan saat WiFi + MQTT sudah OK)
  bool mainboardHealthy = sensorIsHealthy();
  bool displayHealthy = sensorIsDisplayHealthy();
  bool framesOK = mainboardHealthy && displayHealthy;

  if (!framesOK) {
    // 🟡 Frame invalid/timeout (Display/Mainboard)
    ledcWrite(LED_RED_PIN, faultBlinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_BLUE_PIN, 0);
    ledcWrite(LED_GREEN_PIN, faultBlinkState ? BRIGHT_DIM : 0);
    return;
  }

  // 🟢 All OK: WiFi + MQTT + Frames -> GREEN solid
  ledcWrite(LED_RED_PIN, 0);
  ledcWrite(LED_BLUE_PIN, 0);
  ledcWrite(LED_GREEN_PIN, BRIGHT_DIM);
}

void checkNetworkHealth() {
  // Update mqttOK to reflect actual MQTT connection state
  mqttOK = dataIsConnected();
  
  // GANTI: mqtt.connected() -> dataIsConnected()
  // Karena 'mqtt' tersembunyi di data.cpp, kita pakai fungsi publiknya
  if (WiFi.status() == WL_CONNECTED && dataIsConnected()) {

    lastConnectionTime = millis();
  } 
  // Jika salah satu putus, timer tidak di-update (membiarkan waktu berjalan)
  // Cek apakah sudah terlalu lama putus?
  if (millis() - lastConnectionTime > REBOOT_TIMEOUT) {
    Serial.println("xx CRITICAL: Network Down > 15 mins. Force Restarting! xx");
    delay(1000);
    feedWatchdogAndYield();
    ESP.restart(); // <--- Paksa Reboot
  }
}

void setup() {
  Serial.begin(115200);
  ensureTaskWatchdogReady();
  delay(200);
  Serial.Println("\n=== ESP32-C3 AC Gree Control Setup ===\n");

  Serial.println("\n\n==============================");
  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_TAG); 
  Serial.println("==============================\n");

  crashMonitorInit();
  // Initialize LED status indicators
  initLED();

  // Initialize WiFi with proper settings
  WiFi.persistent(false); // Don't save to flash
  WiFi.setAutoReconnect(true); // Enable auto reconnection
  WiFi.mode(WIFI_STA);
  cfg.device_id = WiFi.macAddress();
  Serial.Printf("[Setup] Device ID (MAC): %s\n", cfg.device_id.c_str());
  WiFi.mode(WIFI_OFF);
  delay(100);
  feedWatchdogAndYield();

  // ═══════════════════════════════════════════════════════════════
  // WiFi Event Handler untuk tracking disconnect yang akurat
  // Event ini dipicu SEBELUM auto-reconnect, jadi counter tetap akurat
  // ═══════════════════════════════════════════════════════════════
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (wifiOK) {
      wifiDisconnectCount++;
      // Set offline reason di sini (event handler) karena paling reliable.
      // Di loop() else-branch bisa unreachable jika event sudah set wifiOK=false duluan.
      if (!isManualDisconnect) {
        lastDisconnectReason = REASON_WIFI_LOST;
        lastDisconnectTime = millis();
      }
      Serial.Printf("[WiFi Event] Disconnect detected! Count: %lu, Reason: %d\n", 
                wifiDisconnectCount, info.wifi_sta_disconnected.reason);
    }
    wifiOK = false;
    // Nyalakan LED merah seketika tanpa menunggu loop()
    ledcWrite(LED_RED_PIN, 50);  // RED on
    ledcWrite(LED_BLUE_PIN, 0);  // BLUE off
    ledcWrite(LED_GREEN_PIN, 0); // GREEN off
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.Printf("[WiFi Event] Reconnected, IP: %s\n", WiFi.localIP().toString().c_str());
    wifiOK = true;
    // Matikan LED merah seketika saat WiFi reconnect
    ledcWrite(LED_RED_PIN, 0);   // RED off
    ledcWrite(LED_BLUE_PIN, 0);  // BLUE off
    ledcWrite(LED_GREEN_PIN, 0); // GREEN off (updateLEDStatus() atur warna tepat di loop)
  }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  ensureTaskWatchdogReady();

  if (!LittleFS.begin()) {
    LittleFS.begin(true);
  }
  esp_task_wdt_reset();
  yield();
  
  // 1. Load config
  Serial.Println("[SETUP] 1. Load config...");
  if (!cfgLoad()) {
    Serial.Println("[SETUP] Config load gagal, menggunakan default");
  } else {
    Serial.Println("[SETUP] Config berhasil dimuat");
  }
  
  // Load MQTT logging status from config
  
  cfgApply();
  esp_task_wdt_reset();
  yield();

  // 2. Syncron waktu sistem dari waktu RTC PCF8563T
  Serial.Println("[SETUP] 2. Sinkronisasi waktu RTC...");
  esp_task_wdt_reset();
  yield();
  
  unsigned long rtcStartTime = millis();
  rtcOK = false;
  
  // Try RTC initialization with overall timeout
  if (millis() - rtcStartTime < 10000) { // 10 second total timeout for RTC
    if (cfgInitRTC()) {
      rtcOK = true;
      esp_task_wdt_reset();
      yield();
      
      if (cfgSyncFromRTC()) {
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeStr[64];
          strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
          Serial.Printf("[SETUP] RTC OK: %s\n", timeStr);
          // RTC found and time read OK
        }
      } else {
        Serial.Println("[SETUP] Gagal sinkronisasi dari RTC");
      }
    } else {
      Serial.Println("[SETUP] RTC tidak ditemukan");
    }
  } else {
    Serial.Println("[SETUP] RTC timeout");
  }
  
  esp_task_wdt_reset();
  yield();

  // 3. Koneksi wifi, gagal coba ulangi 3x, masih gagal skip
  Serial.Println("[SETUP] 3. Mencoba koneksi WiFi (3x percobaan)...");
  esp_task_wdt_reset();
  yield();
  
  unsigned long wifiStartTime = millis();
  if (tryConnectWiFi()) {
    wifiOK = true;
    
    // === REVISI: POSISI YANG BENAR ===
    // Set sleep false hanya setelah koneksi terbentuk agar tidak ter-reset
    WiFi.setSleep(false);
    Serial.Println("[SETUP] WiFi No-Sleep Mode: ACTIVATED");
    // =================================
    
    Serial.Println("[SETUP] WiFi berhasil terhubung");
    
    // Tampilkan waktu sebelum sinkronisasi NTP
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[64];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      Serial.Printf("[SETUP] Waktu sebelum NTP sync: %s\n", timeStr);
    }
    
    // Sinkronisasi waktu dari NTP jika WiFi terhubung
    esp_task_wdt_reset();
    yield();
    Serial.Println("[SETUP] Sinkronisasi waktu NTP...");
    cfgSyncTime(5000); // Increase timeout untuk memastikan sinkronisasi berhasil
    esp_task_wdt_reset();
    yield();
    
    // Tampilkan waktu setelah sinkronisasi NTP
    if (getLocalTime(&timeinfo)) {
      char timeStr[64];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      Serial.Printf("[SETUP] Waktu setelah NTP sync: %s\n", timeStr);
    }
  } else {
    Serial.Println("[SETUP] WiFi gagal, lanjut tanpa WiFi");
  }
  
  esp_task_wdt_reset();
  yield();

  // 4. Inisialisasi MQTT Config (TANPA CONNECT)
  // === REVISI: HAPUS LOGIKA CONNECT DISINI ===
  if (wifiOK) {
    Serial.Println("[SETUP] 4. Init MQTT Config (Connect otomatis di Loop)...");
    dataInit();
    // Kita biarkan loop() yang menangani koneksi agar tidak blocking
    // dan tidak membuat koneksi "zombie"
    esp_task_wdt_reset();
    yield();
  } else {
    Serial.Println("[SETUP] 4. Skip Init MQTT (WiFi tidak terhubung)");
  }

  // 5. Baca data dari unit gree (tampilkan)
  Serial.Println("[SETUP] 5. Inisialisasi komunikasi dengan unit Gree ...");
  esp_task_wdt_reset();
  yield();
  
  sensorInit();
  delay(500); // Reduced stabilization time
  yield();
  esp_task_wdt_reset();

  otaInit();
  yield();
  esp_task_wdt_reset();
  
  // Inisialisasi IR control
  Serial.Println("[SETUP] 5a. Inisialisasi IR control...");
  controllInit();
  esp_task_wdt_reset();
  yield();
  
  Serial.Println("[SETUP] 5b. Baca data dari unit Gree...");
  // Coba baca data beberapa kali untuk memastikan dapat frame yang valid
  bool frameFound = false;
  for (int i = 0; i < 12; i++) {
    esp_task_wdt_reset();
    yield();
    
    sensorPoll();
    delay(250);
    yield();
    esp_task_wdt_reset();
    
    BPFrame testFrame = sensorGetLastFrame();
    if (testFrame.valid) {
      frameFound = true;
      Serial.Printf("[SETUP] Frame valid ditemukan pada percobaan %d\n", i+1);
      break;
    }
  }
  
  esp_task_wdt_reset();
  yield();
  BPFrame frame = sensorGetLastFrame();
  if (frame.valid) {
    Serial.Println("[SETUP] Data Gree berhasil dibaca:");
    Serial.Printf("  - Suhu: %dC\n", frame.current_temp);
    Serial.Printf("  - Power: %s\n", frame.power ? "ON" : "OFF");
    Serial.Printf("  - Mode: %s\n", frame.mode.c_str());
    Serial.Printf("  - Fan Speed: %s\n", frame.fan.c_str());
    Serial.Printf("  - Status Code: %s\n", frame.code_hex.c_str());
    Serial.Printf("  - Error Code: %s\n", frame.error_val16.c_str());
    Serial.Printf("  - Errors: %s\n", frame.errors.c_str());
    Serial.Printf("  - Frame: %s\n", frame.frameHex.c_str());
  } else {
    Serial.Println("[SETUP] Gagal membaca data dari unit Gree");
    unsigned long mainboardBytes = sensorGetMainboardByteCount();
    unsigned long displayBytes = sensorGetDisplayByteCount();
    Serial.Printf("  - Mainboard bytes RX0(IO20): %lu\n", mainboardBytes);
    Serial.Printf("  - Display bytes RX1(IO3): %lu\n", displayBytes);
    
    if (sensorHasValidFrame()) {
      unsigned long timeSince = sensorTimeSinceLastFrame();
      Serial.Printf("  - Frame terakhir diterima %lu ms yang lalu\n", timeSince);
    } else {
      Serial.Println("  - Belum pernah menerima frame yang valid");
      if (mainboardBytes == 0) {
        Serial.Println("  - GPIO20 tidak menerima byte sama sekali -> cek wiring G_MB_RX, divider R10/R11, dan common GND");
      } else {
        Serial.Println("  - GPIO20 menerima byte, tapi tidak ada frame 1E 14 yang valid -> cek level sinyal/baud/noise/line source");
      }
    }
  }

  // 6. Kirim data ke server (SKIP DULU DI SETUP)
  // === REVISI: Karena belum connect MQTT, bagian ini di-skip ===
  Serial.Println("[SETUP] 6. Skip kirim data awal (Menunggu MQTT Connect di Loop)");

  // 7. Automation & Activity — initialized by TaskAutomation (no longer in setup)
  Serial.println("[SETUP] 7. Automation will be initialized by TaskAutomation");
  esp_task_wdt_reset();
  yield();

  // Set initial LED status
  updateLEDStatus();

  g_cfgMutex = xSemaphoreCreateMutex();
  g_sensorMutex = xSemaphoreCreateMutex();
  g_irCmdQueue = xQueueCreate(IR_CMD_QUEUE_LEN, sizeof(IrCmdMsg));
  configASSERT(g_cfgMutex);
  configASSERT(g_sensorMutex);
  configASSERT(g_irCmdQueue);

  // ESP32-C3 = single-core, all tasks on core 0 via xTaskCreate (not PinnedToCore)
  // Priority: Sensor(3) > Auto=Ctrl=Net(2) > Health(1)
  xTaskCreateNetwork(TaskNetwork_MQTT, "NetTask", 8192, nullptr, 2, &g_taskNetwork);   // MQTT + WiFi management
  xTaskCreate_RTOS(TaskSensor, "SnsTask", 4096, nullptr, 3, &g_taskSensor);             // Highest: UART must not miss data
  xTaskCreate_RTOS(TaskAutomation, "AutoTask", 6144, nullptr, 2, &g_taskAutomation);    // Fast resync on external change
  xTaskCreate_RTOS(TaskControl, "CtrlTask", 6144, nullptr, 2, &g_taskControl);          // IR execution (mostly idle)
  xTaskCreate_RTOS(TaskHealth, "Health", 2048, nullptr, 1, &g_taskHealth);              // Lowest: monitoring only
  Serial.println("[Setup] FreeRTOS tasks created");

  Serial.Println("\n=== Setup Selesai ===\n");
}

void loop() {
  esp_task_wdt_delete(NULL);  // Remove loopTask from WDT before deletion
  vTaskDelete(NULL);
}
