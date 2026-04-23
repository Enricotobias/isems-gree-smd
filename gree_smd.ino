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
#include "logger.h"
#include "ota.h"
#include "activity.h"

#define WDT_TIMEOUT 60  // 60 seconds

// LED Status Pins (ESP32-C3 GPIO)
#define LED_RED_PIN 0     // IO0 - WiFi disconnected - blink red
#define LED_BLUE_PIN 2    // IO2 - MQTT disconnected - blink blue  
#define LED_GREEN_PIN 1   // IO1 - All connected - solid green
#define GREEN_BLINK_INTERVAL_MS 1000

#define AUTOMATION_INTERVAL_MS 5000UL // 5 secons
#define PUBLISH_INTERVAL_MS 60000UL    // 1 minutes
#define RETRY_INTERVAL_MS 180000UL    // 3 minutes
#define MAX_RETRY_ATTEMPTS 6
#define WIFI_RETRY_DELAY_MS 5000    

static unsigned long lastMemoryCheckMs = 0;
const unsigned long MEMORY_CHECK_INTERVAL_MS = 60000UL; // 1 menit

const char* FIRMWARE_TAG __attribute__((used)) = "ISEMS_VER:" FW_VERSION;
const char* AC_BRAND_TAG __attribute__((used)) = "ISEMS_AC_BRAND:" AC_BRAND;

unsigned long lastConnectionTime = 0;
const unsigned long REBOOT_TIMEOUT = 15 * 60 * 1000UL; // 15 Menit

unsigned long lastAutomationMs = 0;
unsigned long lastPublishMs = 0;
unsigned long lastWifiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;

void dataScheduleImmediateWiFiRetry() {
  if (millis() >= RETRY_INTERVAL_MS) {
    lastWifiRetryMs = millis() - RETRY_INTERVAL_MS;
  } else {
    lastWifiRetryMs = 0;
  }
}

static bool lastReadUnitState = false;

bool wifiOK = false;
bool mqttOK = false;
bool rtcOK = true;
volatile unsigned long wifiDisconnectEpoch = 0; // Set oleh WiFi event handler, flush di dataLoop
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
      logPrintf("[WDT] Init failed: %d\n", (int)initResult);
    }
  } else {
    esp_err_t reconfigResult = esp_task_wdt_reconfigure(&wdtConfig);
    if (reconfigResult != ESP_OK && reconfigResult != ESP_ERR_INVALID_STATE) {
      logPrintf("[WDT] Reconfigure failed: %d\n", (int)reconfigResult);
    }
  }

  status = esp_task_wdt_status(NULL);
  if (status == ESP_ERR_NOT_FOUND) {
    esp_err_t addResult = esp_task_wdt_add(NULL);
    if (addResult != ESP_OK) {
      logPrintf("[WDT] Add current task failed: %d\n", (int)addResult);
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
    logPrintf("[WiFi] Percobaan %d/%d\n", i, MAX_RETRY_ATTEMPTS);
    feedWatchdogAndYield();
    
    cfgStartWiFiConnect();
    uint32_t t0 = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 10000) {
      delay(200);
      feedWatchdogAndYield();
      
      wl_status_t status = WiFi.status();
      if (status == WL_NO_SSID_AVAIL || status == WL_CONNECT_FAILED) {
        logPrintf("Gagal: %s\n", getWiFiStatusText(status).c_str());
        break;
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.setSleep(false);
      logPrintf("Terhubung ke: %s\n", cfg.koneksi.wifi_ssid.c_str());
      logPrintf("IP: %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    
    if (i < MAX_RETRY_ATTEMPTS) {
      logPrintln("Retry dalam 2 detik...");
      WiFi.disconnect(true);
      delay(2000);
      feedWatchdogAndYield();
    }
  }
  
  logPrintln("WiFi gagal terhubung");
  return false;
}

bool tryConnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  for (int i = 1; i <= MAX_RETRY_ATTEMPTS; i++) {
    logPrintf("[MQTT] Percobaan %d/%d\n", i, MAX_RETRY_ATTEMPTS);
    feedWatchdogAndYield();
    
    // Force immediate connection attempt
    dataForceConnect();
    
    // Check connection after short delay
    delay(1000);
    feedWatchdogAndYield();
    
    if (dataIsConnected()) {
      logPrintln("[MQTT] Terhubung!");
      return true;
    }
    
    if (i < MAX_RETRY_ATTEMPTS) {
      logPrintln("[MQTT] Retry dalam 2 detik...");
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
  uint32_t minHeap = ESP.getMinFreeHeap();
  
  // Warning threshold
  if (freeHeap < 20000) {
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  ⚠️ MEMORY WARNING                    ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.printf("Free Heap: %d bytes\n", freeHeap);
    Serial.printf("Min Heap: %d bytes\n", minHeap);
    
    // Log ke MQTT jika connected
    if (dataIsConnected()) {
      String alert = "⚠️ Low memory: " + String(freeHeap) + " bytes free";
      dataPublishAck("memory", "warning", alert);
    }
  }
  
  // Critical threshold - emergency reboot
  if (freeHeap < 10000) {
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  🚨 CRITICAL: OUT OF MEMORY           ║");
    Serial.println("║  Emergency reboot initiated...        ║");
    Serial.println("╚════════════════════════════════════════╝");
    
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
  // 2) WiFi down -> RED blink  (sebelum frame check: frame timeout terjadi saat tryConnectWiFi blocking)
  // 3) Frame invalid/timeout (display or mainboard) -> YELLOW blink
  // 4) MQTT down -> BLUE blink
  // 5) All OK -> GREEN solid

  if (!rtcOK) {
    // 🟣 RTC not found
    ledcWrite(LED_RED_PIN, faultBlinkState ? BRIGHT_DIM : 0); // RED
    ledcWrite(LED_BLUE_PIN, faultBlinkState ? BRIGHT_DIM : 0); // BLUE
    ledcWrite(LED_GREEN_PIN, 0);                                // GREEN
    return;
  }

  if (!wifiOK) {
    // 🔴 WiFi down
    ledcWrite(LED_RED_PIN, blinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_BLUE_PIN, 0);
    ledcWrite(LED_GREEN_PIN, 0);
    return;
  }

  // Frame health (hanya relevan saat WiFi up)
  bool mainboardHealthy = sensorIsHealthy();
  bool displayHealthy = sensorIsDisplayHealthy();
  bool framesOK = mainboardHealthy && displayHealthy;

  if (!framesOK) {
    // 🟡 Frame invalid/timeout (Display/Mainboard)
    ledcWrite(LED_RED_PIN, faultBlinkState ? BRIGHT_DIM : 0); // RED
    ledcWrite(LED_BLUE_PIN, 0);                                // BLUE
    ledcWrite(LED_GREEN_PIN, faultBlinkState ? BRIGHT_DIM : 0); // GREEN
    return;
  }

  if (!mqttOK) {
    // 🔵 MQTT down
    ledcWrite(LED_RED_PIN, 0);
    ledcWrite(LED_BLUE_PIN, blinkState ? BRIGHT_DIM : 0);
    ledcWrite(LED_GREEN_PIN, 0);

  } else if (wifiOK && mqttOK) {
    // 🟢 ALL OK → GREEN SOLID
    ledcWrite(LED_RED_PIN, 0);
    ledcWrite(LED_BLUE_PIN, 0);
    ledcWrite(LED_GREEN_PIN, BRIGHT_DIM);

  } else {
    ledcWrite(LED_RED_PIN, 0);
    ledcWrite(LED_BLUE_PIN, 0);
    ledcWrite(LED_GREEN_PIN, 0);
  }
}

void checkNetworkHealth() {
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
  logInit();
  ensureTaskWatchdogReady();
  delay(200);
  logPrintln("\n=== ESP32-C3 AC Gree Control Setup ===\n");

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
      // Catat epoch saat ini agar dataLoop() bisa flush ke LittleFS
      // dan update lastOnlineWibStr dengan waktu tepat saat disconnect.
      wifiDisconnectEpoch = (unsigned long)time(nullptr);
      logPrintf("[WiFi Event] Disconnect detected! Count: %lu, Reason: %d\n", 
                wifiDisconnectCount, info.wifi_sta_disconnected.reason);
    }
    wifiOK = false;
    // Nyalakan LED merah seketika tanpa menunggu loop()
    ledcWrite(LED_RED_PIN, 50);  // RED on
    ledcWrite(LED_BLUE_PIN, 0);  // BLUE off
    ledcWrite(LED_GREEN_PIN, 0); // GREEN off
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    logPrintf("[WiFi Event] Reconnected, IP: %s\n", WiFi.localIP().toString().c_str());
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
  logPrintln("[SETUP] 1. Load config...");
  if (!cfgLoad()) {
    logPrintln("[SETUP] Config load gagal, menggunakan default");
  } else {
    logPrintln("[SETUP] Config berhasil dimuat");
  }
  
  // Load MQTT logging status from config
  logSetMqttEnabled(cfg.setting.mqtt_log_enabled);
  logPrintf("[SETUP] MQTT logging: %s\n", cfg.setting.mqtt_log_enabled ? "enabled" : "disabled");
  
  cfgApply();
  esp_task_wdt_reset();
  yield();

  // 2. Syncron waktu sistem dari waktu RTC PCF8563T
  logPrintln("[SETUP] 2. Sinkronisasi waktu RTC...");
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
          logPrintf("[SETUP] RTC OK: %s\n", timeStr);
          // RTC found and time read OK
        }
      } else {
        logPrintln("[SETUP] Gagal sinkronisasi dari RTC");
      }
    } else {
      logPrintln("[SETUP] RTC tidak ditemukan");
    }
  } else {
    logPrintln("[SETUP] RTC timeout");
  }
  
  esp_task_wdt_reset();
  yield();

  // 3. Koneksi wifi, gagal coba ulangi 3x, masih gagal skip
  logPrintln("[SETUP] 3. Mencoba koneksi WiFi (3x percobaan)...");
  esp_task_wdt_reset();
  yield();
  
  unsigned long wifiStartTime = millis();
  if (tryConnectWiFi()) {
    wifiOK = true;
    
    // === REVISI: POSISI YANG BENAR ===
    // Set sleep false hanya setelah koneksi terbentuk agar tidak ter-reset
    WiFi.setSleep(false);
    logPrintln("[SETUP] WiFi No-Sleep Mode: ACTIVATED");
    // =================================
    
    logPrintln("[SETUP] WiFi berhasil terhubung");
    
    // Tampilkan waktu sebelum sinkronisasi NTP
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timeStr[64];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      logPrintf("[SETUP] Waktu sebelum NTP sync: %s\n", timeStr);
    }
    
    // Sinkronisasi waktu dari NTP jika WiFi terhubung
    esp_task_wdt_reset();
    yield();
    logPrintln("[SETUP] Sinkronisasi waktu NTP...");
    cfgSyncTime(5000); // Increase timeout untuk memastikan sinkronisasi berhasil
    esp_task_wdt_reset();
    yield();
    
    // Tampilkan waktu setelah sinkronisasi NTP
    if (getLocalTime(&timeinfo)) {
      char timeStr[64];
      strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
      logPrintf("[SETUP] Waktu setelah NTP sync: %s\n", timeStr);
    }
  } else {
    logPrintln("[SETUP] WiFi gagal, lanjut tanpa WiFi");
  }
  
  esp_task_wdt_reset();
  yield();

  // 4. Inisialisasi MQTT Config (TANPA CONNECT)
  // === REVISI: HAPUS LOGIKA CONNECT DISINI ===
  if (wifiOK) {
    logPrintln("[SETUP] 4. Init MQTT Config (Connect otomatis di Loop)...");
    dataInit();
    // Kita biarkan loop() yang menangani koneksi agar tidak blocking
    // dan tidak membuat koneksi "zombie"
    esp_task_wdt_reset();
    yield();
  } else {
    logPrintln("[SETUP] 4. Skip Init MQTT (WiFi tidak terhubung)");
  }

  // 5. Baca data dari unit gree (tampilkan)
  logPrintln("[SETUP] 5. Inisialisasi komunikasi dengan unit Gree ...");
  esp_task_wdt_reset();
  yield();
  
  sensorInit();
  delay(500); // Reduced stabilization time
  yield();
  esp_task_wdt_reset();

  otaInit();
  yield();
  esp_task_wdt_reset();

  BPFrame initFrame = sensorGetLastFrame();
  lastReadUnitState = initFrame.valid;
  
  // Inisialisasi IR control
  logPrintln("[SETUP] 5a. Inisialisasi IR control...");
  controllInit();
  esp_task_wdt_reset();
  yield();
  
  logPrintln("[SETUP] 5b. Baca data dari unit Gree...");
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
      logPrintf("[SETUP] Frame valid ditemukan pada percobaan %d\n", i+1);
      break;
    }
  }
  
  esp_task_wdt_reset();
  yield();
  BPFrame frame = sensorGetLastFrame();
  if (frame.valid) {
    logPrintln("[SETUP] Data Gree berhasil dibaca:");
    logPrintf("  - Suhu: %d°C\n", frame.current_temp);
    logPrintf("  - Power: %s\n", frame.power ? "ON" : "OFF");
    logPrintf("  - Mode: %s\n", frame.mode.c_str());
    logPrintf("  - Fan Speed: %s\n", frame.fan.c_str());
    logPrintf("  - Status Code: %s\n", frame.code_hex.c_str());
    logPrintf("  - Error Code: %s\n", frame.error_val16.c_str());
    logPrintf("  - Errors: %s\n", frame.errors.c_str());
    logPrintf("  - Frame: %s\n", frame.frameHex.c_str());
  } else {
    logPrintln("[SETUP] Gagal membaca data dari unit Gree");
    unsigned long mainboardBytes = sensorGetMainboardByteCount();
    unsigned long displayBytes = sensorGetDisplayByteCount();
    logPrintf("  - Mainboard bytes RX0(IO20): %lu\n", mainboardBytes);
    logPrintf("  - Display bytes RX1(IO3): %lu\n", displayBytes);
    
    if (sensorHasValidFrame()) {
      unsigned long timeSince = sensorTimeSinceLastFrame();
      logPrintf("  - Frame terakhir diterima %lu ms yang lalu\n", timeSince);
    } else {
      logPrintln("  - Belum pernah menerima frame yang valid");
      if (mainboardBytes == 0) {
        logPrintln("  - GPIO20 tidak menerima byte sama sekali -> cek wiring G_MB_RX, divider R10/R11, dan common GND");
      } else {
        logPrintln("  - GPIO20 menerima byte, tapi tidak ada frame 1E 14 yang valid -> cek level sinyal/baud/noise/line source");
      }
    }
  }

  // 6. Kirim data ke server (SKIP DULU DI SETUP)
  // === REVISI: Karena belum connect MQTT, bagian ini di-skip ===
  logPrintln("[SETUP] 6. Skip kirim data awal (Menunggu MQTT Connect di Loop)");

  // 7. Jalankan fitur automation
  logPrintln("[SETUP] 7. Inisialisasi automation...");
  automationInit();
  esp_task_wdt_reset();
  yield();

  logPrintln("[SETUP] 7a. Inisialisasi activity tracker...");
  activityInit();
  esp_task_wdt_reset();
  yield();
  
  if (cfg.setting.automation) {
    logPrintln("[SETUP] Menjalankan automation...");
    automationCheck();
    logPrintln("[SETUP] Automation berhasil dijalankan");
  } else {
    logPrintln("[SETUP] Automation dinonaktifkan");
  }
  esp_task_wdt_reset();
  yield();
  
  lastAutomationMs = millis();
  lastPublishMs = millis();
  dataScheduleImmediateWiFiRetry();
  lastMqttRetryMs = 0;

  // Set initial LED status
  updateLEDStatus();

  logPrintln("\n=== Setup Selesai ===\n");
}

void loop() {
  static bool mqttInitPublished = false;
  static bool mqttInfoPublished = false;
  static int wifiRetryCount = 0; // Counter jumlah kali tryConnectWiFi gagal berturut-turut
  unsigned long now = millis();

  esp_task_wdt_reset();
  yield();

  // Service MQTT keepalive early (mqtt.loop only, no connection management)
  // Full dataLoop() runs later in the main block below.
  if (WiFi.status() == WL_CONNECTED) {
    dataKeepAlive();
    logLoop();
  }
  
  if (now - lastMemoryCheckMs >= MEMORY_CHECK_INTERVAL_MS) {
    lastMemoryCheckMs = now;
    checkMemoryHealth();
  }

  // Poll sensor terlebih dahulu untuk update data AC
  sensorPoll();
  yield();
  esp_task_wdt_reset();
  
  // Check IR LED health (compare display frame with expected state)
  irHealthCheckResponse();
  yield();
  esp_task_wdt_reset();

  checkMemoryHealth();
  yield();
  esp_task_wdt_reset();

  monitorExternalChange();
  yield();
  esp_task_wdt_reset();

  checkNetworkHealth();
  yield();
  esp_task_wdt_reset();

  // Check automation with timeout
  if (millis() - lastAutomationMs >= AUTOMATION_INTERVAL_MS) {
    lastAutomationMs = millis();
    yield();
    esp_task_wdt_reset();
    automationCheck();
    yield();
    esp_task_wdt_reset();
  }

  // WiFi connection management
  if (!wifiOK) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiOK = true;
      wifiRetryCount = 0; // Reset counter saat WiFi reconnect otomatis
      WiFi.setSleep(false);
      feedWatchdogAndYield();
      logPrintf("[WiFi] Reconnected, IP: %s\n", WiFi.localIP().toString().c_str());
      
      // Sinkronisasi waktu ketika WiFi reconnect
      logPrintln("[WiFi] Re-syncing time after reconnection...");
      cfgSyncTime(3000);
      
      // Reset MQTT connection untuk fresh SSL handshake
      dataResetConnection();
      
      lastMqttRetryMs = 0;
      esp_task_wdt_reset();
    } else if (millis() - lastWifiRetryMs >= RETRY_INTERVAL_MS) {
      lastWifiRetryMs = millis();
      yield();
      esp_task_wdt_reset();

      logPrintln("[WiFi] Attempting reconnect...");
      
      // Gunakan tryConnectWiFi yang sudah dibatasi 3 percobaan
      if (tryConnectWiFi()) {
        wifiOK = true;
        wifiRetryCount = 0; // Reset counter saat berhasil
        WiFi.setSleep(false);
        feedWatchdogAndYield();
        
        // Sinkronisasi waktu setelah reconnect berhasil
        logPrintln("[WiFi] Re-syncing time after successful reconnection...");
        cfgSyncTime(3000);
        
        // Reset MQTT connection untuk fresh SSL handshake
        dataResetConnection();
        
        lastMqttRetryMs = 0;
        esp_task_wdt_reset();
      } else {
        wifiRetryCount++;
        logPrintf("[WiFi] Retry ke-%d gagal\n", wifiRetryCount);
        if (wifiRetryCount >= 3) {
          logPrintln("[WiFi] 3x retry gagal - publish status offline...");
          dataNotifyOffline();
          wifiRetryCount = 0; // Reset setelah notify agar tidak spam
        }
      }
    }
  } else {
    if (WiFi.status() != WL_CONNECTED) {
      // ═══════════════════════════════════════════════════════════
      // CATATAN: wifiDisconnectCount sudah di-handle oleh WiFi Event Handler
      // Jadi TIDAK perlu increment lagi di sini (hindari double counting)
      // ═══════════════════════════════════════════════════════════
      logPrintf("[WiFi] Disconnected (status: %d), akan retry sesuai interval\n", WiFi.status());
      
      // Pastikan wifiOK di-set false (event handler mungkin belum fired)
      wifiOK = false;
      lastDisconnectReason = REASON_WIFI_LOST;
      lastDisconnectTime = millis();
      
      logPrintf("[WiFi] Disconnected (status: %d), reason set to WIFI_LOST\n", WiFi.status());
      mqttOK = false;
      dataScheduleImmediateWiFiRetry();
    }
  }

  esp_task_wdt_reset();
  yield();

  // MQTT connection management
  if (wifiOK) {
    // Fungsi ini akan menangani koneksi otomatis jika belum connect
    dataLoop(); 
    dataCheckPendingSave();
    logLoop(); // Process log buffer untuk MQTT publish
    yield();
    esp_task_wdt_reset();
    
    if (dataIsConnected()) {
      if (!mqttOK) {
        mqttOK = true;
        mqttInfoPublished = false;
        lastPublishMs = millis();
        logPrintln("[MQTT] Connected");
        otaValidateFirmware();

        if (!mqttInitPublished) {
          logPrintln("[Init] MQTT connected, waiting for publish cooldown before initial burst...");
        }
      }

      // ═══════════════════════════════════════════════════════════
      // 🆕 PUBLISH INITIAL DATA SAAT PERTAMA KALI CONNECT
      // ═══════════════════════════════════════════════════════════
      if (!mqttInitPublished && dataIsPublishReady()) {
        mqttInitPublished = true;
        
        logPrintln("╔════════════════════════════════════════╗");
        logPrintln("║  📡 PUBLISHING INITIAL DATA           ║");
        logPrintln("╚════════════════════════════════════════╝");
        
        delay(500); // Stabilization delay
        esp_task_wdt_reset();
        
        // Publish crash diagnostics
        logPrintln("[Init] Publishing crash diagnostics...");
        dataPublishDiagnostics();
        delay(300);
        esp_task_wdt_reset();
        
        // Publish system info
        logPrintln("[Init] Publishing system info...");
        mqttInfoPublished = dataPublishInfo();
        delay(300);
        esp_task_wdt_reset();

        // Publish initial telemetry so dashboard has data immediately
        logPrintln("[Init] Publishing initial telemetry...");
        {
          BPFrame f = sensorGetLastFrame();
          bool isHealthy = sensorIsHealthy();
          dataPublishSuhu(f, isHealthy);
          lastReadUnitState = isHealthy;
          lastPublishMs = millis();
        }
        
        logPrintln("✅ Initial data published successfully");
      }

      if (!mqttInfoPublished && dataIsPublishReady()) {
        logPrintln("[Init] Retrying retained system info publish...");
        mqttInfoPublished = dataPublishInfo();
      }
      // ═══════════════════════════════════════════════════════════
      // INFO: Tidak publish info saat reconnect, hanya by request
      // ═══════════════════════════════════════════════════════════
    } else {
      if (mqttOK) {
        mqttOK = false;
        if (isManualDisconnect) {
            // Jika manual, jangan ubah reason (karena sudah diset REASON_MANUAL di data.cpp)
            logPrintln("[MQTT] Disconnected (Manual/Config Save)");
        } else {
            // Jika bukan manual, berarti putus karena jaringan/timeout
            lastDisconnectReason = REASON_MQTT_LOST;
            lastDisconnectTime = millis();
            logPrintln("[MQTT] Disconnected (Timeout/Network Error)");
        }
        logPrintln("[MQTT] Disconnected");
      }
      
      if (millis() - lastMqttRetryMs >= RETRY_INTERVAL_MS) {
        lastMqttRetryMs = millis();
        yield();
        esp_task_wdt_reset();
        logPrintln("[MQTT] Retry connection...");
        tryConnectMQTT();
        esp_task_wdt_reset();
      }
    }
  } else {
    // Jika WiFi tidak OK, pastikan MQTT juga tidak OK
    if (mqttOK) {
      mqttOK = false;
      logPrintln("[MQTT] Disconnected (WiFi not available)");
    }
  } 

  esp_task_wdt_reset();
  yield();

  // === IMMEDIATE PUBLISH ON READ_UNIT CHANGE ===
  if (mqttOK) {
    BPFrame curFrame = sensorGetLastFrame();
    bool isHealthy = sensorIsHealthy();

    if (isHealthy != lastReadUnitState) {
      logPrintf(
        "[Data] read_unit changed: %s -> %s, publish immediately\n",
        lastReadUnitState ? "true" : "false",
        isHealthy ? "true" : "false"
      );

      dataPublishSuhu(curFrame, isHealthy);

      lastPublishMs = millis(); // reset interval timer
      lastReadUnitState = isHealthy;
    }
  }

  // Periodic RTC health check (setiap 60 detik, independen dari MQTT/internet)
  static unsigned long lastRtcCheckMs = 0;
  if (millis() - lastRtcCheckMs >= 60000UL) {
    lastRtcCheckMs = millis();
    rtcOK = cfgCheckRTC();
  }

  // Periodic lastOnline timestamp save (setiap 2 menit, untuk recovery setelah power cut)
  static unsigned long lastTsSaveMs = 0;
  if (millis() - lastTsSaveMs >= 120000UL) {
    lastTsSaveMs = millis();
    if ((unsigned long)time(nullptr) > 1600000000UL) {
      cfgSaveDisconnectTimestamp();
    }
  }

  // Data publishing
  if (mqttOK && (millis() - lastPublishMs >= PUBLISH_INTERVAL_MS)) {
    lastPublishMs = millis();
    yield();
    esp_task_wdt_reset();
    
    BPFrame f = sensorGetLastFrame();
    bool isHealthy = sensorIsHealthy();

    // Check sensor health dan log jika ada masalah
    if (!isHealthy && sensorHasValidFrame()) {
      unsigned long timeSince = sensorTimeSinceLastFrame();
      logPrintf("[Warning] Gree communication timeout: %lu ms since last frame\n", timeSince);
    }
    
    // Gunakan isHealthy (bukan f.valid) sebagai read_ok agar data stale saat
    // timeout tidak dilaporkan sebagai data fresh (read_mainboard: true).
    dataPublishSuhu(f, isHealthy);
    yield();
    esp_task_wdt_reset();
  }

  // Update LED status indicators SETELAH semua status diupdate
  updateLEDStatus();

  delay(100); // 100ms delay - cukup untuk stabilitas tapi tidak terlalu lambat untuk sensor polling
  yield();
}