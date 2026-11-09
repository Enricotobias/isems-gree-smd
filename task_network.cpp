#include "tasks.h"
#include "rtos_common.h"
#include "data.h"
#include "config.h"
#include "sensor.h"
#include "ota.h"
#include "ir_health.h"
#include <WiFi.h>
#include "esp_task_wdt.h"
#include "runtime_health.h"

extern volatile bool wifiOK;
extern volatile bool mqttOK;
extern bool rtcOK;

// Interval constants
static const unsigned long PUBLISH_INTERVAL_MS   = 60000UL;   // 1 minute telemetry
static const unsigned long RETRY_INTERVAL_MS     = 180000UL;  // 3 minute WiFi/MQTT retry
static const unsigned long RTC_CHECK_INTERVAL_MS = 60000UL;   // 1 minute RTC health
static const uint32_t       PRE_PUBLISH_SENSOR_REFRESH_MS = 2000UL;
static const TickType_t    MQTT_LOOP_DELAY       = pdMS_TO_TICKS(50);

static void publishFreshSuhu(bool& lastReadUnitState) {
  sensorRefreshMainboardFrameHeader(PRE_PUBLISH_SENSOR_REFRESH_MS);
  BPFrame f = sensorGetLastFrame();
  bool isHealthy = sensorIsHealthy();
  dataPublishSuhu(f, isHealthy);
  lastReadUnitState = isHealthy;
}

static void markPeriodicTelemetry(unsigned long& lastPublishMs, unsigned long now) {
  if (lastPublishMs == 0 || now - lastPublishMs >= (PUBLISH_INTERVAL_MS * 2UL)) {
    lastPublishMs = now;
  } else {
    lastPublishMs += PUBLISH_INTERVAL_MS;
  }
}

static void publishImmediateSuhu(bool& lastReadUnitState, unsigned long& lastPublishMs) {
  publishFreshSuhu(lastReadUnitState);
  lastPublishMs = millis();
}

static void publishPeriodicSuhuIfDue(bool& lastReadUnitState, unsigned long& lastPublishMs) {
  unsigned long now = millis();
  if (lastPublishMs != 0 && now - lastPublishMs < PUBLISH_INTERVAL_MS) return;

  markPeriodicTelemetry(lastPublishMs, now);
  esp_task_wdt_reset();
  publishFreshSuhu(lastReadUnitState);
}

void TaskNetwork_MQTT(void* param) {
  if (!runtimeRegisterCurrentTask(RUNTIME_NET, "NetTask")) vTaskDelete(NULL);

  // State tracking
  bool mqttInitPublished = false;
  bool mqttInfoPublished = false;
  bool lastReadUnitState = false;
  int  wifiRetryCount    = 0;
  unsigned long lastPublishMs      = 0;
  unsigned long lastWifiRetryMs    = 0;
  unsigned long lastMqttRetryMs    = 0;
  unsigned long lastRtcCheckMs     = 0;
  unsigned long lastStackReport    = 0;

  // dataInit() SELALU dipanggil tanpa kondisi WiFi.
  // syncTime() di dalamnya sudah punya guard sendiri.
  // Ini memastikan mqtt.setCallback() dan setCACert() selalu terpasang,
  // bahkan jika WiFi belum ready saat task pertama kali start.
  dataInit();

  for (;;) {
    runtimeHeartbeat(RUNTIME_NET);
    esp_task_wdt_reset();
    unsigned long now = millis();

    if (dataConsumeImmediateWiFiRetryRequest()) {
      lastWifiRetryMs = now - RETRY_INTERVAL_MS;
    }

    // ════════════════════════════════════════════
    // 1. WiFi Connection Management
    // ════════════════════════════════════════════
    if (!wifiOK) {
      if (WiFi.status() == WL_CONNECTED) {
        wifiOK = true;
        wifiRetryCount = 0;
        WiFi.setSleep(false);
        Serial.printf("[WiFi] Reconnected, IP: %s\n", WiFi.localIP().toString().c_str());
        cfgSyncTime(3000);
        esp_task_wdt_reset();
        dataResetConnection();
        lastMqttRetryMs = 0;
      } else if (now - lastWifiRetryMs >= RETRY_INTERVAL_MS) {
        if (!dataIsConnected()) publishPeriodicSuhuIfDue(lastReadUnitState, lastPublishMs);
        lastWifiRetryMs = now;
        esp_task_wdt_reset();
        if (tryConnectWiFi()) {
          wifiOK = true;
          wifiRetryCount = 0;
          WiFi.setSleep(false);
          cfgSyncTime(3000);
          esp_task_wdt_reset();
          dataResetConnection();
          lastMqttRetryMs = 0;
        } else {
          wifiRetryCount++;
          if (wifiRetryCount >= 3) {
            if (dataIsWiFiCredentialTrialActive()) {
              if (dataIsWiFiCredentialTrialExpired()) {
                dataOnWiFiCredentialTrialFailedAfterRetries();
                wifiRetryCount = 0;
              }
            } else {
              dataNotifyOffline();
              wifiRetryCount = 0;
            }
          }
        }
      }
    } else {
      if (WiFi.status() != WL_CONNECTED) {
        wifiOK = false;
        mqttOK = false;
        lastDisconnectReason = REASON_WIFI_LOST;
        lastDisconnectTime = millis();
        lastWifiRetryMs = 0;
      }
    }

    // ════════════════════════════════════════════
    // 2. MQTT + Core Network Loop
    // ════════════════════════════════════════════
    if (wifiOK) {
      dataLoop();
      dataCheckPendingSave();
      esp_task_wdt_reset();

      if (dataIsConnected()) {
        dataOnNetworkConnected();
        if (!mqttOK) {
          mqttOK = true;
          mqttInfoPublished = false;
          lastPublishMs = millis();
          Serial.println("[MQTT] Connected");
        }

        // Initial publish burst on first connect
        if (!mqttInitPublished && dataIsPublishReady()) {
          mqttInitPublished = true;
          esp_task_wdt_reset();
          mqttInfoPublished = dataPublishInfo();
          publishImmediateSuhu(lastReadUnitState, lastPublishMs);
        }

        if (!mqttInfoPublished && dataIsPublishReady()) {
          mqttInfoPublished = dataPublishInfo();
        }

        // Immediate publish on read_unit state change
        bool isHealthy = sensorIsHealthy();
        if (isHealthy != lastReadUnitState) {
          publishImmediateSuhu(lastReadUnitState, lastPublishMs);
        }

        // Periodic telemetry publish
        publishPeriodicSuhuIfDue(lastReadUnitState, lastPublishMs);

      } else {
        if (mqttOK) {
          mqttOK = false;
          if (!isManualDisconnect) {
            lastDisconnectReason = REASON_MQTT_LOST;
            lastDisconnectTime = millis();
          }
        }
      }
    } else {
      if (mqttOK) mqttOK = false;
    }

    // Offline telemetry sampling tetap jalan saat MQTT tidak connected.
    // dataPublishSuhu() akan menyimpan ke LittleFS jika MQTT putus atau publish gagal.
    if (!dataIsConnected()) publishPeriodicSuhuIfDue(lastReadUnitState, lastPublishMs);

    // ════════════════════════════════════════════
    // 3. IR Health, Memory, Network, LED
    // ════════════════════════════════════════════
    irHealthCheckResponse();
    checkMemoryHealth();
    checkNetworkHealth();
    updateLEDStatus();

    // ════════════════════════════════════════════
    // 4. Periodic Housekeeping (low frequency)
    // ════════════════════════════════════════════
    // RTC health check (every 60s)
    if (now - lastRtcCheckMs >= RTC_CHECK_INTERVAL_MS) {
      lastRtcCheckMs = now;
      rtcOK = cfgCheckRTC();
    }

    if (now - lastStackReport >= 300000UL) {
      lastStackReport = now;
      reportStackHighWaterMarks();
    }

    otaPendingAckService();
    otaValidationService();
    vTaskDelay(MQTT_LOOP_DELAY);
  }
}
