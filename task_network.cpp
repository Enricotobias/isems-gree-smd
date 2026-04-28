#include "tasks.h"
#include "rtos_common.h"
#include "data.h"
#include "config.h"
#include "sensor.h"
#include "ota.h"
#include "ir_health.h"
#include <WiFi.h>
#include "esp_task_wdt.h"

extern volatile bool wifiOK;
extern volatile bool mqttOK;
extern bool rtcOK;

// Interval constants
static const unsigned long PUBLISH_INTERVAL_MS   = 60000UL;   // 1 minute telemetry
static const unsigned long RETRY_INTERVAL_MS     = 180000UL;  // 3 minute WiFi/MQTT retry
static const unsigned long RTC_CHECK_INTERVAL_MS = 60000UL;   // 1 minute RTC health
static const TickType_t    MQTT_LOOP_DELAY       = pdMS_TO_TICKS(50);

void TaskNetwork_MQTT(void* param) {
  esp_task_wdt_add(NULL);

  // State tracking
  bool mqttInitPublished = false;
  bool mqttInfoPublished = false;
  bool lastReadUnitState = false;
  int  wifiRetryCount    = 0;
  unsigned long lastPublishMs      = 0;
  unsigned long lastWifiRetryMs    = 0;
  unsigned long lastMqttRetryMs    = 0;
  unsigned long lastRtcCheckMs     = 0;

  if (WiFi.status() == WL_CONNECTED) {
    dataInit();
  }

  for (;;) {
    esp_task_wdt_reset();
    unsigned long now = millis();

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
            dataNotifyOffline();
            wifiRetryCount = 0;
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
        if (!mqttOK) {
          mqttOK = true;
          mqttInfoPublished = false;
          lastPublishMs = millis();
          Serial.println("[MQTT] Connected");
          otaValidateFirmware();
        }

        // Initial publish burst on first connect
        if (!mqttInitPublished && dataIsPublishReady()) {
          mqttInitPublished = true;
          esp_task_wdt_reset();
          mqttInfoPublished = dataPublishInfo();
          // Initial telemetry
          BPFrame f = sensorGetLastFrame();
          bool isHealthy = sensorIsHealthy();
          dataPublishSuhu(f, isHealthy);
          lastReadUnitState = isHealthy;
          lastPublishMs = millis();
        }

        if (!mqttInfoPublished && dataIsPublishReady()) {
          mqttInfoPublished = dataPublishInfo();
        }

        // Immediate publish on read_unit state change
        BPFrame curFrame = sensorGetLastFrame();
        bool isHealthy = sensorIsHealthy();
        if (isHealthy != lastReadUnitState) {
          dataPublishSuhu(curFrame, isHealthy);
          lastPublishMs = millis();
          lastReadUnitState = isHealthy;
        }

        // Periodic telemetry publish
        if (millis() - lastPublishMs >= PUBLISH_INTERVAL_MS) {
          lastPublishMs = millis();
          esp_task_wdt_reset();
          BPFrame f = sensorGetLastFrame();
          dataPublishSuhu(f, sensorIsHealthy());
        }

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

    vTaskDelay(MQTT_LOOP_DELAY);
  }
}
