#include "logger.h"

// ==========================================
// LOGGER DENGAN MQTT CONTROL
// ==========================================
// Log ke Serial Monitor selalu aktif
// Log ke MQTT dapat diaktifkan/dinonaktifkan via command
// ==========================================

static bool mqttLoggingEnabled = false;
static void (*mqttPublishFunc)(const String&) = nullptr;
static String logBuffer = "";
static unsigned long lastLogPublishMs = 0;
static const unsigned long LOG_PUBLISH_INTERVAL_MS = 500; // Publish setiap 500ms

void logInit() {
  mqttLoggingEnabled = false;
  logBuffer.reserve(512);
}

void logPrint(const String& s) {
  Serial.print(s);
  
  if (mqttLoggingEnabled && mqttPublishFunc != nullptr) {
    logBuffer += s;
  }
}

void logPrintln(const String& s) {
  Serial.println(s);
  
  if (mqttLoggingEnabled && mqttPublishFunc != nullptr) {
    logBuffer += s;
    logBuffer += "\n";
  }
}

void logPrintln(const char* s) {
  Serial.println(s);
  
  if (mqttLoggingEnabled && mqttPublishFunc != nullptr) {
    logBuffer += s;
    logBuffer += "\n";
  }
}

void logPrintln() {
  Serial.println();
  
  if (mqttLoggingEnabled && mqttPublishFunc != nullptr) {
    logBuffer += "\n";
  }
}

void logPrintf(const char* format, ...) {
  char buffer[256];

  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  Serial.print(buffer);
  
  if (mqttLoggingEnabled && mqttPublishFunc != nullptr) {
    logBuffer += String(buffer);
  }
}

void logLoop() {
  // Publish log buffer ke MQTT secara periodik
  if (mqttLoggingEnabled && mqttPublishFunc != nullptr && logBuffer.length() > 0) {
    unsigned long now = millis();
    if (now - lastLogPublishMs >= LOG_PUBLISH_INTERVAL_MS) {
      mqttPublishFunc(logBuffer);
      logBuffer = "";
      lastLogPublishMs = now;
    }
  }
}

// ==========================================
// MQTT LOGGING CONTROL FUNCTIONS
// ==========================================

void logSetMqttEnabled(bool enabled) {
  mqttLoggingEnabled = enabled;
  if (!enabled) {
    logBuffer = ""; // Clear buffer saat dinonaktifkan
  }
}

bool logGetMqttEnabled() {
  return mqttLoggingEnabled;
}

void logSetMqttPublisher(void (*publisher)(const String&)) {
  mqttPublishFunc = publisher;
}
