#ifndef DATA_H
#define DATA_H

#include <Arduino.h>
#include "config.h"
#include "sensor.h"

// ==========================================
// 1. DEFINISI ENUM (HARUS PALING ATAS)
// ==========================================
enum DisconnectReason {
  REASON_NONE,
  REASON_POWER_ON,      // Baru nyala / Restart normal / Power cycle
  REASON_POWER_LOSS,    // Brownout terdeteksi (voltage drop)
  REASON_WIFI_LOST,     // WiFi putus mendadak
  REASON_MQTT_LOST,     // WiFi ada, tapi MQTT timeout
  REASON_WDT_RESET,     // Watchdog Reset (Crash / Hang)
  REASON_CRASH,         // Software panic / exception
  REASON_MANUAL,        // Sengaja diputus (Save Config)
  REASON_USB_RESET,     // Reset oleh peripheral USB
  REASON_JTAG_RESET,    // Reset oleh JTAG
  REASON_EFUSE_RESET,   // Reset akibat error eFuse
  REASON_POWER_GLITCH,  // Gangguan daya singkat
  REASON_CPU_LOCKUP     // CPU lockup / double exception
};

// ==========================================
// 2. GLOBAL VARIABLES (EXTERN)
// Agar gree.ino bisa membaca variabel ini
// ==========================================
extern DisconnectReason lastDisconnectReason;
extern unsigned long lastDisconnectTime;
extern bool isManualDisconnect;     // <--- PENTING: Tambahkan ini agar gree.ino tidak error
extern unsigned long mqttReconnectCount;
extern String topicAck; 
extern String topicCmdUpdate;
extern unsigned long mqttReconnectCount;
extern unsigned long wifiDisconnectCount;

// ==========================================
// 3. FUNCTION PROTOTYPES
// ==========================================
void dataInit();
void dataLoop();
void dataKeepAlive();  // Lightweight: hanya mqtt.loop() tanpa connection management
bool dataIsConnected();
bool dataIsPublishReady();
void dataForceConnect();
void dataResetConnection();
void dataDisconnect(); // Tambahkan deklarasi ini jika belum ada
void dataPauseMqttForOta();
void dataResumeMqttAfterOta();
void dataNotifyOffline();  // Publish retained offline setelah retry gagal 3x
void dataScheduleImmediateWiFiRetry();
bool dataConsumeImmediateWiFiRetryRequest();
bool dataIsWiFiCredentialTrialActive();
bool dataIsWiFiCredentialTrialExpired();
void dataOnWiFiCredentialTrialFailedAfterRetries();
void dataOnNetworkConnected();

// Fungsi Publish
void dataPublishSuhu(const BPFrame& d, bool read_ok);
void dataPublishSetting();
bool dataPublishSettingDay(uint8_t day);
void dataPublishSettingDays(uint8_t dayMask);
bool dataPublishInfo();
void publishStatus(const String& status);
bool dataPublishRaw(const String& topic, const String& payload);

// Fungsi Helper
void dataBuildTopics(const String& device_id);
void dataSetServer(const char* host, uint16_t port, const String& user, const String& pass);
void dataCheckPendingSave();
String getResetReason();

// Fungsi ACK
bool dataPublishAck(const char* command, const char* status, const String& detail);

// Fungsi Helper String Enum
String getDisconnectReasonStr(DisconnectReason reason);

void activityBuildTopic(const String& device_id);

#endif
