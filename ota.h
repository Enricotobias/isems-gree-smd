// ota.h
#ifndef OTA_H
#define OTA_H

#include <Arduino.h>

// Function Declarations
void otaInit();
bool otaProcessManifest(const String& manifestUrl, String& outDetail);
bool otaDownloadAndFlash(const String& url, const String& expectedMD5,
                         String& outDetail, const String& targetVersion = "");
bool otaRollbackToPrevious(String& outDetail);
int otaProbeURL(const String& url);
bool isHTTPS(const String& url);
bool isNewer(String current, String newer);

// 🆕 TAMBAHKAN INI (yang hilang)
void otaValidateFirmware();
void otaValidationService();
void otaPendingAckService();
void otaQueuePendingAck(const char* status, const String& detail, bool forceWrite = false);
void otaClearPendingAck();

#endif
