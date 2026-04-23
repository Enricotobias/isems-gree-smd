// ota.h
#ifndef OTA_H
#define OTA_H

#include <Arduino.h>

// Function Declarations
void otaInit();
void otaProcessManifest(const String& manifestUrl);
void otaDownloadAndFlash(const String& url, const String& expectedMD5);
int otaProbeURL(const String& url);
bool isHTTPS(const String& url);
bool isNewer(String current, String newer);

// 🆕 TAMBAHKAN INI (yang hilang)
void otaValidateFirmware();

#endif