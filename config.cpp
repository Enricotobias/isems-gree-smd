#include "config.h"

#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <Wire.h>
#include "data.h"
#include "esp_task_wdt.h"
#include "esp_sntp.h"
#include "esp_mac.h"
#include <new>


RTC_PCF8563 rtc;

const char* CONFIG_PATH = "/config.json";
static const char* CONFIG_TMP_PATH = "/config.tmp";
static const char* CONFIG_BAK_PATH = "/config.bak";
static const char* SCHEDULE_PATH = "/schedule.bin";
static const char* SCHEDULE_TMP_PATH = "/schedule.tmp";
static const char* SCHEDULE_BAK_PATH = "/schedule.bak";
static const char* SCHEDULE_NEW_PATH = "/schedule.new";
static const size_t CONFIG_JSON_DOC_CAPACITY = 4096;
static const size_t CONFIG_FILE_MAX_BYTES = 16384;
static const uint32_t SCHEDULE_MAGIC = 0x53434847UL; // GCHS
static const uint16_t SCHEDULE_VERSION_V1 = 1;
static const uint16_t SCHEDULE_VERSION = 2;
static const uint8_t SCHEDULE_V1_MAX_PERIODS = 5;

const char* DEF_WIFI_SSID = "IOT";
const char* DEF_WIFI_PASS = "12345678";
const char* DEF_MQTT_HOST = "mqtt.nicourbanindonesia.com";
const uint16_t DEF_MQTT_PORT = 8883;  // Port 8883 untuk MQTT over SSL/TLS
const char* DEF_MQTT_USER = "isems";
const char* DEF_MQTT_PASS = "Atop88ls";

const bool   DEF_AUTOMATION = false;
const int    DEF_GLOBAL_SET_TEMP = 24;
const int    DEF_GLOBAL_MIN_TEMP = 20;
const int    DEF_GLOBAL_MAX_TEMP = 26;
const char*  DEF_GLOBAL_MODE     = "COOL";
const char*  DEF_GLOBAL_FAN      = "AUTO";
const int    DEF_TEMP_COOLDOWN   = 300;
const int    DEF_AUTO_RECOVERY   = 300;
const int8_t DEF_TZ_OFFSET       = 7;     // WIB=7, WITA=8, WIT=9

// Helper: buat POSIX TZ string dari offset
static String cfgGetTZString(int8_t offset) {
  const char* name;
  switch (offset) {
    case 7:  name = "WIB";  break;
    case 8:  name = "WITA"; break;
    case 9:  name = "WIT";  break;
    default: name = "UTC";  break;
  }
  return String(name) + String(-offset);
}

static void feedWatchdogAndYield() {
  esp_task_wdt_reset();
  yield();
}

Config cfg;
static bool cfgRuntimeDefaultDueLoadFailure = false;
static bool cfgDeferredDefaultSaveStarted = false;

static Preferences _wifiPrefs;

bool cfgIsRuntimeDefaultDueLoadFailure() {
  return cfgRuntimeDefaultDueLoadFailure;
}

static void initDayOfftime(DaySchedule& d);

static bool copyConfigFile(const char* srcPath, const char* dstPath) {
  File src = LittleFS.open(srcPath, "r");
  if (!src) return false;

  File dst = LittleFS.open(dstPath, "w");
  if (!dst) {
    src.close();
    return false;
  }

  std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[256]);
  if (!buf) {
    src.close();
    dst.close();
    LittleFS.remove(dstPath);
    return false;
  }
  bool ok = true;
  while (src.available()) {
    size_t n = src.read(buf.get(), 256);
    if (n == 0) break;
    if (dst.write(buf.get(), n) != n) {
      ok = false;
      break;
    }
    yield();
  }

  dst.flush();
  dst.close();
  src.close();

  if (!ok) LittleFS.remove(dstPath);
  return ok;
}

static bool recoverConfigFromBackup(const char* reason) {
  const char* candidates[] = { CONFIG_TMP_PATH, CONFIG_BAK_PATH };
  for (const char* candidate : candidates) {
    if (!LittleFS.exists(candidate)) continue;
    Serial.Printf("[Config] Attempting recovery from %s (%s)\n", candidate, reason ? reason : "-");
    LittleFS.remove(CONFIG_PATH);
    if (copyConfigFile(candidate, CONFIG_PATH)) {
      Serial.Printf("[Config] Recovered active config from %s\n", candidate);
      return true;
    }
  }
  return false;
}

struct BinaryTimePeriod {
  char start[6];
  char end[6];
  char mode[6];
  char fan[8];
  uint8_t set_temp;
  uint8_t min_temp;
  uint8_t max_temp;
} __attribute__((packed));

struct BinaryDaySchedule {
  uint8_t type;
  uint8_t periodCount;
  BinaryTimePeriod periods[MAX_PERIODS_PER_DAY];
} __attribute__((packed));

struct BinaryScheduleFile {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  BinaryDaySchedule days[7];
  uint32_t crc;
} __attribute__((packed));

struct BinaryTimePeriodV1 {
  char start[6];
  char end[6];
  char mode[6];
  char fan[8];
  uint8_t set_temp;
  uint8_t min_temp;
  uint8_t max_temp;
} __attribute__((packed));

struct BinaryDayScheduleV1 {
  uint8_t type;
  uint8_t periodCount;
  BinaryTimePeriodV1 periods[SCHEDULE_V1_MAX_PERIODS];
} __attribute__((packed));

struct BinaryScheduleFileV1 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  BinaryDayScheduleV1 days[7];
  uint32_t crc;
} __attribute__((packed));

struct BinaryScheduleHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
} __attribute__((packed));

static_assert(sizeof(BinaryTimePeriodV1) == sizeof(BinaryTimePeriod),
              "Schedule V1 period layout must remain unchanged");

template <typename T>
static uint32_t calcScheduleCrc(const T& file) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&file);
  const size_t crcOffset = offsetof(T, crc);
  const size_t crcSize = sizeof(file.crc);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < sizeof(file); ++i) {
    const uint8_t value = (i >= crcOffset && i < crcOffset + crcSize) ? 0 : bytes[i];
    hash ^= value;
    hash *= 16777619UL;
  }
  return hash;
}

static void copyTextField(char* dst, size_t dstLen, const char* src) {
  if (!dst || dstLen == 0) return;
  memset(dst, 0, dstLen);
  if (src) {
    strncpy(dst, src, dstLen - 1);
  }
}

static void buildScheduleFile(BinaryScheduleFile& out) {
  memset(&out, 0, sizeof(out));
  out.magic = SCHEDULE_MAGIC;
  out.version = SCHEDULE_VERSION;
  out.size = sizeof(BinaryScheduleFile);

  for (uint8_t i = 0; i < 7; ++i) {
    const DaySchedule& srcDay = cfg.setting.daily[i];
    BinaryDaySchedule& dstDay = out.days[i];
    dstDay.type = static_cast<uint8_t>(srcDay.type);
    dstDay.periodCount = srcDay.periodCount;
    if (dstDay.periodCount > MAX_PERIODS_PER_DAY) dstDay.periodCount = MAX_PERIODS_PER_DAY;

    for (uint8_t p = 0; p < dstDay.periodCount; ++p) {
      const TimePeriod& src = srcDay.periods[p];
      BinaryTimePeriod& dst = dstDay.periods[p];
      copyTextField(dst.start, sizeof(dst.start), src.start);
      copyTextField(dst.end, sizeof(dst.end), src.end);
      copyTextField(dst.mode, sizeof(dst.mode), src.mode);
      copyTextField(dst.fan, sizeof(dst.fan), src.fan);
      dst.set_temp = src.set_temp;
      dst.min_temp = src.min_temp;
      dst.max_temp = src.max_temp;
    }
  }

  out.crc = calcScheduleCrc(out);
}

template <typename T>
static bool applyScheduleFile(const T& file, uint16_t expectedVersion) {
  if (file.magic != SCHEDULE_MAGIC || file.version != expectedVersion || file.size != sizeof(T)) {
    return false;
  }
  if (file.crc != calcScheduleCrc(file)) {
    return false;
  }

  for (uint8_t i = 0; i < 7; ++i) {
    const auto& srcDay = file.days[i];
    DaySchedule& dstDay = cfg.setting.daily[i];
    memset(&dstDay, 0, sizeof(dstDay));

    if (srcDay.type > DAY_PARTTIME) {
      initDayOfftime(dstDay);
      continue;
    }

    dstDay.type = static_cast<DayType>(srcDay.type);
    dstDay.periodCount = srcDay.periodCount;
    if (dstDay.type == DAY_OFFTIME) dstDay.periodCount = 0;
    if (dstDay.periodCount > MAX_PERIODS_PER_DAY) dstDay.periodCount = MAX_PERIODS_PER_DAY;

    for (uint8_t p = 0; p < dstDay.periodCount; ++p) {
      const auto& src = srcDay.periods[p];
      TimePeriod& dst = dstDay.periods[p];
      copyTextField(dst.start, sizeof(dst.start), src.start);
      copyTextField(dst.end, sizeof(dst.end), src.end);
      copyTextField(dst.mode, sizeof(dst.mode), src.mode);
      copyTextField(dst.fan, sizeof(dst.fan), src.fan);
      dst.set_temp = src.set_temp;
      dst.min_temp = src.min_temp;
      dst.max_temp = src.max_temp;
    }
  }

  return true;
}

static bool readScheduleBinaryFrom(const char* path, uint16_t& loadedVersion) {
  loadedVersion = 0;
  File f = LittleFS.open(path, "r");
  if (!f) return false;

  BinaryScheduleHeader header = {};
  const size_t headerRead = f.read(reinterpret_cast<uint8_t*>(&header), sizeof(header));
  if (headerRead != sizeof(header) || header.magic != SCHEDULE_MAGIC) {
    f.close();
    return false;
  }
  f.seek(0);

  if (header.version == SCHEDULE_VERSION_V1 &&
      header.size == sizeof(BinaryScheduleFileV1) &&
      f.size() == sizeof(BinaryScheduleFileV1)) {
    std::unique_ptr<BinaryScheduleFileV1> file(new (std::nothrow) BinaryScheduleFileV1{});
    if (!file) {
      f.close();
      return false;
    }
    const size_t bytesRead = f.read(reinterpret_cast<uint8_t*>(file.get()), sizeof(*file));
    f.close();
    if (bytesRead == sizeof(*file) && applyScheduleFile(*file, SCHEDULE_VERSION_V1)) {
      loadedVersion = SCHEDULE_VERSION_V1;
      return true;
    }
    return false;
  }

  if (header.version == SCHEDULE_VERSION &&
      header.size == sizeof(BinaryScheduleFile) &&
      f.size() == sizeof(BinaryScheduleFile)) {
    std::unique_ptr<BinaryScheduleFile> file(new (std::nothrow) BinaryScheduleFile{});
    if (!file) {
      f.close();
      return false;
    }
    const size_t bytesRead = f.read(reinterpret_cast<uint8_t*>(file.get()), sizeof(*file));
    f.close();
    if (bytesRead == sizeof(*file) && applyScheduleFile(*file, SCHEDULE_VERSION)) {
      loadedVersion = SCHEDULE_VERSION;
      return true;
    }
    return false;
  }

  f.close();
  return false;
}

static bool saveScheduleBinary() {
  std::unique_ptr<BinaryScheduleFile> file(new (std::nothrow) BinaryScheduleFile{});
  if (!file) {
    Serial.Println("[Config] Error: Cannot allocate binary schedule buffer");
    return false;
  }
  buildScheduleFile(*file);

  if (LittleFS.exists(SCHEDULE_NEW_PATH)) {
    LittleFS.remove(SCHEDULE_NEW_PATH);
  }

  File f = LittleFS.open(SCHEDULE_NEW_PATH, "w");
  if (!f) {
    Serial.Println("[Config] Error: Cannot open temp schedule for write");
    return false;
  }

  size_t written = f.write(reinterpret_cast<const uint8_t*>(file.get()), sizeof(*file));
  f.flush();
  f.close();

  if (written != sizeof(*file)) {
    Serial.Println("[Config] Error: Failed to write binary schedule");
    LittleFS.remove(SCHEDULE_NEW_PATH);
    return false;
  }

  bool activeMovedToBackup = false;
  if (LittleFS.exists(SCHEDULE_PATH)) {
    LittleFS.remove(SCHEDULE_BAK_PATH);
    if (!LittleFS.rename(SCHEDULE_PATH, SCHEDULE_BAK_PATH)) {
      Serial.Println("[Config] Error: Cannot preserve old binary schedule");
      return false;
    }
    activeMovedToBackup = true;
  }

  if (!LittleFS.rename(SCHEDULE_NEW_PATH, SCHEDULE_PATH)) {
    Serial.Println("[Config] Error: Cannot promote temp binary schedule");
    if (activeMovedToBackup && !LittleFS.rename(SCHEDULE_BAK_PATH, SCHEDULE_PATH)) {
      Serial.Println("[Config] Error: Cannot restore previous binary schedule");
    }
    return false;
  }

  LittleFS.remove(SCHEDULE_BAK_PATH);
  if (!copyConfigFile(SCHEDULE_PATH, SCHEDULE_BAK_PATH)) {
    Serial.Println("[Config] Warning: binary schedule saved, but backup copy failed");
  }

  Serial.Printf("[Config] Binary schedule saved: %u bytes\n", static_cast<unsigned>(sizeof(*file)));
  return true;
}

bool cfgSaveSchedule() {
  if (!LittleFS.begin()) {
    Serial.Println("[Config] Error: LittleFS unavailable for schedule save");
    return false;
  }
  return saveScheduleBinary();
}

static bool loadScheduleBinary() {
  struct ScheduleCandidate {
    const char* path;
    const char* label;
  };
  const ScheduleCandidate candidates[] = {
      {SCHEDULE_PATH, "active"},
      {SCHEDULE_TMP_PATH, "temp"},
      {SCHEDULE_BAK_PATH, "backup"},
  };

  for (const ScheduleCandidate& candidate : candidates) {
    uint16_t loadedVersion = 0;
    if (!readScheduleBinaryFrom(candidate.path, loadedVersion)) continue;

    Serial.Printf("[Config] Binary schedule loaded from %s (v%u)\n",
                  candidate.label,
                  static_cast<unsigned>(loadedVersion));

    const bool requiresRewrite = loadedVersion != SCHEDULE_VERSION ||
                                 strcmp(candidate.path, SCHEDULE_PATH) != 0;
    if (requiresRewrite) {
      if (strcmp(candidate.path, SCHEDULE_BAK_PATH) == 0) {
        LittleFS.remove(SCHEDULE_TMP_PATH);
        if (!copyConfigFile(SCHEDULE_BAK_PATH, SCHEDULE_TMP_PATH)) {
          Serial.Println("[Config] Warning: Cannot preserve backup source before migration");
        }
      }
      if (!saveScheduleBinary()) {
        Serial.Println("[Config] Warning: Schedule loaded but V2 rewrite failed");
      } else if (loadedVersion == SCHEDULE_VERSION_V1) {
        Serial.Println("[Config] Binary schedule migrated from V1 to V2");
      }
    }
    return true;
  }

  return false;
}

String nvs_getWiFiSSID() {
  _wifiPrefs.begin("wifi", true);
  bool exists = _wifiPrefs.isKey("ssid");
  String v = _wifiPrefs.getString("ssid", DEF_WIFI_SSID);
  _wifiPrefs.end();
  // Jika key belum pernah ditulis, tulis sekarang agar OTA tidak bisa mengubahnya
  if (!exists) {
    _wifiPrefs.begin("wifi", false);
    _wifiPrefs.putString("ssid", v);
    _wifiPrefs.end();
  }
  return v;
}

String nvs_getWiFiPass() {
  _wifiPrefs.begin("wifi", true);
  bool exists = _wifiPrefs.isKey("pass");
  String v = _wifiPrefs.getString("pass", DEF_WIFI_PASS);
  _wifiPrefs.end();
  // Jika key belum pernah ditulis, tulis sekarang agar OTA tidak bisa mengubahnya
  if (!exists) {
    _wifiPrefs.begin("wifi", false);
    _wifiPrefs.putString("pass", v);
    _wifiPrefs.end();
  }
  return v;
}

void nvs_setWiFiSSID(const String& ssid) {
  _wifiPrefs.begin("wifi", false);
  _wifiPrefs.putString("ssid", ssid);
  _wifiPrefs.end();
}

void nvs_setWiFiPass(const String& pass) {
  _wifiPrefs.begin("wifi", false);
  _wifiPrefs.putString("pass", pass);
  _wifiPrefs.end();
}

String getStableDeviceMac() {
  uint8_t mac[6];
  if (esp_efuse_mac_get_default(mac) == ESP_OK) {
    char buf[20];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(buf);
  }
  return WiFi.macAddress();
}

const char* PROD_ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

static void initDayOfftime(DaySchedule& d) {
  d.type = DAY_OFFTIME;
  d.periodCount = 0;
  memset(d.periods, 0, sizeof(d.periods));
}

static void initDayFulltime(DaySchedule& d,
                            uint8_t setTemp = DEF_GLOBAL_SET_TEMP,
                            uint8_t minTemp = DEF_GLOBAL_MIN_TEMP,
                            uint8_t maxTemp = DEF_GLOBAL_MAX_TEMP,
                            const char* mode = DEF_GLOBAL_MODE,
                            const char* fan = DEF_GLOBAL_FAN) {
  d.type = DAY_FULLTIME;
  d.periodCount = 1;
  memset(&d.periods[0], 0, sizeof(TimePeriod));
  strncpy(d.periods[0].mode, mode, sizeof(d.periods[0].mode) - 1);
  strncpy(d.periods[0].fan, fan, sizeof(d.periods[0].fan) - 1);
  d.periods[0].set_temp = setTemp;
  d.periods[0].min_temp = minTemp;
  d.periods[0].max_temp = maxTemp;
}

static void initDayParttime(DaySchedule& d,
                            const char* start, const char* end,
                            uint8_t setTemp = DEF_GLOBAL_SET_TEMP,
                            uint8_t minTemp = DEF_GLOBAL_MIN_TEMP,
                            uint8_t maxTemp = DEF_GLOBAL_MAX_TEMP,
                            const char* mode = DEF_GLOBAL_MODE,
                            const char* fan = DEF_GLOBAL_FAN) {
  d.type = DAY_PARTTIME;
  d.periodCount = 1;
  memset(&d.periods[0], 0, sizeof(TimePeriod));
  strncpy(d.periods[0].start, start, sizeof(d.periods[0].start) - 1);
  strncpy(d.periods[0].end, end, sizeof(d.periods[0].end) - 1);
  strncpy(d.periods[0].mode, mode, sizeof(d.periods[0].mode) - 1);
  strncpy(d.periods[0].fan, fan, sizeof(d.periods[0].fan) - 1);
  d.periods[0].set_temp = setTemp;
  d.periods[0].min_temp = minTemp;
  d.periods[0].max_temp = maxTemp;
}

static void initDefaultSchedule() {
  // Day index standard: 0=Minggu, 1=Senin, ..., 6=Sabtu
  initDayOfftime(cfg.setting.daily[0]);  // Minggu = libur
  for (int i = 1; i <= 5; i++) {
    initDayParttime(cfg.setting.daily[i], "07:55", "17:00"); // Senin-Jumat
  }
  initDayParttime(cfg.setting.daily[6], "07:55", "13:00");   // Sabtu = setengah hari
}

static void parsePeriodFromJson(TimePeriod& p, JsonObject pObj, bool isFulltime = false) {
  memset(&p, 0, sizeof(TimePeriod));
  if (!isFulltime) {
    strncpy(p.start, pObj["start"] | "08:00", sizeof(p.start) - 1);
    strncpy(p.end, pObj["end"] | "17:00", sizeof(p.end) - 1);
  }
  strncpy(p.mode, pObj["mode"] | "", sizeof(p.mode) - 1);
  strncpy(p.fan, pObj["fan"] | "", sizeof(p.fan) - 1);
  for (int i = 0; p.mode[i]; i++) p.mode[i] = toupper(p.mode[i]);
  for (int i = 0; p.fan[i]; i++) p.fan[i] = toupper(p.fan[i]);
  p.set_temp = (uint8_t)(pObj["set_temp"] | 0);
  p.min_temp = (uint8_t)(pObj["min_temp"] | 0);
  p.max_temp = (uint8_t)(pObj["max_temp"] | 0);
}

// Load default configuration into memory
static void loadDefaultConfig() {
  cfg.ac_brand = AC_BRAND;
  cfg.setting.automationPausedByUser = false;
  
  cfg.koneksi.wifi_ssid = nvs_getWiFiSSID();
  cfg.koneksi.wifi_pass = nvs_getWiFiPass();
  cfg.koneksi.mqtt_host = DEF_MQTT_HOST;
  cfg.koneksi.mqtt_port = DEF_MQTT_PORT;
  cfg.koneksi.mqtt_user = DEF_MQTT_USER;
  cfg.koneksi.mqtt_pass = DEF_MQTT_PASS;
  cfg.koneksi.tz_offset = DEF_TZ_OFFSET;

  cfg.setting.automation = DEF_AUTOMATION;
  cfg.setting.global_set_temp = DEF_GLOBAL_SET_TEMP;
  cfg.setting.global_min_temp = DEF_GLOBAL_MIN_TEMP;
  cfg.setting.global_max_temp = DEF_GLOBAL_MAX_TEMP;
  cfg.setting.global_mode = DEF_GLOBAL_MODE;
  cfg.setting.global_fan = DEF_GLOBAL_FAN;
  cfg.setting.temp_cooldown_sec = DEF_TEMP_COOLDOWN;
  cfg.setting.auto_recovery_sec = 300; // 5 Menit (Standar aman kompresor)

  initDefaultSchedule();

  cfg.setting.manualModeActive = false;
  cfg.setting.lastManualControlMs = 0;
}

bool cfgSave() {
  // Mount Check
  if (!LittleFS.begin()) {
    Serial.Println("[Config] WARNING: LittleFS not mounted, trying to mount...");
    if (!LittleFS.begin(true)) {
      Serial.Println("[Config] ERROR: LittleFS mount failed completely");
      return false;
    }
  }

  // === DIAGNOSTIK PARTISI ===
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  Serial.Printf("[Config] Info: Total Space: %d bytes, Used: %d bytes\n", totalBytes, usedBytes);

  if (totalBytes == 0) {
    Serial.Println("[Config] FATAL ERROR: Partition size is 0! Check 'Tools > Partition Scheme' in Arduino IDE.");
    return false;
  }
  // ==========================

  if (!saveScheduleBinary()) {
    Serial.Println("[Config] Error: Binary schedule save failed, config not saved");
    return false;
  }

  DynamicJsonDocument doc(CONFIG_JSON_DOC_CAPACITY);

  // Immutable metadata: always write from firmware constant
  cfg.ac_brand = AC_BRAND;
  doc["ac_brand"] = cfg.ac_brand;

  JsonObject koneksi = doc.createNestedObject("koneksi");
  koneksi["mqtt_host"] = cfg.koneksi.mqtt_host;
  koneksi["mqtt_port"] = cfg.koneksi.mqtt_port;
  koneksi["mqtt_user"] = cfg.koneksi.mqtt_user;
  koneksi["mqtt_pass"] = cfg.koneksi.mqtt_pass;
  koneksi["tz_offset"] = cfg.koneksi.tz_offset;

  JsonObject setting = doc.createNestedObject("setting");
  setting["automation"] = cfg.setting.automation;
  
  setting["automationPausedByUser"] = cfg.setting.automationPausedByUser;
  setting["manualModeActive"] = cfg.setting.manualModeActive;
  setting["lastManualControlMs"] = (unsigned long)cfg.setting.lastManualControlMs;
  setting["schedule_storage"] = "binary";
  setting["schedule_version"] = SCHEDULE_VERSION;

  if (doc.overflowed()) {
    Serial.Println("[Config] Error: JSON document overflow, config not saved");
    return false;
  }

  if (LittleFS.exists(CONFIG_TMP_PATH)) {
    LittleFS.remove(CONFIG_TMP_PATH);
  }

  File f = LittleFS.open(CONFIG_TMP_PATH, "w");

  if (!f) {
    Serial.Println("[Config] Error: Cannot open temp config for write (Persistent failure)");
    return false;
  }

  size_t bytesWritten = serializeJsonPretty(doc, f);
  f.close();
  
  if (bytesWritten == 0) {
    Serial.Println("[Config] Error: Failed to write config (0 bytes written)");
    LittleFS.remove(CONFIG_TMP_PATH);
    return false;
  }

  if (LittleFS.exists(CONFIG_PATH) && !LittleFS.remove(CONFIG_PATH)) {
    Serial.Println("[Config] Error: Cannot remove old config, keeping temp file");
    return false;
  }

  if (!LittleFS.rename(CONFIG_TMP_PATH, CONFIG_PATH)) {
    Serial.Println("[Config] Error: Cannot promote temp config to active config");
    return false;
  }

  LittleFS.remove(CONFIG_BAK_PATH);
  if (!copyConfigFile(CONFIG_PATH, CONFIG_BAK_PATH)) {
    Serial.Println("[Config] Warning: active config saved, but backup copy failed");
  }

  Serial.Printf("[Config] Successfully saved %d bytes to %s\n", bytesWritten, CONFIG_PATH);
  cfgRuntimeDefaultDueLoadFailure = false;
  return true;
} 

static void cfgDeferredDefaultSaveTask(void*) {
  Serial.Println("[Config] Deferred default config save started");
  const bool saved = cfgSave();
  cfgRuntimeDefaultDueLoadFailure = !saved;
  cfgDeferredDefaultSaveStarted = false;
  Serial.Printf("[Config] Deferred default config save %s\n", saved ? "succeeded" : "failed");
  vTaskDelete(nullptr);
}

void cfgStartDeferredDefaultSave() {
  if (!cfgRuntimeDefaultDueLoadFailure || cfgDeferredDefaultSaveStarted) return;

  cfgDeferredDefaultSaveStarted = true;
  const BaseType_t created = xTaskCreate(
      cfgDeferredDefaultSaveTask,
      "CfgSave",
      8192,
      nullptr,
      1,
      nullptr);
  if (created != pdPASS) {
    cfgDeferredDefaultSaveStarted = false;
    Serial.Println("[Config] ERROR: Cannot create deferred default save task");
  }
}

bool cfgLoad() {
  static bool recoveryAttempted = false;
  cfgRuntimeDefaultDueLoadFailure = false;

  if (!LittleFS.begin()) {
    Serial.Println("WARNING: LittleFS not mounted, using default config");
    loadDefaultConfig();
    cfgRuntimeDefaultDueLoadFailure = true;
    return false;
  }

  if (!LittleFS.exists(CONFIG_PATH)) {
    if (!recoveryAttempted && recoverConfigFromBackup("active config missing")) {
      recoveryAttempted = true;
      bool ok = cfgLoad();
      recoveryAttempted = false;
      return ok;
    }
    Serial.Println("WARNING: Config file not found, loading default config");
    loadDefaultConfig();
    cfgRuntimeDefaultDueLoadFailure = true;
    return false;
  }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    if (!recoveryAttempted && recoverConfigFromBackup("active config open failed")) {
      recoveryAttempted = true;
      bool ok = cfgLoad();
      recoveryAttempted = false;
      return ok;
    }
    Serial.Println("ERROR: Cannot open config for read");
    loadDefaultConfig();
    cfgRuntimeDefaultDueLoadFailure = true;
    return false;
  }

  size_t size = f.size();
  
  if (size > CONFIG_FILE_MAX_BYTES) {
    Serial.Printf("ERROR: Config file too large (%u > %u), possibly corrupt\n",
                  (unsigned)size, (unsigned)CONFIG_FILE_MAX_BYTES);
    f.close();
    if (!recoveryAttempted && recoverConfigFromBackup("active config too large")) {
      recoveryAttempted = true;
      bool ok = cfgLoad();
      recoveryAttempted = false;
      return ok;
    }
    loadDefaultConfig();
    cfgRuntimeDefaultDueLoadFailure = true;
    return false;
  }
  
  std::unique_ptr<char[]> buf(new char[size + 1]);
  if (!buf) {
    Serial.Println("ERROR: Failed to allocate memory for config");
    f.close();
    if (!recoveryAttempted && recoverConfigFromBackup("config buffer allocation failed")) {
      recoveryAttempted = true;
      bool ok = cfgLoad();
      recoveryAttempted = false;
      return ok;
    }
    loadDefaultConfig();
    cfgRuntimeDefaultDueLoadFailure = true;
    return false;
  }
  
  f.readBytes(buf.get(), size);
  buf[size] = '\0';
  f.close();

  DynamicJsonDocument doc((size_t)size * 2 + 1024);
  DeserializationError err = deserializeJson(doc, buf.get());
  if (err) {
    Serial.print("ERROR: Parse config error: ");
    Serial.Println(err.c_str());
    if (!recoveryAttempted && recoverConfigFromBackup("active config parse failed")) {
      recoveryAttempted = true;
      bool ok = cfgLoad();
      recoveryAttempted = false;
      return ok;
    }
    loadDefaultConfig();
    cfgRuntimeDefaultDueLoadFailure = true;
    return false;
  }

  // ═══════════════════════════════════════════════════════════════════
  // [FIXED] Load config dari file - TIDAK overwrite dengan DEF_*
  // DEF_* hanya digunakan sebagai fallback jika key tidak ada di file
  // ═══════════════════════════════════════════════════════════════════
  cfg.device_id = getStableDeviceMac();
  // Immutable metadata: always forced to firmware constant
  cfg.ac_brand = AC_BRAND;

  JsonObject k = doc["koneksi"].as<JsonObject>();
  if (!k.isNull()) {
    cfg.koneksi.wifi_ssid = nvs_getWiFiSSID();
    cfg.koneksi.wifi_pass = nvs_getWiFiPass();
    cfg.koneksi.mqtt_host = k["mqtt_host"] | DEF_MQTT_HOST;
    cfg.koneksi.mqtt_port = k["mqtt_port"] | DEF_MQTT_PORT;
    cfg.koneksi.mqtt_user = k["mqtt_user"] | DEF_MQTT_USER;
    cfg.koneksi.mqtt_pass = k["mqtt_pass"] | DEF_MQTT_PASS;
    cfg.koneksi.tz_offset = k["tz_offset"] | DEF_TZ_OFFSET;
  } else {
    cfg.koneksi.wifi_ssid = nvs_getWiFiSSID();
    cfg.koneksi.wifi_pass = nvs_getWiFiPass();
    cfg.koneksi.mqtt_host = DEF_MQTT_HOST;
    cfg.koneksi.mqtt_port = DEF_MQTT_PORT;
    cfg.koneksi.mqtt_user = DEF_MQTT_USER;
    cfg.koneksi.mqtt_pass = DEF_MQTT_PASS;
    cfg.koneksi.tz_offset = DEF_TZ_OFFSET;
  }
  
  // Trim whitespace dari WiFi credentials
  cfg.koneksi.wifi_ssid.trim();
  cfg.koneksi.wifi_pass.trim();

  bool legacyScheduleLoaded = false;
  JsonObject s = doc["setting"].as<JsonObject>();
  if (!s.isNull()) {
    if (s.containsKey("automation")) {
      JsonVariant autoValue = s["automation"];
      if (autoValue.is<bool>()) {
        cfg.setting.automation = autoValue.as<bool>();
      } else if (autoValue.is<const char*>()) {
        const char* strVal = autoValue.as<const char*>();
        cfg.setting.automation = (strcasecmp(strVal, "true") == 0 || strcmp(strVal, "1") == 0);
      } else {
        cfg.setting.automation = autoValue.as<bool>();
      }
    } else {
      cfg.setting.automation = DEF_AUTOMATION;
    }
    
    
    // global_set_temp, global_min/max_temp, global_mode, global_fan,
    // temp_cooldown, auto_recovery — no longer persisted in config.
    // Hardcoded runtime defaults for automation.cpp fallback:
    cfg.setting.global_set_temp = DEF_GLOBAL_SET_TEMP;
    cfg.setting.global_min_temp = DEF_GLOBAL_MIN_TEMP;
    cfg.setting.global_max_temp = DEF_GLOBAL_MAX_TEMP;
    cfg.setting.global_mode = DEF_GLOBAL_MODE;
    cfg.setting.global_fan = DEF_GLOBAL_FAN;
    cfg.setting.temp_cooldown_sec = DEF_TEMP_COOLDOWN;
    cfg.setting.auto_recovery_sec = DEF_AUTO_RECOVERY;

    // Load auto-recovery runtime fields
    cfg.setting.manualModeActive = s["manualModeActive"] | false;
    cfg.setting.lastManualControlMs = s["lastManualControlMs"] | (unsigned long)0;
    cfg.setting.automationPausedByUser = s["automationPausedByUser"] | false;

    // Validate loaded values
    if (cfg.setting.lastManualControlMs == 0) {
      cfg.setting.lastManualControlMs = millis();
    }

    for (int i = 0; i < 7; ++i) {
      initDayOfftime(cfg.setting.daily[i]);
    }

    if (s.containsKey("schedule")) {
      JsonArray schedule = s["schedule"].as<JsonArray>();
      if (!schedule.isNull()) {
        legacyScheduleLoaded = true;
        for (JsonObject dayObj : schedule) {
          if (!dayObj.containsKey("day") || !dayObj["day"].is<int>()) continue;
          int dayNum = dayObj["day"].as<int>();
          int idx = -1;
          if (dayNum >= 0 && dayNum <= 6) {
            // New standard: Minggu=0, Senin=1, ..., Sabtu=6
            idx = DAY_NUM_TO_IDX(dayNum);
          } else if (dayNum >= 1 && dayNum <= 7) {
            // Legacy (old firmware): Senin=1..Minggu=7
            if (dayNum == 7) idx = 0; // Minggu
            else idx = dayNum; // Senin(1) - Sabtu(6)
          } else {
            continue;
          }
          if (idx < 0 || idx > 6) continue;
          const char* type = dayObj["type"] | "offtime";

          if (strcasecmp(type, "offtime") == 0) {
            initDayOfftime(cfg.setting.daily[idx]);
            continue;
          }

          JsonArray data = dayObj["data"].as<JsonArray>();
          if (strcasecmp(type, "fulltime") == 0) {
            initDayFulltime(cfg.setting.daily[idx]);
            if (!data.isNull() && data.size() > 0) {
              parsePeriodFromJson(cfg.setting.daily[idx].periods[0], data[0].as<JsonObject>(), true);
            }
            continue;
          }

          initDayParttime(cfg.setting.daily[idx], "08:00", "17:00");
          cfg.setting.daily[idx].periodCount = 0;
          uint8_t count = 0;
          for (JsonObject pObj : data) {
            if (count >= MAX_PERIODS_PER_DAY) break;
            parsePeriodFromJson(cfg.setting.daily[idx].periods[count], pObj, false);
            count++;
          }
          cfg.setting.daily[idx].periodCount = count;
          if (cfg.setting.daily[idx].periodCount == 0) {
            initDayOfftime(cfg.setting.daily[idx]);
          }
        }
      }
    }
  } else {
    cfg.setting.automation = DEF_AUTOMATION;
    cfg.setting.global_set_temp = DEF_GLOBAL_SET_TEMP;
    cfg.setting.global_min_temp = DEF_GLOBAL_MIN_TEMP;
    cfg.setting.global_max_temp = DEF_GLOBAL_MAX_TEMP;
    cfg.setting.global_mode = DEF_GLOBAL_MODE;
    cfg.setting.global_fan = DEF_GLOBAL_FAN;
    cfg.setting.temp_cooldown_sec = DEF_TEMP_COOLDOWN;
    cfg.setting.auto_recovery_sec = DEF_AUTO_RECOVERY;

    initDefaultSchedule();
    cfg.setting.automationPausedByUser = false;
  }

  if (loadScheduleBinary()) {
    legacyScheduleLoaded = false;
  } else if (legacyScheduleLoaded) {
    Serial.Println("[Config] Migrating legacy JSON schedule to binary storage");
    saveScheduleBinary();
  } else {
    Serial.Println("[Config] Binary schedule missing, using default schedule");
    initDefaultSchedule();
    saveScheduleBinary();
  }

  Serial.Printf("[Config] Loaded - automation: %s\n",
                cfg.setting.automation ? "true" : "false");

  return true;
}

void cfgApply() {
  dataBuildTopics(cfg.device_id);
  dataSetServer(cfg.koneksi.mqtt_host.c_str(), cfg.koneksi.mqtt_port,
                cfg.koneksi.mqtt_user, cfg.koneksi.mqtt_pass);
}

// Helper function to get WiFi status description
static const char* getWiFiStatusText(wl_status_t status) {
  switch (status) {
    case WL_IDLE_STATUS:     return "Idle";
    case WL_NO_SSID_AVAIL:   return "SSID not found";
    case WL_SCAN_COMPLETED:  return "Scan completed";
    case WL_CONNECTED:       return "Connected";
    case WL_CONNECT_FAILED:  return "Connection failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED:    return "Disconnected";
    default:                 return "Unknown";
  }
}

void cfgApplyWiFi() {
  Serial.Println("[Config] Applying WiFi changes");
  cfgApply();
  Serial.Println("[Config] WiFi will reconnect in main loop");
}

void cfgStartWiFiConnect() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    delay(100);
    feedWatchdogAndYield();
  }
  
  WiFi.mode(WIFI_OFF);
  delay(200);
  feedWatchdogAndYield();
  
  cfg.koneksi.wifi_ssid.trim();
  cfg.koneksi.wifi_pass.trim();
  
  Serial.Printf("WiFi: connecting to [%s]...\n", cfg.koneksi.wifi_ssid.c_str());
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  
  WiFi.begin(cfg.koneksi.wifi_ssid.c_str(), cfg.koneksi.wifi_pass.c_str());
  feedWatchdogAndYield();
}

// Blocking version with retry and timeout handling
bool cfgConnectWiFiBlocking() {
  const int MAX_RETRIES = 3;
  const int TIMEOUT_MS = 10000; // 10 seconds timeout per attempt
  
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    // Disconnect first if already connected
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.disconnect();
      delay(1000);
      feedWatchdogAndYield();
    }
    
    // Clear any previous WiFi config
    WiFi.mode(WIFI_OFF);
    delay(500);
    feedWatchdogAndYield();
    
    Serial.Printf("WiFi: Attempt %d/%d - connecting to %s ...\n", 
                  retry + 1, MAX_RETRIES, cfg.koneksi.wifi_ssid.c_str());
    
    // Set WiFi mode and begin connection
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);  // Credentials dikelola sendiri via NVS Preferences — jangan biarkan WiFi library menimpa
    
    // Start connection
    WiFi.begin(cfg.koneksi.wifi_ssid.c_str(), cfg.koneksi.wifi_pass.c_str());
    
    // Wait for connection with timeout
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < TIMEOUT_MS) {
      delay(500);
      Serial.print(".");
      
      feedWatchdogAndYield();
      
      // Check for specific failure states
      if (WiFi.status() == WL_CONNECT_FAILED || 
          WiFi.status() == WL_NO_SSID_AVAIL ||
          WiFi.status() == WL_CONNECTION_LOST) {
        Serial.Printf("\nWiFi: Connection failed - %s\n", getWiFiStatusText(WiFi.status()));
        break;
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.Println();
      Serial.Printf("WiFi: Connected successfully!\n");
      Serial.Printf("WiFi: IP address: %s\n", WiFi.localIP().toString().c_str());
      Serial.Printf("WiFi: Signal strength: %d dBm\n", WiFi.RSSI());
      return true; // Success
    } else {
      Serial.Println();
      Serial.Printf("WiFi: Failed to connect - %s\n", getWiFiStatusText(WiFi.status()));
      
      if (retry < MAX_RETRIES - 1) {
        Serial.Printf("WiFi: Retrying in 2 seconds...\n");
        WiFi.disconnect();
        delay(2000);
        feedWatchdogAndYield();
      }
    }
  }
  
  // All retries failed
  Serial.Printf("WiFi: Failed to connect after %d attempts\n", MAX_RETRIES);
  Serial.Println("WiFi: Please check your WiFi credentials and network availability");
  return false;
}

bool cfgCheckRTC() {
  Wire.beginTransmission(0x51); // PCF8563T I2C address
  return (Wire.endTransmission() == 0);
}

bool cfgInitRTC() {
  #if defined(RTC_SDA_PIN) && defined(RTC_SCL_PIN)
  // I2C bus recovery: toggle SCL 9x untuk release slave yang hold SDA (bus stuck saat boot)
  pinMode(RTC_SCL_PIN, OUTPUT);
  pinMode(RTC_SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(RTC_SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(RTC_SCL_PIN, LOW);  delayMicroseconds(10);
  }
  // STOP condition
  pinMode(RTC_SDA_PIN, OUTPUT);
  digitalWrite(RTC_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(RTC_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(RTC_SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(RTC_SDA_PIN, INPUT);
  pinMode(RTC_SCL_PIN, INPUT);
  
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  #endif
  
  // Beri waktu I2C bus untuk stabil
  delay(200);
  esp_task_wdt_reset();
  yield();
  
  // Cukup probe kehadiran chip - validasi tahun ada di cfgSyncFromRTC()
  unsigned long startTime = millis();
  const unsigned long RTC_TIMEOUT = 5000;
  
  while (millis() - startTime < RTC_TIMEOUT) {
    esp_task_wdt_reset();
    yield();
    
    Wire.beginTransmission(0x51); // PCF8563T address
    if (Wire.endTransmission() == 0) {
      rtc.begin(); // inisialisasi library PCF8563T
      Serial.Println("[RTC] PCF8563T ditemukan di I2C 0x51");
      return true;
    }
    
    delay(200);
    yield();
    esp_task_wdt_reset();
  }
  
  Serial.Println("ERROR: RTC PCF8563T tidak ditemukan (timeout 5s)");
  return false;
}

bool cfgSyncFromRTC() {
  esp_task_wdt_reset();
  yield();
  
  if (!rtc.begin()) {
    Serial.Println("ERROR: Failed to access PCF8563T RTC");
    return false;
  }
  
  DateTime now = rtc.now();
  
  esp_task_wdt_reset();
  yield();
  
  int fullYear = now.year();
  
  if (fullYear < 2020 || fullYear > 2100) {
    Serial.Printf("WARNING: RTC time invalid (year=%d)\n", fullYear);
    return false;
  }
  
  // Set timezone dinamis berdasarkan config (WIB=7, WITA=8, WIT=9)
  String tz = cfgGetTZString(cfg.koneksi.tz_offset);
  setenv("TZ", tz.c_str(), 1);
  tzset();
  
  // Buat struktur waktu dari RTC (anggap RTC sudah menyimpan waktu WIB)
  struct tm t;
  t.tm_year = fullYear - 1900;
  t.tm_mon = now.month() - 1;
  t.tm_mday = now.day();
  t.tm_hour = now.hour();
  t.tm_min = now.minute();
  t.tm_sec = now.second();
  t.tm_isdst = -1;

  esp_task_wdt_reset();
  yield();

  time_t epoch = mktime(&t);
  struct timeval tv = { epoch, 0 };
  settimeofday(&tv, NULL);

  Serial.Printf("[RTC] Time synced from PCF8563T: %04d-%02d-%02d %02d:%02d:%02d WIB\n",
               fullYear, now.month(), now.day(), now.hour(), now.minute(), now.second());

  return true;
}

void cfgSyncTime(unsigned long timeoutMs) {
  bool syncedFromNTP = false;
  time_t now;

  if (WiFi.status() == WL_CONNECTED) {
    Serial.Println("[NTP] Starting time sync...");
    esp_task_wdt_reset();
    yield();

    // Gunakan esp_sntp_* API yang menangani TCPIP locking secara internal
    // (configTime() memanggil raw lwIP tanpa lock - crash pada ESP-IDF v5.x)
    if (esp_sntp_enabled()) {
      esp_sntp_stop();
    }
    // Set timezone dinamis berdasarkan config
    String tz = cfgGetTZString(cfg.koneksi.tz_offset);
    setenv("TZ", tz.c_str(), 1);
    tzset();
    // Konfigurasi dan mulai SNTP
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_init();
    
    unsigned long start = millis();
    now = time(nullptr);
    
    while ((millis() - start) < timeoutMs) {
      delay(250);
      yield();
      esp_task_wdt_reset();

      sntp_sync_status_t syncStatus = sntp_get_sync_status();
      now = time(nullptr);

      if (syncStatus == SNTP_SYNC_STATUS_COMPLETED && now >= 100000) {
        syncedFromNTP = true;
        break;
      }

      if (((millis() - start) % 2000) < 250) {
        Serial.print(".");
        yield();
        esp_task_wdt_reset();
      }
    }

    esp_task_wdt_reset();
    yield();
    
    if (syncedFromNTP) {
      struct tm *timeinfo = localtime(&now);
      
      Serial.Printf("\n[NTP] Got time from server: %04d-%02d-%02d %02d:%02d:%02d WIB\n",
                   timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                   timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
      
      // Update RTC PCF8563T dengan waktu WIB
      rtc.adjust(DateTime(
        timeinfo->tm_year + 1900,
        timeinfo->tm_mon + 1,
        timeinfo->tm_mday,
        timeinfo->tm_hour,
        timeinfo->tm_min,
        timeinfo->tm_sec
      ));
      
      Serial.Printf("[NTP] RTC updated with WIB time: %04d-%02d-%02d %02d:%02d:%02d\n",
                   timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                   timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
                   
      // Verifikasi waktu yang tersimpan di RTC
      delay(100);
      DateTime rtcNow = rtc.now();
      
      Serial.Printf("[NTP] RTC verification: %04d-%02d-%02d %02d:%02d:%02d\n",
                   rtcNow.year(), rtcNow.month(), rtcNow.day(),
                   rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    } else {
      Serial.Println("\n[NTP] Timeout, no confirmed SNTP sync. Fallback to RTC");
    }
  }

  if (!syncedFromNTP) {
    cfgSyncFromRTC();
  }
  
  esp_task_wdt_reset();
  yield();
}

bool cfgFormatLittleFS() {
  Serial.Println("[FORMAT] Formatting LittleFS...");
  Serial.Println("[FORMAT] WARNING: All data will be erased!");
  
  LittleFS.end();
  delay(100);
  
  if (!LittleFS.begin(true)) {  // true = force format
    Serial.Println("[FORMAT] FAILED to format LittleFS!");
    return false;
  }
  
  Serial.Println("[FORMAT] LittleFS successfully formatted");
  Serial.Println("[FORMAT] Creating default config...");
  
  // Load default config
  loadDefaultConfig();
  
  // Save to file
  if (cfgSave()) {
    Serial.Println("[FORMAT] Default config successfully saved");
    return true;
  } else {
    Serial.Println("[FORMAT] FAILED to save default config");
    return false;
  }
}
