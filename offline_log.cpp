#include "offline_log.h"

#include <LittleFS.h>
#include <string.h>

namespace {

struct OfflineLogHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t recordSize;
  uint32_t capacity;
  uint32_t head;
  uint32_t tail;
  uint32_t count;
  uint32_t sequence;
  uint32_t crc;
} __attribute__((packed));

static const char* LOG_PATH = "/offline_log.bin";
static const uint32_t LOG_MAGIC = 0x474F4C4FUL; // OLOG
static const uint16_t LOG_VERSION = 1;
static const uint8_t RECORD_FLAG_VALID = 0xA5;
static const uint32_t MAX_LOG_RECORDS = 9000UL;
static const size_t MAX_LOG_BYTES = (sizeof(OfflineLogHeader) * 2) + (MAX_LOG_RECORDS * sizeof(OfflineLogRecord));
static const size_t MIN_RESERVED_FS_BYTES = 32UL * 1024UL;

static OfflineLogHeader header = {};
static bool ready = false;
static uint8_t activeHeaderSlot = 0;

uint32_t fnv1a(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t calcHeaderCrc(const OfflineLogHeader& h) {
  OfflineLogHeader copy = h;
  copy.crc = 0;
  return fnv1a(reinterpret_cast<const uint8_t*>(&copy), sizeof(copy));
}

uint32_t computeCapacity() {
  size_t total = LittleFS.totalBytes();
  if (total == 0) return 0;

  size_t used = LittleFS.usedBytes();
  size_t existingLogBytes = 0;
  File existing = LittleFS.open(LOG_PATH, "r");
  if (existing) {
    existingLogBytes = existing.size();
    existing.close();
  }

  size_t usedWithoutLog = (used > existingLogBytes) ? (used - existingLogBytes) : used;
  if (total <= usedWithoutLog + MIN_RESERVED_FS_BYTES) return 0;

  size_t usable = total - usedWithoutLog - MIN_RESERVED_FS_BYTES;
  if (usable > MAX_LOG_BYTES) usable = MAX_LOG_BYTES;

  size_t headerBytes = sizeof(OfflineLogHeader) * 2;
  if (usable <= headerBytes) return 0;
  uint32_t capacity = static_cast<uint32_t>((usable - headerBytes) / sizeof(OfflineLogRecord));
  return capacity > MAX_LOG_RECORDS ? MAX_LOG_RECORDS : capacity;
}

size_t getLogFileBytes() {
  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) return 0;
  size_t bytes = f.size();
  f.close();
  return bytes;
}

bool isHeaderValid(const OfflineLogHeader& h, uint32_t maxCapacity) {
  if (h.magic != LOG_MAGIC) return false;
  if (h.version != LOG_VERSION) return false;
  if (h.recordSize != sizeof(OfflineLogRecord)) return false;
  if (h.capacity == 0 || h.capacity > maxCapacity) return false;
  if (h.head >= h.capacity || h.tail >= h.capacity) return false;
  if (h.count > h.capacity) return false;
  return h.crc == calcHeaderCrc(h);
}

size_t headerOffset(uint8_t slot) {
  return static_cast<size_t>(slot) * sizeof(OfflineLogHeader);
}

size_t recordOffset(uint32_t index) {
  return (sizeof(OfflineLogHeader) * 2) + (static_cast<size_t>(index) * sizeof(OfflineLogRecord));
}

bool readHeader(uint8_t slot, OfflineLogHeader& out, uint32_t maxCapacity) {
  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) return false;
  if (!f.seek(headerOffset(slot), SeekSet)) {
    f.close();
    return false;
  }
  size_t n = f.read(reinterpret_cast<uint8_t*>(&out), sizeof(out));
  f.close();
  return n == sizeof(out) && isHeaderValid(out, maxCapacity);
}

bool writeHeader(uint8_t slot, const OfflineLogHeader& h) {
  File f = LittleFS.open(LOG_PATH, "r+");
  if (!f) return false;
  if (!f.seek(headerOffset(slot), SeekSet)) {
    f.close();
    return false;
  }
  size_t n = f.write(reinterpret_cast<const uint8_t*>(&h), sizeof(h));
  f.flush();
  f.close();
  return n == sizeof(h);
}

bool saveHeader() {
  OfflineLogHeader next = header;
  next.sequence++;
  next.crc = calcHeaderCrc(next);

  uint8_t nextSlot = activeHeaderSlot == 0 ? 1 : 0;
  if (!writeHeader(nextSlot, next)) return false;

  header = next;
  activeHeaderSlot = nextSlot;
  return true;
}

bool writeRecord(uint32_t index, const OfflineLogRecord& record) {
  File f = LittleFS.open(LOG_PATH, "r+");
  if (!f) return false;
  if (!f.seek(recordOffset(index), SeekSet)) {
    f.close();
    return false;
  }
  size_t n = f.write(reinterpret_cast<const uint8_t*>(&record), sizeof(record));
  f.flush();
  f.close();
  return n == sizeof(record);
}

bool readRecord(uint32_t index, OfflineLogRecord& record) {
  File f = LittleFS.open(LOG_PATH, "r");
  if (!f) return false;
  if (!f.seek(recordOffset(index), SeekSet)) {
    f.close();
    return false;
  }
  size_t n = f.read(reinterpret_cast<uint8_t*>(&record), sizeof(record));
  f.close();
  return n == sizeof(record);
}

bool createFreshLog(uint32_t capacity) {
  if (capacity < 16) {
    Serial.println("[OfflineLog] LittleFS capacity too small for offline telemetry");
    return false;
  }

  LittleFS.remove(LOG_PATH);
  yield();

  header = {};
  header.magic = LOG_MAGIC;
  header.version = LOG_VERSION;
  header.recordSize = sizeof(OfflineLogRecord);
  header.capacity = capacity;
  header.sequence = 1;
  header.crc = calcHeaderCrc(header);
  activeHeaderSlot = 0;

  File f = LittleFS.open(LOG_PATH, "w");
  if (!f) {
    Serial.println("[OfflineLog] Failed to create log file");
    return false;
  }
  size_t writtenA = f.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  size_t writtenB = f.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  f.flush();
  f.close();

  if (writtenA != sizeof(header) || writtenB != sizeof(header)) {
    Serial.println("[OfflineLog] Failed to initialize log headers");
    return false;
  }
  return true;
}

}

bool offlineLogInit() {
  ready = false;
  uint32_t maxCapacity = computeCapacity();
  if (maxCapacity < 16) {
    Serial.println("[OfflineLog] Disabled: insufficient LittleFS capacity");
    return false;
  }

  OfflineLogHeader h0 = {};
  OfflineLogHeader h1 = {};
  bool v0 = readHeader(0, h0, maxCapacity);
  bool v1 = readHeader(1, h1, maxCapacity);

  if (v0 || v1) {
    if (v0 && (!v1 || h0.sequence >= h1.sequence)) {
      header = h0;
      activeHeaderSlot = 0;
    } else {
      header = h1;
      activeHeaderSlot = 1;
    }
    ready = true;
    if (header.count == 0 && header.capacity != maxCapacity) {
      Serial.printf("[OfflineLog] Recreate empty log capacity: %lu -> %lu\n",
                    static_cast<unsigned long>(header.capacity),
                    static_cast<unsigned long>(maxCapacity));
      ready = createFreshLog(maxCapacity);
    }
  } else {
    ready = createFreshLog(maxCapacity);
  }

  if (ready) {
    size_t fsTotal = LittleFS.totalBytes();
    size_t fsUsed = LittleFS.usedBytes();
    size_t fsFree = fsTotal > fsUsed ? fsTotal - fsUsed : 0;
    Serial.printf("[OfflineLog] Ready: count=%lu capacity=%lu record=%u file=%u fs_used=%u fs_free=%u fs_total=%u\n",
                  static_cast<unsigned long>(header.count),
                  static_cast<unsigned long>(header.capacity),
                  static_cast<unsigned>(sizeof(OfflineLogRecord)),
                  static_cast<unsigned>(getLogFileBytes()),
                  static_cast<unsigned>(fsUsed),
                  static_cast<unsigned>(fsFree),
                  static_cast<unsigned>(fsTotal));
  }
  return ready;
}

bool offlineLogAppend(uint32_t epoch, bool power, int16_t roomTemp) {
  if (!ready && !offlineLogInit()) return false;

  OfflineLogRecord record = {};
  record.epoch = epoch;
  record.room_temp = roomTemp;
  record.power = power ? 1 : 0;
  record.flags = RECORD_FLAG_VALID;

  uint32_t writeIndex = header.head;
  if (!writeRecord(writeIndex, record)) return false;

  header.head = (header.head + 1) % header.capacity;
  if (header.count >= header.capacity) {
    header.tail = (header.tail + 1) % header.capacity;
    header.count = header.capacity;
  } else {
    header.count++;
  }

  return saveHeader();
}

bool offlineLogPeek(OfflineLogRecord& record) {
  if (!ready || header.count == 0) return false;
  if (!readRecord(header.tail, record)) return false;
  return record.flags == RECORD_FLAG_VALID;
}

bool offlineLogPop() {
  if (!ready || header.count == 0) return false;
  header.tail = (header.tail + 1) % header.capacity;
  header.count--;
  if (header.count == 0) {
    header.head = 0;
    header.tail = 0;
  }
  if (!saveHeader()) return false;

  if (header.count == 0) {
    uint32_t targetCapacity = computeCapacity();
    if (targetCapacity < 16) targetCapacity = header.capacity;
    if (targetCapacity >= 16) {
      Serial.printf("[OfflineLog] Recreate empty log capacity: %lu -> %lu\n",
                    static_cast<unsigned long>(header.capacity),
                    static_cast<unsigned long>(targetCapacity));
      ready = createFreshLog(targetCapacity);
      return ready;
    }
  }

  return true;
}

uint32_t offlineLogCount() {
  return ready ? header.count : 0;
}

uint32_t offlineLogCapacity() {
  return ready ? header.capacity : 0;
}

bool offlineLogReady() {
  return ready;
}

size_t offlineLogFileBytes() {
  return getLogFileBytes();
}

size_t offlineLogFsUsedBytes() {
  return LittleFS.usedBytes();
}

size_t offlineLogFsTotalBytes() {
  return LittleFS.totalBytes();
}

size_t offlineLogFsFreeBytes() {
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  return total > used ? total - used : 0;
}
