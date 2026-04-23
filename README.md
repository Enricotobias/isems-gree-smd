# ESP32 Gree AC Control System (SMD Variant)

Sistem kontrol AC Gree versi SMD menggunakan ESP32-C3 dengan komunikasi IR dan monitoring melalui MQTT over SSL/TLS. Program ini dapat mengontrol AC secara otomatis berdasarkan jadwal dan suhu, serta dapat dikontrol manual melalui MQTT commands.

> **Perbedaan dari varian standar (`gree/`):**
> - Berjalan di ESP32-C3 (SMD package, tanpa pin tambahan)
> - Mendukung **manual override detection** dan **auto recovery** tracking di activity log
> - Menambahkan field `error_source: "INFERRED"` pada data payload saat error disimpulkan dari kegagalan komunikasi

## 🏗️ Alur Konsep & Cara Kerja Program

### 1. **Inisialisasi Sistem (Setup)**

```
┌─────────────────┐
│   ESP32 Start   │
└─────────┬───────┘
          │
    ┌─────▼─────┐
    │ Load Config │ ← config.json dari LittleFS
    └─────┬─────┘
          │
    ┌─────▼─────┐
    │ Init RTC  │ ← DS3231 Real Time Clock
    └─────┬─────┘
          │
    ┌─────▼─────┐
    │ Init WiFi │ ← Koneksi ke router
    └─────┬─────┘
          │
    ┌─────▼─────┐
    │ Init MQTT │ ← Koneksi ke broker (SSL/TLS port 8883)
    └─────┬─────┘
          │
    ┌─────▼─────┐
    │ Init Sensor│ ← Serial komunikasi dengan AC
    └─────┬─────┘
          │
    ┌─────▼─────┐
    │ Main Loop │
    └───────────┘
```

### 2. **Main Loop Execution**

Program berjalan dalam loop utama dengan interval 100ms:

```
Main Loop (100ms) ──┐
                    │
    ┌───────────────▼───────────────┐
    │         Connection Check      │
    │  • WiFi Status Monitor        │
    │  • MQTT Connection Manager    │
    │  • Auto Reconnect (backoff)   │
    └───────────────┬───────────────┘
                    │
    ┌───────────────▼───────────────┐
    │         Sensor Polling        │
    │  • Read AC mainboard frame    │
    │  • Parse display & RTC data   │
    │  • Immediate error detection  │ ← deteksi error_code tanpa tunggu publish
    └───────────────┬───────────────┘
                    │
    ┌───────────────▼───────────────┐
    │      MQTT Data Publishing     │
    │  • Publish AC data (1 menit)  │
    │  • Heartbeat last_alive (2 m) │
    │  • Activity on state change   │
    └───────────────┬───────────────┘
                    │
    ┌───────────────▼───────────────┐
    │      Automation Check         │
    │  • Schedule validation        │
    │  • Temperature control        │
    │  • Auto ON/OFF                │
    └───────────────┬───────────────┘
                    │
    ┌───────────────▼───────────────┐
    │       LED Status Update       │
    │  • Red: WiFi disconnected     │
    │  • Blue: MQTT disconnected    │
    │  • Green: All connected       │
    └───────────────┬───────────────┘
                    │
                 Loop ◄──────────────┘
```

## 📡 MQTT Topics

### **Topic Structure**

| Prefix | Pola | Arah |
|--------|------|------|
| Publish | `isems/devices/{device_id}/...` | ESP32 → Broker |
| Subscribe | `isems/command/{device_id}/...` | Broker → ESP32 |

---

## 📤 Publish Topics (ESP32 → Broker)

### **1. Status Topic**

**Topic:** `isems/devices/{device_id}/status`
**Retained:** Ya

Dikirim saat connect (online) dan disconnect (offline/LWT).

**Payload online:**
```json
{
  "status": "online",
  "disconnect_reason": "MQTT_LOST",
  "last_disconnect": "2026-03-14 08:45:10 WIB",
  "offline_duration": "0h 2m 15s",
  "mqtt_log_enabled": true
}
```

**Payload offline (LWT / graceful):**
```json
{
  "status": "offline",
  "reason": "CONNECTION_LOST",
  "mqtt_log_enabled": true
}
```

**Nilai `reason` / `disconnect_reason`:**
- `CONNECTION_LOST` — koneksi terputus tak terduga
- `WIFI_LOST` — WiFi terputus
- `MQTT_LOST` — MQTT broker tidak dapat dijangkau
- `GRACEFUL` — shutdown/reboot normal
- `UNKNOWN` — alasan tidak diketahui

---

### **2. Data Topic**

**Topic:** `isems/devices/{device_id}/data`
**Retained:** Ya
**Interval:** Setiap ~1 menit

**Payload (normal):**
```json
{
  "read_mainboard": true,
  "read_display": true,
  "read_rtc": true,
  "mac": "AA:BB:CC:DD:EE:FF",
  "power": true,
  "wifi_ssid": "MyWiFi",
  "wifi_rssi": -45,
  "mode": "COOL",
  "fan_speed": "AUTO",
  "set_temp": 24,
  "room_temp": 25,
  "frame_hex": "A55A0C00...",
  "error_hex": "0000",
  "error_code": "-",
  "device_time": "2026-03-14 09:00:00"
}
```

**Payload saat komunikasi AC gagal + error disimpulkan:**
```json
{
  "read_mainboard": false,
  "error_code": "E5",
  "error_source": "INFERRED",
  ...
}
```

> `error_source: "INFERRED"` muncul **hanya** saat `read_mainboard = false` dan sistem menyimpulkan error dari kegagalan komunikasi. Tidak ada field ini saat error terbaca langsung dari AC.

**Deskripsi field:**

| Field | Tipe | Keterangan |
|-------|------|-----------|
| `read_mainboard` | bool | Frame mainboard AC terbaca |
| `read_display` | bool | Frame display AC terbaca |
| `read_rtc` | bool | Data RTC terbaca |
| `mac` | string | MAC address ESP32 |
| `power` | bool | Status AC ON/OFF |
| `wifi_ssid` | string | Nama WiFi yang terhubung |
| `wifi_rssi` | int | Kekuatan sinyal WiFi (dBm) |
| `mode` | string | Mode AC: `COOL`, `HEAT`, `FAN`, `DRY`, `AUTO` |
| `fan_speed` | string | Kecepatan kipas: `AUTO`, `LOW`, `MED`, `HIGH` |
| `set_temp` | int | Suhu yang di-set (°C) |
| `room_temp` | int | Suhu ruangan dari sensor AC (°C) |
| `frame_hex` | string | Raw hex frame dari mainboard AC |
| `error_hex` | string | Raw hex error dari AC |
| `error_code` | string | Kode error terformat (`"-"` jika normal) |
| `error_source` | string | `"INFERRED"` bila error disimpulkan, tidak ada bila terbaca langsung |
| `device_time` | string | Waktu dari RTC (format `YYYY-MM-DD HH:MM:SS`) |

---

### **3. Setting Topic**

**Topic:** `isems/devices/{device_id}/setting`
**Retained:** Ya
**Dikirim:** Saat startup dan setiap perubahan konfigurasi

**Payload:**
```json
{
  "automation": true,
  "max_temp": 28,
  "min_temp": 18,
  "set_temp": 24,
  "set_mode": "COOL",
  "default_fan": "AUTO",
  "temp_cooldown": 300,
  "system_mode": "auto",
  "pause_reason": "",
  "daily": [
    {
      "day": "senin",
      "type": "part_time",
      "waktu": { "mulai": "08:00", "selesai": "17:00" }
    },
    { "day": "selasa", "type": "full_time" },
    { "day": "sabtu", "type": "libur" }
  ]
}
```

| Field | Tipe | Keterangan |
|-------|------|-----------|
| `automation` | bool | Aktifkan/nonaktifkan automation |
| `max_temp` | int | Suhu maksimal sebelum AC dinyalakan (°C) |
| `min_temp` | int | Suhu minimal sebelum AC dimatikan (°C) |
| `set_temp` | int | Target suhu saat AC ON (°C) |
| `set_mode` | string | Mode AC default saat automation menyalakan AC |
| `default_fan` | string | Kecepatan kipas default (`AUTO`/`LOW`/`MED`/`HIGH`) |
| `temp_cooldown` | int | Cooldown (detik) setelah AC dimatikan/dinyalakan |
| `system_mode` | string | Mode sistem: `auto`, `pause`, `manual` |
| `pause_reason` | string | Alasan pause bila `system_mode = "pause"` |
| `daily` | array | Jadwal harian 7 hari |
| `daily[].type` | string | `full_time`, `part_time`, atau `libur` |
| `daily[].waktu` | object | `mulai` & `selesai` (hanya untuk `part_time`) |

---

### **4. Activity Topic**

**Topic:** `isems/devices/{device_id}/activity`
**Retained:** Tidak
**Dikirim:** Segera saat terjadi perubahan state atau deteksi error

**Payload:**
```json
{
  "timestamp": "2026-03-14 09:15:30 WIB",
  "type": "error_change",
  "source": "system",
  "changes": {
    "error": { "old": "-", "new": "E1" }
  },
  "description": "Error code changed: - → E1 (via system)"
}
```

**Nilai `type` (termasuk yang khusus SMD):**

| Nilai | Pemicu |
|-------|--------|
| `power_on` | AC dinyalakan |
| `power_off` | AC dimatikan |
| `mode_change` | Mode AC berubah |
| `fan_change` | Kecepatan kipas berubah |
| `temp_change` | Set temperature berubah |
| `error_change` | Kode error AC berubah |
| `manual_override` | ⭐ Automation dinonaktifkan karena command manual remote |
| `auto_recovery` | ⭐ Automation diaktifkan kembali setelah periode manual |

**Nilai `source`:**

| Nilai | Keterangan |
|-------|-----------|
| `automation` | Dijalankan oleh sistem automasi |
| `mqtt_remote` | Diperintahkan via MQTT command `/remote` |
| `physical_remote` | Terdeteksi dari remote fisik |
| `system` | Internal (error detection, startup) |

**Catatan `timestamp`:**
- Format: `"YYYY-MM-DD HH:MM:SS WIB"` setelah NTP sync
- Fallback sebelum NTP: `"UPTIME-Xs"` (detik sejak boot)

---

### **5. Info Topic**

**Topic:** `isems/devices/{device_id}/info`
**Retained:** Ya

```json
{
  "fw_version": "1.0.0",
  "ac_brand": "gree",
  "ip": "192.168.1.150"
}
```

---

### **6. ACK Topic**

**Topic:** `isems/devices/{device_id}/ack`
**Retained:** Tidak

```json
{
  "cmd": "remote",
  "status": "success",
  "detail": "IR sent",
  "ts": 12345
}
```

---

### **7. Logs Topic**

**Topic:** `isems/devices/{device_id}/logs`

Setiap baris log Serial dikirim ke topic ini sebagai plain string bila `mqtt_log_enabled = true`.

---

## 📥 Command Topics (Broker → ESP32)

### **A. Setting Command**

**Topic:** `isems/command/{device_id}/setting`

```json
{
  "automation": true,
  "max_temp": 28,
  "set_temp": 24,
  "set_mode": "COOL",
  "default_fan": "AUTO",
  "system_mode": "auto"
}
```

Semua field opsional.

---

### **B. Schedule Command**

**Topic:** `isems/command/{device_id}/schedule`

```json
{
  "daily": [
    { "day": "senin", "type": "part_time", "waktu": { "mulai": "08:00", "selesai": "17:00" } },
    { "day": "sabtu", "type": "libur" }
  ]
}
```

---

### **C. Remote Control Command**

**Topic:** `isems/command/{device_id}/remote`

```json
{
  "power": true,
  "temp": 24,
  "mode": "COOL",
  "fan": "AUTO"
}
```

Mengirim command ini akan memicu `manual_override` di activity log.

---

### **D. Connection Command**

**Topic:** `isems/command/{device_id}/koneksi`

```json
{
  "wifi_ssid": "NewWiFi",
  "wifi_pass": "newpassword123",
  "mqtt_host": "mqtt.example.com",
  "mqtt_port": 8883,
  "mqtt_user": "mqttuser",
  "mqtt_pass": "mqttpass"
}
```

---

### **E. Reboot Command**

**Topic:** `isems/command/{device_id}/reboot`

Payload: `"reboot"` atau `{}`. Memicu `ESP.restart()`.

---

### **F. Diagnostics Command**

**Topic:** `isems/command/{device_id}/diagnostics`

Payload: `"run"`. Mempublikasikan ulang info sistem ke topic `/info`.

---

### **G. Info Command**

**Topic:** `isems/command/{device_id}/info`

Payload: `"get"`. Meminta device mempublikasikan payload info.

---

### **H. OTA Update Command**

**Topic:** `isems/command/{device_id}/update`

Memicu OTA firmware update (lihat ota.cpp).

---

## 🔧 Troubleshooting

### **MQTT Connection Errors**

| rc | Arti | Solusi |
|----|------|--------|
| `-2` | TCP/TLS gagal | Cek host, port (8883), dan sertifikat CA |
| `1` | Protocol version salah | Pastikan MQTT 3.1.1 |
| `4` | Bad credentials | Cek username/password MQTT |
| `5` | Not authorized | Cek ACL di broker |

### **SSL Heap Exhaustion**

Jika koneksi SSL selesai dalam `< 5ms` (instant fail), device akan log free heap dan otomatis restart setelah 3 detik.

### **Log Monitoring**

Serial Monitor (115200 baud):

```
[WiFi] Connected to: MyWiFi, IP: 192.168.1.150
[MQTT] ✓ Connected!
[Data] ✓ Suhu/status published
[Activity] ✓ Published: Error code changed: - → E1 (via system)
[Activity] ✓ Published: manual_override (via mqtt_remote)
```

---

_Program ini dirancang untuk integrasi dengan Home Assistant atau dashboard MQTT lainnya untuk monitoring dan kontrol AC Gree secara remote._

