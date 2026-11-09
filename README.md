# ESP32 Gree AC Control System (SMD Variant)

Sistem kontrol AC Gree versi SMD menggunakan ESP32-C3 dengan komunikasi IR dan monitoring melalui MQTT over SSL/TLS. Program ini mengontrol AC secara otomatis berdasarkan jadwal dan dapat dikontrol manual melalui MQTT commands.

**Firmware:** `2.4.7` - Schedule binary V2, maksimal 10 periode per hari,
atomic replace/insert, periode overnight, transisi periode kontinu, dan retained
setting per hari.

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

Payload dasar setting hanya berisi konfigurasi umum seperti `automation`.
Jadwal mingguan tidak lagi disisipkan ke topic ini. Jadwal dipublish sebagai
retained topic terpisah per hari agar update satu hari tidak mengirim ulang
hingga 70 periode sekaligus.

**Topic:** `isems/devices/{device_id}/setting`
**Retained:** Ya
**Dikirim:** Saat startup dan setiap perubahan konfigurasi

**Payload:**
```json
{
  "automation": true
}
```

Firmware memublikasikan satu hari per retained topic berikut:

```text
isems/devices/{device_id}/setting/day/0
...
isems/devices/{device_id}/setting/day/6
```

Contoh payload topic `/setting/day/1`:

```json
{
  "day": 1,
  "type": "parttime",
  "data": [
    { "start": "08:00", "end": "16:59", "mode": "COOL", "fan": "MEDIUM", "set_temp": 24 }
  ]
}
```

Nomor hari menggunakan `0=Minggu` sampai `6=Sabtu`. Server menyusun satu
minggu dari ketujuh retained topic. Topic hari yang belum diterima dianggap
belum diketahui dan tidak boleh menghapus hari lain. Firmware memublikasikan
ulang seluruh hari setelah boot dan hanya hari yang berubah setelah command
schedule berhasil.

Field legacy `min_temp`, `max_temp`, serta `temp_cooldown` boleh dikirim oleh
command setting lama untuk kompatibilitas, tetapi tidak diproses sebagai kontrol
suhu dan tidak dipublish kembali.

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

`min_temp` dan `max_temp` pada command lama tetap diterima, tetapi diabaikan.
Automation mengikuti `set_temp`, `mode`, dan `fan` dari periode schedule.

---

### **B. Schedule Command**

**Topic:** `isems/command/{device_id}/schedule`

```json
[
  {
    "day": 1,
    "operation": "replace",
    "type": "parttime",
    "data": [
      { "start": "08:00", "end": "16:59", "mode": "COOL", "fan": "MEDIUM", "set_temp": 24 }
    ]
  }
]
```

`operation` opsional. Jika tidak dikirim, firmware menggunakan `replace`.
Replace hanya menimpa hari yang disebutkan dan menerima maksimal 10 periode
per hari.

Insert menambahkan periode tanpa menghapus periode lama:

```json
[
  {
    "day": 5,
    "operation": "insert",
    "type": "parttime",
    "data": [
      { "start": "23:00", "end": "03:00", "mode": "COOL", "fan": "LOW", "set_temp": 25 }
    ]
  }
]
```

Periode lintas tengah malam dimiliki oleh hari mulai. Contoh di atas tetap
aktif pada Sabtu hingga `03:00` walaupun jadwal Sabtu bertipe `offtime`.
Pengiriman ulang insert yang identik dianggap berhasil tanpa membuat duplikat.

Semua hari dalam satu command diproses secara atomik. Seluruh command ditolak
dan jadwal lama dipertahankan jika ditemukan waktu tidak valid, operasi tidak
dikenal, lebih dari 10 periode, overlap, kekurangan blok heap untuk parsing,
atau kegagalan penyimpanan LittleFS.

Contoh ACK berhasil:

```json
{
  "cmd": "schedule",
  "status": "success",
  "detail": "Inserted schedule day=5 added=1 total=2",
  "ts": 12345
}
```

Contoh ACK gagal:

```json
{
  "cmd": "schedule",
  "status": "failed",
  "detail": "Schedule overlap: day=5 period=1 conflicts_day=6 conflicts_period=0",
  "ts": 12345
}
```

Rentang waktu memakai menit akhir inklusif. Karena itu `00:00-06:59` yang
dilanjutkan `07:00-16:59` tidak mematikan AC di batas periode; automation hanya
menyinkronkan mode, fan, dan set temperature yang berubah. Jika periode pertama
berakhir `06:58`, menit `06:59` tetap menjadi gap OFF normal.

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

Top-level key yang dikenali: `power`, `temp`, `mode`, `fan`, `swing`, `sleep`, `turbo`, `lights`, `health`.

#### Swing Command

Mengontrol swing vertical dan horizontal AC. Mendukung beberapa format:

**Boolean (vertical + horizontal sekaligus):**
```json
{"swing": true}    // vertical auto sweep + horizontal on
{"swing": false}   // vertical off + horizontal off
```

**String (vertical saja):**
```json
{"swing": "auto"}          // vertical auto sweep
{"swing": "on"}            // vertical auto (alias)
{"swing": "off"}           // vertical off + horizontal off
{"swing": "highest"}       // vertical fixed posisi atas
{"swing": "upper_middle"}  // vertical fixed posisi atas-tengah
{"swing": "middle"}        // vertical fixed posisi tengah
{"swing": "lower_middle"}  // vertical fixed posisi bawah-tengah
{"swing": "lowest"}        // vertical fixed posisi bawah
```

**Object (vertical dan horizontal独立):**
```json
{"swing": {"vertical": "auto", "horizontal": true}}
{"swing": {"vertical": "middle", "horizontal": false}}
{"swing": {"vertical": "off"}}                // hanya vertical off
{"swing": {"horizontal": true}}               // hanya horizontal on
{"swing": {"horizontal": false}}              // hanya horizontal off
```

**Dikombinasikan dengan command lain:**
```json
{"power": true, "mode": "COOL", "temp": 24, "swing": "auto"}
{"fan": "HIGH", "swing": {"vertical": "lowest", "horizontal": true}}
```

> **Catatan protokol:** Ketika horizontal swing aktif, AC otomatis mengaktifkan auto-sweep vertical (keterbatasan protokol Kelvinator/Gree). Untuk menggunakan vertical fixed position, horizontal harus dimatikan terlebih dahulu.

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

Update melalui manifest:

```json
{
  "action": "manifest",
  "manifest_url": "https://example.com/manifest.json"
}
```

`action` boleh dihilangkan untuk kompatibilitas payload lama. Manifest wajib
berisi `version`, `url`, dan checksum `md5` 32 karakter.

Update langsung dari binary:

```json
{
  "action": "direct",
  "binary_url": "https://example.com/firmware.bin",
  "md5": "0123456789abcdef0123456789abcdef"
}
```

`download_url` atau `url` juga diterima sebagai pengganti `binary_url`.

Rollback ke slot OTA sebelumnya:

```json
{
  "action": "rollback",
  "confirm": true
}
```

Rollback tidak mengunduh firmware dan hanya dapat memilih slot OTA sebelumnya.
Firmware baru divalidasi setelah task RTOS stabil selama 60 detik dan heap
memenuhi batas aman. MQTT tidak menjadi syarat health validation.

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

