# Orange Sensor — ESP8266 Multi-Sensor Dashboard

[![Platform](https://img.shields.io/badge/platform-ESP8266-blue)](https://www.espressif.com/en/products/socs/esp8266)
[![Framework](https://img.shields.io/badge/framework-Arduino-teal)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)

A compact IoT weather/environment station built on the ESP8266, with a local web dashboard, TM1638 LED display, CSV logging, OTA updates, and a remote command terminal.

---

## Table of contents

- [Features](#features)
- [Hardware](#hardware)
- [Project structure](#project-structure)
- [Building](#building)
- [HTTP API](#http-api)
- [Terminal client (`esp-term`)](#terminal-client-esp-term)
- [Screens](#screens)
- [Known limitations](#known-limitations)
- [License](#license)

---

## Features

- **Sensors**
  - AHT20 — indoor temperature and humidity
  - BMP280 — barometric pressure (auto-converted to mmHg) with calibration offset
  - MQ2 — gas/smoke analog + digital output (ppm estimate)
  - MAX30102 — heart rate (BPM) and SpO₂ with finger detection
  - wttr.in — outdoor temperature and pressure for Zelenogradsk (configurable)

- **Display (TM1638)**
  - 6 screens: clock+temp, gas level, temp/humidity, pressure, New Year countdown, APPLE + firmware version
  - Button navigation, adjustable brightness (25/50/100%), per-screen LED indicator

- **Web interface**
  - Live dashboard with animated heart, SpO₂, waveform canvas, and all sensor cards
  - History chart (180 points, ~3 hours, 1-minute sampling) rendered client-side
  - Admin panel: settings, CSV management, firmware version, reboot
  - Web console with a full command set (`help` to list)

- **Data logging**
  - CSV written to LittleFS, one file per day (day boundary at **20:11** local time), 30-day rotation
  - Download current/archived logs from the admin panel

- **Persistence (EEPROM)**
  - Graph data (180 points × 4 parameters)
  - MQ2 threshold, brightness, pressure calibration, firmware version, CSV state

- **OTA**
  - ArduinoOTA with password
  - Firmware version counter (`apple N`) auto-incremented on each OTA update

- **Remote terminal**
  - `exec` client script (bash) for issuing commands over HTTP without a token
  - Endpoints `/exec` and `/api/exec` are open; `/admin/exec` requires auth

---

## Hardware

| Component       | ESP8266 Pin |
|-----------------|-------------|
| SDA             | GPIO4 (D2)  |
| SCL             | GPIO5 (D1)  |
| TM1638 STB      | D5          |
| TM1638 CLK      | D6          |
| TM1638 DIO      | D7          |
| MQ2 analog      | A0          |
| MQ2 digital     | D0          |
| LED             | GPIO2       |
| Buzzer          | GPIO15      |

I2C bus runs at 400 kHz.

---

## Project structure

```
apple-sensor.ino     — main firmware
passwords.h          — Wi-Fi / admin credentials (not committed)
esp-term             — bash terminal client (no token required)
.gitignore           — ignores passwords.h and build artifacts
README.md
LICENSE
```

### `.gitignore` example

```gitignore
passwords.h
*.bin
*.elf
build/
```

### `passwords.h` example

```c
// passwords.h — DO NOT COMMIT
#define CFG_WIFI_SSID  "your-ssid"
#define CFG_WIFI_PASS  "your-password"
#define CFG_ADMIN_USER "admin"
#define CFG_ADMIN_PASS "secret"
```

---

## Building

1. Install the Arduino ESP8266 core and libraries:
   - `Adafruit AHTX0`
   - `Adafruit BMP280 Library`
   - `MAX30105` (SparkFun)
   - `spo2_algorithm` (MAX30102 reference algorithm)
   - `TM1638plus`
2. Create `passwords.h` with your credentials (see above).
3. Select board **Generic ESP8266 Module**, flash size ≥ 1 MB with SPIFFS/LittleFS partition.
4. Upload the sketch.

On first boot the firmware formats LittleFS if mounting fails and initializes EEPROM defaults.

---

## HTTP API

| Endpoint                        | Auth         | Description                         |
|---------------------------------|--------------|-------------------------------------|
| `GET /`                         | optional     | Dashboard                           |
| `GET /login`, `POST /login`     | —            | Login form / submit                 |
| `GET /logout`                   | —            | Clear session                       |
| `GET /admin`                    | cookie/token | Admin panel                         |
| `GET /admin/console`            | cookie/token | Web console                         |
| `GET /admin/exec?cmd=`          | cookie/token | Command execution (authenticated)   |
| `GET /exec?cmd=`                | **none**     | Command execution (open)            |
| `GET /api/exec?cmd=`            | **none**     | Command execution (open)            |
| `GET /api/data`                 | none         | Live sensor JSON                    |
| `GET /api/graph`                | none         | History series JSON                 |
| `GET /api/logs`                 | token        | List CSV files                      |
| `GET /api/log?f=`               | token        | Download CSV                        |
| `GET /api/refresh`              | none         | Force sensor + weather refresh      |
| `GET /api/reboot`               | none         | Save state and restart              |
| `GET /api/save?mq2thr=&tmbr=`   | none         | Save settings                       |
| `GET /api/fwver?n=`             | token        | Set firmware version                |

Authentication uses a 16-hex-char token stored in EEPROM; pass it as `?t=<token>` or a `Cookie: auth=<token>` header. Successful login sets a 24-hour cookie.

> ⚠️ `/exec` and `/api/exec` are intentionally unauthenticated — keep the device on a trusted network. If you need auth, use `/admin/exec?t=<token>` instead.

---

## Terminal client (`esp-term`)

A small bash wrapper around `/exec` for issuing commands from a PC without a browser.

### Requirements

- `bash`
- `curl`
- `python3` (used only for URL-encoding)

### Setup

1. Make the script executable:
   ```bash
   chmod +x esp-term
   ```
2. Open `esp-term` and edit the `IP` variable at the top if your device address differs from `192.168.3.16`:
   ```bash
   IP="192.168.3.16"
   ```
3. Run it:
   ```bash
   ./esp-term
   ```

### Example session

```
esp@8266> status
===== STATUS =====
IP: 192.168.3.16
RSSI: -54 dBm
Uptime: 3621 s
Heap: 31240 B
Version: apple 7
...
esp@8266> screen 3
Экран 3
esp@8266> press
BMP280 (дом):  752.3 mmHg
wttr.in (улица): 754.1 mmHg
esp@8266> exit
```

### Behavior

- Commands are URL-encoded with `python3` and sent to `http://$IP/exec?cmd=...`
- History is persisted to `~/.esp-term-history` (500 entries)
- `exit` or `quit` leaves the shell
- `--noproxy "*"` is used so a system-wide `HTTP_PROXY` doesn't intercept LAN traffic

> ⚠️ `esp-term` talks to `/exec`, which is **unauthenticated by design**. Run it only on a trusted LAN.

### Command overview (subset)

```
status, version [set N], uptime, heap, free, df, flash, cpu, mac, uname
wifi, scan [best|json], ifconfig, i2c scan
gpio <pin> <0|1>, gpio read <pin>, gpio mode <pin> in|out, gpio all
pin <pin> <0|1>, pin g<N> gnd|vcc, adc
aht read, bmp read, mq2, max read, reset
out read, weather [update], press, cal [N]
tm1638 read, tm1638 write <0-7> <0|1>, screen <0-5>, bright <0|2>
tone <freq> <ms>, beep, melody, led <0|1>
logs, csv sample, graph [clear], reboot, clear
```

Full list: send `help`.

---

## Screens

| # | Content                                      |
|---|----------------------------------------------|
| 0 | `HH.MM` + indoor temperature `°C`            |
| 1 | MQ2 ppm + OK/n gas state                     |
| 2 | Temperature `°C` + humidity `%`              |
| 3 | Pressure (mmHg)                              |
| 4 | New Year countdown `DD.HH.MM.SS`             |
| 5 | `APPLE` + firmware version                   |

Button mapping: 0–4 select screens 0–3 and 5; button 6 clears the CSV-dirty alarm; button 7 cycles brightness.

---

## Known limitations

- `/exec` and `/api/exec` are unauthenticated (by design) — trusted LAN only.
- EEPROM graph history holds 180 points (~3 h at 1-min sampling); older data is overwritten.
- CSV day boundary is **20:11 local time** (not midnight) — see `currentLogDate()` in firmware.
- wttr.in is polled every 10 minutes; if it fails, outdoor values stay stale (`NaN` → `--` in UI).
- MAX30102 SpO₂ is a rough estimate (ratio-of-ratios), not medical-grade.
- `esp-term` hardcodes the device IP — edit the `IP` variable at the top.
- Firmware version `apple N` is stored in EEPROM and is incremented on each OTA update; it is not tied to git tags.

---

## License

MIT — do whatever you like, no warranty.
