# Mazge – ESP32-CAM Streaming PoC

Live MJPEG camera stream from a **Freenove ESP32-S3 WROOM CAM** board with a browser-based control panel, sensors, and OTA updates.

## Features

- **MJPEG streaming** via `multipart/x-mixed-replace` on port 81, toggleable from the web UI
- **Web control panel** on port 80 — adjust quality, FPS cap, brightness, contrast, saturation, gain ceiling, night mode, greyscale
- **Software 2×2 pixel binning** — captures YUV422 at SVGA (800×600), extracts luminance (Y channel), averages 2×2 blocks → 400×300 greyscale JPEG. Improves low-light sensitivity by ~4× compared to single-pixel readout
- **Direct JPEG mode** — hardware JPEG at SVGA (800×600), default mode
- **PIR motion sensor** on GPIO 14 — real-time motion indicator on the web page
- **VL6180X ToF distance sensor** on I2C (SDA=GPIO 47, SCL=GPIO 21) — live distance readout in mm on the web page
- **Burst capture** — automatically takes 5 VGA JPEG photos when ToF distance < 220mm, stored in PSRAM, viewable in a gallery on the web page (5s cooldown between triggers)
- **ArduinoOTA** — wireless firmware updates (throttled to reduce WiFi contention)
- **Performance tuned** — WiFi power save disabled, TCP_NODELAY, 16 KB send buffer, FPS capping, esp_http_server RTOS tasks

## Hardware

| Component | Detail |
|-----------|--------|
| Board | Freenove ESP32-S3 WROOM CAM |
| SoC | ESP32-S3 (QFN56 rev v0.2) |
| RAM | 320 KB internal + 8 MB PSRAM (OPI) |
| Flash | 8 MB (QIO) |
| Camera | OV2640 |
| USB | Two USB-C ports — OTG (left), TTL/UART (right) |
| PIR sensor | Any 3.3V digital PIR module (e.g. HC-SR501 with 3.3V mod, AM312) → GPIO 14 |
| ToF sensor | VL6180X (I2C addr 0x29) → SDA=GPIO 47, SCL=GPIO 21 |

### Wiring

| Sensor | Pin | ESP32 GPIO | Notes |
|--------|-----|------------|-------|
| PIR | OUT | 14 | Digital HIGH = motion. Long wires pick up WiFi EMI — keep short or add 100nF cap on signal + 10µF on VCC at sensor end |
| PIR | VCC | 3.3V | |
| PIR | GND | GND | |
| VL6180X | SDA | 47 | I2C, internal pull-ups used |
| VL6180X | SCL | 21 | I2C, internal pull-ups used |
| VL6180X | VIN | 3.3V | |
| VL6180X | GND | GND | |

### PIR noise troubleshooting

The ESP32 WiFi radio can cause false PIR triggers, especially with long wires:
- Add a **10µF electrolytic + 100nF ceramic** cap between PIR VCC and GND at the sensor end
- Add a **100nF ceramic** cap between PIR signal and GND at the ESP32 end
- Keep wires short and away from the antenna area
- Turn down the sensitivity potentiometer on the PIR module
- Software debounce can be added as a further mitigation

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

### USB flash (first time or if OTA fails)

1. Connect the **right** USB-C port (TTL/UART)
2. Hold **BOOT** button during upload
3. Run:
   ```
   pio run -e esp32s3cam -t upload
   ```
4. Press **RST** after flash completes
5. For serial monitor: `pio device monitor --rts 0 --dtr 0`

### OTA flash (over WiFi)

```
pio run -e ota -t upload
```

> OTA may fail if a browser is actively streaming — close the stream tab first.

## Usage

1. After boot, the board connects to WiFi and prints its IP to serial
2. Open `http://<board-ip>` in a browser
3. The live stream starts automatically; controls are below the image
4. Toggle **Binning** to switch between:
   - **Off** — SVGA 800×600 direct JPEG (colour, default)
   - **On** — SVGA 800×600 → 2×2 binned 400×300 greyscale (better in low light)
5. Use **Stop Stream / Start Stream** to toggle the video feed
6. When stream is off, distance and motion update at 5Hz (200ms) instead of 1Hz
7. **Burst capture** triggers automatically when an object is < 220mm from the ToF sensor — 5 VGA images appear in the gallery below the controls (click to view full-size)

## Web UI Endpoints

| Path | Description |
|------|-------------|
| `/` | Main page with stream, controls, sensor readouts, burst gallery |
| `/stream` (port 81) | Raw MJPEG stream |
| `/stats` | JSON telemetry: FPS, frame size, motion, distance, burst count |
| `/cmd?key=val` | Camera/stream settings (quality, fps, brightness, contrast, etc.) |
| `/burst?i=N` | Serve burst image N (0–4) as JPEG |

## Project Structure

```
├── platformio.ini     # Build config: USB (esp32s3cam) and OTA environments
├── src/
│   └── main.cpp       # Complete firmware — camera, WiFi, streaming, web UI, OTA, sensors, burst capture
├── doc/
│   └── pinout.md      # Full GPIO pinout and wiring reference
└── .gitignore
```

## WiFi Configuration

Edit the credentials at the top of `src/main.cpp`:

```cpp
const char *WIFI_SSID = "YourNetwork";
const char *WIFI_PASS = "YourPassword";
```
