# Mazge – ESP32-CAM Streaming PoC

Live MJPEG camera stream from a **Freenove ESP32-S3 WROOM CAM** board with a browser-based control panel, sensors, and OTA updates.

## Features

- **MJPEG streaming** via `multipart/x-mixed-replace` on port 81, toggleable from the web UI
- **Web control panel** on port 80 — adjust quality, FPS cap, brightness, contrast, saturation, gain ceiling, night mode, greyscale
- **Software 2×2 pixel binning** — captures YUV422 at SVGA (800×600), extracts luminance (Y channel), averages 2×2 blocks → 400×300 greyscale JPEG. Improves low-light sensitivity by ~4× compared to single-pixel readout
- **Direct JPEG mode** — hardware JPEG at SVGA (800×600), default mode
- **VL6180X ToF distance sensor** on I2C (SDA=GPIO 47, SCL=GPIO 21) — live distance readout in mm, triggers burst capture
- **Burst capture** — ring buffer of 10 HDR-bracketed frames. Triggered by ToF distance < 500mm (configurable). 8 pre-trigger + 2 post-trigger frames stored in PSRAM, downloadable via HTTP. 15s cooldown between triggers
- **Camera AEC probe** — reads OV2640 auto-exposure registers every 2s when idle, used as base for HDR bracket scaling
- **Persistent event log** — up to 50 events stored in NVS flash, survive reboots. Tracks burst generation, frame count, API result, distance range
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
| ToF sensor | VL6180X (I2C addr 0x29) → SDA=GPIO 47, SCL=GPIO 21 — burst trigger + distance tracking |

### Wiring

| Sensor | Pin | ESP32 GPIO | Notes |
|--------|-----|------------|-------|
| VL6180X | SDA | 47 | I2C bit-bang, internal pull-ups |
| VL6180X | SCL | 21 | I2C bit-bang, internal pull-ups |
| VL6180X | VIN | 3.3V | |
| VL6180X | GND | GND | |

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
7. **Burst capture** triggers automatically when ToF distance < 500mm — 10 HDR-bracketed frames (8 pre-trigger + 2 post-trigger) appear in the gallery

## Web UI Endpoints

| Path | Description |
|------|-------------|
| `/` | Main page with stream, controls, sensor readouts, burst gallery |
| `/stream` (port 81) | Raw MJPEG stream |
| `/stats` | JSON telemetry: FPS, frame size, distance, AEC, burst count |
| `/cmd?key=val` | Camera/stream settings (quality, fps, brightness, contrast, etc.) |
| `/burst?a=N&i=M` | Serve frame M from burst archive N as JPEG |
| `/burstmeta?a=N` | JSON per-archive metadata (timing, distances, API results) |
| `/burststream?a=N` | MJPEG stream of all frames in archive N |
| `/burst_wait?gen=N` | Long-poll — blocks until burst generation > N |
| `/getevents` | JSON array of persistent event log entries |

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
