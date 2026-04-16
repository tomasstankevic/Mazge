# Mazge – ESP32-CAM Streaming PoC

Live MJPEG camera stream from a **Freenove ESP32-S3 WROOM CAM** board with a browser-based control panel and OTA updates.

## Features

- **MJPEG streaming** via `multipart/x-mixed-replace` on port 81
- **Web control panel** on port 80 — adjust quality, FPS cap, brightness, contrast, saturation, gain ceiling, night mode, greyscale
- **Software 2×2 pixel binning** — captures YUV422 at SVGA (800×600), extracts luminance (Y channel), averages 2×2 blocks → 400×300 greyscale JPEG. Improves low-light sensitivity by ~4× compared to single-pixel readout
- **Direct JPEG mode** — hardware JPEG at VGA (640×480), switchable from the UI
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
   - **Off** — VGA 640×480 direct JPEG (colour)
   - **On** — SVGA 800×600 → 2×2 binned 400×300 greyscale (better in low light)

## Project Structure

```
├── platformio.ini     # Build config: USB (esp32s3cam) and OTA environments
├── src/
│   └── main.cpp       # Complete firmware — camera, WiFi, streaming, web UI, OTA, binning
└── .gitignore
```

## WiFi Configuration

Edit the credentials at the top of `src/main.cpp`:

```cpp
const char *WIFI_SSID = "YourNetwork";
const char *WIFI_PASS = "YourPassword";
```
