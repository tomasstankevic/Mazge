# Mazge — Architecture

Cat flap prey detection system built on an **ESP32-S3 WROOM** (Freenove board) with an OV2640 camera and VL6180X time-of-flight sensor. When a cat approaches, the system captures a burst of images and either streams them to a laptop for analysis or autonomously calls a cloud prey-detection API.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3 WROOM, 8 MB PSRAM (OPI), WiFi |
| Camera | OV2640, 640×480 JPEG, greyscale (special_effect=2) |
| ToF sensor | VL6180X (TOF050C module), I²C on SDA=47 SCL=21, range + ambient light |
| PIR sensor | GPIO 14, motion detection (supplementary trigger) |

### Pin Map

Camera uses Freenove ESP32-S3 CAM layout (XCLK=15, SIOD=4, SIOC=5, Y2–Y9, VSYNC=6, HREF=7, PCLK=13). Full mapping in `src/main.cpp`.

## Firmware Overview (`src/main.cpp`)

The firmware runs on Arduino framework via PlatformIO. All logic is in a single file (~1400 lines).

### Boot Sequence

1. **I²C init** → VL6180X ToF sensor init (mandatory register sequence per AN4545)
2. **Camera init** → OV2640 at VGA, JPEG mode, manual exposure/gain control, greyscale
3. **WiFi connect** → joins configured SSID, disables power-save, sets TX power to 19.5 dBm
4. **HTTP servers start** → UI server on port 80, stream server on port 81
5. **OTA init** → ArduinoOTA for wireless firmware updates

### Main Loop

Runs at ~10 fps. Each iteration:

1. **OTA handle** — polled every 500 ms (throttled to reduce mDNS overhead)
2. **PIR read** — motion flag updated
3. **ToF read** — continuous ranging, 3-second watchdog with auto-reinit
4. **ALS read** — ambient light every 500 ms (used for adaptive HDR)
5. **Ring buffer capture** — continuously fills a 10-frame JPEG ring buffer with HDR-bracketed exposures
6. **Burst trigger** — when ToF < 480 mm and cooldown expired (5 s), starts post-trigger countdown (2 more frames), then freezes ring to archive
7. **Autonomous API** — if laptop absent (no HTTP contact for 30 s), sends best 3 frames to prey API

### HDR Gain+Exposure Bracketing

10 steps cycling through `(gain, aec)` pairs, from low-gain/short-exposure to high-gain for IR night. ALS-adaptive: caps gain in bright conditions, caps exposure in dark.

| Step | Gain | AEC | Purpose |
|------|------|-----|---------|
| 0–1  | 0    | 100/300 | Very low gain, short+long exposure |
| 2–3  | 2    | 100/300 | Low gain bracket |
| 4–5  | 6    | 100/300 | Medium gain bracket |
| 6–7  | 12   | 100/300 | Higher gain bracket |
| 8–9  | 20/30| 100     | High gain, short exposure only (night) |

### Burst Capture System

- **Ring buffer**: 10 JPEG frames in PSRAM, ~100 ms apart, continuously overwritten
- **Trigger**: VL6180X distance < 480 mm
- **Post-trigger**: captures 2 more frames after trigger, then freezes ring to archive
- **Blown-out filter**: frames with very small JPEG size (<8 KB) or high byte entropy are skipped
- **Archive rotation**: up to 40 burst archives in PSRAM; oldest evicted when full
- **Cooldown**: 5 seconds between bursts

### Autonomous Prey Detection

When the laptop hasn't contacted the ESP32 for 30 seconds (`LAPTOP_TIMEOUT_MS`), the board calls the prey API directly after each burst:

1. **Frame selection** — picks 3 largest JPEGs from the burst (most detail)
2. **Crop** — decodes 640×480 JPEG → RGB888, crops to 384×384 (removes right 128 px occluded area + margins), re-encodes to JPEG
3. **Base64 + JSON** — wraps cropped JPEG in `{"image_base64":"..."}`
4. **HTTPS POST** — `WiFiClientSecure` with `setInsecure()` (no cert pinning) to Cloudflare Workers endpoint
5. **Result** — stores `detected: true/false` per frame in `BurstArchive.apiResults[]`

Crop coordinates: `x=64..448, y=48..432` (8-aligned for JPEG MCU). Pipeline: ~350 ms crop + ~2.5 s API call.

### HTTP Endpoints

**Port 80 (UI server, 16 KB stack):**

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web UI with live stream, burst gallery, camera controls |
| `/stats` | GET | JSON telemetry (FPS, distance, lux, burst count, PSRAM, laptop presence) |
| `/cmd` | GET | Camera controls (quality, brightness, contrast, gain, exposure, reboot) |
| `/burst?a=N&i=M` | GET | Single JPEG frame from burst archive N, image M |
| `/burststream?a=N` | GET | MJPEG stream of all frames in burst archive N |
| `/burstmeta?a=N` | GET | JSON metadata for burst archive (timing, API results) |
| `/apitest` | GET | Capture frame → crop → call prey API → return JSON result |

**Port 81 (stream server, 8 KB stack):**

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/stream` | GET | Live MJPEG stream (capped at `targetFps`) |
| `/burst_wait?gen=N` | GET | Long-poll: blocks until burst generation > N (used by burst_saver) |

### OTA Updates

ArduinoOTA with resource cleanup on start: stops stream server, deinits camera to free DMA/PSRAM. UI server stays alive for emergency `/cmd?reboot=1`.

Build: `~/.platformio/penv/bin/pio run -e ota`
Flash: `espota.py -i <IP> -p 3232 -f .pio/build/ota/firmware.bin`

## Laptop Tools (`tools/`)

Python scripts that run on the laptop to download and analyze burst captures.

### `burst_saver.py`

Connects to `/burst_wait` (long-poll) and `/burststream` (MJPEG). On each new burst:
- Downloads all frames via MJPEG stream
- Rotates 90° (camera is mounted sideways)
- Crops 20% from top (occluded area)
- Saves as high-quality JPEG (q=97) to `captures/burst_YYYYMMDD_HHMMSS_genN/`

Updates `lastLaptopContactMs` on the ESP32 so the board knows the laptop is present.

### `prey_analyzer.py`

Watches the captures directory. For each new burst:
- Runs YOLO object detection (yolo11n, NCNN backend) 
- Runs Haar cascade face detection
- Calls the cloud prey-detection API
- Annotates images with bounding boxes
- Writes results to `prey_analysis.json`

API key loaded from `API_key_prey_detector.txt` or `PREY_DETECTOR_API_KEY` env var.

### Other Tools

| Tool | Purpose |
|------|---------|
| `analyze_image.py` | Single-image analysis (YOLO + Haar) |
| `grab_burst.py` | One-shot burst download |
| `ota_upload.py` / `ota_retry.sh` | Automated OTA with retry |
| `generate_report.py` / `combined_report.py` / `face_report.py` | Analysis reports |
| `frame_stats.py` / `haar_sizes.py` | Image statistics |

## Secrets Management

Credentials are stored in `src/secrets.h` (gitignored). Copy `src/secrets.h.example` and fill in:
- `WIFI_SSID` / `WIFI_PASS` — WiFi network
- `PREY_API_URL` — prey detection API endpoint
- `PREY_API_KEY` — Bearer token for the API

The laptop tools read the API key from `API_key_prey_detector.txt` (also gitignored) or the `PREY_DETECTOR_API_KEY` environment variable.

## Build & Deploy

```bash
# Build firmware
~/.platformio/penv/bin/pio run -e ota

# OTA flash (ESP32 must be on the same network)
python3 ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
  -i 192.168.0.41 -p 3232 -f .pio/build/ota/firmware.bin -t 60

# Run burst saver (laptop)
.venv/bin/python tools/burst_saver.py --ip 192.168.0.41 --dir captures

# Run prey analyzer (laptop)
.venv/bin/python tools/prey_analyzer.py --watch captures
```

## Memory Layout

- **PSRAM (~8 MB)**: Ring buffer (10 × ~10 KB JPEG), burst archives (40 × 10 frames), crop working buffers (~1.8 MB transient), base64/JSON buffers
- **Internal SRAM**: Stack, globals, WiFi/TLS buffers
- **Free PSRAM**: typically ~8.1 MB with burst archives populated

## Key Design Decisions

1. **Greyscale JPEG** — saves ~40% bandwidth and PSRAM vs. color; sufficient for prey detection
2. **Manual HDR bracketing** — auto-exposure disabled; 10-step gain/exposure cycle ensures usable frames in all lighting
3. **Ring buffer** — captures the past, not the future; trigger freezes what's already in memory
4. **Dual HTTP servers** — stream on port 81 (high priority RTOS task) doesn't block UI on port 80
5. **WiFi power-save off** — eliminates 100–300 ms TCP stalls from DTIM sleep
6. **TCP_NODELAY on streams** — prevents Nagle buffering of small MJPEG chunks
7. **Blown-out frame filter** — HDR bracketing produces some overexposed frames; filtered before archiving
8. **Laptop presence detection** — seamless fallback to on-device API calls when laptop unavailable
