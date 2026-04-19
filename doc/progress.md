# Mazge — Progress Log

## 2026-04-19: Autonomous Prey Detection Working

### What's Done

- **Burst capture system** — 10-frame ring buffer with ToF trigger, HDR bracketing, blown-out frame filter, 40 burst archive slots
- **Laptop pipeline** — `burst_saver.py` downloads bursts via long-poll + MJPEG, rotates/crops, saves JPEG; `prey_analyzer.py` runs YOLO + Haar + cloud API
- **Autonomous prey API** — ESP32 calls Cloudflare Workers prey API directly via HTTPS when laptop absent
  - Crops 640×480 → 384×384 JPEG on-device (~350 ms)
  - WiFiClientSecure + HTTPClient with `setInsecure()` for TLS
  - Sends 3 best frames per burst, stores results in archive metadata
  - Full API call: ~2.8 s per frame (crop + TLS handshake + POST)
- **Laptop presence detection** — tracks last HTTP contact from `burst_saver`; falls back to on-device API after 30 s silence
- **Web UI** — live MJPEG stream, burst gallery, camera controls, stats (FPS, distance, lux, PSRAM, laptop status)
- **OTA updates** — ArduinoOTA with resource cleanup, multiple helper scripts
- **Secrets management** — WiFi creds and API keys in gitignored `secrets.h`

### Architecture Highlights

- Single-file firmware (`main.cpp`, ~1400 lines)
- Dual HTTP servers (port 80 UI, port 81 stream) in separate RTOS tasks
- HDR 10-step gain/exposure bracketing, ALS-adaptive
- VL6180X ToF for proximity trigger + ambient light sensing
- Greyscale JPEG to save bandwidth/memory

### Performance

| Metric | Value |
|--------|-------|
| Ring buffer FPS | ~10 fps |
| JPEG frame size | 8–15 KB (greyscale VGA) |
| Crop latency | 341–350 ms |
| API call (full) | ~2.8 s (incl. TLS + network) |
| Free PSRAM | ~8.1 MB |
| Burst cooldown | 5 s |
| ToF trigger | < 480 mm |

### Issues Resolved

1. **esp_http_client TLS failure** — `ESP_ERR_HTTP_CONNECT` (0x7002) with Cloudflare Workers. Switched to Arduino `WiFiClientSecure` + `HTTPClient` which works.
2. **httpd stack too small for TLS** — 4 KB default stack caused hang during TLS handshake. Increased UI server to 16 KB.
3. **Double crop bug** — `apitest_handler` was cropping before passing to `callPreyApi`, which cropped again. Fixed by letting `callPreyApi` handle cropping internally.
4. **WiFi streaming stalls** — disabled power-save mode, enabled TCP_NODELAY, increased send buffer.
5. **OTA reliability** — needs different `-P` port each time, sometimes requires multiple retries.
6. **fmt2rgb888 vs jpg2rgb888** — `jpg2rgb888` not available in this SDK; using `fmt2rgb888(src, len, PIXFORMAT_JPEG, buf)` instead.

### What's Next

- [ ] Collect training data — run burst_saver overnight to accumulate 40+ bursts
- [ ] Evaluate autonomous API accuracy vs laptop YOLO pipeline
- [ ] Add prey detection result to web UI burst gallery
- [ ] Consider storing API results to flash/SPIFFS for persistence across reboots
- [ ] Optimize TLS — investigate session resumption or keep-alive to reduce per-call overhead
- [ ] Add LED/buzzer alert on prey detection
