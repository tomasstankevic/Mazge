# Frame Processing Pipeline

How captured frames are cropped, rotated, and sent to the prey detection API.

## Camera Orientation

The OV2640 is mounted **sideways** in the cat flap housing. Raw frames are 640×480 in landscape, but the actual scene is portrait. The top ~20% of the rotated image is occluded by the cat flap frame.

Mapping (raw → rotated):
- Raw right edge → rotated top (occluded by cat flap frame)
- Raw left edge → rotated bottom

## Autonomous Mode (ESP32)

Runs when the laptop doesn't respond within `apiFallbackMs` (default 5s).

### Lossless JPEG Crop

Source: `src/main.cpp` (`cropJpegForApi()`) + `src/jpeg_lossless_crop.c`

```
Raw 640×480:
┌──────────────────────────────────────────┐
│         64px          384px       192px   │
│ left ◄──────► ┌──────────────┐ ◄───────► │  ← right = occluded (128px)
│ margin        │              │  right     │     + 64px extra margin
│               │  CROP REGION │  margin    │
│    48px ▲     │  384 × 384   │            │ 480px
│    top  │     │              │            │
│         ▼     └──────────────┘            │
│               ▲ 48px bottom margin        │
└──────────────────────────────────────────┘
                       640px
```

- **Crop origin**: (64, 48) — MCU-aligned for lossless extraction
- **Crop size**: 384×384
- **Asymmetric**: 64px left vs 192px right margin — right side has the cat flap occlusion
- **No rotation** — the API receives the raw (sideways) orientation
- **No decode/re-encode** — operates on MCU blocks directly (~15-30ms vs ~420ms full re-encode)

### Frame Selection & API Call

Source: `src/main.cpp` (`autonomousApiCheck()`, `callPreyApi()`)

1. Selects **3 largest frames** by JPEG size (most visual detail)
2. Lossless-crops each to 384×384
3. Base64-encodes (`mbedtls_base64_encode`)
4. POSTs `{"image_base64": "<b64>"}` to Cloudflare Worker over persistent TLS
5. **Short-circuits** on first prey detection (saves time)
6. Records per-frame timing: crop, base64, TLS handshake, POST

### Fallback Flow

Source: `src/main.cpp` (`apiFallbackTask()`)

```
Burst captured
    │
    ├─► Laptop notified (laptopPresent flag)
    │   Wait up to apiFallbackMs (5s)
    │       │
    │       ├─ Laptop responds with /cmd?result=0|1  → done
    │       └─ Timeout → autonomousApiCheck() runs
    │
    └─► Result stored in burstArchive, event logged to NVS
```

## Laptop Mode (burst_saver + prey_analyzer)

### Stage 1: Download & Save (burst_saver.py)

Source: `tools/burst_saver.py`

1. Polls `/stats` for new burst archives
2. Downloads frames via `/burststream?a=N` (MJPEG) or `/burst?a=N&i=I` (individual)
3. **Rotates 90° CW** — corrects for sideways-mounted camera
4. **Crops top 20%** — removes cat flap frame occlusion
5. Saves as JPEG quality 97

```
Raw 640×480 → rotate 90° CW → 480×640 → crop top 20% → 480×512
```

6. Fetches `/burstmeta?a=N` and saves as `burst_meta.json` (timing, distances, gain/AEC per frame)
7. Notifies ESP32 immediately that laptop is processing (resets fallback timer)

### Stage 2: Detection & API (prey_analyzer.py)

Source: `tools/prey_analyzer.py`

Multi-stage pipeline on the rotated/cropped frames:

1. **YOLO11n (NCNN)** — detects cat bounding boxes (class 15, confidence ≥ 0.15)
2. **Motion crop** — frame-differencing finds active region, validates YOLO boxes overlap with motion
3. **SSIM dedup** — skips near-identical frames (threshold 0.90)
4. **Smart crop** (`prepare_api_image()`):
   - Priority: YOLO cat box > motion ROI > full frame
   - 15-20% padding around detected region
   - Optional background subtraction (zeros static pixels)
   - Proportional resize to fit **≤384×384**
   - JPEG quality 90
5. POSTs `{"image_base64": "..."}` to same API endpoint
6. Notifies ESP32 of result via `GET /cmd?result=0|1&a=N`

## Comparison

| Aspect | ESP32 (autonomous) | Laptop |
|---|---|---|
| Rotation | None (raw orientation) | 90° CW |
| Crop | Fixed lossless 384×384 (occlusion-aware) | 20% top + smart YOLO/motion crop |
| Resize | None | Proportional to ≤384×384 |
| Frame selection | Top 3 by JPEG size | YOLO+motion validated, SSIM-deduped |
| Background sub | No | Yes (optional) |
| Latency | ~2-5s total (crop+API) | ~8-15s (download+YOLO+API) |
| Quality | Good (fixed crop may miss subject) | Better (crop centered on cat) |
| API format | `{"image_base64": "..."}` | Same |
| Endpoint | Cloudflare Worker | Same |
