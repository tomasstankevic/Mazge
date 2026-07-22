# Mazge local inference server

FastAPI service that replaces the cloud prey API with a LAN-local endpoint
running on the 24/7 Mac (`mazge.local`, Tailscale `100.85.62.100`).

- Wire contract: [`doc/inference_api_v2_contract.md`](../doc/inference_api_v2_contract.md)
- Migration plan: [`doc/local_inference_server_migration.md`](../doc/local_inference_server_migration.md)
- Bench numbers + ops notes: `/memories/repo/mazge-server-mac.md`

## Endpoints

### `GET /healthz`

Liveness probe used by launchd / HA.

```json
{"ok": true, "backend": "onnx", "uptime_s": 168}
```

### `POST /v1/compat` (legacy contract)

For unmodified firmware. JSON in / JSON out. **This is what the ESP32 calls today** when
`PREY_API_URL` is pointed at the local server.

```sh
curl -sS http://mazge.local:8080/v1/compat \
  -H 'Content-Type: application/json' \
  -d "{\"image_base64\":\"$(base64 < frame.jpg | tr -d '\n')\"}"
```

Response:

```json
{"detected": false, "score": 0.0231, "cat_id": "mazge"}
```

`detected` is a boolean derived from `prey_score` + `cat_recognized` — same
semantics as the cloud Worker's response, so existing firmware needs no parser
change. Only the URL changes.

### `POST /v2/frame` (v2 contract)

Raw JPEG body + required headers. Use this from updated firmware to get
`severity`, `lockout_seconds`, `cat_id`, `should_continue_burst`, etc.

Required headers:

| header | value |
|---|---|
| `X-Contract-Version` | `2` |
| `X-Device-Id` | e.g. `mazge-frontdoor-01` |
| `X-Burst-Id` | e.g. `20260605_120000_gen17` |
| `X-Frame-Index` | `0`-based int |
| `X-Frame-Ts-Ms` | unix epoch ms |
| `X-Request-Id` | uuid (used for idempotent retries) |
| `Content-Type` | `image/jpeg` |

```sh
curl -sS http://mazge.local:8080/v2/frame \
  -H 'Content-Type: image/jpeg' \
  -H 'X-Contract-Version: 2' \
  -H 'X-Device-Id: mazge-frontdoor-01' \
  -H 'X-Burst-Id: 20260605_120000_gen17' \
  -H 'X-Frame-Index: 0' \
  -H "X-Frame-Ts-Ms: $(($(date +%s) * 1000))" \
  -H "X-Request-Id: $(uuidgen)" \
  --data-binary @frame.jpg
```

Response (truncated):

```json
{
  "contract_version": 2,
  "request_id": "5ac6a467-...",
  "detected": false,
  "prey_score": 0.0231,
  "cat_recognized": true,
  "cat_id": "mazge",
  "cat_confidence": 0.94,
  "severity": "none",
  "lockout_seconds": 0,
  "door_action": "allow",
  "model_version": "prey_v3",
  "decision_ms": 510,
  "server_ts_ms": 1780673504867,
  "should_continue_burst": false,
  "reason": "high_confidence_non_prey"
}
```

Error format (all 4xx / 5xx):

```json
{
  "contract_version": 2,
  "request_id": "<echo>",
  "error": "unsupported_media_type",
  "message": "expect image/jpeg"
}
```

Status mapping is per the v2 contract: 400, 409, 413, 415, 422, 500.
Idempotency: same `X-Request-Id` + identical body returns the cached prior
response; same `X-Request-Id` + different body returns 409.

## Configuration

All knobs are environment variables (see [`server/config.py`](config.py)):

| var | default | notes |
|---|---|---|
| `MAZGE_MODEL_BACKEND` | `stub` | `stub` (no model) or `onnx` |
| `MAZGE_YOLO_ONNX` | (none) | absolute path to yolo11s ONNX |
| `MAZGE_PREY_ONNX` | (none) | absolute path to prey_v3 ONNX |
| `MAZGE_YOLO_IMGSZ` | `640` | must match the imgsz the ONNX was exported at |
| `MAZGE_THREADS` | `0` | ORT intra-op threads; 0 = ORT default |
| `MAZGE_PORT` | `8080` | |
| `MAZGE_LOG_DIR` | `./logs/server` | JSON-line request log |
| `MAZGE_MQTT_HOST` | `` | empty = MQTT disabled |
| `MAZGE_MQTT_PORT` | `1883` | |
| `MAZGE_MQTT_USER`, `MAZGE_MQTT_PASS` | `` | |
| `MAZGE_IDEMPOTENCY_TTL_S` | `300` | per v2 contract |

## Running

### From a shell (foreground, for dev)

```sh
cd /Users/tomas/Mazge
set -a; source ~/.config/mazge/server.env; set +a
.venv-server/bin/uvicorn server.app:app --host 0.0.0.0 --port 8080
```

### As a launchd Agent (24/7)

Plist at [`ops/launchd/com.mazge.server.plist`](../ops/launchd/com.mazge.server.plist).
It is symlinked into `~/Library/LaunchAgents/` and loaded once at install time;
launchd brings it back up automatically after crashes, restarts, and macOS
reboot when the user logs in. The job runs uvicorn through `caffeinate -i -s`,
so the inference host cannot enter idle/system sleep while the server is live.
This is required for deterministic door latency; launchd cannot serve requests
or restart a process while the whole Mac is asleep.

Install:

```sh
ln -sf /Users/tomas/Mazge/ops/launchd/com.mazge.server.plist \
       ~/Library/LaunchAgents/com.mazge.server.plist
launchctl load -w ~/Library/LaunchAgents/com.mazge.server.plist
```

Check / control:

```sh
launchctl list com.mazge.server                 # status + PID
launchctl kickstart -k gui/$(id -u)/com.mazge.server   # restart (e.g. after model swap)
launchctl unload  ~/Library/LaunchAgents/com.mazge.server.plist  # stop
tail -f /Users/tomas/Mazge/logs/server/launchd.err.log          # uvicorn stderr
tail -f /Users/tomas/Mazge/logs/server/server.jsonl              # one JSON line per request
```

## Model files

The production checkpoints under `models/prey_v3/` are PyTorch `.pt` files.
For the server we export them once to ONNX (gitignored under
[`_bench_weights/`](../_bench_weights/)).

```sh
.venv-server/bin/python -c "
import onnx  # warm any deps
from tools.bench_onnx_local import export_prey_v3
from pathlib import Path
export_prey_v3(
    Path('models/prey_v3/bodyA/best_burst_f1.pt'),
    Path('_bench_weights/prey_v3_bodyA_224.onnx'),
    224,
)
"
```

The yolo11s ONNX export is done by Ultralytics:

```sh
.venv-server/bin/python -c "
from ultralytics import YOLO
YOLO('_bench_weights/yolo11s.pt').export(format='onnx', imgsz=640, simplify=True)
"
mv _bench_weights/yolo11s.onnx _bench_weights/yolo11s_640.onnx
```

When `bodyA_s` lands, repeat the prey_v3 export pointing at
`models/prey_v3/bodyA_s/best_burst_f1.pt`, change `MAZGE_PREY_ONNX` in the
plist, then `launchctl kickstart -k gui/$(id -u)/com.mazge.server`.

## Tests

```sh
.venv-server/bin/python -m pytest server/tests/ -q
```

Covers every item from the v2 contract's "conformance checklist".

## Latency on this host

2016 13" MBP, Intel i5-6360U (2C/4T), 8 GiB RAM, macOS 12.7.6, ONNX Runtime
CPU. Synthetic frames; real JPEG decode adds 5–10 ms.

| pipeline | p50 | p95 | note |
|---|---:|---:|---|
| **yolo11s @ 480 + prey_v3 bodyA_s480 @ 224** | **~240 ms** | **~320 ms** | **current production** — zero prey leaks in 655-burst eval |
| yolo11s @ 640 + prey_v3 bodyA_s @ 224 | ~430 ms | ~790 ms | previous; 1 prey leak |
| yolo11s @ 384 + prey_v3 bodyA @ 224 | ~165 ms | ~250 ms | fastest, but bodyA mismatched with yolo11s detector |

First inference after a reload is **5–18 s cold** as ORT initializes mkldnn
kernels; subsequent calls drop to the warm numbers above. The launchd plist's
warmup pre-runs both models once at startup so the first real frame is warm.
