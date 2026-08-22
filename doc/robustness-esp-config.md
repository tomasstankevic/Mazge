# ESP32 Configuration & Monitoring Robustness

## Problem That Occurred (2026-08-22)
Dashboard showed 2-day-old image because the ESP proxy wasn't running. Root cause: the server process wasn't loading `MAZGE_ESP_HOST` from `~/.config/mazge/server.env` at startup.

This wasn't an IP address change issue—it was a **missing configuration validation** on server startup.

## Solution: Three Layers of Robustness

### 1. Startup Validation (ops/launchd/run-server.sh)
**Problem**: Server could start without loading critical config, silently using default empty `esp_host`.

**Fix**: Added mandatory checks in startup script:
- Verify `~/.config/mazge/server.env` exists (fail fast if missing)
- Verify `MAZGE_ESP_HOST` is set (don't silently default to empty)
- Log the ESP host at startup for visibility

```bash
if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: $ENV_FILE not found. Server requires critical config..." >&2
  exit 1
fi

if [[ -z "$MAZGE_ESP_HOST" ]]; then
  echo "ERROR: MAZGE_ESP_HOST not set in $ENV_FILE" >&2
  exit 1
fi
```

### 2. ESP Proxy Status Tracking (server/esp_proxy.py)
**Problem**: When ESP32 became unreachable (network issue, IP change, etc.), server had no visibility into the problem.

**Fix**: Added `ProxyStatus` dataclass to track:
- `last_success_ms`: timestamp of last successful request through proxy
- `success_count` / `error_count`: cumulative statistics
- `is_healthy`: True if we've had a successful request in the last 60 seconds
- `last_error`: error message from most recent failure

Status is updated on every proxy request (success or failure).

### 3. Enhanced Health Check Endpoint (/healthz)
**Problem**: `/healthz` only reported server backend & uptime, not ESP connectivity.

**Fix**: Enhanced endpoint to include ESP proxy status:
```json
{
  "ok": true,
  "backend": "onnx",
  "uptime_s": 42,
  "esp": {
    "reachable": true,
    "last_success_s_ago": 0,
    "success_count": 1,
    "error_count": 0
  }
}
```

This allows monitoring systems to detect when ESP32 becomes unreachable.

## How to Use

### Verify Configuration at Startup
```bash
# Server logs will show:
# Starting Mazge server with ESP_HOST=192.168.0.41
# esp32 proxy listening on :8081 → http://192.168.0.41
```

### Monitor ESP Connectivity
```bash
# Check ESP proxy health
curl http://127.0.0.1:8080/healthz | jq '.esp'

# Expected output when healthy:
# {
#   "reachable": true,
#   "last_success_s_ago": 5,
#   "success_count": 42,
#   "error_count": 0
# }

# When ESP32 is unreachable:
# {
#   "reachable": false,
#   "last_success_s_ago": 3600,
#   "success_count": 42,
#   "error_count": 12
# }
```

### Alert Rules (for external monitoring)
- Alert if `esp.reachable == false` for > 5 minutes
- Alert if `esp.error_count` increases without corresponding `success_count` increases
- Check dashboard shows recent images (not 2+ days old)

## What This Prevents

1. **Silent startup failures**: Config validation prevents server from running without ESP connectivity
2. **Undetected disconnections**: Status tracking makes ESP issues visible in `/healthz`
3. **Cascading failures**: Clear error messages help identify root cause quickly (IP change, network issue, ESP32 crash, etc.)

## Future Improvements (Not Yet Implemented)

- Auto-retry with backoff when ESP becomes unreachable
- Publish ESP status to MQTT for external monitoring
- Dashboard alert when esp.reachable = false
- Automatic restart of server if esp_host cannot be resolved for > 10 minutes
