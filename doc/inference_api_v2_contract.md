# inference api v2 contract (esp32 <-> local mac server)

last updated: 2026-05-31

## purpose

Freeze a low-latency LAN protocol between firmware and the local inference server.

Important versioning note:

- model version: prey_v3 (already trained and committed)
- api contract version: v2 (this document)

## transport and endpoint

- protocol: HTTP/1.1 on trusted LAN (no TLS)
- method: POST
- path: /v2/frame
- content type: image/jpeg
- request body: raw JPEG bytes for one frame

### required request headers

- X-Contract-Version: 2
- X-Device-Id: unique camera id, for example mazge-frontdoor-01
- X-Burst-Id: burst generation id, for example 20260531_221530_gen17
- X-Frame-Index: zero-based index in burst
- X-Frame-Ts-Ms: capture timestamp in unix epoch milliseconds
- X-Request-Id: unique id for idempotent retries

### optional request headers

- X-Trace-Id: distributed trace id for diagnostics
- X-Firmware-Version: firmware version string
- X-Model-Hint: optional model selector, default prey_v3

### request limits

- max body size: 512000 bytes
- max header size: 8 KB total

If a request exceeds limits, server returns 413.

## success response

Status: 200 OK

Content-Type: application/json

```json
{
  "contract_version": 2,
  "request_id": "9f4c4e48-7e7f-4c3f-9c3a-80d6f3170be5",
  "detected": true,
  "prey_score": 0.9321,
  "cat_recognized": true,
  "cat_id": "mazge",
  "cat_confidence": 0.9872,
  "severity": "high",
  "lockout_seconds": 600,
  "door_action": "deny",
  "model_version": "prey_v3",
  "decision_ms": 34,
  "server_ts_ms": 1780182985123,
  "should_continue_burst": false,
  "reason": "high_confidence_prey"
}
```

Field definitions:

- contract_version: integer, must be 2
- request_id: string echo of X-Request-Id
- detected: boolean prey verdict for this frame
- prey_score: float in [0,1]
- cat_recognized: boolean, true only when cat identity confidence is above configured threshold
- cat_id: string label or unknown
- cat_confidence: float in [0,1]
- severity: enum in [none, low, medium, high, critical]
- lockout_seconds: integer lockout duration to apply from this decision
- door_action: enum in [allow, deny]
- model_version: string, expected prey_v3 initially
- decision_ms: integer server compute time in ms
- server_ts_ms: integer unix epoch ms at response
- should_continue_burst: boolean hint for firmware burst loop
- reason: short machine-readable reason code

## door and lockout policy contract

Firmware must enforce these rules exactly:

1. If cat_recognized is false, then door_action must be deny.
2. A frame can result in allow only when cat_recognized is true and severity is none.
3. Firmware must apply lockout_seconds from the response when door_action is deny.
4. Firmware must store cat_id, severity, lockout_seconds, and request_id in event logs for auditing.

### normative severity to lockout mapping

Server returns severity and lockout_seconds based on current policy:

- none -> 0 seconds
- low -> 30 seconds
- medium -> 120 seconds
- high -> 600 seconds
- critical -> 1800 seconds

The mapping is part of the wire contract and must remain stable unless contract_version changes.

### suggested decision policy inputs

- no cat recognized: severity=medium, door_action=deny
- cat recognized and high prey score: severity=high or critical, door_action=deny
- cat recognized and low prey score: severity=none, door_action=allow

### reason enum

- high_confidence_prey
- high_confidence_non_prey
- uncertain_need_more_frames
- decode_error
- model_unavailable
- no_cat_recognized
- lockout_by_severity

## error responses

All error responses use JSON:

```json
{
  "contract_version": 2,
  "request_id": "9f4c4e48-7e7f-4c3f-9c3a-80d6f3170be5",
  "error": "bad_request",
  "message": "missing header X-Burst-Id"
}
```

Status mapping:

- 400 bad_request: malformed headers/body
- 401 unauthorized: optional future auth rejected
- 404 not_found: unknown endpoint
- 409 duplicate_request: duplicate X-Request-Id with conflicting payload
- 413 payload_too_large: JPEG too large
- 415 unsupported_media_type: not image/jpeg
- 422 unprocessable_frame: cannot decode usable image
- 429 rate_limited: temporary load shedding
- 500 internal_error: unexpected server failure
- 503 model_unavailable: model not loaded or warming up

## latency, timeout, and retry rules

Firmware side rules:

- request timeout: 350 ms per frame
- retry count: at most 1 retry
- retry condition: timeout or 5xx only
- retry backoff: fixed 60 ms
- retries must reuse the same X-Request-Id

Server side rules:

- target p50 latency <= 60 ms
- target p95 latency <= 140 ms
- if processing cannot complete before 300 ms budget, fail fast with 503

## idempotency contract

- X-Request-Id uniquely identifies one logical frame decision
- if same X-Request-Id and identical payload repeats, return cached prior response
- if same X-Request-Id but payload differs, return 409 duplicate_request

Recommended cache TTL: 5 minutes.

## burst control contract

Firmware sends frames in ascending X-Frame-Index order.

Stop conditions for current burst:

- stop early when response has should_continue_burst = false
- continue when should_continue_burst = true and frame budget not exhausted
- always stop at configured max frames per burst

Default policy guidance:

- if prey_score >= 0.90: detected=true, should_continue_burst=false
- if prey_score <= 0.10 for 3 consecutive frames: detected=false, should_continue_burst=false
- otherwise: should_continue_burst=true

## backward compatibility

To keep existing firmware running during rollout, server should also expose a compatibility endpoint:

- path: /v1/compat
- request content type: application/json
- request body: {"image_base64":"..."}
- response body must include detected boolean

Migration strategy:

1. deploy server with both /v1/compat and /v2/frame
2. switch firmware to /v2/frame in staged rollout
3. remove /v1/compat only after stable operation window

## observability requirements

Server must log one line per request with:

- request_id
- device_id
- burst_id
- frame_index
- cat_id
- cat_recognized
- severity
- lockout_seconds
- door_action
- status_code
- prey_score
- detected
- decision_ms

Metrics to expose:

- requests_total by status code
- inference_latency_ms histogram
- duplicate_request_hits_total
- model_unavailable_total

## conformance checklist

- [ ] rejects non-jpeg payloads with 415
- [ ] enforces X-Contract-Version = 2
- [ ] echoes request_id in all responses
- [ ] returns detected in 200 responses
- [ ] returns cat_recognized in 200 responses
- [ ] returns cat_id in 200 responses
- [ ] returns severity and lockout_seconds in 200 responses
- [ ] never returns door_action=allow when cat_recognized=false
- [ ] supports idempotent replay by X-Request-Id
- [ ] emits should_continue_burst hint
- [ ] meets timeout and latency targets in LAN tests
