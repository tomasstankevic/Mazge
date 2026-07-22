# v2 Inference Latency Investigation — July 2026

## Symptom

Trigger-to-door latency, originally ~4-5 s, had crept up and was frequently
spiking to 6-10 s. The goal was to get the common case back under 5 s.

## Method

Latency was measured end-to-end with the on-device instrumentation:

- `/getevents` — NVS-persisted per-burst `lat` (survives reboots).
- `/v2stats` — per-frame `httpMs` (wall-clock POST) vs `serverMs`
  (server-reported `decision_ms`), plus burst `wallMs`. Resets each boot.
- `/burstmeta?a=<n>` — per-burst `apiFramesSent` and per-frame HTTP codes.
- `/diag` — boot count, reset reason, RSSI, `apiAbandons`, and (new) WiFi
  reconnect counters.

Ping triangulation (dev-Mac -> gateway vs dev-Mac -> ESP32) isolated which
network hop carried the loss/jitter.

## Root cause: the ESP32's 2.4 GHz uplink, not firmware or the server

The decisive evidence: across many bursts that all sent the same 5 frames, the
server's `decision_ms` stayed rock-steady at ~120-160 ms, yet total burst
latency ranged from **1.2 s to 11.6 s**. All of that variance is `httpMs` —
the time to upload the raw JPEG over WiFi. The inference server is never the
bottleneck.

Ping triangulation confirmed it:

| Path | Latency | Loss |
|------|---------|------|
| dev-Mac -> gateway | 5 ms | 0% |
| dev-Mac -> ESP32 (via AP -> 2.4 GHz) | 120-380 ms, up to 20% loss | fluctuating |

Both paths share the dev-Mac -> AP hop, so the excess latency and loss are
entirely on the **AP -> ESP32 2.4 GHz hop**. The band is congested (neighbour
networks occupy channels 1, 8 and 11 — every usable 2.4 GHz channel), and the
router (Sagemcom FAST3890V3-RDK) exposes no manual channel selection.

The ESP32-S3 is 2.4 GHz-only hardware, so it cannot move to the clean 5 GHz
band. The inference server (a Mac) must stay on 5 GHz: the router applies
client isolation on 2.4 GHz, so a same-band ESP32<->server pair cannot see each
other, while the cross-band path (ESP32 2.4 GHz -> AP -> server 5 GHz) works.

## Firmware changes made

### 1. One TCP connection per burst (keep-alive)

Previously the firmware tore down and rebuilt the `WiFiClient` + `HTTPClient`
for every frame. It now reuses a single connection for all frames in a burst
(`v2OpenForCall` checks `connected()` first; `setReuse(true)`). Header
accumulation is not an issue because `HTTPClient::addHeader` replaces same-name
headers by default. This removed the per-frame handshake overhead that
dominated the timing variance on a healthy link.

### 2. Always send all 5 frames; recall-safe early-exit

A mid-investigation experiment added a `DOOR_DECISION_BUDGET_MS` that stopped a
burst once ~3 s had elapsed. On a slow link this capped bursts at 1-4 frames
instead of the full 5, silently reducing prey-detection recall. It also changed
the early-exit to fire on the server's `should_continue_burst=false`, which the
server sets for **both** confident-prey and confident-clean-cat frames — so a
clean first frame could stop analysis before prey appeared in a later frame.

Both were reverted to the pre-July behaviour:

- The door budget was removed entirely — every burst sends all
  `MAX_API_FRAMES` (5).
- Early-exit now fires **only** on confidently-detected prey
  (`v2Detected && severity in {high, critical}`), where the prey is already
  caught and more frames cannot change the deny decision. It never stops on a
  clean-cat frame.

Frames are still sent in prey-probability priority order `[8,7,9,5,6,...]`, so
the most informative frames go out first.

### 3. Re-assert WiFi radio settings on every reconnect

`esp_wifi_set_ps(WIFI_PS_NONE)` and max TX power were set once at boot, but the
Arduino core resets power-save to `WIFI_PS_MIN_MODEM` (DTIM sleep, ~100-300 ms
stalls) on every auto-reconnect. A `WiFi.onEvent` handler now re-applies both on
each `STA_CONNECTED`, and `/diag` exposes `wifiReconnects` / `wifiDisconnects`
so link stability is observable. (Measured impact this session was small — the
link rarely dropped — but it closes a real failure mode.)

## What was tried and rejected

Tightening `V2_HTTP_TIMEOUT_MS` (1500 -> 1000 ms), adding a short connect
timeout, and lwIP `SO_SNDTIMEO`/`SO_RCVTIMEO` socket timeouts: this **tripled**
the error rate (3% -> 20-26%) and did not bound single-frame upload stalls,
because the raw-JPEG upload blocks inside HTTPClient's write loop below the
timeout knobs. Reverted; `V2_HTTP_TIMEOUT_MS` stays at 1500 ms.

Payload shrinking (a smaller JPEG for the API copy) would directly cut upload
airtime but was ruled out to preserve model-input fidelity.

## Measured outcome

With all 5 frames sent, on a mix of calm and congested air:

- Latencies: `1.2, 1.7, 2.2, 3.0, 3.0, 3.4, 3.8, 3.9, 5.6, 6.4, 7.3, 8.7, 11.6 s`
- Success: ~95% of frames (`ok`), zero task abandonments, no reboots.
- Server `decision_ms`: steady ~120-160 ms throughout.

Best case (calm air) is excellent (~1.2-2.2 s). Real cats are faster still,
because a confidently-detected prey frame early-exits after 1-2 frames.

## Remaining limitation

The residual spikes are physical 2.4 GHz airtime contention, characterised but
not solvable in firmware without shrinking the payload. The remaining levers are
all network-side:

- A dedicated 2.4 GHz AP nearer the ESP32, or reduced congestion.
- Keeping the ESP32 web UI closed when not actively watching — its page polls
  `/stats` 5x/second and can stream MJPEG on port 81, both of which steal
  2.4 GHz airtime and the httpd's limited sockets from inference traffic.
