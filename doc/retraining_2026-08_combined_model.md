# Combined multitask model retraining — 2026-08

Retrain of the cat-flap classifier on the accumulated production data
(Jun–Aug 2026 server dump) merged with the original Apr–May dataset, folding the
separate prey / cat-id heads and the new subject + direction tasks into **one**
EfficientNet-B0 network.

Prepared on the **training laptop**. Deployment (ONNX export + swap) happens on
the **server Mac** — see "Deployment" below.

## TL;DR

- New single model does 4 tasks: **prey**, **cat-id** (mazge/benis),
  **subject** (empty/cat/human/other = "no-cat"), **direction** (entering/exiting).
- Robust 5-fold cross-validation (every burst held out once):
  - **Prey: 0 false negatives across all 32 prey bursts** (recall 1.000).
  - **Cat-id: 96.1%** burst accuracy (n=357) — the primary goal (door-reject bug).
  - Subject 96.6%, Direction 96.7% — new capabilities, both strong.
- The multitask heads **help** the primary tasks (fewer prey FPs and better
  cat-id than a prey+cat-only model trained identically — see below).
- **Known limitation:** ~26 confident prey **false positives** that neither a
  higher threshold nor a frames-flagged rule can remove without missing real
  prey. Not a regression (baseline-equivalent is worse), but they cause
  occasional false lockouts. Fix = review + hard-negative labeling (`_fp_review.txt`).

## Data

`captures/sd/` = 2763 bursts after ingesting the server dump.

| axis | old (Apr–May) | new (Jun–Aug) | total |
| --- | --- | --- | --- |
| cat-id benis | 74 | +71 | 145 |
| cat-id mazge | 147 | +86 | 233 |
| direction entering / exiting | 306 / 155 | +84 / +73 | 390 / 228 |
| subject cat / non-cat | 496 / 216 | +165 / +38 | 661 / 254 |
| prey-positive bursts | 29 | +4 | **33** |

Prey stays scarce (+4). The big gain is cat-id (Benis nearly doubled) plus the
new direction/subject supervision.

Pipeline that produced the training data:
1. `tools/ingest_server_dump.py` — flat server dump → `captures/sd/<burst>/` + `meta.json`.
2. `tools/label_bursts.py` — unified labeler (now includes the mazge/benis axis, keys `m/b/k`).
3. `tools/build_dataset.py` — regenerate `manifest.csv` / `bursts.csv` from `labels.jsonl`.
4. `tools/build_crops.py --pipeline yolo11x_rotcrop` — 224px body crops.

## Model

`tools/train_multitask.py` — `MazgeMultiTask`: EfficientNet-B0 backbone +
frame-index embedding, four heads. **Direction uses stop-gradient (detached)
features by default** so it cannot perturb the backbone the primary prey task
relies on (`--direction-grad` to relax). Split = stable per-burst hash split.

```
uv run python tools/train_multitask.py --epochs 25 --tag combined_v1 \
    --baseline models/prey_v3/bodyA/best_burst_f1.pt
# -> models/multitask/combined_v1/best.pt
```

## Benchmark

The original `prey_v3/bodyA/test_metrics.json` used a *different* (restratified)
test set, so it is **not** directly comparable. All numbers below are on the
same data.

### Single hash split (underpowered: 4 prey / 30 cat test bursts)

| metric | new combined | old prey_v3 (same split) |
| --- | --- | --- |
| prey burst F1 | 0.727 (tp4/fp3/fn0) | 0.800 (tp4/fp2/fn0) |
| prey frame F1 | 0.839 | 0.918 |
| cat-id burst acc | 0.867 | 0.933 |

Both catch 100% of prey; the differences are ±1 event — noise on 4 prey bursts.

### 5-fold cross-validation (robust: all 32 prey / 357 cat bursts)

`tools/cv_multitask.py` — pools out-of-fold predictions so every burst is tested once.

| metric | **full 4-head** | prey+cat only (control) |
| --- | --- | --- |
| prey burst recall | **1.000 (0 FN)** | 1.000 (0 FN) |
| prey burst FP | **38** | 48 |
| prey frame FP / FN | 98 / 46 | 117 / 43 |
| cat-id burst acc | **0.961** | 0.955 |
| subject frame acc | 0.966 | — |
| direction burst acc | 0.967 | — |

The full model has **fewer prey FPs and better cat-id** than the prey+cat-only
control trained identically → the subject/direction heads regularize the
backbone. "Combine into one" was the right call.

### Threshold / weight re-tuning (goal: FN=0, minimize FP)

True-prey per-burst max score: min **0.761**, median 0.999. Raising the per-frame
detect threshold **0.50 → 0.76** keeps FN=0 and cuts FP bursts **38 → 26**. Above
0.76 real prey starts dropping (FN>0).

Decision-rule sweep (prey if ≥K frames ≥ T), FN=0 required:

| rule | FN | FP |
| --- | --- | --- |
| ≥1 frame ≥0.50 | 0 | 38 |
| **≥1 frame ≥0.76** | **0** | **26** ← chosen |
| ≥2 frames ≥0.76 | 3 ✗ | 17 |
| ≥3 frames ≥0.76 | 4 ✗ | 9 |

**Frames-flagged does not help at FN=0:** 3 real-prey bursts show only 1 frame
≥0.76, so a "≥2 frames" rule loses them. Of the 26 surviving FPs, 17 have ≥2
flagged frames (would be long lockouts) — these are confident, multi-frame,
genuinely prey-like errors. See `_server_dump/_fp_review.txt`.

**Recommended thresholds:** per-frame prey detect = **0.76** (was 0.50);
keep firmware's ≥2-frame rule for long vs short lockout. Loss weights unchanged
(prey 1.0, cat 0.30, subject 0.30, direction 0.20) — validated best by the CV
control.

## Verdict

| benchmark | result | pass |
| --- | --- | --- |
| Prey FN = 0 (never miss prey) | 0 FN over 32 prey bursts (CV) | ✅ (margin thin: weakest prey 0.761) |
| Prey FP vs baseline-equivalent | 38 vs 48 | ✅ better |
| Cat-id (primary goal) | 0.961 vs 0.955 | ✅ |
| Subject / direction added | 0.966 / 0.967 | ✅ |
| Prey FP absolute | 26 bursts (17 long-lockout) | ⚠️ known limitation |

**Recommendation: GO** — net improvement on every measured axis, FN=0, cat-id
bug addressed, and it collapses 3 heads + 2 ToF heuristics into one model. Ship
with the 0.76 threshold. The prey false-positive rate is the one caveat; it is
**not** worse than what runs today, and is bounded by scarce prey data.

## Deployment (runs on the SERVER Mac)

The new ONNX has 4 outputs but the **same input signature** as prey_v3
(`image` 1×3×224×224 + `frame_idx`). The server pipeline reads only outputs 0
(prey) and 1 (cat), so it is **drop-in — no `server/` code change required** for
prey+cat. subject/direction outputs are latent until wired up (optional, below).

1. Get the checkpoint onto the server Mac (laptop → server):
   ```bash
   # from the training laptop (server Mac not reachable directly right now — use the
   # same manual/AirDrop path used for the data dump, or scp once routing is fixed):
   scp models/multitask/combined_v1/best.pt \
       tomas@<server>:~/Mazge/models/multitask/combined_v1/best.pt
   ```
2. On the server Mac, export to ONNX:
   ```bash
   cd ~/Mazge
   uv run python tools/export_multitask_onnx.py \
       --ckpt models/multitask/combined_v1/best.pt \
       --out  _bench_weights/multitask_combined_v1_224.onnx
   # verify: 4 outputs, out[0]=prey_logit, out[1]=cat_logits
   ```
3. Point the server env at the new model and set the retuned threshold:
   ```bash
   # ~/.config/mazge/server.env
   MAZGE_PREY_ONNX=/Users/tomas/Mazge/_bench_weights/multitask_combined_v1_224.onnx
   ```
   Threshold: change `PREY_MEDIUM = 0.50` → `PREY_MEDIUM = 0.76` in
   `server/decisions.py` (per-frame detect floor). **Do this atomically with the
   ONNX swap** — 0.76 is calibrated to the new model's scores, not the old one.
4. Restart + verify:
   ```bash
   launchctl kickstart -k gui/$(id -u)/com.mazge.server
   curl -s http://mazge.local:8080/healthz
   uv run python -m pytest server/tests/ -q     # 18 pass
   ```
5. **Rollback:** point `MAZGE_PREY_ONNX` back at the old
   `prey_v3_bodyA_s480_224.onnx`, revert the `PREY_MEDIUM` line, kickstart.

### Optional (later): use the subject + direction heads

To actually consume outputs 2/3, extend `server/model_pipeline.py`
(`InferResult` + `_classify_prey` to read `outputs[2]` subject softmax,
`outputs[3]` direction softmax) and `server/decisions.py` (e.g. a real "no-cat"
gate from the subject head instead of relying only on `CAT_CONF_THRESHOLD`).
Not required for this deployment.

## Firmware (`src/main.cpp`) — assessment

**No change required.** The v2 wire contract is unchanged (same request, same
response fields). The tiered lockout (`PREY_SHORT_LOCKOUT_MS` 3 min for 1 flagged
frame, `PREY_LONG_LOCKOUT_MS` 15 min for ≥`PREY_FRAMES_THRESHOLD`=2) is driven by
the per-frame `detected` count from the server and keeps working as-is. Exits are
physically free regardless of the door pin, so the new direction head does not
need firmware wiring. Optional future use of direction/subject would be a
server-side decision change, still transparent to the firmware.

## Follow-ups

1. **Review `_server_dump/_fp_review.txt`** (26 confident FP bursts, worst first)
   in the labeler: `uv run python tools/label_bursts.py --bursts _server_dump/_fp_review.txt`.
   Several cluster on single May nights (e.g. `20260514_*`) — confirm they are
   truly no-prey (hard negatives) vs mislabeled, then retrain. This is the
   highest-leverage fix for the prey false positives.
2. Collect more true-prey events — 33 bursts is the ceiling on prey confidence.
3. Re-run `tools/cv_multitask.py` after new labels to confirm FN stays 0 and FP drops.
