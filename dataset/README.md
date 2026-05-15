# Prey-detection dataset

Auto-generated from `captures/sd/`. Source of truth = the raw burst folders +
this directory's `labels.jsonl`. The CSV files can be deleted and rebuilt at
any time with:

```bash
uv run python tools/build_dataset.py
```

## Files

| File              | Purpose                                                                                  |
| ----------------- | ---------------------------------------------------------------------------------------- |
| `manifest.csv`    | One row per FRAME with current best labels. **Regenerated every run.**                   |
| `bursts.csv`      | One row per BURST with summary. **Regenerated every run.**                               |
| `labels.jsonl`    | Append-only label log with full provenance. **Source of truth for labels.**              |
| `README.md`       | This file.                                                                               |

## `manifest.csv` columns

| Column              | Meaning                                                                              |
| ------------------- | ------------------------------------------------------------------------------------ |
| `image_id`          | Path relative to repo root, e.g. `captures/sd/20260515_031942_gen14/f08_0800ms.jpg`. |
| `burst_id`          | Burst folder name, e.g. `20260515_031942_gen14`.                                     |
| `frame_idx`         | Index in the burst, 0..9.                                                            |
| `offset_ms`         | Frame timing within the burst (0 = first captured frame).                            |
| `captured_epoch`    | Unix epoch seconds when the burst was triggered (best-effort).                       |
| `trigger_ms`        | Device `millis()` when triggered (only meaningful within one boot).                  |
| `gain`, `aec`       | Camera gain (0–30) and exposure (0–1200 lines). `-1` = locked/unknown.               |
| `distance_mm`       | ToF reading at frame capture. `-1` = no/invalid reading.                             |
| `bytes`             | JPG file size on disk.                                                               |
| `night`             | Heuristic (`1` if `aec >= 150 && gain >= 8`, IR mode).                               |
| `fw_label`          | On-device API result: `0` no-prey, `1` prey, empty = not analyzed.                   |
| `fw_burst_label`    | Firmware burst-level verdict: `0` or `1`.                                            |
| `full_label`        | Laptop-side full-frame reanalysis: `0`, `1`, empty = not analyzed.                   |
| `full_burst_label`  | Burst-level verdict from full reanalysis (>=2 prey frames).                          |
| `best_label`        | `full_label` if known, else `fw_label`. Strict per-frame label (use for **eval**).   |
| `weak_label`        | **Training label**: `best_label` if known, else burst-propagated. Always set.        |
| `weak_confidence`   | Sample weight: `1.0` for hard labels, `0.4` for burst-propagated prey.               |
| `has_jpg`           | `1` if the JPG file exists locally, `0` if only `meta.json` was pulled.              |
| `split`             | `train` / `val` / `test`. Deterministic hash of `burst_id` (80/10/10).               |

## `labels.jsonl` records

Each line is one labeling **event** with provenance, e.g.:

```json
{"image_id": "captures/sd/20260515_032008_gen16/f08_0800ms.jpg",
 "source": "firmware", "label": 1, "confidence": null,
 "ts": "2026-05-15T13:50:00+00:00", "notes": null}
{"image_id": "captures/sd/20260515_032008_gen16/f08_0800ms.jpg",
 "source": "api_full", "label": 1, "confidence": null,
 "ts": "2026-05-15T13:50:00+00:00", "notes": null}
```

Sources currently in use:

| Source         | Origin                                                                              |
| -------------- | ----------------------------------------------------------------------------------- |
| `firmware`     | On-device call to `prey-detection.florian-mutel.workers.dev`. Sparse (5/10 frames). |
| `api_full`     | Laptop-side reanalysis on all 10 frames via `tools/reanalyze_all_frames.py`.        |

Suggested future sources (just append more JSONL records, no schema change):

| Source         | Use                                                                                 |
| -------------- | ----------------------------------------------------------------------------------- |
| `human:tomas`  | Manual ground-truth labels (active learning, error analysis, eval).                 |
| `model:v1.cnn` | Predictions from your own trained classifier (for self-distillation, drift checks). |

Dedup key on append is `(image_id, source, label)`. Re-running
`build_dataset.py` will not duplicate rows. To **change** a label, append a
new record with a different `label` value and update `manifest.csv` to read
the latest record per source (currently `build_dataset.py` overwrites
manifest from raw API data each run, but the JSONL keeps full history).

## Preprocessing applied to the API-trained labels

The on-device firmware **pre-crops** every frame before sending to the API:

```
640x480 raw camera frame
  → crop (64, 48, 384, 384)        # remove right ~128px (occluded), center
  → rotate 90° CCW                 # camera mounted sideways
  → grayscale + JPEG quality 85
  → 384x384 grayscale JPEG sent to API
```

The same preprocessing is reproduced in `tools/reanalyze_all_frames.py`
(`preprocess_frame()`). If you train your own model, apply the **same crop
+ rotate** to the raw 640x480 JPEGs in `captures/sd/`. Keeping the JPGs in
their original camera orientation in this dataset is intentional: it
preserves all information and lets future preprocessing experiments use
different crops without touching the raw data.

Reference implementation:

```python
from PIL import Image
img = Image.open("captures/sd/20260515_031942_gen14/f08_0800ms.jpg")
img = img.crop((64, 48, 64 + 384, 48 + 384))
img = img.rotate(90, expand=True)        # CCW
img = img.convert("L")                   # grayscale
```

## Splits

Train / val / test split is **per burst**, using SHA-1(`burst_id`) modulo
1000. All 10 frames of one burst go to the same split — never split frames
of one burst across train/val/test, they are highly correlated (same cat,
same lighting, ~1s apart).

Ratios are 80% / 10% / 10%. To re-split, change `SPLIT_RATIOS` in
`tools/build_dataset.py` and re-run.

## Class balance (current)

```
HARD labels (per-frame, API-confirmed):
  prey:    80 frames  (1.5%)
  no-prey: 4211 frames (98.5%)

WEAK labels (training-friendly, includes burst-propagated):
  prey hard (conf 1.0):  80 frames
  prey weak (conf 0.4):  132 frames     <- propagated from burst-positive
  no-prey  (conf 1.0):   5381 frames
  TOTAL:                 5593 frames    <- 100% coverage
```

## The label-fidelity problem

When the cat carries prey, it carries it through **all 10 frames** of the
burst. But the API only flags frames where prey is clearly visible (close,
in focus, well-lit) — typically `f7`–`f9`. Earlier frames (`f0`–`f4`) show
the same prey but at greater distance, often <20px wide, and the API
reliably misses them.

This matters for training: if you only train on per-frame API labels, the
model learns "prey is a thing visible at <30cm" rather than
"prey is *physically present* in this scene". The burst-level decision is
what actually controls the door.

### How we handle it

We emit two kinds of labels:

| Source             | Confidence | Origin                                                  |
| ------------------ | ---------- | ------------------------------------------------------- |
| `firmware`         | 1.0        | On-device API said this frame has prey (or not).        |
| `api_full`         | 1.0        | Laptop reanalyzed all 10 frames, this frame's verdict.  |
| `burst_propagated` | 0.4        | Burst is confirmed prey; this frame was not flagged but the cat is physically carrying prey. |
| `human:<name>`     | 1.0        | Manual ground truth (future).                           |

`weak_label` and `weak_confidence` in `manifest.csv` consolidate this:
- If any source confidently labels the frame → use that label, weight 1.0.
- Else if the burst is confirmed prey → label = 1, weight 0.4.
- Else if the burst is confirmed no-prey → label = 0, weight 1.0
  (cat passed clean, absence is reliable).

### Recommended training/eval strategy

**Training:** use `weak_label` with `weak_confidence` as the per-sample
weight in BCE loss. PyTorch:

```python
import torch
import torch.nn.functional as F

logits = model(img)                        # (B,)
target = batch["weak_label"].float()       # (B,) in {0, 1}
weight = batch["weak_confidence"].float()  # (B,) in (0, 1]
# Heavy class imbalance — also use pos_weight
pos_weight = torch.tensor([50.0])  # ~5400 neg / ~210 pos
loss = F.binary_cross_entropy_with_logits(
    logits, target, weight=weight, pos_weight=pos_weight)
```

**Evaluation:** there are two complementary metrics:

1. **Per-frame** (apples-to-apples with the API): filter `val`/`test` to
   rows where `best_label != ""` (hard labels only) and compute
   precision/recall on `best_label` vs `model_pred`.
2. **Per-burst** (matches product behavior): for each test burst, run the
   model on all frames, aggregate (e.g. `max(pred) > 0.5`), compare to
   `bursts.csv`'s `full_burst_label`. **This is the metric you actually
   care about** — false positives at frame level cancel out across the
   burst, false negatives are usually compensated by other frames.

## Class balance breakdown by split

```
 split   prey_hard   prey_weak      noprey
 train          67         107        4379
   val          10          13         512
  test           3          12         490
```

Test has only 3 hard-prey frames, but 15 prey-positive frames once weak
labels are included — enough for ~5 prey bursts. Add more by labelling
bursts manually (`source: human:tomas` in `labels.jsonl`) when you collect
more data.

For training:

- Class-balance via `pos_weight≈50` in BCE, or oversample positive bursts.
- Hard-negative mining: sample more aggressively from frames where firmware
  was uncertain (`fw_label == 0` but `distance_mm < 200` and `night == 1`).
- Augmentation: horizontal flip OK, vertical flip NOT (gravity matters).

## How to extend

### Add a human-labeled image

```python
import json, datetime as dt
rec = {"image_id": "captures/sd/20260515_032008_gen16/f08_0800ms.jpg",
       "source": "human:tomas", "label": 1, "confidence": 1.0,
       "ts": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
       "notes": "clear vole in mouth"}
with open("dataset/labels.jsonl", "a") as f:
    f.write(json.dumps(rec, sort_keys=True) + "\n")
```

Then optionally extend `build_dataset.py` to lift human labels into
`manifest.csv` as a `human_label` column.

### Add a new analysis source

1. Run your analysis, save per-frame results somewhere.
2. Either: extend `build_dataset.py` to read your output and emit
   records, or write a small one-off script that appends to
   `labels.jsonl` with a unique `source` string.
3. Re-run `build_dataset.py` so `manifest.csv`'s `best_label` reflects
   whatever priority you choose.
