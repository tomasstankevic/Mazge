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
| `best_label`        | `full_label` if known, else `fw_label`. Used as default training label.              |
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
prey:    80 frames  (1.5%)
no-prey: 4211 frames (98.5%)
```

Severely imbalanced — typical for cat-flap data. For training:

- Oversample prey or use weighted loss (`pos_weight ≈ 50` for BCE).
- Hard-negative mining: most no-prey frames are trivially "empty flap"
  shots; sample more aggressively from frames where firmware was uncertain
  or where ToF distance was small (`distance_mm < 200`).
- Add data augmentation: horizontal flip OK, vertical flip NOT (gravity
  matters for prey appearance).

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
