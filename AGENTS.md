# AGENTS.md — Mazge

Guidance for AI agents working in this repo. Read this first, then the linked
docs before making changes.

## What this project is

Cat-flap prey-detection system: an **ESP32-S3** camera (`src/main.cpp`, firmware)
captures a burst of frames on a ToF trigger and POSTs them to a **local
inference server** (`server/`, FastAPI on a 24/7 Mac) that returns a prey/cat
decision; the firmware opens or locks the flap.

- Wire contract: [doc/inference_api_v2_contract.md](doc/inference_api_v2_contract.md)
- Architecture: [doc/architecture.md](doc/architecture.md)
- Classifier design/history: [doc/prey_classifier_v2.md](doc/prey_classifier_v2.md)

## Where things run (two machines)

| Machine | Role | Notes |
| --- | --- | --- |
| **Server Mac** (`mazge` / `mazge.local`, LAN `192.168.0.134`) | Runs `server/` 24/7 under launchd; receives frames; stores data | The **only** place ONNX export + deploy happens. |
| **Training laptop** (this machine, for most agents reading this) | Trains models on accumulated data | Produces `.pt` checkpoints; does NOT run the production server. |

The server Mac and the training laptop do **not** share a filesystem. Move data
between them with `rsync` (see below).

## Conventions

- **Run Python with `uv`**: `uv run python tools/<script>.py …`.
- **Never commit** logs, captures, crops, or model weights — they are gitignored
  (`logs/server/*`, `captures/`, `dataset/crops_*`, `*.pt`, `*.onnx`,
  `_bench_weights/`). Keep the repo to code + docs + `dataset/labels.jsonl`.
- **`dataset/labels.jsonl` is the append-only source of truth for labels.** Never
  rewrite it; append. `manifest.csv` / `bursts.csv` are regenerated.
- Make minimal, targeted changes; don't refactor unrelated code.

## The retraining task (why you're probably here)

The cat-ID head (mazge vs benis) sometimes rejects a real cat: the server gate is
`CAT_CONF_THRESHOLD = 0.70` in [server/decisions.py](server/decisions.py), and
mazge frames occasionally land at 0.64–0.70 → `cat_recognized=false` → door stays
shut. Goal: retrain a better-separated classifier on the months of accumulated
real production frames so genuine cats clear the threshold, without letting
strangers in (note: the head is 2-class with **no "stranger" class**, so the
threshold is the only stranger gate — don't lower it blindly, improve the model).

### Data sources

1. **Canonical training data** — `captures/sd/<burst>/` burst folders (frames +
   `meta.json`), labeled via `dataset/labels.jsonl`. Pulled from the ESP SD card
   / `burst_saver.py`. `dataset/README.md` documents the schema.
2. **Accumulated production frames on the server Mac** —
   `logs/server/debug-dump/YYYY-MM-DD/*.jpg`, ~8k+ full frames since 2026-06-05,
   plus per-request metadata in `logs/server/server.jsonl*` (kept forever as of
   2026-08-23). Filenames are `mazge-frontdoor-01_<burstid>_gen<G>_f<NN>_<ts>.jpg`;
   `server.jsonl` carries `burst_id`, `frame_index`, predicted `cat_id`,
   `cat_confidence`, `prey_score` per frame. These are **full frames and
   unlabeled** (only the model's own predictions exist) — they must be human-
   labeled for `cat_id` before use, and are richest exactly in the borderline
   0.5–0.7 confidence band that causes the failures.

Pull server data to the training laptop:

```bash
rsync -avz mazge.local:/Users/tomas/Mazge/logs/server/debug-dump/ ./_server_dump/
rsync -avz mazge.local:/Users/tomas/Mazge/logs/server/'server.jsonl*' ./_server_logs/
```

### Pipeline: raw frames → crops → model

```bash
# 1. (re)build the frame/burst manifest from captures/sd/ + labels.jsonl
uv run python tools/build_dataset.py

# 2. generate 224x224 body crops the classifier trains on
uv run python tools/build_crops.py --pipeline yolo11x_rotcrop

# 3. train the mazge-vs-benis head (EfficientNet-B0 on body crops)
uv run python tools/train_cat_id_v2.py --epochs 20 --tag <run_tag>
#    -> models/cat_id_v2/<run_tag>/best_val_burst_acc.pt
```

Prey classifier retraining is analogous via `tools/train_prey_v3.py`. The split
is **per-burst** (frames from one burst never cross train/val/test) — preserve
that to avoid leakage.

### Deploy a new checkpoint (runs on the SERVER Mac, not here)

Export `.pt` → ONNX, point the server env at it, restart. This is documented in
`/memories/repo/mazge-server-mac.md` on the server Mac and in
[server/README.md](server/README.md). The training laptop's job ends at a
validated `.pt`; hand it to the server Mac for ONNX export + `launchctl
kickstart`.

## Before you finish

- Validate: `uv run python -m pytest server/tests/ -q` for any `server/` change.
- Report metrics **per-burst**, not just per-frame (product aggregates 10 frames
  into one decision; per-frame F1 is noisy on rare classes).
