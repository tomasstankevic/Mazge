"""Build the prey-detection dataset from captures/sd/.

Outputs:
  dataset/manifest.csv  — one row per FRAME with current labels (rebuilt every run)
  dataset/bursts.csv    — one row per BURST with summary
  dataset/labels.jsonl  — append-only label log with full provenance
                          (idempotent: each (image_id, source, label) added at most once)

Sources currently understood:
  - "firmware":  per-frame label from on-device API call (meta.json apiResults[i])
                 -1 means firmware did not analyze that frame; we still emit the
                 record with label=null so downstream knows it was skipped.
  - "api_full":  per-frame label from laptop-side reanalysis on ALL frames
                 (full_analysis.json). Run via tools/reanalyze_all_frames.py.

Future sources (just append more JSONL records):
  - "human:<name>": manual ground-truth labels
  - "model:<id>":   predictions from a trained classifier (active learning)

Usage:
  uv run python tools/build_dataset.py                 # full rebuild
  uv run python tools/build_dataset.py --since 20260501
  uv run python tools/build_dataset.py --root captures/sd  --out dataset

The manifest can be re-derived at any time from the raw data + labels.jsonl,
so the CSV files are safe to delete and rebuild. Only labels.jsonl is the
source of truth for labels (especially human ones).
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEFAULT_ROOT = REPO / "captures" / "sd"
DEFAULT_OUT = REPO / "dataset"

# Threshold used by the firmware to mark a burst as PREY confirmed.
# Documented in main.cpp PREY_FRAMES_THRESHOLD.
PREY_FRAMES_THRESHOLD = 2

# Confidence weight for burst-propagated weak labels: when a burst is
# confirmed prey-positive (≥PREY_FRAMES_THRESHOLD frames flagged), we
# propagate label=1 to all OTHER frames in that burst at this confidence.
# Rationale: the cat is physically carrying prey throughout the burst, but
# in early frames the prey is small/distant and the API may not see it.
# These weak labels let a trained model learn distance-invariant features.
# 0.4 means "40% as confident as a hard label" — use as sample weight in BCE.
BURST_PROP_CONFIDENCE = 0.4

# Train/val/test split ratios by burst (NOT by frame — frames in the same
# burst are highly correlated and must stay together to avoid leakage).
SPLIT_RATIOS = {"train": 0.80, "val": 0.10, "test": 0.10}


def burst_split(burst_id: str) -> str:
    """Deterministic train/val/test assignment from a stable hash of burst_id."""
    h = int(hashlib.sha1(burst_id.encode()).hexdigest()[:8], 16)
    r = (h % 1000) / 1000.0
    if r < SPLIT_RATIOS["train"]:
        return "train"
    if r < SPLIT_RATIOS["train"] + SPLIT_RATIOS["val"]:
        return "val"
    return "test"


def burst_date(burst_id: str) -> dt.date | None:
    try:
        return dt.datetime.strptime(burst_id[:8], "%Y%m%d").date()
    except ValueError:
        return None


def load_json(p: Path) -> dict | None:
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except Exception:
        return None


def is_night(meta_image: dict) -> bool:
    """Heuristic: high AEC (long exposure) + non-trivial gain == IR/night mode.
    Frames where the firmware locked exposure (aec=-1) inherit night status
    from the burst's first analyzed frame; we just return False here and let
    the caller decide using burst-level info if needed."""
    aec = meta_image.get("aec", -1)
    gain = meta_image.get("gain", -1)
    if aec < 0 or gain < 0:
        return False
    # Empirically: night frames in this dataset have aec >= 150 and gain >= 8
    return aec >= 150 and gain >= 8


def build_frame_rows(burst_dir: Path) -> tuple[list[dict], dict | None, dict | None]:
    """Return (frame_rows, burst_summary, raw_records_to_emit_to_labels_log).

    raw_records is a list of (image_id, source, label, confidence, ts, notes)
    tuples that should be appended to labels.jsonl if not already present.
    """
    burst_id = burst_dir.name
    meta = load_json(burst_dir / "meta.json")
    full = load_json(burst_dir / "full_analysis.json")
    if meta is None:
        return [], None, []

    images = meta.get("images", [])
    fw_results = meta.get("apiResults", [-1] * len(images))
    fw_burst = 1 if meta.get("apiResult") == 1 else 0
    captured_epoch = meta.get("epoch")  # None for very old bursts (pre-epoch field)
    trigger_ms = meta.get("triggerMs")

    full_results = (full or {}).get("full_results", [])
    # full_results may be shorter or longer than meta images (e.g. burst saved
    # before all frames captured). We index defensively.

    # Compute full burst-level label
    full_prey_count = sum(1 for r in full_results if r is True)
    full_burst_label = (
        1 if full_prey_count >= PREY_FRAMES_THRESHOLD
        else 0 if full_results
        else None  # not analyzed
    )

    split = burst_split(burst_id)
    rows: list[dict] = []
    label_records: list[dict] = []

    for i, img in enumerate(images):
        fname = img.get("f")
        if not fname:
            continue
        jpg_path = burst_dir / fname
        if not jpg_path.exists():
            # JPG missing locally (e.g., we only pulled meta.json). Skip
            # the row but still emit any labels we have so they are not lost.
            jpg_size = None
            has_jpg = False
        else:
            jpg_size = jpg_path.stat().st_size
            has_jpg = True

        # Path stored in manifest is relative to repo root for portability
        try:
            rel = jpg_path.relative_to(REPO)
        except ValueError:
            rel = jpg_path
        image_id = str(rel)

        fw_raw = fw_results[i] if i < len(fw_results) else -1
        fw_label = fw_raw if fw_raw in (0, 1) else None  # -1 -> skipped

        full_raw = full_results[i] if i < len(full_results) else None
        if full_raw is True:
            full_label = 1
        elif full_raw is False:
            full_label = 0
        else:
            full_label = None  # error / not analyzed

        best_label = full_label if full_label is not None else fw_label

        rows.append({
            "image_id": image_id,
            "burst_id": burst_id,
            "frame_idx": i,
            "offset_ms": img.get("offsetMs", img.get("ms")),
            "captured_epoch": captured_epoch,
            "trigger_ms": trigger_ms,
            "gain": img.get("gain", -1),
            "aec": img.get("aec", -1),
            "distance_mm": img.get("dist", img.get("distance", -1)),
            "bytes": jpg_size if jpg_size is not None else "",
            "night": int(is_night(img)),
            "fw_label": "" if fw_label is None else fw_label,
            "fw_burst_label": fw_burst,
            "full_label": "" if full_label is None else full_label,
            "full_burst_label": "" if full_burst_label is None else full_burst_label,
            "best_label": "" if best_label is None else best_label,
            # Training-friendly columns:
            #   weak_label       — label to use when training (incl. burst-propagated)
            #   weak_confidence  — sample weight: 1.0 for hard labels, 0.4 for propagated
            "weak_label": "",      # filled in below after the loop knows burst verdict
            "weak_confidence": "", # filled in below
            "has_jpg": int(has_jpg),
            "split": split,
        })

        # Emit one label record per (source, label) we know about.
        # Skipped (-1) firmware frames produce no record (label is unknown).
        if fw_label is not None:
            label_records.append({
                "image_id": image_id,
                "source": "firmware",
                "label": fw_label,
                "confidence": None,
                "notes": None,
            })
        if full_label is not None:
            label_records.append({
                "image_id": image_id,
                "source": "api_full",
                "label": full_label,
                "confidence": None,
                "notes": None,
            })

    # === Determine the consolidated burst-level verdict ===========
    # Prefer the laptop-side full reanalysis if available, otherwise the
    # firmware decision. Used to (a) backfill weak_label for all frames
    # of a confirmed-prey burst and (b) emit burst_propagated label records.
    consolidated_burst_label = (
        full_burst_label if full_burst_label is not None else fw_burst
    )

    # Fill weak_label / weak_confidence on the rows we just built.
    for row in rows:
        # Hard label wins: if any source already says prey/no-prey for this
        # frame, use it at confidence 1.0.
        hard = row["best_label"]
        if hard != "":
            row["weak_label"] = hard
            row["weak_confidence"] = 1.0
        elif consolidated_burst_label == 1:
            # Burst is confirmed prey but THIS frame was not analyzed (or
            # was analyzed and came back 0 — in which case best_label != ""
            # and we already took the hard branch). Propagate burst label
            # at reduced confidence.
            row["weak_label"] = 1
            row["weak_confidence"] = BURST_PROP_CONFIDENCE
        elif consolidated_burst_label == 0:
            # Burst confirmed no-prey: propagate label=0 to unanalyzed
            # frames at full confidence (cat went through and was clean,
            # so the absence of prey in unseen frames is reliable).
            row["weak_label"] = 0
            row["weak_confidence"] = 1.0

    # Emit burst_propagated label records for frames where weak label
    # differs from any hard label we already have.
    if consolidated_burst_label == 1:
        for row in rows:
            if row["best_label"] != 1 and row["weak_label"] == 1:
                label_records.append({
                    "image_id": row["image_id"],
                    "source": "burst_propagated",
                    "label": 1,
                    "confidence": BURST_PROP_CONFIDENCE,
                    "notes": f"burst {burst_id} confirmed prey-positive",
                })

    burst_summary = {
        "burst_id": burst_id,
        "date": burst_date(burst_id).isoformat() if burst_date(burst_id) else "",
        "frames": len(images),
        "fw_burst_label": fw_burst,
        "fw_frames_sent": meta.get("apiFramesSent", 0),
        "fw_prey_count": sum(1 for r in fw_results if r == 1),
        "full_burst_label": "" if full_burst_label is None else full_burst_label,
        "full_prey_count": full_prey_count if full_results else "",
        "has_full_analysis": int(full is not None),
        "has_jpgs": int(any((burst_dir / img.get("f", "")).exists() for img in images)),
        "split": burst_split(burst_id),
        "epoch": captured_epoch or "",
    }

    return rows, burst_summary, label_records


def append_labels_jsonl(jsonl_path: Path, new_records: list[dict]) -> int:
    """Append records that are not already present.
    Dedup key is (image_id, source, label) — we do NOT dedup on confidence/notes
    so a re-run of the same source overwrites nothing (no duplicate row).
    """
    seen: set[tuple[str, str, int]] = set()
    if jsonl_path.exists():
        with jsonl_path.open() as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                seen.add((rec.get("image_id", ""), rec.get("source", ""),
                          int(rec.get("label", -1))))

    now = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    added = 0
    with jsonl_path.open("a") as f:
        for r in new_records:
            key = (r["image_id"], r["source"], int(r["label"]))
            if key in seen:
                continue
            seen.add(key)
            r = {**r, "ts": now}
            f.write(json.dumps(r, sort_keys=True) + "\n")
            added += 1
    return added


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT,
                    help="Root captures dir (default: captures/sd)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT,
                    help="Dataset output dir (default: dataset/)")
    ap.add_argument("--since", help="YYYYMMDD; skip bursts older than this")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    since_date = (
        dt.datetime.strptime(args.since, "%Y%m%d").date() if args.since else None
    )

    burst_dirs = sorted(d for d in args.root.iterdir() if d.is_dir())
    if since_date:
        burst_dirs = [
            d for d in burst_dirs
            if (bd := burst_date(d.name)) and bd >= since_date
        ]
    print(f"Scanning {len(burst_dirs)} burst folders under {args.root}...")

    all_frame_rows: list[dict] = []
    all_burst_rows: list[dict] = []
    all_label_records: list[dict] = []

    for d in burst_dirs:
        rows, summary, labels = build_frame_rows(d)
        all_frame_rows.extend(rows)
        if summary:
            all_burst_rows.append(summary)
        all_label_records.extend(labels)

    # Write manifest.csv
    manifest_path = args.out / "manifest.csv"
    if all_frame_rows:
        with manifest_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(all_frame_rows[0].keys()))
            w.writeheader()
            w.writerows(all_frame_rows)

    # Write bursts.csv
    bursts_path = args.out / "bursts.csv"
    if all_burst_rows:
        with bursts_path.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(all_burst_rows[0].keys()))
            w.writeheader()
            w.writerows(all_burst_rows)

    # Append new labels (idempotent)
    labels_path = args.out / "labels.jsonl"
    added = append_labels_jsonl(labels_path, all_label_records)

    # Summary
    n_jpgs = sum(1 for r in all_frame_rows if r["has_jpg"])
    n_with_label = sum(1 for r in all_frame_rows if r["best_label"] != "")
    n_prey = sum(1 for r in all_frame_rows if r["best_label"] == 1)
    n_no_prey = sum(1 for r in all_frame_rows if r["best_label"] == 0)
    by_split: dict[str, int] = {}
    for r in all_frame_rows:
        if r["has_jpg"]:
            by_split[r["split"]] = by_split.get(r["split"], 0) + 1

    print()
    print("=" * 64)
    print(f"Bursts:                   {len(all_burst_rows)}")
    print(f"Frames (rows):            {len(all_frame_rows)}")
    print(f"  with local JPG:         {n_jpgs}")
    print(f"  with any label:         {n_with_label}  (prey={n_prey}, no-prey={n_no_prey})")
    print(f"Frames by split:          {by_split}")
    print(f"Labels appended to log:   {added} new (total file: {labels_path})")
    print(f"Wrote {manifest_path}")
    print(f"Wrote {bursts_path}")
    print(f"Wrote {labels_path}")


if __name__ == "__main__":
    main()
