#!/usr/bin/env python3
"""Ingest the server production dump into captures/sd/ burst folders.

The 24/7 inference server saves every frame it decides on to
``logs/server/debug-dump/YYYY-MM-DD/<device>_<counter>_gen<G>_f<NN>_<ts>.jpg``
(full 640x480 RGB frames, same distribution as the SD-card captures) and logs
one JSON line per frame in ``logs/server/server.jsonl*`` with the model's own
prediction (prey_score, cat_id, cat_confidence, cat_softmax).

That dump is copied to this laptop under::

    _server_dump/YYYY-MM-DD/*.jpg      (the frames)
    _server_logs/server.jsonl*         (per-frame model predictions)

This tool reconstructs the burst-folder layout the rest of the pipeline
(``label_bursts.py``, ``build_dataset.py``, ``build_crops.py``) expects::

    captures/sd/<YYYYMMDD_HHMMSS_genG>/
        f00.jpg .. fNN.jpg
        meta.json

``meta.json`` mirrors the firmware SD schema (``images`` list with per-frame
``f``/``bytes``/``dist``/``gain``/``aec``/``ms``, plus ``apiResults`` /
``apiResult`` / ``epoch``) so the existing tools work unchanged. Camera state
(dist/gain/aec) is unknown for server frames and stored as ``-1``.

Server-only extras are recorded for provenance + label prioritisation:
    meta["source"]           = "server_dump"
    meta["server_burst_id"]  = "<counter>_genG"
    meta["server"]["frames"] = [{idx, prey_score, cat_id, cat_recognized,
                                 cat_confidence, cat_softmax}]

Idempotent: a sidecar ``_server_dump/_ingest_map.json`` records which server
bursts have been ingested, so re-running only picks up new data.

Usage::

    uv run python tools/ingest_server_dump.py                 # ingest all new
    uv run python tools/ingest_server_dump.py --dry-run       # report only
    uv run python tools/ingest_server_dump.py --limit 50      # first N bursts
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import time
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DUMP_DIR = REPO / "_server_dump"
LOGS_DIR = REPO / "_server_logs"
SD_DIR = REPO / "captures" / "sd"
MAP_PATH = DUMP_DIR / "_ingest_map.json"

# <device>_<counter>_gen<G>_f<NN>_<ts>.jpg  (device may contain hyphens)
FNAME_RE = re.compile(
    r"^(?P<device>.+?)_(?P<counter>\d+)_gen(?P<gen>\d+)_f(?P<fidx>\d+)_(?P<ts>\d+)\.jpg$"
)

PREY_POS_THRESHOLD = 0.5  # server prey_score -> weak fw prey label for queue priority


def load_predictions() -> dict[str, dict]:
    """Map basename(image_path) -> latest server /v2/frame record for that frame."""
    preds: dict[str, dict] = {}
    if not LOGS_DIR.exists():
        return preds
    for p in sorted(LOGS_DIR.glob("server.jsonl*")):
        with p.open(errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line.startswith("{"):
                    continue  # plain log message, not a JSON record
                try:
                    rec = json.loads(line)
                except Exception:
                    continue
                if rec.get("ep") != "/v2/frame":
                    continue
                img = rec.get("image_path")
                if not img:
                    continue
                preds[Path(img).name] = rec
    return preds


def parse_frame(path: Path) -> dict | None:
    m = FNAME_RE.match(path.name)
    if not m:
        return None
    return {
        "path": path,
        "device": m.group("device"),
        "counter": m.group("counter"),
        "gen": int(m.group("gen")),
        "frame_index": int(m.group("fidx")),
        "ts_ms": int(m.group("ts")),
        "server_burst_id": f"{m.group('counter')}_gen{m.group('gen')}",
    }


def group_bursts(dump_dir: Path) -> dict[str, list[dict]]:
    bursts: dict[str, list[dict]] = defaultdict(list)
    unparsed = 0
    for jpg in dump_dir.rglob("*.jpg"):
        info = parse_frame(jpg)
        if info is None:
            unparsed += 1
            continue
        bursts[info["server_burst_id"]].append(info)
    if unparsed:
        print(f"  warning: {unparsed} files did not match the expected name pattern")
    return bursts


def folder_name_for(frames: list[dict], gen: int, taken: set[str]) -> str:
    t0 = min(fr["ts_ms"] for fr in frames) / 1000.0
    base = time.strftime("%Y%m%d_%H%M%S", time.localtime(t0)) + f"_gen{gen}"
    name = base
    n = 1
    # Disambiguate the vanishingly rare same-second+gen collision.
    while name in taken or (SD_DIR / name).exists():
        n += 1
        name = f"{base}_{n}"
    taken.add(name)
    return name


def build_meta(frames: list[dict], preds: dict[str, dict], gen: int) -> dict:
    frames = sorted(frames, key=lambda fr: fr["frame_index"])
    t0 = min(fr["ts_ms"] for fr in frames)
    images = []
    api_results = []
    server_frames = []
    any_prey = False
    for fr in frames:
        rec = preds.get(fr["path"].name)
        images.append(
            {
                "f": f"f{fr['frame_index']:02d}.jpg",
                "bytes": fr["path"].stat().st_size,
                "dist": -1,  # ToF not available server-side
                "gain": -1,
                "aec": -1,
                "ms": fr["ts_ms"] - t0,
            }
        )
        if rec is not None:
            prey = float(rec.get("prey_score", 0.0))
            is_prey = 1 if prey >= PREY_POS_THRESHOLD else 0
            any_prey = any_prey or bool(is_prey)
            api_results.append(is_prey)
            server_frames.append(
                {
                    "idx": fr["frame_index"],
                    "prey_score": prey,
                    "cat_id": rec.get("cat_id"),
                    "cat_recognized": rec.get("cat_recognized"),
                    "cat_confidence": rec.get("cat_confidence"),
                    "cat_softmax": rec.get("cat_softmax"),
                }
            )
        else:
            api_results.append(-1)  # frame the server never logged a decision for
    return {
        "gen": gen,
        "frames": len(frames),
        "epoch": t0 // 1000,
        "apiResult": 1 if any_prey else 0,
        "apiFramesSent": sum(1 for r in api_results if r != -1),
        "apiResults": api_results,
        "images": images,
        "source": "server_dump",
        "server_burst_id": frames[0]["server_burst_id"],
        "server": {"frames": server_frames},
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true", help="report only, write nothing")
    ap.add_argument("--limit", type=int, default=0, help="ingest at most N new bursts")
    args = ap.parse_args()

    if not DUMP_DIR.exists():
        raise SystemExit(f"missing {DUMP_DIR} — copy the server dump there first")
    SD_DIR.mkdir(parents=True, exist_ok=True)

    print(f"loading predictions from {LOGS_DIR} ...")
    preds = load_predictions()
    print(f"  {len(preds)} frames have a server prediction record")

    print(f"scanning {DUMP_DIR} ...")
    bursts = group_bursts(DUMP_DIR)
    total_frames = sum(len(v) for v in bursts.values())
    print(f"  {len(bursts)} bursts / {total_frames} frames in the dump")

    ingest_map: dict[str, str] = {}
    if MAP_PATH.exists():
        ingest_map = json.loads(MAP_PATH.read_text())
    taken: set[str] = set(ingest_map.values())

    new_ids = [b for b in sorted(bursts) if b not in ingest_map]
    print(f"  {len(ingest_map)} already ingested, {len(new_ids)} new")
    if args.limit:
        new_ids = new_ids[: args.limit]

    made = 0
    frames_copied = 0
    with_preds = 0
    prey_bursts = 0
    for sbid in new_ids:
        frames = bursts[sbid]
        gen = frames[0]["gen"]
        folder = folder_name_for(frames, gen, taken)
        meta = build_meta(frames, preds, gen)
        if meta["apiResult"] == 1:
            prey_bursts += 1
        if meta["apiFramesSent"] > 0:
            with_preds += 1
        if args.dry_run:
            made += 1
            frames_copied += len(frames)
            continue
        dest = SD_DIR / folder
        dest.mkdir(parents=True, exist_ok=True)
        for fr in frames:
            shutil.copy2(fr["path"], dest / f"f{fr['frame_index']:02d}.jpg")
            frames_copied += 1
        (dest / "meta.json").write_text(json.dumps(meta, indent=2))
        ingest_map[sbid] = folder
        made += 1

    if not args.dry_run:
        MAP_PATH.write_text(json.dumps(ingest_map, indent=2, sort_keys=True))

    verb = "would ingest" if args.dry_run else "ingested"
    print(
        f"\n{verb} {made} bursts ({frames_copied} frames) into {SD_DIR}\n"
        f"  {with_preds} bursts carry server predictions, "
        f"{prey_bursts} have >=1 frame with prey_score>={PREY_POS_THRESHOLD}"
    )
    if not args.dry_run:
        print(f"  ingest map: {MAP_PATH}")
        print("\nnext: uv run python tools/build_dataset.py  (regenerate manifest/CSVs)")


if __name__ == "__main__":
    main()
