"""Re-run all 10 frames of each prey burst through the CURRENT production
pipeline (lossless DCT crop+rotate+drop-chroma -> upright 384x384 grayscale)
and report per-frame API hits.

This shows what the current firmware WOULD detect if it processed all frames,
giving us a clean view of which frames actually contain detectable prey
(versus what was sent live during capture, which may have used the old
unrotated pipeline).
"""
import argparse
import base64
import csv
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests

# Use the validated Python reference of the C lossless rotate
sys.path.insert(0, str(Path(__file__).resolve().parent))
from jpeg_lossless_crop_rotate_gray import process

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent /
           "API_key_prey_detector.txt").read_text().strip()

# Production crop settings (match src/main.cpp CROP_X/CROP_Y/CROP_SZ)
CROP_X, CROP_Y, CROP_W, CROP_H = 64, 48, 384, 384


def call_api(jpg: bytes) -> tuple[int, float, str]:
    """Returns (verdict, elapsed_ms, error). verdict: 1=prey, 0=clear, -1=err."""
    t0 = time.time()
    try:
        r = requests.post(
            API_URL,
            headers={"Authorization": f"Bearer {API_KEY}",
                     "Content-Type": "application/json"},
            json={"image_base64": base64.b64encode(jpg).decode()},
            timeout=20,
        )
    except Exception as e:
        return -1, (time.time() - t0) * 1000, f"req_err {e}"
    elapsed = (time.time() - t0) * 1000
    if r.status_code != 200:
        return -1, elapsed, f"HTTP{r.status_code}"
    try:
        return (1 if r.json().get("detected") else 0), elapsed, ""
    except Exception as e:
        return -1, elapsed, f"json_err {e}"


def find_frame_jpg(folder: Path, idx: int) -> Path | None:
    """Find the file for frame idx, supports fNN.jpg and fNN_NNNNms.jpg."""
    cands = [p for p in folder.iterdir()
             if p.is_file() and p.suffix == ".jpg"
             and p.stem.startswith(f"f{idx:02d}")
             and (len(p.stem) == 3 or p.stem[3] == "_")]
    return cands[0] if cands else None


def process_frame(folder: Path, idx: int):
    """Read raw VGA jpg, run lossless crop+rotate+drop-chroma, send to API."""
    src_path = find_frame_jpg(folder, idx)
    if src_path is None:
        return idx, None, 0, 0, "missing"
    try:
        src = src_path.read_bytes()
        out = process(src, CROP_X, CROP_Y, CROP_W, CROP_H)
        out_len = len(out)
    except Exception as e:
        return idx, None, 0, 0, f"prep_err {e}"
    verdict, api_ms, err = call_api(out)
    return idx, verdict, out_len, api_ms, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="captures/prey_review",
                    help="Folder containing burst subfolders")
    ap.add_argument("--workers", type=int, default=4,
                    help="Concurrent API calls")
    ap.add_argument("--out", default="captures/prey_review/reprocess.json",
                    help="Where to write detailed JSON results")
    ap.add_argument("--csv", default="captures/prey_review/reprocess.csv",
                    help="Where to write summary CSV")
    args = ap.parse_args()

    root = Path(args.root)
    bursts = sorted([d for d in root.iterdir() if d.is_dir()])
    print(f"Reprocessing {len(bursts)} bursts in {root}")
    print(f"Pipeline: lossless DCT crop({CROP_X},{CROP_Y},{CROP_W},{CROP_H}) "
          f"+ 90 CCW rotate + drop chroma -> upright 384x384 grayscale\n")

    all_results = {}
    csv_rows = [("burst", "f00", "f01", "f02", "f03", "f04",
                 "f05", "f06", "f07", "f08", "f09", "hits", "old_hits")]

    for bi, burst in enumerate(bursts, 1):
        meta_path = burst / "meta.json"
        old_results = []
        if meta_path.exists():
            try:
                old_results = json.loads(meta_path.read_text()).get("apiResults") or []
            except Exception:
                pass

        # Skip if no jpgs yet
        jpg_count = sum(1 for p in burst.iterdir() if p.suffix == ".jpg")
        if jpg_count == 0:
            print(f"[{bi}/{len(bursts)}] {burst.name}  (no jpgs, skip)")
            continue

        verdicts = [None] * 10
        sizes = [0] * 10
        api_times = [0] * 10
        errs = [""] * 10

        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            futs = {ex.submit(process_frame, burst, i): i for i in range(10)}
            for fut in as_completed(futs):
                idx, verdict, sz, api_ms, err = fut.result()
                verdicts[idx] = verdict
                sizes[idx] = sz
                api_times[idx] = api_ms
                errs[idx] = err

        hits = sum(1 for v in verdicts if v == 1)
        old_hits = sum(1 for r in old_results if r == 1)

        line = f"[{bi:2d}/{len(bursts)}] {burst.name:32s}  "
        line += "[" + "".join("P" if v == 1 else
                              "." if v == 0 else
                              "-" if v is None else "?"
                              for v in verdicts) + "]"
        line += f"  hits={hits}/10  (was {old_hits})"
        print(line)
        for i, e in enumerate(errs):
            if e:
                print(f"     f{i:02d}: {e}")

        all_results[burst.name] = {
            "verdicts": verdicts,
            "sizes": sizes,
            "api_times_ms": api_times,
            "errs": errs,
            "hits": hits,
            "old_hits": old_hits,
            "old_results": old_results,
        }

        def cell(v):
            if v == 1: return "P"
            if v == 0: return "."
            return "-"
        csv_rows.append((burst.name, *[cell(v) for v in verdicts], hits, old_hits))

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(all_results, indent=2))
    with open(args.csv, "w") as f:
        csv.writer(f).writerows(csv_rows)
    print(f"\nWrote {args.out} and {args.csv}")

    # Aggregate: per-frame hit rate
    print("\n=== Per-frame hit count (out of {} bursts) ===".format(len(all_results)))
    from collections import Counter
    cnt = Counter()
    chk = Counter()
    for r in all_results.values():
        for i, v in enumerate(r["verdicts"]):
            if v in (0, 1):
                chk[i] += 1
            if v == 1:
                cnt[i] += 1
    print(f"{'frame':>6} {'prey':>6} {'checked':>8} {'rate':>6}")
    for i in range(10):
        rate = (cnt[i] * 100 / chk[i]) if chk[i] else 0
        bar = "#" * cnt[i]
        print(f"  f{i:02d}    {cnt[i]:>5}   {chk[i]:>5}    {rate:>5.1f}% {bar}")


if __name__ == "__main__":
    main()
