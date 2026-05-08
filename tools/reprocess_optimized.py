"""Re-run all 10 frames of each prey burst through the OPTIMIZED pipeline:
full pixel decode + crop right occlusion + 90 CCW rotate + proportional resize
to fit 384x384 + grayscale + JPEG q=90.

Saves results to captures/prey_review/reprocess_optimized.json.
"""
import argparse
import base64
import json
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pipeline_optimized import process_optimized

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent /
           "API_key_prey_detector.txt").read_text().strip()


def call_api(jpg: bytes):
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


def find_frame_jpg(folder, idx):
    for p in folder.iterdir():
        if (p.is_file() and p.suffix == ".jpg"
                and p.stem.startswith(f"f{idx:02d}")
                and (len(p.stem) == 3 or p.stem[3] == "_")):
            return p
    return None


def process_frame(folder, idx):
    src_path = find_frame_jpg(folder, idx)
    if src_path is None:
        return idx, None, 0, 0, "missing"
    try:
        out = process_optimized(src_path.read_bytes())
    except Exception as e:
        return idx, None, 0, 0, f"prep_err {e}"
    verdict, api_ms, err = call_api(out)
    return idx, verdict, len(out), api_ms, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="captures/prey_review")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default="captures/prey_review/reprocess_optimized.json")
    args = ap.parse_args()

    root = Path(args.root)
    bursts = sorted([d for d in root.iterdir() if d.is_dir()])
    print(f"Reprocessing {len(bursts)} bursts with OPTIMIZED pipeline\n")

    all_results = {}
    for bi, burst in enumerate(bursts, 1):
        if sum(1 for p in burst.iterdir() if p.suffix == ".jpg") == 0:
            continue
        verdicts = [None] * 10
        sizes = [0] * 10
        api_times = [0] * 10
        errs = [""] * 10
        with ThreadPoolExecutor(max_workers=args.workers) as ex:
            futs = {ex.submit(process_frame, burst, i): i for i in range(10)}
            for fut in as_completed(futs):
                idx, v, sz, t, err = fut.result()
                verdicts[idx] = v
                sizes[idx] = sz
                api_times[idx] = t
                errs[idx] = err
        hits = sum(1 for v in verdicts if v == 1)
        line = "[" + "".join("P" if v == 1 else "." if v == 0 else "-" if v is None else "?"
                             for v in verdicts) + "]"
        print(f"[{bi:2d}/{len(bursts)}] {burst.name:32s}  {line}  hits={hits}/10")
        for i, e in enumerate(errs):
            if e:
                print(f"     f{i:02d}: {e}")
        all_results[burst.name] = {
            "verdicts": verdicts, "sizes": sizes,
            "api_times_ms": api_times, "errs": errs, "hits": hits,
        }

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(all_results, indent=2))
    print(f"\nWrote {args.out}")

    from collections import Counter
    cnt, chk = Counter(), Counter()
    for r in all_results.values():
        for i, v in enumerate(r["verdicts"]):
            if v in (0, 1):
                chk[i] += 1
            if v == 1:
                cnt[i] += 1
    print("\n=== Per-frame hit count (optimized) ===")
    for i in range(10):
        rate = (cnt[i] * 100 / chk[i]) if chk[i] else 0
        print(f"  f{i:02d}    {cnt[i]:>3}/{chk[i]:<3}  {rate:>5.1f}%  {'#' * cnt[i]}")


if __name__ == "__main__":
    main()
