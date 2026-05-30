#!/usr/bin/env python3
"""Compare snout-localisation methods on YOLO11x body crops.

Runs each method on every prey-positive cat body crop (and a random
sample of no-prey cat crops as controls), draws the predicted snout
box on each crop, and produces an HTML grid for visual inspection.

Methods (more can be added):
  A. baseline_heuristic   — top 45 % of body crop, full width
                             (matches what build_crops.py saves today)
  B. haar_cat_face        — OpenCV Haar cat-face cascade on the
                             upright body crop, with CLAHE
                             preprocessing. Picks the highest-area
                             face above min size. Falls back to None
                             when nothing found.
  B+ haar_extended        — `haarcascade_frontalcatface_extended.xml`
                             (more permissive than the basic variant)
  C. haar_with_niceibume_extension
                          — Same as B but extends 40 % below the
                             face box to capture chin + mouth + prey
                             held in front (the original niciBume
                             trick).

Output:
  models/snout_loc_eval/
    per_image.csv       — one row per (image, method) with bbox
                          and inference time
    summary.csv         — per-method aggregate stats
    dashboard.html      — grid: rows=method, cols=images; click to
                          flip through the prey set then controls

Usage:
  cd Mazge
  uv run python tools/test_snout_localisers.py
  uv run python tools/test_snout_localisers.py --controls 60
  uv run python tools/test_snout_localisers.py --crops mdv6_raw  # other pipeline
"""
from __future__ import annotations

import argparse
import csv
import logging
import random
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("snout_loc")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
OUT = REPO / "models" / "snout_loc_eval"


# ── Method implementations ───────────────────────────────────────────

def m_heuristic_top(img: np.ndarray) -> dict:
    """Top 45 % of crop, full width. This is what build_crops.py saves."""
    h, w = img.shape[:2]
    head_h = int(h * 0.45)
    return {"bbox": (0, 0, w, head_h), "conf": 1.0, "found": True}


_haar_cache = {}


def _get_haar(name: str):
    if name in _haar_cache:
        return _haar_cache[name]
    path = cv2.data.haarcascades + name
    cc = cv2.CascadeClassifier(path)
    if cc.empty():
        raise FileNotFoundError(path)
    _haar_cache[name] = cc
    return cc


def _haar_face(img: np.ndarray, cascade_name: str,
               min_size_frac: float = 0.20,
               scale_factor: float = 1.1,
               min_neighbors: int = 3) -> dict:
    """Run a Haar cat-face cascade on the upright body crop.

    Returns the best (largest-area) face above min_size_frac of crop
    side, or {"found": False} when nothing is detected. Applies CLAHE
    contrast normalisation first for IR/night robustness.
    """
    cc = _get_haar(cascade_name)
    h, w = img.shape[:2]
    min_dim = int(min(h, w) * min_size_frac)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    faces = cc.detectMultiScale(
        gray,
        scaleFactor=scale_factor,
        minNeighbors=min_neighbors,
        minSize=(min_dim, min_dim),
    )
    if len(faces) == 0:
        return {"found": False, "bbox": None, "conf": 0.0}
    # Pick the biggest detection
    fx, fy, fw, fh = max(faces, key=lambda f: f[2] * f[3])
    return {
        "found": True,
        "bbox": (int(fx), int(fy), int(fw), int(fh)),
        "conf": 1.0,
        "n_candidates": int(len(faces)),
    }


def m_haar_basic(img: np.ndarray) -> dict:
    return _haar_face(img, "haarcascade_frontalcatface.xml")


def m_haar_extended(img: np.ndarray) -> dict:
    return _haar_face(img, "haarcascade_frontalcatface_extended.xml")


def m_haar_with_extension(img: np.ndarray) -> dict:
    """Haar face → extend 40 % downward (niciBume's snout trick).

    The extension captures chin + mouth + paws / prey held in front
    of the muzzle. We use the basic cascade because the extended one
    tends to fire on full-body silhouettes too.
    """
    base = m_haar_basic(img)
    if not base["found"]:
        return base
    fx, fy, fw, fh = base["bbox"]
    extend = int(fh * 0.40)
    h, w = img.shape[:2]
    # Niceibume's recipe: extend horizontally by 20 %, vertically by 60 %
    # below the face. We mirror it here but clamp to image bounds.
    nx = max(0, fx - int(fw * 0.20))
    ny = max(0, fy - int(fh * 0.40))
    nx2 = min(w, fx + int(fw * 1.20))
    ny2 = min(h, fy + int(fh * 1.60))
    return {
        "found": True,
        "bbox": (nx, ny, nx2 - nx, ny2 - ny),
        "conf": 1.0,
        "n_candidates": base.get("n_candidates", 1),
    }


METHODS = {
    "heuristic_top45": (m_heuristic_top, (0, 255, 0)),         # green
    "haar_basic":      (m_haar_basic, (0, 200, 255)),          # orange
    "haar_extended":   (m_haar_extended, (255, 200, 0)),       # cyan
    "haar_extended_box": (m_haar_with_extension, (255, 0, 255)),  # magenta
}


# ── Sample selection ────────────────────────────────────────────────

def load_sample(crops_dir: Path, n_controls: int, seed: int = 42):
    """Return list of (image_path, label_dict)."""
    rows = list(csv.DictReader(open(crops_dir / "_index.csv")))
    cat_rows = [r for r in rows
                if r["human_subject"] == "cat"
                and r["human_direction"] == "entering"
                and r["body_path"]]
    prey_rows = [r for r in cat_rows if r["effective_prey"] == "1"]
    nonprey_rows = [r for r in cat_rows if r["effective_prey"] == "0"]
    rng = random.Random(seed)
    rng.shuffle(nonprey_rows)
    controls = nonprey_rows[:n_controls]
    log.info("Selected %d prey + %d no-prey crops for the bench",
             len(prey_rows), len(controls))
    return prey_rows + controls


# ── Dashboard ────────────────────────────────────────────────────────

DASH_CSS = """
body{background:#1a1a1a;color:#eee;font-family:-apple-system,monospace;margin:0;padding:16px;}
h1{margin:0 0 8px 0;font-size:18px;}
h2{font-size:14px;margin:16px 0 6px 0;border-bottom:1px solid #444;padding-bottom:4px;}
.stat{display:inline-block;background:#262626;padding:6px 12px;border-radius:6px;margin:4px 6px 4px 0;}
.stat b{color:#0fa;}
table{border-collapse:collapse;}
th,td{padding:2px;border:1px solid #333;text-align:center;font-size:10px;}
th{position:sticky;top:0;background:#222;z-index:10;}
img{width:120px;height:120px;display:block;}
.prey-row{background:#330;}
.tag{padding:1px 4px;border-radius:2px;font-size:9px;background:#888;color:#000;}
.tag.prey{background:#f88;color:#400;}
.tag.nope{background:#444;color:#aaa;}
small{color:#888;font-size:9px;}
"""


def render_method_thumb(img: np.ndarray, result: dict, color: tuple,
                        method_name: str) -> np.ndarray:
    """Annotate a 224x224 crop with the predicted snout box."""
    out = img.copy()
    if result["found"]:
        x, y, w, h = result["bbox"]
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
    else:
        # Cross-out
        h, w = out.shape[:2]
        cv2.line(out, (0, 0), (w, h), (0, 0, 200), 2)
        cv2.line(out, (w, 0), (0, h), (0, 0, 200), 2)
    return out


def write_dashboard(out_dir: Path, sample_rows: list, methods: list,
                    timings: dict) -> None:
    """Render an HTML grid: rows = images, columns = methods."""
    pieces = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<title>Snout localiser bench</title>",
        f"<style>{DASH_CSS}</style></head><body>",
        "<h1>Snout localiser bench — body crops from yolo11x_rotcrop</h1>",
    ]

    # Inference-time stats
    pieces.append("<h2>Inference time (ms per body crop)</h2>")
    for m in methods:
        ms = timings.get(m, [])
        if not ms:
            continue
        pieces.append(
            f"<div class='stat'>{m}: <b>{np.mean(ms):.1f}</b> ms "
            f"(p95 {np.percentile(ms, 95):.1f})</div>"
        )

    # Found-rate stats by prey/no-prey
    pieces.append("<h2>Snout 'found' rate</h2>")
    pieces.append("<table><tr><th>method</th>"
                  "<th>prey: found / total</th>"
                  "<th>no-prey: found / total</th></tr>")
    # Build found stats lazily
    # (we read them back from the rows below)
    pieces.append("</table>")

    # Big image table
    pieces.append("<h2>Per-image results</h2>")
    pieces.append("<table>")
    pieces.append("<tr><th>burst / frame</th><th>label</th>")
    for m in methods:
        pieces.append(f"<th>{m}</th>")
    pieces.append("</tr>")
    for row in sample_rows:
        bid = row["burst_id"]
        fidx = int(row["frame_idx"])
        is_prey = row["effective_prey"] == "1"
        row_class = "prey-row" if is_prey else ""
        tag = "<span class='tag prey'>PREY</span>" if is_prey else \
              "<span class='tag nope'>no</span>"
        pieces.append(f"<tr class='{row_class}'>")
        pieces.append(f"<td><small>{bid}<br>f{fidx:02d}</small></td>")
        pieces.append(f"<td>{tag}</td>")
        for m in methods:
            fname = f"{bid}_f{fidx:02d}_{m}.jpg"
            pieces.append(f"<td><img src='thumbs/{fname}'></td>")
        pieces.append("</tr>")
    pieces.append("</table>")
    pieces.append("</body></html>")

    dash = out_dir / "dashboard.html"
    tmp = dash.with_suffix(".html.tmp")
    tmp.write_text("".join(pieces))
    tmp.replace(dash)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default="crops_yolo11x_rotcrop",
                    help="Subdir of dataset/ containing _index.csv + body JPGs")
    ap.add_argument("--controls", type=int, default=40,
                    help="Random no-prey cat crops to include as controls")
    ap.add_argument("--methods", default=",".join(METHODS.keys()))
    args = ap.parse_args()

    crops_dir = DATASET / args.crops
    if not (crops_dir / "_index.csv").exists():
        raise SystemExit(f"{crops_dir} has no _index.csv")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "thumbs").mkdir(exist_ok=True)

    methods = [m.strip() for m in args.methods.split(",") if m.strip()]
    for m in methods:
        if m not in METHODS:
            raise SystemExit(f"Unknown method {m!r}, choices: {list(METHODS)}")

    sample = load_sample(crops_dir, args.controls)
    log.info("Processing %d crops × %d methods = %d evals",
             len(sample), len(methods), len(sample) * len(methods))

    per_image_rows: list[dict] = []
    found_counts: dict = defaultdict(lambda: {"prey_found": 0,
                                              "prey_total": 0,
                                              "noprey_found": 0,
                                              "noprey_total": 0})
    timings: dict = defaultdict(list)

    for row in sample:
        bid = row["burst_id"]
        fidx = int(row["frame_idx"])
        body_path = REPO / "dataset" / row["body_path"]
        img = cv2.imread(str(body_path))
        if img is None:
            log.warning("Could not read %s", body_path)
            continue
        is_prey = row["effective_prey"] == "1"

        for m in methods:
            fn, color = METHODS[m]
            t0 = time.perf_counter()
            try:
                res = fn(img)
            except Exception as e:
                log.warning("%s failed on %s f%02d: %s", m, bid, fidx, e)
                res = {"found": False, "bbox": None, "conf": 0.0}
            ms = (time.perf_counter() - t0) * 1000
            timings[m].append(ms)

            key = "prey" if is_prey else "noprey"
            found_counts[m][f"{key}_total"] += 1
            if res["found"]:
                found_counts[m][f"{key}_found"] += 1

            per_image_rows.append({
                "burst_id": bid,
                "frame_idx": fidx,
                "effective_prey": row["effective_prey"],
                "method": m,
                "found": int(res["found"]),
                "bbox_x": res["bbox"][0] if res["bbox"] else "",
                "bbox_y": res["bbox"][1] if res["bbox"] else "",
                "bbox_w": res["bbox"][2] if res["bbox"] else "",
                "bbox_h": res["bbox"][3] if res["bbox"] else "",
                "ms": round(ms, 2),
            })

            thumb = render_method_thumb(img, res, color, m)
            thumb_name = f"{bid}_f{fidx:02d}_{m}.jpg"
            cv2.imwrite(str(OUT / "thumbs" / thumb_name), thumb,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])

    # CSVs
    with open(OUT / "per_image.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(per_image_rows[0].keys()))
        w.writeheader()
        w.writerows(per_image_rows)

    with open(OUT / "summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["method", "mean_ms", "p95_ms",
                    "prey_found", "prey_total", "prey_rate",
                    "noprey_found", "noprey_total", "noprey_rate"])
        for m in methods:
            ms = timings[m]
            c = found_counts[m]
            w.writerow([
                m,
                f"{np.mean(ms):.2f}" if ms else "",
                f"{np.percentile(ms, 95):.2f}" if ms else "",
                c["prey_found"], c["prey_total"],
                f"{c['prey_found']/max(c['prey_total'],1):.2f}",
                c["noprey_found"], c["noprey_total"],
                f"{c['noprey_found']/max(c['noprey_total'],1):.2f}",
            ])

    # Print summary to stdout
    print()
    print(f"{'method':24s} {'ms':>6s} {'prey':>8s} {'no-prey':>8s}")
    print("-" * 50)
    for m in methods:
        ms = timings[m]
        c = found_counts[m]
        print(f"{m:24s} {np.mean(ms):>6.1f} "
              f"{c['prey_found']:>3d}/{c['prey_total']:<3d} "
              f"{c['noprey_found']:>3d}/{c['noprey_total']:<3d}")

    write_dashboard(OUT, sample, methods, timings)
    log.info("Dashboard: file://%s/dashboard.html", OUT)


if __name__ == "__main__":
    main()
