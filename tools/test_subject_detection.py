#!/usr/bin/env python3
"""Evaluate YOLO11n (cat body) and Haar cat-face cascade on captured bursts.

Goal: before retraining the prey classifier, find out whether we can reliably
locate the CAT (or its FACE) in our captures. If neither stage can find the
cat, cropping to the cat region won't help — we'd need a different approach.

Outputs:
  models/subject_detection_eval/
    summary.csv              — one row per burst with detection stats
    per_frame.csv            — one row per frame
    aggregate.txt            — printable aggregate stats
    contact_sheets/<burst>.jpg — 5x2 thumbnail grid with overlays
    burst_index.html         — clickable thumbnail index of all bursts

Stratified sample by default (override with --all-bursts):
  - All prey-positive cat bursts
  - 30 random no-prey cat bursts (day)
  - 15 random no-prey cat bursts (night)
  - 10 random non-cat bursts (empty/human) as controls

Usage:
  cd Mazge && uv run python tools/test_subject_detection.py
  uv run python tools/test_subject_detection.py --last5    # only frames f05-f09
  uv run python tools/test_subject_detection.py --all-bursts
"""
from __future__ import annotations

import argparse
import csv
import logging
import random
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).parent))
from prey_analyzer import YOLODetector, CatFaceDetector, CAT_CLASS_ID  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("subj_eval")

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
OUT = REPO / "models" / "subject_detection_eval"


def load_bursts_csv() -> list[dict]:
    with open(REPO / "dataset" / "bursts.csv") as f:
        return list(csv.DictReader(f))


def load_manifest() -> list[dict]:
    with open(REPO / "dataset" / "manifest.csv") as f:
        return list(csv.DictReader(f))


def pick_sample(bursts: list[dict], all_bursts: bool, seed: int = 42) -> list[dict]:
    rng = random.Random(seed)
    have_jpg = [b for b in bursts if b["has_jpgs"] == "1"]
    if all_bursts:
        return have_jpg
    prey_cat = [b for b in have_jpg
                if b["human_prey"] == "1" and b["human_subject"] == "cat"]
    no_prey_cat = [b for b in have_jpg
                   if b["human_prey"] == "0" and b["human_subject"] == "cat"]
    non_cat = [b for b in have_jpg
               if b["human_subject"] in {"empty", "human", "other"}]
    rng.shuffle(no_prey_cat)
    rng.shuffle(non_cat)

    # Night = bursts where ANY frame is flagged night (we approximate by
    # using captured hour in the burst_id: 22:00-06:00 → night).
    def is_night(b):
        try:
            hour = int(b["burst_id"][9:11])
            return hour >= 22 or hour < 6
        except (ValueError, IndexError):
            return False

    no_prey_day = [b for b in no_prey_cat if not is_night(b)]
    no_prey_night = [b for b in no_prey_cat if is_night(b)]

    sample = (
        prey_cat
        + no_prey_day[:30]
        + no_prey_night[:15]
        + non_cat[:10]
    )
    # Deduplicate while keeping order
    seen = set()
    out = []
    for b in sample:
        if b["burst_id"] not in seen:
            out.append(b)
            seen.add(b["burst_id"])
    return out


def is_night_frame(row: dict) -> bool:
    return row.get("night", "0") == "1"


def annotate(img_bgr: np.ndarray, yolo_dets: list[dict],
             haar_faces: list[tuple[int, int, int, int]]) -> Image.Image:
    img = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    for d in yolo_dets:
        if d["class_id"] != CAT_CLASS_ID:
            continue
        x, y, w, h = d["x"], d["y"], d["w"], d["h"]
        draw.rectangle([x, y, x + w, y + h], outline="lime", width=2)
        draw.text((x + 2, max(0, y - 11)),
                  f"YOLO {d['confidence']:.2f}", fill="lime")
    for fx, fy, fw, fh in haar_faces:
        draw.rectangle([fx, fy, fx + fw, fy + fh], outline="cyan", width=2)
        draw.text((fx + 2, max(0, fy - 11)), "Haar face", fill="cyan")
    return img


def make_contact_sheet(burst_id: str, frames: list[Image.Image],
                       stats: list[dict], human_label: str) -> Image.Image:
    cols, rows = 5, 2
    tw, th = 240, 180
    pad = 2
    label_h = 26
    sheet_w = cols * (tw + pad) + pad
    sheet_h = rows * (th + label_h + pad) + pad + 24
    sheet = Image.new("RGB", (sheet_w, sheet_h), (24, 24, 24))
    draw = ImageDraw.Draw(sheet)
    draw.text((4, 4), f"{burst_id}  [{human_label}]", fill="white")
    for i, (img, s) in enumerate(zip(frames, stats)):
        col, row = i % cols, i // cols
        x = pad + col * (tw + pad)
        y = 24 + pad + row * (th + label_h + pad)
        thumb = img.resize((tw, th), Image.LANCZOS)
        sheet.paste(thumb, (x, y))
        tag = f"f{s['frame_idx']:02d}"
        if s["yolo_cat"]:
            tag += f"  YOLO={s['yolo_max_conf']:.2f}"
        if s["haar_face"]:
            tag += f"  Haar"
        if not s["yolo_cat"] and not s["haar_face"]:
            tag += "  ---"
        color = "lime" if s["yolo_cat"] else ("cyan" if s["haar_face"] else "gray")
        draw.text((x + 2, y + th + 2), tag, fill=color)
    return sheet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--last5", action="store_true",
                    help="Only process frames f05-f09 (cat presumably closer)")
    ap.add_argument("--all-bursts", action="store_true",
                    help="Process every burst with JPGs (slow)")
    ap.add_argument("--save-frames", action="store_true",
                    help="Also save individual annotated JPGs (lots of files)")
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "contact_sheets").mkdir(exist_ok=True)
    if args.save_frames:
        (OUT / "frames").mkdir(exist_ok=True)

    bursts = load_bursts_csv()
    manifest = load_manifest()
    night_lookup = {(r["burst_id"], int(r["frame_idx"])): r["night"] == "1"
                    for r in manifest}

    sample = pick_sample(bursts, args.all_bursts)
    log.info("Sample: %d bursts (last5=%s, all=%s)",
             len(sample), args.last5, args.all_bursts)

    log.info("Loading models...")
    yolo = YOLODetector()
    face_det = CatFaceDetector()

    per_frame: list[dict] = []
    per_burst: list[dict] = []
    t0 = time.time()
    yolo_ms: list[float] = []
    haar_ms: list[float] = []

    for bi, b in enumerate(sample, 1):
        burst_id = b["burst_id"]
        bdir = SD / burst_id
        if not bdir.exists():
            log.warning("Missing dir %s", bdir)
            continue
        jpgs = sorted(bdir.glob("f*.jpg"))
        if args.last5:
            jpgs = [p for p in jpgs if int(p.stem.split("_")[0][1:]) >= 5]
        if not jpgs:
            continue

        thumbs = []
        stats = []
        burst_yolo_frames = 0
        burst_haar_frames = 0
        burst_max_conf = 0.0
        burst_max_box_frac = 0.0
        for jp in jpgs:
            fidx = int(jp.stem.split("_")[0][1:])
            img_bgr = cv2.imread(str(jp))
            if img_bgr is None:
                continue
            h, w = img_bgr.shape[:2]
            img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

            t_y = time.perf_counter()
            raw = yolo.detect(img_rgb)
            dets = yolo.detect_to_image_coords(img_rgb, raw)
            yolo_ms.append((time.perf_counter() - t_y) * 1000)

            t_h = time.perf_counter()
            haar = face_det.detect_face(img_bgr)
            haar_ms.append((time.perf_counter() - t_h) * 1000)

            cat_dets = [d for d in dets if d["class_id"] == CAT_CLASS_ID]
            yolo_max = max((d["confidence"] for d in cat_dets), default=0.0)
            best_box = max(cat_dets,
                           key=lambda d: d["w"] * d["h"],
                           default=None)
            box_frac = (best_box["w"] * best_box["h"] / (w * h)
                        if best_box else 0.0)

            if cat_dets:
                burst_yolo_frames += 1
                burst_max_conf = max(burst_max_conf, yolo_max)
                burst_max_box_frac = max(burst_max_box_frac, box_frac)
            if haar:
                burst_haar_frames += 1

            per_frame.append({
                "burst_id": burst_id,
                "frame_idx": fidx,
                "night": int(night_lookup.get((burst_id, fidx), False)),
                "yolo_cat": int(bool(cat_dets)),
                "yolo_max_conf": round(yolo_max, 3),
                "yolo_box_frac": round(box_frac, 4),
                "haar_face": int(bool(haar)),
                "haar_n_faces": len(haar),
                "human_subject": b["human_subject"],
                "human_prey": b["human_prey"],
            })

            ann = annotate(img_bgr, dets, haar)
            thumbs.append(ann)
            stats.append({
                "frame_idx": fidx,
                "yolo_cat": bool(cat_dets),
                "yolo_max_conf": yolo_max,
                "haar_face": bool(haar),
            })
            if args.save_frames and (cat_dets or haar):
                ann.save(OUT / "frames" / f"{burst_id}_f{fidx:02d}.jpg",
                         quality=85)

        # Pad to 10 thumbs for a uniform contact sheet
        while len(thumbs) < 10:
            blank = Image.new("RGB", (640, 480), (0, 0, 0))
            thumbs.append(blank)
            stats.append({"frame_idx": -1, "yolo_cat": False,
                          "yolo_max_conf": 0.0, "haar_face": False})

        human_label = (
            f"prey={b['human_prey'] or '?'}  "
            f"subj={b['human_subject'] or '?'}  "
            f"dir={b['human_direction'] or '?'}"
        )
        sheet = make_contact_sheet(burst_id, thumbs[:10], stats[:10],
                                   human_label)
        sheet.save(OUT / "contact_sheets" / f"{burst_id}.jpg", quality=85)

        per_burst.append({
            "burst_id": burst_id,
            "human_prey": b["human_prey"],
            "human_subject": b["human_subject"],
            "human_direction": b["human_direction"],
            "n_frames": len(jpgs),
            "yolo_frames": burst_yolo_frames,
            "haar_frames": burst_haar_frames,
            "yolo_any": int(burst_yolo_frames > 0),
            "haar_any": int(burst_haar_frames > 0),
            "yolo_or_haar_any": int(burst_yolo_frames > 0 or burst_haar_frames > 0),
            "burst_max_conf": round(burst_max_conf, 3),
            "burst_max_box_frac": round(burst_max_box_frac, 4),
        })

        if bi % 10 == 0 or bi == len(sample):
            elapsed = time.time() - t0
            log.info("[%d/%d] %s yolo=%d/%d haar=%d/%d (%.1fs)",
                     bi, len(sample), burst_id,
                     burst_yolo_frames, len(jpgs),
                     burst_haar_frames, len(jpgs), elapsed)

    # ── Write CSVs ────────────────────────────────────────────────────
    with open(OUT / "per_frame.csv", "w", newline="") as f:
        if per_frame:
            w = csv.DictWriter(f, fieldnames=list(per_frame[0].keys()))
            w.writeheader()
            w.writerows(per_frame)
    with open(OUT / "summary.csv", "w", newline="") as f:
        if per_burst:
            w = csv.DictWriter(f, fieldnames=list(per_burst[0].keys()))
            w.writeheader()
            w.writerows(per_burst)

    # ── Aggregate stats ───────────────────────────────────────────────
    lines: list[str] = []
    def out(s=""):
        lines.append(s)
        print(s)

    out("=" * 72)
    out(f"YOLO11n + Haar cat-face evaluation  ({len(per_burst)} bursts, "
        f"{len(per_frame)} frames)")
    out("=" * 72)
    out(f"YOLO mean inference: {np.mean(yolo_ms):.1f} ms  "
        f"(p50={np.percentile(yolo_ms, 50):.1f}, "
        f"p95={np.percentile(yolo_ms, 95):.1f})")
    out(f"Haar mean inference: {np.mean(haar_ms):.1f} ms  "
        f"(p50={np.percentile(haar_ms, 50):.1f}, "
        f"p95={np.percentile(haar_ms, 95):.1f})")
    out()

    # Per-frame breakdown by human_subject
    out("Per-FRAME detection rate by human_subject:")
    out(f"  {'subject':10s}  {'N':>5s}  {'YOLO%':>7s}  {'Haar%':>7s}  {'either%':>8s}")
    by_subj = defaultdict(list)
    for r in per_frame:
        by_subj[r["human_subject"] or "?"].append(r)
    for subj in ("cat", "empty", "human", "other", "?"):
        rs = by_subj.get(subj, [])
        if not rs:
            continue
        n = len(rs)
        yolo_pct = 100 * sum(r["yolo_cat"] for r in rs) / n
        haar_pct = 100 * sum(r["haar_face"] for r in rs) / n
        either = 100 * sum(r["yolo_cat"] or r["haar_face"] for r in rs) / n
        out(f"  {subj:10s}  {n:>5d}  {yolo_pct:>6.1f}%  {haar_pct:>6.1f}%  {either:>7.1f}%")
    out()

    # Cat frames — day vs night
    out("CAT frames split by day/night:")
    out(f"  {'cond':10s}  {'N':>5s}  {'YOLO%':>7s}  {'Haar%':>7s}  {'either%':>8s}")
    cat_rs = [r for r in per_frame if r["human_subject"] == "cat"]
    for cond, mask in (("day", lambda r: r["night"] == 0),
                       ("night", lambda r: r["night"] == 1)):
        rs = [r for r in cat_rs if mask(r)]
        if not rs:
            continue
        n = len(rs)
        yolo_pct = 100 * sum(r["yolo_cat"] for r in rs) / n
        haar_pct = 100 * sum(r["haar_face"] for r in rs) / n
        either = 100 * sum(r["yolo_cat"] or r["haar_face"] for r in rs) / n
        out(f"  {cond:10s}  {n:>5d}  {yolo_pct:>6.1f}%  {haar_pct:>6.1f}%  {either:>7.1f}%")
    out()

    # Cat frames — by frame index (closer to end = cat closer to camera)
    out("CAT frames per-frame-index (does later frame help?):")
    out(f"  {'fidx':4s}  {'N':>4s}  {'YOLO%':>6s}  {'Haar%':>6s}  {'either%':>8s}  "
        f"{'mean_conf':>9s}  {'mean_box':>9s}")
    by_idx = defaultdict(list)
    for r in cat_rs:
        by_idx[r["frame_idx"]].append(r)
    for fidx in sorted(by_idx):
        rs = by_idx[fidx]
        n = len(rs)
        yolo_pct = 100 * sum(r["yolo_cat"] for r in rs) / n
        haar_pct = 100 * sum(r["haar_face"] for r in rs) / n
        either = 100 * sum(r["yolo_cat"] or r["haar_face"] for r in rs) / n
        mean_conf = np.mean([r["yolo_max_conf"] for r in rs])
        mean_box = np.mean([r["yolo_box_frac"] for r in rs])
        out(f"  {fidx:>4d}  {n:>4d}  {yolo_pct:>5.1f}%  {haar_pct:>5.1f}%  "
            f"{either:>7.1f}%  {mean_conf:>9.3f}  {mean_box:>9.4f}")
    out()

    # Per-burst (any-frame) detection rate
    out("Per-BURST: did we find the cat in at least one frame?")
    out(f"  {'subject':10s}  {'prey':6s}  {'N':>4s}  {'YOLO_any%':>9s}  "
        f"{'Haar_any%':>9s}  {'either%':>8s}")
    for subj in ("cat", "empty", "human", "other"):
        for prey in ("1", "0", "unclear", ""):
            rs = [r for r in per_burst
                  if r["human_subject"] == subj and r["human_prey"] == prey]
            if not rs:
                continue
            n = len(rs)
            yolo_pct = 100 * sum(r["yolo_any"] for r in rs) / n
            haar_pct = 100 * sum(r["haar_any"] for r in rs) / n
            either = 100 * sum(r["yolo_or_haar_any"] for r in rs) / n
            label_prey = prey or "?"
            out(f"  {subj:10s}  {label_prey:6s}  {n:>4d}  {yolo_pct:>8.1f}%  "
                f"{haar_pct:>8.1f}%  {either:>7.1f}%")
    out()

    # Bounding box size on cat bursts
    if cat_rs:
        cat_with_yolo = [r for r in cat_rs if r["yolo_cat"]]
        if cat_with_yolo:
            box_fracs = [r["yolo_box_frac"] for r in cat_with_yolo]
            out(f"YOLO cat bbox area (% of frame) on cat hits: "
                f"mean={np.mean(box_fracs)*100:.1f}%, "
                f"p50={np.percentile(box_fracs, 50)*100:.1f}%, "
                f"p95={np.percentile(box_fracs, 95)*100:.1f}%")

    (OUT / "aggregate.txt").write_text("\n".join(lines) + "\n")

    # ── HTML index ────────────────────────────────────────────────────
    html_lines = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<title>Subject detection eval</title>",
        "<style>body{background:#222;color:#eee;font-family:monospace;}",
        "a{color:#9cf;text-decoration:none}",
        "img{width:100%;display:block;margin-bottom:4px}",
        ".burst{display:inline-block;width:520px;margin:8px;vertical-align:top;}",
        ".tag{padding:1px 6px;border-radius:3px;margin-right:4px;}",
        ".tag.prey{background:#a33}.tag.cat{background:#363}",
        ".tag.empty{background:#555}.tag.night{background:#225}",
        "</style></head><body>",
        f"<pre>{chr(10).join(lines)}</pre><hr>",
    ]
    per_burst_sorted = sorted(
        per_burst,
        key=lambda r: (-int(r["human_prey"] == "1"),
                       r["human_subject"] != "cat",
                       -r["yolo_or_haar_any"]),
    )
    for r in per_burst_sorted:
        tags = []
        if r["human_prey"] == "1":
            tags.append("<span class='tag prey'>PREY</span>")
        if r["human_subject"]:
            tags.append(f"<span class='tag {r['human_subject']}'>"
                        f"{r['human_subject']}</span>")
        html_lines.append("<div class='burst'>")
        html_lines.append(f"<a href='contact_sheets/{r['burst_id']}.jpg'>"
                          f"<img src='contact_sheets/{r['burst_id']}.jpg'></a>")
        html_lines.append(
            f"{''.join(tags)} {r['burst_id']} — "
            f"YOLO {r['yolo_frames']}/{r['n_frames']} (max={r['burst_max_conf']:.2f}, "
            f"box={r['burst_max_box_frac']*100:.1f}%), "
            f"Haar {r['haar_frames']}/{r['n_frames']}"
        )
        html_lines.append("</div>")
    html_lines.append("</body></html>")
    (OUT / "burst_index.html").write_text("\n".join(html_lines))

    out()
    out(f"Wrote: {OUT}/aggregate.txt")
    out(f"       {OUT}/per_frame.csv  ({len(per_frame)} rows)")
    out(f"       {OUT}/summary.csv    ({len(per_burst)} rows)")
    out(f"       {OUT}/contact_sheets/  ({len(per_burst)} sheets)")
    out(f"       {OUT}/burst_index.html")
    out(f"Done in {time.time() - t0:.1f}s.")


if __name__ == "__main__":
    main()
