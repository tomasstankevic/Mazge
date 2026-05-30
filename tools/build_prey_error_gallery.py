#!/usr/bin/env python3
"""Build a visual error gallery for prey_v2 vs cloud API.

Reads the existing burst-level evaluation outputs, recomputes per-frame prey_v2
scores for review, and writes a standalone HTML report with:
  - overall method comparison
  - focus-method breakdown by split/subject/direction/cat/daypart
  - false-positive / false-negative galleries
  - focus-vs-cloud disagreement galleries

Usage:
  cd Mazge
  /Users/ruta/Tomas/repos/Mazge/.venv/bin/python tools/build_prey_error_gallery.py
  /Users/ruta/Tomas/repos/Mazge/.venv/bin/python tools/build_prey_error_gallery.py \
      --focus-method v2_top3mean --focus-threshold 0.7
"""
from __future__ import annotations

import argparse
import csv
import html
import json
import os
from collections import defaultdict
from datetime import datetime
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

import torch
from PIL import Image
from torchvision import transforms

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_RUN = REPO / "models" / "prey_v2" / "bodyA"
DEFAULT_CROPS = DATASET / "crops_yolo11x_rotcrop"
IMG_SIZE = 224
SUMMARY_METHODS = [
    ("cloud_api", None),
    ("v2_max", 0.50),
    ("v2_top3mean", 0.70),
    ("v2_top3mean", 0.80),
]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", default=str(DEFAULT_RUN))
    ap.add_argument("--ckpt", default="best_burst_f1")
    ap.add_argument("--crops", default=str(DEFAULT_CROPS))
    ap.add_argument("--device", default="auto")
    ap.add_argument("--focus-method", choices=("cloud_api", "v2_max", "v2_top3mean"),
                    default="v2_top3mean")
    ap.add_argument("--focus-threshold", type=float, default=0.70)
    ap.add_argument("--limit", type=int, default=32,
                    help="max bursts to show per gallery section")
    return ap.parse_args()


class PreyClassifier(torch.nn.Module):
    def __init__(self):
        super().__init__()
        from torchvision.models import efficientnet_b0, EfficientNet_B0_Weights
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        in_features = backbone.classifier[1].in_features
        backbone.classifier = torch.nn.Sequential(
            torch.nn.Dropout(0.2),
            torch.nn.Linear(in_features, 1),
        )
        self.model = backbone

    def forward(self, x):
        return self.model(x).squeeze(-1)


@torch.no_grad()
def predict_probs(model, device, image_paths: list[Path]) -> list[float]:
    if not image_paths:
        return []
    tf = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])
    imgs = torch.stack([tf(Image.open(path).convert("RGB")) for path in image_paths])
    logits = model(imgs.to(device))
    return [float(x) for x in torch.sigmoid(logits).cpu().numpy()]


def load_model(run_dir: Path, ckpt_name: str, device: torch.device):
    ckpt_path = run_dir / f"{ckpt_name}.pt"
    if not ckpt_path.exists():
        raise SystemExit(f"No checkpoint at {ckpt_path}")
    model = PreyClassifier().to(device)
    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model, ckpt


def load_csv(path: Path) -> list[dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def normalize_float(value: str | float | int | None, default: float = 0.0) -> float:
    if value in (None, ""):
        return default
    return float(value)


def daypart_for_burst(burst_id: str) -> str:
    try:
        stamp = burst_id.split("_gen", 1)[0]
        dt = datetime.strptime(stamp, "%Y%m%d_%H%M%S")
    except ValueError:
        return "unknown"
    return "day" if 7 <= dt.hour < 19 else "night"


def method_label(method: str, threshold: float | None) -> str:
    if method == "cloud_api":
        return "cloud_api"
    return f"{method}@{threshold:.2f}"


def burst_score(row: dict, method: str) -> float | None:
    if method == "cloud_api":
        v = row["cloud_api"]
        return None if v not in ("0", "1") else float(v)
    if method == "v2_max":
        return normalize_float(row["v2_max_p"])
    if method == "v2_top3mean":
        return normalize_float(row["v2_top3_mean"])
    raise ValueError(method)


def burst_pred(row: dict, method: str, threshold: float | None) -> int | None:
    if method == "cloud_api":
        v = row["cloud_api"]
        return None if v not in ("0", "1") else int(v)
    score = burst_score(row, method)
    if score is None or threshold is None:
        return None
    return 1 if score >= threshold else 0


def confusion(rows: list[dict], method: str, threshold: float | None) -> dict[str, float | int]:
    tp = fp = fn = tn = 0
    for row in rows:
        yt = int(row["effective_human"])
        yp = burst_pred(row, method, threshold)
        if yp is None:
            continue
        if yt == 1 and yp == 1:
            tp += 1
        elif yt == 1 and yp == 0:
            fn += 1
        elif yt == 0 and yp == 1:
            fp += 1
        else:
            tn += 1
    n = tp + fp + fn + tn
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-8)
    return {
        "tp": tp, "fp": fp, "fn": fn, "tn": tn, "n": n,
        "precision": precision, "recall": recall, "f1": f1,
    }


def bucket_rows(rows: list[dict], key: str) -> list[tuple[str, dict[str, float | int]]]:
    groups: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        value = row.get(key, "") or "?"
        groups[value].append(row)
    items = []
    for value, bucket in groups.items():
        stats = confusion(bucket, FOCUS_METHOD, FOCUS_THRESHOLD)
        stats["value"] = value
        items.append((value, stats))
    items.sort(key=lambda item: (-(item[1]["fp"] + item[1]["fn"]), -item[1]["n"], item[0]))
    return items


def html_rel(out_dir: Path, target: Path) -> str:
    return os.path.relpath(target, out_dir).replace(os.sep, "/")


def build_frame_rows(crop_rows: list[dict], out_dir: Path, probs_by_frame: dict[int, float]) -> list[dict]:
    rows = []
    for crop in sorted(crop_rows, key=lambda row: int(row["frame_idx"])):
        frame_idx = int(crop["frame_idx"])
        image_path = REPO / crop["image_id"]
        body_path = DATASET / crop["body_path"] if crop.get("body_path") else None
        snout_path = DATASET / crop["snout_path"] if crop.get("snout_path") else None
        rows.append({
            "frame_idx": frame_idx,
            "image_href": html_rel(out_dir, image_path) if image_path.exists() else "",
            "body_href": html_rel(out_dir, body_path) if body_path and body_path.exists() else "",
            "snout_href": html_rel(out_dir, snout_path) if snout_path and snout_path.exists() else "",
            "cat_found": crop.get("cat_found", "0"),
            "cat_conf": normalize_float(crop.get("cat_conf", 0.0)),
            "prob": probs_by_frame.get(frame_idx),
        })
    return rows


def render_frame_strip(frames: list[dict]) -> str:
    cells = []
    for frame in frames:
        if frame["body_href"]:
            img_href = frame["body_href"]
            img_class = "body"
        elif frame["image_href"]:
            img_href = frame["image_href"]
            img_class = "raw"
        else:
            cells.append(f'<div class="frame missing">f{frame["frame_idx"]:02d}</div>')
            continue
        badges = [f'f{frame["frame_idx"]:02d}']
        if frame["prob"] is not None:
            badges.append(f'p={frame["prob"]:.2f}')
        if frame["cat_found"] == "1":
            badges.append(f'cat={frame["cat_conf"]:.2f}')
        thumb = [
            f'<a class="frame {img_class}" href="{html.escape(img_href)}" target="_blank">',
            f'  <img loading="lazy" src="{html.escape(img_href)}" alt="f{frame["frame_idx"]:02d}">',
            f'  <span class="lab">{" | ".join(html.escape(b) for b in badges)}</span>',
        ]
        if frame["image_href"] and frame["body_href"] and frame["image_href"] != frame["body_href"]:
            thumb.append(
                f'  <span class="meta-links"><a href="{html.escape(frame["image_href"])}" target="_blank">raw</a></span>'
            )
        thumb.append("</a>")
        cells.append("\n".join(thumb))
    return "\n".join(cells)


def score_sort_key(row: dict, method: str) -> tuple[float, str]:
    score = burst_score(row, method)
    return (-(score if score is not None else -1.0), row["burst_id"])


def render_cards(title: str, rows: list[dict], crop_index: dict[str, list[dict]],
                 out_dir: Path, limit: int) -> str:
    if not rows:
        return f"<section><h2>{html.escape(title)}</h2><p class=\"empty\">None.</p></section>"
    parts = [f"<section><h2>{html.escape(title)}</h2><div class=\"cards\">"]
    for row in rows[:limit]:
        frames = build_frame_rows(crop_index[row["burst_id"]], out_dir, row["frame_probs"])
        score_text = "--" if row["focus_score"] is None else f'{row["focus_score"]:.3f}'
        parts.append(
            "<article class=\"card\">"
            f"<h3>{html.escape(row['burst_id'])}</h3>"
            f"<div class=\"meta\">split={html.escape(row['split_train'] or '?')} | "
            f"subject={html.escape(row['human_subject'] or '?')} | "
            f"direction={html.escape(row['human_direction'] or '?')} | "
            f"cat={html.escape(row['cat_id'] or '?')} | "
            f"daypart={html.escape(row['daypart'])}</div>"
            f"<div class=\"meta\">truth={row['effective_human']} | cloud={html.escape(row['cloud_api'] or '?')} | "
            f"v2_max={normalize_float(row['v2_max_p']):.3f} | "
            f"v2_top3={normalize_float(row['v2_top3_mean']):.3f} | "
            f"focus={html.escape(FOCUS_LABEL)} score={score_text}</div>"
            f"<div class=\"grid\">{render_frame_strip(frames)}</div>"
            "</article>"
        )
    parts.append("</div></section>")
    return "\n".join(parts)


def render_method_table(rows: list[dict]) -> str:
    head = (
        "<table><thead><tr><th>method</th><th>TP</th><th>FP</th><th>FN</th><th>TN</th>"
        "<th>N</th><th>P</th><th>R</th><th>F1</th></tr></thead><tbody>"
    )
    body = []
    for method, threshold in SUMMARY_METHODS:
        stats = confusion(rows, method, threshold)
        body.append(
            "<tr>"
            f"<td>{html.escape(method_label(method, threshold))}</td>"
            f"<td>{stats['tp']}</td><td>{stats['fp']}</td><td>{stats['fn']}</td><td>{stats['tn']}</td>"
            f"<td>{stats['n']}</td><td>{stats['precision']:.2f}</td><td>{stats['recall']:.2f}</td><td>{stats['f1']:.2f}</td>"
            "</tr>"
        )
    return head + "\n".join(body) + "</tbody></table>"


def render_breakdown(title: str, rows: list[dict], key: str) -> str:
    items = bucket_rows(rows, key)
    if not items:
        return ""
    body = []
    for value, stats in items:
        body.append(
            "<tr>"
            f"<td>{html.escape(value)}</td><td>{stats['tp']}</td><td>{stats['fp']}</td>"
            f"<td>{stats['fn']}</td><td>{stats['tn']}</td><td>{stats['n']}</td>"
            f"<td>{stats['precision']:.2f}</td><td>{stats['recall']:.2f}</td><td>{stats['f1']:.2f}</td>"
            "</tr>"
        )
    return (
        f"<section><h2>{html.escape(title)}</h2>"
        "<table><thead><tr><th>bucket</th><th>TP</th><th>FP</th><th>FN</th><th>TN</th><th>N</th><th>P</th><th>R</th><th>F1</th>"
        "</tr></thead><tbody>" + "\n".join(body) + "</tbody></table></section>"
    )


def build_report(comparison_rows: list[dict], crop_index: dict[str, list[dict]], out_dir: Path,
                 focus_rows: dict[str, list[dict]], ckpt: dict, args: argparse.Namespace) -> str:
    focus_stats = confusion(comparison_rows, FOCUS_METHOD, FOCUS_THRESHOLD)
    cloud_stats = confusion(comparison_rows, "cloud_api", None)
    correct_vs_api = focus_rows["focus_correct_api_wrong"]
    api_correct_focus_wrong = focus_rows["api_correct_focus_wrong"]
    return f"""<!doctype html>
<html><head><meta charset=\"utf-8\">
<title>prey_v2 error gallery</title>
<style>
body {{ margin: 0; font: 14px/1.4 ui-sans-serif, -apple-system, BlinkMacSystemFont, sans-serif; background: #f3efe6; color: #1f1b16; }}
main {{ max-width: 1500px; margin: 0 auto; padding: 20px; }}
h1, h2, h3 {{ margin: 0 0 8px; }}
p {{ margin: 0 0 12px; }}
.hero {{ background: linear-gradient(135deg, #f7d98b, #e58f65 55%, #8b3b31); color: #20140f; padding: 18px 20px; border-radius: 18px; box-shadow: 0 10px 30px rgba(0,0,0,0.15); }}
.hero .stats {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin-top: 14px; }}
.hero .stat {{ background: rgba(255,255,255,0.5); border: 1px solid rgba(32,20,15,0.12); border-radius: 12px; padding: 10px 12px; }}
section {{ margin-top: 22px; }}
table {{ width: 100%; border-collapse: collapse; background: rgba(255,255,255,0.75); border-radius: 12px; overflow: hidden; }}
th, td {{ padding: 8px 10px; border-bottom: 1px solid #d9cdbd; text-align: left; }}
th {{ background: #efe4d2; }}
.cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap: 16px; }}
.card {{ background: #fffaf2; border: 1px solid #dfcfbb; border-radius: 14px; padding: 12px; box-shadow: 0 8px 24px rgba(64,36,16,0.08); }}
.meta {{ color: #6c5a49; font-size: 12px; margin-bottom: 6px; }}
.grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(110px, 1fr)); gap: 8px; }}
.frame {{ position: relative; display: block; background: #ddd3c3; border-radius: 10px; overflow: hidden; text-decoration: none; color: inherit; min-height: 110px; }}
.frame img {{ width: 100%; height: 110px; object-fit: cover; display: block; }}
.frame .lab {{ position: absolute; left: 0; right: 0; bottom: 0; background: rgba(24,20,18,0.78); color: #f6efe6; font-size: 11px; padding: 4px 5px; }}
.frame .meta-links {{ position: absolute; right: 6px; top: 6px; background: rgba(255,250,242,0.88); border-radius: 999px; padding: 2px 6px; font-size: 11px; }}
.frame .meta-links a {{ color: #7e3f2b; text-decoration: none; }}
.frame.missing {{ display: flex; align-items: center; justify-content: center; color: #8d7a67; font-size: 12px; }}
.empty {{ color: #6c5a49; font-style: italic; }}
.callout {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; }}
.callout > div {{ background: #fffaf2; border: 1px solid #dfcfbb; border-radius: 12px; padding: 10px 12px; }}
code {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }}
</style></head>
<body><main>
<div class=\"hero\">
  <h1>prey_v2 error gallery</h1>
  <p>Focus method: <b>{html.escape(FOCUS_LABEL)}</b>. Checkpoint: <code>{html.escape(args.ckpt)}.pt</code> from epoch {ckpt['epoch']}.</p>
  <div class=\"stats\">
    <div class=\"stat\"><div>focus precision</div><b>{focus_stats['precision']:.2f}</b></div>
    <div class=\"stat\"><div>focus recall</div><b>{focus_stats['recall']:.2f}</b></div>
    <div class=\"stat\"><div>focus FP / FN</div><b>{focus_stats['fp']} / {focus_stats['fn']}</b></div>
    <div class=\"stat\"><div>cloud FP / FN</div><b>{cloud_stats['fp']} / {cloud_stats['fn']}</b></div>
    <div class=\"stat\"><div>focus correct, cloud wrong</div><b>{len(correct_vs_api)}</b></div>
    <div class=\"stat\"><div>cloud correct, focus wrong</div><b>{len(api_correct_focus_wrong)}</b></div>
  </div>
</div>

<section>
  <h2>Overall method comparison</h2>
  {render_method_table(comparison_rows)}
</section>

<section>
  <h2>What to inspect first</h2>
  <div class=\"callout\">
    <div><b>False positives</b><br>{focus_stats['fp']} bursts where the focus method would block without prey.</div>
    <div><b>False negatives</b><br>{focus_stats['fn']} bursts where prey is present but the focus method would miss it.</div>
    <div><b>Disagreement wins</b><br>{len(correct_vs_api)} bursts where the focus method matches human labels and cloud does not.</div>
    <div><b>Disagreement losses</b><br>{len(api_correct_focus_wrong)} bursts where cloud is right and the focus method is not.</div>
  </div>
</section>

{render_breakdown(f'Focus breakdown by split ({FOCUS_LABEL})', comparison_rows, 'split_train')}
{render_breakdown(f'Focus breakdown by subject ({FOCUS_LABEL})', comparison_rows, 'human_subject')}
{render_breakdown(f'Focus breakdown by direction ({FOCUS_LABEL})', comparison_rows, 'human_direction')}
{render_breakdown(f'Focus breakdown by cat ({FOCUS_LABEL})', comparison_rows, 'cat_id')}
{render_breakdown(f'Focus breakdown by daypart ({FOCUS_LABEL})', comparison_rows, 'daypart')}

{render_cards(f'False positives for {FOCUS_LABEL}', focus_rows['fp'], crop_index, out_dir, args.limit)}
{render_cards(f'False negatives for {FOCUS_LABEL}', focus_rows['fn'], crop_index, out_dir, args.limit)}
{render_cards(f'{FOCUS_LABEL} correct, cloud wrong', focus_rows['focus_correct_api_wrong'], crop_index, out_dir, args.limit)}
{render_cards(f'Cloud correct, {FOCUS_LABEL} wrong', focus_rows['api_correct_focus_wrong'], crop_index, out_dir, args.limit)}

</main></body></html>
"""


if __name__ == "__main__":
    args = parse_args()
    run_dir = Path(args.run_dir)
    crops_dir = Path(args.crops)
    out_dir = run_dir / "vs_cloud_api"
    out_dir.mkdir(parents=True, exist_ok=True)

    global FOCUS_METHOD, FOCUS_THRESHOLD, FOCUS_LABEL
    FOCUS_METHOD = args.focus_method
    FOCUS_THRESHOLD = None if args.focus_method == "cloud_api" else args.focus_threshold
    FOCUS_LABEL = method_label(args.focus_method, FOCUS_THRESHOLD)

    device = (torch.device("mps") if args.device == "auto" and torch.backends.mps.is_available()
              else torch.device(args.device if args.device != "auto" else "cpu"))
    model, ckpt = load_model(run_dir, args.ckpt, device)

    comparison_rows = load_csv(out_dir / "per_burst.csv")
    crop_rows = load_csv(crops_dir / "_index.csv")
    crop_index: dict[str, list[dict]] = defaultdict(list)
    for crop in crop_rows:
        if crop.get("body_path"):
            crop_index[crop["burst_id"]].append(crop)

    for row in comparison_rows:
        row["cat_id"] = row.get("cat_id", "") or "?"
        row["daypart"] = daypart_for_burst(row["burst_id"])
        burst_crops = sorted(crop_index[row["burst_id"]], key=lambda item: int(item["frame_idx"]))
        frame_paths = [DATASET / crop["body_path"] for crop in burst_crops]
        probs = predict_probs(model, device, frame_paths)
        row["frame_probs"] = {int(crop["frame_idx"]): prob for crop, prob in zip(burst_crops, probs)}
        row["focus_score"] = burst_score(row, args.focus_method)

    focus_rows: dict[str, list[dict]] = {"fp": [], "fn": [], "focus_correct_api_wrong": [], "api_correct_focus_wrong": []}
    for row in comparison_rows:
        truth = int(row["effective_human"])
        focus_pred = burst_pred(row, args.focus_method, FOCUS_THRESHOLD)
        cloud_pred = burst_pred(row, "cloud_api", None)
        if focus_pred is not None:
            if focus_pred == 1 and truth == 0:
                focus_rows["fp"].append(row)
            elif focus_pred == 0 and truth == 1:
                focus_rows["fn"].append(row)
        if focus_pred is not None and cloud_pred is not None and focus_pred != cloud_pred:
            if focus_pred == truth:
                focus_rows["focus_correct_api_wrong"].append(row)
            elif cloud_pred == truth:
                focus_rows["api_correct_focus_wrong"].append(row)

    focus_rows["fp"].sort(key=lambda row: score_sort_key(row, args.focus_method))
    focus_rows["fn"].sort(key=lambda row: score_sort_key(row, args.focus_method))
    focus_rows["focus_correct_api_wrong"].sort(key=lambda row: score_sort_key(row, args.focus_method))
    focus_rows["api_correct_focus_wrong"].sort(key=lambda row: score_sort_key(row, args.focus_method))

    report_html = build_report(comparison_rows, crop_index, out_dir, focus_rows, ckpt, args)
    html_path = out_dir / f"error_gallery_{args.focus_method}_{str(args.focus_threshold).replace('.', 'p')}.html"
    html_path.write_text(report_html)

    summary = {
        "focus_method": args.focus_method,
        "focus_threshold": FOCUS_THRESHOLD,
        "focus_label": FOCUS_LABEL,
        "n_bursts": len(comparison_rows),
        "focus": confusion(comparison_rows, args.focus_method, FOCUS_THRESHOLD),
        "cloud": confusion(comparison_rows, "cloud_api", None),
        "focus_correct_api_wrong": len(focus_rows["focus_correct_api_wrong"]),
        "api_correct_focus_wrong": len(focus_rows["api_correct_focus_wrong"]),
        "html": html_path.name,
    }
    summary_path = out_dir / f"error_gallery_{args.focus_method}_{str(args.focus_threshold).replace('.', 'p')}.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
