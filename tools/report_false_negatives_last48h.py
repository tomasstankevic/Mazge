#!/usr/bin/env python3
"""Build false-negative report from firmware vs full-frame reanalysis (last 48h)."""

from __future__ import annotations

import json
import csv
from datetime import datetime, timedelta
from pathlib import Path


def is_timestamp_folder(name: str) -> bool:
    return len(name) >= 15 and name[8] == "_" and name[:8].isdigit() and name[9:15].isdigit()


def main() -> None:
    sd = Path("captures/sd")
    bursts_csv = Path("dataset/bursts.csv")
    cutoff = (datetime.now() - timedelta(hours=48)).strftime("%Y%m%d_%H%M%S")

    rows: list[dict] = []
    for d in sorted(sd.iterdir()):
        if not d.is_dir() or not is_timestamp_folder(d.name) or d.name[:15] < cutoff:
            continue

        meta_path = d / "meta.json"
        full_path = d / "full_analysis.json"
        if not (meta_path.exists() and full_path.exists()):
            continue

        try:
            meta = json.loads(meta_path.read_text())
            full = json.loads(full_path.read_text())
        except Exception:
            continue

        fw_pos = sum(1 for x in meta.get("apiResults", []) if x == 1)
        if isinstance(full, dict):
            full_results = full.get("full_results", [])
            full_pos = int(full.get("full_prey_count", sum(1 for x in full_results if x is True)))
            full_n = int(full.get("frames_total", len(full_results)))
        elif isinstance(full, list):
            full_pos = sum(1 for x in full if isinstance(x, dict) and x.get("prey") == 1)
            full_n = len(full)
        else:
            full_pos = 0
            full_n = 0

        rows.append(
            {
                "burst_id": d.name,
                "fw_pos": fw_pos,
                "fw_n": len(meta.get("apiResults", [])),
                "full_pos": full_pos,
                "full_n": full_n,
                "apiResult": meta.get("apiResult"),
                "direction": meta.get("direction"),
                "minDist": meta.get("directionMinDist"),
                "firstDist": meta.get("directionFirstDist"),
            }
        )

    rows.sort(key=lambda r: r["burst_id"])

    frame_level_fn = [r for r in rows if r["fw_pos"] == 0 and r["full_pos"] > 0]
    burst_level_fn = [r for r in rows if r["fw_pos"] < 2 and r["full_pos"] >= 2]

    human_vs_fw_fn: list[dict] = []
    if bursts_csv.exists():
        with bursts_csv.open() as f:
            for r in csv.DictReader(f):
                bid = r.get("burst_id", "")
                if len(bid) < 15 or bid[:15] < cutoff:
                    continue
                if r.get("human_prey") == "1" and r.get("fw_burst_label") == "0":
                    human_vs_fw_fn.append(
                        {
                            "burst_id": bid,
                            "human_prey": r.get("human_prey"),
                            "fw_burst_label": r.get("fw_burst_label"),
                            "full_burst_label": r.get("full_burst_label"),
                            "human_direction": r.get("human_direction"),
                            "fw_direction": r.get("fw_direction"),
                        }
                    )
    human_vs_fw_fn.sort(key=lambda r: r["burst_id"])

    out_json = Path("captures/false_negative_last48h.json")
    out_txt = Path("captures/false_negative_last48h.txt")

    payload = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "cutoff": cutoff,
        "total_reanalyzed_last48h": len(rows),
        "frame_level_false_negatives": frame_level_fn,
        "burst_level_false_negatives": burst_level_fn,
        "human_vs_fw_false_negatives": human_vs_fw_fn,
    }
    out_json.write_text(json.dumps(payload, indent=2))

    lines = [
        f"total_reanalyzed_last48h={len(rows)}",
        f"frame_level_false_negatives={len(frame_level_fn)}",
    ]
    for r in frame_level_fn:
        lines.append(
            f"  {r['burst_id']} fw={r['fw_pos']}/{r['fw_n']} full={r['full_pos']}/{r['full_n']} "
            f"dir={r['direction']} minDist={r['minDist']} firstDist={r['firstDist']}"
        )

    lines.append(f"burst_level_false_negatives={len(burst_level_fn)}")
    for r in burst_level_fn:
        lines.append(
            f"  {r['burst_id']} fw={r['fw_pos']}/{r['fw_n']} full={r['full_pos']}/{r['full_n']} "
            f"dir={r['direction']} minDist={r['minDist']} firstDist={r['firstDist']}"
        )

    lines.append(f"human_vs_fw_false_negatives={len(human_vs_fw_fn)}")
    for r in human_vs_fw_fn:
        direction = r.get("human_direction") or r.get("fw_direction") or ""
        lines.append(
            f"  {r['burst_id']} human={r['human_prey']} fw={r['fw_burst_label']} "
            f"full={r['full_burst_label']} dir={direction}"
        )

    out_txt.write_text("\n".join(lines) + "\n")

    print("\n".join(lines))


if __name__ == "__main__":
    main()
