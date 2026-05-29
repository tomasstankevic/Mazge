#!/usr/bin/env python3
"""Summarize recent non-prey bursts and flag simple suspect cases."""

from __future__ import annotations

import json
from datetime import datetime, timedelta
from pathlib import Path


def is_timestamp_folder(name: str) -> bool:
    return len(name) >= 15 and name[8] == "_" and name[:8].isdigit() and name[9:15].isdigit()


def main() -> None:
    hours = 48
    sd = Path("captures/sd")
    cutoff = (datetime.now() - timedelta(hours=hours)).strftime("%Y%m%d_%H%M%S")

    rec: list[dict] = []
    for d in sorted(sd.iterdir()):
        if not d.is_dir() or not is_timestamp_folder(d.name):
            continue
        if d.name[:15] < cutoff:
            continue

        meta_path = d / "meta.json"
        if not meta_path.exists():
            continue

        try:
            m = json.loads(meta_path.read_text())
        except Exception:
            continue

        if m.get("apiResult") != 0:
            continue

        rec.append(
            {
                "burst_id": d.name,
                "apiResult": m.get("apiResult"),
                "direction": m.get("direction"),
                "directionMinDist": m.get("directionMinDist"),
                "directionFirstDist": m.get("directionFirstDist"),
                "images": len(m.get("images", [])),
                "jpg_count": len(list(d.glob("*.jpg"))),
            }
        )

    rec.sort(key=lambda x: x["burst_id"])

    suspects = [
        r
        for r in rec
        if r.get("direction") in (0, 1)
        and isinstance(r.get("directionMinDist"), (int, float))
        and r.get("directionMinDist") <= 120
    ]

    out_all = Path("captures/non_prey_last48h_summary.json")
    out_sus = Path("captures/non_prey_last48h_suspects.json")
    out_txt = Path("captures/non_prey_last48h_suspects.txt")

    out_all.write_text(json.dumps(rec, indent=2))
    out_sus.write_text(json.dumps(suspects, indent=2))

    lines = [f"non_prey_last48h={len(rec)}", f"suspects={len(suspects)}", ""]
    for s in suspects:
        lines.append(
            f"{s['burst_id']} direction={s.get('direction')} "
            f"minDist={s.get('directionMinDist')} firstDist={s.get('directionFirstDist')} "
            f"jpgs={s.get('jpg_count')}"
        )
    out_txt.write_text("\n".join(lines) + "\n")

    print(f"non_prey_last48h={len(rec)}")
    print(f"suspects={len(suspects)}")
    for s in suspects:
        print(
            f"{s['burst_id']} direction={s.get('direction')} "
            f"minDist={s.get('directionMinDist')} firstDist={s.get('directionFirstDist')} "
            f"jpgs={s.get('jpg_count')}"
        )


if __name__ == "__main__":
    main()
