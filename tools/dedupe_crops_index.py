#!/usr/bin/env python3
"""Deduplicate a crops _index.csv by (burst_id, frame_idx, image_id).

Keeps the latest occurrence of each key so refreshed rows (e.g. with body_path)
replace earlier stale rows.

Usage:
  uv run python tools/dedupe_crops_index.py
  uv run python tools/dedupe_crops_index.py --crops crops_mdv6_raw --dry-run
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default="crops_yolo11x_rotcrop",
                    help="Dataset crops directory under dataset/")
    ap.add_argument("--dry-run", action="store_true",
                    help="Report duplicates without rewriting the file")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    index_csv = DATASET / args.crops / "_index.csv"
    if not index_csv.exists():
        raise SystemExit(f"Missing index file: {index_csv}")

    with open(index_csv, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        if not fieldnames:
            raise SystemExit(f"No header found in {index_csv}")

        first_pos: dict[tuple[str, str, str], int] = {}
        latest_rows: dict[tuple[str, str, str], dict] = {}
        order: list[tuple[str, str, str]] = []
        total_rows = 0

        for row in reader:
            total_rows += 1
            key = (row.get("burst_id", ""), row.get("frame_idx", ""), row.get("image_id", ""))
            if key not in first_pos:
                first_pos[key] = len(order)
                order.append(key)
            latest_rows[key] = row

    unique_rows = len(latest_rows)
    duplicates = total_rows - unique_rows

    print(f"index={index_csv}")
    print(f"total_rows={total_rows}")
    print(f"unique_keys={unique_rows}")
    print(f"duplicates={duplicates}")

    if duplicates == 0:
        print("No duplicates found. Nothing to do.")
        return

    if args.dry_run:
        print("Dry-run only. No file changes written.")
        return

    ordered_unique = [latest_rows[k] for k in order]
    with open(index_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(ordered_unique)

    print(f"Rewrote file with {unique_rows} unique rows.")


if __name__ == "__main__":
    main()
