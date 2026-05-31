#!/usr/bin/env python3
"""Build a contact-sheet image per burst showing raw upright frame, yolo11x crop, yolo11s crop."""
from __future__ import annotations
import sys
from pathlib import Path
import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
OUT = REPO / "models" / "subject_detection_eval" / "fn_inspection"
OUT.mkdir(parents=True, exist_ok=True)


def to_upright(bgr):
    rot = cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return rot[160:, :, :]


def make_sheet(burst: str, out_path: Path):
    raw_dir = REPO / "captures" / "sd" / burst
    x_dir = REPO / "dataset" / "crops_yolo11x_rotcrop" / burst
    s_dir = REPO / "dataset" / "crops_yolo11s_rotcrop" / burst
    raws = sorted(raw_dir.glob("f*.jpg"))
    cell = 240
    pad = 4
    rows = []
    for raw in raws:
        fidx = int(raw.stem.split("_")[0][1:])
        raw_img = cv2.imread(str(raw))
        upr = to_upright(raw_img)
        upr_s = cv2.resize(upr, (cell, cell))

        x_path = x_dir / f"f{fidx:02d}_body.jpg"
        s_path = s_dir / f"f{fidx:02d}_body.jpg"
        x_img = cv2.imread(str(x_path)) if x_path.exists() else np.full((cell, cell, 3), 50, np.uint8)
        s_img = cv2.imread(str(s_path)) if s_path.exists() else np.full((cell, cell, 3), 50, np.uint8)
        x_img = cv2.resize(x_img, (cell, cell))
        s_img = cv2.resize(s_img, (cell, cell))

        cv2.rectangle(x_img, (0, 0), (cell - 1, cell - 1),
                      (0, 200, 0) if x_path.exists() else (0, 0, 200), 3)
        cv2.rectangle(s_img, (0, 0), (cell - 1, cell - 1),
                      (0, 200, 0) if s_path.exists() else (0, 0, 200), 3)
        if not x_path.exists():
            cv2.putText(x_img, "MISS", (40, cell // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
        if not s_path.exists():
            cv2.putText(s_img, "MISS", (40, cell // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

        lbl = np.full((cell, 60, 3), 30, np.uint8)
        cv2.putText(lbl, f"f{fidx}", (5, cell // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        rows.append(np.hstack([lbl, upr_s, x_img, s_img]))

    sheet = np.vstack([np.pad(r, ((pad, pad), (0, 0), (0, 0)), constant_values=10) for r in rows])
    header = np.full((50, sheet.shape[1], 3), 0, np.uint8)
    cv2.putText(header, f"BURST {burst}  |  upright  |  YOLO11x crop  |  YOLO11s crop",
                (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    sheet = np.vstack([header, sheet])
    cv2.imwrite(str(out_path), sheet)
    print(f"wrote {out_path}  shape={sheet.shape}")


if __name__ == "__main__":
    bursts = sys.argv[1:] or [
        "20260505_220948_gen9",
        "20260521_013251_gen4",
    ]
    for b in bursts:
        make_sheet(b, OUT / f"FN_{b}.jpg")
