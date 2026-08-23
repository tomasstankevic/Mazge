#!/usr/bin/env python3
"""Export the combined multitask checkpoint to ONNX for the inference server.

Produces a single ONNX with four outputs:
    prey_logit        (1,)   binary prey
    cat_logits        (2,)   mazge / benis
    subject_logits    (4,)   empty / cat / human / other
    direction_logits  (2,)   entering / exiting

Input signature matches prey_v3 (image 1x3x224x224 + frame_idx int64) so the
server's crop/preprocess path is unchanged; only the number of outputs grows.

Usage:
  uv run python tools/export_multitask_onnx.py \
      --ckpt models/multitask/combined_v1/best.pt \
      --out  _bench_weights/multitask_combined_v1_224.onnx
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from tools.train_multitask import MazgeMultiTask  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--imgsz", type=int, default=224)
    ap.add_argument("--opset", type=int, default=17)
    args = ap.parse_args()

    model = MazgeMultiTask(direction_grad=False).eval()
    sd = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    if isinstance(sd, dict) and "model" in sd:
        sd = sd["model"]
    model.load_state_dict(sd, strict=True)

    x = torch.randn(1, 3, args.imgsz, args.imgsz)
    fidx = torch.tensor([0], dtype=torch.long)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model, (x, fidx), str(out),
        input_names=["image", "frame_idx"],
        output_names=["prey_logit", "cat_logits", "subject_logits", "direction_logits"],
        opset_version=args.opset,
        dynamic_axes=None,
        dynamo=False,
    )
    print(f"wrote {out}")

    # Sanity: run it through onnxruntime if available.
    try:
        import numpy as np
        import onnxruntime as ort
        sess = ort.InferenceSession(str(out), providers=["CPUExecutionProvider"])
        feeds = {"image": x.numpy(), "frame_idx": fidx.numpy().astype("int64")}
        res = sess.run(None, feeds)
        print("onnxruntime output shapes:", [r.shape for r in res])
    except Exception as exc:  # pragma: no cover
        print(f"(skipped ORT check: {exc})")


if __name__ == "__main__":
    main()
