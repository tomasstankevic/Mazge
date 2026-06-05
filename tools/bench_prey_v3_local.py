"""Latency micro-bench for the prey_v3 EfficientNet-B0 head.

Loads the actual trained checkpoint when given a path; otherwise uses the
ImageNet-pretrained backbone (same compute, latency-equivalent).

Usage:
    .venv-server/bin/python tools/bench_prey_v3_local.py \\
        models/prey_v3/bodyA/best_burst_f1.pt --imgsz 224 --warmup 5 --runs 30
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torchvision.models import EfficientNet_B0_Weights, efficientnet_b0

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))


class PreyV3Classifier(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        self.features = backbone.features
        self.avgpool = backbone.avgpool
        feat_dim = backbone.classifier[1].in_features

        self.frame_emb = nn.Embedding(10, 16)
        head_in = feat_dim + 16

        self.prey_head = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(head_in, 1),
        )
        self.cat_head = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(head_in, 2),
        )

    def forward(self, x, frame_idx):
        feat = self.features(x)
        feat = self.avgpool(feat).flatten(1)
        emb = self.frame_emb(frame_idx.clamp(min=0, max=9))
        feat = torch.cat([feat, emb], dim=1)
        prey_logit = self.prey_head(feat).squeeze(-1)
        cat_logits = self.cat_head(feat)
        return prey_logit, cat_logits


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ckpt", nargs="?", default=None,
                        help="optional .pt checkpoint; otherwise use ImageNet weights")
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--jit", action="store_true", help="torch.jit.trace the model")
    args = parser.parse_args()

    if args.threads:
        torch.set_num_threads(args.threads)
    print(f"torch={torch.__version__} threads={torch.get_num_threads()}")

    model = PreyV3Classifier().eval()
    if args.ckpt:
        sd = torch.load(args.ckpt, map_location="cpu", weights_only=False)
        if isinstance(sd, dict) and "state_dict" in sd:
            sd = sd["state_dict"]
        if isinstance(sd, dict) and "model" in sd:
            sd = sd["model"]
        try:
            model.load_state_dict(sd, strict=False)
            print(f"loaded checkpoint: {args.ckpt}")
        except Exception as exc:  # noqa: BLE001
            print(f"checkpoint load failed ({exc}); using ImageNet weights (latency-equivalent)")
    else:
        print("no checkpoint given; using ImageNet weights (latency-equivalent)")

    x = torch.randn(1, 3, args.imgsz, args.imgsz)
    fidx = torch.tensor([0], dtype=torch.long)

    if args.jit:
        with torch.no_grad():
            model = torch.jit.trace(model, (x, fidx))
        print("traced with torch.jit.trace")

    with torch.no_grad():
        for _ in range(args.warmup):
            model(x, fidx)

        samples_ms: list[float] = []
        for _ in range(args.runs):
            t0 = time.perf_counter()
            model(x, fidx)
            samples_ms.append((time.perf_counter() - t0) * 1000.0)

    samples_ms.sort()
    p50 = statistics.median(samples_ms)
    p95 = samples_ms[int(0.95 * (len(samples_ms) - 1))]
    p99 = samples_ms[int(0.99 * (len(samples_ms) - 1))]
    print()
    print(f"prey_v3 imgsz={args.imgsz} runs={args.runs}")
    print(f"  min={samples_ms[0]:7.1f} ms")
    print(f"  p50={p50:7.1f} ms")
    print(f"  p95={p95:7.1f} ms")
    print(f"  p99={p99:7.1f} ms")
    print(f"  max={samples_ms[-1]:7.1f} ms")
    print(f"  mean={statistics.mean(samples_ms):7.1f} ms")


if __name__ == "__main__":
    main()
