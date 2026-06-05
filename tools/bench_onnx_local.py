"""Bench ONNX Runtime CPU latency for yolo11s and prey_v3 EffNet-B0.

Exports the models to ONNX as needed (cached in --out-dir) and benchmarks.
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
from torch import nn
from torchvision.models import EfficientNet_B0_Weights, efficientnet_b0

ROOT = Path(__file__).resolve().parents[1]


class PreyV3Classifier(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        self.features = backbone.features
        self.avgpool = backbone.avgpool
        feat_dim = backbone.classifier[1].in_features
        self.frame_emb = nn.Embedding(10, 16)
        head_in = feat_dim + 16
        self.prey_head = nn.Sequential(nn.Dropout(0.2), nn.Linear(head_in, 1))
        self.cat_head = nn.Sequential(nn.Dropout(0.2), nn.Linear(head_in, 2))

    def forward(self, x, frame_idx):
        feat = self.features(x)
        feat = self.avgpool(feat).flatten(1)
        emb = self.frame_emb(frame_idx.clamp(min=0, max=9))
        feat = torch.cat([feat, emb], dim=1)
        return self.prey_head(feat).squeeze(-1), self.cat_head(feat)


def export_prey_v3(ckpt: Path, out: Path, imgsz: int) -> None:
    model = PreyV3Classifier().eval()
    sd = torch.load(ckpt, map_location="cpu", weights_only=False)
    if isinstance(sd, dict) and "model" in sd:
        sd = sd["model"]
    model.load_state_dict(sd, strict=False)
    x = torch.randn(1, 3, imgsz, imgsz)
    fidx = torch.tensor([0], dtype=torch.long)
    torch.onnx.export(
        model, (x, fidx), str(out),
        input_names=["image", "frame_idx"],
        output_names=["prey_logit", "cat_logits"],
        opset_version=17,
        dynamic_axes=None,
    )


def percentiles(samples_ms: list[float]) -> tuple[float, float, float]:
    s = sorted(samples_ms)
    p50 = statistics.median(s)
    p95 = s[int(0.95 * (len(s) - 1))]
    p99 = s[int(0.99 * (len(s) - 1))]
    return p50, p95, p99


def bench_session(sess: ort.InferenceSession, feeds: dict, warmup: int, runs: int) -> dict:
    for _ in range(warmup):
        sess.run(None, feeds)
    samples: list[float] = []
    for _ in range(runs):
        t0 = time.perf_counter()
        sess.run(None, feeds)
        samples.append((time.perf_counter() - t0) * 1000.0)
    p50, p95, p99 = percentiles(samples)
    return {
        "min": min(samples), "p50": p50, "p95": p95, "p99": p99,
        "max": max(samples), "mean": statistics.mean(samples),
    }


def fmt(label: str, m: dict) -> str:
    return (f"{label:36s} "
            f"min={m['min']:6.1f}  p50={m['p50']:6.1f}  "
            f"p95={m['p95']:6.1f}  p99={m['p99']:6.1f}  mean={m['mean']:6.1f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default=str(ROOT / "_bench_weights"))
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--threads", type=int, default=0)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(exist_ok=True)

    so = ort.SessionOptions()
    if args.threads:
        so.intra_op_num_threads = args.threads
        so.inter_op_num_threads = 1

    print(f"onnxruntime {ort.__version__} providers={ort.get_available_providers()}")
    print(f"threads: intra={so.intra_op_num_threads or 'default'} inter={so.inter_op_num_threads or 'default'}")
    print()

    # --- prey_v3 ONNX ---
    ckpt = ROOT / "models" / "prey_v3" / "bodyA" / "best_burst_f1.pt"
    prey_onnx = out_dir / "prey_v3_bodyA_224.onnx"
    if not prey_onnx.exists():
        print(f"exporting {prey_onnx} ...")
        export_prey_v3(ckpt, prey_onnx, 224)

    sess = ort.InferenceSession(str(prey_onnx), so, providers=["CPUExecutionProvider"])
    feeds = {
        "image": np.random.randn(1, 3, 224, 224).astype(np.float32),
        "frame_idx": np.array([0], dtype=np.int64),
    }
    m = bench_session(sess, feeds, args.warmup, args.runs)
    print(fmt("prey_v3 ONNX @224", m))

    # --- yolo11s ONNX (use Ultralytics exports we may already have) ---
    from ultralytics import YOLO
    for imgsz in (384, 480):
        path = out_dir / f"yolo11s_{imgsz}.onnx"
        if not path.exists():
            print(f"exporting {path} ...")
            tmp = YOLO(str(out_dir / "yolo11s.pt")).export(
                format="onnx", imgsz=imgsz, dynamic=False, simplify=True
            )
            Path(tmp).rename(path)
        sess = ort.InferenceSession(str(path), so, providers=["CPUExecutionProvider"])
        name = sess.get_inputs()[0].name
        feeds = {name: np.random.randn(1, 3, imgsz, imgsz).astype(np.float32)}
        m = bench_session(sess, feeds, args.warmup, args.runs)
        print(fmt(f"yolo11s ONNX @{imgsz}", m))


if __name__ == "__main__":
    main()
