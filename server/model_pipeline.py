"""Model pipeline backends.

Two implementations:

- StubPipeline: returns deterministic dummy outputs so the wire contract can be
  tested without loading any model.
- OnnxPipeline: yolo11s body detect -> crop -> prey_v3 EffNet-B0 (prey + cat_id).

The pipeline owns its own warmup; the FastAPI app calls .infer() per request.
"""

from __future__ import annotations

import io
import logging
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class InferResult:
    prey_score: float
    cat_logits_softmax: list[float]
    decision_ms: int
    backend: str


class StubPipeline:
    backend = "stub"

    def warmup(self) -> None:
        pass

    def infer(self, jpeg: bytes) -> InferResult:
        return InferResult(
            prey_score=0.0,
            cat_logits_softmax=[],
            decision_ms=0,
            backend=self.backend,
        )


class OnnxPipeline:
    backend = "onnx"

    def __init__(
        self,
        yolo_onnx: Path,
        prey_onnx: Path,
        yolo_imgsz: int,
        threads: int,
    ) -> None:
        import onnxruntime as ort

        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = threads
            so.inter_op_num_threads = 1

        self._yolo = ort.InferenceSession(
            str(yolo_onnx), so, providers=["CPUExecutionProvider"]
        )
        self._yolo_input = self._yolo.get_inputs()[0].name
        self._yolo_imgsz = yolo_imgsz

        self._prey = ort.InferenceSession(
            str(prey_onnx), so, providers=["CPUExecutionProvider"]
        )
        self._prey_inputs = [i.name for i in self._prey.get_inputs()]

        self._cat_cls = 15  # COCO cat
        self._cat_conf = 0.15

    def warmup(self) -> None:
        # One pass through each model so ORT initializes its kernel cache.
        yolo_dummy = {
            self._yolo_input: np.zeros(
                (1, 3, self._yolo_imgsz, self._yolo_imgsz), dtype=np.float32
            )
        }
        self._yolo.run(None, yolo_dummy)
        prey_dummy = {
            self._prey_inputs[0]: np.zeros((1, 3, 224, 224), dtype=np.float32),
            self._prey_inputs[1]: np.zeros((1,), dtype=np.int64),
        }
        self._prey.run(None, prey_dummy)
        logger.info("onnx pipeline warmup done")

    def infer(self, jpeg: bytes) -> InferResult:
        import cv2

        t0 = time.perf_counter()
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        frame_bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame_bgr is None:
            raise ValueError("jpeg decode failed")
        # Same to_upright as build_crops.py: rotate 90 CCW + drop top strip.
        upright = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        h = upright.shape[0]
        upright = upright[int(h * 0.20) :, :, :]

        body = self._detect_body(upright)
        if body is None:
            return InferResult(
                prey_score=0.0,
                cat_logits_softmax=[],
                decision_ms=int((time.perf_counter() - t0) * 1000),
                backend=self.backend,
            )
        prey_score, cat_softmax = self._classify_prey(upright, body)
        return InferResult(
            prey_score=prey_score,
            cat_logits_softmax=cat_softmax,
            decision_ms=int((time.perf_counter() - t0) * 1000),
            backend=self.backend,
        )

    # --- helpers ---

    def _detect_body(self, upright_bgr: np.ndarray) -> tuple[int, int, int, int] | None:
        import cv2

        img = self._yolo_preprocess(upright_bgr)
        outputs = self._yolo.run(None, {self._yolo_input: img})[0]
        # Ultralytics yolo11 ONNX output: (1, 84, N) = (1, 4 + 80 classes, anchors).
        # Channels 0..3 = xc, yc, w, h (in imgsz coords); 4.. = per-class logits.
        out = outputs[0]
        cat_scores = out[4 + self._cat_cls, :]
        best = int(np.argmax(cat_scores))
        score = float(cat_scores[best])
        if score < self._cat_conf:
            return None
        xc, yc, w, h = (float(v) for v in out[:4, best])
        # Undo letterbox: img was letterboxed into a square of self._yolo_imgsz
        # with `pad` border and `scale` scale. _yolo_preprocess stashes them.
        scale, pad_x, pad_y = self._last_letterbox
        x1 = max(0, int(round((xc - w / 2 - pad_x) / scale)))
        y1 = max(0, int(round((yc - h / 2 - pad_y) / scale)))
        x2 = max(0, int(round((xc + w / 2 - pad_x) / scale)))
        y2 = max(0, int(round((yc + h / 2 - pad_y) / scale)))
        H, W = upright_bgr.shape[:2]
        x2 = min(W - 1, x2)
        y2 = min(H - 1, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        return x1, y1, x2, y2

    def _yolo_preprocess(self, bgr: np.ndarray) -> np.ndarray:
        import cv2

        H, W = bgr.shape[:2]
        size = self._yolo_imgsz
        scale = min(size / W, size / H)
        new_w = int(round(W * scale))
        new_h = int(round(H * scale))
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        pad_x = (size - new_w) // 2
        pad_y = (size - new_h) // 2
        canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = resized
        self._last_letterbox = (scale, pad_x, pad_y)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        chw = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        return chw[None]

    def _classify_prey(
        self, upright_bgr: np.ndarray, body: tuple[int, int, int, int]
    ) -> tuple[float, list[float]]:
        import cv2

        x1, y1, x2, y2 = body
        # 10% pad (matches build_crops.py BODY_PAD_FRAC).
        bw = x2 - x1
        bh = y2 - y1
        pad = int(0.10 * max(bw, bh))
        H, W = upright_bgr.shape[:2]
        cx1 = max(0, x1 - pad)
        cy1 = max(0, y1 - pad)
        cx2 = min(W, x2 + pad)
        cy2 = min(H, y2 + pad)
        crop = upright_bgr[cy1:cy2, cx1:cx2]
        crop = cv2.resize(crop, (224, 224), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        # ImageNet normalisation (same as training transform).
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        rgb = (rgb - mean) / std
        chw = rgb.transpose(2, 0, 1)[None]
        outputs = self._prey.run(
            None,
            {
                self._prey_inputs[0]: chw,
                self._prey_inputs[1]: np.zeros((1,), dtype=np.int64),
            },
        )
        prey_logit = float(outputs[0].squeeze())
        cat_logits = outputs[1].squeeze()
        prey_prob = 1.0 / (1.0 + float(np.exp(-prey_logit)))
        cat_softmax = _softmax(cat_logits).tolist()
        return prey_prob, cat_softmax


def _softmax(x: np.ndarray) -> np.ndarray:
    x = x - x.max()
    e = np.exp(x)
    return e / e.sum()


def build_pipeline(backend: str, **kwargs):
    if backend == "stub":
        return StubPipeline()
    if backend == "onnx":
        return OnnxPipeline(**kwargs)
    raise ValueError(f"unknown backend: {backend}")
