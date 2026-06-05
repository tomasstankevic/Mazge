"""FastAPI app exposing /v1/compat (legacy) and /v2/frame (v2 contract).

Run:

    .venv-server/bin/uvicorn server.app:app --host 0.0.0.0 --port 8080
"""

from __future__ import annotations

import base64
import json
import logging
import time
import uuid
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Header, HTTPException, Request, Response
from fastapi.responses import JSONResponse

from .config import Config
from .decisions import Decision, decide
from .idempotency import IdempotencyCache
from .model_pipeline import StubPipeline, build_pipeline
from .mqtt import MqttPublisher

CONTRACT_VERSION = 2
MAX_BODY_BYTES = 512_000

logger = logging.getLogger("mazge.server")


def _now_ms() -> int:
    return int(time.time() * 1000)


def _setup_logging(log_dir: Path) -> None:
    log_dir.mkdir(parents=True, exist_ok=True)
    handler = logging.FileHandler(log_dir / "server.jsonl")
    handler.setFormatter(logging.Formatter("%(message)s"))
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.addHandler(handler)
    root.addHandler(logging.StreamHandler())


def create_app(cfg: Config | None = None) -> FastAPI:
    cfg = cfg or Config.from_env()
    _setup_logging(cfg.log_dir)

    if cfg.model_backend == "onnx" and (cfg.prey_onnx is None or cfg.yolo_onnx is None):
        logger.warning(
            "model_backend=onnx but MAZGE_PREY_ONNX or MAZGE_YOLO_ONNX missing; using stub"
        )
        pipeline = StubPipeline()
    else:
        pipeline = build_pipeline(
            cfg.model_backend,
            yolo_onnx=cfg.yolo_onnx,
            prey_onnx=cfg.prey_onnx,
            yolo_imgsz=cfg.yolo_imgsz,
            threads=cfg.threads,
        ) if cfg.model_backend == "onnx" else StubPipeline()

    pipeline.warmup()

    idem = IdempotencyCache(cfg.idempotency_ttl_s)
    mqtt = MqttPublisher(cfg)
    started_at = time.time()

    app = FastAPI(title="mazge-server", version="0.1.0")

    def _audit(payload: dict[str, Any]) -> None:
        logger.info(json.dumps(payload, separators=(",", ":")))

    def _decision_response(
        decision: Decision,
        request_id: str,
        decision_ms: int,
        device_id: str,
        burst_id: str,
        frame_index: int,
    ) -> dict[str, Any]:
        return {
            "contract_version": CONTRACT_VERSION,
            "request_id": request_id,
            "detected": decision.detected,
            "prey_score": round(decision.prey_score, 4),
            "cat_recognized": decision.cat_recognized,
            "cat_id": decision.cat_id,
            "cat_confidence": round(decision.cat_confidence, 4),
            "severity": decision.severity,
            "lockout_seconds": decision.lockout_seconds,
            "door_action": decision.door_action,
            "model_version": "prey_v3",
            "decision_ms": decision_ms,
            "server_ts_ms": _now_ms(),
            "should_continue_burst": decision.should_continue_burst,
            "reason": decision.reason,
        }

    @app.get("/healthz")
    def healthz() -> dict[str, Any]:
        return {
            "ok": True,
            "backend": pipeline.backend,
            "uptime_s": int(time.time() - started_at),
        }

    @app.post("/v1/compat")
    async def v1_compat(request: Request) -> JSONResponse:
        try:
            body = await request.json()
            b64 = body["image_base64"]
            jpeg = base64.b64decode(b64)
        except Exception:
            return JSONResponse(
                {"detected": False, "error": "bad_request"}, status_code=400
            )
        t0 = time.perf_counter()
        try:
            result = pipeline.infer(jpeg)
        except Exception as exc:  # noqa: BLE001
            logger.exception("inference failed: %s", exc)
            return JSONResponse(
                {"detected": False, "error": "internal_error"}, status_code=500
            )
        decision = decide(result.prey_score, result.cat_logits_softmax)
        ms = int((time.perf_counter() - t0) * 1000)
        _audit(
            {
                "ep": "/v1/compat",
                "ts_ms": _now_ms(),
                "decision_ms": ms,
                "prey_score": result.prey_score,
                "cat_id": decision.cat_id,
                "cat_recognized": decision.cat_recognized,
                "severity": decision.severity,
                "door_action": decision.door_action,
                "detected": decision.detected,
                "status_code": 200,
            }
        )
        return JSONResponse(
            {
                "detected": decision.detected,
                "score": round(decision.prey_score, 4),
                "cat_id": decision.cat_id,
            }
        )

    @app.post("/v2/frame")
    async def v2_frame(
        request: Request,
        x_contract_version: str = Header(...),
        x_device_id: str = Header(...),
        x_burst_id: str = Header(...),
        x_frame_index: str = Header(...),
        x_frame_ts_ms: str = Header(...),
        x_request_id: str = Header(...),
        content_type: str | None = Header(default=None),
    ) -> Response:
        request_id = x_request_id or str(uuid.uuid4())
        try:
            frame_index = int(x_frame_index)
        except ValueError:
            return _error(request_id, 400, "bad_request", "X-Frame-Index not int")

        if x_contract_version.strip() != str(CONTRACT_VERSION):
            return _error(request_id, 400, "bad_request", "contract_version mismatch")

        if content_type and not content_type.lower().startswith("image/jpeg"):
            return _error(request_id, 415, "unsupported_media_type", "expect image/jpeg")

        jpeg = await request.body()
        if len(jpeg) > MAX_BODY_BYTES:
            return _error(request_id, 413, "payload_too_large", f"{len(jpeg)} bytes")
        if not jpeg:
            return _error(request_id, 400, "bad_request", "empty body")

        payload_hash = IdempotencyCache.hash_bytes(jpeg)
        cached = idem.get(request_id, payload_hash)
        if cached and cached[0] == "hit":
            return JSONResponse(cached[1])
        if cached and cached[0] == "conflict":
            return _error(request_id, 409, "duplicate_request", "payload differs")

        t0 = time.perf_counter()
        try:
            result = pipeline.infer(jpeg)
        except ValueError as exc:
            return _error(request_id, 422, "unprocessable_frame", str(exc))
        except Exception as exc:  # noqa: BLE001
            logger.exception("inference failed: %s", exc)
            return _error(request_id, 500, "internal_error", "model crashed")
        decision = decide(result.prey_score, result.cat_logits_softmax)
        ms = int((time.perf_counter() - t0) * 1000)
        body = _decision_response(decision, request_id, ms, x_device_id, x_burst_id, frame_index)
        idem.put(request_id, payload_hash, body)
        mqtt.publish_decision(x_device_id, x_burst_id, decision)
        _audit(
            {
                "ep": "/v2/frame",
                "ts_ms": _now_ms(),
                "request_id": request_id,
                "device_id": x_device_id,
                "burst_id": x_burst_id,
                "frame_index": frame_index,
                "decision_ms": ms,
                "prey_score": result.prey_score,
                "cat_id": decision.cat_id,
                "cat_recognized": decision.cat_recognized,
                "severity": decision.severity,
                "lockout_seconds": decision.lockout_seconds,
                "door_action": decision.door_action,
                "detected": decision.detected,
                "status_code": 200,
            }
        )
        return JSONResponse(body)

    def _error(request_id: str, status: int, code: str, message: str) -> JSONResponse:
        return JSONResponse(
            {
                "contract_version": CONTRACT_VERSION,
                "request_id": request_id,
                "error": code,
                "message": message,
            },
            status_code=status,
        )

    return app


app = create_app()
