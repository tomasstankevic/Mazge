"""Runtime configuration loaded from environment.

Variables (all optional, sensible defaults):

  MAZGE_MODEL_BACKEND   stub | onnx                          (default stub)
  MAZGE_PREY_ONNX       path to prey_v3 .onnx
  MAZGE_YOLO_ONNX       path to yolo11s .onnx
  MAZGE_YOLO_IMGSZ      640 | 480 | 384                      (default 640)
  MAZGE_THREADS         intra-op threads for ORT             (default 0 = ORT default)
  MAZGE_PORT            HTTP port                            (default 8080)
  MAZGE_MQTT_HOST       mosquitto host                       (default empty = disabled)
  MAZGE_MQTT_PORT       (default 1883)
  MAZGE_MQTT_USER, MAZGE_MQTT_PASS
  MAZGE_LOG_DIR         JSON log directory                   (default ./logs/server)
  MAZGE_IDEMPOTENCY_TTL_S  (default 300)
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Config:
    model_backend: str
    prey_onnx: Path | None
    yolo_onnx: Path | None
    yolo_imgsz: int
    threads: int
    port: int
    mqtt_host: str
    mqtt_port: int
    mqtt_user: str
    mqtt_pass: str
    log_dir: Path
    idempotency_ttl_s: int
    debug_dump_dir: Path | None

    @classmethod
    def from_env(cls) -> "Config":
        def _path(env: str) -> Path | None:
            v = os.environ.get(env, "").strip()
            return Path(v) if v else None

        return cls(
            model_backend=os.environ.get("MAZGE_MODEL_BACKEND", "stub").lower(),
            prey_onnx=_path("MAZGE_PREY_ONNX"),
            yolo_onnx=_path("MAZGE_YOLO_ONNX"),
            yolo_imgsz=int(os.environ.get("MAZGE_YOLO_IMGSZ", "640")),
            threads=int(os.environ.get("MAZGE_THREADS", "0")),
            port=int(os.environ.get("MAZGE_PORT", "8080")),
            mqtt_host=os.environ.get("MAZGE_MQTT_HOST", "").strip(),
            mqtt_port=int(os.environ.get("MAZGE_MQTT_PORT", "1883")),
            mqtt_user=os.environ.get("MAZGE_MQTT_USER", ""),
            mqtt_pass=os.environ.get("MAZGE_MQTT_PASS", ""),
            log_dir=Path(os.environ.get("MAZGE_LOG_DIR", str(ROOT / "logs" / "server"))),
            idempotency_ttl_s=int(os.environ.get("MAZGE_IDEMPOTENCY_TTL_S", "300")),
            debug_dump_dir=_path("MAZGE_DEBUG_DUMP_DIR"),
        )
