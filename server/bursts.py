"""Group recent decision audits into bursts for the Home Assistant UI.

A burst = all frames sharing the same (device_id, burst_id). Frames are read
from two sources:

1. An in-memory ring of recent /v2/frame audit dicts (live, fast).
2. The last N lines of today's (and yesterday's) `server.jsonl` on startup,
   so a restart does not blank the UI.

The actual JPEGs live under `MAZGE_DEBUG_DUMP_DIR/YYYY-MM-DD/<basename>.jpg`.
We only emit a relative `image_url` like `/v1/bursts/2026-06-20/foo.jpg`; the
HTTP route validates the path before serving.
"""

from __future__ import annotations

import json
import re
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_DAY_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
_BASENAME_RE = re.compile(r"^[A-Za-z0-9._-]+\.jpg$")


@dataclass
class _Frame:
    frame_index: int
    ts_ms: int
    image_url: str | None
    prey_score: float
    cat_id: str
    severity: str
    door_action: str
    detected: bool


@dataclass
class _Burst:
    device_id: str
    burst_id: str
    latest_ts_ms: int = 0
    frames: dict[int, _Frame] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        frames = sorted(self.frames.values(), key=lambda f: f.frame_index)
        worst = max(
            frames,
            key=lambda f: (f.prey_score, f.detected, f.ts_ms),
            default=None,
        )
        return {
            "device_id": self.device_id,
            "burst_id": self.burst_id,
            "latest_ts_ms": self.latest_ts_ms,
            "frame_count": len(frames),
            "max_prey_score": round(worst.prey_score, 4) if worst else 0.0,
            "any_detected": any(f.detected for f in frames),
            "severity": worst.severity if worst else "none",
            "door_action": worst.door_action if worst else "unknown",
            "cat_id": worst.cat_id if worst else "unknown",
            "frames": [
                {
                    "frame_index": f.frame_index,
                    "ts_ms": f.ts_ms,
                    "prey_score": round(f.prey_score, 4),
                    "cat_id": f.cat_id,
                    "severity": f.severity,
                    "door_action": f.door_action,
                    "detected": f.detected,
                    "image_url": f.image_url,
                }
                for f in frames
            ],
        }


class BurstIndex:
    """Thread-safe in-memory index of recent bursts."""

    def __init__(self, max_audits: int = 2000) -> None:
        self._lock = threading.Lock()
        self._ring: deque[dict[str, Any]] = deque(maxlen=max_audits)
        self._last_image_url: str | None = None
        self._last_decision: dict[str, Any] | None = None

    def record(self, audit: dict[str, Any]) -> None:
        """Add a /v2/frame audit record to the ring."""
        with self._lock:
            self._ring.append(audit)
            url = _image_url(audit.get("image_path"))
            if url:
                self._last_image_url = url
            self._last_decision = audit

    def hydrate_from_jsonl(self, log_dir: Path, max_lines: int = 4000) -> int:
        """Read the tail of recent server.jsonl files into the ring.

        Returns the number of audit records loaded.
        """
        files = sorted(log_dir.glob("server.jsonl*"))
        if not files:
            return 0
        # Newest first; collect lines until we have enough.
        lines: list[str] = []
        for f in reversed(files):
            try:
                tail = f.read_text(encoding="utf-8", errors="replace").splitlines()
            except OSError:
                continue
            lines = tail + lines
            if len(lines) >= max_lines:
                lines = lines[-max_lines:]
                break
        loaded = 0
        for line in lines:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if rec.get("ep") != "/v2/frame":
                continue
            self.record(rec)
            loaded += 1
        return loaded

    def recent_bursts(self, limit: int = 20) -> list[dict[str, Any]]:
        with self._lock:
            audits = list(self._ring)
        bursts: dict[tuple[str, str], _Burst] = {}
        for rec in audits:
            device = str(rec.get("device_id") or "")
            burst_id = str(rec.get("burst_id") or "")
            if not device or not burst_id:
                continue
            key = (device, burst_id)
            b = bursts.setdefault(key, _Burst(device, burst_id))
            try:
                fi = int(rec.get("frame_index") or 0)
            except (TypeError, ValueError):
                fi = 0
            ts_ms = int(rec.get("frame_ts_ms") or rec.get("ts_ms") or 0)
            url = _image_url(rec.get("image_path"))
            b.frames[fi] = _Frame(
                frame_index=fi,
                ts_ms=ts_ms,
                image_url=url,
                prey_score=float(rec.get("prey_score") or 0.0),
                cat_id=str(rec.get("cat_id") or "unknown"),
                severity=str(rec.get("severity") or "none"),
                door_action=str(rec.get("door_action") or "unknown"),
                detected=bool(rec.get("detected")),
            )
            if ts_ms > b.latest_ts_ms:
                b.latest_ts_ms = ts_ms
        ordered = sorted(bursts.values(), key=lambda b: b.latest_ts_ms, reverse=True)
        return [b.to_dict() for b in ordered[:limit]]

    def latest_image_url(self) -> str | None:
        with self._lock:
            return self._last_image_url

    def latest_decision(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._last_decision) if self._last_decision else None


def _image_url(image_path: str | None) -> str | None:
    """Translate an absolute dump path into `/v1/bursts/<day>/<basename>`."""
    if not image_path:
        return None
    try:
        p = Path(image_path)
    except (TypeError, ValueError):
        return None
    day = p.parent.name
    base = p.name
    if not _DAY_RE.match(day) or not _BASENAME_RE.match(base):
        return None
    return f"/v1/bursts/{day}/{base}"


def safe_dump_path(dump_dir: Path, day: str, filename: str) -> Path | None:
    """Resolve a request to a JPEG path, guarding against traversal."""
    if not _DAY_RE.match(day) or not _BASENAME_RE.match(filename):
        return None
    candidate = (dump_dir / day / filename).resolve()
    try:
        candidate.relative_to(dump_dir.resolve())
    except ValueError:
        return None
    if not candidate.is_file():
        return None
    return candidate
