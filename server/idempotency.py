"""Tiny TTL cache for X-Request-Id idempotent replay (v2 contract section)."""

from __future__ import annotations

import hashlib
import threading
import time
from dataclasses import dataclass
from typing import Any


@dataclass
class _Entry:
    payload_hash: str
    response: Any
    inserted_at: float


class IdempotencyCache:
    def __init__(self, ttl_s: int) -> None:
        self._ttl_s = ttl_s
        self._lock = threading.Lock()
        self._store: dict[str, _Entry] = {}

    @staticmethod
    def hash_bytes(data: bytes) -> str:
        return hashlib.sha256(data).hexdigest()

    def get(self, request_id: str, payload_hash: str) -> tuple[str, Any] | None:
        """Return ("hit", response) on replay, ("conflict", None) on payload mismatch,
        or None on miss."""
        now = time.time()
        with self._lock:
            entry = self._store.get(request_id)
            if entry is None:
                return None
            if now - entry.inserted_at > self._ttl_s:
                self._store.pop(request_id, None)
                return None
            if entry.payload_hash != payload_hash:
                return ("conflict", None)
            return ("hit", entry.response)

    def put(self, request_id: str, payload_hash: str, response: Any) -> None:
        with self._lock:
            self._store[request_id] = _Entry(payload_hash, response, time.time())
            # Opportunistic GC: keep the store small.
            if len(self._store) > 4096:
                cutoff = time.time() - self._ttl_s
                self._store = {
                    k: v for k, v in self._store.items() if v.inserted_at >= cutoff
                }
