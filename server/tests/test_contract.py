"""Conformance + smoke tests for the v1/compat and v2/frame endpoints.

Uses the StubPipeline (MAZGE_MODEL_BACKEND=stub, default) so we exercise the
wire contract without needing model files.
"""

from __future__ import annotations

import base64
import io
import os
import uuid

import pytest
from fastapi.testclient import TestClient
from PIL import Image

# Ensure stub backend before importing app
os.environ.setdefault("MAZGE_MODEL_BACKEND", "stub")
os.environ.setdefault("MAZGE_LOG_DIR", "/tmp/mazge-test-logs")

from server.app import CONTRACT_VERSION, create_app  # noqa: E402


@pytest.fixture(scope="module")
def client() -> TestClient:
    return TestClient(create_app())


def _jpeg_bytes(size: int = 64) -> bytes:
    buf = io.BytesIO()
    Image.new("RGB", (size, size), color=(128, 128, 128)).save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def _headers(jpeg_size: int = 0) -> dict[str, str]:
    return {
        "X-Contract-Version": str(CONTRACT_VERSION),
        "X-Device-Id": "mazge-test",
        "X-Burst-Id": "20260601_000000_gen1",
        "X-Frame-Index": "0",
        "X-Frame-Ts-Ms": "1780261000000",
        "X-Request-Id": str(uuid.uuid4()),
        "Content-Type": "image/jpeg",
    }


# --- /healthz ---


def test_healthz(client: TestClient) -> None:
    r = client.get("/healthz")
    assert r.status_code == 200
    assert r.json()["ok"] is True
    assert "backend" in r.json()


# --- /v1/compat ---


def test_v1_compat_smoke(client: TestClient) -> None:
    jpeg = _jpeg_bytes()
    r = client.post("/v1/compat", json={"image_base64": base64.b64encode(jpeg).decode()})
    assert r.status_code == 200
    j = r.json()
    assert "detected" in j
    assert isinstance(j["detected"], bool)


def test_v1_compat_bad_json(client: TestClient) -> None:
    r = client.post("/v1/compat", json={"oops": "no image"})
    assert r.status_code == 400
    assert r.json()["detected"] is False


# --- /v2/frame: conformance items from doc/inference_api_v2_contract.md ---


def test_v2_rejects_non_jpeg_content_type(client: TestClient) -> None:
    h = _headers()
    h["Content-Type"] = "application/octet-stream"
    r = client.post("/v2/frame", content=b"\x00\x01", headers=h)
    assert r.status_code == 415
    assert r.json()["error"] == "unsupported_media_type"


def test_v2_enforces_contract_version(client: TestClient) -> None:
    h = _headers()
    h["X-Contract-Version"] = "1"
    r = client.post("/v2/frame", content=_jpeg_bytes(), headers=h)
    assert r.status_code == 400
    assert r.json()["error"] == "bad_request"


def test_v2_payload_too_large(client: TestClient) -> None:
    h = _headers()
    big = b"\xff" * 600_000
    r = client.post("/v2/frame", content=big, headers=h)
    assert r.status_code == 413


def test_v2_echoes_request_id(client: TestClient) -> None:
    h = _headers()
    rid = h["X-Request-Id"]
    r = client.post("/v2/frame", content=_jpeg_bytes(), headers=h)
    assert r.status_code == 200
    assert r.json()["request_id"] == rid


def test_v2_returns_required_fields(client: TestClient) -> None:
    r = client.post("/v2/frame", content=_jpeg_bytes(), headers=_headers())
    j = r.json()
    for field in (
        "detected",
        "cat_recognized",
        "cat_id",
        "severity",
        "lockout_seconds",
        "door_action",
        "should_continue_burst",
        "model_version",
        "decision_ms",
    ):
        assert field in j, field


def test_v2_door_action_invariant_when_cat_not_recognized(client: TestClient) -> None:
    # StubPipeline yields prey_score=0 and empty cat softmax -> cat_recognized=False.
    r = client.post("/v2/frame", content=_jpeg_bytes(), headers=_headers())
    j = r.json()
    if j["cat_recognized"] is False:
        assert j["door_action"] != "allow"


def test_v2_idempotency_replay(client: TestClient) -> None:
    h = _headers()
    jpeg = _jpeg_bytes()
    a = client.post("/v2/frame", content=jpeg, headers=h)
    b = client.post("/v2/frame", content=jpeg, headers=h)
    assert a.status_code == 200 and b.status_code == 200
    assert a.json() == b.json()


def test_v2_idempotency_conflict(client: TestClient) -> None:
    h = _headers()
    client.post("/v2/frame", content=_jpeg_bytes(64), headers=h)
    r = client.post("/v2/frame", content=_jpeg_bytes(128), headers=h)
    assert r.status_code == 409
    assert r.json()["error"] == "duplicate_request"


def test_v2_unprocessable_jpeg(client: TestClient) -> None:
    # Stub pipeline accepts any bytes; check the empty-body path which we treat as 400.
    h = _headers()
    r = client.post("/v2/frame", content=b"", headers=h)
    assert r.status_code == 400
