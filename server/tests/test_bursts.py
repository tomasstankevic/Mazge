"""Tests for the burst-listing + image-serving routes used by Home Assistant."""

from __future__ import annotations

import io
import os
import uuid
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from PIL import Image


@pytest.fixture
def client_with_dump(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> TestClient:
    dump = tmp_path / "dump"
    monkeypatch.setenv("MAZGE_MODEL_BACKEND", "stub")
    monkeypatch.setenv("MAZGE_LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.setenv("MAZGE_DEBUG_DUMP_DIR", str(dump))
    from server.app import create_app  # imported here so env applies

    return TestClient(create_app())


def _jpeg() -> bytes:
    buf = io.BytesIO()
    Image.new("RGB", (32, 32), color=(0, 0, 0)).save(buf, format="JPEG")
    return buf.getvalue()


def _post_frame(client: TestClient, burst_id: str, frame_index: int) -> None:
    headers = {
        "X-Contract-Version": "2",
        "X-Device-Id": "mazge-test",
        "X-Burst-Id": burst_id,
        "X-Frame-Index": str(frame_index),
        "X-Frame-Ts-Ms": "1780000000000",
        "X-Request-Id": str(uuid.uuid4()),
        "Content-Type": "image/jpeg",
    }
    r = client.post("/v2/frame", content=_jpeg(), headers=headers)
    assert r.status_code == 200, r.text


def test_recent_bursts_empty(client_with_dump: TestClient) -> None:
    r = client_with_dump.get("/v1/bursts/recent")
    assert r.status_code == 200
    j = r.json()
    assert j["bursts"] == []
    assert "server_ts_ms" in j


def test_recent_bursts_groups_frames(client_with_dump: TestClient) -> None:
    _post_frame(client_with_dump, "burst_A", 0)
    _post_frame(client_with_dump, "burst_A", 1)
    _post_frame(client_with_dump, "burst_B", 0)
    r = client_with_dump.get("/v1/bursts/recent?limit=10")
    assert r.status_code == 200
    bursts = r.json()["bursts"]
    by_id = {b["burst_id"]: b for b in bursts}
    assert set(by_id) == {"burst_A", "burst_B"}
    assert by_id["burst_A"]["frame_count"] == 2
    assert by_id["burst_B"]["frame_count"] == 1
    for frame in by_id["burst_A"]["frames"]:
        assert frame["image_url"].startswith("/v1/bursts/")
        assert frame["image_url"].endswith(".jpg")


def test_burst_image_serves_file(client_with_dump: TestClient) -> None:
    _post_frame(client_with_dump, "burst_C", 0)
    bursts = client_with_dump.get("/v1/bursts/recent").json()["bursts"]
    url = bursts[0]["frames"][0]["image_url"]
    r = client_with_dump.get(url)
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"
    assert len(r.content) > 0


def test_burst_image_rejects_traversal(client_with_dump: TestClient) -> None:
    r = client_with_dump.get("/v1/bursts/..%2F..%2Fetc/passwd")
    assert r.status_code == 404


def test_latest_frame_after_post(client_with_dump: TestClient) -> None:
    _post_frame(client_with_dump, "burst_D", 0)
    r = client_with_dump.get("/v1/latest_frame.jpg")
    assert r.status_code == 200
    assert r.headers["content-type"] == "image/jpeg"


def test_status_includes_latest(client_with_dump: TestClient) -> None:
    _post_frame(client_with_dump, "burst_E", 0)
    r = client_with_dump.get("/v1/status")
    assert r.status_code == 200
    j = r.json()
    assert j["ok"] is True
    assert j["latest_image_url"] is not None
    assert j["latest_decision"] is not None
