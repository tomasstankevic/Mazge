"""Thin paho-mqtt wrapper. No-op if no host configured."""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import asdict

from .config import Config
from .decisions import Decision

logger = logging.getLogger(__name__)

TOPIC = "mazge"
TOPIC_STATUS = f"{TOPIC}/server/status"


class MqttPublisher:
    def __init__(self, cfg: Config) -> None:
        self._cfg = cfg
        self._client = None
        self._lock = threading.Lock()
        if not cfg.mqtt_host:
            logger.info("mqtt disabled (MAZGE_MQTT_HOST empty)")
            return
        try:
            import paho.mqtt.client as mqtt
        except Exception as exc:  # noqa: BLE001
            logger.warning("paho-mqtt not available: %s", exc)
            return
        client = mqtt.Client(
            client_id="mazge-server",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        )
        if cfg.mqtt_user:
            client.username_pw_set(cfg.mqtt_user, cfg.mqtt_pass)
        # LWT: broker publishes "offline" if we disconnect ungracefully.
        client.will_set(TOPIC_STATUS, "offline", qos=1, retain=True)
        client.connect_async(cfg.mqtt_host, cfg.mqtt_port, keepalive=30)
        client.loop_start()
        self._client = client
        logger.info("mqtt connecting to %s:%s", cfg.mqtt_host, cfg.mqtt_port)

    def publish_server_online(self, backend: str) -> None:
        if self._client is None:
            return
        with self._lock:
            self._client.publish(TOPIC_STATUS, "online", qos=1, retain=True)
            self._client.publish(f"{TOPIC}/server/backend", backend, qos=0, retain=True)
            self._client.publish(
                f"{TOPIC}/server/started_at_ms",
                str(int(time.time() * 1000)),
                qos=0,
                retain=True,
            )

    def publish_decision(
        self,
        device_id: str,
        burst_id: str,
        dec: Decision,
        *,
        frame_index: int | None = None,
        image_url: str | None = None,
    ) -> None:
        if self._client is None:
            return
        payload = json.dumps(
            {
                **asdict(dec),
                "device_id": device_id,
                "burst_id": burst_id,
                "frame_index": frame_index,
                "image_url": image_url,
                "ts_ms": int(time.time() * 1000),
            }
        )
        with self._lock:
            self._client.publish(f"{TOPIC}/decision/last", payload, qos=0, retain=True)
            self._client.publish(
                f"{TOPIC}/decision/door_action", dec.door_action, qos=0, retain=True
            )
            self._client.publish(
                f"{TOPIC}/decision/severity", dec.severity, qos=0, retain=True
            )
            self._client.publish(
                f"{TOPIC}/decision/cat_id", dec.cat_id, qos=0, retain=True
            )
            self._client.publish(
                f"{TOPIC}/decision/prey_score",
                f"{dec.prey_score:.4f}",
                qos=0,
                retain=True,
            )
            self._client.publish(
                f"{TOPIC}/decision/lockout_seconds",
                str(int(dec.lockout_seconds)),
                qos=0,
                retain=True,
            )
            self._client.publish(
                f"{TOPIC}/decision/detected",
                "ON" if dec.detected else "OFF",
                qos=0,
                retain=True,
            )
            self._client.publish(
                f"{TOPIC}/decision/burst_id", burst_id, qos=0, retain=True
            )
            self._client.publish(
                f"{TOPIC}/decision/device_id", device_id, qos=0, retain=True
            )
            if image_url is not None:
                self._client.publish(
                    f"{TOPIC}/decision/image_url", image_url, qos=0, retain=True
                )

    def publish_heartbeat(self, alive_for_s: float) -> None:
        if self._client is None:
            return
        with self._lock:
            self._client.publish(
                f"{TOPIC}/server/heartbeat", f"{int(alive_for_s)}", qos=0, retain=True
            )
