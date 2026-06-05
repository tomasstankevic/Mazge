"""Thin paho-mqtt wrapper. No-op if no host configured."""

from __future__ import annotations

import json
import logging
import threading
from dataclasses import asdict

from .config import Config
from .decisions import Decision

logger = logging.getLogger(__name__)


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
        client.connect_async(cfg.mqtt_host, cfg.mqtt_port, keepalive=30)
        client.loop_start()
        self._client = client
        logger.info("mqtt connecting to %s:%s", cfg.mqtt_host, cfg.mqtt_port)

    def publish_decision(self, device_id: str, burst_id: str, dec: Decision) -> None:
        if self._client is None:
            return
        topic_prefix = "mazge"
        payload = json.dumps(asdict(dec))
        with self._lock:
            self._client.publish(f"{topic_prefix}/decision/last", payload, qos=0, retain=True)
            self._client.publish(
                f"{topic_prefix}/decision/door_action", dec.door_action, qos=0, retain=True
            )
            self._client.publish(
                f"{topic_prefix}/decision/severity", dec.severity, qos=0, retain=True
            )
            self._client.publish(
                f"{topic_prefix}/decision/cat_id", dec.cat_id, qos=0, retain=True
            )
            self._client.publish(
                f"{topic_prefix}/decision/prey_score",
                f"{dec.prey_score:.4f}",
                qos=0,
                retain=True,
            )

    def publish_heartbeat(self, alive_for_s: float) -> None:
        if self._client is None:
            return
        with self._lock:
            self._client.publish(
                "mazge/server/heartbeat", f"{int(alive_for_s)}", qos=0, retain=True
            )
