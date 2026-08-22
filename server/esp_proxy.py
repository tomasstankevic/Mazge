"""Reverse-proxy the ESP32 web UI on a second port.

Only the read/control HTTP API on port 80 is forwarded. The MJPEG stream on
port 81 is intentionally not proxied. The page's one absolute URL pattern
(`'http://' + location.hostname`) is rewritten to an empty string so its
fetches become relative and stay on the proxy port.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import httpx
from fastapi import FastAPI, Request, Response

logger = logging.getLogger("mazge.esp_proxy")

_HOP_BY_HOP = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailers",
    "transfer-encoding",
    "upgrade",
    "host",
    "content-length",
}

_BLOCKED_PREFIXES = ("stream",)
_HTML_REWRITE = b"'http://'+location.hostname"
_HTML_REWRITE_TO = b"''"


@dataclass
class ProxyStatus:
    """Track ESP32 proxy connectivity status."""
    esp_host: str
    last_success_ms: int | None = None  # timestamp of last successful request
    last_error: str | None = None
    error_count: int = 0
    success_count: int = 0
    
    @property
    def is_healthy(self) -> bool:
        """Consider proxy healthy if we've had a success in the last 60 seconds."""
        if self.last_success_ms is None:
            return False
        return (time.time() * 1000 - self.last_success_ms) < 60_000


_proxy_status: ProxyStatus | None = None


def make_proxy_app(esp_host: str, timeout_s: float = 10.0) -> FastAPI:
    base_url = f"http://{esp_host}"
    app = FastAPI(title="mazge-esp-proxy")
    client = httpx.Client(base_url=base_url, timeout=timeout_s, follow_redirects=False)
    
    global _proxy_status
    if _proxy_status is None:
        _proxy_status = ProxyStatus(esp_host=esp_host)

    @app.api_route(
        "/{path:path}",
        methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"],
    )
    async def forward(path: str, request: Request) -> Response:
        if path.split("/", 1)[0] in _BLOCKED_PREFIXES:
            return Response(status_code=404, content=b"stream proxy disabled")
        target = "/" + path
        if request.url.query:
            target += "?" + request.url.query
        fwd_headers = {
            k: v
            for k, v in request.headers.items()
            if k.lower() not in _HOP_BY_HOP
        }
        body = await request.body()
        try:
            r = client.request(
                request.method,
                target,
                headers=fwd_headers,
                content=body if body else None,
            )
            # Track successful connection
            _proxy_status.last_success_ms = int(time.time() * 1000)
            _proxy_status.success_count += 1
            _proxy_status.last_error = None
        except httpx.HTTPError as exc:
            _proxy_status.error_count += 1
            _proxy_status.last_error = str(exc)
            logger.warning("esp32 proxy error %s %s: %s", request.method, target, exc)
            return Response(
                status_code=502,
                content=f"esp32 unreachable: {exc}".encode(),
                media_type="text/plain",
            )
        content = r.content
        ct = r.headers.get("content-type", "")
        if "html" in ct.lower():
            content = content.replace(_HTML_REWRITE, _HTML_REWRITE_TO)
        resp_headers = {
            k: v
            for k, v in r.headers.items()
            if k.lower() not in _HOP_BY_HOP and k.lower() != "content-encoding"
        }
        return Response(content=content, status_code=r.status_code, headers=resp_headers)

    return app


def start_in_thread(esp_host: str, port: int) -> None:
    """Run the proxy on its own uvicorn server in a daemon thread."""
    import uvicorn

    app = make_proxy_app(esp_host)
    cfg = uvicorn.Config(app, host="0.0.0.0", port=port, log_level="warning")
    server = uvicorn.Server(cfg)
    th = threading.Thread(target=server.run, name="esp-proxy", daemon=True)
    th.start()
    logger.warning("esp32 proxy listening on :%d → http://%s", port, esp_host)


def get_status() -> ProxyStatus | None:
    """Return current proxy status, or None if proxy not started."""
    return _proxy_status
