"""Reverse-proxy the ESP32 web UI on a second port.

Only the read/control HTTP API on port 80 is forwarded. The MJPEG stream on
port 81 is intentionally not proxied. The page's one absolute URL pattern
(`'http://' + location.hostname`) is rewritten to an empty string so its
fetches become relative and stay on the proxy port.
"""

from __future__ import annotations

import logging
import threading

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


def make_proxy_app(esp_host: str, timeout_s: float = 10.0) -> FastAPI:
    base_url = f"http://{esp_host}"
    app = FastAPI(title="mazge-esp-proxy")
    client = httpx.Client(base_url=base_url, timeout=timeout_s, follow_redirects=False)

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
        except httpx.HTTPError as exc:
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
