"""Mazge local inference server (FastAPI + ONNX Runtime).

Two endpoints:

- POST /v1/compat    {"image_base64": "..."} -> {"detected": bool, ...}
- POST /v2/frame     raw image/jpeg + X-* headers -> v2 contract JSON

See doc/inference_api_v2_contract.md.
"""
