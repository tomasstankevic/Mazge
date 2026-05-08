"""Optimized variant: full pixel-decode + crop right occlusion + 90 CCW rotate
+ proportional resize to fit 384x384 + grayscale + JPEG q=90.

Produces a higher-quality 384x336 (or so) image that may improve detection.
"""
import io
from pathlib import Path

from PIL import Image


def process_optimized(jpg_bytes: bytes,
                      crop_right_px: int = 128,
                      max_dim: int = 384,
                      quality: int = 90) -> bytes:
    """Decode + crop right + rotate 90 CCW + proportional resize + grayscale + encode."""
    img = Image.open(io.BytesIO(jpg_bytes))
    # Crop right occlusion: 640x480 -> 512x480
    w, h = img.size
    cropped = img.crop((0, 0, w - crop_right_px, h))
    # Rotate 90 CCW (PIL: positive = CCW)
    rotated = cropped.rotate(90, expand=True)  # 480x512
    # Proportional resize so longest side = max_dim
    rw, rh = rotated.size
    scale = max_dim / max(rw, rh)
    nw, nh = int(rw * scale), int(rh * scale)
    # Round down to 16/8 alignment for JPEG efficiency (not required by API)
    nw, nh = (nw // 16) * 16, (nh // 8) * 8
    resized = rotated.resize((nw, nh), Image.LANCZOS)
    # Grayscale
    gray = resized.convert("L")
    buf = io.BytesIO()
    gray.save(buf, format="JPEG", quality=quality, subsampling=2)
    return buf.getvalue()
