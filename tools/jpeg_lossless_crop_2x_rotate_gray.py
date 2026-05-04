"""Python reference: lossless DCT crop+2x-downscale+rotate+drop-chroma.

Pipeline:
  640x480 4:2:2 source
   → lossless DCT crop right 128px → 512x480
   → 2x DCT-domain downscale (drop chroma) → 256x240 grayscale
   → lossless DCT 90 CCW rotate → 240x256 grayscale
   → output JPEG

The 2x downscale uses the JPG_SCALE_2X technique:
  - Take top-left 4x4 of each 8x8 input DCT block (low-freq quadrant)
  - IDCT-4 each → 4x4 spatial samples (representing the original 8x8 block at 2x downscale)
  - For each 2x2 group of input blocks (16x16 spatial area):
      Arrange the four 4x4 spatial patches into one 8x8 spatial block
  - DCT-8 the 8x8 spatial block → output coefficient block
  - Re-quantize with the same Y QT
"""
from __future__ import annotations
import math
import struct
import sys
from pathlib import Path
from typing import List, Tuple

# Reuse the existing JPEG codec helpers via subprocess of the existing tool
sys.path.insert(0, str(Path(__file__).resolve().parent))
from jpeg_lossless_crop_rotate_gray import (
    BitReader, BitWriter, HuffTable, ZZ,
    extend, encode_value, parse_jpeg, decode_block, encode_block,
    rotate90ccw_block, write_marker, write_seg, serialize_dht, serialize_dqt,
    M_SOI, M_EOI, M_SOF0, M_DHT, M_DQT, M_SOS,
)


# ============================================================
# 4x4 IDCT and 8x8 DCT (direct matrix form, simple but correct)
# ============================================================

def _dct_basis(N: int):
    """Build N×N DCT basis matrix C[k][n] = α(k) * cos((2n+1)*k*π / (2N))."""
    basis = [[0.0] * N for _ in range(N)]
    for k in range(N):
        alpha = math.sqrt(1.0 / N) if k == 0 else math.sqrt(2.0 / N)
        for n in range(N):
            basis[k][n] = alpha * math.cos(math.pi * (2 * n + 1) * k / (2 * N))
    return basis


_C8 = _dct_basis(8)
_C4 = _dct_basis(4)


# Standard JPEG luminance quantization table (Annex K, sample table for quality~50)
# In ZIGZAG order.
_Q50_LUM_ZIGZAG = [
    16, 11, 12, 14, 12, 10, 16, 14,
    13, 14, 18, 17, 16, 19, 24, 40,
    26, 24, 22, 22, 24, 49, 35, 37,
    29, 40, 58, 51, 61, 60, 57, 51,
    56, 55, 64, 72, 92, 78, 64, 68,
    87, 69, 55, 56, 80, 109, 81, 87,
    95, 98, 103, 104, 103, 62, 77, 113,
    121, 112, 100, 120, 92, 101, 103, 99,
]


def quality_qt_natural(quality: int) -> List[int]:
    """Generate a standard JPEG luminance Q table for given quality (1-100).
    Returns 64-element list in NATURAL row-major order.
    """
    if quality < 1: quality = 1
    if quality > 100: quality = 100
    scale = 5000 // quality if quality < 50 else 200 - 2 * quality
    qt_zz = []
    for v in _Q50_LUM_ZIGZAG:
        q = (v * scale + 50) // 100
        if q < 1: q = 1
        if q > 255: q = 255
        qt_zz.append(q)
    # Convert zigzag → natural
    qt_natural = [0] * 64
    for k in range(64):
        qt_natural[ZZ[k]] = qt_zz[k]
    return qt_natural


def qt_natural_to_zigzag(qt_nat: List[int]) -> List[int]:
    return [qt_nat[ZZ[k]] for k in range(64)]


def idct_8x8(coefs: List[List[float]]) -> List[List[float]]:
    """8x8 IDCT: x = C^T * X * C."""
    tmp = [[0.0] * 8 for _ in range(8)]
    for k in range(8):
        for n in range(8):
            s = 0.0
            for m in range(8):
                s += _C8[m][n] * coefs[k][m]
            tmp[k][n] = s
    out = [[0.0] * 8 for _ in range(8)]
    for i in range(8):
        for n in range(8):
            s = 0.0
            for k in range(8):
                s += _C8[k][i] * tmp[k][n]
            out[i][n] = s
    return out


def idct_4x4(coefs: List[List[float]]) -> List[List[float]]:
    """4x4 IDCT: x = C^T * X * C (where C is the 4x4 DCT basis)."""
    # First pass: rows. y[k][n] = sum_m C[m][n] * X[k][m]
    tmp = [[0.0] * 4 for _ in range(4)]
    for k in range(4):
        for n in range(4):
            s = 0.0
            for m in range(4):
                s += _C4[m][n] * coefs[k][m]
            tmp[k][n] = s
    # Second pass: cols. x[i][n] = sum_k C[k][i] * tmp[k][n]
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for n in range(4):
            s = 0.0
            for k in range(4):
                s += _C4[k][i] * tmp[k][n]
            out[i][n] = s
    return out


def dct_8x8(spatial: List[List[float]]) -> List[List[float]]:
    """8x8 DCT: X = C * x * C^T."""
    # First pass: rows.
    tmp = [[0.0] * 8 for _ in range(8)]
    for k in range(8):
        for n in range(8):
            s = 0.0
            for m in range(8):
                s += _C8[k][m] * spatial[m][n]
            tmp[k][n] = s
    # Second pass: cols.
    out = [[0.0] * 8 for _ in range(8)]
    for k in range(8):
        for n in range(8):
            s = 0.0
            for i in range(8):
                s += tmp[k][i] * _C8[n][i]
            out[k][n] = s
    return out


# ============================================================
# 2x downscale of a 2x2 group of 8x8 DCT blocks → one 8x8 block
# ============================================================

def downscale_2x2_to_1(b00, b01, b10, b11, qt_in, qt_out):
    """
    Combine 4 DCT blocks (16x16 spatial area) into 1 DCT block (8x8 representing
    the same area at half resolution).

    Reference algorithm (slow but rigorous):
      For each input block:
        1. Dequantize
        2. Full 8x8 IDCT → 8x8 spatial samples
      Arrange the four 8x8 patches into one 16x16 spatial buffer
      2x2 average → 8x8 spatial
      Add JPEG level shift back: spatial += 128 was already gone, no shift here
      DCT-8 → 8x8 coefficients
      Quantize: out[i] = round(coefs[i] / qt_out[i])
    Returns 64-element list (natural order, integer).
    """
    full16 = [[0.0] * 16 for _ in range(16)]
    for blk, (oy, ox) in [(b00, (0, 0)), (b01, (0, 8)),
                          (b10, (8, 0)), (b11, (8, 8))]:
        # Dequantize all 64 coefficients
        c8 = [[0.0] * 8 for _ in range(8)]
        for r in range(8):
            for c in range(8):
                c8[r][c] = blk[r * 8 + c] * qt_in[r * 8 + c]
        # Full 8x8 IDCT
        s8 = idct_8x8(c8)
        for r in range(8):
            for c in range(8):
                full16[oy + r][ox + c] = s8[r][c]
    # 2x2 spatial average → 8x8
    spatial_8x8 = [[0.0] * 8 for _ in range(8)]
    for r in range(8):
        for c in range(8):
            spatial_8x8[r][c] = (
                full16[r * 2][c * 2] + full16[r * 2][c * 2 + 1] +
                full16[r * 2 + 1][c * 2] + full16[r * 2 + 1][c * 2 + 1]
            ) / 4.0
    # DCT-8
    coefs8 = dct_8x8(spatial_8x8)
    # Quantize and clamp to int16
    out = [0] * 64
    for r in range(8):
        for c in range(8):
            v = round(coefs8[r][c] / qt_out[r * 8 + c])
            if v > 32767: v = 32767
            if v < -32768: v = -32768
            out[r * 8 + c] = int(v)
    return out


# ============================================================
# Main pipeline: crop right 128px + 2x downscale (drop chroma) + rotate 90 CCW
# ============================================================

def process(src_jpeg: bytes, crop_right_px: int = 128) -> bytes:
    """Full pipeline. Returns output JPEG bytes."""
    info = parse_jpeg(src_jpeg)
    if info.width % 16 != 0 or info.height % 8 != 0:
        raise ValueError("input dims must be multiple of 16x8")
    # Crop right: keep first (width - crop_right_px) columns
    crop_w = info.width - crop_right_px  # 512 if 640-128
    crop_h = info.height
    if crop_w % 16 != 0:
        raise ValueError("crop dims not 16-aligned")

    # Source MCU geometry (4:2:2): MCU = 16x8 px, 2 Y blocks horizontal
    src_mcu_cols = info.width // 16
    src_mcu_rows = info.height // 8
    crop_mcu_cols = crop_w // 16  # 32 for 512
    crop_mcu_rows = crop_h // 8   # 60 for 480

    # Y blocks within crop region: 2*crop_mcu_cols wide, crop_mcu_rows tall
    y_in_w = 2 * crop_mcu_cols  # 64 Y blocks wide (512 px)
    y_in_h = crop_mcu_rows       # 60 Y blocks tall (480 px)

    # SOS table mapping
    sos_by_id = {cid: (dc, ac) for (cid, dc, ac) in info.scan_components}
    y_id = info.components[0][0]
    cb_id = info.components[1][0]
    cr_id = info.components[2][0]
    y_qid = info.components[0][3]
    y_dc_t, y_ac_t = sos_by_id[y_id]
    cb_dc_t, cb_ac_t = sos_by_id[cb_id]
    cr_dc_t, cr_ac_t = sos_by_id[cr_id]
    y_dc_ht = info.dht_dc[y_dc_t]
    y_ac_ht = info.dht_ac[y_ac_t]
    cb_dc_ht = info.dht_dc[cb_dc_t]
    cb_ac_ht = info.dht_ac[cb_ac_t]
    cr_dc_ht = info.dht_dc[cr_dc_t]
    cr_ac_ht = info.dht_ac[cr_ac_t]

    # Decode all source MCUs, store Y blocks (in crop region) in coefficient buffer
    y_blocks = [[None] * y_in_w for _ in range(y_in_h)]
    br = BitReader(src_jpeg, info.scan_data_start)
    dc_y, dc_cb, dc_cr = 0, 0, 0
    for mr in range(src_mcu_rows):
        for mc in range(src_mcu_cols):
            # 2 Y blocks
            blk0, dc_y = decode_block(br, dc_y, y_dc_ht, y_ac_ht)
            blk1, dc_y = decode_block(br, dc_y, y_dc_ht, y_ac_ht)
            # Cb, Cr (decode but discard)
            _, dc_cb = decode_block(br, dc_cb, cb_dc_ht, cb_ac_ht)
            _, dc_cr = decode_block(br, dc_cr, cr_dc_ht, cr_ac_ht)
            # Store Y if in crop region (left part: mc < crop_mcu_cols)
            if mc < crop_mcu_cols:
                bx = 2 * mc
                y_blocks[mr][bx] = blk0
                y_blocks[mr][bx + 1] = blk1

    # Y QT (input table from source). For output, use a fresh standard q=80 table
    # since after 2x downscale the source QT (sized for full-res) over-quantizes.
    qt_in_zigzag = info.dqt[y_qid]
    qt_in_natural = [0] * 64
    for k in range(64):
        qt_in_natural[ZZ[k]] = qt_in_zigzag[k]
    qt_out_natural = quality_qt_natural(80)
    qt_out_zigzag = qt_natural_to_zigzag(qt_out_natural)

    # 2x downscale: combine 2x2 groups of Y blocks into 1 output block
    # Output Y grid: (y_in_w/2) wide, (y_in_h/2) tall
    out_w = y_in_w // 2  # 32 blocks (256 px)
    out_h = y_in_h // 2  # 30 blocks (240 px)
    ds_blocks = [[None] * out_w for _ in range(out_h)]
    for by in range(out_h):
        for bx in range(out_w):
            b00 = y_blocks[by * 2][bx * 2]
            b01 = y_blocks[by * 2][bx * 2 + 1]
            b10 = y_blocks[by * 2 + 1][bx * 2]
            b11 = y_blocks[by * 2 + 1][bx * 2 + 1]
            ds_blocks[by][bx] = downscale_2x2_to_1(b00, b01, b10, b11,
                                                   qt_in_natural, qt_out_natural)

    # Apply 90 CCW rotation: per-block transpose + sign flip, then re-arrange grid
    # Input grid out_w × out_h, output grid out_h × out_w
    rot_w = out_h  # 30 (240 px)
    rot_h = out_w  # 32 (256 px)
    rot_blocks = [[None] * rot_w for _ in range(rot_h)]
    for out_by in range(rot_h):
        for out_bx in range(rot_w):
            src_bx = out_w - 1 - out_by
            src_by = out_bx
            rot_blocks[out_by][out_bx] = rotate90ccw_block(ds_blocks[src_by][src_bx])

    # Build output JPEG
    out = bytearray()
    write_marker(out, M_SOI)
    write_seg(out, 0xE0, b"JFIF\x00" + struct.pack(">BBBHHBB", 1, 1, 0, 1, 1, 0, 0))
    # DQT (use fresh q=80 Y QT)
    write_seg(out, M_DQT, serialize_dqt(0, qt_out_zigzag))
    # SOF0 with rotated dims (240 wide × 256 tall)
    out_pix_w = rot_w * 8  # 240
    out_pix_h = rot_h * 8  # 256
    sof = struct.pack(">BHHB", 8, out_pix_h, out_pix_w, 1)
    sof += bytes([y_id, 0x11, 0])
    write_seg(out, M_SOF0, sof)
    write_seg(out, M_DHT, serialize_dht(0, 0, y_dc_ht))
    write_seg(out, M_DHT, serialize_dht(1, 0, y_ac_ht))
    sos = bytes([1, y_id, 0x00, 0, 63, 0])
    write_seg(out, M_SOS, sos)
    bw = BitWriter()
    dc_pred = 0
    for by in range(rot_h):
        for bx in range(rot_w):
            dc_pred = encode_block(rot_blocks[by][bx], dc_pred, bw, y_dc_ht, y_ac_ht)
    bw.flush()
    out += bytes(bw.out)
    write_marker(out, M_EOI)
    return bytes(out)


if __name__ == "__main__":
    src_path = sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1/f05.jpg"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/lossless_2x_rotate_out.jpg"
    src = Path(src_path).read_bytes()
    out = process(src)
    Path(out_path).write_bytes(out)
    print(f"wrote {out_path} ({len(out)} bytes from {len(src)} input)")
