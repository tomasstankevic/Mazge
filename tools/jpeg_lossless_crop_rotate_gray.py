"""Pure-Python reference: lossless DCT crop + 90 CCW rotate + drop chroma.

Input:  Baseline 4:2:2 grayscale-content JPEG (e.g. our OV2640 output, 640x480)
Output: Grayscale JPEG (1 component), 384x384, rotated 90 CCW
        — without IDCT/DCT, only Huffman decode/encode + DCT coefficient transpose.

Validated by sending output to the prey detection API.
"""
from __future__ import annotations
import struct
import sys
from pathlib import Path
from typing import List, Tuple

# JPEG markers
M_SOI, M_EOI = 0xD8, 0xD9
M_SOF0, M_DHT, M_DQT, M_SOS = 0xC0, 0xC4, 0xDB, 0xDA

# Standard JPEG zigzag order (8x8): zigzag[i] = (row, col) for natural order index
ZZ = [
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
]
INV_ZZ = [0]*64
for i, p in enumerate(ZZ):
    INV_ZZ[p] = i


# ============================================================
# Bit reader (with FF00 byte de-stuffing)
# ============================================================
class BitReader:
    def __init__(self, data: bytes, start: int):
        self.data = data
        self.pos = start
        self.bits = 0
        self.nbits = 0

    def _read_byte(self) -> int:
        if self.pos >= len(self.data):
            return -1
        b = self.data[self.pos]
        self.pos += 1
        if b == 0xFF:
            if self.pos >= len(self.data):
                return -1
            n = self.data[self.pos]
            self.pos += 1
            if n != 0x00:
                # Marker — back up
                self.pos -= 2
                return -1
        return b

    def fill(self, n: int) -> bool:
        while self.nbits < n:
            b = self._read_byte()
            if b < 0:
                return False
            self.bits = (self.bits << 8) | b
            self.nbits += 8
        return True

    def peek(self, n: int) -> int:
        return (self.bits >> (self.nbits - n)) & ((1 << n) - 1)

    def skip(self, n: int):
        self.nbits -= n
        self.bits &= (1 << self.nbits) - 1

    def read(self, n: int) -> int:
        if n == 0:
            return 0
        if not self.fill(n):
            return -1
        v = self.peek(n)
        self.skip(n)
        return v


# ============================================================
# Bit writer (with FF -> FF00 byte stuffing)
# ============================================================
class BitWriter:
    def __init__(self):
        self.out = bytearray()
        self.bits = 0
        self.nbits = 0

    def write(self, v: int, n: int):
        if n == 0:
            return
        self.bits = (self.bits << n) | (v & ((1 << n) - 1))
        self.nbits += n
        while self.nbits >= 8:
            self.nbits -= 8
            byte = (self.bits >> self.nbits) & 0xFF
            self.bits &= (1 << self.nbits) - 1
            self.out.append(byte)
            if byte == 0xFF:
                self.out.append(0x00)

    def flush(self):
        if self.nbits > 0:
            # Pad with 1s to byte boundary
            byte = ((self.bits << (8 - self.nbits)) | ((1 << (8 - self.nbits)) - 1)) & 0xFF
            self.out.append(byte)
            if byte == 0xFF:
                self.out.append(0x00)
            self.nbits = 0
            self.bits = 0


# ============================================================
# Huffman table
# ============================================================
class HuffTable:
    """Decoder + encoder for one Huffman table."""

    def __init__(self, bits: List[int], vals: List[int]):
        self.bits = bits  # 16 entries
        self.vals = vals
        # Build decoder
        self.maxcode = [0] * 17
        self.valoffset = [0] * 17
        code = 0
        si = 0
        for L in range(1, 17):
            count = bits[L - 1]
            self.maxcode[L] = code + count
            self.valoffset[L] = si - code
            code = (code + count) << 1
            si += count
        # Build encoder
        self.enc_code = [0] * 256
        self.enc_len = [0] * 256
        code = 0
        si = 0
        for L in range(1, 17):
            count = bits[L - 1]
            for _ in range(count):
                self.enc_code[vals[si]] = code
                self.enc_len[vals[si]] = L
                code += 1
                si += 1
            code <<= 1

    def decode(self, br: BitReader) -> int:
        # Slow but correct: read 1 bit at a time
        code = 0
        for L in range(1, 17):
            if not br.fill(L):
                return -1
            code = br.peek(L)
            if self.maxcode[L] > 0 and code < self.maxcode[L]:
                br.skip(L)
                return self.vals[code + self.valoffset[L]]
        return -1

    def encode_into(self, bw: BitWriter, sym: int):
        L = self.enc_len[sym]
        if L == 0:
            raise ValueError(f"symbol {sym} not in Huffman table")
        bw.write(self.enc_code[sym], L)


def extend(v: int, cat: int) -> int:
    if cat == 0:
        return 0
    half = 1 << (cat - 1)
    return v if v >= half else v - (2 * half - 1)


def encode_value(v: int) -> Tuple[int, int]:
    """Return (bits, nbits) for a coefficient value."""
    if v == 0:
        return 0, 0
    av = -v if v < 0 else v
    nbits = av.bit_length()
    bits = v if v >= 0 else v + (1 << nbits) - 1
    return bits, nbits


# ============================================================
# JPEG parser
# ============================================================
class JpegInfo:
    def __init__(self):
        self.width = 0
        self.height = 0
        self.components = []        # list of (id, h_samp, v_samp, qt_id)
        self.scan_components = []   # list of (id, dc_table, ac_table)
        self.dqt = {}               # qt_id -> 64-byte table (zigzag order)
        self.dht_dc = {}            # dest -> HuffTable
        self.dht_ac = {}            # dest -> HuffTable
        self.scan_data_start = 0
        self.scan_data_end = 0


def parse_jpeg(data: bytes) -> JpegInfo:
    info = JpegInfo()
    i = 0
    while i < len(data) - 1:
        if data[i] != 0xFF:
            i += 1
            continue
        m = data[i + 1]
        if m == M_SOI:
            i += 2
            continue
        if m == M_EOI:
            break
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        seg = data[i + 2:i + 2 + seg_len]  # incl. length bytes

        if m == M_DQT:
            # May contain multiple tables
            j = 2  # skip length
            while j < seg_len:
                pq_tq = seg[j]
                tq = pq_tq & 0x0F
                pq = pq_tq >> 4
                j += 1
                if pq != 0:
                    raise NotImplementedError("16-bit DQT")
                info.dqt[tq] = list(seg[j:j + 64])
                j += 64
        elif m == M_DHT:
            j = 2
            while j < seg_len:
                tc_th = seg[j]
                th = tc_th & 0x0F  # destination
                tc = tc_th >> 4    # 0=DC, 1=AC
                j += 1
                bits = list(seg[j:j + 16])
                j += 16
                nsym = sum(bits)
                vals = list(seg[j:j + nsym])
                j += nsym
                ht = HuffTable(bits, vals)
                if tc == 0:
                    info.dht_dc[th] = ht
                else:
                    info.dht_ac[th] = ht
        elif m == M_SOF0:
            prec = seg[2]
            assert prec == 8
            info.height = struct.unpack(">H", seg[3:5])[0]
            info.width = struct.unpack(">H", seg[5:7])[0]
            ncomp = seg[7]
            for c in range(ncomp):
                cid = seg[8 + c * 3]
                samp = seg[9 + c * 3]
                qid = seg[10 + c * 3]
                info.components.append((cid, samp >> 4, samp & 0xF, qid))
        elif m == M_SOS:
            ncomp = seg[2]
            for c in range(ncomp):
                cid = seg[3 + c * 2]
                tab = seg[3 + c * 2 + 1]
                info.scan_components.append((cid, tab >> 4, tab & 0xF))
            info.scan_data_start = i + 2 + seg_len
            i = info.scan_data_start
            # Find end of entropy data: next non-stuffed marker
            while i < len(data) - 1:
                if data[i] == 0xFF and data[i + 1] != 0x00:
                    break
                i += 1
            info.scan_data_end = i
            return info
        else:
            pass
        i += 2 + seg_len
    return info


# ============================================================
# Decode all blocks of one MCU
# ============================================================
def decode_block(br: BitReader, dc_pred: int, dc_ht: HuffTable,
                 ac_ht: HuffTable) -> Tuple[List[int], int]:
    """Decode one 8x8 block. Returns (natural-order coefs[64], new_dc_pred)."""
    coefs = [0] * 64
    # DC
    dc_cat = dc_ht.decode(br)
    if dc_cat < 0:
        raise RuntimeError("DC decode fail")
    dc_extra = br.read(dc_cat)
    if dc_cat > 0 and dc_extra < 0:
        raise RuntimeError("DC extra bits fail")
    dc_diff = extend(dc_extra, dc_cat)
    dc_val = dc_pred + dc_diff
    coefs[0] = dc_val  # DC at natural index 0
    # AC
    k = 1
    while k < 64:
        rs = ac_ht.decode(br)
        if rs < 0:
            raise RuntimeError("AC decode fail")
        run = rs >> 4
        cat = rs & 0x0F
        if cat == 0:
            if run == 15:
                k += 16  # ZRL
                continue
            break  # EOB
        k += run
        if k >= 64:
            raise RuntimeError("AC overrun")
        extra = br.read(cat)
        if extra < 0:
            raise RuntimeError("AC extra bits fail")
        coefs[ZZ[k]] = extend(extra, cat)  # store in natural order
        k += 1
    return coefs, dc_val


# ============================================================
# Encode one block (zigzag + RLE + huffman)
# ============================================================
def encode_block(coefs: List[int], dc_pred: int, bw: BitWriter,
                 dc_ht: HuffTable, ac_ht: HuffTable) -> int:
    """coefs in natural order. Returns new dc_pred."""
    dc_val = coefs[0]
    dc_diff = dc_val - dc_pred
    bits, nbits = encode_value(dc_diff)
    dc_ht.encode_into(bw, nbits)
    if nbits > 0:
        bw.write(bits, nbits)
    # AC: walk zigzag, RLE
    run = 0
    last_nz = 0
    for k in range(1, 64):
        if coefs[ZZ[k]] != 0:
            last_nz = k
    for k in range(1, last_nz + 1):
        v = coefs[ZZ[k]]
        if v == 0:
            run += 1
            if run == 16:
                ac_ht.encode_into(bw, 0xF0)  # ZRL
                run = 0
        else:
            bits, nbits = encode_value(v)
            sym = (run << 4) | nbits
            ac_ht.encode_into(bw, sym)
            bw.write(bits, nbits)
            run = 0
    if last_nz < 63:
        ac_ht.encode_into(bw, 0x00)  # EOB
    return dc_val


# ============================================================
# DCT 90° CCW: out[u*8+v] = (-1)^u * src[v*8+u]
# ============================================================
def rotate90ccw_block(src: List[int]) -> List[int]:
    out = [0] * 64
    for u in range(8):
        sign = 1 if (u % 2 == 0) else -1
        for v in range(8):
            out[u * 8 + v] = sign * src[v * 8 + u]
    return out


# ============================================================
# Build output JPEG
# ============================================================
def write_marker(out: bytearray, m: int):
    out.append(0xFF)
    out.append(m)


def write_seg(out: bytearray, m: int, payload: bytes):
    write_marker(out, m)
    out += struct.pack(">H", len(payload) + 2)
    out += payload


def serialize_dht(tc: int, th: int, ht: HuffTable) -> bytes:
    return bytes([(tc << 4) | th]) + bytes(ht.bits) + bytes(ht.vals)


def serialize_dqt(tq: int, table_zigzag: List[int]) -> bytes:
    return bytes([tq]) + bytes(table_zigzag)


# ============================================================
# Main: crop 384x384 + rotate 90 CCW + drop chroma
# ============================================================
def process(src_jpeg: bytes,
            crop_x: int, crop_y: int,
            crop_w: int, crop_h: int) -> bytes:
    info = parse_jpeg(src_jpeg)
    if info.width % 16 != 0 or info.height % 8 != 0:
        raise ValueError("input dims must be multiple of MCU 16x8")
    if crop_x % 16 or crop_y % 8 or crop_w % 16 or crop_h % 8:
        raise ValueError("crop must be MCU-aligned (16x8)")

    # MCU layout: source 4:2:2, MCU = 16x8 pixels
    mcu_w_px, mcu_h_px = 16, 8
    src_mcu_cols = info.width // mcu_w_px
    src_mcu_rows = info.height // mcu_h_px
    crop_mcu_x = crop_x // mcu_w_px
    crop_mcu_y = crop_y // mcu_h_px
    crop_mcu_cols = crop_w // mcu_w_px   # 24 for 384
    crop_mcu_rows = crop_h // mcu_h_px   # 48 for 384

    # Each source MCU has 2 Y blocks horizontally + 1 Cb + 1 Cr
    # In crop region, Y blocks are: 2*crop_mcu_cols x crop_mcu_rows
    # i.e., 48 x 48 Y blocks for 384x384 crop
    out_y_cols = 2 * crop_mcu_cols   # 48
    out_y_rows = crop_mcu_rows       # 48
    assert out_y_cols == out_y_rows == crop_w // 8 == crop_h // 8

    # Decode all MCUs into Y-block coefficient buffer (natural order)
    # y_blocks[by][bx] in source coords (within crop region)
    y_blocks = [[None] * out_y_cols for _ in range(out_y_rows)]

    # Determine which DC/AC Huffman tables each component uses (from SOS)
    sos_by_id = {cid: (dc, ac) for (cid, dc, ac) in info.scan_components}

    # First component is Y (assume id=1 with h=2)
    sof_by_id = {cid: (h, v, q) for (cid, h, v, q) in info.components}
    y_id = info.components[0][0]
    cb_id = info.components[1][0]
    cr_id = info.components[2][0]
    y_dc_t, y_ac_t = sos_by_id[y_id]
    cb_dc_t, cb_ac_t = sos_by_id[cb_id]
    cr_dc_t, cr_ac_t = sos_by_id[cr_id]

    y_dc_ht = info.dht_dc[y_dc_t]
    y_ac_ht = info.dht_ac[y_ac_t]
    cb_dc_ht = info.dht_dc[cb_dc_t]
    cb_ac_ht = info.dht_ac[cb_ac_t]
    cr_dc_ht = info.dht_dc[cr_dc_t]
    cr_ac_ht = info.dht_ac[cr_ac_t]

    br = BitReader(src_jpeg, info.scan_data_start)
    dc_y, dc_cb, dc_cr = 0, 0, 0
    for mr in range(src_mcu_rows):
        for mc in range(src_mcu_cols):
            # 2 Y blocks (left, right)
            blk0, dc_y = decode_block(br, dc_y, y_dc_ht, y_ac_ht)
            blk1, dc_y = decode_block(br, dc_y, y_dc_ht, y_ac_ht)
            # 1 Cb, 1 Cr — decode but discard
            _, dc_cb = decode_block(br, dc_cb, cb_dc_ht, cb_ac_ht)
            _, dc_cr = decode_block(br, dc_cr, cr_dc_ht, cr_ac_ht)
            # Store Y blocks if in crop region
            if (crop_mcu_y <= mr < crop_mcu_y + crop_mcu_rows and
                    crop_mcu_x <= mc < crop_mcu_x + crop_mcu_cols):
                src_by = mr - crop_mcu_y                   # 0..47
                src_bx = 2 * (mc - crop_mcu_x)             # 0,2,4,...,46
                y_blocks[src_by][src_bx] = blk0
                y_blocks[src_by][src_bx + 1] = blk1

    # Apply per-block 90° CCW rotation, then place into output grid
    # Output block (out_bx, out_by) ← source block (src_bx = N-1 - out_by, src_by = out_bx)
    # where N = out_y_cols (== rows for square)
    N = out_y_cols
    out_blocks = [[None] * N for _ in range(N)]
    for out_by in range(N):
        for out_bx in range(N):
            src_bx = N - 1 - out_by
            src_by = out_bx
            src_blk = y_blocks[src_by][src_bx]
            assert src_blk is not None
            out_blocks[out_by][out_bx] = rotate90ccw_block(src_blk)

    # Build output JPEG (Y-only, MCU = 8x8)
    out = bytearray()
    write_marker(out, M_SOI)
    # JFIF APP0 (optional but helps identification)
    write_seg(out, 0xE0, b"JFIF\x00" + struct.pack(">BBBHHBB", 1, 1, 0, 1, 1, 0, 0))
    # DQT[0] - Y quantization table only
    write_seg(out, M_DQT, serialize_dqt(0, info.dqt[0]))
    # SOF0: 1 component, swapped dims (still 384x384 since square)
    sof = struct.pack(">BHHB", 8, crop_h, crop_w, 1)
    sof += bytes([y_id, 0x11, 0])  # h=1, v=1, qt=0
    write_seg(out, M_SOF0, sof)
    # DHT: Y DC and Y AC
    write_seg(out, M_DHT, serialize_dht(0, 0, y_dc_ht))
    write_seg(out, M_DHT, serialize_dht(1, 0, y_ac_ht))
    # SOS: 1 component
    sos = bytes([1, y_id, 0x00, 0, 63, 0])  # dc/ac=0/0, Ss=0 Se=63 AhAl=0
    write_seg(out, M_SOS, sos)
    # Entropy data
    bw = BitWriter()
    dc_pred = 0
    for by in range(N):
        for bx in range(N):
            dc_pred = encode_block(out_blocks[by][bx], dc_pred, bw, y_dc_ht, y_ac_ht)
    bw.flush()
    out += bytes(bw.out)
    write_marker(out, M_EOI)
    return bytes(out)


# ============================================================
# CLI
# ============================================================
if __name__ == "__main__":
    src_path = sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1/f05.jpg"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/lossless_rotate_out.jpg"
    src = Path(src_path).read_bytes()
    out = process(src, crop_x=64, crop_y=48, crop_w=384, crop_h=384)
    Path(out_path).write_bytes(out)
    print(f"wrote {out_path} ({len(out)} bytes from {len(src)} input)")
