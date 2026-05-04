/*
 * jpeg_lossless_rotate.c — DCT-domain crop + 90 CCW rotate + drop-chroma.
 *
 * Algorithm (mirrors tools/jpeg_lossless_crop_rotate_gray.py):
 *
 *   Phase 1: parse markers (DQT, DHT, SOF0, SOS), build Huffman tables,
 *            locate entropy stream.
 *   Phase 2: decode source entropy stream MCU-by-MCU. For each MCU:
 *              read 2 Y blocks (because Y has h_samp=2)
 *              read 1 Cb block and 1 Cr block — discard
 *            Store Y blocks within crop region into a coefficient buffer
 *            (natural row-major order, NOT zigzag).
 *   Phase 3: emit output JPEG header (Y-only, dims swapped if asymmetric).
 *   Phase 4: walk output block grid; for each output block, source-locate
 *            the input block and apply per-block 90 CCW transpose:
 *              out[u*8 + v] = (-1)^u * src[v*8 + u]
 *            Then re-encode (zigzag + RLE + Huffman).
 *
 * Memory: 384x384 crop -> 48x48 Y blocks -> 48*48*64*2 = ~295 KB int16 buffer.
 * Allocates from PSRAM.
 */

#include "jpeg_lossless_rotate.h"
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"

/* ============================================================
 * Marker definitions
 * ============================================================ */
#define M_SOI  0xD8
#define M_EOI  0xD9
#define M_SOF0 0xC0
#define M_DHT  0xC4
#define M_DQT  0xDB
#define M_SOS  0xDA
#define M_APP0 0xE0

/* ============================================================
 * Standard JPEG zigzag order: ZZ[k] = natural-order index of
 * the k-th coefficient in zigzag scan.
 * ============================================================ */
static const uint8_t ZZ[64] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
};

/* ============================================================
 * Bit reader (with FF00 byte de-stuffing)
 * ============================================================ */
typedef struct {
    const uint8_t *data;
    size_t         len;
    size_t         pos;
    uint32_t       bits;
    int            nbits;
} BitReader;

static void br_init(BitReader *br, const uint8_t *data, size_t len, size_t pos) {
    br->data = data;
    br->len = len;
    br->pos = pos;
    br->bits = 0;
    br->nbits = 0;
}

static int br_read_byte(BitReader *br) {
    if (br->pos >= br->len) return -1;
    uint8_t b = br->data[br->pos++];
    if (b == 0xFF) {
        if (br->pos >= br->len) return -1;
        uint8_t n = br->data[br->pos++];
        if (n != 0x00) {
            /* It's a marker — back up so caller can handle it */
            br->pos -= 2;
            return -1;
        }
    }
    return b;
}

static int br_fill(BitReader *br, int n) {
    while (br->nbits < n) {
        int b = br_read_byte(br);
        if (b < 0) return -1;
        br->bits = (br->bits << 8) | (uint32_t)b;
        br->nbits += 8;
    }
    return 0;
}

static uint32_t br_peek(BitReader *br, int n) {
    return (br->bits >> (br->nbits - n)) & ((1u << n) - 1);
}

static void br_skip(BitReader *br, int n) {
    br->nbits -= n;
    br->bits &= (1u << br->nbits) - 1;
}

static int br_read(BitReader *br, int n) {
    if (n == 0) return 0;
    if (br_fill(br, n) < 0) return -1;
    int v = (int)br_peek(br, n);
    br_skip(br, n);
    return v;
}

/* ============================================================
 * Bit writer (with FF -> FF00 byte stuffing). Writes into
 * a caller-allocated buffer; no realloc.
 * ============================================================ */
typedef struct {
    uint8_t *out;
    size_t   cap;
    size_t   pos;
    uint32_t bits;
    int      nbits;
    int      err;
} BitWriter;

static void bw_init(BitWriter *bw, uint8_t *out, size_t cap) {
    bw->out = out;
    bw->cap = cap;
    bw->pos = 0;
    bw->bits = 0;
    bw->nbits = 0;
    bw->err = 0;
}

static int bw_write(BitWriter *bw, uint32_t v, int n) {
    if (n == 0) return 0;
    if (bw->err) return -1;
    bw->bits = (bw->bits << n) | (v & ((1u << n) - 1));
    bw->nbits += n;
    while (bw->nbits >= 8) {
        bw->nbits -= 8;
        uint8_t byte = (bw->bits >> bw->nbits) & 0xFF;
        bw->bits &= (1u << bw->nbits) - 1;
        if (bw->pos >= bw->cap) { bw->err = 1; return -1; }
        bw->out[bw->pos++] = byte;
        if (byte == 0xFF) {
            if (bw->pos >= bw->cap) { bw->err = 1; return -1; }
            bw->out[bw->pos++] = 0x00;
        }
    }
    return 0;
}

static int bw_flush(BitWriter *bw) {
    if (bw->err) return -1;
    if (bw->nbits > 0) {
        uint8_t byte = ((bw->bits << (8 - bw->nbits)) |
                        ((1u << (8 - bw->nbits)) - 1)) & 0xFF;
        if (bw->pos >= bw->cap) { bw->err = 1; return -1; }
        bw->out[bw->pos++] = byte;
        if (byte == 0xFF) {
            if (bw->pos >= bw->cap) { bw->err = 1; return -1; }
            bw->out[bw->pos++] = 0x00;
        }
        bw->nbits = 0;
        bw->bits = 0;
    }
    return 0;
}

/* ============================================================
 * Huffman table (decoder + encoder)
 * ============================================================ */
typedef struct {
    /* Decoder */
    int      maxcode[17];   /* maxcode[L] = code+count for length L */
    int      valoffset[17]; /* val index offset for each length */
    uint8_t  vals[256];     /* symbol values in code order */
    /* Encoder */
    uint16_t enc_code[256];
    uint8_t  enc_len[256];
    /* Original DHT data (for re-emission) */
    uint8_t  bits[16];
    int      nsym;
} HuffTable;

static int huff_build(HuffTable *ht, const uint8_t *bits, const uint8_t *vals) {
    memcpy(ht->bits, bits, 16);
    int nsym = 0;
    for (int i = 0; i < 16; i++) nsym += bits[i];
    ht->nsym = nsym;
    if (nsym > 256) return -1;
    memcpy(ht->vals, vals, nsym);

    /* Build decoder: maxcode and valoffset */
    int code = 0;
    int si = 0;
    for (int L = 1; L <= 16; L++) {
        int count = bits[L - 1];
        ht->maxcode[L] = code + count;
        ht->valoffset[L] = si - code;
        code = (code + count) << 1;
        si += count;
    }

    /* Build encoder */
    memset(ht->enc_len, 0, sizeof(ht->enc_len));
    code = 0;
    si = 0;
    for (int L = 1; L <= 16; L++) {
        int count = bits[L - 1];
        for (int i = 0; i < count; i++) {
            ht->enc_code[vals[si]] = (uint16_t)code;
            ht->enc_len[vals[si]] = (uint8_t)L;
            code++;
            si++;
        }
        code <<= 1;
    }
    return 0;
}

static int huff_decode(BitReader *br, const HuffTable *ht) {
    int code = 0;
    for (int L = 1; L <= 16; L++) {
        if (br_fill(br, L) < 0) return -1;
        code = (int)br_peek(br, L);
        if (ht->maxcode[L] > 0 && code < ht->maxcode[L]) {
            br_skip(br, L);
            return ht->vals[code + ht->valoffset[L]];
        }
    }
    return -1;
}

static int huff_encode(BitWriter *bw, const HuffTable *ht, int sym) {
    int L = ht->enc_len[sym];
    if (L == 0) return -1;
    return bw_write(bw, ht->enc_code[sym], L);
}

/* ============================================================
 * Coefficient value <-> (cat, bits) helpers
 * ============================================================ */
static int extend(int v, int cat) {
    if (cat == 0) return 0;
    int half = 1 << (cat - 1);
    return (v >= half) ? v : v - (2 * half - 1);
}

static void encode_value(int v, int *bits, int *nbits) {
    if (v == 0) { *bits = 0; *nbits = 0; return; }
    int av = (v < 0) ? -v : v;
    int n = 0;
    int t = av;
    while (t > 0) { n++; t >>= 1; }
    *nbits = n;
    *bits = (v >= 0) ? v : (v + (1 << n) - 1);
}

/* ============================================================
 * Decode one 8x8 block from entropy stream.
 * Returns 0 on success, -1 on error.
 * Writes coefs in NATURAL row-major order (un-zigzag).
 * Updates *dc_pred with new value.
 * ============================================================ */
static int decode_block(BitReader *br, int *dc_pred,
                        const HuffTable *dc_ht, const HuffTable *ac_ht,
                        int16_t *coefs) {
    memset(coefs, 0, 64 * sizeof(int16_t));
    /* DC */
    int dc_cat = huff_decode(br, dc_ht);
    if (dc_cat < 0) return -1;
    int dc_extra = br_read(br, dc_cat);
    if (dc_cat > 0 && dc_extra < 0) return -1;
    int dc_val = *dc_pred + extend(dc_extra, dc_cat);
    *dc_pred = dc_val;
    coefs[0] = (int16_t)dc_val;
    /* AC */
    int k = 1;
    while (k < 64) {
        int rs = huff_decode(br, ac_ht);
        if (rs < 0) return -1;
        int run = rs >> 4;
        int cat = rs & 0x0F;
        if (cat == 0) {
            if (run == 15) { k += 16; continue; }  /* ZRL */
            break;  /* EOB */
        }
        k += run;
        if (k >= 64) return -1;
        int extra = br_read(br, cat);
        if (extra < 0) return -1;
        coefs[ZZ[k]] = (int16_t)extend(extra, cat);
        k++;
    }
    return 0;
}

/* ============================================================
 * Encode one 8x8 block (zigzag scan, RLE, Huffman).
 * coefs in natural row-major order.
 * Returns new dc_pred on success, INT_MIN on error.
 * ============================================================ */
static int encode_block(const int16_t *coefs, int dc_pred,
                        BitWriter *bw,
                        const HuffTable *dc_ht, const HuffTable *ac_ht) {
    int dc_val = coefs[0];
    int dc_diff = dc_val - dc_pred;
    int bits, nbits;
    encode_value(dc_diff, &bits, &nbits);
    if (huff_encode(bw, dc_ht, nbits) < 0) return -1000000;
    if (nbits > 0 && bw_write(bw, (uint32_t)bits, nbits) < 0) return -1000000;

    /* AC: walk zigzag and find last non-zero */
    int last_nz = 0;
    for (int k = 1; k < 64; k++) {
        if (coefs[ZZ[k]] != 0) last_nz = k;
    }
    int run = 0;
    for (int k = 1; k <= last_nz; k++) {
        int v = coefs[ZZ[k]];
        if (v == 0) {
            run++;
            if (run == 16) {
                if (huff_encode(bw, ac_ht, 0xF0) < 0) return -1000000;  /* ZRL */
                run = 0;
            }
        } else {
            encode_value(v, &bits, &nbits);
            int sym = (run << 4) | nbits;
            if (huff_encode(bw, ac_ht, sym) < 0) return -1000000;
            if (bw_write(bw, (uint32_t)bits, nbits) < 0) return -1000000;
            run = 0;
        }
    }
    if (last_nz < 63) {
        if (huff_encode(bw, ac_ht, 0x00) < 0) return -1000000;  /* EOB */
    }
    return dc_val;
}

/* ============================================================
 * Per-block 90 CCW rotation in DCT domain.
 * out[u*8 + v] = (-1)^u * src[v*8 + u]
 * ============================================================ */
static void rotate90ccw_block(const int16_t *src, int16_t *out) {
    for (int u = 0; u < 8; u++) {
        int sign = (u & 1) ? -1 : 1;
        for (int v = 0; v < 8; v++) {
            out[u * 8 + v] = (int16_t)(sign * src[v * 8 + u]);
        }
    }
}

/* ============================================================
 * Marker writer helpers
 * ============================================================ */
static int write_byte(uint8_t *buf, size_t cap, size_t *pos, uint8_t b) {
    if (*pos >= cap) return -1;
    buf[(*pos)++] = b;
    return 0;
}

static int write_marker(uint8_t *buf, size_t cap, size_t *pos, uint8_t m) {
    if (write_byte(buf, cap, pos, 0xFF) < 0) return -1;
    return write_byte(buf, cap, pos, m);
}

static int write_seg(uint8_t *buf, size_t cap, size_t *pos, uint8_t marker,
                     const uint8_t *payload, size_t paylen) {
    if (write_marker(buf, cap, pos, marker) < 0) return -1;
    /* segment length includes the 2-byte length field */
    size_t total = paylen + 2;
    if (total > 0xFFFF) return -1;
    if (*pos + 2 + paylen > cap) return -1;
    buf[(*pos)++] = (uint8_t)(total >> 8);
    buf[(*pos)++] = (uint8_t)(total & 0xFF);
    memcpy(buf + *pos, payload, paylen);
    *pos += paylen;
    return 0;
}

static uint16_t read16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

/* ============================================================
 * Main entry
 * ============================================================ */
uint8_t *jpeg_lossless_crop_rotate_gray(
    const uint8_t *src, size_t srcLen,
    int cropX, int cropY, int cropW, int cropH,
    size_t *outLen)
{
    *outLen = 0;
    if ((cropX % 16) || (cropY % 8)) return NULL;
    if ((cropW % 16) || (cropH % 8)) return NULL;

    /* ---- Phase 1: parse source markers ---- */
    HuffTable dc_tables[4];
    HuffTable ac_tables[4];
    uint8_t   dqt[4][64];
    int       dqt_present[4] = {0};
    int       img_w = 0, img_h = 0;
    int       y_id = 0, cb_id = 0, cr_id = 0;
    int       y_qid = 0;
    int       y_dc_t = 0, y_ac_t = 0;
    int       cb_dc_t = 0, cb_ac_t = 0;
    int       cr_dc_t = 0, cr_ac_t = 0;
    int       y_hsamp = 0, y_vsamp = 0;
    size_t    sos_scan_start = 0;

    size_t i = 0;
    while (i + 1 < srcLen) {
        if (src[i] != 0xFF) { i++; continue; }
        uint8_t m = src[i + 1];
        if (m == M_SOI) { i += 2; continue; }
        if (m == M_EOI) break;
        if (i + 4 > srcLen) return NULL;
        size_t seg_len = read16(src + i + 2);
        if (i + 2 + seg_len > srcLen) return NULL;
        const uint8_t *seg = src + i + 2;

        if (m == M_DQT) {
            size_t j = 2;
            while (j < seg_len) {
                uint8_t pq_tq = seg[j++];
                int tq = pq_tq & 0x0F;
                int pq = pq_tq >> 4;
                if (pq != 0) return NULL;            /* 16-bit DQT not supported */
                if (tq >= 4) return NULL;
                if (j + 64 > seg_len) return NULL;
                memcpy(dqt[tq], seg + j, 64);
                dqt_present[tq] = 1;
                j += 64;
            }
        } else if (m == M_DHT) {
            size_t j = 2;
            while (j < seg_len) {
                uint8_t tc_th = seg[j++];
                int th = tc_th & 0x0F;
                int tc = tc_th >> 4;
                if (th >= 4) return NULL;
                if (j + 16 > seg_len) return NULL;
                const uint8_t *bits = seg + j;
                j += 16;
                int nsym = 0;
                for (int k = 0; k < 16; k++) nsym += bits[k];
                if (j + nsym > seg_len) return NULL;
                const uint8_t *vals = seg + j;
                j += nsym;
                HuffTable *target = (tc == 0) ? &dc_tables[th] : &ac_tables[th];
                if (huff_build(target, bits, vals) < 0) return NULL;
            }
        } else if (m == M_SOF0) {
            if (seg_len < 8) return NULL;
            int prec = seg[2];
            if (prec != 8) return NULL;
            img_h = read16(seg + 3);
            img_w = read16(seg + 5);
            int ncomp = seg[7];
            if (ncomp != 3) return NULL;             /* expect YCbCr 4:2:2 */
            if (8 + ncomp * 3 > (int)seg_len) return NULL;
            for (int c = 0; c < 3; c++) {
                int cid = seg[8 + c * 3];
                int samp = seg[9 + c * 3];
                int qid = seg[10 + c * 3];
                int hs = samp >> 4;
                int vs = samp & 0xF;
                if (c == 0) {
                    y_id = cid; y_qid = qid;
                    y_hsamp = hs; y_vsamp = vs;
                } else if (c == 1) {
                    cb_id = cid;
                } else {
                    cr_id = cid;
                }
            }
            if (y_hsamp != 2 || y_vsamp != 1) return NULL;  /* require 4:2:2 */
        } else if (m == M_SOS) {
            if (seg_len < 6) return NULL;
            int ncomp = seg[2];
            if (ncomp != 3) return NULL;
            if (3 + ncomp * 2 > (int)seg_len) return NULL;
            for (int c = 0; c < 3; c++) {
                int cid = seg[3 + c * 2];
                uint8_t tab = seg[3 + c * 2 + 1];
                int dc = tab >> 4;
                int ac = tab & 0xF;
                if (cid == y_id)      { y_dc_t = dc; y_ac_t = ac; }
                else if (cid == cb_id) { cb_dc_t = dc; cb_ac_t = ac; }
                else if (cid == cr_id) { cr_dc_t = dc; cr_ac_t = ac; }
            }
            sos_scan_start = i + 2 + seg_len;
            i = sos_scan_start;
            goto done_parse;
        }
        i += 2 + seg_len;
    }
done_parse:
    if (img_w == 0 || img_h == 0) return NULL;
    if (sos_scan_start == 0) return NULL;
    if (cropX + cropW > img_w || cropY + cropH > img_h) return NULL;
    if (!dqt_present[y_qid]) return NULL;

    /* ---- MCU geometry (4:2:2 -> MCU = 16x8 px) ---- */
    const int mcu_w_px = 16, mcu_h_px = 8;
    int src_mcu_cols = (img_w + mcu_w_px - 1) / mcu_w_px;
    int src_mcu_rows = (img_h + mcu_h_px - 1) / mcu_h_px;
    int crop_mcu_x = cropX / mcu_w_px;
    int crop_mcu_y = cropY / mcu_h_px;
    int crop_mcu_cols = cropW / mcu_w_px;
    int crop_mcu_rows = cropH / mcu_h_px;
    /* Y block grid in INPUT coords (within crop region):
     *   width  Y_W = 2 * crop_mcu_cols (each MCU has 2 Y blocks horizontally)
     *   height Y_H = crop_mcu_rows
     * After 90 CCW rotation, OUTPUT block grid:
     *   width  Y_W' = Y_H, height Y_H' = Y_W
     */
    int Y_W = 2 * crop_mcu_cols;       /* input Y blocks wide */
    int Y_H = crop_mcu_rows;           /* input Y blocks tall */
    int OUT_W = Y_H;                   /* output Y blocks wide */
    int OUT_H = Y_W;                   /* output Y blocks tall */
    int out_pix_w = cropH;             /* output image width  in pixels */
    int out_pix_h = cropW;             /* output image height in pixels */

    /* ---- Allocate Y coefficient buffer in PSRAM ----
     * Layout: y_blocks[by * Y_W + bx] = int16_t[64] in natural order.
     * For 384x384, Y_W=Y_H=48 -> 48*48*64*2 = 294912 bytes (~288 KB).
     */
    size_t y_buf_count = (size_t)Y_W * Y_H;
    int16_t *y_blocks = (int16_t *)heap_caps_malloc(
        y_buf_count * 64 * sizeof(int16_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!y_blocks) return NULL;

    /* ---- Phase 2: decode source entropy stream ---- */
    BitReader br;
    br_init(&br, src, srcLen, sos_scan_start);
    int dc_y = 0, dc_cb = 0, dc_cr = 0;
    int16_t scratch[64];

    HuffTable *y_dc_ht = &dc_tables[y_dc_t];
    HuffTable *y_ac_ht = &ac_tables[y_ac_t];
    HuffTable *cb_dc_ht = &dc_tables[cb_dc_t];
    HuffTable *cb_ac_ht = &ac_tables[cb_ac_t];
    HuffTable *cr_dc_ht = &dc_tables[cr_dc_t];
    HuffTable *cr_ac_ht = &ac_tables[cr_ac_t];

    for (int mr = 0; mr < src_mcu_rows; mr++) {
        for (int mc = 0; mc < src_mcu_cols; mc++) {
            int in_y = (mr >= crop_mcu_y && mr < crop_mcu_y + crop_mcu_rows &&
                        mc >= crop_mcu_x && mc < crop_mcu_x + crop_mcu_cols);
            int by_dst = (mr - crop_mcu_y);
            int bx_dst = 2 * (mc - crop_mcu_x);

            /* 2 Y blocks (left, right) */
            int16_t *slot0 = in_y
                ? &y_blocks[(by_dst * Y_W + bx_dst) * 64]
                : scratch;
            if (decode_block(&br, &dc_y, y_dc_ht, y_ac_ht, slot0) < 0) goto fail;

            int16_t *slot1 = in_y
                ? &y_blocks[(by_dst * Y_W + bx_dst + 1) * 64]
                : scratch;
            if (decode_block(&br, &dc_y, y_dc_ht, y_ac_ht, slot1) < 0) goto fail;

            /* Cb, Cr — decode but discard */
            if (decode_block(&br, &dc_cb, cb_dc_ht, cb_ac_ht, scratch) < 0) goto fail;
            if (decode_block(&br, &dc_cr, cr_dc_ht, cr_ac_ht, scratch) < 0) goto fail;
        }
    }

    /* ---- Phase 3: build output JPEG ---- */
    /* Output buffer: source size is generous upper bound */
    size_t out_cap = srcLen + 1024;
    uint8_t *out = (uint8_t *)heap_caps_malloc(out_cap,
                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!out) goto fail;

    size_t op = 0;
    if (write_marker(out, out_cap, &op, M_SOI) < 0) goto fail2;

    /* Minimal JFIF APP0: "JFIF\0" version 1.1, units=0, density 1x1, 0x0 thumb */
    {
        uint8_t app0[14] = {
            'J','F','I','F', 0x00, 0x01, 0x01, 0x00,
            0x00, 0x01, 0x00, 0x01, 0x00, 0x00
        };
        if (write_seg(out, out_cap, &op, M_APP0, app0, sizeof(app0)) < 0) goto fail2;
    }

    /* DQT - Y table only (qid mapped to 0 in output) */
    {
        uint8_t pl[65];
        pl[0] = 0x00;  /* prec=0, tq=0 */
        memcpy(pl + 1, dqt[y_qid], 64);
        if (write_seg(out, out_cap, &op, M_DQT, pl, sizeof(pl)) < 0) goto fail2;
    }

    /* SOF0: 1 component. After 90 deg rotation, dimensions are swapped:
     * output width = cropH (input height), output height = cropW (input width).
     * Payload: prec(1) H(2) W(2) Nf(1) [Ci(1) HV(1) Tq(1)]*Nf
     * For 1 component: 6 + 3 = 9 bytes total.
     */
    {
        uint8_t pl[9];
        pl[0] = 8;                          /* precision */
        pl[1] = (uint8_t)(out_pix_h >> 8);  /* height */
        pl[2] = (uint8_t)(out_pix_h & 0xFF);
        pl[3] = (uint8_t)(out_pix_w >> 8);  /* width */
        pl[4] = (uint8_t)(out_pix_w & 0xFF);
        pl[5] = 1;                          /* Nf */
        pl[6] = (uint8_t)y_id;              /* component id */
        pl[7] = 0x11;                       /* h=1 v=1 */
        pl[8] = 0x00;                       /* qt=0 */
        if (write_seg(out, out_cap, &op, M_SOF0, pl, 9) < 0) goto fail2;
    }

    /* DHT for Y DC (class 0, dest 0) */
    {
        const HuffTable *ht = y_dc_ht;
        uint8_t pl[1 + 16 + 256];
        pl[0] = 0x00;  /* tc=0 th=0 */
        memcpy(pl + 1, ht->bits, 16);
        memcpy(pl + 1 + 16, ht->vals, ht->nsym);
        if (write_seg(out, out_cap, &op, M_DHT, pl, 1 + 16 + ht->nsym) < 0) goto fail2;
    }
    /* DHT for Y AC (class 1, dest 0) */
    {
        const HuffTable *ht = y_ac_ht;
        uint8_t pl[1 + 16 + 256];
        pl[0] = 0x10;  /* tc=1 th=0 */
        memcpy(pl + 1, ht->bits, 16);
        memcpy(pl + 1 + 16, ht->vals, ht->nsym);
        if (write_seg(out, out_cap, &op, M_DHT, pl, 1 + 16 + ht->nsym) < 0) goto fail2;
    }

    /* SOS: 1 component */
    {
        uint8_t pl[6];
        pl[0] = 1;
        pl[1] = (uint8_t)y_id;
        pl[2] = 0x00;  /* DC=0 AC=0 */
        pl[3] = 0;     /* Ss */
        pl[4] = 63;    /* Se */
        pl[5] = 0;     /* AhAl */
        if (write_seg(out, out_cap, &op, M_SOS, pl, 6) < 0) goto fail2;
    }

    /* ---- Phase 4: emit rotated entropy stream ---- */
    BitWriter bw;
    bw_init(&bw, out + op, out_cap - op - 2);  /* leave room for EOI */
    int dc_pred = 0;
    int16_t rot_blk[64];
    /* Walk OUTPUT block grid (OUT_W x OUT_H).
     * For 90 CCW: output (out_bx, out_by) <- input (src_bx = Y_W-1-out_by, src_by = out_bx)
     */
    for (int out_by = 0; out_by < OUT_H; out_by++) {
        for (int out_bx = 0; out_bx < OUT_W; out_bx++) {
            int src_bx = Y_W - 1 - out_by;
            int src_by = out_bx;
            const int16_t *src_blk = &y_blocks[(src_by * Y_W + src_bx) * 64];
            rotate90ccw_block(src_blk, rot_blk);
            int new_dc = encode_block(rot_blk, dc_pred, &bw, y_dc_ht, y_ac_ht);
            if (new_dc <= -1000000) goto fail2;
            dc_pred = new_dc;
        }
    }
    if (bw_flush(&bw) < 0) goto fail2;
    op += bw.pos;

    /* EOI */
    if (write_marker(out, out_cap, &op, M_EOI) < 0) goto fail2;

    free(y_blocks);
    *outLen = op;
    return out;

fail2:
    free(out);
fail:
    free(y_blocks);
    return NULL;
}
