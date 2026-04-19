/*
 * jpeg_lossless_crop.c — Lossless JPEG crop by MCU block extraction.
 *
 * Instead of decode→crop→re-encode (420ms on ESP32), this:
 *   1. Copies JPEG headers, updating dimensions in SOF0
 *   2. Huffman-decodes MCU blocks (bitstream parsing only, no IDCT)
 *   3. Re-encodes only the MCU blocks within the crop region
 *   4. Adjusts DC coefficient deltas at crop-row boundaries
 *
 * Supports baseline sequential JPEG with any subsampling (tested with 4:2:2).
 * Expected performance: ~15-30ms on ESP32-S3 vs ~420ms for full decode/re-encode.
 */

#include "jpeg_lossless_crop.h"
#include <string.h>
#include <stdlib.h>

#ifdef JPEG_CROP_USE_PSRAM
#include "esp_heap_caps.h"
static void *crop_ps_malloc(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}
#define CROP_MALLOC(sz) crop_ps_malloc(sz)
#else
#define CROP_MALLOC(sz) malloc(sz)
#endif

/* ---------- JPEG marker definitions ---------- */
#define M_SOI  0xD8
#define M_EOI  0xD9
#define M_SOF0 0xC0
#define M_DHT  0xC4
#define M_DQT  0xDB
#define M_SOS  0xDA
#define M_DRI  0xDD
#define M_APP0 0xE0
#define M_COM  0xFE

/* ---------- Huffman table ---------- */
#define MAX_HUFFMAN_TABLES 4
typedef struct {
    /* Fast lookup: huffval[code] for codes up to 8 bits */
    uint8_t  fast_val[256];   /* symbol value for 8-bit prefix */
    uint8_t  fast_len[256];   /* code length (0 = not an 8-bit code) */
    /* Slow path for codes > 8 bits */
    int      maxcode[17];     /* max code value +1 for each length, -1 if none */
    int      valoffset[17];   /* offset into vals[] for each length */
    uint8_t  vals[256];       /* symbol values in order */
    int      num_symbols;
} HuffTable;

/* ---------- Component info from SOF ---------- */
#define MAX_COMPONENTS 4
typedef struct {
    uint8_t id;
    uint8_t hsamp;   /* horizontal sampling factor */
    uint8_t vsamp;   /* vertical sampling factor */
    uint8_t qt_id;
    uint8_t dc_table; /* assigned in SOS */
    uint8_t ac_table; /* assigned in SOS */
} CompInfo;

/* ---------- Bit reader ---------- */
typedef struct {
    const uint8_t *data;
    size_t         len;
    size_t         pos;       /* byte position */
    uint32_t       bits;      /* bit buffer */
    int            nbits;     /* valid bits in buffer */
} BitReader;

static void br_init(BitReader *br, const uint8_t *data, size_t len, size_t pos) {
    br->data = data;
    br->len = len;
    br->pos = pos;
    br->bits = 0;
    br->nbits = 0;
}

/* Read next byte, handling FF00 stuffing */
static int br_read_byte(BitReader *br) {
    if (br->pos >= br->len) return -1;
    uint8_t b = br->data[br->pos++];
    if (b == 0xFF) {
        if (br->pos >= br->len) return -1;
        uint8_t next = br->data[br->pos++];
        if (next != 0x00) return -1; /* unexpected marker */
    }
    return b;
}

/* Ensure at least n bits in buffer */
static int br_fill(BitReader *br, int n) {
    while (br->nbits < n) {
        int b = br_read_byte(br);
        if (b < 0) return -1;
        br->bits = (br->bits << 8) | (uint32_t)b;
        br->nbits += 8;
    }
    return 0;
}

/* Peek top n bits without consuming */
static uint32_t br_peek(BitReader *br, int n) {
    return (br->bits >> (br->nbits - n)) & ((1u << n) - 1);
}

/* Consume n bits */
static void br_skip(BitReader *br, int n) {
    br->nbits -= n;
}

/* Read n bits and consume */
static int br_read(BitReader *br, int n) {
    if (n == 0) return 0;
    if (br_fill(br, n) < 0) return -1;
    uint32_t val = br_peek(br, n);
    br_skip(br, n);
    return (int)val;
}

/* ---------- Bit writer ---------- */
typedef struct {
    uint8_t *data;
    size_t   cap;
    size_t   pos;
    uint32_t bits;
    int      nbits;
} BitWriter;

static void bw_init(BitWriter *bw, uint8_t *data, size_t cap) {
    bw->data = data;
    bw->cap = cap;
    bw->pos = 0;
    bw->bits = 0;
    bw->nbits = 0;
}

/* Flush a single byte, with FF stuffing */
static int bw_write_byte(BitWriter *bw, uint8_t b) {
    if (bw->pos >= bw->cap) return -1;
    bw->data[bw->pos++] = b;
    if (b == 0xFF) {
        if (bw->pos >= bw->cap) return -1;
        bw->data[bw->pos++] = 0x00;
    }
    return 0;
}

/* Write n bits (MSB first) */
static int bw_write(BitWriter *bw, uint32_t val, int n) {
    bw->bits = (bw->bits << n) | (val & ((1u << n) - 1));
    bw->nbits += n;
    while (bw->nbits >= 8) {
        bw->nbits -= 8;
        uint8_t b = (uint8_t)(bw->bits >> bw->nbits);
        if (bw_write_byte(bw, b) < 0) return -1;
    }
    return 0;
}

/* Pad remaining bits with 1s and flush */
static int bw_flush(BitWriter *bw) {
    if (bw->nbits > 0) {
        int pad = 8 - bw->nbits;
        return bw_write(bw, (1u << pad) - 1, pad);
    }
    return 0;
}

/* ---------- Huffman table building ---------- */
static int build_hufftable(HuffTable *ht, const uint8_t *bits, const uint8_t *vals, int nsymbols) {
    ht->num_symbols = nsymbols;
    memcpy(ht->vals, vals, nsymbols);

    /* Build maxcode/valoffset tables */
    int code = 0;
    int si = 0;
    memset(ht->fast_len, 0, sizeof(ht->fast_len));

    for (int len = 1; len <= 16; len++) {
        int count = bits[len - 1];
        if (count == 0) {
            ht->maxcode[len] = -1;
            ht->valoffset[len] = 0;
        } else {
            ht->valoffset[len] = si - code;
            for (int i = 0; i < count; i++) {
                /* Build fast 8-bit lookup for short codes */
                if (len <= 8) {
                    int prefix = code << (8 - len);
                    int fill = 1 << (8 - len);
                    for (int f = 0; f < fill; f++) {
                        ht->fast_val[prefix + f] = vals[si];
                        ht->fast_len[prefix + f] = (uint8_t)len;
                    }
                }
                code++;
                si++;
            }
            ht->maxcode[len] = code;
        }
        code <<= 1;
    }
    return 0;
}

/* Decode one Huffman symbol */
static int huff_decode(BitReader *br, HuffTable *ht) {
    if (br_fill(br, 16) < 0) {
        /* Try with what we have */
        if (br->nbits == 0) return -1;
    }

    /* Fast path: 8-bit lookup */
    if (br->nbits >= 8) {
        uint32_t peek = br_peek(br, 8);
        int len = ht->fast_len[peek];
        if (len > 0) {
            br_skip(br, len);
            return ht->fast_val[peek];
        }
    }

    /* Slow path: use bits already in buffer, check code length by length */
    int code = 0;
    for (int len = 1; len <= 16; len++) {
        if (len > br->nbits) {
            if (br_fill(br, len) < 0) return -1;
        }
        /* Read the top 'len' bits as the code candidate */
        code = (int)br_peek(br, len);
        if (ht->maxcode[len] > 0 && code < ht->maxcode[len]) {
            br_skip(br, len);
            return ht->vals[code + ht->valoffset[len]];
        }
    }
    return -1; /* invalid code */
}

/* ---------- DC coefficient encoding/decoding helpers ---------- */

/* Sign-extend a value based on category (bit length) */
static int extend(int val, int cat) {
    if (cat == 0) return 0;
    int half = 1 << (cat - 1);
    if (val < half) {
        val = val - (2 * half - 1);
    }
    return val;
}

/* Encode a DC/AC coefficient value: returns (bits, nbits) for the magnitude */
static void encode_value(int val, int *out_bits, int *out_nbits) {
    if (val == 0) {
        *out_bits = 0;
        *out_nbits = 0;
        return;
    }
    int absval = val < 0 ? -val : val;
    int nbits = 0;
    int tmp = absval;
    while (tmp > 0) { nbits++; tmp >>= 1; }
    int bits = val < 0 ? val + (1 << nbits) - 1 : val;
    *out_bits = bits;
    *out_nbits = nbits;
}

/* ---------- Huffman encoding helper ---------- */

/* Encode a Huffman symbol: need the code and length for each symbol.
   We build these from the same bits/vals arrays used for decoding. */
typedef struct {
    uint16_t code[256];
    uint8_t  len[256];
} HuffEnc;

static void build_huff_enc(HuffEnc *he, const uint8_t *bits, const uint8_t *vals, int nsymbols) {
    memset(he->len, 0, sizeof(he->len));
    int code = 0;
    int si = 0;
    for (int l = 1; l <= 16; l++) {
        int count = bits[l - 1];
        for (int i = 0; i < count; i++) {
            he->code[vals[si]] = (uint16_t)code;
            he->len[vals[si]] = (uint8_t)l;
            code++;
            si++;
        }
        code <<= 1;
    }
    (void)nsymbols;
}

/* Write a Huffman-coded symbol */
static int huff_encode(BitWriter *bw, HuffEnc *he, uint8_t symbol) {
    int len = he->len[symbol];
    if (len == 0) return -1; /* symbol not in table */
    return bw_write(bw, he->code[symbol], len);
}

/* ---------- Main crop function ---------- */

/* Read a 16-bit big-endian value */
static uint16_t read16(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | p[1];
}

/* Write a 16-bit big-endian value */
static void write16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xFF);
}

/* ---------- Heap-allocated context (too large for stack on ESP32) ---------- */
typedef struct {
    HuffTable   dc_tables[MAX_HUFFMAN_TABLES];
    HuffTable   ac_tables[MAX_HUFFMAN_TABLES];
    HuffEnc     dc_enc[MAX_HUFFMAN_TABLES];
    HuffEnc     ac_enc[MAX_HUFFMAN_TABLES];
    uint8_t     dht_bits[MAX_HUFFMAN_TABLES * 2][16];
    uint8_t     dht_vals[MAX_HUFFMAN_TABLES * 2][256];
    int         dht_nsym[MAX_HUFFMAN_TABLES * 2];
} CropContext;

uint8_t *jpeg_lossless_crop(
    const uint8_t *src, size_t srcLen,
    int cropX, int cropY,
    int cropW, int cropH,
    size_t *outLen)
{
    *outLen = 0;

    /* Allocate context on heap — ~15KB, too large for ESP32 task stacks */
    CropContext *ctx = (CropContext *)CROP_MALLOC(sizeof(CropContext));
    if (!ctx) return NULL;

    /* --- Phase 1: Parse headers --- */
    int ncomp = 0;
    CompInfo comps[MAX_COMPONENTS];
    int maxHSamp = 1, maxVSamp = 1;
    int imgW = 0, imgH = 0;

    memset(ctx->dc_tables, 0, sizeof(ctx->dc_tables));
    memset(ctx->ac_tables, 0, sizeof(ctx->ac_tables));
    memset(ctx->dht_nsym, 0, sizeof(ctx->dht_nsym));

    size_t sof_offset = 0; /* offset of SOF0 marker in src, for patching dimensions */
    size_t sos_scan_start = 0; /* byte offset where entropy data begins */
    size_t headers_end = 0; /* byte after SOS (start of scan data) */

    /* Number of components in the scan */
    int scan_ncomp = 0;

    size_t pos = 0;
    if (pos + 2 > srcLen || src[pos] != 0xFF || src[pos+1] != M_SOI) {
        free(ctx);
        return NULL;
    }
    pos += 2;

    while (pos + 2 <= srcLen) {
        if (src[pos] != 0xFF) return NULL;
        uint8_t marker = src[pos + 1];
        pos += 2;

        if (marker == M_EOI) break;

        if (marker == M_SOS) {
            /* SOS: parse component → table assignments */
            if (pos + 2 > srcLen) { free(ctx); return NULL; }
            uint16_t len = read16(src + pos);
            scan_ncomp = src[pos + 2];
            for (int i = 0; i < scan_ncomp; i++) {
                uint8_t cs = src[pos + 3 + i * 2];
                uint8_t td_ta = src[pos + 4 + i * 2];
                for (int c = 0; c < ncomp; c++) {
                    if (comps[c].id == cs) {
                        comps[c].dc_table = td_ta >> 4;
                        comps[c].ac_table = td_ta & 0x0F;
                    }
                }
            }
            sos_scan_start = pos + len;
            headers_end = pos + len;
            break;
        }

        if (pos + 2 > srcLen) { free(ctx); return NULL; }
        uint16_t seg_len = read16(src + pos);
        if (pos + seg_len > srcLen) { free(ctx); return NULL; }

        if (marker == M_SOF0) {
            sof_offset = pos - 2; /* points to FF C0 */
            imgH = read16(src + pos + 3);
            imgW = read16(src + pos + 5);
            ncomp = src[pos + 7];
            if (ncomp > MAX_COMPONENTS) { free(ctx); return NULL; }
            for (int i = 0; i < ncomp; i++) {
                comps[i].id = src[pos + 8 + i * 3];
                uint8_t sf = src[pos + 9 + i * 3];
                comps[i].hsamp = sf >> 4;
                comps[i].vsamp = sf & 0x0F;
                comps[i].qt_id = src[pos + 10 + i * 3];
                if (comps[i].hsamp > maxHSamp) maxHSamp = comps[i].hsamp;
                if (comps[i].vsamp > maxVSamp) maxVSamp = comps[i].vsamp;
            }
        } else if (marker == M_DHT) {
            /* May contain multiple tables */
            size_t dht_pos = pos + 2; /* skip length */
            size_t dht_end = pos + seg_len;
            while (dht_pos < dht_end) {
                uint8_t info = src[dht_pos++];
                int tc = info >> 4; /* 0=DC, 1=AC */
                int th = info & 0x0F;
                if (th >= MAX_HUFFMAN_TABLES) { free(ctx); return NULL; }

                uint8_t bits[16];
                memcpy(bits, src + dht_pos, 16);
                dht_pos += 16;

                int nsym = 0;
                for (int i = 0; i < 16; i++) nsym += bits[i];

                int idx = tc * MAX_HUFFMAN_TABLES + th;
                memcpy(ctx->dht_bits[idx], bits, 16);
                memcpy(ctx->dht_vals[idx], src + dht_pos, nsym);
                ctx->dht_nsym[idx] = nsym;

                if (tc == 0) {
                    build_hufftable(&ctx->dc_tables[th], bits, src + dht_pos, nsym);
                    build_huff_enc(&ctx->dc_enc[th], bits, src + dht_pos, nsym);
                } else {
                    build_hufftable(&ctx->ac_tables[th], bits, src + dht_pos, nsym);
                    build_huff_enc(&ctx->ac_enc[th], bits, src + dht_pos, nsym);
                }
                dht_pos += nsym;
            }
        }

        pos += seg_len;
    }

    /* Validate alignment */
    int mcuW = maxHSamp * 8;
    int mcuH = maxVSamp * 8;
    if (cropX % mcuW != 0 || cropY % mcuH != 0 ||
        cropW % mcuW != 0 || cropH % mcuH != 0) {
        free(ctx); return NULL;
    }
    if (cropX + cropW > imgW || cropY + cropH > imgH) {
        free(ctx); return NULL;
    }

    int imgMcuCols = (imgW + mcuW - 1) / mcuW;
    int imgMcuRows = (imgH + mcuH - 1) / mcuH;
    int cropMcuX = cropX / mcuW;
    int cropMcuY = cropY / mcuH;
    int cropMcuCols = cropW / mcuW;
    int cropMcuRows = cropH / mcuH;

    (void)imgMcuRows; /* used implicitly in scan traversal */

    /* --- Phase 2: Build output headers --- */
    /* Allocate output: headers + scan data (at most as big as input) */
    size_t outCap = srcLen; /* conservative: output <= input */
    uint8_t *out = (uint8_t *)CROP_MALLOC(outCap);
    if (!out) { free(ctx); return NULL; }

    /* Copy all headers from input up to (but not including) the SOS scan data,
       patching the SOF0 dimensions */
    size_t outPos = 0;

    /* We need to patch SOF0 in the copied headers. */
    /* SOF0 dimensions are at sof_offset + 2 (marker) + 2 (length) + 1 (precision) = sof_offset + 5 */
    /* Actually: sof_offset points to FF C0. Length starts at sof_offset+2.
       Data: [len:2][prec:1][H:2][W:2]... so H is at sof_offset+5, W is at sof_offset+7 */
    /* But sof_offset = pos-2 where pos pointed right after FF C0, so sof_offset is
       the offset of FF. The length field is at sof_offset+2. */

    /* Copy everything from SOI up to and including SOS header */
    memcpy(out, src, headers_end);
    outPos = headers_end;

    /* Patch SOF0 height and width. SOF0 header:
       FF C0 [len:2] [prec:1] [height:2] [width:2] ...
       sof_offset points to FF, so height is at sof_offset+5, width at sof_offset+7 */
    write16(out + sof_offset + 5, (uint16_t)cropH);
    write16(out + sof_offset + 7, (uint16_t)cropW);

    /* --- Phase 3: Decode input scan, re-encode crop region --- */
    BitReader br;
    br_init(&br, src, srcLen, sos_scan_start);

    BitWriter bw;
    bw_init(&bw, out + outPos, outCap - outPos - 2); /* -2 for EOI */

    /* DC prediction for each component (in input stream) */
    int dc_pred[MAX_COMPONENTS];
    memset(dc_pred, 0, sizeof(dc_pred));

    /* DC prediction for output stream */
    int dc_pred_out[MAX_COMPONENTS];
    memset(dc_pred_out, 0, sizeof(dc_pred_out));

    /* Process all MCU rows */
    for (int mcuRow = 0; mcuRow < imgMcuRows; mcuRow++) {
        for (int mcuCol = 0; mcuCol < imgMcuCols; mcuCol++) {

            int inCropRegion = (mcuRow >= cropMcuY && mcuRow < cropMcuY + cropMcuRows &&
                                mcuCol >= cropMcuX && mcuCol < cropMcuX + cropMcuCols);

            /* Process each component's blocks in this MCU */
            for (int c = 0; c < ncomp; c++) {
                int nblocks = comps[c].hsamp * comps[c].vsamp;
                HuffTable *dc_ht = &ctx->dc_tables[comps[c].dc_table];
                HuffTable *ac_ht = &ctx->ac_tables[comps[c].ac_table];
                HuffEnc *dc_he = &ctx->dc_enc[comps[c].dc_table];
                HuffEnc *ac_he = &ctx->ac_enc[comps[c].ac_table];

                for (int b = 0; b < nblocks; b++) {
                    /* --- Decode DC coefficient --- */
                    int dc_cat = huff_decode(&br, dc_ht);
                    if (dc_cat < 0) goto fail;
                    int dc_extra = br_read(&br, dc_cat);
                    if (dc_cat > 0 && dc_extra < 0) goto fail;
                    int dc_diff = extend(dc_extra, dc_cat);
                    int dc_val = dc_pred[c] + dc_diff;
                    dc_pred[c] = dc_val;

                    /* --- Decode AC coefficients (we need to parse them even if skipping) --- */
                    /* Store run-length pairs for re-encoding */
                    uint8_t ac_rs[63];   /* rs byte (run<<4 | size) */
                    int     ac_extra_val[63]; /* extra bits value */
                    int     ac_count = 0;

                    int k = 1;
                    while (k < 64) {
                        int rs = huff_decode(&br, ac_ht);
                        if (rs < 0) goto fail;
                        int run = rs >> 4;
                        int cat = rs & 0x0F;

                        if (cat == 0) {
                            if (run == 0) {
                                /* EOB */
                                ac_rs[ac_count] = 0x00;
                                ac_extra_val[ac_count] = 0;
                                ac_count++;
                                break;
                            } else if (run == 15) {
                                /* ZRL: skip 16 zeros */
                                ac_rs[ac_count] = 0xF0;
                                ac_extra_val[ac_count] = 0;
                                ac_count++;
                                k += 16;
                                continue;
                            }
                        }

                        int extra = br_read(&br, cat);
                        if (cat > 0 && extra < 0) goto fail;

                        ac_rs[ac_count] = (uint8_t)rs;
                        ac_extra_val[ac_count] = extra;
                        ac_count++;

                        k += run + 1;
                    }

                    /* --- Re-encode if in crop region --- */
                    if (inCropRegion) {
                        /* DC: encode delta from output prediction */
                        int out_dc_diff = dc_val - dc_pred_out[c];
                        dc_pred_out[c] = dc_val;

                        int dc_bits, dc_nbits;
                        encode_value(out_dc_diff, &dc_bits, &dc_nbits);

                        if (huff_encode(&bw, dc_he, (uint8_t)dc_nbits) < 0) goto fail;
                        if (dc_nbits > 0) {
                            if (bw_write(&bw, (uint32_t)dc_bits, dc_nbits) < 0) goto fail;
                        }

                        /* AC: re-encode stored run-length pairs */
                        for (int i = 0; i < ac_count; i++) {
                            if (huff_encode(&bw, ac_he, ac_rs[i]) < 0) goto fail;
                            int cat = ac_rs[i] & 0x0F;
                            if (cat > 0) {
                                if (bw_write(&bw, (uint32_t)ac_extra_val[i], cat) < 0) goto fail;
                            }
                        }
                    }
                }
            }
        }
    }

    /* Flush remaining bits */
    if (bw_flush(&bw) < 0) goto fail;

    outPos += bw.pos;

    /* Write EOI */
    if (outPos + 2 > outCap) goto fail;
    out[outPos++] = 0xFF;
    out[outPos++] = M_EOI;

    *outLen = outPos;
    free(ctx);
    return out;

fail:
    free(out);
    free(ctx);
    return NULL;
}
