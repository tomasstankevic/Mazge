/*
 * pipeline_tests.c — JPEG pipeline variants for live ESP32 timing tests.
 *
 * Each variant decodes (optionally), transforms, and encodes a JPEG.
 * All buffers in PSRAM. Caller must free returned buffer.
 */

#include "pipeline_tests.h"
#include "jpeg_lossless_crop.h"
#include "jpeg_lossless_rotate.h"
#include "jpeg_api_preprocess.h"
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_jpg_decode.h"
#include "img_converters.h"

/* Input JPEG is assumed 640x480 grayscale */
#define INPUT_W 640
#define INPUT_H 480

/* Lossless crop variant (A) settings */
#define CROP_X  64
#define CROP_Y  48
#define CROP_SZ 384

/* ---- Decode context ---- */
typedef struct {
    const uint8_t *input;
    size_t input_len;
    uint8_t *pixels;
    int dst_w;
    int dst_h;
} DecodeCtx;

static size_t reader_cb(void *arg, size_t index, uint8_t *buf, size_t len) {
    DecodeCtx *ctx = (DecodeCtx *)arg;
    if (index >= ctx->input_len) return 0;
    if (index + len > ctx->input_len) {
        len = ctx->input_len - index;
    }
    if (buf) memcpy(buf, ctx->input + index, len);
    return len;
}

/* Writer for grayscale: decoder outputs RGB888, take R channel.
 * data == NULL during init phase (after header parse) — return true. */
static bool writer_cb(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
    if (!data) return true;
    DecodeCtx *ctx = (DecodeCtx *)arg;
    for (int row = 0; row < h; row++) {
        int dst_y = y + row;
        if (dst_y >= ctx->dst_h) break;
        for (int col = 0; col < w; col++) {
            int dst_x = x + col;
            if (dst_x >= ctx->dst_w) break;
            ctx->pixels[dst_y * ctx->dst_w + dst_x] = data[(row * w + col) * 3];
        }
    }
    return true;
}

/*
 * Rotate 90° CCW: input WxH → output HxW.
 * out[oy][ox] = in[ox][in_w - 1 - oy]   for oy in [0, in_w), ox in [0, in_h)
 */
static void rotate_90ccw(const uint8_t *in, int in_w, int in_h, uint8_t *out) {
    int out_w = in_h;
    int out_h = in_w;
    for (int oy = 0; oy < out_h; oy++) {
        int src_x = in_w - 1 - oy;
        const uint8_t *col = in + src_x;
        for (int ox = 0; ox < out_w; ox++) {
            int src_y = ox;
            out[oy * out_w + ox] = col[src_y * in_w];
        }
    }
}

/*
 * Crop right: copy first crop_w columns of WxH into new buffer (crop_w x H).
 */
static void crop_right(const uint8_t *in, int in_w, int in_h, int crop_w, uint8_t *out) {
    for (int y = 0; y < in_h; y++) {
        memcpy(out + y * crop_w, in + y * in_w, crop_w);
    }
}

/*
 * Generic decode+transform+encode pipeline.
 * half_scale=1 → JPG_SCALE_2X (output 320x240); else full 640x480.
 * crop_right_px (in decoded coords) trimmed before optional rotation.
 * rotate_ccw=1 → 90° CCW.
 * quality: 1-100.
 */
static uint8_t *decode_transform_encode(const uint8_t *jpg, size_t jpgLen,
                                        int half_scale, int crop_right_px,
                                        int rotate_ccw, int quality,
                                        size_t *outLen) {
    *outLen = 0;
    int dec_w = half_scale ? (INPUT_W / 2) : INPUT_W;
    int dec_h = half_scale ? (INPUT_H / 2) : INPUT_H;

    /* Decode */
    uint8_t *pixels = (uint8_t *)heap_caps_malloc(dec_w * dec_h,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pixels) return NULL;

    DecodeCtx ctx = {jpg, jpgLen, pixels, dec_w, dec_h};
    esp_err_t err = esp_jpg_decode(jpgLen,
                                   half_scale ? JPG_SCALE_2X : JPG_SCALE_NONE,
                                   reader_cb, writer_cb, &ctx);
    if (err != ESP_OK) {
        free(pixels);
        return NULL;
    }

    int cur_w = dec_w;
    int cur_h = dec_h;
    uint8_t *cur = pixels;

    /* Crop right */
    if (crop_right_px > 0) {
        int new_w = cur_w - crop_right_px;
        if (new_w <= 0) { free(pixels); return NULL; }
        uint8_t *cropped = (uint8_t *)heap_caps_malloc(new_w * cur_h,
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!cropped) { free(pixels); return NULL; }
        crop_right(cur, cur_w, cur_h, new_w, cropped);
        free(cur);
        cur = cropped;
        cur_w = new_w;
    }

    /* Rotate */
    if (rotate_ccw) {
        uint8_t *rotated = (uint8_t *)heap_caps_malloc(cur_w * cur_h,
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!rotated) { free(cur); return NULL; }
        rotate_90ccw(cur, cur_w, cur_h, rotated);
        free(cur);
        cur = rotated;
        int t = cur_w; cur_w = cur_h; cur_h = t;
    }

    /* Encode */
    uint8_t *jpgOut = NULL;
    size_t jpgOutLen = 0;
    bool ok = fmt2jpg(cur, cur_w * cur_h, cur_w, cur_h,
                      PIXFORMAT_GRAYSCALE, (uint8_t)quality, &jpgOut, &jpgOutLen);
    free(cur);

    if (!ok || !jpgOut) return NULL;
    *outLen = jpgOutLen;
    return jpgOut;
}

uint8_t *pipeline_run(int variant, const uint8_t *jpg, size_t jpgLen, size_t *outLen) {
    *outLen = 0;
    switch (variant) {
        case 'A': {
            /* Lossless crop only — no decode/rotate/encode */
            return jpeg_lossless_crop(jpg, jpgLen, CROP_X, CROP_Y, CROP_SZ, CROP_SZ, outLen);
        }
        case 'B':
            return decode_transform_encode(jpg, jpgLen, 1, 0,  1, 80, outLen);
        case 'C':
            return decode_transform_encode(jpg, jpgLen, 0, 0,  1, 80, outLen);
        case 'D':
            return decode_transform_encode(jpg, jpgLen, 1, 64, 1, 80, outLen);
        case 'E':
            return decode_transform_encode(jpg, jpgLen, 0, 128, 1, 80, outLen);
        case 'F':
            return decode_transform_encode(jpg, jpgLen, 1, 0,  1, 60, outLen);
        case 'H':
            return decode_transform_encode(jpg, jpgLen, 1, 0,  0, 80, outLen);
        case 'Z': {
            /* No-op: just copy input — validates task/POST plumbing */
            uint8_t *out = (uint8_t *)heap_caps_malloc(jpgLen,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!out) return NULL;
            memcpy(out, jpg, jpgLen);
            *outLen = jpgLen;
            return out;
        }
        case 'Y': {
            /* Decode-only: return decoded grayscale pixels (not a JPEG) */
            int dec_w = INPUT_W / 2;
            int dec_h = INPUT_H / 2;
            uint8_t *pixels = (uint8_t *)heap_caps_malloc(dec_w * dec_h,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!pixels) return NULL;
            DecodeCtx ctx = {jpg, jpgLen, pixels, dec_w, dec_h};
            esp_err_t err = esp_jpg_decode(jpgLen, JPG_SCALE_2X,
                                           reader_cb, writer_cb, &ctx);
            if (err != ESP_OK) { free(pixels); return NULL; }
            *outLen = dec_w * dec_h;
            return pixels;
        }
        case 'W': {
            /* Encode-only: feed a flat gray buffer to fmt2jpg (no decode) */
            int w = 320, h = 240;
            uint8_t *pixels = (uint8_t *)heap_caps_malloc(w * h,
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!pixels) return NULL;
            memset(pixels, 128, w * h);
            uint8_t *jpgOut = NULL;
            size_t jpgOutLen = 0;
            bool ok = fmt2jpg(pixels, w * h, w, h,
                              PIXFORMAT_GRAYSCALE, 80, &jpgOut, &jpgOutLen);
            free(pixels);
            if (!ok || !jpgOut) return NULL;
            *outLen = jpgOutLen;
            return jpgOut;
        }
        case 'P': {
            /* Original jpeg_api_preprocess: half-scale decode + rotate + encode */
            return jpeg_api_preprocess(jpg, jpgLen, outLen, 80);
        }
        case 'R': {
            /* Lossless DCT crop + rotate + drop chroma — no IDCT/DCT */
            return jpeg_lossless_crop_rotate_gray(jpg, jpgLen,
                                                  CROP_X, CROP_Y, CROP_SZ, CROP_SZ,
                                                  outLen);
        }
        case 'S': {
            /* Two-stage: lossless crop FIRST (drops out-of-region MCUs from
             * bitstream), THEN lossless rotate (only decodes the 384x384 region).
             * Should be faster than R since rotate skips ~75% wasted decode work.
             */
            size_t cropped_len = 0;
            uint8_t *cropped = jpeg_lossless_crop(jpg, jpgLen,
                                                  CROP_X, CROP_Y, CROP_SZ, CROP_SZ,
                                                  &cropped_len);
            if (!cropped) return NULL;
            /* Rotate input is now 384x384 starting at (0,0) */
            uint8_t *out = jpeg_lossless_crop_rotate_gray(cropped, cropped_len,
                                                          0, 0, CROP_SZ, CROP_SZ,
                                                          outLen);
            free(cropped);
            return out;
        }
        default:
            return NULL;
    }
}
