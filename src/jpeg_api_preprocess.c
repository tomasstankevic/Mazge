/*
 * jpeg_api_preprocess.c — Fast JPEG preprocessing for prey detection API.
 *
 * Pipeline: JPG_SCALE_2X decode (320×240) → rotate 90° CCW (240×320) → JPEG encode.
 * Uses esp_jpg_decode (ROM decoder, DCT half-scale = free 2×2 averaging) and fmt2jpg.
 * All buffers in PSRAM. Expected timing: ~20ms on ESP32-S3.
 */

#include "jpeg_api_preprocess.h"
#include <string.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_jpg_decode.h"
#include "img_converters.h"

/* Half-scale decoded dimensions */
#define HALF_W 320
#define HALF_H 240

/* ---- JPEG decode context ---- */
typedef struct {
    const uint8_t *input;
    size_t input_len;
    uint8_t *pixels;      /* decoded grayscale buffer (HALF_W * HALF_H) */
} DecodeCtx;

/* Reader callback for esp_jpg_decode */
static size_t jpg_reader(void *arg, size_t index, uint8_t *buf, size_t len) {
    DecodeCtx *ctx = (DecodeCtx *)arg;
    if (index >= ctx->input_len) return 0;
    if (index + len > ctx->input_len) {
        len = ctx->input_len - index;
    }
    if (buf) memcpy(buf, ctx->input + index, len);
    return len;
}

/* Writer callback — receives decoded pixels in RGB888 blocks.
 * For grayscale JPEG, the decoder outputs RGB888 (R=G=B=Y).
 * We take just the first byte (R channel = luminance).
 * data == NULL during init phase (after header parse) — return true. */
static bool jpg_writer(void *arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
    if (!data) return true;
    DecodeCtx *ctx = (DecodeCtx *)arg;
    for (int row = 0; row < h; row++) {
        int dst_y = y + row;
        if (dst_y >= HALF_H) break;
        for (int col = 0; col < w; col++) {
            int dst_x = x + col;
            if (dst_x >= HALF_W) break;
            ctx->pixels[dst_y * HALF_W + dst_x] = data[(row * w + col) * 3];
        }
    }
    return true;
}

/*
 * Rotate 90° CCW: input 320×240 → output 240×320.
 * Mapping: output[oy][ox] = input[ox][HALF_W - 1 - oy]
 */
static void rotate_90ccw(const uint8_t *in, uint8_t *out) {
    for (int oy = 0; oy < API_OUT_H; oy++) {       /* 0..319 */
        int src_x = HALF_W - 1 - oy;               /* 319..0 */
        for (int ox = 0; ox < API_OUT_W; ox++) {    /* 0..239 */
            int src_y = ox;                          /* 0..239 */
            out[oy * API_OUT_W + ox] = in[src_y * HALF_W + src_x];
        }
    }
}

uint8_t *jpeg_api_preprocess(const uint8_t *jpgBuf, size_t jpgLen, size_t *outLen, int quality) {
    *outLen = 0;

    /* Allocate decode buffer in PSRAM (320*240 = 77KB) */
    uint8_t *pixels = (uint8_t *)heap_caps_malloc(HALF_W * HALF_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pixels) return NULL;

    /* Decode JPEG at half-scale (DCT shortcut: 320×240 from 640×480) */
    DecodeCtx ctx;
    ctx.input = jpgBuf;
    ctx.input_len = jpgLen;
    ctx.pixels = pixels;

    esp_err_t err = esp_jpg_decode(jpgLen, JPG_SCALE_2X, jpg_reader, jpg_writer, &ctx);
    if (err != ESP_OK) {
        free(pixels);
        return NULL;
    }

    /* Allocate rotated pixel buffer (240*320 = 77KB) */
    uint8_t *rotated = (uint8_t *)heap_caps_malloc(API_OUT_W * API_OUT_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rotated) {
        free(pixels);
        return NULL;
    }

    /* Rotate 90° CCW */
    rotate_90ccw(pixels, rotated);
    free(pixels);

    /* Encode to JPEG */
    uint8_t *jpgOut = NULL;
    size_t jpgOutLen = 0;
    bool ok = fmt2jpg(rotated, API_OUT_W * API_OUT_H, API_OUT_W, API_OUT_H,
                      PIXFORMAT_GRAYSCALE, (uint8_t)quality, &jpgOut, &jpgOutLen);
    free(rotated);

    if (!ok || !jpgOut) return NULL;

    *outLen = jpgOutLen;
    return jpgOut;
}
