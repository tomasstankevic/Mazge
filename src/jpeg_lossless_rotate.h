/*
 * jpeg_lossless_rotate.h — Lossless DCT-domain crop + 90 CCW rotate +
 * drop-chroma for OV2640 4:2:2 baseline JPEGs.
 *
 * Performs:
 *   1. Huffman-decode source entropy stream
 *   2. Crop region of Y blocks (chroma blocks discarded)
 *   3. Per-block 90 CCW DCT transpose: out[u][v] = (-1)^u * src[v][u]
 *   4. Re-emit as grayscale-only baseline JPEG (1 component)
 *
 * NO IDCT/DCT. Only Huffman codec + coefficient transpose.
 *
 * Constraints:
 *   - Source must be baseline JPEG, 4:2:2 (Y h=2 v=1, Cb/Cr h=1 v=1)
 *   - Crop must be MCU-aligned: x and w multiple of 16, y and h multiple of 8
 *   - Crop must be square (width == height) for 90 deg rotation
 *
 * Returns: PSRAM buffer with output JPEG (caller frees), or NULL on failure.
 */
#ifndef JPEG_LOSSLESS_ROTATE_H
#define JPEG_LOSSLESS_ROTATE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t *jpeg_lossless_crop_rotate_gray(
    const uint8_t *src, size_t srcLen,
    int cropX, int cropY,
    int cropW, int cropH,
    size_t *outLen);

#ifdef __cplusplus
}
#endif

#endif
