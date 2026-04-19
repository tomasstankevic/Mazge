/*
 * jpeg_lossless_crop.h — Lossless JPEG crop for 8-aligned boundaries.
 *
 * Operates directly on Huffman-coded MCU blocks without full decode/re-encode.
 * Supports baseline JPEG (SOF0) with any number of components and subsampling.
 *
 * Constraints:
 *   - Crop coordinates must be aligned to MCU boundaries
 *   - Input must be baseline sequential JPEG (no progressive, no arithmetic)
 *   - No restart markers expected (OV2640 doesn't emit them)
 */
#ifndef JPEG_LOSSLESS_CROP_H
#define JPEG_LOSSLESS_CROP_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Losslessly crop a JPEG image by extracting MCU blocks.
 *
 * src/srcLen:    Input JPEG buffer
 * cropX, cropY:  Top-left pixel offset (must be MCU-aligned: multiple of 8*maxHSamp x 8*maxVSamp)
 * cropW, cropH:  Crop dimensions in pixels (must be MCU-aligned)
 * outLen:        Output: length of cropped JPEG
 *
 * Returns: Newly allocated buffer with cropped JPEG (caller must free).
 *          NULL on error.
 *
 * If JPEG_CROP_USE_PSRAM is defined, uses ps_malloc(); otherwise uses malloc().
 */
uint8_t *jpeg_lossless_crop(
    const uint8_t *src, size_t srcLen,
    int cropX, int cropY,
    int cropW, int cropH,
    size_t *outLen
);

#ifdef __cplusplus
}
#endif

#endif /* JPEG_LOSSLESS_CROP_H */
