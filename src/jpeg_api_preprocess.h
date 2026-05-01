/*
 * jpeg_api_preprocess.h — Prepare JPEG for prey detection API.
 *
 * Pipeline (fast, ~20ms on ESP32-S3):
 *   1. JPEG decode at half-scale (JPG_SCALE_2X) → 320×240 grayscale (77KB)
 *   2. Rotate 90° CCW → 240×320
 *   3. JPEG encode → output buffer (~8KB)
 *
 * Output: 240×320 grayscale JPEG in PSRAM. Caller must free().
 */

#ifndef JPEG_API_PREPROCESS_H
#define JPEG_API_PREPROCESS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Output dimensions after half-scale decode + 90° CCW rotation */
#define API_OUT_W 240
#define API_OUT_H 320

/*
 * Preprocess a 640×480 grayscale JPEG for the prey detection API.
 *
 * Steps: decode at half-scale (320×240), rotate 90° CCW (→240×320), JPEG encode.
 *
 * @param jpgBuf   Input JPEG data
 * @param jpgLen   Input JPEG length
 * @param outLen   [out] Output JPEG length
 * @param quality  JPEG encode quality (1-100, recommend 80)
 * @return         Pointer to output JPEG in PSRAM (caller frees), or NULL on failure
 */
uint8_t *jpeg_api_preprocess(const uint8_t *jpgBuf, size_t jpgLen, size_t *outLen, int quality);

#ifdef __cplusplus
}
#endif

#endif /* JPEG_API_PREPROCESS_H */
