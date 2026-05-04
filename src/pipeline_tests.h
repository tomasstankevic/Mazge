/*
 * pipeline_tests.h — JPEG pipeline variants for live ESP32 timing tests.
 *
 * Each variant takes an input JPEG and produces a processed JPEG.
 * Returned buffer is in PSRAM and must be freed by caller.
 */
#ifndef PIPELINE_TESTS_H
#define PIPELINE_TESTS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Run a single pipeline variant on the given JPEG.
 *
 * variant:  'A' lossless crop 384x384 only (no rotate)
 *           'B' JPG_SCALE_2X decode + rotate 90 CCW + encode (240x320)
 *           'C' full decode + rotate 90 CCW + encode (480x640)
 *           'D' JPG_SCALE_2X decode + crop right 64px + rotate + encode (240x256)
 *           'E' full decode + crop right 128px + rotate + encode (480x512)
 *           'F' same as B but quality=60
 *           'H' JPG_SCALE_2X decode + encode (320x240, no rotate) — isolate rotate cost
 *           'P' existing jpeg_api_preprocess (decode + rotate + encode)
 *           'R' lossless DCT crop+rotate+drop-chroma (no IDCT/DCT)
 *
 * jpg/jpgLen:  Input JPEG (assumed grayscale, 640x480)
 * outLen:      [out] Output JPEG length
 *
 * Returns: PSRAM buffer with processed JPEG (caller frees), or NULL on failure.
 */
uint8_t *pipeline_run(int variant, const uint8_t *jpg, size_t jpgLen, size_t *outLen);

#ifdef __cplusplus
}
#endif

#endif /* PIPELINE_TESTS_H */
