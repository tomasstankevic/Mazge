/*
 * test_jpeg_crop.c — Laptop test for lossless JPEG crop.
 *
 * Build: cc -O2 -o test_jpeg_crop tools/test_jpeg_crop.c src/jpeg_lossless_crop.c -I src
 * Usage: ./test_jpeg_crop input.jpg output.jpg [cropX cropY cropW cropH]
 *
 * Default crop: 64,48 → 384x384 (matching ESP32 CROP_X/Y/SZ)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "jpeg_lossless_crop.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s input.jpg output.jpg [cropX cropY cropW cropH]\n", argv[0]);
        return 1;
    }

    const char *inpath = argv[1];
    const char *outpath = argv[2];
    int cropX = argc > 3 ? atoi(argv[3]) : 64;
    int cropY = argc > 4 ? atoi(argv[4]) : 48;
    int cropW = argc > 5 ? atoi(argv[5]) : 384;
    int cropH = argc > 6 ? atoi(argv[6]) : 384;

    /* Read input file */
    FILE *f = fopen(inpath, "rb");
    if (!f) { perror("fopen input"); return 1; }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *buf = (uint8_t *)malloc(fsize);
    if (!buf) { perror("malloc"); fclose(f); return 1; }
    fread(buf, 1, fsize, f);
    fclose(f);

    printf("Input: %s (%ld bytes)\n", inpath, fsize);
    printf("Crop: (%d,%d) %dx%d\n", cropX, cropY, cropW, cropH);

    /* Time the crop */
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    size_t outLen = 0;
    uint8_t *out = jpeg_lossless_crop(buf, (size_t)fsize,
                                       cropX, cropY, cropW, cropH,
                                       &outLen);

    clock_gettime(CLOCK_MONOTONIC, &t1);
    double ms = (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6;

    if (!out) {
        fprintf(stderr, "ERROR: crop failed\n");
        free(buf);
        return 1;
    }

    printf("Output: %zu bytes (%.1f%% of input)\n", outLen, 100.0 * outLen / fsize);
    printf("Time: %.2f ms\n", ms);

    /* Verify it starts with SOI and ends with EOI */
    if (outLen >= 4 && out[0] == 0xFF && out[1] == 0xD8 &&
        out[outLen-2] == 0xFF && out[outLen-1] == 0xD9) {
        printf("Structure: OK (SOI...EOI)\n");
    } else {
        printf("Structure: INVALID\n");
    }

    /* Write output */
    f = fopen(outpath, "wb");
    if (!f) { perror("fopen output"); free(out); free(buf); return 1; }
    fwrite(out, 1, outLen, f);
    fclose(f);

    printf("Written: %s\n", outpath);

    /* Run 100 iterations for benchmark */
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int i = 0; i < 100; i++) {
        size_t tmpLen = 0;
        uint8_t *tmp = jpeg_lossless_crop(buf, (size_t)fsize,
                                           cropX, cropY, cropW, cropH,
                                           &tmpLen);
        if (tmp) free(tmp);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    ms = (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6;
    printf("Benchmark: 100 iterations in %.1f ms (%.2f ms/iter)\n", ms, ms / 100.0);

    free(out);
    free(buf);
    return 0;
}
