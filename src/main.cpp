#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "img_converters.h"
#include <lwip/sockets.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "mbedtls/base64.h"
#include "secrets.h"

// ===== Freenove ESP32-S3 WROOM CAM pin map =====
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM  13

// ===== PIR motion sensor =====
#define PIR_PIN 14
volatile bool motionDetected = false;

// ===== TOF050C / VL6180X ToF sensor (raw I2C, 16-bit registers) =====
#define TOF_SDA 47
#define TOF_SCL 21
#define TOF_ADDR 0x29
volatile int tofDistance = -2; // mm, -1 = no object (>range), -2 = sensor error
volatile uint16_t alsLux = 0;  // ambient light from VL6180X ALS
bool tofReady = false;

void tofWriteReg(uint16_t reg, uint8_t val) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write((reg >> 8) & 0xFF);
  Wire.write(reg & 0xFF);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t tofReadReg(uint16_t reg) {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write((reg >> 8) & 0xFF);
  Wire.write(reg & 0xFF);
  Wire.endTransmission();
  Wire.requestFrom(TOF_ADDR, 1);
  return Wire.available() ? Wire.read() : 0;
}

void tofInit() {
  // Mandatory init sequence from VL6180X datasheet (AN4545)
  // Only write private regs if fresh out of reset
  if (tofReadReg(0x016) == 1) {
    tofWriteReg(0x0207, 0x01);
    tofWriteReg(0x0208, 0x01);
    tofWriteReg(0x0096, 0x00);
    tofWriteReg(0x0097, 0xFD);
    tofWriteReg(0x00E3, 0x00);
    tofWriteReg(0x00E4, 0x04);
    tofWriteReg(0x00E5, 0x02);
    tofWriteReg(0x00E6, 0x01);
    tofWriteReg(0x00E7, 0x03);
    tofWriteReg(0x00F5, 0x02);
    tofWriteReg(0x00D9, 0x05);
    tofWriteReg(0x00DB, 0xCE);
    tofWriteReg(0x00DC, 0x03);
    tofWriteReg(0x00DD, 0xF8);
    tofWriteReg(0x009F, 0x00);
    tofWriteReg(0x00A3, 0x3C);
    tofWriteReg(0x00B7, 0x00);
    tofWriteReg(0x00BB, 0x3C);
    tofWriteReg(0x00B2, 0x09);
    tofWriteReg(0x00CA, 0x09);
    tofWriteReg(0x0198, 0x01);
    tofWriteReg(0x01B0, 0x17);
    tofWriteReg(0x01AD, 0x00);
    tofWriteReg(0x00FF, 0x05);
    tofWriteReg(0x0100, 0x05);
    tofWriteReg(0x0199, 0x05);
    tofWriteReg(0x01A6, 0x1B);
    tofWriteReg(0x01AC, 0x3E);
    tofWriteReg(0x01A7, 0x1F);
    tofWriteReg(0x0030, 0x00);
    tofWriteReg(0x0016, 0x00); // clear fresh out of reset
    Serial.println("TOF050C: wrote private init regs");
  } else {
    Serial.println("TOF050C: already initialized, skipping private regs");
  }
  // Always configure for ranging (safe to re-apply)
  tofWriteReg(0x0011, 0x10); // GPIO1 = new sample ready
  tofWriteReg(0x010A, 0x30); // averaging period = 48
  tofWriteReg(0x003F, 0x46); // ALS analogue gain
  tofWriteReg(0x0031, 0xFF); // cal every 255 measurements
  tofWriteReg(0x0041, 0x63); // ALS integration time 100ms
  tofWriteReg(0x002E, 0x00); // ranging inter-measurement period = minimum
  tofWriteReg(0x001B, 0x05); // max convergence time 5.0ms (faster, less accurate at edge)
  tofWriteReg(0x003E, 0x31); // range check enables
  tofWriteReg(0x0014, 0x24); // range/ALS interrupt config
  tofWriteReg(0x0015, 0x07); // clear any pending interrupts
}

// ===== MJPEG stream constants =====
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ===== Globals =====
httpd_handle_t stream_httpd = NULL;
httpd_handle_t ui_httpd = NULL;

// Telemetry (updated by stream handler)
volatile float streamFps = 0;
volatile uint32_t lastFrameBytes = 0;
volatile uint32_t lastFrameMs = 0;
volatile uint32_t frameCount = 0;
volatile int targetFps = 15;  // adjustable from web UI
volatile int jpegQuality = 95;

// ===== Frame dimensions =====
#define CAM_W 640
#define CAM_H 480

// ===== Burst capture (pre-trigger ring buffer) =====
#define RING_SIZE 10         // 8 pre-trigger + 2 post-trigger
#define BURST_TRIGGER_MM 500
#define BURST_ARCHIVES 40

// ===== Prey Detection API (autonomous mode) =====
#define API_FRAMES_PER_BURST 3   // send up to 3 best frames per burst
#define LAPTOP_TIMEOUT_MS 30000  // consider laptop absent after 30s no contact
volatile unsigned long lastLaptopContactMs = 0;  // last /burst_wait or /burststream request

// ===== HDR gain + exposure bracketing =====
// Manual exposure: capped at 1/40s to avoid motion blur on moving cats.
// OV2640 AEC value ≈ line count. At 20MHz XCLK, VGA: 1 line ≈ 80µs.
// 1/40s = 25ms → ~312 lines. We bracket exposure in 2 steps within that cap.
#define AEC_MAX 300       // ~24ms ≈ 1/40s
#define AEC_LOW 100       // ~8ms — short exposure for IR night
// Gain brackets: 10 frames cycling through (gain, exposure) pairs.
// Low gain+short exposure first (best for IR night), then ramp up for day.
struct HdrStep { int gain; int aec; };
static const HdrStep HDR_STEPS[] = {
  {0,  AEC_LOW},  {0,  AEC_MAX},   // very dark, short+long
  {2,  AEC_LOW},  {2,  AEC_MAX},   // low gain
  {6,  AEC_LOW},  {6,  AEC_MAX},   // medium gain
  {12, AEC_LOW},  {12, AEC_MAX},   // higher gain
  {20, AEC_LOW},  {30, AEC_LOW},   // high gain, short exposure only (night)
};
#define HDR_STEP_COUNT 10
struct BurstImage {
  uint8_t *buf;
  size_t len;
  unsigned long captureMs;
};
struct BurstArchive {
  BurstImage images[RING_SIZE];
  int count;
  unsigned long timestamp;
  unsigned long triggerMs;
  unsigned long firstFrameMs;
  unsigned long lastFrameMs;
  int generation;
  // API results (autonomous mode)
  int8_t apiResults[RING_SIZE]; // -1=not checked, 0=no prey, 1=prey detected
  int apiFramesSent;            // how many frames sent to API
  unsigned long apiCallMs;      // when API was called
  int8_t apiPreyDetected;       // -1=not checked, 0=no prey, 1=prey found
};
BurstArchive burstArchives[BURST_ARCHIVES];
volatile int burstArchiveCount = 0;
volatile int burstGen = 0;  // increments on each new burst
volatile bool burstCapturing = false;
volatile bool otaInProgress = false;
unsigned long burstCooldown = 0;
unsigned long pendingBurstTriggerMs = 0;

// Post-trigger: capture N more frames after trigger before freezing
#define POST_TRIGGER_FRAMES 2
int postTriggerRemaining = 0;  // >0 means we're in post-trigger phase

// ===== Check if a JPEG frame is blown out =====
// Sample brightness from raw JPEG data by checking luminance in a grid pattern
// We decode a small sample of the JPEG's pixel data to estimate average brightness
bool isFrameBlownOut(const uint8_t *jpgBuf, size_t jpgLen) {
  // Quick heuristic: sample bytes in the compressed JPEG stream.
  // High byte values in JPEG data correlate with bright images.
  // More reliable: check file size. Blown-out frames compress very small.
  // For a 640x480 JPEG, a very bright/blown frame compresses to <10KB at q=95
  // while a normal frame is 30-80KB.
  if (jpgLen < 8000) return true;  // suspiciously small = likely blown out
  // Also sample the JPEG payload for high-value byte runs
  int highCount = 0;
  int sampleCount = 0;
  // Skip JPEG header (first ~600 bytes), sample every 100th byte
  for (size_t i = 600; i < jpgLen && i < jpgLen - 2; i += 100) {
    sampleCount++;
    if (jpgBuf[i] > 0xF0) highCount++;
  }
  if (sampleCount > 0 && (highCount * 100 / sampleCount) > 60) return true;
  return false;
}

// Ring buffer: continuously captures frames so we have the PAST frames on trigger
BurstImage ringBuf[RING_SIZE];
int ringHead = 0;
int ringCount = 0;

void freezeRingToArchive() {
  burstCapturing = true;
  Serial.println("Burst: freezing ring buffer...");

  // Shift archives if full
  if (burstArchiveCount >= BURST_ARCHIVES) {
    for (int i = 0; i < RING_SIZE; i++) {
      if (burstArchives[0].images[i].buf) { free(burstArchives[0].images[i].buf); }
    }
    for (int a = 0; a < BURST_ARCHIVES - 1; a++) {
      burstArchives[a] = burstArchives[a + 1];
    }
    burstArchiveCount = BURST_ARCHIVES - 1;
  }

  int slot = burstArchiveCount;
  memset(&burstArchives[slot], 0, sizeof(BurstArchive));
  burstArchives[slot].triggerMs = pendingBurstTriggerMs;
  burstArchives[slot].apiPreyDetected = -1;
  burstArchives[slot].apiFramesSent = 0;
  burstArchives[slot].apiCallMs = 0;
  for (int i = 0; i < RING_SIZE; i++) burstArchives[slot].apiResults[i] = -1;

  // Save ring frames (oldest first), skip blown-out frames
  int available = ringCount;
  int start = (ringCount < RING_SIZE) ? 0 : ringHead;
  unsigned long firstCaptureMs = 0;
  unsigned long lastCaptureMs = 0;
  int saved = 0;
  for (int i = 0; i < available; i++) {
    int idx = (start + i) % RING_SIZE;
    if (!ringBuf[idx].buf) continue;
    // Filter blown-out frames
    if (isFrameBlownOut(ringBuf[idx].buf, ringBuf[idx].len)) {
      Serial.printf("Burst: skipping blown-out frame %d (%u bytes)\n", i, ringBuf[idx].len);
      free(ringBuf[idx].buf);
      ringBuf[idx].buf = NULL;
      ringBuf[idx].len = 0;
      ringBuf[idx].captureMs = 0;
      continue;
    }
    burstArchives[slot].images[saved].buf = ringBuf[idx].buf;
    burstArchives[slot].images[saved].len = ringBuf[idx].len;
    burstArchives[slot].images[saved].captureMs = ringBuf[idx].captureMs;
    if (saved == 0 || ringBuf[idx].captureMs < firstCaptureMs) firstCaptureMs = ringBuf[idx].captureMs;
    if (ringBuf[idx].captureMs > lastCaptureMs) lastCaptureMs = ringBuf[idx].captureMs;
    ringBuf[idx].buf = NULL;
    ringBuf[idx].len = 0;
    ringBuf[idx].captureMs = 0;
    saved++;
  }
  burstArchives[slot].count = saved;
  burstArchives[slot].timestamp = millis();
  burstArchives[slot].firstFrameMs = firstCaptureMs;
  burstArchives[slot].lastFrameMs = lastCaptureMs;
  burstArchives[slot].generation = burstGen + 1;
  burstArchiveCount = slot + 1;
  burstGen++;
  pendingBurstTriggerMs = 0;

  // Free remaining ring frames and reset
  for (int i = 0; i < RING_SIZE; i++) {
    if (ringBuf[i].buf) { free(ringBuf[i].buf); ringBuf[i].buf = NULL; }
    ringBuf[i].len = 0;
    ringBuf[i].captureMs = 0;
  }
  ringCount = 0;
  ringHead = 0;

  burstCapturing = false;
  burstCooldown = millis();
  Serial.printf("Burst archived: %d frames (archive %d/%d)\n",
    burstArchives[slot].count, burstArchiveCount, BURST_ARCHIVES);
}

// ===== Autonomous prey API call (when laptop absent) =====
// HTTP event handler for collecting response body
static char apiResponseBuf[256];
static int apiResponseLen = 0;
static int lastApiEspErr = 0;
static int lastApiHttpStatus = 0;

// ===== Crop JPEG for API =====
// From 640x480: remove right 128px (occluded), then center-crop to 384x384.
// All offsets 8-aligned (JPEG MCU boundary for greyscale).
// Crop region in original image: x=64..448, y=48..432.
// Returns new JPEG in PSRAM (caller must free). Sets outLen. NULL on failure.
#define CROP_X  64    // left margin (64px, 8-aligned)
#define CROP_Y  48    // top margin  (48px, 8-aligned)
#define CROP_SZ 384   // output 384x384

static uint8_t *cropJpegForApi(const uint8_t *jpgBuf, size_t jpgLen, size_t *outLen) {
  unsigned long t0 = millis();
  *outLen = 0;
  Serial.printf("Crop: freeHeap=%u freePSRAM=%u jpgLen=%u\n",
    ESP.getFreeHeap(), ESP.getFreePsram(), jpgLen);

  // Decode JPEG to RGB888 (greyscale gets expanded to 3 channels)
  size_t rgbLen = CAM_W * CAM_H * 3;
  uint8_t *rgb = (uint8_t *)ps_malloc(rgbLen);
  if (!rgb) {
    Serial.println("Crop: rgb malloc failed");
    return NULL;
  }
  bool ok = fmt2rgb888(jpgBuf, jpgLen, PIXFORMAT_JPEG, rgb);
  if (!ok) {
    free(rgb);
    Serial.println("Crop: JPEG decode failed");
    return NULL;
  }
  unsigned long t1 = millis();

  // Allocate destination: 384x384x3 = 442368 bytes in PSRAM
  uint8_t *dst = (uint8_t *)ps_malloc(CROP_SZ * CROP_SZ * 3);
  if (!dst) {
    free(rgb);
    Serial.println("Crop: dst malloc failed");
    return NULL;
  }

  // Pure crop — copy rows, no scaling
  for (int dy = 0; dy < CROP_SZ; dy++) {
    int sy = CROP_Y + dy;
    const uint8_t *srcRow = rgb + (sy * CAM_W + CROP_X) * 3;
    uint8_t *dstRow = dst + dy * CROP_SZ * 3;
    memcpy(dstRow, srcRow, CROP_SZ * 3);
  }
  free(rgb);
  unsigned long t2 = millis();

  // Re-encode to JPEG
  uint8_t *outJpg = NULL;
  size_t outJpgLen = 0;
  ok = fmt2jpg(dst, CROP_SZ * CROP_SZ * 3, CROP_SZ, CROP_SZ, PIXFORMAT_RGB888, 80, &outJpg, &outJpgLen);
  free(dst);
  unsigned long t3 = millis();

  if (!ok || !outJpg) {
    Serial.println("Crop: JPEG encode failed");
    return NULL;
  }

  *outLen = outJpgLen;
  Serial.printf("Crop: 640x480→384x384 JPEG %uB→%uB (decode=%lums crop=%lums encode=%lums total=%lums)\n",
    jpgLen, outJpgLen, t1 - t0, t2 - t1, t3 - t2, t3 - t0);
  return outJpg;
}

// Send a single JPEG frame to the prey API. Returns: -1=error, 0=no prey, 1=prey
static int callPreyApi(const uint8_t *jpgBuf, size_t jpgLen) {
  // Crop to 384x384 before sending
  size_t croppedLen = 0;
  uint8_t *croppedJpg = cropJpegForApi(jpgBuf, jpgLen, &croppedLen);
  const uint8_t *sendBuf = croppedJpg ? croppedJpg : jpgBuf;
  size_t sendLen = croppedJpg ? croppedLen : jpgLen;
  Serial.printf("API: sending %uB %s\n", sendLen, croppedJpg ? "(cropped)" : "(original, crop failed)");

  // Base64 encode the JPEG
  size_t b64Len = 0;
  mbedtls_base64_encode(NULL, 0, &b64Len, sendBuf, sendLen);
  char *b64Buf = (char *)ps_malloc(b64Len + 1);
  if (!b64Buf) { if (croppedJpg) free(croppedJpg); Serial.println("API: base64 malloc failed"); return -1; }
  mbedtls_base64_encode((unsigned char *)b64Buf, b64Len + 1, &b64Len, sendBuf, sendLen);
  b64Buf[b64Len] = 0;
  if (croppedJpg) free(croppedJpg);

  // Build JSON body: {"image_base64": "..."}
  size_t jsonLen = b64Len + 32;
  char *jsonBody = (char *)ps_malloc(jsonLen);
  if (!jsonBody) { free(b64Buf); Serial.println("API: json malloc failed"); return -1; }
  snprintf(jsonBody, jsonLen, "{\"image_base64\":\"%s\"}", b64Buf);
  free(b64Buf);

  // HTTPS POST using Arduino WiFiClientSecure
  WiFiClientSecure tlsClient;
  tlsClient.setInsecure(); // skip cert verification (Cloudflare workers)
  HTTPClient http;

  apiResponseLen = 0;
  apiResponseBuf[0] = 0;
  lastApiEspErr = 0;
  lastApiHttpStatus = 0;

  unsigned long startMs = millis();
  Serial.println("API: connecting...");

  if (!http.begin(tlsClient, PREY_API_URL)) {
    free(jsonBody);
    lastApiEspErr = -1;
    Serial.println("API: http.begin() failed");
    return -1;
  }

  char authHeader[128];
  snprintf(authHeader, sizeof(authHeader), "Bearer %s", PREY_API_KEY);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", authHeader);
  http.setTimeout(15000);

  int httpCode = http.POST((uint8_t *)jsonBody, strlen(jsonBody));
  free(jsonBody);

  unsigned long elapsed = millis() - startMs;
  lastApiHttpStatus = httpCode;

  if (httpCode <= 0) {
    Serial.printf("API: POST failed, error=%d (%lums): %s\n",
      httpCode, elapsed, http.errorToString(httpCode).c_str());
    lastApiEspErr = httpCode;
    http.end();
    return -1;
  }

  String response = http.getString();
  http.end();

  // Copy response to global buffer for diagnostics
  strncpy(apiResponseBuf, response.c_str(), sizeof(apiResponseBuf) - 1);
  apiResponseBuf[sizeof(apiResponseBuf) - 1] = 0;
  apiResponseLen = response.length();

  Serial.printf("API: HTTP %d, %dB response in %lums: %s\n",
    httpCode, apiResponseLen, elapsed, apiResponseBuf);

  if (httpCode != 200) return -1;

  // Parse "detected":true/false from response
  if (strstr(apiResponseBuf, "\"detected\":true") || strstr(apiResponseBuf, "\"detected\": true"))
    return 1;
  return 0;
}

// Pick best N frames (largest JPEG = most detail) and call API for each
void autonomousApiCheck(int archIdx) {
  if (archIdx < 0 || archIdx >= burstArchiveCount) return;
  BurstArchive &archive = burstArchives[archIdx];
  if (archive.count == 0) return;

  // Sort frame indices by JPEG size descending (pick largest = most detail)
  int indices[RING_SIZE];
  for (int i = 0; i < archive.count; i++) indices[i] = i;
  // Simple selection sort for top N
  for (int i = 0; i < min(API_FRAMES_PER_BURST, archive.count); i++) {
    for (int j = i + 1; j < archive.count; j++) {
      if (archive.images[indices[j]].len > archive.images[indices[i]].len) {
        int tmp = indices[i]; indices[i] = indices[j]; indices[j] = tmp;
      }
    }
  }

  int framesToSend = min(API_FRAMES_PER_BURST, archive.count);
  archive.apiCallMs = millis();
  archive.apiPreyDetected = 0; // default: no prey

  Serial.printf("API: autonomous check, sending %d/%d frames from archive %d (gen %d)\n",
    framesToSend, archive.count, archIdx, archive.generation);

  for (int i = 0; i < framesToSend; i++) {
    int fIdx = indices[i];
    if (!archive.images[fIdx].buf) continue;

    Serial.printf("API: frame %d (%u bytes)...\n", fIdx, archive.images[fIdx].len);
    int result = callPreyApi(archive.images[fIdx].buf, archive.images[fIdx].len);
    archive.apiResults[fIdx] = result;
    archive.apiFramesSent++;

    if (result == 1) {
      archive.apiPreyDetected = 1;
      Serial.println("API: *** PREY DETECTED ***");
    }

    // Small delay between calls to not overload
    if (i < framesToSend - 1) vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  Serial.printf("API: done. %d frames sent, prey=%d\n",
    archive.apiFramesSent, archive.apiPreyDetected);
}

// ===== HTML page =====
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32-CAM</title>
  <style>
    * { box-sizing: border-box; }
    body { background: #111; color: #eee; font-family: sans-serif;
           display: flex; flex-direction: column; align-items: center;
           margin: 0; padding: 12px; }
    h1 { margin: 0 0 4px; font-size: 1.3em; }
    #stats { color: #888; margin: 0 0 8px; font-family: monospace; font-size: 0.85em; }
    img { max-width: 100%; border: 2px solid #333; border-radius: 8px; }
    .rot90 { transform: rotate(-90deg); }
    #stream-wrap { display:inline-block; position:relative; overflow:hidden; max-width:400px; }
    #stream-wrap img { position:absolute; top:50%; left:50%; transform:rotate(-90deg) translate(-50%,-50%); transform-origin:0 0; max-width:none; }
    .thumb-wrap { display:inline-block; position:relative; width:19%; aspect-ratio:3/4; overflow:hidden; border:1px solid #555; border-radius:3px; cursor:pointer; }
    .thumb-wrap img { position:absolute; top:50%; left:50%; transform:rotate(-90deg) translate(-50%,-50%); transform-origin:0 0; width:133.33%; max-width:none; border:none; border-radius:0; }
    .controls { display: grid; grid-template-columns: 110px 1fr 50px;
                gap: 4px 8px; align-items: center; width: 100%; max-width: 500px;
                margin-top: 12px; font-size: 0.85em; }
    .controls label { text-align: right; color: #aaa; }
    .controls select, .controls input[type=range] { width: 100%; }
    .controls .val { color: #6f6; font-family: monospace; }
  </style>
</head>
<body>
  <h1>ESP32-CAM Live</h1>
  <p id="stats">Connecting...</p>
  <p id="motion" style="font-size:1.1em;font-weight:bold;margin:4px 0 8px;">Motion: <span id="motion-val" style="color:#888;">--</span> | Distance: <span id="dist-val" style="color:#888;">--</span></p>
  <button id="toggle-stream" onclick="toggleStream()" style="margin:0 0 8px;padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#a44;color:#fff;">Start Stream</button>
  <button onclick="fetch('/cmd?trigger=1')" style="margin:0 0 8px 8px;padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#c80;color:#fff;">Fake Trigger</button>
  <div id="stream-wrap"><img id="stream" src="" onload="var w=this.naturalHeight,h=this.naturalWidth;this.parentElement.style.width=w+'px';this.parentElement.style.height=h+'px';this.style.width=h+'px';" /></div>
  <div id="burst-section" style="width:100%;max-width:700px;margin-top:16px;">
    <h2 style="font-size:1.1em;margin:0 0 6px;">Burst Archives: <span id="burst-archive-count">0</span></h2>
    <div id="burst-archives"></div>
  </div>
  <div class="controls">
    <label>Quality</label>
    <input type="range" id="quality" min="10" max="100" value="95">
    <span class="val" id="quality-val">80</span>

    <label>FPS Cap</label>
    <input type="range" id="fps" min="1" max="30" value="15">
    <span class="val" id="fps-val">15</span>

    <label>Brightness</label>
    <input type="range" id="brightness" min="-2" max="2" value="0">
    <span class="val" id="brightness-val">0</span>

    <label>Contrast</label>
    <input type="range" id="contrast" min="-2" max="2" value="0">
    <span class="val" id="contrast-val">0</span>

    <label>Auto Exposure</label>
    <select id="aec">
      <option value="1" selected>On</option>
      <option value="0">Off</option>
    </select><span></span>

    <label>AE Level</label>
    <input type="range" id="ae_level" min="-2" max="2" value="0">
    <span class="val" id="ae_level-val">0</span>

    <label>Gain Ceil</label>
    <select id="gainceiling">
      <option value="0">2x</option>
      <option value="2" selected>8x</option>
      <option value="4">32x</option>
      <option value="6">128x</option>
    </select><span></span>

    <label>Night Mode</label>
    <select id="nightmode">
      <option value="0" selected>Off</option>
      <option value="1">On</option>
    </select><span></span>
  </div>
  <script>
    let streamOn = false;
    const streamImg = document.getElementById('stream');
    const toggleBtn = document.getElementById('toggle-stream');

    function toggleStream() {
      if (streamOn) {
        streamImg.src = '';
        toggleBtn.textContent = 'Start Stream';
        toggleBtn.style.background = '#a44';
        streamOn = false;
        clearInterval(statsInterval);
        statsInterval = setInterval(pollStats, 200);
      } else {
        streamImg.src = 'http://' + location.hostname + ':81/stream?t=' + Date.now();
        toggleBtn.textContent = 'Stop Stream';
        toggleBtn.style.background = '#4a4';
        streamOn = true;
        clearInterval(statsInterval);
        statsInterval = setInterval(pollStats, 1000);
      }
    }

    async function pollStats() {
      try {
        const r = await fetch('/stats');
        const s = await r.json();
        document.getElementById('stats').textContent =
          `FPS: ${s.fps.toFixed(1)} | Frame: ${(s.frameBytes/1024).toFixed(1)} KB | Send: ${s.frameMs} ms | Total: ${s.totalFrames}`;
        const mv = document.getElementById('motion-val');
        if (s.motion) { mv.textContent = 'DETECTED'; mv.style.color = '#f44'; }
        else { mv.textContent = 'None'; mv.style.color = '#4f4'; }
        const dv = document.getElementById('dist-val');
        if (s.distance >= 0) { dv.textContent = s.distance + ' mm'; dv.style.color = '#4cf'; }
        else if (s.distance === -1) { dv.textContent = '> 500 mm'; dv.style.color = '#888'; }
        else { dv.textContent = 'Sensor Error'; dv.style.color = '#f84'; }
        // Update burst gallery
        const bac = document.getElementById('burst-archive-count');
        const ba = document.getElementById('burst-archives');
        bac.textContent = s.burstArchives;
        const key = s.burstGen;
        if (s.burstArchives > 0 && key !== parseInt(ba.dataset.key || '0')) {
          ba.dataset.key = key;
          ba.innerHTML = '';
          for (let a = s.burstArchives - 1; a >= 0; a--) {
            const isLatest = (a === s.burstArchives - 1);
            const div = document.createElement('div');
            div.style.cssText = 'margin:8px 0;padding:8px;background:#1a1a2e;border-radius:6px;cursor:pointer;';
            const h = document.createElement('h3');
            h.style.cssText = 'font-size:0.95em;margin:0 0 4px;color:#aaa;';
            h.textContent = (isLatest ? '\u25BC ' : '\u25B6 ') + 'Burst #' + (a+1) + ' (' + s.burstCounts[a] + ' frames)';
            div.appendChild(h);
            const gallery = document.createElement('div');
            gallery.style.cssText = 'display:flex;flex-wrap:wrap;gap:4px;' + (isLatest ? '' : 'display:none;');
            h.onclick = function() {
              gallery.style.display = gallery.style.display === 'none' ? 'flex' : 'none';
              h.textContent = (gallery.style.display === 'none' ? '\u25B6 ' : '\u25BC ') + 'Burst #' + (a+1) + ' (' + s.burstCounts[a] + ' frames)';
              if (gallery.childElementCount === 0) {
                for (let i = 0; i < s.burstCounts[a]; i++) {
                  const wrap = document.createElement('div');
                  wrap.className = 'thumb-wrap';
                  const img = document.createElement('img');
                  img.src = '/burst?a=' + a + '&i=' + i + '&t=' + Date.now();
                  wrap.onclick = function(e) { e.stopPropagation(); window.open(img.src); };
                  wrap.appendChild(img);
                  gallery.appendChild(wrap);
                }
              }
            };
            if (isLatest) {
              for (let i = 0; i < s.burstCounts[a]; i++) {
                const wrap = document.createElement('div');
                wrap.className = 'thumb-wrap';
                const img = document.createElement('img');
                img.src = '/burst?a=' + a + '&i=' + i + '&t=' + Date.now();
                wrap.onclick = function(e) { e.stopPropagation(); window.open(img.src); };
                wrap.appendChild(img);
                gallery.appendChild(wrap);
              }
            }
            div.appendChild(gallery);
            ba.appendChild(div);
          }
        }
      } catch(e) {}
    }
    let statsInterval = setInterval(pollStats, 200);

    // Send command helper
    async function cmd(k, v) {
      try { await fetch(`/cmd?${k}=${v}`); } catch(e) {}
    }

    // Wire up controls
    for (const id of ['quality','fps','brightness','contrast','ae_level']) {
      const el = document.getElementById(id);
      const valEl = document.getElementById(id + '-val');
      el.addEventListener('input', () => { valEl.textContent = el.value; });
      el.addEventListener('change', () => { cmd(id, el.value); });
    }
    document.getElementById('aec').addEventListener('change', function() {
      cmd('aec', this.value);
    });
    document.getElementById('gainceiling').addEventListener('change', function() {
      cmd('gainceiling', this.value);
    });
    document.getElementById('nightmode').addEventListener('change', function() {
      cmd('nightmode', this.value);
    });
  </script>
</body>
</html>
)rawliteral";

// ===== Camera init =====
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  // JPEG mode: camera does RGB Bayer→debayer→JPEG internally
  // Better quality than YUV since it uses all physical pixel data
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_VGA;    // 640x480
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;  // 1-63, lower = better quality

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_exposure_ctrl(s, 0);  // DISABLE auto exposure — we bracket manually
    s->set_aec2(s, 0);           // disable DSP auto exposure
    s->set_gain_ctrl(s, 0);      // DISABLE auto gain — we bracket manually
    s->set_aec_value(s, AEC_LOW); // start with short exposure
    s->set_agc_gain(s, 0);       // start at low gain
    s->set_gainceiling(s, (gainceiling_t)6); // 128x max gain ceiling
    s->set_special_effect(s, 2); // greyscale — saves ~40% JPEG size + RAM
  }
  return true;
}

// ===== MJPEG stream handler (runs on port 81) =====
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];
  int64_t fpsTimer = esp_timer_get_time();
  int fpsCount = 0;

  // FIX 5: Set TCP_NODELAY to disable Nagle's algorithm on the stream socket.
  // Without this, small chunks (boundary strings, headers) get buffered for
  // up to 200ms before being sent, causing visible stuttering.
  int fd = httpd_req_to_sockfd(req);
  int yes = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
  // Increase TCP send buffer for burst tolerance during motion
  int sndbuf = 16384;
  setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Stream: capture failed");
      res = ESP_FAIL;
      break;
    }

    int64_t frameStart = esp_timer_get_time();

    // Camera outputs JPEG directly — no conversion needed
    uint8_t *jpg_buf = fb->buf;
    size_t jpg_len = fb->len;

    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, jpg_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_len);

    lastFrameBytes = jpg_len;
    lastFrameMs = (esp_timer_get_time() - frameStart) / 1000;

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) {
      Serial.println("Stream client disconnected");
      break;
    }

    // Cap fps to avoid WiFi saturation on high-motion scenes
    int64_t sendTime = esp_timer_get_time() - frameStart;
    int64_t minFrameTime = (targetFps > 0) ? (1000000 / targetFps) : 33000;
    if (sendTime < minFrameTime) {
      vTaskDelay((minFrameTime - sendTime) / 1000 / portTICK_PERIOD_MS);
    }

    // Update telemetry
    fpsCount++;
    frameCount++;
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - fpsTimer;
    if (elapsed >= 1000000) { // every second
      streamFps = fpsCount * 1000000.0f / elapsed;
      fpsTimer = now;
      fpsCount = 0;
      Serial.printf("FPS: %.1f  Frame: %u bytes  Time: %u ms\n",
                    streamFps, lastFrameBytes, lastFrameMs);
    }
  }
  return res;
}

// FIX 4: Serve the UI page via esp_http_server instead of Arduino WebServer.
// This eliminates handleClient() from the main loop, which was blocking and
// competing with ArduinoOTA for CPU time. esp_http_server runs in its own
// RTOS task, so it never blocks the main loop.
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// Long-poll: blocks until burstGen > gen parameter, then returns stats JSON
static esp_err_t burst_wait_handler(httpd_req_t *req) {
  lastLaptopContactMs = millis();
  char buf[32];
  int knownGen = 0;
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    char val[16];
    if (httpd_query_key_value(buf, "gen", val, sizeof(val)) == ESP_OK)
      knownGen = atoi(val);
  }
  // Wait up to 30s for a new burst
  for (int i = 0; i < 600 && burstGen <= knownGen; i++) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
  // Fall through to stats response (even on timeout — client retries)
  // Build the same JSON as stats_handler
  char json[512];
  char archBuf[80] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[12];
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", burstArchives[a].count);
    strlcat(archBuf, tmp, sizeof(archBuf));
  }
  strlcat(archBuf, "]", sizeof(archBuf));
  bool laptopPresent1 = (lastLaptopContactMs > 0) &&
                        (millis() - lastLaptopContactMs < LAPTOP_TIMEOUT_MS);
  snprintf(json, sizeof(json),
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u,\"motion\":%s,\"distance\":%d,\"lux\":%u,\"burstArchives\":%d,\"burstGen\":%d,\"burstCounts\":%s,\"laptopPresent\":%s,\"freePsram\":%u,\"uptimeMs\":%lu}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount,
    motionDetected ? "true" : "false", tofDistance, alsLux, burstArchiveCount, burstGen, archBuf,
    laptopPresent1 ? "true" : "false", ESP.getFreePsram(), millis());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t stats_handler(httpd_req_t *req) {
  char json[512];
  // Build burst archive counts array
  char archBuf[80] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[12];
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", burstArchives[a].count);
    strlcat(archBuf, tmp, sizeof(archBuf));
  }
  strlcat(archBuf, "]", sizeof(archBuf));
  bool laptopPresent2 = (lastLaptopContactMs > 0) &&
                        (millis() - lastLaptopContactMs < LAPTOP_TIMEOUT_MS);
  snprintf(json, sizeof(json),
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u,\"motion\":%s,\"distance\":%d,\"lux\":%u,\"burstArchives\":%d,\"burstGen\":%d,\"burstCounts\":%s,\"laptopPresent\":%s,\"freePsram\":%u,\"uptimeMs\":%lu}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount,
    motionDetected ? "true" : "false", tofDistance, alsLux, burstArchiveCount, burstGen, archBuf,
    laptopPresent2 ? "true" : "false", ESP.getFreePsram(), ESP.getFreePsram(), millis());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t burstmeta_handler(httpd_req_t *req) {
  char buf[48];
  char val[8];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len <= 1 || len > (int)sizeof(buf)) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  int archIdx = 0;
  if (httpd_query_key_value(buf, "a", val, sizeof(val)) == ESP_OK) archIdx = atoi(val);
  if (archIdx < 0 || archIdx >= burstArchiveCount) { httpd_resp_send_404(req); return ESP_FAIL; }

  BurstArchive &archive = burstArchives[archIdx];
  char frameBuf[160] = "[";
  for (int i = 0; i < archive.count; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%lu", i > 0 ? "," : "", archive.images[i].captureMs);
    strlcat(frameBuf, tmp, sizeof(frameBuf));
  }
  strlcat(frameBuf, "]", sizeof(frameBuf));

  char json[768];
  // API results per frame
  char apiBuf[80] = "[";
  for (int i = 0; i < archive.count; i++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%s%d", i > 0 ? "," : "", archive.apiResults[i]);
    strlcat(apiBuf, tmp, sizeof(apiBuf));
  }
  strlcat(apiBuf, "]", sizeof(apiBuf));

  snprintf(json, sizeof(json),
    "{\"archive\":%d,\"generation\":%d,\"count\":%d,\"triggerMs\":%lu,\"archiveMs\":%lu,\"firstFrameMs\":%lu,\"lastFrameMs\":%lu,\"frameCaptureMs\":%s,"
    "\"apiPreyDetected\":%d,\"apiFramesSent\":%d,\"apiCallMs\":%lu,\"apiResults\":%s,\"uptimeMs\":%lu}",
    archIdx, archive.generation, archive.count, archive.triggerMs, archive.timestamp,
    archive.firstFrameMs, archive.lastFrameMs, frameBuf,
    archive.apiPreyDetected, archive.apiFramesSent, archive.apiCallMs, apiBuf, millis());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

// ===== API test endpoint — single frame capture → crop → API call =====
static esp_err_t apitest_handler(httpd_req_t *req) {
  char json[512];
  // Grab a frame from camera
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    snprintf(json, sizeof(json), "{\"error\":\"camera_fail\"}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, strlen(json));
  }
  size_t origLen = fb->len;

  // callPreyApi handles cropping internally
  unsigned long t1 = millis();
  int result = callPreyApi(fb->buf, fb->len);
  unsigned long apiMs = millis() - t1;
  esp_camera_fb_return(fb);

  snprintf(json, sizeof(json),
    "{\"result\":%d,\"origLen\":%u,\"apiMs\":%lu,\"freePsram\":%u,\"espErr\":\"0x%x\",\"httpStatus\":%d,\"apiResponse\":\"%s\"}",
    result, origLen, apiMs, ESP.getFreePsram(), lastApiEspErr, lastApiHttpStatus, apiResponseBuf);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char buf[64];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len <= 1 || len > (int)sizeof(buf)) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  httpd_req_get_url_query_str(req, buf, sizeof(buf));

  char val[16];
  sensor_t *s = esp_camera_sensor_get();

  if (httpd_query_key_value(buf, "quality", val, sizeof(val)) == ESP_OK) {
    jpegQuality = atoi(val);
  } else if (httpd_query_key_value(buf, "trigger", val, sizeof(val)) == ESP_OK) {
    // Fake trigger: simulate ToF detection for testing
    if (!burstCapturing && postTriggerRemaining == 0) {
      pendingBurstTriggerMs = millis();
      postTriggerRemaining = POST_TRIGGER_FRAMES;
      Serial.println("Fake trigger from web UI");
    }
  } else if (httpd_query_key_value(buf, "brightness", val, sizeof(val)) == ESP_OK) {
    s->set_brightness(s, atoi(val));
  } else if (httpd_query_key_value(buf, "contrast", val, sizeof(val)) == ESP_OK) {
    s->set_contrast(s, atoi(val));
  } else if (httpd_query_key_value(buf, "aec", val, sizeof(val)) == ESP_OK) {
    s->set_exposure_ctrl(s, atoi(val));
  } else if (httpd_query_key_value(buf, "ae_level", val, sizeof(val)) == ESP_OK) {
    s->set_ae_level(s, atoi(val));
  } else if (httpd_query_key_value(buf, "fps", val, sizeof(val)) == ESP_OK) {
    targetFps = atoi(val);
  } else if (httpd_query_key_value(buf, "gainceiling", val, sizeof(val)) == ESP_OK) {
    s->set_gainceiling(s, (gainceiling_t)atoi(val));
  } else if (httpd_query_key_value(buf, "nightmode", val, sizeof(val)) == ESP_OK) {
    s->set_aec2(s, atoi(val));
  } else if (httpd_query_key_value(buf, "reboot", val, sizeof(val)) == ESP_OK) {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "Rebooting...", 12);
    delay(500);
    ESP.restart();
  }

  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

void startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32769;
  // FIX 3: Increase httpd task stack from default 4096 to 8192.
  // The stream handler does PSRAM access + chunk encoding which needs more stack.
  config.stack_size = 8192;
  // FIX 3b: Raise httpd task priority above Arduino loop (priority 1).
  // This ensures the stream task is not starved by loop() work.
  config.task_priority = tskIDLE_PRIORITY + 5;

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  config.max_uri_handlers = 3;
  config.lru_purge_enable = true;
  config.max_open_sockets = 3;  // stream + burst_wait + margin

  httpd_uri_t burst_wait_uri = {
    .uri = "/burst_wait",
    .method = HTTP_GET,
    .handler = burst_wait_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &burst_wait_uri);
    Serial.println("Stream server started on port 81");
  }
}

// ===== MJPEG burst stream: all frames in one multipart response =====
static esp_err_t burststream_handler(httpd_req_t *req) {
  lastLaptopContactMs = millis();
  char buf[48], val[8];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len <= 1 || len > (int)sizeof(buf)) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  int archIdx = 0;
  if (httpd_query_key_value(buf, "a", val, sizeof(val)) == ESP_OK) archIdx = atoi(val);
  if (archIdx < 0 || archIdx >= burstArchiveCount) { httpd_resp_send_404(req); return ESP_FAIL; }

  int fd = httpd_req_to_sockfd(req);
  int yes = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

  httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part_buf[80];
  BurstArchive &archive = burstArchives[archIdx];

  for (int i = 0; i < archive.count; i++) {
    if (!archive.images[i].buf) continue;
    // Burst images are already JPEG
    httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, archive.images[i].len);
    httpd_resp_send_chunk(req, part_buf, hlen);
    esp_err_t res = httpd_resp_send_chunk(req, (const char *)archive.images[i].buf, archive.images[i].len);
    if (res != ESP_OK) return res;  // client disconnected
  }
  // Final boundary + empty chunk to signal end of stream
  httpd_resp_send_chunk(req, "\r\n--" PART_BOUNDARY "--\r\n", strlen("\r\n--" PART_BOUNDARY "--\r\n"));
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static esp_err_t burst_handler(httpd_req_t *req) {
  char buf[48];
  char val[8];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len <= 1 || len > (int)sizeof(buf)) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  // /burst?a=archiveIdx&i=imageIdx
  int archIdx = 0, imgIdx = 0;
  if (httpd_query_key_value(buf, "a", val, sizeof(val)) == ESP_OK) archIdx = atoi(val);
  if (httpd_query_key_value(buf, "i", val, sizeof(val)) == ESP_OK) imgIdx = atoi(val);
  if (archIdx < 0 || archIdx >= burstArchiveCount) { httpd_resp_send_404(req); return ESP_FAIL; }
  if (imgIdx < 0 || imgIdx >= burstArchives[archIdx].count || !burstArchives[archIdx].images[imgIdx].buf) { httpd_resp_send_404(req); return ESP_FAIL; }
  // Burst images are already JPEG — serve directly
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, (const char *)burstArchives[archIdx].images[imgIdx].buf,
                         burstArchives[archIdx].images[imgIdx].len);
}

void startUIServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.stack_size = 16384;
  config.max_uri_handlers = 10;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stats_uri = {
    .uri = "/stats",
    .method = HTTP_GET,
    .handler = stats_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/cmd",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t burst_uri = {
    .uri = "/burst",
    .method = HTTP_GET,
    .handler = burst_handler,
    .user_ctx = NULL
  };

  httpd_uri_t burstmeta_uri = {
    .uri = "/burstmeta",
    .method = HTTP_GET,
    .handler = burstmeta_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&ui_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(ui_httpd, &index_uri);
    httpd_register_uri_handler(ui_httpd, &stats_uri);
    httpd_register_uri_handler(ui_httpd, &cmd_uri);
    httpd_register_uri_handler(ui_httpd, &burst_uri);
    httpd_register_uri_handler(ui_httpd, &burstmeta_uri);
    httpd_uri_t apitest_uri = {
      .uri = "/apitest",
      .method = HTTP_GET,
      .handler = apitest_handler,
    };
    httpd_register_uri_handler(ui_httpd, &apitest_uri);
    httpd_uri_t burststream_uri = {
      .uri = "/burststream",
      .method = HTTP_GET,
      .handler = burststream_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(ui_httpd, &burststream_uri);
    Serial.println("UI server started on port 80");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(PIR_PIN, INPUT);

  Wire.begin(TOF_SDA, TOF_SCL);
  Wire.setClock(400000); // 400kHz fast mode
  Wire.beginTransmission(TOF_ADDR);
  if (Wire.endTransmission() == 0) {
    tofInit();
    // Start continuous ranging
    tofWriteReg(0x0018, 0x03); // SYSRANGE__START = continuous mode
    tofReady = true;
    Serial.println("TOF050C sensor ready (continuous mode)");
  } else {
    Serial.println("TOF050C not found on I2C");
  }

  if (!initCamera()) {
    Serial.println("Camera init failed \u2013 restarting in 5 s");
    delay(5000);
    ESP.restart();
  }
  Serial.printf("Camera ready: JPEG %dx%d, HDR gain bracketing\n", CAM_W, CAM_H);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // FIX 2: Disable WiFi power save mode.
  // Default is WIFI_PS_MIN_MODEM — the radio sleeps between DTIM beacons
  // (~100-300ms), causing periodic TCP stalls visible as stream freezes.
  esp_wifi_set_ps(WIFI_PS_NONE);

  // FIX 7: Set WiFi TX power to near maximum for strongest signal.
  // Value is in 0.25dBm units. 78 = 19.5dBm (near max of 20dBm).
  esp_wifi_set_max_tx_power(78);

  // FIX 4: Both servers now use esp_http_server (own RTOS tasks)
  startUIServer();
  startStreamServer();

  // OTA updates — start AFTER HTTP servers so reboot cmd is always available
  delay(500); // let WiFi + HTTP fully settle
  ArduinoOTA.setHostname("esp32cam");
  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    burstCapturing = true;
    Serial.println("OTA Start - freeing resources");
    // Stop stream server first (frees WiFi bandwidth + RTOS task)
    if (stream_httpd) { httpd_stop(stream_httpd); stream_httpd = NULL; }
    // Keep ui_httpd alive so /cmd?reboot=1 still works as emergency escape
    // Deinit camera to free DMA buffers and PSRAM
    esp_camera_deinit();
    delay(100); // let pending WiFi packets drain
    Serial.println("OTA: resources freed, ready for upload");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA End - rebooting");
    otaInProgress = false;
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPct = 999;
    unsigned int pct = progress / (total / 100);
    if (pct != lastPct) {
      Serial.printf("OTA: %u%%\n", pct);
      lastPct = pct;
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u] - rebooting to recover\n", error);
    delay(1000);
    ESP.restart(); // clean reboot is safer than partial reinit
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");
}

// ===== Loop =====
// FIX 1: Throttle ArduinoOTA.handle() to reduce mDNS polling overhead.
// ArduinoOTA.handle() calls mDNS internally, which sends/receives UDP
// multicast packets that compete with MJPEG TCP traffic on the WiFi radio.
// Calling it every 500ms instead of every ~1ms is sufficient for OTA discovery
// while dramatically reducing WiFi contention during streaming.
void loop() {
  unsigned long now = millis();

  // During OTA: only handle OTA, skip everything else
  if (otaInProgress) {
    ArduinoOTA.handle();
    delay(1); // yield to RTOS — prevents task WDT reset
    return;
  }

  static unsigned long lastOTA = 0;
  if (now - lastOTA >= 500) {
    lastOTA = now;
    ArduinoOTA.handle();
  }
  motionDetected = digitalRead(PIR_PIN) == HIGH;

  static unsigned long lastTof = 0;
  static unsigned long lastValidTof = now; // watchdog: last time we got a valid reading
  if (tofReady && now - lastTof >= 1) { // poll as fast as possible
    lastTof = now;
    uint8_t status = tofReadReg(0x004F); // RESULT__INTERRUPT_STATUS_GPIO
    if (status & 0x04) { // range ready
      uint8_t rangeStatus = (tofReadReg(0x004D) >> 4) & 0x0F;
      // Read 2 bytes from result register 0x0062, divide by 100 for mm
      Wire.beginTransmission(TOF_ADDR);
      Wire.write(0x00); Wire.write(0x62);
      Wire.endTransmission();
      Wire.requestFrom(TOF_ADDR, 2);
      if (Wire.available() == 2) {
        int hi = Wire.read();
        int lo = Wire.read();
        int d = ((hi << 8) | lo) / 100;
        if (rangeStatus == 0 && d <= 600) {
          tofDistance = d;  // valid reading
        } else if (rangeStatus == 6 || rangeStatus == 5 || d > 600) {
          tofDistance = -1;  // no target / out of range
        } else {
          tofDistance = -2;  // sensor error
        }
        lastValidTof = now;
      }
      tofWriteReg(0x0015, 0x07); // clear interrupts
    }
    // Watchdog: if no valid reading for 3 seconds, reinit sensor
    if (now - lastValidTof > 3000) {
      Serial.println("TOF watchdog: no reading for 3s, reinitializing...");
      tofWriteReg(0x0018, 0x00); // stop continuous mode
      delay(10);
      tofInit();
      tofWriteReg(0x0018, 0x03); // restart continuous mode
      lastValidTof = now;
      Serial.println("TOF watchdog: sensor restarted");
    }
  }

  if (tofReady && tofDistance >= 0 && tofDistance < 480
      && !burstCapturing && postTriggerRemaining == 0 && (now - burstCooldown > 5000)) {
    pendingBurstTriggerMs = now;
    postTriggerRemaining = POST_TRIGGER_FRAMES; // start post-trigger countdown
  }

  // === ALS (ambient light sensor) reading from VL6180X ===
  static unsigned long lastAls = 0;
  if (tofReady && now - lastAls >= 500) { // read ALS every 500ms
    lastAls = now;
    // Start single-shot ALS measurement
    tofWriteReg(0x0038, 0x01); // SYSALS__START = single shot
    // Wait for result (typically <110ms with 100ms integration)
    for (int w = 0; w < 20; w++) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      uint8_t st = tofReadReg(0x004F);
      if (st & 0x20) break; // ALS ready
    }
    // Read raw ALS value (16-bit, register 0x0050-0x0051)
    Wire.beginTransmission(TOF_ADDR);
    Wire.write(0x00); Wire.write(0x50);
    Wire.endTransmission();
    Wire.requestFrom(TOF_ADDR, 2);
    if (Wire.available() == 2) {
      uint16_t raw = (Wire.read() << 8) | Wire.read();
      // Convert to lux: lux = raw * 0.32 / gain / integration_time_ms * 100
      // With gain=1.0 (reg 0x3F=0x46→gain=1.0) and integration=100ms:
      alsLux = (uint16_t)(raw * 0.32f);
    }
    tofWriteReg(0x0015, 0x07); // clear interrupts
  }

  // Continuously fill ring buffer with JPEG frames (HDR gain+exposure bracketing)
  static unsigned long lastRing = 0;
  static int hdrIdx = 0;  // cycles through HDR_STEPS[]
  if (!burstCapturing && now - lastRing >= 100) { // ~10 fps ring buffer
    lastRing = now;

    // Set gain + exposure for this frame
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
      int step = hdrIdx % HDR_STEP_COUNT;
      int gain = HDR_STEPS[step].gain;
      int aec  = HDR_STEPS[step].aec;
      // In bright conditions (ALS), cap gain and use longer exposure
      if (alsLux > 100) {
        gain = (gain > 6) ? 6 : gain;
      }
      // In dark conditions, cap exposure to avoid IR blowout
      if (alsLux < 10) {
        aec = (aec > AEC_LOW) ? AEC_LOW : aec;
      }
      s->set_agc_gain(s, gain);
      s->set_aec_value(s, aec);
    }
    hdrIdx++;

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      // Store JPEG directly — free old buffer if different size
      if (ringBuf[ringHead].buf) {
        free(ringBuf[ringHead].buf);
        ringBuf[ringHead].buf = NULL;
      }
      // Copy JPEG to PSRAM
      ringBuf[ringHead].buf = (uint8_t *)ps_malloc(fb->len);
      if (ringBuf[ringHead].buf) {
        memcpy(ringBuf[ringHead].buf, fb->buf, fb->len);
        ringBuf[ringHead].len = fb->len;
        ringBuf[ringHead].captureMs = now;
        ringHead = (ringHead + 1) % RING_SIZE;
        if (ringCount < RING_SIZE) ringCount++;
        // Count down post-trigger frames, freeze when done
        if (postTriggerRemaining > 0) {
          postTriggerRemaining--;
          if (postTriggerRemaining == 0) {
            freezeRingToArchive();
            // If laptop absent, call prey API autonomously
            bool laptopPresent = (lastLaptopContactMs > 0) &&
                                 (millis() - lastLaptopContactMs < LAPTOP_TIMEOUT_MS);
            if (!laptopPresent && burstArchiveCount > 0) {
              autonomousApiCheck(burstArchiveCount - 1);
            }
          }
        }
      }
      esp_camera_fb_return(fb);
    }
  }
}
