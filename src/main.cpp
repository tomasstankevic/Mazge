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
#include <Preferences.h>
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

// ===== TOF050C / VL6180X ToF sensor (raw I2C, 16-bit registers) =====
#define TOF_SDA 47
#define TOF_SCL 21
#define TOF_ADDR 0x29
#define TOF_MIN_MM 30  // VL6180X minimum reliable range; below this is noise
volatile int tofDistance = -2; // mm, -1 = no object (>range), -2 = sensor error
volatile uint16_t alsLux = 0;  // ambient light from VL6180X ALS (broken - always 0)
volatile int autoBaseAec = 100; // camera-determined base exposure via periodic AEC probe
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
#define RING_SIZE 10         // 5 pre-trigger + 5 post-trigger
#define BURST_ARCHIVES 40

// ===== Prey Detection API (autonomous mode) =====
#define API_FRAMES_PER_BURST 3   // send up to 3 best frames per burst
#define LAPTOP_TIMEOUT_MS 30000  // consider laptop absent after 30s no contact
volatile unsigned long lastLaptopContactMs = 0;  // last /burst_wait or /burststream request

// ===== HDR gain + exposure bracketing =====
// Manual exposure: capped at 1/40s to avoid motion blur on moving cats.
// OV2640 AEC value ≈ line count. At 20MHz XCLK, VGA: 1 line ≈ 80µs.
// 1/40s = 25ms → ~312 lines. We bracket exposure in 2 steps within that cap.
#define AEC_MAX_DEFAULT 300       // ~24ms ≈ 1/40s
#define AEC_LOW_DEFAULT 100       // ~8ms — short exposure for IR night
volatile int aecMax = AEC_MAX_DEFAULT;
volatile int aecLow = AEC_LOW_DEFAULT;
// Day/night mode based on autoBaseAec (camera auto-exposure probe).
// autoBaseAec is high when background is dark (IR night), low when bright (day).
// At night, IR LEDs blow out the cat face (close object) while background stays
// dark, so auto-brightness (which sees only background) overexposes the cat.
// Solution: when autoBaseAec > nightAecThreshold, aggressively underexpose.
volatile int nightAecThreshold = 200; // autoBaseAec above this → night/IR mode
volatile int nightGainCap = 8;        // max gain in night mode (moderate for IR)
volatile int nightExposureCap = 300;  // max AEC in night mode (~24ms ≈ 1/42s, avoids motion blur)
volatile int dayLuxThreshold = 100;   // (legacy, unused — kept for settings compat)
volatile int nightLuxThreshold = 10;  // (legacy, unused)
volatile int dayGainCap = 2;          // (legacy, unused)
volatile int dayExposureDiv = 4;      // (legacy, unused)
volatile int dayMinExposure = 20;     // (legacy, unused)
volatile int apiFallbackMs = 5000;    // fallback timeout (ms)
volatile int burstTriggerMm = 480;    // ToF trigger distance (mm)
volatile int burstCooldownMs = 15000;  // cooldown between bursts (ms)
// Gain brackets: 10 frames cycling through (gain, exposure) pairs.
// Low gain+short exposure first (best for IR night), then ramp up for day.
struct HdrStep { int gain; int aec; };
// Runtime-editable HDR steps (initialized from defaults)
HdrStep hdrSteps[10] = {
  {0,  AEC_LOW_DEFAULT},  {0,  AEC_MAX_DEFAULT},
  {2,  AEC_LOW_DEFAULT},  {2,  AEC_MAX_DEFAULT},
  {6,  AEC_LOW_DEFAULT},  {6,  AEC_MAX_DEFAULT},
  {12, AEC_LOW_DEFAULT},  {12, AEC_MAX_DEFAULT},
  {20, AEC_LOW_DEFAULT},  {30, AEC_LOW_DEFAULT},
};
#define HDR_STEP_COUNT 10
struct BurstImage {
  uint8_t *buf;
  size_t len;
  unsigned long captureMs;
  int16_t distanceMm;  // ToF distance at capture time
  int16_t gainApplied;  // AGC gain value applied to this frame
  int16_t aecApplied;   // AEC exposure value applied to this frame
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
  // Per-frame timing breakdown (up to API_FRAMES_PER_BURST)
  unsigned long cropMs[RING_SIZE];
  unsigned long b64Ms[RING_SIZE];
  unsigned long tlsMs[RING_SIZE];
  unsigned long postMs[RING_SIZE];
  unsigned long totalMs[RING_SIZE];
  unsigned long apiDoneMs;        // when autonomousApiCheck finished
};
BurstArchive burstArchives[BURST_ARCHIVES];
volatile int burstArchiveCount = 0;
volatile int burstGen = 0;  // increments on each new burst
volatile bool burstCapturing = false;
volatile bool otaInProgress = false;
unsigned long burstCooldown = 0;
unsigned long pendingBurstTriggerMs = 0;

// Post-trigger: capture N more frames after trigger before freezing
#define POST_TRIGGER_FRAMES 5
int postTriggerRemaining = 0;  // >0 means we're in post-trigger phase

// ===== Persistent event log (NVS) =====
#define MAX_EVENTS 50
struct EventEntry {
  uint32_t uptimeMs;   // millis() when result finalized
  int16_t  gen;        // burst generation
  int8_t   frameCount; // number of frames in burst
  int8_t   result;     // -1=pending, 0=no prey, 1=prey
  int16_t  distMin;    // min distance mm (-1 = unknown)
  int16_t  distMax;    // max distance mm (-1 = unknown)
  uint8_t  mode;       // 0=laptop, 1=autonomous
  int8_t   trend;      // 0=unknown, 1=entering (far→close), 2=exiting (close→far), 3=passing
}; // 13 bytes per entry
Preferences nvsPrefs;
EventEntry eventLog[MAX_EVENTS];
int eventCount = 0;

void saveEventLog() {
  nvsPrefs.begin("evlog", false);
  nvsPrefs.putInt("count", eventCount);
  nvsPrefs.putBytes("entries", eventLog, sizeof(EventEntry) * eventCount);
  nvsPrefs.end();
}

void loadEventLog() {
  nvsPrefs.begin("evlog", true);
  eventCount = nvsPrefs.getInt("count", 0);
  if (eventCount > MAX_EVENTS) eventCount = MAX_EVENTS;
  if (eventCount > 0) {
    nvsPrefs.getBytes("entries", eventLog, sizeof(EventEntry) * eventCount);
  }
  nvsPrefs.end();
  Serial.printf("Loaded %d events from NVS\n", eventCount);
}

// Classify distance trend from per-frame distances:
// 1=entering (decreasing: far→close), 2=exiting (close→far), 3=passing, 0=unknown
int classifyDistTrend(BurstArchive &archive) {
  // Find first and last valid distance readings
  int first = -1, last = -1;
  for (int i = 0; i < archive.count; i++) {
    int d = archive.images[i].distanceMm;
    if (d >= 0) { if (first < 0) first = d; last = d; }
  }
  if (first < 0 || last < 0) return 0; // no valid readings

  // Count frames that are close (<500mm) in first half vs second half
  int half = archive.count / 2;
  if (half < 1) half = 1;
  int closeFirst = 0, closeLast = 0;
  for (int i = 0; i < half; i++) {
    int d = archive.images[i].distanceMm;
    if (d >= 0 && d < 500) closeFirst++;
  }
  for (int i = archive.count - half; i < archive.count; i++) {
    int d = archive.images[i].distanceMm;
    if (d >= 0 && d < 500) closeLast++;
  }

  // Entering: first half mostly far, second half close (distance decreasing)
  if (closeLast > closeFirst + 1) return 1; // entering
  // Exiting: first half close, second half far (distance increasing)
  if (closeFirst > closeLast + 1) return 2; // exiting
  // Passing: close throughout
  if (closeFirst > 0 && closeLast > 0) return 3; // passing through
  return 0; // unknown
}

void addEvent(int gen, int frameCount, int result, int distMin, int distMax, int trend, bool autonomous) {
  if (eventCount >= MAX_EVENTS) {
    // Shift out oldest half to make room
    int keep = MAX_EVENTS / 2;
    memmove(eventLog, eventLog + (eventCount - keep), sizeof(EventEntry) * keep);
    eventCount = keep;
  }
  EventEntry &e = eventLog[eventCount];
  e.uptimeMs = millis();
  e.gen = (int16_t)gen;
  e.frameCount = (int8_t)frameCount;
  e.result = (int8_t)result;
  e.distMin = (int16_t)distMin;
  e.distMax = (int16_t)distMax;
  e.mode = autonomous ? 1 : 0;
  e.trend = (int8_t)trend;
  eventCount++;
  saveEventLog();
}

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

  // Free image data from oldest archives if PSRAM is running low (keep metadata)
  uint32_t freePsram = ESP.getFreePsram();
  Serial.printf("Burst: freePsram=%u before freeze\n", freePsram);
  if (freePsram < 500000 && burstArchiveCount > 0) {
    for (int a = 0; a < burstArchiveCount && ESP.getFreePsram() < 500000; a++) {
      bool hasImages = false;
      for (int i = 0; i < RING_SIZE; i++) {
        if (burstArchives[a].images[i].buf) { hasImages = true; break; }
      }
      if (hasImages) {
        for (int i = 0; i < RING_SIZE; i++) {
          if (burstArchives[a].images[i].buf) {
            free(burstArchives[a].images[i].buf);
            burstArchives[a].images[i].buf = NULL;
            burstArchives[a].images[i].len = 0;
          }
        }
        Serial.printf("Burst: freed images from archive %d (gen %d) to reclaim PSRAM (%u free)\n",
          a, burstArchives[a].generation, ESP.getFreePsram());
      }
    }
  }

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
    burstArchives[slot].images[saved].distanceMm = ringBuf[idx].distanceMm;
    burstArchives[slot].images[saved].gainApplied = ringBuf[idx].gainApplied;
    burstArchives[slot].images[saved].aecApplied = ringBuf[idx].aecApplied;
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
    ringBuf[i].distanceMm = -2;
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

// ===== Persistent TLS connection for prey API =====
static WiFiClientSecure *tlsClient = NULL;
static HTTPClient *httpApi = NULL;
static bool tlsConnected = false;
static unsigned long lastTlsConnectMs = 0;

// Ensure TLS connection is alive, reconnect if needed
static bool ensureTlsConnection() {
  if (tlsConnected && tlsClient && tlsClient->connected()) {
    return true;
  }
  Serial.println("TLS: (re)connecting...");
  unsigned long t0 = millis();

  if (httpApi) { httpApi->end(); delete httpApi; httpApi = NULL; }
  if (tlsClient) { tlsClient->stop(); delete tlsClient; tlsClient = NULL; }
  tlsConnected = false;

  tlsClient = new WiFiClientSecure();
  if (!tlsClient) { Serial.println("TLS: alloc failed"); return false; }
  tlsClient->setInsecure();

  httpApi = new HTTPClient();
  httpApi->setReuse(true); // enable HTTP keep-alive

  if (!httpApi->begin(*tlsClient, PREY_API_URL)) {
    Serial.println("TLS: begin failed");
    delete httpApi; httpApi = NULL;
    delete tlsClient; tlsClient = NULL;
    return false;
  }

  char authHeader[128];
  snprintf(authHeader, sizeof(authHeader), "Bearer %s", PREY_API_KEY);
  httpApi->addHeader("Content-Type", "application/json");
  httpApi->addHeader("Authorization", authHeader);
  httpApi->addHeader("Connection", "keep-alive");
  httpApi->setTimeout(15000);

  tlsConnected = true;
  lastTlsConnectMs = millis();
  Serial.printf("TLS: connected in %lums\n", millis() - t0);
  return true;
}

// ===== Crop JPEG for API =====
// From 640x480: remove right 128px (occluded), then center-crop to 384x384.
// All offsets MCU-aligned (16x8 for 4:2:2 YCbCr).
// Crop region in original image: x=64..448, y=48..432.
// Returns new JPEG in PSRAM (caller must free). Sets outLen. NULL on failure.
#define CROP_X  64    // left margin (64px, MCU-aligned)
#define CROP_Y  48    // top margin  (48px, MCU-aligned)
#define CROP_SZ 384   // output 384x384

#include "jpeg_lossless_crop.h"

static uint8_t *cropJpegForApi(const uint8_t *jpgBuf, size_t jpgLen, size_t *outLen) {
  unsigned long t0 = millis();
  *outLen = 0;
  Serial.printf("Crop: freeHeap=%u freePSRAM=%u jpgLen=%u\n",
    ESP.getFreeHeap(), ESP.getFreePsram(), jpgLen);

  uint8_t *result = jpeg_lossless_crop(jpgBuf, jpgLen,
    CROP_X, CROP_Y, CROP_SZ, CROP_SZ, outLen);

  unsigned long t1 = millis();

  if (!result) {
    Serial.println("Crop: lossless crop failed");
    return NULL;
  }

  Serial.printf("Crop: 640x480→384x384 JPEG %uB→%uB (lossless %lums)\n",
    jpgLen, *outLen, t1 - t0);
  return result;
}

// Timing breakdown for a single API call
struct ApiTiming {
  unsigned long b64Ms;
  unsigned long tlsMs;
  unsigned long postMs;
};

// Send a JPEG frame to the prey API. If croppedJpg/croppedLen provided, skip cropping.
// Returns: -1=error, 0=no prey, 1=prey
static int callPreyApi(const uint8_t *jpgBuf, size_t jpgLen,
                       const uint8_t *preCropped = NULL, size_t preCroppedLen = 0,
                       ApiTiming *timing = NULL) {
  const uint8_t *sendBuf;
  size_t sendLen;
  uint8_t *croppedJpg = NULL;

  if (preCropped && preCroppedLen > 0) {
    sendBuf = preCropped;
    sendLen = preCroppedLen;
    Serial.printf("API: sending %uB (pre-cropped)\n", sendLen);
  } else {
    size_t cl = 0;
    croppedJpg = cropJpegForApi(jpgBuf, jpgLen, &cl);
    sendBuf = croppedJpg ? croppedJpg : jpgBuf;
    sendLen = croppedJpg ? cl : jpgLen;
    Serial.printf("API: sending %uB %s\n", sendLen, croppedJpg ? "(cropped)" : "(original)");
  }

  // Base64 encode
  unsigned long tb0 = millis();
  size_t b64Len = 0;
  mbedtls_base64_encode(NULL, 0, &b64Len, sendBuf, sendLen);
  char *b64Buf = (char *)ps_malloc(b64Len + 1);
  if (!b64Buf) { if (croppedJpg) free(croppedJpg); Serial.println("API: base64 malloc failed"); return -1; }
  mbedtls_base64_encode((unsigned char *)b64Buf, b64Len + 1, &b64Len, sendBuf, sendLen);
  b64Buf[b64Len] = 0;
  if (croppedJpg) free(croppedJpg);

  // Build JSON body
  size_t jsonLen = b64Len + 32;
  char *jsonBody = (char *)ps_malloc(jsonLen);
  if (!jsonBody) { free(b64Buf); Serial.println("API: json malloc failed"); return -1; }
  snprintf(jsonBody, jsonLen, "{\"image_base64\":\"%s\"}", b64Buf);
  free(b64Buf);
  unsigned long tb1 = millis();

  // Ensure persistent TLS connection
  unsigned long tt0 = millis();
  if (!ensureTlsConnection()) {
    free(jsonBody);
    lastApiEspErr = -1;
    return -1;
  }
  unsigned long tt1 = millis();

  apiResponseLen = 0;
  apiResponseBuf[0] = 0;
  lastApiEspErr = 0;
  lastApiHttpStatus = 0;

  unsigned long startMs = millis();
  int httpCode = httpApi->POST((uint8_t *)jsonBody, strlen(jsonBody));
  free(jsonBody);

  unsigned long elapsed = millis() - startMs;
  lastApiHttpStatus = httpCode;

  if (timing) {
    timing->b64Ms = tb1 - tb0;
    timing->tlsMs = tt1 - tt0;
    timing->postMs = elapsed;
  }
  Serial.printf("API: timing b64=%lums tls=%lums post=%lums\n", tb1 - tb0, tt1 - tt0, elapsed);

  if (httpCode <= 0) {
    Serial.printf("API: POST failed, error=%d (%lums): %s\n",
      httpCode, elapsed, httpApi->errorToString(httpCode).c_str());
    lastApiEspErr = httpCode;
    // Connection broken — force reconnect next time
    tlsConnected = false;
    return -1;
  }

  String response = httpApi->getString();

  strncpy(apiResponseBuf, response.c_str(), sizeof(apiResponseBuf) - 1);
  apiResponseBuf[sizeof(apiResponseBuf) - 1] = 0;
  apiResponseLen = response.length();

  Serial.printf("API: HTTP %d, %dB in %lums: %s\n",
    httpCode, apiResponseLen, elapsed, apiResponseBuf);

  if (httpCode != 200) return -1;

  if (strstr(apiResponseBuf, "\"detected\":true") || strstr(apiResponseBuf, "\"detected\": true"))
    return 1;
  return 0;
}

// Pick best N frames, crop+send each sequentially (keeps TLS warm between calls)
void autonomousApiCheck(int archIdx) {
  if (archIdx < 0 || archIdx >= burstArchiveCount) return;
  BurstArchive &archive = burstArchives[archIdx];
  if (archive.count == 0) return;

  // Sort frame indices by JPEG size descending (pick largest = most detail)
  int indices[RING_SIZE];
  for (int i = 0; i < archive.count; i++) indices[i] = i;
  for (int i = 0; i < min(API_FRAMES_PER_BURST, archive.count); i++) {
    for (int j = i + 1; j < archive.count; j++) {
      if (archive.images[indices[j]].len > archive.images[indices[i]].len) {
        int tmp = indices[i]; indices[i] = indices[j]; indices[j] = tmp;
      }
    }
  }

  int framesToSend = min(API_FRAMES_PER_BURST, archive.count);
  archive.apiCallMs = millis();
  archive.apiPreyDetected = 0;

  Serial.printf("API: autonomous check, %d/%d frames, archive %d (gen %d)\n",
    framesToSend, archive.count, archIdx, archive.generation);

  // Ensure TLS is connected before first call
  ensureTlsConnection();

  // Crop and send each frame sequentially (keeps TLS connection warm)
  for (int i = 0; i < framesToSend; i++) {
    int fIdx = indices[i];
    if (!archive.images[fIdx].buf) continue;

    unsigned long frameT0 = millis();

    // Crop this frame
    unsigned long ct0 = millis();
    size_t croppedLen = 0;
    uint8_t *cropped = cropJpegForApi(archive.images[fIdx].buf, archive.images[fIdx].len, &croppedLen);
    unsigned long cropDt = millis() - ct0;

    // Send immediately (connection still warm)
    ApiTiming timing = {0, 0, 0};
    int result = callPreyApi(archive.images[fIdx].buf, archive.images[fIdx].len,
                             cropped, croppedLen, &timing);
    unsigned long frameDt = millis() - frameT0;

    if (cropped) free(cropped);

    archive.apiResults[fIdx] = result;
    archive.cropMs[i] = cropDt;
    archive.b64Ms[i] = timing.b64Ms;
    archive.tlsMs[i] = timing.tlsMs;
    archive.postMs[i] = timing.postMs;
    archive.totalMs[i] = frameDt;
    archive.apiFramesSent++;

    Serial.printf("API: frame[%d] crop=%lums b64=%lums tls=%lums post=%lums total=%lums result=%d\n",
      fIdx, cropDt, timing.b64Ms, timing.tlsMs, timing.postMs, frameDt, result);

    if (result == 1) {
      archive.apiPreyDetected = 1;
      Serial.println("API: *** PREY DETECTED ***");
    }
  }

  Serial.printf("API: done. %d frames sent, prey=%d\n",
    archive.apiFramesSent, archive.apiPreyDetected);
  archive.apiDoneMs = millis();
  // Persist event to NVS
  int dMin = 9999, dMax = -9999;
  for (int i = 0; i < archive.count; i++) {
    int d = archive.images[i].distanceMm;
    if (d >= 0) { if (d < dMin) dMin = d; if (d > dMax) dMax = d; }
  }
  if (dMin > dMax) { dMin = -1; dMax = -1; }
  int trend = classifyDistTrend(archive);
  addEvent(archive.generation, archive.count, archive.apiPreyDetected, dMin, dMax, trend, true);
}

// FreeRTOS task wrapper for autonomousApiCheck
static void apiCheckTask(void *param) {
  int archIdx = (int)(intptr_t)param;
  autonomousApiCheck(archIdx);
  vTaskDelete(NULL);
}

// Fallback task: wait for laptop to process, then run autonomous
static void apiFallbackTask(void *param) {
  int archIdx = (int)(intptr_t)param;
  unsigned long t0 = millis();
  // Wait up to 15s, checking every 500ms if laptop has set a result
  while (millis() - t0 < (unsigned long)apiFallbackMs) {
    if (archIdx < 0 || archIdx >= burstArchiveCount) { vTaskDelete(NULL); return; }
    if (burstArchives[archIdx].apiPreyDetected != -1) {
      Serial.printf("API fallback: archive %d already processed (prey=%d), skipping\n",
        archIdx, burstArchives[archIdx].apiPreyDetected);
      vTaskDelete(NULL);
      return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  // Laptop didn't process in time — run autonomous check
  if (archIdx >= 0 && archIdx < burstArchiveCount &&
      burstArchives[archIdx].apiPreyDetected == -1) {
    Serial.printf("API fallback: laptop timeout, running autonomous check on archive %d\n", archIdx);
    autonomousApiCheck(archIdx);
  }
  vTaskDelete(NULL);
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
    .thumb-wrap { display:inline-block; position:relative; width:19%; aspect-ratio:3/4; overflow:hidden; border:2px solid #555; border-radius:3px; cursor:pointer; }
    .thumb-wrap .dist-label { position:absolute; bottom:0; left:0; right:0; background:rgba(0,0,0,0.7); color:#4cf; font-size:0.65em; text-align:center; padding:1px 0; z-index:1; pointer-events:none; }
    .thumb-wrap.api-prey { border-color:#f44; border-width:3px; box-shadow:0 0 6px #f44; }
    .thumb-wrap.api-clear { border-color:#4f4; border-width:3px; box-shadow:0 0 6px #4f4; }
    .thumb-wrap.api-err { border-color:#f84; border-width:3px; }
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
  <p id="sensor-line" style="font-size:1.1em;font-weight:bold;margin:4px 0 8px;">Distance: <span id="dist-val" style="color:#888;">--</span> | AEC: <span id="aec-val" style="color:#888;">--</span></p>
  <p id="mode-line" style="font-size:1em;margin:2px 0 6px;">Mode: <span id="mode-val" style="padding:2px 10px;border-radius:4px;font-weight:bold;">--</span></p>
  <button id="toggle-stream" onclick="toggleStream()" style="margin:0 0 8px;padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#a44;color:#fff;">Start Stream</button>
  <button onclick="fetch('/cmd?trigger=1')" style="margin:0 0 8px 8px;padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#c80;color:#fff;">Fake Trigger</button>
  <a href="/settings" style="margin-left:8px;padding:8px 16px;font-size:0.9em;border-radius:6px;background:#335;color:#8af;text-decoration:none;display:inline-block;">⚙ Settings</a>
  <div id="stream-wrap"><img id="stream" src="" onload="var w=this.naturalHeight,h=this.naturalWidth;this.parentElement.style.width=w+'px';this.parentElement.style.height=h+'px';this.style.width=h+'px';" /></div>
  <div id="events-section" style="width:100%;max-width:700px;margin-top:16px;">
    <h2 style="font-size:1.1em;margin:0 0 6px;">Events Log</h2>
    <div id="events-log" style="background:#0a0a1a;border:1px solid #333;border-radius:6px;padding:8px;max-height:200px;overflow-y:auto;font-family:monospace;font-size:0.8em;color:#aaa;"></div>
  </div>
  <div id="burst-section" style="width:100%;max-width:700px;margin-top:16px;">
    <h2 style="font-size:1.1em;margin:0 0 6px;">Burst Archives: <span id="burst-archive-count">0</span> | PSRAM: <span id="psram-val">--</span></h2>
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

    // Fetch per-frame API results and colorize thumbnail borders
    async function colorizeGallery(archIdx, galEl) {
      try {
        const r = await fetch('/burstmeta?a=' + archIdx);
        const m = await r.json();
        if (!m.apiResults) return;
        galEl.dataset.colored = '1';
        const wraps = galEl.querySelectorAll('.thumb-wrap');
        for (let i = 0; i < wraps.length && i < m.apiResults.length; i++) {
          const res = m.apiResults[i];
          if (res === 1) wraps[i].classList.add('api-prey');
          else if (res === 0) wraps[i].classList.add('api-clear');
          else if (res === -1) { /* not checked */ }
          else wraps[i].classList.add('api-err');
        }
        // Add distance labels
        if (m.distanceMm) {
          for (let i = 0; i < wraps.length && i < m.distanceMm.length; i++) {
            if (!wraps[i].querySelector('.dist-label')) {
              const lbl = document.createElement('span');
              lbl.className = 'dist-label';
              const d = m.distanceMm[i];
              lbl.textContent = d >= 0 ? d + 'mm' : (d === -1 ? '>500' : '--');
              wraps[i].appendChild(lbl);
            }
          }
        }
      } catch(e) {}
    }
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

    // Load persisted events from NVS on page load
    async function loadPersistedEvents() {
      try {
        const r = await fetch('/getevents');
        const events = await r.json();
        const el = document.getElementById('events-log');
        for (let i = events.length - 1; i >= 0; i--) {
          const e = events[i];
          const agoSec = (e.ago / 1000).toFixed(0);
          let mode, modeColor;
          if (e.mode === 1) { mode = '\u{1F916} AUTO'; modeColor = '#fc4'; }
          else { mode = '\u{1F4BB} LAPTOP'; modeColor = '#4f4'; }
          let distStr;
          if (e.dMin >= 0 && e.dMax >= 0) {
            distStr = e.dMin === e.dMax ? e.dMin + 'mm' : e.dMin + '\u2192' + e.dMax + 'mm';
          } else { distStr = '--'; }
          let resStr, resColor;
          if (e.res === 1) { resStr = '\u{1F534} PREY'; resColor = '#f44'; }
          else if (e.res === 0) { resStr = '\u{1F7E2} CLEAR'; resColor = '#4f4'; }
          else { resStr = '\u23F3 PENDING'; resColor = '#888'; }
          const div = document.createElement('div');
          div.id = 'pev-' + e.t;
          div.style.cssText = 'padding:3px 0;border-bottom:1px solid #222;opacity:0.7;';
          div.innerHTML = '<span style="color:#666">' + agoSec + 's ago</span> ' +
            '<b style="color:#aaa">gen' + e.gen + '</b> ' +
            '<span style="color:#6af">' + e.nf + 'f</span> ' +
            '<span style="color:' + modeColor + '">' + mode + '</span> ' +
            '<span style="color:#4cf">' + distStr + '</span> ' +
            '<span style="color:' + resColor + '">' + resStr + '</span>';
          el.appendChild(div);
        }
      } catch(e) { console.log('Failed to load persisted events:', e); }
    }
    loadPersistedEvents();

    async function pollStats() {
      try {
        const r = await fetch('/stats');
        const s = await r.json();
        document.getElementById('stats').textContent =
          `FPS: ${s.fps.toFixed(1)} | Frame: ${(s.frameBytes/1024).toFixed(1)} KB | Send: ${s.frameMs} ms | Total: ${s.totalFrames}`;
        const dv = document.getElementById('dist-val');
        if (s.distance >= 0) { dv.textContent = s.distance + ' mm'; dv.style.color = '#4cf'; }
        else if (s.distance === -1) { dv.textContent = '> 500 mm'; dv.style.color = '#888'; }
        else { dv.textContent = 'Sensor Error'; dv.style.color = '#f84'; }
        const av = document.getElementById('aec-val');
        if (av && s.autoAec !== undefined) { av.textContent = s.autoAec; av.style.color = s.autoAec > 300 ? '#fa0' : '#4f4'; }
        // PSRAM display
        const psv = document.getElementById('psram-val');
        if (psv && s.freePsram !== undefined) psv.textContent = (s.freePsram/1024).toFixed(0) + ' KB';
        // Mode indicator
        const modeEl = document.getElementById('mode-val');
        if (modeEl) {
          if (s.laptopPresent) {
            modeEl.textContent = '\u{1F4BB} LAPTOP';
            modeEl.style.background = '#264'; modeEl.style.color = '#4f4';
          } else {
            modeEl.textContent = '\u{1F916} AUTONOMOUS';
            modeEl.style.background = '#642'; modeEl.style.color = '#fc4';
          }
        }
        // Update events log with rich per-burst data
        if (s.apiResults && s.apiResults.length > 0) {
          const el = document.getElementById('events-log');
          const up = s.uptimeMs || 0;
          for (let a = 0; a < s.apiResults.length; a++) {
            const gen = s.burstGens ? s.burstGens[a] : (a+1);
            const res = s.apiResults[a];
            const sent = s.apiSent ? s.apiSent[a] : 0;
            const trigMs = s.triggerMs ? s.triggerMs[a] : 0;
            const doneMs = s.apiDoneMs ? s.apiDoneMs[a] : 0;
            const dMin = s.distMin ? s.distMin[a] : -1;
            const dMax = s.distMax ? s.distMax[a] : -1;
            const nf = s.burstCounts ? s.burstCounts[a] : 0;
            // Time: seconds since boot when triggered
            const tSec = (trigMs / 1000).toFixed(0);
            const tAgo = ((up - trigMs) / 1000).toFixed(0);
            // Mode: autonomous if apiFramesSent > 0, laptop if result set but no esp frames sent
            let mode, modeColor;
            if (res === -1) { mode = '⏳'; modeColor = '#888'; }
            else if (sent > 0) { mode = '🤖 AUTO'; modeColor = '#fc4'; }
            else { mode = '💻 LAPTOP'; modeColor = '#4f4'; }
            // Distance gradient
            let distStr;
            if (dMin >= 0 && dMax >= 0) {
              distStr = dMin === dMax ? dMin + 'mm' : dMin + '→' + dMax + 'mm';
            } else { distStr = '--'; }
            // Result
            let resStr, resColor;
            if (res === 1) { resStr = '🔴 PREY'; resColor = '#f44'; }
            else if (res === 0) { resStr = '🟢 CLEAR'; resColor = '#4f4'; }
            else { resStr = '⏳ PENDING'; resColor = '#888'; }
            // Processing time
            let procStr = '';
            if (doneMs > 0 && trigMs > 0) {
              procStr = ' ' + ((doneMs - trigMs) / 1000).toFixed(1) + 's';
            }
            const entryId = 'ev-' + gen;
            let existing = document.getElementById(entryId);
            const html = '<span style="color:#666">' + tAgo + 's ago</span> ' +
              '<b style="color:#aaa">gen' + gen + '</b> ' +
              '<span style="color:#6af">' + nf + 'f</span> ' +
              '<span style="color:' + modeColor + '">' + mode + '</span> ' +
              '<span style="color:#4cf">' + distStr + '</span> ' +
              '<span style="color:' + resColor + '">' + resStr + '</span>' +
              '<span style="color:#666">' + procStr + '</span>';
            if (!existing) {
              const div = document.createElement('div');
              div.id = entryId;
              div.style.cssText = 'padding:3px 0;border-bottom:1px solid #222;';
              div.innerHTML = html;
              el.insertBefore(div, el.firstChild);
            } else if (existing.dataset.res !== String(res) || existing.dataset.up !== String(up)) {
              existing.innerHTML = html;
            }
            if (existing) { existing.dataset.res = String(res); existing.dataset.up = String(up); }
          }
          el.dataset.count = String(s.apiResults.length);
        }
        // Update burst gallery
        const bac = document.getElementById('burst-archive-count');
        const ba = document.getElementById('burst-archives');
        bac.textContent = s.burstArchives;
        // Also update API result colors on existing galleries (only if thumbnails loaded)
        if (ba.dataset.key && s.burstArchives > 0) {
          for (let a = 0; a < s.burstArchives; a++) {
            const gal = document.getElementById('gal-' + a);
            if (gal && gal.childElementCount > 0 && !gal.dataset.colored && s.apiResults && s.apiResults[a] !== -1) {
              colorizeGallery(a, gal);
            }
          }
        }
        const key = s.burstGen;
        if (s.burstArchives > 0 && key !== parseInt(ba.dataset.key || '0')) {
          ba.dataset.key = key;
          ba.innerHTML = '';
          for (let a = s.burstArchives - 1; a >= 0; a--) {
            const isLatest = (a === s.burstArchives - 1);
            const apiRes = s.apiResults ? s.apiResults[a] : -1;
            const apiTag = apiRes === 1 ? ' \u{1F534}PREY' : apiRes === 0 ? ' \u{1F7E2}OK' : '';
            const gen = s.burstGens ? s.burstGens[a] : '';
            const div = document.createElement('div');
            div.style.cssText = 'margin:8px 0;padding:8px;background:#1a1a2e;border-radius:6px;cursor:pointer;';
            if (apiRes === 1) div.style.background = '#2e1a1a';
            const h = document.createElement('h3');
            h.style.cssText = 'font-size:0.95em;margin:0 0 4px;color:#aaa;';
            h.textContent = (isLatest ? '\u25BC ' : '\u25B6 ') + 'Burst #' + gen + ' (' + s.burstCounts[a] + 'f)' + apiTag;
            div.appendChild(h);
            const gallery = document.createElement('div');
            gallery.id = 'gal-' + a;
            gallery.style.cssText = 'display:flex;flex-wrap:wrap;gap:4px;' + (isLatest ? '' : 'display:none;');
            h.onclick = function() {
              gallery.style.display = gallery.style.display === 'none' ? 'flex' : 'none';
              h.textContent = (gallery.style.display === 'none' ? '\u25B6 ' : '\u25BC ') + 'Burst #' + gen + ' (' + s.burstCounts[a] + 'f)' + apiTag;
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
                colorizeGallery(a, gallery);
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
              colorizeGallery(a, gallery);
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
    s->set_exposure_ctrl(s, 0);  // DISABLE auto exposure — we control manually
    s->set_aec2(s, 0);           // disable DSP auto exposure
    s->set_gain_ctrl(s, 0);      // DISABLE auto gain — we control manually
    s->set_aec_value(s, aecLow); // start with short exposure
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
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u,\"distance\":%d,\"lux\":%u,\"autoAec\":%d,\"burstArchives\":%d,\"burstGen\":%d,\"burstCounts\":%s,\"laptopPresent\":%s,\"freePsram\":%u,\"uptimeMs\":%lu}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount,
    tofDistance, alsLux, autoBaseAec, burstArchiveCount, burstGen, archBuf,
    laptopPresent1 ? "true" : "false", ESP.getFreePsram(), millis());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t stats_handler(httpd_req_t *req) {
  char json[2048];
  // Build burst archive counts array
  char archBuf[80] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[12];
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", burstArchives[a].count);
    strlcat(archBuf, tmp, sizeof(archBuf));
  }
  strlcat(archBuf, "]", sizeof(archBuf));
  // Build per-archive API results array: -1=pending, 0=no prey, 1=prey
  char apiResBuf[120] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", burstArchives[a].apiPreyDetected);
    strlcat(apiResBuf, tmp, sizeof(apiResBuf));
  }
  strlcat(apiResBuf, "]", sizeof(apiResBuf));
  // Build per-archive generation array
  char genBuf[200] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[12];
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", burstArchives[a].generation);
    strlcat(genBuf, tmp, sizeof(genBuf));
  }
  strlcat(genBuf, "]", sizeof(genBuf));
  // Build per-archive apiFramesSent array (>0 means autonomous processed)
  char sentBuf[120] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", burstArchives[a].apiFramesSent);
    strlcat(sentBuf, tmp, sizeof(sentBuf));
  }
  strlcat(sentBuf, "]", sizeof(sentBuf));
  // Build per-archive trigger times and distance min/max
  char trigBuf[200] = "[";
  char distMinBuf[120] = "[";
  char distMaxBuf[120] = "[";
  char apiDoneBuf[200] = "[";
  for (int a = 0; a < burstArchiveCount; a++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%lu", a > 0 ? "," : "", burstArchives[a].triggerMs);
    strlcat(trigBuf, tmp, sizeof(trigBuf));
    snprintf(tmp, sizeof(tmp), "%s%lu", a > 0 ? "," : "", burstArchives[a].apiDoneMs);
    strlcat(apiDoneBuf, tmp, sizeof(apiDoneBuf));
    // Compute distance min/max across frames
    int dMin = 9999, dMax = -9999;
    for (int i = 0; i < burstArchives[a].count; i++) {
      int d = burstArchives[a].images[i].distanceMm;
      if (d >= 0) { if (d < dMin) dMin = d; if (d > dMax) dMax = d; }
    }
    if (dMin > dMax) { dMin = -1; dMax = -1; } // no valid readings
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", dMin);
    strlcat(distMinBuf, tmp, sizeof(distMinBuf));
    snprintf(tmp, sizeof(tmp), "%s%d", a > 0 ? "," : "", dMax);
    strlcat(distMaxBuf, tmp, sizeof(distMaxBuf));
  }
  strlcat(trigBuf, "]", sizeof(trigBuf));
  strlcat(distMinBuf, "]", sizeof(distMinBuf));
  strlcat(distMaxBuf, "]", sizeof(distMaxBuf));
  strlcat(apiDoneBuf, "]", sizeof(apiDoneBuf));
  bool laptopPresent2 = (lastLaptopContactMs > 0) &&
                        (millis() - lastLaptopContactMs < LAPTOP_TIMEOUT_MS);
  snprintf(json, sizeof(json),
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u,\"distance\":%d,\"lux\":%u,\"autoAec\":%d,"
    "\"burstArchives\":%d,\"burstGen\":%d,\"burstCounts\":%s,\"apiResults\":%s,\"burstGens\":%s,"
    "\"apiSent\":%s,\"triggerMs\":%s,\"apiDoneMs\":%s,\"distMin\":%s,\"distMax\":%s,"
    "\"laptopPresent\":%s,\"freePsram\":%u,\"uptimeMs\":%lu}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount,
    tofDistance, alsLux, autoBaseAec, burstArchiveCount, burstGen, archBuf,
    apiResBuf, genBuf, sentBuf, trigBuf, apiDoneBuf, distMinBuf, distMaxBuf,
    laptopPresent2 ? "true" : "false", ESP.getFreePsram(), millis());
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

  // Per-frame distance
  char distBuf[160] = "[";
  for (int i = 0; i < archive.count; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%d", i > 0 ? "," : "", archive.images[i].distanceMm);
    strlcat(distBuf, tmp, sizeof(distBuf));
  }
  strlcat(distBuf, "]", sizeof(distBuf));

  // Per-frame gain and AEC
  char gainBuf[120] = "[";
  char aecBuf[120] = "[";
  for (int i = 0; i < archive.count; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%d", i > 0 ? "," : "", archive.images[i].gainApplied);
    strlcat(gainBuf, tmp, sizeof(gainBuf));
    snprintf(tmp, sizeof(tmp), "%s%d", i > 0 ? "," : "", archive.images[i].aecApplied);
    strlcat(aecBuf, tmp, sizeof(aecBuf));
  }
  strlcat(gainBuf, "]", sizeof(gainBuf));
  strlcat(aecBuf, "]", sizeof(aecBuf));

  char json[1792];
  // API results per frame
  char apiBuf[80] = "[";
  for (int i = 0; i < archive.count; i++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%s%d", i > 0 ? "," : "", archive.apiResults[i]);
    strlcat(apiBuf, tmp, sizeof(apiBuf));
  }
  strlcat(apiBuf, "]", sizeof(apiBuf));

  // Per-frame timing arrays (up to apiFramesSent entries)
  char cropBuf[80] = "[", b64Buf[80] = "[", tlsBuf2[80] = "[", postBuf[80] = "[", totBuf[80] = "[";
  for (int i = 0; i < archive.apiFramesSent; i++) {
    char tmp[16];
    snprintf(tmp, sizeof(tmp), "%s%lu", i > 0 ? "," : "", archive.cropMs[i]); strlcat(cropBuf, tmp, sizeof(cropBuf));
    snprintf(tmp, sizeof(tmp), "%s%lu", i > 0 ? "," : "", archive.b64Ms[i]);  strlcat(b64Buf, tmp, sizeof(b64Buf));
    snprintf(tmp, sizeof(tmp), "%s%lu", i > 0 ? "," : "", archive.tlsMs[i]);  strlcat(tlsBuf2, tmp, sizeof(tlsBuf2));
    snprintf(tmp, sizeof(tmp), "%s%lu", i > 0 ? "," : "", archive.postMs[i]); strlcat(postBuf, tmp, sizeof(postBuf));
    snprintf(tmp, sizeof(tmp), "%s%lu", i > 0 ? "," : "", archive.totalMs[i]);strlcat(totBuf, tmp, sizeof(totBuf));
  }
  strlcat(cropBuf, "]", sizeof(cropBuf));
  strlcat(b64Buf, "]", sizeof(b64Buf));
  strlcat(tlsBuf2, "]", sizeof(tlsBuf2));
  strlcat(postBuf, "]", sizeof(postBuf));
  strlcat(totBuf, "]", sizeof(totBuf));

  snprintf(json, sizeof(json),
    "{\"archive\":%d,\"generation\":%d,\"count\":%d,\"triggerMs\":%lu,\"archiveMs\":%lu,\"firstFrameMs\":%lu,\"lastFrameMs\":%lu,\"frameCaptureMs\":%s,\"distanceMm\":%s,"
    "\"gainApplied\":%s,\"aecApplied\":%s,"
    "\"apiPreyDetected\":%d,\"apiFramesSent\":%d,\"apiCallMs\":%lu,\"apiResults\":%s,"
    "\"cropMs\":%s,\"b64Ms\":%s,\"tlsMs\":%s,\"postMs\":%s,\"totalMs\":%s,\"apiDoneMs\":%lu,\"uptimeMs\":%lu}",
    archIdx, archive.generation, archive.count, archive.triggerMs, archive.timestamp,
    archive.firstFrameMs, archive.lastFrameMs, frameBuf, distBuf,
    gainBuf, aecBuf,
    archive.apiPreyDetected, archive.apiFramesSent, archive.apiCallMs, apiBuf,
    cropBuf, b64Buf, tlsBuf2, postBuf, totBuf, archive.apiDoneMs, millis());
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

  // Escape quotes in apiResponseBuf for valid JSON embedding
  char escapedResp[384];
  int ei = 0;
  for (int i = 0; apiResponseBuf[i] && ei < (int)sizeof(escapedResp) - 2; i++) {
    if (apiResponseBuf[i] == '"') { escapedResp[ei++] = '\\'; }
    escapedResp[ei++] = apiResponseBuf[i];
  }
  escapedResp[ei] = 0;

  snprintf(json, sizeof(json),
    "{\"result\":%d,\"origLen\":%u,\"apiMs\":%lu,\"freePsram\":%u,\"espErr\":\"0x%x\",\"httpStatus\":%d,\"apiResponse\":\"%s\"}",
    result, origLen, apiMs, ESP.getFreePsram(), lastApiEspErr, lastApiHttpStatus, escapedResp);
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
  } else if (httpd_query_key_value(buf, "result", val, sizeof(val)) == ESP_OK) {
    // Laptop reporting prey detection result: /cmd?result=0&a=2
    int preyResult = atoi(val);
    char aVal[8];
    int archIdx = burstArchiveCount - 1; // default: latest archive
    if (httpd_query_key_value(buf, "a", aVal, sizeof(aVal)) == ESP_OK) {
      archIdx = atoi(aVal);
    }
    if (archIdx >= 0 && archIdx < burstArchiveCount) {
      burstArchives[archIdx].apiPreyDetected = preyResult ? 1 : 0;
      burstArchives[archIdx].apiDoneMs = millis();
      Serial.printf("Laptop result: archive %d prey=%d\n", archIdx, preyResult);
      // Persist event to NVS
      BurstArchive &la = burstArchives[archIdx];
      int ldMin = 9999, ldMax = -9999;
      for (int i = 0; i < la.count; i++) {
        int d = la.images[i].distanceMm;
        if (d >= 0) { if (d < ldMin) ldMin = d; if (d > ldMax) ldMax = d; }
      }
      if (ldMin > ldMax) { ldMin = -1; ldMax = -1; }
      int ltrend = classifyDistTrend(la);
      addEvent(la.generation, la.count, la.apiPreyDetected, ldMin, ldMax, ltrend, false);
    }
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

// ===== Persistent events API =====
static esp_err_t getevents_handler(httpd_req_t *req) {
  // Stream JSON array of persisted events
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send_chunk(req, "[", 1);
  char buf[128];
  unsigned long now = millis();
  for (int i = 0; i < eventCount; i++) {
    EventEntry &e = eventLog[i];
    int len = snprintf(buf, sizeof(buf),
      "%s{\"t\":%lu,\"ago\":%lu,\"gen\":%d,\"nf\":%d,\"res\":%d,\"dMin\":%d,\"dMax\":%d,\"mode\":%d,\"trend\":%d}",
      i > 0 ? "," : "",
      e.uptimeMs, (now > e.uptimeMs) ? (now - e.uptimeMs) : 0,
      e.gen, e.frameCount, e.result, e.distMin, e.distMax, e.mode, e.trend);
    httpd_resp_send_chunk(req, buf, len);
  }
  httpd_resp_send_chunk(req, "]", 1);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ===== Settings API =====
static esp_err_t getsettings_handler(httpd_req_t *req) {
  char json[768];
  char hdrBuf[256] = "[";
  for (int i = 0; i < HDR_STEP_COUNT; i++) {
    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%s[%d,%d]", i > 0 ? "," : "", hdrSteps[i].gain, hdrSteps[i].aec);
    strlcat(hdrBuf, tmp, sizeof(hdrBuf));
  }
  strlcat(hdrBuf, "]", sizeof(hdrBuf));
  snprintf(json, sizeof(json),
    "{\"aecMax\":%d,\"aecLow\":%d,\"dayLux\":%d,\"nightLux\":%d,"
    "\"dayGainCap\":%d,\"dayExpDiv\":%d,\"dayMinExp\":%d,\"nightExpCap\":%d,"
    "\"nightAecThr\":%d,\"nightGainCap\":%d,"
    "\"apiFallbackMs\":%d,\"triggerMm\":%d,\"cooldownMs\":%d,\"autoBaseAec\":%d,\"hdrSteps\":%s}",
    aecMax, aecLow, dayLuxThreshold, nightLuxThreshold,
    dayGainCap, dayExposureDiv, dayMinExposure, nightExposureCap,
    nightAecThreshold, nightGainCap,
    apiFallbackMs, burstTriggerMm, burstCooldownMs, autoBaseAec, hdrBuf);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

static esp_err_t setsettings_handler(httpd_req_t *req) {
  char buf[256];
  int len = httpd_req_get_url_query_len(req) + 1;
  if (len <= 1 || len > (int)sizeof(buf)) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_req_get_url_query_str(req, buf, sizeof(buf));
  char val[16];
  if (httpd_query_key_value(buf, "aecMax", val, sizeof(val)) == ESP_OK) aecMax = atoi(val);
  if (httpd_query_key_value(buf, "aecLow", val, sizeof(val)) == ESP_OK) aecLow = atoi(val);
  if (httpd_query_key_value(buf, "dayLux", val, sizeof(val)) == ESP_OK) dayLuxThreshold = atoi(val);
  if (httpd_query_key_value(buf, "nightLux", val, sizeof(val)) == ESP_OK) nightLuxThreshold = atoi(val);
  if (httpd_query_key_value(buf, "dayGainCap", val, sizeof(val)) == ESP_OK) dayGainCap = atoi(val);
  if (httpd_query_key_value(buf, "dayExpDiv", val, sizeof(val)) == ESP_OK) dayExposureDiv = atoi(val);
  if (httpd_query_key_value(buf, "dayMinExp", val, sizeof(val)) == ESP_OK) dayMinExposure = atoi(val);
  if (httpd_query_key_value(buf, "nightExpCap", val, sizeof(val)) == ESP_OK) nightExposureCap = atoi(val);
  if (httpd_query_key_value(buf, "nightAecThr", val, sizeof(val)) == ESP_OK) nightAecThreshold = atoi(val);
  if (httpd_query_key_value(buf, "nightGainCap", val, sizeof(val)) == ESP_OK) nightGainCap = atoi(val);
  if (httpd_query_key_value(buf, "apiFallbackMs", val, sizeof(val)) == ESP_OK) apiFallbackMs = atoi(val);
  if (httpd_query_key_value(buf, "triggerMm", val, sizeof(val)) == ESP_OK) burstTriggerMm = atoi(val);
  if (httpd_query_key_value(buf, "cooldownMs", val, sizeof(val)) == ESP_OK) burstCooldownMs = atoi(val);
  // HDR steps: hdr0g=gain&hdr0e=aec ... hdr9g=gain&hdr9e=aec
  for (int i = 0; i < HDR_STEP_COUNT; i++) {
    char gKey[8], eKey[8];
    snprintf(gKey, sizeof(gKey), "hdr%dg", i);
    snprintf(eKey, sizeof(eKey), "hdr%de", i);
    if (httpd_query_key_value(buf, gKey, val, sizeof(val)) == ESP_OK) hdrSteps[i].gain = atoi(val);
    if (httpd_query_key_value(buf, eKey, val, sizeof(val)) == ESP_OK) hdrSteps[i].aec = atoi(val);
  }
  Serial.printf("Settings updated: nightAecThr=%d nightExpCap=%d nightGainCap=%d triggerMm=%d apiFallback=%d cooldown=%d autoBaseAec=%d\n",
    nightAecThreshold, nightExposureCap, nightGainCap, burstTriggerMm, apiFallbackMs, burstCooldownMs, autoBaseAec);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

// ===== Settings page HTML =====
const char SETTINGS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32-CAM Settings</title>
  <style>
    * { box-sizing: border-box; }
    body { background: #111; color: #eee; font-family: sans-serif; margin: 0; padding: 12px; max-width: 600px; margin: 0 auto; }
    h1 { font-size: 1.3em; margin: 0 0 4px; }
    a { color: #6af; }
    .section { background: #1a1a2e; padding: 12px; border-radius: 8px; margin: 12px 0; }
    .section h2 { font-size: 1em; margin: 0 0 8px; color: #aaa; }
    .row { display: grid; grid-template-columns: 140px 1fr 60px; gap: 4px 8px; align-items: center; margin: 4px 0; }
    .row label { text-align: right; color: #888; font-size: 0.85em; }
    .row input { width: 100%; background: #222; color: #eee; border: 1px solid #444; border-radius: 4px; padding: 4px; font-family: monospace; }
    .row .unit { color: #666; font-size: 0.8em; }
    .hdr-grid { display: grid; grid-template-columns: 30px 1fr 1fr; gap: 2px 6px; align-items: center; font-size: 0.85em; }
    .hdr-grid .idx { color: #555; text-align: right; }
    .hdr-grid input { width: 100%; background: #222; color: #eee; border: 1px solid #444; border-radius: 3px; padding: 3px; font-family: monospace; text-align: center; }
    .hdr-grid .hdr-head { color: #888; text-align: center; font-size: 0.8em; }
    button { padding: 10px 24px; font-size: 1em; cursor: pointer; border-radius: 6px; border: none; color: #fff; margin: 4px; }
    .save-btn { background: #2a6; }
    .reset-btn { background: #a44; }
    #status { color: #4f4; font-size: 0.9em; margin-top: 8px; }
    .lux-bar { height: 6px; background: #333; border-radius: 3px; margin-top: 4px; position: relative; }
    .lux-fill { height: 100%; border-radius: 3px; background: linear-gradient(90deg, #224, #4af, #ff4); }
  </style>
</head>
<body>
  <h1>⚙️ Imaging Settings</h1>
  <p><a href="/">← Back to Live View</a> | Auto Base AEC: <span id="lux-val">--</span></p>
  <div class="lux-bar"><div class="lux-fill" id="lux-fill" style="width:0%"></div></div>

  <div class="section">
    <h2>� Night / IR Mode</h2>
    <p style="color:#888;font-size:0.8em;margin:0 0 8px">When autoBaseAec &gt; threshold, IR is active. Underexpose to avoid blowing out the cat face.</p>
    <div class="row"><label>Auto Base AEC</label><span id="autoAecLive" style="color:#4af;font-family:monospace">--</span><span class="unit">(live)</span></div>
    <div class="row"><label>Night AEC Thr</label><input type="number" id="nightAecThr"><span class="unit">autoBaseAec &gt; this = night</span></div>
    <div class="row"><label>Night Exp Cap</label><input type="number" id="nightExpCap"><span class="unit">max AEC in night</span></div>
    <div class="row"><label>Night Gain Cap</label><input type="number" id="nightGainCap"><span class="unit">max gain in night</span></div>
  </div>

  <div class="section">
    <h2>📷 Exposure Brackets</h2>
    <div class="row"><label>AEC Max</label><input type="number" id="aecMax"><span class="unit">lines</span></div>
    <div class="row"><label>AEC Low</label><input type="number" id="aecLow"><span class="unit">lines</span></div>
  </div>

  <div class="section">
    <h2>🎯 HDR Steps (10 frames)</h2>
    <div class="hdr-grid">
      <span></span><span class="hdr-head">Gain</span><span class="hdr-head">Exposure</span>
    </div>
    <div class="hdr-grid" id="hdr-grid"></div>
  </div>

  <div class="section">
    <h2>⚡ Trigger & Timing</h2>
    <div class="row"><label>Trigger Distance</label><input type="number" id="triggerMm"><span class="unit">mm</span></div>
    <div class="row"><label>API Fallback</label><input type="number" id="apiFallbackMs"><span class="unit">ms</span></div>
    <div class="row"><label>Burst Cooldown</label><input type="number" id="cooldownMs"><span class="unit">ms</span></div>
  </div>

  <div>
    <button class="save-btn" onclick="saveSettings()">💾 Save</button>
    <button class="reset-btn" onclick="loadSettings()">↺ Reload</button>
  </div>
  <div id="status"></div>

  <script>
    const fields = ['aecMax','aecLow','nightAecThr','nightExpCap','nightGainCap','apiFallbackMs','triggerMm','cooldownMs'];
    async function loadSettings() {
      const r = await fetch('/getsettings');
      const s = await r.json();
      for (const f of fields) {
        const el = document.getElementById(f);
        if (el && s[f] !== undefined) el.value = s[f];
      }
      if (s.autoBaseAec !== undefined) {
        const el = document.getElementById('autoAecLive');
        if (el) {
          el.textContent = s.autoBaseAec;
          el.style.color = s.autoBaseAec > (s.nightAecThr || 200) ? '#f84' : '#4f4';
        }
      }
      const grid = document.getElementById('hdr-grid');
      grid.innerHTML = '';
      if (s.hdrSteps) {
        for (let i = 0; i < s.hdrSteps.length; i++) {
          grid.innerHTML += '<span class="idx">' + i + '</span>' +
            '<input type="number" id="hdr' + i + 'g" value="' + s.hdrSteps[i][0] + '">' +
            '<input type="number" id="hdr' + i + 'e" value="' + s.hdrSteps[i][1] + '">';
        }
      }
      document.getElementById('status').textContent = 'Loaded ✓';
    }
    async function saveSettings() {
      let q = '';
      for (const f of fields) {
        const el = document.getElementById(f);
        if (el) q += (q ? '&' : '') + f + '=' + el.value;
      }
      for (let i = 0; i < 10; i++) {
        const g = document.getElementById('hdr' + i + 'g');
        const e = document.getElementById('hdr' + i + 'e');
        if (g && e) q += '&hdr' + i + 'g=' + g.value + '&hdr' + i + 'e=' + e.value;
      }
      await fetch('/setsettings?' + q);
      document.getElementById('status').textContent = 'Saved ✓ ' + new Date().toLocaleTimeString();
    }
    loadSettings();
    // Live lux display
    setInterval(async () => {
      try {
        const r = await fetch('/stats');
        const s = await r.json();
        document.getElementById('lux-val').textContent = s.lux;
        const pct = Math.min(100, s.lux / 10);
        document.getElementById('lux-fill').style.width = pct + '%';
      } catch(e) {}
    }, 1000);
  </script>
</body>
</html>
)rawliteral";

static esp_err_t settings_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, SETTINGS_HTML, strlen(SETTINGS_HTML));
}

void startUIServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.stack_size = 16384;
  config.max_uri_handlers = 16;

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
    httpd_uri_t settings_uri = { .uri = "/settings", .method = HTTP_GET, .handler = settings_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &settings_uri);
    httpd_uri_t getsettings_uri = { .uri = "/getsettings", .method = HTTP_GET, .handler = getsettings_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &getsettings_uri);
    httpd_uri_t setsettings_uri = { .uri = "/setsettings", .method = HTTP_GET, .handler = setsettings_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &setsettings_uri);
    httpd_uri_t getevents_uri = { .uri = "/getevents", .method = HTTP_GET, .handler = getevents_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &getevents_uri);
    Serial.println("UI server started on port 80");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  Serial.println();

  loadEventLog();

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
        if (rangeStatus == 0 && d >= TOF_MIN_MM && d <= 600) {
          tofDistance = d;  // valid reading
        } else if (rangeStatus == 0 && d < TOF_MIN_MM) {
          tofDistance = -1;  // below minimum range, treat as noise
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

  // ToF trigger with debounce: require 3 consecutive close readings to avoid noise
  static int tofCloseCount = 0;
  if (tofReady && tofDistance >= 0 && tofDistance < burstTriggerMm) {
    tofCloseCount++;
  } else {
    tofCloseCount = 0;
  }
  if (tofCloseCount >= 3 && !burstCapturing && postTriggerRemaining == 0
      && (now - burstCooldown > (unsigned long)burstCooldownMs)) {
    pendingBurstTriggerMs = now;
    postTriggerRemaining = POST_TRIGGER_FRAMES;
    tofCloseCount = 0;
  }

  // === Auto-exposure probe: periodically sample camera's AEC to detect day/night ===
  static unsigned long aecProbeStart = 0;
  static uint8_t aecProbeState = 0; // 0=manual mode, 1=AEC settling
  if (!burstCapturing && postTriggerRemaining == 0) {
    if (aecProbeState == 0 && now - aecProbeStart >= 2000) {
      sensor_t *s = esp_camera_sensor_get();
      if (s) {
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
      }
      aecProbeState = 1;
      aecProbeStart = now;
    }
    else if (aecProbeState == 1 && now - aecProbeStart >= 500) {
      sensor_t *s = esp_camera_sensor_get();
      if (s) {
        s->set_reg(s, 0xFF, 0xFF, 0x01);
        int aec_hi  = s->get_reg(s, 0x45, 0x3F);
        int aec_mid = s->get_reg(s, 0x10, 0xFF);
        int aec_lo  = s->get_reg(s, 0x04, 0x03);
        int readAec = (aec_hi << 10) | (aec_mid << 2) | aec_lo;
        if (readAec >= 4) autoBaseAec = readAec;
        s->set_exposure_ctrl(s, 0);
        s->set_gain_ctrl(s, 0);
      }
      aecProbeState = 0;
      aecProbeStart = now;
    }
  } else if (aecProbeState == 1) {
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
      s->set_exposure_ctrl(s, 0);
      s->set_gain_ctrl(s, 0);
    }
    aecProbeState = 0;
    aecProbeStart = now;
  }

  // === TLS pre-connect: keep connection warm when laptop absent ===
  static unsigned long lastTlsCheck = 0;
  if (!burstCapturing && now - lastTlsCheck >= 10000) {
    lastTlsCheck = now;
    bool laptopHere = (lastLaptopContactMs > 0) &&
                      (now - lastLaptopContactMs < LAPTOP_TIMEOUT_MS);
    if (!laptopHere && !tlsConnected) {
      ensureTlsConnection();
    }
  }

  // Continuously fill ring buffer with JPEG frames
  static unsigned long lastRing = 0;
  if (!burstCapturing && now - lastRing >= 100) { // ~10 fps ring buffer
    lastRing = now;

    // Set exposure based on distance (night mode) or HDR bracket (day mode)
    sensor_t *s = esp_camera_sensor_get();
    int appliedGain = -1, appliedAec = -1;
    if (s && aecProbeState == 0) {
      if (autoBaseAec > nightAecThreshold) {
        // Night/IR mode: distance-based exposure.
        // IR reflection follows inverse-square law: closer = brighter.
        // Scale exposure proportional to distance² relative to trigger distance.
        int gain = nightGainCap;
        int aec = nightExposureCap;
        if (tofDistance >= TOF_MIN_MM && tofDistance <= burstTriggerMm) {
          // d²/triggerDist² — e.g. at 240/480 → 0.25, at 120/480 → 0.0625
          long d = tofDistance;
          long trig = burstTriggerMm;
          aec = (int)(nightExposureCap * d * d / (trig * trig));
          if (aec < 4) aec = 4;
        }
        s->set_agc_gain(s, gain);
        s->set_aec_value(s, aec);
        appliedGain = gain;
        appliedAec = aec;
      } else {
        // Day mode: use HDR bracket cycling
        static int hdrIdx = 0;
        int step = hdrIdx % HDR_STEP_COUNT;
        int gain = hdrSteps[step].gain;
        int aec = (int)((long)autoBaseAec * hdrSteps[step].aec / AEC_LOW_DEFAULT);
        if (aec > 1200) aec = 1200;
        if (aec < 4) aec = 4;
        s->set_agc_gain(s, gain);
        s->set_aec_value(s, aec);
        appliedGain = gain;
        appliedAec = aec;
        hdrIdx++;
      }
    }

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
        ringBuf[ringHead].distanceMm = tofDistance;
        ringBuf[ringHead].gainApplied = (int16_t)appliedGain;
        ringBuf[ringHead].aecApplied = (int16_t)appliedAec;
        ringHead = (ringHead + 1) % RING_SIZE;
        if (ringCount < RING_SIZE) ringCount++;
        // Count down post-trigger frames, freeze when done
        if (postTriggerRemaining > 0) {
          postTriggerRemaining--;
          if (postTriggerRemaining == 0) {
            freezeRingToArchive();
            if (burstArchiveCount > 0) {
              int archIdx = burstArchiveCount - 1;
              // Always use fallback: wait 5s for laptop result, then autonomous
              xTaskCreatePinnedToCore(apiFallbackTask, "apiFallback",
                16384, (void *)(intptr_t)archIdx,
                tskIDLE_PRIORITY + 3, NULL, 0);
            }
          }
        }
      }
      esp_camera_fb_return(fb);
    }
  }
}
