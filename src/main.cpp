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
#include "SD_MMC.h"
#include "secrets.h"

// Blynk IoT (template defines are in secrets.h, must precede this include)
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>

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

// ===== SD card (1-bit SD_MMC on Freenove ESP32-S3 WROOM CAM) =====
#define SD_CLK  39
#define SD_CMD  38
#define SD_D0   40
bool sdReady = false;

// ===== Cat door control (transistor on GPIO 14) =====
// Door is normally OPEN: pin HIGH = open (default), pin LOW = closed.
// Default state on boot is HIGH (door open) — set BEFORE pinMode(OUTPUT) to
// avoid a brief low pulse during init that would slam the door shut.
//
// On a trigger (ToF or fake) we IMMEDIATELY close the door. After API
// analysis completes:
//   - prey detected on any frame  -> stay closed for PREY_LOCKOUT_MS (15 min)
//   - no prey on any frame        -> reopen the door
// During lockout, manual "open" requests are blocked.
#define DOOR_PIN 14
#define PREY_LOCKOUT_MS (15UL * 60UL * 1000UL)  // 15 minutes
// After a no-prey verdict, give the cat a re-try window where the door
// stays open and triggers are ignored (no analysis, no door close). Lets
// the cat back off and try again without the 7-second analysis delay.
#define GREEN_LIGHT_MS (60UL * 1000UL)          // 1 minute
// Rate-limit triggers to prevent the API pipeline from being overwhelmed:
//   - During green-light: NO triggers (cat just passed, no prey)
//   - During lockout:     at most 1 trigger per LOCKOUT_TRIGGER_INTERVAL_MS
// (cat is locked out anyway, repeated analysis is wasteful).
#define LOCKOUT_TRIGGER_INTERVAL_MS (2UL * 60UL * 1000UL)  // 2 minutes
// Number of frames in a burst that must independently flag prey before
// triggering the lockout. Tuned via threshold_analysis.py:
//   N=1 (early-exit): 100% recall, 46% precision (13 false closures)
//   N=2 (this):       82% recall, 50% precision  (9 false closures)
//   N=3:              64% recall, 70% precision  (3 false closures)
// N=2 chosen as best F1 trade-off between missing prey and annoying owner.
#define PREY_FRAMES_THRESHOLD 2
// Max frames to send to API per burst. Tuned via early_stop_analysis.py:
// With the optimal frame priority order [8,7,9,5,6,3,4,2,1,0], checking only
// the first 5 frames yields IDENTICAL recall to checking all 10 (9/11 true
// prey detected) but FEWER false positives (7 vs 9) and HALF the latency
// when no prey is found (5.5s vs 11s).
#define MAX_API_FRAMES 5
volatile bool doorOpen = true;
volatile unsigned long preyLockoutUntilMs = 0;  // millis() value; 0 = no lockout
volatile unsigned long greenLightUntilMs = 0;   // millis() value; 0 = no green light
volatile bool blynkDoorChanged = false;         // flag for Blynk door state update
volatile unsigned long lastBurstTriggerMs = 0;  // millis() of most recent accepted trigger

static inline bool doorLockoutActive() {
  unsigned long now = millis();
  // Handle millis() wraparound: only active if `until` is in future and within window
  return preyLockoutUntilMs != 0 &&
         (long)(preyLockoutUntilMs - now) > 0;
}

static inline bool greenLightActive() {
  unsigned long now = millis();
  return greenLightUntilMs != 0 &&
         (long)(greenLightUntilMs - now) > 0;
}

static void doorOpenNow(const char *reason) {
  digitalWrite(DOOR_PIN, HIGH);
  doorOpen = true;
  blynkDoorChanged = true;
  Serial.printf("Door: OPEN (%s)\n", reason);
}

static void doorCloseNow(const char *reason) {
  digitalWrite(DOOR_PIN, LOW);
  doorOpen = false;
  blynkDoorChanged = true;
  Serial.printf("Door: CLOSED (%s)\n", reason);
}

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
  // Per-frame timing breakdown (up to RING_SIZE entries)
  unsigned long cropMs[RING_SIZE];
  unsigned long b64Ms[RING_SIZE];
  unsigned long tlsMs[RING_SIZE];
  unsigned long postMs[RING_SIZE];
  unsigned long totalMs[RING_SIZE];
  unsigned long apiDoneMs;        // when autonomousApiCheck finished
  char sdPath[48];                 // SD card directory path for this burst
};
BurstArchive burstArchives[BURST_ARCHIVES];
volatile int burstArchiveCount = 0;
volatile int burstGen = 0;  // increments on each new burst
volatile bool burstCapturing = false;
volatile bool postTriggerCapturing = false;  // true after freeze done, capture loop can append
volatile bool otaInProgress = false;
unsigned long burstCooldown = 0;
unsigned long pendingBurstTriggerMs = 0;

// Post-trigger: capture N more frames after trigger before freezing
#define POST_TRIGGER_FRAMES 5
// Burst-shift delay: after a trigger, keep filling the ring buffer for this
// many ms (so the latest frames in the burst are post-trigger frames where
// the cat's body and prey are most visible). Empirical: prey hits peak at
// f08/f09 (head past camera). Adding 200ms shifts so we capture 2 extra
// post-trigger frames overwriting the 2 oldest pre-trigger ones.
#define BURST_SHIFT_MS 200
volatile unsigned long pendingFreezeAtMs = 0;  // 0 = no pending freeze
volatile int frozenAec = -1;                   // exposure to lock during shift
volatile int frozenGain = -1;
int postTriggerRemaining = 0;  // >0 means we're in post-trigger phase

// ===== Persistent event log (NVS) =====
#define MAX_EVENTS 50
struct EventEntry {
  uint32_t uptimeMs;   // millis() when result finalized
  int32_t  epochSec;   // Unix epoch seconds (real clock)
  int16_t  gen;        // burst generation
  int8_t   frameCount; // number of frames in burst
  int8_t   result;     // -1=pending, 0=no prey, 1=prey
  int16_t  distMin;    // min distance mm (-1 = unknown)
  int16_t  distMax;    // max distance mm (-1 = unknown)
  uint8_t  mode;       // 0=laptop, 1=autonomous
  int8_t   trend;      // 0=unknown, 1=entering (far→close), 2=exiting (close→far), 3=passing
}; // 17 bytes per entry
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

// Classify distance trend via linear regression slope on valid readings.
// 1=entering (slope < -15 mm/frame), 2=exiting (slope > +15), 3=flat/passing, 0=unknown
int classifyDistTrend(BurstArchive &archive) {
  // Collect valid readings (>= 30mm) with their frame indices
  int n = 0;
  float sum_i = 0, sum_d = 0, sum_id = 0, sum_ii = 0;
  for (int i = 0; i < archive.count; i++) {
    int d = archive.images[i].distanceMm;
    if (d >= 30) {
      sum_i += i; sum_d += d;
      sum_id += (float)i * d; sum_ii += (float)i * i;
      n++;
    }
  }
  if (n < 3) return 0; // not enough data
  float denom = n * sum_ii - sum_i * sum_i;
  if (denom == 0) return 0;
  float slope = (n * sum_id - sum_i * sum_d) / denom;
  Serial.printf("ToF trend: n=%d slope=%.1f mm/frame\n", n, slope);
  if (slope < -15) return 1; // entering (getting closer)
  if (slope > 15)  return 2; // exiting (moving away)
  return 3; // flat / passing (cat sitting at flap)
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
  time_t nowEpoch;
  time(&nowEpoch);
  e.epochSec = (int32_t)nowEpoch;
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

// ===== Blynk IoT telemetry (event-driven only, no periodic polling) =====
// Virtual pins:
//   V0  = Enter or Exit (Integer: 1 on trigger, 0 when green light expires)
//   V1  = Prey frames (Integer: cumulative prey-positive frames during lockout, 0 after)
//   V2  = Chart — prey events counter (Integer, cumulative all-time)
//   V3  = Datetime (String: detailed event summary)
//   V4  = Lockout timer (Integer: remaining lockout minutes)
//   V14 = Door open (Integer: 1=open, 0=closed; also button input)
volatile int blynkPreyTotal = 0;       // all-time cumulative prey events (for chart)
volatile int blynkLockoutPreyFrames = 0; // prey frames accumulated during current lockout
volatile bool blynkEventPending = false; // flag to push event data
volatile int blynkPendingPrey = 0;
volatile int blynkPendingPreyFrames = 0;
volatile int blynkPendingTotalFrames = 0;
volatile unsigned long blynkPendingLatencyMs = 0;
volatile bool blynkV0Active = false;    // V0 is currently 1 (cat present)
volatile bool blynkLockoutWasActive = false; // track lockout→expired transition

// Blynk button handler: user presses door button on dashboard
BLYNK_WRITE(V14) {
  int val = param.asInt();
  if (val == 1) {
    if (doorLockoutActive()) {
      Serial.println("Blynk: door open requested but lockout active — overriding");
      preyLockoutUntilMs = 0;  // clear lockout
    }
    doorOpenNow("Blynk dashboard");
  } else {
    doorCloseNow("Blynk dashboard");
  }
}

// Reset pulse pins on (re)connect
BLYNK_CONNECTED() {
  Blynk.virtualWrite(V0, blynkV0Active ? 1 : 0);
  Blynk.virtualWrite(V1, blynkLockoutPreyFrames);
  Blynk.virtualWrite(V14, doorOpen ? 1 : 0);
  unsigned long now = millis();
  Blynk.virtualWrite(V4, doorLockoutActive() ?
    (int)((preyLockoutUntilMs - now) / 60000UL) + 1 : 0);
}

// Called from door open/close to update door state pin (1 write per event)
void blynkSendDoorState() {
  if (!Blynk.connected()) return;
  Blynk.virtualWrite(V14, doorOpen ? 1 : 0);
  unsigned long now = millis();
  int lockoutMin = 0;
  if (doorLockoutActive()) {
    lockoutMin = (int)((preyLockoutUntilMs - now) / 60000UL) + 1;
  }
  Blynk.virtualWrite(V4, lockoutMin);
}

// Called on trigger to set V0=1 immediately
void blynkSetTriggerActive() {
  blynkV0Active = true;
  if (Blynk.connected()) Blynk.virtualWrite(V0, 1);
}

// Called from API result path to push event
void blynkPushEvent(int preyDetected, int preyFrames, int totalFrames, unsigned long latencyMs) {
  blynkPendingPrey = preyDetected;
  blynkPendingPreyFrames = preyFrames;
  blynkPendingTotalFrames = totalFrames;
  blynkPendingLatencyMs = latencyMs;
  if (preyDetected) {
    blynkPreyTotal++;
    blynkLockoutPreyFrames += preyFrames;
  }
  blynkEventPending = true;
}

void blynkSendPendingEvent() {
  if (!blynkEventPending) return;
  if (!Blynk.connected()) return;  // keep pending, retry next loop
  blynkEventPending = false;
  Blynk.virtualWrite(V1, blynkLockoutPreyFrames);
  Blynk.virtualWrite(V2, blynkPreyTotal);
  // V3: detailed event summary string
  time_t now;
  time(&now);
  struct tm ti;
  localtime_r(&now, &ti);
  char detailBuf[80];
  if (blynkPendingPrey) {
    snprintf(detailBuf, sizeof(detailBuf),
      "%02d:%02d PREY %d/%d frames %.1fs",
      ti.tm_hour, ti.tm_min,
      blynkPendingPreyFrames, blynkPendingTotalFrames,
      blynkPendingLatencyMs / 1000.0f);
  } else {
    snprintf(detailBuf, sizeof(detailBuf),
      "%02d:%02d Clear %.1fs",
      ti.tm_hour, ti.tm_min,
      blynkPendingLatencyMs / 1000.0f);
  }
  Blynk.virtualWrite(V3, detailBuf);
  Blynk.virtualWrite(V14, doorOpen ? 1 : 0);
  Blynk.virtualWrite(V4, doorLockoutActive() ?
    (int)((preyLockoutUntilMs - millis()) / 60000UL) + 1 : 0);
}

void freezeRingToArchive() {
  burstCapturing = true;
  vTaskDelay(pdMS_TO_TICKS(20));  // Let capture loop finish any in-progress frame write
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
  burstArchives[slot].sdPath[0] = '\0';
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

  // Free remaining ring frames and reset (only if no post-trigger phase pending)
  if (postTriggerRemaining == 0) {
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
  }
  Serial.printf("Burst archived: %d frames (archive %d/%d), postTrigger=%d\n",
    burstArchives[slot].count, burstArchiveCount, BURST_ARCHIVES, postTriggerRemaining);
}

// Save burst archive to SD card (called after API analysis is complete)
void saveBurstToSd(int archIdx) {
  if (!sdReady || archIdx < 0 || archIdx >= burstArchiveCount) return;
  BurstArchive &arch = burstArchives[archIdx];
  if (arch.count == 0) return;

  char dirPath[48];
  time_t now;
  time(&now);
  struct tm ti;
  localtime_r(&now, &ti);
  if (now > 1000000000) {
    snprintf(dirPath, sizeof(dirPath), "/%04d%02d%02d_%02d%02d%02d_gen%d",
      ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec,
      arch.generation);
  } else {
    unsigned long sec = arch.timestamp / 1000;
    snprintf(dirPath, sizeof(dirPath), "/burst_%06lu_gen%d", sec, arch.generation);
  }
  SD_MMC.mkdir(dirPath);
  strlcpy(arch.sdPath, dirPath, sizeof(arch.sdPath));

  for (int i = 0; i < arch.count; i++) {
    if (!arch.images[i].buf) continue;
    char fpath[64];
    unsigned long offsetMs = (arch.images[i].captureMs >= arch.firstFrameMs) ?
      (arch.images[i].captureMs - arch.firstFrameMs) : 0;
    snprintf(fpath, sizeof(fpath), "%s/f%02d_%04lums.jpg", dirPath, i, offsetMs);
    File f = SD_MMC.open(fpath, FILE_WRITE);
    if (f) {
      f.write(arch.images[i].buf, arch.images[i].len);
      f.close();
    }
  }

  // Save metadata JSON (includes API results if available)
  char metaPath[64];
  snprintf(metaPath, sizeof(metaPath), "%s/meta.json", dirPath);
  File mf = SD_MMC.open(metaPath, FILE_WRITE);
  if (mf) {
    mf.printf("{\"gen\":%d,\"frames\":%d,\"triggerMs\":%lu,\"firstMs\":%lu,\"lastMs\":%lu,\"uptimeMs\":%lu,\"epoch\":%ld,"
      "\"apiResult\":%d,\"apiFramesSent\":%d,\"apiCallMs\":%lu,\"apiDoneMs\":%lu,",
      arch.generation, arch.count, arch.triggerMs, arch.firstFrameMs,
      arch.lastFrameMs, arch.timestamp, (long)now,
      arch.apiPreyDetected, arch.apiFramesSent, arch.apiCallMs, arch.apiDoneMs);
    mf.print("\"apiResults\":[");
    for (int i = 0; i < arch.count; i++) {
      if (i > 0) mf.print(",");
      mf.printf("%d", arch.apiResults[i]);
    }
    mf.print("],\"cropMs\":[");
    for (int i = 0; i < arch.count; i++) {
      if (i > 0) mf.print(",");
      mf.printf("%lu", arch.cropMs[i]);
    }
    mf.print("],\"totalMs\":[");
    for (int i = 0; i < arch.count; i++) {
      if (i > 0) mf.print(",");
      mf.printf("%lu", arch.totalMs[i]);
    }
    mf.print("],\"images\":[");
    for (int i = 0; i < arch.count; i++) {
      if (i > 0) mf.print(",");
      unsigned long offsetMs = (arch.images[i].captureMs >= arch.firstFrameMs) ?
        (arch.images[i].captureMs - arch.firstFrameMs) : 0;
      mf.printf("{\"f\":\"f%02d_%04lums.jpg\",\"bytes\":%u,\"dist\":%d,\"gain\":%d,\"aec\":%d,\"ms\":%lu,\"offsetMs\":%lu}",
        i, offsetMs, (unsigned)arch.images[i].len,
        arch.images[i].distanceMm, arch.images[i].gainApplied,
        arch.images[i].aecApplied, arch.images[i].captureMs, offsetMs);
    }
    mf.print("]}");
    mf.close();
  }
  Serial.printf("SD: saved %d frames + meta to %s\n", arch.count, dirPath);
}

// ===== Autonomous prey API call (when laptop absent) =====
// HTTP event handler for collecting response body
static char apiResponseBuf[256];
static int apiResponseLen = 0;
static int lastApiEspErr = 0;
static int lastApiHttpStatus = 0;

// ===== Concurrent API infrastructure =====
// Pool of N persistent TLS clients, each owned by one worker task.
// Frame work items pushed to a FreeRTOS queue; workers pull and process.
// N=3 caused crashes (TLS heap pressure). N=2 is stable and gives ~1.75x speedup.
#define N_API_WORKERS 2

static EventGroupHandle_t apiEventGroup = NULL;
#define API_PREY_BIT   BIT0
#define API_DONE_BIT1  BIT1
#define API_DONE_BIT2  BIT2
#define API_DONE_BIT3  BIT3
#define API_ALL_DONE   (API_DONE_BIT1 | API_DONE_BIT2 | API_DONE_BIT3)

struct ApiTaskParam {
  int frameIdx;
  int archIdx;
  uint8_t *prepBuf;
  size_t prepLen;
  int result;
  unsigned long cropMs;
  unsigned long b64Ms;
  unsigned long tlsMs;
  unsigned long postMs;
  unsigned long totalMs;
  EventBits_t doneBit;
};

// Work item for the concurrent worker pool
struct ApiWorkItem {
  int frameIdx;          // ring index of source frame
  uint8_t *prepBuf;      // preprocessed JPEG (worker frees)
  size_t prepLen;
  unsigned long enqueueMs;
};
struct ApiResultItem {
  int frameIdx;
  int result;            // -1 err, 0 no prey, 1 prey
  unsigned long b64Ms;
  unsigned long tlsMs;
  unsigned long postMs;
  unsigned long totalMs;
};

static QueueHandle_t apiWorkQueue = NULL;     // ApiWorkItem
static QueueHandle_t apiResultQueue = NULL;   // ApiResultItem
static volatile bool apiWorkersRunning = false;

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
  httpApi->setTimeout(10000);  // 10s — keep total burst latency bounded

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
// Output: 384x384 grayscale JPEG, rotated 90 CCW (camera mounted sideways).
#define CROP_X  64    // left margin (64px, MCU-aligned)
#define CROP_Y  48    // top margin  (48px, MCU-aligned)
#define CROP_SZ 384   // output 384x384

#include "jpeg_lossless_crop.h"
#include "jpeg_lossless_rotate.h"
#include "pipeline_tests.h"

static uint8_t *cropJpegForApi(const uint8_t *jpgBuf, size_t jpgLen, size_t *outLen) {
  unsigned long t0 = millis();
  *outLen = 0;

  // Lossless DCT crop + 90 CCW rotate + drop chroma — single pass, no IDCT/DCT.
  // Validated to produce 6/10 hits on prey burst vs 1/10 for crop-only baseline.
  // ~70ms on ESP32-S3 (vs ~324ms for decode-and-re-encode pipeline).
  uint8_t *result = jpeg_lossless_crop_rotate_gray(jpgBuf, jpgLen,
    CROP_X, CROP_Y, CROP_SZ, CROP_SZ, outLen);

  unsigned long t1 = millis();

  if (!result) {
    Serial.println("Crop+Rot: lossless rotate failed");
    return NULL;
  }

  Serial.printf("Crop+Rot: 640x480→384x384 grayscale JPEG %uB→%uB (lossless %lums)\n",
    jpgLen, *outLen, t1 - t0);
  return result;
}

// Timing breakdown for a single API call
struct ApiTiming {
  unsigned long b64Ms;
  unsigned long tlsMs;
  unsigned long postMs;
};

// ===== Per-worker TLS client (one per concurrent worker) =====
// Each worker keeps a persistent WiFiClientSecure + HTTPClient pair. Workers
// use these directly (instead of the shared tlsClient/httpApi).
struct ApiWorker {
  WiFiClientSecure *tls;
  HTTPClient *http;
  bool connected;
  unsigned long connectedAtMs;  // when this TLS session was opened
  TaskHandle_t task;
  int id;
};
static ApiWorker apiWorkers[N_API_WORKERS];

// Force a fresh TLS connection if the existing one is older than this.
// Long-idle TLS connections often get silently dropped by the server but
// the client still thinks they are alive. 5 minutes is a reasonable balance.
#define WORKER_TLS_MAX_AGE_MS (5UL * 60UL * 1000UL)

static bool ensureWorkerTls(ApiWorker *w) {
  unsigned long now = millis();
  bool tooOld = w->connected && (now - w->connectedAtMs > WORKER_TLS_MAX_AGE_MS);
  if (w->connected && !tooOld && w->tls && w->tls->connected()) return true;
  if (tooOld) {
    Serial.printf("API[w%d]: TLS aged out (%lus), reconnecting\n",
                  w->id, (now - w->connectedAtMs) / 1000);
  }
  if (w->http) { w->http->end(); delete w->http; w->http = NULL; }
  if (w->tls)  { w->tls->stop(); delete w->tls;  w->tls  = NULL; }
  w->connected = false;

  w->tls = new WiFiClientSecure();
  if (!w->tls) return false;
  w->tls->setInsecure();

  w->http = new HTTPClient();
  w->http->setReuse(true);
  if (!w->http->begin(*w->tls, PREY_API_URL)) {
    delete w->http; w->http = NULL;
    delete w->tls;  w->tls  = NULL;
    return false;
  }
  char authHeader[128];
  snprintf(authHeader, sizeof(authHeader), "Bearer %s", PREY_API_KEY);
  w->http->addHeader("Content-Type", "application/json");
  w->http->addHeader("Authorization", authHeader);
  w->http->addHeader("Connection", "keep-alive");
  w->http->setTimeout(10000);  // 10s per call — bounds worst-case latency
  w->connected = true;
  w->connectedAtMs = millis();
  return true;
}

// Worker variant: call API using the worker's own TLS client.
// Returns: -1=error, 0=no prey, 1=prey. Frees prepBuf.
static int callPreyApiWithWorker(ApiWorker *w,
                                  uint8_t *prepBuf, size_t prepLen,
                                  ApiTiming *timing) {
  // Base64 encode (PSRAM)
  unsigned long tb0 = millis();
  size_t b64Len = 0;
  mbedtls_base64_encode(NULL, 0, &b64Len, prepBuf, prepLen);
  char *b64Buf = (char *)ps_malloc(b64Len + 1);
  if (!b64Buf) { free(prepBuf); return -1; }
  mbedtls_base64_encode((unsigned char *)b64Buf, b64Len + 1, &b64Len, prepBuf, prepLen);
  b64Buf[b64Len] = 0;
  free(prepBuf);
  size_t jsonLen = b64Len + 32;
  char *jsonBody = (char *)ps_malloc(jsonLen);
  if (!jsonBody) { free(b64Buf); return -1; }
  snprintf(jsonBody, jsonLen, "{\"image_base64\":\"%s\"}", b64Buf);
  free(b64Buf);
  unsigned long tb1 = millis();

  unsigned long tt0 = millis();
  if (!ensureWorkerTls(w)) { free(jsonBody); return -1; }
  unsigned long tt1 = millis();

  unsigned long startMs = millis();
  int httpCode = w->http->POST((uint8_t *)jsonBody, strlen(jsonBody));
  free(jsonBody);
  unsigned long elapsed = millis() - startMs;

  if (timing) {
    timing->b64Ms = tb1 - tb0;
    timing->tlsMs = tt1 - tt0;
    timing->postMs = elapsed;
  }

  if (httpCode <= 0) {
    Serial.printf("API[w%d]: POST failed err=%d (%lums)\n", w->id, httpCode, elapsed);
    w->connected = false;
    return -1;
  }
  String response = w->http->getString();
  if (httpCode != 200) {
    Serial.printf("API[w%d]: HTTP %d in %lums\n", w->id, httpCode, elapsed);
    return -1;
  }
  bool prey = response.indexOf("\"detected\":true") >= 0 ||
              response.indexOf("\"detected\": true") >= 0;
  Serial.printf("API[w%d]: %lums result=%d\n", w->id, elapsed, prey ? 1 : 0);
  return prey ? 1 : 0;
}

// Worker task: drains apiWorkQueue, posts results to apiResultQueue.
static void apiWorkerTask(void *arg) {
  ApiWorker *w = (ApiWorker *)arg;
  Serial.printf("API worker %d started\n", w->id);
  while (apiWorkersRunning) {
    ApiWorkItem item;
    // Wait up to 100ms for work (so we can check apiWorkersRunning)
    if (xQueueReceive(apiWorkQueue, &item, pdMS_TO_TICKS(100)) != pdPASS) {
      continue;
    }
    if (item.frameIdx < 0) {
      // Sentinel: stop signal
      break;
    }
    unsigned long t0 = millis();
    ApiTiming timing = {0, 0, 0};
    int res = callPreyApiWithWorker(w, item.prepBuf, item.prepLen, &timing);
    unsigned long total = millis() - t0;
    ApiResultItem r = {item.frameIdx, res, timing.b64Ms, timing.tlsMs,
                       timing.postMs, total};
    xQueueSend(apiResultQueue, &r, portMAX_DELAY);
  }
  Serial.printf("API worker %d exiting\n", w->id);
  vTaskDelete(NULL);
}

static void startApiWorkers() {
  if (!apiWorkQueue) {
    apiWorkQueue   = xQueueCreate(16, sizeof(ApiWorkItem));
    apiResultQueue = xQueueCreate(16, sizeof(ApiResultItem));
  }
  if (apiWorkersRunning) return;
  apiWorkersRunning = true;
  for (int i = 0; i < N_API_WORKERS; i++) {
    apiWorkers[i].id = i;
    apiWorkers[i].task = NULL;
    char name[16];
    snprintf(name, sizeof(name), "apiw%d", i);
    xTaskCreatePinnedToCore(apiWorkerTask, name, 8192,
                             &apiWorkers[i],
                             tskIDLE_PRIORITY + 2, &apiWorkers[i].task,
                             /*core*/ 0);
  }
}

static void stopApiWorkers() {
  if (!apiWorkersRunning) return;
  apiWorkersRunning = false;
  // Push sentinels so workers wake up and exit
  ApiWorkItem stop = {-1, NULL, 0, 0};
  for (int i = 0; i < N_API_WORKERS; i++) {
    xQueueSend(apiWorkQueue, &stop, pdMS_TO_TICKS(100));
  }
}

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

// Forward decls for the API watchdog (full definitions further below).
extern volatile bool apiAbandonRequested;
extern volatile uint32_t apiAbandonCount;
#define API_TASK_DEADLINE_MS 30000

// Send frames in priority order with concurrent first batch.
// First API_CONCURRENT frames sent in parallel, then sequential if no prey found.
void autonomousApiCheck(int archIdx) {
  if (archIdx < 0 || archIdx >= burstArchiveCount) return;
  BurstArchive &archive = burstArchives[archIdx];
  if (archive.count == 0) return;

  archive.apiCallMs = millis();
  archive.apiPreyDetected = 0;
  apiAbandonRequested = false;  // fresh task, clear any stale abandon flag

  Serial.printf("API: autonomous check, %d frames ready, archive %d (gen %d)\n",
    archive.count, archIdx, archive.generation);

  // Build frame order: prioritize by EMPIRICAL prey-detection hit rate from
  // analysis of 11 confirmed-prey bursts on the production pipeline:
  //   f08=82%  f07=64%  f09=55%  f05=45%  f06=45%  f03=36%  f04=36%
  //   f02=18%  f01=9%   f00=9%
  // Trying high-hit frames first reduces average latency to first prey detection.
  static const int order[RING_SIZE] = {8, 7, 9, 5, 6, 3, 4, 2, 1, 0};
  int orderCount = RING_SIZE;
  int orderIdx = 0;

  // === Phase 2: Concurrent worker pool ===
  // N_API_WORKERS persistent TLS clients run in parallel. We push up to
  // MAX_API_FRAMES preprocessed frames to a queue and collect results.
  // Early-exit when PREY_FRAMES_THRESHOLD prey-flagged frames received.
  startApiWorkers();
  // Drain any stale items from previous runs
  ApiWorkItem dummy;
  while (xQueueReceive(apiResultQueue, &dummy, 0) == pdPASS) {}
  while (xQueueReceive(apiWorkQueue,   &dummy, 0) == pdPASS) {}

  {
  int preySoFar = 0;
  int sentSoFar = 0;
  int receivedSoFar = 0;
  int inFlight = 0;

  // Keep pushing frames as long as we have capacity, frames left, and no
  // threshold met. Receive results between pushes.
  unsigned long apiTaskBeganMs = millis();
  while (true) {
    bool haveCapacity = (inFlight < N_API_WORKERS);
    bool moreToSend  = (sentSoFar < MAX_API_FRAMES) && (orderIdx < orderCount);
    bool thresholdHit = (preySoFar >= PREY_FRAMES_THRESHOLD);
    bool deadlineHit = (millis() - apiTaskBeganMs) > API_TASK_DEADLINE_MS;

    if (thresholdHit) {
      Serial.println("API: threshold reached \u2014 stopping early");
      break;
    }
    if (apiAbandonRequested || deadlineHit) {
      Serial.printf("API: ABANDON (%s) after %lums (sent=%d, recv=%d, prey=%d)\n",
                    apiAbandonRequested ? "watchdog" : "deadline",
                    millis() - apiTaskBeganMs,
                    sentSoFar, receivedSoFar, preySoFar);
      apiAbandonCount++;
      break;
    }

    // Push a new frame if we have capacity and frames to send
    if (haveCapacity && moreToSend) {
      int i = order[orderIdx++];
      if (i >= archive.count || !archive.images[i].buf) continue;

      unsigned long ct0 = millis();
      size_t croppedLen = 0;
      uint8_t *cropped = cropJpegForApi(archive.images[i].buf,
                                         archive.images[i].len, &croppedLen);
      unsigned long cropDt = millis() - ct0;
      if (!cropped) continue;
      archive.cropMs[i] = cropDt;

      ApiWorkItem w = {i, cropped, croppedLen, millis()};
      if (xQueueSend(apiWorkQueue, &w, pdMS_TO_TICKS(100)) == pdPASS) {
        inFlight++;
        sentSoFar++;
        archive.apiFramesSent++;
        Serial.printf("API: queued frame[%d] (inFlight=%d, sent=%d/%d)\n",
                      i, inFlight, sentSoFar, MAX_API_FRAMES);
      } else {
        free(cropped);
        Serial.println("API: queue full, dropping frame");
      }
      continue;  // try to push more before blocking on receive
    }

    // Nothing more to push. If nothing in flight, we're done.
    if (inFlight == 0) break;

    // Block waiting for a result
    ApiResultItem r;
    if (xQueueReceive(apiResultQueue, &r, pdMS_TO_TICKS(2000)) != pdPASS) {
      // Don't error — just loop back so the deadline/abandon check fires.
      // Workers may take up to setTimeout() (10s) per call.
      if ((millis() - apiTaskBeganMs) > API_TASK_DEADLINE_MS) continue;
      Serial.println("API: still waiting for worker result...");
      continue;
    }
    inFlight--;
    receivedSoFar++;
    if (r.frameIdx >= 0 && r.frameIdx < RING_SIZE) {
      archive.apiResults[r.frameIdx] = r.result;
      archive.b64Ms[r.frameIdx]    = r.b64Ms;
      archive.tlsMs[r.frameIdx]    = r.tlsMs;
      archive.postMs[r.frameIdx]   = r.postMs;
      archive.totalMs[r.frameIdx]  = r.totalMs;
    }
    if (r.result == 1) preySoFar++;
    Serial.printf("API: got frame[%d] result=%d (preySoFar=%d/%d, recv=%d, inFlight=%d)\n",
      r.frameIdx, r.result, preySoFar, PREY_FRAMES_THRESHOLD,
      receivedSoFar, inFlight);
  }

  // Clean up any results still in flight (don't leak buffers)
  // The workers themselves free prepBuf inside callPreyApiWithWorker, so we
  // just drain remaining results without acting on them. Bounded wait so a
  // hung worker can't pin this task forever.
  unsigned long drainStart = millis();
  while (inFlight > 0 && (millis() - drainStart) < 5000) {
    ApiResultItem r;
    if (xQueueReceive(apiResultQueue, &r, pdMS_TO_TICKS(1000)) == pdPASS) {
      inFlight--;
      // Optionally record late results (cheap to do)
      if (r.frameIdx >= 0 && r.frameIdx < RING_SIZE && archive.apiResults[r.frameIdx] == -1) {
        archive.apiResults[r.frameIdx] = r.result;
      }
    }
  }
  if (inFlight > 0) {
    Serial.printf("API: %d worker(s) still in flight after drain \u2014 leaving for next loop\n", inFlight);
  }
  }  // end Phase 2 scope

api_done:
  // Count frames flagged as prey (threshold-based decision)
  int preyFrameCount = 0;
  for (int i = 0; i < archive.count; i++) {
    if (archive.apiResults[i] == 1) preyFrameCount++;
  }
  Serial.printf("API: done. %d frames sent, %d flagged prey (threshold=%d)\n",
    archive.apiFramesSent, preyFrameCount, PREY_FRAMES_THRESHOLD);
  archive.apiDoneMs = millis();

  // Door logic: require >= PREY_FRAMES_THRESHOLD flagged frames to lockout.
  bool preyConfirmed = (preyFrameCount >= PREY_FRAMES_THRESHOLD);
  // Keep apiPreyDetected as the threshold-based verdict (used by event log)
  archive.apiPreyDetected = preyConfirmed ? 1 : 0;
  if (preyConfirmed) {
    preyLockoutUntilMs = millis() + PREY_LOCKOUT_MS;
    if (preyLockoutUntilMs == 0) preyLockoutUntilMs = 1;  // avoid sentinel
    greenLightUntilMs = 0;  // clear any pending green light
    char reason[80];
    snprintf(reason, sizeof(reason),
             "prey on %d frames (>= %d) — 15 min lockout",
             preyFrameCount, PREY_FRAMES_THRESHOLD);
    doorCloseNow(reason);
  } else {
    if (preyFrameCount > 0) {
      Serial.printf("Door: prey on %d frame(s) but threshold is %d — opening anyway\n",
                    preyFrameCount, PREY_FRAMES_THRESHOLD);
    }
    if (doorLockoutActive()) {
      Serial.println("Door: stay closed (lockout still active)");
    } else {
      doorOpenNow(preyFrameCount == 0 ? "no prey detected" : "below prey threshold");
    }
    // Start green-light window: cat may retry within GREEN_LIGHT_MS without
    // any door close on trigger. Lets the cat back off + retry without the
    // 7s analysis delay each time.
    greenLightUntilMs = millis() + GREEN_LIGHT_MS;
    if (greenLightUntilMs == 0) greenLightUntilMs = 1;  // avoid sentinel
    Serial.printf("Green light: door stays open on next %lus of triggers\n",
                  GREEN_LIGHT_MS / 1000);
  }

  // Save all frames + meta to SD (deferred from freeze time)
  saveBurstToSd(archIdx);

  // Persist event to NVS
  int dMin = 9999, dMax = -9999;
  for (int i = 0; i < archive.count; i++) {
    int d = archive.images[i].distanceMm;
    if (d >= 0) { if (d < dMin) dMin = d; if (d > dMax) dMax = d; }
  }
  if (dMin > dMax) { dMin = -1; dMax = -1; }
  int trend = classifyDistTrend(archive);
  addEvent(archive.generation, archive.count, archive.apiPreyDetected, dMin, dMax, trend, true);

  // Blynk: push event telemetry (prey result + detail)
  unsigned long latencyMs = archive.apiDoneMs - archive.apiCallMs;
  blynkPushEvent(archive.apiPreyDetected == 1 ? 1 : 0, preyFrameCount, archive.count, latencyMs);
}

// Guard: only one API task at a time (timestamp-based, auto-expires).
// On expiry the watchdog also requests the in-flight task to abandon.
volatile unsigned long apiTaskStartMs = 0;
volatile bool apiAbandonRequested = false;          // signal to in-flight task to stop early
volatile uint32_t apiAbandonCount = 0;              // total abandons since boot
// API_TASK_DEADLINE_MS forward-declared above near autonomousApiCheck
#define API_TASK_HARD_RESET_AFTER 5                 // consecutive abandons before reboot
static inline bool apiTaskBusy() {
  unsigned long start = apiTaskStartMs;
  if (start == 0) return false;
  if ((millis() - start) > (API_TASK_DEADLINE_MS + 15000)) {
    apiTaskStartMs = 0;  // ultimate safety: clear stuck flag after deadline + grace
    Serial.println("API: task flag stuck >45s, force-clearing");
    return false;
  }
  return true;
}

// Force-tear down both the legacy shared TLS client and per-worker TLS clients.
// Used by the watchdog after abandoning an in-flight analysis. The next call
// will reconnect from scratch.
static void resetAllApiConnections(const char *reason) {
  Serial.printf("API: resetting TLS connections (%s)\n", reason);
  tlsConnected = false;
  if (httpApi)   { httpApi->end();   delete httpApi;   httpApi   = NULL; }
  if (tlsClient) { tlsClient->stop(); delete tlsClient; tlsClient = NULL; }
  for (int i = 0; i < N_API_WORKERS; i++) {
    apiWorkers[i].connected = false;
    if (apiWorkers[i].http) { apiWorkers[i].http->end();  delete apiWorkers[i].http; apiWorkers[i].http = NULL; }
    if (apiWorkers[i].tls)  { apiWorkers[i].tls->stop();  delete apiWorkers[i].tls;  apiWorkers[i].tls  = NULL; }
  }
}

// FreeRTOS task wrapper for autonomousApiCheck
static void apiCheckTask(void *param) {
  int archIdx = (int)(intptr_t)param;
  autonomousApiCheck(archIdx);
  if (!apiAbandonRequested) apiAbandonCount = 0;  // healthy completion
  apiTaskStartMs = 0;
  vTaskDelete(NULL);
}

// Fallback task: wait briefly for laptop, then run autonomous immediately
static void apiFallbackTask(void *param) {
  int archIdx = (int)(intptr_t)param;
  unsigned long t0 = millis();
  // Wait up to apiFallbackMs for laptop to process first
  while (millis() - t0 < (unsigned long)apiFallbackMs) {
    if (archIdx < 0 || archIdx >= burstArchiveCount) { apiTaskStartMs = 0; vTaskDelete(NULL); return; }
    if (burstArchives[archIdx].apiPreyDetected != -1) {
      Serial.printf("API fallback: archive %d already processed (prey=%d), skipping\n",
        archIdx, burstArchives[archIdx].apiPreyDetected);
      apiTaskStartMs = 0;
      vTaskDelete(NULL);
      return;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  // Laptop didn't process in time — run autonomous check on ALL frames
  if (archIdx >= 0 && archIdx < burstArchiveCount &&
      burstArchives[archIdx].apiPreyDetected == -1) {
    Serial.printf("API fallback: laptop timeout, running autonomous check on archive %d\n", archIdx);
    autonomousApiCheck(archIdx);
  }
  if (!apiAbandonRequested) apiAbandonCount = 0;  // healthy completion
  apiTaskStartMs = 0;
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
  <div id="door-banner" style="width:100%;max-width:500px;margin:6px 0 10px;padding:14px 16px;border-radius:8px;text-align:center;font-weight:bold;font-size:1.25em;background:#333;color:#888;border:3px solid #555;box-shadow:0 0 8px #000;">
    <span id="door-icon">🚪</span> <span id="door-state">DOOR: --</span>
    <div id="lockout-line" style="display:none;font-size:0.75em;font-weight:normal;margin-top:6px;color:#fc4;">
      Lockout ends in <span id="lockout-time" style="font-family:monospace;font-weight:bold;">--:--</span>
    </div>
  </div>
  <p id="stats">Connecting...</p>
  <p id="sensor-line" style="font-size:1.1em;font-weight:bold;margin:4px 0 8px;">Distance: <span id="dist-val" style="color:#888;">--</span> | AEC: <span id="aec-val" style="color:#888;">--</span></p>
  <p id="mode-line" style="font-size:1em;margin:2px 0 6px;">Mode: <span id="mode-val" style="padding:2px 10px;border-radius:4px;font-weight:bold;">--</span></p>
  <div style="display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin:0 0 8px;">
    <button id="toggle-stream" onclick="toggleStream()" style="padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#a44;color:#fff;">Start Stream</button>
    <button onclick="fetch('/cmd?trigger=1')" style="padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#c80;color:#fff;">Fake Trigger</button>
    <button onclick="fetch('/cmd?door=1')" style="padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#284;color:#fff;">Open Door</button>
    <button onclick="fetch('/cmd?door=0')" style="padding:8px 20px;font-size:1em;cursor:pointer;border-radius:6px;border:none;background:#822;color:#fff;">Close Door</button>
    <a href="/settings" style="padding:8px 16px;font-size:0.9em;border-radius:6px;background:#335;color:#8af;text-decoration:none;display:inline-block;">⚙ Settings</a>
    <a href="/sd" style="padding:8px 16px;font-size:0.9em;border-radius:6px;background:#253;color:#8fa;text-decoration:none;display:inline-block;">💾 SD Card</a>
  </div>
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
    let doorOpenState = null;
    let lockoutEndsAt = 0; // local timestamp (ms); 0 = no lockout

    function renderDoorBanner() {
      const banner = document.getElementById('door-banner');
      const icon   = document.getElementById('door-icon');
      const state  = document.getElementById('door-state');
      const lockLn = document.getElementById('lockout-line');
      const lockT  = document.getElementById('lockout-time');
      if (!banner) return;
      const remainingMs = lockoutEndsAt > 0 ? (lockoutEndsAt - Date.now()) : 0;
      const inLockout = remainingMs > 0;
      if (doorOpenState === null) {
        banner.style.background = '#333'; banner.style.color = '#888'; banner.style.borderColor = '#555';
        icon.textContent = '🚪'; state.textContent = 'DOOR: --';
        lockLn.style.display = 'none';
      } else if (doorOpenState) {
        // Open
        banner.style.background = '#0a3a14'; banner.style.color = '#7fff8f'; banner.style.borderColor = '#4f4';
        banner.style.boxShadow = '0 0 12px #4f4';
        icon.textContent = '🟢'; state.textContent = 'DOOR OPEN';
        lockLn.style.display = 'none';
      } else {
        // Closed
        if (inLockout) {
          banner.style.background = '#3a1a0a'; banner.style.color = '#ffb060'; banner.style.borderColor = '#f84';
          banner.style.boxShadow = '0 0 12px #f84';
          icon.textContent = '🔒'; state.textContent = 'DOOR LOCKED (PREY DETECTED)';
          const totalSec = Math.max(0, Math.ceil(remainingMs / 1000));
          const mm = String(Math.floor(totalSec / 60)).padStart(2, '0');
          const ss = String(totalSec % 60).padStart(2, '0');
          lockT.textContent = mm + ':' + ss;
          lockLn.style.display = 'block';
        } else {
          banner.style.background = '#3a0a0a'; banner.style.color = '#ff7070'; banner.style.borderColor = '#f44';
          banner.style.boxShadow = '0 0 12px #f44';
          icon.textContent = '🔴'; state.textContent = 'DOOR CLOSED';
          lockLn.style.display = 'none';
        }
      }
    }
    // Local 1 Hz tick so the lockout countdown updates smoothly between /stats polls
    setInterval(renderDoorBanner, 1000);

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
        const data = await r.json();
        const events = data.events || data;
        const bootEpoch = data.epoch ? (data.epoch - data.uptimeMs/1000) : null;
        const el = document.getElementById('events-log');
        for (let i = events.length - 1; i >= 0; i--) {
          const e = events[i];
          let timeStr;
          if (bootEpoch && e.t) {
            const d = new Date((bootEpoch + e.t/1000) * 1000);
            timeStr = d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit'});
          } else {
            timeStr = (e.ago / 1000).toFixed(0) + 's ago';
          }
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
          const trendLabels = ['\u2753','\u27A1\uFE0F IN','\u2B05\uFE0F OUT','\u21C6 FLAT'];
          const trendColors = ['#666','#4f4','#f84','#fc4'];
          const tl = e.trend >= 0 && e.trend <= 3 ? e.trend : 0;
          div.innerHTML = '<span style="color:#666">' + timeStr + '</span> ' +
            '<b style="color:#aaa">gen' + e.gen + '</b> ' +
            '<span style="color:#6af">' + e.nf + 'f</span> ' +
            '<span style="color:' + modeColor + '">' + mode + '</span> ' +
            '<span style="color:#4cf">' + distStr + '</span> ' +
            '<span style="color:' + trendColors[tl] + '">' + trendLabels[tl] + '</span> ' +
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
        // Door / lockout indicator
        if (s.doorOpen !== undefined) {
          doorOpenState = !!s.doorOpen;
          // Sync lockout deadline to local clock so we can tick smoothly between polls
          lockoutEndsAt = (s.lockoutMs && s.lockoutMs > 0) ? (Date.now() + s.lockoutMs) : 0;
          renderDoorBanner();
        }
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
  unsigned long nowMs = millis();
  unsigned long lockoutMsRemaining = 0;
  if (preyLockoutUntilMs != 0 && (long)(preyLockoutUntilMs - nowMs) > 0) {
    lockoutMsRemaining = preyLockoutUntilMs - nowMs;
  }
  snprintf(json, sizeof(json),
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u,\"distance\":%d,\"lux\":%u,\"autoAec\":%d,"
    "\"burstArchives\":%d,\"burstGen\":%d,\"burstCounts\":%s,\"apiResults\":%s,\"burstGens\":%s,"
    "\"apiSent\":%s,\"triggerMs\":%s,\"apiDoneMs\":%s,\"distMin\":%s,\"distMax\":%s,"
    "\"laptopPresent\":%s,\"freePsram\":%u,\"uptimeMs\":%lu,"
    "\"doorOpen\":%s,\"lockoutMs\":%lu,"
    "\"apiBusy\":%s,\"apiTaskAgeMs\":%lu,\"apiAbandons\":%u,\"lastTriggerMs\":%lu}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount,
    tofDistance, alsLux, autoBaseAec, burstArchiveCount, burstGen, archBuf,
    apiResBuf, genBuf, sentBuf, trigBuf, apiDoneBuf, distMinBuf, distMaxBuf,
    laptopPresent2 ? "true" : "false", ESP.getFreePsram(), nowMs,
    doorOpen ? "true" : "false", lockoutMsRemaining,
    apiTaskStartMs ? "true" : "false",
    apiTaskStartMs ? (nowMs - apiTaskStartMs) : 0UL,
    (unsigned)apiAbandonCount, lastBurstTriggerMs);
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

  char json[2560];
  // API results per frame
  char apiBuf[80] = "[";
  for (int i = 0; i < archive.count; i++) {
    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%s%d", i > 0 ? "," : "", archive.apiResults[i]);
    strlcat(apiBuf, tmp, sizeof(apiBuf));
  }
  strlcat(apiBuf, "]", sizeof(apiBuf));

  // Per-frame timing arrays (up to apiFramesSent entries)
  char cropBuf[160] = "[", b64Buf[160] = "[", tlsBuf2[160] = "[", postBuf[160] = "[", totBuf[160] = "[";
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

// ===== Pipeline test endpoint =====
// POST /pipetest?pipe=A|B|C|D|E|F|H&iters=N&out=0|1
// Body: raw JPEG bytes (640x480 grayscale expected)
// Response: JSON with timings (out=0) or processed JPEG (out=1, last iter)

// Run pipeline on a dedicated high-stack task to avoid httpd stack overflow
struct PipeTaskArg {
  int variant;
  const uint8_t *jpg;
  size_t jpgLen;
  int iters;
  unsigned long *times_us;
  size_t *out_lens;
  uint8_t **lastOut;
  size_t *lastOutLen;
  TaskHandle_t caller;
};

static void pipeTaskFn(void *arg) {
  PipeTaskArg *a = (PipeTaskArg *)arg;
  uint8_t *lastOut = NULL;
  for (int i = 0; i < a->iters; i++) {
    if (lastOut) { free(lastOut); lastOut = NULL; }
    int64_t t0 = esp_timer_get_time();
    size_t outLen = 0;
    uint8_t *out = pipeline_run(a->variant, a->jpg, a->jpgLen, &outLen);
    int64_t t1 = esp_timer_get_time();
    a->times_us[i] = (unsigned long)(t1 - t0);
    a->out_lens[i] = outLen;
    lastOut = out;
    if (!out) break;
  }
  *a->lastOut = lastOut;
  if (lastOut) *a->lastOutLen = a->out_lens[a->iters - 1];
  xTaskNotifyGive(a->caller);
  vTaskDelete(NULL);
}

static esp_err_t pipetest_handler(httpd_req_t *req) {
  char query[64];
  char val[16];
  int variant = 'A';
  int iters = 1;
  int wantOut = 0;
  int qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen > 1 && qlen <= (int)sizeof(query)) {
    httpd_req_get_url_query_str(req, query, sizeof(query));
    if (httpd_query_key_value(query, "pipe", val, sizeof(val)) == ESP_OK) {
      variant = (int)val[0];
    }
    if (httpd_query_key_value(query, "iters", val, sizeof(val)) == ESP_OK) {
      iters = atoi(val);
      if (iters < 1) iters = 1;
      if (iters > 20) iters = 20;
    }
    if (httpd_query_key_value(query, "out", val, sizeof(val)) == ESP_OK) {
      wantOut = atoi(val);
    }
  }

  // Receive POST body (JPEG)
  int contentLen = req->content_len;
  if (contentLen <= 0 || contentLen > 200 * 1024) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad content length");
    return ESP_FAIL;
  }
  uint8_t *jpgBuf = (uint8_t *)heap_caps_malloc(contentLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!jpgBuf) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no psram");
    return ESP_FAIL;
  }
  int received = 0;
  while (received < contentLen) {
    int r = httpd_req_recv(req, (char *)(jpgBuf + received), contentLen - received);
    if (r <= 0) {
      free(jpgBuf);
      httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "recv fail");
      return ESP_FAIL;
    }
    received += r;
  }

  // Run pipeline iters times
  unsigned long times_us[20] = {0};
  size_t out_lens[20] = {0};
  uint8_t *lastOut = NULL;
  size_t lastOutLen = 0;
  uint32_t freeHeapBefore = ESP.getFreeHeap();
  uint32_t freePsramBefore = ESP.getFreePsram();

  PipeTaskArg pa = {variant, jpgBuf, (size_t)contentLen, iters,
                    times_us, out_lens, &lastOut, &lastOutLen,
                    xTaskGetCurrentTaskHandle()};
  // Use a large stack — esp_jpg_decode + fmt2jpg need significant stack
  TaskHandle_t th = NULL;
  BaseType_t rc = xTaskCreatePinnedToCore(pipeTaskFn, "pipetest", 32768,
                                          &pa, tskIDLE_PRIORITY + 2, &th, 0);
  if (rc != pdPASS) {
    free(jpgBuf);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "task create fail");
    return ESP_FAIL;
  }
  // Wait up to 30s for task to finish
  ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30000));

  if (wantOut && lastOut) {
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char hdr[32];
    snprintf(hdr, sizeof(hdr), "%lu", times_us[iters - 1]);
    httpd_resp_set_hdr(req, "X-Pipe-Time-Us", hdr);
    snprintf(hdr, sizeof(hdr), "%u", (unsigned)lastOutLen);
    httpd_resp_set_hdr(req, "X-Pipe-Out-Len", hdr);
    httpd_resp_send(req, (const char *)lastOut, lastOutLen);
    free(lastOut);
    free(jpgBuf);
    return ESP_OK;
  }

  if (lastOut) free(lastOut);

  // Build JSON response
  char json[1024];
  int n = snprintf(json, sizeof(json),
    "{\"pipe\":\"%c\",\"iters\":%d,\"input_len\":%d,"
    "\"free_heap_before\":%u,\"free_psram_before\":%u,"
    "\"times_us\":[",
    variant, iters, contentLen, freeHeapBefore, freePsramBefore);
  for (int i = 0; i < iters; i++) {
    n += snprintf(json + n, sizeof(json) - n, "%s%lu", i ? "," : "", times_us[i]);
  }
  n += snprintf(json + n, sizeof(json) - n, "],\"out_lens\":[");
  for (int i = 0; i < iters; i++) {
    n += snprintf(json + n, sizeof(json) - n, "%s%u", i ? "," : "", (unsigned)out_lens[i]);
  }
  n += snprintf(json + n, sizeof(json) - n, "]}");

  free(jpgBuf);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, n);
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
  } else if (httpd_query_key_value(buf, "door", val, sizeof(val)) == ESP_OK) {
    // /cmd?door=0 -> close, /cmd?door=1 -> open (manually overrides lockout)
    int v = atoi(val);
    if (v) {
      if (doorLockoutActive()) {
        unsigned long remain = (preyLockoutUntilMs - millis()) / 1000;
        Serial.printf("Door: manual OPEN overriding lockout (%lus remaining)\n", remain);
        preyLockoutUntilMs = 0;  // clear lockout on manual override
      }
      doorOpenNow("manual");
    } else {
      doorCloseNow("manual");
    }
  } else if (httpd_query_key_value(buf, "trigger", val, sizeof(val)) == ESP_OK) {
    // Fake trigger: simulate ToF detection for testing.
    // Honor the same rate limit as the ToF path.
    unsigned long nowMs = millis();
    bool busy   = burstCapturing || postTriggerRemaining != 0 || pendingFreezeAtMs != 0;
    bool greenL = greenLightActive();
    bool apiBz  = apiTaskBusy();
    bool lockRL = doorLockoutActive() && lastBurstTriggerMs != 0 &&
                  (nowMs - lastBurstTriggerMs) < LOCKOUT_TRIGGER_INTERVAL_MS;
    if (busy || greenL || apiBz || lockRL) {
      Serial.printf("Fake trigger REJECTED: busy=%d green=%d api=%d lockoutRL=%d\n",
                    busy, greenL, apiBz, lockRL);
      httpd_resp_set_type(req, "text/plain");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      const char *why = busy ? "BUSY" : greenL ? "GREEN_LIGHT" :
                        apiBz ? "API_BUSY" : "LOCKOUT_RATE_LIMIT";
      return httpd_resp_send(req, why, strlen(why));
    }
    doorCloseNow("trigger (fake)");
    blynkSetTriggerActive();
    lastBurstTriggerMs = nowMs;
    pendingBurstTriggerMs = nowMs;
    // Lock current exposure for the shift window so post-trigger frames
    // don't auto-brighten when ToF object leaves range.
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
      frozenAec  = s->status.aec_value;
      frozenGain = s->status.agc_gain;
    }
    // Schedule freeze BURST_SHIFT_MS in the future. The capture loop will
    // continue filling the ring during this window (overwriting the 2
    // oldest entries with 2 new post-trigger frames). The capture loop
    // owns the actual freeze + apiCheckTask launch (see below).
    pendingFreezeAtMs = millis() + BURST_SHIFT_MS;
    Serial.printf("Fake trigger: deferring freeze by %d ms (locked aec=%d gain=%d)\n",
                  BURST_SHIFT_MS, frozenAec, frozenGain);
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
  } else if (httpd_query_key_value(buf, "formatsd", val, sizeof(val)) == ESP_OK) {
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (!sdReady) {
      return httpd_resp_send(req, "SD not mounted", 14);
    }
    // Remove all files recursively
    File root = SD_MMC.open("/");
    File entry = root.openNextFile();
    while (entry) {
      if (!entry.isDirectory()) {
        SD_MMC.remove(entry.path());
      } else {
        // Remove files in subdirectory then the dir
        File sub = SD_MMC.open(entry.path());
        File sf = sub.openNextFile();
        while (sf) {
          SD_MMC.remove(sf.path());
          sf = sub.openNextFile();
        }
        sub.close();
        SD_MMC.rmdir(entry.path());
      }
      entry = root.openNextFile();
    }
    root.close();
    Serial.println("SD card formatted (all files removed)");
    return httpd_resp_send(req, "SD formatted", 12);
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

// ===== SD card browser HTML page =====
static esp_err_t sdbrowser_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  const char *html = R"rawhtml(
<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>SD Card Browser</title>
<style>
body{font-family:monospace;margin:1em;background:#1a1a2e;color:#e0e0e0;font-size:14px}
h1{color:#0f9;font-size:1.2em;margin:0 0 4px}
.info{color:#888;margin:0 0 8px;font-size:0.85em}
.act{margin-bottom:12px}
.act button{background:#0f9;color:#000;border:none;padding:5px 12px;border-radius:3px;cursor:pointer;margin-right:6px;font-family:monospace}
.act button.danger{background:#f44}
table{width:100%;border-collapse:collapse;font-size:0.9em}
th{text-align:left;border-bottom:1px solid #444;padding:4px 8px;color:#8af}
td{padding:3px 8px;border-bottom:1px solid #222}
tr:hover{background:#222}
a{color:#4cf;text-decoration:none}
a:hover{text-decoration:underline}
.dl-btn{color:#0f9;cursor:pointer;margin-left:6px}
.dl-btn:hover{color:#4f4}
.folder-row td{color:#adf}
.meta{color:#888;font-size:0.8em}
#progress{display:none;margin:8px 0;padding:6px;background:#16213e;border-radius:4px;font-size:0.85em}
</style></head><body>
<h1>SD Card Browser</h1>
<div class='info' id='info'>Loading...</div>
<div class='act'>
<button onclick='loadAll()'>Refresh</button>
<button onclick='downloadAll()'>Download All</button>
</div>
<div id='progress'></div>
<div id='content'></div>
<script>
const B='http://'+location.hostname;
async function loadAll(){
  let r=await fetch(B+'/sdinfo');let info=await r.json();
  document.getElementById('info').textContent=info.ok?`${info.type} | ${info.totalMB}MB total | ${info.usedMB}MB used | ${info.freeMB}MB free`:'SD not mounted';
  r=await fetch(B+'/sdlist');let d=await r.json();
  if(!d.ok){document.getElementById('content').innerHTML='<p>SD not available</p>';return;}
  let folders={};
  d.files.forEach(f=>{
    let parts=f.split('/');
    let dir=parts.length>1?parts.slice(0,-1).join('/'):'(root)';
    if(!folders[dir])folders[dir]=[];
    folders[dir].push(parts[parts.length-1]);
  });
  let html='<table><tr><th></th><th>Folder</th><th>Files</th><th>Date/Time</th><th></th></tr>';
  Object.keys(folders).sort().reverse().forEach(dir=>{
    let timeStr=dir;
    let m=dir.match(/(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})_gen(\d+)/);
    if(m){timeStr=`${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}:${m[6]} UTC  gen${m[7]}`;}
    let m2=dir.match(/burst_(\d+)_gen(\d+)/);
    if(m2){let s=parseInt(m2[1]);let h=Math.floor(s/3600);let mn=Math.floor((s%3600)/60);let sc=s%60;timeStr=`${h}h ${mn}m ${sc}s uptime  gen${m2[2]}`;}
    let n=folders[dir].length;
    let jpgs=folders[dir].filter(f=>f.endsWith('.jpg')).length;
    let hasMeta=folders[dir].includes('meta.json');
    html+=`<tr class='folder-row'><td>📁</td><td><a href='#' onclick='toggleFolder("${dir}");return false'>${dir}</a></td><td>${jpgs} img${hasMeta?' +meta':''}</td><td>${timeStr}</td><td><span class='dl-btn' onclick='downloadFolder("${dir}",[${folders[dir].map(f=>'"'+f+'"').join(',')}])'>⬇ Download</span></td></tr>`;
    html+=`<tr id='detail_${dir.replace(/[^a-z0-9]/gi,'_')}' style='display:none'><td></td><td colspan='4'><table>`;
    folders[dir].forEach(f=>{
      let url=B+'/sdget?f='+encodeURIComponent(dir+'/'+f);
      html+=`<tr><td><a href='${url}' target='_blank'>${f}</a></td></tr>`;
    });
    html+='</table></td></tr>';
  });
  html+='</table>';
  document.getElementById('content').innerHTML=html||'<p>No files</p>';
}
function toggleFolder(dir){let el=document.getElementById('detail_'+dir.replace(/[^a-z0-9]/gi,'_'));if(el)el.style.display=el.style.display==='none'?'':'none';}
async function downloadFolder(dir,files){
  let prog=document.getElementById('progress');prog.style.display='block';
  prog.textContent='Downloading '+dir+'...';
  for(let i=0;i<files.length;i++){
    prog.textContent=`Downloading ${dir}: ${i+1}/${files.length}`;
    let url=B+'/sdget?f='+encodeURIComponent(dir+'/'+files[i]);
    let r=await fetch(url);let blob=await r.blob();
    let a=document.createElement('a');a.href=URL.createObjectURL(blob);
    a.download=dir+'_'+files[i];a.click();URL.revokeObjectURL(a.href);
    await new Promise(r=>setTimeout(r,200));
  }
  prog.textContent='Done!';setTimeout(()=>{prog.style.display='none';},2000);
}
async function downloadAll(){
  let r=await fetch(B+'/sdlist');let d=await r.json();
  if(!d.ok||!d.files.length){alert('No files');return;}
  let prog=document.getElementById('progress');prog.style.display='block';
  for(let i=0;i<d.files.length;i++){
    prog.textContent=`Downloading all: ${i+1}/${d.files.length}`;
    let url=B+'/sdget?f='+encodeURIComponent(d.files[i]);
    let r2=await fetch(url);let blob=await r2.blob();
    let fname=d.files[i].replace(/\//g,'_');
    let a=document.createElement('a');a.href=URL.createObjectURL(blob);
    a.download=fname;a.click();URL.revokeObjectURL(a.href);
    await new Promise(r=>setTimeout(r,200));
  }
  prog.textContent='All done!';setTimeout(()=>{prog.style.display='none';},2000);
}
loadAll();
</script></body></html>
)rawhtml";
  httpd_resp_sendstr(req, html);
  return ESP_OK;
}

// ===== SD card file listing =====
static esp_err_t sdlist_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (!sdReady) {
    httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SD not mounted\"}");
    return ESP_OK;
  }

  // Optional ?dir=foldername returns just files in that folder.
  // No dir = list all (may be large).
  char dir_q[64] = {0};
  int qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen > 1 && qlen <= (int)sizeof(dir_q) + 16) {
    char qbuf[80];
    httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf));
    httpd_query_key_value(qbuf, "dir", dir_q, sizeof(dir_q));
  }

  httpd_resp_send_chunk(req, "{\"ok\":true,\"files\":[", -1);
  char entry_buf[128];
  bool first = true;

  if (dir_q[0]) {
    // List a single subfolder
    char path[80];
    snprintf(path, sizeof(path), "/%s", dir_q);
    File sub = SD_MMC.open(path);
    if (sub && sub.isDirectory()) {
      File sf = sub.openNextFile();
      while (sf) {
        if (!sf.isDirectory()) {
          int len = snprintf(entry_buf, sizeof(entry_buf), "%s\"%s/%s\"",
            first ? "" : ",", dir_q, sf.name());
          httpd_resp_send_chunk(req, entry_buf, len);
          first = false;
        }
        sf = sub.openNextFile();
      }
    }
    if (sub) sub.close();
  } else {
    File root = SD_MMC.open("/");
    int dirCount = 0;
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      if (entry.isDirectory()) {
        const char *eName = entry.name();
        char ePath[64];
        snprintf(ePath, sizeof(ePath), "/%s", eName);
        entry.close();
        File sub = SD_MMC.open(ePath);
        if (sub) {
          while (true) {
            File sf = sub.openNextFile();
            if (!sf) break;
            if (!sf.isDirectory()) {
              int len = snprintf(entry_buf, sizeof(entry_buf), "%s\"%s/%s\"",
                first ? "" : ",", eName, sf.name());
              httpd_resp_send_chunk(req, entry_buf, len);
              first = false;
            }
            sf.close();
          }
          sub.close();
        }
        dirCount++;
        if (dirCount % 10 == 0) vTaskDelay(1);  // yield to prevent WDT
      } else {
        int len = snprintf(entry_buf, sizeof(entry_buf), "%s\"%s\"",
          first ? "" : ",", entry.name());
        httpd_resp_send_chunk(req, entry_buf, len);
        first = false;
        entry.close();
      }
    }
    root.close();
  }

  httpd_resp_send_chunk(req, "]}", -1);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// Lightweight: return only top-level folder names (no file enumeration)
static esp_err_t sdfolders_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (!sdReady) {
    httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SD not mounted\"}");
    return ESP_OK;
  }
  httpd_resp_send_chunk(req, "{\"ok\":true,\"folders\":[", -1);
  File root = SD_MMC.open("/");
  bool first = true;
  char buf[80];
  int count = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (entry.isDirectory()) {
      int len = snprintf(buf, sizeof(buf), "%s\"%s\"", first ? "" : ",", entry.name());
      httpd_resp_send_chunk(req, buf, len);
      first = false;
      count++;
    }
    entry.close();
    if (count % 20 == 0) vTaskDelay(1);  // yield to prevent WDT
  }
  root.close();
  httpd_resp_send_chunk(req, "]}", -1);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ===== SD card file download =====
static esp_err_t sdget_handler(httpd_req_t *req) {
  if (!sdReady) { httpd_resp_send_404(req); return ESP_FAIL; }
  char query[128];
  int qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1 || qlen > (int)sizeof(query)) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_req_get_url_query_str(req, query, sizeof(query));
  char filepath[96];
  if (httpd_query_key_value(query, "f", filepath, sizeof(filepath)) != ESP_OK) {
    httpd_resp_send_404(req); return ESP_FAIL;
  }
  // Ensure path starts with /
  char fullpath[128];
  if (filepath[0] == '/') {
    snprintf(fullpath, sizeof(fullpath), "%s", filepath);
  } else {
    snprintf(fullpath, sizeof(fullpath), "/%s", filepath);
  }
  File f = SD_MMC.open(fullpath);
  if (!f || f.isDirectory()) { httpd_resp_send_404(req); return ESP_FAIL; }
  // Set content type based on extension
  const char *ct = "application/octet-stream";
  if (strstr(fullpath, ".jpg") || strstr(fullpath, ".jpeg")) ct = "image/jpeg";
  else if (strstr(fullpath, ".json")) ct = "application/json";
  httpd_resp_set_type(req, ct);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  uint8_t buf[4096];
  size_t bytesRead;
  while ((bytesRead = f.read(buf, sizeof(buf))) > 0) {
    httpd_resp_send_chunk(req, (const char *)buf, bytesRead);
  }
  f.close();
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// ===== Persistent events API =====
static esp_err_t sdinfo_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char json[256];
  if (!sdReady) {
    snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"SD card not mounted\"}");
  } else {
    uint64_t total = SD_MMC.totalBytes();
    uint64_t used = SD_MMC.usedBytes();
    uint64_t cardSize = SD_MMC.cardSize();
    uint8_t type = SD_MMC.cardType();
    const char *typeStr = (type == 1) ? "MMC" : (type == 2) ? "SDSC" : (type == 3) ? "SDHC" : "UNKNOWN";
    snprintf(json, sizeof(json),
      "{\"ok\":true,\"type\":\"%s\",\"cardMB\":%llu,\"totalMB\":%llu,\"usedMB\":%llu,\"freeMB\":%llu}",
      typeStr, cardSize / (1024*1024), total / (1024*1024), used / (1024*1024), (total - used) / (1024*1024));
  }
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

static esp_err_t getevents_handler(httpd_req_t *req) {
  // Stream JSON array of persisted events
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  // Send object with epoch for real-time calculation
  char hdr[96];
  time_t nowEpoch;
  time(&nowEpoch);
  unsigned long nowMs = millis();
  int hdrLen = snprintf(hdr, sizeof(hdr), "{\"epoch\":%ld,\"uptimeMs\":%lu,\"events\":[", (long)nowEpoch, nowMs);
  httpd_resp_send_chunk(req, hdr, hdrLen);
  char buf[160];
  for (int i = 0; i < eventCount; i++) {
    EventEntry &e = eventLog[i];
    int len = snprintf(buf, sizeof(buf),
      "%s{\"t\":%lu,\"epoch\":%ld,\"ago\":%lu,\"gen\":%d,\"nf\":%d,\"res\":%d,\"dMin\":%d,\"dMax\":%d,\"mode\":%d,\"trend\":%d}",
      i > 0 ? "," : "",
      e.uptimeMs, (long)e.epochSec, (nowMs > e.uptimeMs) ? (nowMs - e.uptimeMs) : 0,
      e.gen, e.frameCount, e.result, e.distMin, e.distMax, e.mode, e.trend);
    httpd_resp_send_chunk(req, buf, len);
  }
  httpd_resp_send_chunk(req, "]}", 2);
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
  config.max_uri_handlers = 20;

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
    httpd_uri_t pipetest_uri = {
      .uri = "/pipetest",
      .method = HTTP_POST,
      .handler = pipetest_handler,
      .user_ctx = NULL
    };
    httpd_register_uri_handler(ui_httpd, &pipetest_uri);
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
    httpd_uri_t sdinfo_uri = { .uri = "/sdinfo", .method = HTTP_GET, .handler = sdinfo_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &sdinfo_uri);
    httpd_uri_t sdlist_uri = { .uri = "/sdlist", .method = HTTP_GET, .handler = sdlist_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &sdlist_uri);
    httpd_uri_t sdfolders_uri = { .uri = "/sdfolders", .method = HTTP_GET, .handler = sdfolders_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &sdfolders_uri);
    httpd_uri_t sdget_uri = { .uri = "/sdget", .method = HTTP_GET, .handler = sdget_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &sdget_uri);
    httpd_uri_t sdbrowser_uri = { .uri = "/sd", .method = HTTP_GET, .handler = sdbrowser_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &sdbrowser_uri);
    Serial.println("UI server started on port 80");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  Serial.println();

  // Initialize door control IMMEDIATELY: drive HIGH (open) before pinMode
  // so the brief glitch at boot doesn't close the door.
  digitalWrite(DOOR_PIN, HIGH);
  pinMode(DOOR_PIN, OUTPUT);
  digitalWrite(DOOR_PIN, HIGH);
  doorOpen = true;
  Serial.println("Door: opened (boot default)");

  // Drive on-board WS2812 RGB LED to OFF (GPIO 48) to keep it dark.
  // (The red power LED is hardwired and cannot be disabled in firmware.)
  pinMode(48, OUTPUT);
  digitalWrite(48, LOW);

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

  // SD card init (1-bit mode)
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  if (SD_MMC.begin("/sdcard", true)) { // true = 1-bit mode
    sdReady = true;
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD card ready: %llu MB, type %d\n", cardSize, SD_MMC.cardType());
  } else {
    Serial.println("SD card mount failed");
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

  // NTP time sync (CET/CEST)
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time");
  time_t now = 0;
  for (int i = 0; i < 20 && now < 1000000000; i++) {
    delay(500);
    time(&now);
    Serial.print(".");
  }
  if (now > 1000000000) {
    struct tm ti;
    localtime_r(&now, &ti);
    Serial.printf("\nNTP synced: %04d-%02d-%02d %02d:%02d:%02d CET/CEST\n",
      ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday, ti.tm_hour, ti.tm_min, ti.tm_sec);
  } else {
    Serial.println("\nNTP sync failed, using uptime");
  }

  // FIX 2: Disable WiFi power save mode.
  // Default is WIFI_PS_MIN_MODEM — the radio sleeps between DTIM beacons
  // (~100-300ms), causing periodic TCP stalls visible as stream freezes.
  esp_wifi_set_ps(WIFI_PS_NONE);

  // FIX 7: Set WiFi TX power to near maximum for strongest signal.
  // Value is in 0.25dBm units. 78 = 19.5dBm (near max of 20dBm).
  esp_wifi_set_max_tx_power(78);

  // Blynk IoT: connect using already-established WiFi (no Blynk.begin!)
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
  // Set initial states so dashboard doesn't show stale values
  if (Blynk.connected()) {
    Blynk.virtualWrite(V0, 0);
    Blynk.virtualWrite(V1, 0);
    Blynk.virtualWrite(V14, doorOpen ? 1 : 0);
    Blynk.virtualWrite(V4, 0);
  }
  Serial.println("Blynk: configured (event-driven only)");

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

  // Blynk: run connection handler (non-blocking)
  Blynk.run();
  blynkSendPendingEvent();

  // === API watchdog ============================================
  // If an analysis task has been running >API_TASK_DEADLINE_MS without
  // setting apiTaskStartMs back to 0, ask it to abandon and tear down all
  // TLS connections so the next burst starts fresh. After
  // API_TASK_HARD_RESET_AFTER consecutive abandons (TCP/TLS likely wedged),
  // perform a soft reboot for a clean recovery.
  {
    unsigned long apiStart = apiTaskStartMs;
    static unsigned long lastWatchdogActionMs = 0;
    if (apiStart != 0 && (now - apiStart) > API_TASK_DEADLINE_MS &&
        !apiAbandonRequested && (now - lastWatchdogActionMs) > 10000) {
      Serial.printf("API WATCHDOG: task running %lums > %dms \u2014 requesting abandon\n",
                    now - apiStart, API_TASK_DEADLINE_MS);
      apiAbandonRequested = true;
      lastWatchdogActionMs = now;
      resetAllApiConnections("watchdog abandon");
      if (apiAbandonCount >= API_TASK_HARD_RESET_AFTER) {
        Serial.printf("API WATCHDOG: %u consecutive abandons \u2014 rebooting for clean recovery\n",
                      (unsigned)apiAbandonCount);
        delay(200);
        ESP.restart();
      }
    }
  }
  // =============================================================

  // Blynk: send door state on change
  if (blynkDoorChanged) {
    blynkDoorChanged = false;
    blynkSendDoorState();
  }

  // Blynk: V0 — reset to 0 when green light expires (cat no longer "present")
  if (blynkV0Active && !greenLightActive() && !burstCapturing && !apiTaskBusy() && !doorLockoutActive()) {
    blynkV0Active = false;
    if (Blynk.connected()) Blynk.virtualWrite(V0, 0);
  }

  // Blynk: V1 — reset prey frame count to 0 when lockout expires
  if (blynkLockoutWasActive && !doorLockoutActive()) {
    blynkLockoutWasActive = false;
    if (Blynk.connected()) Blynk.virtualWrite(V1, 0);
    blynkLockoutPreyFrames = 0;
  }
  if (doorLockoutActive()) blynkLockoutWasActive = true;

  // Blynk: update lockout timer once per minute while active
  static unsigned long lastBlynkLockout = 0;
  if (doorLockoutActive() && now - lastBlynkLockout >= 60000) {
    lastBlynkLockout = now;
    if (Blynk.connected()) {
      Blynk.virtualWrite(V4, (int)((preyLockoutUntilMs - now) / 60000UL) + 1);
    }
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
      && pendingFreezeAtMs == 0
      && (now - burstCooldown > (unsigned long)burstCooldownMs)) {
    // Rate-limit gates: skip trigger entirely if green light is active or
    // if API is still chewing on the previous burst, or if we're in lockout
    // and the last trigger was less than LOCKOUT_TRIGGER_INTERVAL_MS ago.
    if (greenLightActive()) {
      static unsigned long lastGreenSkipLog = 0;
      if (now - lastGreenSkipLog > 5000) {
        unsigned long remain = (greenLightUntilMs - now) / 1000;
        Serial.printf("ToF trigger SKIPPED: GREEN LIGHT (%lus left)\n", remain);
        lastGreenSkipLog = now;
      }
      tofCloseCount = 0;  // reset so we don't immediately re-fire
    } else if (apiTaskBusy()) {
      static unsigned long lastApiSkipLog = 0;
      if (now - lastApiSkipLog > 5000) {
        Serial.println("ToF trigger SKIPPED: API analysis still in progress");
        lastApiSkipLog = now;
      }
      tofCloseCount = 0;
    } else if (doorLockoutActive() && lastBurstTriggerMs != 0 &&
               (now - lastBurstTriggerMs) < LOCKOUT_TRIGGER_INTERVAL_MS) {
      static unsigned long lastLockoutSkipLog = 0;
      if (now - lastLockoutSkipLog > 10000) {
        unsigned long sinceLast = (now - lastBurstTriggerMs) / 1000;
        Serial.printf("ToF trigger SKIPPED: lockout rate-limit (last trigger %lus ago, min %lus)\n",
                      sinceLast, LOCKOUT_TRIGGER_INTERVAL_MS / 1000);
        lastLockoutSkipLog = now;
      }
      tofCloseCount = 0;
    } else {
      doorCloseNow("trigger (ToF)");
      blynkSetTriggerActive();
      lastBurstTriggerMs = now;
      pendingBurstTriggerMs = now;
      tofCloseCount = 0;
      sensor_t *s = esp_camera_sensor_get();
      if (s) {
        frozenAec  = s->status.aec_value;
        frozenGain = s->status.agc_gain;
      }
      pendingFreezeAtMs = now + BURST_SHIFT_MS;
      Serial.printf("ToF trigger: deferring freeze by %d ms (locked aec=%d gain=%d)\n",
                    BURST_SHIFT_MS, frozenAec, frozenGain);
      // Note: apiCheckTask is launched after the deferred freeze (see capture loop).
    }
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

    // Set exposure based on distance (night mode) or HDR bracket (day mode).
    // EXCEPTION: during the post-trigger shift window, lock to the trigger-time
    // exposure so frames captured after the cat passes the ToF don't suddenly
    // brighten (which would otherwise blow them out and add motion blur).
    sensor_t *s = esp_camera_sensor_get();
    int appliedGain = -1, appliedAec = -1;
    if (s && pendingFreezeAtMs != 0 && frozenAec >= 0) {
      // Burst-shift: keep exposure constant, but HALVE aec because the cat
      // typically moves directly UNDER the IR illuminator during the shift
      // window (face is closer to IR than ToF distance suggests, so IR
      // reflection is much stronger). Empirical: full locked aec produced
      // overexposed faces in late frames.
      int aec = frozenAec / 2;
      if (aec < 4) aec = 4;
      s->set_agc_gain(s, frozenGain);
      s->set_aec_value(s, aec);
      appliedGain = frozenGain;
      appliedAec = aec;
    } else if (s && aecProbeState == 0) {
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
        // Day mode: use HDR bracket cycling with clamped gain
        // In daylight, high gain blows out the image even with short exposure.
        // Cap gain at 4 to keep usable dynamic range.
        static int hdrIdx = 0;
        int step = hdrIdx % HDR_STEP_COUNT;
        int gain = hdrSteps[step].gain;
        if (gain > 4) gain = 4;  // clamp gain for daylight
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
        // Post-trigger: capture remaining frames, append to archive on each one
        if (postTriggerRemaining > 0) {
          postTriggerRemaining--;
          // Append this frame to the active archive (already created at trigger time)
          if (burstArchiveCount > 0) {
            BurstArchive &arch = burstArchives[burstArchiveCount - 1];
            int prevIdx = (ringHead + RING_SIZE - 1) % RING_SIZE;
            if (arch.count < RING_SIZE && ringBuf[prevIdx].buf) {
              if (!isFrameBlownOut(ringBuf[prevIdx].buf, ringBuf[prevIdx].len)) {
                int slot = arch.count;
                arch.images[slot].buf = ringBuf[prevIdx].buf;
                arch.images[slot].len = ringBuf[prevIdx].len;
                arch.images[slot].captureMs = ringBuf[prevIdx].captureMs;
                arch.images[slot].distanceMm = ringBuf[prevIdx].distanceMm;
                arch.images[slot].gainApplied = ringBuf[prevIdx].gainApplied;
                arch.images[slot].aecApplied = ringBuf[prevIdx].aecApplied;
                if (ringBuf[prevIdx].captureMs > arch.lastFrameMs)
                  arch.lastFrameMs = ringBuf[prevIdx].captureMs;
                ringBuf[prevIdx].buf = NULL;  // ownership transferred
                ringBuf[prevIdx].len = 0;
                arch.count++;
              } else {
                Serial.printf("Burst: post-trigger frame blown out, skipping\n");
                free(ringBuf[prevIdx].buf);
                ringBuf[prevIdx].buf = NULL;
                ringBuf[prevIdx].len = 0;
              }
            }
          }
          if (postTriggerRemaining == 0) {
            postTriggerCapturing = false;
            burstCapturing = false;
            burstCooldown = millis();
            // Clean up remaining ring buffer
            for (int i = 0; i < RING_SIZE; i++) {
              if (ringBuf[i].buf) { free(ringBuf[i].buf); ringBuf[i].buf = NULL; }
              ringBuf[i].len = 0; ringBuf[i].captureMs = 0;
            }
            ringCount = 0; ringHead = 0;
            Serial.printf("Burst: post-trigger complete, total %d frames\n",
              burstArchives[burstArchiveCount - 1].count);
          }
        }
      }
      esp_camera_fb_return(fb);
    }
  }

  // === Deferred freeze: trigger asked for burst-shift, time elapsed → freeze ===
  if (pendingFreezeAtMs != 0 && now >= pendingFreezeAtMs) {
    Serial.printf("Burst-shift complete after %lu ms — freezing ring\n",
                  now - (pendingFreezeAtMs - BURST_SHIFT_MS));
    burstCapturing = true;        // block ring fill during freeze
    pendingFreezeAtMs = 0;        // clear pending
    frozenAec = -1;               // release exposure lock
    frozenGain = -1;
    freezeRingToArchive();
    if (burstArchiveCount > 0) {
      int archIdx = burstArchiveCount - 1;
      if (apiTaskBusy()) {
        Serial.printf("API: skipping archive %d (task already running)\n", archIdx);
        burstArchives[archIdx].apiPreyDetected = 0;
        burstArchives[archIdx].apiDoneMs = millis();
      } else {
        apiTaskStartMs = millis();
        if (apiTaskStartMs == 0) apiTaskStartMs = 1;
        bool laptopHere = (lastLaptopContactMs > 0) &&
                          (now - lastLaptopContactMs < LAPTOP_TIMEOUT_MS);
        if (laptopHere) {
          xTaskCreatePinnedToCore(apiFallbackTask, "apiFallback",
            16384, (void *)(intptr_t)archIdx,
            tskIDLE_PRIORITY + 3, NULL, 0);
        } else {
          xTaskCreatePinnedToCore(apiCheckTask, "apiCheck",
            16384, (void *)(intptr_t)archIdx,
            tskIDLE_PRIORITY + 3, NULL, 0);
        }
      }
    }
  }
}
