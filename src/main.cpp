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
#include "esp_task_wdt.h"

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
// Serializes all SD_MMC access. SD_MMC is NOT thread-safe — concurrent
// reads from multiple httpd worker tasks crash the firmware. All file
// open/read/close in handler code paths must take this first.
SemaphoreHandle_t sdMutex = NULL;

// ===== Cat door control (transistor on GPIO 14) =====
// Door is normally CLOSED (entry side): pin HIGH = open, pin LOW = closed.
// The physical lock is one-way: exits are always free regardless of pin state.
// Default state on boot is LOW (closed) so that crashes / power cycles never
// leave the entry unlocked. Set BEFORE pinMode(OUTPUT) so init glitches don't
// open the door.
//
// On a trigger (ToF or fake) the door is already closed; we still call
// doorCloseNow() to mark the trigger in logs/Blynk. After API analysis:
//   - prey on 1 frame             -> stay closed for PREY_SHORT_LOCKOUT_MS (3 min)
//   - prey on >=2 frames          -> stay closed for PREY_LONG_LOCKOUT_MS (15 min)
//   - no prey on any frame        -> open the door for GREEN_LIGHT_MS, then
//                                    close again automatically
// During lockout, manual "open" requests are blocked.
#define DOOR_PIN 14
#define PREY_SHORT_LOCKOUT_MS (3UL * 60UL * 1000UL)   // 3 minutes (low certainty)
#define PREY_LONG_LOCKOUT_MS  (15UL * 60UL * 1000UL)  // 15 minutes (high certainty)
// After a no-prey verdict, give the cat a re-try window where the door
// stays open and triggers are ignored (no analysis, no door close). Lets
// the cat back off and try again without the 7-second analysis delay.
// When this window expires the door auto-closes again.
#define GREEN_LIGHT_MS (60UL * 1000UL)          // 1 minute
// Rate-limit triggers to prevent the API pipeline from being overwhelmed:
//   - During green-light: NO triggers (cat just passed, no prey)
//   - During lockout:     at most 1 trigger per LOCKOUT_TRIGGER_INTERVAL_MS
// (cat is locked out anyway, repeated analysis is wasteful).
#define LOCKOUT_TRIGGER_INTERVAL_MS (2UL * 60UL * 1000UL)  // 2 minutes
// Number of frames in a burst that must independently flag prey before
// escalating to HIGH-CERTAINTY (long) lockout. Tuned via threshold_analysis.py:
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
volatile bool doorOpen = false;
volatile unsigned long preyLockoutUntilMs = 0;  // millis() value; 0 = no lockout
// Lockout end-time in UNIX seconds, persisted to NVS so a crash mid-lockout
// doesn't drop the lockout on reboot. Restored in setup() if still in future.
volatile int32_t preyLockoutUntilEpoch = 0;
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
// Number of burst metadata structs kept in RAM. Each is ~535B static so
// 40 = ~21KB of internal heap permanently reserved. The web UI typically
// only shows the last 5-10 anyway, and SD has the full history. Cut to 15
// to free ~13KB internal heap for TLS / WiFi / lwIP buffers.
#define BURST_ARCHIVES 15

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
#ifdef PREY_V2_URL
  // Shadow v2 inference (does NOT feed door logic; logged only)
  int8_t   v2Status[RING_SIZE];       // -1=not run, 0=http err, 1=ok
  int8_t   v2Detected[RING_SIZE];     // -1=unknown, 0=false, 1=true
  int8_t   v2CatRecognized[RING_SIZE]; // -1/0/1
  float    v2PreyScore[RING_SIZE];    // 0..1 (NaN if unknown)
  float    v2CatConfidence[RING_SIZE];
  uint16_t v2DecisionMs[RING_SIZE];   // server-reported decision time
  uint16_t v2HttpMs[RING_SIZE];       // wall-clock POST + parse
  int16_t  v2HttpCode[RING_SIZE];     // raw HTTP status (0 if no conn)
  char     v2CatId[RING_SIZE][12];    // "mazge"/"benis"/"unknown"
  char     v2Severity[RING_SIZE][12]; // "none"/"low"/"medium"/"high"
  char     v2DoorAction[RING_SIZE][12]; // "allow"/"block"/...
#endif
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
struct __attribute__((packed)) EventEntry {
  uint32_t uptimeMs;   // millis() when result finalized
  int32_t  epochSec;   // Unix epoch seconds (real clock)
  int16_t  gen;        // burst generation
  int8_t   frameCount; // number of frames in burst
  int8_t   result;     // -1=pending, 0=clear, N=prey frames detected (RING_SIZE max = 10)
  int16_t  distMin;    // min distance mm (-1 = unknown)
  int16_t  distMax;    // max distance mm (-1 = unknown)
  uint8_t  mode;       // 0=laptop (legacy, always 1 now), 1=autonomous
  int8_t   trend;      // 0=unknown, 1=entering (far→close), 2=exiting (close→far), 3=passing
  uint16_t latencyMs;  // API processing latency (apiDoneMs - apiCallMs), 0 = unknown
#ifdef PREY_V2_URL
  // Shadow v2 inference summary per burst (for visibility — not used for door)
  uint16_t v2MaxPreyX1000; // max prey_score across burst, scaled by 1000 (0..1000)
  uint8_t  v2OkFrames;     // number of frames where v2 returned 200 OK
  uint8_t  v2Flags;        // bit0=anyDetected, bit1=anyCatRecognized,
                           // bits2-3=modalCatId (0=unknown,1=mazge,2=benis,3=other)
#endif
}; // packed = exactly 20 bytes (24 with v2)
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
    size_t storedBytes = nvsPrefs.getBytesLength("entries");
    size_t expectedBytes = sizeof(EventEntry) * eventCount;
    if (storedBytes != expectedBytes) {
      // EventEntry layout changed (e.g. v2 shadow fields added). Drop old log
      // to avoid mis-aligned reads. Acceptable: NVS event log is transient.
      Serial.printf("EventLog: stored=%u expected=%u (struct changed?), dropping %d events\n",
                    (unsigned)storedBytes, (unsigned)expectedBytes, eventCount);
      eventCount = 0;
    } else {
      nvsPrefs.getBytes("entries", eventLog, expectedBytes);
    }
  }
  nvsPrefs.end();
  Serial.printf("Loaded %d events from NVS\n", eventCount);
}

// === Persistent prey-lockout state ============================================
// preyLockoutUntilMs is RAM-only (millis-based), so a crash or reboot mid-lockout
// would drop the lockout. We mirror the absolute end-time to NVS as a UNIX
// epoch second every time the lockout is set/cleared, and restore on boot.
// One NVS write per prey trigger + one per reboot — cheap.
void persistLockoutEpoch(int32_t untilEpoch) {
  preyLockoutUntilEpoch = untilEpoch;
  nvsPrefs.begin("state", false);
  nvsPrefs.putInt("lockUntil", untilEpoch);
  nvsPrefs.end();
}

// Restore lockout from NVS. Returns the seconds remaining (>0 = still active),
// or 0 if no lockout pending. Caller is responsible for translating remaining
// seconds into preyLockoutUntilMs once millis() is available and time is synced.
int32_t loadLockoutEpoch() {
  nvsPrefs.begin("state", true);
  int32_t until = nvsPrefs.getInt("lockUntil", 0);
  nvsPrefs.end();
  preyLockoutUntilEpoch = until;
  return until;
}
// ==============================================================================

// Classify burst direction from per-frame ToF readings already captured
// in archive.images[i].distanceMm.
//
// Rule (validated on 155 human-labelled bursts, F1=0.872 for exiting):
//   min_dist  = smallest valid (>=0 mm) reading across the burst
//   first_dist = first valid reading in the burst
//   if min_dist < 180mm AND first_dist < 230mm -> 2 EXITING
//   else if any valid readings                  -> 1 ENTERING
//   else                                        -> 0 UNKNOWN
//
// Encoding kept compatible with the existing event log (uint8 trend field):
//   0 = unknown   1 = entering (far -> close)   2 = exiting (close)   3 = passing (unused)
//
// Calibration: 0/21 prey-positive bursts misclassified as exit
// (see tools/check_exit_rule_safety.py). 90.3% accuracy overall.
//
// Stage 0 of the exit-detector rollout: this is decision-only — the value
// is recorded in the event log + meta.json + web UI but does NOT yet skip
// the API call. Future stages will add the ring buffer for richer signal.
#define DIRECTION_EXIT_MIN_DIST_MM   180
#define DIRECTION_EXIT_FIRST_DIST_MM 230
int classifyDistTrend(BurstArchive &archive) {
  int min_dist = -1;
  int first_dist = -1;
  int n_valid = 0;
  for (int i = 0; i < archive.count; i++) {
    int d = archive.images[i].distanceMm;
    if (d < 0) continue;             // -1 no-target / -2 sensor error
    if (first_dist < 0) first_dist = d;
    if (min_dist < 0 || d < min_dist) min_dist = d;
    n_valid++;
  }
  if (n_valid == 0) return 0;        // unknown
  bool is_exit = (min_dist < DIRECTION_EXIT_MIN_DIST_MM) &&
                 (first_dist < DIRECTION_EXIT_FIRST_DIST_MM);
  Serial.printf("Direction: n_valid=%d min=%dmm first=%dmm -> %s\n",
                n_valid, min_dist, first_dist, is_exit ? "EXIT" : "ENTER");
  return is_exit ? 2 : 1;
}

void addEvent(int gen, int frameCount, int result, int distMin, int distMax, int trend, bool autonomous, time_t epochOverride = 0, uint16_t latencyMs = 0
#ifdef PREY_V2_URL
              , uint16_t v2MaxPreyX1000 = 0, uint8_t v2OkFrames = 0, uint8_t v2Flags = 0
#endif
              ) {
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
  // Prefer the burst dir's actual creation epoch (passed in) so the UI can
  // reconstruct the SD dir name exactly. Falls back to "now" if caller has none.
  e.epochSec = (int32_t)((epochOverride > 1000000000) ? epochOverride : nowEpoch);
  e.gen = (int16_t)gen;
  e.frameCount = (int8_t)frameCount;
  e.result = (int8_t)result;
  e.distMin = (int16_t)distMin;
  e.distMax = (int16_t)distMax;
  e.mode = autonomous ? 1 : 0;
  e.trend = (int8_t)trend;
  e.latencyMs = latencyMs;
#ifdef PREY_V2_URL
  e.v2MaxPreyX1000 = v2MaxPreyX1000;
  e.v2OkFrames     = v2OkFrames;
  e.v2Flags        = v2Flags;
#endif
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
      persistLockoutEpoch(0);
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
#ifdef PREY_V2_URL
  for (int i = 0; i < RING_SIZE; i++) {
    burstArchives[slot].v2Status[i] = -1;
    burstArchives[slot].v2Detected[i] = -1;
    burstArchives[slot].v2CatRecognized[i] = -1;
    burstArchives[slot].v2PreyScore[i] = -1.0f;
    burstArchives[slot].v2CatConfidence[i] = -1.0f;
    burstArchives[slot].v2DecisionMs[i] = 0;
    burstArchives[slot].v2HttpMs[i] = 0;
    burstArchives[slot].v2HttpCode[i] = 0;
    burstArchives[slot].v2CatId[i][0] = '\0';
    burstArchives[slot].v2Severity[i][0] = '\0';
    burstArchives[slot].v2DoorAction[i][0] = '\0';
  }
#endif

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

  // Take SD mutex for the whole write — keeps HTTP /sdget readers from
  // touching the controller mid-write (causes data corruption / crash).
  // Long timeout: writes can take several seconds for 10 frames.
  if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(30000)) != pdTRUE) {
    Serial.println("SD: saveBurstToSd timeout waiting for mutex — skipping");
    return;
  }

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
    // Direction prediction from existing per-frame ToF (Stage 0 of exit detector).
    // Recompute here so it is always up to date with the classifier in firmware.
    int direction = classifyDistTrend(arch);  // 0 unknown, 1 enter, 2 exit
    int dirMin = -1, dirFirst = -1;
    for (int i = 0; i < arch.count; i++) {
      int d = arch.images[i].distanceMm;
      if (d < 0) continue;
      if (dirFirst < 0) dirFirst = d;
      if (dirMin < 0 || d < dirMin) dirMin = d;
    }
    mf.printf("],\"direction\":%d,\"directionMinDist\":%d,\"directionFirstDist\":%d",
              direction, dirMin, dirFirst);
#ifdef PREY_V2_URL
    // Shadow v2 inference results (not used for door logic; for offline A/B).
    mf.print(",\"v2\":{\"server\":\"" PREY_V2_URL "\",\"frames\":[");
    for (int i = 0; i < arch.count; i++) {
      if (i > 0) mf.print(",");
      // Escape strings (only a-z + digits + dash expected, but be safe).
      auto esc = [&](const char *s) {
        mf.print('"');
        for (const char *p = s; *p; ++p) {
          if (*p == '"' || *p == '\\') mf.print('\\');
          mf.print(*p);
        }
        mf.print('"');
      };
      mf.printf("{\"status\":%d,\"http\":%d,\"httpMs\":%u,\"decisionMs\":%u,"
                "\"detected\":%d,\"catRecognized\":%d,"
                "\"preyScore\":%.4f,\"catConfidence\":%.4f,",
                (int)arch.v2Status[i], (int)arch.v2HttpCode[i],
                (unsigned)arch.v2HttpMs[i], (unsigned)arch.v2DecisionMs[i],
                (int)arch.v2Detected[i], (int)arch.v2CatRecognized[i],
                arch.v2PreyScore[i], arch.v2CatConfidence[i]);
      mf.print("\"catId\":");      esc(arch.v2CatId[i]);
      mf.print(",\"severity\":");   esc(arch.v2Severity[i]);
      mf.print(",\"doorAction\":"); esc(arch.v2DoorAction[i]);
      mf.print('}');
    }
    mf.print("]}");
#endif
    mf.print("}");
    mf.close();
  }
  Serial.printf("SD: saved %d frames + meta to %s\n", arch.count, dirPath);
  if (sdMutex) xSemaphoreGive(sdMutex);
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
#ifdef PREY_V2_URL
  BurstArchive *archive; // shadow v2 result destination (NULL = skip v2)
  int archiveGen;        // sanity check (verify archive not recycled)
#endif
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
//
// IMPORTANT: only the worker task itself ever touches w->tls / w->http
// (allocates, calls, frees). Other tasks (loop()'s watchdog, idle-TLS-free
// path, ToF-trigger pre-warm) communicate by setting the volatile request
// flags below. The worker checks them between queue iterations and acts on
// them locally. This avoids a use-after-free race that previously caused
// PANIC crashes when the API watchdog tore down TLS pointers while the
// worker was mid-POST.
struct ApiWorker {
  WiFiClientSecure *tls;
  HTTPClient *http;
  bool connected;
  unsigned long connectedAtMs;     // when this TLS session was opened
  TaskHandle_t task;
  int id;
  volatile bool resetTlsRequested; // set by other tasks; worker tears down on next iteration
  volatile bool prewarmRequested;  // set by other tasks; worker ensures TLS is connected
#ifdef PREY_V2_URL
  // Shadow client to the local LAN v2 inference server (plain HTTP, no TLS).
  WiFiClient *v2_tcp;
  HTTPClient *v2_http;
  bool        v2_connected;
  unsigned long v2_connectedAtMs;
#endif
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

#ifdef PREY_V2_URL
// ===== Shadow v2 inference client =====
//
// Posts the same cropped JPEG to a local LAN server implementing the v2
// contract (doc/inference_api_v2_contract.md). Result is recorded in the
// BurstArchive but NEVER feeds the door logic. Purpose: shadow A/B against
// the production cloud API while we accumulate signal on the new model.
//
// Endpoint expects raw image/jpeg body + X-* headers. Plain HTTP only —
// the server runs on the same LAN and TLS would just burn CPU.

#define V2_HTTP_TIMEOUT_MS 1500   // v2 server p95 ~320ms on the i5 box; 1.5s is generous
#define V2_TLS_MAX_AGE_MS  (10UL * 60UL * 1000UL)

static bool ensureWorkerV2Http(ApiWorker *w) {
  unsigned long now = millis();
  bool tooOld = w->v2_connected && (now - w->v2_connectedAtMs > V2_TLS_MAX_AGE_MS);
  if (w->v2_connected && !tooOld && w->v2_tcp && w->v2_tcp->connected()) return true;
  if (w->v2_http) { w->v2_http->end(); delete w->v2_http; w->v2_http = NULL; }
  if (w->v2_tcp)  { w->v2_tcp->stop(); delete w->v2_tcp;  w->v2_tcp  = NULL; }
  w->v2_connected = false;

  w->v2_tcp = new WiFiClient();
  if (!w->v2_tcp) return false;

  w->v2_http = new HTTPClient();
  w->v2_http->setReuse(true);
  if (!w->v2_http->begin(*w->v2_tcp, PREY_V2_URL)) {
    delete w->v2_http; w->v2_http = NULL;
    delete w->v2_tcp;  w->v2_tcp  = NULL;
    return false;
  }
  w->v2_http->setTimeout(V2_HTTP_TIMEOUT_MS);
  w->v2_connected = true;
  w->v2_connectedAtMs = millis();
  return true;
}

// Tiny JSON helpers (no library dep). Robust enough for the v2 response
// shape, which is generated server-side with separators=(',',':') style.
static bool v2_extract_field(const String &body, const char *key, String &out) {
  String pat = String("\"") + key + "\":";
  int k = body.indexOf(pat);
  if (k < 0) return false;
  int p = k + pat.length();
  while (p < (int)body.length() && body[p] == ' ') p++;
  if (p >= (int)body.length()) return false;
  if (body[p] == '"') {
    int end = body.indexOf('"', p + 1);
    if (end < 0) return false;
    out = body.substring(p + 1, end);
    return true;
  }
  int end = p;
  while (end < (int)body.length() && body[end] != ',' && body[end] != '}'
         && body[end] != '\n') end++;
  out = body.substring(p, end);
  out.trim();
  return out.length() > 0;
}

static float v2_field_float(const String &body, const char *key, float dflt) {
  String s; if (!v2_extract_field(body, key, s)) return dflt;
  return s.toFloat();
}
static int v2_field_int(const String &body, const char *key, int dflt) {
  String s; if (!v2_extract_field(body, key, s)) return dflt;
  return s.toInt();
}
static int v2_field_bool(const String &body, const char *key) {
  // returns -1 unknown, 0 false, 1 true
  String s; if (!v2_extract_field(body, key, s)) return -1;
  if (s == "true") return 1;
  if (s == "false") return 0;
  return -1;
}
static void v2_field_str(const String &body, const char *key,
                         char *out, size_t cap) {
  String s; if (!v2_extract_field(body, key, s)) { out[0] = '\0'; return; }
  strncpy(out, s.c_str(), cap - 1);
  out[cap - 1] = '\0';
}

static void v2_make_request_id(char *out, size_t cap) {
  // RFC4122-shape uuid v4, sourced from esp_random(). Not crypto, fine for idem.
  uint32_t a = esp_random(), b = esp_random(), c = esp_random(), d = esp_random();
  snprintf(out, cap,
           "%08x-%04x-%04x-%04x-%04x%08x",
           a,
           (b >> 16) & 0xffff,
           ((b & 0x0fff) | 0x4000),               // version 4
           ((c >> 16) & 0x3fff) | 0x8000,         // variant 10
           c & 0xffff, d);
}

// Posts a single frame to the v2 endpoint and fills the archive slot.
// jpgBuf is NOT freed by this function (caller still owns it).
static void callPreyV2Shadow(ApiWorker *w, BurstArchive *arch, int frameIdx,
                              const uint8_t *jpgBuf, size_t jpgLen) {
  if (!arch || frameIdx < 0 || frameIdx >= RING_SIZE) return;
  arch->v2Status[frameIdx] = 0;  // default to error until success

  unsigned long t0 = millis();
  if (!ensureWorkerV2Http(w)) {
    arch->v2HttpCode[frameIdx] = 0;
    arch->v2HttpMs[frameIdx]   = (uint16_t)(millis() - t0);
    Serial.printf("V2[w%d] f[%d]: connect failed\n", w->id, frameIdx);
    return;
  }

  char reqId[40];     v2_make_request_id(reqId, sizeof(reqId));
  char tsBuf[24];     snprintf(tsBuf, sizeof(tsBuf), "%lld",
                                (long long)((uint64_t)time(NULL) * 1000ULL));
  char burstId[40];   snprintf(burstId, sizeof(burstId),
                                "%lu_gen%d",
                                (unsigned long)arch->triggerMs, arch->generation);
  char fIdxBuf[8];    snprintf(fIdxBuf, sizeof(fIdxBuf), "%d", frameIdx);

  w->v2_http->addHeader("Content-Type", "image/jpeg");
  w->v2_http->addHeader("X-Contract-Version", "2");
  w->v2_http->addHeader("X-Device-Id", PREY_V2_DEVICE_ID);
  w->v2_http->addHeader("X-Burst-Id", burstId);
  w->v2_http->addHeader("X-Frame-Index", fIdxBuf);
  w->v2_http->addHeader("X-Frame-Ts-Ms", tsBuf);
  w->v2_http->addHeader("X-Request-Id", reqId);

  int httpCode = w->v2_http->POST(const_cast<uint8_t *>(jpgBuf), jpgLen);
  unsigned long elapsed = millis() - t0;
  arch->v2HttpCode[frameIdx] = (int16_t)httpCode;
  arch->v2HttpMs[frameIdx]   = (uint16_t)(elapsed > 65535 ? 65535 : elapsed);

  if (httpCode <= 0) {
    Serial.printf("V2[w%d] f[%d]: POST err=%d (%lums)\n",
                  w->id, frameIdx, httpCode, elapsed);
    w->v2_connected = false;
    return;
  }
  String body = w->v2_http->getString();
  if (httpCode != 200) {
    Serial.printf("V2[w%d] f[%d]: HTTP %d in %lums body=%s\n",
                  w->id, frameIdx, httpCode, elapsed, body.c_str());
    return;
  }

  arch->v2Status[frameIdx]        = 1;
  arch->v2Detected[frameIdx]      = v2_field_bool(body, "detected");
  arch->v2CatRecognized[frameIdx] = v2_field_bool(body, "cat_recognized");
  arch->v2PreyScore[frameIdx]     = v2_field_float(body, "prey_score", -1.0f);
  arch->v2CatConfidence[frameIdx] = v2_field_float(body, "cat_confidence", -1.0f);
  arch->v2DecisionMs[frameIdx]    = (uint16_t)v2_field_int(body, "decision_ms", 0);
  v2_field_str(body, "cat_id",      arch->v2CatId[frameIdx],      sizeof(arch->v2CatId[0]));
  v2_field_str(body, "severity",    arch->v2Severity[frameIdx],   sizeof(arch->v2Severity[0]));
  v2_field_str(body, "door_action", arch->v2DoorAction[frameIdx], sizeof(arch->v2DoorAction[0]));

  Serial.printf(
    "V2[w%d] f[%d]: %lums prey=%.3f cat=%s/%.2f sev=%s door=%s detected=%d (server=%ums)\n",
    w->id, frameIdx, elapsed,
    arch->v2PreyScore[frameIdx],
    arch->v2CatId[frameIdx][0] ? arch->v2CatId[frameIdx] : "?",
    arch->v2CatConfidence[frameIdx],
    arch->v2Severity[frameIdx][0] ? arch->v2Severity[frameIdx] : "?",
    arch->v2DoorAction[frameIdx][0] ? arch->v2DoorAction[frameIdx] : "?",
    (int)arch->v2Detected[frameIdx],
    (unsigned)arch->v2DecisionMs[frameIdx]);
}
#endif  // PREY_V2_URL

// Worker task: drains apiWorkQueue, posts results to apiResultQueue.
static void apiWorkerTask(void *arg) {
  ApiWorker *w = (ApiWorker *)arg;
  Serial.printf("API worker %d started\n", w->id);
  while (apiWorkersRunning) {
    // === Race-free self-managed TLS state ===
    // Other tasks (loop watchdog, idle-TLS-free, ToF pre-warm) may set
    // resetTlsRequested or prewarmRequested. We act on them here, with no
    // other task touching w->tls / w->http.
    if (w->resetTlsRequested) {
      w->resetTlsRequested = false;
      if (w->http) { w->http->end(); delete w->http; w->http = NULL; }
      if (w->tls)  { w->tls->stop(); delete w->tls;  w->tls  = NULL; }
      w->connected = false;
#ifdef PREY_V2_URL
      if (w->v2_http) { w->v2_http->end(); delete w->v2_http; w->v2_http = NULL; }
      if (w->v2_tcp)  { w->v2_tcp->stop(); delete w->v2_tcp;  w->v2_tcp  = NULL; }
      w->v2_connected = false;
#endif
      Serial.printf("API[w%d]: TLS torn down on request (heap %u)\n",
                    w->id, (unsigned)ESP.getFreeHeap());
    }
    if (w->prewarmRequested) {
      w->prewarmRequested = false;
      if (!w->connected) {
        unsigned long t0 = millis();
        bool ok = ensureWorkerTls(w);
        Serial.printf("API[w%d]: pre-warm TLS %s in %lums\n",
                      w->id, ok ? "OK" : "FAILED", millis() - t0);
      }
    }

    ApiWorkItem item;
    // Wait up to 100ms for work (so we can check apiWorkersRunning + flags)
    if (xQueueReceive(apiWorkQueue, &item, pdMS_TO_TICKS(100)) != pdPASS) {
      continue;
    }
    if (item.frameIdx < 0) {
      // Sentinel: stop signal
      break;
    }
    unsigned long t0 = millis();
#ifdef PREY_V2_URL
    // Dup the cropped JPEG so the shadow v2 call can reuse it after the
    // legacy call frees prepBuf. The dup lives in PSRAM (5-15 KB).
    uint8_t *v2Buf = NULL;
    size_t   v2Len = 0;
    if (item.archive != NULL) {
      v2Buf = (uint8_t *)ps_malloc(item.prepLen);
      if (v2Buf) {
        memcpy(v2Buf, item.prepBuf, item.prepLen);
        v2Len = item.prepLen;
      }
    }
#endif
    ApiTiming timing = {0, 0, 0};
    int res = callPreyApiWithWorker(w, item.prepBuf, item.prepLen, &timing);
    unsigned long total = millis() - t0;
#ifdef PREY_V2_URL
    if (v2Buf) {
      // Fire shadow v2 call BEFORE posting the legacy result so the meta
      // serializer (which runs after autonomousApiCheck collects results)
      // sees both values for every frame. Pull-side wait is bounded by
      // V2_HTTP_TIMEOUT_MS.
      if (item.archive && item.archive->generation == item.archiveGen) {
        callPreyV2Shadow(w, item.archive, item.frameIdx, v2Buf, v2Len);
      }
      free(v2Buf);
    }
#endif
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
    xTaskCreatePinnedToCore(apiWorkerTask, name, 16384,
                             &apiWorkers[i],
                             tskIDLE_PRIORITY + 2, &apiWorkers[i].task,
                             /*core*/ 0);
  }
}

static void stopApiWorkers() {
  if (!apiWorkersRunning) return;
  apiWorkersRunning = false;
  // Push sentinels so workers wake up and exit
  ApiWorkItem stop = {-1, NULL, 0, 0
#ifdef PREY_V2_URL
    , NULL, 0
#endif
  };
  for (int i = 0; i < N_API_WORKERS; i++) {
    xQueueSend(apiWorkQueue, &stop, pdMS_TO_TICKS(100));
  }
}

// Free per-worker TLS clients while keeping the worker tasks alive.
// Each WiFiClientSecure + HTTPClient pair holds ~50-60KB of internal heap
// (mbedTLS context). The first burst of a triggers reinitialises them in
// ~1s; for back-to-back triggers the per-worker keep-alive avoids redundant
// reconnects. After GREEN_LIGHT_MS of inactivity (no burst, no cat) we can
// safely drop these to claw back ~120KB of internal heap.
//
// SIGNAL ONLY: we never touch w->tls / w->http from this task. We just set
// the request flag; the worker tears down its own pointers from its own
// task. This eliminates the use-after-free race that previously crashed
// during back-to-back bursts.
static void freeIdleWorkerTls() {
  uint32_t signaled = 0;
  for (int i = 0; i < N_API_WORKERS; i++) {
    ApiWorker *w = &apiWorkers[i];
    if (w->connected || w->http || w->tls) {
      w->resetTlsRequested = true;
      signaled++;
    }
  }
  if (signaled) {
    Serial.printf("API: requested %u workers to free TLS (will happen within 100ms)\n",
                  (unsigned)signaled);
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

      ApiWorkItem w = {i, cropped, croppedLen, millis()
#ifdef PREY_V2_URL
        , &archive, archive.generation
#endif
      };
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

  // Door logic: tiered confidence lockout.
  //   - 0 prey frames: open (normal behavior)
  //   - 1 prey frame : short lockout (3 min)
  //   - >=2 frames   : long lockout (15 min)
  bool preyAny = (preyFrameCount >= 1);
  bool preyHighConfidence = (preyFrameCount >= PREY_FRAMES_THRESHOLD);
  // Keep apiPreyDetected as "any prey evidence" for event logs.
  archive.apiPreyDetected = preyAny ? 1 : 0;
  if (preyAny) {
    unsigned long lockoutMs = preyHighConfidence ? PREY_LONG_LOCKOUT_MS : PREY_SHORT_LOCKOUT_MS;
    preyLockoutUntilMs = millis() + lockoutMs;
    if (preyLockoutUntilMs == 0) preyLockoutUntilMs = 1;  // avoid sentinel
    // Persist absolute end-time as epoch so the lockout survives a reboot.
    {
      time_t nowEpoch;
      time(&nowEpoch);
      persistLockoutEpoch((int32_t)nowEpoch + (int32_t)(lockoutMs / 1000UL));
    }
    greenLightUntilMs = 0;  // clear any pending green light
    char reason[80];
    snprintf(
      reason,
      sizeof(reason),
      "prey on %d frame(s): %s lockout",
      preyFrameCount,
      preyHighConfidence ? "15 min" : "3 min"
    );
    doorCloseNow(reason);
  } else {
    if (doorLockoutActive()) {
      Serial.println("Door: stay closed (lockout still active)");
    } else {
      doorOpenNow("no prey detected");
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
  // result encodes prey-frame count: -1=pending, 0=clear, N=N prey frames
  int eventResult = (archive.apiPreyDetected < 0) ? -1 : preyFrameCount;
  // Parse dir epoch from sdPath ("/YYYYMMDD_HHMMSS_genN") so the UI can
  // reconstruct the SD dir name exactly without searching.
  time_t dirEpoch = 0;
  if (archive.sdPath[0] == '/') {
    int y, mo, d, hh, mm, ss, g;
    if (sscanf(archive.sdPath + 1, "%4d%2d%2d_%2d%2d%2d_gen%d",
               &y, &mo, &d, &hh, &mm, &ss, &g) == 7) {
      struct tm ti = {};
      ti.tm_year = y - 1900; ti.tm_mon = mo - 1; ti.tm_mday = d;
      ti.tm_hour = hh; ti.tm_min = mm; ti.tm_sec = ss; ti.tm_isdst = -1;
      dirEpoch = mktime(&ti);
    }
  }
  // API processing latency (cap at uint16 max = 65535 ms)
  unsigned long latMs = (archive.apiDoneMs > archive.apiCallMs)
    ? (archive.apiDoneMs - archive.apiCallMs) : 0;
  if (latMs > 65535) latMs = 65535;

#ifdef PREY_V2_URL
  // Shadow v2 burst summary: max prey_score across frames, modal cat_id,
  // number of frames where v2 "detected" matched legacy. Logged AND packed
  // into the event log so the events UI surfaces it without touching SD.
  int v2Ok = 0, v2Det = 0, v2CatRec = 0;
  float v2MaxPrey = -1.0f;
  int   v2MaxIdx = -1;
  int   catMazge = 0, catBenis = 0, catOther = 0;
  unsigned long v2HttpTot = 0, v2SrvTot = 0;
  for (int i = 0; i < archive.count; i++) {
    if (archive.v2Status[i] != 1) continue;
    v2Ok++;
    if (archive.v2Detected[i] == 1) v2Det++;
    if (archive.v2CatRecognized[i] == 1) v2CatRec++;
    if (archive.v2PreyScore[i] > v2MaxPrey) {
      v2MaxPrey = archive.v2PreyScore[i]; v2MaxIdx = i;
    }
    if      (!strcmp(archive.v2CatId[i], "mazge")) catMazge++;
    else if (!strcmp(archive.v2CatId[i], "benis")) catBenis++;
    else if (archive.v2CatId[i][0])                catOther++;
    v2HttpTot += archive.v2HttpMs[i];
    v2SrvTot  += archive.v2DecisionMs[i];
  }
  const char *modalCat = (catMazge >= catBenis && catMazge >= catOther && catMazge > 0) ? "mazge"
                       : (catBenis >= catOther && catBenis > 0)                          ? "benis"
                       : (catOther > 0)                                                  ? "other"
                                                                                         : "?";
  uint8_t modalCatBits = 0;  // 0=unknown
  if      (catMazge >= catBenis && catMazge >= catOther && catMazge > 0) modalCatBits = 1;
  else if (catBenis >= catOther && catBenis > 0)                          modalCatBits = 2;
  else if (catOther > 0)                                                  modalCatBits = 3;
  uint16_t v2MaxPreyX1000 = 0;
  if (v2MaxPrey >= 0.0f) {
    int s = (int)(v2MaxPrey * 1000.0f + 0.5f);
    if (s < 0) s = 0;
    if (s > 1000) s = 1000;
    v2MaxPreyX1000 = (uint16_t)s;
  }
  uint8_t v2Flags = 0;
  if (v2Det > 0)    v2Flags |= 0x01;
  if (v2CatRec > 0) v2Flags |= 0x02;
  v2Flags |= (modalCatBits & 0x03) << 2;
  uint8_t v2OkFrames = (uint8_t)(v2Ok > 255 ? 255 : v2Ok);
  Serial.printf(
    "V2-SUMMARY gen=%d frames=%d v2ok=%d v2det=%d v2catRec=%d maxPrey=%.3f@f%d "
    "cat=%s(m=%d/b=%d/o=%d) httpAvg=%lums srvAvg=%lums | legacyPrey=%d/%d\n",
    archive.generation, archive.count, v2Ok, v2Det, v2CatRec,
    v2MaxPrey, v2MaxIdx, modalCat, catMazge, catBenis, catOther,
    v2Ok ? v2HttpTot / v2Ok : 0,
    v2Ok ? v2SrvTot / v2Ok  : 0,
    preyFrameCount, archive.count);
  addEvent(archive.generation, archive.count, eventResult, dMin, dMax, trend, true,
           dirEpoch, (uint16_t)latMs, v2MaxPreyX1000, v2OkFrames, v2Flags);
#else
  addEvent(archive.generation, archive.count, eventResult, dMin, dMax, trend, true,
           dirEpoch, (uint16_t)latMs);
#endif

  // Blynk: push event telemetry (prey result + detail)
  blynkPushEvent(archive.apiPreyDetected == 1 ? 1 : 0, preyFrameCount, archive.count, latMs);
}

// Guard: only one API task at a time (timestamp-based, auto-expires).
// On expiry the watchdog also requests the in-flight task to abandon.
volatile unsigned long apiTaskStartMs = 0;
volatile bool apiAbandonRequested = false;          // signal to in-flight task to stop early
volatile uint32_t apiAbandonCount = 0;              // total abandons since boot

// Diagnostics: persisted across boots so we can see crash patterns remotely.
static uint32_t bootResetReason = 0;       // ESP reset-reason code from this boot
static uint32_t bootCounter = 0;           // total boots since the firmware was first flashed
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

// Force-tear down the legacy shared TLS client and request worker tear down.
// Used by the watchdog after abandoning an in-flight analysis. The next call
// will reconnect from scratch.
//
// Worker TLS is torn down via a flag the worker checks itself — never from
// this task — to avoid use-after-free races while the worker is mid-POST.
static void resetAllApiConnections(const char *reason) {
  Serial.printf("API: resetting TLS connections (%s)\n", reason);
  tlsConnected = false;
  if (httpApi)   { httpApi->end();   delete httpApi;   httpApi   = NULL; }
  if (tlsClient) { tlsClient->stop(); delete tlsClient; tlsClient = NULL; }
  for (int i = 0; i < N_API_WORKERS; i++) {
    apiWorkers[i].resetTlsRequested = true;
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

// ===== HTML page =====
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32-CAM Live</title>
  <style>
    * { box-sizing: border-box; }
    body {
      background: radial-gradient(circle at 25% 10%, #1e2538 0%, #0d1017 35%, #07080b 100%);
      color: #e8edf7;
      font-family: sans-serif;
      margin: 0;
      padding: 10px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    h1 { margin: 0 0 4px; font-size: 1.35em; }
    #stats { color: #7f8aa4; margin: 0 0 8px; font-family: monospace; font-size: 0.82em; }
    #stream-wrap {
      display: inline-block;
      position: relative;
      overflow: hidden;
      max-width: 420px;
      border-radius: 10px;
      border: 1px solid #29314a;
      background: #05070d;
    }
    #stream-wrap img {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: rotate(-90deg) translate(-50%,-50%);
      transform-origin: 0 0;
      max-width: none;
    }
    .btn-row {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      justify-content: center;
      margin: 0 0 8px;
    }
    .btn-row button, .btn-row a {
      padding: 8px 14px;
      border: none;
      border-radius: 7px;
      color: #fff;
      text-decoration: none;
      font-size: 0.92em;
      cursor: pointer;
      display: inline-block;
    }
    .btn-main { background: #8b2e2e; }
    .btn-warn { background: #a06a16; }
    .btn-safe { background: #2f7a49; }
    .btn-danger { background: #822f2f; }
    .btn-link { background: #314d85; }
    .btn-link2 { background: #37543d; }
    .panel {
      width: 100%;
      max-width: 920px;
      margin-top: 14px;
      background: rgba(14, 20, 33, 0.9);
      border: 1px solid #25304a;
      border-radius: 10px;
      padding: 10px;
    }
    .panel h2 { font-size: 1.06em; margin: 0 0 8px; }
    #events-log {
      max-height: 180px;
      overflow-y: auto;
      font-family: monospace;
      font-size: 0.8em;
      color: #97a6c9;
      border-top: 1px solid #202a3f;
      padding-top: 5px;
    }
    .event-row { padding: 2px 0; border-bottom: 1px solid #182034; }
    .event-row:hover { background: #1d2740; }
    .burst-card {
      margin: 8px 0;
      padding: 8px;
      border-radius: 8px;
      background: #131b2b;
      border: 1px solid #2a3754;
    }
    .burst-head {
      display: flex;
      justify-content: space-between;
      gap: 8px;
      align-items: baseline;
      margin-bottom: 6px;
      font-size: 0.86em;
    }
    .burst-date { color: #c6d4f3; font-weight: bold; }
    .burst-meta { color: #7f90b6; }
    .burst-grid {
      display: grid;
      grid-template-columns: repeat(5, 1fr);
      gap: 6px;
    }
    .thumb-wrap {
      position: relative;
      width: 100%;
      aspect-ratio: 3 / 4;
      overflow: hidden;
      border: 2px solid #4e5976;
      border-radius: 5px;
      background: #060911;
      cursor: pointer;
    }
    .thumb-wrap img {
      /* Camera shoots landscape but device is mounted sideways → rotate
         90° CCW. After rotation a 4:3 source matches the 3:4 thumb-wrap.
         We set width/height to the wrap's *opposite* dimension and translate
         so the rotated image fills the box. */
      position: absolute;
      top: 50%;
      left: 50%;
      width: 133.333%;       /* = wrap height / wrap width (4/3) */
      height: 75%;           /* = wrap width  / wrap height (3/4) */
      transform: translate(-50%, -50%) rotate(-90deg);
      transform-origin: center center;
      object-fit: cover;
      display: block;
      border: none;
    }
    .thumb-wrap.prey { border-color: #f54; box-shadow: 0 0 8px rgba(255, 80, 70, 0.7); }
    .thumb-wrap.clear { border-color: #4d5; box-shadow: 0 0 8px rgba(80, 240, 80, 0.55); }
    .thumb-wrap.pending { border-color: #cc8; }
    .thumb-wrap.missing::before {
      content: 'missing';
      position: absolute; top: 50%; left: 0; right: 0;
      transform: translateY(-50%);
      text-align: center; color: #888; font-size: 0.75em;
    }
    .idx {
      position: absolute;
      left: 0;
      right: 0;
      bottom: 0;
      text-align: center;
      font-size: 0.67em;
      padding: 1px 0;
      background: rgba(0, 0, 0, 0.68);
      color: #c9d7f8;
    }
    #prey-empty { color: #94a4c8; font-size: 0.9em; }
    @media (max-width: 760px) {
      .burst-grid { grid-template-columns: repeat(2, 1fr); }
    }
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
  <div class="btn-row">
    <button id="toggle-stream" class="btn-main" onclick="toggleStream()">Start Stream</button>
    <button class="btn-warn" onclick="fetch('/cmd?trigger=1')">Fake Trigger</button>
    <button class="btn-safe" onclick="fetch('/cmd?door=1')">Open Door</button>
    <button class="btn-danger" onclick="fetch('/cmd?door=0')">Close Door</button>
    <a href="/controls" class="btn-link">Camera Controls</a>
    <a href="/settings" class="btn-link">Advanced Settings</a>
    <a href="/sd" class="btn-link2">SD Card</a>
  </div>
  <div id="stream-wrap"><img id="stream" src="" onload="var w=this.naturalHeight,h=this.naturalWidth;this.parentElement.style.width=w+'px';this.parentElement.style.height=h+'px';this.style.width=h+'px';" /></div>
  <div class="panel">
    <h2>Events Log</h2>
    <div id="events-log"></div>
  </div>

  <div class="panel">
    <h2>Burst Viewer</h2>
    <div id="prey-meta" style="font-size:0.84em;color:#8da0c9;margin-bottom:6px;">Click an event row above to load its burst frames from SD.</div>
    <div id="prey-empty" style="display:none;"></div>
    <div id="prey-list"></div>
  </div>

  <script>
    let streamOn = false;
    let doorOpenState = null;
    let lockoutEndsAt = 0; // local timestamp (ms); 0 = no lockout
    let lastEventsSignature = '';

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

    function fmtEpoch(epoch) {
      if (!epoch || epoch < 1700000000) return 'time unknown';
      const d = new Date(epoch * 1000);
      return d.toLocaleString('sv-SE', { timeZone: 'Europe/Bratislava', hour12: false });
    }

    function frameClassByResult(res) {
      if (res === 1) return 'prey';
      if (res === 0) return 'clear';
      return 'pending';
    }

    let currentBurstKey = null;
    // Show a burst that's still in RAM (latest archives). Instant — no SD reads.
    async function showRamBurst(archIdx, label) {
      const key = 'ram:' + archIdx + ':' + label;
      const listEl = document.getElementById('prey-list');
      const emptyEl = document.getElementById('prey-empty');
      const metaEl = document.getElementById('prey-meta');
      currentBurstKey = key;
      metaEl.textContent = 'Loading burst ' + label + ' (RAM)...';
      listEl.innerHTML = '';
      emptyEl.style.display = 'none';
      try {
        const r = await fetch('/raminfo?a=' + archIdx);
        const data = await r.json();
        if (currentBurstKey !== key) return;
        if (!data.ok) { metaEl.textContent = label + ' — ' + (data.error || 'failed'); return; }
        const frames = data.frames || [];
        metaEl.textContent = label + ' — RAM archive #' + data.a + ' gen' + data.gen + ' (' + frames.length + ' frames)';
        if (frames.length === 0) { emptyEl.textContent = 'No frames'; emptyEl.style.display = ''; return; }
        let html = '<div class="burst-card"><div class="burst-grid">';
        for (let fr of frames) {
          const cls = frameClassByResult(fr.res);
          const u = '/ramburst?a=' + data.a + '&i=' + fr.idx;
          html += '<div class="thumb-wrap ' + cls + '" data-url="' + u + '">';
          html += '<img src="' + u + '" onerror="this.style.display=\'none\';this.parentNode.classList.add(\'missing\');">';
          html += '<div class="idx">f' + String(fr.idx).padStart(2, '0') + (fr.res === 1 ? ' PREY' : (fr.res === 0 ? ' CLEAR' : ' ?')) + '</div>';
          html += '</div>';
        }
        html += '</div></div>';
        listEl.innerHTML = html;
        for (const el of listEl.querySelectorAll('.thumb-wrap')) {
          el.onclick = () => { const u = el.getAttribute('data-url'); if (u) window.open(u, '_blank'); };
        }
      } catch (e) {
        if (currentBurstKey === key) document.getElementById('prey-meta').textContent = 'Failed: ' + e;
      }
    }

    async function showBurst(epoch, gen, label) {
      const key = epoch + ':' + gen;
      const listEl = document.getElementById('prey-list');
      const emptyEl = document.getElementById('prey-empty');
      const metaEl = document.getElementById('prey-meta');
      currentBurstKey = key;
      metaEl.textContent = 'Loading burst ' + label + '...';
      listEl.innerHTML = '';
      emptyEl.style.display = 'none';
      try {
        const r = await fetch('/burstinfo?epoch=' + epoch + '&gen=' + gen);
        const data = await r.json();
        if (currentBurstKey !== key) return; // user clicked another row already
        if (!data.ok) {
          metaEl.textContent = label + ' — ' + (data.error || 'failed');
          return;
        }
        const frames = data.frames || [];
        metaEl.textContent = label + ' — ' + data.dir + ' (' + frames.length + ' frames)';
        if (frames.length === 0) { emptyEl.textContent = 'No frames'; emptyEl.style.display = ''; return; }
        let html = '<div class="burst-card"><div class="burst-grid">';
        for (let fr of frames) {
          const cls = frameClassByResult(fr.res);
          const u = '/sdget?f=' + encodeURIComponent(data.dir + '/' + fr.file);
          html += '<div class="thumb-wrap ' + cls + '" data-url="' + u + '">';
          html += '<img src="' + u + '" onerror="this.style.display=\'none\';this.parentNode.classList.add(\'missing\');">';
          html += '<div class="idx">f' + String(fr.idx).padStart(2, '0') + (fr.res === 1 ? ' PREY' : (fr.res === 0 ? ' CLEAR' : ' ?')) + '</div>';
          html += '</div>';
        }
        html += '</div></div>';
        listEl.innerHTML = html;
        for (const el of listEl.querySelectorAll('.thumb-wrap')) {
          el.onclick = () => { const u = el.getAttribute('data-url'); if (u) window.open(u, '_blank'); };
        }
      } catch (e) {
        if (currentBurstKey === key) {
          document.getElementById('prey-meta').textContent = 'Failed to load burst: ' + e;
        }
      }
    }

    // Load persisted events from NVS on page load
    async function loadPersistedEvents() {
      try {
        const r = await fetch('/getevents');
        const data = await r.json();
        const events = data.events || data;
        const bootEpoch = data.epoch ? (data.epoch - data.uptimeMs/1000) : null;
        // Force CET / Europe/Bratislava timezone so times match real-world clock
        // regardless of which timezone the laptop happens to be in.
        const TZ_OPTS = { timeZone: 'Europe/Bratislava' };
        const FMT_TIME = new Intl.DateTimeFormat('sv-SE', {
          ...TZ_OPTS, hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
        const FMT_DATETIME = new Intl.DateTimeFormat('sv-SE', {
          ...TZ_OPTS, year: 'numeric', month: '2-digit', day: '2-digit',
          hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
        const NOW_LOCAL_DATE = FMT_DATETIME.format(new Date()).slice(0, 10); // YYYY-MM-DD
        function formatEventTime(ev) {
          // Prefer per-event epoch (set by firmware at the moment the event
          // was recorded -- survives reboots). Falls back to bootEpoch math
          // only for events recorded before NTP synced.
          let epochSec = null;
          if (ev.epoch && ev.epoch > 1700000000) {  // sane Unix time
            epochSec = ev.epoch;
          } else if (bootEpoch && ev.t) {
            epochSec = bootEpoch + ev.t / 1000;
          }
          if (!epochSec) {
            return (ev.ago / 1000).toFixed(0) + 's ago';
          }
          const d = new Date(epochSec * 1000);
          const full = FMT_DATETIME.format(d);            // "2026-05-15 23:22:03"
          const datePart = full.slice(0, 10);
          const timePart = full.slice(11);
          return datePart === NOW_LOCAL_DATE ? timePart : full;
        }
        const el = document.getElementById('events-log');
        for (let i = events.length - 1; i >= 0; i--) {
          const e = events[i];
          const timeStr = formatEventTime(e);
          let distStr;
          if (e.dMin >= 0 && e.dMax >= 0) {
            distStr = e.dMin === e.dMax ? e.dMin + 'mm' : e.dMin + '\u2192' + e.dMax + 'mm';
          } else { distStr = '--'; }
          let resStr, resColor;
          if (e.res >= 2) { resStr = '\u{1F534} PREY (' + e.res + 'f)'; resColor = '#f44'; }
          else if (e.res === 1) { resStr = '\u{1F7E0} PREY (1f)'; resColor = '#fa3'; }
          else if (e.res === 0) { resStr = '\u{1F7E2} CLEAR'; resColor = '#4f4'; }
          else { resStr = '\u23F3 PENDING'; resColor = '#888'; }
          const div = document.createElement('div');
          div.id = 'pev-' + e.t;
          div.className = 'event-row';
          div.style.opacity = '0.7';
          const trendLabels = ['\u2753','\u27A1\uFE0F IN','\u2B05\uFE0F OUT','\u21C6 FLAT'];
          const trendColors = ['#666','#4f4','#f84','#fc4'];
          const tl = e.trend >= 0 && e.trend <= 3 ? e.trend : 0;
          div.innerHTML = '<span style="color:#666">' + timeStr + '</span> ' +
            '<b style="color:#aaa">gen' + e.gen + '</b> ' +
            '<span style="color:#6af">' + e.nf + 'f</span> ' +
            '<span style="color:#4cf">' + distStr + '</span> ' +
            '<span style="color:' + trendColors[tl] + '">' + trendLabels[tl] + '</span> ' +
            '<span style="color:' + resColor + '">' + resStr + '</span>' +
            (e.lat > 0 ? ' <span style="color:#607296">' + (e.lat / 1000).toFixed(1) + 's</span>' : '') +
            (e.v2 && e.v2.okFrames > 0
              ? ' <span style="color:#9af;font-size:0.85em">[v2 ' +
                e.v2.maxPrey.toFixed(2) + ' ' + e.v2.catId + (e.v2.det ? ' \u26A0' : '') + ']</span>'
              : (e.v2 ? ' <span style="color:#666;font-size:0.85em">[v2 \u2205]</span>' : ''));
          // Make rows with a real epoch clickable: load the burst on demand
          if (e.epoch && e.epoch > 1700000000) {
            div.style.cursor = 'pointer';
            const ep = e.epoch, gn = e.gen, lbl = timeStr + ' gen' + e.gen;
            div.onclick = () => showBurst(ep, gn, lbl);
          }
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
        // Mode indicator (always autonomous now)
        const modeEl = document.getElementById('mode-val');
        if (modeEl) {
          modeEl.textContent = '\u{1F916} AUTONOMOUS';
          modeEl.style.background = '#642'; modeEl.style.color = '#fc4';
        }
        // Lightweight live summary row (newest burst only)
        if (s.apiResults && s.apiResults.length > 0) {
          const last = s.apiResults.length - 1;
          const sig = String(s.burstGen) + ':' + String(s.apiResults[last]);
          if (sig !== lastEventsSignature) {
            lastEventsSignature = sig;
            const el = document.getElementById('events-log');
            const gen = s.burstGens ? s.burstGens[last] : s.burstGen;
            const res = s.apiResults[last];
            const dMin = s.distMin ? s.distMin[last] : -1;
            const dMax = s.distMax ? s.distMax[last] : -1;
            const nf = s.burstCounts ? s.burstCounts[last] : 0;
            const trigMs = s.triggerMs ? s.triggerMs[last] : 0;
            const doneMs = s.apiDoneMs ? s.apiDoneMs[last] : 0;
            const up = s.uptimeMs || 0;
            const tAgo = trigMs > 0 ? ((up - trigMs) / 1000).toFixed(0) : '--';
            const distStr = (dMin >= 0 && dMax >= 0) ? (dMin === dMax ? dMin + 'mm' : (dMin + '→' + dMax + 'mm')) : '--';
            let resText = res === 1 ? 'PREY' : res === 0 ? 'CLEAR' : 'PENDING';
            let resColor = res === 1 ? '#f44' : res === 0 ? '#4f4' : '#999';
            const proc = (doneMs > 0 && trigMs > 0) ? (((doneMs - trigMs) / 1000).toFixed(1) + 's') : '';
            const row = document.createElement('div');
            row.className = 'event-row';
            row.innerHTML = '<span style="color:#607296">' + tAgo + 's ago</span> ' +
              '<b style="color:#a7b9de">gen' + gen + '</b> ' +
              '<span style="color:#85a5da">' + nf + 'f</span> ' +
              '<span style="color:#64b7d7">' + distStr + '</span> ' +
              '<span style="color:' + resColor + '">' + resText + '</span> ' +
              '<span style="color:#607296">' + proc + '</span>';
            // Click → show from RAM (instant, no SD).
            row.style.cursor = 'pointer';
            const archIdx = last;
            const lbl = tAgo + 's ago gen' + gen;
            row.onclick = () => showRamBurst(archIdx, lbl);
            el.insertBefore(row, el.firstChild);
            while (el.childElementCount > 80) el.removeChild(el.lastChild);
            // Auto-display newly-finished burst (only when API is done — res != -1)
            if (res !== -1) showRamBurst(archIdx, lbl);
          }
        }
      } catch(e) {}
    }
    let statsInterval = setInterval(pollStats, 200);
    pollStats();
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
  // Never cache the HTML — UI changes ship via OTA and we want them visible immediately.
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, must-revalidate");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
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
  unsigned long nowMs = millis();
  unsigned long lockoutMsRemaining = 0;
  if (preyLockoutUntilMs != 0 && (long)(preyLockoutUntilMs - nowMs) > 0) {
    lockoutMsRemaining = preyLockoutUntilMs - nowMs;
  }
  snprintf(json, sizeof(json),
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u,\"distance\":%d,\"lux\":%u,\"autoAec\":%d,"
    "\"burstArchives\":%d,\"burstGen\":%d,\"burstCounts\":%s,\"apiResults\":%s,\"burstGens\":%s,"
    "\"apiSent\":%s,\"triggerMs\":%s,\"apiDoneMs\":%s,\"distMin\":%s,\"distMax\":%s,"
    "\"freePsram\":%u,\"uptimeMs\":%lu,"
    "\"doorOpen\":%s,\"lockoutMs\":%lu,"
    "\"apiBusy\":%s,\"apiTaskAgeMs\":%lu,\"apiAbandons\":%u,\"lastTriggerMs\":%lu}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount,
    tofDistance, alsLux, autoBaseAec, burstArchiveCount, burstGen, archBuf,
    apiResBuf, genBuf, sentBuf, trigBuf, apiDoneBuf, distMinBuf, distMaxBuf,
    ESP.getFreePsram(), nowMs,
    doorOpen ? "true" : "false", lockoutMsRemaining,
    apiTaskStartMs ? "true" : "false",
    apiTaskStartMs ? (nowMs - apiTaskStartMs) : 0UL,
    (unsigned)apiAbandonCount, lastBurstTriggerMs);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

// Lightweight diagnostics endpoint — heap, boot count, reset reason, worker
// stack high-water marks. Lets us correlate crashes over time without serial.
static esp_err_t diag_handler(httpd_req_t *req) {
  char json[512];
  UBaseType_t w0 = apiWorkers[0].task ? uxTaskGetStackHighWaterMark(apiWorkers[0].task) : 0;
  UBaseType_t w1 = (N_API_WORKERS > 1 && apiWorkers[N_API_WORKERS > 1 ? 1 : 0].task)
                     ? uxTaskGetStackHighWaterMark(apiWorkers[N_API_WORKERS > 1 ? 1 : 0].task) : 0;
  uint32_t storedBoot = 0;
  uint32_t lastRR = 0;
  nvsPrefs.begin("state", true);
  storedBoot = nvsPrefs.getUInt("bootCount", 0);
  lastRR     = nvsPrefs.getUInt("lastRR", 0);
  nvsPrefs.end();
  const char *rrName = "?";
  switch ((esp_reset_reason_t)bootResetReason) {
    case ESP_RST_POWERON:   rrName = "POWERON";   break;
    case ESP_RST_EXT:       rrName = "EXT";       break;
    case ESP_RST_SW:        rrName = "SW";        break;
    case ESP_RST_PANIC:     rrName = "PANIC";     break;
    case ESP_RST_INT_WDT:   rrName = "INT_WDT";   break;
    case ESP_RST_TASK_WDT:  rrName = "TASK_WDT";  break;
    case ESP_RST_WDT:       rrName = "WDT";       break;
    case ESP_RST_DEEPSLEEP: rrName = "DEEPSLEEP"; break;
    case ESP_RST_BROWNOUT:  rrName = "BROWNOUT";  break;
    case ESP_RST_SDIO:      rrName = "SDIO";      break;
    default: break;
  }
  snprintf(json, sizeof(json),
    "{\"bootCount\":%u,\"resetReason\":%u,\"resetReasonName\":\"%s\","
    "\"persistedRR\":%u,\"uptimeMs\":%lu,"
    "\"freeHeap\":%u,\"minFreeHeap\":%u,\"freePsram\":%u,\"minFreePsram\":%u,"
    "\"workerStackHW0\":%u,\"workerStackHW1\":%u,"
    "\"apiAbandons\":%u,\"rssi\":%d}",
    (unsigned)bootCounter, (unsigned)bootResetReason, rrName,
    (unsigned)lastRR, millis(),
    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(),
    (unsigned)ESP.getFreePsram(), (unsigned)ESP.getMinFreePsram(),
    (unsigned)w0, (unsigned)w1,
    (unsigned)apiAbandonCount, (int)WiFi.RSSI());
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
        persistLockoutEpoch(0);
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
    // Pre-warm worker TLS during the BURST_SHIFT_MS + capture window so the
    // first API POST does not pay TLS handshake cost.
    for (int i = 0; i < N_API_WORKERS; i++) {
      if (!apiWorkers[i].connected) apiWorkers[i].prewarmRequested = true;
    }
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
  config.max_open_sockets = 2;  // stream + margin

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Stream server started on port 81");
  }
}

static bool parseJsonIntField(const char *json, const char *key, long *outVal) {
  const char *p = strstr(json, key);
  if (!p) return false;
  p += strlen(key);
  while (*p == ' ' || *p == '\t' || *p == ':') p++;
  if (*p == '\0') return false;
  char *endPtr = NULL;
  long v = strtol(p, &endPtr, 10);
  if (endPtr == p) return false;
  *outVal = v;
  return true;
}

static int parseApiResultsArray(const char *json, int8_t outRes[RING_SIZE]) {
  for (int i = 0; i < RING_SIZE; i++) outRes[i] = -1;
  const char *p = strstr(json, "\"apiResults\":[");
  if (!p) return 0;
  p = strchr(p, '[');
  if (!p) return 0;
  p++;
  int count = 0;
  while (*p && *p != ']' && count < RING_SIZE) {
    while (*p == ' ' || *p == '\t' || *p == ',') p++;
    if (*p == ']') break;
    char *endPtr = NULL;
    long v = strtol(p, &endPtr, 10);
    if (endPtr == p) break;
    if (v < -1) v = -1;
    if (v > 1) v = 1;
    outRes[count++] = (int8_t)v;
    p = endPtr;
    while (*p == ' ' || *p == '\t' || *p == ',') p++;
  }
  return count;
}

static bool loadMetaJson(const char *dirName, char *metaBuf, size_t metaBufSize) {
  if (!dirName || !metaBuf || metaBufSize < 32) return false;
  char path[96];
  snprintf(path, sizeof(path), "/%s/meta.json", dirName);
  File f = SD_MMC.open(path);
  if (!f || f.isDirectory()) return false;
  size_t n = f.readBytes(metaBuf, metaBufSize - 1);
  metaBuf[n] = '\0';
  f.close();
  return n > 0;
}

static bool fileNameEndsWithJpg(const char *name) {
  if (!name) return false;
  const char *base = strrchr(name, '/');
  if (base) name = base + 1;
  int n = strlen(name);
  if (n < 4) return false;
  const char *tail = name + n - 4;
  return strcmp(tail, ".jpg") == 0 || strcmp(tail, ".JPG") == 0;
}

static int frameIndexFromName(const char *name) {
  if (!name) return -1;
  const char *base = strrchr(name, '/');
  if (base) name = base + 1;
  if (strlen(name) < 3) return -1;
  if (name[0] == 'f' && name[1] >= '0' && name[1] <= '9' && name[2] >= '0' && name[2] <= '9') {
    return (name[1] - '0') * 10 + (name[2] - '0');
  }
  return -1;
}

static int collectFrameFilesForBurst(const char *dirName,
                                     char frameFileByIdx[RING_SIZE][24],
                                     bool hasByIdx[RING_SIZE]) {
  for (int i = 0; i < RING_SIZE; i++) {
    frameFileByIdx[i][0] = '\0';
    hasByIdx[i] = false;
  }
  char path[64];
  snprintf(path, sizeof(path), "/%s", dirName);
  File sub = SD_MMC.open(path);
  if (!sub || !sub.isDirectory()) return 0;
  int jpgCount = 0;
  while (true) {
    File sf = sub.openNextFile();
    if (!sf) break;
    if (!sf.isDirectory()) {
      const char *nm = sf.name();
      if (fileNameEndsWithJpg(nm)) {
        jpgCount++;
        int idx = frameIndexFromName(nm);
        if (idx >= 0 && idx < RING_SIZE) {
          const char *base = strrchr(nm, '/');
          if (base) nm = base + 1;
          strlcpy(frameFileByIdx[idx], nm, sizeof(frameFileByIdx[idx]));
          hasByIdx[idx] = true;
        }
      }
    }
    sf.close();
  }
  sub.close();
  return jpgCount;
}

// Try to find SD directory for a logged event by reconstructing its name
// from epochSec + gen. New events store the exact dir-creation epoch so the
// first reconstruction usually hits. For legacy events (event epoch != dir
// epoch), we fall back to a single root scan that picks the closest dir.
static bool findEventDir(time_t epochSec, int gen, char *outName, size_t outSize) {
  if (!sdReady || epochSec < 1000000000) return false;
  struct tm ti;
  char tryName[48], metaPath[80];

  // Fast path: try the exact reconstructed name.
  localtime_r(&epochSec, &ti);
  snprintf(tryName, sizeof(tryName), "%04d%02d%02d_%02d%02d%02d_gen%d",
    ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday,
    ti.tm_hour, ti.tm_min, ti.tm_sec, gen);
  snprintf(metaPath, sizeof(metaPath), "/%s/meta.json", tryName);
  File f = SD_MMC.open(metaPath);
  if (f) {
    bool isDir = f.isDirectory();
    f.close();
    if (!isDir) {
      strlcpy(outName, tryName, outSize);
      return true;
    }
  }

  // Fallback: single root scan to find closest dir within ±60s for same gen.
  // One openNextFile sweep instead of dozens of blind opens.
  File root = SD_MMC.open("/");
  if (!root) return false;
  char bestName[48] = "";
  long bestDelta = 61; // > tolerance threshold
  for (;;) {
    File entry = root.openNextFile();
    if (!entry) break;
    bool isDir = entry.isDirectory();
    char nm[64];
    strlcpy(nm, entry.name(), sizeof(nm));
    entry.close();
    if (!isDir) continue;
    const char *base = strrchr(nm, '/');
    base = base ? base + 1 : nm;
    int y, mo, d, hh, mm, ss, g;
    if (sscanf(base, "%4d%2d%2d_%2d%2d%2d_gen%d", &y, &mo, &d, &hh, &mm, &ss, &g) != 7) continue;
    if (g != gen) continue;
    struct tm dt = {};
    dt.tm_year = y - 1900; dt.tm_mon = mo - 1; dt.tm_mday = d;
    dt.tm_hour = hh; dt.tm_min = mm; dt.tm_sec = ss; dt.tm_isdst = -1;
    time_t dirEpoch = mktime(&dt);
    long delta = (long)dirEpoch - (long)epochSec;
    if (delta < 0) delta = -delta;
    if (delta < bestDelta) {
      bestDelta = delta;
      strlcpy(bestName, base, sizeof(bestName));
    }
  }
  root.close();
  if (bestName[0]) {
    strlcpy(outName, bestName, outSize);
    return true;
  }
  return false;
}

// /raminfo?a=<archIdx> — JSON manifest of in-RAM burst archive (no SD).
// Used for the "latest burst" view so newly-finished bursts show instantly.
static esp_err_t raminfo_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char query[32], val[8];
  int qlen = httpd_req_get_url_query_len(req) + 1;
  int archIdx = burstArchiveCount - 1; // default: latest
  if (qlen > 1 && qlen <= (int)sizeof(query)) {
    httpd_req_get_url_query_str(req, query, sizeof(query));
    if (httpd_query_key_value(query, "a", val, sizeof(val)) == ESP_OK) archIdx = atoi(val);
  }
  if (archIdx < 0 || archIdx >= burstArchiveCount) {
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"bad archIdx\"}", HTTPD_RESP_USE_STRLEN);
  }
  BurstArchive &arch = burstArchives[archIdx];
  char out[1024];
  int pos = snprintf(out, sizeof(out),
    "{\"ok\":true,\"a\":%d,\"gen\":%d,\"count\":%d,\"frames\":[",
    archIdx, arch.generation, arch.count);
  bool first = true;
  for (int i = 0; i < arch.count && pos < (int)sizeof(out) - 80; i++) {
    if (!arch.images[i].buf || arch.images[i].len == 0) continue;
    pos += snprintf(out + pos, sizeof(out) - pos,
      "%s{\"idx\":%d,\"res\":%d,\"bytes\":%u}",
      first ? "" : ",", i, arch.apiResults[i], (unsigned)arch.images[i].len);
    first = false;
  }
  if (pos < (int)sizeof(out) - 4) pos += snprintf(out + pos, sizeof(out) - pos, "]}");
  return httpd_resp_send(req, out, pos);
}

// /ramburst?a=<archIdx>&i=<imgIdx> — raw JPEG from in-RAM burst archive.
static esp_err_t ramburst_handler(httpd_req_t *req) {
  char query[32], val[8];
  int qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1 || qlen > (int)sizeof(query)) { httpd_resp_send_404(req); return ESP_FAIL; }
  httpd_req_get_url_query_str(req, query, sizeof(query));
  int archIdx = -1, imgIdx = -1;
  if (httpd_query_key_value(query, "a", val, sizeof(val)) == ESP_OK) archIdx = atoi(val);
  if (httpd_query_key_value(query, "i", val, sizeof(val)) == ESP_OK) imgIdx = atoi(val);
  if (archIdx < 0 || archIdx >= burstArchiveCount) { httpd_resp_send_404(req); return ESP_FAIL; }
  BurstArchive &arch = burstArchives[archIdx];
  if (imgIdx < 0 || imgIdx >= arch.count || !arch.images[imgIdx].buf || arch.images[imgIdx].len == 0) {
    httpd_resp_send_404(req); return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  // RAM frames are tied to a session's burstArchives slot; safe to cache short-term.
  return httpd_resp_send(req, (const char *)arch.images[imgIdx].buf, arch.images[imgIdx].len);
}

// /burstinfo?epoch=<sec>&gen=<n> — find SD dir for this event and return its
// frame manifest. Returns {"ok":true,"dir":"...","frames":[{"idx":N,"res":-1/0/1,"file":"f02_0123ms.jpg"}, ...]}.
// On-demand: no boot scan, no in-RAM cache. Frames are then fetched via /sdget.
static esp_err_t burstinfo_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (!sdReady) {
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"SD not mounted\"}", HTTPD_RESP_USE_STRLEN);
  }
  char query[64], val[24];
  int qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1 || qlen > (int)sizeof(query)) {
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"missing query\"}", HTTPD_RESP_USE_STRLEN);
  }
  httpd_req_get_url_query_str(req, query, sizeof(query));
  time_t epoch = 0;
  int gen = 0;
  if (httpd_query_key_value(query, "epoch", val, sizeof(val)) == ESP_OK) epoch = (time_t)strtoll(val, NULL, 10);
  if (httpd_query_key_value(query, "gen", val, sizeof(val)) == ESP_OK) gen = atoi(val);
  if (epoch < 1000000000) {
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"bad epoch\"}", HTTPD_RESP_USE_STRLEN);
  }

  if (!sdMutex || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"SD busy\"}", HTTPD_RESP_USE_STRLEN);
  }

  char dirName[48];
  if (!findEventDir(epoch, gen, dirName, sizeof(dirName))) {
    xSemaphoreGive(sdMutex);
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"dir not found\"}", HTTPD_RESP_USE_STRLEN);
  }

  // Read API results from meta.json (if present)
  int8_t apiResults[RING_SIZE];
  int apiCount = 0;
  {
    char metaBuf[4096];
    if (loadMetaJson(dirName, metaBuf, sizeof(metaBuf))) {
      apiCount = parseApiResultsArray(metaBuf, apiResults);
    }
  }

  // Enumerate JPEGs
  char frameFileByIdx[RING_SIZE][24];
  bool hasByIdx[RING_SIZE];
  int jpgCount = collectFrameFilesForBurst(dirName, frameFileByIdx, hasByIdx);
  xSemaphoreGive(sdMutex);
  if (jpgCount == 0) {
    return httpd_resp_send(req, "{\"ok\":false,\"error\":\"no frames\"}", HTTPD_RESP_USE_STRLEN);
  }

  // Build JSON: include all available frames so UI can pick which to show
  char out[1024];
  int pos = snprintf(out, sizeof(out), "{\"ok\":true,\"dir\":\"%s\",\"frames\":[", dirName);
  bool first = true;
  for (int idx = 0; idx < RING_SIZE && pos < (int)sizeof(out) - 80; idx++) {
    if (!hasByIdx[idx]) continue;
    int8_t res = (idx < apiCount) ? apiResults[idx] : (int8_t)-1;
    pos += snprintf(out + pos, sizeof(out) - pos,
      "%s{\"idx\":%d,\"res\":%d,\"file\":\"%s\"}",
      first ? "" : ",", idx, res, frameFileByIdx[idx]);
    first = false;
  }
  if (pos < (int)sizeof(out) - 4) {
    pos += snprintf(out + pos, sizeof(out) - pos, "]}");
  }
  return httpd_resp_send(req, out, pos);
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

  // Long sweep: take SD mutex for the whole walk (60s timeout).
  if (!sdMutex || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(60000)) != pdTRUE) {
    httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SD busy\"}");
    return ESP_OK;
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
  xSemaphoreGive(sdMutex);
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
  if (!sdMutex || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
    httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SD busy\"}");
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
  xSemaphoreGive(sdMutex);
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
  // URL-decode %XX sequences in path (JS encodeURIComponent encodes '/' as %2F)
  char decoded[96];
  {
    int di = 0;
    for (int si = 0; filepath[si] && di < (int)sizeof(decoded) - 1; si++) {
      if (filepath[si] == '%' && filepath[si+1] && filepath[si+2]) {
        auto hex = [](char c) -> int {
          if (c >= '0' && c <= '9') return c - '0';
          if (c >= 'a' && c <= 'f') return 10 + c - 'a';
          if (c >= 'A' && c <= 'F') return 10 + c - 'A';
          return -1;
        };
        int h1 = hex(filepath[si+1]);
        int h2 = hex(filepath[si+2]);
        if (h1 >= 0 && h2 >= 0) {
          decoded[di++] = (char)((h1 << 4) | h2);
          si += 2;
          continue;
        }
      }
      decoded[di++] = filepath[si];
    }
    decoded[di] = '\0';
  }
  // Ensure path starts with /
  char fullpath[128];
  if (decoded[0] == '/') {
    snprintf(fullpath, sizeof(fullpath), "%s", decoded);
  } else {
    snprintf(fullpath, sizeof(fullpath), "/%s", decoded);
  }
  // SD_MMC is single-threaded — serialize across httpd worker tasks.
  // Generous timeout because parallel browser fetches queue up here.
  if (!sdMutex || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD busy");
    return ESP_FAIL;
  }
  File f = SD_MMC.open(fullpath);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    xSemaphoreGive(sdMutex);
    httpd_resp_send_404(req); return ESP_FAIL;
  }
  // Set content type based on extension
  const char *ct = "application/octet-stream";
  if (strstr(fullpath, ".jpg") || strstr(fullpath, ".jpeg")) ct = "image/jpeg";
  else if (strstr(fullpath, ".json")) ct = "application/json";
  httpd_resp_set_type(req, ct);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  // SD files are immutable (each burst dir is written once). Cache for a year
  // so the browser only fetches each frame once per session/device.
  httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=31536000, immutable");
  // 8 KB stack buffer: fewer syscalls than the old 4 KB, no PSRAM alloc churn.
  uint8_t buf[8192];
  size_t n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    if (httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) break;
  }
  f.close();
  xSemaphoreGive(sdMutex);
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
  char buf[260];
  for (int i = 0; i < eventCount; i++) {
    EventEntry &e = eventLog[i];
#ifdef PREY_V2_URL
    static const char *catNames[] = {"unknown", "mazge", "benis", "other"};
    int catBits = (e.v2Flags >> 2) & 0x03;
    int v2Det = (e.v2Flags & 0x01) ? 1 : 0;
    int v2CatRec = (e.v2Flags & 0x02) ? 1 : 0;
    int len = snprintf(buf, sizeof(buf),
      "%s{\"t\":%lu,\"epoch\":%ld,\"ago\":%lu,\"gen\":%d,\"nf\":%d,\"res\":%d,\"dMin\":%d,\"dMax\":%d,\"mode\":%d,\"trend\":%d,\"lat\":%u,"
      "\"v2\":{\"maxPrey\":%.3f,\"okFrames\":%u,\"det\":%d,\"catRec\":%d,\"catId\":\"%s\"}}",
      i > 0 ? "," : "",
      e.uptimeMs, (long)e.epochSec, (nowMs > e.uptimeMs) ? (nowMs - e.uptimeMs) : 0,
      e.gen, e.frameCount, e.result, e.distMin, e.distMax, e.mode, e.trend, (unsigned)e.latencyMs,
      e.v2MaxPreyX1000 / 1000.0f, (unsigned)e.v2OkFrames,
      v2Det, v2CatRec, catNames[catBits]);
#else
    int len = snprintf(buf, sizeof(buf),
      "%s{\"t\":%lu,\"epoch\":%ld,\"ago\":%lu,\"gen\":%d,\"nf\":%d,\"res\":%d,\"dMin\":%d,\"dMax\":%d,\"mode\":%d,\"trend\":%d,\"lat\":%u}",
      i > 0 ? "," : "",
      e.uptimeMs, (long)e.epochSec, (nowMs > e.uptimeMs) ? (nowMs - e.uptimeMs) : 0,
      e.gen, e.frameCount, e.result, e.distMin, e.distMax, e.mode, e.trend, (unsigned)e.latencyMs);
#endif
    httpd_resp_send_chunk(req, buf, len);
  }
  httpd_resp_send_chunk(req, "]}", 2);
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

// POST /setevents — overwrite NVS event log with binary payload.
// Body = packed array of EventEntry structs (18 bytes each).
// Up to MAX_EVENTS entries; extra are ignored.
static esp_err_t setevents_handler(httpd_req_t *req) {
  int contentLen = req->content_len;
  const int entrySize = (int)sizeof(EventEntry);
  if (contentLen <= 0 || contentLen % entrySize != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body size");
    return ESP_FAIL;
  }
  int nEntries = contentLen / entrySize;
  if (nEntries > MAX_EVENTS) nEntries = MAX_EVENTS;

  EventEntry tmp[MAX_EVENTS];
  int wantBytes = nEntries * entrySize;
  int received = 0;
  while (received < wantBytes) {
    int r = httpd_req_recv(req, (char *)tmp + received, wantBytes - received);
    if (r <= 0) {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
      return ESP_FAIL;
    }
    received += r;
  }
  // Drain any tail bytes (in case more than MAX_EVENTS sent)
  char drain[64];
  int leftover = contentLen - wantBytes;
  while (leftover > 0) {
    int r = httpd_req_recv(req, drain, leftover > (int)sizeof(drain) ? (int)sizeof(drain) : leftover);
    if (r <= 0) break;
    leftover -= r;
  }
  memcpy(eventLog, tmp, wantBytes);
  eventCount = nEntries;
  saveEventLog();
  char resp[64];
  int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"count\":%d}", eventCount);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, resp, n);
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
    "\"triggerMm\":%d,\"cooldownMs\":%d,\"autoBaseAec\":%d,\"hdrSteps\":%s}",
    aecMax, aecLow, dayLuxThreshold, nightLuxThreshold,
    dayGainCap, dayExposureDiv, dayMinExposure, nightExposureCap,
    nightAecThreshold, nightGainCap,
    burstTriggerMm, burstCooldownMs, autoBaseAec, hdrBuf);
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
  Serial.printf("Settings updated: nightAecThr=%d nightExpCap=%d nightGainCap=%d triggerMm=%d cooldown=%d autoBaseAec=%d\n",
    nightAecThreshold, nightExposureCap, nightGainCap, burstTriggerMm, burstCooldownMs, autoBaseAec);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "OK", 2);
}

const char CONTROLS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Camera Controls</title>
  <style>
    * { box-sizing: border-box; }
    body { background: #111; color: #eee; font-family: sans-serif; margin: 0; padding: 12px; max-width: 700px; margin-left: auto; margin-right: auto; }
    h1 { margin: 0 0 6px; font-size: 1.25em; }
    a { color: #8af; }
    .panel { background: #1a1a2e; border: 1px solid #2e3750; border-radius: 8px; padding: 12px; }
    .controls { display: grid; grid-template-columns: 120px 1fr 52px;
                gap: 6px 8px; align-items: center; width: 100%; font-size: 0.9em; }
    .controls label { text-align: right; color: #aaa; }
    .controls select, .controls input[type=range] { width: 100%; }
    .controls .val { color: #6f6; font-family: monospace; }
    .hint { color: #888; font-size: 0.83em; margin: 0 0 10px; }
    #status { margin-top: 10px; color: #8ac; font-size: 0.85em; font-family: monospace; }
  </style>
</head>
<body>
  <h1>Camera Controls</h1>
  <p class="hint"><a href="/">Back to Live View</a> | Camera tuning controls moved off the main page.</p>
  <div class="panel">
    <div class="controls">
      <label>Quality</label>
      <input type="range" id="quality" min="10" max="100" value="95">
      <span class="val" id="quality-val">95</span>

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
    <div id="status">Ready</div>
  </div>

  <script>
    async function cmd(k, v) {
      try {
        await fetch('/cmd?' + k + '=' + v);
        const st = document.getElementById('status');
        st.textContent = 'Updated ' + k + '=' + v + ' at ' + new Date().toLocaleTimeString();
      } catch(e) {}
    }

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

static esp_err_t controls_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, CONTROLS_HTML, strlen(CONTROLS_HTML));
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
    <div class="row"><label>Burst Cooldown</label><input type="number" id="cooldownMs"><span class="unit">ms</span></div>
  </div>

  <div>
    <button class="save-btn" onclick="saveSettings()">💾 Save</button>
    <button class="reset-btn" onclick="loadSettings()">↺ Reload</button>
  </div>
  <div id="status"></div>

  <script>
    const fields = ['aecMax','aecLow','nightAecThr','nightExpCap','nightGainCap','triggerMm','cooldownMs'];
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
  config.max_uri_handlers = 26;

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

  httpd_uri_t burstmeta_uri = {
    .uri = "/burstmeta",
    .method = HTTP_GET,
    .handler = burstmeta_handler,
    .user_ctx = NULL
  };

  httpd_uri_t diag_uri = {
    .uri = "/diag",
    .method = HTTP_GET,
    .handler = diag_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&ui_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(ui_httpd, &index_uri);
    httpd_register_uri_handler(ui_httpd, &stats_uri);
    httpd_register_uri_handler(ui_httpd, &cmd_uri);
    httpd_register_uri_handler(ui_httpd, &burstmeta_uri);
    httpd_register_uri_handler(ui_httpd, &diag_uri);
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
    httpd_uri_t prey24h_uri = { .uri = "/burstinfo", .method = HTTP_GET, .handler = burstinfo_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &prey24h_uri);
    httpd_uri_t raminfo_uri = { .uri = "/raminfo", .method = HTTP_GET, .handler = raminfo_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &raminfo_uri);
    httpd_uri_t ramburst_uri = { .uri = "/ramburst", .method = HTTP_GET, .handler = ramburst_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &ramburst_uri);
    httpd_uri_t controls_uri = { .uri = "/controls", .method = HTTP_GET, .handler = controls_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &controls_uri);
    httpd_uri_t settings_uri = { .uri = "/settings", .method = HTTP_GET, .handler = settings_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &settings_uri);
    httpd_uri_t getsettings_uri = { .uri = "/getsettings", .method = HTTP_GET, .handler = getsettings_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &getsettings_uri);
    httpd_uri_t setsettings_uri = { .uri = "/setsettings", .method = HTTP_GET, .handler = setsettings_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &setsettings_uri);
    httpd_uri_t getevents_uri = { .uri = "/getevents", .method = HTTP_GET, .handler = getevents_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &getevents_uri);
    httpd_uri_t setevents_uri = { .uri = "/setevents", .method = HTTP_POST, .handler = setevents_handler, .user_ctx = NULL };
    httpd_register_uri_handler(ui_httpd, &setevents_uri);
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

  // Log the reason for this boot. Helps correlate crashes with bursts/API calls.
  // Codes: 1=POWERON 2=EXT 3=SW 4=PANIC 5=INT_WDT 6=TASK_WDT 7=WDT 8=DEEPSLEEP 9=BROWNOUT 10=SDIO
  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char *name = "?";
    switch (rr) {
      case ESP_RST_POWERON:   name = "POWERON";  break;
      case ESP_RST_EXT:       name = "EXT";      break;
      case ESP_RST_SW:        name = "SW";       break;
      case ESP_RST_PANIC:     name = "PANIC";    break;
      case ESP_RST_INT_WDT:   name = "INT_WDT";  break;
      case ESP_RST_TASK_WDT:  name = "TASK_WDT"; break;
      case ESP_RST_WDT:       name = "WDT";      break;
      case ESP_RST_DEEPSLEEP: name = "DEEPSLEEP";break;
      case ESP_RST_BROWNOUT:  name = "BROWNOUT"; break;
      case ESP_RST_SDIO:      name = "SDIO";     break;
      default: break;
    }
    Serial.printf("Reset reason: %s (%d)\n", name, (int)rr);
    // Persist a counter and last reason so we can correlate post-mortem.
    nvsPrefs.begin("state", false);
    uint32_t bootCount = nvsPrefs.getUInt("bootCount", 0) + 1;
    nvsPrefs.putUInt("bootCount", bootCount);
    nvsPrefs.putUInt("lastRR", (uint32_t)rr);
    nvsPrefs.end();
    bootResetReason = (uint32_t)rr;
    bootCounter = bootCount;
    Serial.printf("Boot #%u (last reset reason persisted)\n", (unsigned)bootCount);
  }

  // Initialize door control IMMEDIATELY: drive LOW (CLOSED) before pinMode
  // so any boot glitch / crash recovery leaves the door SAFE-closed rather
  // than open. The lock is one-way (exit is always free) so this only blocks
  // entry until the firmware has time to evaluate the next ToF trigger.
  digitalWrite(DOOR_PIN, LOW);
  pinMode(DOOR_PIN, OUTPUT);
  digitalWrite(DOOR_PIN, LOW);
  doorOpen = false;
  Serial.println("Door: closed (boot default — normally-closed entry side)");

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

  // SD card init: try 1-bit @ 40 MHz (high-speed) first, fall back to 20 MHz.
  // Signature: begin(mountpoint, mode1bit, format_if_mount_failed, freqHz, maxOpenFiles)
  sdMutex = xSemaphoreCreateMutex();
  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);
  bool sdMounted = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_HIGHSPEED);
  unsigned sdFreqMhz = 40;
  if (!sdMounted) {
    Serial.println("SD card mount @40MHz failed, retrying @20MHz...");
    sdMounted = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT);
    sdFreqMhz = 20;
  }
  if (sdMounted) {
    sdReady = true;
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD card ready: %llu MB, type %d, %u MHz\n", cardSize, SD_MMC.cardType(), sdFreqMhz);
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

    // Restore any lockout that survived a crash/reboot.
    int32_t lockUntil = loadLockoutEpoch();
    if (lockUntil > (int32_t)now) {
      int32_t remainSec = lockUntil - (int32_t)now;
      preyLockoutUntilMs = millis() + (unsigned long)remainSec * 1000UL;
      if (preyLockoutUntilMs == 0) preyLockoutUntilMs = 1;
      Serial.printf("Lockout RESTORED from NVS: %ds remaining (until epoch %d)\n",
                    (int)remainSec, (int)lockUntil);
    } else if (lockUntil != 0) {
      Serial.printf("Lockout in NVS already expired (epoch %d, now %d) — clearing\n",
                    (int)lockUntil, (int)now);
      persistLockoutEpoch(0);
    }
  } else {
    Serial.println("\nNTP sync failed, using uptime");
    // Without NTP we can't know if the persisted lockout is still in window.
    // Be SAFE: if a lockout is persisted at all, restore the full long-lockout
    // window. Worst case we lock for 15 extra minutes after recovery — acceptable.
    int32_t lockUntil = loadLockoutEpoch();
    if (lockUntil != 0) {
      preyLockoutUntilMs = millis() + PREY_LONG_LOCKOUT_MS;
      if (preyLockoutUntilMs == 0) preyLockoutUntilMs = 1;
      Serial.println("Lockout RESTORED conservatively (no NTP): full 15 min");
    }
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

  // Hardware Task Watchdog — last line of defence against full lockups.
  // If loop() doesn't feed the WDT within LOOP_WDT_TIMEOUT_S seconds,
  // ESP-IDF performs a hard reset at hardware level (no software path
  // can prevent it). This is what prevents indefinite bricks like the
  // one we hit during testing.
  //
  // 60s is generous: a normal loop() iteration is <100ms; even worst-case
  // burst capture + SD save shouldn't block loop() for more than ~5s
  // because the heavy work runs in separate FreeRTOS tasks.
#define LOOP_WDT_TIMEOUT_S 60
  esp_task_wdt_init(LOOP_WDT_TIMEOUT_S, /*panic on timeout*/ true);
  esp_task_wdt_add(NULL);  // subscribe the current (loop / Arduino main) task
  Serial.printf("Task WDT armed: %ds\n", LOOP_WDT_TIMEOUT_S);
}

// ===== Loop =====
// FIX 1: Throttle ArduinoOTA.handle() to reduce mDNS polling overhead.
// ArduinoOTA.handle() calls mDNS internally, which sends/receives UDP
// multicast packets that compete with MJPEG TCP traffic on the WiFi radio.
// Calling it every 500ms instead of every ~1ms is sufficient for OTA discovery
// while dramatically reducing WiFi contention during streaming.
void loop() {
  unsigned long now = millis();

  // Feed the Task WDT first thing every loop iteration. If we ever fail to
  // reach this point within LOOP_WDT_TIMEOUT_S the chip hard-resets.
  esp_task_wdt_reset();

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

  // Auto-close after green-light window expires (normally-closed entry side).
  // Tracks the open->closed edge so we only call doorCloseNow() once.
  static bool greenLightWasActive = false;
  bool greenLightNow = greenLightActive();
  if (greenLightWasActive && !greenLightNow && doorOpen && !doorLockoutActive()) {
    doorCloseNow("green-light expired");
  }
  greenLightWasActive = greenLightNow;

  // Free idle worker TLS to reclaim ~120KB internal heap during quiet periods.
  // Trigger condition: no API task busy, no burst capturing, no green light or
  // lockout active, and >IDLE_TLS_FREE_MS since the last trigger. Re-init on
  // next burst costs ~1s of TLS handshake (per worker, in parallel) which is
  // hidden behind the BURST_SHIFT_MS pre-capture window.
#define IDLE_TLS_FREE_MS (90UL * 1000UL)
  static unsigned long lastTlsFreeCheckMs = 0;
  if (now - lastTlsFreeCheckMs > 5000) {  // check every 5s
    lastTlsFreeCheckMs = now;
    bool idle = !apiTaskBusy() && !burstCapturing && !greenLightNow &&
                !doorLockoutActive() && (postTriggerRemaining == 0) &&
                (pendingFreezeAtMs == 0);
    bool quiet = (lastBurstTriggerMs == 0) ||
                 ((now - lastBurstTriggerMs) > IDLE_TLS_FREE_MS);
    if (idle && quiet) {
      freeIdleWorkerTls();
    }
  }

  // Heartbeat: log key state once a minute so a serial capture can show
  // exactly how far the firmware got before any crash. Cheap (one Serial.printf).
  static unsigned long lastHeartbeat = 0;
  if (now - lastHeartbeat >= 60000UL) {
    lastHeartbeat = now;
    time_t hbEpoch;
    time(&hbEpoch);
    uint32_t freeHeap   = ESP.getFreeHeap();
    uint32_t minHeap    = ESP.getMinFreeHeap();
    uint32_t freePsram  = ESP.getFreePsram();
    int rssi            = WiFi.RSSI();
    long lockRemainS    = doorLockoutActive() ? (long)((preyLockoutUntilMs - now) / 1000UL) : 0;
    long greenRemainS   = greenLightActive() ? (long)((greenLightUntilMs - now) / 1000UL) : 0;
    // Worker stack high-water (smallest stack free observed so far per worker)
    UBaseType_t w0 = apiWorkers[0].task ? uxTaskGetStackHighWaterMark(apiWorkers[0].task) : 0;
    UBaseType_t w1 = (N_API_WORKERS > 1 && apiWorkers[N_API_WORKERS > 1 ? 1 : 0].task)
                       ? uxTaskGetStackHighWaterMark(apiWorkers[N_API_WORKERS > 1 ? 1 : 0].task) : 0;
    Serial.printf("HB up=%lus epoch=%ld door=%s lock=%lds green=%lds "
                  "heap=%u/min=%u psram=%u rssi=%ddBm wifi=%d blynk=%d "
                  "apiBusy=%d burst=%d abandons=%u boot=#%u rr=%u wHW=%u/%u\n",
                  now / 1000UL, (long)hbEpoch, doorOpen ? "OPEN" : "CLOSED",
                  lockRemainS, greenRemainS,
                  (unsigned)freeHeap, (unsigned)minHeap, (unsigned)freePsram, rssi,
                  (int)WiFi.isConnected(), (int)Blynk.connected(),
                  (int)apiTaskBusy(), (int)burstCapturing,
                  (unsigned)apiAbandonCount, (unsigned)bootCounter,
                  (unsigned)bootResetReason, (unsigned)w0, (unsigned)w1);
  }

  // Blynk: V1 — reset prey frame count to 0 when lockout expires
  if (blynkLockoutWasActive && !doorLockoutActive()) {
    blynkLockoutWasActive = false;
    if (Blynk.connected()) Blynk.virtualWrite(V1, 0);
    blynkLockoutPreyFrames = 0;
    // Lockout naturally expired — clear the persisted NVS entry too.
    if (preyLockoutUntilEpoch != 0) persistLockoutEpoch(0);
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
      // Pre-warm worker TLS NOW: BURST_SHIFT_MS (200ms) + capture (~1s) gives
      // the workers ~1.2s to complete a TLS handshake before the first API
      // POST is issued. Hides the freeIdleWorkerTls cost on cold bursts and
      // restores pre-2026-05-21 API latency (~3-4s/burst instead of 5-6s).
      for (int i = 0; i < N_API_WORKERS; i++) {
        if (!apiWorkers[i].connected) apiWorkers[i].prewarmRequested = true;
      }
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

  // === TLS pre-connect: keep connection warm so first API call is fast ===
  static unsigned long lastTlsCheck = 0;
  if (!burstCapturing && now - lastTlsCheck >= 10000) {
    lastTlsCheck = now;
    if (!tlsConnected) {
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
      // Burst-shift: cat is moving under the IR illuminator.
      // Time-based exposure ramp-down: more reliable than ToF distance which
      // can be noisy during fast cat passage.  Cat moves from trigger distance
      // (~480mm) to very close (~150mm) in ~200ms, so IR intensity can increase
      // ~10x.  Ramp BOTH AEC and gain down over the shift window.
      unsigned long triggerMs = pendingFreezeAtMs - BURST_SHIFT_MS;
      unsigned long elapsed = (now > triggerMs) ? (now - triggerMs) : 0;
      if (elapsed > BURST_SHIFT_MS) elapsed = BURST_SHIFT_MS;
      int gain = frozenGain;
      int aec;
      if (autoBaseAec > nightAecThreshold) {
        // Night/IR: ramp AEC from frozenAec → frozenAec/5, gain → 0
        long frac1000 = (long)elapsed * 1000 / BURST_SHIFT_MS;  // 0→1000
        // AEC: scale 1000 → 200 (100% → 20% = 1/5)
        long scale = 1000 - frac1000 * 800 / 1000;  // 1000→200
        aec = (int)(frozenAec * scale / 1000);
        // Gain: ramp from nightGainCap → 0 (minimum analog gain)
        gain = (int)(nightGainCap * (1000 - frac1000) / 1000);
      } else {
        // Day mode: gentle halve, same as before
        aec = frozenAec / 2;
      }
      if (aec < 4) aec = 4;
      if (gain < 0) gain = 0;
      s->set_agc_gain(s, gain);
      s->set_aec_value(s, aec);
      appliedGain = gain;
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
        xTaskCreatePinnedToCore(apiCheckTask, "apiCheck",
          16384, (void *)(intptr_t)archIdx,
          tskIDLE_PRIORITY + 3, NULL, 0);
      }
    }
  }
}
