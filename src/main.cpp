#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "img_converters.h"
#include <lwip/sockets.h>

// ===== WiFi credentials =====
const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASS = "YOUR_WIFI_PASS";

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
volatile bool useBinning = true; // YUV→bin mode
volatile int jpegQuality = 12;

// Pre-allocated binning buffer in PSRAM (400*300 = 120000 bytes)
uint8_t *binBuf = NULL;

// ===== 2x2 binning: extract Y from YUV422 and average 4 pixels =====
// YUV422 layout: Y0 U0 Y1 V0 Y2 U1 Y3 V1 ...
// Y is at byte offsets 0, 2, 4, 6 ... (every even byte)
void bin2x2_yuv422(const uint8_t *src, int srcW, int srcH, uint8_t *dst) {
  int dstW = srcW / 2;
  int srcStride = srcW * 2;  // 2 bytes per pixel in YUV422
  for (int y = 0; y < srcH; y += 2) {
    const uint8_t *row0 = src + y * srcStride;
    const uint8_t *row1 = row0 + srcStride;
    uint8_t *dstRow = dst + (y / 2) * dstW;
    for (int x = 0; x < srcW; x += 2) {
      // Average 4 Y values from the 2x2 block
      uint16_t sum = row0[x * 2] + row0[(x + 1) * 2]
                   + row1[x * 2] + row1[(x + 1) * 2];
      dstRow[x / 2] = sum >> 2;
    }
  }
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
  <img id="stream" src="" />
  <div class="controls">
    <label>Binning</label>
    <select id="binning">
      <option value="0">Off (VGA 640x480)</option>
      <option value="1" selected>On (SVGA→2x2 400x300)</option>
    </select><span></span>

    <label>JPEG Quality</label>
    <input type="range" id="quality" min="4" max="63" value="12">
    <span class="val" id="quality-val">12</span>

    <label>FPS Cap</label>
    <input type="range" id="fps" min="1" max="30" value="15">
    <span class="val" id="fps-val">15</span>

    <label>Brightness</label>
    <input type="range" id="brightness" min="-2" max="2" value="0">
    <span class="val" id="brightness-val">0</span>

    <label>Contrast</label>
    <input type="range" id="contrast" min="-2" max="2" value="0">
    <span class="val" id="contrast-val">0</span>

    <label>Saturation</label>
    <input type="range" id="saturation" min="-2" max="2" value="0">
    <span class="val" id="saturation-val">0</span>

    <label>Greyscale</label>
    <select id="greyscale">
      <option value="0">Off</option>
      <option value="1" selected>On</option>
    </select><span></span>

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
    document.getElementById('stream').src = 'http://' + location.hostname + ':81/stream';

    // Stats polling
    setInterval(async () => {
      try {
        const r = await fetch('/stats');
        const s = await r.json();
        document.getElementById('stats').textContent =
          `FPS: ${s.fps.toFixed(1)} | Frame: ${(s.frameBytes/1024).toFixed(1)} KB | Send: ${s.frameMs} ms | Total: ${s.totalFrames}`;
      } catch(e) {}
    }, 1000);

    // Send command helper
    async function cmd(k, v) {
      try { await fetch(`/cmd?${k}=${v}`); } catch(e) {}
    }

    // Wire up controls
    for (const id of ['quality','fps','brightness','contrast','saturation']) {
      const el = document.getElementById(id);
      const valEl = document.getElementById(id + '-val');
      el.addEventListener('input', () => { valEl.textContent = el.value; });
      el.addEventListener('change', () => { cmd(id, el.value); });
    }
    document.getElementById('greyscale').addEventListener('change', function() {
      cmd('greyscale', this.value);
    });
    document.getElementById('binning').addEventListener('change', function() {
      cmd('binning', this.value);
      // Stream needs to restart after camera reinit
      setTimeout(() => {
        document.getElementById('stream').src = 'http://' + location.hostname + ':81/stream?t=' + Date.now();
      }, 1500);
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
bool initCamera(bool binning) {
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

  if (binning) {
    // YUV422 at SVGA — we'll extract Y and 2x2 bin to 400x300
    config.pixel_format = PIXFORMAT_YUV422;
    config.frame_size   = FRAMESIZE_SVGA;   // 800x600
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12; // unused for YUV but required
  } else {
    // Hardware JPEG at VGA
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = jpegQuality;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  if (!binning) {
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_special_effect(s, 2); // greyscale
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

    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool needFreeJpg = false;

    if (fb->format == PIXFORMAT_JPEG) {
      // Direct JPEG from camera
      jpg_buf = fb->buf;
      jpg_len = fb->len;
    } else {
      // YUV422 → extract Y + 2x2 bin → JPEG
      bin2x2_yuv422(fb->buf, fb->width, fb->height, binBuf);
      int outW = fb->width / 2;
      int outH = fb->height / 2;
      if (fmt2jpg(binBuf, outW * outH, outW, outH, PIXFORMAT_GRAYSCALE,
                  jpegQuality, &jpg_buf, &jpg_len)) {
        needFreeJpg = true;
      } else {
        Serial.println("JPEG encode failed");
        esp_camera_fb_return(fb);
        fb = NULL;
        continue;
      }
    }

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

    if (needFreeJpg) free(jpg_buf);
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

static esp_err_t stats_handler(httpd_req_t *req) {
  char json[128];
  snprintf(json, sizeof(json),
    "{\"fps\":%.1f,\"frameBytes\":%u,\"frameMs\":%u,\"totalFrames\":%u}",
    streamFps, lastFrameBytes, lastFrameMs, frameCount);
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
    int q = atoi(val);
    jpegQuality = q;
    s->set_quality(s, q);
  } else if (httpd_query_key_value(buf, "framesize", val, sizeof(val)) == ESP_OK) {
    if (!useBinning) s->set_framesize(s, (framesize_t)atoi(val));
  } else if (httpd_query_key_value(buf, "brightness", val, sizeof(val)) == ESP_OK) {
    s->set_brightness(s, atoi(val));
  } else if (httpd_query_key_value(buf, "contrast", val, sizeof(val)) == ESP_OK) {
    s->set_contrast(s, atoi(val));
  } else if (httpd_query_key_value(buf, "saturation", val, sizeof(val)) == ESP_OK) {
    s->set_saturation(s, atoi(val));
  } else if (httpd_query_key_value(buf, "greyscale", val, sizeof(val)) == ESP_OK) {
    s->set_special_effect(s, atoi(val) ? 2 : 0);
  } else if (httpd_query_key_value(buf, "fps", val, sizeof(val)) == ESP_OK) {
    targetFps = atoi(val);
  } else if (httpd_query_key_value(buf, "binning", val, sizeof(val)) == ESP_OK) {
    bool want = atoi(val) != 0;
    if (want != useBinning) {
      useBinning = want;
      esp_camera_deinit();
      initCamera(useBinning);
      Serial.printf("Camera reinit: binning=%d\n", useBinning);
    }
  } else if (httpd_query_key_value(buf, "gainceiling", val, sizeof(val)) == ESP_OK) {
    s->set_gainceiling(s, (gainceiling_t)atoi(val));
  } else if (httpd_query_key_value(buf, "nightmode", val, sizeof(val)) == ESP_OK) {
    s->set_aec2(s, atoi(val));
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

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Stream server started on port 81");
  }
}

void startUIServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.stack_size = 4096;
  config.max_uri_handlers = 8;

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

  if (httpd_start(&ui_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(ui_httpd, &index_uri);
    httpd_register_uri_handler(ui_httpd, &stats_uri);
    httpd_register_uri_handler(ui_httpd, &cmd_uri);
    Serial.println("UI server started on port 80");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!initCamera(useBinning)) {
    Serial.println("Camera init failed \u2013 restarting in 5 s");
    delay(5000);
    ESP.restart();
  }

  // Allocate binning buffer in PSRAM (400*300 greyscale)
  binBuf = (uint8_t *)ps_malloc(400 * 300);
  if (!binBuf) {
    Serial.println("Failed to allocate binning buffer");
  }

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

  // OTA updates
  ArduinoOTA.setHostname("esp32cam");
  ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA: %u%%\r", progress / (total / 100));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");

  // FIX 4: Both servers now use esp_http_server (own RTOS tasks)
  startUIServer();
  startStreamServer();
}

// ===== Loop =====
// FIX 1: Throttle ArduinoOTA.handle() to reduce mDNS polling overhead.
// ArduinoOTA.handle() calls mDNS internally, which sends/receives UDP
// multicast packets that compete with MJPEG TCP traffic on the WiFi radio.
// Calling it every 500ms instead of every ~1ms is sufficient for OTA discovery
// while dramatically reducing WiFi contention during streaming.
void loop() {
  static unsigned long lastOTA = 0;
  unsigned long now = millis();
  if (now - lastOTA >= 500) {
    lastOTA = now;
    ArduinoOTA.handle();
  }
}
