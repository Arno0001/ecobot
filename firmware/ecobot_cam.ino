#include <Garbage-FOMO-Classifier_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"



#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_FRAME_BYTE_SIZE        3

#define CAM_TX 14
#define CAM_RX 15

#define WIFI_SSID  "Amigos"      // must match the Dev Board network
// password is case-sensitive: capital A
const char* WIFI_PASSES[] = { "Amigoslife12" };
const int WIFI_PASS_COUNT = 1;

#define CONFIDENCE_THRESHOLD  0.6     // GARBAGE_DETECTED / classify
#define SCAN_THRESHOLD        0.75    // stricter trigger for GARBAGE_DETECTED (fewer false alarms)
#define SCAN_CONFIRM_FRAMES   2       // consecutive frames above SCAN_THRESHOLD before GARBAGE_DETECTED
#define TRACK_THRESHOLD       0.45    // lower while tracking so we do not lose the target
#define ROI_MIN_Y_PCT         30      // ignore boxes whose centre is above this % of the image (false hits at the top). 0 = off
#define CLASSIFY_THRESHOLD    0.40    // BIO/NON_BIO decision at the claw (object is very close, scores lower than at scan range)
#define CLASSIFY_SAMPLES      5       // frames examined for the final classification (best one wins)
#define SCAN_VIEW_MIN         0.30    // scan runs at this low floor so the log shows the REAL score; GARBAGE_DETECTED still needs SCAN_THRESHOLD
#define CHECK_THRESHOLD       0.30    // obstacle check: LOW threshold, whole image (garbage close/partly visible scores low)
#define CHECK_FRAMES          5       // frames examined for an obstacle check (1 hit is enough)
#define CHECK_FRAME_GAP_MS    120     // small pause between check frames (fresh image each time)
#define ALIVE_INTERVAL_MS     2000
#define SCAN_LOG_INTERVAL_MS  1000
#define BUSY_TIMEOUT_MS       40000   // safety: auto-resume if READY never arrives
#define TRACK_TIMEOUT_MS      25000   // safety: leave TRACK mode if Dev Board disappears

static bool is_initialised = false;
static bool isBusy         = false;
static bool tracking       = false;
static unsigned long busySince  = 0;
static unsigned long trackSince = 0;
uint8_t *snapshot_buf;
httpd_handle_t stream_httpd = NULL;

struct DetectionResult {
    String label;
    float confidence;
    int x, y, w, h;         
};

//  on-device latency measurement 
static uint32_t lastInfMs   = 0;   
static uint32_t lastFrameMs = 0;   
static uint32_t latN = 0, latMin = 0xFFFFFFFF, latMax = 0;
static double   latSum = 0, latSumSq = 0;

static void latRecord(uint32_t ms) {
    latN++; latSum += ms; latSumSq += (double)ms * ms;
    if (ms < latMin) latMin = ms;
    if (ms > latMax) latMax = ms;
    if (latN % 50 == 0) {                            
        double mean = latSum / latN;
        double var  = latN > 1 ? (latSumSq - latN * mean * mean) / (latN - 1) : 0;
        if (var < 0) var = 0;
        Serial.printf("[LAT] n=%u mean=%.1f ms sd=%.1f min=%u max=%u (classifier only)\n",
                      (unsigned)latN, mean, sqrt(var), (unsigned)latMin, (unsigned)latMax);
    }
}

static camera_config_t camera_config = {
    .pin_pwdn     = PWDN_GPIO_NUM,
    .pin_reset    = RESET_GPIO_NUM,
    .pin_xclk     = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href  = HREF_GPIO_NUM,
    .pin_pclk  = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count     = 1,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
};

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];
    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            size_t hlen = snprintf(part_buf, 64, STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK)
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;
    }
    return res;
}

void startMJPEGStream() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 81;
    httpd_uri_t stream_uri = {
        .uri     = "/stream",
        .method  = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("[STREAM] Started on port 81");
        Serial.print("[STREAM] URL: http://");
        Serial.print(WiFi.localIP());
        Serial.println(":81/stream");
    }
}

bool ei_camera_init(void) {
    if (is_initialised) return true;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }
    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height,
                       uint8_t *out_buf) {
    if (!is_initialised) return false;
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return false;
    bool converted = fmt2rgb888(fb->buf, fb->len,
                                PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if (!converted) return false;
    if (img_width  != EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
        img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf, img_width, img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length,
                               float *out_ptr) {
    size_t pixel_ix    = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix  = 0;
    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] =
            (snapshot_buf[pixel_ix + 2] << 16) +
            (snapshot_buf[pixel_ix + 1] << 8)  +
             snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}


static float sdkThreshold = -1;
static void setSdkThreshold(float th) {
    if (th < 0.20f) th = 0.20f;
    if (th > 0.95f) th = 0.95f;
    if (th == sdkThreshold) return;
    EI_IMPULSE_ERROR e = set_threshold_postprocessing(
        &ei_default_impulse.impulse->postprocessing_blocks[0], "min_score", th);
    if (e == EI_IMPULSE_OK) sdkThreshold = th;
    else Serial.printf("[CAM] could not set model threshold (err %d)\n", e);
}


DetectionResult runDetection(float threshold = CONFIDENCE_THRESHOLD, bool nearest = false, bool useRoi = true) {
    DetectionResult res;
    res.label      = "background";
    res.confidence = 0;
    res.x = res.y = res.w = res.h = 0;

    if (!snapshot_buf) {                       
        Serial.println("[CAM] no frame buffer (enable PSRAM / board = AI Thinker ESP32-CAM)");
        return res;
    }

    ei::signal_t signal;
    signal.total_length =
        EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    unsigned long frameT0 = millis();
    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH,
                           EI_CLASSIFIER_INPUT_HEIGHT,
                           snapshot_buf)) {
        Serial.println("[CAM] capture failed");
        return res;
    }

    setSdkThreshold(threshold);                
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("[CAM] classifier error %d\n", err);
        return res;
    }
    lastFrameMs = millis() - frameT0;
    lastInfMs   = result.timing.dsp + result.timing.classification;
    latRecord(lastInfMs);

    float best_val = threshold;
    float bestCy = -1;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value <= threshold || String(bb.label) == "background") continue;
        float cyPct = (bb.y + bb.height / 2.0f) * 100.0f / EI_CLASSIFIER_INPUT_HEIGHT;
        if (useRoi && cyPct < ROI_MIN_Y_PCT) continue;       // outside floor region of interest
        bool take;
        if (nearest) {                       // TRACK: pick the box closest to the robot (lowest in image)
            float cyb = bb.y + bb.height / 2.0f;
            take = (cyb > bestCy);
            if (take) bestCy = cyb;
        } else {                             // SCAN / CLASSIFY: pick highest confidence
            take = (bb.value > best_val);
            if (take) best_val = bb.value;
        }
        if (take) {
            res.label      = String(bb.label);
            res.confidence = bb.value;
            res.x = bb.x; res.y = bb.y;
            res.w = bb.width; res.h = bb.height;
        }
    }
    return res;
}


void sendTarget(const DetectionResult &r) {
    int cx = (int)((r.x + r.w / 2.0f) * 100.0f / EI_CLASSIFIER_INPUT_WIDTH);
    int cy = (int)((r.y + r.h / 2.0f) * 100.0f / EI_CLASSIFIER_INPUT_HEIGHT);
    int w  = r.w * 100 / EI_CLASSIFIER_INPUT_WIDTH;
    int h  = r.h * 100 / EI_CLASSIFIER_INPUT_HEIGHT;
    Serial1.printf("TGT,%d,%d,%d,%d,%.2f\n", cx, cy, w, h, r.confidence);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("[CAM] Booting...");

    Serial1.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
    Serial1.setTimeout(100);

    
    size_t fbSize = EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3;
    snapshot_buf = (uint8_t*)ps_malloc(fbSize);
    if (!snapshot_buf) snapshot_buf = (uint8_t*)malloc(fbSize);
    Serial.printf("[CAM] PSRAM %s, free heap %u, free PSRAM %u, frame buffer %s\n",
                  psramFound() ? "FOUND" : "NOT FOUND (select AI Thinker ESP32-CAM board!)",
                  ESP.getFreeHeap(), ESP.getFreePsram(), snapshot_buf ? "OK" : "FAILED");

    if (!ei_camera_init()) {
        Serial.println("[CAM] Camera init failed! Check ribbon cable.");
    } else {
        Serial.println("[CAM] Camera ready");
    }

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    int wifiIdx = 0;
    for (int i = 0; i < WIFI_PASS_COUNT && WiFi.status() != WL_CONNECTED; i++) {
        wifiIdx = i;
        if (i > 0) WiFi.disconnect(false);
        WiFi.begin(WIFI_SSID, WIFI_PASSES[i]);
        Serial.printf("[WiFi] Connecting (password #%d)", i + 1);
        unsigned long wifiStart = millis();
        while (WiFi.status() != WL_CONNECTED &&
               millis() - wifiStart < 12000) {
            delay(500); Serial.print(".");
        }
        Serial.println();
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Connected! password #%d worked\n", wifiIdx + 1);
        Serial.print("[WiFi] CAM IP: ");
        Serial.println(WiFi.localIP());
        startMJPEGStream();
    } else {
        Serial.println("\n[WiFi] Failed — detection still runs, no stream");
    }

    Serial.println("[CAM] Ready — scanning floor continuously");
}

void loop() {
    static unsigned long lastAlive = 0;
    static unsigned long lastScanLog = 0;

    // Commands from Dev Board
    while (Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) continue;
        Serial.println("[CMD] " + cmd);

        if (cmd == "BUSY") {
            isBusy = true;
            tracking = false;
            busySince = millis();
            Serial.println("[CAM] Paused");

        } else if (cmd == "TRACK") {
            tracking = true;
            isBusy = false;
            trackSince = millis();
            Serial.println("[CAM] TRACK mode (sending TGT)");

        } else if (cmd == "READY") {
            isBusy = false;
            tracking = false;
            Serial.println("[CAM] Resumed");

        } else if (cmd.startsWith("CHECK")) { // Dev Board stopped at an obstacle: is it garbage?  ("CHECK" or "CHECK,<id>")
            tracking = false;
            String tag = "";                  // echo the request id back so the Dev Board can drop stale replies
            int comma = cmd.indexOf(',');
            if (comma > 0) tag = "," + cmd.substring(comma + 1);
            bool found = false;
            for (int i = 0; i < CHECK_FRAMES && !found; i++) {
                DetectionResult r = runDetection(CHECK_THRESHOLD, false, false);
                Serial.println("[CHECK] frame " + String(i + 1) + "/" + String(CHECK_FRAMES) + ": " +
                               r.label + " conf " + String(r.confidence) +
                               " x=" + String(r.x + r.w / 2) + " y=" + String(r.y + r.h / 2));
                if (r.label != "background" && r.confidence > 0) found = true;
                else delay(CHECK_FRAME_GAP_MS);
            }
            Serial1.println(String(found ? "CHECK_GARBAGE" : "CHECK_CLEAR") + tag);
            Serial.println(found ? "[CAM] CHECK -> garbage" : "[CAM] CHECK -> clear");

        } else if (cmd == "CLASSIFY") {
            tracking = false;
            Serial.println("[CAM] Classifying (" + String(CLASSIFY_SAMPLES) + " samples)...");
            String output = "UNKNOWN";
            float bestConf = 0;
            for (int i = 0; i < CLASSIFY_SAMPLES; i++) {
                DetectionResult r = runDetection(CLASSIFY_THRESHOLD, false, false);
                Serial.println("[CLASSIFY] sample " + String(i + 1) + "/" + String(CLASSIFY_SAMPLES) + ": " +
                               r.label + " conf " + String(r.confidence) +
                               "  [INF " + String(lastInfMs) + " ms, frame " + String(lastFrameMs) + " ms]");
                if (r.label != "background" && r.confidence > bestConf) {
                    bestConf = r.confidence;
                    if (r.label == "biodegradable")          output = "BIO";
                    else if (r.label == "non-biodegradable") output = "NON_BIO";
                }
                delay(150);
            }
            Serial1.println("E2INFO," + String(bestConf, 2) + "," + String(lastInfMs));   // Dev prints this -> Excel G (conf) and H (INF ms)
            Serial1.println(output);
            Serial.println("[CAM] Sent: " + output +
                           " (" + String(bestConf) + ")");
            isBusy = true;                 // hold until Dev Board sends READY
            busySince = millis();
        }
    }

    // Safety timeouts (if the Dev Board never sends READY / leaves TRACK)
    if (isBusy && millis() - busySince > BUSY_TIMEOUT_MS) {
        isBusy = false;
        Serial.println("[CAM] BUSY timeout -> resumed");
    }
    if (tracking && millis() - trackSince > TRACK_TIMEOUT_MS) {
        tracking = false;
        Serial.println("[CAM] TRACK timeout -> scan mode");
    }

    if (isBusy) { delay(50); return; }

    // Heartbeat so the Dev Board can confirm the UART link
    if (millis() - lastAlive >= ALIVE_INTERVAL_MS) {
        lastAlive = millis();
        Serial1.println("ALIVE");
    }

    // TRACK mode: report target position every frame 
    if (tracking) {
        DetectionResult t = runDetection(TRACK_THRESHOLD, true);
        if (t.label != "background" && t.confidence > 0) {
            sendTarget(t);
        }
        delay(20);
        return;
    }

    //SCAN mode: continuous floor scan 
    DetectionResult r = runDetection(SCAN_VIEW_MIN);
    static int scanHits = 0;

    
    if (millis() - lastScanLog >= SCAN_LOG_INTERVAL_MS) {
        lastScanLog = millis();
        int px = (int)((r.x + r.w / 2.0f) * 100.0f / EI_CLASSIFIER_INPUT_WIDTH);
        int py = (int)((r.y + r.h / 2.0f) * 100.0f / EI_CLASSIFIER_INPUT_HEIGHT);
        String note = "";
        if (r.confidence >= SCAN_THRESHOLD)  note = "  <- TRIGGER";
        else if (r.confidence > 0)           note = "  (below " + String(SCAN_THRESHOLD) + ")";
        Serial.println("[SCAN] " + r.label + " conf " + String(r.confidence) +
                       " x=" + String(px) + "% y=" + String(py) + "%" + note +
                       "  [INF " + String(lastInfMs) + " ms, frame " + String(lastFrameMs) + " ms]");
    }

    if (r.label != "background" && r.confidence >= SCAN_THRESHOLD) scanHits++;
    else scanHits = 0;

    if (scanHits == 1 && SCAN_CONFIRM_FRAMES > 1) {
        Serial1.println("MAYBE");          // first sighting: Dev Board stops the robot immediately
        Serial.println("[CAM] MAYBE (first sighting) -> " + r.label + " " + String(r.confidence));
    }

    if (scanHits >= SCAN_CONFIRM_FRAMES) {
        scanHits = 0;
        Serial1.println("GARBAGE_DETECTED");
        Serial.println("[CAM] GARBAGE_DETECTED — " + r.label +
                       " (" + String(r.confidence) + ")");
        delay(100);                        // short: Dev Board's TRACK command must be read quickly
    }

    delay(100);
}

#if !defined(EI_CLASSIFIER_SENSOR) || \
    EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
