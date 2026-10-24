#include <WiFi.h>
#include <Wire.h>
#include <WebSocketsServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_VL53L0X.h>          // library: "Adafruit_VL53L0X" (version 1.2.0 or newer)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_task_wdt.h"


const char* WIFI_SSID = "Amigos";

const char* WIFI_PASSES[] = { "Amigoslife12" };
const int WIFI_PASS_COUNT = 1;
int wifiPassIdx = 0;

// Pins 
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 32
#define TRIG_PIN 18
#define ECHO_PIN 5
#define SCAN_SERVO_PIN 4
#define FLAME_PIN 34
#define BUZZER_PIN 13
#define CAM_RX 16            // Dev RX2  <- CAM TX (GPIO14)
#define CAM_TX 17            // Dev TX2  -> CAM RX (GPIO15)
#define SDA_PIN 21
#define SCL_PIN 22
#define PCA_ADDR 0x40

// ---- Motor ----
#define MOTOR_MAX 200
#define AUTO_SPEED 130               // slower = CAM (1-2 s per frame) has time to see garbage
#define MANUAL_SPEED 170
#define APPROACH_SPEED 165
#define TURN_SPEED 150
#define RAMP_STEP 15                 // soft start
#define LEFT_INVERT false            // true if left side runs backwards
#define RIGHT_INVERT false

// ---- Ultrasonic / approach ----
#define OBSTACLE_CM 27
#define OBSTACLE_CONFIRM 2           // consecutive readings
#define VLX_OBSTACLE_MM 270          // VLX closer than this = obstacle -> stop + ask CAM (CAM sees garbage at ~28 cm). Must be larger than VLX_PICK_MM
#define VLX_CLASSIFY_MM 270          // while approaching: stop here ONCE and classify (CAM sees best at ~26 cm), then continue to VLX_PICK_MM. 0 = classify only at the claw
#define VLX_PICK_MM 130              // claw is ~12-13 cm in front of the VLX -> garbage at the claw reads ~130 mm. CALIBRATE with key 'v'
#define USE_VLX 1                    // 0 = ignore VLX
#define OBS_BACK_MS 1200             // obstacle: FIRST back up this long (CAM gets a bigger view), then look
#define OBS_SCAN_TURN_MS 250         // obstacle: turn this long left, then right, and CAM-check each direction (calibrate ~20 deg)
#define OBS_FIRE_WAIT_MS 800         // extra wait at obstacle: is it a candle/flame? (don't run away from fire)
#define OBS_CHECK_WAIT_MS 12000      // obstacle: ask CAM 'is it garbage?' (CAM runs CHECK_FRAMES frames, 1-2 s each), then avoid only if not
#define PICK_DIST_CM 13              // ultrasonic fallback: sensor sits on top of the VLX, claw is ~12-13 cm ahead -> garbage at the claw reads ~13 cm
#define VISION_APPROACH 1            // 1 = CAM measures position (recommended), 0 = ultrasonic only
#define PICK_Y_PCT 80                // CALIBRATE: target y (%) when garbage is at the claw
#define CENTER_X_PCT 50              // CALIBRATE: target x (%) when garbage is centred on the claw
#define ALIGN_PCT 10                 // allowed x error (%)
#define STEER_KP 1.4f                // steering gain
#define STEP_MS 150                  // creep (near target) per CAM update, then stop
#define STEP_MID_MS 300              // creep when medium distance
#define STEP_FAR_MS 500              // creep when far (CAM gives only ~1 update/second)
#define APPROACH_VISION_TIMEOUT_MS 25000
#define TGT_FIRST_WAIT_MS 4000       // wait for first TGT, else ultrasonic fallback
#define TGT_LOST_MS 2500             // no TGT for this long = target lost (CAM sends ~1/s)
#define TGT_GIVEUP_MS 5000           // lost this long = give up
#define ULTRA_SAFE_CM 10             // closer than this = already past the claw tip: stop
#define PRINT_TGT 1                  // print TGT lines (use to calibrate PICK_Y_PCT)
#define MIN_VALID_CM 3
#define APPROACH_TIMEOUT_MS 8000
#define SCAN_LEFT 150
#define SCAN_RIGHT 30
#define SCAN_CENTER 90
#define TURN_MS 450
#define BACKUP_MS 300

// ---- Give-up logic (target unreachable or unknown) ----
#define SKIP_BACK_MS 500             // give up: reverse this long (no rear sensor - keep short!)
#define SKIP_COOLDOWN_MS 6000        // give up: ignore garbage this long so the robot drives away instead of retrying the same item

// ---- Safety watchdog ----
#define USE_WDT 1                    // 1 = reboot (motors stop) if the main loop hangs
#define WDT_TIMEOUT_S 10

// ---- Detection logic ----
#define DETECT_CONFIRM_COUNT 1
#define DETECT_WINDOW_MS 5000
#define CLASSIFY_TIMEOUT_MS 12000    // CAM needs ~5-6 s for 5 samples (sometimes 6.1 s) -> 12 s is safe
#define CLASSIFY_TRIES 2
#define COOLDOWN_MS 3000
#define MAYBE_HOLD_MS 4000           // CAM saw something once: stand still this long and wait for confirmation

// ---- Fire ----
#define FIRE_ACTIVE_LEVEL LOW        // flame sensor DO level when fire is seen
#define FIRE_CONFIRM_MS 300          // fire must stay this long before stop
#define FIRE_CLEAR_MS 3000           // no fire this long before resuming

// ---- Servos on PCA9685 ----
#define CLAW_SERVO_CH 0
#define ARM_SERVO_CH 1
#define BASE_SERVO_CH 2
#define SERVO_US_MIN 500             // must match the pulse range you calibrated with
#define SERVO_US_MAX 2500
#define CLAW_OPEN 60
#define CLAW_CLOSE 135
#define ARM_REST 90                  // arm rest position (boot, while driving, after every drop)
#define ARM_PICK 25                  // arm lowered to grab the garbage
#define ARM_CLEAR 60                 // arm MUST be here BEFORE the base moves between park and pick (else the arm gets stuck)
#define ARM_NONBIO 135               // arm at the back of the car: NON_BIO drop (measured)
#define ARM_BIO 135                  // arm at the back of the car: BIO drop (measured)
#define BASE_PICK 60                 // base angle at pick time (measured). Pick always starts here
#define BASE_NONBIO 40               // base angle at the NON_BIO bin (measured; moved to only AFTER the arm is up at 135)
#define BASE_BIO 10                  // base angle at the BIO bin (moved to only AFTER the arm is up at 135)
#define ARM_PARK 25                  // after the drop: arm returns to the picking angle ...
#define BASE_PARK 20                 // ... then the base turns here (arm/claw out of the VLX beam) and waits for the next garbage
#define BASE_STEP 1                  // base turn: degrees per step (smaller = slower)
#define BASE_MS 40                   // base turn: ms per step (bigger = slower)
#define ARM_STEP 2
#define ARM_MS 35                    // arm lower/raise: ms per step (bigger = slower)
#define CLAW_STEP 2
#define CLAW_MS 35                   // claw open/close: ms per step (bigger = slower)
#define GRIP_SETTLE_MS 500
#define ARM_BIN_STEP 1               // swing to the bin: degrees per step (smaller = slower)
#define ARM_BIN_MS 30                // swing to the bin: ms per step (bigger = slower)
#define DROP_SETTLE_MS 500           // wait at the bin before opening the claw
#define CLAW_DROP_STEP 1             // claw release (drop): degrees per step (smaller = slower)
#define CLAW_DROP_MS 40              // claw release (drop): ms per step (bigger = slower)
#define DROP_AFTER_MS 600            // wait after the drop before the arm returns

#define DISABLE_BROWNOUT 1

// PWM compatibility 
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch, f, r) ledcAttachChannel(pin, f, r, ch)
  #define PWM_WRITE(pin, ch, d) ledcWrite(pin, d)
#else
  #define PWM_ATTACH(pin, ch, f, r) do { ledcSetup(ch, f, r); ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, d) ledcWrite(ch, d)
#endif
#define CH_ENA 4
#define CH_ENB 5
#define CH_SCAN 6
#define CH_BUZ 0                     // own timer (not shared with scan servo / motors)
#define BUZZER_FREQ_A 2700           // alarm tone (Hz)
#define BUZZER_FREQ_B 2000
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define BUZ_ATTACH() ledcAttachChannel(BUZZER_PIN, 2000, 8, CH_BUZ)
  #define BUZ_TONE(f) ledcWriteTone(BUZZER_PIN, f)
#else
  #define BUZ_ATTACH() do { ledcSetup(CH_BUZ, 2000, 8); ledcAttachPin(BUZZER_PIN, CH_BUZ); } while (0)
  #define BUZ_TONE(f) ledcWriteTone(CH_BUZ, f)
#endif


Adafruit_PWMServoDriver pca(PCA_ADDR);
WebSocketsServer ws(82);
bool pcaReady = false, wsStarted = false;

// motor
int tgtL = 0, tgtR = 0, curL = 0, curR = 0;
unsigned long lastRamp = 0;

// state
bool autoMode = true, busy = false, armResetReq = false;
bool e2Quiet = false;                      // E2 test mode: hide the repeated MAYBE / GARBAGE_DETECTED lines
int manualDir = 0;                   // 1 fwd, -1 back, 0 none/turn
float lastDist = -1;
Adafruit_VL53L0X lox;
bool vlxReady = false;
int vlxMm = -1;                      // -1 = no reading / out of range
unsigned long vlxT = 0;
unsigned long lastDistT = 0, lastDistSend = 0, lastStatus = 0;
int obsCount = 0;
unsigned long cooldownUntil = 0;
unsigned long maybeUntil = 0;        // robot holds still until this time (CAM first sighting)
int bioCount = 0, nonBioCount = 0;

// CAM
String camLine;
unsigned long lastCamMsg = 0, lastCamWarn = 0;
bool camSeen = false;
int detCount = 0;
unsigned long detFirst = 0;
bool garbageFlag = false;
int checkResult = 0;                 // 0 waiting, 1 = CAM says garbage, 2 = CAM says not garbage
int checkSeq = 0;                    // id of the current CHECK request (stale replies are ignored)
int skipTurnDir = -1;                // direction of the turn after giving up (alternates left/right)
int tgtCx = 50, tgtCy = 0;
unsigned long tgtT = 0;
bool tgtNew = false;
String labelResult = "";
bool labelReady = false;

// fire
bool fireLatched = false;
unsigned long fireSince = 0, fireClearSince = 0;
// E7 timing (fire-stop latency, measured inside the firmware)
unsigned long fireLastCallUs = 0, fireGapMaxUs = 0, fireStopUs = 0;

// servo
int curClaw = CLAW_OPEN, curArm = ARM_REST, curBase = BASE_PICK;

// wifi
unsigned long lastWifiTry = 0;
int wifiChan = 0;                    // learned from scan
uint8_t wifiBssid[6] = {0};
bool wifiHaveBssid = false;

void wifiBegin() {                   // connect using channel+BSSID found by scan (more reliable)
  if (wifiHaveBssid) WiFi.begin(WIFI_SSID, WIFI_PASSES[wifiPassIdx], wifiChan, wifiBssid);
  else WiFi.begin(WIFI_SSID, WIFI_PASSES[wifiPassIdx]);
}

//HELPERS
void sendToApp(const char* msg) {
  if (wsStarted && ws.connectedClients() > 0) ws.broadcastTXT(msg);
}

void sendEvent(const char* ev, const char* key, const char* val) {
  char b[160];
  snprintf(b, sizeof(b), "{\"event\":\"%s\",\"%s\":\"%s\"}", ev, key, val);
  sendToApp(b);
}

void sendEventNum(const char* ev, float v) {
  char b[120];
  snprintf(b, sizeof(b), "{\"event\":\"%s\",\"value\":%.1f}", ev, v);
  sendToApp(b);
}

void logEvent(const char* ev, const char* label, float val) {
  Serial.printf("LOG,%lu,%s,%s,%.2f\n", millis(), ev, label, val);
}

void sendCAM(const char* s) {
  Serial2.println(s);
  Serial.printf("[UART] Sent %s\n", s);
}

//  MOTORS 
void motorApply(int l, int r) {
  if (LEFT_INVERT) l = -l;
  if (RIGHT_INVERT) r = -r;
  digitalWrite(IN1, l > 0); digitalWrite(IN2, l < 0);
  digitalWrite(IN3, r > 0); digitalWrite(IN4, r < 0);
  PWM_WRITE(ENA, CH_ENA, constrain(abs(l), 0, MOTOR_MAX));
  PWM_WRITE(ENB, CH_ENB, constrain(abs(r), 0, MOTOR_MAX));
}

void stopMotors() {
  tgtL = tgtR = 0;
  curL = curR = 0;
  motorApply(0, 0);
}

void drive(int l, int r) {           // -MOTOR_MAX..MOTOR_MAX, soft start
  if (fireLatched || fireSince != 0) { stopMotors(); return; }   // FIRE: motors can never be commanded
  tgtL = constrain(l, -MOTOR_MAX, MOTOR_MAX);
  tgtR = constrain(r, -MOTOR_MAX, MOTOR_MAX);
}

void motorTick() {
  if (millis() - lastRamp < 10) return;
  lastRamp = millis();
  if (curL == tgtL && curR == tgtR) return;
  auto step = [](int c, int t) {
    if (c < t) return min(c + RAMP_STEP, t);
    if (c > t) return max(c - RAMP_STEP, t);
    return c;
  };
  curL = step(curL, tgtL);
  curR = step(curR, tgtR);
  motorApply(curL, curR);
}

// SCAN SERVO (SG90, GPIO4) 
void scanServo(int angle) {
  angle = constrain(angle, 0, 180);
  float us = 500 + (2400 - 500) * (angle / 180.0);
  PWM_WRITE(SCAN_SERVO_PIN, CH_SCAN, (uint32_t)(us * 16383.0 / 20000.0));
}
void scanServoOff() { PWM_WRITE(SCAN_SERVO_PIN, CH_SCAN, 0); }

// ULTRASONIC
float readCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 25000);
  if (d == 0) return -1;
  return d * 0.0343f / 2.0f;
}

//  VLX 
void vlxPoll() {
#if USE_VLX
  if (!vlxReady) return;
  if (millis() - vlxT < 30) return;
  vlxT = millis();
  if (lox.isRangeComplete()) {
    uint16_t r = lox.readRangeResult();
    vlxMm = (r >= 1200) ? -1 : (int)r;       // 8190 / very large = out of range
  }
#endif
}

bool vlxHit() {
#if USE_VLX
  return vlxReady && vlxMm > 0 && vlxMm <= VLX_OBSTACLE_MM;
#else
  return false;
#endif
}

//  FIRE 
void checkFire() {
  unsigned long tCall = micros();                              // E7: start of this flame check
  if (fireLastCallUs != 0) {                                   // E7: gap between two flame checks (= worst-case polling delay)
    unsigned long g = tCall - fireLastCallUs;
    if (g > fireGapMaxUs) fireGapMaxUs = g;
  }
  fireLastCallUs = tCall;
  bool raw = (digitalRead(FLAME_PIN) == FIRE_ACTIVE_LEVEL);
  unsigned long now = millis();
  if (raw) {
    fireClearSince = 0;
    if (fireSince == 0) { fireSince = now; stopMotors(); fireStopUs = micros() - tCall; }   // stop IMMEDIATELY on first sight (E7: stop time in us)
    if (!fireLatched && now - fireSince >= FIRE_CONFIRM_MS) {
      fireLatched = true;
      stopMotors();
      sendEvent("fire", "state", "ON");
      logEvent("FIRE", "ON", 1);
      Serial.println("[FIRE] CONFIRMED -> STOP");
      Serial.printf("[E7] stop_us=%lu confirm_ms=%lu maxgap_ms=%.1f\n", fireStopUs, now - fireSince, fireGapMaxUs / 1000.0);
      fireGapMaxUs = 0;                                        // E7: new window for next trial
    }
  } else {
    fireSince = 0;
    if (fireLatched) {
      if (fireClearSince == 0) fireClearSince = now;
      if (now - fireClearSince >= FIRE_CLEAR_MS) {
        fireLatched = false;
        BUZ_TONE(0);
        sendEvent("fire", "state", "OFF");
        logEvent("FIRE", "CLEARED", 0);
        Serial.println("[FIRE] cleared -> resume");
      }
    }
  }
  if (fireLatched) {
    static int lastPhase = -1;               // two-tone siren, 250 ms each
    int phase = (now / 250) % 2;
    if (phase != lastPhase) { lastPhase = phase; BUZ_TONE(phase ? BUZZER_FREQ_A : BUZZER_FREQ_B); }
    if (curL != 0 || curR != 0 || tgtL != 0 || tgtR != 0) stopMotors();
  }
}

// ===================== CAM UART =====================
void handleCamLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  lastCamMsg = millis();
  camSeen = true;

  if (s == "ALIVE") return;
  if (s.startsWith("CHECK_GARBAGE") || s.startsWith("CHECK_CLEAR")) {
    int comma = s.indexOf(',');
    if (comma > 0 && s.substring(comma + 1).toInt() != checkSeq) {   // reply to an older CHECK -> ignore
      Serial.printf("[CAM] stale %s ignored (now waiting for #%d)\n", s.c_str(), checkSeq);
      return;
    }
    bool g = s.startsWith("CHECK_GARBAGE");
    checkResult = g ? 1 : 2;
    Serial.println(g ? "[CAM] CHECK -> GARBAGE" : "[CAM] CHECK -> not garbage");
    return;
  }
  if (s == "MAYBE") {                                   // CAM saw garbage once -> stop NOW, wait for confirmation
    if (!busy && !fireLatched && millis() > cooldownUntil) {
      maybeUntil = millis() + MAYBE_HOLD_MS;
      stopMotors();
      if (!e2Quiet) Serial.println("[CAM] MAYBE garbage -> stopped, waiting confirm");
    }
    return;
  }
  if (s.startsWith("TGT,")) {
    int cx, cy, w, h; float cf;
    if (sscanf(s.c_str(), "TGT,%d,%d,%d,%d,%f", &cx, &cy, &w, &h, &cf) == 5) {
      tgtCx = cx; tgtCy = cy; tgtT = millis(); tgtNew = true;
#if PRINT_TGT
      Serial.printf("[TGT] x=%d%% y=%d%% w=%d%% h=%d%% conf=%.2f\n", cx, cy, w, h, cf);
#endif
    }
    return;
  }
  if (s == "GARBAGE_DETECTED") {
    unsigned long now = millis();
    if (now - detFirst > DETECT_WINDOW_MS) { detFirst = now; detCount = 0; }
    detCount++;
    if (detCount >= DETECT_CONFIRM_COUNT) garbageFlag = true;
    if (!e2Quiet) Serial.println("[CAM] GARBAGE_DETECTED");
    return;
  }
  if (s == "BIO" || s == "NON_BIO" || s == "UNKNOWN") {
    labelResult = s;
    labelReady = true;
    Serial.printf("[CAM] LABEL %s\n", s.c_str());
    return;
  }
  if (s.startsWith("ERR")) { Serial.printf("[CAM] %s\n", s.c_str()); return; }
  if (s.startsWith("SCAN")) { Serial.printf("[SCAN] %s\n", s.c_str() + 5); return; }
  Serial.printf("[CAM] %s\n", s.c_str());
}

void uartPoll() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') { handleCamLine(camLine); camLine = ""; }
    else if (c != '\r') { if (camLine.length() < 120) camLine += c; }
  }
  if (millis() > 20000 && millis() - lastCamMsg > 10000 && millis() - lastCamWarn > 10000) {
    lastCamWarn = millis();
    Serial.println("[CAM] NO signal for 10 s (check wiring / CAM power)");
  }
}


void onManual(int l, int r, int dir) {
  autoMode = false;
  manualDir = dir;
  if (busy || fireLatched) return;
  drive(l, r);
}

void wsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] client %u connected\n", num);
    return;
  }
  if (type != WStype_TEXT) return;
  String cmd = String((char*)payload);
  cmd.trim();
  cmd.toUpperCase();
  Serial.printf("[WS] cmd %s\n", cmd.c_str());
  if (cmd == "STOP") { autoMode = false; manualDir = 0; stopMotors(); }
  else if (cmd == "FORWARD")  onManual(MANUAL_SPEED, MANUAL_SPEED, 1);
  else if (cmd == "BACKWARD") onManual(-MANUAL_SPEED, -MANUAL_SPEED, -1);
  else if (cmd == "LEFT")     onManual(-TURN_SPEED, TURN_SPEED, 0);
  else if (cmd == "RIGHT")    onManual(TURN_SPEED, -TURN_SPEED, 0);
  else if (cmd == "AUTO")     { autoMode = true; manualDir = 0; }
  else if (cmd == "ARM_RESET") { armResetReq = true; }   // handled in loop(), motors stopped first
}

void wifiKeep() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wsStarted) {
      ws.begin();
      ws.onEvent(wsEvent);
      wsStarted = true;
      Serial.printf("[WiFi] WebSocket ws://%s:82\n", WiFi.localIP().toString().c_str());
    }
  } else if (millis() - lastWifiTry > 15000) {
    lastWifiTry = millis();
    wifiPassIdx = (wifiPassIdx + 1) % WIFI_PASS_COUNT;
    Serial.printf("[WiFi] retry (status=%d)\n", WiFi.status());
    WiFi.disconnect(false);
    delay(100);
    wifiBegin();
  }
}

void wifiSetup() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);       // full power (weak signal otherwise)

  Serial.println("[WiFi] scanning...");
  int n = WiFi.scanNetworks();
  int bestRssi = -999;
  for (int i = 0; i < n; i++) {
    if (WiFi.SSID(i) == String(WIFI_SSID) && WiFi.RSSI(i) > bestRssi) {
      bestRssi = WiFi.RSSI(i);
      wifiChan = WiFi.channel(i);
      memcpy(wifiBssid, WiFi.BSSID(i), 6);
      wifiHaveBssid = true;
    }
  }
  if (wifiHaveBssid) Serial.printf("[WiFi] found '%s' RSSI %d dBm, channel %d (move closer if worse than -75)\n", WIFI_SSID, bestRssi, wifiChan);
  else Serial.printf("[WiFi] '%s' NOT visible (2.4 GHz? too far? name exact?)\n", WIFI_SSID);
  WiFi.scanDelete();

  for (int i = 0; i < WIFI_PASS_COUNT && WiFi.status() != WL_CONNECTED; i++) {
    wifiPassIdx = i;
    if (i > 0) { WiFi.disconnect(false); delay(100); }
    wifiBegin();
    Serial.printf("[WiFi] Connecting (password #%d)", i + 1);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
      delay(500); Serial.print(".");
    }
    Serial.printf(" status=%d\n", WiFi.status());
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected! IP %s RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[WiFi] Not connected - robot works offline, retrying in background");
  }
}

void svc() {
#if USE_WDT
  esp_task_wdt_reset();
#endif
  if (wsStarted) ws.loop();
  uartPoll();
  vlxPoll();
  checkFire();
  motorTick();
  wifiKeep();
}

void waitMs(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) { svc(); delay(2); }
}

// ===================== ARM (PCA9685) =====================
uint16_t angleToTicks(int a) {
  a = constrain(a, 0, 180);
  float us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * (a / 180.0);
  return (uint16_t)(us * 4096.0 / 20000.0);
}

void servoWrite(int ch, int a) { if (pcaReady) pca.setPWM(ch, 0, angleToTicks(a)); }

void servosEnergize() {
  servoWrite(CLAW_SERVO_CH, curClaw);   // hold last known angles -> no jump
  servoWrite(ARM_SERVO_CH, curArm);
  servoWrite(BASE_SERVO_CH, curBase);
  waitMs(250);
}

void servosOff() {
  if (!pcaReady) return;
  pca.setPWM(CLAW_SERVO_CH, 0, 4096);   // full off
  pca.setPWM(ARM_SERVO_CH, 0, 4096);
  pca.setPWM(BASE_SERVO_CH, 0, 4096);
}

void smoothMove(int ch, int& cur, int target, int stepDeg, int ms) {
  while (cur != target) {
    if (cur < target) cur = min(cur + stepDeg, target);
    else cur = max(cur - stepDeg, target);
    servoWrite(ch, cur);
    waitMs(ms);
  }
}
void clawMove(int a) { smoothMove(CLAW_SERVO_CH, curClaw, a, CLAW_STEP, CLAW_MS); }
void armMove(int a)  { smoothMove(ARM_SERVO_CH, curArm, a, ARM_STEP, ARM_MS); }
void baseMove(int a) { smoothMove(BASE_SERVO_CH, curBase, a, BASE_STEP, BASE_MS); }

void armReset() {
  servosEnergize();
  clawMove(CLAW_OPEN);
  armMove(ARM_CLEAR);                 // arm to 60 first, then the base (arm must not be low while the base moves)
  baseMove(BASE_PICK);
  armMove(ARM_REST);
  servosOff();
  Serial.println("[ARM] reset done");
}

// base back to BASE_PICK = 60 (arm to 60 first; only the base is powered; claw and arm stay OFF). Called when the CAM reports garbage.
void baseToPick() {
  if (!pcaReady || curBase == BASE_PICK) return;
  Serial.printf("[BASE] garbage signal -> base %d -> %d\n", curBase, BASE_PICK);
  servoWrite(ARM_SERVO_CH, curArm);     // hold last known angle -> no jump
  servoWrite(BASE_SERVO_CH, curBase);
  waitMs(250);
  if (curArm != ARM_CLEAR) armMove(ARM_CLEAR);   // arm FIRST to 60, only then the base (otherwise the arm gets stuck)
  baseMove(BASE_PICK);
  pca.setPWM(BASE_SERVO_CH, 0, 4096);   // base OFF again (arm stays powered at 60 so it cannot sag)
}

void armPick(int binAngle, int baseAngle) {
  if (!pcaReady) { Serial.println("[ARM] pick start failed: PCA9685 not ready"); return; }
  Serial.println("[ARM] pick start");
  servosEnergize();
  if (curBase != BASE_PICK) { if (curArm != ARM_CLEAR) armMove(ARM_CLEAR); baseMove(BASE_PICK); }   // safety: arm to 60 first, then base to BASE_PICK
  clawMove(CLAW_OPEN);                // claw open before lowering
  armMove(ARM_PICK);                  // arm lowers from rest (90) to the garbage (25)
  clawMove(CLAW_CLOSE);               // grip, stays closed until bin
  waitMs(GRIP_SETTLE_MS);
  smoothMove(ARM_SERVO_CH, curArm, binAngle, ARM_BIN_STEP, ARM_BIN_MS);      // arm up to the bin angle (slow)
  baseMove(baseAngle);                // NON_BIO: base turns to 40 | BIO: base turns to 10 (only now, arm is already up at 135)
  waitMs(DROP_SETTLE_MS);
  smoothMove(CLAW_SERVO_CH, curClaw, CLAW_OPEN, CLAW_DROP_STEP, CLAW_DROP_MS);  // drop (slow release)
  waitMs(DROP_AFTER_MS);
  if (curBase != BASE_PICK) baseMove(BASE_PICK);   // BIO: arm is still UP (135) here -> base returns to pick angle first
  armMove(ARM_CLEAR);                 // arm to 60 BEFORE the base moves to park (same rule as on the way in)
  baseMove(BASE_PARK);                // base turns to BASE_PARK = arm/claw out of the VLX beam
  armMove(ARM_PARK);                  // arm down to the park angle
  servosOff();
  Serial.printf("[ARM] pick done (parked: arm %d, base %d)\n", ARM_PARK, BASE_PARK);
}

// ===================== GARBAGE SEQUENCE =====================
String classifyGarbage();            // defined below (used by the early classify in approachVision)
String preLabel = "";                // result of the early classify at VLX_CLASSIFY_MM (empty = not done / unknown)
bool approachUltra() {
  unsigned long t0 = millis();
  int close = 0;
  drive(APPROACH_SPEED, APPROACH_SPEED);
  while (millis() - t0 < APPROACH_TIMEOUT_MS) {
    svc();
    if (fireLatched) return false;
    float d = readCm();
    if (d > 0 && d < 400) { lastDist = d; }
#if USE_VLX
    if (vlxReady && vlxMm > 0 && vlxMm <= VLX_PICK_MM) { stopMotors(); waitMs(200); return true; }
#endif
    if (d > 0 && d <= PICK_DIST_CM) {
      close++;
      if (close >= 2) { stopMotors(); waitMs(200); return true; }
    } else {
      close = 0;
    }
    delay(40);
  }
  stopMotors();
  return false;                       // timeout
}


bool approachVision() {
  unsigned long t0 = millis();
  tgtNew = false;
  unsigned long w = millis();
  while (millis() - w < TGT_FIRST_WAIT_MS && !tgtNew) {
    svc(); delay(5);
    if (fireLatched) return false;
  }
  if (!tgtNew) {
    Serial.println("[APPROACH] no TGT from CAM -> ultrasonic fallback");
    return approachUltra();
  }

  int ok = 0;
  bool preTried = false;
  unsigned long lostSince = 0, moveUntil = 0;
  while (millis() - t0 < APPROACH_VISION_TIMEOUT_MS) {
    svc();
    if (fireLatched) return false;
    if (moveUntil && millis() > moveUntil) { stopMotors(); moveUntil = 0; }

    float d = readCm();                              // safety only
    if (d > 0 && d <= ULTRA_SAFE_CM) { stopMotors(); waitMs(200); return true; }
#if USE_VLX
    if (vlxReady && vlxMm > 0 && vlxMm <= VLX_PICK_MM) { stopMotors(); Serial.printf("[APPROACH] VLX %d mm -> arrived\n", vlxMm); waitMs(200); return true; }
    if (VLX_CLASSIFY_MM > 0 && !preTried && vlxReady && vlxMm > VLX_PICK_MM && vlxMm <= VLX_CLASSIFY_MM) {
      preTried = true;                               // classify ONCE at ~26 cm where the CAM sees the garbage best
      stopMotors(); moveUntil = 0; waitMs(300);
      String l = classifyGarbage();
      Serial.printf("[APPROACH] early classify at VLX %d mm -> %s\n", vlxMm, l.c_str());
      if (l != "UNKNOWN") preLabel = l;
      sendCAM("TRACK");                              // CAM is busy after CLASSIFY: resume target tracking
      tgtNew = false; tgtT = millis(); lostSince = 0;
      continue;
    }
#endif

    if (millis() - tgtT > TGT_LOST_MS) {             // target lost
      stopMotors();
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > TGT_GIVEUP_MS) { Serial.println("[APPROACH] target lost"); return false; }
      delay(10); continue;
    }
    lostSince = 0;
    if (!tgtNew) { delay(5); continue; }
    tgtNew = false;

    int err = tgtCx - CENTER_X_PCT;                  // + = target is to the right
    Serial.printf("[STEP] err=%d cy=%d vlx=%d\n", err, tgtCy, vlxMm);
    bool aligned = abs(err) <= ALIGN_PCT;

    if (tgtCy >= PICK_Y_PCT && aligned) {            // arrived
      stopMotors(); moveUntil = 0;
      if (++ok >= 2) { waitMs(200); return true; }
      continue;
    }
    ok = 0;
    if (!aligned && tgtCy >= PICK_Y_PCT) {           // close but off-centre: rotate in place
      int sp = max(abs(err) * 3, 110);
      sp = min(sp, TURN_SPEED);
      if (err > 0) drive(sp, -sp); else drive(-sp, sp);
      moveUntil = millis() + STEP_MS;
    } else {                                          // steer while creeping forward
      int turn = constrain((int)(err * STEER_KP), -60, 60);
      drive(APPROACH_SPEED + turn, APPROACH_SPEED - turn);
      int gap = PICK_Y_PCT - tgtCy;                   // far -> longer creep
      unsigned long stepMs = (gap > 25) ? STEP_FAR_MS : (gap > 12) ? STEP_MID_MS : STEP_MS;
      moveUntil = millis() + stepMs;                  // stop-and-go
    }
  }
  stopMotors();
  return false;
}

String classifyGarbage() {
  for (int i = 0; i < CLASSIFY_TRIES; i++) {
    while (Serial2.available()) Serial2.read();   // flush old lines
    labelReady = false;
    sendCAM("CLASSIFY");
    unsigned long t = millis();
    while (millis() - t < CLASSIFY_TIMEOUT_MS && !labelReady) { svc(); delay(2); }
    if (labelReady && labelResult != "UNKNOWN") return labelResult;
    Serial.println("[CLASSIFY] unknown/timeout, retry");
  }
  return "UNKNOWN";
}

// could not pick the item: back off a little and turn away so the robot does not retry the same item forever
void giveUpTarget(const char* why) {
  logEvent("GIVEUP", why, 0);
  Serial.printf("[GARBAGE] giving up (%s) -> back off and turn away\n", why);
  stopMotors();
  drive(-TURN_SPEED, -TURN_SPEED);
  waitMs(SKIP_BACK_MS);
  stopMotors();
  waitMs(200);
  skipTurnDir = -skipTurnDir;                 // alternate left / right
  if (skipTurnDir < 0) drive(-TURN_SPEED, TURN_SPEED); else drive(TURN_SPEED, -TURN_SPEED);
  waitMs(TURN_MS);
  stopMotors();
}

void handleGarbage() {
  busy = true;
  bool picked = false;
  maybeUntil = 0;
  unsigned long t0 = millis();
  garbageFlag = false; detCount = 0;
  preLabel = "";
  stopMotors();
  sendCAM(VISION_APPROACH ? "TRACK" : "BUSY");
  sendEvent("garbage", "state", "DETECTED");
  logEvent("DETECT", "garbage", lastDist);

  baseToPick();                                    // garbage signal -> arm 60 first, then base to BASE_PICK 60 (claw still OFF)

  bool reached = VISION_APPROACH ? approachVision() : approachUltra();
  logEvent("APPROACH", reached ? "OK" : "TIMEOUT", lastDist);

  if (reached && !fireLatched) {
    String label = (preLabel != "") ? preLabel : classifyGarbage();   // early result if we have one, else classify at the claw
    logEvent("CLASSIFY", label.c_str(), 0);
    sendEvent("garbage", "label", label.c_str());
    if (fireLatched || fireSince != 0) {            // fire appeared while classifying: never start the arm
      Serial.println("[GARBAGE] fire during classify -> pick cancelled");
      logEvent("PICK", "CANCELLED_FIRE", millis() - t0);
    } else if (label == "BIO") {
      armPick(ARM_BIO, BASE_BIO);
      picked = true;
      bioCount++;                      // NOTE: counts pick ATTEMPTS (no grip sensor)
      logEvent("PICK", "BIO", millis() - t0);
    } else if (label == "NON_BIO") {
      armPick(ARM_NONBIO, BASE_NONBIO);
      picked = true;
      nonBioCount++;                   // NOTE: counts pick ATTEMPTS (no grip sensor)
      logEvent("PICK", "NON_BIO", millis() - t0);
    } else {
      logEvent("PICK", "SKIPPED", millis() - t0);
      if (!fireLatched) giveUpTarget("unknown");
    }
  } else {
    Serial.println("[GARBAGE] could not reach target, skipping");
    if (!fireLatched) giveUpTarget("unreached");
  }

  sendCAM("READY");
  cooldownUntil = millis() + (picked ? COOLDOWN_MS : SKIP_COOLDOWN_MS);
  garbageFlag = false; detCount = 0;
  busy = false;
}


// ask CAM "is there garbage in view?"  returns 1 = garbage, 2 = no garbage / timeout, 3 = fire (abort)
int camCheck() {
  uartPoll();
  checkResult = 0;
  checkSeq++;
  char req[24];
  snprintf(req, sizeof(req), "CHECK,%d", checkSeq);
  sendCAM(req);
  unsigned long t = millis();
  while (millis() - t < OBS_CHECK_WAIT_MS && checkResult != 2) {
    svc(); delay(2);
    if (fireLatched) return 3;
    if (garbageFlag || checkResult == 1) {
      if (millis() > cooldownUntil) return 1;
      garbageFlag = false; detCount = 0; checkResult = 2;   // cooldown: ignore
    }
  }
  return 2;
}

// turn in place: dir -1 = left, +1 = right. returns false if fire (abort)
bool obsTurn(int dir, unsigned long ms) {
  busy = true;
  if (dir < 0) drive(-TURN_SPEED, TURN_SPEED); else drive(TURN_SPEED, -TURN_SPEED);
  waitMs(ms);
  stopMotors();
  waitMs(300);                                   // settle (camera blur)
  busy = false;
  return !fireLatched;
}

void obstacleRoutine() {
  stopMotors();
  sendEventNum("obstacle", lastDist);
  logEvent("OBSTACLE", "stop", lastDist);
  Serial.printf("[OBS] sonic %.0f cm | VLX %d mm\n", lastDist, vlxMm);
  int e8TrigVlx = vlxMm;                          // E8: values at the moment the obstacle was detected
  float e8TrigSonic = lastDist;

  // 1) candle/flame check: stay still, let the flame sensor decide before moving
  unsigned long t = millis();
  while (millis() - t < OBS_FIRE_WAIT_MS) {
    svc(); delay(2);
    if (fireLatched || fireSince != 0) { obsCount = 0; return; }   // FIRE: stay stopped, siren handles it
    if (garbageFlag && millis() > cooldownUntil) { handleGarbage(); return; }
  }
  {                                               // E8: robot has stopped completely -> read the final gap (no ruler needed)
    float e8Sonic = readCm();
    Serial.printf("[E8] trig_vlx=%d trig_sonic=%.0f settled_vlx=%d settled_sonic=%.0f overshoot_mm=%d\n",
                  e8TrigVlx, e8TrigSonic, vlxMm, e8Sonic, e8TrigVlx - vlxMm);
  }

  // 2) go BACK first (camera gets a better view, claw is not touching the object)
  busy = true;
  drive(-TURN_SPEED, -TURN_SPEED);
  waitMs(OBS_BACK_MS);
  stopMotors();
  busy = false;
  if (fireLatched) return;

  // 3) look for garbage with the CAM: straight, then left, then right
  int r = camCheck();
  if (r == 1) { handleGarbage(); return; }
  if (r == 3) return;
  Serial.println("[OBS] straight: no garbage -> look left");
  if (!obsTurn(-1, OBS_SCAN_TURN_MS)) return;
  r = camCheck();
  if (r == 1) { handleGarbage(); return; }
  if (r == 3) return;
  Serial.println("[OBS] left: no garbage -> look right");
  if (!obsTurn(+1, OBS_SCAN_TURN_MS * 2)) return;
  r = camCheck();
  if (r == 1) { handleGarbage(); return; }
  if (r == 3) return;
  Serial.println("[OBS] right: no garbage -> back to centre");
  if (!obsTurn(-1, OBS_SCAN_TURN_MS)) return;

  busy = true;
  scanServo(SCAN_LEFT);  waitMs(400);
  float dl = readCm(); if (dl < 0) dl = 400;
  scanServo(SCAN_RIGHT); waitMs(600);
  float dr = readCm(); if (dr < 0) dr = 400;
  scanServo(SCAN_CENTER); waitMs(300);
  scanServoOff();
  Serial.printf("[SCAN] Left %.0f  Right %.0f\n", dl, dr);

  if (fireLatched) { scanServoOff(); stopMotors(); busy = false; return; }
  if (dl >= dr) drive(-TURN_SPEED, TURN_SPEED);
  else drive(TURN_SPEED, -TURN_SPEED);
  waitMs(TURN_MS);
  stopMotors();
  obsCount = 0;
  maybeUntil = 0;
  busy = false;
  Serial.println("[E8] avoided -> resume");        
}


void autoStep() {
  if (garbageFlag) {
    if (millis() > cooldownUntil) { handleGarbage(); return; }
    garbageFlag = false; detCount = 0;      
  }
  if (millis() < maybeUntil) {                
    if (curL != 0 || curR != 0 || tgtL != 0 || tgtR != 0) stopMotors();
    return;
  }
  if (obsCount >= OBSTACLE_CONFIRM) { obstacleRoutine(); return; }
  if (tgtL != AUTO_SPEED || tgtR != AUTO_SPEED) drive(AUTO_SPEED, AUTO_SPEED);
}


void serialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '1') { Serial.println("TEST pick BIO");     stopMotors(); busy = true; armPick(ARM_BIO, BASE_BIO);    busy = false; }
    else if (c == '2') { Serial.println("TEST pick NON_BIO"); stopMotors(); busy = true; armPick(ARM_NONBIO, BASE_NONBIO); busy = false; }
    else if (c == 'r') { stopMotors(); busy = true; armReset(); busy = false; }
    else if (c == 'a') { autoMode = true; e2Quiet = false; fireGapMaxUs = 0; Serial.println("AUTO ON"); }
    else if (c == 's') { autoMode = false; manualDir = 0; stopMotors(); Serial.println("STOP (manual)"); }
    else if (c == 'd') { Serial.printf("dist %.1f cm\n", readCm()); }
    else if (c == 'f') { Serial.printf("flame pin = %d (fire level %d)\n", digitalRead(FLAME_PIN), FIRE_ACTIVE_LEVEL); }
    else if (c == 't') { autoMode = false; stopMotors(); sendCAM("TRACK"); Serial.println("CALIBRATE: CAM sends [TGT] x,y (hold garbage in front of claw). g = stop"); }
    else if (c == 'c') {                             // E2 test: ONE classification only (no retry, motors off, no arm)
      autoMode = false; manualDir = 0; stopMotors(); e2Quiet = true;
      while (Serial2.available()) Serial2.read();
      labelReady = false;
      sendCAM("CLASSIFY");
      unsigned long t = millis();
      while (millis() - t < CLASSIFY_TIMEOUT_MS && !labelReady) { svc(); delay(2); }
      Serial.printf("[E2] result = %s\n", labelReady ? labelResult.c_str() : "TIMEOUT");
      sendCAM("READY");
    }
    else if (c == 'b') { BUZ_TONE(BUZZER_FREQ_A); delay(300); BUZ_TONE(BUZZER_FREQ_B); delay(300); BUZ_TONE(0); Serial.println("buzzer test"); }
    else if (c == 'g') { sendCAM("READY"); Serial.println("CALIBRATE end"); }
    else if (c == 'v') { Serial.printf("VLX %d mm (ready=%d) | sonic %.1f cm\n", vlxMm, vlxReady, readCm()); }
    else if (c == 'h') { Serial.println("v VLX test | 1 bio pick | 2 nonbio pick | r arm reset | a auto | s stop | d dist | f flame | t calibrate(TGT) | g end calibrate | c classify once (E2 test) | b buzzer test"); }
  }
}


void setup() {
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== EcoBot Dev Board ===");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  PWM_ATTACH(ENA, CH_ENA, 1000, 8);
  PWM_ATTACH(ENB, CH_ENB, 1000, 8);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(FLAME_PIN, INPUT);
  BUZ_ATTACH(); BUZ_TONE(0);

  PWM_ATTACH(SCAN_SERVO_PIN, CH_SCAN, 50, 14);
  scanServo(SCAN_CENTER); delay(400); scanServoOff();

  Serial2.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() == 0) {
    pca.begin();
    pca.setOscillatorFrequency(27000000);
    pca.setPWMFreq(50);
    pcaReady = true;
    servosOff();                       // servos OFF while driving
    Serial.println("[PCA9685] OK");
  } else {
    Serial.println("[PCA9685] NOT FOUND (arm disabled) - check SDA21/SCL22/power");
  }

#if USE_VLX
  if (lox.begin()) {
    Wire.begin(SDA_PIN, SCL_PIN);       // make sure the shared I2C bus keeps SDA21/SCL22 after VLX init
    lox.startRangeContinuous(50);
    vlxReady = true;
    Serial.println("[VLX] OK");
  } else {
    Serial.println("[VLX] NOT FOUND (check SDA21/SCL22/3V3) - running without it");
  }
#endif

  wifiSetup();
  Serial.println("CSV: LOG,millis,event,label,value");
  Serial.println("USB keys: h for help. Robot starts in AUTO.");
  lastCamMsg = millis();

#if USE_WDT
  // start the watchdog only now: Wi-Fi connect above may block for many seconds
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = { (uint32_t)WDT_TIMEOUT_S * 1000, 0, true };
  esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
  Serial.println("[WDT] watchdog on");
#endif
}

void loop() {
  svc();
  serialCommands();

  // distance (every 70 ms)
  if (millis() - lastDistT >= 70 && !busy) {
    lastDistT = millis();
    lastDist = readCm();
    if (fireLatched || fireSince != 0) obsCount = 0;
    else if ((lastDist > 0 && lastDist <= OBSTACLE_CM) || vlxHit()) obsCount++; else obsCount = 0;
  }
  if (millis() - lastDistSend >= 500) {
    lastDistSend = millis();
    if (lastDist > 0) sendEventNum("distance", lastDist);
  }
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    char b[200];
    snprintf(b, sizeof(b),
      "{\"event\":\"status\",\"mode\":\"%s\",\"fire\":%d,\"bio\":%d,\"nonbio\":%d,\"cam\":%d,\"rssi\":%d}",
      autoMode ? "AUTO" : "MANUAL", fireLatched, bioCount, nonBioCount,
      (millis() - lastCamMsg < 6000) ? 1 : 0, WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
    sendToApp(b);
  }

  if (armResetReq && !busy && !fireLatched) {
    armResetReq = false;
    stopMotors();                      // DC motors OFF before servos ON
    busy = true; armReset(); busy = false;
  }

  if (fireLatched || busy) return;

  if (!autoMode) {
    garbageFlag = false; detCount = 0;
    if (manualDir == 1 && obsCount >= OBSTACLE_CONFIRM) {   // safety stop when driving forward manually
      stopMotors(); manualDir = 0;
      sendEventNum("obstacle", lastDist);
    }
    return;
  }
  autoStep();
}