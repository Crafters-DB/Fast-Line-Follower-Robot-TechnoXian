/*
  =========================================================
  ESP32 LINE FOLLOWER v4 - WiFi dashboard, sliders, OTA
  =========================================================
  Connect to WiFi "LineFollower_AP", open http://192.168.4.1/

  QUICK START
   1. Put the line under the MIDDLE of the sensor array, at the
      height the robot will actually run at.
   2. Press "Calibrate (auto rock)" - or "Calibrate (by hand)" and
      slide the robot left/right across the line yourself.
   3. Press Start. If it turns AWAY from the line, tick
      "Reverse steering". If it drives backwards, tick
      "Reverse motors". Both are saved.

  SENSING
   - Readings are calibrated millivolts (analogReadMilliVolts),
     3x oversampled, and refreshed live in every state, so the
     dashboard always shows what the sensors really see.
   - Orange bars = the reading is clipping at the ADC ceiling
     (~3.1 V). If black AND white both read ~3100 mV, SIG is above
     the ESP32's range: add a divider (e.g. 10k top / 15k bottom).
   - Line polarity (dark-on-light vs light-on-dark) is detected
     automatically at calibration; tick "Line reads LOW" to force.

  MASTER SLIDER (t = master / 100)
     Speed    = 40 + 160*t
     Kp       = 0.9 * Speed / 100
     Kd       = 12 * Kp
     Ki       = 0.0005 * Kp
     Slowdown = 0.15 + 0.35*t

  EDIT THE PIN DEFINITIONS BELOW to match your actual wiring.
  =========================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

// ---------------- WiFi AP ----------------
const char* AP_SSID      = "LineFollower_AP";
const char* AP_PASSWORD  = "linefollow123";   // 8+ chars
const char* OTA_HOSTNAME = "linefollower";

// ---------------- Pin Definitions (EDIT THESE) ----------------
#define MUX_S0   32
#define MUX_S1   33
#define MUX_S2   25
#define MUX_S3   26
#define MUX_SIG  34   // must be an ADC1 pin (WiFi is on)
#define MUX_EN   27   // active LOW enable

#define AIN1     15
#define AIN2     14
#define PWMA     13
#define BIN1     19
#define BIN2     21
#define PWMB     18
#define STBY     23

#define CAL_BTN_PIN    4    // press = auto calibrate
#define START_BTN_PIN  5    // press = start / stop

// ---------------- Sensor config ----------------
const int   NUM_SENSORS      = 16;
const int   ADC_SAMPLES      = 3;       // oversampling per channel
const int   MUX_SETTLE_US    = 30;
const int   SENSOR_MIN_RANGE = 25;      // min calibration swing (mV) to trust a channel
const int   ADC_SAT_MV       = 3000;    // at/above this a reading is considered clipped
const float NOISE_FLOOR      = 0.15f;   // normalized readings below this are ignored
const float MIN_LINE_SUM     = 0.25f;   // min total activation to say "line seen"

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
volatile int sensorValue[NUM_SENSORS];  // mV
float sensorWeight[NUM_SENSORS];        // -100..+100
bool sensorValid[NUM_SENSORS];
int validSensorCount = 0;

// ---------------- Tunables (all live from the dashboard) ----------------
volatile float Kp = 0.5f;
volatile float Ki = 0.0f;
volatile float Kd = 6.0f;               // derivative per millisecond
volatile int   baseSpeed = 70;          // 0-255
volatile float turnSlow = 0.25f;        // fraction of speed removed at max error
volatile int   masterPct = 30;
volatile bool  invertSensor = false;    // false: line reads HIGHER than background
volatile bool  reverseSteer = false;
volatile bool  reverseMotors = false;
volatile bool  optsDirty = false;       // loop() saves tuning when set

// ---------------- Fixed constants ----------------
const int MOTOR_DEADBAND = 35;                    // PWM where the motors actually start to turn
const int CALIB_SPEED    = 70;
const unsigned long CALIB_AUTO_MS   = 6000;
const unsigned long CALIB_MANUAL_MS = 8000;
const unsigned long RAMP_MS         = 400;        // soft start
const unsigned long LOST_TIMEOUT_MS = 2000;
const float INT_LIMIT = 2000.0f;

// ---------------- State ----------------
enum RobotState { IDLE, CALIBRATING, READY, RUNNING };
volatile RobotState g_state = IDLE;
volatile int  calibRequest = 0;         // 0 none, 1 auto rock, 2 by hand
volatile bool toggleRequested = false;
int calibMode = 1;
unsigned long calibStartTime = 0;

float lastError = 0, integral = 0, dFilt = 0, lastPos = 0;
unsigned long lastLoopUs = 0, runStartMs = 0, lineLostStart = 0;

// ---------------- Telemetry ----------------
volatile float g_position = 0;
volatile float g_correction = 0;
volatile int g_leftSpeed = 0;
volatile int g_rightSpeed = 0;
volatile bool g_calibrated = false;

// ---------------- Preferences ----------------
Preferences prefsTune;   // used only from loop()/web handlers
Preferences prefsCal;    // used only from the control task

// ---------------- Log ring buffer ----------------
#define LOG_LINES 40
String logBuf[LOG_LINES];
int logHead = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void logMsg(const String &msg) {
  Serial.println(msg);
  portENTER_CRITICAL(&logMux);
  logBuf[logHead] = msg;
  logHead = (logHead + 1) % LOG_LINES;
  portEXIT_CRITICAL(&logMux);
}

String getLogJson() {
  String out = "[";
  portENTER_CRITICAL(&logMux);
  for (int i = 0; i < LOG_LINES; i++) {
    int idx = (logHead + i) % LOG_LINES;
    if (logBuf[idx].length() == 0) continue;
    String esc = logBuf[idx];
    esc.replace("\\", "\\\\");
    esc.replace("\"", "\\\"");
    if (out.length() > 1) out += ",";
    out += "\"" + esc + "\"";
  }
  portEXIT_CRITICAL(&logMux);
  out += "]";
  return out;
}

// ---------------- Web page ----------------
WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Line Follower</title>
<style>
  body { font-family: -apple-system, Arial, sans-serif; background:#111; color:#eee; margin:0; padding:12px; }
  h1 { font-size:18px; margin:8px 0; }
  .card { background:#1c1c1c; border-radius:10px; padding:14px; margin-bottom:12px; }
  .row { display:flex; gap:10px; flex-wrap:wrap; align-items:center; margin-bottom:10px; }
  label { font-size:13px; color:#aaa; width:78px; display:inline-block; }
  input[type=number] { width:74px; padding:6px; border-radius:6px; border:1px solid #444; background:#222; color:#fff; }
  input[type=range] { flex:1; min-width:120px; height:28px; accent-color:#eab308; touch-action:pan-y; }
  button { padding:10px 14px; border:none; border-radius:8px; font-size:14px; font-weight:600; cursor:pointer; }
  .btn-cal { background:#3b82f6; color:#fff; }
  .btn-cal2 { background:#8b5cf6; color:#fff; }
  .btn-start { background:#22c55e; color:#000; }
  .btn-stop { background:#ef4444; color:#fff; }
  .btn-step { background:#333; color:#fff; padding:8px 12px; }
  .status { font-size:16px; font-weight:700; padding:6px 10px; border-radius:6px; display:inline-block; }
  .st-IDLE { background:#444; }
  .st-CALIBRATING { background:#eab308; color:#000; }
  .st-READY { background:#3b82f6; }
  .st-RUNNING { background:#22c55e; color:#000; }
  .bars { display:flex; gap:3px; height:70px; align-items:flex-end; }
  .bar { flex:1; background:#3b82f6; border-radius:2px 2px 0 0; }
  .bar.bad { background:#666; }
  .bar.sat { background:#f97316; }
  .nums { display:flex; gap:3px; margin-top:4px; }
  .n { flex:1; font-size:9px; text-align:center; color:#9ca3af; line-height:1.2; }
  .n small { display:block; color:#555; }
  .n.sat { color:#f97316; }
  #log { background:#000; color:#0f0; font-family:monospace; font-size:12px; height:200px; overflow-y:auto; padding:8px; border-radius:6px; white-space:pre-wrap; }
  .metrics { display:flex; gap:10px; flex-wrap:wrap; font-size:13px; margin-top:10px; }
  .metrics div { background:#222; padding:6px 10px; border-radius:6px; }
  .tag { font-size:11px; color:#999; margin-left:8px; }
  .hint { font-size:12px; color:#888; line-height:1.5; }
  .chk { display:flex; align-items:center; gap:8px; font-size:14px; margin-bottom:8px; }
  .chk input { width:20px; height:20px; }
  .big { font-size:22px; font-weight:700; min-width:52px; text-align:right; }
</style>
</head>
<body>
<h1>Line Follower</h1>

<div class="card">
  <span id="statusBadge" class="status st-IDLE">IDLE</span>
  <span id="calibTag" class="tag"></span>
  <div class="row" style="margin-top:10px;">
    <button class="btn-cal" onclick="calibrate(1)">Calibrate (auto rock)</button>
    <button class="btn-cal2" onclick="calibrate(2)">Calibrate (by hand)</button>
    <button id="startBtn" class="btn-start" onclick="toggle()">Start</button>
  </div>
  <div class="hint">Put the line under the MIDDLE sensors first. "By hand": slide the robot left and right across the line for 8 s.</div>
</div>

<div class="card">
  <h1>Master (moves everything together)</h1>
  <div class="row">
    <input id="masterRange" type="range" min="0" max="100" step="1" oninput="masterInput()" onchange="masterSend()">
    <span id="masterVal" class="big">30</span><span>%</span>
  </div>
  <div class="hint">t = M/100 &nbsp;|&nbsp; Speed = 40 + 160t &nbsp;|&nbsp; Kp = 0.9&middot;Speed/100 &nbsp;|&nbsp; Kd = 12&middot;Kp &nbsp;|&nbsp; Ki = 0.0005&middot;Kp &nbsp;|&nbsp; Slowdown = 0.15 + 0.35t</div>
</div>

<div class="card">
  <h1>PID / Speed</h1>
  <div class="row"><label>Kp</label><input id="kpRange" type="range" min="0" max="3" step="0.01" oninput="syncNum('kp')" onchange="sendTuning()"><input id="kp" type="number" step="0.01" oninput="syncRange('kp')" onchange="sendTuning()"></div>
  <div class="row"><label>Ki</label><input id="kiRange" type="range" min="0" max="0.01" step="0.0001" oninput="syncNum('ki')" onchange="sendTuning()"><input id="ki" type="number" step="0.0001" oninput="syncRange('ki')" onchange="sendTuning()"></div>
  <div class="row"><label>Kd</label><input id="kdRange" type="range" min="0" max="40" step="0.1" oninput="syncNum('kd')" onchange="sendTuning()"><input id="kd" type="number" step="0.1" oninput="syncRange('kd')" onchange="sendTuning()"></div>
  <div class="row">
    <label>Speed</label>
    <button class="btn-step" onclick="adjustSpeed(-10)">-10</button>
    <input id="speedRange" type="range" min="0" max="255" step="1" oninput="syncNum('speed')" onchange="sendTuning()">
    <input id="speed" type="number" step="1" min="0" max="255" oninput="syncRange('speed')" onchange="sendTuning()">
    <button class="btn-step" onclick="adjustSpeed(10)">+10</button>
  </div>
  <div class="row"><label>Slowdown</label><input id="slowRange" type="range" min="0" max="0.8" step="0.01" oninput="syncNum('slow')" onchange="sendTuning()"><input id="slow" type="number" step="0.01" oninput="syncRange('slow')" onchange="sendTuning()"></div>
  <div class="hint">Slowdown = how much speed is shed in sharp curves (0 = none).</div>
</div>

<div class="card">
  <h1>Hardware options</h1>
  <label class="chk"><input id="invS" type="checkbox" onchange="sendOpts()"> Line reads LOW (invert sensors)</label>
  <label class="chk"><input id="revSteer" type="checkbox" onchange="sendOpts()"> Reverse steering (left/right)</label>
  <label class="chk"><input id="revMot" type="checkbox" onchange="sendOpts()"> Reverse motors (forward/back)</label>
  <div class="hint">Turns AWAY from the line &rarr; tick Reverse steering. Drives backwards &rarr; tick Reverse motors. Polarity is auto-detected at calibration.</div>
</div>

<div class="card">
  <h1>Live sensors (mV)</h1>
  <div class="bars" id="bars"></div>
  <div class="nums" id="nums"></div>
  <div class="metrics">
    <div>Pos: <span id="mPos">0</span></div>
    <div>Corr: <span id="mCorr">0</span></div>
    <div>L: <span id="mLeft">0</span></div>
    <div>R: <span id="mRight">0</span></div>
    <div>OK: <span id="mValid">-</span></div>
  </div>
  <div class="hint">Grey = weak/ignored channel. Orange = clipping at the ADC ceiling (~3.1 V).</div>
</div>

<div class="card">
  <h1>Log</h1>
  <div id="log"></div>
</div>

<script>
const $ = id => document.getElementById(id);
let holdUntil = 0;
let busy = false;

function hold() { holdUntil = Date.now() + 1500; }
document.querySelectorAll('input').forEach(el => {
  ['input','change','pointerdown','touchstart','mousedown','focus'].forEach(ev =>
    el.addEventListener(ev, hold, {passive:true}));
});

function post(url) { return fetch(url, {method:'POST'}); }
function calibrate(mode) { post('/calibrate?mode=' + mode).then(refresh); }
function toggle() { post('/toggle').then(refresh); }
function syncNum(id) { $(id).value = $(id + 'Range').value; }
function syncRange(id) { $(id + 'Range').value = $(id).value; }
function setBoth(id, v) { $(id).value = v; $(id + 'Range').value = v; }

function sendTuning() {
  const q = ['kp','ki','kd','speed','slow'].map(k => k + '=' + encodeURIComponent($(k).value)).join('&');
  post('/set?' + q).then(() => { holdUntil = 0; refresh(); });
}
function adjustSpeed(delta) {
  post('/speed?delta=' + delta).then(() => { holdUntil = 0; refresh(); });
}
function masterInput() { $('masterVal').textContent = $('masterRange').value; }
function masterSend() {
  post('/master?m=' + $('masterRange').value).then(() => { holdUntil = 0; refresh(); });
}
function sendOpts() {
  post('/opts?inv=' + ($('invS').checked ? 1 : 0) +
       '&steer=' + ($('revSteer').checked ? 1 : 0) +
       '&mot=' + ($('revMot').checked ? 1 : 0)).then(() => { holdUntil = 0; refresh(); });
}

function refresh() {
  if (busy) return;
  busy = true;
  fetch('/data').then(r => r.json()).then(d => {
    const badge = $('statusBadge');
    badge.textContent = d.state + (d.state === 'CALIBRATING' ? ' ' + d.calibPct + '%' : '');
    badge.className = 'status st-' + d.state;
    $('calibTag').textContent = d.calibrated ? 'saved calibration: yes' : 'saved calibration: no';
    $('startBtn').textContent = (d.state === 'RUNNING') ? 'Stop' : 'Start';
    $('startBtn').className = (d.state === 'RUNNING') ? 'btn-stop' : 'btn-start';

    if (Date.now() > holdUntil) {
      setBoth('kp', d.kp); setBoth('ki', d.ki); setBoth('kd', d.kd);
      setBoth('speed', d.speed); setBoth('slow', d.slow);
      $('masterRange').value = d.master; $('masterVal').textContent = d.master;
      $('invS').checked = d.invS; $('revSteer').checked = d.revSteer; $('revMot').checked = d.revMot;
    }

    $('mPos').textContent = d.position.toFixed(1);
    $('mCorr').textContent = d.correction.toFixed(1);
    $('mLeft').textContent = d.left;
    $('mRight').textContent = d.right;
    $('mValid').textContent = d.validSensors + '/' + d.totalSensors;

    const bars = $('bars'); bars.innerHTML = '';
    const nums = $('nums'); nums.innerHTML = '';
    d.sensors.forEach((v, i) => {
      const sat = v >= 3000;
      const b = document.createElement('div');
      b.className = 'bar' + (d.sensorValid[i] ? '' : ' bad') + (sat ? ' sat' : '');
      b.style.height = Math.max(2, v / 3300 * 100) + '%';
      bars.appendChild(b);
      const n = document.createElement('div');
      n.className = 'n' + (sat ? ' sat' : '');
      n.innerHTML = '<small>' + i + '</small>' + v;
      nums.appendChild(n);
    });

    const logDiv = $('log');
    const atBottom = logDiv.scrollTop + logDiv.clientHeight >= logDiv.scrollHeight - 10;
    logDiv.innerHTML = d.log.join('<br>');
    if (atBottom) logDiv.scrollTop = logDiv.scrollHeight;
  }).catch(() => {}).finally(() => { busy = false; });
}

setInterval(refresh, 300);
refresh();
</script>
</body>
</html>
)HTML";

String stateName(RobotState s) {
  switch (s) {
    case IDLE: return "IDLE";
    case CALIBRATING: return "CALIBRATING";
    case READY: return "READY";
    case RUNNING: return "RUNNING";
  }
  return "IDLE";
}

// ---------------- Master formula ----------------
void applyMaster(int pct) {
  pct = constrain(pct, 0, 100);
  masterPct = pct;
  float t = pct / 100.0f;
  baseSpeed = (int)(40 + 160.0f * t + 0.5f);
  Kp = (baseSpeed / 100.0f) * 0.9f;
  Kd = Kp * 12.0f;
  Ki = Kp * 0.0005f;
  turnSlow = 0.15f + 0.35f * t;
}

// ---------------- Persistence ----------------
void saveTuning() {
  prefsTune.begin("lf4t", false);
  prefsTune.putFloat("kp", Kp);
  prefsTune.putFloat("ki", Ki);
  prefsTune.putFloat("kd", Kd);
  prefsTune.putInt("spd", baseSpeed);
  prefsTune.putFloat("slow", turnSlow);
  prefsTune.putInt("mst", masterPct);
  prefsTune.putBool("inv", invertSensor);
  prefsTune.putBool("rs", reverseSteer);
  prefsTune.putBool("rm", reverseMotors);
  prefsTune.end();
}

void saveCalibration() {
  prefsCal.begin("lf4c", false);
  prefsCal.putBool("calib", true);
  for (int i = 0; i < NUM_SENSORS; i++) {
    prefsCal.putInt(("mn" + String(i)).c_str(), sensorMin[i]);
    prefsCal.putInt(("mx" + String(i)).c_str(), sensorMax[i]);
    prefsCal.putBool(("vd" + String(i)).c_str(), sensorValid[i]);
  }
  prefsCal.end();
  g_calibrated = true;
}

void loadSettings() {
  prefsTune.begin("lf4t", false);
  Kp = prefsTune.getFloat("kp", Kp);
  Ki = prefsTune.getFloat("ki", Ki);
  Kd = prefsTune.getFloat("kd", Kd);
  baseSpeed = prefsTune.getInt("spd", baseSpeed);
  turnSlow = prefsTune.getFloat("slow", turnSlow);
  masterPct = prefsTune.getInt("mst", masterPct);
  invertSensor = prefsTune.getBool("inv", invertSensor);
  reverseSteer = prefsTune.getBool("rs", reverseSteer);
  reverseMotors = prefsTune.getBool("rm", reverseMotors);
  prefsTune.end();

  prefsCal.begin("lf4c", false);
  g_calibrated = prefsCal.getBool("calib", false);
  validSensorCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (g_calibrated) {
      sensorMin[i] = prefsCal.getInt(("mn" + String(i)).c_str(), 0);
      sensorMax[i] = prefsCal.getInt(("mx" + String(i)).c_str(), 3300);
      sensorValid[i] = prefsCal.getBool(("vd" + String(i)).c_str(), true);
    } else {
      sensorMin[i] = 0;
      sensorMax[i] = 3300;
      sensorValid[i] = true;
    }
    if (sensorValid[i]) validSensorCount++;
  }
  prefsCal.end();
}

// ---------------- Web handlers ----------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  String json;
  json.reserve(6000);
  json = "{";
  json += "\"state\":\"" + stateName(g_state) + "\",";

  int calibPct = 0;
  if (g_state == CALIBRATING) {
    unsigned long total = (calibMode == 2) ? CALIB_MANUAL_MS : CALIB_AUTO_MS;
    calibPct = constrain((int)((millis() - calibStartTime) * 100 / total), 0, 100);
  }
  json += "\"calibPct\":" + String(calibPct) + ",";
  json += "\"calibrated\":" + String(g_calibrated ? "true" : "false") + ",";
  json += "\"validSensors\":" + String(validSensorCount) + ",";
  json += "\"totalSensors\":" + String(NUM_SENSORS) + ",";

  json += "\"kp\":" + String(Kp, 3) + ",";
  json += "\"ki\":" + String(Ki, 4) + ",";
  json += "\"kd\":" + String(Kd, 2) + ",";
  json += "\"speed\":" + String(baseSpeed) + ",";
  json += "\"slow\":" + String(turnSlow, 2) + ",";
  json += "\"master\":" + String(masterPct) + ",";
  json += "\"invS\":" + String(invertSensor ? "true" : "false") + ",";
  json += "\"revSteer\":" + String(reverseSteer ? "true" : "false") + ",";
  json += "\"revMot\":" + String(reverseMotors ? "true" : "false") + ",";

  json += "\"position\":" + String(g_position, 2) + ",";
  json += "\"correction\":" + String(g_correction, 2) + ",";
  json += "\"left\":" + String(g_leftSpeed) + ",";
  json += "\"right\":" + String(g_rightSpeed) + ",";

  json += "\"sensors\":[";
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) json += ",";
    json += String(sensorValue[i]);
  }
  json += "],\"sensorValid\":[";
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) json += ",";
    json += sensorValid[i] ? "true" : "false";
  }
  json += "],";

  json += "\"log\":" + getLogJson();
  json += "}";

  server.send(200, "application/json", json);
}

// Handlers only set flags / config; the control task owns the motors.
void handleCalibrate() {
  int mode = server.hasArg("mode") ? server.arg("mode").toInt() : 1;
  calibRequest = (mode == 2) ? 2 : 1;
  server.send(200, "text/plain", "ok");
}

void handleToggle() {
  toggleRequested = true;
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("kp"))    Kp = constrain(server.arg("kp").toFloat(), 0.0f, 10.0f);
  if (server.hasArg("ki"))    Ki = constrain(server.arg("ki").toFloat(), 0.0f, 1.0f);
  if (server.hasArg("kd"))    Kd = constrain(server.arg("kd").toFloat(), 0.0f, 200.0f);
  if (server.hasArg("speed")) baseSpeed = constrain(server.arg("speed").toInt(), 0, 255);
  if (server.hasArg("slow"))  turnSlow = constrain(server.arg("slow").toFloat(), 0.0f, 0.95f);
  saveTuning();
  logMsg("Saved: Kp=" + String(Kp, 2) + " Ki=" + String(Ki, 4) + " Kd=" + String(Kd, 1) +
         " Speed=" + String(baseSpeed) + " Slow=" + String(turnSlow, 2));
  server.send(200, "text/plain", "ok");
}

void handleSpeedAdjust() {
  if (server.hasArg("delta")) {
    int delta = server.arg("delta").toInt();
    baseSpeed = constrain(baseSpeed + delta, 0, 255);
    saveTuning();
    logMsg("Speed = " + String(baseSpeed));
  }
  server.send(200, "text/plain", "ok");
}

void handleMaster() {
  if (server.hasArg("m")) {
    applyMaster(server.arg("m").toInt());
    saveTuning();
    logMsg("Master " + String(masterPct) + "%: Speed=" + String(baseSpeed) + " Kp=" + String(Kp, 2) +
           " Ki=" + String(Ki, 4) + " Kd=" + String(Kd, 1) + " Slow=" + String(turnSlow, 2));
  }
  server.send(200, "text/plain", "ok");
}

void handleOpts() {
  if (server.hasArg("inv"))   invertSensor  = server.arg("inv").toInt() != 0;
  if (server.hasArg("steer")) reverseSteer  = server.arg("steer").toInt() != 0;
  if (server.hasArg("mot"))   reverseMotors = server.arg("mot").toInt() != 0;
  saveTuning();
  logMsg(String("Options: line reads ") + (invertSensor ? "LOW" : "HIGH") +
         ", steering " + (reverseSteer ? "REVERSED" : "normal") +
         ", motors " + (reverseMotors ? "REVERSED" : "normal"));
  server.send(200, "text/plain", "ok");
}

// ---------------- Motors ----------------
void setMotor(int in1, int in2, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (reverseMotors) speed = -speed;
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  }
  ledcWrite(pwmPin, speed);
}

void stopMotorsHard() {
  ledcWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMB, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  g_leftSpeed = 0;
  g_rightSpeed = 0;
}

// Linear remap: 1..255 command -> DEADBAND..255 PWM, so the real wheel speed is
// continuous through zero (no jerk when a wheel crosses 0).
int applyDeadband(int s) {
  if (s == 0) return 0;
  int mag = abs(s);
  if (mag > 255) mag = 255;
  int pwm = MOTOR_DEADBAND + (int)((mag / 255.0f) * (255 - MOTOR_DEADBAND));
  return (s > 0) ? pwm : -pwm;
}

void driveMotors(int left, int right) {
  left  = applyDeadband(left);
  right = applyDeadband(right);
  g_leftSpeed  = left;
  g_rightSpeed = right;
  setMotor(AIN1, AIN2, PWMA, left);
  setMotor(BIN1, BIN2, PWMB, right);
}

// ---------------- Sensors ----------------
void selectMuxChannel(int channel) {
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX_S2, (channel >> 2) & 0x01);
  digitalWrite(MUX_S3, (channel >> 3) & 0x01);
}

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(MUX_SETTLE_US);
    uint32_t sum = 0;
    for (int s = 0; s < ADC_SAMPLES; s++) sum += analogReadMilliVolts(MUX_SIG);
    sensorValue[i] = (int)(sum / ADC_SAMPLES);
  }
}

// 0..1, where 1 always means "on the line" (polarity applied)
float normSensor(int i) {
  int range = sensorMax[i] - sensorMin[i];
  if (range < 1) range = 1;
  float n = (float)(sensorValue[i] - sensorMin[i]) / (float)range;
  if (n < 0.0f) n = 0.0f;
  if (n > 1.0f) n = 1.0f;
  return invertSensor ? (1.0f - n) : n;
}

// Weighted average over ALL sensors (continuous, sub-sensor resolution).
// Returns -100..+100 (positive = line to the right).
float computePosition(bool &seen) {
  float wsum = 0, tot = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensorValid[i]) continue;
    float n = normSensor(i);
    if (n < NOISE_FLOOR) continue;
    n = (n - NOISE_FLOOR) / (1.0f - NOISE_FLOOR);
    wsum += sensorWeight[i] * n;
    tot += n;
  }
  seen = (tot >= MIN_LINE_SUM);
  if (!seen) return 0;
  return wsum / tot;
}

// ---------------- Calibration ----------------
void beginCalibration(int mode) {
  stopMotorsHard();

  // Detect polarity from a stationary snapshot: line should be under the middle sensors.
  long cSum = 0, eSum = 0;
  int cN = 0, eN = 0;
  int mid = NUM_SENSORS / 2;
  for (int pass = 0; pass < 8; pass++) {
    readAllSensors();
    for (int i = mid - 2; i <= mid + 1; i++) { cSum += sensorValue[i]; cN++; }
    for (int i = 0; i < 3; i++) {
      eSum += sensorValue[i];
      eSum += sensorValue[NUM_SENSORS - 1 - i];
      eN += 2;
    }
  }
  float diff = (float)cSum / cN - (float)eSum / eN;
  if (fabsf(diff) >= 30.0f) {
    invertSensor = (diff < 0);
    optsDirty = true;
    logMsg(String("Polarity: line reads ") + (invertSensor ? "LOWER" : "HIGHER") +
           " than background (center-edge = " + String((int)diff) + " mV).");
  } else {
    logMsg("Center vs edge differ by only " + String((int)fabsf(diff)) +
           " mV - polarity unchanged. Is the line under the MIDDLE sensors and the array close to the floor?");
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 5000;
    sensorMax[i] = 0;
  }
  calibMode = mode;
  calibStartTime = millis();
  g_state = CALIBRATING;
  if (mode == 2) {
    logMsg("Calibrating BY HAND for 8 s - slide the robot left/right so every sensor crosses the line.");
  } else {
    logMsg("Calibrating - robot rocks left/right. Keep the line under the array.");
  }
}

void finishCalibration() {
  validSensorCount = 0;
  int bestRange = 0, satCount = 0;
  String rng = "Range mV:";
  String bad = "";
  for (int i = 0; i < NUM_SENSORS; i++) {
    int range = sensorMax[i] - sensorMin[i];
    bool ok = (range >= SENSOR_MIN_RANGE);
    sensorValid[i] = ok;
    if (ok) {
      validSensorCount++;
    } else {
      if (bad.length()) bad += ",";
      bad += String(i);
    }
    if (range > bestRange) bestRange = range;
    if (sensorMax[i] >= ADC_SAT_MV) satCount++;
    rng += " " + String(i) + ":" + String(range);
  }
  logMsg(rng);

  if (satCount > 0) {
    logMsg("WARNING: " + String(satCount) + " sensor(s) reach ~3.1 V (ADC ceiling). If SIG can exceed 3.3 V, "
           "add a divider (10k top / 15k bottom) - clipped sensors lose contrast.");
  }
  if (bestRange < 150) {
    logMsg("Signal is weak (best swing " + String(bestRange) +
           " mV): lower the array toward the floor and give the sensor its own 5 V supply.");
  }

  if (validSensorCount >= NUM_SENSORS / 2) {
    saveCalibration();
    g_state = READY;
    if (bad.length()) {
      logMsg("Calibration OK: " + String(validSensorCount) + "/" + String(NUM_SENSORS) +
             " sensors. Ignoring: " + bad);
    } else {
      logMsg("Calibration OK: all " + String(NUM_SENSORS) + " sensors. Press Start.");
    }
  } else {
    g_state = IDLE;
    logMsg("Calibration FAILED: only " + String(validSensorCount) + "/" + String(NUM_SENSORS) +
           " sensors responded. Check the live mV values above, then retry (try 'by hand').");
  }
}

void calibrationStep() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = sensorValue[i];
    if (v < sensorMin[i]) sensorMin[i] = v;
    if (v > sensorMax[i]) sensorMax[i] = v;
  }

  unsigned long elapsed = millis() - calibStartTime;
  unsigned long total = (calibMode == 2) ? CALIB_MANUAL_MS : CALIB_AUTO_MS;

  if (calibMode == 1) {
    // left 1/4, right 1/2, left 1/4: covers both sides and ends where it started
    int dir;  // +1 rotate right, -1 rotate left
    if (elapsed < total / 4) dir = -1;
    else if (elapsed < (total * 3) / 4) dir = 1;
    else dir = -1;
    setMotor(AIN1, AIN2, PWMA, applyDeadband( dir * CALIB_SPEED));
    setMotor(BIN1, BIN2, PWMB, applyDeadband(-dir * CALIB_SPEED));
  }

  if (elapsed >= total) {
    stopMotorsHard();
    finishCalibration();
  }
}

// ---------------- Run ----------------
void doToggle() {
  if (g_state == READY) {
    integral = 0;
    lastError = 0;
    dFilt = 0;
    lastPos = 0;
    lineLostStart = 0;
    lastLoopUs = micros();
    runStartMs = millis();
    g_state = RUNNING;
    logMsg("Running!");
  } else if (g_state == RUNNING) {
    g_state = READY;
    stopMotorsHard();
    logMsg("Stopped.");
  } else if (g_state == CALIBRATING) {
    logMsg("Wait for calibration to finish.");
  } else {
    logMsg("Cannot start - calibrate first.");
  }
}

void runStep() {
  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) / 1000.0f;   // ms
  lastLoopUs = nowUs;
  if (dt < 0.2f) dt = 0.2f;
  if (dt > 50.0f) dt = 50.0f;

  bool seen = false;
  float pos = computePosition(seen);

  if (!seen) {
    if (lineLostStart == 0) lineLostStart = millis();
    if (millis() - lineLostStart > LOST_TIMEOUT_MS) {
      g_state = READY;
      stopMotorsHard();
      lineLostStart = 0;
      logMsg("Line lost for 2 s - stopped.");
      return;
    }
    // recovery: pivot toward the side the line was last seen on
    float dir = (lastPos >= 0) ? 1.0f : -1.0f;
    if (reverseSteer) dir = -dir;
    int rec = constrain((int)baseSpeed, 50, 110);
    driveMotors((int)(dir * rec), (int)(-dir * rec));
    g_position = lastPos;
    return;
  }
  lineLostStart = 0;
  lastPos = pos;

  float error = pos;
  float dRaw = (error - lastError) / dt;
  dFilt = 0.7f * dFilt + 0.3f * dRaw;          // low-pass the derivative
  lastError = error;

  if (fabsf(error) < 60.0f) {                  // anti-windup: only integrate near center
    integral += error * dt;
    integral = constrain(integral, -INT_LIMIT, INT_LIMIT);
  }

  float correction = Kp * error + Ki * integral + Kd * dFilt;
  if (reverseSteer) correction = -correction;

  float ramp = (float)(millis() - runStartMs) / (float)RAMP_MS;
  if (ramp > 1.0f) ramp = 1.0f;
  float e01 = fabsf(error) / 100.0f;
  if (e01 > 1.0f) e01 = 1.0f;
  float sp = baseSpeed * ramp * (1.0f - turnSlow * e01);

  int left  = (int)constrain(sp + correction, -255.0f, 255.0f);
  int right = (int)constrain(sp - correction, -255.0f, 255.0f);

  g_position = error;
  g_correction = correction;
  driveMotors(left, right);
}

// ---------------- OTA ----------------
void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() {
    stopMotorsHard();
    g_state = IDLE;
    logMsg("OTA update starting...");
  });
  ArduinoOTA.onEnd([]() {
    logMsg("OTA update complete. Rebooting.");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    logMsg("OTA error code: " + String((int)error));
  });
  ArduinoOTA.begin();
  logMsg("OTA ready: Arduino IDE > Tools > Port > Network Port (" + String(OTA_HOSTNAME) + ").");
}

// ---------------- Control task (owns ALL motor writes) ----------------
void controlTask(void *param) {
  bool lastCalBtn = HIGH, lastStartBtn = HIGH;
  unsigned long lastCalPress = 0, lastStartPress = 0;

  for (;;) {
    bool calBtn = digitalRead(CAL_BTN_PIN);
    if (calBtn == LOW && lastCalBtn == HIGH && millis() - lastCalPress > 300) {
      calibRequest = 1;
      lastCalPress = millis();
    }
    lastCalBtn = calBtn;

    bool startBtn = digitalRead(START_BTN_PIN);
    if (startBtn == LOW && lastStartBtn == HIGH && millis() - lastStartPress > 300) {
      toggleRequested = true;
      lastStartPress = millis();
    }
    lastStartBtn = startBtn;

    if (calibRequest != 0) {
      int mode = calibRequest;
      calibRequest = 0;
      if (g_state != CALIBRATING) beginCalibration(mode);
    }

    if (toggleRequested) {
      toggleRequested = false;
      doToggle();
    }

    readAllSensors();   // always, so the dashboard shows live values in every state

    if (g_state == CALIBRATING) {
      calibrationStep();
    } else if (g_state == RUNNING) {
      runStep();
    }

    vTaskDelay(1);
  }
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_EN, OUTPUT);
  digitalWrite(MUX_EN, LOW);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(CAL_BTN_PIN, INPUT_PULLUP);
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);
  stopMotorsHard();

  analogReadResolution(12);
  analogSetPinAttenuation(MUX_SIG, ADC_11db);

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorWeight[i] = -100.0f + (200.0f * i) / (NUM_SENSORS - 1);
  }

  loadSettings();
  g_state = g_calibrated ? READY : IDLE;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  IPAddress ip = WiFi.softAPIP();
  logMsg("AP started. SSID: " + String(AP_SSID));
  logMsg("Open http://" + ip.toString() + "/");
  if (g_calibrated) {
    logMsg("Loaded saved calibration (" + String(validSensorCount) + "/" + String(NUM_SENSORS) + " sensors OK).");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/speed", HTTP_POST, handleSpeedAdjust);
  server.on("/master", HTTP_POST, handleMaster);
  server.on("/opts", HTTP_POST, handleOpts);
  server.begin();
  logMsg("Web server started.");

  setupOTA();

  // Core 1, priority 2: isolated from the WiFi stack (core 0) so control timing stays steady.
  xTaskCreatePinnedToCore(controlTask, "control", 8192, NULL, 2, NULL, 1);

  logMsg("Ready. Calibrate with the line under the middle sensors, then Start.");
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  if (optsDirty) {
    optsDirty = false;
    saveTuning();
  }
  delay(1);
}
