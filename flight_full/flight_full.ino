#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ═══════════════════════════════════════════════════
//  WIFI CONFIG — change these
// ═══════════════════════════════════════════════════

const char* WIFI_SSID     = "donut shop mini";
const char* WIFI_PASSWORD = "hardpass";

// ═══════════════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════════════

#define PIN_CH1_AIL   34
#define PIN_CH2_ELE   35
#define PIN_CH3_THR   32
#define PIN_CH4_RUD   33
#define PIN_CH5_AUX1  12   // Switch: LOW = Stabilize, HIGH = Rate
#define PIN_CH6_AUX2  13
#define PIN_M1_FL     14
#define PIN_M2_FR     27
#define PIN_M3_RL     26
#define PIN_M4_RR     25
#define PIN_LED        2
#define I2C_SDA       21
#define I2C_SCL       22

// ═══════════════════════════════════════════════════
//  RC CALIBRATION
// ═══════════════════════════════════════════════════

struct RCChannel { int min, mid, max; };

RCChannel RC[6] = {
  {1054, 1477, 1900},
  {1184, 1520, 1856},
  {1145, 1488, 1831},
  {1097, 1507, 1917},
  {1000, 1511, 2023},
  {1000, 1511, 2023},
};

// ═══════════════════════════════════════════════════
//  PID
// ═══════════════════════════════════════════════════

struct PID {
  float kp, ki, kd;
  float integral  = 0;
  float prevError = 0;

  // Added currentRate parameter so D-term can use Gyro (Derivative of measurement) 
  // instead of Derivative of error, eliminating "derivative kick" on stick movements!
  float compute(float error, float dt, float currentRate) {
    integral += error * dt;
    integral  = constrain(integral, -200, 200);
    
    // Standard Drone PID practice: D term uses negative Gyro rate to damp oscillations
    float d = -currentRate; 
    
    prevError = error;
    return (kp * error) + (ki * integral) + (kd * d);
  }
  void reset() { integral = 0; prevError = 0; }
};

// Stabilize (Angle) PID tuned based on video's logic (P for response, D for damping)
PID stabRoll  = {1.5,  0.0,  0.4};
PID stabPitch = {1.5,  0.0,  0.4};
PID stabYaw   = {2.0,  0.0,  0.0}; // Yaw is usually tuned differently

// Rate (Acro) PID
PID rateRoll  = {1.0, 0.0, 0.02};
PID ratePitch = {1.0, 0.0, 0.02};
PID rateYaw   = {2.0, 0.0, 0.0};

// ═══════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════

Servo m1, m2, m3, m4;
MPU6050 mpu;
Adafruit_BMP280 bmp;
WebServer server(80);

bool  armed    = false;
bool  mpuOK    = false;
bool  bmpOK    = false;
bool  rateMode = false;

float roll = 0, pitch = 0, yaw = 0;
float gyroRollRate = 0, gyroPitchRate = 0, gyroYawRate = 0;
float altitude     = 0;

// Calibration offsets
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accAngleX_offset = 0, accAngleY_offset = 0;

float thr, ail, ele, rud, aux1, aux2;
int   rcRaw[6]     = {0};
int   motorUS[4]   = {1000, 1000, 1000, 1000};

uint32_t lastLoopTime = 0;
uint32_t lastRCTime   = 0;

enum LEDPattern { LED_BOOT, LED_READY, LED_ARMED, LED_NOSIGNAL, LED_SENSOR_ERR };
LEDPattern ledPattern = LED_BOOT;

// ═══════════════════════════════════════════════════
//  HTML PAGE
// ═══════════════════════════════════════════════════

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>FlightController</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }

    body {
      background: #0a0f1a;
      color: #e0e0e0;
      font-family: 'Courier New', monospace;
      padding: 16px;
    }

    h1 {
      text-align: center;
      color: #00ffdc;
      font-size: 1.4em;
      letter-spacing: 4px;
      margin-bottom: 20px;
      text-transform: uppercase;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 14px;
    }

    .card {
      background: #111827;
      border: 1px solid #1f2d40;
      border-radius: 10px;
      padding: 16px;
    }

    .card h2 {
      font-size: 0.7em;
      letter-spacing: 3px;
      color: #4a90a4;
      margin-bottom: 12px;
      text-transform: uppercase;
    }

    /* Status pill */
    .status-row {
      display: flex;
      align-items: center;
      gap: 10px;
      margin-bottom: 8px;
    }

    .pill {
      padding: 4px 14px;
      border-radius: 20px;
      font-size: 0.85em;
      font-weight: bold;
      letter-spacing: 1px;
    }

    .pill.armed    { background: #00ff64; color: #000; }
    .pill.disarmed { background: #ff3232; color: #fff; }
    .pill.ok       { background: #00ff64; color: #000; }
    .pill.fail     { background: #ff3232; color: #fff; }
    .pill.stab     { background: #ffb400; color: #000; }
    .pill.rate     { background: #00aaff; color: #000; }
    .pill.signal   { background: #00ff64; color: #000; }
    .pill.nosignal { background: #ff3232; color: #fff; }

    /* Stat rows */
    .stat {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 5px 0;
      border-bottom: 1px solid #1f2d40;
      font-size: 0.9em;
    }
    .stat:last-child { border-bottom: none; }
    .stat label { color: #6b7a8d; font-size: 0.8em; letter-spacing: 1px; }
    .stat .val  { color: #00ffdc; font-size: 1em; }

    /* Bar */
    .bar-wrap {
      background: #0a0f1a;
      border-radius: 4px;
      height: 10px;
      width: 100%;
      margin-top: 4px;
      overflow: hidden;
    }
    .bar-fill {
      height: 100%;
      border-radius: 4px;
      transition: width 0.1s;
      background: #00ff64;
    }

    /* Attitude indicator */
    .horizon-wrap {
      display: flex;
      justify-content: center;
      margin: 10px 0;
    }

    canvas { border-radius: 50%; }

    /* RC channels */
    .ch-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }

    .ch-item label { font-size: 0.72em; color: #6b7a8d; letter-spacing: 1px; }
    .ch-item .val  { color: #ffb400; font-size: 0.95em; }

    /* Motor grid */
    .motor-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 6px;
    }

    .motor-box {
      background: #0a0f1a;
      border-radius: 8px;
      padding: 10px;
      text-align: center;
    }

    .motor-box label {
      font-size: 0.7em;
      color: #6b7a8d;
      letter-spacing: 2px;
      display: block;
      margin-bottom: 6px;
    }

    .motor-box .us {
      font-size: 1.1em;
      color: #00ffdc;
      display: block;
      margin-bottom: 6px;
    }

    /* Footer */
    .footer {
      text-align: center;
      margin-top: 20px;
      font-size: 0.7em;
      color: #2a3a4a;
      letter-spacing: 2px;
    }

    #uptime { color: #4a90a4; }

    /* PID */
    .pid-grid {
      display: grid;
      grid-template-columns: 2fr 1fr 1fr 1fr;
      gap: 6px;
      align-items: center;
      font-size: 0.8em;
    }
    .pid-grid input {
      background: #0a0f1a;
      border: 1px solid #1f2d40;
      color: #00ffdc;
      padding: 4px;
      border-radius: 4px;
      width: 100%;
      text-align: center;
    }
    .btn {
      background: #00ffdc;
      color: #0a0f1a;
      border: none;
      padding: 8px;
      border-radius: 4px;
      cursor: pointer;
      font-weight: bold;
      width: 100%;
      margin-top: 14px;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    .btn:active { background: #00cca4; }

  </style>
</head>
<body>

<h1>&#9992; Flight Controller</h1>

<div class="grid">

  <!-- STATUS CARD -->
  <div class="card">
    <h2>System Status</h2>
    <div class="status-row">
      <span class="pill" id="pill-arm">DISARMED</span>
      <span class="pill" id="pill-mode">STABILIZE</span>
      <span class="pill" id="pill-signal">NO SIGNAL</span>
    </div>
    <div class="stat"><label>MPU6050</label><span class="val" id="mpu-status">—</span></div>
    <div class="stat"><label>BMP280</label><span class="val" id="bmp-status">—</span></div>
    <div class="stat"><label>UPTIME</label><span class="val" id="uptime">—</span></div>
    <div class="stat"><label>LOOP HZ</label><span class="val" id="loop-hz">—</span></div>
    <div class="stat"><label>IP ADDRESS</label><span class="val" id="ip-addr">—</span></div>
  </div>

  <!-- ATTITUDE CARD -->
  <div class="card">
    <h2>Attitude</h2>
    <div class="horizon-wrap">
      <canvas id="horizon" width="160" height="160"></canvas>
    </div>
    <div class="stat"><label>ROLL</label><span class="val" id="roll">0.0°</span></div>
    <div class="stat"><label>PITCH</label><span class="val" id="pitch">0.0°</span></div>
    <div class="stat"><label>YAW</label><span class="val" id="yaw">0.0°</span></div>
    <div class="stat"><label>ALTITUDE</label><span class="val" id="altitude">0.0 m</span></div>
  </div>

  <!-- RC INPUTS CARD -->
  <div class="card">
    <h2>RC Inputs</h2>
    <div class="ch-grid">
      <div class="ch-item">
        <label>CH1 AILERON</label>
        <div class="val" id="ch1">0</div>
        <div class="bar-wrap"><div class="bar-fill" id="bar-ch1" style="width:50%"></div></div>
      </div>
      <div class="ch-item">
        <label>CH2 ELEVATOR</label>
        <div class="val" id="ch2">0</div>
        <div class="bar-wrap"><div class="bar-fill" id="bar-ch2" style="width:50%"></div></div>
      </div>
      <div class="ch-item">
        <label>CH3 THROTTLE</label>
        <div class="val" id="ch3">0</div>
        <div class="bar-wrap"><div class="bar-fill" id="bar-ch3" style="width:0%"></div></div>
      </div>
      <div class="ch-item">
        <label>CH4 RUDDER</label>
        <div class="val" id="ch4">0</div>
        <div class="bar-wrap"><div class="bar-fill" id="bar-ch4" style="width:50%"></div></div>
      </div>
      <div class="ch-item">
        <label>CH5 AUX1 (MODE)</label>
        <div class="val" id="ch5">0</div>
        <div class="bar-wrap"><div class="bar-fill" id="bar-ch5" style="width:50%"></div></div>
      </div>
      <div class="ch-item">
        <label>CH6 AUX2</label>
        <div class="val" id="ch6">0</div>
        <div class="bar-wrap"><div class="bar-fill" id="bar-ch6" style="width:50%"></div></div>
      </div>
    </div>
  </div>

  <!-- MOTORS CARD -->
  <div class="card">
    <h2>Motor Outputs (µs)</h2>
    <div class="motor-grid">
      <div class="motor-box">
        <label>M1 FRONT LEFT</label>
        <span class="us" id="m1">1000</span>
        <div class="bar-wrap"><div class="bar-fill" id="bar-m1" style="width:0%"></div></div>
      </div>
      <div class="motor-box">
        <label>M2 FRONT RIGHT</label>
        <span class="us" id="m2">1000</span>
        <div class="bar-wrap"><div class="bar-fill" id="bar-m2" style="width:0%"></div></div>
      </div>
      <div class="motor-box">
        <label>M3 REAR LEFT</label>
        <span class="us" id="m3">1000</span>
        <div class="bar-wrap"><div class="bar-fill" id="bar-m3" style="width:0%"></div></div>
      </div>
      <div class="motor-box">
        <label>M4 REAR RIGHT</label>
        <span class="us" id="m4">1000</span>
        <div class="bar-wrap"><div class="bar-fill" id="bar-m4" style="width:0%"></div></div>
      </div>
    </div>
  </div>

  <!-- PID CARD -->
  <div class="card" style="grid-column: 1 / -1;">
    <h2>Live PID Tuning</h2>
    <div style="display:flex; flex-wrap:wrap; gap:20px;">
      <div style="flex: 1 1 280px;">
        <h3 style="font-size:0.8em; margin-bottom:8px; color:#6b7a8d;">STABILIZE MODE</h3>
        <div class="pid-grid">
          <div style="color:#6b7a8d;">Axis</div><div>P</div><div>I</div><div>D</div>
          <div>ROLL</div>
          <input type="number" id="sR_p" step="0.1"><input type="number" id="sR_i" step="0.01"><input type="number" id="sR_d" step="0.01">
          <div>PITCH</div>
          <input type="number" id="sP_p" step="0.1"><input type="number" id="sP_i" step="0.01"><input type="number" id="sP_d" step="0.01">
          <div>YAW</div>
          <input type="number" id="sY_p" step="0.1"><input type="number" id="sY_i" step="0.01"><input type="number" id="sY_d" step="0.01">
        </div>
      </div>
      <div style="flex: 1 1 280px;">
        <h3 style="font-size:0.8em; margin-bottom:8px; color:#6b7a8d;">RATE MODE</h3>
        <div class="pid-grid">
          <div style="color:#6b7a8d;">Axis</div><div>P</div><div>I</div><div>D</div>
          <div>ROLL</div>
          <input type="number" id="rR_p" step="0.1"><input type="number" id="rR_i" step="0.01"><input type="number" id="rR_d" step="0.01">
          <div>PITCH</div>
          <input type="number" id="rP_p" step="0.1"><input type="number" id="rP_i" step="0.01"><input type="number" id="rP_d" step="0.01">
          <div>YAW</div>
          <input type="number" id="rY_p" step="0.1"><input type="number" id="rY_i" step="0.01"><input type="number" id="rY_d" step="0.01">
        </div>
      </div>
    </div>
    <button class="btn" onclick="updatePID()">Push Values to Drone</button>
  </div>

</div>

<div class="footer">FS-CT6B ✦ ESP32 ✦ REFRESHING @ 10Hz</div>

<script>
  const horizonCanvas = document.getElementById('horizon');
  const ctx = horizonCanvas.getContext('2d');

  function drawHorizon(rollDeg, pitchDeg) {
    const w = horizonCanvas.width;
    const h = horizonCanvas.height;
    const cx = w / 2, cy = h / 2, r = w / 2 - 2;

    ctx.clearRect(0, 0, w, h);
    ctx.save();
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.clip();

    const rollRad  = rollDeg  * Math.PI / 180;
    const pitchPx  = pitchDeg * 2;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(rollRad);

    // Sky
    ctx.fillStyle = '#143060';
    ctx.fillRect(-w, -h - pitchPx, w * 2, h * 2);

    // Ground
    ctx.fillStyle = '#4a2e0a';
    ctx.fillRect(-w, -pitchPx, w * 2, h * 2);

    // Horizon line
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-w, -pitchPx);
    ctx.lineTo(w, -pitchPx);
    ctx.stroke();

    ctx.restore();

    // Fixed aircraft symbol
    ctx.strokeStyle = '#00ffdc';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(cx - 35, cy); ctx.lineTo(cx - 12, cy);
    ctx.moveTo(cx + 12, cy); ctx.lineTo(cx + 35, cy);
    ctx.moveTo(cx, cy - 8);  ctx.lineTo(cx, cy + 8);
    ctx.stroke();

    // Border
    ctx.strokeStyle = '#1f4060';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    ctx.restore();
  }

  function mapToPercent(us, min, max) {
    return Math.min(100, Math.max(0, (us - min) / (max - min) * 100));
  }

  let pidLoaded = false;
  const pidKeys = [
    'sR_p','sR_i','sR_d','sP_p','sP_i','sP_d','sY_p','sY_i','sY_d',
    'rR_p','rR_i','rR_d','rP_p','rP_i','rP_d','rY_p','rY_i','rY_d'
  ];

  function updatePID() {
    const payload = {};
    pidKeys.forEach(k => {
      payload[k] = parseFloat(document.getElementById(k).value) || 0.0;
    });
    fetch('/update_pid', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    }).then(r => r.json()).then(res => {
      if(res.status === 'ok') {
        const b = document.querySelector('.btn');
        b.textContent = "SAVED!";
        setTimeout(() => b.textContent = "Push Values to Drone", 2000);
      }
    });
  }

  function update() {
    fetch('/data')
      .then(r => r.json())
      .then(d => {

        if (!pidLoaded && d.pid) {
          pidKeys.forEach(k => {
             document.getElementById(k).value = d.pid[k];
          });
          pidLoaded = true;
        }

        // Pills
        const armPill = document.getElementById('pill-arm');
        armPill.textContent = d.armed ? 'ARMED' : 'DISARMED';
        armPill.className   = 'pill ' + (d.armed ? 'armed' : 'disarmed');

        const modePill = document.getElementById('pill-mode');
        modePill.textContent = d.rateMode ? 'RATE' : 'STABILIZE';
        modePill.className   = 'pill ' + (d.rateMode ? 'rate' : 'stab');

        const sigPill = document.getElementById('pill-signal');
        sigPill.textContent = d.signal ? 'SIGNAL OK' : 'NO SIGNAL';
        sigPill.className   = 'pill ' + (d.signal ? 'signal' : 'nosignal');

        // System
        document.getElementById('mpu-status').textContent = d.mpuOK ? '✅ OK' : '❌ FAIL';
        document.getElementById('bmp-status').textContent = d.bmpOK ? '✅ OK' : '❌ FAIL';
        document.getElementById('uptime').textContent     = d.uptime;
        document.getElementById('loop-hz').textContent    = d.loopHz + ' Hz';
        document.getElementById('ip-addr').textContent    = d.ip;

        // Attitude
        document.getElementById('roll').textContent     = d.roll.toFixed(1) + '°';
        document.getElementById('pitch').textContent    = d.pitch.toFixed(1) + '°';
        document.getElementById('yaw').textContent      = d.yaw.toFixed(1) + '°';
        document.getElementById('altitude').textContent = d.altitude.toFixed(1) + ' m';
        drawHorizon(d.roll, d.pitch);

        // RC channels
        const chs  = d.rcRaw;
        const mins = [1054, 1184, 1145, 1097, 1000, 1000];
        const maxs = [1900, 1856, 1831, 1917, 2023, 2023];
        for (let i = 0; i < 6; i++) {
          document.getElementById('ch' + (i+1)).textContent = chs[i] + ' µs';
          document.getElementById('bar-ch' + (i+1)).style.width =
            mapToPercent(chs[i], mins[i], maxs[i]) + '%';
        }

        // Motors
        for (let i = 0; i < 4; i++) {
          document.getElementById('m' + (i+1)).textContent = d.motorUS[i] + ' µs';
          document.getElementById('bar-m' + (i+1)).style.width =
            mapToPercent(d.motorUS[i], 1000, 2000) + '%';
        }
      })
      .catch(() => {});
  }

  drawHorizon(0, 0);
  setInterval(update, 100);
</script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════

int readPWM(int pin) {
  return pulseIn(pin, HIGH, 25000);
}

float normalizeStick(int us, RCChannel &r) {
  us = constrain(us, r.min, r.max);
  return (us <= r.mid)
    ? (float)(us - r.mid) / (r.mid - r.min)
    : (float)(us - r.mid) / (r.max - r.mid);
}

float normalizeThrottle(int us, RCChannel &r) {
  us = constrain(us, r.min, r.max);
  return (float)(us - r.min) / (r.max - r.min);
}

void writeMotors(float fl, float fr, float rl, float rr) {
  motorUS[0] = 1000 + (int)(constrain(fl, 0.0, 1.0) * 1000);
  motorUS[1] = 1000 + (int)(constrain(fr, 0.0, 1.0) * 1000);
  motorUS[2] = 1000 + (int)(constrain(rl, 0.0, 1.0) * 1000);
  motorUS[3] = 1000 + (int)(constrain(rr, 0.0, 1.0) * 1000);
  m1.writeMicroseconds(motorUS[0]);
  m2.writeMicroseconds(motorUS[1]);
  m3.writeMicroseconds(motorUS[2]);
  m4.writeMicroseconds(motorUS[3]);
}

void motorsOff() {
  for (int i = 0; i < 4; i++) motorUS[i] = 1000;
  m1.writeMicroseconds(1000);
  m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000);
  m4.writeMicroseconds(1000);
}

// ═══════════════════════════════════════════════════
//  LED (non-blocking)
// ═══════════════════════════════════════════════════

void updateLED() {
  static uint32_t t = 0;
  static int step   = 0;
  static bool state = false;
  uint32_t now = millis();

  switch (ledPattern) {
    case LED_BOOT:
      if (now - t > 500) { state = !state; digitalWrite(PIN_LED, state); t = now; }
      break;
    case LED_READY:
      if (now - t > 150) {
        step = (step + 1) % 6;
        digitalWrite(PIN_LED, step < 2 ? (step % 2 == 0) : LOW);
        t = now;
      }
      break;
    case LED_ARMED:
      digitalWrite(PIN_LED, HIGH);
      break;
    case LED_NOSIGNAL:
      if (now - t > 100) { state = !state; digitalWrite(PIN_LED, state); t = now; }
      break;
    case LED_SENSOR_ERR:
      if (now - t > 120) {
        step = (step + 1) % 10;
        digitalWrite(PIN_LED, step < 6 ? (step % 2 == 0) : LOW);
        t = now;
      }
      break;
  }
}

// ═══════════════════════════════════════════════════
//  IMU
// ═══════════════════════════════════════════════════

void readIMU(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Scaled for FS_4 and FS_500
  float rawAccX = ax / 8192.0;
  float rawAccY = ay / 8192.0;
  float rawAccZ = az / 8192.0;
  float rawGyroX = gx / 65.5;
  float rawGyroY = gy / 65.5;
  float rawGyroZ = gz / 65.5;

  // Apply 90-deg sideways mount mappings (Swapped X/Y)
  float accX = rawAccY;
  float accY = -rawAccZ;
  float accZ = rawAccX;

  gyroRollRate  = (-rawGyroY) - gyroX_offset;
  gyroPitchRate = (rawGyroZ) - gyroY_offset;
  gyroYawRate   = rawGyroX - gyroZ_offset;

  // Simple Low-Pass Filter on Accelerometer
  static float filteredAccX = 0, filteredAccY = 0, filteredAccZ = 1;
  float lpfAlpha = 0.1;
  filteredAccX = (lpfAlpha * accX) + ((1.0 - lpfAlpha) * filteredAccX);
  filteredAccY = (lpfAlpha * accY) + ((1.0 - lpfAlpha) * filteredAccY);
  filteredAccZ = (lpfAlpha * accZ) + ((1.0 - lpfAlpha) * filteredAccZ);

  // Calculate accelerometer angles from filtered data
  float accelRoll  = (atan2(filteredAccY, filteredAccZ) * (180.0 / PI)) - accAngleX_offset;
  float accelPitch = (atan2(-filteredAccX, sqrt(filteredAccY * filteredAccY + filteredAccZ * filteredAccZ)) * (180.0 / PI)) - accAngleY_offset;

  roll  = 0.98 * (roll  + gyroRollRate  * dt) + 0.02 * accelRoll;
  pitch = 0.98 * (pitch + gyroPitchRate * dt) + 0.02 * accelPitch;
  yaw  += gyroYawRate * dt;
  if (yaw >  180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

// ═══════════════════════════════════════════════════
//  WEB ROUTES
// ═══════════════════════════════════════════════════

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  // Uptime formatting
  uint32_t s   = millis() / 1000;
  uint32_t m   = s / 60; s %= 60;
  uint32_t h   = m / 60; m %= 60;
  char uptime[20];
  sprintf(uptime, "%02d:%02d:%02d", h, m, s);

  // Loop Hz
  static uint32_t lastHz = 0;
  static int       hzCount = 0;
  static float     loopHz  = 0;
  hzCount++;
  if (millis() - lastHz >= 1000) {
    loopHz  = hzCount;
    hzCount = 0;
    lastHz  = millis();
  }

  bool signal = (millis() - lastRCTime < 1000);

  StaticJsonDocument<1024> doc;
  doc["armed"]    = armed;
  doc["rateMode"] = rateMode;
  doc["signal"]   = signal;
  doc["mpuOK"]    = mpuOK;
  doc["bmpOK"]    = bmpOK;
  doc["roll"]     = roll;
  doc["pitch"]    = pitch;
  doc["yaw"]      = yaw;
  doc["altitude"] = altitude;
  doc["uptime"]   = uptime;
  doc["loopHz"]   = (int)loopHz;
  doc["ip"]       = WiFi.localIP().toString();

  JsonArray rc = doc.createNestedArray("rcRaw");
  for (int i = 0; i < 6; i++) rc.add(rcRaw[i]);

  JsonArray motors = doc.createNestedArray("motorUS");
  for (int i = 0; i < 4; i++) motors.add(motorUS[i]);

  JsonObject pid = doc.createNestedObject("pid");
  pid["sR_p"] = stabRoll.kp; pid["sR_i"] = stabRoll.ki; pid["sR_d"] = stabRoll.kd;
  pid["sP_p"] = stabPitch.kp; pid["sP_i"] = stabPitch.ki; pid["sP_d"] = stabPitch.kd;
  pid["sY_p"] = stabYaw.kp; pid["sY_i"] = stabYaw.ki; pid["sY_d"] = stabYaw.kd;
  
  pid["rR_p"] = rateRoll.kp; pid["rR_i"] = rateRoll.ki; pid["rR_d"] = rateRoll.kd;
  pid["rP_p"] = ratePitch.kp; pid["rP_i"] = ratePitch.ki; pid["rP_d"] = ratePitch.kd;
  pid["rY_p"] = rateYaw.kp; pid["rY_i"] = rateYaw.ki; pid["rY_d"] = rateYaw.kd;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleUpdatePID() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, body);
    if (!error) {
      stabRoll.kp = doc["sR_p"]; stabRoll.ki = doc["sR_i"]; stabRoll.kd = doc["sR_d"];
      stabPitch.kp = doc["sP_p"]; stabPitch.ki = doc["sP_i"]; stabPitch.kd = doc["sP_d"];
      stabYaw.kp = doc["sY_p"]; stabYaw.ki = doc["sY_i"]; stabYaw.kd = doc["sY_d"];
      
      rateRoll.kp = doc["rR_p"]; rateRoll.ki = doc["rR_i"]; rateRoll.kd = doc["rR_d"];
      ratePitch.kp = doc["rP_p"]; ratePitch.ki = doc["rP_i"]; ratePitch.kd = doc["rP_d"];
      rateYaw.kp = doc["rY_p"]; rateYaw.ki = doc["rY_i"]; rateYaw.kd = doc["rY_d"];
      
      Serial.println("PIDs Updated remotely!");
      server.send(200, "application/json", "{\"status\":\"ok\"}");
      return;
    }
  }
  server.send(400, "application/json", "{\"status\":\"error\"}");
}

// ═══════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  ledPattern = LED_BOOT;

  // RC pins
  pinMode(PIN_CH1_AIL,  INPUT);
  pinMode(PIN_CH2_ELE,  INPUT);
  pinMode(PIN_CH3_THR,  INPUT);
  pinMode(PIN_CH4_RUD,  INPUT);
  pinMode(PIN_CH5_AUX1, INPUT);
  pinMode(PIN_CH6_AUX2, INPUT);

  // ESCs
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  m1.setPeriodHertz(50); m1.attach(PIN_M1_FL, 1000, 2000);
  m2.setPeriodHertz(50); m2.attach(PIN_M2_FR, 1000, 2000);
  m3.setPeriodHertz(50); m3.attach(PIN_M3_RL, 1000, 2000);
  m4.setPeriodHertz(50); m4.attach(PIN_M4_RR, 1000, 2000);
  motorsOff();

  // I2C + sensors
  Wire.begin(I2C_SDA, I2C_SCL);

  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpuOK = true;
  
  Serial.println("Letting IMU settle...");
  delay(1000);
  Serial.println("Calibrating IMU (Keep Board Flat and still)...");
  int num_readings = 500;
  for (int i = 0; i < num_readings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Scale for FS_4 (8192 LSB/g) and FS_500 (65.5 LSB/dps)
    float rawAccX = ax / 8192.0;
    float rawAccY = ay / 8192.0;
    float rawAccZ = az / 8192.0;
    float rawGyroX = gx / 65.5;
    float rawGyroY = gy / 65.5;
    float rawGyroZ = gz / 65.5;

    // Apply 90-deg sideways mount mappings (Swapped X/Y)
    float calAccX = rawAccY;
    float calAccY = -rawAccZ;
    float calAccZ = rawAccX;

    gyroX_offset += (-rawGyroY);
    gyroY_offset += (rawGyroZ);
    gyroZ_offset += rawGyroX;

    accAngleX_offset += atan2(calAccY, calAccZ) * (180.0 / PI);
    accAngleY_offset += atan2(-calAccX, sqrt(calAccY * calAccY + calAccZ * calAccZ)) * (180.0 / PI);
    delay(3);
  }
  gyroX_offset /= num_readings;
  gyroY_offset /= num_readings;
  gyroZ_offset /= num_readings;
  accAngleX_offset /= num_readings;
  accAngleY_offset /= num_readings;
  Serial.println("IMU Calibrated!");

  if (bmp.begin(0x76)) {
    bmpOK = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 OK");
  } else {
    Serial.println("BMP280 FAILED");
  }

  ledPattern = (!mpuOK || !bmpOK) ? LED_SENSOR_ERR : LED_READY;

  // WiFi
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected! Dashboard: http://%s\n",
      WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed — continuing without dashboard");
  }

  // Web routes
  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.on("/update_pid", HTTP_POST, handleUpdatePID);
  server.begin();

  // ESC arm delay
  Serial.println("ESCs arming — 3s...");
  motorsOff();
  delay(3000);
  Serial.println("Ready. Arm: throttle LOW + rudder RIGHT (hold 2s)");

  lastLoopTime = micros();
}

// ═══════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════

void loop() {
  server.handleClient();  // serve web requests

  uint32_t now = micros();
  float dt = (now - lastLoopTime) / 1000000.0;
  lastLoopTime = now;
  dt = constrain(dt, 0.001, 0.05);

  // ── Read RC ───────────────────────────────────────
  rcRaw[0] = readPWM(PIN_CH1_AIL);
  rcRaw[1] = readPWM(PIN_CH2_ELE);
  rcRaw[2] = readPWM(PIN_CH3_THR);
  rcRaw[3] = readPWM(PIN_CH4_RUD);
  rcRaw[4] = readPWM(PIN_CH5_AUX1);
  rcRaw[5] = readPWM(PIN_CH6_AUX2);

  if (rcRaw[2] == 0) {
    if (armed) { Serial.println("⚠ SIGNAL LOST"); armed = false; motorsOff(); }
    ledPattern = LED_NOSIGNAL;
    updateLED();
    return;
  }

  lastRCTime = millis();

  thr  = normalizeThrottle(rcRaw[2], RC[2]);
  ail  = normalizeStick(rcRaw[0], RC[0]);
  ele  = normalizeStick(rcRaw[1], RC[1]);
  rud  = normalizeStick(rcRaw[3], RC[3]);
  aux1 = normalizeThrottle(rcRaw[4], RC[4]);
  aux2 = normalizeThrottle(rcRaw[5], RC[5]);

  // AUX1 > 1500 = Rate mode
  rateMode = (rcRaw[4] > 1500);

  // ── IMU ───────────────────────────────────────────
  if (mpuOK) readIMU(dt);
  if (bmpOK)  altitude = bmp.readAltitude(1013.25);

  // ── Arm / Disarm ──────────────────────────────────
  static uint32_t armTimer    = 0;
  static uint32_t disarmTimer = 0;

  if (!armed) {
    if (thr < 0.05 && rud > 0.8) {
      if (armTimer == 0) armTimer = millis();
      if (millis() - armTimer > 2000) {
        armed = true;
        stabRoll.reset(); stabPitch.reset();
        rateRoll.reset(); ratePitch.reset(); rateYaw.reset();
        ledPattern = LED_ARMED;
        Serial.println("✅ ARMED");
        armTimer = 0;
      }
    } else { armTimer = 0; }
    motorsOff();
    return;
  }

  if (thr < 0.05 && rud < -0.8) {
    if (disarmTimer == 0) disarmTimer = millis();
    if (millis() - disarmTimer > 2000) {
      armed = false;
      motorsOff();
      ledPattern = LED_READY;
      Serial.println("🔴 DISARMED");
      disarmTimer = 0;
      return;
    }
  } else { disarmTimer = 0; }

  updateLED();

  // ── PID + Motor Mixing ────────────────────────────
  float rollOut = 0, pitchOut = 0, yawOut = 0;

  float rollSP  =  ail *  30.0;
  float pitchSP = -ele *  30.0;
  float yawSP   =  rud *  90.0;

  if (!rateMode) {
    // STABILIZE MODE: Uses angle error, damped by Gyro (D-term)
    rollOut  = stabRoll.compute(rollSP - roll, dt, gyroRollRate);
    pitchOut = stabPitch.compute(pitchSP - pitch, dt, gyroPitchRate);
    yawOut   = stabYaw.compute(yawSP - yaw, dt, gyroYawRate);
  } else {
    // RATE MODE: Uses pure gyro rate error, damped by acceleration of gyro!
    // Since calculating acceleration of gyro is noisy, it's typically computed
    // from error-change directly or left simple.
    float rollErr = rollSP - gyroRollRate;
    float pitchErr = pitchSP - gyroPitchRate;
    float yawErr = yawSP - gyroYawRate;
    
    rollOut  = rateRoll.compute(rollErr, dt, (rollErr - rateRoll.prevError)/dt);
    pitchOut = ratePitch.compute(pitchErr, dt, (pitchErr - ratePitch.prevError)/dt);
    yawOut   = rateYaw.compute(yawErr, dt, (yawErr - rateYaw.prevError)/dt);
  }

  float pidScale = 0.01; // Allows use of standard PID numbers (like P=5.0 instead of 0.05)
  rollOut  = constrain(rollOut  * pidScale, -0.3, 0.3);
  pitchOut = constrain(pitchOut * pidScale, -0.3, 0.3);
  yawOut   = constrain(yawOut   * pidScale, -0.3, 0.3);

  if (armed && thr > 0.02) {
    writeMotors(
      thr - rollOut + pitchOut - yawOut,  // FL
      thr + rollOut + pitchOut + yawOut,  // FR
      thr - rollOut - pitchOut + yawOut,  // RL
      thr + rollOut - pitchOut - yawOut   // RR
    );
  } else {
    motorsOff();
  }

  // ── Serial debug ──────────────────────────────────
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.printf(
      "[%s][%s] THR:%.2f AIL:%.2f ELE:%.2f RUD:%.2f | "
      "R:%.1f P:%.1f Y:%.1f | Alt:%.1fm\n",
      armed ? "ARMED" : "DISARMED",
      rateMode ? "RATE" : "STAB",
      thr, ail, ele, rud,
      roll, pitch, yaw, altitude
    );
    lastPrint = millis();
  }
}