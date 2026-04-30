#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// ═══════════════════════════════════════════════════
//  PINS — unchanged from your code
// ═══════════════════════════════════════════════════

#define PIN_CH1_AIL   34
#define PIN_CH2_ELE   35
#define PIN_CH3_THR   32
#define PIN_CH4_RUD   33
#define PIN_CH5_AUX1  13   // ← live P gain adjust
#define PIN_CH6_AUX2  12   // ← live D gain adjust
#define PIN_M1_FL     14
#define PIN_M2_FR     27
#define PIN_M3_RL     26
#define PIN_M4_RR     25
#define PIN_LED        2
#define I2C_SDA       21
#define I2C_SCL       22

// ═══════════════════════════════════════════════════
//  RC CALIBRATION — unchanged
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
//  ROLL PID — only one that matters here
//  AUX1 knob overrides kp live (0.0 → 10.0)
//  AUX2 knob overrides kd live (0.0 → 2.0)
// ═══════════════════════════════════════════════════

struct PID {
  float kp, ki, kd;
  float integral  = 0;
  float prevError = 0;
  float compute(float error, float dt, float currentRate) {
    integral += error * dt;
    integral  = constrain(integral, -200, 200);
    float d   = -currentRate;
    prevError = error;
    return (kp * error) + (ki * integral) + (kd * d);
  }
  void reset() { integral = 0; prevError = 0; }
};

PID rollPID = {1.5, 0.0, 0.4};  // starting point — knobs override kp/kd live

// ═══════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════

Servo m1, m2, m3, m4;
MPU6050 mpu;

bool  armed   = false;
bool  mpuOK   = false;
float roll    = 0;
float gyroRollRate = 0;

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accAngleX_offset = 0, accAngleY_offset = 0;

float thr, ail, rud, aux1, aux2;
int   motorUS[4] = {1000,1000,1000,1000};

volatile uint32_t rcRiseTime[6] = {0};
volatile int      rcRaw[6]      = {1000,1500,1000,1500,1000,1500};

uint32_t lastLoopTime = 0;

// ── ISRs — identical to your code ─────────────────

void IRAM_ATTR isr_ch1() {
  if (digitalRead(PIN_CH1_AIL)) { rcRiseTime[0]=micros(); }
  else if (rcRiseTime[0]>0) { uint32_t p=micros()-rcRiseTime[0]; if(p>=800&&p<=2200) rcRaw[0]=p; rcRiseTime[0]=0; }
}
void IRAM_ATTR isr_ch2() {
  if (digitalRead(PIN_CH2_ELE)) { rcRiseTime[1]=micros(); }
  else if (rcRiseTime[1]>0) { uint32_t p=micros()-rcRiseTime[1]; if(p>=800&&p<=2200) rcRaw[1]=p; rcRiseTime[1]=0; }
}
void IRAM_ATTR isr_ch3() {
  if (digitalRead(PIN_CH3_THR)) { rcRiseTime[2]=micros(); }
  else if (rcRiseTime[2]>0) { uint32_t p=micros()-rcRiseTime[2]; if(p>=800&&p<=2200) rcRaw[2]=p; rcRiseTime[2]=0; }
}
void IRAM_ATTR isr_ch4() {
  if (digitalRead(PIN_CH4_RUD)) { rcRiseTime[3]=micros(); }
  else if (rcRiseTime[3]>0) { uint32_t p=micros()-rcRiseTime[3]; if(p>=800&&p<=2200) rcRaw[3]=p; rcRiseTime[3]=0; }
}
void IRAM_ATTR isr_ch5() {
  if (digitalRead(PIN_CH5_AUX1)) { rcRiseTime[4]=micros(); }
  else if (rcRiseTime[4]>0) { uint32_t p=micros()-rcRiseTime[4]; if(p>=800&&p<=2200) rcRaw[4]=p; rcRiseTime[4]=0; }
}
void IRAM_ATTR isr_ch6() {
  if (digitalRead(PIN_CH6_AUX2)) { rcRiseTime[5]=micros(); }
  else if (rcRiseTime[5]>0) { uint32_t p=micros()-rcRiseTime[5]; if(p>=800&&p<=2200) rcRaw[5]=p; rcRiseTime[5]=0; }
}

// ═══════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════

float normalizeStick(int us, RCChannel &r) {
  us = constrain(us, r.min, r.max);
  return (us<=r.mid) ? (float)(us-r.mid)/(r.mid-r.min) : (float)(us-r.mid)/(r.max-r.mid);
}
float normalizeThrottle(int us, RCChannel &r) {
  us = constrain(us, r.min, r.max);
  return (float)(us-r.min)/(r.max-r.min);
}

void writeMotors(float fl, float fr, float rl, float rr) {
  motorUS[0] = 1000+(int)(constrain(fl,0,1)*1000);
  motorUS[1] = 1000+(int)(constrain(fr,0,1)*1000);
  motorUS[2] = 1000+(int)(constrain(rl,0,1)*1000);
  motorUS[3] = 1000+(int)(constrain(rr,0,1)*1000);
  m1.writeMicroseconds(motorUS[0]);
  m2.writeMicroseconds(motorUS[1]);
  m3.writeMicroseconds(motorUS[2]);
  m4.writeMicroseconds(motorUS[3]);
}

void motorsOff() {
  for(int i=0;i<4;i++) motorUS[i]=1000;
  m1.writeMicroseconds(1000); m2.writeMicroseconds(1000);
  m3.writeMicroseconds(1000); m4.writeMicroseconds(1000);
}

// ═══════════════════════════════════════════════════
//  IMU — only roll, identical axis mapping to your code
// ═══════════════════════════════════════════════════

void readIMU(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float rawAccX = ax / 8192.0;
  float rawAccY = ay / 8192.0;
  float rawAccZ = az / 8192.0;

  // Your axis remapping (90-deg sideways mount)
  float accX = -rawAccY;
  float accY =  rawAccZ;
  float accZ =  rawAccX;

  gyroRollRate = (rawAccY > 0 ? 1 : -1) * (gy / 131.0) - gyroX_offset;
  // Using your mapping: gyroX = rawGyroY
  gyroRollRate = (gy / 131.0) - gyroX_offset;

  float accelRoll = (atan2(accY, accZ) * 180.0/PI) - accAngleX_offset;

  // Snap to accel on ground, complementary filter in air
  if (thr < 0.1) {
    roll = accelRoll;
  } else {
    roll = 0.98*(roll + gyroRollRate*dt) + 0.02*accelRoll;
  }
}

// ═══════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  attachInterrupt(PIN_CH1_AIL,  isr_ch1, CHANGE);
  attachInterrupt(PIN_CH2_ELE,  isr_ch2, CHANGE);
  attachInterrupt(PIN_CH3_THR,  isr_ch3, CHANGE);
  attachInterrupt(PIN_CH4_RUD,  isr_ch4, CHANGE);
  attachInterrupt(PIN_CH5_AUX1, isr_ch5, CHANGE);
  attachInterrupt(PIN_CH6_AUX2, isr_ch6, CHANGE);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  m1.setPeriodHertz(50); m1.attach(PIN_M1_FL, 1000, 2000);
  m2.setPeriodHertz(50); m2.attach(PIN_M2_FR, 1000, 2000);
  m3.setPeriodHertz(50); m3.attach(PIN_M3_RL, 1000, 2000);
  m4.setPeriodHertz(50); m4.attach(PIN_M4_RR, 1000, 2000);
  motorsOff();

  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpuOK = true;

  // Calibrate — keep flat
  Serial.println("Calibrating IMU...");
  delay(1000);
  for (int i = 0; i < 500; i++) {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    float rawAccX=-( ay/8192.0), rawAccY=(az/8192.0), rawAccZ=(ax/8192.0);
    gyroX_offset     += gy/131.0;
    accAngleX_offset += atan2(rawAccY, rawAccZ) * 180.0/PI;
    delay(3);
  }
  gyroX_offset     /= 500;
  accAngleX_offset /= 500;

  // Cold start
  {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    float accX=-(ay/8192.0), accY=(az/8192.0), accZ=(ax/8192.0);
    roll = (atan2(accY,accZ)*180.0/PI) - accAngleX_offset;
  }

  Serial.println("ESCs arming — 3s...");
  motorsOff();
  delay(3000);
  Serial.println("Ready! Arm: THR LOW + RUD RIGHT (2s)");
  Serial.println("AUX1 knob = P gain (0-10), AUX2 knob = D gain (0-2)");

  lastLoopTime = micros();
}

// ═══════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════

void loop() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastLoopTime) / 1000000.0;
  lastLoopTime = nowUs;
  dt = constrain(dt, 0.001, 0.05);

  // RC
  thr  = normalizeThrottle(rcRaw[2], RC[2]);
  ail  = normalizeStick(rcRaw[0],    RC[0]);
  rud  = normalizeStick(rcRaw[3],    RC[3]);
  aux1 = normalizeThrottle(rcRaw[4], RC[4]);  // 0.0 - 1.0
  aux2 = normalizeThrottle(rcRaw[5], RC[5]);  // 0.0 - 1.0

  // ── Live PID tuning via knobs ─────────────────────
  rollPID.kp = aux1 * 10.0;   // AUX1 full left=0, full right=10
  rollPID.kd = aux2 *  2.0;   // AUX2 full left=0, full right=2
  // ki stays 0 during tuning — add it after P and D are good

  bool signalLost = (rcRaw[2] < 800 || rcRaw[2] > 2200);
  if (signalLost) { motorsOff(); armed = false; return; }

  // IMU
  if (mpuOK) readIMU(dt);

  // ── Arm / Disarm ──────────────────────────────────
  static uint32_t armTimer = 0, disarmTimer = 0;
  uint32_t nowMs = millis();

  if (!armed) {
    if (thr < 0.05 && rud > 0.8) {
      if (armTimer == 0) armTimer = nowMs;
      if (nowMs - armTimer > 2000) {
        armed = true;
        rollPID.reset();
        digitalWrite(PIN_LED, HIGH);
        Serial.println("ARMED");
        armTimer = 0;
      }
    } else { armTimer = 0; }
    motorsOff();
    return;
  }

  if (thr < 0.05 && rud < -0.8) {
    if (disarmTimer == 0) disarmTimer = nowMs;
    if (nowMs - disarmTimer > 2000) {
      armed = false; motorsOff();
      digitalWrite(PIN_LED, LOW);
      Serial.println("DISARMED");
      disarmTimer = 0; return;
    }
  } else { disarmTimer = 0; }

  // ── Roll PID only ─────────────────────────────────
  if (thr < 0.1) rollPID.integral = 0;  // no windup on ground

  float rollSP  = ail * 30.0;           // stick → ±30° setpoint
  float rollOut = rollPID.compute(rollSP - roll, dt, gyroRollRate);

  float pidScale = 0.01;
  rollOut = constrain(rollOut * pidScale, -0.3, 0.3);

  // ── Motor mixing — ROLL ONLY ──────────────────────
  // pitch=0, yaw=0 — only left/right motor differential
  float base = max(thr, 0.05f);

  writeMotors(
    base + rollOut,   // FL — left side  ↑ when rolling right
    base - rollOut,   // FR — right side ↓ when rolling right
    base + rollOut,   // RL — left side  ↑ when rolling right
    base - rollOut    // RR — right side ↓ when rolling right
  );

  // ── Serial output @ 10Hz ─────────────────────────
  static uint32_t lastPrint = 0;
  if (nowMs - lastPrint > 100) {
    Serial.printf(
      "ROLL:%+6.1f° | SP:%+5.1f° | ERR:%+5.1f° | OUT:%+.3f | "
      "P:%.2f D:%.3f | "
      "M1:%4d M2:%4d M3:%4d M4:%4d\n",
      roll, rollSP, rollSP-roll, rollOut,
      rollPID.kp, rollPID.kd,
      motorUS[0], motorUS[1], motorUS[2], motorUS[3]
    );
    lastPrint = nowMs;
  }
}