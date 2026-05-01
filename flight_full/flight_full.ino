#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
// ═══════════════════════════════════════════════════
//  PINS
// ═══════════════════════════════════════════════════

#define PIN_CH1_AIL   34
#define PIN_CH2_ELE   35
#define PIN_CH3_THR   32
#define PIN_CH4_RUD   33
#define PIN_CH5_AUX1  13
#define PIN_CH6_AUX2  12   // Switch: LOW = Stabilize, HIGH = Rate
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
int   rcRaw[6]     = {1000, 1500, 1000, 1500, 1000, 1500};
int   motorUS[4]   = {1000, 1000, 1000, 1000};

uint32_t lastLoopTime = 0;
uint32_t lastRCTime   = 0;

// ── Read PWM using pulseIn (simple, blocking, but reliable) ─────────────────
int readPWM(int pin) {
  return pulseIn(pin, HIGH, 25000);
}

enum LEDPattern { LED_BOOT, LED_READY, LED_ARMED, LED_NOSIGNAL, LED_SENSOR_ERR };
LEDPattern ledPattern = LED_BOOT;

// ═══════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════

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

void printDebug(
  uint32_t loopHz,
  bool signalLost,
  bool armThrOk,
  bool armRudOk,
  uint32_t armHoldMs,
  bool disarmThrOk,
  bool disarmRudOk,
  uint32_t disarmHoldMs
) {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint < 100) return;  // 10Hz update
  lastPrint = now;

  Serial.println("============================================================");
  Serial.printf("SYS   | %s | %s | %s | LOOP:%lu Hz\n",
    armed ? "ARMED" : "DISARMED",
    rateMode ? "RATE" : "STABILIZE",
    signalLost ? "NO SIGNAL" : "SIGNAL OK",
    (unsigned long)loopHz
  );
  Serial.printf("ARM   | THR<0.05:%s | RUD>0.80:%s | HOLD:%lums\n",
    armThrOk ? "YES" : "NO",
    armRudOk ? "YES" : "NO",
    (unsigned long)armHoldMs
  );
  Serial.printf("DISARM| THR<0.05:%s | RUD<-0.80:%s | HOLD:%lums\n",
    disarmThrOk ? "YES" : "NO",
    disarmRudOk ? "YES" : "NO",
    (unsigned long)disarmHoldMs
  );
  Serial.printf("RC    | CH1:%4d CH2:%4d CH3:%4d CH4:%4d CH5:%4d CH6:%4d\n",
    rcRaw[0], rcRaw[1], rcRaw[2], rcRaw[3], rcRaw[4], rcRaw[5]
  );
  Serial.printf("STICK | AIL:%+.2f ELE:%+.2f THR:%.2f RUD:%+.2f AUX1:%.2f AUX2:%.2f\n",
    ail, ele, thr, rud, aux1, aux2
  );
  Serial.printf("IMU   | R:%+.1f P:%+.1f Y:%+.1f ALT:%.1f\n", roll, pitch, yaw, altitude);
  Serial.printf("MOTOR | M1:%4d M2:%4d M3:%4d M4:%4d\n", motorUS[0], motorUS[1], motorUS[2], motorUS[3]);
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
    case LED_BOOT: // Slow blink: 500ms on, 500ms off
      if (now - t > 500) { state = !state; digitalWrite(PIN_LED, state); t = now; }
      break;
    case LED_READY: // Single quick blink: 100ms on, 900ms off
      if (now - t > 100) {
        step = (step + 1) % 10; // 10 steps of 100ms = 1s cycle
        digitalWrite(PIN_LED, (step == 0) ? HIGH : LOW);
        t = now;
      }
      break;
    case LED_ARMED: // Solid ON
      digitalWrite(PIN_LED, HIGH);
      break;
    case LED_NOSIGNAL: // Fast blink: 100ms on, 100ms off
      if (now - t > 100) { state = !state; digitalWrite(PIN_LED, state); t = now; }
      break;
    case LED_SENSOR_ERR: // Triple quick blink
      if (now - t > 100) {
        step = (step + 1) % 10;
        bool ledState = (step == 0 || step == 2 || step == 4);
        digitalWrite(PIN_LED, ledState ? HIGH : LOW);
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

  // Scaled for FS_4 and FS_250
  float rawAccX = ax / 8192.0;
  float rawAccY = ay / 8192.0;
  float rawAccZ = az / 8192.0;

  // Apply 90-deg sideways mount mappings (Swapped X/Y)
  float accX = -rawAccY;
  float accY = rawAccZ;
  float accZ = rawAccX;

  gyroRollRate  = ((gy / 131.0)) - gyroX_offset;
  gyroPitchRate = (-(gz / 131.0)) - gyroY_offset;
  gyroYawRate   = (gx / 131.0) - gyroZ_offset;

  // Simple Low-Pass Filter on Accelerometer
  static float filteredAccX = 0, filteredAccY = 0, filteredAccZ = 1;
  float lpfAlpha = 0.1;
  filteredAccX = (lpfAlpha * accX) + ((1.0 - lpfAlpha) * filteredAccX);
  filteredAccY = (lpfAlpha * accY) + ((1.0 - lpfAlpha) * filteredAccY);
  filteredAccZ = (lpfAlpha * accZ) + ((1.0 - lpfAlpha) * filteredAccZ);

  // Calculate accelerometer angles from filtered data
  float accelRoll  = (atan2(filteredAccY, filteredAccZ) * (180.0 / PI)) - accAngleX_offset;
  float accelPitch = (atan2(-filteredAccX, sqrt(filteredAccY * filteredAccY + filteredAccZ * filteredAccZ)) * (180.0 / PI)) - accAngleY_offset;

  if (thr < 0.1) {
    roll  = accelRoll;
    pitch = accelPitch;
    yaw  += gyroYawRate * dt;
  } else {
    roll  = 0.98 * (roll  + gyroRollRate  * dt) + 0.02 * accelRoll;
    pitch = 0.98 * (pitch + gyroPitchRate * dt) + 0.02 * accelPitch;
    yaw  += gyroYawRate * dt;
  }

  if (yaw >  180) yaw -= 360;
  if (yaw < -180) yaw += 360;
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
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  mpuOK = true;
  
  Serial.println("Letting IMU settle...");
  delay(1000);
  Serial.println("Calibrating IMU (Keep Board Flat and still)...");
  int num_readings = 500;
  for (int i = 0; i < num_readings; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Scale for FS_4 (8192 LSB/g) and FS_250 (131.0 LSB/dps)
    float rawAccX = ax / 8192.0;
    float rawAccY = ay / 8192.0;
    float rawAccZ = az / 8192.0;
    float rawGyroX = gx / 131.0;
    float rawGyroY = gy / 131.0;
    float rawGyroZ = gz / 131.0;

    // Apply 90-deg sideways mount mappings (Swapped X/Y)
    float calAccX = -rawAccY;
    float calAccY = rawAccZ;
    float calAccZ = rawAccX;

    gyroX_offset += (rawGyroY);
    gyroY_offset += (-rawGyroZ);
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

  // Initialize roll/pitch from accelerometer to avoid complementary filter cold-start
  {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float initAccX = -(ay / 8192.0);
    float initAccY = (az / 8192.0);
    float initAccZ = (ax / 8192.0);
    roll  = (atan2(initAccY, initAccZ) * (180.0 / PI)) - accAngleX_offset;
    pitch = (atan2(-initAccX, sqrt(initAccY * initAccY + initAccZ * initAccZ)) * (180.0 / PI)) - accAngleY_offset;
    yaw   = 0;
    Serial.printf("Initial angles: Roll=%.1f  Pitch=%.1f\n", roll, pitch);
  }

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
  static uint32_t hzTimer = 0;
  static uint32_t loops = 0;
  static uint32_t loopHz = 0;
  static uint32_t armTimer = 0;
  static uint32_t disarmTimer = 0;

  loops++;
  uint32_t nowMs = millis();
  if (nowMs - hzTimer >= 1000) {
    loopHz = loops;
    loops = 0;
    hzTimer = nowMs;
  }

  uint32_t nowUs = micros();
  float dt = (nowUs - lastLoopTime) / 1000000.0;
  lastLoopTime = nowUs;
  dt = constrain(dt, 0.001, 0.05);

  // ── Read RC ───────────────────────────────────────
  // Read PWM pulses from RC receiver
  rcRaw[0] = readPWM(PIN_CH1_AIL);
  rcRaw[1] = readPWM(PIN_CH2_ELE);
  rcRaw[2] = readPWM(PIN_CH3_THR);
  rcRaw[3] = readPWM(PIN_CH4_RUD);
  rcRaw[4] = readPWM(PIN_CH5_AUX1);
  rcRaw[5] = readPWM(PIN_CH6_AUX2);

  thr  = normalizeThrottle(rcRaw[2], RC[2]);
  ail  = normalizeStick(rcRaw[0], RC[0]);
  ele  = normalizeStick(rcRaw[1], RC[1]);
  rud  = normalizeStick(rcRaw[3], RC[3]);
  aux1 = normalizeThrottle(rcRaw[4], RC[4]);
  aux2 = normalizeThrottle(rcRaw[5], RC[5]);

  // AUX1 > 1500 = Rate mode
  rateMode = (rcRaw[4] > 1500);

  bool armThrOk = (thr < 0.05);
  bool armRudOk = (rud > 0.8);
  bool disarmThrOk = (thr < 0.05);
  bool disarmRudOk = (rud < -0.8);

  bool signalLost = (rcRaw[2] < 800 || rcRaw[2] > 2200);

  if (signalLost) {
    if (armed) {
      Serial.println("SIGNAL LOST");
      armed = false;
      motorsOff();
    }
    ledPattern = LED_NOSIGNAL;
    updateLED();
    printDebug(loopHz, true, armThrOk, armRudOk, 0, disarmThrOk, disarmRudOk, 0);
    return;
  }

  lastRCTime = nowMs;

  // ── IMU ───────────────────────────────────────────
  if (mpuOK) readIMU(dt);
  if (bmpOK)  altitude = bmp.readAltitude(1013.25);

  // ── Arm / Disarm ──────────────────────────────────
  if (!armed) {
    if (armThrOk && armRudOk) {
      if (armTimer == 0) armTimer = nowMs;
      if (nowMs - armTimer > 2000) {
        armed = true;
        stabRoll.reset(); stabPitch.reset(); stabYaw.reset();
        rateRoll.reset(); ratePitch.reset(); rateYaw.reset();
        ledPattern = LED_ARMED;
        Serial.println("ARMED");
        armTimer = 0;
      }
    } else { armTimer = 0; }

    uint32_t armHoldMs = (armTimer == 0) ? 0 : (nowMs - armTimer);
    printDebug(loopHz, false, armThrOk, armRudOk, armHoldMs, disarmThrOk, disarmRudOk, 0);
    motorsOff();
    updateLED();
    return;
  }

  if (disarmThrOk && disarmRudOk) {
    if (disarmTimer == 0) disarmTimer = nowMs;
    if (nowMs - disarmTimer > 2000) {
      armed = false;
      motorsOff();
      ledPattern = LED_READY;
      Serial.println("DISARMED");
      disarmTimer = 0;
      updateLED();
      printDebug(loopHz, false, armThrOk, armRudOk, 0, disarmThrOk, disarmRudOk, 0);
      return;
    }
  } else { disarmTimer = 0; }

  updateLED();

  // ── PID + Motor Mixing ────────────────────────────

  // Reset integral at very low throttle to prevent windup while on the ground
  if (thr < 0.1) {
    stabRoll.integral = 0;  stabPitch.integral = 0;  stabYaw.integral = 0;
    rateRoll.integral = 0;  ratePitch.integral = 0;  rateYaw.integral = 0;
  }

  float rollOut = 0, pitchOut = 0, yawOut = 0;

  float rollSP  =  ail *  30.0;
  float pitchSP = -ele *  30.0;
  float yawSP   =  rud *  90.0;

  if (!rateMode) {
    // STABILIZE MODE: Uses angle error, damped by Gyro (D-term)
    rollOut  = stabRoll.compute(rollSP - roll, dt, gyroRollRate);
    pitchOut = stabPitch.compute(pitchSP - pitch, dt, gyroPitchRate);
    yawOut   = rud * 0.2;  // direct passthrough, no PID
  } else {
    // RATE MODE: Uses pure gyro rate error, damped by gyro rate directly
    float rollErr  = rollSP  - gyroRollRate;
    float pitchErr = pitchSP - gyroPitchRate;
    float yawErr   = yawSP   - gyroYawRate;

    // D-term uses the actual gyro rate (measurement) for clean damping,
    // NOT error-derivative which amplifies noise and causes derivative kick
    rollOut  = rateRoll.compute(rollErr, dt, gyroRollRate);
    pitchOut = ratePitch.compute(pitchErr, dt, gyroPitchRate);
    yawOut   = rateYaw.compute(yawErr, dt, gyroYawRate);
  }

  float pidScale = 0.01; // Allows use of standard PID numbers (like P=5.0 instead of 0.05)
  rollOut  = constrain(rollOut  * pidScale, -0.3, 0.3);
  pitchOut = constrain(pitchOut * pidScale, -0.3, 0.3);
  yawOut   = constrain(yawOut   * pidScale, -0.3, 0.3);

  // Motor mixing — standard X-configuration
  //   +rollOut  → roll right → increase FL/RL (left side), decrease FR/RR (right side)
  //   +pitchOut → pitch back  → increase FL/FR (front),    decrease RL/RR (rear)
  //   +yawOut   → yaw right   → increase FR/RL (CW props), decrease FL/RR (CCW props)
  float idleSpeed = 0.05;  // 5% minimum spin keeps all motors alive when armed
  float base = (thr > idleSpeed) ? thr : idleSpeed;

  writeMotors(
    base + rollOut + pitchOut + yawOut,  // FL (M1, CCW)
    base - rollOut + pitchOut - yawOut,  // FR (M2, CW)
    base + rollOut - pitchOut - yawOut,  // RL (M3, CW)
    base - rollOut - pitchOut + yawOut   // RR (M4, CCW)
  );

  uint32_t disarmHoldMs = (disarmTimer == 0) ? 0 : (nowMs - disarmTimer);
  printDebug(loopHz, false, armThrOk, armRudOk, 0, disarmThrOk, disarmRudOk, disarmHoldMs);
}