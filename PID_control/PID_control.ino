#include <Wire.h>
#include <ESP32Servo.h>

// ========= PIN CONFIG =========
#define PIN_CH1_AIL   34
#define PIN_CH2_ELE   35
#define PIN_CH3_THR   32
#define PIN_CH4_RUD   33
#define PIN_CH5_AUX1  13
#define PIN_CH6_AUX2  12

#define PIN_M1_FL     14
#define PIN_M2_FR     27
#define PIN_M3_RL     26
#define PIN_M4_RR     25

#define I2C_SDA 21
#define I2C_SCL 22

// ========= MPU =========
#define MPU_ADDR 0x68

float roll = 0;
float gyroRate = 0;

// ========= PID =========
struct PID {
  float kp, ki, kd;
  float integral;
  float lastError;
};

PID anglePID = {3.0, 0.0, 0.0, 0, 0};
PID ratePID  = {0.15, 0.0, 0.003, 0, 0};

// ========= RC INPUT =========
volatile uint32_t rise[6];
volatile uint16_t ch[6] = {1500,1500,1000,1500,1500,1500};

int pins[6] = {
  PIN_CH1_AIL, PIN_CH2_ELE, PIN_CH3_THR,
  PIN_CH4_RUD, PIN_CH5_AUX1, PIN_CH6_AUX2
};

void IRAM_ATTR isr0(){ handleISR(0); }
void IRAM_ATTR isr1(){ handleISR(1); }
void IRAM_ATTR isr2(){ handleISR(2); }
void IRAM_ATTR isr3(){ handleISR(3); }
void IRAM_ATTR isr4(){ handleISR(4); }
void IRAM_ATTR isr5(){ handleISR(5); }

void handleISR(int i){
  if (digitalRead(pins[i])) {
    rise[i] = micros();
  } else {
    ch[i] = micros() - rise[i];
  }
}

// ========= ESC =========
Servo m1, m2, m3, m4;

// ========= TIME =========
unsigned long lastTime;

// ========= PID =========
float computePID(PID &pid, float error, float dt) {
  pid.integral += error * dt;
  float derivative = (error - pid.lastError) / dt;
  pid.lastError = error;
  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

// ========= MPU =========
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readMPU(float dt) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  float accX = (Wire.read()<<8 | Wire.read()) / 16384.0;
  float accY = (Wire.read()<<8 | Wire.read()) / 16384.0;
  float accZ = (Wire.read()<<8 | Wire.read()) / 16384.0;

  Wire.read(); Wire.read();

  float gyroX = (Wire.read()<<8 | Wire.read()) / 131.0;

  float accRoll = atan2(accY, accZ) * 180 / PI;

  roll = 0.98 * (roll + gyroX * dt) + 0.02 * accRoll;
  gyroRate = gyroX;
}

// ========= SETUP =========
void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  initMPU();

  // RC setup
  for (int i=0;i<6;i++) {
    pinMode(pins[i], INPUT);
  }

  attachInterrupt(pins[0], isr0, CHANGE);
  attachInterrupt(pins[1], isr1, CHANGE);
  attachInterrupt(pins[2], isr2, CHANGE);
  attachInterrupt(pins[3], isr3, CHANGE);
  attachInterrupt(pins[4], isr4, CHANGE);
  attachInterrupt(pins[5], isr5, CHANGE);

  // ESC setup
  m1.attach(PIN_M1_FL, 1000, 2000);
  m2.attach(PIN_M2_FR, 1000, 2000);
  m3.attach(PIN_M3_RL, 1000, 2000);
  m4.attach(PIN_M4_RR, 1000, 2000);

  lastTime = micros();
}

// ========= LOOP =========
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  lastTime = now;

  if (dt <= 0 || dt > 0.02) return;

  readMPU(dt);

  // ========= RC NORMALIZATION =========
  float ail = (ch[0] - 1500) / 500.0;
  float thr = (ch[2] - 1000) / 1000.0;

  thr = constrain(thr, 0, 1);

  float rollSP = ail * 10.0;

  // ========= PID =========
  float angleError = rollSP - roll;
  float rateSP = computePID(anglePID, angleError, dt);

  float rateError = rateSP - gyroRate;
  float rollOut = computePID(ratePID, rateError, dt);

  // ========= MOTOR MIX (ROLL ONLY) =========
  float m1_out = thr + rollOut;
  float m2_out = thr - rollOut;
  float m3_out = thr + rollOut;
  float m4_out = thr - rollOut;

  m1_out = constrain(m1_out, 0, 1);
  m2_out = constrain(m2_out, 0, 1);
  m3_out = constrain(m3_out, 0, 1);
  m4_out = constrain(m4_out, 0, 1);

  // ========= OUTPUT =========
  m1.writeMicroseconds(1000 + m1_out * 1000);
  m2.writeMicroseconds(1000 + m2_out * 1000);
  m3.writeMicroseconds(1000 + m3_out * 1000);
  m4.writeMicroseconds(1000 + m4_out * 1000);

  // ========= DEBUG =========
  Serial.print("Roll:"); Serial.print(roll);
  Serial.print(" SP:"); Serial.print(rollSP);
  Serial.print(" OUT:"); Serial.println(rollOut);
}