#include <Wire.h>
#include <Servo.h>
#include "config.h"

Servo esc1, esc2, esc3, esc4;

// ── RC Input (PCINT2 ISR) ──
volatile uint32_t rcRiseTime[NUM_CH] = {0};
volatile uint16_t rcPulse[NUM_CH] = {0};
volatile uint8_t lastPIND = 0;
volatile bool rcUpdated = false;

ISR(PCINT2_vect) {
  uint8_t nowPIND = PIND;
  uint8_t changed = nowPIND ^ lastPIND;
  uint32_t now = micros();
  if (changed & (1 << PIN_CH1)) { if (nowPIND & (1 << PIN_CH1)) rcRiseTime[0] = now; else { rcPulse[0] = (uint16_t)(now - rcRiseTime[0]); rcUpdated = true; } }
  if (changed & (1 << PIN_CH2)) { if (nowPIND & (1 << PIN_CH2)) rcRiseTime[1] = now; else { rcPulse[1] = (uint16_t)(now - rcRiseTime[1]); rcUpdated = true; } }
  if (changed & (1 << PIN_CH3)) { if (nowPIND & (1 << PIN_CH3)) rcRiseTime[2] = now; else { rcPulse[2] = (uint16_t)(now - rcRiseTime[2]); rcUpdated = true; } }
  if (changed & (1 << PIN_CH4)) { if (nowPIND & (1 << PIN_CH4)) rcRiseTime[3] = now; else { rcPulse[3] = (uint16_t)(now - rcRiseTime[3]); rcUpdated = true; } }
  if (changed & (1 << PIN_CH5)) { if (nowPIND & (1 << PIN_CH5)) rcRiseTime[4] = now; else { rcPulse[4] = (uint16_t)(now - rcRiseTime[4]); } }
  if (changed & (1 << PIN_CH6)) { if (nowPIND & (1 << PIN_CH6)) rcRiseTime[5] = now; else { rcPulse[5] = (uint16_t)(now - rcRiseTime[5]); } }
  lastPIND = nowPIND;
}

// ── RC Calibration ──
const RCCh rcCal[NUM_CH] = {
  {RC_CH1_MIN, RC_CH1_MID, RC_CH1_MAX},
  {RC_CH2_MIN, RC_CH2_MID, RC_CH2_MAX},
  {RC_CH3_MIN, RC_CH3_MID, RC_CH3_MAX},
  {RC_CH4_MIN, RC_CH4_MID, RC_CH4_MAX},
  {RC_CH5_MIN, RC_CH5_MID, RC_CH5_MAX},
  {RC_CH6_MIN, RC_CH6_MID, RC_CH6_MAX},
};

uint16_t rcCopy[NUM_CH];
uint32_t rcLastSeen = 0;

int convertRC(byte ch) {
  uint16_t actual = rcCopy[ch];
  uint16_t low = rcCal[ch].min;
  uint16_t center = rcCal[ch].mid;
  uint16_t high = rcCal[ch].max;
  int difference;
  if (actual < center) {
    if (actual < low) actual = low;
    difference = ((long)(center - actual) * 500L) / (center - low);
    return 1500 - difference;
  } else if (actual > center) {
    if (actual > high) actual = high;
    difference = ((long)(actual - center) * 500L) / (high - center);
    return 1500 + difference;
  }
  return 1500;
}

// ── IMU ──
float gyro_roll, gyro_pitch, gyro_yaw;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
float angle_roll, angle_pitch;
float angle_roll_acc, angle_pitch_acc;
long acc_x, acc_y, acc_z, acc_total_vector;
bool mpuOK = false;
bool gyro_angles_set = false;

void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission(true);
}
uint8_t readMPU(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

void readIMU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  acc_x = (Wire.read() << 8) | Wire.read();
  acc_y = (Wire.read() << 8) | Wire.read();
  acc_z = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  gyro_roll  = (gx - GYRO_BIAS_X);
  gyro_pitch = (gy - GYRO_BIAS_Y);
  gyro_yaw   = (gz - GYRO_BIAS_Z);
}

// ── PID ──
float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;
int pid_max_roll = 400;

float pid_p_gain_pitch = 1.3;
float pid_i_gain_pitch = 0.04;
float pid_d_gain_pitch = 18.0;
int pid_max_pitch = 400;

float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;

float roll_level_adjust, pitch_level_adjust;
bool auto_level = true;

void calculate_pid() {
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll) pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1) pid_i_mem_roll = pid_max_roll * -1;
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll) pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1) pid_output_roll = pid_max_roll * -1;
  pid_last_roll_d_error = pid_error_temp;

  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch) pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1) pid_i_mem_pitch = pid_max_pitch * -1;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch) pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1) pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw) pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1) pid_i_mem_yaw = pid_max_yaw * -1;
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw) pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1) pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;
}

// ── Motors ──
int esc_1, esc_2, esc_3, esc_4;
int throttle;

// ── Loop Timer ──
unsigned long loop_timer;

// ── LED / Arming ──
#define LED_PIN 13
byte start = 0;
uint32_t armTimer = 0;
uint32_t blinkTimer = 0;
bool blinkState = false;

void updateLED(byte mode) {
  uint16_t interval = (mode == 0) ? 500 : (mode == 1) ? 200 : 80;
  if (millis() - blinkTimer > interval) {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    blinkTimer = millis();
  }
}

// ── Setup ──
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  esc1.attach(PIN_M1, 1000, 2000);
  esc2.attach(PIN_M2, 1000, 2000);
  esc3.attach(PIN_M3, 1000, 2000);
  esc4.attach(PIN_M4, 1000, 2000);
  esc1.writeMicroseconds(1000); esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000); esc4.writeMicroseconds(1000);

  Serial.println(F("=== Flight Controller v2 (YMFC-style) ==="));
  Serial.println(F("ESCs at 1000us. Waiting 5s..."));
  delay(5000);

  for (uint8_t i = 2; i <= 7; i++) pinMode(i, INPUT_PULLUP);
  lastPIND = PIND;
  PCICR |= (1 << PCIE2);
  PCMSK2 = 0xFC;

  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  uint8_t who = readMPU(0x75);
  Serial.print(F("MPU WHO_AM_I=0x")); Serial.println(who, HEX);
  if (who == 0x68 || who == 0x70 || who == 0x72) {
    mpuOK = true;
    writeMPU(0x6B, 0x01); delay(100);
    writeMPU(0x19, 0x01);
    writeMPU(0x1A, 0x03);
    writeMPU(0x1B, GYRO_FS_SEL);
    writeMPU(0x1C, ACCEL_FS_SEL);
    delay(200);
    for (int i = 0; i < 2000; i++) { readIMU(); delay(3); }
    angle_pitch = 0; angle_roll = 0;
    gyro_angles_set = false;
    Serial.println(F("MPU OK"));
  } else {
    Serial.println(F("MPU FAIL"));
  }

  Serial.println(F("Waiting for RC..."));
  uint32_t waitStart = millis();
  while (millis() - waitStart < 10000) {
    noInterrupts();
    for (uint8_t i = 0; i < NUM_CH; i++) rcCopy[i] = rcPulse[i];
    interrupts();
    if (rcCopy[2] > 800 && rcCopy[2] < 2200) break;
    delay(100);
  }

  Serial.println(F("Arm: Throttle LOW + Yaw LEFT (hold 2s)"));
  Serial.println(F("Disarm: Throttle LOW + Yaw RIGHT (hold 2s)"));
  Serial.println();
  loop_timer = micros();
}

// ── Main Loop (4000us = 250Hz) ──

void loop() {
  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  // ── Read IMU ──
  if (mpuOK) readIMU();

  // ── Gyro low-pass filter (0.7/0.3 like YMFC) ──
  gyro_roll_input  = (gyro_roll_input  * 0.7) + ((gyro_roll  / GYRO_LSB_PER_DPS) * 0.3);
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / GYRO_LSB_PER_DPS) * 0.3);
  gyro_yaw_input   = (gyro_yaw_input   * 0.7) + ((gyro_yaw   / GYRO_LSB_PER_DPS) * 0.3);

  // ── Angle integration from gyro (65.5 LSB/°/s, dt=0.004s => 0.0000611) ──
  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll  += gyro_roll  * 0.0000611;

  // ── Cross-axis correction for yaw ──
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll  += angle_pitch * sin(gyro_yaw * 0.000001066);

  // ── Accelerometer angle (atan2 — matches our proven calibration) ──
  float ax_g = (float)acc_x / ACCEL_LSB_PER_G;
  float ay_g = (float)acc_y / ACCEL_LSB_PER_G;
  float az_g = (float)acc_z / ACCEL_LSB_PER_G;

  angle_roll_acc  = atan2(ay_g, az_g) * 57.296 - LEVEL_ROLL;
  angle_pitch_acc = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 57.296 - LEVEL_PITCH;

  // ── Complementary filter (0.96 gyro / 0.04 accel — fast enough to correct) ──
  angle_pitch = angle_pitch * 0.96 + angle_pitch_acc * 0.04;
  angle_roll  = angle_roll  * 0.96 + angle_roll_acc  * 0.04;

  // ── Level adjust for auto-level (angle * 15 => max 15° correction) ──
  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust  = angle_roll  * 15;
  if (!auto_level) { pitch_level_adjust = 0; roll_level_adjust = 0; }

  // ── Copy RC ──
  noInterrupts();
  for (uint8_t i = 0; i < NUM_CH; i++) rcCopy[i] = rcPulse[i];
  bool signalUpdated = rcUpdated;
  rcUpdated = false;
  interrupts();

  // ── Signal check ──
  if (rcCopy[2] > 800 && rcCopy[2] < 2200) rcLastSeen = millis();
  if (millis() - rcLastSeen > 1000) {
    start = 0;
    esc_1 = 1000; esc_2 = 1000; esc_3 = 1000; esc_4 = 1000;
    esc1.writeMicroseconds(esc_1); esc2.writeMicroseconds(esc_2);
    esc3.writeMicroseconds(esc_3); esc4.writeMicroseconds(esc_4);
    updateLED(2);
    return;
  }

  // ── Convert RC to 1000-2000us ──
  int rc_roll  = convertRC(0);
  int rc_pitch = convertRC(1);
  int rc_throttle = convertRC(2);
  int rc_yaw   = convertRC(3);

  // ── Kill switch (CH5 low) ──
  if (rcCopy[4] < 1200 && start == 2) {
    start = 0;
    Serial.println(F("=== KILL SWITCH ==="));
  }

  // ── Arming (YMFC-style: throttle low + yaw left = arm, throttle low + yaw right = disarm) ──
  if (start == 0 && rc_throttle < 1050 && rc_yaw < 1050) {
    start = 1;
  }
  if (start == 1 && rc_throttle < 1050 && rc_yaw > 1450) {
    start = 2;
    pid_i_mem_roll = 0; pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0; pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0; pid_last_yaw_d_error = 0;
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    gyro_angles_set = true;
    Serial.println(F("=== ARMED ==="));
  }
  if (start == 2 && rc_throttle < 1050 && rc_yaw > 1950) {
    start = 0;
    Serial.println(F("=== DISARMED ==="));
  }

  // ── PID setpoints (YMFC style) ──
  pid_roll_setpoint = 0;
  if (rc_roll > 1508) pid_roll_setpoint = rc_roll - 1508;
  else if (rc_roll < 1492) pid_roll_setpoint = rc_roll - 1492;
  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;

  pid_pitch_setpoint = 0;
  if (rc_pitch > 1508) pid_pitch_setpoint = rc_pitch - 1508;
  else if (rc_pitch < 1492) pid_pitch_setpoint = rc_pitch - 1492;
  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;

  pid_yaw_setpoint = 0;
  if (rc_throttle > 1050) {
    if (rc_yaw > 1508) pid_yaw_setpoint = (rc_yaw - 1508) / 3.0;
    else if (rc_yaw < 1492) pid_yaw_setpoint = (rc_yaw - 1492) / 3.0;
  }

  calculate_pid();

  // ── Motor mixing (YMFC style) ──
  // M1=Front-Left CCW, M2=Front-Right CW, M3=Rear-Left CW, M4=Rear-Right CCW
  throttle = rc_throttle;

  if (start == 2) {
    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if (esc_1 > 2000) esc_1 = 2000;
    if (esc_2 > 2000) esc_2 = 2000;
    if (esc_3 > 2000) esc_3 = 2000;
    if (esc_4 > 2000) esc_4 = 2000;
  } else {
    esc_1 = 1000; esc_2 = 1000; esc_3 = 1000; esc_4 = 1000;
  }

  esc1.writeMicroseconds(esc_1);
  esc2.writeMicroseconds(esc_2);
  esc3.writeMicroseconds(esc_3);
  esc4.writeMicroseconds(esc_4);

  updateLED(start == 2 ? 1 : 0);

  // ── Debug at 5Hz ──
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.print(F("ST:")); Serial.print(start);
    Serial.print(F(" RC:")); Serial.print(rcCopy[0]); Serial.print(',');
    Serial.print(rcCopy[1]); Serial.print(','); Serial.print(rcCopy[2]); Serial.print(',');
    Serial.print(rcCopy[3]); Serial.print(','); Serial.print(rcCopy[4]); Serial.print(',');
    Serial.print(rcCopy[5]);
    Serial.print(F("|IMU:")); Serial.print(angle_roll, 1); Serial.print(',');
    Serial.print(angle_pitch, 1); Serial.print(',');
    Serial.print(gyro_yaw_input, 1); Serial.print(',');
    Serial.print(gyro_roll_input, 1); Serial.print(',');
    Serial.print(gyro_pitch_input, 1); Serial.print(',');
    Serial.print(gyro_yaw_input, 1);
    Serial.print(F("|PID:")); Serial.print(pid_roll_setpoint, 1); Serial.print(',');
    Serial.print(pid_pitch_setpoint, 1); Serial.print(',');
    Serial.print(pid_yaw_setpoint, 1); Serial.print(',');
    Serial.print(pid_output_roll, 0); Serial.print(',');
    Serial.print(pid_output_pitch, 0); Serial.print(',');
    Serial.print(pid_output_yaw, 0);
    Serial.print(F("|ESC:")); Serial.print(esc_1); Serial.print(',');
    Serial.print(esc_2); Serial.print(','); Serial.print(esc_3); Serial.print(',');
    Serial.print(esc_4);
    Serial.println();
  }
}