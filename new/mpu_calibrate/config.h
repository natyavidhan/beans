// ═══════════════════════════════════════════════════════════════
//  Shared flight controller configuration — Arduino Nano
//
//  All sketches include this so pin mappings, RC calibration,
//  and IMU offsets stay consistent.
// ═══════════════════════════════════════════════════════════════

#ifndef CONFIG_H
#define CONFIG_H

// ── RC Receiver Pins (FS-R6B, PWM, PCINT on PORTD) ──
#define PIN_CH1 2   // Roll / Aileron
#define PIN_CH2 3   // Pitch / Elevator
#define PIN_CH3 4   // Throttle
#define PIN_CH4 5   // Yaw / Rudder
#define PIN_CH5 6   // AUX1 (rotary)
#define PIN_CH6 7   // AUX2 (rotary)

#define NUM_CH 6

// ── RC Calibration (from receiver_calibrate) ──
// {min, mid, max} in microseconds
#define RC_CH1_MIN  1048
#define RC_CH1_MID  1490
#define RC_CH1_MAX  1872

#define RC_CH2_MIN  1144
#define RC_CH2_MID  1449
#define RC_CH2_MAX  1816

#define RC_CH3_MIN  1120
#define RC_CH3_MID  1378
#define RC_CH3_MAX  1804

#define RC_CH4_MIN  1084
#define RC_CH4_MID  1513
#define RC_CH4_MAX  1912

#define RC_CH5_MIN  992
#define RC_CH5_MID  1496  // midpoint estimate (rotary knob)
#define RC_CH5_MAX  2004

#define RC_CH6_MIN  992
#define RC_CH6_MID  1496  // midpoint estimate (rotary knob)
#define RC_CH6_MAX  2004

// ── Motor Pins (ESC PWM) ──
#define PIN_M1  9   // Front-Left  (CCW)
#define PIN_M2  10  // Front-Right (CW)
#define PIN_M3  11  // Rear-Left   (CW)
#define PIN_M4  12  // Rear-Right  (CCW)

// ── MPU6500 (I2C) ──
#define MPU_ADDR 0x68

// Gyro bias (raw LSB at ±500°/s, 65.5 LSB/°/s)
#define GYRO_BIAS_X  135
#define GYRO_BIAS_Y  33
#define GYRO_BIAS_Z  -27

// Accel level offsets (degrees, from mpu_calibrate)
#define LEVEL_ROLL   0.7913
#define LEVEL_PITCH -2.5562

// ── IMU Config ──
#define GYRO_FS_SEL  0x08   // ±500°/s
#define ACCEL_FS_SEL 0x08   // ±4g
#define GYRO_LSB_PER_DPS  65.5f
#define ACCEL_LSB_PER_G   8192.0f

#endif