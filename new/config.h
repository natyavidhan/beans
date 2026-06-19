#ifndef CONFIG_H
#define CONFIG_H

// ── RC Receiver Pins (FS-R6B, PWM, PCINT on PORTD) ──
#define PIN_CH1 2   // Roll / Aileron
#define PIN_CH2 3   // Pitch / Elevator
#define PIN_CH3 4   // Throttle
#define PIN_CH4 5   // Yaw / Rudder
#define PIN_CH5 6   // AUX1
#define PIN_CH6 7   // AUX2

#define NUM_CH 6

// ── RC Calibration struct ──
struct RCCh { uint16_t min, mid, max; };

// ── RC Calibration (from receiver_calibrate) ──
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
#define RC_CH5_MID  1496
#define RC_CH5_MAX  2004

#define RC_CH6_MIN  992
#define RC_CH6_MID  1496
#define RC_CH6_MAX  2004

// ── Motor Pins (ESC PWM) ──
#define PIN_M1  9   // Front-Left  (CCW)
#define PIN_M2  10  // Front-Right (CW)
#define PIN_M3  11  // Rear-Left   (CW)
#define PIN_M4  12  // Rear-Right  (CCW)

// ── MPU6500 (I2C) ──
#define MPU_ADDR 0x68

// Gyro bias (raw LSB at ±500°/s)
#define GYRO_BIAS_X  142
#define GYRO_BIAS_Y  22
#define GYRO_BIAS_Z  -37

// Accel level offsets (degrees)
#define LEVEL_ROLL   -0.1848
#define LEVEL_PITCH  2.9959

// ── IMU Config ──
#define GYRO_FS_SEL  0x08   // ±500°/s
#define ACCEL_FS_SEL 0x08   // ±4g
#define GYRO_LSB_PER_DPS  65.5f
#define ACCEL_LSB_PER_G   8192.0f
#define LOOP_PERIOD_S     0.004f    // 250Hz
#define GYRO_ANGLE_DT     (LOOP_PERIOD_S / GYRO_LSB_PER_DPS)   // ≈0.0000611
#define GYRO_YAW_RAD      (GYRO_ANGLE_DT * 0.01745329f)        // ≈0.000001066

#endif