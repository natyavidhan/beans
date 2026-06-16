// ═══════════════════════════════════════════════════════════════
//  MPU6050/MPU6500 Calibration — Arduino Nano
//
//  Wiring: SDA→A4, SCL→A5, VCC→5V, GND→GND
//  Serial: 115200 baud
//
//  NO hardware offset registers used — all corrections in software.
//  MPU6500 registers differ from MPU6050 and cause problems.
//
//  Keep board PERFECTLY LEVEL and STILL during calibration.
//  Steps (guided on serial):
//    1. Gyro bias — 2000 samples (~8s)
//    2. Verify near-zero gyro
//    3. Accel level — 500 samples (~2s)
//    4. Prints #define constants
//    5. Live streaming with complementary filter
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>

#define MPU_ADDR 0x68

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

void readAll(int16_t* ax, int16_t* ay, int16_t* az,
             int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  *ax = (Wire.read() << 8) | Wire.read();
  *ay = (Wire.read() << 8) | Wire.read();
  *az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  *gx = (Wire.read() << 8) | Wire.read();
  *gy = (Wire.read() << 8) | Wire.read();
  *gz = (Wire.read() << 8) | Wire.read();
}

void waitForSerial(const char* msg) {
  Serial.println(msg);
  Serial.println(F("  => Send any character to continue..."));
  while (!Serial.available());
  while (Serial.available()) Serial.read();
}

int32_t gyroBiasX, gyroBiasY, gyroBiasZ;
float levelRollOffset, levelPitchOffset;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  uint8_t who = readReg(0x75);
  if (who == 0x68) {
    Serial.println(F("MPU6050 found at 0x68"));
  } else if (who == 0x70) {
    Serial.println(F("MPU6500 found at 0x68"));
  } else if (who == 0x72) {
    Serial.println(F("MPU6500 found at 0x68"));
  } else {
    Serial.print(F("ERROR: WHO_AM_I = 0x")); Serial.println(who, HEX);
    while (1) delay(1000);
  }

  // Wake up — NO reset, just clock source
  writeReg(0x6B, 0x01);
  delay(100);

  // Clear any leftover offset registers (zero them out)
  writeReg(0x13, 0x00); writeReg(0x14, 0x00);
  writeReg(0x15, 0x00); writeReg(0x16, 0x00);
  writeReg(0x17, 0x00); writeReg(0x18, 0x00);
  delay(50);

  // 500Hz sample rate, DLPF 44Hz
  writeReg(0x19, 0x01);
  writeReg(0x1A, 0x03);

  // Gyro ±500°/s (65.5 LSB/°/s), Accel ±4g (8192 LSB/g)
  writeReg(0x1B, 0x08);
  writeReg(0x1C, 0x08);
  delay(200);

  Serial.println(F("=== MPU Calibration (software offsets only) ==="));
  Serial.println();

  // ── Phase 1: Gyro Bias ─────────────────────────
  waitForSerial(
    "PHASE 1: GYRO BIAS CALIBRATION\n"
    "  Place board FLAT and PERFECTLY STILL.\n"
    "  Takes ~8 seconds, sampling at ~250Hz."
  );

  Serial.println(F("  Calibrating..."));
  int16_t ax, ay, az, gx, gy, gz;
  gyroBiasX = 0; gyroBiasY = 0; gyroBiasZ = 0;
  const int GYRO_SAMPLES = 2000;

  for (int i = 0; i < GYRO_SAMPLES; i++) {
    readAll(&ax, &ay, &az, &gx, &gy, &gz);
    gyroBiasX += gx;
    gyroBiasY += gy;
    gyroBiasZ += gz;
    if (i % 250 == 0) Serial.print('.');
    delay(4);
  }
  gyroBiasX /= GYRO_SAMPLES;
  gyroBiasY /= GYRO_SAMPLES;
  gyroBiasZ /= GYRO_SAMPLES;

  Serial.println(F(" done!"));
  Serial.print(F("  Gyro bias X: ")); Serial.println(gyroBiasX);
  Serial.print(F("  Gyro bias Y: ")); Serial.println(gyroBiasY);
  Serial.print(F("  Gyro bias Z: ")); Serial.println(gyroBiasZ);
  Serial.println();

  // ── Verify near-zero ─────────────────────────────
  Serial.println(F("  Verifying gyro is now near-zero (software subtraction)..."));
  delay(500);
  int32_t vgx=0, vgy=0, vgz=0;
  for (int i = 0; i < 500; i++) {
    readAll(&ax, &ay, &az, &gx, &gy, &gz);
    vgx += (gx - gyroBiasX);
    vgy += (gy - gyroBiasY);
    vgz += (gz - gyroBiasZ);
    delay(4);
  }
  vgx /= 500; vgy /= 500; vgz /= 500;
  Serial.print(F("  Residual X: ")); Serial.print(vgx);
  Serial.print(F(" LSB (")); Serial.print(vgx / 65.5, 2); Serial.println(F(" °/s)"));
  Serial.print(F("  Residual Y: ")); Serial.print(vgy);
  Serial.print(F(" LSB (")); Serial.print(vgy / 65.5, 2); Serial.println(F(" °/s)"));
  Serial.print(F("  Residual Z: ")); Serial.print(vgz);
  Serial.print(F(" LSB (")); Serial.print(vgz / 65.5, 2); Serial.println(F(" °/s)"));

  bool gyroOK = (abs(vgx) < 50 && abs(vgy) < 50 && abs(vgz) < 50);
  if (gyroOK) Serial.println(F("  GYRO: OK (residual < 1°/s)"));
  else Serial.println(F("  GYRO: MARGINAL — try recalibrating on a more stable surface"));
  Serial.println();

  // ── Phase 2: Accel Level ──────────────────────────
  waitForSerial(
    "PHASE 2: ACCEL LEVEL CALIBRATION\n"
    "  Keep board FLAT and LEVEL.\n"
    "  This calculates pitch/roll angle offsets.\n"
    "  Takes ~2 seconds."
  );

  Serial.println(F("  Sampling accelerometer..."));
  int32_t sax=0, say=0, saz=0;
  const int ACCEL_SAMPLES = 500;
  for (int i = 0; i < ACCEL_SAMPLES; i++) {
    readAll(&ax, &ay, &az, &gx, &gy, &gz);
    sax += ax; say += ay; saz += az;
    delay(4);
  }
  sax /= ACCEL_SAMPLES;
  say /= ACCEL_SAMPLES;
  saz /= ACCEL_SAMPLES;

  float fax = sax / 8192.0;
  float fay = say / 8192.0;
  float faz = saz / 8192.0;
  float mag = sqrt(fax*fax + fay*fay + faz*faz);

  Serial.print(F("  Accel X: ")); Serial.print(fax, 4); Serial.println(F("g"));
  Serial.print(F("  Accel Y: ")); Serial.print(fay, 4); Serial.println(F("g"));
  Serial.print(F("  Accel Z: ")); Serial.print(faz, 4); Serial.println(F("g"));
  Serial.print(F("  Magnitude: ")); Serial.print(mag, 4); Serial.println(F("g (should be ≈1.000)"));
  Serial.println();

  levelRollOffset  = atan2(fay, faz) * 180.0 / PI;
  levelPitchOffset = atan2(-fax, sqrt(fay*fay + faz*faz)) * 180.0 / PI;

  Serial.print(F("  Level roll offset:  ")); Serial.print(levelRollOffset, 4); Serial.println(F("°"));
  Serial.print(F("  Level pitch offset: ")); Serial.print(levelPitchOffset, 4); Serial.println(F("°"));
  Serial.println();

  if (faz > 0.7)       Serial.println(F("  Orientation: Z-up (standard)"));
  else if (faz < -0.7) Serial.println(F("  Orientation: Z-down (inverted)"));
  else                  Serial.println(F("  Orientation: board on its side"));

  // ── Final Report ──────────────────────────────
  Serial.println();
  Serial.println(F("╔══════════════════════════════════════════════════╗"));
  Serial.println(F("║       MPU CALIBRATION RESULTS                    ║"));
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ Gyro bias (raw LSB, subtract in software):       ║"));
  Serial.print(F("║   X = ")); Serial.println(gyroBiasX);
  Serial.print(F("║   Y = ")); Serial.println(gyroBiasY);
  Serial.print(F("║   Z = ")); Serial.println(gyroBiasZ);
  Serial.println(F("║                                                  ║"));
  Serial.println(F("║ Accel level offsets (degrees):                    ║"));
  Serial.print(F("║   Roll  = ")); Serial.print(levelRollOffset, 4); Serial.println(F("°"));
  Serial.print(F("║   Pitch = ")); Serial.print(levelPitchOffset, 4); Serial.println(F("°"));
  Serial.println(F("╚══════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("// ─── Copy-paste into flight controller ───"));
  Serial.println(F("// Gyro bias — raw LSB at ±500°/s (65.5 LSB/°/s)"));
  Serial.print(F("#define GYRO_BIAS_X ")); Serial.println(gyroBiasX);
  Serial.print(F("#define GYRO_BIAS_Y ")); Serial.println(gyroBiasY);
  Serial.print(F("#define GYRO_BIAS_Z ")); Serial.println(gyroBiasZ);
  Serial.println(F("// Accel level angles (degrees)"));
  Serial.print(F("#define LEVEL_ROLL  ")); Serial.println(levelRollOffset, 4);
  Serial.print(F("#define LEVEL_PITCH ")); Serial.println(levelPitchOffset, 4);
  Serial.println();

  // ── Live readout with complementary filter ───────────
  Serial.println(F("Streaming calibrated data. Send 'r' to recalibrate."));
  Serial.println(F("Board still = near-zero R/P/Y. Yaw drifts slowly (normal—no mag)."));
  Serial.println();

  float roll = 0, pitch = 0, yaw = 0;
  roll  = levelRollOffset ? 0 : 0;
  pitch = levelPitchOffset ? 0 : 0;
  yaw = 0;
  uint32_t lastTime = micros();

  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'r' || c == 'R') { setup(); return; }
    }

    readAll(&ax, &ay, &az, &gx, &gy, &gz);
    float dt = (micros() - lastTime) / 1000000.0;
    lastTime = micros();
    if (dt > 0.1 || dt < 0.001) dt = 0.004;

    float gx_dps = (gx - gyroBiasX) / 65.5;
    float gy_dps = (gy - gyroBiasY) / 65.5;
    float gz_dps = (gz - gyroBiasZ) / 65.5;
    float ax_g = ax / 8192.0;
    float ay_g = ay / 8192.0;
    float az_g = az / 8192.0;

    float accelRoll  = atan2(ay_g, az_g) * 180.0 / PI - levelRollOffset;
    float accelPitch = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0 / PI - levelPitchOffset;

    roll  = 0.96 * (roll + gx_dps * dt) + 0.04 * accelRoll;
    pitch = 0.96 * (pitch + gy_dps * dt) + 0.04 * accelPitch;
    yaw  += gz_dps * dt;
    if (yaw > 180)  yaw -= 360;
    if (yaw < -180)  yaw += 360;

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print(F("R:")); Serial.print(roll, 1);
      Serial.print(F("° P:")); Serial.print(pitch, 1);
      Serial.print(F("° Y:")); Serial.print(yaw, 1);
      Serial.print(F("° | Gx:")); Serial.print(gx_dps, 1);
      Serial.print(F(" Gy:")); Serial.print(gy_dps, 1);
      Serial.print(F(" Gz:")); Serial.println(gz_dps, 1);
    }

    delay(3);
  }
}

void loop() {}