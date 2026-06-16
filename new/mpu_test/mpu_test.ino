// ═══════════════════════════════════════════════════════════════
//  MPU6050/MPU6500 Test — Arduino Nano
//
//  Wiring:
//    MPU6050 VCC → 5V (or 3.3V, module dependent)
//    MPU6050 GND → GND
//    MPU6050 SDA → A4
//    MPU6050 SCL → A5
//
//  Serial: 115200 baud
//
//  This sketch:
//    1. Scans I2C for MPU6050
//    2. Reads WHO_AM_I
//    3. Resets device, configures ±4g accel / ±500°/s gyro
//    4. Runs factory self-test
//    5. Streams raw data at ~100Hz for visual verification
//
//  Send commands over serial:
//    'r' — reset MPU6050
//    's' — run self-test
//    't' — toggle temperature print
//    'h' — print this help
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>

#define MPU_ADDR 0x68

#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG   0x1B
#define REG_ACCEL_CFG  0x1C
#define REG_PWR_MGMT1  0x6B
#define REG_PWR_MGMT2  0x6C
#define REG_WHO_AM_I   0x75

#define GYRO_FS_250   0x00
#define GYRO_FS_500   0x08
#define GYRO_FS_1000  0x10
#define GYRO_FS_2000  0x18

#define ACCEL_FS_2    0x00
#define ACCEL_FS_4    0x08
#define ACCEL_FS_8    0x10
#define ACCEL_FS_16   0x18

bool showTemp = false;
uint32_t errCount = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  if (Wire.endTransmission() != 0) errCount++;
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.read();
}

int16_t readReg16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

void readAllRaw(int16_t* ax, int16_t* ay, int16_t* az,
                int16_t* gx, int16_t* gy, int16_t* gz,
                int16_t* temp) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  *ax   = (Wire.read() << 8) | Wire.read();
  *ay   = (Wire.read() << 8) | Wire.read();
  *az   = (Wire.read() << 8) | Wire.read();
  *temp = (Wire.read() << 8) | Wire.read();
  *gx   = (Wire.read() << 8) | Wire.read();
  *gy   = (Wire.read() << 8) | Wire.read();
  *gz   = (Wire.read() << 8) | Wire.read();
}

void scanI2C() {
  Serial.println(F("Scanning I2C bus..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Found device at 0x"));
      if (addr < 0x10) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  Serial.print(found);
  Serial.println(F(" device(s) found"));
}

void printConfig() {
  Serial.println(F("--- MPU6050 Config ---"));
  Serial.print(F("  PWR_MGMT_1: 0x")); Serial.println(readReg(REG_PWR_MGMT1), HEX);
  Serial.print(F("  PWR_MGMT_2: 0x")); Serial.println(readReg(REG_PWR_MGMT2), HEX);
  Serial.print(F("  SMPLRT_DIV: 0x")); Serial.println(readReg(REG_SMPLRT_DIV), HEX);
  Serial.print(F("  CONFIG:     0x")); Serial.println(readReg(REG_CONFIG), HEX);
  Serial.print(F("  GYRO_CFG:   0x")); Serial.println(readReg(REG_GYRO_CFG), HEX);
  Serial.print(F("  ACCEL_CFG:  0x")); Serial.println(readReg(REG_ACCEL_CFG), HEX);

  uint8_t gf = readReg(REG_GYRO_CFG) & 0x18;
  uint8_t af = readReg(REG_ACCEL_CFG) & 0x18;
  Serial.print(F("  Gyro range:  "));
  if (gf == 0x00) Serial.println(F("±250°/s"));
  else if (gf == 0x08) Serial.println(F("±500°/s"));
  else if (gf == 0x10) Serial.println(F("±1000°/s"));
  else Serial.println(F("±2000°/s"));
  Serial.print(F("  Accel range: "));
  if (af == 0x00) Serial.println(F("±2g"));
  else if (af == 0x08) Serial.println(F("±4g"));
  else if (af == 0x10) Serial.println(F("±8g"));
  else Serial.println(F("±16g"));

  Serial.print(F("  Gyro offsets: X=")); Serial.print(readReg16(0x13));
  Serial.print(F(" Y=")); Serial.print(readReg16(0x15));
  Serial.print(F(" Z=")); Serial.println(readReg16(0x17));
  Serial.println();
}

void runSelfTest() {
  Serial.println(F("Running MPU6050 self-test..."));

  int16_t gx_st=0, gy_st=0, gz_st=0, ax_st=0, ay_st=0, az_st=0;
  int16_t gx=0, gy=0, gz=0, ax=0, ay=0, az=0;
  int16_t tmp;

  // Read self-test factory values
  uint8_t self_test[4];
  self_test[0] = readReg(0x0D); // XA_TEST
  self_test[1] = readReg(0x0E); // YA_TEST
  self_test[2] = readReg(0x10); // ZA_TEST
  self_test[3] = readReg(0x13); // XG/YG/ZG_TEST combined

  Serial.print(F("  Factory self-test bytes: "));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(self_test[i], HEX); Serial.print(' ');
  }
  Serial.println();

  // Enable self-test on all axes
  writeReg(0x36, 0x0E); // XA_ST=1, YA_ST=1, ZA_ST=1 (bits 7,5,3)
  // Also enable gyro self-test
  uint8_t ctrl = readReg(0x36);
  writeReg(0x36, ctrl | 0x3F); // enable all self-test bits
  delay(50);

  // Average 200 readings with self-test enabled
  for (int i = 0; i < 200; i++) {
    int16_t tax, tay, taz, tgx, tgy, tgz;
    readAllRaw(&tax, &tay, &taz, &tgx, &tgy, &tgz, &tmp);
    ax_st += tax; ay_st += tay; az_st += taz;
    gx_st += tgx; gy_st += tgy; gz_st += tgz;
    delay(3);
  }
  ax_st /= 200; ay_st /= 200; az_st /= 200;
  gx_st /= 200; gy_st /= 200; gz_st /= 200;

  // Disable self-test
  writeReg(0x36, 0x00);
  delay(50);

  // Average 200 normal readings
  for (int i = 0; i < 200; i++) {
    int16_t tax, tay, taz, tgx, tgy, tgz;
    readAllRaw(&tax, &tay, &taz, &tgx, &tgy, &tgz, &tmp);
    ax += tax; ay += tay; az += taz;
    gx += tgx; gy += tgy; gz += tgz;
    delay(3);
  }
  ax /= 200; ay /= 200; az /= 200;
  gx /= 200; gy /= 200; gz /= 200;

  Serial.println(F("  Normal vs Self-Test response:"));
  Serial.print(F("    Accel X: ")); Serial.print(ax); Serial.print(F(" / ")); Serial.println(ax_st);
  Serial.print(F("    Accel Y: ")); Serial.print(ay); Serial.print(F(" / ")); Serial.println(ay_st);
  Serial.print(F("    Accel Z: ")); Serial.print(az); Serial.print(F(" / ")); Serial.println(az_st);
  Serial.print(F("    Gyro  X: ")); Serial.print(gx); Serial.print(F(" / ")); Serial.println(gx_st);
  Serial.print(F("    Gyro  Y: ")); Serial.print(gy); Serial.print(F(" / ")); Serial.println(gy_st);
  Serial.print(F("    Gyro  Z: ")); Serial.print(gz); Serial.print(F(" / ")); Serial.println(gz_st);
  Serial.println();

  // Check if response is within spec (self-test should produce significant change)
  bool pass = true;
  if (abs(ax_st - ax) < 300) pass = false;
  if (abs(ay_st - ay) < 300) pass = false;
  if (abs(az_st - az) < 300) pass = false;

  if (pass) Serial.println(F("  SELF-TEST: PASSED"));
  else      Serial.println(F("  SELF-TEST: FAILED — weak or no response"));

  // Restore normal config
  configureMPU();
}

void configureMPU() {
  // Wake up, use PLL with X gyro clock (most stable)
  writeReg(REG_PWR_MGMT1, 0x01);
  delay(100);

  // Sample rate: 1kHz / (1+1) = 500Hz
  writeReg(REG_SMPLRT_DIV, 0x01);

  // DLPF config: ~44Hz bandwidth, 4.9ms delay (good balance for drones)
  writeReg(REG_CONFIG, 0x03);

  // Gyro: ±500°/s (sensitivity 65.5 LSB/°/s)
  writeReg(REG_GYRO_CFG, GYRO_FS_500);

  // Accel: ±4g (sensitivity 8192 LSB/g)
  writeReg(REG_ACCEL_CFG, ACCEL_FS_4);

  delay(100);
}

void verifyOrientation() {
  int16_t ax, ay, az, gx, gy, gz, tmp;
  int32_t sax=0, say=0, saz=0;
  int16_t samples = 200;

  Serial.println(F("Reading accelerometer to detect orientation..."));
  Serial.println(F("Hold the board LEVEL and STILL for 1 second."));

  for (int i = 0; i < samples; i++) {
    readAllRaw(&ax, &ay, &az, &gx, &gy, &gz, &tmp);
    sax += ax; say += ay; saz += az;
    delay(4);
  }
  sax /= samples; say /= samples; saz /= samples;

  Serial.print(F("  Avg Accel — X: ")); Serial.print(sax);
  Serial.print(F("  Y: ")); Serial.print(say);
  Serial.print(F("  Z: ")); Serial.println(saz);
  Serial.println();

  Serial.println(F("  Expected when level (Z-up):"));
  Serial.println(F("    X ≈ 0, Y ≈ 0, Z ≈ +8192 (≈1g upward)"));
  Serial.println();

  if (saz > 5000) {
    Serial.println(F("  => Z-axis points UP — standard orientation"));
  } else if (saz < -5000) {
    Serial.println(F("  => Z-axis points DOWN — board is inverted!"));
  } else if (sax > 5000 || sax < -5000) {
    Serial.println(F("  => Board is on its SIDE (X dominant)"));
  } else if (say > 5000 || say < -5000) {
    Serial.println(F("  => Board is on its SIDE (Y dominant)"));
  } else {
    Serial.println(F("  => WARNING: No axis near ±1g — check wiring!"));
  }

  Serial.print(F("  Gravity magnitude: "));
  float mag = sqrt((float)sax*sax + (float)say*say + (float)saz*saz);
  Serial.print(mag / 8192.0, 3);
  Serial.println(F("g (should be ≈1.000)"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("=== MPU6050 Diagnostics ==="));
  Serial.println();

  // I2C scan
  scanI2C();
  Serial.println();

  // WHO_AM_I
  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.print(F("WHO_AM_I: 0x"));
  Serial.println(who, HEX);
  if (who == 0x68) {
    Serial.println(F("  MPU6050 detected!"));
  } else if (who == 0x70) {
    Serial.println(F("  MPU6500 detected! (register-compatible)"));
  } else if (who == 0x72) {
    Serial.println(F("  MPU6500 detected! (register-compatible)"));
  } else {
    Serial.println(F("  UNEXPECTED — check wiring!"));
    while (1) delay(1000);
  }
  Serial.println();

  // Configure
  configureMPU();
  Serial.println(F("Configured: ±500°/s gyro, ±4g accel, 500Hz sample, DLPF 44Hz"));
  Serial.println();

  // Print config
  printConfig();

  // Orientation check
  verifyOrientation();

  // Self-test
  runSelfTest();

  Serial.println(F("============================="));
  Serial.println(F("Streaming raw data. Commands:"));
  Serial.println(F("  'r' = reset MPU"));
  Serial.println(F("  's' = run self-test"));
  Serial.println(F("  't' = toggle temperature"));
  Serial.println(F("  'c' = print config"));
  Serial.println(F("  'o' = orientation check"));
  Serial.println(F("============================="));
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') { configureMPU(); Serial.println(F("MPU reset.")); }
    if (c == 's' || c == 'S') { runSelfTest(); }
    if (c == 't' || c == 'T') { showTemp = !showTemp; Serial.print(F("Temp: ")); Serial.println(showTemp ? "ON" : "OFF"); }
    if (c == 'c' || c == 'C') { printConfig(); }
    if (c == 'o' || c == 'O') { verifyOrientation(); }
  }

  int16_t ax, ay, az, gx, gy, gz, temp;
  readAllRaw(&ax, &ay, &az, &gx, &gy, &gz, &temp);

  // Convert to engineering units
  float gx_dps = gx / 65.5;
  float gy_dps = gy / 65.5;
  float gz_dps = gz / 65.5;
  float ax_g   = ax / 8192.0;
  float ay_g   = ay / 8192.0;
  float az_g   = az / 8192.0;

  Serial.print(F("Accel(g) X:")); Serial.print(ax_g, 3);
  Serial.print(F(" Y:")); Serial.print(ay_g, 3);
  Serial.print(F(" Z:")); Serial.print(az_g, 3);
  Serial.print(F(" | Gyro(°/s) X:")); Serial.print(gx_dps, 1);
  Serial.print(F(" Y:")); Serial.print(gy_dps, 1);
  Serial.print(F(" Z:")); Serial.print(gz_dps, 1);

  if (showTemp) {
    float tempC = temp / 340.0 + 36.53;
    Serial.print(F(" | Temp:")); Serial.print(tempC, 1); Serial.print('C');
  }

  // Sanity flags
  float accelMag = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  if (accelMag < 0.8 || accelMag > 1.2) Serial.print(F(" ⚠ ACCEL MAG"));
  if (abs(gx_dps) > 400 || abs(gy_dps) > 400 || abs(gz_dps) > 400) Serial.print(F(" ⚠ HIGH GYRO"));

  Serial.println();
  delay(50);
}