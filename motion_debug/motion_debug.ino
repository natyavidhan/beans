#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>

MPU6050 mpu;
Adafruit_BMP280 bmp;

// Timing
unsigned long prevTime = 0;
float dt;

// Raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Converted values
float accX, accY;
float gyroX, gyroY, gyroZ;

// Angles
float accAngleX, accAngleY;
float angleX = 0, angleY = 0, angleZ = 0;

// Calibration offsets
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accAngleX_offset = 0, accAngleY_offset = 0;

// Complementary filter constant
float alpha = 0.98;

// BMP
float altitude;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22); // SDA, SCL

  // MPU6050 init
  mpu.initialize();
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed");
  //   while (1);
  // }

  // BMP280 init
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 not found");
    // while (1);
  }

  // Let the IMU sensors settle before capturing calibration data
  delay(1000);

  // Serial.println("Calibrating IMU (Keep Board Flat and still)...");
  int num_readings = 500;
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Apply exact same axis mapping
    float rawAccX = ax / 16384.0;
    float rawAccY = ay / 16384.0;
    float rawAccZ = az / 16384.0;
    float rawGyroX = gx / 131.0;
    float rawGyroY = gy / 131.0;
    float rawGyroZ = gz / 131.0;

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

  // Serial.println("Sensors initialized and calibrated...");
// Serial.println("angleX,angleY,accX,accY,gyroX,gyroY,altitude");
}

void loop() {
  // Time delta
  unsigned long currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  // Read MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Axis mapping for 90-degree sideways mount
  // Adjust these assignments if the pitch/roll move in the wrong direction
  float rawAccX = ax / 16384.0;
  float rawAccY = ay / 16384.0;
  float rawAccZ = az / 16384.0;

  float rawGyroX = gx / 131.0;
  float rawGyroY = gy / 131.0;
  float rawGyroZ = gz / 131.0;

  // Swap axes based on mount orientation to keep Z pointing UP
  // Swapped X and Y because pitch was acting on the right side
  accX = rawAccY;
  accY = -rawAccZ;
  float accZ = rawAccX;

  gyroX = (-rawGyroY) - gyroX_offset;
  gyroY = (rawGyroZ) - gyroY_offset;
  gyroZ = rawGyroX - gyroZ_offset;

  // Simple Low-Pass Filter on Accelerometer to reduce noise
  static float filteredAccX = 0, filteredAccY = 0, filteredAccZ = 1;
  float lpfAlpha = 0.1; // Filter coefficient (lower = smoother but slower)
  filteredAccX = (lpfAlpha * accX) + ((1.0 - lpfAlpha) * filteredAccX);
  filteredAccY = (lpfAlpha * accY) + ((1.0 - lpfAlpha) * filteredAccY);
  filteredAccZ = (lpfAlpha * accZ) + ((1.0 - lpfAlpha) * filteredAccZ);

  // Calculate accelerometer angles from filtered data
  accAngleX = (atan2(filteredAccY, filteredAccZ) * (180.0 / PI)) - accAngleX_offset;
  accAngleY = (atan2(-filteredAccX, sqrt(filteredAccY * filteredAccY + filteredAccZ * filteredAccZ)) * (180.0 / PI)) - accAngleY_offset;

  // Complementary filter
  angleX = alpha * (angleX + gyroX * dt) + (1 - alpha) * accAngleX;
  angleY = alpha * (angleY + gyroY * dt) + (1 - alpha) * accAngleY;
  
  // Yaw integration
  angleZ += gyroZ * dt;
  if(angleZ > 180) angleZ -= 360;
  if(angleZ < -180) angleZ += 360;

  // Read BMP280
  altitude = bmp.readAltitude(1013.25); // standard pressure

  // Output specifically for 3D Python Viewer
  Serial.print("DATA:");
  Serial.print(angleX); Serial.print(",");
  Serial.print(angleY); Serial.print(",");
  Serial.println(angleZ);

  delay(10); // ~100Hz loop
}