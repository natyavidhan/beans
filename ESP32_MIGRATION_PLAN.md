# YMFC-AL Arduino to ESP32 Migration Plan

**Date:** May 1, 2026  
**Project:** YMFC-AL Flight Controller Quadcopter  
**Target Platform:** ESP32 (ESP32-WROOM-32)  
**Difficulty:** Medium-High  
**Estimated Time:** 1-2 days

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Hardware Differences](#hardware-differences)
3. [Pin Mapping Strategy](#pin-mapping-strategy)
4. [Code Changes Required](#code-changes-required)
5. [File-by-File Breakdown](#file-by-file-breakdown)
6. [Implementation Steps](#implementation-steps)
7. [Testing Plan](#testing-plan)
8. [Troubleshooting Guide](#troubleshooting-guide)

---

## Executive Summary

### Current Setup (Arduino Uno)
- **Microcontroller:** ATmega328P, 16MHz, 2KB RAM
- **Sensors:** MPU-6050 (I2C)
- **Motor Control:** 4x ESC via PWM (direct port manipulation)
- **Receiver:** 4-channel RC receiver via GPIO interrupts (PCINT)
- **Storage:** EEPROM 1KB
- **Communication:** Serial @ 57600 baud

### Target Setup (ESP32)
- **Microcontroller:** Dual-core Xtensa 32-bit @ 240MHz, 520KB RAM
- **Sensors:** MPU-6050 (I2C) - same
- **Motor Control:** 4x ESC via PWM (LEDC PWM controller)
- **Receiver:** 4-channel RC receiver via GPIO interrupts
- **Storage:** NVS/Preferences (4MB)
- **Communication:** Serial @ 115200 baud (default)
- **Bonus:** WiFi/BLE available

### Key Advantages
✅ 15x more RAM  
✅ 15x faster CPU  
✅ Better interrupt handling  
✅ More GPIO pins  
✅ WiFi for debugging/tuning  

### Key Challenges
⚠️ Remove all AVR register manipulation  
⚠️ Rewrite PWM output system  
⚠️ Replace EEPROM with Preferences  
⚠️ Update I2C clock setup  
⚠️ Change serial baud rates  
⚠️ Verify 3.3V logic compatibility  

---

## Hardware Differences

### Voltage Levels
| Component | Arduino Uno | ESP32 | Compatibility |
|-----------|------------|-------|----------------|
| GPIO I/O | 5V | 3.3V | ⚠️ Need verification |
| I2C (MPU-6050) | 5V tolerant | 3.3V | ✅ OK (open-drain) |
| Analog Input | 0-5V | 0-3.3V | ⚠️ Need rescaling |
| Serial TX/RX | 5V | 3.3V | ⚠️ Usually works |

### Resolution Differences
| Function | Arduino Uno | ESP32 | Impact |
|----------|------------|-------|--------|
| ADC | 10-bit (0-1023) | 12-bit (0-4095) | Battery voltage reading needs rescaling |
| PWM | 8-bit | 16-bit (configurable) | Can improve precision |
| Timer | 16-bit | 64-bit | Better timing precision |

### Speed Differences
| Operation | Arduino | ESP32 | Speedup |
|-----------|---------|-------|---------|
| CPU Clock | 16MHz | 240MHz | 15x |
| I2C Speed | 400kHz | Up to 1MHz | 2-2.5x |
| PWM frequency | Limited | Up to 80MHz | Much higher |
| Loop cycle time | ~4ms | <1ms possible | 4-5x faster |

---

## Pin Mapping Strategy

### Arduino Uno → ESP32 Mapping

#### Motor Control (ESC PWM Outputs)
**Arduino Setup (via direct port manipulation):**
```
DDRD |= B11110000;      // Pins 4, 5, 6, 7 as OUTPUT
PORTD manipulation → PWM pulses to ESC
```

**Arduino Pins:**
- D4 (Pin 4) → ESC 1
- D5 (Pin 5) → ESC 2
- D6 (Pin 6) → ESC 3
- D7 (Pin 7) → ESC 4

**ESP32 Mapping Options:**

| ESC | Arduino Pin | ESP32 GPIO | Rationale |
|-----|------------|-----------|-----------|
| ESC 1 | D4 | GPIO26 | LEDC channel 0 |
| ESC 2 | D5 | GPIO27 | LEDC channel 1 |
| ESC 3 | D6 | GPIO14 | LEDC channel 2 |
| ESC 4 | D7 | GPIO12 | LEDC channel 3 |

**ESP32 PWM Setup:**
```cpp
// Use LEDC (LED PWM Controller)
// 50Hz frequency for ESC (standard servo/ESC frequency)
// Resolution: 16-bit for fine control
ledcSetup(0, 50, 16);      // Channel 0, 50Hz, 16-bit
ledcAttachPin(GPIO26, 0);   // Attach GPIO26 to channel 0
```

#### Receiver Input (RC Signal Capture)
**Arduino Setup:**
```
Pins 8, 9, 10, 11 → Interrupt-driven pulse width capture
PCINT0-3 (Pin Change Interrupts)
```

**Arduino Pins:**
- D8 (Pin 8) → Channel 1 (Roll)
- D9 (Pin 9) → Channel 2 (Pitch)
- D10 (Pin 10) → Channel 3 (Throttle)
- D11 (Pin 11) → Channel 4 (Yaw)

**ESP32 Mapping:**

| Channel | Arduino Pin | ESP32 GPIO | Rationale |
|---------|------------|-----------|-----------|
| Ch 1 (Roll) | D8 | GPIO33 | Full GPIO with pull-up support |
| Ch 2 (Pitch) | D9 | GPIO32 | Full GPIO with pull-up support |
| Ch 3 (Throttle) | D10 | GPIO25 | Full GPIO with pull-up support |
| Ch 4 (Yaw) | D11 | GPIO39 | Input-capable with pull-up support |

**ESP32 Interrupt Setup:**
```cpp
// Much simpler than PCINT!
// Using corrected pins with pull-up support: GPIO33, GPIO32, GPIO25, GPIO39
attachInterrupt(GPIO33, isr_ch1, CHANGE);
attachInterrupt(GPIO32, isr_ch2, CHANGE);
attachInterrupt(GPIO25, isr_ch3, CHANGE);
attachInterrupt(GPIO39, isr_ch4, CHANGE);
```

#### I2C (MPU-6050)
**Arduino Setup:**
- SDA → A4 (Pin 20 internal)
- SCL → A5 (Pin 21 internal)

**ESP32 Setup (Wire library defaults):**
- SDA → GPIO21 (standard)
- SCL → GPIO22 (standard)
- **Or custom:** `Wire.begin(GPIO21, GPIO22)`

#### Battery Voltage & Current (INA219 I2C)
**Arduino:**
- A0 (Pin 14 internal) → Battery voltage divider (analog only)

**ESP32:**
- INA219 connected to I2C bus (GPIO21 SDA, GPIO22 SCL)
- **Advantages over analog:**
  - 16-bit precision (vs 10-bit ADC)
  - Measures voltage under load (accurate ESC monitoring)
  - Measures current draw (motor diagnostics)
  - I2C interface (no ADC noise)
  - Can detect motor stall (current spike)
- **I2C Address:** 0x40 (default) or 0x41 (with A0 tied high)

**Mapping:** A0 (analog) → INA219 (I2C bus)

#### LED Status Indicator
**Arduino:**
- D12 (Pin 12) → Status LED

**ESP32:**
- GPIO4 → Status LED (primary choice)
- Can support multiple status states via blink patterns
- Patterns:
  - Steady ON: Initializing/Calibrating
  - Slow blink (2Hz): Armed, ready to fly
  - Fast blink (5Hz): Flying/Motors active
  - Double blink: Battery warning
  - Rapid blink: Error/fault condition

#### Serial Communication
**Arduino:**
- RX0/TX1 (Built-in UART)
- Baud: 57600

**ESP32:**
- UART0 (GPIO1 TX, GPIO3 RX) - shared with programming
- Baud: 115200 (default, can set to 57600)

### Complete Pin Summary

```
╔════════════════╦══════════════╦═════════╦════════════════════════════════════╗
║ Function       ║ Arduino Pin  ║ ESP32   ║ Notes                              ║
╠════════════════╬══════════════╬═════════╬════════════════════════════════════╣
║ ESC 1 PWM      ║ D4           ║ GPIO26  ║ LEDC Channel 0                     ║
║ ESC 2 PWM      ║ D5           ║ GPIO27  ║ LEDC Channel 1                     ║
║ ESC 3 PWM      ║ D6           ║ GPIO14  ║ LEDC Channel 2                     ║
║ ESC 4 PWM      ║ D7           ║ GPIO12  ║ LEDC Channel 3                     ║
║ RX Ch 1        ║ D8           ║ GPIO33  ║ Full GPIO, pullup support          ║
║ RX Ch 2        ║ D9           ║ GPIO32  ║ Full GPIO, pullup support          ║
║ RX Ch 3        ║ D10          ║ GPIO25  ║ Full GPIO, pullup support          ║
║ RX Ch 4        ║ D11          ║ GPIO39  ║ Input-capable, pullup support      ║
║ Status LED     ║ D12          ║ GPIO4   ║ Status LED (GPIO2 also available)  ║
║ I2C SDA        ║ A4           ║ GPIO21  ║ Wire library default               ║
║ I2C SCL        ║ A5           ║ GPIO22  ║ Wire library default               ║
║ Serial TX      ║ TX1          ║ GPIO1   ║ UART0 (programming port)           ║
║ Serial RX      ║ RX0          ║ GPIO3   ║ UART0 (programming port)           ║
╚════════════════╩══════════════╩═════════╩════════════════════════════════════╝
```

---

## Code Changes Required

### 1. Replace Header Includes & Setup

#### BEFORE (Arduino)
```cpp
#include <Wire.h>
#include <EEPROM.h>
```

#### AFTER (ESP32)
```cpp
#include <Wire.h>              // Same, but different implementation
#include <Preferences.h>       // Replace EEPROM with Preferences (NVS)

// Define GPIO pins for ESP32
#define ESC_1_PIN 26
#define ESC_2_PIN 27
#define ESC_3_PIN 14
#define ESC_4_PIN 12
#define RX_CH1_PIN 33  // Ch 1 (Roll) - Full GPIO, pullup capable
#define RX_CH2_PIN 32  // Ch 2 (Pitch) - Full GPIO, pullup capable
#define RX_CH3_PIN 25  // Ch 3 (Throttle) - Full GPIO, pullup capable
#define RX_CH4_PIN 39  // Ch 4 (Yaw) - Input-capable with pullup
#define STATUS_LED_PIN 4

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define INA219_I2C_ADDRESS 0x40  // Default address

// LEDC PWM channels
#define ESC_1_CH 0
#define ESC_2_CH 1
#define ESC_3_CH 2
#define ESC_4_CH 3
#define PWM_FREQ 50           // 50Hz for ESC (servo standard)
#define PWM_RESOLUTION 16      // 16-bit resolution
```

---

### 2. Replace Pin Mode Setup

#### BEFORE (Arduino - uses AVR registers)
```cpp
void setup() {
  // Configure digital ports 4, 5, 6, 7 as output
  DDRD |= B11110000;           // Direct register manipulation
  
  // Configure digital ports 12, 13 as output
  DDRB |= B00110000;           // Direct register manipulation
  
  // Arduino pins default to inputs
  // No explicit setup needed for digital inputs
  
  // Set I2C clock speed using AVR register
  TWBR = 12;                   // Sets I2C to 400kHz
}
```

#### AFTER (ESP32 - standard Arduino functions)
```cpp
void setup() {
  // *** SERIAL MUST INITIALIZE FIRST for debug output during setup ***
  Serial.begin(57600);
  delay(100);
  Serial.println("\n=== ESP32 Flight Controller Booting ===");
  
  // Configure ESC PWM pins
  ledcSetup(ESC_1_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ESC_1_PIN, ESC_1_CH);
  
  ledcSetup(ESC_2_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ESC_2_PIN, ESC_2_CH);
  
  ledcSetup(ESC_3_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ESC_3_PIN, ESC_3_CH);
  
  ledcSetup(ESC_4_CH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ESC_4_PIN, ESC_4_CH);
  
  // Configure receiver input pins with pull-ups for stable PWM capture
  pinMode(RX_CH1_PIN, INPUT_PULLUP);
  pinMode(RX_CH2_PIN, INPUT_PULLUP);
  pinMode(RX_CH3_PIN, INPUT_PULLUP);
  pinMode(RX_CH4_PIN, INPUT_PULLUP);
  
  // Configure Status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);  // Turn on for initialization
  
  // Configure I2C (for MPU-6050, BMP280, and INA219)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);        // 400kHz clock speed
  
  // Initialize INA219
  if (!ina219.begin(INA219_I2C_ADDRESS)) {
    Serial.println("ERROR: INA219 not found!");
    set_led_status(LED_STATUS_ERROR);
  } else {
    ina219.setCalibration_16V_400mA();  // Or appropriate for your variant
    Serial.println("INA219 initialized");
  }
}
```

---

### 3. Replace Direct Port Manipulation

**This is scattered throughout ALL files. Examples:**

#### Pattern 1: Setting pins HIGH
##### BEFORE
```cpp
PORTD |= B11110000;          // Set pins 4-7 HIGH (simultaneous)
```

##### AFTER
```cpp
// Cannot set multiple pins simultaneously like AVR
// Option A: Individual writes (slightly slower, fine for ESC timing)
digitalWrite(ESC_1_PIN, HIGH);
digitalWrite(ESC_2_PIN, HIGH);
digitalWrite(ESC_3_PIN, HIGH);
digitalWrite(ESC_4_PIN, HIGH);

// Option B: Use LEDC PWM directly (RECOMMENDED for ESC control)
// See PWM Output section below
```

#### Pattern 2: Setting pins LOW
##### BEFORE
```cpp
PORTD &= B00001111;          // Set pins 4-7 LOW (simultaneous)
```

##### AFTER
```cpp
digitalWrite(ESC_1_PIN, LOW);
digitalWrite(ESC_2_PIN, LOW);
digitalWrite(ESC_3_PIN, LOW);
digitalWrite(ESC_4_PIN, LOW);
```

#### Pattern 3: Setting LED
##### BEFORE
```cpp
digitalWrite(12, HIGH);       // Turn on LED (pin 12)
digitalWrite(12, !digitalRead(12));  // Toggle LED
```

##### AFTER
```cpp
digitalWrite(LED_PIN, HIGH);  // Turn on LED
digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Toggle
```

---

### 4. Replace PWM Output System (CRITICAL)

**Current approach is unreliable for precise ESC timing.**

#### BEFORE (Arduino - Manual pulse generation)
```cpp
// Manual pulse generation in esc_pulse_output() function
PORTD |= B11110000;                    // Set pins 4-7 HIGH
delayMicroseconds(esc_1);              // Hold for esc_1 microseconds (1000-2000us)
// (repeat for esc_2, esc_3, esc_4)
PORTD &= B00001111;                    // Set pins 4-7 LOW

// Problems:
// - Single-threaded, can miss receiver interrupts
// - Timing precision depends on loop timing
// - No hardware PWM generation
```

#### AFTER (ESP32 - Hardware PWM via LEDC)
```cpp
// In setup():
ledcSetup(ESC_1_CH, 50, 16);        // 50Hz, 16-bit (65535 max)
ledcAttachPin(ESC_1_PIN, ESC_1_CH);
// ... repeat for other channels

// In main loop or wherever ESC values are updated:
// Convert microseconds (1000-2000) to duty cycle (0-65535)
// Formula: duty = (microseconds * 65535) / 20000
// Where 20000us = 20ms = 1/50Hz
// Using integer math to avoid floating-point rounding errors

// Example:
int esc_1_us = 1500;  // 1.5ms pulse (center stick)
int esc_1_duty = (esc_1_us * 65535) / 20000;  // Integer division for precision
ledcWrite(ESC_1_CH, esc_1_duty);

// Benefits:
// - Hardware PWM, doesn't block
// - Precise timing independent of code
// - Can run multiple PWM channels simultaneously
// - Frees CPU for other tasks
```

**New helper function:**
```cpp
void setEscPulse(int esc_num, int microseconds) {
  // Clamp value to 1000-2000us
  if (microseconds < 1000) microseconds = 1000;
  if (microseconds > 2000) microseconds = 2000;
  
  // Convert to 16-bit duty cycle (0-65535) using integer math
  // At 50Hz: period = 20ms = 20000us
  int duty = (microseconds * 65535) / 20000;  // Integer math for precision
  
  switch(esc_num) {
    case 1: ledcWrite(ESC_1_CH, duty); break;
    case 2: ledcWrite(ESC_2_CH, duty); break;
    case 3: ledcWrite(ESC_3_CH, duty); break;
    case 4: ledcWrite(ESC_4_CH, duty); break;
  }
}
```

**Update all ESC output code:**
- Replace direct PORTD manipulation
- Replace delayMicroseconds() pulse timing
- Use new `setEscPulse()` helper function

---

### 5. Replace Pin Change Interrupts (PCINT)

**Arduino uses PCINT0 for pins 8-11. ESP32 has native GPIO interrupts.**

#### BEFORE (Arduino)
```cpp
// Enable PCINT0 for pins 8-11
PCICR |= (1 << PCIE0);
PCMSK0 |= (1 << PCINT0);  // Pin 8
PCMSK0 |= (1 << PCINT1);  // Pin 9
PCMSK0 |= (1 << PCINT2);  // Pin 10
PCMSK0 |= (1 << PCINT3);  // Pin 11

// ISR for pin change
ISR(PCINT0_vect) {
  // Interrupt handler code
}
```

#### AFTER (ESP32 - Much simpler!)
```cpp
// Attach interrupts
attachInterrupt(RX_CH1_PIN, isr_ch1, CHANGE);
attachInterrupt(RX_CH2_PIN, isr_ch2, CHANGE);
attachInterrupt(RX_CH3_PIN, isr_ch3, CHANGE);
attachInterrupt(RX_CH4_PIN, isr_ch4, CHANGE);

// ISR for each channel (must be in IRAM on ESP32)
void IRAM_ATTR isr_ch1() {
  // Read GPIO and timestamp
  // Update receiver_input_channel_1
}

void IRAM_ATTR isr_ch2() {
  // Similar...
}

void IRAM_ATTR isr_ch3() {
  // Similar...
}

void IRAM_ATTR isr_ch4() {
  // Similar...
}
```

---

### 6. Replace EEPROM with Preferences

**Arduino uses EEPROM directly. ESP32 uses NVS (Non-Volatile Storage) via Preferences library.**

#### BEFORE (Arduino)
```cpp
// Write to EEPROM
EEPROM.write(0, 'J');
EEPROM.write(1, 'M');
EEPROM.write(2, 'B');

// Read from EEPROM
byte data = EEPROM.read(0);
```

#### AFTER (ESP32)
```cpp
#include <Preferences.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BMP280.h>

Preferences prefs;
Adafruit_INA219 ina219;      // Battery voltage/current monitoring (I2C address 0x40)
Adafruit_BMP280 bmp280;      // Pressure/altitude sensor (I2C address 0x76 or 0x77) - Phase 2

// *** SERIAL MUST INITIALIZE FIRST for debug output during setup ***
Serial.begin(57600);
delay(100);  // Wait for serial port to stabilize
Serial.println("\n=== ESP32 Flight Controller Booting ===");

// In setup():
prefs.begin("ymfc", false);  // namespace "ymfc", read/write mode

// Initialize I2C bus SECOND (after Serial)
Wire.begin(21, 22, 400000);

// Initialize INA219
if (!ina219.begin(0x40)) {
  Serial.println("ERROR: INA219 not found on I2C bus!");
}
ina219.setCalibration_16V_400mA();  // Adjust based on your variant
Serial.println("INA219 initialized OK");

// Initialize BMP280 (Phase 2 - Altitude Hold)
if (bmp280.begin(0x76)) {  // Try 0x77 if 0x76 doesn't work
  Serial.println("BMP280 initialized (Phase 2)");
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);
} else {
  Serial.println("WARNING: BMP280 not found (Phase 2 optional)");
}

// Write to NVS
prefs.putChar("sig0", 'J');
prefs.putChar("sig1", 'M');
prefs.putChar("sig2", 'B');
prefs.putUChar("gyro_addr", gyro_address);

// Read from NVS
char sig0 = prefs.getChar("sig0", 0);
byte gyro_addr = prefs.getUChar("gyro_addr", 0);

// In the end:
prefs.end();  // Optional cleanup

// Benefits over Arduino EEPROM:
// - 4MB instead of 1KB
// - Key-value pairs instead of raw addresses
// - Better wear leveling
// - Atomic writes
```

**Refactor current EEPROM arrays:**
```cpp
// OLD Arduino way (raw byte array from EEPROM):
byte eeprom_data[36];
for(start = 0; start <= 35; start++)
  eeprom_data[start] = EEPROM.read(start);

// NEW ESP32 way (key-value):
prefs.begin("ymfc", false);

// Store calibration data
struct {
  int gyro_cal[3];
  byte gyro_address;
  byte sig[3];  // 'J', 'M', 'B'
} cal_data;

// Write:
prefs.putBytes("gyro_cal", (uint8_t*)&cal_data.gyro_cal, 12);
prefs.putUChar("gyro_addr", cal_data.gyro_address);
prefs.putChar("sig0", 'J');
prefs.putChar("sig1", 'M');
prefs.putChar("sig2", 'B');

// Read:
prefs.getBytes("gyro_cal", (uint8_t*)&cal_data.gyro_cal, 12);
```

---

### 7. Fix Serial Communication

#### BEFORE (Arduino)
```cpp
Serial.begin(57600);
```

#### AFTER (ESP32)
```cpp
Serial.begin(57600);
// ESP32 defaults to 115200, explicitly set to 57600 for compatibility
```

**Optional improvement:**
```cpp
// Use 115200 if you can recalibrate:
Serial.begin(115200);
// Faster and standard for ESP32
```

---

### 8. Replace Analog Battery Input with INA219 (I2C)

**Arduino used analogRead(A0) with voltage divider. ESP32 uses INA219 over I2C.**

#### BEFORE (Arduino - Analog Voltage Divider)
```cpp
// 12.6V battery → voltage divider → ~5V @ A0 = 1023 value
// Problems: noisy, limited precision, current unknown
battery_voltage = (analogRead(0) + 65) * 1.2317;  // 10-bit ADC
```

#### AFTER (ESP32 - INA219 over I2C)
```cpp
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

// In setup():
ina219.begin(INA219_I2C_ADDRESS);
ina219.setCalibration_16V_400mA();  // Or your variant

// In loop() - replace analogRead() entirely:
float battery_voltage_mv = ina219.getBusVoltage_V() * 1000;  // in millivolts
float battery_current_ma = ina219.getCurrent_mA();           // in milliamps
float battery_power_mw = ina219.getPower_mW();               // in milliwatts

// Use in PID compensation (same as before, but more accurate):
if (battery_voltage_mv < 12400 && battery_voltage_mv > 8000) {
  // Apply voltage compensation
  esc_compensation_factor = 1240 / (battery_voltage_mv / 10.0);
}

// Bonus: Detect motor stall
if (battery_current_ma > MAX_CURRENT_THRESHOLD) {
  Serial.println("WARNING: Motor stall detected!");
}
```

**Benefits:**
- 16-bit precision (vs 10-bit ADC)
- I2C communication (vs noisy analog)
- Actual voltage under load (not voltage divider estimate)
- Current measurement for diagnostics
- No ADC pin tied up

---

### 9. Fix I2C Clock Speed

#### BEFORE (Arduino - AVR register)
```cpp
TWBR = 12;  // Sets I2C clock to 400kHz
```

#### AFTER (ESP32 - Wire library)
```cpp
Wire.setClock(400000);  // 400kHz explicitly
// Or in begin():
Wire.begin(21, 22, 400000);  // SDA, SCL, frequency
```

---

### 10. Add LED Status Control Function

**New feature: Status LED with multiple states**

```cpp
// LED Status constants
#define LED_STATUS_INIT 0          // Steady ON during init
#define LED_STATUS_ARMED 1         // Slow blink 2Hz
#define LED_STATUS_FLYING 2        // Fast blink 5Hz  
#define LED_STATUS_BATTERY_LOW 3   // Double blink
#define LED_STATUS_ERROR 4         // Rapid blink

volatile int led_status = LED_STATUS_INIT;
unsigned long last_led_toggle = 0;
int led_blink_speed = 0;  // milliseconds between toggles

void set_led_status(int status) {
  static int blink_counter = 0;
  static int double_blink_phase = 0;  // For double-blink state machine
  
  switch(status) {
    case LED_STATUS_INIT:
      digitalWrite(STATUS_LED_PIN, HIGH);  // Steady ON
      return;  // Don't run blink logic
      
    case LED_STATUS_ARMED:
      led_blink_speed = 250;  // 2Hz blink (on 250ms, off 250ms)
      break;
      
    case LED_STATUS_FLYING:
      led_blink_speed = 100;  // 5Hz blink (on 100ms, off 100ms)
      break;
      
    case LED_STATUS_BATTERY_LOW:
      // True double blink pattern: ON 100ms → OFF 50ms → ON 100ms → OFF 400ms → repeat
      if (double_blink_phase == 0) {
        if (blink_counter++ < 100) digitalWrite(STATUS_LED_PIN, HIGH);
        else if (blink_counter < 150) digitalWrite(STATUS_LED_PIN, LOW);
        else { double_blink_phase = 1; blink_counter = 0; }
      } else {
        if (blink_counter++ < 100) digitalWrite(STATUS_LED_PIN, HIGH);
        else if (blink_counter < 500) digitalWrite(STATUS_LED_PIN, LOW);
        else { double_blink_phase = 0; blink_counter = 0; }
      }
      return;  // Skip regular blink logic
      
    case LED_STATUS_ERROR:
      led_blink_speed = 50;   // Rapid blink (~10Hz)
      break;
  }
  
  // Regular blink handler for other statuses
  if (++blink_counter >= led_blink_speed / 2) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    blink_counter = 0;
  }
}

// In main loop:
if (led_blink_speed > 0 && (millis() - last_led_toggle) > led_blink_speed) {
  digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  last_led_toggle = millis();
}
```

**Usage:**
```cpp
if (start == 0) {
  set_led_status(LED_STATUS_INIT);
} else if (start == 1) {
  set_led_status(LED_STATUS_ARMED);
} else if (start == 2) {
  set_led_status(LED_STATUS_FLYING);
}

if (battery_voltage < BATTERY_MINIMUM) {
  set_led_status(LED_STATUS_BATTERY_LOW);
}
```

---

### 11. Replace Serial Input Flushing

#### BEFORE (Arduino)
```cpp
while(Serial.available() > 0)
  loop_counter = Serial.read();  // Flush buffer
```

#### AFTER (ESP32 - same works)
```cpp
while(Serial.available() > 0)
  Serial.read();  // Flush buffer (cleaner)
```

---

## File-by-File Breakdown

### File 1: YMFC-AL_setup.ino

**Purpose:** Setup and calibration routine

**Changes Required:**

1. **Line ~20-30:** Replace includes
   - Add `#define` pins section
   - Add `#include <Preferences.h>`

2. **Line ~50-70:** Replace PCINT setup
   - Remove all `PCICR`, `PCMSK0` lines
   - Add `attachInterrupt()` calls

3. **Line ~80-100:** Replace TWBR
   - Replace `TWBR = 12;` with `Wire.setClock(400000);`

4. **Line ~150-200:** Replace DDRD register manipulation
   - Replace `DDRD |= B11110000;` and `DDRB |= B00110000;`
   - Use `pinMode()` or LEDC setup

5. **Throughout:** Replace `PORTD` manipulation
   - Search for `PORTD |=` and `PORTD &=`
   - Replace with `digitalWrite()` or `ledcWrite()`

6. **Serial setup:**
   - Keep `Serial.begin(57600);` or change to 115200

7. **EEPROM sections:**
   - Replace `EEPROM.read()` / `EEPROM.write()`
   - Use Preferences API

**Search patterns to find all issues:**
```
PCICR
PCMSK0
TWBR
DDRD
DDRB
PORTD
EEPROM
digitalWrite(12
digitalWrite(13
```

---

### File 2: YMFC-AL_Flight_controller.ino

**Purpose:** Main flight controller logic

**Changes Required:**

1. **Line ~20-40:** Add pin definitions and Preferences

2. **Line ~60-120:** Replace all register manipulation in setup()
   - DDRD, DDRB → pinMode() / LEDC setup
   - TWBR → Wire.setClock()
   - PCINT → attachInterrupt()

3. **Line ~150-200:** Replace PORTD LED manipulation
   - `digitalWrite(12, HIGH)` → `digitalWrite(LED_PIN, HIGH)`
   - `digitalWrite(12, !digitalRead(12))` → same with LED_PIN

4. **Line ~300-400:** Look for ESC pulse output function
   - Replace entire `esc_pulse_output()` function
   - Use `ledcWrite()` instead of manual timing

5. **Line ~400-500:** Battery voltage reading
   - Replace `analogRead(0)` with `ina219.getBusVoltage_V()`
   - Ensure INA219 is initialized in setup() via `ina219.begin(0x40)`
   - Multiply by 1000 if code expects millivolts instead of volts

6. **Throughout:** All PORTD operations
   - Search and replace with digitalWrite/ledcWrite

**Critical functions to rewrite:**
- `esc_pulse_output()` → Use LEDC PWM
- Receiver interrupt handlers → Use attachInterrupt() format
- Setup section → Replace register manipulation

---

### File 3: YMFC-AL_esc_calibrate.ino

**Purpose:** ESC calibration routine

**Changes Required:**

1. **Line ~20-40:** Add pin definitions, Preferences, includes

2. **Line ~50-100:** Setup function
   - Replace DDRD, DDRB
   - Replace TWBR
   - Replace PCINT setup

3. **Line ~200+:** ESC pulse output section
   - Find all `esc_pulse_output()` calls
   - Rewrite to use `ledcWrite()` / LEDC PWM

4. **Throughout:** All PORTD manipulation
   - Replace with digitalWrite/ledcWrite

5. **Serial communication:**
   - Verify baud rate (57600 vs 115200)

6. **EEPROM sections:**
   - Replace with Preferences

---

## Implementation Steps

### Phase 1: Preparation (30 min)

- [ ] Create new ESP32 project folder
- [ ] Copy all three .ino files
- [ ] Create backup of original Arduino versions
- [ ] Document all functions that use PORTD, PCINT, TWBR, EEPROM

### Phase 2: Core Replacements (2-3 hours)

- [ ] Add #define pins section to all files
- [ ] Add #include <Preferences.h>
- [ ] Replace all DDRD/DDRB with pinMode()/LEDC setup
- [ ] Replace all PORTD operations with digitalWrite()/ledcWrite()
- [ ] Replace TWBR with Wire.setClock()
- [ ] Replace PCINT with attachInterrupt()
- [ ] Replace EEPROM with Preferences

### Phase 3: PWM System Rewrite (1-2 hours)

- [ ] Create setEscPulse() helper function
- [ ] Rewrite esc_pulse_output() function
- [ ] Update all ESC calculation code to use new PWM system
- [ ] Test PWM frequency (should be stable 50Hz)

### Phase 4: Receiver Input Rewrite (1 hour)

- [ ] Create individual ISR functions for each channel
- [ ] Verify interrupt timing still works
- [ ] Test receiver signal capture

### Phase 5: Testing & Debugging (2-4 hours)

- [ ] Compile and fix errors
- [ ] Serial debugging output
- [ ] Verify pin assignments
- [ ] Test each component individually
- [ ] Full system test on bench (propellers OFF!)

---

## Testing Plan

### Libraries Required

Before compilation, install these libraries in Arduino IDE:
- `Adafruit_INA219` (for battery monitoring)
- `MPU6050` (if using Adafruit version)
- Already included: Wire, EEPROM (for Preferences)

### Level 1: Compilation

```
Goal: Code compiles without errors or warnings
Steps:
1. Install Adafruit INA219 library via Arduino IDE Library Manager
2. Compile all three .ino files together
3. Check for missing includes
4. Verify all pin definitions are valid (GPIO 4, 21, 22, 25, 26, 27, 32, 33, 39)
5. Ensure no syntax errors
```

### Level 2: Individual Component Testing

#### 2A: Serial Communication
```
Goal: Serial monitor shows debug output
Test:
1. Upload code
2. Open Serial Monitor @ 57600 baud
3. Verify startup messages appear
4. Test Serial.println() in setup()
Expected: Messages appear in serial monitor
```

#### 2B: LED
```
Goal: LED blinks as expected
Test:
1. Observe LED on startup
2. LED should turn on during setup
3. LED should blink during calibration
Expected: LED visual feedback works
```

#### 2C: I2C / MPU-6050
```
Goal: I2C communication established
Test:
1. Add debug code to detect MPU-6050
2. Serial print I2C scan results
3. Verify gyro address matches
Expected: MPU-6050 found at address 0x68 or 0x69
```

#### 2D: PWM Outputs (ESC)
```
Goal: PWM signals output on correct pins
Test:
1. Use oscilloscope or logic analyzer on GPIO26, 27, 14, 12
2. Set ESC values manually in code
3. Measure pulse width (should be 1000-2000us @ 50Hz)
Expected: 
- 50Hz frequency
- 1000-2000us pulse width
- All 4 channels active
```

#### 2E: Receiver Input
```
Goal: Receiver signals captured correctly
Test:
1. Connect RC receiver to GPIO33, 32, 25, 39
2. Move RC sticks
3. Serial print receiver values
Expected:
- Values range from ~1000-2000us
- Smooth response to stick movement
- All 4 channels read correctly
- No unstable readings or glitches (pull-ups active)
```

#### 2F: Battery Voltage & Current (INA219)
```
Goal: INA219 reads battery voltage and current correctly
Test:
1. Ensure INA219 is on I2C bus (GPIO21 SDA, GPIO22 SCL)
2. Serial print ina219.getBusVoltage_V() and ina219.getCurrent_mA()
3. Compare voltage with multimeter
4. Monitor current during motor spin-up
Expected:
- Voltage matches multimeter within 0.05V
- Current readings reasonable (0-200mA idle, 100mA+ under load)
- Smooth readings (no digital noise)
- I2C address found (0x40 or 0x41)
```

#### 2G: LED Status Indicator
```
Goal: LED shows flight controller status
Test:
1. Observe LED behavior on startup (should be steady ON)
2. Arm system (should blink slowly 2Hz)
3. Increase throttle (should blink fast 5Hz)
4. Simulate low battery (should blink double)
5. Trigger error condition (should blink rapidly)
Expected:
- LED responds to status changes
- Blink patterns clear and distinguishable
- LED never stays off during normal operation
```

### Level 3: System Integration

#### 3A: Calibration Sequence
```
Goal: Full calibration routine runs
Test:
1. Run YMFC-AL_setup.ino
2. Follow serial prompts
3. Complete gyro calibration
4. Complete receiver calibration
Expected:
- All steps complete successfully
- Data saved to NVS
- No crashes
```

#### 3B: Flight Controller Ready
```
Goal: Flight controller enters ready state
Test:
1. Run YMFC-AL_Flight_controller.ino
2. Power on system
3. Verify PID loops running
4. Monitor serial debug output
Expected:
- No errors
- Gyro readings stable
- Accelerometer readings reasonable
```

### Level 4: Bench Testing (Propellers OFF!)

```
Goal: Basic flight control response without flying
Test:
1. Power up flight controller
2. Motor idle pulse (1000us) should go to all ESCs
3. Manually increase throttle in code
4. ESC pulses should increase to all motors
5. Verify PID responses to simulated motion

Safety:
- PROPELLERS REMOVED
- Safe work environment
- Kill switch accessible
```

### Level 5: Flight Testing (When Ready)

```
After all above tests pass and components verified:
1. Mount on frame with props
2. Start indoors in confined space
3. Gradual thrust increase
4. Stability verification
5. Control input response

This is after migration is fully complete and tested!
```

---

## Troubleshooting Guide

### Compilation Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `'DDRD' was not declared` | AVR register | Replace with pinMode() |
| `'TWBR' was not declared` | AVR register | Use Wire.setClock() |
| `'EEPROM' undeclared` | Missing include | Add `#include <EEPROM.h>` (works on ESP32) |
| `'Preferences' undeclared` | Missing include | Add `#include <Preferences.h>` |
| `Unknown pin XX` | Wrong GPIO number | Check pin mappings table |
| `'ledcWrite' not declared` | ESP-IDF version | Ensure esp32 board selected |

### Runtime Issues

| Issue | Likely Cause | Debug Steps |
|-------|-------------|-------------|
| INA219 not detected | I2C address wrong or no power | Add I2C scan code, verify 0x40/0x41 |
| Battery voltage reads 0 | INA219 not initialized | Call ina219.begin() in setup |
| Current reading always 0 | Shunt resistance issue | Check INA219 variant (0.1Ω vs 0.01Ω) |
| LED doesn't blink | GPIO4 unavailable or wrong | Try GPIO2, verify pin not in use |
| LED patterns wrong | Timing/millis() issue | Check millis() precision |
| No serial output | Wrong baud rate | Try 115200 or 57600 |
| No serial output | Serial on wrong UART | Check TX/RX pins (GPIO1/GPIO3) |
| LED doesn't blink | Wrong pin | Verify GPIO2 available |
| MPU-6050 not found | I2C error | Add I2C scan debug code |
| PWM not working | Pin not attached | Verify ledcAttachPin() called |
| PWM on wrong pin | LEDC channel mismatch | Check channel-to-pin mapping |
| Receiver not reading | GPIO not configured | Verify attachInterrupt() calls |
| Receiver values wrong | Interrupt timing issue | Add debug timestamps |
| Battery voltage reads 0V | INA219 not initialized | Verify `ina219.begin(0x40)` called in setup() |
| Battery voltage unchanged | I2C communication error | Check SDA/SCL pins (GPIO21/22), verify 4.7kΩ pull-ups, check device address |
| Flight unstable | Loop timing changed | ESP32 faster, may need PID retuning |

### Performance Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| High interrupt latency | CPU saturated | Reduce loop load |
| PWM jittery | Manual timing used | Ensure using LEDC PWM |
| Gyro data noisy | I2C speed too high | Lower I2C clock to 400kHz |
| Dropped receiver signals | Interrupt buffer full | Reduce other interrupts |

---

## Verification Checklist

Before considering migration complete:

### Code Quality
- [ ] No AVR register manipulation remaining
- [ ] All EEPROM replaced with Preferences
- [ ] All interrupts use attachInterrupt()
- [ ] All PWM uses LEDC
- [ ] All analog battery reading removed (INA219 only)
- [ ] LED status function implemented
- [ ] Pin definitions centralized in #defines
- [ ] INA219 library included and initialized
- [ ] Code compiles without warnings
- [ ] Code commented for clarity

### Functionality
- [ ] Serial communication works
- [ ] LED shows correct status patterns
- [ ] I2C communication established (all 3 devices)
- [ ] MPU-6050 detected
- [ ] INA219 detected and initialized
- [ ] PWM outputs on all 4 channels
- [ ] Receiver inputs reading all 4 channels
- [ ] Battery voltage reading from INA219 (no analog ADC)
- [ ] Battery current monitored
- [ ] Calibration routine completes
- [ ] Flight controller initializes with LED feedback

### Testing
- [ ] All individual components tested
- [ ] System integration verified
- [ ] Bench testing passed (no props)
- [ ] All PID values reasonable
- [ ] Gyro calibration working
- [ ] ESC values responding correctly
- [ ] No crashes or reboots
- [ ] Serial output shows expected debug info

### Documentation
- [ ] All functions documented
- [ ] New helper functions explained
- [ ] Pin assignments documented
- [ ] Calibration procedure documented
- [ ] Known issues documented

---

## Additional Notes

### Performance Improvements (Bonus)

Since ESP32 is so much faster, consider:

1. **Increase PID loop frequency**
   - Current: ~250Hz (4ms loop)
   - Possible: ~1000Hz (1ms loop) with ESP32
   - Benefit: Better stability

2. **Use FreeRTOS**
   - Run IMU reading on separate core
   - Run PID on main core
   - Better parallelization

3. **WiFi Tuning** (with INA219)
   - Send PID telemetry + power data over WiFi
   - Adjust gains in real-time
   - Monitor flight remotely with power consumption graph
   - Early warning for battery/stall issues

4. **Battery Monitoring (INA219 Enhanced)**
   - More frequent current sampling
   - Detect motor anomalies (stall, shorts)
   - Power consumption profiling
   - Predictive low-battery shutdown
   - Thrust estimation from current draw

5. **LED Telemetry**
   - LED blink patterns communicate full system state
   - No serial connection needed for basic status
   - Useful for field debugging

### Potential Issues

1. **GPIO Pin Limitations**
   - ⚠️ **AVOID GPIO34/35:** Input-only, no pull-up support, unsuitable for RC PWM
   - ✅ **Use GPIO33, 32, 25, 39:** Full GPIO or input+pullup capable
   - Unstable readings if using input-only pins for PWM capture
   - Solution: Use corrected pin mapping provided in migration plan

2. **3.3V vs 5V Logic**
   - MPU-6050: OK on 3.3V
   - Receiver: Usually OK, verify with your model
   - ESC: Usually OK, most are 3.3V tolerant
   - INA219: I2C (open-drain), fully compatible

2. **INA219 Shunt Power Dissipation**
   - Check variant: 0.1Ω vs 0.01Ω shunt
   - 0.1Ω @ 30A: ~90W (too hot, upgrade)
   - 0.01Ω @ 30A: ~9W (acceptable)
   - Monitor temperature if using high-current variant

3. **I2C Bus Conflicts**
   - 3 devices on one bus (MPU-6050, BMP280, INA219)
   - Bandwidth: ~2% of 400kHz capacity
   - Potential: Address conflict if multiple devices at 0x40
   - Solution: Use I2C address scanner during setup

4. **Pull-Up Requirements**
   - RC receiver PWM needs stable high-level detection
   - Pull-ups on GPIO33, 32, 25, 39 prevent floating inputs
   - Internal pull-ups enabled via INPUT_PULLUP
   - Greatly improves signal reliability

5. **Brown-Out Detection**
   - ESP32 has built-in BOD
   - May trigger on power drops
   - Can be configured in code
   - INA219 can warn before BOD triggers

6. **Power Consumption**
   - ESP32 slightly higher than Arduino
   - With WiFi: much higher
   - For drone use: disable WiFi after tuning
   - INA219 adds <1mA overhead

7. **Heat Dissipation**
   - ESP32 generates more heat
   - INA219 shunt generates heat
   - Ensure adequate airflow
   - Monitor temperature in code if needed

---

## Migration Completion Criteria

The migration is complete when:

✅ All 3 .ino files compile without errors  
✅ All AVR register code replaced  
✅ All analog battery code removed (INA219 only)  
✅ LED status function working with patterns  
✅ INA219 initialized and reading voltage/current  
✅ All hardware features working on bench  
✅ Calibration routine successful  
✅ Flight controller initializes properly  
✅ LED shows correct status during startup  
✅ PID controller responding to inputs  
✅ System stable for 5+ minutes on bench  
✅ Serial debugging shows battery/current values  
✅ Ready for initial bench tests (no propellers)  

---

## References

- [ESP32 Pinout & GPIO](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/pins_arduino.h)
- [ESP32 LEDC PWM](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/hw_timer.h)
- [ESP32 Interrupts](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/Arduino.h#L79)
- [Preferences Library](https://github.com/espressif/arduino-esp32/tree/master/libraries/Preferences)
- [MPU-6050 I2C Protocol](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

---

**Created:** May 1, 2026  
**Status:** Ready for Implementation  
**Next Step:** Begin Phase 1 Preparation
