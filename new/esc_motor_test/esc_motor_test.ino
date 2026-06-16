// ═══════════════════════════════════════════════════════════════
//  ESC Calibration & Motor Test — Arduino Nano
//
//  Motor pins: M1=D9  M2=D10  M3=D11  M4=D12
//  ESC protocol: 1000-2000µs (standard PWM)
//
//  Serial: 115200 baud, Newline
//
//  Menu commands:
//    1 = ESC calibration (one at a time)
//    2 = Spin individual motor at chosen throttle %
//    3 = Throttle sweep (single motor)
//    4 = Spin ALL motors at ~25% (direction check)
//    5 = All motors throttle sweep
//    0 = STOP ALL MOTORS
// ═══════════════════════════════════════════════════════════════

#include <Servo.h>
#include "config.h"

Servo esc1, esc2, esc3, esc4;

#define ESC_MIN 1000
#define ESC_MAX 2000

void allStop() {
  esc1.writeMicroseconds(ESC_MIN);
  esc2.writeMicroseconds(ESC_MIN);
  esc3.writeMicroseconds(ESC_MIN);
  esc4.writeMicroseconds(ESC_MIN);
}

void armAllESCs() {
  allStop();
  Serial.println(F("Arming ESCs (sending 1000us for 3s)..."));
  delay(3000);
  Serial.println(F("Armed."));
}

void spinOne(uint8_t motor, uint16_t us) {
  us = constrain(us, ESC_MIN, ESC_MAX);
  switch (motor) {
    case 1: esc1.writeMicroseconds(us); break;
    case 2: esc2.writeMicroseconds(us); break;
    case 3: esc3.writeMicroseconds(us); break;
    case 4: esc4.writeMicroseconds(us); break;
  }
}

void spinAll(uint16_t us) {
  us = constrain(us, ESC_MIN, ESC_MAX);
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

void throttleSweep(uint8_t motor) {
  armAllESCs();
  Serial.print(F("Sweeping motor M")); Serial.println(motor);
  Serial.println(F("  Ramping up 0% -> 100%..."));
  for (int i = ESC_MIN; i <= ESC_MAX; i += 10) {
    spinOne(motor, i);
    delay(30);
  }
  Serial.println(F("  At 100% — 2 second hold"));
  spinOne(motor, ESC_MAX);
  delay(2000);
  Serial.println(F("  Ramping down 100% -> 0%..."));
  for (int i = ESC_MAX; i >= ESC_MIN; i -= 10) {
    spinOne(motor, i);
    delay(30);
  }
  allStop();
  Serial.println(F("  Done."));
}

void throttleSweepAll() {
  armAllESCs();
  Serial.println(F("Sweeping ALL motors"));
  Serial.println(F("  Ramping up 0% -> 100%..."));
  for (int i = ESC_MIN; i <= ESC_MAX; i += 10) {
    spinAll(i);
    delay(30);
  }
  Serial.println(F("  At 100% — 2 second hold"));
  spinAll(ESC_MAX);
  delay(2000);
  Serial.println(F("  Ramping down 100% -> 0%..."));
  for (int i = ESC_MAX; i >= ESC_MIN; i -= 10) {
    spinAll(i);
    delay(30);
  }
  allStop();
  Serial.println(F("  Done."));
}

void escCalibrate() {
  Serial.println(F("=== ESC CALIBRATION ==="));
  Serial.println(F("Calibrates ONE ESC at a time."));
  Serial.println();
  Serial.println(F("STEPS:"));
  Serial.println(F("  1. DISCONNECT battery from ALL ESCs"));
  Serial.println(F("  2. Select motor (1-4)"));
  Serial.println(F("  3. Arduino sends MAX throttle (2000us)"));
  Serial.println(F("  4. Power ON that ESC"));
  Serial.println(F("  5. Wait for calibration beeps (rising tones)"));
  Serial.println(F("  6. Press Enter to send MIN throttle (1000us)"));
  Serial.println(F("  7. Wait for confirmation beep"));
  Serial.println(F("  8. DISCONNECT battery"));
  Serial.println(F("  9. Repeat for next ESC"));
  Serial.println();
  Serial.println(F("Press Enter to confirm battery DISCONNECTED."));

  while (Serial.available()) Serial.read();
  while (!Serial.available());
  while (Serial.available()) Serial.read();

  Serial.print(F("Motor [1-4]: "));
  while (!Serial.available());
  uint8_t motor = Serial.parseInt();
  if (motor < 1 || motor > 4) {
    Serial.println(F("Invalid."));
    return;
  }
  Serial.println(motor);

  Serial.print(F("Sending MAX throttle (2000us) to M"));
  Serial.println(motor);
  Serial.println(F("*** POWER ON that ESC NOW ***"));

  spinOne(motor, ESC_MAX);

  Serial.println(F("Wait for calibration beeps, then press Enter."));
  while (Serial.available()) Serial.read();
  while (!Serial.available());
  while (Serial.available()) Serial.read();

  Serial.println(F("Sending MIN throttle (1000us)..."));
  spinOne(motor, ESC_MIN);

  Serial.println(F("Wait for confirmation beep, then DISCONNECT battery."));
  Serial.println(F("Press Enter when done."));
  while (!Serial.available());
  while (Serial.available()) Serial.read();

  allStop();
  Serial.println(F("ESC calibrated."));
  Serial.println();
}

void spinIndividual() {
  armAllESCs();

  Serial.print(F("Motor [1-4]: "));
  while (!Serial.available());
  uint8_t motor = Serial.parseInt();
  if (motor < 1 || motor > 4) {
    Serial.println(F("Invalid."));
    return;
  }
  Serial.println(motor);

  Serial.print(F("Throttle % [0-100, default 25]: "));
  while (!Serial.available());
  int pct = Serial.parseInt();
  if (pct == 0) pct = 25;
  pct = constrain(pct, 0, 100);
  Serial.println(pct);

  uint16_t us = map(pct, 0, 100, ESC_MIN, ESC_MAX);
  Serial.print(F("M")); Serial.print(motor);
  Serial.print(F(" at ")); Serial.print(pct);
  Serial.print(F("% (")); Serial.print(us); Serial.println(F("us)"));
  Serial.println(F("Send any character to stop."));

  spinOne(motor, us);

  while (!Serial.available());
  while (Serial.available()) Serial.read();
  allStop();
}

void spinAllLow() {
  armAllESCs();
  uint16_t us = 1250;
  Serial.print(F("All motors at ")); Serial.print(us); Serial.println(F("us (~25%)"));
  Serial.println(F("Check rotation direction:"));
  Serial.println(F("  M1(FL) should be CCW"));
  Serial.println(F("  M2(FR) should be CW"));
  Serial.println(F("  M3(RL) should be CW"));
  Serial.println(F("  M4(RR) should be CCW"));
  Serial.println(F("  If wrong, swap any 2 of the 3 ESC-motor wires."));
  Serial.println(F("Send any character to stop."));

  spinAll(us);
  uint32_t start = millis();
  while (!Serial.available()) {
    if (millis() - start > 20000) {
      Serial.println(F("20s timeout — stopping."));
      break;
    }
  }
  while (Serial.available()) Serial.read();
  allStop();
}

void printMenu() {
  Serial.println();
  Serial.println(F("=== ESC Calibration & Motor Test ==="));
  Serial.println(F("  1 = ESC calibration (one at a time)"));
  Serial.println(F("  2 = Spin individual motor"));
  Serial.println(F("  3 = Throttle sweep (single motor)"));
  Serial.println(F("  4 = Spin ALL at 25% (direction check)"));
  Serial.println(F("  5 = Throttle sweep ALL motors"));
  Serial.println(F("  0 = STOP ALL MOTORS"));
  Serial.println(F("  ? = Print this menu"));
  Serial.println();
  Serial.println(F("  M1=D9(FL) M2=D10(FR) M3=D11(RL) M4=D12(RR)"));
  Serial.print(F("> "));
}

void setup() {
  digitalWrite(PIN_M1, LOW);
  digitalWrite(PIN_M2, LOW);
  digitalWrite(PIN_M3, LOW);
  digitalWrite(PIN_M4, LOW);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_M3, OUTPUT);
  pinMode(PIN_M4, OUTPUT);
  delay(10);

  Serial.begin(115200);

  esc1.attach(PIN_M1, ESC_MIN, ESC_MAX);
  esc2.attach(PIN_M2, ESC_MIN, ESC_MAX);
  esc3.attach(PIN_M3, ESC_MIN, ESC_MAX);
  esc4.attach(PIN_M4, ESC_MIN, ESC_MAX);

  allStop();
  Serial.println(F("Sending zero throttle to ESCs..."));
  Serial.println(F("Waiting 5s for ESCs to arm..."));
  delay(5000);

  Serial.println(F("ESCs should be quiet now."));
  Serial.println(F("If still beeping, calibrate with option 1."));
  Serial.println(F("*** REMOVE ALL PROPS! ***"));
  delay(3000);

  for (uint8_t i = 2; i <= 7; i++) pinMode(i, INPUT_PULLUP);

  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case '0':
      allStop();
      Serial.println(F("ALL STOP."));
      break;
    case '1': escCalibrate(); break;
    case '2': spinIndividual(); break;
    case '3':
      Serial.print(F("Motor to sweep [1-4]: "));
      while (!Serial.available());
      { uint8_t m = Serial.parseInt();
        Serial.println(m);
        if (m >= 1 && m <= 4) throttleSweep(m);
        else Serial.println(F("Invalid."));
      }
      break;
    case '4': spinAllLow(); break;
    case '5': throttleSweepAll(); break;
    case '?': break;
    default: return;
  }
  printMenu();
}