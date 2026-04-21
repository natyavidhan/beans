#include <ESP32Servo.h>

// ── Pin definitions ────────────────────────────────────────────
#define PIN_CH3_THROTTLE  32   // Receiver CH3 → ESP32
// #define PIN_CH1_AILERON   21   // Receiver CH1 → ESP32 (optional)
// #define PIN_CH2_ELEVATOR  22   // Receiver CH2 → ESP32 (optional)
// #define PIN_CH4_RUDDER    23   // Receiver CH4 → ESP32 (optional)
#define PIN_ESC_1         14   // ESP32 → ESC 1 signal wire
#define PIN_ESC_2         27   // ESP32 → ESC 2 signal wire
#define PIN_ESC_3         26   // ESP32 → ESC 3 signal wire
#define PIN_ESC_4         25   // ESP32 → ESC 4 signal wire

// ── ESC pulse range (microseconds) ────────────────────────────
#define ESC_MIN_US   1000      // Motor stopped
#define ESC_MAX_US   2000      // Motor full throttle

// ── Safety: throttle must be below this % to arm ──────────────
#define ARM_THRESHOLD_US  1050

Servo esc1, esc2, esc3, esc4;

volatile uint32_t rcRiseTimeThr = 0;
volatile int      rcRawThr      = 1000;

void IRAM_ATTR isr_ch3() { 
  if(digitalRead(PIN_CH3_THROTTLE)) 
    rcRiseTimeThr = micros(); 
  else 
    rcRawThr = micros() - rcRiseTimeThr; 
}

// Map receiver us value to ESC us value with safety cap
int throttleToESC(int us) {
  us = constrain(us, 1000, 2000);
  return map(us, 1000, 2000, ESC_MIN_US, ESC_MAX_US);
}

void armESC() {
  Serial.println("Arming ESCs — sending min throttle for 3 seconds...");
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.writeMicroseconds(ESC_MIN_US);
  esc3.writeMicroseconds(ESC_MIN_US);
  esc4.writeMicroseconds(ESC_MIN_US);
  delay(3000);
  Serial.println("ESCs Armed! You can now apply throttle.");
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(PIN_CH3_THROTTLE, isr_ch3, CHANGE);
  // pinMode(PIN_CH1_AILERON,  INPUT);
  // pinMode(PIN_CH2_ELEVATOR, INPUT);
  // pinMode(PIN_CH4_RUDDER,   INPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  
  // Attach all 4 ESCs
  esc1.setPeriodHertz(50);
  esc1.attach(PIN_ESC_1, ESC_MIN_US, ESC_MAX_US);
  
  esc2.setPeriodHertz(50);
  esc2.attach(PIN_ESC_2, ESC_MIN_US, ESC_MAX_US);
  
  esc3.setPeriodHertz(50);
  esc3.attach(PIN_ESC_3, ESC_MIN_US, ESC_MAX_US);
  
  esc4.setPeriodHertz(50);
  esc4.attach(PIN_ESC_4, ESC_MIN_US, ESC_MAX_US);

  // ── Safety check: throttle must be low before arming ────────
  Serial.println("Checking throttle position...");
  while (true) {
    int thr = rcRawThr;
    Serial.printf("Throttle: %d us\n", thr);
    if (thr > 0 && thr < ARM_THRESHOLD_US) {
      Serial.println("Throttle low — OK to arm.");
      break;
    }
    Serial.println("LOWER THROTTLE TO MINIMUM before arming!");
    delay(500);
  }

  armESC();
}

void loop() {
  // Read all channels
  int thr = rcRawThr;
  // int ail = readPWM(PIN_CH1_AILERON);
  // int ele = readPWM(PIN_CH2_ELEVATOR);
  // int rud = readPWM(PIN_CH4_RUDDER);

  // Safety: if signal lost (pulseIn returns 0), cut throttle
  bool signalLost = (rcRawThr < 800 || rcRawThr > 2200);
  if (signalLost) {
    Serial.println("⚠ SIGNAL LOST — cutting throttle!");
    esc1.writeMicroseconds(ESC_MIN_US);
    esc2.writeMicroseconds(ESC_MIN_US);
    esc3.writeMicroseconds(ESC_MIN_US);
    esc4.writeMicroseconds(ESC_MIN_US);
    delay(100);
    return;
  }

  // Send throttle to all 4 ESCs
  int escSignal = throttleToESC(thr);
  esc1.writeMicroseconds(escSignal);
  esc2.writeMicroseconds(escSignal);
  esc3.writeMicroseconds(escSignal);
  esc4.writeMicroseconds(escSignal);

  // Debug output
  Serial.printf(
    "THR: %4d us → ESC: %4d us | AIL: %4d | ELE: %4d | RUD: %4d\n",
    thr, escSignal//, ail, ele, rud
  );

  delay(20);  // ~50Hz loop
}