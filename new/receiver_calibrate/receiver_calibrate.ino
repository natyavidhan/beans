// ═══════════════════════════════════════════════════════════════
//  FS-R6B RC Calibration — Arduino Nano
//
//  Wiring (same as receiver_readout):
//    CH1→D2  CH2→D3  CH3→D4  CH4→D5  CH5→D6  CH6→D7
//    GND→GND  VCC→5V
//
//  Serial Monitor: 115200 baud, line ending: Newline
//
//  Steps (guided on-screen):
//    1. Power on with sticks centered → records midpoint
//    2. Move all sticks/switches to every extreme → records min/max
//    3. Return sticks to center → prints final calibration table
// ═══════════════════════════════════════════════════════════════

#define PIN_CH1 2
#define PIN_CH2 3
#define PIN_CH3 4
#define PIN_CH4 5
#define PIN_CH5 6
#define PIN_CH6 7

#define NUM_CH 6

const char* chName[NUM_CH] = {"CH1 Roll ", "CH2 Pitch", "CH3 Thr  ", "CH4 Yaw  ", "CH5 AUX1 ", "CH6 AUX2 "};

volatile uint32_t rcRiseTime[NUM_CH] = {0};
volatile uint16_t rcRaw[NUM_CH] = {0};
volatile uint8_t lastPIND = 0;

ISR(PCINT2_vect) {
  uint8_t nowPIND = PIND;
  uint8_t changed = nowPIND ^ lastPIND;
  uint32_t now = micros();

  if (changed & (1 << PIN_CH1)) {
    if (nowPIND & (1 << PIN_CH1)) rcRiseTime[0] = now;
    else rcRaw[0] = (uint16_t)(now - rcRiseTime[0]);
  }
  if (changed & (1 << PIN_CH2)) {
    if (nowPIND & (1 << PIN_CH2)) rcRiseTime[1] = now;
    else rcRaw[1] = (uint16_t)(now - rcRiseTime[1]);
  }
  if (changed & (1 << PIN_CH3)) {
    if (nowPIND & (1 << PIN_CH3)) rcRiseTime[2] = now;
    else rcRaw[2] = (uint16_t)(now - rcRiseTime[2]);
  }
  if (changed & (1 << PIN_CH4)) {
    if (nowPIND & (1 << PIN_CH4)) rcRiseTime[3] = now;
    else rcRaw[3] = (uint16_t)(now - rcRiseTime[3]);
  }
  if (changed & (1 << PIN_CH5)) {
    if (nowPIND & (1 << PIN_CH5)) rcRiseTime[4] = now;
    else rcRaw[4] = (uint16_t)(now - rcRiseTime[4]);
  }
  if (changed & (1 << PIN_CH6)) {
    if (nowPIND & (1 << PIN_CH6)) rcRiseTime[5] = now;
    else rcRaw[5] = (uint16_t)(now - rcRiseTime[5]);
  }

  lastPIND = nowPIND;
}

uint16_t chMid[NUM_CH];
uint16_t chMin[NUM_CH] = {65535, 65535, 65535, 65535, 65535, 65535};
uint16_t chMax[NUM_CH] = {0, 0, 0, 0, 0, 0};

bool signalOK(uint16_t* snap) {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (snap[i] < 800 || snap[i] > 2200) return false;
  }
  return true;
}

void snapChannels(uint16_t* out) {
  noInterrupts();
  for (uint8_t i = 0; i < NUM_CH; i++) out[i] = rcRaw[i];
  interrupts();
}

void waitForSerial(const char* msg) {
  Serial.println(msg);
  Serial.println(F("  => Send any character to continue..."));
  while (!Serial.available());
  while (Serial.available()) Serial.read();
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 2; i <= 7; i++) pinMode(i, INPUT_PULLUP);
  lastPIND = PIND;
  PCICR |= (1 << PCIE2);
  PCMSK2 = 0xFC;

  uint16_t snap[NUM_CH];

  // ── Wait for valid signal ────────────────────────
  Serial.println(F("=== FS-R6B RC Calibration ==="));
  Serial.println();
  Serial.println(F("Power on your transmitter."));
  Serial.println(F("Waiting for valid signal on all 6 channels..."));

  while (1) {
    snapChannels(snap);
    if (signalOK(snap)) break;
    delay(200);
  }
  Serial.println(F("Signal detected!"));

  // ── Step 1: Center sticks ─────────────────────────
  waitForSerial("STEP 1: Center ALL sticks and switches, then press Enter");

  uint32_t sum[NUM_CH] = {0};
  uint16_t count = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    snapChannels(snap);
    if (signalOK(snap)) {
      for (uint8_t i = 0; i < NUM_CH; i++) sum[i] += snap[i];
      count++;
    }
    delay(10);
  }
  for (uint8_t i = 0; i < NUM_CH; i++) chMid[i] = sum[i] / count;

  Serial.println(F("  Midpoints captured:"));
  for (uint8_t i = 0; i < NUM_CH; i++) {
    Serial.print(F("    "));
    Serial.print(chName[i]);
    Serial.print(F(" = "));
    Serial.println(chMid[i]);
  }
  Serial.println();

  // ── Step 2: Move sticks to extremes ───────────────
  waitForSerial("STEP 2: Move ALL sticks and switches to their full extremes.\n  Move roll L/R, pitch F/B, throttle min/max, yaw L/R,\n  flick all switches. Take 10 seconds, then return to center.\n  Press Enter when ready to START recording.");

  Serial.println(F("  Recording... move everything!"));
  t0 = millis();
  while (millis() - t0 < 10000) {
    snapChannels(snap);
    if (signalOK(snap)) {
      for (uint8_t i = 0; i < NUM_CH; i++) {
        if (snap[i] < chMin[i]) chMin[i] = snap[i];
        if (snap[i] > chMax[i]) chMax[i] = snap[i];
      }
    }
    delay(5);
  }

  // ── Step 3: Return to center ──────────────────────
  waitForSerial("STEP 3: Return ALL sticks to center. Press Enter to finish.");

  // ── Final report ────────────────────────────────
  Serial.println();
  Serial.println(F("╔══════════════════════════════════════════════════╗"));
  Serial.println(F("║          RC CALIBRATION RESULTS                 ║"));
  Serial.println(F("╠══════════════════════════════════════════════════╣"));
  Serial.println(F("║ Channel       │ Min   │ Mid   │ Max   │ Range  ║"));
  Serial.println(F("╠───────────────┼───────┼───────┼───────┼────────╣"));

  for (uint8_t i = 0; i < NUM_CH; i++) {
    Serial.print(F("║ "));
    Serial.print(chName[i]);
    Serial.print(F(" │ "));
    if (chMin[i] == 65535) Serial.print(F("  ---")); else { Serial.print(chMin[i]); if (chMin[i] < 1000) Serial.print(' '); }
    Serial.print(F(" │ "));
    Serial.print(chMid[i]);
    Serial.print(F(" │ "));
    Serial.print(chMax[i]);
    Serial.print(F(" │ "));
    Serial.print(chMax[i] - chMin[i]);
    Serial.println(F("  ║"));
  }

  Serial.println(F("╚══════════════════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Copy-paste ready array:"));
  Serial.println();

  Serial.print(F("RCChannel RC[6] = {\n"));
  for (uint8_t i = 0; i < NUM_CH; i++) {
    Serial.print(F("  {"));
    Serial.print(chMin[i]);
    Serial.print(F(", "));
    Serial.print(chMid[i]);
    Serial.print(F(", "));
    Serial.print(chMax[i]);
    if (i < NUM_CH - 1) Serial.println(F("},"));
    else Serial.println(F("}"));
  }
  Serial.println(F("};"));

  Serial.println();
  Serial.println(F("Calibration complete. You can close serial monitor."));
  Serial.println(F("Paste the above array into your flight controller code."));

  while (1);
}

void loop() {}