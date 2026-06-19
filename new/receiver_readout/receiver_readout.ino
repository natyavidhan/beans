// ═══════════════════════════════════════════════════════════════
//  FS-R6B 6-Channel PWM Receiver Reader — Arduino Nano
//
//  Uses Pin Change Interrupts (PCINT2) on PORTD (D2-D7) to
//  measure all 6 PWM channels via the XOR snapshot trick.
//
//  Wiring:
//    FS-R6B CH1 → D2 (Aileron / Roll)
//    FS-R6B CH2 → D3 (Elevator / Pitch)
//    FS-R6B CH3 → D4 (Throttle)
//    FS-R6B CH4 → D5 (Rudder / Yaw)
//    FS-R6B CH5 → D6 (AUX1 / Mode Switch)
//    FS-R6B CH6 → D7 (AUX2)
//    FS-R6B GND → GND
//    FS-R6B VCC → 5V
//
//  Serial Monitor: 115200 baud
//  Send 'r' to reset min/max calibration values
// ═══════════════════════════════════════════════════════════════

#define PIN_CH1 2
#define PIN_CH2 3
#define PIN_CH3 4
#define PIN_CH4 5
#define PIN_CH5 6
#define PIN_CH6 7

#define NUM_CH 6
#define RC_TIMEOUT_MS 500

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

uint16_t ch[NUM_CH];
uint16_t chMin[NUM_CH] = {65535, 65535, 65535, 65535, 65535, 65535};
uint16_t chMax[NUM_CH] = {0, 0, 0, 0, 0, 0};
uint32_t lastUpdate[NUM_CH] = {0};
const char* chName[NUM_CH] = {"Roll ", "Pitch", "Thr  ", "Yaw  ", "AUX1 ", "AUX2 "};

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 2; i <= 7; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  lastPIND = PIND;

  PCICR |= (1 << PCIE2);
  PCMSK2 = 0xFC;

  Serial.println(F("=== FS-R6B Receiver Reader ==="));
  Serial.println(F("Roll Pitch Thr Yaw AUX1 AUX2  ('r'=reset)"));
  Serial.println();
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  noInterrupts();
  for (uint8_t i = 0; i < NUM_CH; i++) ch[i] = rcRaw[i];
  interrupts();

  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (ch[i] > 800 && ch[i] < 2200) {
      lastUpdate[i] = now;
      if (ch[i] < chMin[i]) chMin[i] = ch[i];
      if (ch[i] > chMax[i]) chMax[i] = ch[i];
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      for (uint8_t i = 0; i < NUM_CH; i++) {
        chMin[i] = 65535;
        chMax[i] = 0;
      }
      Serial.println(F("-- min/max reset --"));
    }
  }

  if (now - lastPrint >= 200) {
  lastPrint = now;

  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (now - lastUpdate[i] <= RC_TIMEOUT_MS)
      Serial.print(ch[i]);
    else
      Serial.print(F("---"));
    if (i < NUM_CH - 1) Serial.print(' ');
  }
  Serial.println();
}
}