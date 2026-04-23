#include <Arduino.h>

// ═══════════════════════════════════════════════════
//  PINS (Matches flight_full.ino)
// ═══════════════════════════════════════════════════
#define PIN_CH1_AIL   34
#define PIN_CH2_ELE   35
#define PIN_CH3_THR   32
#define PIN_CH4_RUD   33
#define PIN_CH5_AUX1  12
#define PIN_CH6_AUX2  13

// ═══════════════════════════════════════════════════
//  RC GLOBALS & ISRs
// ═══════════════════════════════════════════════════
volatile uint32_t rcRiseTime[6] = {0};
volatile int      rcRaw[6]      = {1000, 1500, 1000, 1500, 1000, 1500};

void IRAM_ATTR isr_ch1() { if(digitalRead(PIN_CH1_AIL)) rcRiseTime[0]=micros(); else rcRaw[0]=micros()-rcRiseTime[0]; }
void IRAM_ATTR isr_ch2() { if(digitalRead(PIN_CH2_ELE)) rcRiseTime[1]=micros(); else rcRaw[1]=micros()-rcRiseTime[1]; }
void IRAM_ATTR isr_ch3() { if(digitalRead(PIN_CH3_THR)) rcRiseTime[2]=micros(); else rcRaw[2]=micros()-rcRiseTime[2]; }
void IRAM_ATTR isr_ch4() { if(digitalRead(PIN_CH4_RUD)) rcRiseTime[3]=micros(); else rcRaw[3]=micros()-rcRiseTime[3]; }
void IRAM_ATTR isr_ch5() { if(digitalRead(PIN_CH5_AUX1)) rcRiseTime[4]=micros(); else rcRaw[4]=micros()-rcRiseTime[4]; }
void IRAM_ATTR isr_ch6() { if(digitalRead(PIN_CH6_AUX2)) rcRiseTime[5]=micros(); else rcRaw[5]=micros()-rcRiseTime[5]; }

uint32_t lastPrintTime = 0;

void setup() {
  Serial.begin(115200);

  // Setup pins
  pinMode(PIN_CH1_AIL, INPUT);
  pinMode(PIN_CH2_ELE, INPUT);
  pinMode(PIN_CH3_THR, INPUT);
  pinMode(PIN_CH4_RUD, INPUT);
  pinMode(PIN_CH5_AUX1, INPUT);
  pinMode(PIN_CH6_AUX2, INPUT);

  // Attach interrupts
  attachInterrupt(PIN_CH1_AIL,  isr_ch1, CHANGE);
  attachInterrupt(PIN_CH2_ELE,  isr_ch2, CHANGE);
  attachInterrupt(PIN_CH3_THR,  isr_ch3, CHANGE);
  attachInterrupt(PIN_CH4_RUD,  isr_ch4, CHANGE);
  attachInterrupt(PIN_CH5_AUX1, isr_ch5, CHANGE);
  attachInterrupt(PIN_CH6_AUX2, isr_ch6, CHANGE);

  Serial.println("RC Debug Started. Waiting for signals...");
}

void loop() {
  uint32_t now = millis();
  
  // Print every 50ms (20Hz)
  if (now - lastPrintTime >= 50) {
    lastPrintTime = now;
    
    // Disable interrupts briefly to safely copy volatile data
    noInterrupts();
    int ch1 = rcRaw[0];
    int ch2 = rcRaw[1];
    int ch3 = rcRaw[2];
    int ch4 = rcRaw[3];
    int ch5 = rcRaw[4];
    int ch6 = rcRaw[5];
    interrupts();

    // Send formatted string to Python dashboard
    Serial.print("RC:");
    Serial.print(ch1); Serial.print(",");
    Serial.print(ch2); Serial.print(",");
    Serial.print(ch3); Serial.print(",");
    Serial.print(ch4); Serial.print(",");
    Serial.print(ch5); Serial.print(",");
    Serial.println(ch6);
  }
}
