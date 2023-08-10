/*
 * AFE4404 Basic Read library example.
 * https://github.com/rakshithbk/AFE4404-Library
 * 
 * On Arduino Uno -  | AFE4404 | Uno Pin
 *                   | I2C_Dat | A4
 *                   | I2C_Clk | A5
 * get_led_val() will return uint32_t values.
 * 
 * Advanced - you can modify the sample period and other
 * parameters in the library files (refer AFE4404 datasheet)
 */


#include <Wire.h>
#include "AFE_connect.h"
#include "BeatDetector.h"
#include "Filters.h"

AFE A;
int RST = 2;
int RDY = 3;

BeatDetector beatDetector;
FilterBuLp1 lpf;
DCRemover dcRemover;
LPF lpf1;

void setup() {
  Serial.begin(9600);
  Serial.println("AFE4404 basic readings -\n");

  pinMode(RST, OUTPUT);
  pinMode(RDY, INPUT_PULLUP);


  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);
  delay(500);
  A.init();

  attachInterrupt(digitalPinToInterrupt(RDY), dataReady, CHANGE);
  dcRemover = DCRemover(0.95);
}

volatile bool haveData = false;
void dataReady() {
  haveData = true;
}

long tsLastReport = 0;
bool prev = false, now = false;
#define BPM_LEN 10
#define THRESHOLD_BPM 70
int bpm_i = 0;
float bpms[BPM_LEN];
void loop() {
  if (AFE_Reg_Read(ALED1VAL) < 10000) { // finger is on the sensor
    if (haveData) {
      float sample = dcRemover.step(AFE_Reg_Read(LED1VAL));
      // The signal fed to the beat detector is mirrored since the cleanest monotonic spike is below zero
      float filteredSample = lpf1.step(lpf.step(-sample));
      bool beatDetected = beatDetector.addSample(filteredSample);

      Serial.print(beatDetector.getCurrentThreshold());
      Serial.print(" ");
      Serial.print(filteredSample);
      Serial.println();

      haveData = false;
    }

  }

  if (millis() - tsLastReport > 1000) {
    Serial.println(beatDetector.getRate());
    bpms[bpm_i] = beatDetector.getRate();
    bpm_i = (bpm_i + 1) % BPM_LEN;

    bool allHigher = true;
    bool allLower = true;
    for (int i = 0; i < BPM_LEN; i++) {
      if (bpms[i] > THRESHOLD_BPM) allLower = false;
      if (bpms[i] < THRESHOLD_BPM) allHigher = false;
    }
    
    if (allHigher) now = true;
    if (allLower) now = false;

    if (now != prev) {
      if (now) Serial.println("[ctrl] on");
      else Serial.println("[ctrl] off");
    }

    tsLastReport = millis();
    prev = now;
  }
}
