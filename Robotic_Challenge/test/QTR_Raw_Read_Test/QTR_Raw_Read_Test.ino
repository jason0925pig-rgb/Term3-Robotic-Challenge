#include <Arduino.h>

// ---------------------------------------------------------------------------
// QTR raw reading test
// Purpose:
//   Print the 9 raw RC timing readings from the QTR-HD-09RC array.
//   Run this sketch whenever the QTR board, mounting height, or floor/line
//   material changes. Record the lowest readings seen on white floor as the
//   saved "white" calibration, and the highest readings seen on black line as
//   the saved "black" calibration.
//
// Latest replacement-sensor calibration used by the main tests:
//   saved white/min = {98, 82, 82, 82, 86, 94, 105, 121, 142}
//   saved black/max = {1479, 921, 866, 797, 762, 733, 779, 901, 1177}
//
// The line-following code maps raw readings into 0..1000 using:
//   normalized = (raw - savedMin) * 1000 / (savedMax - savedMin)
// so these values directly affect whether each detector is classified as
// "black line" by kLineThreshold.
// ---------------------------------------------------------------------------

constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr uint32_t kPrintIntervalMs = 250;

uint16_t qtrValues[9] = {};
uint16_t sessionMin[9] = {};
uint16_t sessionMax[9] = {};
uint32_t lastPrintMs = 0;

void readQtrRcArray() {
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], OUTPUT);
    digitalWrite(kQtrPins[i], HIGH);
  }
  delayMicroseconds(10);

  const uint32_t start = micros();
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
    qtrValues[i] = kQtrTimeoutUs;
  }

  bool allDone = false;
  while (!allDone && (micros() - start) < kQtrTimeoutUs) {
    allDone = true;
    const uint16_t elapsed = static_cast<uint16_t>(micros() - start);
    for (uint8_t i = 0; i < 9; i++) {
      if (qtrValues[i] == kQtrTimeoutUs) {
        if (digitalRead(kQtrPins[i]) == LOW) {
          qtrValues[i] = elapsed;
        } else {
          allDone = false;
        }
      }
    }
  }
}

void resetSessionRange() {
  for (uint8_t i = 0; i < 9; i++) {
    sessionMin[i] = kQtrTimeoutUs;
    sessionMax[i] = 0;
  }
}

void updateSessionRange() {
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrValues[i] < sessionMin[i]) sessionMin[i] = qtrValues[i];
    if (qtrValues[i] > sessionMax[i]) sessionMax[i] = qtrValues[i];
  }
}

void printArray(const __FlashStringHelper *label, const uint16_t values[9]) {
  Serial.print(label);
  for (uint8_t i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(values[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
  }

  resetSessionRange();
  Serial.println(F("QTR_Raw_Read_Test ready."));
  Serial.println(F("Record RAW on white floor, then on black line."));
  Serial.println(F("For saved calibration: white ~= min, black ~= max."));
}

void loop() {
  readQtrRcArray();
  updateSessionRange();

  if (millis() - lastPrintMs >= kPrintIntervalMs) {
    lastPrintMs = millis();
    printArray(F("raw:"), qtrValues);
    printArray(F("sessionMin:"), sessionMin);
    printArray(F("sessionMax:"), sessionMax);
    Serial.println();
  }
}
