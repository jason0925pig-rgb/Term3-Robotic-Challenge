#include <Arduino.h>

// ---------------------------------------------------------------------------
// Sonar distance test sketch
// Board: Arduino GIGA R1 WiFi
// Purpose: only test three HC-SR04 ultrasonic distance sensors.
// ---------------------------------------------------------------------------

constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kLeftTrigPin = 9;
constexpr uint8_t kRightTrigPin = 10;

constexpr uint8_t kFrontEchoPin = 11;
constexpr uint8_t kLeftEchoPin = 12;
constexpr uint8_t kRightEchoPin = 13;

constexpr uint32_t kEchoTimeoutUs = 30000;  // ~5m max wait; HC-SR04 practical range is less.
constexpr uint32_t kPrintIntervalMs = 250;
constexpr uint16_t kSensorGapMs = 35;       // Avoid ultrasonic cross-talk.

uint32_t lastPrintMs = 0;

float readSonarCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (durationUs == 0) {
    return -1.0f;
  }

  return durationUs / 58.0f;
}

void printDistance(const __FlashStringHelper *label, float cm) {
  Serial.print(label);
  Serial.print(F("="));
  if (cm < 0.0f) {
    Serial.print(F("timeout"));
  } else {
    Serial.print(cm, 1);
    Serial.print(F("cm"));
  }
  Serial.print(' ');
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(kFrontTrigPin, OUTPUT);
  pinMode(kLeftTrigPin, OUTPUT);
  pinMode(kRightTrigPin, OUTPUT);
  pinMode(kFrontEchoPin, INPUT);
  pinMode(kLeftEchoPin, INPUT);
  pinMode(kRightEchoPin, INPUT);

  digitalWrite(kFrontTrigPin, LOW);
  digitalWrite(kLeftTrigPin, LOW);
  digitalWrite(kRightTrigPin, LOW);

  Serial.println(F("Sonar_Distance_Test ready."));
  Serial.println(F("Pins: front trig D8 echo D11, left trig D9 echo D12, right trig D10 echo D13."));
  Serial.println(F("Echo pins must be level-shifted to 3.3V for Arduino GIGA."));
}

void loop() {
  if (millis() - lastPrintMs < kPrintIntervalMs) return;
  lastPrintMs = millis();

  const float frontCm = readSonarCm(kFrontTrigPin, kFrontEchoPin);
  delay(kSensorGapMs);
  const float leftCm = readSonarCm(kLeftTrigPin, kLeftEchoPin);
  delay(kSensorGapMs);
  const float rightCm = readSonarCm(kRightTrigPin, kRightEchoPin);

  printDistance(F("front"), frontCm);
  printDistance(F("left"), leftCm);
  printDistance(F("right"), rightCm);
  Serial.println();
}
