#include <Arduino.h>

// ---------------------------------------------------------------------------
// DS-R005 300-degree positional servo test
// Board: Arduino GIGA R1 WiFi
//
// Wiring:
//   Servo signal -> GIGA D33
//   Servo VCC    -> external regulated 5V-6V
//   Servo GND    -> common GND with GIGA
//
// Behavior:
//   Reset to 0 degrees.
//   Move 60 degrees every 3 seconds until 300 degrees.
//   Reset to 0 degrees, wait 5 seconds, repeat.
// ---------------------------------------------------------------------------

constexpr uint8_t SERVO_PIN = 33;

// 300-degree servo PWM range verified on your DS-R005 test.
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

constexpr int MIN_ANGLE = 0;
constexpr int MAX_ANGLE = 300;
constexpr int STEP_ANGLE = 60;

constexpr unsigned long STOP_EACH_STEP_MS = 3000;
constexpr unsigned long RESET_WAIT_MS = 5000;
constexpr unsigned long MOVE_SETTLE_MS = 600;
constexpr unsigned long SERVO_FRAME_US = 20000;

int angleToPulseUs(int angle) {
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
  return map(angle, MIN_ANGLE, MAX_ANGLE, SERVO_MIN_US, SERVO_MAX_US);
}

void sendServoPulse(int pulseUs) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseUs);

  digitalWrite(SERVO_PIN, LOW);
  delayMicroseconds(SERVO_FRAME_US - pulseUs);
}

void holdAngle(int angle, unsigned long durationMs) {
  const int pulseUs = angleToPulseUs(angle);
  const unsigned long start = millis();

  while (millis() - start < durationMs) {
    sendServoPulse(pulseUs);
  }
}

void moveToAngle(int angle) {
  const int pulseUs = angleToPulseUs(angle);

  Serial.print(F("Move to "));
  Serial.print(angle);
  Serial.print(F(" deg, pulse = "));
  Serial.println(pulseUs);

  holdAngle(angle, MOVE_SETTLE_MS);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(SERVO_PIN, OUTPUT);

  moveToAngle(0);
  holdAngle(0, 1000);

  Serial.println(F("DS-R005 300 degree servo test start."));
}

void loop() {
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += STEP_ANGLE) {
    moveToAngle(angle);
    holdAngle(angle, STOP_EACH_STEP_MS);
  }

  moveToAngle(0);
  holdAngle(0, RESET_WAIT_MS);
}
