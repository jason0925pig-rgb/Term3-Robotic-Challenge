#pragma once

#include <Arduino.h>

constexpr uint8_t kEasyServoPin = 33;
constexpr int kEasyServoMinUs = 500;
constexpr int kEasyServoMaxUs = 2500;
constexpr int kEasyServoMinAngle = 0;
constexpr int kEasyServoMaxAngle = 300;
constexpr int kEasyServoStepAngle = 60;
constexpr uint32_t kEasyServoMoveSettleMs = 600;
constexpr uint32_t kEasyServoHoldAfterDropMs = 1200;
constexpr uint32_t kEasyServoFrameUs = 20000;
constexpr float kEasyPlantCenterOffsetMm = 90.0f;
constexpr int kEasyPlantCenterDriveSpeed = 360;

static int easyCurrentServoAngle = kEasyServoMinAngle;

inline int easyAngleToPulseUs(int angle) {
  angle = constrain(angle, kEasyServoMinAngle, kEasyServoMaxAngle);
  return map(angle, kEasyServoMinAngle, kEasyServoMaxAngle, kEasyServoMinUs, kEasyServoMaxUs);
}

inline void easySendServoPulse(int pulseUs) {
  digitalWrite(kEasyServoPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(kEasyServoPin, LOW);
  delayMicroseconds(kEasyServoFrameUs - pulseUs);
}

inline void holdEasyServoAngle(int angle, uint32_t durationMs) {
  const int pulseUs = easyAngleToPulseUs(angle);
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    handleSerialCommands();
    easyServerLoop();
    updateImu();

    if (killPressed() || serialStopped || !easyMotionAllowedByWifi()) {
      stopMotors();
      return;
    }

    easySendServoPulse(pulseUs);
  }
}

inline void moveEasyServoToAngle(int angle) {
  easyCurrentServoAngle = constrain(angle, kEasyServoMinAngle, kEasyServoMaxAngle);
  Serial.print(F("[SERVO] angle="));
  Serial.print(easyCurrentServoAngle);
  Serial.print(F(" pulseUs="));
  Serial.println(easyAngleToPulseUs(easyCurrentServoAngle));
  holdEasyServoAngle(easyCurrentServoAngle, kEasyServoMoveSettleMs);
}

inline void initializeEasyPlanting() {
  pinMode(kEasyServoPin, OUTPUT);
  moveEasyServoToAngle(kEasyServoMinAngle);
  holdEasyServoAngle(kEasyServoMinAngle, 700);
}

inline bool dropOneEasySeed() {
  if (killPressed() || serialStopped || !easyMotionAllowedByWifi()) {
    stopMotors();
    return false;
  }

  int nextAngle = easyCurrentServoAngle + kEasyServoStepAngle;
  if (nextAngle > kEasyServoMaxAngle) {
    Serial.println(F("[SERVO] reached 300 deg; reset to 0 before continuing."));
    moveEasyServoToAngle(kEasyServoMinAngle);
    holdEasyServoAngle(kEasyServoMinAngle, 500);
    nextAngle = kEasyServoStepAngle;
  }

  Serial.println(F("[PLANT] dropping one seed."));
  moveEasyServoToAngle(nextAngle);
  holdEasyServoAngle(easyCurrentServoAngle, kEasyServoHoldAfterDropMs);
  return !(killPressed() || serialStopped || !easyMotionAllowedByWifi());
}
