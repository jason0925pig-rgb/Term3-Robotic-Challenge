#pragma once

#include <Arduino.h>

#include "constants.h"
#include "globals.h"
#include "types.h"

bool isValidSonarDistance(float mm) {
  return mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm;
}

float readSonarMm(int trigPin, int echoPin) {
  if (!validPin(trigPin) || !validPin(echoPin)) return -1.0f;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (durationUs == 0) return -1.0f;

  return durationUs * 0.1715f;
}

SonarReading readSonars() {
  SonarReading s;

  s.frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  s.frontValid = isValidSonarDistance(s.frontMm);
  if (s.frontValid) lastValidFrontMm = s.frontMm;
  delay(8);

  s.leftMm = readSonarMm(kLeftTrigPin, kLeftEchoPin);
  s.leftValid = isValidSonarDistance(s.leftMm);
  if (s.leftValid) lastValidLeftMm = s.leftMm;
  delay(8);

  s.rightMm = readSonarMm(kRightTrigPin, kRightEchoPin);
  s.rightValid = isValidSonarDistance(s.rightMm);
  if (s.rightValid) lastValidRightMm = s.rightMm;

  return s;
}

bool obstacleAhead(float thresholdMm) {
  const float frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  if (!isValidSonarDistance(frontMm)) return false;
  lastValidFrontMm = frontMm;
  return frontMm <= thresholdMm;
}

int maxWallCorrectionFromRatioLimit(int baseSpeed) {
  if (kMaxFastSlowMotorRatio <= 1.0f || baseSpeed <= 0) return 0;
  const float maxCorrection =
      baseSpeed * (kMaxFastSlowMotorRatio - 1.0f) / (kMaxFastSlowMotorRatio + 1.0f);
  return max(0, static_cast<int>(maxCorrection));
}

float selectedWallDistanceMm(WallSide side, const SonarReading &s, bool *validOut) {
  if (side == WallSide::Left) {
    if (s.leftValid) {
      *validOut = true;
      return s.leftMm;
    }
    if (lastValidLeftMm > 0.0f) {
      *validOut = true;
      return lastValidLeftMm;
    }
  } else {
    if (s.rightValid) {
      *validOut = true;
      return s.rightMm;
    }
    if (lastValidRightMm > 0.0f) {
      *validOut = true;
      return lastValidRightMm;
    }
  }

  *validOut = false;
  return -1.0f;
}

MotorCommand computeWallFollowCommand(WallSide side, float distanceMm, bool *validOut) {
  MotorCommand cmd;

  if (!isValidSonarDistance(distanceMm)) {
    *validOut = false;
    return cmd;
  }
  *validOut = true;

  const uint32_t now = millis();
  float dt = (now - lastWallUpdateMs) / 1000.0f;
  lastWallUpdateMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = kWallLoopDelayMs / 1000.0f;

  const float errorMm = distanceMm - kTargetWallDistanceMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);

  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  const float pid = kWallKp * errorMm + kWallKi * wallIntegral + kWallKd * derivative;
  const int ratioCorrectionLimit = maxWallCorrectionFromRatioLimit(kWallBaseSpeed);
  int activeCorrectionLimit = kWallMaxCorrection;
  if (ratioCorrectionLimit < activeCorrectionLimit) activeCorrectionLimit = ratioCorrectionLimit;

  int correction = constrain(static_cast<int>(pid), -activeCorrectionLimit, activeCorrectionLimit);
  const int turnLeftCorrection = side == WallSide::Left ? correction : -correction;
  cmd.left = kWallBaseSpeed - turnLeftCorrection;
  cmd.right = kWallBaseSpeed + turnLeftCorrection;

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(side == WallSide::Left ? F("LEFT") : F("RIGHT"));
    Serial.print(F(" distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(F(" target="));
    Serial.print(kTargetWallDistanceMm, 1);
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" corr="));
    Serial.print(correction);
    Serial.print(F(" L="));
    Serial.print(cmd.left);
    Serial.print(F(" R="));
    Serial.println(cmd.right);
  }

  return cmd;
}

void applyWallFollowStep(WallSide side) {
  const SonarReading s = readSonars();
  bool validDistance = false;
  const float distanceMm = selectedWallDistanceMm(side, s, &validDistance);

  if (!validDistance) {
    stopMotors();
    Serial.println(F("[WALL] no valid side sonar distance."));
    delay(80);
    return;
  }

  bool validCommand = false;
  const MotorCommand cmd = computeWallFollowCommand(side, distanceMm, &validCommand);
  if (validCommand) setTank(cmd.left, cmd.right);
  else stopMotors();

  updateImu();
  delay(kWallLoopDelayMs);
}
