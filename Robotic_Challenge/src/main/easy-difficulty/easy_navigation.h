#pragma once

#include <Arduino.h>

#include "easy_rfid.h"
#include "easy_server.h"

constexpr float kEasyCellPitchMm = 250.0f;
constexpr int kEasyStraightDriveSpeed = 360;
constexpr uint32_t kEasyMoveTimeoutMs = 12000;

struct EasyMoveResult {
  bool ok = false;
  bool sawUid = false;
  String uid;
  Cell cell = {};
  const __FlashStringHelper *reason = F("none");
};

inline float turnDegreesForDirectionChange(Direction current, Direction desired) {
  if (current == Direction::Error || desired == Direction::Error) return 0.0f;
  const int diff = (static_cast<int>(desired) - static_cast<int>(current) + 4) % 4;
  if (diff == 0) return 0.0f;
  if (diff == 1) return -90.0f;
  if (diff == 3) return 90.0f;
  return 180.0f;
}

inline bool easyMovementSafetyOk() {
  handleSerialCommands();
  easyServerLoop();
  updateImu();

  if (killPressed() || serialStopped || !easyMotionAllowedByWifi()) {
    stopMotors();
    return false;
  }

  return true;
}

inline void encoderOnlyTurnEasyFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();

  while (true) {
    if (!easyMovementSafetyOk()) break;

    const long averageAbs = (absLong(getLeftCount()) + absLong(getRightCount())) / 2;
    if (averageAbs >= targetCounts) break;

    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] easy encoder fallback timeout."));
      break;
    }

    setTurnCommand(direction * abs(speed));
    delay(10);
  }

  stopMotors();
}

inline bool turnDegreesEasy(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnEasyFallback(targetDeg, kTurnMaxSpeed);
    return !(killPressed() || serialStopped || !easyMotionAllowedByWifi());
  }

  const long encoderTarget = turnDegreesToEncoderCounts(targetDeg);
  Serial.print(F("[TURN] easy IMU targetDeg="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" encoderTarget="));
  Serial.println(encoderTarget);

  resetEncoders();
  resetYaw();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    const float errorDeg = targetDeg - yawDeg;
    if (absFloat(errorDeg) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("[TURN] easy IMU target reached."));
      break;
    }

    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] easy IMU timeout."));
      break;
    }

    float commandFloat = kTurnKp * errorDeg - kTurnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, kTurnMinSpeed, kTurnMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;

    setTurnCommand(signedCommand);
    printTurnStatus("[EasyTurn]", targetDeg, errorDeg, signedCommand, encoderTarget);
    delay(10);
  }

  stopMotors();
  updateImu();
  return true;
}

inline bool driveDistanceEasyMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  resetEncoders();
  const uint32_t start = millis();

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;

    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("[DRIVE] easy distance timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);

    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);
    delay(10);
  }

  stopMotors();
  return true;
}

inline bool alignToDirection(Direction *currentHeading, Direction targetDirection) {
  if (targetDirection == Direction::Error) return false;

  const float turnDeg = turnDegreesForDirectionChange(*currentHeading, targetDirection);
  if (absFloat(turnDeg) > 1.0f) {
    Serial.print(F("[NAV] turn from "));
    Serial.print(directionName(*currentHeading));
    Serial.print(F(" to "));
    Serial.print(directionName(targetDirection));
    Serial.print(F(" deg="));
    Serial.println(turnDeg, 1);
    if (!turnDegreesEasy(turnDeg)) return false;
  }

  *currentHeading = targetDirection;
  return true;
}

inline bool acceptMovementUid(const String &uid, const String &currentUid, Cell targetCell,
                              EasyMoveResult *result) {
  if (uid == currentUid) return false;

  Cell seenCell = {};
  if (!cellForUid(uid.c_str(), &seenCell)) {
    stopMotors();
    Serial.print(F("[RFID] unknown UID during movement: "));
    Serial.println(uid);
    result->ok = false;
    result->sawUid = true;
    result->uid = uid;
    result->reason = F("unknown_rfid");
    return true;
  }

  stopMotors();
  result->sawUid = true;
  result->uid = uid;
  result->cell = seenCell;

  if (!sameCell(seenCell, targetCell)) {
    Serial.print(F("[RFID] expected "));
    printCell(targetCell);
    Serial.print(F(" but saw "));
    printCell(seenCell);
    Serial.print(F(" uid="));
    Serial.println(uid);
    result->ok = false;
    result->reason = F("wrong_rfid");
    return true;
  }

  result->ok = true;
  result->reason = F("rfid_target");
  return true;
}

inline bool searchLineUntilAnyEasyRfid(String *uidOut) {
  if (!easyRfidOk) {
    Serial.println(F("[RFID SEARCH] reader not ready."));
    stopMotors();
    return false;
  }

  Serial.println(F("[RFID SEARCH] following line until first RFID tag."));
  lineIntegral = 0.0f;
  lastLineError = 0;

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    String uid;
    if (pollEasyRfidDebounced(&uid)) {
      stopMotors();
      *uidOut = uid;
      Serial.print(F("[RFID SEARCH] detected uid="));
      Serial.println(uid);
      return true;
    }

    applyLineFollowStep();
  }
}

inline EasyMoveResult moveLineFollowUntilRfid(Cell targetCell, const String &currentUid) {
  EasyMoveResult result;
  const uint32_t start = millis();

  Serial.print(F("[NAV] line-follow to "));
  printCell(targetCell);
  Serial.println();

  lineIntegral = 0.0f;
  lastLineError = 0;

  while (millis() - start < kEasyMoveTimeoutMs) {
    if (!easyMovementSafetyOk()) {
      result.reason = F("safety_stop");
      return result;
    }

    String uid;
    if (pollEasyRfidDebounced(&uid) && acceptMovementUid(uid, currentUid, targetCell, &result)) {
      return result;
    }

    applyLineFollowStep();
  }

  stopMotors();
  result.reason = F("line_move_timeout");
  Serial.println(F("[NAV] line-follow timeout before target RFID."));
  return result;
}

inline EasyMoveResult driveStraightUntilRfidOrDistance(Cell targetCell, const String &currentUid) {
  EasyMoveResult result;
  const long targetCounts = distanceMmToCounts(kEasyCellPitchMm);
  const uint32_t start = millis();

  Serial.print(F("[NAV] straight-drive to "));
  printCell(targetCell);
  Serial.print(F(" fallbackMm="));
  Serial.println(kEasyCellPitchMm, 1);

  resetEncoders();

  while (millis() - start < kEasyMoveTimeoutMs) {
    if (!easyMovementSafetyOk()) {
      result.reason = F("safety_stop");
      return result;
    }

    String uid;
    if (pollEasyRfidDebounced(&uid) && acceptMovementUid(uid, currentUid, targetCell, &result)) {
      return result;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) {
      stopMotors();
      result.ok = true;
      result.sawUid = false;
      result.cell = targetCell;
      result.reason = F("distance_fallback");
      Serial.print(F("[NAV] reached fallback distance for "));
      printCell(targetCell);
      Serial.println(F("; using expected UID."));
      return result;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
    setTank(kEasyStraightDriveSpeed - correction, kEasyStraightDriveSpeed + correction);
    delay(10);
  }

  stopMotors();
  result.reason = F("straight_move_timeout");
  Serial.println(F("[NAV] straight-drive timeout before target."));
  return result;
}

inline EasyMoveResult moveToNextEasyCell(Cell currentCell, Cell nextCell, const String &currentUid,
                                         Direction *currentHeading) {
  EasyMoveResult result;
  const Direction targetDirection = directionFromTo(currentCell, nextCell);
  if (!alignToDirection(currentHeading, targetDirection)) {
    result.reason = F("align_failed");
    return result;
  }

  if (movementModeForCell(nextCell) == MovementMode::LineFollow) {
    return moveLineFollowUntilRfid(nextCell, currentUid);
  }

  return driveStraightUntilRfidOrDistance(nextCell, currentUid);
}
