#pragma once

#include "hard_config.h"

inline TurnDir routeTurnAt(uint8_t index) {
  if (routeChoice == RouteChoice::BaseA_Bottom) {
    // A right/lower route: RIGHT, LEFT, LEFT, RIGHT.
    if (index == 0 || index == 3) return TurnDir::Right;
    return TurnDir::Left;
  }

  // B left/upper route: LEFT, RIGHT, RIGHT, LEFT.
  if (index == 0 || index == 3) return TurnDir::Left;
  return TurnDir::Right;
}

/**
 * Convert route turn direction to IMU degrees.
 *
 * @param dir Turn direction.
 * @return +90 for left, -90 for right.
 */
inline float degreesForTurn(TurnDir dir) {
  return dir == TurnDir::Left ? 90.0f : -90.0f;
}

/**
 * Detect the first T/bifurcation using near-all-black QTR topology.
 *
 * A wide black bar can produce a biased weighted line position when one side of
 * the sensor array is darker than the other. For that reason this detector uses
 * coverage across the array instead of rejecting on line.error.
 *
 * @param line Current line reading.
 * @return true when the first T pattern is present.
 */
inline bool firstTDetected(const LineReading &line) {
  if (!line.detected) return false;
  if (currentRouteSegmentTravelMm() < kFirstTMinTravelMm) return false;
  if (line.activeCount < kFirstTMinActiveSensors) return false;
  if (!centerHasLine()) return false;

  const bool leftEdgeStrong =
      qtrNorm[0] >= kFirstTEdgeStrongThreshold || qtrNorm[1] >= kFirstTEdgeStrongThreshold;
  const bool rightEdgeStrong =
      qtrNorm[7] >= kFirstTEdgeStrongThreshold || qtrNorm[8] >= kFirstTEdgeStrongThreshold;
  if (!leftEdgeStrong || !rightEdgeStrong) return false;

  if (activeSensorRangeCount(2, 6, kLineThreshold) < kFirstTMiddleMinActiveSensors) {
    return false;
  }

  return qtrTotalStrength() >= kFirstTMinTotalStrength;
}

/**
 * Detect later hard-turn events only in the expected route direction.
 *
 * @param line Current line reading.
 * @param expected Expected scripted turn direction.
 * @return true when current evidence matches the expected turn.
 */
inline bool expectedSharpTurnDetected(const LineReading &line, TurnDir expected) {
  if (!line.detected) return false;

  if (expected == TurnDir::Left) {
    return line.mode == FollowMode::HardLeft || (leftOuterHasStrongLine() && line.error < -kCenterRecoverError);
  }

  return line.mode == FollowMode::HardRight || (rightOuterHasStrongLine() && line.error > kCenterRecoverError);
}

/**
 * Debounce route-event detection.
 *
 * @param line Current line reading.
 * @return true when the next committed route turn should run.
 */
inline bool routeEventReady(const LineReading &line) {
  if (routeTurnIndex >= kRouteTurnCount) return false;
  if (millis() - lastEventMs < kEventCooldownMs) return false;
  if (postTurnHardIgnoreActive && isHardLineMode(line.mode)) {
    eventStableCount = 0;
    return false;
  }

  const TurnDir expected = routeTurnAt(routeTurnIndex);
  const bool firstRouteTurn = routeTurnIndex == 0;
  const uint32_t now = millis();

  if (firstRouteTurn) {
    if (firstTDetected(line)) {
      if (lastFirstTEvidenceMs == 0 ||
          now - lastFirstTEvidenceMs > kFirstTEvidenceWindowMs) {
        eventStableCount = 0;
      }
      lastFirstTEvidenceMs = now;
      if (eventStableCount < 255) eventStableCount++;
    } else if (lastFirstTEvidenceMs == 0 ||
               now - lastFirstTEvidenceMs > kFirstTEvidenceWindowMs) {
      eventStableCount = 0;
      lastFirstTEvidenceMs = 0;
    }

    return eventStableCount >= kFirstTStableFrames;
  }

  const bool eventNow = expectedSharpTurnDetected(line, expected);

  if (!eventNow) {
    eventStableCount = 0;
    return false;
  }

  if (eventStableCount < 255) eventStableCount++;
  return eventStableCount >= kSharpTurnStableFrames;
}

/**
 * @return true while first-T evidence is being accumulated.
 */
inline bool firstTConfirmationActive() {
  return routeTurnIndex == 0 &&
         eventStableCount > 0 &&
         lastFirstTEvidenceMs != 0 &&
         millis() - lastFirstTEvidenceMs <= kFirstTEvidenceWindowMs;
}

/**
 * Reacquire the line after a committed 90 degree turn.
 *
 * @param dir Direction of the turn just completed.
 * @return false only when stopped or killed.
 */
inline bool reacquireLineAfterTurn(TurnDir dir) {
  const uint32_t start = millis();
  uint8_t stable = 0;
  const int command = dir == TurnDir::Left ? kReacquireTurnSpeed : -kReacquireTurnSpeed;

  while (millis() - start < kReacquireTimeoutMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    const LineReading line = readLine();
    if (line.detected && centerHasLine()) {
      stable++;
      stopMotors();
      if (stable >= kReacquireStableFrames) {
        resetLineController();
        return true;
      }
    } else {
      stable = 0;
      setTurnCommand(command);
    }
    updateImu();
    delay(10);
  }

  stopMotors();
  resetLineController();
  return true;
}

/**
 * Execute one committed turn for the selected base route.
 *
 * @return true if completed, false if stopped or killed.
 */
inline bool performRouteTurn() {
  if (routeTurnIndex >= kRouteTurnCount) return true;

  const TurnDir dir = routeTurnAt(routeTurnIndex);
  const bool firstT = routeTurnIndex == 0;
  const float advanceMm = firstT ? kFirstTAdvanceMm : kSharpTurnAdvanceMm;

  stopMotors();
  delay(80);

  Serial.print(F("[ROUTE] turnIndex="));
  Serial.print(routeTurnIndex);
  Serial.print(F(" route="));
  Serial.print(routeName(routeChoice));
  Serial.print(F(" dir="));
  Serial.println(turnName(dir));

  if (!driveDistanceMm(advanceMm, kRouteAdvanceSpeed)) return false;
  delay(60);
  if (!turnDegreesImu(degreesForTurn(dir))) return false;
  delay(80);
  if (!reacquireLineAfterTurn(dir)) return false;

  routeTurnIndex++;
  eventStableCount = 0;
  lastFirstTEvidenceMs = 0;
  lastEventMs = millis();
  beginPostTurnHardIgnore();
  return true;
}

// ---------------------------------------------------------------------------
// Wall following
// ---------------------------------------------------------------------------

/**
 * Compute the maximum correction allowed by the fast/slow motor ratio limit.
 *
 * @return Maximum correction magnitude.
 */
inline int maxWallCorrectionFromRatioLimit() {
  if (kMaxFastSlowMotorRatio <= 1.0f || kWallBaseSpeed <= 0) return 0;
  const float maxCorrection =
      kWallBaseSpeed * (kMaxFastSlowMotorRatio - 1.0f) / (kMaxFastSlowMotorRatio + 1.0f);
  return maxCorrection > 0.0f ? static_cast<int>(maxCorrection) : 0;
}

/**
 * Read the selected wall-following side sonar.
 *
 * @param usedFallback Output flag true when last valid reading is reused.
 * @return Wall distance in millimeters, or -1 when no usable reading exists.
 */
inline float selectedWallDistanceMm(bool *usedFallback) {
  const bool useLeft = wallSide == WallSide::Left;
  const uint8_t trig = useLeft ? kLeftTrigPin : kRightTrigPin;
  const uint8_t echo = useLeft ? kLeftEchoPin : kRightEchoPin;
  const float mm = readSonarMm(trig, echo);

  if (isValidSonarDistance(mm)) {
    if (useLeft) lastValidLeftMm = mm;
    else lastValidRightMm = mm;
    *usedFallback = false;
    return mm;
  }

  const float fallback = useLeft ? lastValidLeftMm : lastValidRightMm;
  if (fallback > 0.0f && !kStopIfNoWallEcho) {
    *usedFallback = true;
    return fallback;
  }

  *usedFallback = false;
  return -1.0f;
}

/**
 * Apply one wall-following PID step.
 *
 * @return true when a valid wall command was applied; false when sonar failed.
 */
inline bool runWallFollowStep() {
  bool usedFallback = false;
  const float distanceMm = selectedWallDistanceMm(&usedFallback);
  if (distanceMm < 0.0f) {
    stopMotors();
    Serial.println(F("[WALL] no valid wall distance."));
    delay(80);
    return false;
  }

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
  const int ratioCorrectionLimit = maxWallCorrectionFromRatioLimit();
  int activeCorrectionLimit = kWallMaxCorrection;
  if (ratioCorrectionLimit < activeCorrectionLimit) activeCorrectionLimit = ratioCorrectionLimit;
  const int correction = constrain(static_cast<int>(pid), -activeCorrectionLimit, activeCorrectionLimit);

  // Positive turnLeftCorrection slows the left motor and speeds the right.
  const int turnLeftCorrection = wallSide == WallSide::Left ? correction : -correction;
  const int leftSpeed = kWallBaseSpeed - turnLeftCorrection;
  const int rightSpeed = kWallBaseSpeed + turnLeftCorrection;
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(sideName(wallSide));
    Serial.print(F(" distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(usedFallback ? F(" fallback") : F(""));
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" corr="));
    Serial.print(correction);
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
  }

  updateImu();
  delay(kWallLoopDelayMs);
  return true;
}
