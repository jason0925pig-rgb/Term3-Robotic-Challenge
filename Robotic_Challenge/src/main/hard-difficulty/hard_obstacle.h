#pragma once

#include "hard_config.h"

inline void stopObstacleAvoidance(const __FlashStringHelper *reason) {
  stopMotors();
  serialStopped = true;
  Serial.print(F("[OBSTACLE] stopped: "));
  Serial.println(reason);
  setState(MissionState::Stopped);
}

/**
 * @return milliseconds elapsed in the current mission state.
 */
inline uint32_t currentStateElapsedMs() {
  return millis() - stateStartMs;
}

/**
 * Check generic safety conditions used by obstacle-manoeuvre states.
 */
inline bool obstacleMovementSafetyOk() {
  if (serialStopped || handleStartStopButtonEvent()) {
    stopMotors();
    return false;
  }

  const float frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  if (isValidSonarDistance(frontMm) && frontMm <= kObstacleFrontSafetyStopMm) {
    stopObstacleAvoidance(F("front_safety_stop"));
    return false;
  }

  if (currentStateElapsedMs() > kObstacleMovementTimeoutMs) {
    stopObstacleAvoidance(F("movement_timeout"));
    return false;
  }

  return true;
}

/**
 * Poll an RFID grid node with debounce separate from planting-route RFID state.
 */
inline bool pollObstacleGridNode(String *uidOut) {
  String uid;
  if (!pollRfid(&uid, true)) return false;

  if (uid == lastObstacleNodeUid &&
      millis() - lastObstacleNodeUidMs < kObstacleRfidNodeDebounceMs) {
    return false;
  }

  lastObstacleNodeUid = uid;
  lastObstacleNodeUidMs = millis();
  *uidOut = uid;
  return true;
}

/**
 * Choose the side with more measured space for the initial swerve.
 */
inline void chooseObstacleSwerveDirection(const SonarReading &snapshot) {
  if (snapshot.leftValid && snapshot.rightValid) {
    obstacleSwerveDirection =
        snapshot.leftMm > snapshot.rightMm ? WallSide::Left : WallSide::Right;
  } else if (snapshot.leftValid) {
    obstacleSwerveDirection = WallSide::Left;
  } else if (snapshot.rightValid) {
    obstacleSwerveDirection = WallSide::Right;
  } else {
    obstacleSwerveDirection = kObstacleFallbackSwerveDirection;
  }

  obstacleFacingSide =
      obstacleSwerveDirection == WallSide::Left ? WallSide::Right : WallSide::Left;
}

/**
 * @return +1 for a left swerve and -1 for a right swerve.
 */
inline float obstacleTurnSignForSwerve() {
  return obstacleSwerveDirection == WallSide::Left ? 1.0f : -1.0f;
}

/**
 * Check whether side sonar says the obstacle has been cleared.
 */
inline bool obstacleSideCleared(float currentMm) {
  if (obstacleReferenceSideMm < 0.0f) return false;
  if (currentMm < 0.0f) return true;
  return currentMm > obstacleReferenceSideMm + kObstacleClearedSideDistanceIncreaseMm;
}

/**
 * Check whether side sonar still approximately sees the obstacle beside us.
 */
inline bool obstacleStillBeside(float currentMm) {
  if (currentMm < 0.0f) return false;
  if (absFloat(currentMm - obstacleReferenceSideMm) <= kObstacleSameSideDistanceToleranceMm) {
    return true;
  }
  return currentMm <= obstacleReferenceSideMm + kObstacleClearedSideDistanceIncreaseMm;
}

/**
 * Start line-following until the next RFID node, then encoder-center on it.
 */
inline void beginObstacleFindAndCenterOnNextRfid(MissionState nextState, bool countsAsSidewaysStep) {
  obstacleCenteringNextState = nextState;
  obstacleCenteringCountsAsSidewaysStep = countsAsSidewaysStep;
  resetLineController();
  setState(MissionState::ObstacleFindAlignmentTag);
}

/**
 * Start the encoder centering offset toward a later obstacle-bypass state.
 */
inline void beginObstacleEncoderCenteringToState(MissionState nextState) {
  obstacleCenteringNextState = nextState;
  obstacleCenteringCountsAsSidewaysStep = false;
  resetEncoders();
  setState(MissionState::ObstacleDriveRfidCenterOffset);
}

/**
 * Apply one line-following step without the normal mission RFID handling.
 */
inline void applyObstacleLineFollowWithoutDelay(const __FlashStringHelper *label) {
  const LineReading line = readLine();
  applyLineCommand(line, label);
  updateImu();
}

/**
 * Handle the first RFID alignment search before or during a swerve.
 */
inline void handleObstacleAlignmentTagSearch() {
  if (!obstacleMovementSafetyOk()) return;

  String uid;
  if (pollObstacleGridNode(&uid)) {
    stopMotors();
    resetEncoders();
    Serial.print(F("[OBSTACLE ALIGN] uid="));
    Serial.print(uid);
    Serial.print(F(" offsetMm="));
    Serial.print(kObstacleRfidCenteringForwardOffsetMm, 1);
    Serial.print(F(" next="));
    Serial.println(stateName(obstacleCenteringNextState));
    setState(MissionState::ObstacleDriveRfidCenterOffset);
    return;
  }

  applyObstacleLineFollowWithoutDelay(F("[OBSTACLE ALIGN]"));
  delay(kLineLoopDelayMs);
}

/**
 * Update current side-obstacle distance from a full sonar snapshot.
 */
inline void updateObstacleCurrentSideDistance() {
  const SonarReading snapshot = readSonars();
  obstacleCurrentSideMm = sonarDistanceForSide(obstacleFacingSide, snapshot);
  if (!sonarValidForSide(obstacleFacingSide, snapshot)) {
    obstacleCurrentSideMm = -1.0f;
  }
}

/**
 * Handle the node reached after centering on a sideways step.
 */
inline void handleObstacleSidewaysStepAfterCentering() {
  obstacleGridSpacesAway++;
  updateObstacleCurrentSideDistance();

  Serial.print(F("[OBSTACLE NODE] alignment count="));
  Serial.print(obstacleGridSpacesAway);
  Serial.print(F(" sideMm="));
  Serial.println(obstacleCurrentSideMm, 1);

  if (obstacleSideCleared(obstacleCurrentSideMm)) {
    setState(MissionState::ObstacleTurnParallelToOriginal);
    return;
  }

  if (!obstacleStillBeside(obstacleCurrentSideMm)) {
    Serial.println(F("[OBSTACLE SIDE] ambiguous side distance; continuing cautiously."));
  }

  if (obstacleGridSpacesAway >= kObstacleMaxSidewaysGridSpaces) {
    stopObstacleAvoidance(F("side_clear_not_found"));
    return;
  }

  resetLineController();
  setState(MissionState::ObstacleMoveSidewaysAroundObstacle);
}

/**
 * Drive the fixed RFID-centering offset using encoder balancing.
 */
inline void handleObstacleRfidCenterOffsetDrive() {
  if (!obstacleMovementSafetyOk()) return;

  const long targetCounts = distanceMmToCounts(kObstacleRfidCenteringForwardOffsetMm);
  const long leftAbs = absLong(getLeftCount());
  const long rightAbs = absLong(getRightCount());
  const long averageAbs = (leftAbs + rightAbs) / 2;

  if (averageAbs >= targetCounts) {
    stopMotors();
    Serial.print(F("[OBSTACLE ALIGN] centered counts="));
    Serial.print(averageAbs);
    Serial.print(F("/"));
    Serial.print(targetCounts);
    Serial.print(F(" next="));
    Serial.println(stateName(obstacleCenteringNextState));

    const MissionState nextState = obstacleCenteringNextState;
    const bool countsAsSidewaysStep = obstacleCenteringCountsAsSidewaysStep;
    obstacleCenteringCountsAsSidewaysStep = false;

    if (countsAsSidewaysStep) {
      handleObstacleSidewaysStepAfterCentering();
    } else {
      setState(nextState);
    }
    return;
  }

  const long diff = leftAbs - rightAbs;
  int correction = static_cast<int>(diff * kStraightCorrectionKp);
  correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
  setTank(kObstacleManoeuvreSpeed - correction, kObstacleManoeuvreSpeed + correction);
  updateImu();
  delay(10);
}

/**
 * Line-follow across or alongside the obstacle until a node indicates clearance.
 */
inline void handleObstacleRfidLineFollowMovement(
    uint8_t *counter,
    uint8_t maxCounter,
    MissionState nextState) {
  if (!obstacleMovementSafetyOk()) return;

  applyObstacleLineFollowWithoutDelay(F("[OBSTACLE LINE]"));

  String uid;
  if (!pollObstacleGridNode(&uid)) {
    delay(kLineLoopDelayMs);
    return;
  }

  (*counter)++;
  stopMotors();
  updateObstacleCurrentSideDistance();

  Serial.print(F("[OBSTACLE NODE] uid="));
  Serial.print(uid);
  Serial.print(F(" count="));
  Serial.print(*counter);
  Serial.print(F(" sideMm="));
  Serial.println(obstacleCurrentSideMm, 1);

  if (obstacleSideCleared(obstacleCurrentSideMm)) {
    beginObstacleEncoderCenteringToState(nextState);
    return;
  }

  if (!obstacleStillBeside(obstacleCurrentSideMm)) {
    Serial.println(F("[OBSTACLE SIDE] ambiguous side distance; continuing cautiously."));
  }

  if (*counter >= maxCounter) {
    stopObstacleAvoidance(F("side_clear_not_found"));
  }
}

/**
 * Drive back toward the original line by the number of grid spaces moved away.
 */
inline void handleObstacleReturnToLineOffset() {
  if (!obstacleMovementSafetyOk()) return;

  setTank(kObstacleManoeuvreSpeed, kObstacleManoeuvreSpeed);
  updateImu();

  String uid;
  if (pollObstacleGridNode(&uid)) {
    obstacleReturnGridSpaces++;
    Serial.print(F("[OBSTACLE RETURN] uid="));
    Serial.print(uid);
    Serial.print(F(" count="));
    Serial.print(obstacleReturnGridSpaces);
    Serial.print(F("/"));
    Serial.println(obstacleGridSpacesAway);

    if (obstacleReturnGridSpaces >= obstacleGridSpacesAway) {
      stopMotors();
      beginObstacleEncoderCenteringToState(MissionState::ObstacleRestoreOriginalHeading);
    }
  }

  delay(10);
}

/**
 * Finish the bypass and return to the normal grid-entry state.
 */
inline void finishObstacleAvoidance() {
  stopMotors();
  if (lastObstacleNodeUid.length() > 0) {
    lastGridUid = lastObstacleNodeUid;
    lastGridUidMs = millis();
  }

  Serial.print(F("[OBSTACLE] bypass complete; resuming "));
  Serial.println(stateName(obstacleResumeState));
  setState(obstacleResumeState);
}

/**
 * Follow the line after the obstacle before handing back to the main mission.
 */
inline void handleObstaclePostLineFollow() {
  if (!obstacleMovementSafetyOk()) return;

  applyObstacleLineFollowWithoutDelay(F("[OBSTACLE POST]"));

  String uid;
  if (pollObstacleGridNode(&uid)) {
    obstaclePostRfidCount++;
    Serial.print(F("[OBSTACLE POST] uid="));
    Serial.print(uid);
    Serial.print(F(" count="));
    Serial.print(obstaclePostRfidCount);
    Serial.print(F("/"));
    Serial.println(kObstaclePostNodeTarget);

    if (obstaclePostRfidCount >= kObstaclePostNodeTarget) {
      finishObstacleAvoidance();
      return;
    }
  }

  delay(kLineLoopDelayMs);
}

/**
 * Start the line-based persistent-obstacle manoeuvre.
 */
inline void beginPersistentObstacleAvoidance() {
  stopMotors();
  resetObstacleAvoidanceProgress();
  obstacleResumeState = MissionState::FollowLineToFirstGridRfid;
  Serial.print(F("[OBSTACLE] persistent front blockage for "));
  Serial.print(kExitDoorObstacleWaitMs);
  Serial.println(F(" ms; starting line-based bypass."));
  setState(MissionState::ObstaclePrepareSwerve);
}

/**
 * Run one state-machine step for the line-based obstacle bypass.
 */
inline void updateObstacleAvoidance() {
  switch (missionState) {
    case MissionState::ObstaclePrepareSwerve: {
      stopMotors();
      const SonarReading snapshot = readSonars();
      chooseObstacleSwerveDirection(snapshot);
      Serial.print(F("[OBSTACLE SWERVE] direction="));
      Serial.print(sideName(obstacleSwerveDirection));
      Serial.print(F(" obstacleFacing="));
      Serial.println(sideName(obstacleFacingSide));
      beginObstacleFindAndCenterOnNextRfid(MissionState::ObstacleTurnAway, false);
      break;
    }

    case MissionState::ObstacleFindAlignmentTag:
      handleObstacleAlignmentTagSearch();
      break;

    case MissionState::ObstacleDriveRfidCenterOffset:
      handleObstacleRfidCenterOffsetDrive();
      break;

    case MissionState::ObstacleTurnAway: {
      const float turnDeg = 90.0f * obstacleTurnSignForSwerve();
      if (!turnDegreesImu(turnDeg)) {
        stopObstacleAvoidance(F("turn_away_failed"));
        break;
      }

      obstacleHeadingOffsetDeg += turnDeg;
      updateObstacleCurrentSideDistance();
      obstacleReferenceSideMm = obstacleCurrentSideMm;
      obstacleGridSpacesAway = 0;
      beginObstacleFindAndCenterOnNextRfid(
          MissionState::ObstacleMoveSidewaysAroundObstacle,
          true);
      break;
    }

    case MissionState::ObstacleMoveSidewaysAroundObstacle:
      handleObstacleRfidLineFollowMovement(
          &obstacleGridSpacesAway,
          kObstacleMaxSidewaysGridSpaces,
          MissionState::ObstacleTurnParallelToOriginal);
      break;

    case MissionState::ObstacleTurnParallelToOriginal: {
      const float turnDeg = -90.0f * obstacleTurnSignForSwerve();
      if (!turnDegreesImu(turnDeg)) {
        stopObstacleAvoidance(F("turn_parallel_failed"));
        break;
      }

      obstacleHeadingOffsetDeg += turnDeg;
      obstacleGridSpacesPassing = 0;
      resetLineController();
      setState(MissionState::ObstaclePassObstacle);
      break;
    }

    case MissionState::ObstaclePassObstacle:
      handleObstacleRfidLineFollowMovement(
          &obstacleGridSpacesPassing,
          kObstacleMaxPassingGridSpaces,
          MissionState::ObstacleTurnBackTowardLine);
      break;

    case MissionState::ObstacleTurnBackTowardLine: {
      const float turnDeg = -90.0f * obstacleTurnSignForSwerve();
      if (!turnDegreesImu(turnDeg)) {
        stopObstacleAvoidance(F("turn_back_to_line_failed"));
        break;
      }

      obstacleHeadingOffsetDeg += turnDeg;
      obstacleReturnGridSpaces = 0;
      setState(MissionState::ObstacleReturnToOriginalLineOffset);
      break;
    }

    case MissionState::ObstacleReturnToOriginalLineOffset:
      handleObstacleReturnToLineOffset();
      break;

    case MissionState::ObstacleRestoreOriginalHeading: {
      const float restoreTurnDeg = -obstacleHeadingOffsetDeg;
      if (absFloat(restoreTurnDeg) > kTurnToleranceDeg &&
          !turnDegreesImu(restoreTurnDeg)) {
        stopObstacleAvoidance(F("restore_heading_failed"));
        break;
      }

      obstacleHeadingOffsetDeg = 0.0f;
      obstaclePostRfidCount = 0;
      resetLineController();
      setState(MissionState::ObstacleFollowLineAfterObstacle);
      break;
    }

    case MissionState::ObstacleFollowLineAfterObstacle:
      handleObstaclePostLineFollow();
      break;

    default:
      break;
  }
}

/**
 * After requesting return airlock B, follow the line toward the return tunnel.
 *
 * Handoff to wall following happens when the QTR line disappears for six
 * consecutive frames, or after the same 12 second timeout used on base exit.
 */
inline void updateReturnLineToTunnelEntry() {
  if (millis() - stateStartMs > kLineToDoorTimeoutMs) {
    stopMotors();
    Serial.println(F("[RETURN] line-to-tunnel timeout after B request; starting return wall follow."));
    resetWallController();
    setState(MissionState::ReturnWallFollowTunnel);
    return;
  }

  const LineReading line = readLine();
  if (!line.detected) {
    if (returnTunnelNoLineCount < 255) returnTunnelNoLineCount++;
    if (returnTunnelNoLineCount >= kTunnelEntryNoLineFrames) {
      stopMotors();
      Serial.println(F("[RETURN] line disappeared for 6 frames; starting return wall follow."));
      resetWallController();
      setState(MissionState::ReturnWallFollowTunnel);
      return;
    }

    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  returnTunnelNoLineCount = 0;
  applyLineCommand(line, F("[RETURN TO TUNNEL]"));
  delay(kLineLoopDelayMs);
}

/**
 * Wall-follow back through the return tunnel.
 *
 * Unlike the outbound tunnel state, this does not go to RFID search when the
 * line reappears. It immediately resumes normal line following back to base.
 */
inline void updateReturnWallFollowTunnel() {
  const LineReading line = readLine();
  if (line.detected) {
    if (returnWallExitLineStableCount < 255) returnWallExitLineStableCount++;
    if (returnWallExitLineStableCount >= kReturnWallExitLineStableFrames) {
      stopMotors();
      Serial.println(F("[RETURN] QTR line found; leaving wall following and following line back to base."));
      setState(MissionState::ReturnFollowLineToBase);
      return;
    }
  } else {
    returnWallExitLineStableCount = 0;
  }

  runWallFollowStep();
}

/**
 * Follow the rediscovered line back toward base.
 */
inline void updateReturnFollowLineToBase() {
  const LineReading line = readLine();
  applyLineCommand(line, F("[RETURN BASE]"));
  delay(kLineLoopDelayMs);
}
