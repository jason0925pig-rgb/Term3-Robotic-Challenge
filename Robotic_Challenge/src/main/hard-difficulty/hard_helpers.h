#pragma once

#include "hard_config.h"

inline bool killButtonPressedEvent() {
  if (!kUseKillPin) return false;

  const bool reading = digitalRead(kKillPin);
  if (reading != lastKillReading) {
    lastKillReading = reading;
    lastKillChangeMs = millis();
  }

  if (millis() - lastKillChangeMs < kKillDebounceMs) return false;
  if (reading == stableKillReading) return false;

  stableKillReading = reading;
  return stableKillReading == LOW;
}

/**
 * Convert mission state to a flash-stored diagnostic label.
 *
 * @param state Mission state to describe.
 * @return Printable state label.
 */
inline const __FlashStringHelper *stateName(MissionState state) {
  switch (state) {
    case MissionState::Idle: return F("IDLE");
    case MissionState::FollowBaseRoute: return F("FOLLOW_BASE_ROUTE");
    case MissionState::FollowLineToTunnelEntry: return F("FOLLOW_LINE_TO_TUNNEL_ENTRY");
    case MissionState::WallFollowTunnel: return F("WALL_FOLLOW_TUNNEL");
    case MissionState::WaitExitDoorOpen: return F("WAIT_EXIT_DOOR_OPEN");
    case MissionState::ObstaclePrepareSwerve: return F("OBSTACLE_PREPARE_SWERVE");
    case MissionState::ObstacleFindAlignmentTag: return F("OBSTACLE_FIND_ALIGNMENT_TAG");
    case MissionState::ObstacleDriveRfidCenterOffset: return F("OBSTACLE_DRIVE_RFID_CENTER_OFFSET");
    case MissionState::ObstacleTurnAway: return F("OBSTACLE_TURN_AWAY");
    case MissionState::ObstacleMoveSidewaysAroundObstacle: return F("OBSTACLE_MOVE_SIDEWAYS_AROUND");
    case MissionState::ObstacleTurnParallelToOriginal: return F("OBSTACLE_TURN_PARALLEL");
    case MissionState::ObstaclePassObstacle: return F("OBSTACLE_PASS");
    case MissionState::ObstacleTurnBackTowardLine: return F("OBSTACLE_TURN_BACK_TO_LINE");
    case MissionState::ObstacleReturnToOriginalLineOffset: return F("OBSTACLE_RETURN_TO_LINE_OFFSET");
    case MissionState::ObstacleRestoreOriginalHeading: return F("OBSTACLE_RESTORE_HEADING");
    case MissionState::ObstacleFollowLineAfterObstacle: return F("OBSTACLE_FOLLOW_LINE_AFTER");
    case MissionState::SearchFirstRfid: return F("SEARCH_FIRST_RFID");
    case MissionState::AlignOverRfid: return F("ALIGN_OVER_RFID");
    case MissionState::FollowLineToFirstGridRfid: return F("FOLLOW_LINE_TO_FIRST_GRID_RFID");
    case MissionState::EasyGridRoute: return F("EASY_GRID_ROUTE");
    case MissionState::EasyDoorRequest: return F("EASY_DOOR_REQUEST");
    case MissionState::EasyAfterDoorForward: return F("EASY_AFTER_DOOR_FORWARD");
    case MissionState::ReturnLineToTunnelEntry: return F("RETURN_LINE_TO_TUNNEL_ENTRY");
    case MissionState::ReturnWallFollowTunnel: return F("RETURN_WALL_FOLLOW_TUNNEL");
    case MissionState::ReturnFollowLineToBase: return F("RETURN_FOLLOW_LINE_TO_BASE");
    case MissionState::Done: return F("DONE");
    case MissionState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

/**
 * Convert turn direction to a flash-stored label.
 *
 * @param dir Turn direction.
 * @return "LEFT" or "RIGHT".
 */
inline const __FlashStringHelper *turnName(TurnDir dir) {
  return dir == TurnDir::Left ? F("LEFT") : F("RIGHT");
}

/**
 * Convert wall side to a flash-stored label.
 *
 * @param side Selected wall-following side.
 * @return "LEFT" or "RIGHT".
 */
inline const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

/**
 * Convert route choice to a flash-stored label.
 *
 * @param route Selected base route.
 * @return "A_RIGHT" or "B_LEFT".
 */
inline const __FlashStringHelper *routeName(RouteChoice route) {
  return route == RouteChoice::BaseA_Bottom ? F("A_RIGHT") : F("B_LEFT");
}

/**
 * Reset the line-following controller memory.
 *
 * This prevents stale integral/derivative values from affecting a new segment.
 */
inline void resetLineController() {
  lineIntegral = 0.0f;
  lastLineError = 0;
}

/**
 * Check whether a line-follow mode is an aggressive hard-turn correction.
 *
 * @param mode Current line-follow mode.
 * @return true for hard-left or hard-right modes.
 */
inline bool isHardLineMode(FollowMode mode) {
  return mode == FollowMode::HardLeft || mode == FollowMode::HardRight;
}

/**
 * Start the short ignore window after a scripted 90 degree turn.
 *
 * This prevents an imperfect 90 degree turn from being immediately interpreted
 * as the next hard-left/right route event.
 */
inline void beginPostTurnHardIgnore() {
  postTurnHardIgnoreActive = true;
  postTurnHardIgnoreStartMs = millis();
  postTurnHardReleaseCount = 0;
}

/**
 * Measure travel from the current route segment start.
 *
 * Before the first turn, encoders are reset at mission start, so this is the
 * distance from start toward the first T.
 *
 * @return Average wheel travel in millimeters.
 */
inline float currentRouteSegmentTravelMm() {
  const long leftAbs = absLong(getLeftCount());
  const long rightAbs = absLong(getRightCount());
  const long averageCounts = (leftAbs + rightAbs) / 2;
  return averageCounts / (kEncoderCountsPerMm * kDistanceCalibration);
}

/**
 * Update the post-turn ignore state from the latest line reading.
 *
 * @param line Latest QTR line reading.
 */
inline void updatePostTurnHardIgnore(const LineReading &line) {
  if (!postTurnHardIgnoreActive) return;

  if (millis() - postTurnHardIgnoreStartMs >= kPostTurnHardIgnoreMs) {
    postTurnHardIgnoreActive = false;
    postTurnHardReleaseCount = 0;
    return;
  }

  if (isHardLineMode(line.mode)) {
    postTurnHardReleaseCount = 0;
    return;
  }

  if (line.detected && centerHasLine()) {
    if (postTurnHardReleaseCount < 255) postTurnHardReleaseCount++;
    if (postTurnHardReleaseCount >= kPostTurnHardReleaseFrames) {
      postTurnHardIgnoreActive = false;
      postTurnHardReleaseCount = 0;
    }
  } else {
    postTurnHardReleaseCount = 0;
  }
}

/**
 * Build a softened line reading while post-turn hard modes are being ignored.
 *
 * @param line Original QTR line reading.
 * @return Original reading, or a copy forced into normal follow mode.
 */
inline LineReading softenedPostTurnLine(LineReading line) {
  if (postTurnHardIgnoreActive && isHardLineMode(line.mode)) {
    line.mode = FollowMode::Follow;
    line.error = constrain(line.error, -kPostTurnSoftErrorClamp, kPostTurnSoftErrorClamp);
  }
  return line;
}

/**
 * Reset the wall-following controller memory.
 *
 * This is called before starting tunnel wall following.
 */
inline void resetWallController() {
  wallIntegral = 0.0f;
  lastWallErrorMm = 0.0f;
  lastWallUpdateMs = millis();
}

/**
 * Clear door hysteresis counters.
 *
 * Door closed/open checks use stable-frame counters, so each new door wait
 * state should start from a clean counter.
 */
inline void resetDoorCounters() {
  doorClosedStableCount = 0;
  doorOpenStableCount = 0;
}

/**
 * Clear line-based obstacle-avoidance progress.
 */
inline void resetObstacleAvoidanceProgress() {
  obstacleResumeState = MissionState::FollowLineToFirstGridRfid;
  obstacleCenteringNextState = MissionState::ObstacleTurnAway;
  obstacleSwerveDirection = kObstacleFallbackSwerveDirection;
  obstacleFacingSide = WallSide::Right;
  obstacleHeadingOffsetDeg = 0.0f;
  obstacleReferenceSideMm = -1.0f;
  obstacleCurrentSideMm = -1.0f;
  obstacleGridSpacesAway = 0;
  obstacleGridSpacesPassing = 0;
  obstacleReturnGridSpaces = 0;
  obstaclePostRfidCount = 0;
  obstacleCenteringCountsAsSidewaysStep = false;
  lastObstacleNodeUid = "";
  lastObstacleNodeUidMs = 0;
}

/**
 * Set all eight Modulino Pixels to one color.
 *
 * @param color Modulino color constant.
 * @param mode Logical mode used to avoid redundant I2C writes.
 */
inline void setAllPixels(ModulinoColor color, PixelMode mode) {
  if (!pixelsOk) return;
  if (currentPixelMode == mode) return;

  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kPixelBrightness);
  }
  pixels.show();
  currentPixelMode = mode;
}

/**
 * Blue means the robot is normal but not actively executing the base-exit run.
 */
inline void setPixelsNormalBlue() {
  setAllPixels(BLUE, PixelMode::NormalBlue);
}

/**
 * Purple/Violet means the robot is starting, calibrating, or actively running.
 */
inline void setPixelsRunningPurple() {
  setAllPixels(VIOLET, PixelMode::RunningPurple);
}

/**
 * Keep the Modulino Pixels synchronized with mission state.
 *
 * Idle/Stopped/Done are blue. All active run states are purple.
 *
 * @param state Current mission state.
 */
inline void updatePixelsForState(MissionState state) {
  if (state == MissionState::Idle ||
      state == MissionState::Stopped ||
      state == MissionState::Done) {
    setPixelsNormalBlue();
  } else {
    setPixelsRunningPurple();
  }
}

/**
 * Change mission state and record the transition time.
 *
 * @param newState State to enter immediately.
 */
inline void setState(MissionState newState) {
  missionState = newState;
  stateStartMs = millis();
  resetDoorCounters();
  updatePixelsForState(newState);
  if (newState == MissionState::FollowLineToTunnelEntry) {
    tunnelEntryNoLineCount = 0;
  }
  if (newState == MissionState::WallFollowTunnel) {
    wallExitLineStableCount = 0;
  }
  if (newState == MissionState::ReturnLineToTunnelEntry) {
    returnTunnelNoLineCount = 0;
    resetLineController();
  }
  if (newState == MissionState::ReturnWallFollowTunnel) {
    returnWallExitLineStableCount = 0;
    resetWallController();
  }
  if (newState == MissionState::FollowLineToFirstGridRfid ||
      newState == MissionState::EasyGridRoute ||
      newState == MissionState::EasyAfterDoorForward ||
      newState == MissionState::ReturnFollowLineToBase) {
    resetLineController();
  }
  Serial.print(F("[STATE] "));
  Serial.println(stateName(newState));
}

/**
 * Convert a 0-300 degree servo target to a 50 Hz pulse width.
 *
 * @param angle Servo angle in degrees.
 * @return Pulse width in microseconds.
 */
inline int angleToPulseUs(int angle) {
  angle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  return map(angle, kServoMinAngle, kServoMaxAngle, kServoMinUs, kServoMaxUs);
}

/**
 * Send one blocking servo PWM frame.
 *
 * @param pulseUs High pulse width in microseconds.
 */
inline void sendServoPulse(int pulseUs) {
  digitalWrite(kServoPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(kServoPin, LOW);
  delayMicroseconds(kServoFrameUs - pulseUs);
}

/**
 * Hold a servo angle by repeatedly sending PWM frames.
 *
 * @param angle Servo angle in degrees.
 * @param durationMs Duration to keep sending the command.
 */
inline void holdServoAngle(int angle, uint32_t durationMs) {
  const int pulseUs = angleToPulseUs(angle);
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    sendServoPulse(pulseUs);
  }
}

/**
 * Move the seed-release servo to an absolute angle.
 *
 * @param angle Target servo angle.
 */
inline void moveServoToAngle(int angle) {
  currentServoAngle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  Serial.print(F("[SERVO] angle="));
  Serial.print(currentServoAngle);
  Serial.print(F(" pulseUs="));
  Serial.println(angleToPulseUs(currentServoAngle));
  holdServoAngle(currentServoAngle, kServoMoveSettleMs);
}

/**
 * Drop one seed by stepping the 300 degree servo by 60 degrees.
 */
inline void dropOneSeed() {
  int nextAngle = currentServoAngle + kServoStepAngle;
  if (nextAngle > kServoMaxAngle) {
    Serial.println(F("[SERVO] 300deg reached; reset to 0 before next drop."));
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 400);
    nextAngle = kServoStepAngle;
  }

  Serial.println(F("[PLANT] drop one seed."));
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);
}

inline uint8_t activeSensorRangeCount(uint8_t first, uint8_t last, uint16_t threshold) {
  if (first > 8) first = 8;
  if (last > 8) last = 8;
  if (first > last) return 0;

  uint8_t count = 0;
  for (uint8_t i = first; i <= last; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

/**
 * Sum normalized black strength across the full QTR array.
 *
 * @return Total normalized black strength across all 9 sensors.
 */
inline uint16_t qtrTotalStrength() {
  uint16_t total = 0;
  for (uint8_t i = 0; i < 9; i++) {
    total += qtrNorm[i];
  }
  return total;
}


inline bool leftOuterHasStrongLine() {
  return qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
}

/**
 * Check whether the right outer sensors see a strong line.
 *
 * @return true when sensor 7 or 8 is strongly active.
 */
inline bool rightOuterHasStrongLine() {
  return qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;
}


inline void applyLineCommand(const LineReading &line, const __FlashStringHelper *label) {
  const MotorCommand cmd = computeLineMotorCommand(line);

  if (line.mode == FollowMode::Stopped) {
    stopMotors();
  } else {
    setTank(cmd.left, cmd.right);
  }

  if (millis() - lastLinePrintMs >= kLinePrintIntervalMs) {
    lastLinePrintMs = millis();
    Serial.print(label);
    Serial.print(F(" state="));
    Serial.print(stateName(missionState));
    Serial.print(F(" turn="));
    Serial.print(routeTurnIndex);
    Serial.print(F("/"));
    Serial.print(kRouteTurnCount);
    Serial.print(F(" mode="));
    Serial.print(followModeName(line.mode));
    Serial.print(F(" active="));
    Serial.print(line.activeCount);
    Serial.print(F(" err="));
    Serial.print(line.error);
    Serial.print(F(" L="));
    Serial.print(cmd.left);
    Serial.print(F(" R="));
    Serial.println(cmd.right);
  }

  updateImu();
  delay(kLineLoopDelayMs);
}
