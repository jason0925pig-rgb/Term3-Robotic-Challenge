#pragma once

#include "hard_config.h"

inline void updateFollowBaseRoute() {
  if (routeTurnIndex >= kAirlockRfidCheckFromTurnIndex) {
    checkBaseExitRfidAndRequestAirlock();
  }

  const LineReading line = readLine();
  if (routeEventReady(line)) {
    if (!performRouteTurn()) {
      serialStopped = true;
      setState(MissionState::Stopped);
      return;
    }

    if (routeTurnIndex >= kRouteTurnCount) {
      Serial.print(F("[ROUTE] complete route="));
      Serial.print(routeName(routeChoice));
      Serial.println(F("; following line to tunnel entry."));
      setState(MissionState::FollowLineToTunnelEntry);
      return;
    }
  } else {
    if (firstTConfirmationActive()) {
      setTank(kFirstTConfirmSpeed, kFirstTConfirmSpeed);
      updateImu();
      delay(kLineLoopDelayMs);
      return;
    }

    const LineReading followLine = softenedPostTurnLine(line);
    applyLineCommand(followLine, F("[BASE]"));
    updatePostTurnHardIgnore(line);
  }
}

/**
 * Follow the exit line until it disappears at the tunnel entry.
 *
 * The A door should already be opening from the RFID/server request before the
 * robot arrives, so this segment no longer uses front sonar as a collision
 * stop. The tunnel itself has no line; several all-white QTR frames are used as
 * the handoff point into wall following.
 */
inline void updateFollowLineToTunnelEntry() {
  if (checkBaseExitRfidAndRequestAirlock()) {
    return;
  }

  if (millis() - stateStartMs > kLineToDoorTimeoutMs) {
    stopMotors();
    Serial.println(F("[WARN] line-to-tunnel timeout; starting wall follow from current position."));
    resetWallController();
    setState(MissionState::WallFollowTunnel);
    return;
  }

  const LineReading line = readLine();
  if (!line.detected) {
    if (tunnelEntryNoLineCount < 255) tunnelEntryNoLineCount++;
    if (tunnelEntryNoLineCount >= kTunnelEntryNoLineFrames) {
      stopMotors();
      if (!airlockRequestSent) {
        Serial.println(F("[WARN] tunnel entry reached before base-exit RFID was read; airlock request not sent."));
      }
      Serial.println(F("[TUNNEL] base line ended; starting wall following with initial IMU calibration."));
      resetWallController();
      setState(MissionState::WallFollowTunnel);
      return;
    }

    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  tunnelEntryNoLineCount = 0;
  const LineReading followLine = softenedPostTurnLine(line);
  applyLineCommand(followLine, F("[TO TUNNEL]"));
  updatePostTurnHardIgnore(line);
  checkBaseExitRfidAndRequestAirlock();
}

/**
 * Follow the tunnel wall until the front sonar detects the far door.
 */
inline void updateWallFollowTunnel() {
  if (millis() - stateStartMs > kWallTunnelTimeoutMs) {
    stopMotors();
    Serial.println(F("[WARN] wall-follow tunnel timeout; switching to line/RFID search from current position."));
    setState(MissionState::FollowLineToFirstGridRfid);
    return;
  }

  const LineReading exitLine = readLine();
  if (exitLine.detected) {
    if (wallExitLineStableCount < 255) wallExitLineStableCount++;
    if (wallExitLineStableCount >= kWallExitLineStableFrames) {
      stopMotors();
      Serial.println(F("[TUNNEL EXIT] QTR line found; leaving wall following and following line to first grid RFID."));
      setState(MissionState::FollowLineToFirstGridRfid);
      return;
    }
  } else {
    wallExitLineStableCount = 0;
  }

  const DoorReading door = readDoor();
  const bool closed = doorClosedStable(door);
  printDoorStatus(F("[EXIT DOOR APPROACH]"), door);
  if (closed) {
    stopMotors();
    Serial.println(F("[EXIT DOOR] closed door detected; wall following cancelled."));
    setState(MissionState::WaitExitDoorOpen);
    return;
  }

  runWallFollowStep();
}

/**
 * Stop at the far tunnel door until it opens. If the blockage persists beyond
 * the configured wait time, start the line-based obstacle bypass.
 */
inline void updateWaitExitDoorOpen() {
  stopMotors();
  const DoorReading door = readDoor();
  const bool opened = doorOpenStable(door);
  printDoorStatus(F("[EXIT DOOR WAIT]"), door);

  if (opened) {
    Serial.println(F("[EXIT DOOR] open; following line to first grid RFID when available."));
    setState(MissionState::FollowLineToFirstGridRfid);
    return;
  }

  if (currentStateElapsedMs() >= kExitDoorObstacleWaitMs &&
      door.valid &&
      door.frontMm <= kHardObstacleAheadThresholdMm) {
    beginPersistentObstacleAvoidance();
    return;
  }

  delay(20);
}

/**
 * Drive forward without wall following until the first outside RFID tag is read.
 */
inline void updateSearchFirstRfid() {
  if (driveForwardUntilFirstRfid()) {
    setState(MissionState::AlignOverRfid);
  } else {
    setState(MissionState::Stopped);
  }
}

/**
 * Drive the final alignment offset after RFID detection.
 */
inline void updateAlignOverRfid() {
  if (rfidAlignOffsetMm > 0.0f) {
    driveDistanceMm(rfidAlignOffsetMm, kRfidAlignSpeed);
  }
  stopMotors();
  Serial.println(F("[DONE] aligned over first outside RFID spot."));
  setState(MissionState::Done);
}

/**
 * Top-level mission state machine.
 */
inline void updateMission() {
  handleSerialCommands();
  updateWifi();
  servicePendingAirlockRequest();

  if (handleStartStopButtonEvent()) {
    delay(50);
    return;
  }

  if (serialStopped && missionState != MissionState::Stopped) {
    stopMotors();
    setState(MissionState::Stopped);
    return;
  }

  switch (missionState) {
    case MissionState::Idle:
      stopMotors();
      delay(20);
      break;

    case MissionState::FollowBaseRoute:
      updateFollowBaseRoute();
      break;

    case MissionState::FollowLineToTunnelEntry:
      updateFollowLineToTunnelEntry();
      break;

    case MissionState::WallFollowTunnel:
      updateWallFollowTunnel();
      break;

    case MissionState::WaitExitDoorOpen:
      updateWaitExitDoorOpen();
      break;

    case MissionState::ObstaclePrepareSwerve:
    case MissionState::ObstacleFindAlignmentTag:
    case MissionState::ObstacleDriveRfidCenterOffset:
    case MissionState::ObstacleTurnAway:
    case MissionState::ObstacleMoveSidewaysAroundObstacle:
    case MissionState::ObstacleTurnParallelToOriginal:
    case MissionState::ObstaclePassObstacle:
    case MissionState::ObstacleTurnBackTowardLine:
    case MissionState::ObstacleReturnToOriginalLineOffset:
    case MissionState::ObstacleRestoreOriginalHeading:
    case MissionState::ObstacleFollowLineAfterObstacle:
      updateObstacleAvoidance();
      break;

    case MissionState::SearchFirstRfid:
      updateSearchFirstRfid();
      break;

    case MissionState::AlignOverRfid:
      updateAlignOverRfid();
      break;

    case MissionState::FollowLineToFirstGridRfid:
      updateFollowLineToFirstGridRfid();
      break;

    case MissionState::EasyGridRoute:
      updateEasyGridRoute();
      break;

    case MissionState::EasyDoorRequest:
      updateEasyDoorRequest();
      break;

    case MissionState::EasyAfterDoorForward:
      updateEasyAfterDoorForward();
      break;

    case MissionState::ReturnLineToTunnelEntry:
      updateReturnLineToTunnelEntry();
      break;

    case MissionState::ReturnWallFollowTunnel:
      updateReturnWallFollowTunnel();
      break;

    case MissionState::ReturnFollowLineToBase:
      updateReturnFollowLineToBase();
      break;

    case MissionState::Done:
      stopMotors();
      updateImu();
      delay(50);
      break;

    case MissionState::Stopped:
      stopMotors();
      updateImu();
      delay(50);
      break;
  }
}

// ---------------------------------------------------------------------------
// Setup helpers
// ---------------------------------------------------------------------------

/**
 * Initialize the top Modulino Pixels used for mission status.
 */
inline void initializePixels() {
  Wire.begin();
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  setPixelsNormalBlue();
}


/**
 * Initialize QTR pins and load the saved calibration values.
 */
inline void initializeHardQtr() {
  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
  }
  initializeQtrCalibration();
}

/**
 * Initialize sensors and calibrate IMU only after the robot is placed down.
 *
 * The robot may be hand-carried before the start button is pressed, so QTR,
 * sonar, RFID, and IMU setup are delayed until mission start. Motoron is still
 * initialized in setup so the motors can be commanded to a safe stop.
 *
 * @return true if the mission can start, false if start was cancelled.
 */
inline bool initializeMissionSensorsForRun() {
  stopMotors();

  if (!missionSensorsInitialized) {
    Serial.println(F("[INIT] initializing sensors after start button."));

    pinMode(kFrontTrigPin, OUTPUT);
    pinMode(kLeftTrigPin, OUTPUT);
    pinMode(kRightTrigPin, OUTPUT);
    pinMode(kFrontEchoPin, INPUT);
    pinMode(kLeftEchoPin, INPUT);
    pinMode(kRightEchoPin, INPUT);
    digitalWrite(kFrontTrigPin, LOW);
    digitalWrite(kLeftTrigPin, LOW);
    digitalWrite(kRightTrigPin, LOW);

    pinMode(kServoPin, OUTPUT);
    if (kResetServoAtStartup) {
      moveServoToAngle(kServoMinAngle);
      holdServoAngle(kServoMinAngle, 500);
    }

    pinMode(kLeftEncoderAPin, INPUT_PULLUP);
    pinMode(kLeftEncoderBPin, INPUT_PULLUP);
    pinMode(kRightEncoderAPin, INPUT_PULLUP);
    pinMode(kRightEncoderBPin, INPUT_PULLUP);
    if (!encoderInterruptsAttached) {
      attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
      attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);
      encoderInterruptsAttached = true;
    }

    Wire.begin();
    initializeHardQtr();
    initializeRfid();
    initializeImuHardware();

    missionSensorsInitialized = true;
  }

  if (!waitForWifiBeforeCalibration()) {
    return false;
  }

  resetEncoders();
  resetLineController();
  resetWallController();
  lastEventMs = millis();
  lastWallUpdateMs = millis();

  if (imuOk && !calibrateImuGyroBias()) {
    return false;
  }

  return !serialStopped;
}
