#include "line_based.h"

#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// obstacle_avoidance
// Test 7 high-level state machine.
//
// This file intentionally owns only the Test 7 mission logic. It reuses the
// primitive headers for motor control, encoders, line following, sonar, and IMU
// turns.
// ============================================================================

#include "../constants.h"
#include "../types.h"
#include "../globals.h"
#include "../declarations.h"
#include "../basic_utils.h"
#include "../motor_utils.h"
#include "../encoder_utils.h"
#include "../line_following_utils.h"
#include "../sonar_wall_utils.h"
#include "../imu_turn_utils.h"


// ============================================================================
// Run tuning variables
// ============================================================================
//
// These are the values to tune during robot runs. They are deliberately mutable
// so they can be adjusted with Serial Monitor commands without recompiling.

float obstacleAheadThresholdMm = 200.0f;        // First obstacle detection while line following.
float frontSafetyStopMm = 20.0f;                // Emergency front stop while manoeuvring.
float sameSideDistanceToleranceMm = 35.0f;      // "Current ~= reference" side-distance band.
float clearedSideDistanceIncreaseMm = 90.0f;    // "Wall gone" when side distance grows by this much.
float rfidCenteringForwardOffsetMm = 220.0f;    // Drive after RFID detection to center robot on tag.
int manoeuvreSpeed = 260;                       // Slow controlled speed for off-line segments.
uint32_t movementTimeoutMs = 12000;             // Per moving state failsafe.
uint32_t rfidNodeDebounceMs = 900;              // Prevent double counting one tag.
uint32_t debugPrintIntervalMs = 500;              // 0 = print every loop, otherwise throttle.
uint8_t postObstacleNodeTarget = 3;             // Required nodes after returning to line.
uint8_t maxSidewaysGridSpaces = 4;              // Failsafe for one-node obstacle.
uint8_t maxPassingGridSpaces = 4;               // Failsafe for one-node obstacle.
WallSide fallbackSwerveDirection = WallSide::Left;

constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 80;
constexpr uint32_t kKillToggleDebounceMs = 300;

enum class Test7State {
  FollowLine,
  PrepareSwerve,
  FindAlignmentTag,
  DriveRfidCenterOffset,
  TurnAway,
  MoveSidewaysAroundObstacle,
  TurnParallelToOriginal,
  PassObstacle,
  TurnBackTowardLine,
  ReturnToOriginalLineOffset,
  RestoreOriginalHeading,
  FollowLineAfterObstacle,
  Complete,
  SafeStop
};

MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);

Test7State test7State = Test7State::FollowLine;
Test7State resumeAfterKillState = Test7State::FollowLine;
Test7State rfidCenteringNextState = Test7State::TurnAway;
uint32_t stateStartedMs = 0;
uint32_t lastDebugPrintMs = 0;
uint32_t lastKillToggleMs = 0;

bool rfidOk = false;
uint32_t lastRfidPollMs = 0;
uint32_t lastAcceptedRfidMs = 0;
String lastAcceptedUid;
String lastSeenUid;

WallSide swerveDirection = WallSide::Left;
WallSide obstacleFacingSide = WallSide::Right;
float originalHeadingDeg = 0.0f;
float headingOffsetFromOriginalDeg = 0.0f;
float referenceObstacleSideMm = -1.0f;
float currentObstacleSideMm = -1.0f;

uint8_t gridSpacesAway = 0;
uint8_t gridSpacesPassing = 0;
uint8_t returnGridSpaces = 0;
uint8_t postObstacleRfidCount = 0;

SonarReading latestSonar;
LineReading latestLine;
String stopReason = "none";
bool completeAnnounced = false;
bool killPaused = false;
bool killButtonWasPressed = false;
bool rfidCenteringCountsAsSidewaysStep = false;

const __FlashStringHelper *stateName(Test7State state) {
  switch (state) {
    case Test7State::FollowLine: return F("FOLLOW_LINE");
    case Test7State::PrepareSwerve: return F("PREPARE_SWERVE");
    case Test7State::FindAlignmentTag: return F("FIND_ALIGNMENT_TAG");
    case Test7State::DriveRfidCenterOffset: return F("DRIVE_RFID_CENTER_OFFSET");
    case Test7State::TurnAway: return F("TURN_AWAY");
    case Test7State::MoveSidewaysAroundObstacle: return F("MOVE_SIDEWAYS_AROUND_OBSTACLE");
    case Test7State::TurnParallelToOriginal: return F("TURN_PARALLEL_TO_ORIGINAL");
    case Test7State::PassObstacle: return F("PASS_OBSTACLE");
    case Test7State::TurnBackTowardLine: return F("TURN_BACK_TOWARD_LINE");
    case Test7State::ReturnToOriginalLineOffset: return F("RETURN_TO_ORIGINAL_LINE_OFFSET");
    case Test7State::RestoreOriginalHeading: return F("RESTORE_ORIGINAL_HEADING");
    case Test7State::FollowLineAfterObstacle: return F("FOLLOW_LINE_AFTER_OBSTACLE");
    case Test7State::Complete: return F("COMPLETE");
    case Test7State::SafeStop: return F("SAFE_STOP");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

void transitionTo(Test7State nextState) {
  test7State = nextState;
  stateStartedMs = millis();
}

uint32_t stateElapsedMs() {
  return millis() - stateStartedMs;
}

float turnSignForSwerve() {
  return swerveDirection == WallSide::Left ? 1.0f : -1.0f;
}

float sonarDistanceForSide(WallSide side, const SonarReading &sonar) {
  return side == WallSide::Left ? sonar.leftMm : sonar.rightMm;
}

bool sonarValidForSide(WallSide side, const SonarReading &sonar) {
  return side == WallSide::Left ? sonar.leftValid : sonar.rightValid;
}

void updateLatestSensors() {
  latestSonar = readSonars();
  updateImu();

  currentObstacleSideMm = sonarDistanceForSide(obstacleFacingSide, latestSonar);
  if (!sonarValidForSide(obstacleFacingSide, latestSonar)) {
    currentObstacleSideMm = -1.0f;
  }
}

void setSafeStop(const String &reason) {
  stopReason = reason;
  stopMotors();
  transitionTo(Test7State::SafeStop);
}

void resetLineControllerHistory() {
  lineIntegral = 0.0f;
  lastLineError = 0;
  lastSeenLineError = 0;
}

bool handleMechanicalKillSwitch() {
  const bool pressed = killPressed();
  const uint32_t now = millis();

  if (pressed && !killButtonWasPressed && now - lastKillToggleMs >= kKillToggleDebounceMs) {
    lastKillToggleMs = now;

    if (killPaused) {
      killPaused = false;
      serialStopped = false;
      stopReason = "none";
      stopMotors();
      transitionTo(resumeAfterKillState);

      Serial.print(F("[KILL] resumed state="));
      Serial.println(stateName(test7State));
    } else if (test7State != Test7State::Complete && test7State != Test7State::SafeStop) {
      resumeAfterKillState = test7State;
      killPaused = true;
      serialStopped = true;
      stopReason = "kill_pause";
      stopMotors();
      transitionTo(Test7State::SafeStop);

      Serial.print(F("[KILL] paused; resumeState="));
      Serial.println(stateName(resumeAfterKillState));
    }
  }

  killButtonWasPressed = pressed;

  if (pressed || killPaused) {
    stopMotors();
    return true;
  }

  return false;
}

bool movementSafetyOk() {
  if (killPressed() || serialStopped) {
    setSafeStop("kill_or_serial_stop");
    return false;
  }

  if (latestSonar.frontValid && latestSonar.frontMm <= frontSafetyStopMm) {
    setSafeStop("front_safety_stop");
    return false;
  }

  if (stateElapsedMs() > movementTimeoutMs) {
    setSafeStop("movement_timeout");
    return false;
  }

  return true;
}

void initializeRfid() {
  pinMode(kRfidResetPin, OUTPUT);
  digitalWrite(kRfidResetPin, HIGH);
  rfid.PCD_Init();
  const byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
  rfidOk = !(version == 0x00 || version == 0xFF);
  Serial.print(F("[RFID] version=0x"));
  Serial.print(version, HEX);
  Serial.print(F(" status="));
  Serial.println(rfidOk ? F("OK") : F("NOT FOUND"));
}

String rfidUidToString() {
  String uid;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

bool pollRfid(String *uidOut) {
  if (!rfidOk) return false;
  if (millis() - lastRfidPollMs < kRfidPollIntervalMs) return false;
  lastRfidPollMs = millis();

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    *uidOut = rfidUidToString();
    rfid.PICC_HaltA();
    return true;
  }

  return false;
}

bool pollGridNode(String *uidOut) {
  String uid;
  if (!pollRfid(&uid)) return false;

  lastSeenUid = uid;
  if (uid == lastAcceptedUid && millis() - lastAcceptedRfidMs < rfidNodeDebounceMs) {
    return false;
  }

  lastAcceptedUid = uid;
  lastAcceptedRfidMs = millis();
  *uidOut = uid;
  return true;
}

void chooseSwerveDirection() {
  const bool leftValid = latestSonar.leftValid;
  const bool rightValid = latestSonar.rightValid;

  if (leftValid && rightValid) {
    swerveDirection = latestSonar.leftMm > latestSonar.rightMm ? WallSide::Left : WallSide::Right;
  } else if (leftValid) {
    swerveDirection = WallSide::Left;
  } else if (rightValid) {
    swerveDirection = WallSide::Right;
  } else {
    swerveDirection = fallbackSwerveDirection;
  }

  obstacleFacingSide = swerveDirection == WallSide::Left ? WallSide::Right : WallSide::Left;
}

bool obstacleSideCleared(float currentMm) {
  if (referenceObstacleSideMm < 0.0f) return false;
  if (currentMm < 0.0f) return true; // No echo usually means the obstacle is out of side range.
  return currentMm > referenceObstacleSideMm + clearedSideDistanceIncreaseMm;
}

bool obstacleStillBeside(float currentMm) {
  if (currentMm < 0.0f) return false;
  if (absFloat(currentMm - referenceObstacleSideMm) <= sameSideDistanceToleranceMm) return true;
  return currentMm <= referenceObstacleSideMm + clearedSideDistanceIncreaseMm;
}

void beginFindAndCenterOnNextRfid(Test7State nextState, bool countsAsSidewaysStep) {
  rfidCenteringNextState = nextState;
  rfidCenteringCountsAsSidewaysStep = countsAsSidewaysStep;
  transitionTo(Test7State::FindAlignmentTag);
}

void beginEncoderCenteringToState(Test7State nextState) {
  rfidCenteringNextState = nextState;
  rfidCenteringCountsAsSidewaysStep = false;
  resetEncoders();
  transitionTo(Test7State::DriveRfidCenterOffset);
}

void driveForwardSlow() {
  setTank(manoeuvreSpeed, manoeuvreSpeed);
}

void applyLineFollowWithoutDelay() {
  latestLine = readLine();
  const MotorCommand cmd = computeLineMotorCommand(latestLine);
  if (latestLine.mode == FollowMode::Stopped) stopMotors();
  else setTank(cmd.left, cmd.right);
}

void handleAlignmentTagSearch() {
  if (!movementSafetyOk()) return;

  String uid;
  if (pollGridNode(&uid)) {
    stopMotors();
    resetEncoders();
    Serial.print(F("[ALIGN] uid="));
    Serial.print(uid);
    Serial.print(F(" offsetMm="));
    Serial.print(rfidCenteringForwardOffsetMm, 1);
    Serial.print(F(" next="));
    Serial.println(stateName(rfidCenteringNextState));
    transitionTo(Test7State::DriveRfidCenterOffset);
    return;
  }

  applyLineFollowWithoutDelay();
}

void handleSidewaysStepAfterCentering() {
  gridSpacesAway++;
  latestSonar = readSonars();
  currentObstacleSideMm = sonarDistanceForSide(obstacleFacingSide, latestSonar);
  if (!sonarValidForSide(obstacleFacingSide, latestSonar)) {
    currentObstacleSideMm = -1.0f;
  }

  Serial.print(F("[NODE] alignment count="));
  Serial.print(gridSpacesAway);
  Serial.print(F(" sideMm="));
  Serial.println(currentObstacleSideMm, 1);

  if (obstacleSideCleared(currentObstacleSideMm)) {
    transitionTo(Test7State::TurnParallelToOriginal);
    return;
  }

  if (!obstacleStillBeside(currentObstacleSideMm)) {
    Serial.println(F("[SIDE] ambiguous side distance; continuing cautiously."));
  }

  if (gridSpacesAway >= maxSidewaysGridSpaces) {
    setSafeStop("side_clear_not_found");
    return;
  }

  transitionTo(Test7State::MoveSidewaysAroundObstacle);
}

void handleRfidCenterOffsetDrive() {
  if (!movementSafetyOk()) return;

  const long targetCounts = distanceMmToCounts(rfidCenteringForwardOffsetMm);
  const long leftAbs = absLong(getLeftCount());
  const long rightAbs = absLong(getRightCount());
  const long averageAbs = (leftAbs + rightAbs) / 2;

  if (averageAbs >= targetCounts) {
    stopMotors();
    Serial.print(F("[ALIGN] centered by encoder counts="));
    Serial.print(averageAbs);
    Serial.print(F("/"));
    Serial.print(targetCounts);
    Serial.print(F(" next="));
    Serial.println(stateName(rfidCenteringNextState));

    const Test7State nextState = rfidCenteringNextState;
    const bool countsAsSidewaysStep = rfidCenteringCountsAsSidewaysStep;
    rfidCenteringCountsAsSidewaysStep = false;

    if (countsAsSidewaysStep) {
      handleSidewaysStepAfterCentering();
    } else {
      transitionTo(nextState);
    }
    return;
  }

  const long diff = leftAbs - rightAbs;
  int correction = static_cast<int>(diff * kStraightCorrectionKp);
  correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);

  const int base = manoeuvreSpeed;
  setTank(base - correction, base + correction);
}

void handleRfidLineFollowMovement(uint8_t *counter, uint8_t maxCounter, Test7State nextState) {
  if (!movementSafetyOk()) return;

  applyLineFollowWithoutDelay();

  String uid;
  if (!pollGridNode(&uid)) return;

  (*counter)++;
  stopMotors();
  latestSonar = readSonars();
  currentObstacleSideMm = sonarDistanceForSide(obstacleFacingSide, latestSonar);
  if (!sonarValidForSide(obstacleFacingSide, latestSonar)) {
    currentObstacleSideMm = -1.0f;
  }

  Serial.print(F("[NODE] uid="));
  Serial.print(uid);
  Serial.print(F(" count="));
  Serial.print(*counter);
  Serial.print(F(" sideMm="));
  Serial.println(currentObstacleSideMm, 1);

  if (obstacleSideCleared(currentObstacleSideMm)) {
    beginEncoderCenteringToState(nextState);
    return;
  }

  if (!obstacleStillBeside(currentObstacleSideMm)) {
    Serial.println(F("[SIDE] ambiguous side distance; continuing cautiously."));
  }

  if (*counter >= maxCounter) {
    setSafeStop("side_clear_not_found");
  }
}

void handleReturnToLineOffset() {
  if (!movementSafetyOk()) return;

  driveForwardSlow();

  String uid;
  if (pollGridNode(&uid)) {
    returnGridSpaces++;
    Serial.print(F("[RETURN] uid="));
    Serial.print(uid);
    Serial.print(F(" count="));
    Serial.print(returnGridSpaces);
    Serial.print(F("/"));
    Serial.println(gridSpacesAway);

    if (returnGridSpaces >= gridSpacesAway) {
      stopMotors();
      beginEncoderCenteringToState(Test7State::RestoreOriginalHeading);
    }
  }
}

void handlePostObstacleLineFollow() {
  if (!movementSafetyOk()) return;

  applyLineFollowWithoutDelay();

  String uid;
  if (pollGridNode(&uid)) {
    postObstacleRfidCount++;
    Serial.print(F("[POST] uid="));
    Serial.print(uid);
    Serial.print(F(" count="));
    Serial.print(postObstacleRfidCount);
    Serial.print(F("/"));
    Serial.println(postObstacleNodeTarget);

    if (postObstacleRfidCount >= postObstacleNodeTarget) {
      stopMotors();
      transitionTo(Test7State::Complete);
    }
  }
}

void printRunSettings() {
  Serial.println(F("[RUN VARS]"));
  Serial.print(F("  obstacleAheadThresholdMm=")); Serial.println(obstacleAheadThresholdMm, 1);
  Serial.print(F("  frontSafetyStopMm=")); Serial.println(frontSafetyStopMm, 1);
  Serial.print(F("  sameSideDistanceToleranceMm=")); Serial.println(sameSideDistanceToleranceMm, 1);
  Serial.print(F("  clearedSideDistanceIncreaseMm=")); Serial.println(clearedSideDistanceIncreaseMm, 1);
  Serial.print(F("  rfidCenteringForwardOffsetMm=")); Serial.println(rfidCenteringForwardOffsetMm, 1);
  Serial.print(F("  manoeuvreSpeed=")); Serial.println(manoeuvreSpeed);
  Serial.print(F("  movementTimeoutMs=")); Serial.println(movementTimeoutMs);
  Serial.print(F("  rfidNodeDebounceMs=")); Serial.println(rfidNodeDebounceMs);
  Serial.print(F("  debugPrintIntervalMs=")); Serial.println(debugPrintIntervalMs);
  Serial.print(F("  postObstacleNodeTarget=")); Serial.println(postObstacleNodeTarget);
  Serial.print(F("  maxSidewaysGridSpaces=")); Serial.println(maxSidewaysGridSpaces);
  Serial.print(F("  maxPassingGridSpaces=")); Serial.println(maxPassingGridSpaces);
  Serial.print(F("  fallbackSwerveDirection=")); Serial.println(sideName(fallbackSwerveDirection));
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  stop | resume | show | vars"));
  Serial.println(F("  obs 120 | safe 90 | same 35 | clear 90"));
  Serial.println(F("  align 175 | speed 260 | timeout 12000 | debounce 900"));
  Serial.println(F("  debug 0"));
  Serial.println(F("  post 3 | maxaway 4 | maxpass 4"));
  Serial.println(F("  fallback left | fallback right"));
}

void printDebugSnapshot() {
  if (debugPrintIntervalMs > 0 && millis() - lastDebugPrintMs < debugPrintIntervalMs) return;
  lastDebugPrintMs = millis();

  Serial.print(F("[T7] state="));
  Serial.print(stateName(test7State));
  Serial.print(F(" yaw="));
  Serial.print(yawDeg, 2);
  Serial.print(F(" orig="));
  Serial.print(originalHeadingDeg, 2);
  Serial.print(F(" front="));
  Serial.print(latestSonar.frontValid ? latestSonar.frontMm : -1.0f, 1);
  Serial.print(F(" left="));
  Serial.print(latestSonar.leftValid ? latestSonar.leftMm : -1.0f, 1);
  Serial.print(F(" right="));
  Serial.print(latestSonar.rightValid ? latestSonar.rightMm : -1.0f, 1);
  Serial.print(F(" swerve="));
  Serial.print(sideName(swerveDirection));
  Serial.print(F(" obsSide="));
  Serial.print(sideName(obstacleFacingSide));
  Serial.print(F(" refSide="));
  Serial.print(referenceObstacleSideMm, 1);
  Serial.print(F(" curSide="));
  Serial.print(currentObstacleSideMm, 1);
  Serial.print(F(" away="));
  Serial.print(gridSpacesAway);
  Serial.print(F(" passing="));
  Serial.print(gridSpacesPassing);
  Serial.print(F(" post="));
  Serial.print(postObstacleRfidCount);
  Serial.print(F(" alignNext="));
  Serial.print(stateName(rfidCenteringNextState));
  Serial.print(F(" killPaused="));
  Serial.print(killPaused ? F("YES") : F("NO"));
  Serial.print(F(" stop="));
  Serial.println(stopReason);
}

void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printHelp();
    return;
  }
  if (lower == "vars" || lower == "show") {
    printRunSettings();
    return;
  }
  if (lower == "stop") {
    killPaused = false;
    serialStopped = true;
    setSafeStop("serial_stop");
    return;
  }
  if (lower == "resume") {
    const Test7State nextState = killPaused ? resumeAfterKillState : Test7State::FollowLine;
    killPaused = false;
    serialStopped = false;
    stopReason = "none";
    transitionTo(nextState);
    return;
  }
  if (lower == "fallback left") {
    fallbackSwerveDirection = WallSide::Left;
    printRunSettings();
    return;
  }
  if (lower == "fallback right") {
    fallbackSwerveDirection = WallSide::Right;
    printRunSettings();
    return;
  }

  const int space = lower.indexOf(' ');
  if (space <= 0) {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  const String key = lower.substring(0, space);
  const float value = line.substring(space + 1).toFloat();

  if (key == "obs") {
    obstacleAheadThresholdMm = constrain(value, 40.0f, 500.0f);
  } else if (key == "safe") {
    frontSafetyStopMm = constrain(value, 30.0f, 400.0f);
  } else if (key == "same") {
    sameSideDistanceToleranceMm = constrain(value, 5.0f, 200.0f);
  } else if (key == "clear") {
    clearedSideDistanceIncreaseMm = constrain(value, 10.0f, 500.0f);
  } else if (key == "align") {
    rfidCenteringForwardOffsetMm = constrain(value, 0.0f, 500.0f);
  } else if (key == "speed") {
    manoeuvreSpeed = constrain(static_cast<int>(value), 80, kMaxMotorCommand);
  } else if (key == "timeout") {
    movementTimeoutMs = constrain(static_cast<uint32_t>(value), 1000UL, 60000UL);
  } else if (key == "debounce") {
    rfidNodeDebounceMs = constrain(static_cast<uint32_t>(value), 100UL, 5000UL);
  } else if (key == "debug") {
    debugPrintIntervalMs = value <= 0.0f ? 0UL : constrain(static_cast<uint32_t>(value), 1UL, 5000UL);
  } else if (key == "post") {
    postObstacleNodeTarget = constrain(static_cast<int>(value), 1, 10);
  } else if (key == "maxaway") {
    maxSidewaysGridSpaces = constrain(static_cast<int>(value), 1, 10);
  } else if (key == "maxpass") {
    maxPassingGridSpaces = constrain(static_cast<int>(value), 1, 10);
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  printRunSettings();
}

void handleSerialCommands() {
  static String input;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      processSerialCommand(input);
      input = "";
      continue;
    }
    if (input.length() < 90) input += c;
  }
}

void setupPins() {
  if (kUseKillPin) pinMode(kKillPin, INPUT_PULLUP);

  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) pinMode(kQtrPins[i], INPUT);

  if (validPin(kFrontTrigPin)) pinMode(kFrontTrigPin, OUTPUT);
  if (validPin(kFrontEchoPin)) pinMode(kFrontEchoPin, INPUT);
  if (validPin(kLeftTrigPin)) pinMode(kLeftTrigPin, OUTPUT);
  if (validPin(kLeftEchoPin)) pinMode(kLeftEchoPin, INPUT);
  if (validPin(kRightTrigPin)) pinMode(kRightTrigPin, OUTPUT);
  if (validPin(kRightEchoPin)) pinMode(kRightEchoPin, INPUT);
  if (validPin(kFrontTrigPin)) digitalWrite(kFrontTrigPin, LOW);
  if (validPin(kLeftTrigPin)) digitalWrite(kLeftTrigPin, LOW);
  if (validPin(kRightTrigPin)) digitalWrite(kRightTrigPin, LOW);

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  setupPins();
  Wire.begin();
  initializeQtrCalibration();
  initializeMotoron();
  initializeRfid();
  initializeImu();
  resetLineControllerHistory();
  transitionTo(Test7State::FollowLine);

  Serial.println(F("Test 7 obstacle_avoidance ready."));
  printHelp();
  printRunSettings();
}

void loop() {
  handleSerialCommands();

  if (handleMechanicalKillSwitch()) {
    updateImu();
    printDebugSnapshot();
    return;
  }

  updateLatestSensors();

  // switch (test7State) {
  //   case Test7State::FollowLine:
  //     if (latestSonar.frontValid && latestSonar.frontMm <= obstacleAheadThresholdMm) {
  //       stopMotors();
  //       originalHeadingDeg = yawDeg;
  //       headingOffsetFromOriginalDeg = 0.0f;
  //       stopReason = "none";
  //       transitionTo(Test7State::PrepareSwerve);
  //     } else {
  //       applyLineFollowWithoutDelay();
  //     }
  //     break;

  //   case Test7State::PrepareSwerve:
  //     stopMotors();
  //     chooseSwerveDirection();
  //     Serial.print(F("[SWERVE] direction="));
  //     Serial.print(sideName(swerveDirection));
  //     Serial.print(F(" obstacleFacing="));
  //     Serial.println(sideName(obstacleFacingSide));
  //     beginFindAndCenterOnNextRfid(Test7State::TurnAway, false);
  //     break;

  //   case Test7State::FindAlignmentTag:
  //     handleAlignmentTagSearch();
  //     break;

  //   case Test7State::DriveRfidCenterOffset:
  //     handleRfidCenterOffsetDrive();
  //     break;

  //   case Test7State::TurnAway: {
  //     const float turnDeg = 90.0f * turnSignForSwerve();
  //     if (!turnDegreesImu(turnDeg)) {
  //       setSafeStop("turn_away_failed");
  //       break;
  //     }
  //     headingOffsetFromOriginalDeg += turnDeg;
  //     updateLatestSensors();
  //     referenceObstacleSideMm = sonarDistanceForSide(obstacleFacingSide, latestSonar);
  //     if (!sonarValidForSide(obstacleFacingSide, latestSonar)) {
  //       referenceObstacleSideMm = -1.0f;
  //     }
  //     gridSpacesAway = 0;
  //     beginFindAndCenterOnNextRfid(Test7State::MoveSidewaysAroundObstacle, true);
  //     break;
  //   }

  //   case Test7State::MoveSidewaysAroundObstacle:
  //     handleRfidLineFollowMovement(
  //         &gridSpacesAway,
  //         maxSidewaysGridSpaces,
  //         Test7State::TurnParallelToOriginal);
  //     break;

  //   case Test7State::TurnParallelToOriginal: {
  //     const float turnDeg = -90.0f * turnSignForSwerve();
  //     if (!turnDegreesImu(turnDeg)) {
  //       setSafeStop("turn_parallel_failed");
  //       break;
  //     }
  //     headingOffsetFromOriginalDeg += turnDeg;
  //     gridSpacesPassing = 0;
  //     transitionTo(Test7State::PassObstacle);
  //     break;
  //   }

  //   case Test7State::PassObstacle:
  //     handleRfidLineFollowMovement(
  //         &gridSpacesPassing,
  //         maxPassingGridSpaces,
  //         Test7State::TurnBackTowardLine);
  //     break;

  //   case Test7State::TurnBackTowardLine: {
  //     const float turnDeg = -90.0f * turnSignForSwerve();
  //     if (!turnDegreesImu(turnDeg)) {
  //       setSafeStop("turn_back_to_line_failed");
  //       break;
  //     }
  //     headingOffsetFromOriginalDeg += turnDeg;
  //     returnGridSpaces = 0;
  //     transitionTo(Test7State::ReturnToOriginalLineOffset);
  //     break;
  //   }
    
  //   case Test7State::ReturnToOriginalLineOffset:
  //     handleReturnToLineOffset();
  //     break;

  //   case Test7State::RestoreOriginalHeading: {
  //     const float restoreTurnDeg = -headingOffsetFromOriginalDeg;
  //     if (absFloat(restoreTurnDeg) > kTurnToleranceDeg) {
  //       if (!turnDegreesImu(restoreTurnDeg)) {
  //         setSafeStop("restore_heading_failed");
  //         break;
  //       }
  //     }
  //     headingOffsetFromOriginalDeg = 0.0f;
  //     resetLineControllerHistory();
  //     postObstacleRfidCount = 0;
  //     transitionTo(Test7State::FollowLineAfterObstacle);
  //     break;
  //   }

  //   case Test7State::FollowLineAfterObstacle:
  //     handlePostObstacleLineFollow();
  //     break;

  //   case Test7State::Complete:
  //     stopMotors();
  //     if (!completeAnnounced) {
  //       Serial.println(F("[DONE] Test 7 obstacle manoeuvre complete."));
  //       completeAnnounced = true;
  //     }
  //     break;

  //   case Test7State::SafeStop:
  //     stopMotors();
  //     break;
  // }

  applyLineFollowStep();
  // setTank(manoeuvreSpeed, manoeuvreSpeed);

  printDebugSnapshot();
}
