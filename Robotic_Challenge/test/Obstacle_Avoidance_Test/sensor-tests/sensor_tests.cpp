#include <Arduino.h>

#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "../constants.h"
#include "../types.h"
#include "../globals.h"
#include "../declarations.h"
#include "../basic_utils.h"
#include "../motor_utils.h"
#include "../encoder_utils.h"
#include "../sonar_wall_utils.h"
#include "../imu_turn_utils.h"


// ============================================================================
// Run tuning variables
// ============================================================================

float obstacleAheadThresholdMm = 200.0f;
float frontSafetyStopMm = 20.0f;
float targetRightWallDistanceMm = kTargetWallDistanceMm;
float sameSideDistanceToleranceMm = 35.0f;
float clearedSideDistanceIncreaseMm = 90.0f;
float rfidCenteringForwardOffsetMm = 175.0f;
int rightWallBaseSpeed = kWallBaseSpeed;
int manoeuvreSpeed = 260;
uint32_t movementTimeoutMs = 12000;
uint32_t rfidNodeDebounceMs = 900;
uint32_t debugPrintIntervalMs = 500;
uint8_t postObstacleNodeTarget = 3;
uint8_t maxSidewaysGridSpaces = 4;
uint8_t maxPassingGridSpaces = 4;

constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 80;
constexpr uint32_t kKillToggleDebounceMs = 300;
const WallSide kControlWallSide = WallSide::Right;
const WallSide kFixedSwerveDirection = WallSide::Left;

enum class RightWallState {
  FollowRightWall,
  PrepareAvoidance,
  FindAlignmentTag,
  DriveRfidCenterOffset,
  TurnAway,
  MoveSidewaysAroundObstacle,
  TurnParallelToOriginal,
  PassObstacle,
  TurnBackTowardWall,
  ReturnToOriginalWallOffset,
  RestoreOriginalHeading,
  FollowRightWallAfterObstacle,
  Complete,
  SafeStop
};

MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);

RightWallState rightWallState = RightWallState::FollowRightWall;
RightWallState resumeAfterKillState = RightWallState::FollowRightWall;
RightWallState rfidCenteringNextState = RightWallState::TurnAway;
uint32_t stateStartedMs = 0;
uint32_t lastDebugPrintMs = 0;
uint32_t lastKillToggleMs = 0;

bool rfidOk = false;
uint32_t lastRfidPollMs = 0;
uint32_t lastAcceptedRfidMs = 0;
String lastAcceptedUid;
String lastSeenUid;

WallSide swerveDirection = kFixedSwerveDirection;
float originalHeadingDeg = 0.0f;
float headingOffsetFromOriginalDeg = 0.0f;
float referenceRightWallMm = -1.0f;
float currentRightWallMm = -1.0f;
bool useObstacleReferenceTarget = false;

uint8_t gridSpacesAway = 0;
uint8_t gridSpacesPassing = 0;
uint8_t returnGridSpaces = 0;
uint8_t postObstacleRfidCount = 0;

SonarReading latestSonar;
String stopReason = "none";
bool completeAnnounced = false;
bool killPaused = false;
bool killButtonWasPressed = false;
bool rfidCenteringCountsAsSidewaysStep = false;

const __FlashStringHelper *stateName(RightWallState state) {
  switch (state) {
    case RightWallState::FollowRightWall: return F("FOLLOW_RIGHT_WALL");
    case RightWallState::PrepareAvoidance: return F("PREPARE_AVOIDANCE");
    case RightWallState::FindAlignmentTag: return F("FIND_ALIGNMENT_TAG");
    case RightWallState::DriveRfidCenterOffset: return F("DRIVE_RFID_CENTER_OFFSET");
    case RightWallState::TurnAway: return F("TURN_AWAY");
    case RightWallState::MoveSidewaysAroundObstacle: return F("MOVE_SIDEWAYS_AROUND_OBSTACLE");
    case RightWallState::TurnParallelToOriginal: return F("TURN_PARALLEL_TO_ORIGINAL");
    case RightWallState::PassObstacle: return F("PASS_OBSTACLE");
    case RightWallState::TurnBackTowardWall: return F("TURN_BACK_TOWARD_WALL");
    case RightWallState::ReturnToOriginalWallOffset: return F("RETURN_TO_ORIGINAL_WALL_OFFSET");
    case RightWallState::RestoreOriginalHeading: return F("RESTORE_ORIGINAL_HEADING");
    case RightWallState::FollowRightWallAfterObstacle: return F("FOLLOW_RIGHT_WALL_AFTER_OBSTACLE");
    case RightWallState::Complete: return F("COMPLETE");
    case RightWallState::SafeStop: return F("SAFE_STOP");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

void transitionTo(RightWallState nextState) {
  rightWallState = nextState;
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

  currentRightWallMm = latestSonar.rightValid ? latestSonar.rightMm : -1.0f;
}

void setSafeStop(const String &reason) {
  stopReason = reason;
  stopMotors();
  transitionTo(RightWallState::SafeStop);
}

void resetWallControllerHistory() {
  wallIntegral = 0.0f;
  lastWallErrorMm = 0.0f;
  lastWallUpdateMs = millis();
}

float activeWallTargetDistanceMm() {
  if (useObstacleReferenceTarget && referenceRightWallMm > 0.0f) {
    return referenceRightWallMm;
  }
  return targetRightWallDistanceMm;
}

int wallCorrectionLimitForBase(int baseSpeed) {
  const int ratioCorrectionLimit = maxWallCorrectionFromRatioLimit(baseSpeed);
  int activeCorrectionLimit = kWallMaxCorrection;
  if (ratioCorrectionLimit < activeCorrectionLimit) activeCorrectionLimit = ratioCorrectionLimit;
  return activeCorrectionLimit;
}

MotorCommand computeRightWallFollowCommand(
    float distanceMm,
    int baseSpeed,
    float targetMm,
    bool *validOut) {
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

  const float errorMm = distanceMm - targetMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);

  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  const float pid = kWallKp * errorMm + kWallKi * wallIntegral + kWallKd * derivative;
  const int correctionLimit = wallCorrectionLimitForBase(baseSpeed);
  const int correction = constrain(static_cast<int>(pid), -correctionLimit, correctionLimit);

  // For a right wall, positive error means the robot is too far from the wall,
  // so the left motor speeds up and the robot turns right.
  cmd.left = baseSpeed + correction;
  cmd.right = baseSpeed - correction;

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[RIGHT_WALL] distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(F(" target="));
    Serial.print(targetMm, 1);
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

bool applyRightWallFollowStep(int baseSpeed) {
  bool validDistance = false;
  const float distanceMm = selectedWallDistanceMm(kControlWallSide, latestSonar, &validDistance);

  if (!validDistance) {
    stopMotors();
    if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
      lastWallPrintMs = millis();
      Serial.println(F("[RIGHT_WALL] no valid right sonar distance."));
    }
    return false;
  }

  bool validCommand = false;
  const MotorCommand cmd = computeRightWallFollowCommand(
      distanceMm,
      baseSpeed,
      activeWallTargetDistanceMm(),
      &validCommand);

  if (validCommand) {
    setTank(cmd.left, cmd.right);
  } else {
    stopMotors();
  }

  return validCommand;
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
      resetWallControllerHistory();
      transitionTo(resumeAfterKillState);

      Serial.print(F("[KILL] resumed state="));
      Serial.println(stateName(rightWallState));
    } else if (rightWallState != RightWallState::Complete &&
               rightWallState != RightWallState::SafeStop) {
      resumeAfterKillState = rightWallState;
      killPaused = true;
      serialStopped = true;
      stopReason = "kill_pause";
      stopMotors();
      transitionTo(RightWallState::SafeStop);

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

bool obstacleSideCleared(float currentMm) {
  if (referenceRightWallMm < 0.0f) return false;
  if (currentMm < 0.0f) return true;
  return currentMm > referenceRightWallMm + clearedSideDistanceIncreaseMm;
}

bool obstacleStillBeside(float currentMm) {
  if (currentMm < 0.0f) return false;
  if (absFloat(currentMm - referenceRightWallMm) <= sameSideDistanceToleranceMm) return true;
  return currentMm <= referenceRightWallMm + clearedSideDistanceIncreaseMm;
}

void beginFindAndCenterOnNextRfid(RightWallState nextState, bool countsAsSidewaysStep) {
  rfidCenteringNextState = nextState;
  rfidCenteringCountsAsSidewaysStep = countsAsSidewaysStep;
  resetWallControllerHistory();
  transitionTo(RightWallState::FindAlignmentTag);
}

void beginWallCenteringToState(RightWallState nextState) {
  rfidCenteringNextState = nextState;
  rfidCenteringCountsAsSidewaysStep = false;
  resetEncoders();
  resetWallControllerHistory();
  transitionTo(RightWallState::DriveRfidCenterOffset);
}

void handleSidewaysStepAfterCentering() {
  gridSpacesAway++;
  latestSonar = readSonars();
  currentRightWallMm = latestSonar.rightValid ? latestSonar.rightMm : -1.0f;

  Serial.print(F("[NODE] alignment count="));
  Serial.print(gridSpacesAway);
  Serial.print(F(" rightMm="));
  Serial.println(currentRightWallMm, 1);

  if (obstacleSideCleared(currentRightWallMm)) {
    transitionTo(RightWallState::TurnParallelToOriginal);
    return;
  }

  if (!obstacleStillBeside(currentRightWallMm)) {
    Serial.println(F("[SIDE] ambiguous right distance; continuing cautiously."));
  }

  if (gridSpacesAway >= maxSidewaysGridSpaces) {
    setSafeStop("side_clear_not_found");
    return;
  }

  resetWallControllerHistory();
  transitionTo(RightWallState::MoveSidewaysAroundObstacle);
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
    transitionTo(RightWallState::DriveRfidCenterOffset);
    return;
  }

  applyRightWallFollowStep(manoeuvreSpeed);
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

    const RightWallState nextState = rfidCenteringNextState;
    const bool countsAsSidewaysStep = rfidCenteringCountsAsSidewaysStep;
    rfidCenteringCountsAsSidewaysStep = false;

    if (countsAsSidewaysStep) {
      handleSidewaysStepAfterCentering();
    } else {
      resetWallControllerHistory();
      transitionTo(nextState);
    }
    return;
  }

  applyRightWallFollowStep(manoeuvreSpeed);
}

void handleRfidRightWallMovement(uint8_t *counter, uint8_t maxCounter, RightWallState nextState) {
  if (!movementSafetyOk()) return;

  if (!applyRightWallFollowStep(manoeuvreSpeed)) return;

  String uid;
  if (!pollGridNode(&uid)) return;

  (*counter)++;
  stopMotors();
  latestSonar = readSonars();
  currentRightWallMm = latestSonar.rightValid ? latestSonar.rightMm : -1.0f;

  Serial.print(F("[NODE] uid="));
  Serial.print(uid);
  Serial.print(F(" count="));
  Serial.print(*counter);
  Serial.print(F(" rightMm="));
  Serial.println(currentRightWallMm, 1);

  if (obstacleSideCleared(currentRightWallMm)) {
    beginWallCenteringToState(nextState);
    return;
  }

  if (!obstacleStillBeside(currentRightWallMm)) {
    Serial.println(F("[SIDE] ambiguous right distance; continuing cautiously."));
  }

  if (*counter >= maxCounter) {
    setSafeStop("side_clear_not_found");
  }
}

void handleReturnToOriginalWallOffset() {
  if (!movementSafetyOk()) return;

  if (!applyRightWallFollowStep(manoeuvreSpeed)) return;

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
      beginWallCenteringToState(RightWallState::RestoreOriginalHeading);
    }
  }
}

void handlePostObstacleRightWallFollow() {
  if (!movementSafetyOk()) return;

  if (!applyRightWallFollowStep(rightWallBaseSpeed)) return;

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
      transitionTo(RightWallState::Complete);
    }
  }
}

void printRunSettings() {
  Serial.println(F("[RUN VARS]"));
  Serial.print(F("  obstacleAheadThresholdMm=")); Serial.println(obstacleAheadThresholdMm, 1);
  Serial.print(F("  frontSafetyStopMm=")); Serial.println(frontSafetyStopMm, 1);
  Serial.print(F("  targetRightWallDistanceMm=")); Serial.println(targetRightWallDistanceMm, 1);
  Serial.print(F("  sameSideDistanceToleranceMm=")); Serial.println(sameSideDistanceToleranceMm, 1);
  Serial.print(F("  clearedSideDistanceIncreaseMm=")); Serial.println(clearedSideDistanceIncreaseMm, 1);
  Serial.print(F("  rfidCenteringForwardOffsetMm=")); Serial.println(rfidCenteringForwardOffsetMm, 1);
  Serial.print(F("  rightWallBaseSpeed=")); Serial.println(rightWallBaseSpeed);
  Serial.print(F("  manoeuvreSpeed=")); Serial.println(manoeuvreSpeed);
  Serial.print(F("  movementTimeoutMs=")); Serial.println(movementTimeoutMs);
  Serial.print(F("  rfidNodeDebounceMs=")); Serial.println(rfidNodeDebounceMs);
  Serial.print(F("  debugPrintIntervalMs=")); Serial.println(debugPrintIntervalMs);
  Serial.print(F("  postObstacleNodeTarget=")); Serial.println(postObstacleNodeTarget);
  Serial.print(F("  maxSidewaysGridSpaces=")); Serial.println(maxSidewaysGridSpaces);
  Serial.print(F("  maxPassingGridSpaces=")); Serial.println(maxPassingGridSpaces);
  Serial.print(F("  controlSide=")); Serial.println(sideName(kControlWallSide));
  Serial.print(F("  swerveDirection=")); Serial.println(sideName(kFixedSwerveDirection));
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  stop | resume | show | vars"));
  Serial.println(F("  obs 120 | safe 90 | target 62"));
  Serial.println(F("  same 35 | clear 90 | align 175"));
  Serial.println(F("  speed 600 | avoid 260 | timeout 12000 | debounce 900"));
  Serial.println(F("  debug 0"));
  Serial.println(F("  post 3 | maxaway 4 | maxpass 4"));
}

void printDebugSnapshot() {
  if (debugPrintIntervalMs > 0 && millis() - lastDebugPrintMs < debugPrintIntervalMs) return;
  lastDebugPrintMs = millis();

  Serial.print(F("[T7RW] state="));
  Serial.print(stateName(rightWallState));
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
  Serial.print(F(" target="));
  Serial.print(activeWallTargetDistanceMm(), 1);
  Serial.print(F(" refRight="));
  Serial.print(referenceRightWallMm, 1);
  Serial.print(F(" refTarget="));
  Serial.print(useObstacleReferenceTarget ? F("YES") : F("NO"));
  Serial.print(F(" away="));
  Serial.print(gridSpacesAway);
  Serial.print(F(" passing="));
  Serial.print(gridSpacesPassing);
  Serial.print(F(" post="));
  Serial.print(postObstacleRfidCount);
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
    killPaused = false;
    serialStopped = false;
    stopReason = "none";
    useObstacleReferenceTarget = false;
    referenceRightWallMm = -1.0f;
    resetWallControllerHistory();
    transitionTo(RightWallState::FollowRightWall);
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
    frontSafetyStopMm = constrain(value, 20.0f, 400.0f);
  } else if (key == "target") {
    targetRightWallDistanceMm = constrain(value, kMinValidSonarMm, 400.0f);
  } else if (key == "same") {
    sameSideDistanceToleranceMm = constrain(value, 5.0f, 200.0f);
  } else if (key == "clear") {
    clearedSideDistanceIncreaseMm = constrain(value, 10.0f, 500.0f);
  } else if (key == "align") {
    rfidCenteringForwardOffsetMm = constrain(value, 0.0f, 500.0f);
  } else if (key == "speed") {
    rightWallBaseSpeed = constrain(static_cast<int>(value), 80, kMaxMotorCommand);
  } else if (key == "avoid") {
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
  initializeMotoron();
  initializeRfid();
  initializeImu();
  resetWallControllerHistory();
  transitionTo(RightWallState::FollowRightWall);

  Serial.println(F("Test 7 obstacle_avoidance right-wall PID ready."));
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



  printDebugSnapshot();
}
