#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// wall_following
// Board: Arduino GIGA R1 WiFi
//
// Wall-following sketch adapted from ../Wall_Following/Wall_Following.ino.
// It reuses the obstacle-avoidance primitives split into local headers, while
// keeping RFID corner counting and mission flow in this file.
//
// Flow:
//   wall follow -> RFID count -> drive offset -> turn away from wall -> repeat
// ============================================================================

#include "constants.h"
#include "types.h"
#include "globals.h"
#include "declarations.h"
#include "basic_utils.h"
#include "motor_utils.h"
#include "encoder_utils.h"
#include "sonar_wall_utils.h"
#include "imu_turn_utils.h"

// Wall-following mission constants copied from Wall_Following.ino.
constexpr WallSide kWallDefaultSide = WallSide::Left;
constexpr uint8_t kWallRfidsBeforeCorner = 5;
constexpr float kWallRfidForwardOffsetMm = 90.0f;
constexpr float kWallCornerTurnDeg = 90.0f;
constexpr int kWallMaxCornerCycles = 0;
constexpr int kWallOffsetDriveSpeed = 360;
constexpr bool kWallStopIfNoSonarEcho = false;

constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 80;

constexpr uint32_t kPauseAfterRfidMs = 250;
constexpr uint32_t kPauseAfterDriveMs = 250;
constexpr uint32_t kPauseAfterTurnMs = 500;

enum class WallMissionState {
  WallFollow,
  DriveOffset,
  TurnCorner,
  Complete,
  Stopped
};

MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);

WallMissionState missionState = WallMissionState::WallFollow;
WallSide wallSide = kWallDefaultSide;

float activeWallKp = kWallKp;
float activeWallKi = kWallKi;
float activeWallKd = kWallKd;
float activeTargetWallDistanceMm = kTargetWallDistanceMm;
int activeWallBaseSpeed = kWallBaseSpeed;
int activeWallMaxCorrection = kWallMaxCorrection;

bool rfidOk = false;
uint32_t lastRfidPollMs = 0;
String lastUid;
uint8_t rfidCountSinceLastCorner = 0;
int cornerCycleCount = 0;
bool completeAnnounced = false;

const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

const __FlashStringHelper *stateName(WallMissionState state) {
  switch (state) {
    case WallMissionState::WallFollow: return F("WALL_FOLLOW");
    case WallMissionState::DriveOffset: return F("DRIVE_OFFSET");
    case WallMissionState::TurnCorner: return F("TURN_CORNER");
    case WallMissionState::Complete: return F("COMPLETE");
    case WallMissionState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  side left | side right"));
  Serial.println(F("  dist 62.5 | base 600 | maxcorr 220"));
  Serial.println(F("  p 1.0 | i 0 | d 0"));
  Serial.println(F("  stop | resume | show"));
}

void printSettings() {
  Serial.print(F("[SETTINGS] state="));
  Serial.print(stateName(missionState));
  Serial.print(F(" side="));
  Serial.print(sideName(wallSide));
  Serial.print(F(" targetMm="));
  Serial.print(activeTargetWallDistanceMm, 1);
  Serial.print(F(" base="));
  Serial.print(activeWallBaseSpeed);
  Serial.print(F(" maxcorr="));
  Serial.print(activeWallMaxCorrection);
  Serial.print(F(" ratioLimit="));
  Serial.print(kMaxFastSlowMotorRatio, 2);
  Serial.print(F(" P="));
  Serial.print(activeWallKp, 3);
  Serial.print(F(" I="));
  Serial.print(activeWallKi, 3);
  Serial.print(F(" D="));
  Serial.print(activeWallKd, 3);
  Serial.print(F(" cycles="));
  Serial.print(cornerCycleCount);
  Serial.print(F("/"));
  Serial.print(kWallMaxCornerCycles);
  Serial.print(F(" rfidCount="));
  Serial.print(rfidCountSinceLastCorner);
  Serial.print(F("/"));
  Serial.print(kWallRfidsBeforeCorner);
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
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

float selectedWallDistanceForSide(bool *usedFallback) {
  const bool useLeft = wallSide == WallSide::Left;
  const int trigPin = useLeft ? kLeftTrigPin : kRightTrigPin;
  const int echoPin = useLeft ? kLeftEchoPin : kRightEchoPin;
  const float mm = readSonarMm(trigPin, echoPin);

  if (isValidSonarDistance(mm)) {
    if (useLeft) lastValidLeftMm = mm;
    else lastValidRightMm = mm;
    *usedFallback = false;
    return mm;
  }

  const float fallback = useLeft ? lastValidLeftMm : lastValidRightMm;
  if (fallback > 0.0f && !kWallStopIfNoSonarEcho) {
    *usedFallback = true;
    return fallback;
  }

  *usedFallback = false;
  return -1.0f;
}

int activeWallCorrectionLimit() {
  const int ratioCorrectionLimit = maxWallCorrectionFromRatioLimit(activeWallBaseSpeed);
  return min(activeWallMaxCorrection, ratioCorrectionLimit);
}

void pauseStopped(uint32_t durationMs) {
  stopMotors();
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    handleSerialCommands();
    if (serialStopped || killPressed()) {
      stopMotors();
      return;
    }
    updateImu();
    delay(20);
  }
}

void runWallFollowStep() {
  String uid;
  if (pollRfid(&uid)) {
    lastUid = uid;
    rfidCountSinceLastCorner++;
    Serial.print(F("[RFID] detected UID="));
    Serial.print(lastUid);
    Serial.print(F(" count="));
    Serial.print(rfidCountSinceLastCorner);
    Serial.print(F("/"));
    Serial.println(kWallRfidsBeforeCorner);

    if (rfidCountSinceLastCorner >= kWallRfidsBeforeCorner) {
      rfidCountSinceLastCorner = 0;
      stopMotors();
      pauseStopped(kPauseAfterRfidMs);
      missionState = WallMissionState::DriveOffset;
    }
    return;
  }

  bool usedFallback = false;
  const float distanceMm = selectedWallDistanceForSide(&usedFallback);
  if (distanceMm < 0.0f) {
    stopMotors();
    Serial.println(F("[WALL] no valid sonar distance; motors stopped."));
    delay(80);
    return;
  }

  const uint32_t now = millis();
  float dt = (now - lastWallUpdateMs) / 1000.0f;
  lastWallUpdateMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = kWallLoopDelayMs / 1000.0f;

  const float errorMm = distanceMm - activeTargetWallDistanceMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);
  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  const float pid = activeWallKp * errorMm + activeWallKi * wallIntegral + activeWallKd * derivative;
  const int correctionLimit = activeWallCorrectionLimit();
  const int correction = constrain(static_cast<int>(pid), -correctionLimit, correctionLimit);

  const int turnLeftCorrection = wallSide == WallSide::Left ? correction : -correction;
  const int leftSpeed = activeWallBaseSpeed - turnLeftCorrection;
  const int rightSpeed = activeWallBaseSpeed + turnLeftCorrection;
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(sideName(wallSide));
    Serial.print(F(" distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(usedFallback ? F(" fallback") : F(""));
    Serial.print(F(" target="));
    Serial.print(activeTargetWallDistanceMm, 1);
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" corr="));
    Serial.print(correction);
    Serial.print(F(" cap="));
    Serial.print(correctionLimit);
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.print(rightSpeed);
    Serial.print(F(" uid="));
    if (lastUid.length() > 0) Serial.println(lastUid);
    else Serial.println(F("none"));
  }

  updateImu();
  delay(kWallLoopDelayMs);
}

float cornerTurnForSelectedWall() {
  return wallSide == WallSide::Left ? -kWallCornerTurnDeg : kWallCornerTurnDeg;
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
  if (lower == "show") {
    printSettings();
    return;
  }
  if (lower == "stop") {
    serialStopped = true;
    missionState = WallMissionState::Stopped;
    stopMotors();
    Serial.println(F("[SERIAL] stopped."));
    return;
  }
  if (lower == "resume") {
    serialStopped = false;
    completeAnnounced = false;
    rfidCountSinceLastCorner = 0;
    wallIntegral = 0.0f;
    lastWallErrorMm = 0.0f;
    missionState = WallMissionState::WallFollow;
    Serial.println(F("[SERIAL] resumed wall following."));
    return;
  }
  if (lower == "side left") {
    wallSide = WallSide::Left;
    printSettings();
    return;
  }
  if (lower == "side right") {
    wallSide = WallSide::Right;
    printSettings();
    return;
  }

  const int space = line.indexOf(' ');
  if (space <= 0) {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  const String key = lower.substring(0, space);
  const float value = line.substring(space + 1).toFloat();

  if (key == "dist") {
    activeTargetWallDistanceMm = constrain(value, 20.0f, 500.0f);
  } else if (key == "base") {
    activeWallBaseSpeed = constrain(static_cast<int>(value), 0, kMaxMotorCommand);
  } else if (key == "maxcorr") {
    activeWallMaxCorrection = constrain(static_cast<int>(value), 0, kMaxMotorCommand);
  } else if (key == "p") {
    activeWallKp = value;
  } else if (key == "i") {
    activeWallKi = value;
  } else if (key == "d") {
    activeWallKd = value;
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  printSettings();
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
    if (input.length() < 80) input += c;
  }
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) pinMode(kKillPin, INPUT_PULLUP);

  pinMode(kLeftTrigPin, OUTPUT);
  pinMode(kRightTrigPin, OUTPUT);
  pinMode(kLeftEchoPin, INPUT);
  pinMode(kRightEchoPin, INPUT);
  digitalWrite(kLeftTrigPin, LOW);
  digitalWrite(kRightTrigPin, LOW);

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);

  Wire.begin();
  initializeMotoron();
  initializeRfid();
  initializeImu();
  lastWallUpdateMs = millis();

  Serial.println(F("wall_following ready."));
  Serial.println(F("Flow: wall follow -> RFID -> drive 90mm -> IMU turn away from selected wall -> repeat."));
  Serial.print(F("Encoder counts per mm estimate = "));
  Serial.println(kEncoderCountsPerMm, 4);
  printHelp();
  printSettings();
  pauseStopped(1200);
}

void loop() {
  handleSerialCommands();

  if (killPressed()) {
    serialStopped = true;
    missionState = WallMissionState::Stopped;
    stopMotors();
  }

  switch (missionState) {
    case WallMissionState::WallFollow:
      runWallFollowStep();
      break;

    case WallMissionState::DriveOffset:
      if (driveDistanceMm(kWallRfidForwardOffsetMm, kWallOffsetDriveSpeed)) {
        pauseStopped(kPauseAfterDriveMs);
        missionState = WallMissionState::TurnCorner;
      } else {
        missionState = WallMissionState::Stopped;
      }
      break;

    case WallMissionState::TurnCorner:
      if (turnDegreesImu(cornerTurnForSelectedWall())) {
        cornerCycleCount++;
        wallIntegral = 0.0f;
        lastWallErrorMm = 0.0f;
        pauseStopped(kPauseAfterTurnMs);
        if (kWallMaxCornerCycles > 0 && cornerCycleCount >= kWallMaxCornerCycles) {
          missionState = WallMissionState::Complete;
        } else {
          missionState = WallMissionState::WallFollow;
        }
      } else {
        missionState = WallMissionState::Stopped;
      }
      break;

    case WallMissionState::Complete:
      stopMotors();
      if (!completeAnnounced) {
        Serial.println(F("[DONE] wall-following corner sequence complete. Motors stopped."));
        completeAnnounced = true;
      }
      updateImu();
      delay(100);
      break;

    case WallMissionState::Stopped:
      stopMotors();
      updateImu();
      delay(50);
      break;
  }
}
