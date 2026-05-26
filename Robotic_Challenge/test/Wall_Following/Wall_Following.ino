#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Wall_Following
// Board: Arduino GIGA R1 WiFi
//
// Behavior:
//   1. Follow either the left or right wall using the selected side HC-SR04.
//   2. Keep the robot kTargetWallDistanceMm away from the wall using PID.
//   3. Count RFID detections while wall following.
//   4. On every kRfidsBeforeCorner-th RFID, drive forward kRfidForwardOffsetMm
//      using both motor encoders.
//   5. If following LEFT wall, turn RIGHT 90 deg with IMU.
//      If following RIGHT wall, turn LEFT 90 deg with IMU.
//   6. Resume wall following and repeat.
//
// Hardware:
//   Left sonar        -> trig D9, echo D12 through 5V-to-3.3V level shifter
//   Right sonar       -> trig D10, echo D13 through 5V-to-3.3V level shifter
//   RFID              -> Wire / D20 SDA-D21 SCL, address 0x28
//   IMU ICM20948      -> Wire / D20 SDA-D21 SCL, address 0x68
//   Motoron           -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1/M2     -> left/right motor
//   Left encoder      -> D34/D35, C1/A rising-edge count, C2/B direction
//   Right encoder     -> D36/D37, C1/A rising-edge count, C2/B direction
//   Kill button       -> D32 to GND, INPUT_PULLUP
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

enum class WallSide {
  Left,
  Right
};

constexpr WallSide kDefaultWallSide = WallSide::Left;
constexpr float kTargetWallDistanceMm = 62.5f;
constexpr uint8_t kRfidsBeforeCorner = 5;       // Turn only after this many RFID detections.
constexpr float kRfidForwardOffsetMm = 90.0f;
constexpr float kCornerTurnDeg = 90.0f;
constexpr uint8_t kMaxCornerCycles = 0;        // 0 = repeat forever.

// Wall-following PID. Start with PD; keep Ki at 0 until P/D are stable.
constexpr int kWallBaseSpeed = 600;
constexpr int kWallMaxCorrection = 220;
constexpr float kMaxFastSlowMotorRatio = 1.40f; // Faster motor <= slower motor * 1.40 during wall following.
constexpr float kWallKp = 1.0f;
constexpr float kWallKi = 0.0f;
constexpr float kWallKd = 0.0f;
constexpr float kWallIntegralClamp = 80.0f;
constexpr uint32_t kWallLoopDelayMs = 20;
constexpr uint32_t kWallPrintIntervalMs = 180;

// Sonar pins.
constexpr uint8_t kLeftTrigPin = 9;
constexpr uint8_t kRightTrigPin = 10;
constexpr uint8_t kLeftEchoPin = 12;
constexpr uint8_t kRightEchoPin = 13;
constexpr uint32_t kEchoTimeoutUs = 12000;     // About 2 m max wait.
constexpr float kMinValidSonarMm = 20.0f;
constexpr float kMaxValidSonarMm = 900.0f;
constexpr bool kStopIfNoSonarEcho = false;     // false = keep using last valid reading if possible.

// RFID.
constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 80;

// Motoron and encoders.
constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr uint16_t kMotoronReferenceMv = 3300;
constexpr auto kMotoronVinType = MotoronVinSenseType::Motoron550;

constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;

constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

// Encoder distance control.
constexpr int kOffsetDriveSpeed = 360;
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;
constexpr uint32_t kDriveTimeoutMs = 9000;

// Waveshare DCGM-N20-12V-EN-200RPM motor model.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kMotorNoLoadRpm = 200.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f; // C1/A rising-edge pulses before gearbox; 1050 per wheel rev.
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;
constexpr float kTurnCalibration = 1.5f;

// IMU turn control. Positive target is assumed to be LEFT turn, matching the
// Motor_IMU sketch. If physical direction is reversed, flip kTurnCommandSign
// or kImuYawSign.
constexpr uint8_t kImuAddress = 0x68;
constexpr uint16_t kGyroBiasSamples = 500;
constexpr uint16_t kGyroBiasSampleDelayMs = 4;
constexpr int kTurnCommandSign = 1;
constexpr int kImuYawSign = 1;
constexpr int kTurnMaxSpeed = 600;
constexpr int kTurnMinSpeed = 115;
constexpr float kTurnKp = 500.0f;
constexpr float kTurnKd = 0.0f;
constexpr float kTurnToleranceDeg = 2.0f;
constexpr float kGyroStopRateDps = 10.0f;
constexpr bool kUseImuTurnTimeout = false;
constexpr uint32_t kTurnTimeoutMs = 50000;
constexpr uint32_t kTurnPrintIntervalMs = 120;

// Mechanical kill.
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

constexpr uint32_t kPauseAfterRfidMs = 250;
constexpr uint32_t kPauseAfterDriveMs = 250;
constexpr uint32_t kPauseAfterTurnMs = 500;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;

volatile long leftCount = 0;
volatile long rightCount = 0;

WallSide wallSide = kDefaultWallSide;
float wallKp = kWallKp;
float wallKi = kWallKi;
float wallKd = kWallKd;
float targetWallDistanceMm = kTargetWallDistanceMm;
int wallBaseSpeed = kWallBaseSpeed;
int wallMaxCorrection = kWallMaxCorrection;
float wallIntegral = 0.0f;
float lastWallErrorMm = 0.0f;
float lastValidLeftMm = -1.0f;
float lastValidRightMm = -1.0f;
uint32_t lastWallUpdateMs = 0;
uint32_t lastWallPrintMs = 0;
uint32_t lastRfidPollMs = 0;

bool rfidOk = false;
bool imuOk = false;
float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;
uint32_t lastTurnPrintMs = 0;
String lastUid;
uint8_t rfidCountSinceLastCorner = 0;
uint8_t cornerCycleCount = 0;
bool serialStopped = false;
bool completeAnnounced = false;

enum class MissionState {
  WallFollow,
  DriveOffset,
  TurnCorner,
  Complete,
  Stopped
};

MissionState missionState = MissionState::WallFollow;

const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

const __FlashStringHelper *stateName(MissionState state) {
  switch (state) {
    case MissionState::WallFollow: return F("WALL_FOLLOW");
    case MissionState::DriveOffset: return F("DRIVE_OFFSET");
    case MissionState::TurnCorner: return F("TURN_CORNER");
    case MissionState::Complete: return F("COMPLETE");
    case MissionState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

long absLong(long value) {
  return value < 0 ? -value : value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
}

int maxWallCorrectionFromRatioLimit() {
  if (kMaxFastSlowMotorRatio <= 1.0f || wallBaseSpeed <= 0) {
    return 0;
  }

  const float maxCorrection =
      wallBaseSpeed * (kMaxFastSlowMotorRatio - 1.0f) / (kMaxFastSlowMotorRatio + 1.0f);
  return max(0, static_cast<int>(maxCorrection));
}

void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
}

long getLeftCount() {
  noInterrupts();
  const long count = leftCount;
  interrupts();
  return count;
}

long getRightCount() {
  noInterrupts();
  const long count = rightCount;
  interrupts();
  return count;
}

void resetEncoders() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  side left | side right"));
  Serial.println(F("  dist 62.5 | base 260 | maxcorr 220"));
  Serial.println(F("  p 3.0 | i 0 | d 16.0"));
  Serial.println(F("  stop | resume | show"));
}

void printSettings() {
  Serial.print(F("[SETTINGS] state="));
  Serial.print(stateName(missionState));
  Serial.print(F(" side="));
  Serial.print(sideName(wallSide));
  Serial.print(F(" targetMm="));
  Serial.print(targetWallDistanceMm, 1);
  Serial.print(F(" base="));
  Serial.print(wallBaseSpeed);
  Serial.print(F(" maxcorr="));
  Serial.print(wallMaxCorrection);
  Serial.print(F(" ratioLimit="));
  Serial.print(kMaxFastSlowMotorRatio, 2);
  Serial.print(F(" P="));
  Serial.print(wallKp, 3);
  Serial.print(F(" I="));
  Serial.print(wallKi, 3);
  Serial.print(F(" D="));
  Serial.print(wallKd, 3);
  Serial.print(F(" cycles="));
  Serial.print(cornerCycleCount);
  Serial.print(F("/"));
  Serial.print(kMaxCornerCycles);
  Serial.print(F(" rfidCount="));
  Serial.print(rfidCountSinceLastCorner);
  Serial.print(F("/"));
  Serial.print(kRfidsBeforeCorner);
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
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
    missionState = MissionState::Stopped;
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
    missionState = MissionState::WallFollow;
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
    targetWallDistanceMm = constrain(value, 20.0f, 500.0f);
  } else if (key == "base") {
    wallBaseSpeed = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "maxcorr") {
    wallMaxCorrection = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "p") {
    wallKp = value;
  } else if (key == "i") {
    wallKi = value;
  } else if (key == "d") {
    wallKd = value;
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
    if (input.length() < 80) {
      input += c;
    }
  }
}

float readSonarMm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (durationUs == 0) {
    return -1.0f;
  }

  return durationUs * 0.1715f;
}

float selectedWallDistanceMm(bool *usedFallback) {
  const bool useLeft = wallSide == WallSide::Left;
  const uint8_t trig = useLeft ? kLeftTrigPin : kRightTrigPin;
  const uint8_t echo = useLeft ? kLeftEchoPin : kRightEchoPin;
  float mm = readSonarMm(trig, echo);

  if (mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm) {
    if (useLeft) {
      lastValidLeftMm = mm;
    } else {
      lastValidRightMm = mm;
    }
    *usedFallback = false;
    return mm;
  }

  const float fallback = useLeft ? lastValidLeftMm : lastValidRightMm;
  if (fallback > 0.0f && !kStopIfNoSonarEcho) {
    *usedFallback = true;
    return fallback;
  }

  *usedFallback = false;
  return -1.0f;
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

bool updateImu() {
  if (!imuOk) return false;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  const uint32_t now = micros();
  float dt = (now - lastImuUpdateUs) / 1000000.0f;
  lastImuUpdateUs = now;
  if (dt < 0.0f || dt > 0.25f) dt = 0.0f;

  const float rawGyroZDps = gyro.gyro.z * kRadToDeg;
  gyroZDegPerSec = (rawGyroZDps - gyroZBiasDps) * kImuYawSign;
  yawDeg += gyroZDegPerSec * dt;
  return true;
}

void resetYaw() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

void setTurnCommand(int signedTurnSpeed) {
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  setTank(-command, command);
}

void printTurnStatus(const char *label, float targetDeg, float errorDeg, int command, long encoderTarget) {
  if (millis() - lastTurnPrintMs < kTurnPrintIntervalMs) return;
  lastTurnPrintMs = millis();

  Serial.print(label);
  Serial.print(F(" target="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" yaw="));
  Serial.print(yawDeg, 2);
  Serial.print(F(" err="));
  Serial.print(errorDeg, 2);
  Serial.print(F(" gyroZ="));
  Serial.print(gyroZDegPerSec, 1);
  Serial.print(F(" cmd="));
  Serial.print(command);
  Serial.print(F(" L="));
  Serial.print(getLeftCount());
  Serial.print(F(" R="));
  Serial.print(getRightCount());
  Serial.print(F(" encTarget="));
  Serial.println(encoderTarget);
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

bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("[DRIVE] distanceMm="));
  Serial.print(distanceMm, 1);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    if (serialStopped || killPressed()) {
      stopMotors();
      return false;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;
    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("[DRIVE] timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);

    if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
      lastWallPrintMs = millis();
      Serial.print(F("[DRIVE] L="));
      Serial.print(getLeftCount());
      Serial.print(F(" R="));
      Serial.print(getRightCount());
      Serial.print(F(" target="));
      Serial.println(targetCounts);
    }
    delay(10);
  }

  stopMotors();
  return true;
}

void encoderOnlyTurnFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;

  Serial.print(F("[TURN] IMU missing, encoder fallback degrees="));
  Serial.print(degrees, 1);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    if (serialStopped || killPressed()) break;

    const long averageAbs = (absLong(getLeftCount()) + absLong(getRightCount())) / 2;
    if (averageAbs >= targetCounts) break;
    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] encoder fallback timeout."));
      break;
    }

    setTurnCommand(direction * abs(speed));
    printTurnStatus("[EncoderTurn]", degrees, 0.0f, direction * abs(speed), targetCounts);
    delay(10);
  }
  stopMotors();
}

bool turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, kTurnMaxSpeed);
    return true;
  }

  const long encoderTarget = turnDegreesToEncoderCounts(targetDeg);
  Serial.print(F("[TURN] IMU targetDeg="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" encoderTarget="));
  Serial.println(encoderTarget);

  resetEncoders();
  resetYaw();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();

  while (true) {
    handleSerialCommands();
    if (serialStopped || killPressed()) {
      stopMotors();
      return false;
    }

    updateImu();
    const float errorDeg = targetDeg - yawDeg;
    if (absFloat(errorDeg) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("[TURN] IMU target reached."));
      break;
    }
    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] IMU timeout."));
      break;
    }

    float commandFloat = kTurnKp * errorDeg - kTurnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, kTurnMinSpeed, kTurnMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;

    setTurnCommand(signedCommand);
    printTurnStatus("[IMUTurn]", targetDeg, errorDeg, signedCommand, encoderTarget);
    delay(10);
  }

  stopMotors();
  updateImu();
  printTurnStatus("[IMUTurn final]", targetDeg, targetDeg - yawDeg, 0, encoderTarget);
  return true;
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
    Serial.println(kRfidsBeforeCorner);

    if (rfidCountSinceLastCorner >= kRfidsBeforeCorner) {
      rfidCountSinceLastCorner = 0;
      stopMotors();
      pauseStopped(kPauseAfterRfidMs);
      missionState = MissionState::DriveOffset;
    }
    return;
  }

  bool usedFallback = false;
  const float distanceMm = selectedWallDistanceMm(&usedFallback);
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

  const float errorMm = distanceMm - targetWallDistanceMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);
  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  const float pid = wallKp * errorMm + wallKi * wallIntegral + wallKd * derivative;
  const int ratioCorrectionLimit = maxWallCorrectionFromRatioLimit();
  int activeCorrectionLimit = wallMaxCorrection;
  if (ratioCorrectionLimit < activeCorrectionLimit) {
    activeCorrectionLimit = ratioCorrectionLimit;
  }
  int correction = constrain(static_cast<int>(pid), -activeCorrectionLimit, activeCorrectionLimit);

  // Positive turnLeftCorrection means left motor slower, right motor faster.
  // Left wall: too far from wall -> turn left. Right wall: too far -> turn right.
  const int turnLeftCorrection = wallSide == WallSide::Left ? correction : -correction;
  const int leftSpeed = wallBaseSpeed - turnLeftCorrection;
  const int rightSpeed = wallBaseSpeed + turnLeftCorrection;
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(sideName(wallSide));
    Serial.print(F(" distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(usedFallback ? F(" fallback") : F(""));
    Serial.print(F(" target="));
    Serial.print(targetWallDistanceMm, 1);
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" corr="));
    Serial.print(correction);
    Serial.print(F(" cap="));
    Serial.print(activeCorrectionLimit);
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.print(rightSpeed);
    Serial.print(F(" uid="));
    if (lastUid.length() > 0) {
      Serial.println(lastUid);
    } else {
      Serial.println(F("none"));
    }
  }

  updateImu();
  delay(kWallLoopDelayMs);
}

float cornerTurnForSelectedWall() {
  // Positive target is LEFT turn. Left-wall following needs a RIGHT turn.
  return wallSide == WallSide::Left ? -kCornerTurnDeg : kCornerTurnDeg;
}

void initializeMotoron() {
  Wire1.begin();
  motoron.setBus(&Wire1);
  motoron.reinitialize();
  delay(10);
  motoron.disableCrc();
  motoron.clearResetFlag();
  motoron.setCommandTimeoutMilliseconds(1000);
  motoron.setMaxAcceleration(kMotoronLeftChannel, 0);
  motoron.setMaxDeceleration(kMotoronLeftChannel, 0);
  motoron.setMaxAcceleration(kMotoronRightChannel, 0);
  motoron.setMaxDeceleration(kMotoronRightChannel, 0);
  stopMotors();
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

bool initializeImu() {
  Serial.print(F("[IMU] starting ICM20948 at 0x"));
  Serial.println(kImuAddress, HEX);

  if (!imu.begin_I2C(kImuAddress, &Wire)) {
    Serial.println(F("[IMU] not found. Encoder fallback will be used for turns."));
    imuOk = false;
    return false;
  }

  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;

  Serial.println(F("[IMU] found. Keep robot still for gyro bias calibration."));
  delay(800);

  float sum = 0.0f;
  for (uint16_t i = 0; i < kGyroBiasSamples; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp, &mag);
    sum += gyro.gyro.z * kRadToDeg;
    delay(kGyroBiasSampleDelayMs);
  }

  gyroZBiasDps = sum / kGyroBiasSamples;
  resetYaw();
  Serial.print(F("[IMU] gyro Z bias = "));
  Serial.print(gyroZBiasDps, 4);
  Serial.println(F(" deg/s"));
  return true;
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

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
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);

  Wire.begin();
  initializeMotoron();
  initializeRfid();
  initializeImu();
  lastWallUpdateMs = millis();

  Serial.println(F("Wall_Following ready."));
  Serial.println(F("Flow: wall follow -> RFID -> drive 90mm -> IMU turn away from selected wall -> repeat."));
  Serial.print(F("Motor spec: no-load rpm="));
  Serial.print(kMotorNoLoadRpm, 1);
  Serial.print(F(", gear ratio=1:"));
  Serial.print(kGearRatio, 0);
  Serial.print(F(", encoder PPR="));
  Serial.println(kEncoderCountsPerMotorRev, 1);
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
    missionState = MissionState::Stopped;
    stopMotors();
  }

  switch (missionState) {
    case MissionState::WallFollow:
      runWallFollowStep();
      break;

    case MissionState::DriveOffset:
      if (driveDistanceMm(kRfidForwardOffsetMm, kOffsetDriveSpeed)) {
        pauseStopped(kPauseAfterDriveMs);
        missionState = MissionState::TurnCorner;
      } else {
        missionState = MissionState::Stopped;
      }
      break;

    case MissionState::TurnCorner:
      if (turnDegreesImu(cornerTurnForSelectedWall())) {
        cornerCycleCount++;
        wallIntegral = 0.0f;
        lastWallErrorMm = 0.0f;
        pauseStopped(kPauseAfterTurnMs);
        if (kMaxCornerCycles > 0 && cornerCycleCount >= kMaxCornerCycles) {
          missionState = MissionState::Complete;
        } else {
          missionState = MissionState::WallFollow;
        }
      } else {
        missionState = MissionState::Stopped;
      }
      break;

    case MissionState::Complete:
      stopMotors();
      if (!completeAnnounced) {
        Serial.println(F("[DONE] wall-following corner sequence complete. Motors stopped."));
        completeAnnounced = true;
      }
      updateImu();
      delay(100);
      break;

    case MissionState::Stopped:
      stopMotors();
      updateImu();
      delay(50);
      break;
  }
}
