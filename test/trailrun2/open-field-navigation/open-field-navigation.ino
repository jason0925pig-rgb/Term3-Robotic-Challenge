#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD ""
#endif
#ifndef BROKER_HOST
#define BROKER_HOST "192.168.0.74"
#endif
#ifndef BROKER_PORT
#define BROKER_PORT 1883
#endif
#ifndef GROUP_ID
#define GROUP_ID "2"
#endif

// ---------------------------------------------------------------------------
// Open_Field_Navigation
// Board: Arduino GIGA R1 WiFi
//
// Behavior:
//   1. Calibrate IMU at startup and keep this yaw reference for the full run.
//   2. Drive straight with equal motor commands until RFID #1, query/log its
//      coordinate, drive forward to the planting point, and drop one seed.
//   3. Continue straight with equal motor commands until RFID #2. The first two
//      coordinates define the robot's starting grid heading.
//   4. Turn right to absolute yaw -90 deg relative to startup yaw, then use the
//      side sonar that is closer to a wall. The target distance is computed from
//      the RFID coordinate and the corrected arena geometry.
//   5. At RFID #3, plant, then turn left back to absolute yaw 0 deg.
//   6. Continue wall-following and plant RFID #4 and RFID #5, then stop.
//
// Corrected arena geometry from team measurement:
//   - Grid pitch = 250 mm.
//   - Node-to-wall distance: top/left = 190 mm, bottom/right = 250 mm.
//   - Robot width = 190 mm, so side-sonar-to-wall target at nearest node:
//       top/left = 190 - 95 = 95 mm
//       bottom/right = 250 - 95 = 155 mm
//   - Moving one grid cell farther from a wall adds exactly 250 mm.
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
//   Servo signal      -> D33
//   Kill button       -> D32 to GND, INPUT_PULLUP
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

constexpr const char *kBoardId = "Yu7GT_OpenField";
constexpr bool kUseMiniMessenger = true;
constexpr uint32_t kRegisterIntervalMs = 8000;
constexpr uint32_t kWifiStatusPrintMs = 2500;

constexpr uint8_t kRequiredSeeds = 5;
constexpr bool kSkipFirstRfidAtStart = false;  // Set true only if the first detected tag is the start tag.
constexpr float kPlantingOffsetMm = 205.0f;
constexpr int kOpenStraightSpeed = 400;        // Pure equal-speed dead reckoning before first turn.
constexpr int kCenteringDriveSpeed = 360;
constexpr uint32_t kPauseAfterRfidMs = 250;
constexpr uint32_t kPauseAfterPlantMs = 300;
constexpr uint32_t kPauseAfterTurnMs = 500;
constexpr uint32_t kSameRfidCooldownMs = 1200;

// Absolute IMU yaw targets. Startup heading is 0 deg.
constexpr float kFirstTurnAbsoluteYawDeg = -90.0f;  // Right turn.
constexpr float kSecondTurnAbsoluteYawDeg = 0.0f;   // Return to original heading.

// Wall-following PID. PID input is smoothed side-sonar distance corrected by
// IMU yaw: perpendicularDistance ~= sonarFiltered * cos(yaw-wallReferenceYaw).
constexpr int kWallBaseSpeed = 380;
constexpr int kWallMaxCorrection = 190;
constexpr float kWallKp = 2.8f;
constexpr float kWallKi = 0.0f;
constexpr float kWallKd = 0.8f;
constexpr float kWallIntegralClamp = 80.0f;
constexpr uint32_t kWallLoopDelayMs = 20;
constexpr uint32_t kWallPrintIntervalMs = 180;

// Arena geometry.
constexpr float kCellPitchMm = 250.0f;
constexpr float kRobotWidthMm = 190.0f;
constexpr float kHalfRobotWidthMm = kRobotWidthMm / 2.0f;
constexpr float kTopNodeToWallMm = 190.0f;
constexpr float kLeftNodeToWallMm = 190.0f;
constexpr float kBottomNodeToWallMm = 250.0f;
constexpr float kRightNodeToWallMm = 250.0f;
constexpr float kTopSideWallAtRowAmm = kTopNodeToWallMm - kHalfRobotWidthMm;       // 95 mm
constexpr float kLeftSideWallAtCol1mm = kLeftNodeToWallMm - kHalfRobotWidthMm;     // 95 mm
constexpr float kBottomSideWallAtRowImm = kBottomNodeToWallMm - kHalfRobotWidthMm; // 155 mm
constexpr float kRightSideWallAtCol9mm = kRightNodeToWallMm - kHalfRobotWidthMm;   // 155 mm

// Sonar pins and filtering.
constexpr uint8_t kLeftTrigPin = 9;
constexpr uint8_t kRightTrigPin = 10;
constexpr uint8_t kLeftEchoPin = 12;
constexpr uint8_t kRightEchoPin = 13;
constexpr uint32_t kEchoTimeoutUs = 12000;
constexpr float kMinValidSonarMm = 20.0f;
constexpr float kMaxValidSonarMm = 900.0f;
constexpr bool kStopIfNoSonarEcho = false;
constexpr bool kUseSonarSmoothing = true;
constexpr float kSonarEmaAlpha = 0.35f;
constexpr float kSonarRejectJumpMm = 120.0f;
constexpr uint8_t kSonarJumpConfirmSamples = 3;

constexpr bool kUseImuWallDistanceCorrection = true;
constexpr float kMaxWallAngleCorrectionDeg = 60.0f;
constexpr float kMinWallCosFactor = 0.50f;

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
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;
constexpr uint32_t kDriveDistanceTimeoutMs = 10000;

// Waveshare DCGM-N20-12V-EN-200RPM, 1:150 gearbox, 7 C1/A rising pulses per motor rev.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kMotorNoLoadRpm = 200.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f;
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;
constexpr float kTurnCalibration = 1.5f;

// IMU turn control. Positive target uses the existing Motor_IMU convention: left turn.
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

// RFID and servo.
constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 80;

constexpr uint8_t kServoPin = 33;
constexpr int kServoMinUs = 500;
constexpr int kServoMaxUs = 2500;
constexpr int kServoMinAngle = 0;
constexpr int kServoMaxAngle = 300;
constexpr int kServoStepAngle = 60;
constexpr uint32_t kServoMoveSettleMs = 600;
constexpr uint32_t kServoHoldAfterDropMs = 1200;
constexpr uint32_t kServoFrameUs = 20000;
constexpr bool kResetServoAtStartup = true;

constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kDegToRad = 0.0174532925f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

// ---------------------------------------------------------------------------
// Types and RFID map fallback
// ---------------------------------------------------------------------------
struct Cell {
  uint8_t row;
  uint8_t col;
};

struct KnownTag {
  const char *uid;
  Cell cell;
};

enum class Heading : uint8_t {
  North = 0,
  East = 1,
  South = 2,
  West = 3
};

enum class WallSide : uint8_t {
  Left,
  Right
};

enum class ArenaWall : uint8_t {
  Top,
  Bottom,
  Left,
  Right
};

enum class MissionState : uint8_t {
  StraightNoPid,
  WallFollow,
  DriveOffset,
  DropSeed,
  TurnToYaw,
  Complete,
  Stopped
};

struct SonarMeasurement {
  float rawMm;
  float filteredMm;
  bool rawValid;
  bool usedFallback;
  bool rejectedJump;
};

const KnownTag kKnownTags[] = {
  {"1F27AB41", {1, 1}},  {"390DAB41", {1, 2}},  {"9017AB41", {1, 3}},
  {"F63BAB41", {1, 4}},  {"76F0AA41", {1, 5}},  {"8145A941", {1, 6}},
  {"375CAB41", {1, 7}},  {"528AAB41", {1, 8}},  {"F07EAB41", {1, 9}},
  {"F642AB41", {2, 1}},  {"573DAB41", {2, 2}},  {"3385AB41", {2, 3}},
  {"07F6AA41", {2, 4}},  {"BCCFAA41", {2, 5}},  {"C47CAB41", {2, 6}},
  {"E74BA941", {2, 7}},  {"2A60AB41", {2, 8}},  {"7C88AB41", {2, 9}},
  {"10C7AA41", {3, 1}},  {"F052AB41", {3, 2}},  {"E840AB41", {3, 3}},
  {"AC5CAB41", {3, 4}},  {"9312AB41", {3, 5}},  {"8A45AB41", {3, 6}},
  {"6D19AB41", {3, 7}},  {"B493AB41", {3, 8}},  {"7451AB41", {3, 9}},
  {"773DAB41", {4, 1}},  {"47FAAA41", {4, 2}},  {"F459AB41", {4, 3}},
  {"6C5FAB41", {4, 4}},  {"3ACEAA41", {4, 5}},  {"B811AB41", {4, 6}},
  {"70D7AA41", {4, 7}},  {"3D84AB41", {4, 8}},  {"9259AB41", {4, 9}},
  {"D157AB41", {5, 1}},  {"FCD6AA41", {5, 2}},  {"41AB4141", {5, 3}},
  {"AE55AA41", {5, 4}},  {"4FC0AA41", {5, 5}},  {"F85EAB41", {5, 6}},
  {"48CBAA41", {5, 7}},  {"0077AB41", {5, 8}},  {"5663AB41", {5, 9}},
  {"A142AB41", {6, 1}},  {"0D46AB41", {6, 2}},  {"D3DDAA41", {6, 3}},
  {"B3DA2ADD", {6, 4}},  {"6666AA41", {6, 5}},  {"060DAB41", {6, 6}},
  {"CE9CAA41", {6, 7}},  {"F94FAB41", {6, 8}},  {"BD47AB41", {6, 9}},
  {"28E3AA41", {7, 1}},  {"4E4DAB41", {7, 2}},  {"8CE5AA41", {7, 3}},
  {"54C4AA41", {7, 4}},  {"E238A941", {7, 5}},  {"9C01AB41", {7, 6}},
  {"E7F7AA41", {7, 7}},  {"685EAB41", {7, 8}},  {"4A12AB41", {7, 9}},
  {"A335126A", {8, 1}},  {"F164AB41", {8, 2}},  {"A42DAB41", {8, 3}},
  {"F6B6A941", {8, 4}},  {"1D65AA41", {8, 5}},  {"03CCAA41", {8, 6}},
  {"DF54A941", {8, 7}},  {"7074AB41", {8, 8}},  {"2802AB41", {8, 9}},
  {"43DB2CDD", {9, 1}},  {"418BAB41", {9, 2}},  {"1B0AAB41", {9, 3}},
  {"7447AB41", {9, 4}},  {"6E54A641", {9, 5}},  {"70CBAA41", {9, 6}},
  {"855AAB41", {9, 7}},  {"671BAB41", {9, 8}},  {"C3DFAA41", {9, 9}},
};
constexpr uint8_t kKnownTagCount = sizeof(kKnownTags) / sizeof(kKnownTags[0]);

// ---------------------------------------------------------------------------
// Global objects and state
// ---------------------------------------------------------------------------
MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;
MiniMessenger messenger;

volatile long leftCount = 0;
volatile long rightCount = 0;

bool rfidOk = false;
bool imuOk = false;
bool messengerStarted = false;
bool serialStopped = false;
bool completeAnnounced = false;

MissionState missionState = MissionState::StraightNoPid;
MissionState stateAfterPlant = MissionState::StraightNoPid;

uint8_t seedsPlanted = 0;
uint8_t tagsSeen = 0;
bool skippedFirstTag = false;
Cell currentCell = {0, 0};
Cell firstCell = {0, 0};
Cell secondCell = {0, 0};
bool haveCurrentCell = false;
bool haveFirstCell = false;
bool haveSecondCell = false;
Heading initialHeading = Heading::North;
Heading heading = Heading::North;
WallSide wallSide = WallSide::Right;
float targetWallDistanceMm = kRightSideWallAtCol9mm;

char lastUid[12] = "";
uint32_t lastRfidPollMs = 0;
uint32_t lastSameRfidMs = 0;
uint32_t lastRegisterMs = 0;
uint32_t lastWifiStatusPrintMs = 0;
uint32_t lastWallUpdateMs = 0;
uint32_t lastWallPrintMs = 0;
uint32_t lastTurnPrintMs = 0;

float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
float targetYawDeg = 0.0f;
float wallReferenceYawDeg = 0.0f;
uint32_t lastImuUpdateUs = 0;

float wallIntegral = 0.0f;
float lastWallErrorMm = 0.0f;
float lastValidLeftMm = -1.0f;
float lastValidRightMm = -1.0f;
float filteredLeftMm = -1.0f;
float filteredRightMm = -1.0f;
uint8_t leftSonarJumpCount = 0;
uint8_t rightSonarJumpCount = 0;

int currentServoAngle = kServoMinAngle;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
bool validCell(Cell c) {
  return c.row >= 1 && c.row <= 9 && c.col >= 1 && c.col <= 9;
}

char rowLabel(uint8_t row) {
  return static_cast<char>('A' + row - 1);
}

void printCell(Cell cell) {
  if (!validCell(cell)) {
    Serial.print(F("unknown"));
    return;
  }
  Serial.print(rowLabel(cell.row));
  Serial.print(cell.col);
}

const __FlashStringHelper *headingName(Heading h) {
  switch (h) {
    case Heading::North: return F("NORTH");
    case Heading::East: return F("EAST");
    case Heading::South: return F("SOUTH");
    case Heading::West: return F("WEST");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

const __FlashStringHelper *stateName(MissionState state) {
  switch (state) {
    case MissionState::StraightNoPid: return F("STRAIGHT_NO_PID");
    case MissionState::WallFollow: return F("WALL_FOLLOW");
    case MissionState::DriveOffset: return F("DRIVE_OFFSET");
    case MissionState::DropSeed: return F("DROP_SEED");
    case MissionState::TurnToYaw: return F("TURN_TO_YAW");
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

float normalizeAngleDeg(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
}

bool sameCell(Cell a, Cell b) {
  return a.row == b.row && a.col == b.col;
}

bool cellForUid(const char *uid, Cell *out) {
  for (uint8_t i = 0; i < kKnownTagCount; i++) {
    if (strcmp(uid, kKnownTags[i].uid) == 0) {
      *out = kKnownTags[i].cell;
      return true;
    }
  }
  return false;
}

Heading headingFromCells(Cell from, Cell to) {
  if (to.row < from.row) return Heading::North;
  if (to.row > from.row) return Heading::South;
  if (to.col > from.col) return Heading::East;
  return Heading::West;
}

Heading rightOf(Heading h) {
  return static_cast<Heading>((static_cast<uint8_t>(h) + 1) % 4);
}

// ---------------------------------------------------------------------------
// Motor and encoder helpers
// ---------------------------------------------------------------------------
void leftEncoderIsr() {
  const bool a = digitalRead(kLeftEncoderAPin);
  const bool b = digitalRead(kLeftEncoderBPin);
  leftCount += (a == b) ? 1 : -1;
}

void rightEncoderIsr() {
  const bool a = digitalRead(kRightEncoderAPin);
  const bool b = digitalRead(kRightEncoderBPin);
  rightCount += (a == b) ? 1 : -1;
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

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

// ---------------------------------------------------------------------------
// WiFi/server helpers
// ---------------------------------------------------------------------------
void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print(F("[SERVER RX] from "));
  Serial.print(metadata.fromBoardId);
  Serial.print(F(": "));
  Serial.println(msg);
}

void serviceWifi() {
  if (!messengerStarted) return;
  messenger.loop();

  if (messenger.isConnected() && (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs)) {
    lastRegisterMs = millis();
    char reg[96];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kBoardId);
    messenger.sendToBoard("server", reg);
    Serial.print(F("[WIFI] register: "));
    Serial.println(reg);
  }

  if (millis() - lastWifiStatusPrintMs >= kWifiStatusPrintMs) {
    lastWifiStatusPrintMs = millis();
    Serial.print(F("[WIFI] connected="));
    Serial.print(messenger.isConnected() ? F("YES") : F("NO"));
    Serial.print(F(" status="));
    Serial.println(WiFi.status());
  }
}

void queryServerForTag(const char *uid, Cell fallbackCell) {
  Serial.print(F("[SERVER] query coordinate/state for uid="));
  Serial.print(uid);
  Serial.print(F(" fallbackCell="));
  printCell(fallbackCell);
  Serial.println();

  if (messengerStarted && messenger.isConnected()) {
    char msg[128];
    snprintf(msg, sizeof(msg), "type=queryTag tagId=%s uid=%s", uid, uid);
    messenger.sendToBoard("server", msg);
    Serial.print(F("[SERVER TX] "));
    Serial.println(msg);
  }
}

void notifySeedPlanted(const char *uid, Cell cell) {
  Serial.print(F("[SERVER] seed planted uid="));
  Serial.print(uid);
  Serial.print(F(" cell="));
  printCell(cell);
  Serial.println();

  if (messengerStarted && messenger.isConnected()) {
    char msg[128];
    snprintf(msg, sizeof(msg), "type=seedPlanted tagId=%s cell=%c%d", uid, rowLabel(cell.row), cell.col);
    messenger.sendToBoard("server", msg);
    Serial.print(F("[SERVER TX] "));
    Serial.println(msg);
  }
}

void serviceBackground() {
  serviceWifi();
  if (killPressed()) {
    serialStopped = true;
    missionState = MissionState::Stopped;
    stopMotors();
  }
}

// ---------------------------------------------------------------------------
// IMU helpers
// ---------------------------------------------------------------------------
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
  yawDeg = normalizeAngleDeg(yawDeg + gyroZDegPerSec * dt);
  return true;
}

void resetYawAtStartup() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

void setTurnCommand(int signedTurnSpeed) {
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  setTank(-command, command);
}

void printTurnStatus(const char *label, float targetDeg, float errorDeg, int command) {
  if (millis() - lastTurnPrintMs < kTurnPrintIntervalMs) return;
  lastTurnPrintMs = millis();

  Serial.print(label);
  Serial.print(F(" targetYaw="));
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
  Serial.println(getRightCount());
}

bool turnToAbsoluteYaw(float absoluteTargetDeg) {
  if (!imuOk) {
    Serial.println(F("[TURN] IMU missing; cannot do absolute yaw turn reliably."));
    stopMotors();
    return false;
  }

  targetYawDeg = normalizeAngleDeg(absoluteTargetDeg);
  Serial.print(F("[TURN] absolute yaw target="));
  Serial.println(targetYawDeg, 1);

  resetEncoders();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();

  while (true) {
    serviceBackground();
    if (serialStopped) {
      stopMotors();
      return false;
    }

    updateImu();
    const float errorDeg = normalizeAngleDeg(targetYawDeg - yawDeg);
    if (absFloat(errorDeg) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("[TURN] absolute yaw target reached."));
      break;
    }
    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] absolute yaw timeout."));
      break;
    }

    float commandFloat = kTurnKp * errorDeg - kTurnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, kTurnMinSpeed, kTurnMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;
    setTurnCommand(signedCommand);
    printTurnStatus("[AbsTurn]", targetYawDeg, errorDeg, signedCommand);
    delay(10);
  }

  stopMotors();
  updateImu();
  printTurnStatus("[AbsTurn final]", targetYawDeg, normalizeAngleDeg(targetYawDeg - yawDeg), 0);
  return true;
}

// ---------------------------------------------------------------------------
// RFID and servo helpers
// ---------------------------------------------------------------------------
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

int angleToPulseUs(int angle) {
  angle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  return map(angle, kServoMinAngle, kServoMaxAngle, kServoMinUs, kServoMaxUs);
}

void sendServoPulse(int pulseUs) {
  digitalWrite(kServoPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(kServoPin, LOW);
  delayMicroseconds(kServoFrameUs - pulseUs);
}

void holdServoAngle(int angle, uint32_t durationMs) {
  const int pulseUs = angleToPulseUs(angle);
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    serviceBackground();
    if (serialStopped) return;
    sendServoPulse(pulseUs);
  }
}

void moveServoToAngle(int angle) {
  currentServoAngle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  Serial.print(F("[SERVO] moveTo angle="));
  Serial.print(currentServoAngle);
  Serial.print(F(" pulseUs="));
  Serial.println(angleToPulseUs(currentServoAngle));
  holdServoAngle(currentServoAngle, kServoMoveSettleMs);
}

void dropOneSeedNoReturn() {
  int nextAngle = currentServoAngle + kServoStepAngle;
  if (nextAngle > kServoMaxAngle) {
    Serial.println(F("[SERVO] reached 300 deg; reset to 0 before continuing."));
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 500);
    nextAngle = kServoStepAngle;
  }

  Serial.print(F("[PLANT] drop seed #"));
  Serial.println(seedsPlanted + 1);
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);
}

// ---------------------------------------------------------------------------
// Distance drive
// ---------------------------------------------------------------------------
bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("[DRIVE] distanceMm="));
  Serial.print(distanceMm, 1);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    serviceBackground();
    if (serialStopped) {
      stopMotors();
      return false;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;
    if (millis() - start > kDriveDistanceTimeoutMs) {
      Serial.println(F("[DRIVE] timeout; continuing from current position."));
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

// ---------------------------------------------------------------------------
// Sonar/wall-follow helpers
// ---------------------------------------------------------------------------
float readSonarMm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (durationUs == 0) return -1.0f;
  return durationUs * 0.1715f;
}

float updateSonarFilter(bool useLeft, float rawMm, bool *rejectedJump) {
  float &filtered = useLeft ? filteredLeftMm : filteredRightMm;
  uint8_t &jumpCount = useLeft ? leftSonarJumpCount : rightSonarJumpCount;
  *rejectedJump = false;

  if (!kUseSonarSmoothing || filtered < 0.0f) {
    filtered = rawMm;
    jumpCount = 0;
    return filtered;
  }

  const float jumpMm = absFloat(rawMm - filtered);
  if (jumpMm > kSonarRejectJumpMm) {
    jumpCount++;
    if (jumpCount < kSonarJumpConfirmSamples) {
      *rejectedJump = true;
      return filtered;
    }
    filtered = rawMm;
    jumpCount = 0;
    return filtered;
  }

  jumpCount = 0;
  filtered = filtered + kSonarEmaAlpha * (rawMm - filtered);
  return filtered;
}

SonarMeasurement selectedWallDistanceMm() {
  SonarMeasurement measurement;
  measurement.rawMm = -1.0f;
  measurement.filteredMm = -1.0f;
  measurement.rawValid = false;
  measurement.usedFallback = false;
  measurement.rejectedJump = false;

  const bool useLeft = wallSide == WallSide::Left;
  const uint8_t trig = useLeft ? kLeftTrigPin : kRightTrigPin;
  const uint8_t echo = useLeft ? kLeftEchoPin : kRightEchoPin;
  const float mm = readSonarMm(trig, echo);
  measurement.rawMm = mm;

  if (mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm) {
    if (useLeft) {
      lastValidLeftMm = mm;
    } else {
      lastValidRightMm = mm;
    }
    measurement.rawValid = true;
    measurement.filteredMm = updateSonarFilter(useLeft, mm, &measurement.rejectedJump);
    return measurement;
  }

  const float fallback = useLeft ? filteredLeftMm : filteredRightMm;
  if (fallback > 0.0f && !kStopIfNoSonarEcho) {
    measurement.usedFallback = true;
    measurement.filteredMm = fallback;
    return measurement;
  }
  return measurement;
}

ArenaWall wallForSide(Heading h, WallSide side) {
  if (h == Heading::North) return side == WallSide::Left ? ArenaWall::Left : ArenaWall::Right;
  if (h == Heading::East) return side == WallSide::Left ? ArenaWall::Top : ArenaWall::Bottom;
  if (h == Heading::South) return side == WallSide::Left ? ArenaWall::Right : ArenaWall::Left;
  return side == WallSide::Left ? ArenaWall::Bottom : ArenaWall::Top; // Heading::West
}

float targetDistanceToWall(Cell cell, ArenaWall wall) {
  if (!validCell(cell)) return 155.0f;
  switch (wall) {
    case ArenaWall::Top:
      return kTopSideWallAtRowAmm + (cell.row - 1) * kCellPitchMm;
    case ArenaWall::Bottom:
      return kBottomSideWallAtRowImm + (9 - cell.row) * kCellPitchMm;
    case ArenaWall::Left:
      return kLeftSideWallAtCol1mm + (cell.col - 1) * kCellPitchMm;
    case ArenaWall::Right:
      return kRightSideWallAtCol9mm + (9 - cell.col) * kCellPitchMm;
    default:
      return 155.0f;
  }
}

void chooseWallFollowSideAndTarget() {
  const float leftTarget = targetDistanceToWall(currentCell, wallForSide(heading, WallSide::Left));
  const float rightTarget = targetDistanceToWall(currentCell, wallForSide(heading, WallSide::Right));
  const bool leftReachable = leftTarget <= kMaxValidSonarMm;
  const bool rightReachable = rightTarget <= kMaxValidSonarMm;

  if (leftReachable && (!rightReachable || leftTarget <= rightTarget)) {
    wallSide = WallSide::Left;
    targetWallDistanceMm = leftTarget;
  } else {
    wallSide = WallSide::Right;
    targetWallDistanceMm = rightTarget;
  }

  wallIntegral = 0.0f;
  lastWallErrorMm = 0.0f;
  wallReferenceYawDeg = yawDeg;
  lastWallUpdateMs = millis();

  Serial.print(F("[WALL] selected side="));
  Serial.print(sideName(wallSide));
  Serial.print(F(" targetMm="));
  Serial.print(targetWallDistanceMm, 1);
  Serial.print(F(" cell="));
  printCell(currentCell);
  Serial.print(F(" heading="));
  Serial.print(headingName(heading));
  Serial.print(F(" yawRef="));
  Serial.println(wallReferenceYawDeg, 2);
}

float wallCosFactor(float yawOffsetDeg) {
  if (!imuOk || !kUseImuWallDistanceCorrection) return 1.0f;
  const float limitedAngleDeg = constrain(absFloat(yawOffsetDeg), 0.0f, kMaxWallAngleCorrectionDeg);
  float factor = cos(limitedAngleDeg * kDegToRad);
  if (factor < kMinWallCosFactor) factor = kMinWallCosFactor;
  return factor;
}

void runWallFollowMotion() {
  updateImu();

  const SonarMeasurement sonar = selectedWallDistanceMm();
  if (sonar.filteredMm < 0.0f) {
    Serial.println(F("[WALL] no sonar; drive straight fallback."));
    setTank(kWallBaseSpeed, kWallBaseSpeed);
    delay(kWallLoopDelayMs);
    return;
  }

  const float yawOffsetDeg = normalizeAngleDeg(yawDeg - wallReferenceYawDeg);
  const float cosFactor = wallCosFactor(yawOffsetDeg);
  const float distanceMm = sonar.filteredMm * cosFactor;

  const uint32_t now = millis();
  float dt = (now - lastWallUpdateMs) / 1000.0f;
  lastWallUpdateMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = kWallLoopDelayMs / 1000.0f;

  const float errorMm = distanceMm - targetWallDistanceMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);
  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  const float pid = kWallKp * errorMm + kWallKi * wallIntegral + kWallKd * derivative;
  const int correction = constrain(static_cast<int>(pid), -kWallMaxCorrection, kWallMaxCorrection);
  const int turnLeftCorrection = wallSide == WallSide::Left ? correction : -correction;
  const int leftSpeed = kWallBaseSpeed - turnLeftCorrection;
  const int rightSpeed = kWallBaseSpeed + turnLeftCorrection;
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(sideName(wallSide));
    Serial.print(F(" rawMm="));
    Serial.print(sonar.rawMm, 1);
    Serial.print(F(" filtMm="));
    Serial.print(sonar.filteredMm, 1);
    Serial.print(sonar.usedFallback ? F(" fallback") : F(""));
    Serial.print(sonar.rejectedJump ? F(" jumpReject") : F(""));
    Serial.print(F(" yawOff="));
    Serial.print(yawOffsetDeg, 2);
    Serial.print(F(" cos="));
    Serial.print(cosFactor, 3);
    Serial.print(F(" distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(F(" target="));
    Serial.print(targetWallDistanceMm, 1);
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" corr="));
    Serial.print(correction);
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
  }

  delay(kWallLoopDelayMs);
}

// ---------------------------------------------------------------------------
// Mission logic
// ---------------------------------------------------------------------------
void printStatus() {
  Serial.print(F("[STATUS] state="));
  Serial.print(stateName(missionState));
  Serial.print(F(" seeds="));
  Serial.print(seedsPlanted);
  Serial.print(F("/"));
  Serial.print(kRequiredSeeds);
  Serial.print(F(" tags="));
  Serial.print(tagsSeen);
  Serial.print(F(" cell="));
  printCell(currentCell);
  Serial.print(F(" heading="));
  Serial.print(headingName(heading));
  Serial.print(F(" yaw="));
  Serial.println(yawDeg, 2);
}

void handleDetectedRfid(const char *uid) {
  if (strcmp(uid, lastUid) == 0 && millis() - lastSameRfidMs < kSameRfidCooldownMs) {
    return;
  }
  strncpy(lastUid, uid, sizeof(lastUid) - 1);
  lastUid[sizeof(lastUid) - 1] = '\0';
  lastSameRfidMs = millis();

  tagsSeen++;
  stopMotors();

  Cell cell = {0, 0};
  const bool known = cellForUid(uid, &cell);
  currentCell = cell;
  haveCurrentCell = known;

  Serial.print(F("[RFID] uid="));
  Serial.print(uid);
  Serial.print(F(" tagCount="));
  Serial.print(tagsSeen);
  Serial.print(F(" cell="));
  printCell(cell);
  Serial.print(F(" known="));
  Serial.println(known ? F("YES") : F("NO"));

  queryServerForTag(uid, cell);

  if (kSkipFirstRfidAtStart && !skippedFirstTag) {
    skippedFirstTag = true;
    Serial.println(F("[RFID] first tag skipped by parameter; continuing."));
    delay(kPauseAfterRfidMs);
    return;
  }

  if (seedsPlanted >= kRequiredSeeds) {
    missionState = MissionState::Complete;
    return;
  }

  delay(kPauseAfterRfidMs);
  missionState = MissionState::DriveOffset;
}

void runStraightNoPid() {
  String uid;
  if (pollRfid(&uid)) {
    char uidBuffer[12];
    uid.toCharArray(uidBuffer, sizeof(uidBuffer));
    handleDetectedRfid(uidBuffer);
    return;
  }

  updateImu();
  setTank(kOpenStraightSpeed, kOpenStraightSpeed);

  if (millis() - lastWallPrintMs >= 350) {
    lastWallPrintMs = millis();
    Serial.print(F("[OPEN] straight no PID speed="));
    Serial.print(kOpenStraightSpeed);
    Serial.print(F(" yaw="));
    Serial.print(yawDeg, 2);
    Serial.print(F(" seeds="));
    Serial.println(seedsPlanted);
  }
  serviceBackground();
  delay(15);
}

void runWallFollow() {
  String uid;
  if (pollRfid(&uid)) {
    char uidBuffer[12];
    uid.toCharArray(uidBuffer, sizeof(uidBuffer));
    handleDetectedRfid(uidBuffer);
    return;
  }
  runWallFollowMotion();
}

void afterPlantingDecideNextState() {
  if (seedsPlanted >= kRequiredSeeds) {
    missionState = MissionState::Complete;
    return;
  }

  if (seedsPlanted == 1) {
    missionState = MissionState::StraightNoPid;
    return;
  }

  if (seedsPlanted == 2) {
    if (haveFirstCell && haveSecondCell) {
      initialHeading = headingFromCells(firstCell, secondCell);
      heading = rightOf(initialHeading);
      Serial.print(F("[LOCALIZE] initial heading from first two RFIDs = "));
      Serial.print(headingName(initialHeading));
      Serial.print(F(", heading after right turn = "));
      Serial.println(headingName(heading));
    } else {
      Serial.println(F("[LOCALIZE] first two cell coordinates missing; still using default heading."));
      heading = rightOf(initialHeading);
    }
    targetYawDeg = kFirstTurnAbsoluteYawDeg;
    stateAfterPlant = MissionState::WallFollow;
    missionState = MissionState::TurnToYaw;
    return;
  }

  if (seedsPlanted == 3) {
    heading = initialHeading;
    targetYawDeg = kSecondTurnAbsoluteYawDeg;
    stateAfterPlant = MissionState::WallFollow;
    missionState = MissionState::TurnToYaw;
    return;
  }

  missionState = MissionState::WallFollow;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------
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
    Serial.println(F("[IMU] not found. Open-field absolute turns will not run."));
    imuOk = false;
    return false;
  }

  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;

  Serial.println(F("[IMU] found. Keep robot still for startup yaw calibration."));
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
  resetYawAtStartup();
  Serial.print(F("[IMU] gyro Z bias = "));
  Serial.print(gyroZBiasDps, 4);
  Serial.println(F(" deg/s; startup yaw is now 0 deg."));
  return true;
}

void initializeWifi() {
  if (!kUseMiniMessenger || strlen(WIFI_SSID) == 0) {
    Serial.println(F("[WIFI] MiniMessenger disabled or WIFI_SSID empty; server query will print only."));
    messengerStarted = false;
    return;
  }

  messenger.onMessage(onWifiMessage);
  messengerStarted = true;
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kBoardId);
  Serial.print(F("[WIFI] begin group="));
  Serial.print(GROUP_ID);
  Serial.print(F(" board="));
  Serial.println(kBoardId);
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) pinMode(kKillPin, INPUT_PULLUP);

  pinMode(kLeftTrigPin, OUTPUT);
  pinMode(kRightTrigPin, OUTPUT);
  pinMode(kLeftEchoPin, INPUT);
  pinMode(kRightEchoPin, INPUT);

  pinMode(kServoPin, OUTPUT);
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
  initializeWifi();

  if (kResetServoAtStartup) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 700);
  }

  Serial.println(F("Open_Field_Navigation ready."));
  Serial.println(F("Flow: straight to RFID1/RFID2, absolute right yaw -90, sonar wall-follow, absolute yaw 0, wall-follow."));
  Serial.print(F("Arena side sonar nearest targets top/left/bottom/right = "));
  Serial.print(kTopSideWallAtRowAmm, 1);
  Serial.print(F(", "));
  Serial.print(kLeftSideWallAtCol1mm, 1);
  Serial.print(F(", "));
  Serial.print(kBottomSideWallAtRowImm, 1);
  Serial.print(F(", "));
  Serial.println(kRightSideWallAtCol9mm, 1);
  printStatus();
}

void loop() {
  serviceBackground();
  if (serialStopped) {
    stopMotors();
    updateImu();
    delay(50);
    return;
  }

  switch (missionState) {
    case MissionState::StraightNoPid:
      runStraightNoPid();
      break;

    case MissionState::WallFollow:
      runWallFollow();
      break;

    case MissionState::DriveOffset:
      if (driveDistanceMm(kPlantingOffsetMm, kCenteringDriveSpeed)) {
        missionState = MissionState::DropSeed;
      } else {
        missionState = MissionState::Stopped;
      }
      break;

    case MissionState::DropSeed:
      dropOneSeedNoReturn();
      seedsPlanted++;
      if (seedsPlanted == 1) {
        firstCell = currentCell;
        haveFirstCell = haveCurrentCell;
      } else if (seedsPlanted == 2) {
        secondCell = currentCell;
        haveSecondCell = haveCurrentCell;
      }
      notifySeedPlanted(lastUid, currentCell);
      Serial.print(F("[PLANT] seedsPlanted="));
      Serial.print(seedsPlanted);
      Serial.print(F("/"));
      Serial.println(kRequiredSeeds);
      delay(kPauseAfterPlantMs);
      afterPlantingDecideNextState();
      break;

    case MissionState::TurnToYaw:
      if (turnToAbsoluteYaw(targetYawDeg)) {
        delay(kPauseAfterTurnMs);
        if (stateAfterPlant == MissionState::WallFollow) {
          chooseWallFollowSideAndTarget();
        }
        missionState = stateAfterPlant;
      } else {
        missionState = MissionState::Stopped;
      }
      break;

    case MissionState::Complete:
      stopMotors();
      updateImu();
      if (!completeAnnounced) {
        Serial.println(F("[DONE] open-field navigation complete: 5 seeds planted. Motors stopped."));
        completeAnnounced = true;
      }
      delay(100);
      break;

    case MissionState::Stopped:
      stopMotors();
      updateImu();
      delay(50);
      break;
  }
}
