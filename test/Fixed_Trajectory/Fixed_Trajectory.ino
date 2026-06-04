#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <MiniMessenger.h>
#include <Arduino_Modulino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

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
// Fixed_Trajectory
// Board: Arduino GIGA R1 WiFi
//
// Current route:
//   Enter at C9, exit at G9.
//   C9 -> A9 -> A1 -> B1 -> B8 -> C8 -> C1 -> D1 -> D9
//   -> E9 -> E1 -> F1 -> F9 -> G9 -> G1 -> H1 -> H9 -> I9 -> I1
//
// High-level behavior:
//   - Follow the fixed route and scan every RFID card.
//   - Ask the server whether the scanned tag is fertile and unplanted.
//   - If yes, drive forward 7.5 cm, record LDR value, drop one seed, and
//     report seedPlanted to the server.
//   - If emergency return is triggered, route directly to G9 and skip RFID
//     planting/server checks. RFID may still be used only for localization.
//   - Front sonar obstacle: stop and confirm. If persistent, run the line-based
//     swerve-around obstacle strategy. If that cannot complete, fall back to
//     reverse-to-RFID recovery and BFS.
//   - Mechanical kill button toggles pause/resume. WiFi disable pauses; once
//     re-enabled, the robot immediately starts emergency return to G9.
//   - Normal pixels: solid red. Paused/waiting: blinking red. Rescue success:
//     green for 5 seconds.
//
// Server commands that Alex has not published yet are currently printed to
// Serial only. Known MiniMessenger messages are implemented behind
// kUseMiniMessengerServerMessages.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

constexpr const char *kBoardId = "Yu7GT";
constexpr bool kUseMiniMessengerServerMessages = true;
constexpr bool kDefaultMotionAllowedBeforeServer = true;
constexpr uint32_t kRegisterIntervalMs = 8000;
constexpr uint32_t kServerReplyTimeoutMs = 800;
constexpr uint32_t kWifiStatusPrintMs = 2500;

constexpr uint32_t kMissionLimitMs = 5UL * 60UL * 1000UL;
constexpr uint32_t kEmergencyReturnReserveMs = 40UL * 1000UL;
constexpr uint32_t kBrightestReportReserveMs = 10UL * 1000UL;

constexpr int kRouteBaseSpeed = 380;
constexpr int kReturnBaseSpeed = 520;
constexpr int kSearchTurnSpeed = 220;
constexpr int kHardTurnSpeed = 480;
constexpr int kMaxLineCorrection = 560;
constexpr float kLineKp = 0.80f;
constexpr float kLineKd = 0.08f;
constexpr uint16_t kLineThreshold = 230;
constexpr uint16_t kStrongLineThreshold = 650;
constexpr uint32_t kLineLostBeforeWallMs = 350;

constexpr int kWallBaseSpeed = 350;
constexpr int kWallMaxCorrection = 180;
constexpr float kWallKp = 2.4f;
constexpr float kWallKd = 0.5f;
constexpr float kWallSwitchCloserMarginMm = 45.0f;
constexpr float kCellPitchMm = 250.0f;
constexpr float kTopWallFirstCellMm = 95.0f;
constexpr float kLeftWallFirstCellMm = 95.0f;
constexpr float kBottomWallFirstCellMm = 165.0f;
constexpr float kRightWallFirstCellMm = 95.0f; // Usually not used, but kept for fallback.

constexpr float kPlantCenterOffsetMm = 75.0f;
constexpr int kCenteringDriveSpeed = 360;
constexpr int kReverseSearchSpeed = 240;
constexpr uint32_t kReverseToRfidTimeoutMs = 15000;
constexpr uint32_t kDriveDistanceTimeoutMs = 10000;

constexpr float kFrontObstacleThresholdMm = 100.0f;
constexpr uint32_t kObstacleConfirmMs = 10000;
constexpr uint32_t kReturnObstacleConfirmMs = 5000;
constexpr float kAvoidanceFrontSafetyStopMm = 20.0f;
constexpr float kAvoidanceSameSideDistanceToleranceMm = 35.0f;
constexpr float kAvoidanceClearedSideDistanceIncreaseMm = 90.0f;
constexpr float kAvoidanceRfidCenteringOffsetMm = 220.0f;
constexpr int kAvoidanceManoeuvreSpeed = 260;
constexpr uint32_t kAvoidanceMovementTimeoutMs = 12000;
constexpr uint32_t kAvoidanceRfidNodeDebounceMs = 900;
constexpr uint8_t kAvoidancePostObstacleNodeTarget = 3;
constexpr uint8_t kAvoidanceMaxSidewaysGridSpaces = 4;
constexpr uint8_t kAvoidanceMaxPassingGridSpaces = 4;

constexpr uint8_t kMaxSeedsToPlant = 5;
constexpr bool kPlantIfNoServerReply = false;
constexpr uint32_t kSameRfidCooldownMs = 900;

// Rescue behavior. Unknown server rescue commands are Serial placeholders for now.
constexpr int kRescueSlowSpeed = 200;
constexpr uint8_t kRescueConsecutiveRfidLimit = 2;
constexpr float kRescueTurnBackDeg = 180.0f;
constexpr uint32_t kRescueGreenMs = 5000;

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

// QTR-HD-09RC line sensor.
constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr uint16_t kMinUsefulCalibrationSpan = 20;
constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};

// RFID, sonar, servo, buttons, LDR, LEDs.
constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 80;

constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kLeftTrigPin = 9;
constexpr uint8_t kRightTrigPin = 10;
constexpr uint8_t kFrontEchoPin = 11;
constexpr uint8_t kLeftEchoPin = 12;
constexpr uint8_t kRightEchoPin = 13;
constexpr uint32_t kEchoTimeoutUs = 12000;
constexpr float kMinValidSonarMm = 20.0f;
constexpr float kMaxValidSonarMm = 900.0f;

constexpr uint8_t kServoPin = 33;
constexpr int kServoMinUs = 500;
constexpr int kServoMaxUs = 2500;
constexpr int kServoMinAngle = 0;
constexpr int kServoMaxAngle = 300;
constexpr int kServoStepAngle = 60;
constexpr uint32_t kServoMoveSettleMs = 600;
constexpr uint32_t kServoHoldAfterDropMs = 1200;
constexpr uint32_t kServoFrameUs = 20000;

constexpr uint8_t kRevivalPin = 31;
constexpr uint8_t kKillPin = 32;
constexpr uint32_t kButtonDebounceMs = 250;

constexpr uint8_t kLdrPin = A0;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kLdrFixedResistorOhms = 10000.0f;
constexpr bool kLdrToGround = true;

constexpr uint8_t kLedBrightness = 70;
constexpr uint32_t kPausedBlinkMs = 350;
constexpr uint32_t kStatusPrintIntervalMs = 200;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

// ---------------------------------------------------------------------------
// Types and fixed map data
// ---------------------------------------------------------------------------
struct Cell {
  uint8_t row;
  uint8_t col;
};

struct KnownTag {
  const char *uid;
  Cell cell;
};

struct LightSample {
  Cell cell;
  char uid[12];
  int raw;
  float voltage;
  float resistanceOhms;
};

struct ServerReply {
  bool valid;
  bool fertile;
  bool planted;
  Cell cell;
  uint32_t receivedMs;
};

enum class Heading : uint8_t {
  North = 0,
  East = 1,
  South = 2,
  West = 3
};

enum class MissionMode : uint8_t {
  FixedRoute,
  ReturnHome,
  Rescue,
  Complete
};

enum class FollowMode : uint8_t {
  Line,
  Wall,
  Search,
  Stopped
};

enum class WallSide : uint8_t {
  Left,
  Right
};

constexpr Cell kStartCell = {3, 9};  // C9
constexpr Cell kFinishCell = {7, 9}; // G9
constexpr Heading kInitialHeading = Heading::North;

constexpr Cell kFixedRoute[] = {
  {3, 9},  {2, 9},  {1, 9},  {1, 8},  {1, 7},  {1, 6},
  {1, 5},  {1, 4},  {1, 3},  {1, 2},  {1, 1},  {2, 1},
  {2, 2},  {2, 3},  {2, 4},  {2, 5},  {2, 6},  {2, 7},
  {2, 8},  {3, 8},  {3, 7},  {3, 6},  {3, 5},  {3, 4},
  {3, 3},  {3, 2},  {3, 1},  {4, 1},  {4, 2},  {4, 3},
  {4, 4},  {4, 5},  {4, 6},  {4, 7},  {4, 8},  {4, 9},
  {5, 9},  {5, 8},  {5, 7},  {5, 6},  {5, 5},  {5, 4},
  {5, 3},  {5, 2},  {5, 1},  {6, 1},  {6, 2},  {6, 3},
  {6, 4},  {6, 5},  {6, 6},  {6, 7},  {6, 8},  {6, 9},
  {7, 9},  {7, 8},  {7, 7},  {7, 6},  {7, 5},  {7, 4},
  {7, 3},  {7, 2},  {7, 1},  {8, 1},  {8, 2},  {8, 3},
  {8, 4},  {8, 5},  {8, 6},  {8, 7},  {8, 8},  {8, 9},
  {9, 9},  {9, 8},  {9, 7},  {9, 6},  {9, 5},  {9, 4},
  {9, 3},  {9, 2},  {9, 1},
};

constexpr uint8_t kFixedRouteLength = sizeof(kFixedRoute) / sizeof(kFixedRoute[0]);

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
ModulinoPixels pixels;

volatile long leftCount = 0;
volatile long rightCount = 0;

uint16_t qtrRaw[9] = {};
uint16_t qtrNorm[9] = {};

bool rfidOk = false;
bool imuOk = false;
bool pixelsOk = false;
bool messengerStarted = false;
bool wifiSafetyEnabled = kDefaultMotionAllowedBeforeServer;
bool wifiPaused = false;
bool mechanicalPaused = false;
bool emergencyReturnRequested = false;
bool brightestReported = false;
bool redBlinkOn = false;
bool lineDetected = false;

MissionMode missionMode = MissionMode::FixedRoute;
Heading heading = kInitialHeading;
Cell currentCell = kStartCell;
uint8_t routeIndex = 0;
uint8_t seedsPlanted = 0;
int currentServoAngle = kServoMinAngle;

bool obstacleGrid[10][10] = {};
Cell plannedPath[81];
uint8_t plannedPathLength = 0;
uint8_t plannedPathIndex = 0;
bool usingBfsPath = false;

Cell rescueTarget = {0, 0};
bool rescuePending = false;
bool rescueCompleted = false;
bool rescueModeActive = false;
uint8_t rescueRfidCounter = 0;

float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;

int lastLineError = 0;
int lastSeenLineError = 0;
uint32_t lastLineSeenMs = 0;
float lastWallErrorMm = 0.0f;
uint32_t lastWallUpdateMs = 0;

uint32_t lastRfidPollMs = 0;
uint32_t lastStatusPrintMs = 0;
uint32_t lastWifiStatusPrintMs = 0;
uint32_t lastRegisterMs = 0;
uint32_t missionStartMs = 0;
uint32_t lastBlinkMs = 0;
uint32_t greenUntilMs = 0;
uint32_t lastButtonToggleMs = 0;
uint32_t lastHandledRfidMs = 0;
char lastHandledUid[12] = "";
uint32_t lastAvoidanceRfidMs = 0;
char lastAvoidanceUid[12] = "";

char pendingServerUid[12] = "";
bool waitingForServerReply = false;
ServerReply lastServerReply = {};

LightSample lightSamples[81];
uint8_t lightSampleCount = 0;

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------
long absLong(long value) { return value < 0 ? -value : value; }
float absFloat(float value) { return value < 0.0f ? -value : value; }

bool sameCell(Cell a, Cell b) {
  return a.row == b.row && a.col == b.col;
}

bool validCell(Cell c) {
  return c.row >= 1 && c.row <= 9 && c.col >= 1 && c.col <= 9;
}

char rowLabel(uint8_t row) {
  return static_cast<char>('A' + row - 1);
}

void printCell(Cell cell) {
  if (!validCell(cell)) {
    Serial.print(F("none"));
    return;
  }
  Serial.print(rowLabel(cell.row));
  Serial.print(cell.col);
}

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
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

long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(abs(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

const __FlashStringHelper *headingName(Heading h) {
  switch (h) {
    case Heading::North: return F("N");
    case Heading::East: return F("E");
    case Heading::South: return F("S");
    case Heading::West: return F("W");
    default: return F("?");
  }
}

const __FlashStringHelper *modeName(MissionMode mode) {
  switch (mode) {
    case MissionMode::FixedRoute: return F("FIXED_ROUTE");
    case MissionMode::ReturnHome: return F("RETURN_HOME");
    case MissionMode::Rescue: return F("RESCUE");
    case MissionMode::Complete: return F("COMPLETE");
    default: return F("UNKNOWN");
  }
}

// ---------------------------------------------------------------------------
// Hardware IO
// ---------------------------------------------------------------------------
void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
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

bool killPressedRaw() {
  return digitalRead(kKillPin) == LOW;
}

bool revivalPressed() {
  return digitalRead(kRevivalPin) == LOW;
}

float readSonarMm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t duration = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (duration == 0) {
    return -1.0f;
  }
  return duration * 0.343f / 2.0f;
}

bool sonarReadingValid(float mm) {
  return mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm;
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

bool cellForUid(const char *uid, Cell *out) {
  for (uint8_t i = 0; i < kKnownTagCount; i++) {
    if (strcmp(uid, kKnownTags[i].uid) == 0) {
      *out = kKnownTags[i].cell;
      return true;
    }
  }
  return false;
}

const char *uidForCell(Cell cell) {
  for (uint8_t i = 0; i < kKnownTagCount; i++) {
    if (sameCell(cell, kKnownTags[i].cell)) {
      return kKnownTags[i].uid;
    }
  }
  return "";
}

int readLdrRaw() {
  return analogRead(kLdrPin);
}

float ldrVoltageFromRaw(int raw) {
  return (raw / 4095.0f) * kAdcReferenceVoltage;
}

float ldrResistanceFromVoltage(float voltage) {
  if (voltage <= 0.001f || voltage >= kAdcReferenceVoltage - 0.001f) {
    return NAN;
  }
  if (kLdrToGround) {
    return kLdrFixedResistorOhms * voltage / (kAdcReferenceVoltage - voltage);
  }
  return kLdrFixedResistorOhms * (kAdcReferenceVoltage / voltage - 1.0f);
}

void setAllPixels(ModulinoColor color, uint8_t brightness = kLedBrightness) {
  if (!pixelsOk) return;
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, brightness);
  }
  pixels.show();
}

void setPixelsOff() {
  if (!pixelsOk) return;
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, BLACK, 0);
  }
  pixels.show();
}

void updatePixels() {
  if (!pixelsOk) return;

  if (millis() < greenUntilMs) {
    setAllPixels(GREEN, kLedBrightness);
    return;
  }

  const bool paused = mechanicalPaused || wifiPaused;
  if (!paused) {
    setAllPixels(RED, kLedBrightness);
    return;
  }

  if (millis() - lastBlinkMs >= kPausedBlinkMs) {
    lastBlinkMs = millis();
    redBlinkOn = !redBlinkOn;
    if (redBlinkOn) {
      setAllPixels(RED, kLedBrightness);
    } else {
      setPixelsOff();
    }
  }
}

// ---------------------------------------------------------------------------
// IMU and turning
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
  yawDeg += gyroZDegPerSec * dt;
  return true;
}

void resetYaw() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

bool safetyPaused() {
  return mechanicalPaused || wifiPaused;
}

void serviceBackground();

bool waitWhilePausedOrReturnRequested() {
  while (safetyPaused()) {
    stopMotors();
    serviceBackground();
    if (emergencyReturnRequested && !wifiPaused && !mechanicalPaused) {
      return false;
    }
    delay(20);
  }
  return !emergencyReturnRequested;
}

void setTurnCommand(int signedTurnSpeed) {
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  setTank(-command, command);
}

void encoderOnlyTurnFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    serviceBackground();
    if (!waitWhilePausedOrReturnRequested()) break;
    if (absLong(getLeftCount()) + absLong(getRightCount()) >= 2 * targetCounts) break;
    if (millis() - start > kTurnTimeoutMs) break;
    setTurnCommand(direction * abs(speed));
    delay(10);
  }
  stopMotors();
}

void turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, kTurnMaxSpeed);
    return;
  }

  Serial.print(F("[TURN] targetDeg="));
  Serial.println(targetDeg, 1);
  resetEncoders();
  resetYaw();

  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;
  while (true) {
    serviceBackground();
    if (!waitWhilePausedOrReturnRequested()) break;
    updateImu();

    const float errorDeg = targetDeg - yawDeg;
    const float absError = absFloat(errorDeg);
    if (absError <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("[TURN] target reached."));
      break;
    }
    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] timeout."));
      break;
    }

    float commandFloat = kTurnKp * errorDeg - kTurnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, kTurnMinSpeed, kTurnMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;
    setTurnCommand(signedCommand);

    if (millis() - lastPrintMs >= 140) {
      lastPrintMs = millis();
      Serial.print(F("[TURN] yaw="));
      Serial.print(yawDeg, 2);
      Serial.print(F(" err="));
      Serial.print(errorDeg, 2);
      Serial.print(F(" cmd="));
      Serial.println(signedCommand);
    }
    delay(10);
  }
  stopMotors();
}

Heading headingFromCells(Cell from, Cell to) {
  if (to.row < from.row) return Heading::North;
  if (to.row > from.row) return Heading::South;
  if (to.col > from.col) return Heading::East;
  return Heading::West;
}

bool adjacentCells(Cell a, Cell b) {
  return abs(static_cast<int>(a.row) - static_cast<int>(b.row)) +
         abs(static_cast<int>(a.col) - static_cast<int>(b.col)) == 1;
}

float turnDegreesForHeadingChange(Heading current, Heading desired) {
  const int diff = (static_cast<int>(desired) - static_cast<int>(current) + 4) % 4;
  if (diff == 0) return 0.0f;
  if (diff == 1) return -90.0f;  // Right turn.
  if (diff == 3) return 90.0f;   // Left turn.
  return 180.0f;
}

void faceCell(Cell nextCell) {
  if (!adjacentCells(currentCell, nextCell)) return;
  const Heading desired = headingFromCells(currentCell, nextCell);
  const float turnDeg = turnDegreesForHeadingChange(heading, desired);
  if (absFloat(turnDeg) > 1.0f) {
    Serial.print(F("[NAV] turn from "));
    Serial.print(headingName(heading));
    Serial.print(F(" to "));
    Serial.print(headingName(desired));
    Serial.print(F(" deg="));
    Serial.println(turnDeg, 1);
    turnDegreesImu(turnDeg);
  }
  heading = desired;
}

Cell stepCell(Cell from, Heading h) {
  Cell out = from;
  if (h == Heading::North && out.row > 1) out.row--;
  if (h == Heading::South && out.row < 9) out.row++;
  if (h == Heading::East && out.col < 9) out.col++;
  if (h == Heading::West && out.col > 1) out.col--;
  return out;
}

// ---------------------------------------------------------------------------
// QTR line and wall following
// ---------------------------------------------------------------------------
void readQtrRcArray() {
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], OUTPUT);
    digitalWrite(kQtrPins[i], HIGH);
  }
  delayMicroseconds(10);

  const uint32_t start = micros();
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
    qtrRaw[i] = kQtrTimeoutUs;
  }

  bool allDone = false;
  while (!allDone && (micros() - start) < kQtrTimeoutUs) {
    allDone = true;
    const uint16_t elapsed = static_cast<uint16_t>(micros() - start);
    for (uint8_t i = 0; i < 9; i++) {
      if (qtrRaw[i] == kQtrTimeoutUs) {
        if (digitalRead(kQtrPins[i]) == LOW) {
          qtrRaw[i] = elapsed;
        } else {
          allDone = false;
        }
      }
    }
  }
}

int computeLinePosition() {
  uint32_t weighted = 0;
  uint32_t sum = 0;
  lineDetected = false;

  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t minV = kSavedQtrMin[i];
    const uint16_t maxV = kSavedQtrMax[i];
    const uint16_t span = maxV > minV ? maxV - minV : 0;
    if (span < kMinUsefulCalibrationSpan || qtrRaw[i] <= minV) {
      qtrNorm[i] = 0;
    } else if (qtrRaw[i] >= maxV) {
      qtrNorm[i] = 1000;
    } else {
      qtrNorm[i] = static_cast<uint16_t>((static_cast<uint32_t>(qtrRaw[i] - minV) * 1000UL) / span);
    }

    const uint16_t weight = qtrNorm[i] >= kLineThreshold ? qtrNorm[i] : 0;
    if (weight > 0) {
      lineDetected = true;
      weighted += static_cast<uint32_t>(weight) * (i * 1000);
      sum += weight;
    }
  }

  if (sum == 0) {
    return 4000 + lastSeenLineError;
  }

  const int position = static_cast<int>(weighted / sum);
  lastSeenLineError = position - 4000;
  lastLineSeenMs = millis();
  return position;
}

void lineFollowStep(int baseSpeed) {
  readQtrRcArray();
  const int position = computeLinePosition();
  const int error = position - 4000;

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;
  if (!lineDetected) {
    if (lastSeenLineError < 0) {
      leftSpeed = -kSearchTurnSpeed;
      rightSpeed = kSearchTurnSpeed;
    } else {
      leftSpeed = kSearchTurnSpeed;
      rightSpeed = -kSearchTurnSpeed;
    }
  } else {
    const int derivative = error - lastLineError;
    lastLineError = error;
    int correction = static_cast<int>(kLineKp * error + kLineKd * derivative);
    correction = constrain(correction, -kMaxLineCorrection, kMaxLineCorrection);

    if (error < -2600 || qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold) {
      leftSpeed = -kHardTurnSpeed;
      rightSpeed = kHardTurnSpeed;
    } else if (error > 2600 || qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold) {
      leftSpeed = kHardTurnSpeed;
      rightSpeed = -kHardTurnSpeed;
    } else {
      leftSpeed = baseSpeed + correction;
      rightSpeed = baseSpeed - correction;
    }
  }

  setTank(leftSpeed, rightSpeed);

  if (millis() - lastStatusPrintMs >= kStatusPrintIntervalMs) {
    lastStatusPrintMs = millis();
    Serial.print(F("[LINE] pos="));
    Serial.print(position);
    Serial.print(F(" err="));
    Serial.print(error);
    Serial.print(F(" detected="));
    Serial.print(lineDetected ? F("YES") : F("NO"));
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
  }
}

Heading relativeSensorHeading(bool leftSensor) {
  int value = static_cast<int>(heading) + (leftSensor ? 3 : 1);
  value %= 4;
  return static_cast<Heading>(value);
}

float expectedWallDistanceForCardinal(Heading cardinal) {
  switch (cardinal) {
    case Heading::North:
      return kTopWallFirstCellMm + (currentCell.row - 1) * kCellPitchMm;
    case Heading::South:
      return kBottomWallFirstCellMm + (9 - currentCell.row) * kCellPitchMm;
    case Heading::West:
      return kLeftWallFirstCellMm + (currentCell.col - 1) * kCellPitchMm;
    case Heading::East:
    default:
      return kRightWallFirstCellMm + (9 - currentCell.col) * kCellPitchMm;
  }
}

void wallFollowStep(int baseSpeed) {
  const float leftMm = readSonarMm(kLeftTrigPin, kLeftEchoPin);
  const float rightMm = readSonarMm(kRightTrigPin, kRightEchoPin);
  const bool leftValid = sonarReadingValid(leftMm);
  const bool rightValid = sonarReadingValid(rightMm);

  if (!leftValid && !rightValid) {
    setTank(baseSpeed / 2, baseSpeed / 2);
    return;
  }

  bool useLeft = leftValid;
  if (leftValid && rightValid) {
    useLeft = leftMm <= rightMm + kWallSwitchCloserMarginMm;
  } else if (!leftValid && rightValid) {
    useLeft = false;
  }

  const float measuredMm = useLeft ? leftMm : rightMm;
  const Heading wallDirection = relativeSensorHeading(useLeft);
  const float targetMm = expectedWallDistanceForCardinal(wallDirection);

  const uint32_t now = millis();
  float dt = (now - lastWallUpdateMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastWallUpdateMs = now;

  const float errorMm = measuredMm - targetMm;
  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  int correction = static_cast<int>(kWallKp * errorMm + kWallKd * derivative);
  correction = constrain(correction, -kWallMaxCorrection, kWallMaxCorrection);

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;
  if (useLeft) {
    leftSpeed = baseSpeed + correction;
    rightSpeed = baseSpeed - correction;
  } else {
    leftSpeed = baseSpeed - correction;
    rightSpeed = baseSpeed + correction;
  }
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastStatusPrintMs >= kStatusPrintIntervalMs) {
    lastStatusPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(useLeft ? F("L") : F("R"));
    Serial.print(F(" measured="));
    Serial.print(measuredMm, 1);
    Serial.print(F(" target="));
    Serial.print(targetMm, 1);
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
  }
}

void hybridFollowStep(int baseSpeed) {
  readQtrRcArray();
  computeLinePosition();
  if (lineDetected || millis() - lastLineSeenMs < kLineLostBeforeWallMs) {
    lineFollowStep(baseSpeed);
  } else {
    wallFollowStep(baseSpeed);
  }
}

// ---------------------------------------------------------------------------
// Servo and planting
// ---------------------------------------------------------------------------
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
    if (safetyPaused()) {
      stopMotors();
    }
    sendServoPulse(pulseUs);
  }
}

void moveServoToAngle(int angle) {
  currentServoAngle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  Serial.print(F("[SERVO] angle="));
  Serial.println(currentServoAngle);
  holdServoAngle(currentServoAngle, kServoMoveSettleMs);
}

void dropOneSeedNoReturn() {
  int nextAngle = currentServoAngle + kServoStepAngle;
  if (nextAngle > kServoMaxAngle) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 500);
    nextAngle = kServoStepAngle;
  }
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);
}

void recordLightSample(Cell cell, const char *uid) {
  const int raw = readLdrRaw();
  const float voltage = ldrVoltageFromRaw(raw);
  const float resistance = ldrResistanceFromVoltage(voltage);

  if (lightSampleCount < 81) {
    LightSample &sample = lightSamples[lightSampleCount++];
    sample.cell = cell;
    strncpy(sample.uid, uid, sizeof(sample.uid) - 1);
    sample.uid[sizeof(sample.uid) - 1] = '\0';
    sample.raw = raw;
    sample.voltage = voltage;
    sample.resistanceOhms = resistance;
  }

  Serial.print(F("[LIGHT] cell="));
  printCell(cell);
  Serial.print(F(" uid="));
  Serial.print(uid);
  Serial.print(F(" raw="));
  Serial.print(raw);
  Serial.print(F(" voltage="));
  Serial.print(voltage, 3);
  Serial.print(F(" resistanceOhm="));
  Serial.println(resistance, 1);
}

void printBrightestLight() {
  if (lightSampleCount == 0) {
    Serial.println(F("[LIGHT] No LDR samples recorded yet."));
    return;
  }

  uint8_t best = 0;
  for (uint8_t i = 1; i < lightSampleCount; i++) {
    if (lightSamples[i].raw > lightSamples[best].raw) {
      best = i;
    }
  }

  Serial.print(F("[LIGHT] Brightest recorded cell="));
  printCell(lightSamples[best].cell);
  Serial.print(F(" uid="));
  Serial.print(lightSamples[best].uid);
  Serial.print(F(" raw="));
  Serial.print(lightSamples[best].raw);
  Serial.print(F(" voltage="));
  Serial.println(lightSamples[best].voltage, 3);
}

bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();

  while (true) {
    serviceBackground();
    if (!waitWhilePausedOrReturnRequested()) return false;

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;
    if (millis() - start > kDriveDistanceTimeoutMs) {
      Serial.println(F("[DRIVE] distance timeout."));
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
// MiniMessenger and server placeholders
// ---------------------------------------------------------------------------
void parseServerReply(const char *msg) {
  if (!strstr(msg, "type=isFertileReply")) return;

  lastServerReply.valid = true;
  lastServerReply.fertile = strstr(msg, "fertile=true") != nullptr;
  lastServerReply.planted = strstr(msg, "planted=true") != nullptr;
  lastServerReply.receivedMs = millis();
  lastServerReply.cell = {0, 0};

  const char *xPtr = strstr(msg, "x=");
  const char *yPtr = strstr(msg, "y=");
  if (xPtr && yPtr) {
    lastServerReply.cell.col = atoi(xPtr + 2);
    lastServerReply.cell.row = atoi(yPtr + 2);
  }
  waitingForServerReply = false;
}

void requestRescueFromMessage(const char *msg) {
  // Alex has not published this command yet. For now, accept either:
  //   type=rescue cell=C5
  //   type=rescue row=3 col=5
  if (!strstr(msg, "rescue")) return;

  const char *cellPtr = strstr(msg, "cell=");
  const bool hasCellLabel =
      cellPtr &&
      ((cellPtr[5] >= 'A' && cellPtr[5] <= 'I') || (cellPtr[5] >= 'a' && cellPtr[5] <= 'i')) &&
      (cellPtr[6] >= '1' && cellPtr[6] <= '9');
  if (hasCellLabel) {
    rescueTarget.row = toupper(cellPtr[5]) - 'A' + 1;
    rescueTarget.col = atoi(cellPtr + 6);
    rescuePending = validCell(rescueTarget);
  } else {
    const char *rowPtr = strstr(msg, "row=");
    const char *colPtr = strstr(msg, "col=");
    if (rowPtr && colPtr) {
      rescueTarget.row = atoi(rowPtr + 4);
      rescueTarget.col = atoi(colPtr + 4);
      rescuePending = validCell(rescueTarget);
    }
  }

  if (rescuePending) {
    Serial.print(F("[RESCUE] request received for "));
    printCell(rescueTarget);
    Serial.println(F(". Continue route until rescue column/approach point."));
  } else {
    Serial.println(F("[RESCUE] rescue-like message received but no valid coordinate parsed."));
  }
}

void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print(F("[WIFI RX] from "));
  Serial.print(metadata.fromBoardId);
  Serial.print(F(": "));
  Serial.println(msg);

  parseServerReply(msg);
  requestRescueFromMessage(msg);

  if (strstr(msg, "type=heartbeat enable=0") || strstr(msg, "type=disable") ||
      strstr(msg, "enabled=false") || strstr(msg, "type=emergency")) {
    wifiPaused = true;
    wifiSafetyEnabled = false;
    stopMotors();
    Serial.println(F("[WIFI SAFETY] paused by server."));
  }

  if (strstr(msg, "type=heartbeat enable=1") || strstr(msg, "enable=1") || strstr(msg, "enabled=true")) {
    const bool wasPaused = wifiPaused;
    wifiPaused = false;
    wifiSafetyEnabled = true;
    if (wasPaused) {
      emergencyReturnRequested = true;
      Serial.println(F("[WIFI SAFETY] re-enabled; emergency return will start."));
    }
  }
}

void updateWifi() {
  if (!messengerStarted) return;
  messenger.loop();

  if (messenger.isConnected() && (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs)) {
    lastRegisterMs = millis();
    char reg[96];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kBoardId);
    const bool ok = messenger.sendToBoard("server", reg);
    Serial.print(F("[WIFI] register "));
    Serial.print(ok ? F("sent: ") : F("failed: "));
    Serial.println(reg);
    if (ok && missionStartMs == 0) {
      missionStartMs = millis();
      Serial.println(F("[TIME] Mission timer started after first successful registration."));
    }
  }

  if (millis() - lastWifiStatusPrintMs >= kWifiStatusPrintMs) {
    lastWifiStatusPrintMs = millis();
    Serial.print(F("[WIFI] connected="));
    Serial.print(messenger.isConnected() ? F("YES") : F("NO"));
    Serial.print(F(" paused="));
    Serial.print(wifiPaused ? F("YES") : F("NO"));
    Serial.print(F(" timerStarted="));
    Serial.println(missionStartMs > 0 ? F("YES") : F("NO"));
  }
}

bool queryServerForTag(const char *uid, ServerReply *reply) {
  reply->valid = false;
  if (!kUseMiniMessengerServerMessages || !messengerStarted || !messenger.isConnected()) {
    Serial.print(F("[SERVER TODO] Would ask: type=isFertile team_id="));
    Serial.print(GROUP_ID);
    Serial.print(F(" board_id="));
    Serial.print(kBoardId);
    Serial.print(F(" tag_id="));
    Serial.println(uid);
    return false;
  }

  char msg[128];
  snprintf(msg, sizeof(msg), "type=isFertile team_id=%s board_id=%s tag_id=%s", GROUP_ID, kBoardId, uid);
  strncpy(pendingServerUid, uid, sizeof(pendingServerUid) - 1);
  pendingServerUid[sizeof(pendingServerUid) - 1] = '\0';
  waitingForServerReply = true;
  lastServerReply.valid = false;

  messenger.sendToBoard("server", msg);
  Serial.print(F("[SERVER] sent "));
  Serial.println(msg);

  const uint32_t start = millis();
  while (millis() - start < kServerReplyTimeoutMs) {
    serviceBackground();
    if (!waitingForServerReply && lastServerReply.valid) {
      *reply = lastServerReply;
      Serial.print(F("[SERVER] reply fertile="));
      Serial.print(reply->fertile ? F("true") : F("false"));
      Serial.print(F(" planted="));
      Serial.print(reply->planted ? F("true") : F("false"));
      Serial.print(F(" cell="));
      printCell(reply->cell);
      Serial.println();
      return true;
    }
    delay(10);
  }

  waitingForServerReply = false;
  Serial.println(F("[SERVER] isFertile reply timeout."));
  return false;
}

void notifySeedPlanted(const char *uid, Cell cell) {
  char msg[128];
  snprintf(msg, sizeof(msg), "type=seedPlanted team_id=%s board_id=%s tag_id=%s", GROUP_ID, kBoardId, uid);

  Serial.print(F("[SERVER] seed planted at "));
  printCell(cell);
  Serial.print(F(": "));
  Serial.println(msg);

  if (kUseMiniMessengerServerMessages && messengerStarted && messenger.isConnected()) {
    messenger.sendToBoard("server", msg);
  } else {
    Serial.println(F("[SERVER TODO] MiniMessenger unavailable; printed seedPlanted only."));
  }
}

// ---------------------------------------------------------------------------
// Pause, timing, and serial debug
// ---------------------------------------------------------------------------
void triggerEmergencyReturn(const __FlashStringHelper *reason) {
  if (missionMode == MissionMode::ReturnHome || missionMode == MissionMode::Complete) return;
  Serial.print(F("[RETURN] emergency return triggered: "));
  Serial.println(reason);
  emergencyReturnRequested = true;
}

void handleMechanicalPauseButton() {
  static bool lastReading = HIGH;
  const bool reading = killPressedRaw();
  if (reading == LOW && lastReading == HIGH && millis() - lastButtonToggleMs > kButtonDebounceMs) {
    lastButtonToggleMs = millis();
    mechanicalPaused = !mechanicalPaused;
    stopMotors();
    Serial.print(F("[PAUSE] mechanical toggle -> "));
    Serial.println(mechanicalPaused ? F("PAUSED") : F("RUNNING"));
  }
  lastReading = reading;
}

void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  String lower = line;
  lower.toLowerCase();

  if (lower == "pause") {
    mechanicalPaused = true;
    stopMotors();
  } else if (lower == "resume") {
    mechanicalPaused = false;
  } else if (lower == "return") {
    triggerEmergencyReturn(F("serial command"));
  } else if (lower.startsWith("rescue ")) {
    const String cellText = line.substring(7);
    if (cellText.length() >= 2) {
      rescueTarget.row = toupper(cellText[0]) - 'A' + 1;
      rescueTarget.col = cellText.substring(1).toInt();
      rescuePending = validCell(rescueTarget);
      Serial.print(F("[SERIAL] rescue target set to "));
      printCell(rescueTarget);
      Serial.println();
    }
  } else if (lower == "brightest") {
    printBrightestLight();
  } else if (lower == "show") {
    Serial.print(F("[SHOW] mode="));
    Serial.print(modeName(missionMode));
    Serial.print(F(" cell="));
    printCell(currentCell);
    Serial.print(F(" heading="));
    Serial.print(headingName(heading));
    Serial.print(F(" routeIndex="));
    Serial.print(routeIndex);
    Serial.print(F(" seeds="));
    Serial.println(seedsPlanted);
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
  }
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

uint32_t missionElapsedMs() {
  if (missionStartMs == 0) return 0;
  return millis() - missionStartMs;
}

void checkMissionTimer() {
  if (missionStartMs == 0) return;
  const uint32_t elapsed = missionElapsedMs();
  if (!brightestReported && elapsed >= kMissionLimitMs - kBrightestReportReserveMs) {
    brightestReported = true;
    printBrightestLight();
  }
  if (elapsed >= kMissionLimitMs - kEmergencyReturnReserveMs) {
    triggerEmergencyReturn(F("40 second reserve"));
  }
}

void serviceBackground() {
  handleSerialCommands();
  handleMechanicalPauseButton();
  updateWifi();
  updatePixels();
  updateImu();
  checkMissionTimer();
}

// ---------------------------------------------------------------------------
// BFS and route helpers
// ---------------------------------------------------------------------------
uint8_t cellIndex(Cell cell) {
  return (cell.row - 1) * 9 + (cell.col - 1);
}

Cell cellFromIndex(uint8_t idx) {
  return {static_cast<uint8_t>(idx / 9 + 1), static_cast<uint8_t>(idx % 9 + 1)};
}

bool cellBlocked(Cell cell) {
  return validCell(cell) && obstacleGrid[cell.row][cell.col];
}

bool planBfsPath(Cell start, Cell goal) {
  if (!validCell(start) || !validCell(goal) || cellBlocked(goal)) return false;

  bool visited[81] = {};
  int8_t parent[81];
  uint8_t queue[81];
  for (uint8_t i = 0; i < 81; i++) parent[i] = -1;

  uint8_t head = 0;
  uint8_t tail = 0;
  const uint8_t s = cellIndex(start);
  const uint8_t g = cellIndex(goal);
  queue[tail++] = s;
  visited[s] = true;

  while (head < tail) {
    const uint8_t curIdx = queue[head++];
    const Cell cur = cellFromIndex(curIdx);
    if (curIdx == g) break;

    const int8_t dr[4] = {-1, 0, 1, 0};
    const int8_t dc[4] = {0, 1, 0, -1};
    for (uint8_t i = 0; i < 4; i++) {
      Cell nxt = {static_cast<uint8_t>(cur.row + dr[i]), static_cast<uint8_t>(cur.col + dc[i])};
      if (!validCell(nxt) || cellBlocked(nxt)) continue;
      const uint8_t ni = cellIndex(nxt);
      if (visited[ni]) continue;
      visited[ni] = true;
      parent[ni] = curIdx;
      queue[tail++] = ni;
    }
  }

  if (!visited[g]) return false;

  Cell reversePath[81];
  uint8_t count = 0;
  int idx = g;
  while (idx >= 0 && count < 81) {
    reversePath[count++] = cellFromIndex(idx);
    if (idx == s) break;
    idx = parent[idx];
  }

  plannedPathLength = count;
  plannedPathIndex = 0;
  for (uint8_t i = 0; i < count; i++) {
    plannedPath[i] = reversePath[count - 1 - i];
  }
  usingBfsPath = plannedPathLength >= 2;

  Serial.print(F("[BFS] path "));
  printCell(start);
  Serial.print(F(" -> "));
  printCell(goal);
  Serial.print(F(" len="));
  Serial.println(plannedPathLength);
  return usingBfsPath;
}

int findRouteIndex(Cell cell, uint8_t startIndex) {
  for (uint8_t i = startIndex; i < kFixedRouteLength; i++) {
    if (sameCell(kFixedRoute[i], cell)) return i;
  }
  return -1;
}

Cell nextFixedRouteGoal() {
  for (uint8_t i = routeIndex + 1; i < kFixedRouteLength; i++) {
    if (!cellBlocked(kFixedRoute[i])) return kFixedRoute[i];
  }
  return kFinishCell;
}

Cell currentNavigationTarget() {
  if (missionMode == MissionMode::ReturnHome) return kFinishCell;
  return nextFixedRouteGoal();
}

bool nextNavigationCell(Cell *out) {
  if (usingBfsPath && plannedPathIndex + 1 < plannedPathLength) {
    *out = plannedPath[plannedPathIndex + 1];
    return true;
  }

  if (missionMode == MissionMode::ReturnHome) {
    if (sameCell(currentCell, kFinishCell)) return false;
    if (!planBfsPath(currentCell, kFinishCell)) return false;
    *out = plannedPath[1];
    return true;
  }

  if (routeIndex + 1 < kFixedRouteLength) {
    *out = kFixedRoute[routeIndex + 1];
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Obstacle and RFID-centered recovery
// ---------------------------------------------------------------------------
void updateRouteProgressFromCell(Cell cell);
void handleRfidEvent(const char *uid);

const __FlashStringHelper *wallSideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

void resetLineControllerHistory() {
  lastLineError = 0;
  lastSeenLineError = 0;
  lastLineSeenMs = millis();
}

void markFrontCellBlocked() {
  const Cell blocked = stepCell(currentCell, heading);
  if (validCell(blocked) && !sameCell(blocked, currentCell)) {
    obstacleGrid[blocked.row][blocked.col] = true;
    Serial.print(F("[OBSTACLE] marked blocked cell "));
    printCell(blocked);
    Serial.println();
  }
}

float readSideSonarMm(WallSide side, bool *validOut) {
  const float mm = side == WallSide::Left
                       ? readSonarMm(kLeftTrigPin, kLeftEchoPin)
                       : readSonarMm(kRightTrigPin, kRightEchoPin);
  const bool valid = sonarReadingValid(mm);
  if (validOut) *validOut = valid;
  return valid ? mm : -1.0f;
}

float turnSignForSwerve(WallSide swerveDirection) {
  return swerveDirection == WallSide::Left ? 1.0f : -1.0f;
}

bool obstacleSideCleared(float currentMm, float referenceMm) {
  if (referenceMm < 0.0f) return false;
  if (currentMm < 0.0f) return true;
  return currentMm > referenceMm + kAvoidanceClearedSideDistanceIncreaseMm;
}

bool obstacleStillBeside(float currentMm, float referenceMm) {
  if (referenceMm < 0.0f || currentMm < 0.0f) return false;
  if (absFloat(currentMm - referenceMm) <= kAvoidanceSameSideDistanceToleranceMm) return true;
  return currentMm <= referenceMm + kAvoidanceClearedSideDistanceIncreaseMm;
}

void chooseSwerveDirection(WallSide *swerveDirection, WallSide *obstacleFacingSide) {
  bool leftValid = false;
  bool rightValid = false;
  const float leftMm = readSideSonarMm(WallSide::Left, &leftValid);
  const float rightMm = readSideSonarMm(WallSide::Right, &rightValid);

  if (leftValid && rightValid) {
    *swerveDirection = leftMm > rightMm ? WallSide::Left : WallSide::Right;
  } else if (leftValid) {
    *swerveDirection = WallSide::Left;
  } else if (rightValid) {
    *swerveDirection = WallSide::Right;
  } else {
    *swerveDirection = WallSide::Left;
  }

  *obstacleFacingSide = *swerveDirection == WallSide::Left ? WallSide::Right : WallSide::Left;

  Serial.print(F("[SWERVE] direction="));
  Serial.print(wallSideName(*swerveDirection));
  Serial.print(F(" obstacleFacing="));
  Serial.print(wallSideName(*obstacleFacingSide));
  Serial.print(F(" leftMm="));
  Serial.print(leftMm, 1);
  Serial.print(F(" rightMm="));
  Serial.println(rightMm, 1);
}

void updateHeadingAfterRelativeTurn(float degrees) {
  if (absFloat(degrees) < 45.0f) return;

  int headingValue = static_cast<int>(heading);
  if (absFloat(degrees) >= 135.0f) {
    headingValue = (headingValue + 2) % 4;
  } else if (degrees > 0.0f) {
    headingValue = (headingValue + 3) % 4; // Positive IMU target is a left turn.
  } else {
    headingValue = (headingValue + 1) % 4;
  }
  heading = static_cast<Heading>(headingValue);
}

void turnRelativeAndTrackHeading(float degrees, float *headingOffsetFromOriginalDeg) {
  if (absFloat(degrees) <= kTurnToleranceDeg) return;
  turnDegreesImu(degrees);
  updateHeadingAfterRelativeTurn(degrees);
  *headingOffsetFromOriginalDeg += degrees;
}

void initializeAvoidanceRfidDebounce() {
  strncpy(lastAvoidanceUid, lastHandledUid, sizeof(lastAvoidanceUid) - 1);
  lastAvoidanceUid[sizeof(lastAvoidanceUid) - 1] = '\0';
  lastAvoidanceRfidMs = millis();
}

bool pollAvoidanceGridNode(Cell *cellOut, char *uidOut, size_t uidOutSize) {
  String uid;
  if (!pollRfid(&uid)) return false;

  char uidBuf[12];
  uid.toCharArray(uidBuf, sizeof(uidBuf));

  Cell cell = {0, 0};
  if (!cellForUid(uidBuf, &cell)) {
    Serial.print(F("[AVOID RFID] unknown UID="));
    Serial.println(uid);
    return false;
  }

  if (strcmp(uidBuf, lastAvoidanceUid) == 0 &&
      millis() - lastAvoidanceRfidMs < kAvoidanceRfidNodeDebounceMs) {
    return false;
  }

  strncpy(lastAvoidanceUid, uidBuf, sizeof(lastAvoidanceUid) - 1);
  lastAvoidanceUid[sizeof(lastAvoidanceUid) - 1] = '\0';
  lastAvoidanceRfidMs = millis();

  currentCell = cell;
  updateRouteProgressFromCell(cell);

  if (cellOut) *cellOut = cell;
  if (uidOut && uidOutSize > 0) {
    strncpy(uidOut, uidBuf, uidOutSize - 1);
    uidOut[uidOutSize - 1] = '\0';
  }

  Serial.print(F("[AVOID RFID] uid="));
  Serial.print(uidBuf);
  Serial.print(F(" cell="));
  printCell(cell);
  Serial.print(F(" routeIndex="));
  Serial.println(routeIndex);
  return true;
}

bool avoidanceMovementSafetyOk(const char *phase, uint32_t startedMs, bool checkFrontSafety) {
  serviceBackground();
  if (!waitWhilePausedOrReturnRequested()) {
    stopMotors();
    Serial.print(F("[AVOID] aborted phase="));
    Serial.println(phase);
    return false;
  }

  if (checkFrontSafety) {
    const float frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
    if (sonarReadingValid(frontMm) && frontMm <= kAvoidanceFrontSafetyStopMm) {
      stopMotors();
      Serial.print(F("[AVOID] front safety stop phase="));
      Serial.print(phase);
      Serial.print(F(" frontMm="));
      Serial.println(frontMm, 1);
      return false;
    }
  }

  if (millis() - startedMs > kAvoidanceMovementTimeoutMs) {
    stopMotors();
    Serial.print(F("[AVOID] movement timeout phase="));
    Serial.println(phase);
    return false;
  }

  return true;
}

bool driveAvoidanceDistanceMm(float distanceMm, int speed, bool checkFrontSafety, const char *phase) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t startedMs = millis();

  while (true) {
    if (!avoidanceMovementSafetyOk(phase, startedMs, checkFrontSafety)) return false;

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;

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

bool followLineToNextAvoidanceNode(const char *phase, Cell *nodeOut, char *uidOut, size_t uidOutSize) {
  const uint32_t startedMs = millis();
  while (true) {
    if (!avoidanceMovementSafetyOk(phase, startedMs, true)) return false;

    if (pollAvoidanceGridNode(nodeOut, uidOut, uidOutSize)) {
      stopMotors();
      return true;
    }

    lineFollowStep(kAvoidanceManoeuvreSpeed);
    delay(10);
  }
}

bool driveStraightToNextAvoidanceNode(const char *phase, Cell *nodeOut, char *uidOut, size_t uidOutSize) {
  const uint32_t startedMs = millis();
  while (true) {
    if (!avoidanceMovementSafetyOk(phase, startedMs, true)) return false;

    if (pollAvoidanceGridNode(nodeOut, uidOut, uidOutSize)) {
      stopMotors();
      return true;
    }

    setTank(kAvoidanceManoeuvreSpeed, kAvoidanceManoeuvreSpeed);
    delay(10);
  }
}

void printAvoidanceSideSample(const __FlashStringHelper *label, uint8_t count, float currentMm) {
  Serial.print(label);
  Serial.print(F(" count="));
  Serial.print(count);
  Serial.print(F(" sideMm="));
  Serial.println(currentMm, 1);
}

bool followLineUntilObstacleSideCleared(
    uint8_t *counter,
    uint8_t maxCounter,
    WallSide obstacleFacingSide,
    float referenceObstacleSideMm,
    const char *phase) {
  const uint32_t startedMs = millis();

  while (true) {
    if (!avoidanceMovementSafetyOk(phase, startedMs, true)) return false;

    Cell node = {0, 0};
    char uid[12] = "";
    if (pollAvoidanceGridNode(&node, uid, sizeof(uid))) {
      (*counter)++;
      stopMotors();

      const float currentSideMm = readSideSonarMm(obstacleFacingSide, nullptr);
      printAvoidanceSideSample(F("[NODE]"), *counter, currentSideMm);

      if (obstacleSideCleared(currentSideMm, referenceObstacleSideMm)) {
        return driveAvoidanceDistanceMm(
            kAvoidanceRfidCenteringOffsetMm,
            kAvoidanceManoeuvreSpeed,
            true,
            "center_on_clear_node");
      }

      if (!obstacleStillBeside(currentSideMm, referenceObstacleSideMm)) {
        Serial.println(F("[SIDE] ambiguous side distance; continuing cautiously."));
      }

      if (*counter >= maxCounter) {
        Serial.print(F("[AVOID] side clear not found phase="));
        Serial.println(phase);
        return false;
      }
    }

    lineFollowStep(kAvoidanceManoeuvreSpeed);
    delay(10);
  }
}

bool returnToOriginalLineOffset(uint8_t gridSpacesAway) {
  uint8_t returnGridSpaces = 0;
  while (returnGridSpaces < gridSpacesAway) {
    Cell node = {0, 0};
    char uid[12] = "";
    if (!driveStraightToNextAvoidanceNode("return_to_original_line", &node, uid, sizeof(uid))) {
      return false;
    }

    returnGridSpaces++;
    Serial.print(F("[RETURN] uid="));
    Serial.print(uid);
    Serial.print(F(" count="));
    Serial.print(returnGridSpaces);
    Serial.print(F("/"));
    Serial.println(gridSpacesAway);
  }

  return driveAvoidanceDistanceMm(
      kAvoidanceRfidCenteringOffsetMm,
      kAvoidanceManoeuvreSpeed,
      true,
      "center_on_original_line");
}

bool followPostObstacleNodes() {
  uint8_t postObstacleCount = 0;
  uint32_t startedMs = millis();

  while (postObstacleCount < kAvoidancePostObstacleNodeTarget) {
    if (!avoidanceMovementSafetyOk("follow_line_after_obstacle", startedMs, true)) return false;

    Cell node = {0, 0};
    char uid[12] = "";
    if (pollAvoidanceGridNode(&node, uid, sizeof(uid))) {
      postObstacleCount++;
      Serial.print(F("[POST] uid="));
      Serial.print(uid);
      Serial.print(F(" count="));
      Serial.print(postObstacleCount);
      Serial.print(F("/"));
      Serial.println(kAvoidancePostObstacleNodeTarget);
      handleRfidEvent(uid);
      startedMs = millis();
      if (postObstacleCount >= kAvoidancePostObstacleNodeTarget) break;
    }

    lineFollowStep(kAvoidanceManoeuvreSpeed);
    delay(10);
  }

  stopMotors();
  return true;
}

bool applyLineBasedObstacleAvoidance() {
  Serial.println(F("[OBSTACLE] applying line-based avoidance strategy."));
  initializeAvoidanceRfidDebounce();
  resetLineControllerHistory();

  WallSide swerveDirection = WallSide::Left;
  WallSide obstacleFacingSide = WallSide::Right;
  chooseSwerveDirection(&swerveDirection, &obstacleFacingSide);

  Cell node = {0, 0};
  char uid[12] = "";
  if (!followLineToNextAvoidanceNode("find_alignment_tag", &node, uid, sizeof(uid))) {
    return false;
  }
  if (!driveAvoidanceDistanceMm(
          kAvoidanceRfidCenteringOffsetMm,
          kAvoidanceManoeuvreSpeed,
          true,
          "center_before_swerve")) {
    return false;
  }

  float headingOffsetFromOriginalDeg = 0.0f;
  const float swerveSign = turnSignForSwerve(swerveDirection);
  turnRelativeAndTrackHeading(90.0f * swerveSign, &headingOffsetFromOriginalDeg);

  const float referenceObstacleSideMm = readSideSonarMm(obstacleFacingSide, nullptr);
  Serial.print(F("[SIDE] referenceMm="));
  Serial.println(referenceObstacleSideMm, 1);

  uint8_t gridSpacesAway = 0;
  if (!followLineToNextAvoidanceNode("move_sideways_first_node", &node, uid, sizeof(uid))) {
    return false;
  }
  if (!driveAvoidanceDistanceMm(
          kAvoidanceRfidCenteringOffsetMm,
          kAvoidanceManoeuvreSpeed,
          true,
          "center_sideways_node")) {
    return false;
  }

  gridSpacesAway++;
  float currentObstacleSideMm = readSideSonarMm(obstacleFacingSide, nullptr);
  printAvoidanceSideSample(F("[NODE] alignment"), gridSpacesAway, currentObstacleSideMm);

  if (!obstacleSideCleared(currentObstacleSideMm, referenceObstacleSideMm)) {
    if (!obstacleStillBeside(currentObstacleSideMm, referenceObstacleSideMm)) {
      Serial.println(F("[SIDE] ambiguous side distance; continuing cautiously."));
    }
    if (gridSpacesAway >= kAvoidanceMaxSidewaysGridSpaces) {
      Serial.println(F("[AVOID] side clear not found after first sideways node."));
      return false;
    }
    if (!followLineUntilObstacleSideCleared(
            &gridSpacesAway,
            kAvoidanceMaxSidewaysGridSpaces,
            obstacleFacingSide,
            referenceObstacleSideMm,
            "move_sideways_around_obstacle")) {
      return false;
    }
  }

  turnRelativeAndTrackHeading(-90.0f * swerveSign, &headingOffsetFromOriginalDeg);

  uint8_t gridSpacesPassing = 0;
  if (!followLineUntilObstacleSideCleared(
          &gridSpacesPassing,
          kAvoidanceMaxPassingGridSpaces,
          obstacleFacingSide,
          referenceObstacleSideMm,
          "pass_obstacle")) {
    return false;
  }

  turnRelativeAndTrackHeading(-90.0f * swerveSign, &headingOffsetFromOriginalDeg);

  if (!returnToOriginalLineOffset(gridSpacesAway)) {
    return false;
  }

  const float restoreTurnDeg = -headingOffsetFromOriginalDeg;
  turnRelativeAndTrackHeading(restoreTurnDeg, &headingOffsetFromOriginalDeg);
  resetLineControllerHistory();

  if (!followPostObstacleNodes()) {
    return false;
  }

  usingBfsPath = false;
  if (missionMode == MissionMode::ReturnHome && !sameCell(currentCell, kFinishCell)) {
    planBfsPath(currentCell, kFinishCell);
  }

  Serial.print(F("[OBSTACLE] avoidance complete at "));
  printCell(currentCell);
  Serial.print(F(" heading="));
  Serial.println(headingName(heading));
  return true;
}

bool reverseUntilAnyRfid(Cell *foundCell, char *foundUid, size_t foundUidSize) {
  Serial.println(F("[RECOVERY] reversing until any RFID is detected."));
  const uint32_t start = millis();
  while (millis() - start < kReverseToRfidTimeoutMs) {
    serviceBackground();
    if (!waitWhilePausedOrReturnRequested()) return false;

    String uid;
    if (pollRfid(&uid)) {
      uid.toCharArray(foundUid, foundUidSize);
      if (cellForUid(foundUid, foundCell)) {
        currentCell = *foundCell;
        Serial.print(F("[RECOVERY] found RFID "));
        Serial.print(foundUid);
        Serial.print(F(" at "));
        printCell(*foundCell);
        Serial.println();
        stopMotors();
        return true;
      }
      Serial.print(F("[RECOVERY] unknown UID while reversing: "));
      Serial.println(uid);
    }
    setTank(-kReverseSearchSpeed, -kReverseSearchSpeed);
    delay(10);
  }
  stopMotors();
  Serial.println(F("[RECOVERY] reverse RFID search timeout."));
  return false;
}

bool frontObstacleDetected() {
  const float frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  return sonarReadingValid(frontMm) && frontMm <= kFrontObstacleThresholdMm;
}

void handlePersistentFrontObstacle() {
  markFrontCellBlocked();

  if (applyLineBasedObstacleAvoidance()) {
    return;
  }

  Serial.println(F("[OBSTACLE] line-based avoidance failed; falling back to RFID reverse and BFS."));

  Cell recoveredCell = currentCell;
  char recoveredUid[12] = "";
  if (reverseUntilAnyRfid(&recoveredCell, recoveredUid, sizeof(recoveredUid))) {
    driveDistanceMm(kPlantCenterOffsetMm, kCenteringDriveSpeed);
  }

  const Cell goal = currentNavigationTarget();
  if (!planBfsPath(currentCell, goal)) {
    Serial.println(F("[BFS] no path to next goal; emergency return requested."));
    triggerEmergencyReturn(F("BFS failed"));
  }
}

bool checkAndHandleObstacle() {
  if (!frontObstacleDetected()) return false;

  const uint32_t waitMs = missionMode == MissionMode::ReturnHome ? kReturnObstacleConfirmMs : kObstacleConfirmMs;
  Serial.print(F("[OBSTACLE] front object <10cm. Stop and wait ms="));
  Serial.println(waitMs);
  stopMotors();

  const uint32_t start = millis();
  while (millis() - start < waitMs) {
    serviceBackground();
    if (!waitWhilePausedOrReturnRequested()) return true;
    delay(20);
  }

  if (frontObstacleDetected()) {
    Serial.println(F("[OBSTACLE] still blocked after wait."));
    handlePersistentFrontObstacle();
  } else {
    Serial.println(F("[OBSTACLE] cleared; continue."));
  }
  return true;
}

// ---------------------------------------------------------------------------
// Planting, rescue, and RFID event handling
// ---------------------------------------------------------------------------
bool shouldPlantAtTag(const char *uid, Cell cell) {
  if (missionMode == MissionMode::ReturnHome || seedsPlanted >= kMaxSeedsToPlant) return false;

  ServerReply reply = {};
  const bool gotReply = queryServerForTag(uid, &reply);
  if (!gotReply) {
    Serial.print(F("[PLANT] no server reply at "));
    printCell(cell);
    Serial.print(F(". plantIfNoReply="));
    Serial.println(kPlantIfNoServerReply ? F("YES") : F("NO"));
    return kPlantIfNoServerReply;
  }

  return reply.fertile && !reply.planted;
}

void plantAtCurrentTag(const char *uid, Cell cell) {
  Serial.print(F("[PLANT] eligible tag at "));
  printCell(cell);
  Serial.println(F(". Centering over hole."));

  if (!driveDistanceMm(kPlantCenterOffsetMm, kCenteringDriveSpeed)) return;
  recordLightSample(cell, uid);
  dropOneSeedNoReturn();
  seedsPlanted++;
  notifySeedPlanted(uid, cell);

  Serial.print(F("[PLANT] seedsPlanted="));
  Serial.print(seedsPlanted);
  Serial.print(F("/"));
  Serial.println(kMaxSeedsToPlant);
}

bool rescueApproachReached(Cell cell) {
  if (!rescuePending || rescueCompleted || !validCell(rescueTarget)) return false;

  const int rescueRouteIndex = findRouteIndex(rescueTarget, 0);
  if (rescueRouteIndex > 0 && routeIndex == rescueRouteIndex - 1) {
    return true;
  }

  // Fallback until the official rescue message format is known:
  // when we reach the requested column, begin rescue approach.
  return cell.col == rescueTarget.col;
}

void performRescueSequence() {
  Serial.print(F("[RESCUE] approach near target "));
  printCell(rescueTarget);
  Serial.println(F(". Slow until D31 revival OR two RFID detections."));

  rescueModeActive = true;
  rescueRfidCounter = 0;
  const uint32_t start = millis();
  while (true) {
    serviceBackground();
    if (!waitWhilePausedOrReturnRequested()) {
      rescueModeActive = false;
      return;
    }

    String uid;
    if (pollRfid(&uid)) {
      rescueRfidCounter++;
      Serial.print(F("[RESCUE] pass RFID "));
      Serial.print(rescueRfidCounter);
      Serial.print(F(" uid="));
      Serial.println(uid);
    }

    if (revivalPressed() || rescueRfidCounter >= kRescueConsecutiveRfidLimit) {
      Serial.println(F("[RESCUE] trigger reached; reverse to previous RFID."));
      break;
    }

    // Safety guard so a bad rescue message does not make the robot drive forever.
    if (millis() - start > 30000UL) {
      Serial.println(F("[RESCUE] approach timeout; abort rescue and continue route."));
      rescueModeActive = false;
      return;
    }

    setTank(kRescueSlowSpeed, kRescueSlowSpeed);
    delay(10);
  }

  Cell foundCell = currentCell;
  char foundUid[12] = "";
  if (reverseUntilAnyRfid(&foundCell, foundUid, sizeof(foundUid))) {
    driveDistanceMm(kPlantCenterOffsetMm, kCenteringDriveSpeed);
    turnDegreesImu(kRescueTurnBackDeg);
    rescueCompleted = true;
    rescuePending = false;
    greenUntilMs = millis() + kRescueGreenMs;
    Serial.println(F("[RESCUE] success; green LED for 5 seconds."));
  }
  rescueModeActive = false;
}

void updateRouteProgressFromCell(Cell cell) {
  if (usingBfsPath && plannedPathIndex + 1 < plannedPathLength && sameCell(cell, plannedPath[plannedPathIndex + 1])) {
    plannedPathIndex++;
    if (plannedPathIndex + 1 >= plannedPathLength) {
      usingBfsPath = false;
      Serial.println(F("[BFS] detour complete."));
    }
  }

  const int found = findRouteIndex(cell, routeIndex);
  if (found >= 0) {
    routeIndex = static_cast<uint8_t>(found);
  }
}

void handleRfidEvent(const char *uid) {
  if (strcmp(uid, lastHandledUid) == 0 && millis() - lastHandledRfidMs < kSameRfidCooldownMs) {
    return;
  }
  strncpy(lastHandledUid, uid, sizeof(lastHandledUid) - 1);
  lastHandledUid[sizeof(lastHandledUid) - 1] = '\0';
  lastHandledRfidMs = millis();

  Cell cell = {0, 0};
  if (!cellForUid(uid, &cell)) {
    Serial.print(F("[RFID] unknown UID="));
    Serial.println(uid);
    return;
  }

  currentCell = cell;
  updateRouteProgressFromCell(cell);

  Serial.print(F("[RFID] uid="));
  Serial.print(uid);
  Serial.print(F(" cell="));
  printCell(cell);
  Serial.print(F(" routeIndex="));
  Serial.println(routeIndex);

  if (missionMode == MissionMode::ReturnHome) {
    return;
  }

  if (rescueApproachReached(cell)) {
    performRescueSequence();
    return;
  }

  if (shouldPlantAtTag(uid, cell)) {
    plantAtCurrentTag(uid, cell);
  } else {
    Serial.println(F("[PLANT] not eligible or server unavailable; continue."));
  }
}

void pollAndHandleRfid() {
  String uid;
  if (pollRfid(&uid)) {
    char uidBuf[12];
    uid.toCharArray(uidBuf, sizeof(uidBuf));
    handleRfidEvent(uidBuf);
  }
}

// ---------------------------------------------------------------------------
// Navigation loop
// ---------------------------------------------------------------------------
void startEmergencyReturnIfRequested() {
  if (!emergencyReturnRequested) return;
  missionMode = MissionMode::ReturnHome;
  emergencyReturnRequested = false;
  usingBfsPath = false;
  planBfsPath(currentCell, kFinishCell);
  Serial.println(F("[RETURN] Active. Server/planting checks disabled."));
}

void navigationStep() {
  startEmergencyReturnIfRequested();

  if (missionMode == MissionMode::Complete) {
    stopMotors();
    return;
  }

  if (sameCell(currentCell, kFinishCell) && missionMode == MissionMode::ReturnHome) {
    Serial.println(F("[RETURN] Arrived at G9. Mission complete."));
    missionMode = MissionMode::Complete;
    stopMotors();
    return;
  }

  if (routeIndex + 1 >= kFixedRouteLength && missionMode == MissionMode::FixedRoute) {
    Serial.println(F("[ROUTE] Fixed trajectory complete."));
    missionMode = MissionMode::Complete;
    stopMotors();
    return;
  }

  if (checkAndHandleObstacle()) {
    return;
  }

  Cell nextCell = currentCell;
  if (nextNavigationCell(&nextCell)) {
    faceCell(nextCell);
  }

  const int speed = missionMode == MissionMode::ReturnHome ? kReturnBaseSpeed : kRouteBaseSpeed;
  hybridFollowStep(speed);
  pollAndHandleRfid();
}

void printStatus() {
  if (millis() - lastStatusPrintMs < 800) return;
  lastStatusPrintMs = millis();

  Serial.print(F("[STATUS] mode="));
  Serial.print(modeName(missionMode));
  Serial.print(F(" cell="));
  printCell(currentCell);
  Serial.print(F(" heading="));
  Serial.print(headingName(heading));
  Serial.print(F(" routeIndex="));
  Serial.print(routeIndex);
  Serial.print(F(" seeds="));
  Serial.print(seedsPlanted);
  Serial.print(F(" paused="));
  Serial.print(safetyPaused() ? F("YES") : F("NO"));
  Serial.print(F(" elapsedMs="));
  Serial.println(missionElapsedMs());
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

void initializeImu() {
  Serial.print(F("[IMU] start 0x"));
  Serial.println(kImuAddress, HEX);
  if (!imu.begin_I2C(kImuAddress, &Wire)) {
    Serial.println(F("[IMU] not found; turns use encoder fallback."));
    imuOk = false;
    return;
  }

  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;
  Serial.println(F("[IMU] found. Keep still for calibration."));
  delay(800);

  float sum = 0.0f;
  for (uint16_t i = 0; i < kGyroBiasSamples; i++) {
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
  Serial.print(F("[IMU] gyroZBias="));
  Serial.println(gyroZBiasDps, 4);
}

void initializePixels() {
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  if (pixelsOk) setAllPixels(RED, kLedBrightness);
}

void initializeWifi() {
  if (!kUseMiniMessengerServerMessages || strlen(WIFI_SSID) == 0) {
    Serial.println(F("[WIFI] MiniMessenger disabled or WIFI_SSID empty. Server commands will print only."));
    messengerStarted = false;
    missionStartMs = millis(); // Bench mode timer starts immediately.
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

  pinMode(kKillPin, INPUT_PULLUP);
  pinMode(kRevivalPin, INPUT_PULLUP);
  pinMode(kServoPin, OUTPUT);
  pinMode(kLdrPin, INPUT);

  pinMode(kFrontTrigPin, OUTPUT);
  pinMode(kLeftTrigPin, OUTPUT);
  pinMode(kRightTrigPin, OUTPUT);
  pinMode(kFrontEchoPin, INPUT);
  pinMode(kLeftEchoPin, INPUT);
  pinMode(kRightEchoPin, INPUT);

  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);

  analogReadResolution(12);
  Wire.begin();
  Wire1.begin();

  initializePixels();
  initializeMotoron();
  initializeRfid();
  initializeImu();
  initializeWifi();

  currentCell = kStartCell;
  heading = kInitialHeading;
  routeIndex = 0;
  lastLineSeenMs = millis();
  moveServoToAngle(kServoMinAngle);

  Serial.println(F("Fixed_Trajectory ready."));
  Serial.println(F("Route: C9 -> ... -> I1, emergency return to G9."));
  Serial.print(F("Counts/mm="));
  Serial.print(kEncoderCountsPerMm, 4);
  Serial.print(F(" motor rpm="));
  Serial.print(kMotorNoLoadRpm, 1);
  Serial.print(F(" gear=1:"));
  Serial.println(kGearRatio, 0);
  Serial.println(F("Serial commands: show | pause | resume | return | rescue C5 | brightest"));
}

void loop() {
  serviceBackground();

  if (safetyPaused()) {
    stopMotors();
    printStatus();
    delay(20);
    return;
  }

  navigationStep();
  printStatus();
  delay(10);
}
