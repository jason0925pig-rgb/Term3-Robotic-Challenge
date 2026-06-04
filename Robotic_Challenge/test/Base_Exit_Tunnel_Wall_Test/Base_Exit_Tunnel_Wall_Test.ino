#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino_Modulino.h>

#if __has_include("secrets.h")
#define HAS_TUNNEL_WIFI_SECRETS 1
#else
#define HAS_TUNNEL_WIFI_SECRETS 0
#endif

#ifndef USE_WIFI_AIRLOCK_REQUEST
#define USE_WIFI_AIRLOCK_REQUEST HAS_TUNNEL_WIFI_SECRETS
#endif

#if USE_WIFI_AIRLOCK_REQUEST
#include <MiniMessenger.h>
#include "secrets.h"
#endif

// ---------------------------------------------------------------------------
// Base_Exit_Tunnel_Wall_Test
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   This sketch is intentionally limited to the "base to arena" manoeuvre.
//   It does not plant seeds and does not execute the later fixed arena
//   trajectory. Keeping this test narrow makes it easier to inspect and tune:
//   1. wait in blue-idle mode while WiFi/MQTT registers the robot;
//   2. press D32 to start, switch top LEDs to purple, and calibrate the IMU;
//   3. follow the base line through the four scripted base turns;
//   4. after the second committed base turn, read the base-exit RFID tag and
//      request airlock A from the server;
//   5. continue to the tunnel entry, then switch to sonar wall following when
//      the solid base line disappears;
//   6. leave wall following as soon as the QTR array detects the arena grid
//      line again, then stop in DONE.
//
// Hardware summary:
//   QTR CTRL odd/even -> D2/D3
//   QTR sensors       -> D22-D30, left to right when viewed from robot front
//   Front sonar       -> trig D8, echo D11 through 5V-to-3.3V level shifter
//   Left sonar        -> trig D9, echo D12 through 5V-to-3.3V level shifter
//   Right sonar       -> trig D10, echo D13 through 5V-to-3.3V level shifter
//   RFID2             -> Wire / D20 SDA-D21 SCL, address 0x28
//   IMU ICM20948      -> Wire / D20 SDA-D21 SCL, address 0x68
//   Modulino Pixels   -> Wire / D20 SDA-D21 SCL, address 0x36
//   Motoron M3S550    -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1/M2     -> left/right motor
//   Left encoder      -> D34/D35
//   Right encoder     -> D36/D37
//   Start/stop button -> D32 to GND, INPUT_PULLUP
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Tunable mission parameters
// ---------------------------------------------------------------------------

constexpr uint32_t kSerialBaud = 115200;
constexpr bool kStartOnBoot = false;  // Keep false for lab demos; press D32 after placing the robot.

enum class RouteChoice {
  BaseA_Bottom,
  BaseB_Top
};

// The base has four route events before the tunnel handoff. For the A/lower
// path these are RIGHT, LEFT, LEFT, RIGHT. For B/upper they are mirrored.
constexpr RouteChoice kDefaultRouteChoice = RouteChoice::BaseA_Bottom;
constexpr uint8_t kRouteTurnCount = 4;

// QTR route-event geometry. The robot should roll forward after detecting a T
// or hard turn so the track/wheel axis, not the QTR board, is near the corner
// before the IMU turn starts. The 52 mm values were tuned after measuring the
// sensor board offset on the current chassis.
constexpr float kFirstTAdvanceMm = 52.0f;
constexpr float kSharpTurnAdvanceMm = 52.0f;
constexpr int kRouteAdvanceSpeed = 300;

// Airlock A RFID handling. The exit tag is physically reached after the second
// base-route turn on our selected path, so the sketch begins burst polling from
// routeTurnIndex >= 2. The 800 ms stop gives MiniMessenger time to queue the
// openAirlock request while the robot is still over the tag.
constexpr uint8_t kAirlockRfidCheckFromTurnIndex = 2;
constexpr uint32_t kStopOverAirlockTagMs = 800;
constexpr uint32_t kAirlockRequestRetryMs = 1000;

// Tunnel handoff. The base line ends at the tunnel, so six consecutive no-line
// QTR frames, or a conservative 12 s timeout after the base route, switches the
// robot from line following to wall following.
constexpr uint32_t kLineToTunnelTimeoutMs = 12000;
constexpr uint8_t kTunnelEntryNoLineFrames = 6;
constexpr int kTunnelEntryConfirmSpeed = 260;

// Door/front-sonar checks. These thresholds are deliberately wider than the
// tunnel wall target to avoid noisy HC-SR04 readings flickering the door state.
constexpr float kDoorClosedThresholdMm = 170.0f;
constexpr float kDoorOpenThresholdMm = 320.0f;
constexpr uint8_t kDoorStableFrames = 3;
constexpr bool kTreatNoEchoAsOpen = true;
constexpr uint32_t kDoorPrintIntervalMs = 250;

// Tunnel wall following. The 62.5 mm target is the intended side clearance used
// in earlier tunnel tests. The 1.40 ratio limit prevents one track from being
// commanded dramatically faster than the other when the sonar has a short spike.
constexpr uint32_t kWallTunnelTimeoutMs = 25000;
constexpr uint8_t kWallExitLineStableFrames = 2;
constexpr int kWallBaseSpeed = 520;
constexpr float kTargetWallDistanceMm = 62.5f;
constexpr int kWallMaxCorrection = 190;
constexpr float kMaxFastSlowMotorRatio = 1.40f;
constexpr float kWallKp = 1.0f;
constexpr float kWallKi = 0.0f;
constexpr float kWallKd = 0.0f;
constexpr float kWallIntegralClamp = 80.0f;
constexpr uint32_t kWallLoopDelayMs = 20;
constexpr uint32_t kWallPrintIntervalMs = 180;

enum class WallSide {
  Left,
  Right
};

constexpr WallSide kDefaultWallSide = WallSide::Left;

// Line following. All QTR raw values are normalized to 0..1000 using the saved
// calibration below, then kLineThreshold decides whether a channel sees black.
constexpr int kLineBaseSpeed = 340;
constexpr int kLineMaxCorrection = 560;
constexpr int kLineHardTurnSpeed = 450;
constexpr int kLineSearchTurnSpeed = 210;
constexpr float kLineKp = 0.80f;
constexpr float kLineKi = 0.0f;
constexpr float kLineKd = 0.08f;
constexpr uint16_t kLineThreshold = 230;
constexpr uint16_t kStrongLineThreshold = 650;
constexpr int kHardTurnError = 2500;
constexpr int kCenterRecoverError = 900;
constexpr float kLineIntegralClamp = 120.0f;
constexpr uint32_t kLinePrintIntervalMs = 150;
constexpr uint32_t kLineLoopDelayMs = 8;

// First-T detection is stricter than later turns because the first fork can look
// like a wide line. Later route events only need evidence in the scripted turn
// direction, so kSharpTurnStableFrames can stay at one frame.
constexpr uint8_t kFirstTMinActiveSensors = 7;
constexpr uint8_t kFirstTStableFrames = 3;
constexpr float kFirstTMinTravelMm = 80.0f;
constexpr uint16_t kFirstTEdgeStrongThreshold = 650;
constexpr uint16_t kFirstTMinTotalStrength = 5600;
constexpr uint8_t kFirstTMiddleMinActiveSensors = 4;
constexpr uint8_t kSharpTurnStableFrames = 1;
constexpr uint32_t kEventCooldownMs = 500;
constexpr int kReacquireTurnSpeed = 150;
constexpr uint32_t kReacquireTimeoutMs = 1400;
constexpr uint8_t kReacquireStableFrames = 3;
constexpr uint32_t kPostTurnHardIgnoreMs = 1100;
constexpr uint8_t kPostTurnHardReleaseFrames = 4;
constexpr int kPostTurnSoftErrorClamp = 650;

// QTR-HD-09RC wiring and saved calibration. These values come from the newer
// replacement QTR board. Use QTR_Raw_Read_Test when the board or mounting
// height changes, then paste the measured white-floor minimum and black-line
// maximum arrays here.
constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr uint16_t kMinUsefulCalibrationSpan = 20;
constexpr uint16_t kSavedQtrMin[9] = {98, 82, 82, 82, 86, 94, 105, 121, 142};
constexpr uint16_t kSavedQtrMax[9] = {1479, 921, 866, 797, 762, 733, 779, 901, 1177};

// Sonar pins and validity range.
constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kLeftTrigPin = 9;
constexpr uint8_t kRightTrigPin = 10;
constexpr uint8_t kFrontEchoPin = 11;
constexpr uint8_t kLeftEchoPin = 12;
constexpr uint8_t kRightEchoPin = 13;
constexpr uint32_t kEchoTimeoutUs = 12000;
constexpr float kMinValidSonarMm = 20.0f;
constexpr float kMaxValidSonarMm = 900.0f;

// Motoron, motor signs, encoders, and chassis dimensions. The encoder model is
// the Waveshare 1:150 N20 motor: one C1 rising-edge count gives 7 pulses per
// motor shaft revolution, multiplied by the 150:1 gearbox.
constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;
constexpr int kLineErrorSign = 1;
constexpr int kMaxMotorCommand = 800;
constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f;
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;
constexpr float kTurnCalibration = 1.5f;
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;
constexpr uint32_t kDriveTimeoutMs = 12000;

// IMU yaw turning. kTurnCommandSign and kImuYawSign are left as explicit
// switches because motor wiring and IMU mounting can flip the conventions.
constexpr uint8_t kImuAddress = 0x68;
constexpr uint16_t kGyroBiasSamples = 500;
constexpr uint16_t kGyroBiasSampleDelayMs = 4;
constexpr int kTurnCommandSign = 1;
constexpr int kImuYawSign = 1;
constexpr int kTurnMaxSpeed = 560;
constexpr int kTurnMinSpeed = 115;
constexpr float kTurnKp = 500.0f;
constexpr float kTurnKd = 0.0f;
constexpr float kTurnToleranceDeg = 2.0f;
constexpr float kGyroStopRateDps = 10.0f;
constexpr bool kUseImuTurnTimeout = false;
constexpr uint32_t kTurnTimeoutMs = 50000;
constexpr uint32_t kTurnPrintIntervalMs = 120;

// RFID2 and control button.
constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 20;
constexpr uint8_t kBaseExitRfidBurstPolls = 4;
constexpr uint16_t kBaseExitRfidBurstGapMs = 5;
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;
constexpr uint32_t kKillDebounceMs = 35;

#if USE_WIFI_AIRLOCK_REQUEST
constexpr const char *kBoardId = "YU7GT";
constexpr uint32_t kRegisterIntervalMs = 5000;
#endif
constexpr bool kRequireWifiBeforeCalibration = true;
constexpr uint32_t kWifiConnectTimeoutMs = 0;
constexpr uint32_t kWifiConnectPrintIntervalMs = 1000;
constexpr uint8_t kPixelBrightness = 70;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

// ---------------------------------------------------------------------------
// Types and globals
// ---------------------------------------------------------------------------

enum class MissionState {
  Idle,
  FollowBaseRoute,
  FollowLineToTunnelEntry,
  WallFollowTunnel,
  WaitExitDoorOpen,
  Done,
  Stopped
};

enum class FollowMode {
  Follow,
  HardLeft,
  HardRight,
  SearchLeft,
  SearchRight,
  Stopped
};

enum class TurnDir {
  Left,
  Right
};

struct MotorCommand {
  int left = 0;
  int right = 0;
};

struct LineReading {
  uint16_t raw[9] = {};
  uint16_t norm[9] = {};
  int position = 4000;
  int error = 0;
  uint8_t activeCount = 0;
  FollowMode mode = FollowMode::Stopped;
  bool detected = false;
};

struct DoorReading {
  float frontMm = -1.0f;
  bool valid = false;
  bool closed = false;
  bool open = false;
};

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;
ModulinoPixels pixels;

#if USE_WIFI_AIRLOCK_REQUEST
MiniMessenger messenger;
uint32_t lastRegisterMs = 0;
bool wifiInitialized = false;
#endif

uint16_t qtrRaw[9] = {};
uint16_t qtrMin[9] = {};
uint16_t qtrMax[9] = {};
uint16_t qtrNorm[9] = {};

volatile long leftCount = 0;
volatile long rightCount = 0;

MissionState missionState = kStartOnBoot ? MissionState::FollowBaseRoute : MissionState::Idle;
RouteChoice routeChoice = kDefaultRouteChoice;
WallSide wallSide = kDefaultWallSide;

bool serialStopped = false;
bool missionSensorsInitialized = false;
bool encoderInterruptsAttached = false;
bool rfidOk = false;
bool imuOk = false;
bool pixelsOk = false;
bool wifiSafetyEnabled = true;
bool stoppedByWifiKill = false;

bool airlockRequestSent = false;
bool airlockAccepted = false;
String pendingAirlockUid;
String lastUid;
uint32_t lastAirlockRequestAttemptMs = 0;

uint8_t routeTurnIndex = 0;
uint8_t eventStableCount = 0;
uint8_t tunnelEntryNoLineCount = 0;
uint8_t wallExitLineStableCount = 0;
uint8_t doorClosedStableCount = 0;
uint8_t doorOpenStableCount = 0;

uint32_t lastEventMs = 0;
uint32_t stateStartMs = 0;
uint32_t lastLinePrintMs = 0;
uint32_t lastWallPrintMs = 0;
uint32_t lastDoorPrintMs = 0;
uint32_t lastRfidPollMs = 0;
uint32_t lastTurnPrintMs = 0;
uint32_t lastImuUpdateUs = 0;
uint32_t lastKillChangeMs = 0;

bool lastKillReading = HIGH;
bool stableKillReading = HIGH;

int lastLineError = 0;
int lastSeenLineError = 0;
float lineIntegral = 0.0f;

bool postTurnHardIgnoreActive = false;
uint32_t postTurnHardIgnoreStartMs = 0;
uint8_t postTurnHardReleaseCount = 0;

float wallIntegral = 0.0f;
float lastWallErrorMm = 0.0f;
uint32_t lastWallUpdateMs = 0;

float gyroZBiasDps = 0.0f;
float gyroZDegPerSec = 0.0f;
float yawDeg = 0.0f;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

void updateWifi();
bool handleStartStopButtonEvent();
void stopMotors();
void setState(MissionState newState);
bool initializeMissionSensorsForRun();
bool servicePendingAirlockRequest();

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

long absLong(long value) {
  return value < 0 ? -value : value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

int clampMotorSpeed(int speed) {
  if (speed > kMaxMotorCommand) return kMaxMotorCommand;
  if (speed < -kMaxMotorCommand) return -kMaxMotorCommand;
  return speed;
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

bool killButtonPressedEvent() {
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

const __FlashStringHelper *stateName(MissionState state) {
  switch (state) {
    case MissionState::Idle: return F("IDLE");
    case MissionState::FollowBaseRoute: return F("FOLLOW_BASE_ROUTE");
    case MissionState::FollowLineToTunnelEntry: return F("FOLLOW_LINE_TO_TUNNEL_ENTRY");
    case MissionState::WallFollowTunnel: return F("WALL_FOLLOW_TUNNEL");
    case MissionState::WaitExitDoorOpen: return F("WAIT_EXIT_DOOR_OPEN");
    case MissionState::Done: return F("DONE");
    case MissionState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *followModeName(FollowMode mode) {
  switch (mode) {
    case FollowMode::Follow: return F("FOLLOW");
    case FollowMode::HardLeft: return F("HARD_LEFT");
    case FollowMode::HardRight: return F("HARD_RIGHT");
    case FollowMode::SearchLeft: return F("SEARCH_LEFT");
    case FollowMode::SearchRight: return F("SEARCH_RIGHT");
    case FollowMode::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *turnName(TurnDir dir) {
  return dir == TurnDir::Left ? F("LEFT") : F("RIGHT");
}

const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

const __FlashStringHelper *routeName(RouteChoice route) {
  return route == RouteChoice::BaseA_Bottom ? F("A_RIGHT") : F("B_LEFT");
}

// ---------------------------------------------------------------------------
// LEDs and state transitions
// ---------------------------------------------------------------------------

void setAllPixels(ModulinoColor color) {
  if (!pixelsOk) return;
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kPixelBrightness);
  }
  pixels.show();
}

void updatePixelsForState(MissionState state) {
  if (state == MissionState::Idle || state == MissionState::Stopped || state == MissionState::Done) {
    setAllPixels(BLUE);
  } else {
    setAllPixels(VIOLET);
  }
}

void setState(MissionState newState) {
  if (missionState == newState) return;
  missionState = newState;
  stateStartMs = millis();
  if (newState == MissionState::FollowLineToTunnelEntry) tunnelEntryNoLineCount = 0;
  if (newState == MissionState::WallFollowTunnel) wallExitLineStableCount = 0;
  doorClosedStableCount = 0;
  doorOpenStableCount = 0;
  updatePixelsForState(newState);
  Serial.print(F("[STATE] "));
  Serial.println(stateName(newState));
}

// ---------------------------------------------------------------------------
// Motoron and encoder utilities
// ---------------------------------------------------------------------------

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

void leftEncoderIsr() {
  leftCount += digitalRead(kLeftEncoderBPin) == LOW ? 1 : -1;
}

void rightEncoderIsr() {
  rightCount += digitalRead(kRightEncoderBPin) == LOW ? 1 : -1;
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
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * (absFloat(degrees) / 360.0f);
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

void initializeMotoron() {
  motoron.setBus(&Wire1);
  motoron.reinitialize();
  delay(10);
  motoron.disableCrc();
  motoron.clearResetFlag();
  motoron.setMaxAcceleration(kMotoronLeftChannel, 0);
  motoron.setMaxDeceleration(kMotoronLeftChannel, 0);
  motoron.setMaxAcceleration(kMotoronRightChannel, 0);
  motoron.setMaxDeceleration(kMotoronRightChannel, 0);
  stopMotors();
  Serial.println(F("[MOTORON] initialized on Wire1 at 0x11."));
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------

void printSettings() {
  Serial.print(F("[SETTINGS] route="));
  Serial.print(routeName(routeChoice));
  Serial.print(F(" wallSide="));
  Serial.print(sideName(wallSide));
  Serial.print(F(" airlockCheckAfterTurn="));
  Serial.print(kAirlockRfidCheckFromTurnIndex);
  Serial.print(F(" qtrThreshold="));
  Serial.print(kLineThreshold);
  Serial.print(F(" wallTargetMm="));
  Serial.println(kTargetWallDistanceMm, 1);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  start / restart   start a new base-to-arena run"));
  Serial.println(F("  stop              stop motors and enter STOPPED"));
  Serial.println(F("  route a           use A/right/lower base path"));
  Serial.println(F("  route b           use B/left/upper base path"));
  Serial.println(F("  wall left/right   select tunnel wall-following side"));
  Serial.println(F("  show              print current settings"));
  Serial.println(F("  sonar             print front/left/right sonar snapshot"));
}

void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printHelp();
  } else if (lower == "show") {
    printSettings();
  } else if (lower == "start" || lower == "restart" || lower == "resume") {
    serialStopped = false;
    stoppedByWifiKill = false;
    routeTurnIndex = 0;
    eventStableCount = 0;
    tunnelEntryNoLineCount = 0;
    wallExitLineStableCount = 0;
    airlockRequestSent = false;
    airlockAccepted = false;
    pendingAirlockUid = "";
    lastUid = "";
    lastAirlockRequestAttemptMs = 0;
    if (initializeMissionSensorsForRun()) {
      resetEncoders();
      lastEventMs = millis();
      setState(MissionState::FollowBaseRoute);
    } else {
      serialStopped = true;
      setState(MissionState::Stopped);
    }
  } else if (lower == "stop") {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
  } else if (lower == "route a" || lower == "route right" || lower == "route bottom") {
    routeChoice = RouteChoice::BaseA_Bottom;
    routeTurnIndex = 0;
    printSettings();
  } else if (lower == "route b" || lower == "route left" || lower == "route top") {
    routeChoice = RouteChoice::BaseB_Top;
    routeTurnIndex = 0;
    printSettings();
  } else if (lower == "wall left") {
    wallSide = WallSide::Left;
    printSettings();
  } else if (lower == "wall right") {
    wallSide = WallSide::Right;
    printSettings();
  } else if (lower == "sonar") {
    Serial.println(F("[SONAR] use the live mission output after START."));
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
    } else if (input.length() < 90) {
      input += c;
    }
  }
}

// ---------------------------------------------------------------------------
// IMU and encoder movement
// ---------------------------------------------------------------------------

bool initializeImuHardware() {
  Serial.print(F("[IMU] starting ICM20948 at 0x"));
  Serial.println(kImuAddress, HEX);
  if (!imu.begin_I2C(kImuAddress, &Wire)) {
    imuOk = false;
    Serial.println(F("[IMU] not found; encoder fallback will be used for turns."));
    return false;
  }
  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;
  Serial.println(F("[IMU] found."));
  return true;
}

bool calibrateImuGyroBias() {
  if (!imuOk) return false;
  stopMotors();
  Serial.println(F("[IMU] calibrating gyro Z bias; keep the robot still."));
  delay(800);
  float sum = 0.0f;
  for (uint16_t i = 0; i < kGyroBiasSamples; i++) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) return false;
    sensors_event_t accel, gyro, mag, temp;
    imu.getEvent(&accel, &gyro, &temp, &mag);
    sum += gyro.gyro.z * kRadToDeg;
    delay(kGyroBiasSampleDelayMs);
  }
  gyroZBiasDps = sum / kGyroBiasSamples;
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
  Serial.print(F("[IMU] gyroZBiasDps="));
  Serial.println(gyroZBiasDps, 4);
  return true;
}

bool updateImu() {
  if (!imuOk) return false;
  sensors_event_t accel, gyro, mag, temp;
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

void encoderOnlyTurnFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) break;
    const long averageAbs = (absLong(getLeftCount()) + absLong(getRightCount())) / 2;
    if (averageAbs >= targetCounts) break;
    if (millis() - start > kTurnTimeoutMs) break;
    setTurnCommand(direction * abs(speed));
    printTurnStatus("[EncoderTurn]", degrees, 0.0f, direction * abs(speed), targetCounts);
    delay(10);
  }
  stopMotors();
}

bool turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, kTurnMaxSpeed);
    return !serialStopped;
  }
  const long encoderTarget = turnDegreesToEncoderCounts(targetDeg);
  resetEncoders();
  resetYaw();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }
    updateImu();
    const float errorDeg = targetDeg - yawDeg;
    if (absFloat(errorDeg) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) break;
    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] IMU timeout."));
      break;
    }
    const float commandFloat = kTurnKp * errorDeg - kTurnKd * gyroZDegPerSec;
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

bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }
    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    if ((leftAbs + rightAbs) / 2 >= targetCounts) break;
    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("[DRIVE] timeout."));
      break;
    }
    int correction = static_cast<int>((leftAbs - rightAbs) * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);
    updateImu();
    delay(10);
  }
  stopMotors();
  return true;
}

// ---------------------------------------------------------------------------
// QTR line sensor
// ---------------------------------------------------------------------------

void initializeQtr() {
  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
    qtrMin[i] = kSavedQtrMin[i];
    qtrMax[i] = kSavedQtrMax[i];
  }
  Serial.println(F("[QTR] initialized with saved calibration."));
}

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
        if (digitalRead(kQtrPins[i]) == LOW) qtrRaw[i] = elapsed;
        else allDone = false;
      }
    }
  }
}

void normalizeQtrValues() {
  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t minValue = qtrMin[i];
    const uint16_t maxValue = qtrMax[i];
    const uint16_t span = maxValue > minValue ? maxValue - minValue : 0;
    if (span < kMinUsefulCalibrationSpan || qtrRaw[i] <= minValue) {
      qtrNorm[i] = 0;
    } else if (qtrRaw[i] >= maxValue) {
      qtrNorm[i] = 1000;
    } else {
      qtrNorm[i] = static_cast<uint16_t>((static_cast<uint32_t>(qtrRaw[i] - minValue) * 1000UL) / span);
    }
  }
}

uint8_t activeSensorCount(uint16_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

bool centerHasLine() {
  return qtrNorm[3] >= kLineThreshold || qtrNorm[4] >= kLineThreshold || qtrNorm[5] >= kLineThreshold;
}

bool leftOuterHasStrongLine() {
  return qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
}

bool rightOuterHasStrongLine() {
  return qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;
}

uint8_t middleActiveSensorCount() {
  uint8_t count = 0;
  for (uint8_t i = 2; i <= 6; i++) {
    if (qtrNorm[i] >= kLineThreshold) count++;
  }
  return count;
}

int computeLinePosition(bool *detectedOut) {
  normalizeQtrValues();
  uint32_t weighted = 0;
  uint32_t sum = 0;
  bool detected = false;
  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t weight = qtrNorm[i] >= kLineThreshold ? qtrNorm[i] : 0;
    if (weight > 0) {
      detected = true;
      weighted += static_cast<uint32_t>(weight) * (i * 1000);
      sum += weight;
    }
  }
  *detectedOut = detected;
  if (sum == 0) return 4000 + lastSeenLineError;
  const int position = static_cast<int>(weighted / sum);
  lastSeenLineError = position - 4000;
  return position;
}

FollowMode chooseFollowMode(const LineReading &line) {
  if (serialStopped) return FollowMode::Stopped;
  if (!line.detected) {
    lineIntegral = 0.0f;
    return lastSeenLineError < 0 ? FollowMode::SearchLeft : FollowMode::SearchRight;
  }
  if (line.error < -kHardTurnError || (leftOuterHasStrongLine() && line.error < -kCenterRecoverError)) {
    lineIntegral = 0.0f;
    return FollowMode::HardLeft;
  }
  if (line.error > kHardTurnError || (rightOuterHasStrongLine() && line.error > kCenterRecoverError)) {
    lineIntegral = 0.0f;
    return FollowMode::HardRight;
  }
  if (centerHasLine()) return FollowMode::Follow;
  return line.error < 0 ? FollowMode::HardLeft : FollowMode::HardRight;
}

LineReading readLine() {
  LineReading line;
  readQtrRcArray();
  bool detected = false;
  line.position = computeLinePosition(&detected);
  line.detected = detected;
  line.error = (line.position - 4000) * kLineErrorSign;
  line.activeCount = activeSensorCount(kLineThreshold);
  line.mode = chooseFollowMode(line);
  for (uint8_t i = 0; i < 9; i++) {
    line.raw[i] = qtrRaw[i];
    line.norm[i] = qtrNorm[i];
  }
  return line;
}

void resetLineController() {
  lineIntegral = 0.0f;
  lastLineError = 0;
}

bool isHardLineMode(FollowMode mode) {
  return mode == FollowMode::HardLeft || mode == FollowMode::HardRight;
}

MotorCommand computeLineMotorCommand(const LineReading &line) {
  MotorCommand cmd;
  switch (line.mode) {
    case FollowMode::Stopped:
      return cmd;
    case FollowMode::SearchLeft:
      cmd.left = -kLineSearchTurnSpeed;
      cmd.right = kLineSearchTurnSpeed;
      return cmd;
    case FollowMode::SearchRight:
      cmd.left = kLineSearchTurnSpeed;
      cmd.right = -kLineSearchTurnSpeed;
      return cmd;
    case FollowMode::HardLeft:
      cmd.left = -kLineHardTurnSpeed;
      cmd.right = kLineHardTurnSpeed;
      return cmd;
    case FollowMode::HardRight:
      cmd.left = kLineHardTurnSpeed;
      cmd.right = -kLineHardTurnSpeed;
      return cmd;
    case FollowMode::Follow:
    default:
      break;
  }
  lineIntegral += line.error / 1000.0f;
  lineIntegral = constrain(lineIntegral, -kLineIntegralClamp, kLineIntegralClamp);
  const int derivative = line.error - lastLineError;
  lastLineError = line.error;
  int correction = static_cast<int>(kLineKp * line.error + kLineKi * lineIntegral + kLineKd * derivative);
  correction = constrain(correction, -kLineMaxCorrection, kLineMaxCorrection);
  cmd.left = kLineBaseSpeed + correction;
  cmd.right = kLineBaseSpeed - correction;
  return cmd;
}

void applyLineCommand(const LineReading &line, const __FlashStringHelper *label) {
  const MotorCommand cmd = computeLineMotorCommand(line);
  if (line.mode == FollowMode::Stopped) stopMotors();
  else setTank(cmd.left, cmd.right);

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

// ---------------------------------------------------------------------------
// RFID and MiniMessenger
// ---------------------------------------------------------------------------

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

bool pollRfid(String *uidOut, bool force) {
  if (!rfidOk) return false;
  if (!force && millis() - lastRfidPollMs < kRfidPollIntervalMs) return false;
  lastRfidPollMs = millis();
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    *uidOut = rfidUidToString();
    rfid.PICC_HaltA();
    return true;
  }
  return false;
}

bool pollRfidBurst(String *uidOut, uint8_t attempts, uint16_t gapMs) {
  for (uint8_t i = 0; i < attempts; i++) {
    if (pollRfid(uidOut, true)) return true;
    const uint32_t waitStart = millis();
    while (millis() - waitStart < gapMs) {
      handleSerialCommands();
      updateWifi();
      if (serialStopped || handleStartStopButtonEvent()) return false;
      delay(1);
    }
  }
  return false;
}

#if USE_WIFI_AIRLOCK_REQUEST
void stopForWifiSafety(const __FlashStringHelper *reason) {
  wifiSafetyEnabled = false;
  stoppedByWifiKill = true;
  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.print(F("[WIFI SAFETY] "));
  Serial.println(reason);
}

void allowWifiSafety() {
  wifiSafetyEnabled = true;
  if (stoppedByWifiKill) {
    stoppedByWifiKill = false;
    serialStopped = false;
    Serial.println(F("[WIFI SAFETY] enable=1 received. Press D32 or send start/restart to run again."));
  } else {
    Serial.println(F("[WIFI SAFETY] enable=1 received."));
  }
}

void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  (void)metadata;
  if (length == 6) {
    if (payload[4] == 1) stopForWifiSafety(F("team emergency byte set; stopped."));
    return;
  }

  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';
  Serial.print(F("[WIFI RX] "));
  Serial.println(msg);

  if (strstr(msg, "enable=1") || strstr(msg, "type=enable")) {
    allowWifiSafety();
  } else if (strstr(msg, "enable=0")) {
    stopForWifiSafety(F("enable=0; stopped."));
  } else if (strstr(msg, "type=disable") || strstr(msg, "type=emergency")) {
    stopForWifiSafety(F("disable/emergency text message; stopped."));
  } else if (strstr(msg, "type=openAirlockReply")) {
    airlockAccepted = strstr(msg, "accepted=true") != nullptr;
    Serial.print(F("[AIRLOCK] reply accepted="));
    Serial.println(airlockAccepted ? F("YES") : F("NO"));
  }
}

void initializeWifi() {
  if (wifiInitialized) return;
  messenger.onMessage(onWifiMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kBoardId);
  wifiInitialized = true;
  lastRegisterMs = 0;
  Serial.println(F("[WIFI] MiniMessenger started."));
}

void updateWifi() {
  messenger.loop();
  if (!messenger.isConnected()) return;
  if (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs) {
    lastRegisterMs = millis();
    char reg[128];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kBoardId);
    messenger.sendToBoard("server", reg);
    Serial.print(F("[WIFI REGISTER] "));
    Serial.println(reg);
  }
}

bool waitForWifiBeforeCalibration() {
  if (!kRequireWifiBeforeCalibration) return true;
  Serial.println(F("[WIFI] waiting for MiniMessenger connection before IMU calibration and motion."));
  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;
  while (!messenger.isConnected()) {
    handleSerialCommands();
    messenger.loop();
    if (serialStopped || !wifiSafetyEnabled) return false;
    if (killButtonPressedEvent()) {
      serialStopped = true;
      stopMotors();
      setState(MissionState::Stopped);
      return false;
    }
    if (kWifiConnectTimeoutMs > 0 && millis() - start >= kWifiConnectTimeoutMs) return false;
    if (millis() - lastPrintMs >= kWifiConnectPrintIntervalMs) {
      lastPrintMs = millis();
      Serial.print(F("[WIFI] waiting... board_id="));
      Serial.print(kBoardId);
      Serial.print(F(" group="));
      Serial.println(GROUP_ID);
    }
    delay(20);
  }
  updateWifi();
  Serial.println(F("[WIFI] connected and registered before calibration."));
  return wifiSafetyEnabled;
}

bool sendAirlockOpenRequest(const String &uid, char airlock) {
  airlock = toupper(airlock);
  if (airlock != 'A' && airlock != 'B') airlock = 'A';
  if (!messenger.isConnected()) {
    Serial.print(F("[AIRLOCK] WiFi not connected; cannot request airlock "));
    Serial.println(airlock);
    return false;
  }
  char msg[160];
  snprintf(msg, sizeof(msg), "type=openAirlock team_id=%s airlock=%c tag_id=%s board_id=%s",
           GROUP_ID, airlock, uid.c_str(), kBoardId);
  messenger.sendToBoard("server", msg);
  Serial.print(F("[AIRLOCK] sent "));
  Serial.println(msg);
  return true;
}
#else
void initializeWifi() {}
void updateWifi() {}
bool waitForWifiBeforeCalibration() {
  if (!kRequireWifiBeforeCalibration) return true;
  Serial.println(F("[WIFI] secrets.h is missing; cannot run airlock test."));
  return false;
}
bool sendAirlockOpenRequest(const String &uid, char airlock) {
  Serial.print(F("[AIRLOCK] WiFi disabled. Would request airlock "));
  Serial.print(airlock);
  Serial.print(F(" with tag "));
  Serial.println(uid);
  return false;
}
#endif

bool servicePendingAirlockRequest() {
  if (pendingAirlockUid.length() == 0 || airlockRequestSent) return false;
  if (millis() - lastAirlockRequestAttemptMs < kAirlockRequestRetryMs) return false;
  lastAirlockRequestAttemptMs = millis();
  if (sendAirlockOpenRequest(pendingAirlockUid, 'A')) {
    airlockRequestSent = true;
    return true;
  }
  return false;
}

void checkBaseExitRfidAndRequestAirlock() {
  if (airlockRequestSent || pendingAirlockUid.length() > 0) {
    servicePendingAirlockRequest();
    return;
  }
  String uid;
  if (!pollRfidBurst(&uid, kBaseExitRfidBurstPolls, kBaseExitRfidBurstGapMs)) return;
  lastUid = uid;
  pendingAirlockUid = uid;
  lastAirlockRequestAttemptMs = 0;
  stopMotors();
  Serial.print(F("[AIRLOCK] base-exit RFID UID="));
  Serial.println(uid);
  servicePendingAirlockRequest();
  delay(kStopOverAirlockTagMs);
}

// ---------------------------------------------------------------------------
// Sonar, door, and wall following
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

bool isValidSonarDistance(float mm) {
  return mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm;
}

DoorReading readDoor() {
  DoorReading reading;
  reading.frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  reading.valid = isValidSonarDistance(reading.frontMm);
  reading.closed = reading.valid && reading.frontMm <= kDoorClosedThresholdMm;
  reading.open = (reading.valid && reading.frontMm >= kDoorOpenThresholdMm) ||
                 (!reading.valid && kTreatNoEchoAsOpen);
  return reading;
}

bool doorClosedStable(const DoorReading &reading) {
  if (reading.closed) {
    if (doorClosedStableCount < 255) doorClosedStableCount++;
  } else {
    doorClosedStableCount = 0;
  }
  return doorClosedStableCount >= kDoorStableFrames;
}

bool doorOpenStable(const DoorReading &reading) {
  if (reading.open) {
    if (doorOpenStableCount < 255) doorOpenStableCount++;
  } else {
    doorOpenStableCount = 0;
  }
  return doorOpenStableCount >= kDoorStableFrames;
}

void printDoorStatus(const __FlashStringHelper *label, const DoorReading &reading) {
  if (millis() - lastDoorPrintMs < kDoorPrintIntervalMs) return;
  lastDoorPrintMs = millis();
  Serial.print(label);
  Serial.print(F(" frontMm="));
  Serial.print(reading.frontMm, 1);
  Serial.print(F(" valid="));
  Serial.print(reading.valid ? F("YES") : F("NO"));
  Serial.print(F(" closedStable="));
  Serial.print(doorClosedStableCount);
  Serial.print(F(" openStable="));
  Serial.println(doorOpenStableCount);
}

void resetWallController() {
  wallIntegral = 0.0f;
  lastWallErrorMm = 0.0f;
  lastWallUpdateMs = millis();
}

int maxWallCorrectionFromRatioLimit() {
  const float ratio = kMaxFastSlowMotorRatio;
  if (ratio <= 1.0f) return 0;
  return static_cast<int>((kWallBaseSpeed * (ratio - 1.0f)) / (ratio + 1.0f));
}

int clampWallCorrection(int correction) {
  const int ratioLimit = maxWallCorrectionFromRatioLimit();
  const int limit = min(kWallMaxCorrection, ratioLimit);
  return constrain(correction, -limit, limit);
}

bool runWallFollowStep() {
  const uint32_t now = millis();
  float dt = (now - lastWallUpdateMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;
  lastWallUpdateMs = now;

  const uint8_t trig = wallSide == WallSide::Left ? kLeftTrigPin : kRightTrigPin;
  const uint8_t echo = wallSide == WallSide::Left ? kLeftEchoPin : kRightEchoPin;
  const float rawMm = readSonarMm(trig, echo);
  if (!isValidSonarDistance(rawMm)) {
    setTank(kWallBaseSpeed, kWallBaseSpeed);
    return true;
  }

  const float errorMm = rawMm - kTargetWallDistanceMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);
  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;
  int correction = static_cast<int>(kWallKp * errorMm + kWallKi * wallIntegral + kWallKd * derivative);
  correction = clampWallCorrection(correction);

  int leftSpeed;
  int rightSpeed;
  if (wallSide == WallSide::Left) {
    leftSpeed = kWallBaseSpeed + correction;
    rightSpeed = kWallBaseSpeed - correction;
  } else {
    leftSpeed = kWallBaseSpeed - correction;
    rightSpeed = kWallBaseSpeed + correction;
  }
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(sideName(wallSide));
    Serial.print(F(" rawMm="));
    Serial.print(rawMm, 1);
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

// ---------------------------------------------------------------------------
// Scripted base route
// ---------------------------------------------------------------------------

TurnDir routeTurnAt(uint8_t index) {
  if (routeChoice == RouteChoice::BaseA_Bottom) {
    if (index == 0 || index == 3) return TurnDir::Right;
    return TurnDir::Left;
  }
  if (index == 0 || index == 3) return TurnDir::Left;
  return TurnDir::Right;
}

float degreesForTurn(TurnDir dir) {
  return dir == TurnDir::Left ? 90.0f : -90.0f;
}

float currentRouteSegmentTravelMm() {
  const long averageCounts = (absLong(getLeftCount()) + absLong(getRightCount())) / 2;
  return averageCounts / (kEncoderCountsPerMm * kDistanceCalibration);
}

bool firstTDetected(const LineReading &line) {
  if (!line.detected) return false;
  if (currentRouteSegmentTravelMm() < kFirstTMinTravelMm) return false;
  if (line.activeCount < kFirstTMinActiveSensors) return false;
  if (middleActiveSensorCount() < kFirstTMiddleMinActiveSensors) return false;
  const bool leftEdgeStrong = qtrNorm[0] >= kFirstTEdgeStrongThreshold || qtrNorm[1] >= kFirstTEdgeStrongThreshold;
  const bool rightEdgeStrong = qtrNorm[7] >= kFirstTEdgeStrongThreshold || qtrNorm[8] >= kFirstTEdgeStrongThreshold;
  if (!leftEdgeStrong || !rightEdgeStrong) return false;
  uint16_t totalStrength = 0;
  for (uint8_t i = 0; i < 9; i++) totalStrength += qtrNorm[i];
  return totalStrength >= kFirstTMinTotalStrength;
}

bool expectedSharpTurnDetected(const LineReading &line, TurnDir expected) {
  if (!line.detected) return false;
  if (expected == TurnDir::Left) {
    return line.mode == FollowMode::HardLeft || (leftOuterHasStrongLine() && line.error < -kCenterRecoverError);
  }
  return line.mode == FollowMode::HardRight || (rightOuterHasStrongLine() && line.error > kCenterRecoverError);
}

void beginPostTurnHardIgnore() {
  postTurnHardIgnoreActive = true;
  postTurnHardIgnoreStartMs = millis();
  postTurnHardReleaseCount = 0;
}

void updatePostTurnHardIgnore(const LineReading &line) {
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

LineReading softenedPostTurnLine(LineReading line) {
  if (postTurnHardIgnoreActive && isHardLineMode(line.mode)) {
    line.mode = FollowMode::Follow;
    line.error = constrain(line.error, -kPostTurnSoftErrorClamp, kPostTurnSoftErrorClamp);
  }
  return line;
}

bool routeEventReady(const LineReading &line) {
  if (routeTurnIndex >= kRouteTurnCount) return false;
  if (millis() - lastEventMs < kEventCooldownMs) return false;
  if (postTurnHardIgnoreActive && isHardLineMode(line.mode)) {
    eventStableCount = 0;
    return false;
  }

  const bool firstRouteTurn = routeTurnIndex == 0;
  const bool eventNow = firstRouteTurn ? firstTDetected(line) : expectedSharpTurnDetected(line, routeTurnAt(routeTurnIndex));

  if (!eventNow) {
    eventStableCount = 0;
    return false;
  }
  if (eventStableCount < 255) eventStableCount++;
  const uint8_t requiredFrames = firstRouteTurn ? kFirstTStableFrames : kSharpTurnStableFrames;
  return eventStableCount >= requiredFrames;
}

bool reacquireLineAfterTurn(TurnDir dir) {
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

bool performRouteTurn() {
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
  lastEventMs = millis();
  beginPostTurnHardIgnore();
  return true;
}

// ---------------------------------------------------------------------------
// Mission state handlers
// ---------------------------------------------------------------------------

void updateFollowBaseRoute() {
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
      Serial.println(F("[ROUTE] complete; following line to tunnel entry."));
      setState(MissionState::FollowLineToTunnelEntry);
      return;
    }
  } else {
    const LineReading followLine = softenedPostTurnLine(line);
    applyLineCommand(followLine, F("[BASE]"));
    updatePostTurnHardIgnore(line);
  }
}

void updateFollowLineToTunnelEntry() {
  if (!airlockRequestSent) servicePendingAirlockRequest();
  if (millis() - stateStartMs > kLineToTunnelTimeoutMs) {
    stopMotors();
    Serial.println(F("[TUNNEL] line-to-tunnel timeout; starting wall follow."));
    resetWallController();
    setState(MissionState::WallFollowTunnel);
    return;
  }

  const LineReading line = readLine();
  if (!line.detected) {
    if (tunnelEntryNoLineCount < 255) tunnelEntryNoLineCount++;
    if (tunnelEntryNoLineCount >= kTunnelEntryNoLineFrames) {
      stopMotors();
      Serial.println(F("[TUNNEL] base line disappeared; starting wall follow."));
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
  applyLineCommand(line, F("[TO TUNNEL]"));
}

void updateWallFollowTunnel() {
  if (millis() - stateStartMs > kWallTunnelTimeoutMs) {
    stopMotors();
    Serial.println(F("[ARENA] wall-follow timeout; stopping for inspection."));
    setState(MissionState::Done);
    return;
  }

  const LineReading line = readLine();
  if (line.detected) {
    if (wallExitLineStableCount < 255) wallExitLineStableCount++;
    if (wallExitLineStableCount >= kWallExitLineStableFrames) {
      stopMotors();
      Serial.println(F("[ARENA] grid line detected; base-to-arena test complete."));
      setState(MissionState::Done);
      return;
    }
  } else {
    wallExitLineStableCount = 0;
  }

  const DoorReading door = readDoor();
  if (doorClosedStable(door)) {
    stopMotors();
    Serial.println(F("[DOOR] front sonar sees a closed/blocked door; waiting."));
    setState(MissionState::WaitExitDoorOpen);
    return;
  }
  printDoorStatus(F("[DOOR]"), door);
  runWallFollowStep();
}

void updateWaitExitDoorOpen() {
  const DoorReading door = readDoor();
  if (doorOpenStable(door)) {
    Serial.println(F("[DOOR] opening detected; resuming tunnel wall follow."));
    resetWallController();
    setState(MissionState::WallFollowTunnel);
    return;
  }
  printDoorStatus(F("[DOOR WAIT]"), door);
  stopMotors();
  updateWifi();
  delay(30);
}

void updateMission() {
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
    case MissionState::Done:
    case MissionState::Stopped:
    default:
      stopMotors();
      delay(20);
      break;
  }
}

// ---------------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------------

bool handleStartStopButtonEvent() {
  if (!killButtonPressedEvent()) return false;
  if (missionState == MissionState::Idle || missionState == MissionState::Stopped || missionState == MissionState::Done) {
    processSerialCommand("start");
    return false;
  }
  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.println(F("[BUTTON] press -> STOP"));
  return true;
}

void initializePixels() {
  Wire.begin();
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  setAllPixels(BLUE);
}

bool initializeMissionSensorsForRun() {
  stopMotors();
  if (!missionSensorsInitialized) {
    Serial.println(F("[INIT] initializing base-exit sensors."));
    pinMode(kFrontTrigPin, OUTPUT);
    pinMode(kLeftTrigPin, OUTPUT);
    pinMode(kRightTrigPin, OUTPUT);
    pinMode(kFrontEchoPin, INPUT);
    pinMode(kLeftEchoPin, INPUT);
    pinMode(kRightEchoPin, INPUT);
    digitalWrite(kFrontTrigPin, LOW);
    digitalWrite(kLeftTrigPin, LOW);
    digitalWrite(kRightTrigPin, LOW);

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
    initializeQtr();
    initializeRfid();
    initializeImuHardware();
    missionSensorsInitialized = true;
  }

  if (!waitForWifiBeforeCalibration()) return false;
  resetEncoders();
  resetLineController();
  resetWallController();
  lastEventMs = millis();
  lastWallUpdateMs = millis();
  if (imuOk && !calibrateImuGyroBias()) return false;
  return !serialStopped;
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);
  Serial.println(F("=== Base_Exit_Tunnel_Wall_Test ==="));

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
    lastKillReading = digitalRead(kKillPin);
    stableKillReading = lastKillReading;
    lastKillChangeMs = millis();
  }

  initializePixels();
  initializeWifi();

  Wire1.begin();
  initializeMotoron();

  Serial.print(F("[INIT] encoder counts/mm="));
  Serial.println(kEncoderCountsPerMm, 4);
  printHelp();
  printSettings();

  if (kStartOnBoot) {
    processSerialCommand("start");
  } else {
    setState(MissionState::Idle);
  }
}

void loop() {
  updateMission();
}
