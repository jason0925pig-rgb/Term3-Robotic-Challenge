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
// 6_04easy
// Board: Arduino GIGA R1 WiFi
//
// Hard-coded RFID-count route. The robot does not reason about C2/B3/etc.
// It only:
//   1. follows the grid line forward,
//   2. counts RFID tags,
//   3. turns according to the scripted list below,
//   4. queries server fertility/unplanted status at every RFID,
//   5. plants up to five seeds when eligible,
//   6. sends an openAirlock request after the final scripted left turn.
//
// Safety law:
//   If front sonar sees an object within 6 cm, motors stop until clear.
//
// Start/stop:
//   D32 mechanical button starts from IDLE. During the run, each press toggles
//   pause/resume. WiFi enable=0 pauses; enable=1 resumes.
//
// LEDs:
//   Blue = idle/paused/done. Purple = waiting for WiFi, calibrating, or running.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;
constexpr const char *kBoardId = "YU7GT";

constexpr bool kRequireWifiBeforeStart = true;
constexpr bool kPlantIfNoServerReply = false;
constexpr uint8_t kMaxSeedsToPlant = 5;
constexpr uint32_t kRegisterIntervalMs = 5000;
constexpr uint32_t kWifiStatusPrintMs = 2500;
constexpr uint32_t kServerReplyTimeoutMs = 900;
constexpr uint32_t kRfidCooldownMs = 1300;
constexpr uint32_t kStatusPrintMs = 500;

// Route script: count this many RFID tags, then turn.
enum class TurnDir : uint8_t { None, Left, Right };
struct RouteStep {
  uint8_t rfidCount;
  TurnDir turnAfter;
  bool requestDoorAfterTurn;
};

constexpr RouteStep kRoute[] = {
    {1, TurnDir::Right, false}, {2, TurnDir::Left, false},
    {3, TurnDir::Left, false},  {1, TurnDir::Left, false},
    {2, TurnDir::Right, false}, {1, TurnDir::Right, false},
    {2, TurnDir::Left, false},  {1, TurnDir::Left, false},
    {3, TurnDir::Right, false}, {1, TurnDir::Right, false},
    {3, TurnDir::Left, false},  {1, TurnDir::Left, false},
    {3, TurnDir::Right, false}, {1, TurnDir::Left, true},
};
constexpr uint8_t kRouteLength = sizeof(kRoute) / sizeof(kRoute[0]);

// Line following.
constexpr int kLineBaseSpeed = 360;
constexpr int kLineMaxCorrection = 560;
constexpr float kLineKp = 0.80f;
constexpr float kLineKd = 0.08f;
constexpr uint16_t kLineThreshold = 230;
constexpr uint32_t kLinePrintMs = 160;
constexpr uint32_t kLineLoopDelayMs = 8;

// If the line disappears, keep driving straight instead of searching.
constexpr int kNoLineStraightSpeed = 330;

// Front obstacle stop.
constexpr float kFrontObstacleStopMm = 60.0f;
constexpr uint32_t kObstaclePrintMs = 300;

// Planting.
constexpr float kPlantCenterOffsetMm = 75.0f;
constexpr int kPlantOffsetSpeed = 360;
constexpr uint32_t kDriveDistanceTimeoutMs = 10000;

// Motoron and encoders.
constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;
constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;

// Waveshare DCGM-N20-12V-EN-200RPM, 1:150 gearbox, 7 C1 rising pulses per motor rev.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f;
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;
constexpr float kTurnCalibration = 1.5f;

// IMU turn controller. Positive degrees = left turn.
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
constexpr uint32_t kTurnTimeoutMs = 50000;
constexpr bool kUseTurnTimeout = false;
constexpr int kEncoderTurnSpeed = 360;
constexpr float kTurnBalanceKp = 0.35f;
constexpr int kMaxTurnBalanceCorrection = 80;

// QTR-HD-09RC.
constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};
constexpr uint16_t kMinUsefulCalibrationSpan = 20;

// RFID, sonar, servo, button, pixels.
constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 45;

constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kFrontEchoPin = 11;
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
constexpr bool kResetServoAtStartup = true;

constexpr uint8_t kKillPin = 32;
constexpr uint32_t kKillDebounceMs = 35;
constexpr uint8_t kPixelBrightness = 70;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

// ---------------------------------------------------------------------------
// Types / globals
// ---------------------------------------------------------------------------
enum class MissionState : uint8_t {
  Idle,
  WaitingWifi,
  RunningRoute,
  SendingDoor,
  AfterDoorForward,
  Paused,
  Done,
};

struct LineReading {
  uint16_t raw[9];
  uint16_t norm[9];
  bool detected;
  int position;
  int error;
  uint8_t activeCount;
};

struct CellStatus {
  bool valid;
  bool fertile;
  bool planted;
};

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;
MiniMessenger messenger;
ModulinoPixels pixels;

volatile long leftCount = 0;
volatile long rightCount = 0;

MissionState state = MissionState::Idle;
MissionState stateBeforePause = MissionState::Idle;

bool rfidOk = false;
bool imuOk = false;
bool pixelsOk = false;
bool messengerStarted = false;
bool wifiSafetyEnabled = true;
bool mechanicalPaused = false;
bool routeStarted = false;
bool doorRequestSent = false;

uint8_t routeIndex = 0;
uint8_t segmentRfidCount = 0;
uint8_t seedsPlanted = 0;
int currentServoAngle = kServoMinAngle;

String lastUid;
uint32_t lastUidMs = 0;
uint32_t lastRfidPollMs = 0;
uint32_t lastRegisterMs = 0;
uint32_t lastWifiStatusMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastLinePrintMs = 0;
uint32_t lastObstaclePrintMs = 0;

int lastLineError = 0;
float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;

bool lastKillReading = HIGH;
bool stableKillReading = HIGH;
uint32_t lastKillChangeMs = 0;

bool waitingForCellStatus = false;
CellStatus lastCellStatus = {};

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------
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

const __FlashStringHelper *stateName(MissionState s) {
  switch (s) {
    case MissionState::Idle: return F("IDLE");
    case MissionState::WaitingWifi: return F("WAITING_WIFI");
    case MissionState::RunningRoute: return F("RUNNING_ROUTE");
    case MissionState::SendingDoor: return F("SENDING_DOOR");
    case MissionState::AfterDoorForward: return F("AFTER_DOOR_FORWARD");
    case MissionState::Paused: return F("PAUSED");
    case MissionState::Done: return F("DONE");
    default: return F("UNKNOWN");
  }
}

const __FlashStringHelper *turnName(TurnDir dir) {
  switch (dir) {
    case TurnDir::Left: return F("LEFT");
    case TurnDir::Right: return F("RIGHT");
    default: return F("NONE");
  }
}

float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float angleDiff(float target, float current) {
  return normalizeAngle(target - current);
}

long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

// ---------------------------------------------------------------------------
// Hardware control
// ---------------------------------------------------------------------------
void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
}

long getLeftCount() {
  noInterrupts();
  const long value = leftCount;
  interrupts();
  return value;
}

long getRightCount() {
  noInterrupts();
  const long value = rightCount;
  interrupts();
  return value;
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

void setTurnCommand(int command) {
  command = clampMotorSpeed(command);
  setTank(-command, command);
}

float readSonarMm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t duration = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (duration == 0) return -1.0f;
  return duration * 0.343f / 2.0f;
}

bool sonarValid(float mm) {
  return mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm;
}

bool frontObstacleNow() {
  const float frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  return sonarValid(frontMm) && frontMm <= kFrontObstacleStopMm;
}

void setAllPixels(ModulinoColor color) {
  if (!pixelsOk) return;
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kPixelBrightness);
  }
  pixels.show();
}

void updatePixels() {
  if (!pixelsOk) return;
  const bool active = routeStarted && state != MissionState::Idle &&
                      state != MissionState::Paused && state != MissionState::Done;
  setAllPixels(active ? VIOLET : BLUE);
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
    sendServoPulse(pulseUs);
  }
}

void moveServoToAngle(int angle) {
  currentServoAngle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  Serial.print(F("[SERVO] angle="));
  Serial.print(currentServoAngle);
  Serial.print(F(" pulseUs="));
  Serial.println(angleToPulseUs(currentServoAngle));
  holdServoAngle(currentServoAngle, kServoMoveSettleMs);
}

void dropOneSeed() {
  int nextAngle = currentServoAngle + kServoStepAngle;
  if (nextAngle > kServoMaxAngle) {
    Serial.println(F("[SERVO] 300deg reached, reset to 0 before next drop."));
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 400);
    nextAngle = kServoStepAngle;
  }

  Serial.println(F("[PLANT] drop one seed."));
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);
}

// ---------------------------------------------------------------------------
// Background service
// ---------------------------------------------------------------------------
bool motionAllowed() {
  return routeStarted && wifiSafetyEnabled && !mechanicalPaused && state != MissionState::Paused;
}

void setState(MissionState next) {
  state = next;
  updatePixels();
  Serial.print(F("[STATE] "));
  Serial.println(stateName(state));
}

void parseCellStatusReply(const char *msg) {
  if (!strstr(msg, "type=isFertileReply")) return;
  lastCellStatus.valid = true;
  lastCellStatus.fertile = strstr(msg, "fertile=true") != nullptr;
  lastCellStatus.planted = strstr(msg, "planted=true") != nullptr;
  waitingForCellStatus = false;
}

void onWifiMessage(const MessageMetadata &metadata, const uint8_t *payload, size_t length) {
  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print(F("[WIFI RX] from "));
  Serial.print(metadata.fromBoardId);
  Serial.print(F(": "));
  Serial.println(msg);

  parseCellStatusReply(msg);

  if (strstr(msg, "type=heartbeat enable=0") || strstr(msg, "enable=0") ||
      strstr(msg, "enabled=false") || strstr(msg, "type=disable") ||
      strstr(msg, "type=emergency")) {
    wifiSafetyEnabled = false;
    stopMotors();
    if (routeStarted) {
      stateBeforePause = state;
      setState(MissionState::Paused);
    }
    Serial.println(F("[WIFI SAFETY] paused by server."));
  }

  if (strstr(msg, "type=heartbeat enable=1") || strstr(msg, "enable=1") ||
      strstr(msg, "enabled=true") || strstr(msg, "type=enable")) {
    wifiSafetyEnabled = true;
    if (routeStarted && state == MissionState::Paused && !mechanicalPaused) {
      setState(stateBeforePause);
    }
    Serial.println(F("[WIFI SAFETY] enabled by server."));
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
    Serial.print(F("[WIFI REGISTER] "));
    Serial.print(reg);
    Serial.print(F(" ok="));
    Serial.println(ok ? F("YES") : F("NO"));
  }

  if (millis() - lastWifiStatusMs >= kWifiStatusPrintMs) {
    lastWifiStatusMs = millis();
    Serial.print(F("[WIFI] connected="));
    Serial.print(messenger.isConnected() ? F("YES") : F("NO"));
    Serial.print(F(" status="));
    Serial.print(WiFi.status());
    Serial.print(F(" safety="));
    Serial.println(wifiSafetyEnabled ? F("YES") : F("NO"));
  }
}

void handleKillButton() {
  const bool reading = digitalRead(kKillPin);
  if (reading != lastKillReading) {
    lastKillReading = reading;
    lastKillChangeMs = millis();
  }

  if (millis() - lastKillChangeMs < kKillDebounceMs) return;
  if (reading == stableKillReading) return;
  stableKillReading = reading;

  if (stableKillReading != LOW) return;

  if (!routeStarted || state == MissionState::Idle || state == MissionState::Done) {
    routeStarted = true;
    mechanicalPaused = false;
    wifiSafetyEnabled = true;
    routeIndex = 0;
    segmentRfidCount = 0;
    seedsPlanted = 0;
    doorRequestSent = false;
    lastUid = "";
    resetEncoders();
    setState(MissionState::WaitingWifi);
    Serial.println(F("[BUTTON] start requested."));
    return;
  }

  if (state == MissionState::Paused) {
    mechanicalPaused = false;
    setState(stateBeforePause);
    Serial.println(F("[BUTTON] resume."));
  } else {
    mechanicalPaused = true;
    stateBeforePause = state;
    stopMotors();
    setState(MissionState::Paused);
    Serial.println(F("[BUTTON] pause."));
  }
}

void serviceBackground() {
  handleKillButton();
  updateWifi();
  updateImu();
  updatePixels();
}

bool waitWhilePaused() {
  while (routeStarted && (state == MissionState::Paused || !wifiSafetyEnabled || mechanicalPaused)) {
    stopMotors();
    serviceBackground();
    delay(20);
  }
  return routeStarted;
}

bool waitForFrontObstacleClear() {
  if (!frontObstacleNow()) return true;

  stopMotors();
  Serial.println(F("[OBSTACLE] front sonar <= 6 cm. Waiting until clear."));
  while (frontObstacleNow()) {
    serviceBackground();
    waitWhilePaused();
    if (millis() - lastObstaclePrintMs >= kObstaclePrintMs) {
      lastObstaclePrintMs = millis();
      Serial.println(F("[OBSTACLE] still blocked."));
    }
    delay(30);
  }
  Serial.println(F("[OBSTACLE] clear. Continue."));
  return true;
}

// ---------------------------------------------------------------------------
// IMU and turns
// ---------------------------------------------------------------------------
void updateImu() {
  if (!imuOk) return;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  const uint32_t nowUs = micros();
  if (lastImuUpdateUs == 0) {
    lastImuUpdateUs = nowUs;
    return;
  }

  float dt = (nowUs - lastImuUpdateUs) / 1000000.0f;
  lastImuUpdateUs = nowUs;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  gyroZDegPerSec = gyro.gyro.z * kRadToDeg - gyroZBiasDps;
  yawDeg = normalizeAngle(yawDeg + gyroZDegPerSec * dt * kImuYawSign);
}

bool calibrateImu() {
  Serial.print(F("[IMU] start address 0x"));
  Serial.println(kImuAddress, HEX);
  if (!imu.begin_I2C(kImuAddress, &Wire)) {
    Serial.println(F("[IMU] not found. Turns will use encoder fallback."));
    imuOk = false;
    return false;
  }

  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;

  Serial.println(F("[IMU] found. Keep robot still for gyro bias calibration."));
  delay(600);
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
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
  Serial.print(F("[IMU] gyroZBias="));
  Serial.println(gyroZBiasDps, 4);
  return true;
}

bool turnDegreesEncoder(float degrees) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();

  Serial.print(F("[TURN ENC] degrees="));
  Serial.print(degrees);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  while (true) {
    serviceBackground();
    if (!waitWhilePaused()) return false;
    waitForFrontObstacleClear();

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long avg = (leftAbs + rightAbs) / 2;
    if (avg >= targetCounts) break;
    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN ENC] timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kTurnBalanceKp);
    correction = constrain(correction, -kMaxTurnBalanceCorrection, kMaxTurnBalanceCorrection);
    const int leftMag = constrain(kEncoderTurnSpeed - correction, kTurnMinSpeed, kTurnMaxSpeed);
    const int rightMag = constrain(kEncoderTurnSpeed + correction, kTurnMinSpeed, kTurnMaxSpeed);
    setTank(-direction * leftMag, direction * rightMag);
    delay(10);
  }
  stopMotors();
  return true;
}

bool turnDegreesImu(float degrees) {
  if (!imuOk) {
    return turnDegreesEncoder(degrees);
  }

  const float target = normalizeAngle(yawDeg + degrees);
  const uint32_t start = millis();
  Serial.print(F("[TURN IMU] degrees="));
  Serial.print(degrees);
  Serial.print(F(" startYaw="));
  Serial.print(yawDeg, 1);
  Serial.print(F(" targetYaw="));
  Serial.println(target, 1);

  while (true) {
    serviceBackground();
    if (!waitWhilePaused()) return false;
    waitForFrontObstacleClear();

    const float err = angleDiff(target, yawDeg);
    if (absFloat(err) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      break;
    }
    if (kUseTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN IMU] timeout."));
      break;
    }

    const float pd = kTurnKp * err - kTurnKd * gyroZDegPerSec;
    int command = static_cast<int>(pd);
    command = constrain(command, -kTurnMaxSpeed, kTurnMaxSpeed);
    if (command > 0) command = max(command, kTurnMinSpeed);
    if (command < 0) command = min(command, -kTurnMinSpeed);
    command *= kTurnCommandSign;
    setTurnCommand(command);
    delay(10);
  }
  stopMotors();
  Serial.print(F("[TURN IMU] finalYaw="));
  Serial.println(yawDeg, 1);
  return true;
}

bool performTurn(TurnDir dir) {
  if (dir == TurnDir::None) return true;
  const float deg = dir == TurnDir::Left ? 90.0f : -90.0f;
  Serial.print(F("[ROUTE] turn "));
  Serial.println(turnName(dir));
  stopMotors();
  delay(120);
  const bool ok = turnDegreesImu(deg);
  delay(120);
  return ok;
}

// ---------------------------------------------------------------------------
// QTR line following
// ---------------------------------------------------------------------------
void readQtrRaw(uint16_t *raw) {
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);

  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], OUTPUT);
    digitalWrite(kQtrPins[i], HIGH);
  }
  delayMicroseconds(10);

  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
    raw[i] = kQtrTimeoutUs;
  }

  const uint32_t start = micros();
  bool running = true;
  while (running) {
    const uint32_t elapsed = micros() - start;
    if (elapsed >= kQtrTimeoutUs) break;
    running = false;
    for (uint8_t i = 0; i < 9; i++) {
      if (raw[i] == kQtrTimeoutUs) {
        if (digitalRead(kQtrPins[i]) == LOW) {
          raw[i] = elapsed;
        } else {
          running = true;
        }
      }
    }
  }
}

uint16_t normalizeQtr(uint8_t i, uint16_t raw) {
  const uint16_t minVal = kSavedQtrMin[i];
  const uint16_t maxVal = kSavedQtrMax[i];
  if (maxVal <= minVal + kMinUsefulCalibrationSpan) return 0;
  long value = map(raw, minVal, maxVal, 0, 1000);
  if (value < 0) value = 0;
  if (value > 1000) value = 1000;
  return static_cast<uint16_t>(value);
}

LineReading readLine() {
  LineReading line = {};
  readQtrRaw(line.raw);

  uint32_t weightedSum = 0;
  uint32_t total = 0;
  for (uint8_t i = 0; i < 9; i++) {
    line.norm[i] = normalizeQtr(i, line.raw[i]);
    if (line.norm[i] >= kLineThreshold) {
      line.activeCount++;
    }
    weightedSum += static_cast<uint32_t>(line.norm[i]) * i * 1000UL;
    total += line.norm[i];
  }

  line.detected = line.activeCount > 0 && total > 0;
  line.position = line.detected ? static_cast<int>(weightedSum / total) : 4000;
  line.error = line.position - 4000;
  return line;
}

void applyLineCommand(const LineReading &line) {
  if (!line.detected) {
    setTank(kNoLineStraightSpeed, kNoLineStraightSpeed);
    return;
  }

  const int derivative = line.error - lastLineError;
  lastLineError = line.error;
  int correction = static_cast<int>(kLineKp * line.error + kLineKd * derivative);
  correction = constrain(correction, -kLineMaxCorrection, kLineMaxCorrection);

  const int left = kLineBaseSpeed + correction;
  const int right = kLineBaseSpeed - correction;
  setTank(left, right);

  if (millis() - lastLinePrintMs >= kLinePrintMs) {
    lastLinePrintMs = millis();
    Serial.print(F("[LINE] active="));
    Serial.print(line.activeCount);
    Serial.print(F(" err="));
    Serial.print(line.error);
    Serial.print(F(" L="));
    Serial.print(left);
    Serial.print(F(" R="));
    Serial.println(right);
  }
}

// ---------------------------------------------------------------------------
// RFID and server
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

bool isDuplicateRecentUid(const String &uid) {
  return uid == lastUid && millis() - lastUidMs < kRfidCooldownMs;
}

bool serverOnline() {
  return messengerStarted && messenger.isConnected();
}

void sendMapUpdate(const String &uid, const CellStatus &status) {
  char msg[160];
  snprintf(msg, sizeof(msg),
           "type=mapUpdate team_id=%s board_id=%s tag_id=%s known=%s fertile=%s planted=%s",
           GROUP_ID, kBoardId, uid.c_str(),
           status.valid ? "true" : "false",
           status.fertile ? "true" : "false",
           status.planted ? "true" : "false");

  Serial.print(F("[MAP] "));
  Serial.println(msg);
  if (serverOnline()) {
    messenger.sendToBoard("server", msg);
  }
}

bool queryServerForCellStatus(const String &uid, CellStatus *statusOut) {
  statusOut->valid = false;
  statusOut->fertile = false;
  statusOut->planted = false;

  if (!serverOnline()) {
    Serial.print(F("[SERVER] offline; cannot query tag_id="));
    Serial.println(uid);
    sendMapUpdate(uid, *statusOut);
    return false;
  }

  char msg[128];
  snprintf(msg, sizeof(msg), "type=isFertile team_id=%s board_id=%s tag_id=%s",
           GROUP_ID, kBoardId, uid.c_str());
  lastCellStatus = {};
  waitingForCellStatus = true;

  const bool sent = messenger.sendToBoard("server", msg);
  Serial.print(F("[SERVER] sent "));
  Serial.print(msg);
  Serial.print(F(" ok="));
  Serial.println(sent ? F("YES") : F("NO"));
  if (!sent) {
    waitingForCellStatus = false;
    sendMapUpdate(uid, *statusOut);
    return false;
  }

  const uint32_t start = millis();
  while (millis() - start < kServerReplyTimeoutMs) {
    serviceBackground();
    if (!waitWhilePaused()) return false;
    if (!waitingForCellStatus && lastCellStatus.valid) {
      *statusOut = lastCellStatus;
      Serial.print(F("[SERVER] reply fertile="));
      Serial.print(statusOut->fertile ? F("true") : F("false"));
      Serial.print(F(" planted="));
      Serial.println(statusOut->planted ? F("true") : F("false"));
      sendMapUpdate(uid, *statusOut);
      return true;
    }
    delay(10);
  }

  waitingForCellStatus = false;
  Serial.println(F("[SERVER] isFertile reply timeout."));
  sendMapUpdate(uid, *statusOut);
  return false;
}

void notifySeedPlanted(const String &uid) {
  char msg[128];
  snprintf(msg, sizeof(msg), "type=seedPlanted team_id=%s board_id=%s tag_id=%s",
           GROUP_ID, kBoardId, uid.c_str());
  Serial.print(F("[SERVER] seedPlanted "));
  Serial.println(msg);
  if (serverOnline()) {
    messenger.sendToBoard("server", msg);
  }
}

bool sendDoorOpenRequest(const String &uid) {
  if (!serverOnline()) {
    Serial.println(F("[AIRLOCK] server offline; cannot send openAirlock yet."));
    return false;
  }

  char msg[140];
  snprintf(msg, sizeof(msg), "type=openAirlock team_id=%s airlock=A tag_id=%s board_id=%s",
           GROUP_ID, uid.length() > 0 ? uid.c_str() : "unknown", kBoardId);
  const bool ok = messenger.sendToBoard("server", msg);
  Serial.print(F("[AIRLOCK] sent "));
  Serial.print(msg);
  Serial.print(F(" ok="));
  Serial.println(ok ? F("YES") : F("NO"));
  return ok;
}

// ---------------------------------------------------------------------------
// Motion helpers
// ---------------------------------------------------------------------------
bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;
  resetEncoders();
  const uint32_t start = millis();

  Serial.print(F("[DRIVE] distanceMm="));
  Serial.print(distanceMm);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  while (true) {
    serviceBackground();
    if (!waitWhilePaused()) return false;
    waitForFrontObstacleClear();

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;

    if (millis() - start > kDriveDistanceTimeoutMs) {
      Serial.println(F("[DRIVE] timeout."));
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

bool plantAtUid(const String &uid) {
  if (seedsPlanted >= kMaxSeedsToPlant) {
    Serial.println(F("[PLANT] already planted five seeds; route continues without planting."));
    return false;
  }

  CellStatus status = {};
  const bool gotReply = queryServerForCellStatus(uid, &status);
  const bool eligible = gotReply ? (status.fertile && !status.planted) : kPlantIfNoServerReply;

  if (!eligible) {
    Serial.println(F("[PLANT] not eligible; continue route."));
    return false;
  }

  Serial.println(F("[PLANT] eligible. Drive 7.5 cm to center hole."));
  if (!driveDistanceMm(kPlantCenterOffsetMm, kPlantOffsetSpeed)) return false;
  dropOneSeed();
  seedsPlanted++;
  notifySeedPlanted(uid);
  Serial.print(F("[PLANT] seedsPlanted="));
  Serial.print(seedsPlanted);
  Serial.print(F("/"));
  Serial.println(kMaxSeedsToPlant);
  return true;
}

void handleRfidEvent(const String &uid) {
  if (isDuplicateRecentUid(uid)) return;
  lastUid = uid;
  lastUidMs = millis();

  stopMotors();
  segmentRfidCount++;
  Serial.print(F("[RFID] uid="));
  Serial.print(uid);
  Serial.print(F(" segment="));
  Serial.print(routeIndex + 1);
  Serial.print(F("/"));
  Serial.print(kRouteLength);
  Serial.print(F(" count="));
  Serial.print(segmentRfidCount);
  Serial.print(F("/"));
  Serial.println(kRoute[routeIndex].rfidCount);

  plantAtUid(uid);
}

void runRouteStep() {
  if (routeIndex >= kRouteLength) {
    setState(MissionState::AfterDoorForward);
    return;
  }

  waitForFrontObstacleClear();
  if (!waitWhilePaused()) return;

  String uid;
  if (pollRfid(&uid)) {
    handleRfidEvent(uid);

    if (segmentRfidCount >= kRoute[routeIndex].rfidCount) {
      const RouteStep step = kRoute[routeIndex];
      segmentRfidCount = 0;
      if (!performTurn(step.turnAfter)) {
        stateBeforePause = state;
        setState(MissionState::Paused);
        return;
      }

      routeIndex++;
      if (step.requestDoorAfterTurn) {
        setState(MissionState::SendingDoor);
        return;
      }

      Serial.print(F("[ROUTE] next step "));
      Serial.print(routeIndex + 1);
      Serial.print(F("/"));
      Serial.println(kRouteLength);
    }
  }

  const LineReading line = readLine();
  applyLineCommand(line);
  delay(kLineLoopDelayMs);
}

void runAfterDoorForward() {
  waitForFrontObstacleClear();
  String uid;
  if (pollRfid(&uid)) {
    handleRfidEvent(uid);
  }
  const LineReading line = readLine();
  applyLineCommand(line);
  delay(kLineLoopDelayMs);
}

void runDoorSendState() {
  stopMotors();
  if (!doorRequestSent) {
    Serial.println(F("[AIRLOCK] final scripted left turn complete. Waiting until openAirlock is sent."));
  }

  while (!doorRequestSent) {
    serviceBackground();
    if (!waitWhilePaused()) return;
    doorRequestSent = sendDoorOpenRequest(lastUid);
    if (!doorRequestSent) delay(500);
  }

  Serial.println(F("[AIRLOCK] request sent. Continue forward."));
  setState(MissionState::AfterDoorForward);
}

void printStatus() {
  if (millis() - lastStatusMs < kStatusPrintMs) return;
  lastStatusMs = millis();

  Serial.print(F("[STATUS] state="));
  Serial.print(stateName(state));
  Serial.print(F(" routeStep="));
  Serial.print(routeIndex + 1);
  Serial.print(F("/"));
  Serial.print(kRouteLength);
  Serial.print(F(" rfidInStep="));
  Serial.print(segmentRfidCount);
  Serial.print(F(" seeds="));
  Serial.print(seedsPlanted);
  Serial.print(F(" wifi="));
  Serial.print(wifiSafetyEnabled ? F("EN") : F("DIS"));
  Serial.print(F(" pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT_FOUND"));
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
  Serial.println(rfidOk ? F("OK") : F("NOT_FOUND"));
}

void initializePixels() {
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT_FOUND"));
  updatePixels();
}

void initializeWifi() {
  if (strlen(WIFI_SSID) == 0) {
    messengerStarted = false;
    Serial.println(F("[WIFI] WIFI_SSID empty; WiFi/server disabled."));
    return;
  }

  messenger.onMessage(onWifiMessage);
  messengerStarted = true;
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kBoardId);
  Serial.print(F("[WIFI] begin group="));
  Serial.print(GROUP_ID);
  Serial.print(F(" board="));
  Serial.print(kBoardId);
  Serial.print(F(" broker="));
  Serial.print(BROKER_HOST);
  Serial.print(F(":"));
  Serial.println(BROKER_PORT);
}

void startRunIfReady() {
  if (state != MissionState::WaitingWifi) return;

  if (kRequireWifiBeforeStart && (!messengerStarted || !messenger.isConnected())) {
    stopMotors();
    Serial.println(F("[START] waiting for WiFi/MQTT before motion."));
    delay(400);
    return;
  }

  calibrateImu();
  resetEncoders();
  lastLineError = 0;
  setState(MissionState::RunningRoute);
  Serial.println(F("[START] route running."));
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  pinMode(kKillPin, INPUT_PULLUP);
  lastKillReading = digitalRead(kKillPin);
  stableKillReading = lastKillReading;
  lastKillChangeMs = millis();

  pinMode(kServoPin, OUTPUT);
  pinMode(kFrontTrigPin, OUTPUT);
  pinMode(kFrontEchoPin, INPUT);
  digitalWrite(kFrontTrigPin, LOW);

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

  Wire.begin();
  Wire1.begin();
  initializePixels();
  initializeMotoron();
  initializeRfid();
  initializeWifi();

  if (kResetServoAtStartup) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 500);
  }

  setState(MissionState::Idle);
  Serial.println(F("=== 6_04easy ready ==="));
  Serial.println(F("Press D32 kill/start button once to begin. Press again to pause/resume."));
  Serial.println(F("Route script: 1R,2L,3L,1L,2R,1R,2L,1L,3R,1R,3L,1L,3R,1L+openAirlock."));
  Serial.print(F("Encoder counts/mm="));
  Serial.println(kEncoderCountsPerMm, 4);
}

void loop() {
  serviceBackground();
  printStatus();

  switch (state) {
    case MissionState::Idle:
      stopMotors();
      delay(20);
      break;

    case MissionState::WaitingWifi:
      startRunIfReady();
      break;

    case MissionState::RunningRoute:
      runRouteStep();
      break;

    case MissionState::SendingDoor:
      runDoorSendState();
      break;

    case MissionState::AfterDoorForward:
      runAfterDoorForward();
      break;

    case MissionState::Paused:
      stopMotors();
      delay(20);
      break;

    case MissionState::Done:
      stopMotors();
      delay(50);
      break;
  }
}
