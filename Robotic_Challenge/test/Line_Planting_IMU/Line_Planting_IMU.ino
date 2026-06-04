#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Line_Planting_IMU
// Board: Arduino GIGA R1 WiFi
//
// Behavior:
//   1. Follow the black line using the QTR-HD-09RC array.
//   2. When an RFID card is detected, ignore line following and drive forward
//      by kPlantingOffsetMm using the motor encoders.
//   3. Move the 300-degree seed servo forward by 60 degrees. Do not return it.
//   4. Turn left by kTurnAfterPlantDeg using the ICM20948 IMU.
//   5. Resume line following and repeat. With four RFID cards this forms a
//      square route.
//
// Hardware:
//   QTR CTRL odd/even -> D2/D3
//   QTR sensors       -> D22-D30, left to right when viewed from robot front
//   RFID              -> Wire / D20 SDA-D21 SCL, address 0x28
//   IMU ICM20948      -> Wire / D20 SDA-D21 SCL, address 0x68
//   Motoron           -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1/M2     -> left/right motor
//   Left encoder      -> D34/D35
//   Right encoder     -> D36/D37
//   Servo signal      -> D33
//   Kill button       -> D32 to GND, INPUT_PULLUP
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

// Mission sequence.
// 205 mm is the measured offset from the RFID detection point to the seed
// outlet/chassis planting centre on the current mechanical build. During this
// move, line following is deliberately disabled so the hole can be aligned by
// encoder distance instead of by the line sensor.
constexpr float kPlantingOffsetMm = 205.0f;
constexpr float kTurnAfterPlantDeg = 90.0f;     // Positive uses the current IMU left-turn convention.
constexpr uint8_t kMaxPlantingCycles = 4;       // 4 corners of a square. Set 0 for infinite repeat.
constexpr uint32_t kPauseAfterRfidMs = 250;
constexpr uint32_t kPauseAfterPlantMs = 300;
constexpr uint32_t kPauseAfterTurnMs = 500;

// Line-following control.
constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr uint16_t kMinUsefulCalibrationSpan = 20;
constexpr int kDefaultBaseSpeed = 400;
constexpr int kDefaultMaxCorrection = 600;
constexpr int kDefaultHardTurnSpeed = 500;
constexpr int kDefaultSearchTurnSpeed = 220;
constexpr float kDefaultLineKp = 0.8f;
constexpr float kDefaultLineKi = 0.0f;
constexpr float kDefaultLineKd = 0.08f;
constexpr uint16_t kDefaultLineThreshold = 230;
constexpr uint16_t kStrongLineThreshold = 650;
constexpr int kHardTurnError = 2600;
constexpr int kCenterRecoverError = 900;
constexpr uint8_t kIntegralClamp = 120;
constexpr uint32_t kLinePrintIntervalMs = 160;
constexpr uint32_t kLoopDelayMs = 8;

// Latest measured raw values from QTR_Raw_Read_Test after replacing the QTR
// sensor board. For RC QTR sensors, darker surfaces generally produce larger
// discharge timing values. These arrays map each channel independently to
// 0-1000, because each detector has a slightly different optical response.
constexpr uint16_t kSavedQtrMin[9] = {98, 82, 82, 82, 86, 94, 105, 121, 142};
constexpr uint16_t kSavedQtrMax[9] = {1479, 921, 866, 797, 762, 733, 779, 901, 1177};

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
constexpr int kLineErrorSign = 1;               // Flip to -1 if line correction steers the wrong way.
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;
constexpr int kOffsetDriveSpeed = 400;
constexpr uint32_t kDriveTimeoutMs = 12000;

// Waveshare DCGM-N20-12V-EN-200RPM motor model.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kMotorNoLoadRpm = 200.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f; // 7 PPR before gearbox, 1050 PPR after 1:150.
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;
constexpr float kTurnCalibration = 1.5f;

// IMU turn control.
constexpr uint8_t kImuAddress = 0x68;
constexpr uint16_t kGyroBiasSamples = 500;
constexpr uint16_t kGyroBiasSampleDelayMs = 4;
constexpr int kTurnCommandSign = 1;             // Flip if +90 turns right instead of left.
constexpr int kImuYawSign = 1;                  // Flip if yaw moves away from target during a turn.
constexpr int kDefaultTurnMaxSpeed = 600;
constexpr int kDefaultTurnMinSpeed = 115;
constexpr float kDefaultTurnKp = 500.0f;
constexpr float kDefaultTurnKd = 0.0f;
constexpr float kTurnToleranceDeg = 2.0f;
constexpr float kGyroStopRateDps = 10.0f;
constexpr bool kUseImuTurnTimeout = false;
constexpr uint32_t kTurnTimeoutMs = 50000;
constexpr float kEncoderBalanceKp = 0.0f;       // Keep 0 for equal and opposite motor commands.
constexpr int kMaxEncoderBalanceCorrection = 60;
constexpr uint32_t kTurnPrintIntervalMs = 120;

// DS-R005 300-degree positional servo.
constexpr uint8_t kServoPin = 33;
constexpr int kServoMinUs = 500;
constexpr int kServoMaxUs = 2500;
constexpr int kServoMinAngle = 0;
constexpr int kServoMaxAngle = 300;
constexpr int kServoStepAngle = 60;
constexpr uint32_t kServoMoveSettleMs = 600;
constexpr uint32_t kServoHoldAfterDropMs = 1200;
constexpr uint32_t kServoFrameUs = 20000;
constexpr bool kResetServoAtStartup = true;     // Seed servo starts from 0, then advances by 60 each planting.
constexpr bool kReturnServoAfterDrop = false;   // Requested behavior: do not return after each seed drop.

// Mechanical kill.
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;

uint16_t qtrRaw[9] = {};
uint16_t qtrMin[9] = {};
uint16_t qtrMax[9] = {};
uint16_t qtrNorm[9] = {};

volatile long leftCount = 0;
volatile long rightCount = 0;

float lineKp = kDefaultLineKp;
float lineKi = kDefaultLineKi;
float lineKd = kDefaultLineKd;
int baseSpeed = kDefaultBaseSpeed;
int maxCorrection = kDefaultMaxCorrection;
int hardTurnSpeed = kDefaultHardTurnSpeed;
int searchTurnSpeed = kDefaultSearchTurnSpeed;
uint16_t lineThreshold = kDefaultLineThreshold;
int lastLineError = 0;
int lastSeenLineError = 0;
float integralLineError = 0.0f;
bool lineDetected = false;

bool rfidOk = false;
bool imuOk = false;
float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;
float turnKp = kDefaultTurnKp;
float turnKd = kDefaultTurnKd;
int turnMaxSpeed = kDefaultTurnMaxSpeed;
int turnMinSpeed = kDefaultTurnMinSpeed;

uint32_t lastLinePrintMs = 0;
uint32_t lastTurnPrintMs = 0;
uint32_t lastRfidPollMs = 0;
bool serialStopped = false;
int currentServoAngle = kServoMinAngle;
uint8_t plantingCycleCount = 0;
String lastUid;
bool completeAnnounced = false;

enum class MissionState {
  FollowLine,
  DriveOffset,
  DropSeed,
  TurnLeft,
  Complete,
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

MissionState missionState = MissionState::FollowLine;

const __FlashStringHelper *missionName(MissionState state) {
  switch (state) {
    case MissionState::FollowLine: return F("FOLLOW_LINE");
    case MissionState::DriveOffset: return F("DRIVE_OFFSET");
    case MissionState::DropSeed: return F("DROP_SEED");
    case MissionState::TurnLeft: return F("TURN_LEFT");
    case MissionState::Complete: return F("COMPLETE");
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
  return static_cast<long>(abs(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

void printSettings() {
  Serial.print(F("[SETTINGS] state="));
  Serial.print(missionName(missionState));
  Serial.print(F(" cycles="));
  Serial.print(plantingCycleCount);
  Serial.print(F("/"));
  Serial.print(kMaxPlantingCycles);
  Serial.print(F(" offsetMm="));
  Serial.print(kPlantingOffsetMm, 1);
  Serial.print(F(" turnDeg="));
  Serial.print(kTurnAfterPlantDeg, 1);
  Serial.print(F(" lineP="));
  Serial.print(lineKp, 3);
  Serial.print(F(" lineD="));
  Serial.print(lineKd, 3);
  Serial.print(F(" turnP="));
  Serial.print(turnKp, 2);
  Serial.print(F(" turnD="));
  Serial.print(turnKd, 2);
  Serial.print(F(" servoAngle="));
  Serial.print(currentServoAngle);
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  stop | resume | show"));
  Serial.println(F("  linep 0.8 | lined 0.08 | base 400 | th 230"));
  Serial.println(F("  turnp 500 | turnd 0 | turnmax 600 | turnmin 115"));
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
    integralLineError = 0.0f;
    lastLineError = 0;
    missionState = MissionState::FollowLine;
    Serial.println(F("[SERIAL] resumed from line following."));
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

  if (key == "linep") {
    lineKp = value;
  } else if (key == "linei") {
    lineKi = value;
  } else if (key == "lined") {
    lineKd = value;
  } else if (key == "base") {
    baseSpeed = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "th") {
    lineThreshold = constrain(static_cast<int>(value), 0, 1000);
  } else if (key == "turnp") {
    turnKp = value;
  } else if (key == "turnd") {
    turnKd = value;
  } else if (key == "turnmax") {
    turnMaxSpeed = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "turnmin") {
    turnMinSpeed = constrain(static_cast<int>(value), 0, 800);
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
    if (input.length() < 90) {
      input += c;
    }
  }
}

void readQtrRcArray() {
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

void initializeQtrCalibration() {
  for (uint8_t i = 0; i < 9; i++) {
    qtrMin[i] = kSavedQtrMin[i];
    qtrMax[i] = kSavedQtrMax[i];
  }
  Serial.println(F("[QTR] using saved calibration."));
}

void normalizeQtrValues() {
  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t span = qtrMax[i] > qtrMin[i] ? qtrMax[i] - qtrMin[i] : 0;
    if (span < kMinUsefulCalibrationSpan || qtrRaw[i] <= qtrMin[i]) {
      qtrNorm[i] = 0;
    } else if (qtrRaw[i] >= qtrMax[i]) {
      qtrNorm[i] = 1000;
    } else {
      qtrNorm[i] = static_cast<uint16_t>(
          (static_cast<uint32_t>(qtrRaw[i] - qtrMin[i]) * 1000UL) / span);
    }
  }
}

int computeLinePosition() {
  normalizeQtrValues();

  uint32_t weighted = 0;
  uint32_t sum = 0;
  lineDetected = false;

  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t weight = qtrNorm[i] >= lineThreshold ? qtrNorm[i] : 0;
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
  return position;
}

uint8_t activeSensorCount(uint16_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

bool centerHasLine() {
  return qtrNorm[3] >= lineThreshold || qtrNorm[4] >= lineThreshold || qtrNorm[5] >= lineThreshold;
}

FollowMode chooseFollowMode(int error) {
  if (serialStopped || killPressed()) {
    return FollowMode::Stopped;
  }

  if (!lineDetected) {
    integralLineError = 0.0f;
    return lastSeenLineError < 0 ? FollowMode::SearchLeft : FollowMode::SearchRight;
  }

  const bool leftEdgeStrong = qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
  const bool rightEdgeStrong = qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;

  if (error < -kHardTurnError || (leftEdgeStrong && error < -kCenterRecoverError)) {
    integralLineError = 0.0f;
    return FollowMode::HardLeft;
  }
  if (error > kHardTurnError || (rightEdgeStrong && error > kCenterRecoverError)) {
    integralLineError = 0.0f;
    return FollowMode::HardRight;
  }
  if (centerHasLine()) {
    return FollowMode::Follow;
  }

  return error < 0 ? FollowMode::HardLeft : FollowMode::HardRight;
}

void computeLineMotorCommand(FollowMode mode, int error, int *leftSpeed, int *rightSpeed) {
  switch (mode) {
    case FollowMode::Stopped:
      *leftSpeed = 0;
      *rightSpeed = 0;
      return;
    case FollowMode::SearchLeft:
      *leftSpeed = -searchTurnSpeed;
      *rightSpeed = searchTurnSpeed;
      return;
    case FollowMode::SearchRight:
      *leftSpeed = searchTurnSpeed;
      *rightSpeed = -searchTurnSpeed;
      return;
    case FollowMode::HardLeft:
      *leftSpeed = -hardTurnSpeed;
      *rightSpeed = hardTurnSpeed;
      return;
    case FollowMode::HardRight:
      *leftSpeed = hardTurnSpeed;
      *rightSpeed = -hardTurnSpeed;
      return;
    case FollowMode::Follow:
    default:
      break;
  }

  integralLineError += error / 1000.0f;
  integralLineError = constrain(integralLineError, -static_cast<float>(kIntegralClamp), static_cast<float>(kIntegralClamp));

  const int derivative = error - lastLineError;
  lastLineError = error;

  int correction = static_cast<int>(lineKp * error + lineKi * integralLineError + lineKd * derivative);
  correction = constrain(correction, -maxCorrection, maxCorrection);

  *leftSpeed = baseSpeed + correction;
  *rightSpeed = baseSpeed - correction;
}

void printLineStatus(FollowMode mode, int position, int error, int leftSpeed, int rightSpeed) {
  if (millis() - lastLinePrintMs < kLinePrintIntervalMs) return;
  lastLinePrintMs = millis();

  Serial.print(F("[LINE] cycle="));
  Serial.print(plantingCycleCount);
  Serial.print(F(" mode="));
  Serial.print(followModeName(mode));
  Serial.print(F(" line="));
  Serial.print(lineDetected ? F("YES") : F("NO"));
  Serial.print(F(" active="));
  Serial.print(activeSensorCount(lineThreshold));
  Serial.print(F(" pos="));
  Serial.print(position);
  Serial.print(F(" err="));
  Serial.print(error);
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
    handleSerialCommands();
    if (killPressed() || serialStopped) {
      stopMotors();
      return;
    }
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

  Serial.print(F("[PLANT] seed drop #"));
  Serial.print(plantingCycleCount + 1);
  Serial.print(F(" uid="));
  Serial.println(lastUid);
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);

  if (kReturnServoAfterDrop) {
    moveServoToAngle(kServoMinAngle);
  }
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

void setTurnCommand(int signedTurnSpeed, int balanceCorrection) {
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  const int correction = constrain(balanceCorrection, -kMaxEncoderBalanceCorrection, kMaxEncoderBalanceCorrection);
  setTank(-command - correction, command + correction);
}

void printTurnStatus(const char *label, float targetDeg, float errorDeg, int command, long encoderTarget) {
  if (millis() - lastTurnPrintMs < kTurnPrintIntervalMs) return;
  lastTurnPrintMs = millis();

  const long left = getLeftCount();
  const long right = getRightCount();
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
  Serial.print(left);
  Serial.print(F(" R="));
  Serial.print(right);
  Serial.print(F(" encTarget="));
  Serial.println(encoderTarget);
}

bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("[DRIVE] ignore line, distanceMm="));
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
      Serial.println(F("[DRIVE] timeout; continuing mission from current position."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);

    if (millis() - lastLinePrintMs >= kLinePrintIntervalMs) {
      lastLinePrintMs = millis();
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

    setTurnCommand(direction * abs(speed), 0);
    printTurnStatus("[EncoderTurn]", degrees, 0.0f, direction * abs(speed), targetCounts);
    delay(10);
  }
  stopMotors();
}

bool turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, turnMaxSpeed);
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
    const float absError = absFloat(errorDeg);
    if (absError <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("[TURN] IMU target reached."));
      break;
    }
    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] IMU timeout."));
      break;
    }

    float commandFloat = turnKp * errorDeg - turnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, turnMinSpeed, turnMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;

    const long encoderDiff = absLong(getLeftCount()) - absLong(getRightCount());
    int balanceCorrection = static_cast<int>(encoderDiff * kEncoderBalanceKp);
    balanceCorrection = constrain(balanceCorrection, -kMaxEncoderBalanceCorrection, kMaxEncoderBalanceCorrection);

    setTurnCommand(signedCommand, balanceCorrection);
    printTurnStatus("[IMUTurn]", targetDeg, errorDeg, signedCommand, encoderTarget);
    delay(10);
  }

  stopMotors();
  updateImu();
  printTurnStatus("[IMUTurn final]", targetDeg, targetDeg - yawDeg, 0, encoderTarget);
  return true;
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

void runLineFollowStep() {
  String uid;
  if (pollRfid(&uid)) {
    lastUid = uid;
    stopMotors();
    Serial.print(F("[RFID] detected UID="));
    Serial.println(lastUid);
    pauseStopped(kPauseAfterRfidMs);
    missionState = MissionState::DriveOffset;
    return;
  }

  readQtrRcArray();
  const int position = computeLinePosition();
  const int error = (position - 4000) * kLineErrorSign;
  const FollowMode mode = chooseFollowMode(error);

  int leftSpeed = 0;
  int rightSpeed = 0;
  computeLineMotorCommand(mode, error, &leftSpeed, &rightSpeed);

  if (mode == FollowMode::Stopped) {
    stopMotors();
    missionState = MissionState::Stopped;
  } else {
    setTank(leftSpeed, rightSpeed);
  }

  printLineStatus(mode, position, error, leftSpeed, rightSpeed);
  updateImu();
  delay(kLoopDelayMs);
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

  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
  }

  pinMode(kServoPin, OUTPUT);
  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);

  Wire.begin();
  initializeQtrCalibration();
  initializeMotoron();
  initializeRfid();
  initializeImu();

  if (kResetServoAtStartup) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 700);
  }

  Serial.println(F("Line_Planting_IMU ready."));
  Serial.println(F("Flow: line follow -> RFID -> drive 205mm -> servo +60 deg -> left IMU 90 deg -> repeat."));
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
    case MissionState::FollowLine:
      runLineFollowStep();
      break;

    case MissionState::DriveOffset:
      if (driveDistanceMm(kPlantingOffsetMm, kOffsetDriveSpeed)) {
        missionState = MissionState::DropSeed;
      } else {
        missionState = MissionState::Stopped;
      }
      break;

    case MissionState::DropSeed:
      dropOneSeedNoReturn();
      pauseStopped(kPauseAfterPlantMs);
      missionState = MissionState::TurnLeft;
      break;

    case MissionState::TurnLeft:
      if (turnDegreesImu(kTurnAfterPlantDeg)) {
        plantingCycleCount++;
        pauseStopped(kPauseAfterTurnMs);
        if (kMaxPlantingCycles > 0 && plantingCycleCount >= kMaxPlantingCycles) {
          missionState = MissionState::Complete;
        } else {
          integralLineError = 0.0f;
          lastLineError = 0;
          missionState = MissionState::FollowLine;
        }
      } else {
        missionState = MissionState::Stopped;
      }
      break;

    case MissionState::Complete:
      stopMotors();
      if (!completeAnnounced) {
        Serial.println(F("[DONE] square planting sequence complete. Motors stopped."));
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
