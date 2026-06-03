#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Solid_Grid_Navigation
// Board: Arduino GIGA R1 WiFi
//
// Behavior:
//   1. Follow the solid grid line using the QTR-HD-09RC array.
//   2. If the line is temporarily lost, drive straight instead of searching.
//   3. Plant from the first RFID tag detected.
//   4. Plant 5 seeds while driving this route:
//        forward 2 nodes -> right 90 deg -> forward 1 node ->
//        left 90 deg -> forward 2 nodes.
//   5. Each planting event reuses the previous line-planting logic:
//        RFID -> drive kPlantingOffsetMm -> servo +60 deg -> optional turn.
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

// Trial Run 2 solid-grid route.
constexpr float kPlantingOffsetMm = 90.0f;     // RFID detect point -> seed drop point.
constexpr uint8_t kRequiredSeeds = 5;           // Total seeds to drop for this manoeuvre.
constexpr bool kSkipFirstRfidAtStart = false;   // false = the first detected RFID immediately plants.
constexpr uint32_t kStartUidIgnoreMs = 1500;    // Ignore the skipped start tag briefly while driving away.
constexpr uint8_t kRightTurnAfterSeed = 2;      // After 2 planted nodes, make the right turn.
constexpr uint8_t kLeftTurnAfterSeed = 3;       // After 3 planted nodes, make the left turn.
constexpr float kFirstTurnTargetYawDeg = -90.0f; // Absolute yaw target: right turn from startup heading.
constexpr float kSecondTurnTargetYawDeg = 0.0f;  // Absolute yaw target: return to startup heading.
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
constexpr uint16_t kDefaultLineThreshold = 650;
constexpr uint16_t kStrongLineThreshold = 650;
constexpr int kHardTurnError = 2600;
constexpr int kCenterRecoverError = 900;
constexpr uint8_t kIntegralClamp = 120;
constexpr uint32_t kLinePrintIntervalMs = 160;
constexpr uint32_t kLoopDelayMs = 8;
constexpr bool kDriveStraightIfLineLost = true;
constexpr uint8_t kStraightIfInactiveSensorsAtLeast = 8; // If 8/9 sensors miss the line, drive straight.
constexpr bool kUseImuStraightWhenLineLost = true;
constexpr float kDefaultImuStraightKp = 4.0f;
constexpr float kDefaultImuStraightKi = 0.0f;
constexpr float kDefaultImuStraightKd = 0.15f;
constexpr float kImuStraightIntegralClamp = 80.0f;
constexpr int kDefaultImuStraightMaxCorrection = 180;

// Latest measured raw values from QTR_Raw_Read_Test.
// For RC QTR sensors, darker surfaces generally produce larger timing values.
constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};

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
constexpr uint32_t kKillDebounceMs = 250;

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
float imuStraightKp = kDefaultImuStraightKp;
float imuStraightKi = kDefaultImuStraightKi;
float imuStraightKd = kDefaultImuStraightKd;
float imuStraightIntegral = 0.0f;
int imuStraightMaxCorrection = kDefaultImuStraightMaxCorrection;
float imuStraightTargetYawDeg = 0.0f;
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
bool killButtonLastRawPressed = false;
bool killButtonStablePressed = false;
uint32_t killButtonLastChangeMs = 0;
int currentServoAngle = kServoMinAngle;
uint8_t plantingCycleCount = 0;
uint8_t rfidTagsSeen = 0;
String lastUid;
String skippedStartUid;
uint32_t ignoreStartUidUntilMs = 0;
bool startRfidSkipped = false;
bool completeAnnounced = false;
float pendingTurnTargetYawDeg = 0.0f;
bool pendingTurnActive = false;

enum class MissionState {
  FollowLine,
  DriveOffset,
  DropSeed,
  TurnScheduled,
  Complete,
  Stopped
};

enum class FollowMode {
  Follow,
  HardLeft,
  HardRight,
  LineLostStraight,
  SearchLeft,
  SearchRight,
  Stopped
};

enum class KillButtonEvent {
  None,
  Pressed,
  Released
};

enum class KillButtonMode {
  Running,
  StopPressHeld,
  StoppedWaitingRestart,
  RestartPressHeld
};

MissionState missionState = MissionState::FollowLine;
KillButtonMode killButtonMode = KillButtonMode::Running;

void restartMissionFromKill();

const __FlashStringHelper *missionName(MissionState state) {
  switch (state) {
    case MissionState::FollowLine: return F("FOLLOW_LINE");
    case MissionState::DriveOffset: return F("DRIVE_OFFSET");
    case MissionState::DropSeed: return F("DROP_SEED");
    case MissionState::TurnScheduled: return F("TURN_SCHEDULED");
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
    case FollowMode::LineLostStraight: return F("LINE_LOST_STRAIGHT");
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

float normalizeAngleDeg(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg <= -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

float nearestCardinalYawDeg(float currentYawDeg) {
  const float candidates[4] = {0.0f, 90.0f, 180.0f, 270.0f};
  float bestTarget = 0.0f;
  float bestError = 999.0f;

  for (uint8_t i = 0; i < 4; i++) {
    const float candidate = normalizeAngleDeg(candidates[i]);
    const float error = absFloat(normalizeAngleDeg(candidate - currentYawDeg));
    if (error < bestError) {
      bestError = error;
      bestTarget = candidate;
    }
  }

  return bestTarget;
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

void requestStop(const __FlashStringHelper *reason) {
  serialStopped = true;
  missionState = MissionState::Stopped;
  stopMotors();
  Serial.print(F("[KILL] stopped: "));
  Serial.println(reason);
}

bool missionAbortRequested() {
  if (serialStopped) return true;
  if (killPressed()) {
    killButtonLastRawPressed = true;
    killButtonStablePressed = true;
    killButtonLastChangeMs = millis();
    if (killButtonMode == KillButtonMode::Running) {
      killButtonMode = KillButtonMode::StopPressHeld;
    }
    requestStop(F("kill button pressed"));
    return true;
  }
  return false;
}

KillButtonEvent updateKillButtonEvent() {
  const bool rawPressed = killPressed();
  const uint32_t now = millis();

  if (rawPressed != killButtonLastRawPressed) {
    killButtonLastRawPressed = rawPressed;
    killButtonLastChangeMs = now;
  }

  if (now - killButtonLastChangeMs < kKillDebounceMs) {
    return KillButtonEvent::None;
  }

  if (rawPressed != killButtonStablePressed) {
    killButtonStablePressed = rawPressed;
    return killButtonStablePressed ? KillButtonEvent::Pressed : KillButtonEvent::Released;
  }

  return KillButtonEvent::None;
}

void handleKillButtonStateMachine(KillButtonEvent event) {
  if (event == KillButtonEvent::None) {
    return;
  }

  switch (killButtonMode) {
    case KillButtonMode::Running:
      if (event == KillButtonEvent::Pressed) {
        killButtonMode = KillButtonMode::StopPressHeld;
        requestStop(F("kill button pressed; release to finish stop"));
      }
      break;

    case KillButtonMode::StopPressHeld:
      if (event == KillButtonEvent::Released) {
        killButtonMode = KillButtonMode::StoppedWaitingRestart;
        Serial.println(F("[KILL] stopped. Press and release D32 again to reset/recalibrate."));
      }
      break;

    case KillButtonMode::StoppedWaitingRestart:
      if (event == KillButtonEvent::Pressed) {
        killButtonMode = KillButtonMode::RestartPressHeld;
        Serial.println(F("[KILL] restart armed. Release D32 to reset and recalibrate IMU."));
      }
      break;

    case KillButtonMode::RestartPressHeld:
      if (event == KillButtonEvent::Released) {
        restartMissionFromKill();
      }
      break;
  }
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
  Serial.print(F(" seeds="));
  Serial.print(plantingCycleCount);
  Serial.print(F("/"));
  Serial.print(kRequiredSeeds);
  Serial.print(F(" tagsSeen="));
  Serial.print(rfidTagsSeen);
  Serial.print(F(" offsetMm="));
  Serial.print(kPlantingOffsetMm, 1);
  Serial.print(F(" turnsAfterSeeds="));
  Serial.print(kRightTurnAfterSeed);
  Serial.print(F("->yaw "));
  Serial.print(kFirstTurnTargetYawDeg, 1);
  Serial.print(F(", "));
  Serial.print(kLeftTurnAfterSeed);
  Serial.print(F("->yaw "));
  Serial.print(kSecondTurnTargetYawDeg, 1);
  Serial.print(F(" lineP="));
  Serial.print(lineKp, 3);
  Serial.print(F(" lineD="));
  Serial.print(lineKd, 3);
  Serial.print(F(" imuLineLostP="));
  Serial.print(imuStraightKp, 3);
  Serial.print(F(" imuLineLostI="));
  Serial.print(imuStraightKi, 3);
  Serial.print(F(" imuLineLostD="));
  Serial.print(imuStraightKd, 3);
  Serial.print(F(" imuTarget="));
  Serial.print(imuStraightTargetYawDeg, 1);
  Serial.print(F(" turnP="));
  Serial.print(turnKp, 2);
  Serial.print(F(" turnD="));
  Serial.print(turnKd, 2);
  Serial.print(F(" yaw="));
  Serial.print(yawDeg, 2);
  Serial.print(F(" servoAngle="));
  Serial.print(currentServoAngle);
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  stop | resume | show"));
  Serial.println(F("  linep 0.8 | lined 0.08 | base 400 | th 230"));
  Serial.println(F("  imup 4.0 | imui 0 | imud 0.15 | imumax 180"));
  Serial.println(F("  turnp 500 | turnd 0 | turnmax 600 | turnmin 115"));
  Serial.println(F("Route: plant 5 seeds: 2 nodes, absolute yaw -90, 1 node, return to startup yaw 0, 2 nodes."));
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
    killButtonMode = KillButtonMode::StoppedWaitingRestart;
    stopMotors();
    Serial.println(F("[SERIAL] stopped."));
    return;
  }
  if (lower == "resume") {
    serialStopped = false;
    killButtonMode = KillButtonMode::Running;
    completeAnnounced = false;
    integralLineError = 0.0f;
    imuStraightIntegral = 0.0f;
    lastLineError = 0;
    pendingTurnTargetYawDeg = 0.0f;
    pendingTurnActive = false;
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
  } else if (key == "imup") {
    imuStraightKp = value;
  } else if (key == "imui") {
    imuStraightKi = value;
  } else if (key == "imud") {
    imuStraightKd = value;
  } else if (key == "imumax") {
    imuStraightMaxCorrection = constrain(static_cast<int>(value), 0, 800);
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
    return 4000;
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

uint8_t inactiveSensorCount(uint16_t threshold) {
  return 9 - activeSensorCount(threshold);
}

bool tooFewSensorsSeeLine() {
  return inactiveSensorCount(lineThreshold) >= kStraightIfInactiveSensorsAtLeast;
}

bool centerHasLine() {
  return qtrNorm[3] >= lineThreshold || qtrNorm[4] >= lineThreshold || qtrNorm[5] >= lineThreshold;
}

FollowMode chooseFollowMode(int error) {
  if (missionAbortRequested()) {
    return FollowMode::Stopped;
  }

  if (!lineDetected || tooFewSensorsSeeLine()) {
    integralLineError = 0.0f;
    lastLineError = 0;
    if (kDriveStraightIfLineLost) {
      return FollowMode::LineLostStraight;
    }
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
    case FollowMode::LineLostStraight:
      if (kUseImuStraightWhenLineLost && imuOk) {
        imuStraightTargetYawDeg = nearestCardinalYawDeg(yawDeg);
        const float imuErrorDeg = normalizeAngleDeg(imuStraightTargetYawDeg - yawDeg);
        imuStraightIntegral += imuErrorDeg;
        imuStraightIntegral = constrain(
            imuStraightIntegral, -kImuStraightIntegralClamp, kImuStraightIntegralClamp);

        int imuCorrection = static_cast<int>(
            imuStraightKp * imuErrorDeg + imuStraightKi * imuStraightIntegral -
            imuStraightKd * gyroZDegPerSec);
        imuCorrection = constrain(imuCorrection, -imuStraightMaxCorrection, imuStraightMaxCorrection);

        *leftSpeed = baseSpeed - imuCorrection;
        *rightSpeed = baseSpeed + imuCorrection;
      } else {
        *leftSpeed = baseSpeed;
        *rightSpeed = baseSpeed;
      }
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

  imuStraightIntegral = 0.0f;
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

  Serial.print(F("[LINE] seed="));
  Serial.print(plantingCycleCount);
  Serial.print(F("/"));
  Serial.print(kRequiredSeeds);
  Serial.print(F(" tag="));
  Serial.print(rfidTagsSeen);
  Serial.print(F(" mode="));
  Serial.print(followModeName(mode));
  Serial.print(F(" line="));
  Serial.print(lineDetected ? F("YES") : F("NO"));
  Serial.print(F(" active="));
  Serial.print(activeSensorCount(lineThreshold));
  Serial.print(F(" inactive="));
  Serial.print(inactiveSensorCount(lineThreshold));
  Serial.print(F(" pos="));
  Serial.print(position);
  Serial.print(F(" err="));
  Serial.print(error);
  Serial.print(F(" L="));
  Serial.print(leftSpeed);
  Serial.print(F(" R="));
  Serial.print(rightSpeed);
  Serial.print(F(" yaw="));
  Serial.print(yawDeg, 1);
  Serial.print(F(" imuTarget="));
  Serial.print(imuStraightTargetYawDeg, 1);
  Serial.print(F(" imuErr="));
  Serial.print(normalizeAngleDeg(imuStraightTargetYawDeg - yawDeg), 1);
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
    if (missionAbortRequested()) {
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
  yawDeg = normalizeAngleDeg(yawDeg + gyroZDegPerSec * dt);
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
    if (missionAbortRequested()) {
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
    if (missionAbortRequested()) break;

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

bool turnDegreesImu(float targetYawDeg) {
  targetYawDeg = normalizeAngleDeg(targetYawDeg);
  updateImu();
  const float initialErrorDeg = normalizeAngleDeg(targetYawDeg - yawDeg);

  if (!imuOk) {
    encoderOnlyTurnFallback(initialErrorDeg, turnMaxSpeed);
    yawDeg = normalizeAngleDeg(yawDeg + initialErrorDeg);
    return true;
  }

  const float startYawDeg = yawDeg;
  const long encoderTarget = turnDegreesToEncoderCounts(initialErrorDeg);
  Serial.print(F("[TURN] IMU absoluteTargetYaw="));
  Serial.print(targetYawDeg, 1);
  Serial.print(F(" startYaw="));
  Serial.print(startYawDeg, 1);
  Serial.print(F(" initialError="));
  Serial.print(initialErrorDeg, 1);
  Serial.print(F(" encoderTarget="));
  Serial.println(encoderTarget);

  resetEncoders();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();

  while (true) {
    handleSerialCommands();
    if (missionAbortRequested()) {
      stopMotors();
      return false;
    }

    updateImu();
    const float errorDeg = normalizeAngleDeg(targetYawDeg - yawDeg);
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
    printTurnStatus("[IMUTurn]", targetYawDeg, errorDeg, signedCommand, encoderTarget);
    delay(10);
  }

  stopMotors();
  updateImu();
  printTurnStatus("[IMUTurn final]", targetYawDeg, normalizeAngleDeg(targetYawDeg - yawDeg), 0, encoderTarget);
  return true;
}

void pauseStopped(uint32_t durationMs) {
  stopMotors();
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    handleSerialCommands();
    if (missionAbortRequested()) {
      stopMotors();
      return;
    }
    updateImu();
    delay(20);
  }
}

bool scheduledTurnTargetYawAfterSeed(uint8_t seedsPlanted, float *targetYawDeg) {
  if (seedsPlanted == kRightTurnAfterSeed) {
    *targetYawDeg = kFirstTurnTargetYawDeg;
    return true;
  }
  if (seedsPlanted == kLeftTurnAfterSeed) {
    *targetYawDeg = kSecondTurnTargetYawDeg;
    return true;
  }
  *targetYawDeg = 0.0f;
  return false;
}

void runLineFollowStep() {
  updateImu();

  String uid;
  if (pollRfid(&uid)) {
    const uint32_t now = millis();
    if (uid == skippedStartUid && now < ignoreStartUidUntilMs) {
      updateImu();
      return;
    }

    lastUid = uid;
    rfidTagsSeen++;
    stopMotors();
    Serial.print(F("[RFID] detected UID="));
    Serial.print(lastUid);
    Serial.print(F(" tagCount="));
    Serial.println(rfidTagsSeen);

    if (kSkipFirstRfidAtStart && !startRfidSkipped) {
      startRfidSkipped = true;
      skippedStartUid = lastUid;
      ignoreStartUidUntilMs = now + kStartUidIgnoreMs;
      Serial.println(F("[RFID] start tag skipped; continuing line following without planting."));
      pauseStopped(kPauseAfterRfidMs);
      return;
    }

    if (plantingCycleCount >= kRequiredSeeds) {
      Serial.println(F("[RFID] required seed count already reached; ignoring extra tag."));
      return;
    }

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

void restartMissionFromKill() {
  stopMotors();
  Serial.println(F("[KILL] restart requested. Recalibrating IMU and restarting the route."));
  while (killPressed()) {
    stopMotors();
    delay(10);
  }
  killButtonLastRawPressed = false;
  killButtonStablePressed = false;
  killButtonMode = KillButtonMode::Running;
  killButtonLastChangeMs = millis();

  resetEncoders();
  integralLineError = 0.0f;
  imuStraightIntegral = 0.0f;
  lastLineError = 0;
  lastSeenLineError = 0;
  lineDetected = false;
  lastLinePrintMs = 0;
  lastTurnPrintMs = 0;
  lastRfidPollMs = 0;

  plantingCycleCount = 0;
  rfidTagsSeen = 0;
  lastUid = "";
  skippedStartUid = "";
  ignoreStartUidUntilMs = 0;
  startRfidSkipped = false;
  completeAnnounced = false;
  pendingTurnTargetYawDeg = 0.0f;
  pendingTurnActive = false;

  serialStopped = false;
  missionState = MissionState::FollowLine;

  initializeMotoron();
  initializeRfid();
  initializeImu();
  if (kResetServoAtStartup) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 700);
  }

  Serial.println(F("[KILL] mission restarted from the first RFID."));
  printSettings();
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
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);

  Wire.begin();
  initializeQtrCalibration();
  initializeMotoron();
  initializeRfid();
  initializeImu();

  if (kResetServoAtStartup) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 700);
  }

  Serial.println(F("Solid_Grid_Navigation ready."));
  Serial.println(F("Route: forward 2 nodes, yaw -90, forward 1 node, return yaw 0, forward 2 nodes."));
  Serial.println(F("Line-lost behavior: IMU heading hold toward nearest 0/90/180/270 cardinal angle."));
  Serial.print(F("Skip first RFID at start = "));
  Serial.println(kSkipFirstRfidAtStart ? F("YES") : F("NO"));
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

  const KillButtonEvent killEvent = updateKillButtonEvent();
  handleKillButtonStateMachine(killEvent);
  if ((serialStopped || missionState == MissionState::Stopped) &&
      killButtonMode == KillButtonMode::Running && !killPressed()) {
    killButtonMode = KillButtonMode::StoppedWaitingRestart;
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
      plantingCycleCount++;
      pendingTurnActive = scheduledTurnTargetYawAfterSeed(plantingCycleCount, &pendingTurnTargetYawDeg);
      Serial.print(F("[ROUTE] seeds planted="));
      Serial.print(plantingCycleCount);
      Serial.print(F("/"));
      Serial.print(kRequiredSeeds);
      Serial.print(F(" pendingTurn="));
      if (pendingTurnActive) {
        Serial.print(F("absolute yaw "));
        Serial.println(pendingTurnTargetYawDeg, 1);
      } else {
        Serial.println(F("none"));
      }
      pauseStopped(kPauseAfterPlantMs);
      if (plantingCycleCount >= kRequiredSeeds) {
        missionState = MissionState::Complete;
      } else if (pendingTurnActive) {
        missionState = MissionState::TurnScheduled;
      } else {
        integralLineError = 0.0f;
        imuStraightIntegral = 0.0f;
        lastLineError = 0;
        missionState = MissionState::FollowLine;
      }
      break;

    case MissionState::TurnScheduled:
      if (turnDegreesImu(pendingTurnTargetYawDeg)) {
        pendingTurnTargetYawDeg = 0.0f;
        pendingTurnActive = false;
        pauseStopped(kPauseAfterTurnMs);
        if (plantingCycleCount >= kRequiredSeeds) {
          missionState = MissionState::Complete;
        } else {
          integralLineError = 0.0f;
          imuStraightIntegral = 0.0f;
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
        Serial.println(F("[DONE] solid grid navigation complete: 5 seeds planted. Motors stopped."));
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
