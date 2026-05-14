#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

// ---------------------------------------------------------------------------
// Motor + encoder motion test
// Board: Arduino GIGA R1 WiFi
//
// Hardware assumptions:
//   Motoron M3S550 address 0x11 on Wire1 / shield SDA1-SCL1
//   Motoron M1 = left motor, M2 = right motor
//   Left encoder C1/C2  -> D34/D35
//   Right encoder C1/C2 -> D36/D37
//
// Test sequence:
//   1. Drive forward at 400, stop 3s
//   2. Drive forward at 600, stop 3s
//   3. Drive forward at 800, stop 3s
//   4. Encoder left turn 90 degrees, stop 3s
//   5. Encoder right turn 90 degrees, stop 3s
//   6. Drive a U shape using encoder distance + encoder turns
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr uint16_t kMotoronReferenceMv = 3300;
constexpr auto kMotoronVinType = MotoronVinSenseType::Motoron550;

constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;

// Pull D32 to GND to abort the test if you keep the kill button connected.
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

// Flip these if a motor runs backward relative to the intended command.
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

// Flip this if "left turn" and "right turn" are swapped.
constexpr int kTurnDirectionSign = 1;

// Speed test.
constexpr int kSpeedTestSpeeds[] = {400, 600, 800};
constexpr uint32_t kSpeedTestRunMs = 1500;
constexpr uint32_t kStopBetweenStepsMs = 3000;

// Encoder geometry. Measure/tune these on your actual robot.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;          // Distance between left/right wheel contact centers.
constexpr float kEncoderCountsPerMotorRev = 12.0f;
constexpr float kGearRatio = 150.58f;
constexpr float kDistanceCalibration = 1.0f;     // Increase if robot drives too short; decrease if too far.
constexpr float kTurnCalibration = 1.5f;         // Increase if turns are too small; decrease if too large.

// Encoder-controlled movement.
constexpr int kDriveDistanceSpeed = 450;
constexpr int kTurnSpeed = 360;
constexpr float kTurnAngleDeg = 90.0f;
constexpr uint32_t kDriveTimeoutMs = 12000;
constexpr uint32_t kTurnTimeoutMs = 8000;

// Straight driving correction using encoder counts.
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;

// U-shape path. Path is: forward leg, turn, middle, turn, forward leg.
constexpr float kULegDistanceMm = 300.0f;
constexpr float kUMiddleDistanceMm = 220.0f;
constexpr int kUTurnDirection = 1;               // 1 = left-left U, -1 = right-right U.
constexpr int kUDriveSpeed = 430;
constexpr int kUTurnSpeed = 340;

// true = run the full sequence once, then stop forever.
// false = repeat the full sequence after kRepeatPauseMs.
constexpr bool kRunSequenceOnce = true;
constexpr uint32_t kRepeatPauseMs = 8000;

constexpr uint32_t kPrintIntervalMs = 200;
constexpr float kPi = 3.1415926f;

// ---------------------------------------------------------------------------
// Derived constants
// ---------------------------------------------------------------------------
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);

volatile long leftCount = 0;
volatile long rightCount = 0;

long absLong(long value) {
  return value < 0 ? -value : value;
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

void printMotoronState() {
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t statusErr = motoron.getLastError();
  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t vinErr = motoron.getLastError();

  Serial.print(F("Motoron status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" statusErr="));
  Serial.print(statusErr);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" vinErr="));
  Serial.println(vinErr);
}

void printCounts(const char *label, long targetCounts = -1) {
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs < kPrintIntervalMs) return;
  lastPrintMs = millis();

  const long left = getLeftCount();
  const long right = getRightCount();
  Serial.print(label);
  Serial.print(F(" L="));
  Serial.print(left);
  Serial.print(F(" R="));
  Serial.print(right);
  Serial.print(F(" absL="));
  Serial.print(absLong(left));
  Serial.print(F(" absR="));
  Serial.print(absLong(right));
  if (targetCounts >= 0) {
    Serial.print(F(" target="));
    Serial.print(targetCounts);
  }
  Serial.println();
}

long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(abs(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * abs(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

void pauseStopped(uint32_t durationMs) {
  stopMotors();
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (killPressed()) {
      stopMotors();
      Serial.println(F("KILL pressed during pause."));
      return;
    }
    delay(20);
  }
}

void runTimedForward(int speed, uint32_t durationMs) {
  Serial.print(F("Timed forward speed="));
  Serial.print(speed);
  Serial.print(F(" durationMs="));
  Serial.println(durationMs);

  resetEncoders();
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (killPressed()) {
      Serial.println(F("KILL pressed. Timed forward aborted."));
      break;
    }
    setTank(speed, speed);
    printCounts("TimedForward");
    delay(10);
  }
  stopMotors();
  printCounts("TimedForward final");
}

void driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("Drive distance mm="));
  Serial.print(distanceMm);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    if (killPressed()) {
      Serial.println(F("KILL pressed. Drive distance aborted."));
      break;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) {
      break;
    }

    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("Drive distance timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);

    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);
    printCounts("DriveDistance", targetCounts);
    delay(10);
  }

  stopMotors();
  printCounts("DriveDistance final", targetCounts);
}

void turnDegrees(float degrees, int speed) {
  const long targetCounts = turnDegreesToCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  const int turnDirection = direction * kTurnDirectionSign;

  Serial.print(F("Turn degrees="));
  Serial.print(degrees);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCountsPerSide="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    if (killPressed()) {
      Serial.println(F("KILL pressed. Turn aborted."));
      break;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    if (leftAbs >= targetCounts && rightAbs >= targetCounts) {
      break;
    }

    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("Turn timeout."));
      break;
    }

    const int leftCommand = leftAbs < targetCounts ? -turnDirection * abs(speed) : 0;
    const int rightCommand = rightAbs < targetCounts ? turnDirection * abs(speed) : 0;
    setTank(leftCommand, rightCommand);
    printCounts("Turn", targetCounts);
    delay(10);
  }

  stopMotors();
  printCounts("Turn final", targetCounts);
}

void runSpeedTests() {
  Serial.println(F("=== Speed tests ==="));
  for (uint8_t i = 0; i < sizeof(kSpeedTestSpeeds) / sizeof(kSpeedTestSpeeds[0]); i++) {
    runTimedForward(kSpeedTestSpeeds[i], kSpeedTestRunMs);
    pauseStopped(kStopBetweenStepsMs);
  }
}

void runTurnTests() {
  Serial.println(F("=== Encoder turn tests ==="));
  turnDegrees(kTurnAngleDeg, kTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
  turnDegrees(-kTurnAngleDeg, kTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
}

void runUShape() {
  Serial.println(F("=== U-shape path ==="));
  driveDistanceMm(kULegDistanceMm, kUDriveSpeed);
  pauseStopped(kStopBetweenStepsMs);
  turnDegrees(kUTurnDirection * 90.0f, kUTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
  driveDistanceMm(kUMiddleDistanceMm, kUDriveSpeed);
  pauseStopped(kStopBetweenStepsMs);
  turnDegrees(kUTurnDirection * 90.0f, kUTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
  driveDistanceMm(kULegDistanceMm, kUDriveSpeed);
  pauseStopped(kStopBetweenStepsMs);
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

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);

  initializeMotoron();

  Serial.println(F("Motor_Encoder_U_Test ready. Lift wheels first if you are checking direction."));
  Serial.println(F("Motoron address 0x11 on Wire1. M1=left, M2=right."));
  Serial.println(F("Left encoder D34/D35, right encoder D36/D37."));
  Serial.print(F("Counts per mm estimate = "));
  Serial.println(kEncoderCountsPerMm, 4);
  printMotoronState();
  pauseStopped(2000);
}

void loop() {
  runSpeedTests();
  runTurnTests();
  runUShape();

  Serial.println(F("Full motor test sequence complete."));
  stopMotors();

  if (kRunSequenceOnce) {
    Serial.println(F("Sequence is configured to run once. Motors stopped forever."));
    while (true) {
      if (killPressed()) {
        stopMotors();
      }
      delay(200);
    }
  }

  pauseStopped(kRepeatPauseMs);
}
