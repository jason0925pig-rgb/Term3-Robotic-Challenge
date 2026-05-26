#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>

// ---------------------------------------------------------------------------
// planting
// Board: Arduino GIGA R1 WiFi
//
// Behavior:
//   1. Drive forward slowly while polling the RFID reader.
//   2. When an RFID card is detected, stop and print its UID.
//   3. Drive forward 7 cm using motor encoders.
//   4. Rotate the 300-degree servo by 60 degrees to release one seed.
//   5. Stop forever by default.
//
// Hardware:
//   Motoron M3S550 address 0x11 on Wire1 / shield SDA1-SCL1
//   Motoron M1 = left motor, M2 = right motor
//   Left encoder C1/C2  -> D34/D35
//   Right encoder C1/C2 -> D36/D37
//   RFID I2C address    -> 0x28 on Wire / D20 SDA-D21 SCL
//   Servo signal        -> D33
//   Kill button         -> D32 to GND, INPUT_PULLUP
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

// Search and planting motion.
constexpr int kSearchForwardSpeed = 180;
constexpr float kPlantingOffsetMm = 90.0f;      // RFID detect point -> seed drop point.
constexpr int kOffsetDriveSpeed = 400;
constexpr uint32_t kDriveTimeoutMs = 8000;
constexpr uint32_t kPrintIntervalMs = 200;
constexpr uint32_t kRfidPollIntervalMs = 80;
constexpr bool kStopForeverAfterPlanting = true;

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

// Encoder geometry. Matches the latest Motor_Encoder_U_Test assumptions.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kMotorNoLoadRpm = 200.0f;        // Waveshare DCGM-N20-12V-EN-200RPM no-load speed.
constexpr float kEncoderCountsPerMotorRev = 7.0f; // C1/A rising-edge pulses before gearbox; 1050 per wheel rev after 1:150.
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;

// RFID.
constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;

// Mechanical kill button.
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

// DS-R005 300-degree positional servo.
constexpr uint8_t kServoPin = 33;
constexpr int kServoMinUs = 500;
constexpr int kServoMaxUs = 2500;
constexpr int kServoMinAngle = 0;
constexpr int kServoMaxAngle = 300;
constexpr int kServoStepAngle = 60;
constexpr uint32_t kServoMoveSettleMs = 600;
constexpr uint32_t kServoHoldAfterDropMs = 3000;
constexpr uint32_t kServoFrameUs = 20000;
constexpr bool kResetServoAtStartup = true;

constexpr float kPi = 3.1415926f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);

volatile long leftCount = 0;
volatile long rightCount = 0;

bool rfidOk = false;
uint32_t lastPrintMs = 0;
uint32_t lastRfidPollMs = 0;
int currentServoAngle = kServoMinAngle;

enum class PlantingState {
  SearchRfid,
  DriveOffset,
  DropSeed,
  Done,
  Killed
};

PlantingState state = PlantingState::SearchRfid;
String detectedUid;

long absLong(long value) {
  return value < 0 ? -value : value;
}

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
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
  return static_cast<long>(abs(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
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
    if (killPressed()) {
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

void dropOneSeed() {
  int nextAngle = currentServoAngle + kServoStepAngle;
  if (nextAngle > kServoMaxAngle) {
    Serial.println(F("[SERVO] angle limit reached; resetting to 0 before seed drop."));
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 500);
    nextAngle = kServoStepAngle;
  }

  Serial.println(F("[PLANT] dropping one seed."));
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);
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
  if (!rfidOk) {
    return false;
  }

  if (millis() - lastRfidPollMs < kRfidPollIntervalMs) {
    return false;
  }
  lastRfidPollMs = millis();

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    *uidOut = rfidUidToString();
    rfid.PICC_HaltA();
    return true;
  }

  return false;
}

void printCounts(const __FlashStringHelper *label, long targetCounts = -1) {
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

void printMotoronState() {
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t statusErr = motoron.getLastError();
  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t vinErr = motoron.getLastError();

  Serial.print(F("[MOTORON] status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" statusErr="));
  Serial.print(statusErr);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" vinErr="));
  Serial.println(vinErr);
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

bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("[DRIVE] distanceMm="));
  Serial.print(distanceMm);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    if (killPressed()) {
      Serial.println(F("[SAFETY] kill pressed during drive distance."));
      stopMotors();
      state = PlantingState::Killed;
      return false;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;

    if (averageAbs >= targetCounts) {
      break;
    }

    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("[DRIVE] timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);

    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);
    printCounts(F("[DRIVE]"), targetCounts);
    delay(10);
  }

  stopMotors();
  printCounts(F("[DRIVE final]"), targetCounts);
  return true;
}

void runSearchForRfid() {
  if (killPressed()) {
    state = PlantingState::Killed;
    stopMotors();
    return;
  }

  String uid;
  if (pollRfid(&uid)) {
    detectedUid = uid;
    stopMotors();
    Serial.print(F("[RFID] CARD UID="));
    Serial.println(detectedUid);
    state = PlantingState::DriveOffset;
    delay(300);
    return;
  }

  setTank(kSearchForwardSpeed, kSearchForwardSpeed);
  printCounts(F("[SEARCH]"));
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

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

  if (kResetServoAtStartup) {
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 1000);
  }

  Serial.println(F("planting ready."));
  Serial.println(F("Flow: drive forward -> detect RFID -> drive 70mm -> drop one seed -> stop."));
  Serial.print(F("Motor spec: no-load rpm="));
  Serial.print(kMotorNoLoadRpm, 1);
  Serial.print(F(", gear ratio=1:"));
  Serial.print(kGearRatio, 0);
  Serial.print(F(", encoder PPR="));
  Serial.println(kEncoderCountsPerMotorRev, 1);
  Serial.print(F("Encoder counts per mm estimate = "));
  Serial.println(kEncoderCountsPerMm, 4);
  printMotoronState();
}

void loop() {
  switch (state) {
    case PlantingState::SearchRfid:
      runSearchForRfid();
      break;

    case PlantingState::DriveOffset:
      if (driveDistanceMm(kPlantingOffsetMm, kOffsetDriveSpeed)) {
        state = PlantingState::DropSeed;
      }
      break;

    case PlantingState::DropSeed:
      dropOneSeed();
      state = PlantingState::Done;
      break;

    case PlantingState::Done:
      stopMotors();
      if (kStopForeverAfterPlanting) {
        Serial.println(F("[DONE] planting sequence complete. Stopped forever."));
        while (true) {
          if (killPressed()) {
            stopMotors();
          }
          delay(100);
        }
      }
      detectedUid = "";
      state = PlantingState::SearchRfid;
      break;

    case PlantingState::Killed:
      stopMotors();
      Serial.println(F("[SAFETY] killed. Reset board to run again."));
      while (true) {
        delay(100);
      }
      break;
  }
}
