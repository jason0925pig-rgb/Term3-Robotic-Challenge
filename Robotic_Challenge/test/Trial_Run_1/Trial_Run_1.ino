#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Arduino_Modulino.h>

// ---------------------------------------------------------------------------
// Trial Run 1 checklist demo
// Board: Arduino GIGA R1 WiFi
//
// Demonstrates in one sketch:
//   - Revival button: Modulino Pixels red -> green for 10s -> red
//   - Mechanical kill button: press once stops motors, press again resets and reruns motion demo
//   - Motor speed and heading control: 400/600/800 forward, left 90, right 90, direct 180 U-turn
//   - QTR reflectance array raw readings every 0.5s
//   - RFID card polling every 1s
//   - Front/left/right ultrasonic distances every 0.5s
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Parameters you will tune most often
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

constexpr int kForwardSpeeds[] = {400, 600, 800};
constexpr uint32_t kForwardRunMs = 1500;
constexpr uint32_t kStopAfterEachMoveMs = 3000;

constexpr float kLeftTurnDeg = 90.0f;
constexpr float kRightTurnDeg = -90.0f;
constexpr float kUTurnDeg = 180.0f;             // Change this to any angle later.
constexpr int kTurnSpeed = 360;
constexpr uint32_t kTurnTimeoutMs = 9000;

constexpr float kWheelTrackMm = 165.0f;
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kMotorNoLoadRpm = 200.0f;        // Waveshare DCGM-N20-12V-EN-200RPM no-load speed.
constexpr float kGearRatio = 150.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f; // Waveshare spec: 7 PPR before gearbox, 1050 PPR after 1:150.
constexpr float kTurnCalibration = 1.5f;          // Calibrated value for accurate 90/180 degree turns.

constexpr bool kRunMotionDemoOnce = true;
constexpr uint32_t kRepeatMotionDemoPauseMs = 8000;

// Motor trims. Flip signs if a side moves backward.
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;
constexpr int kTurnDirectionSign = 1;             // Flip if left/right turns are reversed.

// Sensor reporting intervals.
constexpr uint32_t kQtrPrintIntervalMs = 500;
constexpr uint32_t kSonarPrintIntervalMs = 500;
constexpr uint32_t kRfidPollIntervalMs = 1000;

// Revival LED behavior.
constexpr uint32_t kRevivalGreenMs = 10000;
constexpr uint8_t kLedRedBrightness = 25;
constexpr uint8_t kLedGreenBrightness = 35;

// ---------------------------------------------------------------------------
// Pin and device configuration
// ---------------------------------------------------------------------------
constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr uint16_t kMotoronReferenceMv = 3300;
constexpr auto kMotoronVinType = MotoronVinSenseType::Motoron550;

constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;

constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kFrontEchoPin = 11;
constexpr uint8_t kLeftTrigPin = 9;
constexpr uint8_t kLeftEchoPin = 12;
constexpr uint8_t kRightTrigPin = 10;
constexpr uint8_t kRightEchoPin = 13;
constexpr unsigned long kSonarPulseTimeoutUs = 25000UL;

constexpr uint8_t kRevivalButtonPin = 31;
constexpr uint8_t kKillButtonPin = 32;

constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;

constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;

constexpr float kPi = 3.1415926f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
ModulinoPixels pixels;

volatile long leftCount = 0;
volatile long rightCount = 0;

uint16_t qtrValues[9] = {};
uint32_t lastQtrPrintMs = 0;
uint32_t lastSonarPrintMs = 0;
uint32_t lastRfidPollMs = 0;

bool pixelsOk = false;
bool rfidOk = false;
bool killed = false;
bool motionDemoComplete = false;
uint32_t revivalGreenUntilMs = 0;

bool lastKillReading = HIGH;
bool stableKillReading = HIGH;
uint32_t lastKillChangeMs = 0;
bool lastRevivalReading = HIGH;
bool stableRevivalReading = HIGH;
uint32_t lastRevivalChangeMs = 0;
constexpr uint32_t kButtonDebounceMs = 35;

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

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

void setAllPixels(ModulinoColor color, uint8_t brightness) {
  if (!pixelsOk) return;
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, brightness);
  }
  pixels.show();
}

void updateRevivalLed() {
  if (!pixelsOk) return;
  if (millis() < revivalGreenUntilMs) {
    setAllPixels(GREEN, kLedGreenBrightness);
  } else {
    setAllPixels(RED, kLedRedBrightness);
  }
}

bool debouncedPressed(uint8_t pin, bool &lastReading, bool &stableReading, uint32_t &lastChangeMs) {
  const bool reading = digitalRead(pin);
  bool pressedEvent = false;

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = millis();
  }

  if (millis() - lastChangeMs >= kButtonDebounceMs && reading != stableReading) {
    stableReading = reading;
    if (stableReading == LOW) {
      pressedEvent = true;
    }
  }

  return pressedEvent;
}

void resetMotionDemo() {
  stopMotors();
  resetEncoders();
  motionDemoComplete = false;
  Serial.println(F("[MOTION] Demo reset to beginning."));
}

void handleButtons() {
  if (debouncedPressed(kRevivalButtonPin, lastRevivalReading, stableRevivalReading, lastRevivalChangeMs)) {
    revivalGreenUntilMs = millis() + kRevivalGreenMs;
    Serial.println(F("[REVIVAL] Button pressed -> LEDs GREEN for 10 seconds."));
  }

  if (debouncedPressed(kKillButtonPin, lastKillReading, stableKillReading, lastKillChangeMs)) {
    if (!killed) {
      killed = true;
      stopMotors();
      resetEncoders();
      Serial.println(F("[KILL] Pressed -> motors STOPPED. Press again to reset/restart demo."));
    } else {
      killed = false;
      resetMotionDemo();
      Serial.println(F("[KILL] Pressed again -> reset complete, motion demo will restart."));
    }
  }

  updateRevivalLed();
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
    qtrValues[i] = kQtrTimeoutUs;
  }

  bool allDone = false;
  while (!allDone && (micros() - start) < kQtrTimeoutUs) {
    allDone = true;
    const uint16_t elapsed = static_cast<uint16_t>(micros() - start);
    for (uint8_t i = 0; i < 9; i++) {
      if (qtrValues[i] == kQtrTimeoutUs) {
        if (digitalRead(kQtrPins[i]) == LOW) {
          qtrValues[i] = elapsed;
        } else {
          allDone = false;
        }
      }
    }
  }
}

void printQtr() {
  readQtrRcArray();
  Serial.print(F("[QTR] raw:"));
  for (uint8_t i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(qtrValues[i]);
  }
  Serial.println();
}

float readSonarCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const unsigned long duration = pulseIn(echoPin, HIGH, kSonarPulseTimeoutUs);
  if (duration == 0) return NAN;
  return duration * 0.0343f * 0.5f;
}

void printCmOrNaN(float value) {
  if (isnan(value)) {
    Serial.print(F("NaN"));
  } else {
    Serial.print(value, 1);
  }
}

void printSonar() {
  const float front = readSonarCm(kFrontTrigPin, kFrontEchoPin);
  delay(8);
  const float left = readSonarCm(kLeftTrigPin, kLeftEchoPin);
  delay(8);
  const float right = readSonarCm(kRightTrigPin, kRightEchoPin);

  Serial.print(F("[SONAR] front_cm="));
  printCmOrNaN(front);
  Serial.print(F(" left_cm="));
  printCmOrNaN(left);
  Serial.print(F(" right_cm="));
  printCmOrNaN(right);
  Serial.println();
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

void pollRfid() {
  if (!rfidOk) {
    Serial.println(F("[RFID] reader not responding"));
    return;
  }

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    Serial.print(F("[RFID] CARD UID="));
    Serial.println(rfidUidToString());
    rfid.PICC_HaltA();
  } else {
    Serial.println(F("[RFID] no card"));
  }
}

void serviceBackground() {
  handleButtons();

  if (millis() - lastQtrPrintMs >= kQtrPrintIntervalMs) {
    lastQtrPrintMs = millis();
    printQtr();
  }

  if (millis() - lastSonarPrintMs >= kSonarPrintIntervalMs) {
    lastSonarPrintMs = millis();
    printSonar();
  }

  if (millis() - lastRfidPollMs >= kRfidPollIntervalMs) {
    lastRfidPollMs = millis();
    pollRfid();
  }
}

void safeDelay(uint32_t durationMs) {
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    serviceBackground();
    if (killed) {
      stopMotors();
    }
    delay(5);
  }
}

long turnDegreesToCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * abs(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

void printCounts(const __FlashStringHelper *label, long targetCounts = -1) {
  Serial.print(label);
  Serial.print(F(" L="));
  Serial.print(getLeftCount());
  Serial.print(F(" R="));
  Serial.print(getRightCount());
  Serial.print(F(" absL="));
  Serial.print(absLong(getLeftCount()));
  Serial.print(F(" absR="));
  Serial.print(absLong(getRightCount()));
  if (targetCounts >= 0) {
    Serial.print(F(" target="));
    Serial.print(targetCounts);
  }
  Serial.println();
}

bool runTimedForward(int speed, uint32_t durationMs) {
  Serial.print(F("[MOTION] Forward speed="));
  Serial.print(speed);
  Serial.print(F(" duration_ms="));
  Serial.println(durationMs);

  resetEncoders();
  const uint32_t start = millis();
  uint32_t lastMotionPrintMs = 0;

  while (millis() - start < durationMs) {
    serviceBackground();
    if (killed) {
      stopMotors();
      Serial.println(F("[MOTION] Forward interrupted by kill."));
      return false;
    }

    setTank(speed, speed);
    if (millis() - lastMotionPrintMs >= 250) {
      lastMotionPrintMs = millis();
      printCounts(F("[MOTION] Forward counts"));
    }
    delay(5);
  }

  stopMotors();
  printCounts(F("[RESULT] Forward final counts"));
  return true;
}

bool turnDegreesEncoder(float degrees, int speed) {
  const long targetCounts = turnDegreesToCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  const int turnDirection = direction * kTurnDirectionSign;

  Serial.print(F("[MOTION] Turn degrees="));
  Serial.print(degrees);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" target_counts_per_side="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  uint32_t lastMotionPrintMs = 0;

  while (true) {
    serviceBackground();
    if (killed) {
      stopMotors();
      Serial.println(F("[MOTION] Turn interrupted by kill."));
      return false;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    if (leftAbs >= targetCounts && rightAbs >= targetCounts) {
      break;
    }

    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[WARN] Turn timeout."));
      break;
    }

    const int leftCommand = leftAbs < targetCounts ? -turnDirection * abs(speed) : 0;
    const int rightCommand = rightAbs < targetCounts ? turnDirection * abs(speed) : 0;
    setTank(leftCommand, rightCommand);

    if (millis() - lastMotionPrintMs >= 250) {
      lastMotionPrintMs = millis();
      printCounts(F("[MOTION] Turn counts"), targetCounts);
    }
    delay(5);
  }

  stopMotors();
  printCounts(F("[RESULT] Turn final counts"), targetCounts);
  return true;
}

void runMotionDemo() {
  Serial.println(F("[MOTION] === Trial Run 1 motion demo starts ==="));

  for (uint8_t i = 0; i < sizeof(kForwardSpeeds) / sizeof(kForwardSpeeds[0]); i++) {
    if (!runTimedForward(kForwardSpeeds[i], kForwardRunMs)) return;
    Serial.println(F("[MOTION] Stop after forward segment."));
    safeDelay(kStopAfterEachMoveMs);
  }

  if (!turnDegreesEncoder(kLeftTurnDeg, kTurnSpeed)) return;
  Serial.println(F("[MOTION] Stop after left turn."));
  safeDelay(kStopAfterEachMoveMs);

  if (!turnDegreesEncoder(kRightTurnDeg, kTurnSpeed)) return;
  Serial.println(F("[MOTION] Stop after right turn."));
  safeDelay(kStopAfterEachMoveMs);

  if (!turnDegreesEncoder(kUTurnDeg, kTurnSpeed)) return;
  Serial.println(F("[MOTION] Stop after direct 180-degree U-turn."));
  safeDelay(kStopAfterEachMoveMs);

  motionDemoComplete = true;
  Serial.println(F("[RESULT] Trial Run 1 motion demo complete."));
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

  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  Serial.print(F("[INIT] Motoron ready on Wire1 addr 0x11, vin_mV="));
  Serial.println(vinMv);
}

void initializeQtr() {
  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
  }
  Serial.println(F("[INIT] QTR reflectance array ready."));
}

void initializeSonar() {
  pinMode(kFrontTrigPin, OUTPUT);
  pinMode(kLeftTrigPin, OUTPUT);
  pinMode(kRightTrigPin, OUTPUT);
  pinMode(kFrontEchoPin, INPUT);
  pinMode(kLeftEchoPin, INPUT);
  pinMode(kRightEchoPin, INPUT);
  digitalWrite(kFrontTrigPin, LOW);
  digitalWrite(kLeftTrigPin, LOW);
  digitalWrite(kRightTrigPin, LOW);
  Serial.println(F("[INIT] Front/left/right ultrasonic sensors ready."));
}

void initializeRfid() {
  pinMode(kRfidResetPin, OUTPUT);
  digitalWrite(kRfidResetPin, HIGH);
  rfid.PCD_Init();
  const byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
  rfidOk = !(version == 0x00 || version == 0xFF);
  Serial.print(F("[INIT] RFID version=0x"));
  Serial.print(version, HEX);
  Serial.print(F(" status="));
  Serial.println(rfidOk ? F("OK") : F("NOT FOUND"));
}

void initializePixels() {
  pixelsOk = pixels.begin();
  Serial.print(F("[INIT] Modulino Pixels status="));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  updateRevivalLed();
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  Serial.println(F("=== Trial_Run_1 checklist demo ==="));
  Serial.println(F("Mechanical: revival button changes Pixels red -> green 10s -> red."));
  Serial.println(F("Electronics: motor, kill button, QTR, RFID, sonar. WiFi kill intentionally omitted."));

  Wire.begin();
  Wire1.begin();

  pinMode(kRevivalButtonPin, INPUT_PULLUP);
  pinMode(kKillButtonPin, INPUT_PULLUP);

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);

  initializeMotoron();
  initializeQtr();
  initializeSonar();
  initializeRfid();
  initializePixels();

  Serial.print(F("[INIT] Encoder counts per mm estimate="));
  Serial.println(kEncoderCountsPerMm, 4);
  Serial.println(F("[INIT] Press kill once to stop motors; press again to reset/restart motion demo."));
  safeDelay(1000);
}

void loop() {
  serviceBackground();

  if (killed) {
    stopMotors();
    delay(20);
    return;
  }

  if (!motionDemoComplete) {
    runMotionDemo();
    return;
  }

  stopMotors();
  if (!kRunMotionDemoOnce) {
    Serial.println(F("[MOTION] Demo complete; repeating after pause."));
    safeDelay(kRepeatMotionDemoPauseMs);
    resetMotionDemo();
  } else {
    delay(20);
  }
}
