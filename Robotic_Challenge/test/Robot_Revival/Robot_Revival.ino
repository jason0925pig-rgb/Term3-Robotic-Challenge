#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Arduino_Modulino.h>

// ---------------------------------------------------------------------------
// Robot_Revival
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   Standalone rescue/revival behaviour test. The robot drives forward at a
//   normal speed, slows down when the front sonar reports a close object, and
//   stops with green top LEDs when the revival button is pressed.
//
// Hardware:
//   Front sonar     -> trig D8, echo D11 through 5V-to-3.3V level shifter
//   Revival button  -> D31 to GND, INPUT_PULLUP
//   Stop button     -> D32 to GND, INPUT_PULLUP
//   Modulino Pixels -> Wire / D20 SDA-D21 SCL, address 0x36
//   Motoron M3S550  -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1/M2   -> left/right motors
// ---------------------------------------------------------------------------

constexpr uint32_t kSerialBaud = 115200;

// Motoron setup. The Motoron is mounted as an Arduino shield and is connected
// to the GIGA's secondary I2C bus, Wire1. M1 is the left track and M2 is the
// right track on the current robot.
constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kLeftMotorChannel = 1;
constexpr uint8_t kRightMotorChannel = 2;
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;
constexpr int kMaxMotorCommand = 800;

// Button wiring. INPUT_PULLUP means the pin reads HIGH when released and LOW
// when the button shorts the pin to ground.
constexpr uint8_t kRevivalPin = 31;
constexpr uint8_t kStopPin = 32;
constexpr uint32_t kButtonDebounceMs = 35;

// Front sonar. The 20 cm threshold is intentionally generous: in the rescue
// approach we want to slow down well before the robot reaches the target.
constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kFrontEchoPin = 11;
constexpr uint32_t kEchoTimeoutUs = 25000;
constexpr float kApproachThresholdCm = 20.0f;

// Motion speeds. 520 is the normal approach speed used in earlier rescue tests;
// 250 is a slow final approach that is easier to interrupt accurately.
constexpr int kBaseSpeed = 520;
constexpr int kApproachSpeed = 250;

// LED brightness is kept below full power so the Modulino Pixels are visible
// without drawing unnecessary current from the 3.3 V rail.
constexpr uint8_t kLedBrightness = 80;
constexpr uint32_t kPrintIntervalMs = 200;

enum class RevivalState {
  Cruising,
  SlowApproach,
  Revived,
  Stopped
};

MotoronI2C motoron(kMotoronAddress);
ModulinoPixels pixels;

RevivalState state = RevivalState::Cruising;
bool pixelsOk = false;
bool lastRevivalReading = HIGH;
bool stableRevivalReading = HIGH;
uint32_t lastRevivalChangeMs = 0;
uint32_t lastPrintMs = 0;

int clampMotorSpeed(int speed) {
  if (speed > kMaxMotorCommand) return kMaxMotorCommand;
  if (speed < -kMaxMotorCommand) return -kMaxMotorCommand;
  return speed;
}

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kLeftMotorChannel, leftSpeed);
  motoron.setSpeed(kRightMotorChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

void setAllPixels(ModulinoColor color) {
  if (!pixelsOk) return;
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kLedBrightness);
  }
  pixels.show();
}

const __FlashStringHelper *stateName(RevivalState s) {
  switch (s) {
    case RevivalState::Cruising: return F("CRUISING");
    case RevivalState::SlowApproach: return F("SLOW_APPROACH");
    case RevivalState::Revived: return F("REVIVED");
    case RevivalState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

bool revivalPressedEvent() {
  const bool reading = digitalRead(kRevivalPin);
  if (reading != lastRevivalReading) {
    lastRevivalReading = reading;
    lastRevivalChangeMs = millis();
  }
  if (millis() - lastRevivalChangeMs < kButtonDebounceMs) return false;
  if (reading == stableRevivalReading) return false;
  stableRevivalReading = reading;
  return stableRevivalReading == LOW;
}

float readFrontSonarCm() {
  digitalWrite(kFrontTrigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(kFrontTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kFrontTrigPin, LOW);

  const uint32_t durationUs = pulseIn(kFrontEchoPin, HIGH, kEchoTimeoutUs);
  if (durationUs == 0) return -1.0f;
  return durationUs * 0.01715f;
}

void initializeMotoron() {
  Wire1.begin();
  motoron.setBus(&Wire1);
  motoron.reinitialize();
  delay(10);
  motoron.disableCrc();
  motoron.clearResetFlag();
  motoron.setMaxAcceleration(kLeftMotorChannel, 0);
  motoron.setMaxDeceleration(kLeftMotorChannel, 0);
  motoron.setMaxAcceleration(kRightMotorChannel, 0);
  motoron.setMaxDeceleration(kRightMotorChannel, 0);
  stopMotors();
  Serial.println(F("[MOTORON] initialized on Wire1 at 0x11."));
}

void initializePixels() {
  Wire.begin();
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  setAllPixels(RED);
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1000);
  Serial.println(F("=== Robot_Revival ==="));

  pinMode(kFrontTrigPin, OUTPUT);
  pinMode(kFrontEchoPin, INPUT);
  pinMode(kRevivalPin, INPUT_PULLUP);
  pinMode(kStopPin, INPUT_PULLUP);
  digitalWrite(kFrontTrigPin, LOW);

  lastRevivalReading = digitalRead(kRevivalPin);
  stableRevivalReading = lastRevivalReading;
  lastRevivalChangeMs = millis();

  initializePixels();
  initializeMotoron();

  Serial.println(F("[INFO] Red LEDs = moving/ready. Green LEDs = revival button pressed."));
  Serial.println(F("[INFO] D32 stop button is a latched stop for this standalone test."));
}

void loop() {
  if (digitalRead(kStopPin) == LOW) {
    state = RevivalState::Stopped;
  }

  if (revivalPressedEvent()) {
    state = RevivalState::Revived;
  }

  const float frontCm = readFrontSonarCm();
  if (state != RevivalState::Revived && state != RevivalState::Stopped) {
    if (frontCm > 0.0f && frontCm < kApproachThresholdCm) {
      state = RevivalState::SlowApproach;
    } else {
      state = RevivalState::Cruising;
    }
  }

  if (state == RevivalState::Revived) {
    stopMotors();
    setAllPixels(GREEN);
  } else if (state == RevivalState::Stopped) {
    stopMotors();
    setAllPixels(RED);
  } else {
    setAllPixels(RED);
    const int speed = state == RevivalState::SlowApproach ? kApproachSpeed : kBaseSpeed;
    setTank(speed, speed);
  }

  if (millis() - lastPrintMs >= kPrintIntervalMs) {
    lastPrintMs = millis();
    Serial.print(F("[REVIVAL] state="));
    Serial.print(stateName(state));
    Serial.print(F(" frontCm="));
    Serial.print(frontCm, 1);
    Serial.print(F(" revivalPin="));
    Serial.print(digitalRead(kRevivalPin) == LOW ? F("PRESSED") : F("released"));
    Serial.print(F(" stopPin="));
    Serial.println(digitalRead(kStopPin) == LOW ? F("PRESSED") : F("released"));
  }

  delay(20);
}
