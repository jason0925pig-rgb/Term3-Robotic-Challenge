#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

// =====================================================
// Motoron
// =====================================================

constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kLeftMotorChannel = 1;
constexpr uint8_t kRightMotorChannel = 2;

constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

MotoronI2C motoron(kMotoronAddress);

// =====================================================
// Kill button
// =====================================================

constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

// =====================================================
// Sonar Pins
// =====================================================

constexpr uint8_t kFrontTrigPin = 8;
constexpr uint8_t kFrontEchoPin = 11;

constexpr uint32_t kEchoTimeoutUs = 30000;

// =====================================================
// Revival tuning
// =====================================================

// 远距离速度
constexpr int kFastSpeed = 300;

// 中距离速度
constexpr int kMediumSpeed = 180;

// 近距离速度
constexpr int kSlowSpeed = 90;

// 看到目标的距离
constexpr float kDetectDistanceCm = 45.0;

// 开始减速距离
constexpr float kMediumDistanceCm = 25.0;

// 非常接近距离
constexpr float kSlowDistanceCm = 12.0;

// 停止距离，防止撞飞
constexpr float kStopDistanceCm = 7.0;

// 停止后保持时间
constexpr uint32_t kContactHoldMs = 3000;

bool revivalFinished = false;
uint32_t contactStartMs = 0;

// =====================================================
// Motor functions
// =====================================================

int clampMotor(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
}

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotor(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotor(rightSpeed) * kRightMotorSign;

  motoron.setSpeed(kLeftMotorChannel, leftSpeed);
  motoron.setSpeed(kRightMotorChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

void driveForward(int speed) {
  setTank(speed, speed);
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

// =====================================================
// Sonar
// =====================================================

float readSonarCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  uint32_t durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);

  if (durationUs == 0) {
    return -1.0f;
  }

  return durationUs / 58.0f;
}

// =====================================================
// Init
// =====================================================

void initializeMotoron() {
  Wire1.begin();

  motoron.setBus(&Wire1);
  motoron.reinitialize();
  delay(10);

  motoron.disableCrc();
  motoron.clearResetFlag();

  motoron.setCommandTimeoutMilliseconds(1000);

  motoron.setMaxAcceleration(kLeftMotorChannel, 0);
  motoron.setMaxDeceleration(kLeftMotorChannel, 0);

  motoron.setMaxAcceleration(kRightMotorChannel, 0);
  motoron.setMaxDeceleration(kRightMotorChannel, 0);

  stopMotors();

  Serial.println(F("[INIT] Motoron ready"));
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

  pinMode(kFrontTrigPin, OUTPUT);
  pinMode(kFrontEchoPin, INPUT);

  digitalWrite(kFrontTrigPin, LOW);

  Wire.begin();
  Wire1.begin();

  initializeMotoron();

  Serial.println(F("=== Robot Revival Test Ready ==="));
}

// =====================================================
// Loop
// =====================================================

void loop() {
  if (killPressed()) {
    stopMotors();
    Serial.println(F("[KILL] pressed"));
    delay(100);
    return;
  }

  if (revivalFinished) {
    stopMotors();
    return;
  }

  float frontCm = readSonarCm(kFrontTrigPin, kFrontEchoPin);

  Serial.print(F("front="));

  if (frontCm < 0) {
    Serial.println(F("timeout"));
    stopMotors();
    delay(50);
    return;
  }

  Serial.print(frontCm);
  Serial.println(F(" cm"));

  if (frontCm > kDetectDistanceCm) {
    stopMotors();
    Serial.println(F("[REVIVAL] No target detected"));
  }
  else if (frontCm > kMediumDistanceCm) {
    driveForward(kFastSpeed);
    Serial.println(F("[REVIVAL] Fast approach"));
  }
  else if (frontCm > kSlowDistanceCm) {
    driveForward(kMediumSpeed);
    Serial.println(F("[REVIVAL] Medium approach"));
  }
  else if (frontCm > kStopDistanceCm) {
    driveForward(kSlowSpeed);
    Serial.println(F("[REVIVAL] Slow contact approach"));
  }
  else {
    stopMotors();

    if (contactStartMs == 0) {
      contactStartMs = millis();
      Serial.println(F("[REVIVAL] Contact reached. Holding."));
    }

    if (millis() - contactStartMs >= kContactHoldMs) {
      revivalFinished = true;
      Serial.println(F("[REVIVAL] Finished."));
    }
  }

  delay(50);
}
