#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>
#include <Arduino_Modulino.h>

#include "secrets.h"

// =====================================================
// Project Settings
// =====================================================

constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kMotoronAddress = 0x11;

constexpr uint8_t kLeftMotorChannel = 1;
constexpr uint8_t kRightMotorChannel = 2;

constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

constexpr const char *kBoardId = "Team2Robot";

// =====================================================
// Tuning Values
// =====================================================

constexpr int kEntrySpeed = 350;

// how long robot drives into base
constexpr uint32_t kDriveIntoBaseMs = 2600;

// wait at RFID / door before entering
constexpr uint32_t kDoorWaitMs = 2000;

constexpr uint32_t kRegisterIntervalMs = 5000;
constexpr uint32_t kWifiPrintMs = 2000;

constexpr uint8_t kLedBrightness = 70;

// =====================================================

MotoronI2C motoron(kMotoronAddress);
MiniMessenger messenger;
ModulinoPixels pixels;

bool pixelsOk = false;

bool wifiSafetyEnabled = false;
bool baseFinished = false;

uint32_t lastRegisterMs = 0;
uint32_t lastWifiPrintMs = 0;
uint32_t stateStartMs = 0;

enum class BaseState {
  WaitingForEnable,
  WaitingAtDoor,
  DrivingIntoBase,
  Finished
};

BaseState state = BaseState::WaitingForEnable;

// =====================================================
// Motor Functions
// =====================================================

int clampMotor(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
}

void setTank(int leftSpeed, int rightSpeed) {

  leftSpeed =
    clampMotor(leftSpeed) * kLeftMotorSign;

  rightSpeed =
    clampMotor(rightSpeed) * kRightMotorSign;

  motoron.setSpeed(
    kLeftMotorChannel,
    leftSpeed
  );

  motoron.setSpeed(
    kRightMotorChannel,
    rightSpeed
  );
}

void stopMotors() {
  setTank(0, 0);
}

void driveForward(int speed) {
  setTank(speed, speed);
}

bool killPressed() {
  return
    kUseKillPin &&
    digitalRead(kKillPin) == LOW;
}

// =====================================================
// LED
// =====================================================

void setAllPixels(ModulinoColor color) {

  if (!pixelsOk) return;

  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kLedBrightness);
  }

  pixels.show();
}

// =====================================================
// WiFi Safety
// =====================================================

void allowMotion() {

  wifiSafetyEnabled = true;

  Serial.println(
    F("[SAFETY] ENABLED")
  );
}

void blockMotion() {

  wifiSafetyEnabled = false;

  stopMotors();

  Serial.println(
    F("[SAFETY] DISABLED")
  );
}

void onWifiMessage(
  const MessageMetadata& metadata,
  const uint8_t* payload,
  size_t length
) {

  char msg[256];

  size_t copyLen =
    length < sizeof(msg) - 1 ?
    length :
    sizeof(msg) - 1;

  memcpy(msg, payload, copyLen);

  msg[copyLen] = '\0';

  Serial.print(F("[RX] "));
  Serial.println(msg);

  if (
    strstr(msg, "enable=1") ||
    strstr(msg, "enabled=true")
  ) {
    allowMotion();
  }

  if (
    strstr(msg, "enable=0") ||
    strstr(msg, "enabled=false") ||
    strstr(msg, "type=emergency")
  ) {
    blockMotion();
  }
}

void updateWifi() {

  messenger.loop();

  if (
    messenger.isConnected() &&
    (
      lastRegisterMs == 0 ||
      millis() - lastRegisterMs >= kRegisterIntervalMs
    )
  ) {

    lastRegisterMs = millis();

    char reg[128];

    snprintf(
      reg,
      sizeof(reg),
      "type=register team_id=%s board_id=%s",
      GROUP_ID,
      kBoardId
    );

    messenger.sendToBoard("server", reg);

    Serial.print(F("[REGISTER] "));
    Serial.println(reg);
  }

  if (
    millis() - lastWifiPrintMs >= kWifiPrintMs
  ) {

    lastWifiPrintMs = millis();

    Serial.print(F("[WIFI] connected="));

    Serial.print(
      messenger.isConnected() ?
      F("YES") :
      F("NO")
    );

    Serial.print(F(" safety="));

    Serial.println(
      wifiSafetyEnabled ?
      F("ENABLED") :
      F("DISABLED")
    );
  }
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

  motoron.setMaxAcceleration(
    kLeftMotorChannel,
    0
  );

  motoron.setMaxDeceleration(
    kLeftMotorChannel,
    0
  );

  motoron.setMaxAcceleration(
    kRightMotorChannel,
    0
  );

  motoron.setMaxDeceleration(
    kRightMotorChannel,
    0
  );

  stopMotors();

  Serial.println(
    F("[INIT] Motoron Ready")
  );
}

void initializePixels() {

  Modulino.begin(Wire);

  pixelsOk = pixels.begin();

  if (pixelsOk) {
    setAllPixels(RED);
  }

  Serial.print(F("[PIXELS] "));

  Serial.println(
    pixelsOk ?
    F("READY") :
    F("NOT FOUND")
  );
}

void initializeWifi() {

  messenger.onMessage(onWifiMessage);

  messenger.begin(
    WIFI_SSID,
    WIFI_PASSWORD,
    BROKER_HOST,
    BROKER_PORT,
    GROUP_ID,
    kBoardId
  );

  Serial.println(F("[WIFI] STARTED"));
}

// =====================================================
// Base Logic
// =====================================================

void setState(BaseState newState) {

  state = newState;

  stateStartMs = millis();

  Serial.print(F("[STATE] "));

  Serial.println((int)state);
}

void updateBaseLogic() {

  if (killPressed()) {

    stopMotors();

    blockMotion();

    Serial.println(
      F("[KILL] PRESSED")
    );

    delay(100);

    return;
  }

  switch (state) {

    case BaseState::WaitingForEnable:

      stopMotors();

      if (
        wifiSafetyEnabled &&
        !baseFinished
      ) {

        Serial.println(
          F("[BASE] WAITING AT RFID / DOOR")
        );

        setAllPixels(YELLOW);

        setState(BaseState::WaitingAtDoor);
      }

      break;

    case BaseState::WaitingAtDoor:

      stopMotors();

      if (
        millis() - stateStartMs >=
        kDoorWaitMs
      ) {

        Serial.println(
          F("[BASE] ENTERING")
        );

        setAllPixels(GREEN);

        setState(
          BaseState::DrivingIntoBase
        );
      }

      break;

    case BaseState::DrivingIntoBase:

      if (!wifiSafetyEnabled) {

        stopMotors();

        setState(
          BaseState::WaitingForEnable
        );

        break;
      }

      driveForward(kEntrySpeed);

      if (
        millis() - stateStartMs >=
        kDriveIntoBaseMs
      ) {

        stopMotors();

        baseFinished = true;

        setAllPixels(BLUE);

        Serial.println(
          F("[BASE] FINISHED")
        );

        setState(BaseState::Finished);
      }

      break;

    case BaseState::Finished:

      stopMotors();

      break;
  }
}

// =====================================================
// Setup / Loop
// =====================================================

void setup() {

  Serial.begin(kSerialBaud);

  delay(1500);

  Serial.println(
    F("=== BASE ENTRY TEST ===")
  );

  if (kUseKillPin) {
    pinMode(
      kKillPin,
      INPUT_PULLUP
    );
  }

  Wire.begin();

  Wire1.begin();

  initializePixels();

  initializeMotoron();

  initializeWifi();

  setState(
    BaseState::WaitingForEnable
  );
}

void loop() {

  updateWifi();

  updateBaseLogic();

  delay(10);
}
