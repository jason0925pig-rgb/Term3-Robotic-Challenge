#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "../constants.h"
#include "../types.h"
#include "../globals.h"
#include "../declarations.h"
#include "../basic_utils.h"
#include "../motor_utils.h"
#include "../encoder_utils.h"
#include "../line_following_utils.h"
#include "../sonar_wall_utils.h"
#include "../imu_turn_utils.h"
#include "../serial_utils.h"

#include "easy_controller.h"

constexpr uint32_t kEasyStartupWifiCheckMs = 5000;

void setupEasyPins() {
  if (kUseKillPin) pinMode(kKillPin, INPUT_PULLUP);

  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; ++i) {
    pinMode(kQtrPins[i], INPUT);
  }

  if (validPin(kFrontTrigPin)) pinMode(kFrontTrigPin, OUTPUT);
  if (validPin(kFrontEchoPin)) pinMode(kFrontEchoPin, INPUT);
  if (validPin(kLeftTrigPin)) pinMode(kLeftTrigPin, OUTPUT);
  if (validPin(kLeftEchoPin)) pinMode(kLeftEchoPin, INPUT);
  if (validPin(kRightTrigPin)) pinMode(kRightTrigPin, OUTPUT);
  if (validPin(kRightEchoPin)) pinMode(kRightEchoPin, INPUT);

  if (validPin(kFrontTrigPin)) digitalWrite(kFrontTrigPin, LOW);
  if (validPin(kLeftTrigPin)) digitalWrite(kLeftTrigPin, LOW);
  if (validPin(kRightTrigPin)) digitalWrite(kRightTrigPin, LOW);

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);
}

void printStartupCheckStatus(const __FlashStringHelper *label, bool ok) {
  Serial.print(label);
  Serial.print(F(": "));
  Serial.println(ok ? F("OK") : F("CHECK"));
}

void printEasyStrategyStartupCheck() {
  Serial.println(F("[CHECK] Hard-coded line strategy"));
  Serial.println(F("  base exit: route A/right, request airlock A, wall-follow tunnel to field line"));
  Serial.println(F("  enter: find first RFID, center, turn right"));
  Serial.println(F("  row 1: drive until field end (2), turn left"));
  Serial.println(F("  row 2: drive 1, turn left, drive 8/end"));
  Serial.println(F("  row 3: turn right, drive 1, turn right, drive 8/end"));
  Serial.println(F("  row 4: turn left, drive 1, turn left, drive 8/end"));
  Serial.println(F("  return: turn left, drive 3, turn left, drive 2, turn right"));
  Serial.println(F("  per tag: detect RFID, center forward, then poll/plant/turn"));
  Serial.println(F("  RFID UID drives fertility polling; server coordinates ignored for navigation"));
  Serial.print(F("  known RFID UID mappings="));
  Serial.println(kEasyKnownTagCount);
  printStartupCheckStatus(F("  scripted navigation active"), true);
}

void printEasyMotorStartupCheck() {
  Serial.println(F("[CHECK] Motoron / motors"));
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t statusErr = motoron.getLastError();

  Serial.print(F("  statusFlags=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" lastError="));
  Serial.println(statusErr);
  Serial.print(F("  address=0x"));
  Serial.print(kMotoronAddress, HEX);
  Serial.print(F(" leftChannel="));
  Serial.print(kMotoronLeftChannel);
  Serial.print(F(" rightChannel="));
  Serial.println(kMotoronRightChannel);
  printStartupCheckStatus(F("  Motoron I2C command path"), statusErr == 0);
}

void printEasyEncoderStartupCheck() {
  Serial.println(F("[CHECK] Encoders"));
  resetEncoders();
  Serial.print(F("  leftCount="));
  Serial.print(getLeftCount());
  Serial.print(F(" rightCount="));
  Serial.println(getRightCount());
  Serial.print(F("  countsPerMm="));
  Serial.println(kEncoderCountsPerMm, 4);
  printStartupCheckStatus(F("  encoder pins initialized"), true);
}

void printEasySafetyStartupCheck() {
  Serial.println(F("[CHECK] Safety inputs"));
  Serial.print(F("  mechanical kill pin D"));
  Serial.print(kKillPin);
  Serial.print(F(" state="));
  Serial.println(killPressed() ? F("PRESSED") : F("released"));
  Serial.print(F("  serial stop flag="));
  Serial.println(serialStopped ? F("STOPPED") : F("clear"));
  printStartupCheckStatus(F("  safety ready"), !killPressed() && !serialStopped);
}

void printEasyQtrStartupCheck() {
  Serial.println(F("[CHECK] QTR line sensor"));
  const LineReading line = readLine();
  Serial.print(F("  detected="));
  Serial.print(line.detected ? F("YES") : F("NO"));
  Serial.print(F(" active="));
  Serial.print(line.activeCount);
  Serial.print(F(" pos="));
  Serial.print(line.position);
  Serial.print(F(" err="));
  Serial.print(line.error);
  Serial.print(F(" mode="));
  Serial.println(followModeName(line.mode));
  Serial.print(F("  norm=["));
  for (uint8_t i = 0; i < 9; ++i) {
    if (i > 0) Serial.print(',');
    Serial.print(line.norm[i]);
  }
  Serial.println(F("]"));
  printStartupCheckStatus(F("  QTR read path"), true);
}

void printEasySonarStartupCheck() {
  Serial.println(F("[CHECK] Sonars"));
  const SonarReading sonar = readSonars();
  Serial.print(F("  frontMm="));
  Serial.print(sonar.frontValid ? sonar.frontMm : -1.0f, 1);
  Serial.print(F(" leftMm="));
  Serial.print(sonar.leftValid ? sonar.leftMm : -1.0f, 1);
  Serial.print(F(" rightMm="));
  Serial.println(sonar.rightValid ? sonar.rightMm : -1.0f, 1);
  Serial.print(F("  valid front/left/right="));
  Serial.print(sonar.frontValid ? F("Y") : F("N"));
  Serial.print('/');
  Serial.print(sonar.leftValid ? F("Y") : F("N"));
  Serial.print('/');
  Serial.println(sonar.rightValid ? F("Y") : F("N"));
  printStartupCheckStatus(F("  sonar read path"), true);
}

void printEasyRfidStartupCheck() {
  Serial.println(F("[CHECK] RFID"));
  Serial.print(F("  address=0x"));
  Serial.print(kEasyRfidAddress, HEX);
  Serial.print(F(" resetPin=D"));
  Serial.println(kEasyRfidResetPin);
  printStartupCheckStatus(F("  RFID reader"), easyRfidOk);
}

void printEasyImuStartupCheck() {
  Serial.println(F("[CHECK] IMU"));
  const ImuReading reading = getImuReading();
  Serial.print(F("  address=0x"));
  Serial.print(kImuAddress, HEX);
  Serial.print(F(" ok="));
  Serial.print(reading.ok ? F("YES") : F("NO"));
  Serial.print(F(" yawDeg="));
  Serial.print(reading.yawDeg, 2);
  Serial.print(F(" gyroZDps="));
  Serial.println(reading.gyroZDps, 2);
  printStartupCheckStatus(F("  IMU turn source"), reading.ok);
}

void printEasyPlantingStartupCheck() {
  Serial.println(F("[CHECK] Planting servo"));
  Serial.print(F("  pin=D"));
  Serial.print(kEasyServoPin);
  Serial.print(F(" angle="));
  Serial.print(easyCurrentServoAngle);
  Serial.print(F(" stepDeg="));
  Serial.print(kEasyServoStepAngle);
  Serial.print(F(" centerOffsetMm="));
  Serial.println(kEasyPlantCenterOffsetMm, 1);
  printStartupCheckStatus(F("  servo signal configured"), true);
}

void printEasyWifiStartupCheck() {
  Serial.println(F("[CHECK] WiFi / MiniMessenger"));
  Serial.print(F("  group="));
  Serial.print(GROUP_ID);
  Serial.print(F(" board="));
  Serial.print(kEasyBoardId);
  Serial.print(F(" broker="));
  Serial.print(BROKER_HOST);
  Serial.print(F(":"));
  Serial.println(BROKER_PORT);

  const uint32_t start = millis();
  while (millis() - start < kEasyStartupWifiCheckMs && !easyMessenger.isConnected()) {
    easyServerLoop();
    delay(50);
  }

  easyServerLoop();
  Serial.print(F("  connected="));
  Serial.print(easyMessenger.isConnected() ? F("YES") : F("NO"));
  Serial.print(F(" motionAllowed="));
  Serial.println(easyMotionAllowedByWifi() ? F("YES") : F("NO"));
  printStartupCheckStatus(F("  MiniMessenger connection"), easyMessenger.isConnected());
}

void printEasyStartupChecks() {
  Serial.println(F("=== Startup checks begin ==="));
  printEasyStrategyStartupCheck();
  printEasyMotorStartupCheck();
  printEasyEncoderStartupCheck();
  printEasySafetyStartupCheck();
  printEasyQtrStartupCheck();
  printEasySonarStartupCheck();
  printEasyRfidStartupCheck();
  printEasyImuStartupCheck();
  printEasyPlantingStartupCheck();
  printEasyWifiStartupCheck();
  Serial.println(F("=== Startup checks complete; starting loop ==="));
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  Serial.println(F("=== Easy Hard-Coded Line Seed Planting ==="));

  setupEasyPins();
  Wire.begin();
  initializeQtrCalibration();
  initializeMotoron();
  initializeEasyRfid();
  initializeImu();
  initializeEasyServer();
  initializeEasyPlanting();
  initializeEasyMissionController();
  printEasyStartupChecks();

  Serial.println(F("[SERIAL] commands from shared utility: stop | resume | show | line | sonars | turnleft | turnright"));
}

void loop() {
  runEasyMissionStep();
  delay(10);
}
