#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ============================================================================
// Obstacle_Avoidance_Primitives
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   This is not a finished Test 7 state machine. It is a cleaned-up function
//   library / scaffold derived from the uploaded Line_Planting_IMU.ino and
//   Wall_Following.ino sketches.
//
//   Reusable robot primitives now live in the local headers below:
//     - constants.h
//     - types.h
//     - globals.h
//     - basic_utils.h
//     - motor_utils.h
//     - encoder_utils.h
//     - line_following_utils.h
//     - sonar_wall_utils.h
//     - imu_turn_utils.h
//     - obstacle_demo.h
//     - serial_utils.h
// ============================================================================

#include "constants.h"
#include "types.h"
#include "globals.h"
#include "declarations.h"
#include "basic_utils.h"
#include "motor_utils.h"
#include "encoder_utils.h"
#include "line_following_utils.h"
#include "sonar_wall_utils.h"
#include "imu_turn_utils.h"
#include "obstacle_demo.h"
#include "serial_utils.h"

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) pinMode(kKillPin, INPUT_PULLUP);

  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) pinMode(kQtrPins[i], INPUT);

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

  Wire.begin();
  initializeQtrCalibration();
  initializeMotoron();
  initializeImu();
  lastWallUpdateMs = millis();

  Serial.println(F("Obstacle_Avoidance_Primitives ready."));
  Serial.print(F("Encoder counts per mm estimate = "));
  Serial.println(kEncoderCountsPerMm, 4);
  printHelp();
}

void loop() {
  handleSerialCommands();

  if (killPressed()) {
    serialStopped = true;
    stopMotors();
  }

  if (serialStopped) {
    stopMotors();
    updateImu();
    delay(50);
    return;
  }

  if (kRunSimpleLineFollowDemo) {
    simpleObstacleDemoStep();
  } else {
    updateImu();
    delay(50);
  }
}
