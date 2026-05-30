#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "globals.h"

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
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
