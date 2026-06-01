#pragma once

#include <Arduino.h>

#include "constants.h"

long absLong(long value) {
  return value < 0 ? -value : value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

int clampMotorSpeed(int speed) {
  if (speed > kMaxMotorCommand) return kMaxMotorCommand;
  if (speed < -kMaxMotorCommand) return -kMaxMotorCommand;
  return speed;
}

bool validPin(int pin) {
  return pin >= 0;
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

bool handleKillPauseInBlockingMotion() {
  if (!killPressed()) return true;

  stopMotors();
  Serial.println(F("[KILL] paused during blocking motion. Press kill again to resume."));

  while (killPressed()) {
    handleSerialCommands();
    if (serialStopped) return false;
    stopMotors();
    updateImu();
    delay(20);
  }

  while (!killPressed()) {
    handleSerialCommands();
    if (serialStopped) return false;
    stopMotors();
    updateImu();
    delay(20);
  }

  while (killPressed()) {
    handleSerialCommands();
    if (serialStopped) return false;
    stopMotors();
    updateImu();
    delay(20);
  }

  Serial.println(F("[KILL] resumed blocking motion."));
  return true;
}
