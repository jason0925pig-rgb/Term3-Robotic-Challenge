#pragma once

#include <Arduino.h>

#include "constants.h"
#include "globals.h"
#include "types.h"

const __FlashStringHelper *followModeName(FollowMode mode) {
  switch (mode) {
    case FollowMode::Follow: return F("FOLLOW");
    case FollowMode::HardLeft: return F("HARD_LEFT");
    case FollowMode::HardRight: return F("HARD_RIGHT");
    case FollowMode::SearchLeft: return F("SEARCH_LEFT");
    case FollowMode::SearchRight: return F("SEARCH_RIGHT");
    case FollowMode::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

void initializeQtrCalibration() {
  for (uint8_t i = 0; i < 9; i++) {
    qtrMin[i] = kSavedQtrMin[i];
    qtrMax[i] = kSavedQtrMax[i];
  }
  Serial.println(F("[QTR] using saved calibration."));
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
    qtrRaw[i] = kQtrTimeoutUs;
  }

  bool allDone = false;
  while (!allDone && (micros() - start) < kQtrTimeoutUs) {
    allDone = true;
    const uint16_t elapsed = static_cast<uint16_t>(micros() - start);
    for (uint8_t i = 0; i < 9; i++) {
      if (qtrRaw[i] == kQtrTimeoutUs) {
        if (digitalRead(kQtrPins[i]) == LOW) {
          qtrRaw[i] = elapsed;
        } else {
          allDone = false;
        }
      }
    }
  }
}

void normalizeQtrValues() {
  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t span = qtrMax[i] > qtrMin[i] ? qtrMax[i] - qtrMin[i] : 0;
    if (span < kMinUsefulCalibrationSpan || qtrRaw[i] <= qtrMin[i]) {
      qtrNorm[i] = 0;
    } else if (qtrRaw[i] >= qtrMax[i]) {
      qtrNorm[i] = 1000;
    } else {
      qtrNorm[i] = static_cast<uint16_t>(
          (static_cast<uint32_t>(qtrRaw[i] - qtrMin[i]) * 1000UL) / span);
    }
  }
}

uint8_t activeLineSensorCount(uint16_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

bool centerHasLine() {
  return qtrNorm[3] >= kLineThreshold || qtrNorm[4] >= kLineThreshold || qtrNorm[5] >= kLineThreshold;
}

int computeLinePosition(bool *detectedOut) {
  normalizeQtrValues();

  uint32_t weighted = 0;
  uint32_t sum = 0;
  bool detected = false;

  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t weight = qtrNorm[i] >= kLineThreshold ? qtrNorm[i] : 0;
    if (weight > 0) {
      detected = true;
      weighted += static_cast<uint32_t>(weight) * (i * 1000);
      sum += weight;
    }
  }

  *detectedOut = detected;
  if (sum == 0) {
    return 4000 + lastSeenLineError;
  }

  const int position = static_cast<int>(weighted / sum);
  lastSeenLineError = position - 4000;
  return position;
}

FollowMode chooseFollowMode(const LineReading &line) {
  if (serialStopped || killPressed()) return FollowMode::Stopped;

  if (!line.detected) {
    lineIntegral = 0.0f;
    return lastSeenLineError < 0 ? FollowMode::SearchLeft : FollowMode::SearchRight;
  }

  const bool leftEdgeStrong = qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
  const bool rightEdgeStrong = qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;

  if (line.error < -kHardTurnError || (leftEdgeStrong && line.error < -kCenterRecoverError)) {
    lineIntegral = 0.0f;
    return FollowMode::HardLeft;
  }

  if (line.error > kHardTurnError || (rightEdgeStrong && line.error > kCenterRecoverError)) {
    lineIntegral = 0.0f;
    return FollowMode::HardRight;
  }

  if (centerHasLine()) return FollowMode::Follow;
  return line.error < 0 ? FollowMode::HardLeft : FollowMode::HardRight;
}

LineReading readLine() {
  LineReading line;
  readQtrRcArray();

  bool detected = false;
  line.position = computeLinePosition(&detected);
  line.detected = detected;
  line.error = (line.position - 4000) * kLineErrorSign;
  line.activeCount = activeLineSensorCount(kLineThreshold);
  line.mode = chooseFollowMode(line);

  for (uint8_t i = 0; i < 9; i++) {
    line.raw[i] = qtrRaw[i];
    line.norm[i] = qtrNorm[i];
  }

  return line;
}

MotorCommand computeLineMotorCommand(const LineReading &line) {
  MotorCommand cmd;

  switch (line.mode) {
    case FollowMode::Stopped:
      cmd.left = 0;
      cmd.right = 0;
      return cmd;
    case FollowMode::SearchLeft:
      cmd.left = -kLineSearchTurnSpeed;
      cmd.right = kLineSearchTurnSpeed;
      return cmd;
    case FollowMode::SearchRight:
      cmd.left = kLineSearchTurnSpeed;
      cmd.right = -kLineSearchTurnSpeed;
      return cmd;
    case FollowMode::HardLeft:
      cmd.left = -kLineHardTurnSpeed;
      cmd.right = kLineHardTurnSpeed;
      return cmd;
    case FollowMode::HardRight:
      cmd.left = kLineHardTurnSpeed;
      cmd.right = -kLineHardTurnSpeed;
      return cmd;
    case FollowMode::Follow:
    default:
      break;
  }

  lineIntegral += line.error / 1000.0f;
  lineIntegral = constrain(lineIntegral, -kLineIntegralClamp, kLineIntegralClamp);

  const int derivative = line.error - lastLineError;
  lastLineError = line.error;

  int correction = static_cast<int>(kLineKp * line.error + kLineKi * lineIntegral + kLineKd * derivative);
  correction = constrain(correction, -kLineMaxCorrection, kLineMaxCorrection);

  cmd.left = kLineBaseSpeed + correction;
  cmd.right = kLineBaseSpeed - correction;
  return cmd;
}

void applyLineFollowStep() {
  const LineReading line = readLine();
  const MotorCommand cmd = computeLineMotorCommand(line);

  if (line.mode == FollowMode::Stopped) {
    stopMotors();
  } else {
    setTank(cmd.left, cmd.right);
  }

  if (millis() - lastLinePrintMs >= kLinePrintIntervalMs) {
    lastLinePrintMs = millis();
    Serial.print(F("[LINE] mode="));
    Serial.print(followModeName(line.mode));
    Serial.print(F(" detected="));
    Serial.print(line.detected ? F("YES") : F("NO"));
    Serial.print(F(" active="));
    Serial.print(line.activeCount);
    Serial.print(F(" pos="));
    Serial.print(line.position);
    Serial.print(F(" err="));
    Serial.print(line.error);
    Serial.print(F(" L="));
    Serial.print(cmd.left);
    Serial.print(F(" R="));
    Serial.println(cmd.right);
  }

  updateImu();
  delay(kLineLoopDelayMs);
}
