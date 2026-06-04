#pragma once

#include "hard_config.h"

inline DoorReading readDoor() {
  DoorReading reading;
  reading.frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  reading.valid = isValidSonarDistance(reading.frontMm);
  reading.closed = reading.valid && reading.frontMm <= kDoorClosedThresholdMm;
  reading.open = (reading.valid && reading.frontMm >= kDoorOpenThresholdMm) ||
                 (!reading.valid && kTreatNoEchoAsOpen);
  return reading;
}


inline float sonarDistanceForSide(WallSide side, const SonarReading &snapshot) {
  return side == WallSide::Left ? snapshot.leftMm : snapshot.rightMm;
}

/**
 * Check whether a snapshot contains a valid sonar reading for a logical side.
 */
inline bool sonarValidForSide(WallSide side, const SonarReading &snapshot) {
  return side == WallSide::Left ? snapshot.leftValid : snapshot.rightValid;
}

/**
 * Update stable closed-door detection.
 *
 * @param reading Latest front sonar door reading.
 * @return true after the door has looked closed for enough consecutive frames.
 */
inline bool doorClosedStable(const DoorReading &reading) {
  if (reading.closed) {
    if (doorClosedStableCount < 255) doorClosedStableCount++;
  } else {
    doorClosedStableCount = 0;
  }
  return doorClosedStableCount >= kDoorStableFrames;
}

/**
 * Update stable open-door detection.
 *
 * @param reading Latest front sonar door reading.
 * @return true after the door has looked open for enough consecutive frames.
 */
inline bool doorOpenStable(const DoorReading &reading) {
  if (reading.open) {
    if (doorOpenStableCount < 255) doorOpenStableCount++;
  } else {
    doorOpenStableCount = 0;
  }
  return doorOpenStableCount >= kDoorStableFrames;
}

/**
 * Print throttled door sonar diagnostics.
 *
 * @param label Log prefix describing which door is being handled.
 * @param reading Latest door reading.
 */
inline void printDoorStatus(const __FlashStringHelper *label, const DoorReading &reading) {
  if (millis() - lastDoorPrintMs < kDoorPrintIntervalMs) return;
  lastDoorPrintMs = millis();

  Serial.print(label);
  Serial.print(F(" frontMm="));
  Serial.print(reading.frontMm, 1);
  Serial.print(F(" valid="));
  Serial.print(reading.valid ? F("YES") : F("NO"));
  Serial.print(F(" closedStable="));
  Serial.print(doorClosedStableCount);
  Serial.print(F(" openStable="));
  Serial.println(doorOpenStableCount);
}
