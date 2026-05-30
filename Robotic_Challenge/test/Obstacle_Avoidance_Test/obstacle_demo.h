#pragma once

#include <Arduino.h>

#include "constants.h"

void simpleObstacleDemoStep() {
  if (obstacleAhead(kObstacleAheadThresholdMm)) {
    stopMotors();
    Serial.println(F("[OBSTACLE] front obstacle detected; stopped."));
    delay(200);
    return;
  }

  applyLineFollowStep();
}
