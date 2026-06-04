#pragma once

#include <Arduino.h>

#include "easy_navigation.h"

enum class EasyBaseRouteChoice : uint8_t {
  BaseA_Bottom,
  BaseB_Top
};

enum class EasyBaseTurnDir : uint8_t {
  Left,
  Right
};

struct EasyDoorReading {
  float frontMm = -1.0f;
  bool valid = false;
  bool closed = false;
  bool open = false;
};

constexpr EasyBaseRouteChoice kEasyBaseRouteChoice = EasyBaseRouteChoice::BaseA_Bottom;
constexpr WallSide kEasyTunnelWallSide = WallSide::Left;
constexpr uint8_t kEasyBaseRouteTurnCount = 4;
constexpr uint8_t kEasyAirlockRfidCheckFromTurnIndex = 2;
constexpr float kEasyFirstTAdvanceMm = 60.0f;
constexpr float kEasySharpTurnAdvanceMm = 60.0f;
constexpr int kEasyRouteAdvanceSpeed = 300;
constexpr uint32_t kEasyLineToTunnelTimeoutMs = 12000;
constexpr uint32_t kEasyWallTunnelTimeoutMs = 25000;
constexpr uint32_t kEasyPostDoorLineTimeoutMs = 12000;
constexpr uint8_t kEasyTunnelEntryNoLineFrames = 6;
constexpr uint8_t kEasyWallExitLineStableFrames = 2;
constexpr uint8_t kEasyReturnWallExitLineStableFrames = 1;
constexpr int kEasyTunnelEntryConfirmSpeed = 260;
constexpr float kEasyDoorClosedThresholdMm = 170.0f;
constexpr float kEasyDoorOpenThresholdMm = 320.0f;
constexpr uint8_t kEasyDoorStableFrames = 3;
constexpr bool kEasyTreatNoEchoAsOpen = true;
constexpr uint32_t kEasyDoorPrintIntervalMs = 250;
constexpr uint8_t kEasyBaseExitRfidBurstPolls = 4;
constexpr uint16_t kEasyBaseExitRfidBurstGapMs = 5;
constexpr uint32_t kEasyStopOverAirlockTagMs = 800;
constexpr uint32_t kEasyAirlockRequestRetryMs = 1000;
constexpr uint8_t kEasyFirstTMinActiveSensors = 8;
constexpr uint8_t kEasyFirstTStableFrames = 5;
constexpr float kEasyFirstTMinTravelMm = 80.0f;
constexpr uint16_t kEasyFirstTEdgeStrongThreshold = 650;
constexpr uint16_t kEasyFirstTMinTotalStrength = 5600;
constexpr int kEasyFirstTMaxCenterError = 900;
constexpr uint8_t kEasySharpTurnStableFrames = 1;
constexpr uint32_t kEasyEventCooldownMs = 500;
constexpr int kEasyReacquireTurnSpeed = 150;
constexpr uint32_t kEasyReacquireTimeoutMs = 1400;
constexpr uint8_t kEasyReacquireStableFrames = 3;
constexpr uint32_t kEasyPostTurnHardIgnoreMs = 1100;
constexpr uint8_t kEasyPostTurnHardReleaseFrames = 4;
constexpr int kEasyPostTurnSoftErrorClamp = 650;

static uint8_t easyBaseRouteTurnIndex = 0;
static uint8_t easyBaseEventStableCount = 0;
static uint8_t easyBaseDoorClosedStableCount = 0;
static uint8_t easyBaseDoorOpenStableCount = 0;
static uint8_t easyBaseTunnelNoLineCount = 0;
static uint8_t easyBaseWallExitLineStableCount = 0;
static uint8_t easyBaseReturnTunnelNoLineCount = 0;
static uint8_t easyBaseReturnWallExitLineStableCount = 0;
static bool easyBasePostTurnHardIgnoreActive = false;
static uint32_t easyBasePostTurnHardIgnoreStartMs = 0;
static uint8_t easyBasePostTurnHardReleaseCount = 0;
static uint32_t easyBaseLastEventMs = 0;
static uint32_t easyBaseLastDoorPrintMs = 0;
static bool easyBaseAirlockRequestSent = false;
static String easyBasePendingAirlockUid;
static uint32_t easyBaseLastAirlockRequestAttemptMs = 0;

inline const __FlashStringHelper *easyBaseRouteName() {
  return kEasyBaseRouteChoice == EasyBaseRouteChoice::BaseA_Bottom ? F("A_RIGHT") : F("B_LEFT");
}

inline const __FlashStringHelper *easyBaseTurnName(EasyBaseTurnDir dir) {
  return dir == EasyBaseTurnDir::Left ? F("LEFT") : F("RIGHT");
}

inline EasyBaseTurnDir easyBaseRouteTurnAt(uint8_t index) {
  if (kEasyBaseRouteChoice == EasyBaseRouteChoice::BaseA_Bottom) {
    return (index == 0 || index == 3) ? EasyBaseTurnDir::Right : EasyBaseTurnDir::Left;
  }
  return (index == 0 || index == 3) ? EasyBaseTurnDir::Left : EasyBaseTurnDir::Right;
}

inline float easyBaseDegreesForTurn(EasyBaseTurnDir dir) {
  return dir == EasyBaseTurnDir::Left ? 90.0f : -90.0f;
}

inline void resetEasyBaseLineController() {
  lineIntegral = 0.0f;
  lastLineError = 0;
}

inline void resetEasyBaseWallController() {
  wallIntegral = 0.0f;
  lastWallErrorMm = 0.0f;
  lastWallUpdateMs = millis();
}

inline void resetEasyBaseDoorCounters() {
  easyBaseDoorClosedStableCount = 0;
  easyBaseDoorOpenStableCount = 0;
}

inline bool easyBaseIsHardLineMode(FollowMode mode) {
  return mode == FollowMode::HardLeft || mode == FollowMode::HardRight;
}

inline bool easyBaseLeftOuterHasStrongLine() {
  return qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
}

inline bool easyBaseRightOuterHasStrongLine() {
  return qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;
}

inline float easyBaseCurrentRouteSegmentTravelMm() {
  const long leftAbs = absLong(getLeftCount());
  const long rightAbs = absLong(getRightCount());
  const long averageCounts = (leftAbs + rightAbs) / 2;
  return averageCounts / (kEncoderCountsPerMm * kDistanceCalibration);
}

inline void beginEasyBasePostTurnHardIgnore() {
  easyBasePostTurnHardIgnoreActive = true;
  easyBasePostTurnHardIgnoreStartMs = millis();
  easyBasePostTurnHardReleaseCount = 0;
}

inline void updateEasyBasePostTurnHardIgnore(const LineReading &line) {
  if (!easyBasePostTurnHardIgnoreActive) return;

  if (millis() - easyBasePostTurnHardIgnoreStartMs >= kEasyPostTurnHardIgnoreMs) {
    easyBasePostTurnHardIgnoreActive = false;
    easyBasePostTurnHardReleaseCount = 0;
    return;
  }

  if (easyBaseIsHardLineMode(line.mode)) {
    easyBasePostTurnHardReleaseCount = 0;
    return;
  }

  if (line.detected && centerHasLine()) {
    if (easyBasePostTurnHardReleaseCount < 255) easyBasePostTurnHardReleaseCount++;
    if (easyBasePostTurnHardReleaseCount >= kEasyPostTurnHardReleaseFrames) {
      easyBasePostTurnHardIgnoreActive = false;
      easyBasePostTurnHardReleaseCount = 0;
    }
  } else {
    easyBasePostTurnHardReleaseCount = 0;
  }
}

inline LineReading easyBaseSoftenedPostTurnLine(LineReading line) {
  if (easyBasePostTurnHardIgnoreActive && easyBaseIsHardLineMode(line.mode)) {
    line.mode = FollowMode::Follow;
    line.error = constrain(line.error, -kEasyPostTurnSoftErrorClamp, kEasyPostTurnSoftErrorClamp);
  }
  return line;
}

inline void applyEasyBaseLineCommand(const LineReading &line, const __FlashStringHelper *label) {
  const MotorCommand cmd = computeLineMotorCommand(line);
  if (line.mode == FollowMode::Stopped) stopMotors();
  else setTank(cmd.left, cmd.right);

  if (millis() - lastLinePrintMs >= kLinePrintIntervalMs) {
    lastLinePrintMs = millis();
    Serial.print(label);
    Serial.print(F(" turn="));
    Serial.print(easyBaseRouteTurnIndex);
    Serial.print(F("/"));
    Serial.print(kEasyBaseRouteTurnCount);
    Serial.print(F(" mode="));
    Serial.print(followModeName(line.mode));
    Serial.print(F(" active="));
    Serial.print(line.activeCount);
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

inline bool pollEasyBaseExitRfidBurst(String *uidOut) {
  for (uint8_t i = 0; i < kEasyBaseExitRfidBurstPolls; ++i) {
    if (pollEasyRfidRaw(uidOut, true)) return true;
    if (i + 1 >= kEasyBaseExitRfidBurstPolls) break;

    const uint32_t waitStart = millis();
    while (millis() - waitStart < kEasyBaseExitRfidBurstGapMs) {
      if (!easyMovementSafetyOk()) return false;
      delay(1);
    }
  }
  return false;
}

inline bool serviceEasyBasePendingAirlockRequest() {
  if (easyBaseAirlockRequestSent || easyBasePendingAirlockUid.length() == 0) return false;

  const uint32_t now = millis();
  if (easyBaseLastAirlockRequestAttemptMs != 0 &&
      now - easyBaseLastAirlockRequestAttemptMs < kEasyAirlockRequestRetryMs) {
    return false;
  }

  easyBaseLastAirlockRequestAttemptMs = now;
  Serial.print(F("[AIRLOCK] requesting A for base-exit uid="));
  Serial.println(easyBasePendingAirlockUid);

  if (!sendEasyAirlockOpenRequest(easyBasePendingAirlockUid.c_str(), 'A')) {
    Serial.println(F("[AIRLOCK] request not sent yet; will retry."));
    return false;
  }

  easyBaseAirlockRequestSent = true;
  easyBasePendingAirlockUid = "";
  Serial.println(F("[AIRLOCK] request sent; continuing toward tunnel."));
  return true;
}

inline bool checkEasyBaseExitRfidAndRequestAirlock() {
  if (easyBaseAirlockRequestSent) return false;
  if (easyBasePendingAirlockUid.length() > 0) {
    serviceEasyBasePendingAirlockRequest();
    return false;
  }

  String uid;
  if (!pollEasyBaseExitRfidBurst(&uid)) return false;

  stopMotors();
  easyBasePendingAirlockUid = uid;
  easyBaseLastAirlockRequestAttemptMs = 0;
  Serial.print(F("[RFID] base exit tag UID="));
  Serial.println(uid);
  serviceEasyBasePendingAirlockRequest();

  const uint32_t start = millis();
  while (millis() - start < kEasyStopOverAirlockTagMs) {
    if (!easyMovementSafetyOk()) return true;
    delay(20);
  }

  return true;
}

inline EasyDoorReading readEasyTunnelDoor() {
  EasyDoorReading reading;
  reading.frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  reading.valid = isValidSonarDistance(reading.frontMm);
  reading.closed = reading.valid && reading.frontMm <= kEasyDoorClosedThresholdMm;
  reading.open = (reading.valid && reading.frontMm >= kEasyDoorOpenThresholdMm) ||
                 (!reading.valid && kEasyTreatNoEchoAsOpen);
  return reading;
}

inline bool easyDoorClosedStable(const EasyDoorReading &reading) {
  if (reading.closed) {
    if (easyBaseDoorClosedStableCount < 255) easyBaseDoorClosedStableCount++;
  } else {
    easyBaseDoorClosedStableCount = 0;
  }
  return easyBaseDoorClosedStableCount >= kEasyDoorStableFrames;
}

inline bool easyDoorOpenStable(const EasyDoorReading &reading) {
  if (reading.open) {
    if (easyBaseDoorOpenStableCount < 255) easyBaseDoorOpenStableCount++;
  } else {
    easyBaseDoorOpenStableCount = 0;
  }
  return easyBaseDoorOpenStableCount >= kEasyDoorStableFrames;
}

inline void printEasyDoorStatus(const __FlashStringHelper *label, const EasyDoorReading &reading) {
  if (millis() - easyBaseLastDoorPrintMs < kEasyDoorPrintIntervalMs) return;
  easyBaseLastDoorPrintMs = millis();

  Serial.print(label);
  Serial.print(F(" frontMm="));
  Serial.print(reading.frontMm, 1);
  Serial.print(F(" valid="));
  Serial.print(reading.valid ? F("YES") : F("NO"));
  Serial.print(F(" closedStable="));
  Serial.print(easyBaseDoorClosedStableCount);
  Serial.print(F(" openStable="));
  Serial.println(easyBaseDoorOpenStableCount);
}

inline bool easyFirstTDetected(const LineReading &line) {
  if (!line.detected) return false;
  if (easyBaseCurrentRouteSegmentTravelMm() < kEasyFirstTMinTravelMm) return false;
  if (line.activeCount < kEasyFirstTMinActiveSensors) return false;
  if (!centerHasLine()) return false;
  if (abs(line.error) > kEasyFirstTMaxCenterError) return false;

  const bool leftEdgeStrong =
      qtrNorm[0] >= kEasyFirstTEdgeStrongThreshold || qtrNorm[1] >= kEasyFirstTEdgeStrongThreshold;
  const bool rightEdgeStrong =
      qtrNorm[7] >= kEasyFirstTEdgeStrongThreshold || qtrNorm[8] >= kEasyFirstTEdgeStrongThreshold;
  if (!leftEdgeStrong || !rightEdgeStrong) return false;

  uint16_t totalStrength = 0;
  for (uint8_t i = 0; i < 9; ++i) totalStrength += qtrNorm[i];
  return totalStrength >= kEasyFirstTMinTotalStrength;
}

inline bool easyExpectedSharpTurnDetected(const LineReading &line, EasyBaseTurnDir expected) {
  if (!line.detected) return false;
  if (expected == EasyBaseTurnDir::Left) {
    return line.mode == FollowMode::HardLeft ||
           (easyBaseLeftOuterHasStrongLine() && line.error < -kCenterRecoverError);
  }
  return line.mode == FollowMode::HardRight ||
         (easyBaseRightOuterHasStrongLine() && line.error > kCenterRecoverError);
}

inline bool easyBaseRouteEventReady(const LineReading &line) {
  if (easyBaseRouteTurnIndex >= kEasyBaseRouteTurnCount) return false;
  if (millis() - easyBaseLastEventMs < kEasyEventCooldownMs) return false;
  if (easyBasePostTurnHardIgnoreActive && easyBaseIsHardLineMode(line.mode)) {
    easyBaseEventStableCount = 0;
    return false;
  }

  const EasyBaseTurnDir expected = easyBaseRouteTurnAt(easyBaseRouteTurnIndex);
  const bool firstRouteTurn = easyBaseRouteTurnIndex == 0;
  const bool eventNow =
      firstRouteTurn ? easyFirstTDetected(line) : easyExpectedSharpTurnDetected(line, expected);

  if (!eventNow) {
    easyBaseEventStableCount = 0;
    return false;
  }

  if (easyBaseEventStableCount < 255) easyBaseEventStableCount++;
  const uint8_t requiredFrames = firstRouteTurn ? kEasyFirstTStableFrames : kEasySharpTurnStableFrames;
  return easyBaseEventStableCount >= requiredFrames;
}

inline bool reacquireEasyBaseLineAfterTurn(EasyBaseTurnDir dir) {
  const uint32_t start = millis();
  uint8_t stable = 0;
  const int command = dir == EasyBaseTurnDir::Left ? kEasyReacquireTurnSpeed : -kEasyReacquireTurnSpeed;

  while (millis() - start < kEasyReacquireTimeoutMs) {
    if (!easyMovementSafetyOk()) return false;

    const LineReading line = readLine();
    if (line.detected && centerHasLine()) {
      stable++;
      stopMotors();
      if (stable >= kEasyReacquireStableFrames) {
        resetEasyBaseLineController();
        return true;
      }
    } else {
      stable = 0;
      setTurnCommand(command);
    }

    updateImu();
    delay(10);
  }

  stopMotors();
  resetEasyBaseLineController();
  return true;
}

inline bool performEasyBaseRouteTurn() {
  if (easyBaseRouteTurnIndex >= kEasyBaseRouteTurnCount) return true;

  const EasyBaseTurnDir dir = easyBaseRouteTurnAt(easyBaseRouteTurnIndex);
  const bool firstT = easyBaseRouteTurnIndex == 0;
  const float advanceMm = firstT ? kEasyFirstTAdvanceMm : kEasySharpTurnAdvanceMm;

  stopMotors();
  delay(80);

  Serial.print(F("[BASE ROUTE] turnIndex="));
  Serial.print(easyBaseRouteTurnIndex);
  Serial.print(F(" route="));
  Serial.print(easyBaseRouteName());
  Serial.print(F(" dir="));
  Serial.println(easyBaseTurnName(dir));

  if (!driveDistanceEasyMm(advanceMm, kEasyRouteAdvanceSpeed)) return false;
  delay(60);
  if (!turnDegreesEasy(easyBaseDegreesForTurn(dir))) return false;
  delay(80);
  if (!reacquireEasyBaseLineAfterTurn(dir)) return false;

  easyBaseRouteTurnIndex++;
  easyBaseEventStableCount = 0;
  easyBaseLastEventMs = millis();
  beginEasyBasePostTurnHardIgnore();
  return true;
}

inline bool followEasyBaseRoute() {
  resetEncoders();
  resetEasyBaseLineController();

  while (easyBaseRouteTurnIndex < kEasyBaseRouteTurnCount) {
    if (!easyMovementSafetyOk()) return false;
    if (easyBaseRouteTurnIndex >= kEasyAirlockRfidCheckFromTurnIndex) {
      checkEasyBaseExitRfidAndRequestAirlock();
    }

    const LineReading line = readLine();
    if (easyBaseRouteEventReady(line)) {
      if (!performEasyBaseRouteTurn()) return false;
    } else {
      const LineReading followLine = easyBaseSoftenedPostTurnLine(line);
      applyEasyBaseLineCommand(followLine, F("[BASE]"));
      updateEasyBasePostTurnHardIgnore(line);
    }
  }

  Serial.print(F("[BASE ROUTE] complete route="));
  Serial.print(easyBaseRouteName());
  Serial.println(F("; following line to tunnel entry."));
  return true;
}

inline bool followEasyLineToTunnelEntry() {
  easyBaseTunnelNoLineCount = 0;
  resetEasyBaseLineController();
  const uint32_t start = millis();

  while (true) {
    if (!easyMovementSafetyOk()) return false;
    checkEasyBaseExitRfidAndRequestAirlock();

    if (millis() - start > kEasyLineToTunnelTimeoutMs) {
      stopMotors();
      Serial.println(F("[WARN] line-to-tunnel timeout; starting wall follow from current position."));
      resetEasyBaseWallController();
      return true;
    }

    const LineReading line = readLine();
    if (!line.detected) {
      if (easyBaseTunnelNoLineCount < 255) easyBaseTunnelNoLineCount++;
      if (easyBaseTunnelNoLineCount >= kEasyTunnelEntryNoLineFrames) {
        stopMotors();
        if (!easyBaseAirlockRequestSent) {
          Serial.println(F("[WARN] tunnel entry reached before airlock A request was sent."));
        }
        Serial.println(F("[TUNNEL] base line ended; starting wall following."));
        resetEasyBaseWallController();
        return true;
      }

      setTank(kEasyTunnelEntryConfirmSpeed, kEasyTunnelEntryConfirmSpeed);
      updateImu();
      delay(kLineLoopDelayMs);
      continue;
    }

    easyBaseTunnelNoLineCount = 0;
    const LineReading followLine = easyBaseSoftenedPostTurnLine(line);
    applyEasyBaseLineCommand(followLine, F("[TO TUNNEL]"));
    updateEasyBasePostTurnHardIgnore(line);
  }
}

inline bool waitEasyTunnelDoorOpen() {
  stopMotors();
  resetEasyBaseDoorCounters();

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    const EasyDoorReading door = readEasyTunnelDoor();
    const bool opened = easyDoorOpenStable(door);
    printEasyDoorStatus(F("[EXIT DOOR WAIT]"), door);
    if (opened) {
      Serial.println(F("[EXIT DOOR] open; searching for field line."));
      return true;
    }

    delay(20);
  }
}

inline bool driveEasyStraightUntilFieldLine() {
  uint8_t stable = 0;
  const uint32_t start = millis();

  while (millis() - start < kEasyPostDoorLineTimeoutMs) {
    if (!easyMovementSafetyOk()) return false;

    const LineReading line = readLine();
    if (line.detected) {
      if (stable < 255) stable++;
      if (stable >= kEasyWallExitLineStableFrames) {
        stopMotors();
        Serial.println(F("[FIELD] QTR line found after tunnel door."));
        resetEasyBaseLineController();
        return true;
      }
    } else {
      stable = 0;
    }

    setTank(kEasyTunnelEntryConfirmSpeed, kEasyTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
  }

  stopMotors();
  Serial.println(F("[WARN] field line search timeout after tunnel door; handing off to RFID search anyway."));
  resetEasyBaseLineController();
  return true;
}

inline bool wallFollowEasyTunnelToFieldLine() {
  easyBaseWallExitLineStableCount = 0;
  resetEasyBaseDoorCounters();
  resetEasyBaseWallController();
  const uint32_t start = millis();

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    if (millis() - start > kEasyWallTunnelTimeoutMs) {
      stopMotors();
      Serial.println(F("[WARN] wall-follow tunnel timeout; handing off to RFID search."));
      resetEasyBaseLineController();
      return true;
    }

    const LineReading line = readLine();
    if (line.detected) {
      if (easyBaseWallExitLineStableCount < 255) easyBaseWallExitLineStableCount++;
      if (easyBaseWallExitLineStableCount >= kEasyWallExitLineStableFrames) {
        stopMotors();
        Serial.println(F("[TUNNEL EXIT] QTR line found; field handoff ready."));
        resetEasyBaseLineController();
        return true;
      }
    } else {
      easyBaseWallExitLineStableCount = 0;
    }

    const EasyDoorReading door = readEasyTunnelDoor();
    const bool closed = easyDoorClosedStable(door);
    printEasyDoorStatus(F("[EXIT DOOR APPROACH]"), door);
    if (closed) {
      stopMotors();
      Serial.println(F("[EXIT DOOR] closed; waiting for open."));
      if (!waitEasyTunnelDoorOpen()) return false;
      return driveEasyStraightUntilFieldLine();
    }

    applyWallFollowStep(kEasyTunnelWallSide);
  }
}

inline bool followEasyReturnLineToTunnelEntry() {
  easyBaseReturnTunnelNoLineCount = 0;
  resetEasyBaseLineController();
  const uint32_t start = millis();

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    if (millis() - start > kEasyLineToTunnelTimeoutMs) {
      stopMotors();
      Serial.println(F("[RETURN] line-to-tunnel timeout after B request; starting return wall follow."));
      resetEasyBaseWallController();
      return true;
    }

    const LineReading line = readLine();
    if (!line.detected) {
      if (easyBaseReturnTunnelNoLineCount < 255) easyBaseReturnTunnelNoLineCount++;
      if (easyBaseReturnTunnelNoLineCount >= kEasyTunnelEntryNoLineFrames) {
        stopMotors();
        Serial.println(F("[RETURN] line disappeared for 6 frames; starting return wall follow."));
        resetEasyBaseWallController();
        return true;
      }

      setTank(kEasyTunnelEntryConfirmSpeed, kEasyTunnelEntryConfirmSpeed);
      updateImu();
      delay(kLineLoopDelayMs);
      continue;
    }

    easyBaseReturnTunnelNoLineCount = 0;
    applyEasyBaseLineCommand(line, F("[RETURN TO TUNNEL]"));
  }
}

inline bool wallFollowEasyReturnTunnelToBaseLine() {
  easyBaseReturnWallExitLineStableCount = 0;
  resetEasyBaseWallController();

  while (true) {
    if (!easyMovementSafetyOk()) return false;

    const LineReading line = readLine();
    if (line.detected) {
      if (easyBaseReturnWallExitLineStableCount < 255) easyBaseReturnWallExitLineStableCount++;
      if (easyBaseReturnWallExitLineStableCount >= kEasyReturnWallExitLineStableFrames) {
        stopMotors();
        Serial.println(F("[RETURN] QTR line found; leaving wall following at base line."));
        resetEasyBaseLineController();
        return true;
      }
    } else {
      easyBaseReturnWallExitLineStableCount = 0;
    }

    applyWallFollowStep(kEasyTunnelWallSide);
  }
}

inline bool executeEasyReturnTunnelToBaseLine() {
  Serial.println(F("[RETURN] following line to return tunnel."));
  if (!followEasyReturnLineToTunnelEntry()) return false;

  Serial.println(F("[RETURN] wall-following return tunnel."));
  if (!wallFollowEasyReturnTunnelToBaseLine()) return false;

  stopMotors();
  Serial.println(F("[RETURN] base line reached."));
  return true;
}

inline void initializeEasyBaseExitController() {
  easyBaseRouteTurnIndex = 0;
  easyBaseEventStableCount = 0;
  easyBaseDoorClosedStableCount = 0;
  easyBaseDoorOpenStableCount = 0;
  easyBaseTunnelNoLineCount = 0;
  easyBaseWallExitLineStableCount = 0;
  easyBaseReturnTunnelNoLineCount = 0;
  easyBaseReturnWallExitLineStableCount = 0;
  easyBasePostTurnHardIgnoreActive = false;
  easyBasePostTurnHardReleaseCount = 0;
  easyBaseLastEventMs = 0;
  easyBaseLastDoorPrintMs = 0;
  easyBaseAirlockRequestSent = false;
  easyBasePendingAirlockUid = "";
  easyBaseLastAirlockRequestAttemptMs = 0;
}

inline bool executeEasyBaseToField() {
  Serial.print(F("[BASE EXIT] starting route="));
  Serial.print(easyBaseRouteName());
  Serial.print(F(" wallSide="));
  Serial.println(kEasyTunnelWallSide == WallSide::Left ? F("LEFT") : F("RIGHT"));

  initializeEasyBaseExitController();

  if (!followEasyBaseRoute()) return false;
  if (!followEasyLineToTunnelEntry()) return false;
  if (!wallFollowEasyTunnelToFieldLine()) return false;

  stopMotors();
  Serial.println(F("[BASE EXIT] field line reached; starting easy field mission."));
  return true;
}
