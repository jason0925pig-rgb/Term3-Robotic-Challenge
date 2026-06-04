#pragma once

#include "hard_config.h"

inline bool driveForwardUntilFirstRfid() {
  resetEncoders();
  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;

  while (millis() - start < kRfidSearchTimeoutMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    String uid;
    if (pollRfid(&uid, true)) {
      lastUid = uid;
      stopMotors();
      Serial.print(F("[RFID] first outside tag UID="));
      Serial.println(lastUid);
      return true;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
    setTank(kPostTunnelSearchSpeed - correction, kPostTunnelSearchSpeed + correction);

    if (millis() - lastPrintMs >= 250) {
      lastPrintMs = millis();
      Serial.print(F("[RFID SEARCH] L="));
      Serial.print(getLeftCount());
      Serial.print(F(" R="));
      Serial.println(getRightCount());
    }

    updateImu();
    delay(10);
  }

  stopMotors();
  Serial.println(F("[RFID SEARCH] timeout before first RFID tag."));
  return false;
}

// ---------------------------------------------------------------------------
// Post-tunnel fixed grid route
// ---------------------------------------------------------------------------

/**
 * @return true if this UID was already handled recently.
 */
inline bool isDuplicateRecentGridUid(const String &uid) {
  return uid == lastGridUid && millis() - lastGridUidMs < kGridRfidCooldownMs;
}

/**
 * @return true while grid RFID reads should be ignored after a scripted turn.
 */
inline bool suppressGridRfidAfterTurn() {
  return lastGridTurnMs != 0 && millis() - lastGridTurnMs < kPostTurnRfidIgnoreMs;
}

/**
 * Drive from the RFID reader position to the hole-center position, then query
 * the server and plant if the cell is fertile and unplanted.
 *
 * @param uid RFID tag UID.
 * @return true when the UID was handled; false if it was a duplicate or stop.
 */
inline bool handleGridRfidNode(const String &uid) {
  if (isDuplicateRecentGridUid(uid)) {
    return false;
  }

  lastUid = uid;
  lastGridUid = uid;
  lastGridUidMs = millis();

  stopMotors();
  Serial.print(F("[GRID RFID] uid="));
  Serial.println(uid);
  Serial.println(F("[GRID RFID] driving 75 mm to hole center before server query."));

  if (!driveDistanceMm(kGridPlantCenterOffsetMm, kGridPlantOffsetSpeed)) {
    stopMotors();
    return false;
  }

  stopMotors();
  Serial.println(F("[GRID RFID] centered over hole; waiting for isFertileReply."));

  CellStatus status = {};
  const bool gotReply = queryServerForCellStatus(uid, &status);
  const bool eligible = gotReply ? (status.fertile && !status.planted) : kPlantIfNoServerReply;

  if (!eligible) {
    Serial.println(F("[PLANT] not eligible or no server reply; no seed dropped."));
    lastGridUidMs = millis();
    return true;
  }

  if (seedsPlanted >= kMaxSeedsToPlant) {
    Serial.println(F("[PLANT] seed limit reached; route continues without dropping."));
    lastGridUidMs = millis();
    return true;
  }

  dropOneSeed();
  seedsPlanted++;
  notifySeedPlanted(uid);
  Serial.print(F("[PLANT] seedsPlanted="));
  Serial.print(seedsPlanted);
  Serial.print(F("/"));
  Serial.println(kMaxSeedsToPlant);
  lastGridUidMs = millis();
  return true;
}

/**
 * Perform one easy-route turn and reset line controller afterwards.
 *
 * @param dir Turn direction.
 * @return true if the turn completed.
 */
inline bool performEasyTurn(TurnDir dir) {
  Serial.print(F("[EASY ROUTE] turn "));
  Serial.println(turnName(dir));
  stopMotors();
  delay(120);
  const bool ok = turnDegreesImu(degreesForTurn(dir));
  delay(120);
  resetLineController();
  if (ok) {
    lastGridTurnMs = millis();
  }
  return ok;
}

/**
 * Follow the rediscovered line after the tunnel until the first grid RFID.
 *
 * The first grid RFID is handled as a planting candidate, then the robot turns
 * right and starts the fixed route from "2 RFIDs, left".
 */
inline void updateFollowLineToFirstGridRfid() {
  String uid;
  if (pollRfid(&uid, true)) {
    if (handleGridRfidNode(uid)) {
      if (!performEasyTurn(TurnDir::Right)) {
        serialStopped = true;
        setState(MissionState::Stopped);
        return;
      }
      easyRouteIndex = 0;
      easySegmentRfidCount = 0;
      setState(MissionState::EasyGridRoute);
      return;
    }
  }

  const LineReading line = readLine();
  if (!line.detected) {
    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  applyLineCommand(line, F("[GRID FIRST]"));
  delay(kLineLoopDelayMs);
}

/**
 * Follow the hard-coded RFID-count route after the tunnel.
 */
inline void updateEasyGridRoute() {
  if (easyRouteIndex >= kEasyRouteLength) {
    setState(MissionState::EasyAfterDoorForward);
    return;
  }

  if (!suppressGridRfidAfterTurn()) {
    String uid;
    if (pollRfid(&uid, true)) {
      if (handleGridRfidNode(uid)) {
        easySegmentRfidCount++;
        const EasyRouteStep step = kEasyRoute[easyRouteIndex];

        Serial.print(F("[EASY ROUTE] step="));
        Serial.print(easyRouteIndex + 1);
        Serial.print(F("/"));
        Serial.print(kEasyRouteLength);
        Serial.print(F(" count="));
        Serial.print(easySegmentRfidCount);
        Serial.print(F("/"));
        Serial.println(step.rfidCount);

        if (easySegmentRfidCount >= step.rfidCount) {
          easySegmentRfidCount = 0;
          if (!performEasyTurn(step.turnAfter)) {
            serialStopped = true;
            setState(MissionState::Stopped);
            return;
          }

          easyRouteIndex++;
          if (step.requestDoorAfterTurn) {
            Serial.println(F("[EASY ROUTE] final turn complete; requesting airlock with last RFID UID."));
            setState(MissionState::EasyDoorRequest);
            return;
          }
        }
      }
    }
  }

  const LineReading line = readLine();
  if (!line.detected) {
    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  applyLineCommand(line, F("[EASY ROUTE]"));
  delay(kLineLoopDelayMs);
}

/**
 * Send the final openAirlock request after the fixed route.
 */
inline void updateEasyDoorRequest() {
  stopMotors();
  const String uid = lastGridUid.length() > 0 ? lastGridUid : lastUid;
  if (uid.length() == 0) {
    Serial.println(F("[AIRLOCK] no RFID UID available for return airlock B request; starting return line search anyway."));
    setState(MissionState::ReturnLineToTunnelEntry);
    return;
  }

  if (!easyDoorRequestSent) {
    easyDoorRequestSent = sendAirlockOpenRequest(uid, 'B');
  }

  if (easyDoorRequestSent) {
    Serial.println(F("[AIRLOCK] return airlock B request sent; follow line until tunnel/no-line handoff."));
    setState(MissionState::ReturnLineToTunnelEntry);
    return;
  }

  Serial.println(F("[AIRLOCK] return airlock B request not sent yet; retrying."));
  delay(kAirlockRequestRetryMs);
}

/**
 * Continue forward after the final door request. RFID nodes are still queried
 * and planted if eligible, but no more scripted turns are performed.
 */
inline void updateEasyAfterDoorForward() {
  String uid;
  if (pollRfid(&uid, true)) {
    handleGridRfidNode(uid);
  }

  const LineReading line = readLine();
  if (!line.detected) {
    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  applyLineCommand(line, F("[AFTER DOOR]"));
  delay(kLineLoopDelayMs);
}
