#pragma once

#include <Arduino.h>

#include "easy_base_exit.h"
#include "easy_navigation.h"
#include "easy_planting.h"

constexpr uint32_t kEasyEndNoNewRfidTimeoutMs = 3500;
constexpr uint32_t kEasyScriptSegmentTimeoutPerTagMs = 9000;
constexpr uint32_t kEasyPostTurnRfidIgnoreMs = 1000;

enum class EasyGridTurnDir : uint8_t {
  Left,
  Right
};

struct EasyGridRouteStep {
  uint8_t rfidCount;
  EasyGridTurnDir turnAfter;
  bool requestDoorAfterTurn;
};

constexpr EasyGridRouteStep kEasyGridRoute[] = {
    {2, EasyGridTurnDir::Left, false},
    {3, EasyGridTurnDir::Left, false},
    {1, EasyGridTurnDir::Left, false},
    {2, EasyGridTurnDir::Right, false},
    {1, EasyGridTurnDir::Right, false},
    {2, EasyGridTurnDir::Left, false},
    {1, EasyGridTurnDir::Left, false},
    {3, EasyGridTurnDir::Right, false},
    {1, EasyGridTurnDir::Right, false},
    {3, EasyGridTurnDir::Left, false},
    {1, EasyGridTurnDir::Left, false},
    {3, EasyGridTurnDir::Right, false},
    {1, EasyGridTurnDir::Left, true},
};
constexpr uint8_t kEasyGridRouteLength = sizeof(kEasyGridRoute) / sizeof(kEasyGridRoute[0]);

static MissionState easyMissionState = MissionState::Init;
static uint8_t easySeedsRemaining = kEasyInitialSeedCount;
static Cell easyCurrentCell = {};
static CellStatus easyLatestCellStatus = {};
static String easyCurrentUid;
static String easyLastProcessedUid;
static uint32_t easyLastGridTurnMs = 0;
static bool easyCompleteAnnounced = false;
static bool easyErrorAnnounced = false;

inline const __FlashStringHelper *easyMissionStateName(MissionState state) {
  switch (state) {
    case MissionState::Init: return F("INIT");
    case MissionState::ExitBaseToField: return F("EXIT_BASE_TO_FIELD");
    case MissionState::AlignToFirstSegment: return F("ALIGN_TO_FIRST_SEGMENT");
    case MissionState::SearchForRFID: return F("SEARCH_FOR_RFID");
    case MissionState::ReadCurrentCell: return F("READ_CURRENT_CELL");
    case MissionState::QueryServer: return F("QUERY_SERVER");
    case MissionState::DecidePlanting: return F("DECIDE_PLANTING");
    case MissionState::PlantSeed: return F("PLANT_SEED");
    case MissionState::AdvanceRouteIndex: return F("ADVANCE_ROUTE_INDEX");
    case MissionState::MoveToNextCell: return F("MOVE_TO_NEXT_CELL");
    case MissionState::ExecuteScript: return F("EXECUTE_SCRIPT");
    case MissionState::Finished: return F("FINISHED");
    case MissionState::Error: return F("ERROR");
    default: return F("UNKNOWN");
  }
}

inline void setEasyMissionState(MissionState nextState) {
  easyMissionState = nextState;
  Serial.print(F("[EASY] state="));
  Serial.println(easyMissionStateName(easyMissionState));
}

inline bool shouldPlantSeed(CellStatus status, uint8_t seedsRemaining) {
  return status.valid && status.isFertile && !status.alreadyPlanted && seedsRemaining > 0;
}

inline void setEasyError(const __FlashStringHelper *reason) {
  stopMotors();
  Serial.print(F("[EASY] error: "));
  Serial.println(reason);
  setEasyMissionState(MissionState::Error);
}

inline bool centerOverDetectedEasyRfid() {
  Serial.println(F("[NAV] center over detected RFID."));
  return driveDistanceEasyMm(kEasyPlantCenterOffsetMm, kEasyPlantCenterDriveSpeed);
}

inline bool plantAtCurrentEasyUid() {
  Serial.print(F("[PLANT] eligible uid="));
  Serial.print(easyCurrentUid);
  Serial.println(F("; dropping seed."));

  if (!dropOneEasySeed()) {
    Serial.println(F("[PLANT] seed drop failed or stopped."));
    return false;
  }

  --easySeedsRemaining;
  notifyServerSeedPlanted(easyCurrentUid.c_str(), easyCurrentCell);

  Serial.print(F("[PLANT] seedsRemaining="));
  Serial.print(easySeedsRemaining);
  Serial.print(F("/"));
  Serial.println(kEasyInitialSeedCount);
  return true;
}

inline bool processEasyRfidUid(const String &uid, bool *plantedOut) {
  if (plantedOut) *plantedOut = false;
  if (uid.length() == 0) return false;

  easyCurrentUid = uid;
  easyLastProcessedUid = uid;
  easyCurrentCell = {};
  Cell mappedCell = {};
  const bool knownUid = cellForUid(uid.c_str(), &mappedCell);
  if (knownUid) easyCurrentCell = mappedCell;

  Serial.print(F("[RFID] uid="));
  Serial.print(uid);
  if (knownUid) {
    Serial.print(F(" mapped="));
    printCell(mappedCell);
  } else {
    Serial.print(F(" mapped=unknown"));
  }
  Serial.println(F(" navigation=ignored"));

  if (!queryServerForCellStatus(uid.c_str(), &easyLatestCellStatus)) {
    easyLatestCellStatus = {};
    Serial.println(F("[PLANT] server unavailable; skip planting for this UID."));
    return true;
  }

  if (!shouldPlantSeed(easyLatestCellStatus, easySeedsRemaining)) {
    Serial.print(F("[PLANT] skip uid="));
    Serial.print(uid);
    Serial.print(F(" fertile="));
    Serial.print(easyLatestCellStatus.isFertile ? F("true") : F("false"));
    Serial.print(F(" planted="));
    Serial.print(easyLatestCellStatus.alreadyPlanted ? F("true") : F("false"));
    Serial.print(F(" seedsRemaining="));
    Serial.println(easySeedsRemaining);
    return true;
  }

  if (!plantAtCurrentEasyUid()) return false;
  if (plantedOut) *plantedOut = true;
  return true;
}

inline bool suppressEasyRfidAfterTurn() {
  return easyLastGridTurnMs != 0 && millis() - easyLastGridTurnMs < kEasyPostTurnRfidIgnoreMs;
}

inline const __FlashStringHelper *easyGridTurnName(EasyGridTurnDir dir) {
  return dir == EasyGridTurnDir::Left ? F("LEFT") : F("RIGHT");
}

inline bool turnEasyGrid(EasyGridTurnDir dir) {
  Serial.print(F("[SCRIPT] turn "));
  Serial.println(easyGridTurnName(dir));
  stopMotors();
  delay(120);

  const float degrees = dir == EasyGridTurnDir::Left ? 90.0f : -90.0f;
  const bool ok = turnDegreesEasy(degrees);
  delay(120);
  lineIntegral = 0.0f;
  lastLineError = 0;
  if (ok) easyLastGridTurnMs = millis();
  return ok;
}

inline bool turnEasyLeft() {
  return turnEasyGrid(EasyGridTurnDir::Left);
}

inline bool turnEasyRight() {
  return turnEasyGrid(EasyGridTurnDir::Right);
}

inline bool driveLineAndProcessTags(uint8_t targetTags, bool allowEndByNoNewRfid,
                                    const __FlashStringHelper *label) {
  Serial.print(F("[SCRIPT] drive "));
  Serial.print(label);
  Serial.print(F(" targetTags="));
  Serial.print(targetTags);
  Serial.print(F(" allowEnd="));
  Serial.println(allowEndByNoNewRfid ? F("YES") : F("NO"));

  lineIntegral = 0.0f;
  lastLineError = 0;

  uint8_t tagsSeen = 0;
  uint32_t lastNewTagMs = millis();
  const uint32_t timeoutMs =
      targetTags == 0 ? kEasyScriptSegmentTimeoutPerTagMs
                      : targetTags * kEasyScriptSegmentTimeoutPerTagMs;
  const uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (!easyMovementSafetyOk()) return false;

    if (!suppressEasyRfidAfterTurn()) {
      String uid;
      if (pollEasyRfidDebounced(&uid)) {
        if (uid == easyLastProcessedUid) {
          applyLineFollowStep();
          continue;
        }

        stopMotors();
        ++tagsSeen;

        if (!centerOverDetectedEasyRfid()) return false;
        if (!processEasyRfidUid(uid, nullptr)) return false;
        lastNewTagMs = millis();

        if (tagsSeen >= targetTags) {
          Serial.print(F("[SCRIPT] segment complete tagsSeen="));
          Serial.println(tagsSeen);
          return true;
        }
      }
    }

    if (allowEndByNoNewRfid && tagsSeen > 0 &&
        millis() - lastNewTagMs >= kEasyEndNoNewRfidTimeoutMs) {
      stopMotors();
      Serial.print(F("[SCRIPT] field end inferred tagsSeen="));
      Serial.println(tagsSeen);
      return true;
    }

    applyLineFollowStep();
  }

  stopMotors();
  Serial.print(F("[SCRIPT] segment timeout tagsSeen="));
  Serial.println(tagsSeen);
  return allowEndByNoNewRfid && tagsSeen > 0;
}

inline bool executeHardcodedEasyStrategy() {
  if (!turnEasyRight()) return false;

  for (uint8_t i = 0; i < kEasyGridRouteLength; ++i) {
    const EasyGridRouteStep step = kEasyGridRoute[i];
    Serial.print(F("[SCRIPT] grid step="));
    Serial.print(i + 1);
    Serial.print(F("/"));
    Serial.print(kEasyGridRouteLength);
    Serial.print(F(" count="));
    Serial.print(step.rfidCount);
    Serial.print(F(" turn="));
    Serial.println(easyGridTurnName(step.turnAfter));

    if (!driveLineAndProcessTags(step.rfidCount, false, F("grid step"))) return false;
    if (!turnEasyGrid(step.turnAfter)) return false;

    if (step.requestDoorAfterTurn) {
      Serial.println(F("[SCRIPT] final grid turn complete; requesting airlock B."));
      const String uid = easyLastProcessedUid.length() > 0 ? easyLastProcessedUid : easyCurrentUid;
      if (uid.length() == 0) {
        Serial.println(F("[AIRLOCK] no grid RFID UID available for return airlock B request; starting return anyway."));
      } else {
        while (!sendEasyAirlockOpenRequest(uid.c_str(), 'B')) {
          if (!easyMovementSafetyOk()) return false;
          Serial.println(F("[AIRLOCK] return airlock B request not sent yet; retrying."));
          const uint32_t waitStart = millis();
          while (millis() - waitStart < kEasyAirlockRequestRetryMs) {
            if (!easyMovementSafetyOk()) return false;
            delay(20);
          }
        }
        Serial.println(F("[AIRLOCK] return airlock B request sent."));
      }

      if (!executeEasyReturnTunnelToBaseLine()) return false;
    }
  }

  stopMotors();
  return true;
}

inline void initializeEasyMissionController() {
  easyMissionState = MissionState::Init;
  easySeedsRemaining = kEasyInitialSeedCount;
  easyCurrentCell = {};
  easyLatestCellStatus = {};
  easyCurrentUid = "";
  easyLastProcessedUid = "";
  easyLastGridTurnMs = 0;
  easyCompleteAnnounced = false;
  easyErrorAnnounced = false;
}

inline void runEasyMissionStep() {
  easyMovementSafetyOk();

  if (killPressed() || serialStopped || !easyMotionAllowedByWifi()) {
    stopMotors();
    return;
  }

  switch (easyMissionState) {
    case MissionState::Init:
      stopMotors();
      easySeedsRemaining = kEasyInitialSeedCount;
      easyCurrentUid = "";
      easyLastProcessedUid = "";
      easyLastGridTurnMs = 0;
      initializeEasyBaseExitController();
      setEasyMissionState(MissionState::ExitBaseToField);
      break;

    case MissionState::ExitBaseToField:
      if (!executeEasyBaseToField()) {
        setEasyError(F("base_exit_failed"));
        break;
      }
      setEasyMissionState(MissionState::SearchForRFID);
      break;

    case MissionState::SearchForRFID: {
      String uid;
      if (!searchLineUntilAnyEasyRfid(&uid)) {
        setEasyError(F("initial_rfid_search_failed"));
        break;
      }

      if (!centerOverDetectedEasyRfid()) {
        setEasyError(F("initial_center_failed"));
        break;
      }

      if (!processEasyRfidUid(uid, nullptr)) {
        setEasyError(F("initial_rfid_process_failed"));
        break;
      }

      setEasyMissionState(MissionState::ExecuteScript);
      break;
    }

    case MissionState::ExecuteScript:
      if (!executeHardcodedEasyStrategy()) {
        setEasyError(F("script_failed"));
        break;
      }
      setEasyMissionState(MissionState::Finished);
      break;

    case MissionState::Finished:
      stopMotors();
      if (!easyCompleteAnnounced) {
        easyCompleteAnnounced = true;
        Serial.print(F("[DONE] Easy hard-coded line strategy complete. seedsRemaining="));
        Serial.println(easySeedsRemaining);
      }
      break;

    case MissionState::Error:
      stopMotors();
      if (!easyErrorAnnounced) {
        easyErrorAnnounced = true;
        Serial.println(F("[DONE] Easy mission stopped in ERROR. Reset or upload a fix to run again."));
      }
      break;

    case MissionState::AlignToFirstSegment:
    case MissionState::ReadCurrentCell:
    case MissionState::QueryServer:
    case MissionState::DecidePlanting:
    case MissionState::PlantSeed:
    case MissionState::AdvanceRouteIndex:
    case MissionState::MoveToNextCell:
      setEasyError(F("unused_route_state"));
      break;
  }
}
