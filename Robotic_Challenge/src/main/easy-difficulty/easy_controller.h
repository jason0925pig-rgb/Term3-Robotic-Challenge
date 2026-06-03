#pragma once

#include <Arduino.h>

#include "easy_navigation.h"
#include "easy_planting.h"

static MissionState easyMissionState = MissionState::Init;
static uint8_t easyRouteIndex = 0;
static uint8_t easySeedsRemaining = kEasyInitialSeedCount;
static Cell easyCurrentCell = kEasyStartCell;
static Direction easyCurrentHeading = kEasyInitialHeading;
static CellStatus easyLatestCellStatus = {};
static String easyCurrentUid;
static bool easyHasArrivalUid = false;
static String easyArrivalUid;
static bool easyCompleteAnnounced = false;
static bool easyErrorAnnounced = false;

inline const __FlashStringHelper *easyMissionStateName(MissionState state) {
  switch (state) {
    case MissionState::Init: return F("INIT");
    case MissionState::AlignToFirstSegment: return F("ALIGN_TO_FIRST_SEGMENT");
    case MissionState::SearchForRFID: return F("SEARCH_FOR_RFID");
    case MissionState::ReadCurrentCell: return F("READ_CURRENT_CELL");
    case MissionState::QueryServer: return F("QUERY_SERVER");
    case MissionState::DecidePlanting: return F("DECIDE_PLANTING");
    case MissionState::PlantSeed: return F("PLANT_SEED");
    case MissionState::AdvanceRouteIndex: return F("ADVANCE_ROUTE_INDEX");
    case MissionState::MoveToNextCell: return F("MOVE_TO_NEXT_CELL");
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

inline bool loadExpectedUidForCell(Cell cell, String *uidOut) {
  const char *uid = uidForCell(cell);
  if (strlen(uid) == 0) return false;
  *uidOut = uid;
  return true;
}

inline bool readAndConfirmCurrentCell() {
  String uid;

  if (easyHasArrivalUid) {
    uid = easyArrivalUid;
    easyHasArrivalUid = false;
  } else if (!readCurrentEasyRfid(&uid)) {
    if (!loadExpectedUidForCell(easyCurrentCell, &uid)) {
      Serial.print(F("[RFID] no RFID and no expected UID for "));
      printCell(easyCurrentCell);
      Serial.println();
      return false;
    }

    Serial.print(F("[RFID] no live read at "));
    printCell(easyCurrentCell);
    Serial.print(F("; using expected UID="));
    Serial.println(uid);
  }

  Cell seenCell = {};
  if (!cellForUid(uid.c_str(), &seenCell)) {
    Serial.print(F("[RFID] unknown current UID="));
    Serial.println(uid);
    return false;
  }

  const Cell expectedCell = kEasyFixedRoute[easyRouteIndex];
  if (!sameCell(seenCell, expectedCell)) {
    Serial.print(F("[RFID] routeIndex="));
    Serial.print(easyRouteIndex);
    Serial.print(F(" expected "));
    printCell(expectedCell);
    Serial.print(F(" but saw "));
    printCell(seenCell);
    Serial.print(F(" uid="));
    Serial.println(uid);
    return false;
  }

  easyCurrentCell = seenCell;
  easyCurrentUid = uid;

  Serial.print(F("[RFID] current "));
  printCell(easyCurrentCell);
  Serial.print(F(" uid="));
  Serial.print(easyCurrentUid);
  Serial.print(F(" routeIndex="));
  Serial.println(easyRouteIndex);
  return true;
}

inline bool plantAtCurrentEasyCell() {
  Serial.print(F("[PLANT] eligible at "));
  printCell(easyCurrentCell);
  Serial.println(F("; centering over hole."));

  if (!driveDistanceEasyMm(kEasyPlantCenterOffsetMm, kEasyPlantCenterDriveSpeed)) {
    Serial.println(F("[PLANT] center drive failed."));
    return false;
  }

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

inline void initializeEasyMissionController() {
  easyMissionState = MissionState::Init;
  easyRouteIndex = 0;
  easySeedsRemaining = kEasyInitialSeedCount;
  easyCurrentCell = kEasyStartCell;
  easyCurrentHeading = kEasyInitialHeading;
  easyLatestCellStatus = {};
  easyCurrentUid = "";
  easyHasArrivalUid = false;
  easyArrivalUid = "";
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
      if (!validateEasyRoute(true)) {
        setEasyError(F("route_validation_failed"));
        break;
      }
      easyRouteIndex = 0;
      easyCurrentCell = kEasyFixedRoute[0];
      easyCurrentHeading = kEasyInitialHeading;
      easySeedsRemaining = kEasyInitialSeedCount;
      setEasyMissionState(MissionState::AlignToFirstSegment);
      break;

    case MissionState::AlignToFirstSegment:
      if (kEasyFixedRouteLength < 2) {
        setEasyError(F("route_too_short"));
        break;
      }
      if (!alignToDirection(&easyCurrentHeading, directionFromTo(kEasyFixedRoute[0], kEasyFixedRoute[1]))) {
        setEasyError(F("initial_align_failed"));
        break;
      }
      setEasyMissionState(MissionState::SearchForRFID);
      break;

    case MissionState::SearchForRFID:
      if (!searchLineUntilAnyEasyRfid(&easyArrivalUid)) {
        setEasyError(F("initial_rfid_search_failed"));
        break;
      }
      easyHasArrivalUid = true;
      setEasyMissionState(MissionState::ReadCurrentCell);
      break;

    case MissionState::ReadCurrentCell:
      if (!readAndConfirmCurrentCell()) {
        setEasyError(F("read_current_cell_failed"));
        break;
      }
      setEasyMissionState(MissionState::QueryServer);
      break;

    case MissionState::QueryServer:
      if (!queryServerForCellStatus(easyCurrentUid.c_str(), &easyLatestCellStatus)) {
        easyLatestCellStatus = {};
        Serial.println(F("[PLANT] server unavailable; skip planting at this cell."));
      }
      setEasyMissionState(MissionState::DecidePlanting);
      break;

    case MissionState::DecidePlanting:
      if (shouldPlantSeed(easyLatestCellStatus, easySeedsRemaining)) {
        setEasyMissionState(MissionState::PlantSeed);
      } else {
        Serial.print(F("[PLANT] skip at "));
        printCell(easyCurrentCell);
        Serial.print(F(" fertile="));
        Serial.print(easyLatestCellStatus.isFertile ? F("true") : F("false"));
        Serial.print(F(" planted="));
        Serial.print(easyLatestCellStatus.alreadyPlanted ? F("true") : F("false"));
        Serial.print(F(" seedsRemaining="));
        Serial.println(easySeedsRemaining);
        setEasyMissionState(MissionState::AdvanceRouteIndex);
      }
      break;

    case MissionState::PlantSeed:
      if (!plantAtCurrentEasyCell()) {
        setEasyError(F("plant_failed"));
        break;
      }
      setEasyMissionState(MissionState::AdvanceRouteIndex);
      break;

    case MissionState::AdvanceRouteIndex:
      if (easyRouteIndex + 1 >= kEasyFixedRouteLength) {
        setEasyMissionState(MissionState::Finished);
      } else {
        setEasyMissionState(MissionState::MoveToNextCell);
      }
      break;

    case MissionState::MoveToNextCell: {
      const Cell from = kEasyFixedRoute[easyRouteIndex];
      const Cell to = kEasyFixedRoute[easyRouteIndex + 1];
      const EasyMoveResult result =
          moveToNextEasyCell(from, to, easyCurrentUid, &easyCurrentHeading);

      if (!result.ok) {
        setEasyError(result.reason);
        break;
      }

      ++easyRouteIndex;
      easyCurrentCell = to;
      if (result.sawUid) {
        easyArrivalUid = result.uid;
        easyHasArrivalUid = true;
      } else if (loadExpectedUidForCell(to, &easyArrivalUid)) {
        easyHasArrivalUid = true;
      } else {
        setEasyError(F("missing_expected_uid_after_move"));
        break;
      }

      setEasyMissionState(MissionState::ReadCurrentCell);
      break;
    }

    case MissionState::Finished:
      stopMotors();
      if (!easyCompleteAnnounced) {
        easyCompleteAnnounced = true;
        Serial.print(F("[DONE] Easy fixed-route mission complete at "));
        printCell(easyCurrentCell);
        Serial.print(F(" seedsRemaining="));
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
  }
}
