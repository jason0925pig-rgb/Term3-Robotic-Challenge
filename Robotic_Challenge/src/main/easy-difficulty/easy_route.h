#pragma once

#include <Arduino.h>
#include <string.h>

#include "easy_types.h"

// Server cells are stored as numeric x/y coordinates
constexpr Cell kEasyStartCell = {9, 3};
constexpr Cell kEasyFinishCell = {1, 9};
constexpr Direction kEasyInitialHeading = Direction::West;
constexpr uint8_t kEasyInitialSeedCount = 5;
constexpr uint8_t kEasyLineFollowMinX = 6;

constexpr Cell kEasyFixedRoute[] = {
  {9, 3},  {9, 2},  {9, 1},  {8, 1},  {7, 1},  {6, 1},
  {5, 1},  {4, 1},  {3, 1},  {2, 1},  {1, 1},  {1, 2},
  {2, 2},  {3, 2},  {4, 2},  {5, 2},  {6, 2},  {7, 2},
  {8, 2},  {8, 3},  {7, 3},  {6, 3},  {5, 3},  {4, 3},
  {3, 3},  {2, 3},  {1, 3},  {1, 4},  {2, 4},  {3, 4},
  {4, 4},  {5, 4},  {6, 4},  {7, 4},  {8, 4},  {9, 4},
  {9, 5},  {8, 5},  {7, 5},  {6, 5},  {5, 5},  {4, 5},
  {3, 5},  {2, 5},  {1, 5},  {1, 6},  {2, 6},  {3, 6},
  {4, 6},  {5, 6},  {6, 6},  {7, 6},  {8, 6},  {9, 6},
  {9, 7},  {8, 7},  {7, 7},  {6, 7},  {5, 7},  {4, 7},
  {3, 7},  {2, 7},  {1, 7},  {1, 8},  {2, 8},  {3, 8},
  {4, 8},  {5, 8},  {6, 8},  {7, 8},  {8, 8},  {9, 8},
  {9, 9},  {8, 9},  {7, 9},  {6, 9},  {5, 9},  {4, 9},
  {3, 9},  {2, 9},  {1, 9},
};

constexpr uint8_t kEasyFixedRouteLength =
    sizeof(kEasyFixedRoute) / sizeof(kEasyFixedRoute[0]);

constexpr KnownTag kEasyKnownTags[] = {
  {"1F27AB41", {1, 1}},  {"390DAB41", {2, 1}},  {"9017AB41", {3, 1}},
  {"F63BAB41", {4, 1}},  {"76F0AA41", {5, 1}},  {"8145A941", {6, 1}},
  {"375CAB41", {7, 1}},  {"528AAB41", {8, 1}},  {"F07EAB41", {9, 1}},
  {"F642AB41", {1, 2}},  {"573DAB41", {2, 2}},  {"3385AB41", {3, 2}},
  {"07F6AA41", {4, 2}},  {"BCCFAA41", {5, 2}},  {"C47CAB41", {6, 2}},
  {"E74BA941", {7, 2}},  {"2A60AB41", {8, 2}},  {"7C88AB41", {9, 2}},
  {"10C7AA41", {1, 3}},  {"F052AB41", {2, 3}},  {"E840AB41", {3, 3}},
  {"AC5CAB41", {4, 3}},  {"9312AB41", {5, 3}},  {"8A45AB41", {6, 3}},
  {"6D19AB41", {7, 3}},  {"B493AB41", {8, 3}},  {"7451AB41", {9, 3}},
  {"773DAB41", {1, 4}},  {"47FAAA41", {2, 4}},  {"F459AB41", {3, 4}},
  {"6C5FAB41", {4, 4}},  {"3ACEAA41", {5, 4}},  {"B811AB41", {6, 4}},
  {"70D7AA41", {7, 4}},  {"3D84AB41", {8, 4}},  {"9259AB41", {9, 4}},
  {"D157AB41", {1, 5}},  {"FCD6AA41", {2, 5}},  {"41AB4141", {3, 5}},
  {"AE55AA41", {4, 5}},  {"4FC0AA41", {5, 5}},  {"F85EAB41", {6, 5}},
  {"48CBAA41", {7, 5}},  {"0077AB41", {8, 5}},  {"5663AB41", {9, 5}},
  {"A142AB41", {1, 6}},  {"0D46AB41", {2, 6}},  {"D3DDAA41", {3, 6}},
  {"B3DA2ADD", {4, 6}},  {"6666AA41", {5, 6}},  {"060DAB41", {6, 6}},
  {"CE9CAA41", {7, 6}},  {"F94FAB41", {8, 6}},  {"BD47AB41", {9, 6}},
  {"28E3AA41", {1, 7}},  {"4E4DAB41", {2, 7}},  {"8CE5AA41", {3, 7}},
  {"54C4AA41", {4, 7}},  {"E238A941", {5, 7}},  {"9C01AB41", {6, 7}},
  {"E7F7AA41", {7, 7}},  {"685EAB41", {8, 7}},  {"4A12AB41", {9, 7}},
  {"A335126A", {1, 8}},  {"F164AB41", {2, 8}},  {"A42DAB41", {3, 8}},
  {"F6B6A941", {4, 8}},  {"1D65AA41", {5, 8}},  {"03CCAA41", {6, 8}},
  {"DF54A941", {7, 8}},  {"7074AB41", {8, 8}},  {"2802AB41", {9, 8}},
  {"43DB2CDD", {1, 9}},  {"418BAB41", {2, 9}},  {"1B0AAB41", {3, 9}},
  {"7447AB41", {4, 9}},  {"6E54A641", {5, 9}},  {"70CBAA41", {6, 9}},
  {"855AAB41", {7, 9}},  {"671BAB41", {8, 9}},  {"C3DFAA41", {9, 9}},
};

constexpr uint8_t kEasyKnownTagCount =
    sizeof(kEasyKnownTags) / sizeof(kEasyKnownTags[0]);

inline bool sameCell(Cell a, Cell b) {
  return a.x == b.x && a.y == b.y;
}

inline bool validCell(Cell cell) {
  return cell.x >= 1 && cell.x <= 9 && cell.y >= 1 && cell.y <= 9;
}

inline void printCell(Cell cell) {
  if (!validCell(cell)) {
    Serial.print(F("none"));
    return;
  }
  Serial.print(F("x="));
  Serial.print(cell.x);
  Serial.print(F(" y="));
  Serial.print(cell.y);
}

inline bool adjacentCells(Cell a, Cell b) {
  const int dx = static_cast<int>(a.x) - static_cast<int>(b.x);
  const int dy = static_cast<int>(a.y) - static_cast<int>(b.y);
  return abs(dx) + abs(dy) == 1;
}

inline bool isBottomHalf(Cell cell) {
  return cell.x >= kEasyLineFollowMinX;
}

inline MovementMode movementModeForCell(Cell cell) {
  return isBottomHalf(cell) ? MovementMode::LineFollow : MovementMode::DriveStraight;
}

inline Direction directionFromTo(Cell from, Cell to) {
  if (!adjacentCells(from, to)) return Direction::Error;
  if (to.y == from.y + 1) return Direction::East;
  if (to.y + 1 == from.y) return Direction::West;
  if (to.x + 1 == from.x) return Direction::North;
  if (to.x == from.x + 1) return Direction::South;
  return Direction::Error;
}

inline const __FlashStringHelper *directionName(Direction direction) {
  switch (direction) {
    case Direction::North: return F("NORTH");
    case Direction::East: return F("EAST");
    case Direction::South: return F("SOUTH");
    case Direction::West: return F("WEST");
    case Direction::Error:
    default: return F("ERROR");
  }
}

inline bool cellForUid(const char *uid, Cell *out) {
  for (uint8_t i = 0; i < kEasyKnownTagCount; ++i) {
    if (strcmp(uid, kEasyKnownTags[i].uid) == 0) {
      *out = kEasyKnownTags[i].cell;
      return true;
    }
  }
  return false;
}

inline const char *uidForCell(Cell cell) {
  for (uint8_t i = 0; i < kEasyKnownTagCount; ++i) {
    if (sameCell(cell, kEasyKnownTags[i].cell)) {
      return kEasyKnownTags[i].uid;
    }
  }
  return "";
}

inline int routeIndexForCell(Cell cell, uint8_t startIndex = 0) {
  for (uint8_t i = startIndex; i < kEasyFixedRouteLength; ++i) {
    if (sameCell(kEasyFixedRoute[i], cell)) return i;
  }
  return -1;
}

inline bool validateEasyRoute(bool printProblems) {
  bool ok = true;

  for (uint8_t i = 0; i < kEasyFixedRouteLength; ++i) {
    const Cell cell = kEasyFixedRoute[i];
    if (!validCell(cell)) {
      ok = false;
      if (printProblems) {
        Serial.print(F("[ROUTE] invalid cell at index "));
        Serial.println(i);
      }
    }

    if (strlen(uidForCell(cell)) == 0) {
      ok = false;
      if (printProblems) {
        Serial.print(F("[ROUTE] missing UID for "));
        printCell(cell);
        Serial.println();
      }
    }

    if (i + 1 < kEasyFixedRouteLength && !adjacentCells(cell, kEasyFixedRoute[i + 1])) {
      ok = false;
      if (printProblems) {
        Serial.print(F("[ROUTE] non-adjacent step "));
        printCell(cell);
        Serial.print(F(" -> "));
        printCell(kEasyFixedRoute[i + 1]);
        Serial.println();
      }
    }
  }

  return ok;
}
