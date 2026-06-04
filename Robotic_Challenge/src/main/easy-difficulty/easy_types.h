#pragma once

#include <Arduino.h>

struct Cell {
  uint8_t x = 0;  // server x coordinate, 1..9
  uint8_t y = 0;  // server y coordinate, 1..9
};

struct KnownTag {
  const char *uid = "";
  Cell cell = {};
};

struct CellStatus {
  bool valid = false;
  bool isFertile = false;
  bool alreadyPlanted = false;
  Cell cell = {};
};

enum class Direction : uint8_t {
  North = 0,
  East = 1,
  South = 2,
  West = 3,
  Error = 255
};

enum class MovementMode : uint8_t {
  LineFollow,
  DriveStraight
};

enum class MissionState : uint8_t {
  Init,
  ExitBaseToField,
  AlignToFirstSegment,
  SearchForRFID,
  ReadCurrentCell,
  QueryServer,
  DecidePlanting,
  PlantSeed,
  AdvanceRouteIndex,
  MoveToNextCell,
  ExecuteScript,
  Finished,
  Error
};
