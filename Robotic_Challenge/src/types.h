#pragma once

#include <Arduino.h>

enum class FollowMode {
  Follow,
  HardLeft,
  HardRight,
  SearchLeft,
  SearchRight,
  Stopped
};

enum class WallSide {
  Left,
  Right
};

enum class ObstacleState {
  FollowLine,
  StopForObstacle,
  Placeholder
};

struct MotorCommand {
  int left = 0;
  int right = 0;
};

struct LineReading {
  uint16_t raw[9] = {};
  uint16_t norm[9] = {};
  bool detected = false;
  int position = 4000;
  int error = 0;
  uint8_t activeCount = 0;
  FollowMode mode = FollowMode::Stopped;
};

struct SonarReading {
  float frontMm = -1.0f;
  float leftMm = -1.0f;
  float rightMm = -1.0f;
  bool frontValid = false;
  bool leftValid = false;
  bool rightValid = false;
};

struct ImuReading {
  bool ok = false;
  float yawDeg = 0.0f;
  float gyroZDps = 0.0f;
};
