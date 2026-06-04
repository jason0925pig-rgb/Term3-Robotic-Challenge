#pragma once

#include <Arduino.h>

constexpr uint32_t kSerialBaud = 115200;

// Motoron and motors.
constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;

constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;
constexpr int kMaxMotorCommand = 800;

// Encoders.
constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;

constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kEncoderCountsPerMotorRev = 7.0f;
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;
constexpr float kTurnCalibration = 1.5f;
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;
constexpr uint32_t kDriveTimeoutMs = 12000;

// QTR-HD-09RC line sensor.
constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr uint16_t kMinUsefulCalibrationSpan = 20;

constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};

constexpr int kLineErrorSign = 1;
constexpr int kLineBaseSpeed = 340;
constexpr int kLineMaxCorrection = 560;
constexpr int kLineHardTurnSpeed = 450;
constexpr int kLineSearchTurnSpeed = 210;
constexpr float kLineKp = 0.8f;
constexpr float kLineKi = 0.0f;
constexpr float kLineKd = 0.08f;
constexpr uint16_t kLineThreshold = 230;
constexpr uint16_t kStrongLineThreshold = 650;
constexpr int kHardTurnError = 2500;
constexpr int kCenterRecoverError = 900;
constexpr float kLineIntegralClamp = 120.0f;
constexpr uint32_t kLineLoopDelayMs = 8;
constexpr uint32_t kLinePrintIntervalMs = 150;

// Ultrasonic sensors.
constexpr int kLeftTrigPin = 9;
constexpr int kLeftEchoPin = 12;
constexpr int kRightTrigPin = 10;
constexpr int kRightEchoPin = 13;
constexpr int kFrontTrigPin = 8;
constexpr int kFrontEchoPin = 11;

constexpr uint32_t kEchoTimeoutUs = 12000;
constexpr float kMinValidSonarMm = 20.0f;
constexpr float kMaxValidSonarMm = 900.0f;
constexpr float kObstacleAheadThresholdMm = 160.0f;

constexpr float kTargetWallDistanceMm = 62.5f;
constexpr int kWallBaseSpeed = 520;
constexpr int kWallMaxCorrection = 190;
constexpr float kMaxFastSlowMotorRatio = 1.40f;
constexpr float kWallKp = 1.0f;
constexpr float kWallKi = 0.0f;
constexpr float kWallKd = 0.0f;
constexpr float kWallIntegralClamp = 80.0f;
constexpr uint32_t kWallLoopDelayMs = 20;
constexpr uint32_t kWallPrintIntervalMs = 180;

// IMU yaw / turning.
constexpr uint8_t kImuAddress = 0x68;
constexpr uint16_t kGyroBiasSamples = 500;
constexpr uint16_t kGyroBiasSampleDelayMs = 4;
constexpr int kTurnCommandSign = 1;
constexpr int kImuYawSign = 1;
constexpr int kTurnMaxSpeed = 560;
constexpr int kTurnMinSpeed = 115;
constexpr float kTurnKp = 500.0f;
constexpr float kTurnKd = 0.0f;
constexpr float kTurnToleranceDeg = 2.0f;
constexpr float kGyroStopRateDps = 10.0f;
constexpr bool kUseImuTurnTimeout = false;
constexpr uint32_t kTurnTimeoutMs = 50000;
constexpr uint32_t kTurnPrintIntervalMs = 120;

// Safety / demo behaviour.
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;
constexpr bool kRunSimpleLineFollowDemo = false;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;
