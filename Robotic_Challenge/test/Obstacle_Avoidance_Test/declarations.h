#pragma once

#include <Arduino.h>

#include "constants.h"
#include "types.h"

// Basic helpers and safety input.
long absLong(long value);
float absFloat(float value);
int clampMotorSpeed(int speed);
bool validPin(int pin);
bool killPressed();
bool handleKillPauseInBlockingMotion();

// Motoron tank-drive control.
void setTank(int leftSpeed, int rightSpeed);
void stopMotors();
void initializeMotoron();

// Encoder counting and distance driving.
void leftEncoderIsr();
void rightEncoderIsr();
long getLeftCount();
long getRightCount();
void resetEncoders();
long distanceMmToCounts(float distanceMm);
long turnDegreesToEncoderCounts(float degrees);
bool driveDistanceMm(float distanceMm, int speed);

// QTR line-sensor reading and line-following control.
const __FlashStringHelper *followModeName(FollowMode mode);
void initializeQtrCalibration();
void readQtrRcArray();
void normalizeQtrValues();
uint8_t activeLineSensorCount(uint16_t threshold);
bool centerHasLine();
int computeLinePosition(bool *detectedOut);
FollowMode chooseFollowMode(const LineReading &line);
LineReading readLine();
MotorCommand computeLineMotorCommand(const LineReading &line);
void applyLineFollowStep();

// Ultrasonic sensing and wall-following control.
bool isValidSonarDistance(float mm);
float readSonarMm(int trigPin, int echoPin);
SonarReading readSonars();
int maxWallCorrectionFromRatioLimit(int baseSpeed);
float selectedWallDistanceMm(WallSide side, const SonarReading &s, bool *validOut);
MotorCommand computeWallFollowCommand(WallSide side, float distanceMm, bool *validOut);
void applyWallFollowStep(WallSide side);

// Obstacle avoidance
bool obstacleAhead(float thresholdMm = kObstacleAheadThresholdMm);

// IMU yaw integration and turning.
bool initializeImu();
bool updateImu();
void resetYaw();
ImuReading getImuReading();
void setTurnCommand(int signedTurnSpeed);
void printTurnStatus(const char *label, float targetDeg, float errorDeg, int command, long encoderTarget);
void encoderOnlyTurnFallback(float degrees, int speed);
bool turnDegreesImu(float targetDeg);

// Obstacle-avoidance primitives and demo harness.
MotorCommand computeHeadingHoldCommand(float targetYawDeg, int baseSpeed, float headingKp);
void simpleObstacleDemoStep();

// Serial monitor commands and diagnostics.
void printHelp();
void printSensorSnapshot();
void processSerialCommand(String line);
void handleSerialCommands();
