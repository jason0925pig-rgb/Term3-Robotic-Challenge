#pragma once

#include <Arduino.h>
#include <Motoron.h>
#include <Adafruit_ICM20948.h>

#include "constants.h"
#include "types.h"

MotoronI2C motoron(kMotoronAddress);
Adafruit_ICM20948 imu;

volatile long leftCount = 0;
volatile long rightCount = 0;

uint16_t qtrRaw[9] = {};
uint16_t qtrMin[9] = {};
uint16_t qtrMax[9] = {};
uint16_t qtrNorm[9] = {};

bool imuOk = false;
float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;

bool serialStopped = false;
uint32_t lastLinePrintMs = 0;
uint32_t lastWallPrintMs = 0;
uint32_t lastTurnPrintMs = 0;

int lastLineError = 0;
int lastSeenLineError = 0;
float lineIntegral = 0.0f;

float wallIntegral = 0.0f;
float lastWallErrorMm = 0.0f;
uint32_t lastWallUpdateMs = 0;

float lastValidFrontMm = -1.0f;
float lastValidLeftMm = -1.0f;
float lastValidRightMm = -1.0f;

ObstacleState obstacleState = ObstacleState::FollowLine;
