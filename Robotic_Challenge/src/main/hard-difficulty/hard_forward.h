#pragma once

#include <Arduino.h>

#include "hard_config.h"

void restartMission();
bool handleStartStopButtonEvent();
void updateWifi();
bool initializeMissionSensorsForRun();
void handleSerialCommands();
void setState(MissionState newState);
const __FlashStringHelper *stateName(MissionState state);
const __FlashStringHelper *sideName(WallSide side);
const __FlashStringHelper *turnName(TurnDir dir);
void resetLineController();
void resetWallController();
bool pollRfid(String *uidOut, bool force);
bool pollRfid(String *uidOut);
bool pollObstacleGridNode(String *uidOut);
bool servicePendingAirlockRequest();
