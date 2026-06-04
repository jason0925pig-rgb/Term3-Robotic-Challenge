#pragma once

#include "hard_config.h"
#include "hard_forward.h"

inline void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  start | restart | stop | resume | show"));
  Serial.println(F("  route a | route b | route right | route left"));
  Serial.println(F("  wall left | wall right"));
  Serial.println(F("  alignmm 0       set RFID alignment offset after tag detection"));
  Serial.println(F("  sonar           print one front/left/right sonar snapshot"));
  Serial.println(F("  D32 button: press in IDLE/STOPPED to start; press while moving to stop."));
}

/**
 * Print current tunable settings and mission progress.
 */
inline void printSettings() {
  Serial.print(F("[SETTINGS] state="));
  Serial.print(stateName(missionState));
  Serial.print(F(" route="));
  Serial.print(routeName(routeChoice));
  Serial.print(F(" routeTurn="));
  Serial.print(routeTurnIndex);
  Serial.print(F("/"));
  Serial.print(kRouteTurnCount);
  Serial.print(F(" airlockRfidFromTurn="));
  Serial.print(kAirlockRfidCheckFromTurnIndex);
  Serial.print(F(" wall="));
  Serial.print(sideName(wallSide));
  Serial.print(F(" alignMm="));
  Serial.print(rfidAlignOffsetMm, 1);
  Serial.print(F(" obstacleWaitMs="));
  Serial.print(kExitDoorObstacleWaitMs);
  Serial.print(F(" lastUid="));
  Serial.print(lastUid.length() > 0 ? lastUid : String("none"));
  Serial.print(F(" wifiConnected="));
#if USE_WIFI_AIRLOCK_REQUEST
  Serial.print(messenger.isConnected() ? F("YES") : F("NO"));
#else
  Serial.print(F("DISABLED"));
#endif
  Serial.print(F(" wifiSafety="));
  Serial.print(wifiSafetyEnabled ? F("ENABLED") : F("DISABLED"));
  Serial.print(F(" pendingAirlock="));
  Serial.print(pendingAirlockUid.length() > 0 ? pendingAirlockUid : String("none"));
  Serial.print(F(" pixels="));
  Serial.print(pixelsOk ? F("OK") : F("NOT_FOUND"));
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
}

/**
 * Reset all mission progress and start again from the base line.
 */
inline void restartMission() {
  if (!wifiSafetyEnabled) {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    Serial.println(F("[WIFI SAFETY] start blocked because server safety is disabled. Wait for enable=1."));
    return;
  }

  serialStopped = false;
  stoppedByWifiKill = false;
  setPixelsRunningPurple();
  airlockRequestSent = false;
  airlockAccepted = false;
  pendingAirlockUid = "";
  lastAirlockRequestAttemptMs = 0;
  routeTurnIndex = 0;
  eventStableCount = 0;
  lastFirstTEvidenceMs = 0;
  tunnelEntryNoLineCount = 0;
  wallExitLineStableCount = 0;
  returnTunnelNoLineCount = 0;
  returnWallExitLineStableCount = 0;
  easyRouteIndex = 0;
  easySegmentRfidCount = 0;
  seedsPlanted = 0;
  easyDoorRequestSent = false;
  waitingForCellStatus = false;
  lastCellStatus = {};
  postTurnHardIgnoreActive = false;
  postTurnHardReleaseCount = 0;
  lastUid = "";
  lastGridUid = "";
  lastGridUidMs = 0;
  lastGridTurnMs = 0;
  lastEventMs = millis();
  resetObstacleAvoidanceProgress();
  resetLineController();
  resetWallController();

  if (!initializeMissionSensorsForRun()) {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    return;
  }

  setState(MissionState::FollowBaseRoute);
}

/**
 * Handle the D32 button as a mission start/stop control.
 *
 * Pressing the button in Idle/Stopped/Done initializes sensors and starts a new
 * mission. Pressing it while moving stops the robot immediately.
 *
 * @return true if the press requested a stop.
 */
inline bool handleStartStopButtonEvent() {
  if (!killButtonPressedEvent()) return false;

  if (missionState == MissionState::Idle ||
      missionState == MissionState::Stopped ||
      missionState == MissionState::Done) {
    Serial.println(F("[BUTTON] press -> START"));
    restartMission();
    return false;
  }

  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.println(F("[BUTTON] press -> STOP"));
  return true;
}

/**
 * Read and print one sonar snapshot for field debugging.
 */
inline void printSonarSnapshot() {
  if (!missionSensorsInitialized) {
    Serial.println(F("[SONAR] sensors are initialized after START; press D32 or send start first."));
    return;
  }

  const float front = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  delay(8);
  const float left = readSonarMm(kLeftTrigPin, kLeftEchoPin);
  delay(8);
  const float right = readSonarMm(kRightTrigPin, kRightEchoPin);

  Serial.print(F("[SONAR] frontMm="));
  Serial.print(front, 1);
  Serial.print(F(" leftMm="));
  Serial.print(left, 1);
  Serial.print(F(" rightMm="));
  Serial.println(right, 1);
}

/**
 * Parse and execute one complete Serial Monitor command.
 *
 * @param line Raw command line without the trailing newline.
 */
inline void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printHelp();
    return;
  }
  if (lower == "show") {
    printSettings();
    return;
  }
  if (lower == "sonar") {
    printSonarSnapshot();
    return;
  }
  if (lower == "start" || lower == "restart") {
    restartMission();
    return;
  }
  if (lower == "stop") {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    return;
  }
  if (lower == "resume") {
    restartMission();
    return;
  }
  if (lower == "wall left") {
    wallSide = WallSide::Left;
    printSettings();
    return;
  }
  if (lower == "wall right") {
    wallSide = WallSide::Right;
    printSettings();
    return;
  }
  if (lower == "route a" || lower == "route right" || lower == "route bottom") {
    routeChoice = RouteChoice::BaseA_Bottom;
    routeTurnIndex = 0;
    eventStableCount = 0;
    printSettings();
    return;
  }
  if (lower == "route b" || lower == "route left" || lower == "route top") {
    routeChoice = RouteChoice::BaseB_Top;
    routeTurnIndex = 0;
    eventStableCount = 0;
    printSettings();
    return;
  }

  const int space = lower.indexOf(' ');
  if (space <= 0) {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  const String key = lower.substring(0, space);
  const float value = line.substring(space + 1).toFloat();

  if (key == "alignmm") {
    rfidAlignOffsetMm = constrain(value, 0.0f, 300.0f);
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  printSettings();
}

/**
 * Accumulate Serial input and dispatch newline-terminated commands.
 */
inline void handleSerialCommands() {
  static String input;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      processSerialCommand(input);
      input = "";
      continue;
    }
    if (input.length() < 90) input += c;
  }
}
