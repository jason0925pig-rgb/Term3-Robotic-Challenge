#pragma once

#include <Arduino.h>

#include "globals.h"
#include "types.h"

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  stop | resume | show | line | sonars | turnleft | turnright"));
  Serial.println(F("Notes:"));
  Serial.println(F("  kRunSimpleLineFollowDemo=false by default, so loop only prints sensors."));
  Serial.println(F("  Set kFrontTrigPin/kFrontEchoPin before relying on front obstacle detection."));
}

void printSensorSnapshot() {
  const LineReading line = readLine();
  const SonarReading sonar = readSonars();
  const ImuReading imuReading = getImuReading();

  Serial.print(F("[SNAPSHOT] lineDetected="));
  Serial.print(line.detected ? F("YES") : F("NO"));
  Serial.print(F(" lineErr="));
  Serial.print(line.error);
  Serial.print(F(" sonarF="));
  Serial.print(sonar.frontValid ? sonar.frontMm : -1.0f, 1);
  Serial.print(F(" sonarL="));
  Serial.print(sonar.leftValid ? sonar.leftMm : -1.0f, 1);
  Serial.print(F(" sonarR="));
  Serial.print(sonar.rightValid ? sonar.rightMm : -1.0f, 1);
  Serial.print(F(" yaw="));
  Serial.print(imuReading.yawDeg, 2);
  Serial.print(F(" gyroZ="));
  Serial.println(imuReading.gyroZDps, 1);
}

void processSerialCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "help" || line == "?") {
    printHelp();
  } else if (line == "stop") {
    serialStopped = true;
    stopMotors();
    Serial.println(F("[SERIAL] stopped."));
  } else if (line == "resume") {
    serialStopped = false;
    lineIntegral = 0.0f;
    wallIntegral = 0.0f;
    lastLineError = 0;
    lastWallErrorMm = 0.0f;
    Serial.println(F("[SERIAL] resumed."));
  } else if (line == "show") {
    printSensorSnapshot();
  } else if (line == "line") {
    const LineReading r = readLine();
    Serial.print(F("[LINE TEST] detected="));
    Serial.print(r.detected ? F("YES") : F("NO"));
    Serial.print(F(" pos="));
    Serial.print(r.position);
    Serial.print(F(" err="));
    Serial.print(r.error);
    Serial.print(F(" mode="));
    Serial.println(followModeName(r.mode));
  } else if (line == "sonars") {
    const SonarReading s = readSonars();
    Serial.print(F("[SONAR TEST] F="));
    Serial.print(s.frontValid ? s.frontMm : -1.0f, 1);
    Serial.print(F(" L="));
    Serial.print(s.leftValid ? s.leftMm : -1.0f, 1);
    Serial.print(F(" R="));
    Serial.println(s.rightValid ? s.rightMm : -1.0f, 1);
  } else if (line == "turnleft") {
    turnDegreesImu(90.0f);
  } else if (line == "turnright") {
    turnDegreesImu(-90.0f);
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
  }
}

void handleSerialCommands() {
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
