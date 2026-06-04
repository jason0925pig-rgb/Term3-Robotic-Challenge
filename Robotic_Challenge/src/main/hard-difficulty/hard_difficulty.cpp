#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino_Modulino.h>

#include "../constants.h"
#include "../types.h"
#include "../globals.h"
#include "../declarations.h"
#include "../basic_utils.h"
#include "../motor_utils.h"
#include "../line_following_utils.h"
#include "../sonar_wall_utils.h"

#include "hard_config.h"
#include "hard_forward.h"
#include "hard_helpers.h"
#include "hard_motion.h"
#include "hard_serial.h"
#include "hard_sonar.h"
#include "hard_rfid_server.h"
#include "hard_base_route.h"
#include "hard_grid_route.h"
#include "hard_obstacle.h"
#include "hard_controller.h"

// ---------------------------------------------------------------------------
// Hard Difficulty
// Board: Arduino GIGA R1 WiFi
//
// Full hard-mode mission:
//   base route -> airlock A -> tunnel wall follow -> grid RFID planting route
//   -> airlock B -> return tunnel -> base line reacquire.
// Shared hardware primitives live in src/main/*.h; hard-only mission behavior
// lives in the headers beside this file.
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  Serial.println(F("=== Hard Difficulty Base/Tunnel/Grid Mission ==="));

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
    lastKillReading = digitalRead(kKillPin);
    stableKillReading = lastKillReading;
    lastKillChangeMs = millis();
  }

  initializePixels();
  initializeWifi();

  Wire1.begin();
  initializeMotoron();
  Serial.println(F("[INIT] Motoron ready."));

  Serial.print(F("[INIT] encoder counts/mm="));
  Serial.println(kEncoderCountsPerMm, 4);
  printHelp();
  printSettings();

  if (kStartOnBoot) {
    restartMission();
  } else {
    setState(MissionState::Idle);
  }
}

void loop() {
  updateMission();
}
