#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>

#include "secrets.h"

// ---------------------------------------------------------------------------
// Motor + encoder motion test
// Board: Arduino GIGA R1 WiFi
//
// Hardware assumptions:
//   Motoron M3S550 address 0x11 on Wire1 / shield SDA1-SCL1
//   Motoron M1 = left motor, M2 = right motor
//   Left encoder C1/C2  -> D34/D35
//   Right encoder C1/C2 -> D36/D37
//
// Test sequence:
//   1. Drive forward at 400, stop 3s
//   2. Drive forward at 600, stop 3s
//   3. Drive forward at 800, stop 3s
//   4. Encoder left turn 90 degrees, stop 3s
//   5. Encoder right turn 90 degrees, stop 3s
//   6. Direct in-place U-turn, 180 degrees
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kMotoronAddress = 0x11;
constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr uint16_t kMotoronReferenceMv = 3300;
constexpr auto kMotoronVinType = MotoronVinSenseType::Motoron550;

constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;

// Pull D32 to GND to abort the test if you keep the kill button connected.
constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

// WiFi kill switch through MiniMessenger. Put secrets.h in the same sketch
// folder before testing with the lab dashboard.
constexpr bool kUseWifiKillSwitch = true;
constexpr const char *kBoardId = "Team2Robot";       // Must match the dashboard board ID.
constexpr bool kDefaultMotionAllowedBeforeServer = true;
constexpr bool kScanWifiAtStartup = true;
constexpr uint32_t kRegisterIntervalMs = 10000;
constexpr uint32_t kWifiStatusPrintMs = 2000;
constexpr uint32_t kSafetyWaitPrintMs = 1000;

// Flip these if a motor runs backward relative to the intended command.
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

// Flip this if "left turn" and "right turn" are swapped.
constexpr int kTurnDirectionSign = 1;

// Speed test.
constexpr int kSpeedTestSpeeds[] = {400, 600, 800};
constexpr uint32_t kSpeedTestRunMs = 1500;
constexpr uint32_t kStopBetweenStepsMs = 3000;

// Encoder geometry. Measure/tune these on your actual robot.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;          // Distance between left/right wheel contact centers.
constexpr float kMotorNoLoadRpm = 200.0f;        // Waveshare DCGM-N20-12V-EN-200RPM no-load speed.
constexpr float kEncoderCountsPerMotorRev = 7.0f; // C1/A rising-edge pulses before gearbox; 1050 per wheel rev after 1:150.
constexpr float kGearRatio = 150.0f;
constexpr float kDistanceCalibration = 1.0f;     // Increase if robot drives too short; decrease if too far.
constexpr float kTurnCalibration = 1.0f;         // Increase if turns are too small; decrease if too large.

// Encoder-controlled movement.
constexpr int kDriveDistanceSpeed = 450;
constexpr int kTurnSpeed = 360;
constexpr float kTurnAngleDeg = 90.0f;
constexpr uint32_t kDriveTimeoutMs = 12000;
constexpr uint32_t kTurnTimeoutMs = 8000;

// Straight driving correction using encoder counts.
constexpr float kStraightCorrectionKp = 0.35f;
constexpr int kMaxStraightCorrection = 90;

// Turning uses a hard equal-speed rule: left and right motors always receive
// the same magnitude in opposite directions, so the robot rotates about centre.
constexpr int kMinTurnMotorSpeed = 120;

// Direct U-turn test.
constexpr float kUTurnAngleDeg = 180.0f;
constexpr int kUTurnDirection = 1;               // 1 = left-left U, -1 = right-right U.
constexpr int kUTurnSpeed = 340;

// true = run the full sequence once, then stop forever.
// false = repeat the full sequence after kRepeatPauseMs.
constexpr bool kRunSequenceOnce = true;
constexpr uint32_t kRepeatPauseMs = 8000;

constexpr uint32_t kPrintIntervalMs = 200;
constexpr float kPi = 3.1415926f;

// ---------------------------------------------------------------------------
// Derived constants
// ---------------------------------------------------------------------------
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);
MiniMessenger messenger;

volatile long leftCount = 0;
volatile long rightCount = 0;

bool wifiKillConfigured = false;
bool wifiSafetyEnabled = kDefaultMotionAllowedBeforeServer;
uint32_t lastRegisterMs = 0;
uint32_t lastWifiStatusPrintMs = 0;
uint32_t lastSafetyWaitPrintMs = 0;

void stopMotors();
void updateWifiKillSwitch();

long absLong(long value) {
  return value < 0 ? -value : value;
}

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
}

void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
}

long getLeftCount() {
  noInterrupts();
  const long count = leftCount;
  interrupts();
  return count;
}

long getRightCount() {
  noInterrupts();
  const long count = rightCount;
  interrupts();
  return count;
}

void resetEncoders() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print(F("[WIFI RX] from "));
  Serial.print(metadata.fromBoardId);
  Serial.print(F(": "));
  Serial.println(msg);

  if (strstr(msg, "type=heartbeat enable=1")) {
    wifiSafetyEnabled = true;
    Serial.println(F("[WIFI SAFETY] heartbeat enable=1 -> motion allowed."));
  } else if (strstr(msg, "type=heartbeat enable=0")) {
    wifiSafetyEnabled = false;
    stopMotors();
    Serial.println(F("[WIFI SAFETY] heartbeat enable=0 -> motors stopped."));
  }

  if (strstr(msg, "type=emergency enabled=true")) {
    wifiSafetyEnabled = false;
    stopMotors();
    Serial.println(F("[WIFI SAFETY] emergency enabled=true -> motors stopped."));
  }

  if (strstr(msg, "type=disable enabled=false")) {
    wifiSafetyEnabled = false;
    stopMotors();
    Serial.println(F("[WIFI SAFETY] disable enabled=false -> motors stopped."));
  }
}

const __FlashStringHelper *wifiStatusName(int status) {
  switch (status) {
    case WL_IDLE_STATUS: return F("WL_IDLE_STATUS");
    case WL_NO_SSID_AVAIL: return F("WL_NO_SSID_AVAIL");
    case WL_SCAN_COMPLETED: return F("WL_SCAN_COMPLETED");
    case WL_CONNECTED: return F("WL_CONNECTED");
    case WL_CONNECT_FAILED: return F("WL_CONNECT_FAILED");
    case WL_CONNECTION_LOST: return F("WL_CONNECTION_LOST");
    case WL_DISCONNECTED: return F("WL_DISCONNECTED");
    default: return F("UNKNOWN");
  }
}

void printWifiScan() {
  Serial.println(F("[WIFI] Scanning nearby networks..."));
  const int networkCount = WiFi.scanNetworks();
  Serial.print(F("[WIFI] scanNetworks() found "));
  Serial.print(networkCount);
  Serial.println(F(" network(s)."));

  for (int i = 0; i < networkCount; i++) {
    Serial.print(F("[WIFI] SSID="));
    Serial.print(WiFi.SSID(i));
    Serial.print(F(" RSSI="));
    Serial.print(WiFi.RSSI(i));
    Serial.print(F(" dBm"));
    if (String(WiFi.SSID(i)) == WIFI_SSID) {
      Serial.print(F("  <-- configured SSID"));
    }
    Serial.println();
  }
}

void updateWifiKillSwitch() {
  if (!wifiKillConfigured) return;

  messenger.loop();

  if (messenger.isConnected() && (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs)) {
    lastRegisterMs = millis();
    char reg[96];
    snprintf(reg,
             sizeof(reg),
             "type=register team_id=%s board_id=%s",
             GROUP_ID,
             kBoardId);
    messenger.sendToBoard("server", reg);
    Serial.print(F("[WIFI] Registered with server: "));
    Serial.println(reg);
  }

  if (millis() - lastWifiStatusPrintMs >= kWifiStatusPrintMs) {
    lastWifiStatusPrintMs = millis();
    Serial.print(F("[WIFI] connected="));
    Serial.print(messenger.isConnected() ? F("YES") : F("NO"));
    Serial.print(F(" wifiStatus="));
    Serial.print(WiFi.status());
    Serial.print(F("/"));
    Serial.print(wifiStatusName(WiFi.status()));
    Serial.print(F(" safetyEnabled="));
    Serial.println(wifiSafetyEnabled ? F("YES") : F("NO"));
  }
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

bool motionAllowed() {
  updateWifiKillSwitch();

  if (killPressed()) {
    stopMotors();
    return false;
  }

  if (kUseWifiKillSwitch && wifiKillConfigured && !wifiSafetyEnabled) {
    stopMotors();
    return false;
  }

  return true;
}

bool waitForMotionAllowed(const char *context) {
  while (true) {
    updateWifiKillSwitch();

    if (killPressed()) {
      stopMotors();
      Serial.print(F("[SAFETY] Mechanical kill pressed while waiting for "));
      Serial.println(context);
      return false;
    }

    if (!kUseWifiKillSwitch || !wifiKillConfigured || wifiSafetyEnabled) {
      return true;
    }

    stopMotors();
    if (millis() - lastSafetyWaitPrintMs >= kSafetyWaitPrintMs) {
      lastSafetyWaitPrintMs = millis();
      Serial.print(F("[WIFI SAFETY] Waiting for enable=1 before "));
      Serial.println(context);
    }
    delay(20);
  }
}

void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

void stopMotors() {
  setTank(0, 0);
}

void printMotoronState() {
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t statusErr = motoron.getLastError();
  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t vinErr = motoron.getLastError();

  Serial.print(F("Motoron status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" statusErr="));
  Serial.print(statusErr);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" vinErr="));
  Serial.println(vinErr);
}

void printCounts(const char *label, long targetCounts = -1) {
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs < kPrintIntervalMs) return;
  lastPrintMs = millis();

  const long left = getLeftCount();
  const long right = getRightCount();
  Serial.print(label);
  Serial.print(F(" L="));
  Serial.print(left);
  Serial.print(F(" R="));
  Serial.print(right);
  Serial.print(F(" absL="));
  Serial.print(absLong(left));
  Serial.print(F(" absR="));
  Serial.print(absLong(right));
  if (targetCounts >= 0) {
    Serial.print(F(" target="));
    Serial.print(targetCounts);
  }
  Serial.println();
}

long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(abs(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

long turnDegreesToCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * abs(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

void pauseStopped(uint32_t durationMs) {
  stopMotors();
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (!motionAllowed()) {
      stopMotors();
      if (killPressed()) {
        Serial.println(F("KILL pressed during pause."));
        return;
      }
      if (!waitForMotionAllowed("pause")) {
        return;
      }
      start = millis();
    }
    delay(20);
  }
}

void runTimedForward(int speed, uint32_t durationMs) {
  if (!waitForMotionAllowed("timed forward")) return;

  Serial.print(F("Timed forward speed="));
  Serial.print(speed);
  Serial.print(F(" durationMs="));
  Serial.println(durationMs);

  resetEncoders();
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (!motionAllowed()) {
      Serial.println(F("Safety stop. Timed forward aborted."));
      break;
    }
    setTank(speed, speed);
    printCounts("TimedForward");
    delay(10);
  }
  stopMotors();
  printCounts("TimedForward final");
}

void driveDistanceMm(float distanceMm, int speed) {
  if (!waitForMotionAllowed("drive distance")) return;

  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("Drive distance mm="));
  Serial.print(distanceMm);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    if (!motionAllowed()) {
      Serial.println(F("Safety stop. Drive distance aborted."));
      break;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) {
      break;
    }

    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("Drive distance timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);

    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);
    printCounts("DriveDistance", targetCounts);
    delay(10);
  }

  stopMotors();
  printCounts("DriveDistance final", targetCounts);
}

void turnDegrees(float degrees, int speed) {
  if (!waitForMotionAllowed("turn")) return;

  const long targetCounts = turnDegreesToCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;
  const int turnDirection = direction * kTurnDirectionSign;

  Serial.print(F("Turn degrees="));
  Serial.print(degrees);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCountsPerSide="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    if (!motionAllowed()) {
      Serial.println(F("Safety stop. Turn aborted."));
      break;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) {
      break;
    }

    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("Turn timeout."));
      break;
    }

    const int turnMagnitude = constrain(abs(speed), kMinTurnMotorSpeed, 800);
    const int leftCommand = -turnDirection * turnMagnitude;
    const int rightCommand = turnDirection * turnMagnitude;
    setTank(leftCommand, rightCommand);
    printCounts("Turn", targetCounts);
    delay(10);
  }

  stopMotors();
  printCounts("Turn final", targetCounts);
}

void runSpeedTests() {
  Serial.println(F("=== Speed tests ==="));
  for (uint8_t i = 0; i < sizeof(kSpeedTestSpeeds) / sizeof(kSpeedTestSpeeds[0]); i++) {
    runTimedForward(kSpeedTestSpeeds[i], kSpeedTestRunMs);
    pauseStopped(kStopBetweenStepsMs);
  }
}

void runTurnTests() {
  Serial.println(F("=== Encoder turn tests ==="));
  turnDegrees(kTurnAngleDeg, kTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
  turnDegrees(-kTurnAngleDeg, kTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
}

void runUShape() {
  Serial.println(F("=== Direct in-place U-turn ==="));
  turnDegrees(kUTurnDirection * kUTurnAngleDeg, kUTurnSpeed);
  pauseStopped(kStopBetweenStepsMs);
}

void initializeMotoron() {
  Wire1.begin();
  motoron.setBus(&Wire1);
  motoron.reinitialize();
  delay(10);
  motoron.disableCrc();
  motoron.clearResetFlag();
  motoron.setCommandTimeoutMilliseconds(1000);
  motoron.setMaxAcceleration(kMotoronLeftChannel, 0);
  motoron.setMaxDeceleration(kMotoronLeftChannel, 0);
  motoron.setMaxAcceleration(kMotoronRightChannel, 0);
  motoron.setMaxDeceleration(kMotoronRightChannel, 0);
  stopMotors();
}

void initializeWifiKillSwitch() {
  if (!kUseWifiKillSwitch) {
    wifiKillConfigured = false;
    wifiSafetyEnabled = true;
    Serial.println(F("[WIFI] WiFi kill switch disabled by parameter."));
    return;
  }

  wifiKillConfigured = true;
  wifiSafetyEnabled = kDefaultMotionAllowedBeforeServer;
  lastRegisterMs = 0;

  Serial.println(F("[WIFI] Starting MiniMessenger WiFi kill switch."));
  Serial.print(F("[WIFI] Group ID = "));
  Serial.print(GROUP_ID);
  Serial.print(F(", Board ID = "));
  Serial.println(kBoardId);
  Serial.print(F("[WIFI] Default motion before server command = "));
  Serial.println(kDefaultMotionAllowedBeforeServer ? F("ALLOWED") : F("BLOCKED"));
  Serial.println(F("[WIFI] Robot will stop when heartbeat enable=0, disable, or emergency is received."));
  Serial.print(F("[WIFI] SSID = "));
  Serial.println(WIFI_SSID);
  Serial.print(F("[WIFI] Broker = "));
  Serial.print(BROKER_HOST);
  Serial.print(F(":"));
  Serial.println(BROKER_PORT);

  if (kScanWifiAtStartup) {
    printWifiScan();
  }

  messenger.onMessage(onWifiMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kBoardId);
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);

  initializeMotoron();
  initializeWifiKillSwitch();

  Serial.println(F("Motor_Encoder_U_Test ready. Lift wheels first if you are checking direction."));
  Serial.println(F("Motoron address 0x11 on Wire1. M1=left, M2=right."));
  Serial.println(F("Left encoder D34/D35, right encoder D36/D37."));
  Serial.print(F("Motor spec: no-load rpm="));
  Serial.print(kMotorNoLoadRpm, 1);
  Serial.print(F(", gear ratio=1:"));
  Serial.print(kGearRatio, 0);
  Serial.print(F(", encoder PPR="));
  Serial.println(kEncoderCountsPerMotorRev, 1);
  Serial.print(F("Counts per mm estimate = "));
  Serial.println(kEncoderCountsPerMm, 4);
  printMotoronState();
  pauseStopped(2000);
}

void loop() {
  runSpeedTests();
  runTurnTests();
  runUShape();

  Serial.println(F("Full motor test sequence complete."));
  stopMotors();

  if (kRunSequenceOnce) {
    Serial.println(F("Sequence is configured to run once. Motors stopped forever."));
    while (true) {
      updateWifiKillSwitch();
      if (killPressed() || (wifiKillConfigured && !wifiSafetyEnabled)) {
        stopMotors();
      }
      delay(200);
    }
  }

  pauseStopped(kRepeatPauseMs);
}
