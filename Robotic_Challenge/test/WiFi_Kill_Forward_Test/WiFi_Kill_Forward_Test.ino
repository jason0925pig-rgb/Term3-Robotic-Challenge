#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MiniMessenger.h>
#include <Arduino_Modulino.h>

#include "secrets.h"

// ---------------------------------------------------------------------------
// WiFi_Kill_Forward_Test
// Board: Arduino GIGA R1 WiFi
//
// Demo behavior:
//   - When WiFi safety is ENABLED, robot drives forward continuously at speed 400.
//   - When WiFi safety is DISABLED, robot stops.
//   - When stopped, the Modulino Pixels on top blink RED.
//   - When running, the Modulino Pixels show solid GREEN.
//
// Hardware:
//   Motoron M3S550 address 0x11 on Wire1 / shield SDA1-SCL1
//   Motoron M1 = left motor, M2 = right motor
//   Mechanical kill button -> D32 to GND, INPUT_PULLUP
//   Modulino Pixels -> Wire / D20 SDA-D21 SCL, address seen by scanner: 0x36
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

constexpr int kForwardSpeed = 400;
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

// Keep this matching the MiniMessenger dashboard board ID used by the team.
constexpr const char *kBoardId = "YU7GT";
constexpr bool kDefaultMotionAllowedBeforeServer = false;
constexpr bool kScanWifiAtStartup = false;
constexpr uint32_t kRegisterIntervalMs = 5000;
constexpr uint32_t kWifiStatusPrintMs = 2000;
constexpr uint32_t kMotionStatusPrintMs = 500;

constexpr uint8_t kLedBrightness = 70;
constexpr uint32_t kStoppedBlinkMs = 350;
constexpr uint32_t kSerialWaitMs = 5000;

MotoronI2C motoron(kMotoronAddress);
MiniMessenger messenger;
ModulinoPixels pixels;

bool wifiSafetyEnabled = kDefaultMotionAllowedBeforeServer;
bool pixelsOk = false;
bool redBlinkOn = false;
bool lastMotionAllowed = false;
uint32_t lastRegisterMs = 0;
uint32_t lastWifiStatusPrintMs = 0;
uint32_t lastMotionStatusPrintMs = 0;
uint32_t lastBlinkMs = 0;

void stopMotors();

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
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

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

void setAllPixels(ModulinoColor color, uint8_t brightness = kLedBrightness) {
  if (!pixelsOk) {
    return;
  }
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, brightness);
  }
  pixels.show();
}

void setPixelsOff() {
  if (!pixelsOk) {
    return;
  }
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, BLACK, 0);
  }
  pixels.show();
}

void updateStoppedBlinkLed(bool stopped) {
  if (!pixelsOk) {
    return;
  }

  if (!stopped) {
    if (!lastMotionAllowed) {
      redBlinkOn = false;
      setAllPixels(GREEN, kLedBrightness);
    }
    return;
  }

  if (millis() - lastBlinkMs >= kStoppedBlinkMs) {
    lastBlinkMs = millis();
    redBlinkOn = !redBlinkOn;
    if (redBlinkOn) {
      setAllPixels(RED, kLedBrightness);
    } else {
      setPixelsOff();
    }
  }
}

const __FlashStringHelper *wifiStatusName(int status) {
  switch (status) {
    case WL_IDLE_STATUS:
      return F("WL_IDLE_STATUS");
    case WL_NO_SSID_AVAIL:
      return F("WL_NO_SSID_AVAIL");
    case WL_SCAN_COMPLETED:
      return F("WL_SCAN_COMPLETED");
    case WL_CONNECTED:
      return F("WL_CONNECTED");
    case WL_CONNECT_FAILED:
      return F("WL_CONNECT_FAILED");
    case WL_CONNECTION_LOST:
      return F("WL_CONNECTION_LOST");
    case WL_DISCONNECTED:
      return F("WL_DISCONNECTED");
    default:
      return F("UNKNOWN");
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

void allowMotion(const __FlashStringHelper *reason) {
  wifiSafetyEnabled = true;
  Serial.print(F("[WIFI SAFETY] ENABLED: "));
  Serial.println(reason);
}

void blockMotion(const __FlashStringHelper *reason) {
  wifiSafetyEnabled = false;
  stopMotors();
  Serial.print(F("[WIFI SAFETY] DISABLED: "));
  Serial.println(reason);
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

  // Known MiniMessenger safety messages from the challenge server.
  if (strstr(msg, "type=heartbeat enable=1") || strstr(msg, "enable=1") ||
      strstr(msg, "enabled=true")) {
    allowMotion(F("enable command"));
  }

  if (strstr(msg, "type=heartbeat enable=0") || strstr(msg, "enable=0") ||
      strstr(msg, "enabled=false") || strstr(msg, "type=disable") ||
      strstr(msg, "type=emergency")) {
    blockMotion(F("disable/emergency command"));
  }
}

void updateWifiKillSwitch() {
  messenger.loop();

  if (messenger.isConnected() && (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs)) {
    lastRegisterMs = millis();
    char reg[96];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kBoardId);
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

bool motionAllowed() {
  if (killPressed()) {
    blockMotion(F("mechanical kill button"));
    return false;
  }
  return wifiSafetyEnabled;
}

void printMotionStatus(bool allowed) {
  if (millis() - lastMotionStatusPrintMs < kMotionStatusPrintMs) {
    return;
  }
  lastMotionStatusPrintMs = millis();

  Serial.print(F("[MOTION] "));
  Serial.print(allowed ? F("RUNNING") : F("STOPPED"));
  Serial.print(F(" speed="));
  Serial.print(allowed ? kForwardSpeed : 0);
  Serial.print(F(" kill="));
  Serial.print(killPressed() ? F("PRESSED") : F("released"));
  Serial.print(F(" pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT_FOUND"));
}

void printMotoronState() {
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t statusErr = motoron.getLastError();
  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t vinErr = motoron.getLastError();

  Serial.print(F("[MOTORON] status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" statusErr="));
  Serial.print(statusErr);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" vinErr="));
  Serial.println(vinErr);
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

void initializePixels() {
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels begin(): "));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  if (pixelsOk) {
    setAllPixels(RED, kLedBrightness);
  }
}

void initializeWifiKillSwitch() {
  Serial.println(F("[WIFI] Starting MiniMessenger WiFi forward/stop demo."));
  Serial.print(F("[WIFI] Group ID = "));
  Serial.print(GROUP_ID);
  Serial.print(F(", Board ID = "));
  Serial.println(kBoardId);
  Serial.print(F("[WIFI] Default motion before server command = "));
  Serial.println(kDefaultMotionAllowedBeforeServer ? F("ALLOWED") : F("BLOCKED"));
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
  const uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < kSerialWaitMs) {
    delay(10);
  }
  delay(500);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

  Wire.begin();
  Wire1.begin();
  initializePixels();
  initializeMotoron();
  initializeWifiKillSwitch();

  Serial.println(F("WiFi_Kill_Forward_Test ready."));
  Serial.println(F("When stopped: Modulino Pixels blink RED. When enabled: robot drives forward at speed 400."));
  printMotoronState();
}

void loop() {
  updateWifiKillSwitch();

  const bool allowed = motionAllowed();
  if (allowed) {
    setTank(kForwardSpeed, kForwardSpeed);
  } else {
    stopMotors();
  }

  updateStoppedBlinkLed(!allowed);
  printMotionStatus(allowed);
  lastMotionAllowed = allowed;
  delay(10);
}
