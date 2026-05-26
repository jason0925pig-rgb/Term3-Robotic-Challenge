#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

// ---------------------------------------------------------------------------
// Motoron address test
// Purpose: test which Wire1 I2C address actually drives the Motoron shield.
// Lift the robot wheels before uploading.
// ---------------------------------------------------------------------------

constexpr int kTestSpeed = 550;
constexpr uint32_t kRunTimeMs = 1800;
constexpr uint32_t kStopTimeMs = 1000;
constexpr uint16_t kMotoronReferenceMv = 3300;
constexpr auto kMotoronVinType = MotoronVinSenseType::Motoron550;

// Scanner currently showed 0x11 and 0x60 on Wire1.
// 0x11 is the only address that fully ACKed Motoron commands in your latest test.
constexpr uint8_t kAddresses[] = {0x11};

void stopMotors(MotoronI2C &mc) {
  mc.setSpeed(1, 0);
  mc.setSpeed(2, 0);
  mc.setSpeed(3, 0);
}

void printLastError(const char *label, MotoronI2C &mc) {
  Serial.print(label);
  Serial.print(F(" err="));
  Serial.println(mc.getLastError());
}

void printMotoronState(MotoronI2C &mc) {
  const uint16_t status = mc.getStatusFlags();
  const uint8_t statusErr = mc.getLastError();
  const uint32_t vinMv = mc.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t vinErr = mc.getLastError();

  Serial.print(F("status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" statusErr="));
  Serial.print(statusErr);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" vinErr="));
  Serial.print(vinErr);
  Serial.print(F(" noPower="));
  Serial.print(mc.getNoPowerFlag());
  Serial.print(F(" outputEnabled="));
  Serial.print(mc.getMotorOutputEnabledFlag());
  Serial.print(F(" driving="));
  Serial.print(mc.getMotorDrivingFlag());
  Serial.print(F(" errorActive="));
  Serial.println(mc.getErrorActiveFlag());
}

void printMotorSpeedState(MotoronI2C &mc, uint8_t channel) {
  const int16_t target = mc.getTargetSpeed(channel);
  const uint8_t targetErr = mc.getLastError();
  const int16_t current = mc.getCurrentSpeed(channel);
  const uint8_t currentErr = mc.getLastError();

  Serial.print(F("M"));
  Serial.print(channel);
  Serial.print(F(" target="));
  Serial.print(target);
  Serial.print(F(" targetErr="));
  Serial.print(targetErr);
  Serial.print(F(" current="));
  Serial.print(current);
  Serial.print(F(" currentErr="));
  Serial.println(currentErr);
}

void testChannel(MotoronI2C &mc, uint8_t channel) {
  Serial.print(F("Testing motor channel M"));
  Serial.println(channel);

  stopMotors(mc);
  delay(200);
  mc.setSpeed(channel, kTestSpeed);
  printLastError("set forward", mc);
  delay(300);
  printMotorSpeedState(mc, channel);
  printMotoronState(mc);
  delay(kRunTimeMs);

  mc.setSpeed(channel, 0);
  printLastError("set stop", mc);
  delay(kStopTimeMs);
}

void testAddress(uint8_t address) {
  MotoronI2C mc(address);
  mc.setBus(&Wire1);

  Serial.print(F("Testing Motoron address 0x"));
  if (address < 16) Serial.print('0');
  Serial.println(address, HEX);

  mc.reinitialize();
  delay(10);
  printLastError("reinitialize", mc);
  mc.disableCrc();
  printLastError("disableCrc", mc);
  mc.clearResetFlag();
  printLastError("clearResetFlag", mc);

  mc.setMaxAcceleration(1, 0);
  mc.setMaxDeceleration(1, 0);
  mc.setMaxAcceleration(2, 0);
  mc.setMaxDeceleration(2, 0);
  mc.setMaxAcceleration(3, 0);
  mc.setMaxDeceleration(3, 0);

  stopMotors(mc);
  printLastError("initial stop", mc);
  printMotoronState(mc);

  testChannel(mc, 1);
  testChannel(mc, 2);
  testChannel(mc, 3);

  Serial.println(F("Final stop"));
  stopMotors(mc);
  printLastError("final stop", mc);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire1.begin();

  Serial.println(F("Motoron_Address_Test ready. Lift wheels before testing."));
}

void loop() {
  for (uint8_t i = 0; i < sizeof(kAddresses); i++) {
    testAddress(kAddresses[i]);
  }
  Serial.println(F("Address test cycle complete."));
  delay(4000);
}
