#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// I2C scanner for Arduino GIGA.
// Wire scans D20/D21. Wire1 scans the shield SDA1/SCL1 bus.
// Motoron should appear at 0x10 on Wire1 if the shield is plugged into Wire1.
// ---------------------------------------------------------------------------

void scanBus(TwoWire &bus, const __FlashStringHelper *name) {
  uint8_t found = 0;
  Serial.print(F("Scanning "));
  Serial.println(name);

  for (uint8_t address = 1; address < 127; address++) {
    bus.beginTransmission(address);
    const uint8_t error = bus.endTransmission();

    if (error == 0) {
      Serial.print(F("Found 0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      if (address == 0x10) Serial.print(F("  <- Motoron expected address"));
      Serial.println();
      found++;
    } else if (error == 4) {
      Serial.print(F("Unknown error at 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }
  }

  if (found == 0) {
    Serial.print(F("No I2C devices found on "));
    Serial.println(name);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  Wire1.begin();
  Serial.println(F("I2C_Scanner_Test ready."));
}

void loop() {
  scanBus(Wire, F("Wire / D20 SDA / D21 SCL"));
  scanBus(Wire1, F("Wire1 / shield SDA1-SCL1"));
  delay(3000);
}
