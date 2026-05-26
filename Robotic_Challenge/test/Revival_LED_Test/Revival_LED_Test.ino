#include <Arduino.h>
#include <Wire.h>
#include <Arduino_Modulino.h>

// ---------------------------------------------------------------------------
// Revival button + Modulino Pixels test
// Board: Arduino GIGA R1 WiFi
//
// Wiring:
//   Revival button: one side -> D31, other side -> GND
//   Modulino Pixels: SDA -> D20, SCL -> D21, 3V3 -> 3.3V, GND -> common GND
//
// Behavior:
//   Start with RED LEDs.
//   Press revival button once: LEDs become GREEN and stay GREEN.
//   Serial prints I2C scan, Pixels init result, button state, and LED state.
// ---------------------------------------------------------------------------

constexpr uint8_t kRevivalButtonPin = 31;
constexpr uint8_t kLedBrightness = 80;         // Brightness percent, 0-100. Raise for easier debugging.
constexpr uint32_t kDebounceMs = 35;
constexpr uint32_t kStatusPrintMs = 500;
constexpr uint8_t kExpectedPixelsWireAddress = 0x36;  // Normal I2C scanner address. Library internally calls this 0x6C.
constexpr bool kRunI2cScanAtStartup = false;   // Set true only when debugging I2C. Full scan can upset a bad bus.
constexpr uint32_t kSerialWaitMs = 5000;

ModulinoPixels pixels;

bool pixelsOk = false;
bool revived = false;
bool lastReading = HIGH;
bool stableReading = HIGH;
uint32_t lastChangeMs = 0;
uint32_t lastStatusPrintMs = 0;
const char *ledStateText = "RED";

void setAllPixels(ModulinoColor color) {
  if (!pixelsOk) {
    Serial.println(F("[LED] Cannot set pixels: Modulino Pixels was NOT FOUND."));
    return;
  }
  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kLedBrightness);
  }
  pixels.show();
}

void setLedRed() {
  setAllPixels(RED);
  ledStateText = "RED";
  Serial.println(F("[LED] Command sent: RED"));
}

void setLedGreen() {
  setAllPixels(GREEN);
  ledStateText = "GREEN";
  Serial.println(F("[LED] Command sent: GREEN"));
}

void scanI2cBus() {
  Serial.println(F("[I2C] Scanning Wire / D20 SDA / D21 SCL..."));
  uint8_t foundCount = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();
    if (error == 0) {
      foundCount++;
      Serial.print(F("[I2C] Found 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }
  }

  if (foundCount == 0) {
    Serial.println(F("[I2C] No devices found on Wire."));
  }
  Serial.print(F("[I2C] Expected Modulino Pixels scanner address is around 0x"));
  Serial.println(kExpectedPixelsWireAddress, HEX);
}

bool revivalPressedEvent() {
  const bool reading = digitalRead(kRevivalButtonPin);

  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = millis();
  }

  if (millis() - lastChangeMs >= kDebounceMs && reading != stableReading) {
    stableReading = reading;
    return stableReading == LOW;
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  const uint32_t serialStartMs = millis();
  while (!Serial && millis() - serialStartMs < kSerialWaitMs) {
    delay(10);
  }
  delay(500);

  pinMode(kRevivalButtonPin, INPUT_PULLUP);
  Modulino.begin(Wire);

  Serial.println(F("=== Revival_LED_Test diagnostic version ==="));
  Serial.println(F("Brightness value means percent, 0-100. Current brightness = 80."));
  Serial.println(F("Button wiring: D31 -> switch -> GND. INPUT_PULLUP means not pressed=HIGH, pressed=LOW."));
  Serial.println(F("I2C full scan is disabled by default to keep USB serial stable."));
  Serial.println(F("Set kRunI2cScanAtStartup = true only if you need to debug I2C addresses."));
  Serial.println(F("[INIT] Modulino.begin(Wire) completed. Now starting Modulino Pixels..."));
  if (kRunI2cScanAtStartup) {
    scanI2cBus();
  }

  pixelsOk = pixels.begin();
  Serial.print(F("[INIT] Modulino Pixels begin(): "));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));

  setLedRed();
  Serial.println(F("[READY] LED should be RED now. Press D31 button to turn GREEN."));
}

void loop() {
  if (!revived && revivalPressedEvent()) {
    revived = true;
    Serial.println(F("[BUTTON] Revival button pressed event detected."));
    setLedGreen();
  }

  if (millis() - lastStatusPrintMs >= kStatusPrintMs) {
    lastStatusPrintMs = millis();
    Serial.print(F("[STATUS] pixelsOk="));
    Serial.print(pixelsOk ? F("YES") : F("NO"));
    Serial.print(F(" buttonRaw="));
    Serial.print(digitalRead(kRevivalButtonPin) == LOW ? F("LOW/PRESSED") : F("HIGH/NOT_PRESSED"));
    Serial.print(F(" stable="));
    Serial.print(stableReading == LOW ? F("LOW/PRESSED") : F("HIGH/NOT_PRESSED"));
    Serial.print(F(" revived="));
    Serial.print(revived ? F("YES") : F("NO"));
    Serial.print(F(" ledState="));
    Serial.println(ledStateText);
  }
}
