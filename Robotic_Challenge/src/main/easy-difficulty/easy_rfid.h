#pragma once

#include <Arduino.h>
#include <MFRC522_I2C.h>
#include <Wire.h>

#include "easy_route.h"

constexpr uint8_t kEasyRfidAddress = 0x28;
constexpr uint8_t kEasyRfidResetPin = 39;
constexpr uint32_t kEasyRfidPollIntervalMs = 80;
constexpr uint32_t kEasySameRfidCooldownMs = 900;
constexpr uint32_t kEasyReadCurrentRfidTimeoutMs = 900;

static MFRC522_I2C easyRfid(kEasyRfidAddress, kEasyRfidResetPin, &Wire);
static bool easyRfidOk = false;
static uint32_t easyLastRfidPollMs = 0;
static uint32_t easyLastAcceptedRfidMs = 0;
static String easyLastAcceptedUid;

inline void initializeEasyRfid() {
  pinMode(kEasyRfidResetPin, OUTPUT);
  digitalWrite(kEasyRfidResetPin, HIGH);
  easyRfid.PCD_Init();
  const byte version = easyRfid.PCD_ReadRegister(easyRfid.VersionReg);
  easyRfidOk = !(version == 0x00 || version == 0xFF);

  Serial.print(F("[RFID] version=0x"));
  Serial.print(version, HEX);
  Serial.print(F(" status="));
  Serial.println(easyRfidOk ? F("OK") : F("NOT FOUND"));
}

inline String easyRfidUidToString() {
  String uid;
  for (byte i = 0; i < easyRfid.uid.size; ++i) {
    if (easyRfid.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(easyRfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

inline bool pollEasyRfidRaw(String *uidOut, bool forcePoll = false) {
  if (!easyRfidOk) return false;
  if (!forcePoll && millis() - easyLastRfidPollMs < kEasyRfidPollIntervalMs) return false;
  easyLastRfidPollMs = millis();

  if (easyRfid.PICC_IsNewCardPresent() && easyRfid.PICC_ReadCardSerial()) {
    *uidOut = easyRfidUidToString();
    easyRfid.PICC_HaltA();
    return true;
  }

  return false;
}

inline bool pollEasyRfidDebounced(String *uidOut) {
  String uid;
  if (!pollEasyRfidRaw(&uid)) return false;

  if (uid == easyLastAcceptedUid && millis() - easyLastAcceptedRfidMs < kEasySameRfidCooldownMs) {
    return false;
  }

  easyLastAcceptedUid = uid;
  easyLastAcceptedRfidMs = millis();
  *uidOut = uid;
  return true;
}

inline bool readCurrentEasyRfid(String *uidOut, uint32_t timeoutMs = kEasyReadCurrentRfidTimeoutMs) {
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    String uid;
    if (pollEasyRfidRaw(&uid, true)) {
      easyLastAcceptedUid = uid;
      easyLastAcceptedRfidMs = millis();
      *uidOut = uid;
      return true;
    }
    delay(20);
  }
  return false;
}
