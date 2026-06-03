#pragma once

#include <Arduino.h>
#include <MiniMessenger.h>
#include <string.h>

#include "../secrets.h"
#include "easy_route.h"

constexpr const char *kEasyBoardId = "YU7GT";
constexpr uint32_t kEasyRegisterIntervalMs = 8000;
constexpr uint32_t kEasyServerReplyTimeoutMs = 900;
constexpr uint32_t kEasyWifiStatusPrintMs = 2500;

static MiniMessenger easyMessenger;
static bool easyMessengerStarted = false;
static bool easyWifiMotionAllowed = true;
static uint32_t easyLastRegisterMs = 0;
static uint32_t easyLastWifiStatusPrintMs = 0;
static bool easyWaitingForCellStatus = false;
static CellStatus easyLastCellStatus = {};
static char easyPendingUid[12] = "";

inline bool easyMotionAllowedByWifi() {
  return easyWifiMotionAllowed;
}

inline void setEasyWifiStopped(const __FlashStringHelper *reason) {
  easyWifiMotionAllowed = false;
  stopMotors();
  // Serial.print(F("[WIFI SAFETY] stopped: "));
  // Serial.println(reason);
}

inline void setEasyWifiRunning(const __FlashStringHelper *reason) {
  easyWifiMotionAllowed = true;
  // Serial.print(F("[WIFI SAFETY] running: "));
  // Serial.println(reason);
}

inline void parseEasyCellStatusReply(const char *msg) {
  if (!strstr(msg, "type=isFertileReply")) return;

  easyLastCellStatus.valid = true;
  easyLastCellStatus.isFertile = strstr(msg, "fertile=true") != nullptr;
  easyLastCellStatus.alreadyPlanted = strstr(msg, "planted=true") != nullptr;
  easyLastCellStatus.cell = {};

  const char *xPtr = strstr(msg, "x=");
  const char *yPtr = strstr(msg, "y=");
  if (xPtr && yPtr) {
    easyLastCellStatus.cell.x = static_cast<uint8_t>(atoi(xPtr + 2));
    easyLastCellStatus.cell.y = static_cast<uint8_t>(atoi(yPtr + 2));
  }

  easyWaitingForCellStatus = false;
}

inline void onEasyWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  if (length == 6) {
    if (payload[4] == 1) setEasyWifiStopped(F("team emergency byte"));
    return;
  }

  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  // Serial.print(F("[WIFI RX] from "));
  // Serial.print(metadata.fromBoardId);
  // Serial.print(F(": "));
  // Serial.println(msg);

  parseEasyCellStatusReply(msg);

  if (strstr(msg, "type=heartbeat") && strstr(msg, "enable=0")) {
    setEasyWifiStopped(F("heartbeat enable=0"));
    return;
  }

  if (strstr(msg, "enable=0") || strstr(msg, "enabled=false") ||
      strstr(msg, "type=disable") || strstr(msg, "type=emergency")) {
    setEasyWifiStopped(F("disable/emergency message"));
    return;
  }

  if (strstr(msg, "type=heartbeat") && strstr(msg, "enable=1")) {
    setEasyWifiRunning(F("heartbeat enable=1"));
    return;
  }

  if (strstr(msg, "enable=1") || strstr(msg, "enabled=true")) {
    setEasyWifiRunning(F("enable message"));
  }
}

inline void initializeEasyServer() {
  easyMessenger.onMessage(onEasyWifiMessage);
  easyMessengerStarted = true;
  easyLastRegisterMs = 0;
  easyMessenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kEasyBoardId);

  // Serial.print(F("[WIFI] begin group="));
  // Serial.print(GROUP_ID);
  // Serial.print(F(" board="));
  // Serial.print(kEasyBoardId);
  // Serial.print(F(" connected="));
  // Serial.println(easyMessenger.isConnected() ? F("YES") : F("NO"));
}

inline void easyServerLoop() {
  if (!easyMessengerStarted) return;

  easyMessenger.loop();

  if (easyMessenger.isConnected() &&
      (easyLastRegisterMs == 0 || millis() - easyLastRegisterMs >= kEasyRegisterIntervalMs)) {
    easyLastRegisterMs = millis();
    char reg[128];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kEasyBoardId);
    const bool ok = easyMessenger.sendToBoard("server", reg);
    // Serial.print(F("[WIFI] register "));
    // Serial.print(ok ? F("sent: ") : F("failed: "));
    // Serial.println(reg);
  }

  if (millis() - easyLastWifiStatusPrintMs >= kEasyWifiStatusPrintMs) {
    easyLastWifiStatusPrintMs = millis();
    // Serial.print(F("[WIFI] connected="));
    // Serial.print(easyMessenger.isConnected() ? F("YES") : F("NO"));
    // Serial.print(F(" motionAllowed="));
    // Serial.println(easyWifiMotionAllowed ? F("YES") : F("NO"));
  }
}

inline bool queryServerForCellStatus(const char *uid, CellStatus *statusOut) {
  statusOut->valid = false;

  if (!easyMessengerStarted || !easyMessenger.isConnected()) {
    Serial.print(F("[SERVER] offline; skip isFertile for tag_id="));
    Serial.println(uid);
    return false;
  }

  char msg[140];
  snprintf(msg, sizeof(msg), "type=isFertile team_id=%s board_id=%s tag_id=%s",
           GROUP_ID, kEasyBoardId, uid);

  strncpy(easyPendingUid, uid, sizeof(easyPendingUid) - 1);
  easyPendingUid[sizeof(easyPendingUid) - 1] = '\0';
  easyLastCellStatus = {};
  easyWaitingForCellStatus = true;

  const bool sent = easyMessenger.sendToBoard("server", msg);
  Serial.print(F("[SERVER] sent "));
  Serial.print(msg);
  Serial.print(F(" ok="));
  Serial.println(sent ? F("YES") : F("NO"));
  if (!sent) {
    easyWaitingForCellStatus = false;
    return false;
  }

  const uint32_t start = millis();
  while (millis() - start < kEasyServerReplyTimeoutMs) {
    handleSerialCommands();
    easyServerLoop();
    updateImu();

    if (killPressed() || serialStopped || !easyMotionAllowedByWifi()) {
      stopMotors();
      easyWaitingForCellStatus = false;
      return false;
    }

    if (!easyWaitingForCellStatus && easyLastCellStatus.valid) {
      *statusOut = easyLastCellStatus;
      Serial.print(F("[SERVER] reply fertile="));
      Serial.print(statusOut->isFertile ? F("true") : F("false"));
      Serial.print(F(" planted="));
      Serial.print(statusOut->alreadyPlanted ? F("true") : F("false"));
      Serial.print(F(" cell="));
      printCell(statusOut->cell);
      Serial.println();
      return true;
    }

    delay(10);
  }

  easyWaitingForCellStatus = false;
  Serial.println(F("[SERVER] isFertile reply timeout."));
  return false;
}

inline void notifyServerSeedPlanted(const char *uid, Cell cell) {
  char msg[140];
  snprintf(msg, sizeof(msg), "type=seedPlanted team_id=%s board_id=%s tag_id=%s",
           GROUP_ID, kEasyBoardId, uid);

  Serial.print(F("[SERVER] seed planted at "));
  printCell(cell);
  Serial.print(F(": "));
  Serial.println(msg);

  if (!easyMessengerStarted || !easyMessenger.isConnected()) {
    Serial.println(F("[SERVER] offline; seedPlanted not sent."));
    return;
  }

  easyMessenger.sendToBoard("server", msg);
}
