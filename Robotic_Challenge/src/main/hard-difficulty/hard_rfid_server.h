#pragma once

#include <ctype.h>
#include <string.h>

#include "hard_config.h"
#include "hard_forward.h"

inline void initializeRfid() {
  pinMode(kRfidResetPin, OUTPUT);
  digitalWrite(kRfidResetPin, HIGH);
  rfid.PCD_Init();
  const byte version = rfid.PCD_ReadRegister(rfid.VersionReg);
  rfidOk = !(version == 0x00 || version == 0xFF);
  Serial.print(F("[RFID] version=0x"));
  Serial.print(version, HEX);
  Serial.print(F(" status="));
  Serial.println(rfidOk ? F("OK") : F("NOT FOUND"));
}

/**
 * Convert the last read RFID UID to uppercase hex text.
 *
 * @return UID string.
 */
inline String rfidUidToString() {
  String uid;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

/**
 * Poll RFID for a tag.
 *
 * @param uidOut Output UID string when a tag is found.
 * @param force true bypasses the normal rate limit for short burst scans.
 * @return true when a tag was read.
 */
inline bool pollRfid(String *uidOut, bool force) {
  if (!rfidOk) return false;
  if (!force && millis() - lastRfidPollMs < kRfidPollIntervalMs) return false;
  lastRfidPollMs = millis();

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    *uidOut = rfidUidToString();
    rfid.PICC_HaltA();
    return true;
  }
  return false;
}

/**
 * Poll RFID with the normal rate limit.
 *
 * @param uidOut Output UID string when a tag is found.
 * @return true when a tag was read.
 */
inline bool pollRfid(String *uidOut) {
  return pollRfid(uidOut, false);
}

/**
 * Poll RFID several times in a tight window.
 *
 * Use this while crossing a known tag location so a single unlucky poll does
 * not miss the tag as the robot moves over it.
 *
 * @param uidOut Output UID string when a tag is found.
 * @param attempts Number of forced RFID polls to try.
 * @param gapMs Delay between attempts.
 * @return true when a tag was read.
 */
inline bool pollRfidBurst(String *uidOut, uint8_t attempts, uint16_t gapMs) {
  for (uint8_t i = 0; i < attempts; i++) {
    if (pollRfid(uidOut, true)) return true;
    if (i + 1 >= attempts) break;

    const uint32_t waitStart = millis();
    while (millis() - waitStart < gapMs) {
      handleSerialCommands();
      updateWifi();
      if (serialStopped || handleStartStopButtonEvent()) {
        return false;
      }
      delay(1);
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Optional MiniMessenger / server airlock API
// ---------------------------------------------------------------------------

#if USE_WIFI_AIRLOCK_REQUEST
/**
 * Stop the robot because the server-side WiFi safety state is disabled.
 *
 * @param reason Human-readable reason printed to Serial.
 */
inline void stopForWifiSafety(const __FlashStringHelper *reason) {
  wifiSafetyEnabled = false;
  stoppedByWifiKill = true;
  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.print(F("[WIFI SAFETY] "));
  Serial.println(reason);
}

/**
 * Re-enable motion after the server sends enable=1.
 *
 * The mission is not resumed in the middle of a route segment; press D32 or
 * send "start" to restart from the beginning after server safety is enabled.
 */
inline void allowWifiSafety() {
  wifiSafetyEnabled = true;
  if (stoppedByWifiKill) {
    stoppedByWifiKill = false;
    serialStopped = false;
    Serial.println(F("[WIFI SAFETY] enable=1 received. Press D32 or send start/restart to run again."));
  } else {
    Serial.println(F("[WIFI SAFETY] enable=1 received."));
  }
}

/**
 * Parse a server fertility/status reply for the latest RFID query.
 *
 * @param msg Null-terminated MiniMessenger payload.
 */
inline void parseCellStatusReply(const char *msg) {
  if (!strstr(msg, "type=isFertileReply")) return;

  lastCellStatus.valid = true;
  lastCellStatus.fertile = strstr(msg, "fertile=true") != nullptr;
  lastCellStatus.planted = strstr(msg, "planted=true") != nullptr;
  waitingForCellStatus = false;
}

/**
 * Handle server messages from MiniMessenger.
 *
 * The official MiniMessenger protocol uses text key=value messages for normal
 * replies and also sends some fixed-length binary team/global messages. This
 * handler stops the robot on disable/emergency/heartbeat enable=0 and records
 * openAirlockReply acceptance for diagnostics.
 *
 * @param metadata MiniMessenger message metadata.
 * @param payload Raw message bytes.
 * @param length Number of bytes in payload.
 */
inline void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  if (length == 6) {
    const bool emergency = payload[4] == 1;
    if (emergency) {
      stopForWifiSafety(F("team emergency byte set; stopped."));
    }
    return;
  }

  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';

  Serial.print(F("[WIFI RX] "));
  Serial.println(msg);

  parseCellStatusReply(msg);

  if (strstr(msg, "enable=1") || strstr(msg, "type=enable")) {
    allowWifiSafety();
    return;
  }

  if (strstr(msg, "enable=0")) {
    stopForWifiSafety(F("enable=0; stopped."));
    return;
  }

  if (strstr(msg, "type=disable") || strstr(msg, "type=emergency")) {
    stopForWifiSafety(F("disable/emergency text message; stopped."));
    return;
  }

  if (strstr(msg, "type=openAirlockReply")) {
    airlockAccepted = strstr(msg, "accepted=true") != nullptr;
    Serial.print(F("[AIRLOCK] reply "));
    if (strstr(msg, "airlock=B")) Serial.print(F("B "));
    else if (strstr(msg, "airlock=A")) Serial.print(F("A "));
    Serial.print(F("accepted="));
    Serial.println(airlockAccepted ? F("YES") : F("NO"));
  }
}

/**
 * Start MiniMessenger WiFi/MQTT communication if secrets.h is available.
 */
inline void initializeWifi() {
  if (wifiInitialized) return;
  messenger.onMessage(onWifiMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kBoardId);
  lastRegisterMs = 0;
  wifiInitialized = true;
  Serial.println(F("[WIFI] MiniMessenger started."));
}

/**
 * Service MiniMessenger and send periodic register messages.
 *
 * This keeps the robot visible to the server while it is waiting at doors or
 * moving through the route.
 */
inline void updateWifi() {
  messenger.loop();
  if (!messenger.isConnected()) return;

  if (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs) {
    lastRegisterMs = millis();
    char reg[128];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kBoardId);
    messenger.sendToBoard("server", reg);
    Serial.print(F("[WIFI REGISTER] "));
    Serial.println(reg);
  }
}

/**
 * Wait for MiniMessenger to connect before calibration and motion.
 *
 * @return true when connected and server safety is enabled; false if cancelled.
 */
inline bool waitForWifiBeforeCalibration() {
  if (!kRequireWifiBeforeCalibration) {
    return true;
  }

  Serial.println(F("[WIFI] waiting for MiniMessenger connection before IMU calibration and motion."));
  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;

  while (!messenger.isConnected()) {
    handleSerialCommands();
    messenger.loop();

    if (serialStopped || !wifiSafetyEnabled) {
      stopMotors();
      Serial.println(F("[WIFI] startup wait cancelled by stop/safety state."));
      return false;
    }

    if (killButtonPressedEvent()) {
      serialStopped = true;
      stopMotors();
      setState(MissionState::Stopped);
      Serial.println(F("[WIFI] startup wait cancelled by D32 button."));
      return false;
    }

    if (kWifiConnectTimeoutMs > 0 && millis() - start >= kWifiConnectTimeoutMs) {
      serialStopped = true;
      stopMotors();
      setState(MissionState::Stopped);
      Serial.println(F("[WIFI] connection timeout before calibration; startup aborted."));
      return false;
    }

    if (millis() - lastPrintMs >= kWifiConnectPrintIntervalMs) {
      lastPrintMs = millis();
      Serial.print(F("[WIFI] waiting... board_id="));
      Serial.print(kBoardId);
      Serial.print(F(" group="));
      Serial.println(GROUP_ID);
    }
    delay(20);
  }

  updateWifi();
  Serial.println(F("[WIFI] connected and registered before calibration."));
  return wifiSafetyEnabled;
}

/**
 * Request the server to open an airlock.
 *
 * MiniMessenger README documents the payload as key=value text, including
 * type=openAirlock, airlock=A/B, tag_id, and board_id.
 *
 * @param uid RFID UID read at the relevant door tag.
 * @param airlock Airlock letter, usually 'A' when leaving base and 'B' when returning.
 * @return true if the message was queued for sending.
 */
inline bool sendAirlockOpenRequest(const String &uid, char airlock) {
  airlock = toupper(airlock);
  if (airlock != 'A' && airlock != 'B') airlock = 'A';

  if (!messenger.isConnected()) {
    Serial.print(F("[AIRLOCK] WiFi not connected; cannot request airlock "));
    Serial.print(airlock);
    Serial.println(F(" yet."));
    return false;
  }

  char msg[180];
  snprintf(
      msg,
      sizeof(msg),
      "type=openAirlock team_id=%s airlock=%c tag_id=%s board_id=%s",
      GROUP_ID,
      airlock,
      uid.c_str(),
      kBoardId);

  const bool ok = messenger.sendToBoard("server", msg);
  Serial.print(F("[AIRLOCK] sent "));
  Serial.print(msg);
  Serial.print(F(" ok="));
  Serial.println(ok ? F("YES") : F("NO"));
  return ok;
}

/**
 * Backwards-compatible wrapper for the original base-exit airlock A request.
 *
 * @param uid RFID UID read at the base-exit tag.
 * @return true if the message was queued for sending.
 */
inline bool sendAirlockOpenRequest(const String &uid) {
  return sendAirlockOpenRequest(uid, 'A');
}

/**
 * @return true when MiniMessenger is connected to the server.
 */
inline bool serverOnline() {
  return messenger.isConnected();
}

/**
 * Ask the server whether an RFID cell is fertile and unplanted.
 *
 * @param uid RFID tag UID.
 * @param statusOut Filled with the reply when one is received.
 * @return true when an isFertileReply was received before timeout.
 */
inline bool queryServerForCellStatus(const String &uid, CellStatus *statusOut) {
  statusOut->valid = false;
  statusOut->fertile = false;
  statusOut->planted = false;

  if (!serverOnline()) {
    Serial.print(F("[SERVER] offline; cannot query tag_id="));
    Serial.println(uid);
    return false;
  }

  char msg[140];
  snprintf(
      msg,
      sizeof(msg),
      "type=isFertile team_id=%s board_id=%s tag_id=%s",
      GROUP_ID,
      kBoardId,
      uid.c_str());

  lastCellStatus = {};
  waitingForCellStatus = true;
  const bool sent = messenger.sendToBoard("server", msg);
  Serial.print(F("[SERVER] sent "));
  Serial.print(msg);
  Serial.print(F(" ok="));
  Serial.println(sent ? F("YES") : F("NO"));

  if (!sent) {
    waitingForCellStatus = false;
    return false;
  }

  const uint32_t start = millis();
  while (millis() - start < kServerReplyTimeoutMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      waitingForCellStatus = false;
      return false;
    }

    if (!waitingForCellStatus && lastCellStatus.valid) {
      *statusOut = lastCellStatus;
      Serial.print(F("[SERVER] reply fertile="));
      Serial.print(statusOut->fertile ? F("true") : F("false"));
      Serial.print(F(" planted="));
      Serial.println(statusOut->planted ? F("true") : F("false"));
      return true;
    }

    updateImu();
    delay(10);
  }

  waitingForCellStatus = false;
  Serial.println(F("[SERVER] isFertile reply timeout."));
  return false;
}

/**
 * Tell the server that this robot has planted a seed at the current RFID tag.
 *
 * @param uid RFID tag UID.
 */
inline void notifySeedPlanted(const String &uid) {
  char msg[140];
  snprintf(
      msg,
      sizeof(msg),
      "type=seedPlanted team_id=%s board_id=%s tag_id=%s",
      GROUP_ID,
      kBoardId,
      uid.c_str());

  Serial.print(F("[SERVER] seedPlanted "));
  Serial.println(msg);
  if (serverOnline()) {
    messenger.sendToBoard("server", msg);
  }
}
#else
/**
 * No-op WiFi initializer when secrets.h is not present.
 */
inline void initializeWifi() {}

/**
 * No-op WiFi service function when secrets.h is not present.
 */
inline void updateWifi() {}

/**
 * Abort startup when WiFi is required but secrets.h is not available.
 *
 * @return true only when WiFi is not required.
 */
inline bool waitForWifiBeforeCalibration() {
  if (!kRequireWifiBeforeCalibration) {
    return true;
  }

  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.println(F("[WIFI] required before calibration, but secrets.h is missing. Startup aborted."));
  return false;
}

/**
 * Log the airlock request that would be sent if MiniMessenger were enabled.
 *
 * @param uid RFID UID read at the base-exit tag.
 * @param airlock Airlock letter, usually 'A' or 'B'.
 * @return false because no network request was sent.
 */
inline bool sendAirlockOpenRequest(const String &uid, char airlock) {
  airlock = toupper(airlock);
  if (airlock != 'A' && airlock != 'B') airlock = 'A';
  Serial.print(F("[AIRLOCK] WiFi disabled. Would send: type=openAirlock team_id=<GROUP_ID> airlock="));
  Serial.print(airlock);
  Serial.print(F(" tag_id="));
  Serial.print(uid);
  Serial.println(F(" board_id=<kBoardId>. Copy secrets.example.h to secrets.h to enable."));
  return false;
}

/**
 * Log the base-exit airlock A request that would be sent if MiniMessenger were enabled.
 *
 * @param uid RFID UID read at the base-exit tag.
 * @return false because no network request was sent.
 */
inline bool sendAirlockOpenRequest(const String &uid) {
  return sendAirlockOpenRequest(uid, 'A');
}

/**
 * @return false when MiniMessenger is disabled.
 */
inline bool serverOnline() {
  return false;
}

/**
 * Offline fallback for cell status queries.
 *
 * @param uid RFID tag UID.
 * @param statusOut Filled with invalid/false values.
 * @return false because no server reply can be received.
 */
inline bool queryServerForCellStatus(const String &uid, CellStatus *statusOut) {
  statusOut->valid = false;
  statusOut->fertile = false;
  statusOut->planted = false;
  Serial.print(F("[SERVER] WiFi disabled. Would query isFertile tag_id="));
  Serial.println(uid);
  return false;
}

/**
 * Offline fallback for seedPlanted.
 *
 * @param uid RFID tag UID.
 */
inline void notifySeedPlanted(const String &uid) {
  Serial.print(F("[SERVER] WiFi disabled. Would send seedPlanted tag_id="));
  Serial.println(uid);
}
#endif

/**
 * Retry a pending base-exit airlock request without needing to see the RFID tag again.
 *
 * @return true only when the request has just been sent successfully.
 */
inline bool servicePendingAirlockRequest() {
  if (airlockRequestSent || pendingAirlockUid.length() == 0) {
    return false;
  }

  const uint32_t now = millis();
  if (lastAirlockRequestAttemptMs != 0 &&
      now - lastAirlockRequestAttemptMs < kAirlockRequestRetryMs) {
    return false;
  }

  lastAirlockRequestAttemptMs = now;
  Serial.print(F("[AIRLOCK] requesting A for pending tag UID="));
  Serial.println(pendingAirlockUid);

  if (!sendAirlockOpenRequest(pendingAirlockUid)) {
    Serial.println(F("[AIRLOCK] request not sent yet; will retry."));
    return false;
  }

  airlockRequestSent = true;
  pendingAirlockUid = "";
  Serial.println(F("[AIRLOCK] request sent; waiting for openAirlockReply."));
  return true;
}

/**
 * Read the base-exit RFID tag once and request airlock A.
 *
 * @return true if a tag was newly handled on this call.
 */
inline bool checkBaseExitRfidAndRequestAirlock() {
  if (airlockRequestSent) return false;
  if (pendingAirlockUid.length() > 0) {
    servicePendingAirlockRequest();
    return false;
  }

  String uid;
  if (!pollRfidBurst(&uid, kBaseExitRfidBurstPolls, kBaseExitRfidBurstGapMs)) return false;

  lastUid = uid;
  stopMotors();
  Serial.print(F("[RFID] base exit tag UID="));
  Serial.println(lastUid);

  pendingAirlockUid = lastUid;
  lastAirlockRequestAttemptMs = 0;
  servicePendingAirlockRequest();

  const uint32_t start = millis();
  while (millis() - start < kStopOverAirlockTagMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return true;
    }
    delay(20);
  }

  return true;
}
