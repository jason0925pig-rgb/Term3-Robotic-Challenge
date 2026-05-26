#include <Arduino.h>
#include <WiFi.h>
#include "secrets.h"

constexpr unsigned long kSerialBaud = 115200;
constexpr unsigned long kConnectTimeoutMs = 30000;
constexpr unsigned long kRetryDelayMs = 5000;
constexpr unsigned long kStatusPrintIntervalMs = 1000;
constexpr bool kScanBeforeEachAttempt = true;

const char *wifiStatusName(int status) {
  switch (status) {
    case WL_NO_SHIELD:
      return "WL_NO_SHIELD";
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED";
    default:
      return "UNKNOWN";
  }
}

void printWifiStatusLine(const char *prefix) {
  const int status = WiFi.status();
  Serial.print(prefix);
  Serial.print(F(" status="));
  Serial.print(status);
  Serial.print('/');
  Serial.print(wifiStatusName(status));

  if (status == WL_CONNECTED) {
    Serial.print(F(" ip="));
    Serial.print(WiFi.localIP());
    Serial.print(F(" rssi="));
    Serial.print(WiFi.RSSI());
    Serial.print(F(" dBm"));
  }

  Serial.println();
}

void scanNetworks() {
  Serial.println();
  Serial.println(F("[SCAN] Scanning nearby networks..."));
  const int count = WiFi.scanNetworks();

  Serial.print(F("[SCAN] Found "));
  Serial.print(count);
  Serial.println(F(" network(s)."));

  for (int i = 0; i < count; ++i) {
    const String ssid = WiFi.SSID(i);
    Serial.print(F("[SCAN] SSID="));
    Serial.print(ssid);
    Serial.print(F(" RSSI="));
    Serial.print(WiFi.RSSI(i));
    Serial.print(F(" dBm"));

    if (ssid == WIFI_SSID) {
      Serial.print(F(" <-- configured SSID"));
    }

    Serial.println();
  }
}

bool connectWifiOnce() {
  if (kScanBeforeEachAttempt) {
    scanNetworks();
  }

  Serial.println();
  Serial.print(F("[WIFI] Trying SSID: "));
  Serial.println(WIFI_SSID);

  WiFi.disconnect();
  delay(500);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long start = millis();
  unsigned long lastPrint = 0;

  while (millis() - start < kConnectTimeoutMs) {
    const int status = WiFi.status();
    if (status == WL_CONNECTED) {
      printWifiStatusLine("[WIFI] Connected.");
      return true;
    }

    if (millis() - lastPrint >= kStatusPrintIntervalMs) {
      lastPrint = millis();
      printWifiStatusLine("[WIFI] Connecting...");
    }

    delay(100);
  }

  printWifiStatusLine("[WIFI] Connect timeout.");
  return false;
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  Serial.println(F("=== WiFi_Only_Test ==="));
  Serial.println(F("This sketch only tests Arduino GIGA WiFi connection."));
  Serial.println(F("It does not start motors and does not use MiniMessenger."));

  Serial.print(F("[CONFIG] GROUP_ID="));
  Serial.println(GROUP_ID);
  Serial.print(F("[CONFIG] SSID="));
  Serial.println(WIFI_SSID);
  Serial.print(F("[CONFIG] Broker="));
  Serial.print(BROKER_HOST);
  Serial.print(':');
  Serial.println(BROKER_PORT);

  printWifiStatusLine("[WIFI] Initial.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    const bool ok = connectWifiOnce();

    if (!ok) {
      Serial.print(F("[WIFI] Failed. Retrying in "));
      Serial.print(kRetryDelayMs / 1000);
      Serial.println(F(" second(s)."));
      delay(kRetryDelayMs);
      return;
    }
  }

  printWifiStatusLine("[WIFI] Still connected.");
  delay(3000);
}
