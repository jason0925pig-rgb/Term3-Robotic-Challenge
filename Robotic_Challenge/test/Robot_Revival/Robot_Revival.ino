#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Arduino_Modulino.h>

// =====================================================
// MOTORON
// =====================================================
MotoronI2C motoron(0x11);

constexpr int L_CH = 1;
constexpr int R_CH = 2;

// =====================================================
// PINS
// =====================================================
constexpr int TRIG_PIN = 8;
constexpr int ECHO_PIN = 11;

constexpr int REVIVAL_PIN = 31;
constexpr int KILL_PIN = 32;

// =====================================================
// SPEED（只保留两档）
// =====================================================
constexpr int BASE_SPEED = 520;
constexpr int APPROACH_SPEED = 250;

// =====================================================
// LED
// =====================================================
ModulinoPixels pixels;
bool pixelsOk = false;

// =====================================================
// STATE
// =====================================================
enum State {
  WAITING,
  APPROACH,
  STOPPED,
  KILLED
};

State state = WAITING;

// =====================================================
// MOTOR
// =====================================================
void setTank(int l, int r) {
  motoron.setSpeed(L_CH, l);
  motoron.setSpeed(R_CH, r);
}

void stopMotors() {
  setTank(0, 0);
}

// =====================================================
// LED（只用红/绿，不新增任何函数）
// =====================================================
void setRed() {
  if (!pixelsOk) return;
  for (int i = 0; i < 8; i++) pixels.set(i, RED, 80);
  pixels.show();
}

void setGreen() {
  if (!pixelsOk) return;
  for (int i = 0; i < 8; i++) pixels.set(i, GREEN, 80);
  pixels.show();
}

// =====================================================
// SONAR
// =====================================================
long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long t = pulseIn(ECHO_PIN, HIGH, 25000);
  if (t == 0) return 999;
  return t * 0.034 / 2;
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(REVIVAL_PIN, INPUT_PULLUP);
  pinMode(KILL_PIN, INPUT_PULLUP);

  Wire.begin();
  Modulino.begin(Wire);

  pixelsOk = pixels.begin();

  Wire1.begin();
  motoron.setBus(&Wire1);
  motoron.reinitialize();
  motoron.disableCrc();
  motoron.clearResetFlag();

  stopMotors();
  setRed();

  Serial.println("SYSTEM READY");
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  // =================================================
  // 🔴 KILL（最高优先级）
  // =================================================
  if (digitalRead(KILL_PIN) == LOW) {
    stopMotors();
    setRed();
    state = KILLED;
  }

  if (state == KILLED) {
    stopMotors();
    setRed();
    return;
  }

  // =================================================
  // 🟢 REVIVAL（立即停 + 绿灯）
  // =================================================
  if (digitalRead(REVIVAL_PIN) == LOW) {
    stopMotors();
    setGreen();
    state = STOPPED;
  }

  if (state == STOPPED) {
    stopMotors();
    setGreen();
    return;
  }

  // =================================================
  // 🟡 SONAR（20cm开始减速）
  // =================================================
  long dist = readDistanceCM();

  if (dist < 20) {
    state = APPROACH;
  } else {
    state = WAITING;
  }

  // =================================================
  // 🚗 SPEED CONTROL（仅两档）
  // =================================================
  int speed;

  if (state == APPROACH) {
    speed = APPROACH_SPEED;
  } else {
    speed = BASE_SPEED;
  }

  setTank(speed, speed);
}
