#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

// ---------------------------------------------------------------------------
// LineFollow_Test
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   Test QTR-HD-09RC line following with the Motoron motor shield.
//
// Hardware:
//   QTR CTRL ODD/EVEN -> D2/D3
//   QTR sensors       -> D22-D30, left to right when viewed from robot front
//   Kill button       -> D32 to GND, INPUT_PULLUP
//   Motoron           -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1        -> left motor
//   Motoron M2        -> right motor
//
// Serial commands, Newline enabled:
//   p 0.14       set P gain
//   i 0          set I gain
//   d 0.08       set D gain
//   base 220     set base forward speed
//   maxcorr 320  set maximum PD steering correction
//   th 230       set normalized black-line threshold
//   hard 260     set in-place hard-turn speed
//   stop         stop motors and pause line following
//   resume       resume line following
//   show         print current settings
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------
constexpr uint32_t kSerialBaud = 115200;

constexpr uint8_t kQtrCtrlOddPin = 2;
constexpr uint8_t kQtrCtrlEvenPin = 3;
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};
constexpr uint8_t kMechanicalKillPin = 32;

constexpr uint8_t kMotoronLeftChannel = 1;
constexpr uint8_t kMotoronRightChannel = 2;
constexpr uint8_t kMotoronI2cAddress = 0x11;
constexpr uint16_t kMotoronReferenceMv = 3300;
constexpr auto kMotoronVinType = MotoronVinSenseType::Motoron550;

constexpr uint16_t kQtrTimeoutUs = 3000;
constexpr bool kUseSavedCalibration = true;
constexpr uint32_t kCalibrationMs = 5000;
constexpr uint16_t kMinUsefulCalibrationSpan = 20;

// Latest measured raw values from QTR_Raw_Read_Test.
// For RC QTR sensors, darker surfaces generally produce larger timing values.
constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};

constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

// If the robot turns away from the black line, flip this to -1.
constexpr int kLineErrorSign = 1;

constexpr int kDefaultBaseSpeed = 400;
constexpr int kDefaultMaxCorrection = 600;
constexpr int kDefaultHardTurnSpeed = 500;
constexpr int kDefaultSearchTurnSpeed = 220;
constexpr float kDefaultKp = 0.8f;
constexpr float kDefaultKi = 0.0f;
constexpr float kDefaultKd = 0.08f;
constexpr uint16_t kDefaultLineThreshold = 230;
constexpr uint16_t kStrongLineThreshold = 650;
constexpr int kHardTurnError = 2600;
constexpr int kCenterRecoverError = 900;
constexpr uint8_t kIntegralClamp = 120;
constexpr uint32_t kPrintIntervalMs = 120;
constexpr uint32_t kMotoronPrintIntervalMs = 800;
constexpr uint32_t kLoopDelayMs = 8;

MotoronI2C motoron(kMotoronI2cAddress);

uint16_t qtrRaw[9] = {};
uint16_t qtrMin[9] = {};
uint16_t qtrMax[9] = {};
uint16_t qtrNorm[9] = {};

float kp = kDefaultKp;
float ki = kDefaultKi;
float kd = kDefaultKd;
int baseSpeed = kDefaultBaseSpeed;
int maxCorrection = kDefaultMaxCorrection;
int hardTurnSpeed = kDefaultHardTurnSpeed;
int searchTurnSpeed = kDefaultSearchTurnSpeed;
uint16_t lineThreshold = kDefaultLineThreshold;

int lastError = 0;
int lastSeenError = 0;
float integralError = 0.0f;
bool lineDetected = false;
bool serialStopped = false;
uint32_t lastPrintMs = 0;
uint32_t lastMotoronPrintMs = 0;

enum class FollowMode {
  Follow,
  HardLeft,
  HardRight,
  SearchLeft,
  SearchRight,
  Stopped
};

const __FlashStringHelper *modeName(FollowMode mode) {
  switch (mode) {
    case FollowMode::Follow:
      return F("FOLLOW");
    case FollowMode::HardLeft:
      return F("HARD_LEFT");
    case FollowMode::HardRight:
      return F("HARD_RIGHT");
    case FollowMode::SearchLeft:
      return F("SEARCH_LEFT");
    case FollowMode::SearchRight:
      return F("SEARCH_RIGHT");
    case FollowMode::Stopped:
      return F("STOPPED");
    default:
      return F("UNKNOWN");
  }
}

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
  return digitalRead(kMechanicalKillPin) == LOW;
}

void printSettings() {
  Serial.print(F("[CTRL] base="));
  Serial.print(baseSpeed);
  Serial.print(F(" maxcorr="));
  Serial.print(maxCorrection);
  Serial.print(F(" hard="));
  Serial.print(hardTurnSpeed);
  Serial.print(F(" search="));
  Serial.print(searchTurnSpeed);
  Serial.print(F(" P="));
  Serial.print(kp, 4);
  Serial.print(F(" I="));
  Serial.print(ki, 4);
  Serial.print(F(" D="));
  Serial.print(kd, 4);
  Serial.print(F(" threshold="));
  Serial.print(lineThreshold);
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
}

void printHelp() {
  Serial.println(F("Serial commands: p 0.14 | i 0 | d 0.08 | base 230 | maxcorr 320 | th 230 | hard 260 | search 220 | stop | resume | show"));
}

void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printHelp();
    return;
  }

  if (lower == "show") {
    printSettings();
    return;
  }

  if (lower == "stop") {
    serialStopped = true;
    stopMotors();
    Serial.println(F("[SERIAL] stopped."));
    return;
  }

  if (lower == "resume") {
    serialStopped = false;
    integralError = 0.0f;
    lastError = 0;
    Serial.println(F("[SERIAL] resumed."));
    return;
  }

  const int space = line.indexOf(' ');
  if (space <= 0) {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  const String key = lower.substring(0, space);
  const String valueText = line.substring(space + 1);
  const float value = valueText.toFloat();

  if (key == "p" || key == "kp") {
    kp = value;
  } else if (key == "i" || key == "ki") {
    ki = value;
  } else if (key == "d" || key == "kd") {
    kd = value;
  } else if (key == "base") {
    baseSpeed = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "maxcorr") {
    maxCorrection = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "th" || key == "threshold") {
    lineThreshold = constrain(static_cast<int>(value), 0, 1000);
  } else if (key == "hard") {
    hardTurnSpeed = constrain(static_cast<int>(value), 0, 800);
  } else if (key == "search") {
    searchTurnSpeed = constrain(static_cast<int>(value), 0, 800);
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  printSettings();
}

void handleSerialCommands() {
  static String input;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      processSerialCommand(input);
      input = "";
      continue;
    }
    if (input.length() < 80) {
      input += c;
    }
  }
}

void readQtrRcArray() {
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], OUTPUT);
    digitalWrite(kQtrPins[i], HIGH);
  }
  delayMicroseconds(10);

  const uint32_t start = micros();
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
    qtrRaw[i] = kQtrTimeoutUs;
  }

  bool allDone = false;
  while (!allDone && (micros() - start) < kQtrTimeoutUs) {
    allDone = true;
    const uint16_t elapsed = static_cast<uint16_t>(micros() - start);
    for (uint8_t i = 0; i < 9; i++) {
      if (qtrRaw[i] == kQtrTimeoutUs) {
        if (digitalRead(kQtrPins[i]) == LOW) {
          qtrRaw[i] = elapsed;
        } else {
          allDone = false;
        }
      }
    }
  }
}

void resetCalibration() {
  for (uint8_t i = 0; i < 9; i++) {
    qtrMin[i] = kQtrTimeoutUs;
    qtrMax[i] = 0;
  }
}

void updateCalibration() {
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrRaw[i] < qtrMin[i]) qtrMin[i] = qtrRaw[i];
    if (qtrRaw[i] > qtrMax[i]) qtrMax[i] = qtrRaw[i];
  }
}

void normalizeQtrValues() {
  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t span = qtrMax[i] > qtrMin[i] ? qtrMax[i] - qtrMin[i] : 0;
    if (span < kMinUsefulCalibrationSpan || qtrRaw[i] <= qtrMin[i]) {
      qtrNorm[i] = 0;
    } else if (qtrRaw[i] >= qtrMax[i]) {
      qtrNorm[i] = 1000;
    } else {
      qtrNorm[i] = static_cast<uint16_t>(
          (static_cast<uint32_t>(qtrRaw[i] - qtrMin[i]) * 1000UL) / span);
    }
  }
}

void printCalibration() {
  Serial.print(F("QTR min:"));
  for (uint8_t i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(qtrMin[i]);
  }
  Serial.println();

  Serial.print(F("QTR max:"));
  for (uint8_t i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(qtrMax[i]);
  }
  Serial.println();
}

void initializeQtrCalibration() {
  if (kUseSavedCalibration) {
    for (uint8_t i = 0; i < 9; i++) {
      qtrMin[i] = kSavedQtrMin[i];
      qtrMax[i] = kSavedQtrMax[i];
    }
    Serial.println(F("Using saved QTR calibration."));
    printCalibration();
    return;
  }

  Serial.println(F("Calibrating QTR for 5 seconds. Move sensor across white floor and black line."));
  resetCalibration();
  const uint32_t start = millis();
  while (millis() - start < kCalibrationMs) {
    readQtrRcArray();
    updateCalibration();
    delay(10);
  }
  printCalibration();
}

int computeLinePosition() {
  normalizeQtrValues();

  uint32_t weighted = 0;
  uint32_t sum = 0;
  lineDetected = false;

  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t weight = qtrNorm[i] >= lineThreshold ? qtrNorm[i] : 0;
    if (weight > 0) {
      lineDetected = true;
      weighted += static_cast<uint32_t>(weight) * (i * 1000);
      sum += weight;
    }
  }

  if (sum == 0) {
    return 4000 + lastSeenError;
  }

  const int position = static_cast<int>(weighted / sum);
  lastSeenError = position - 4000;
  return position;
}

uint8_t activeSensorCount(uint16_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

bool centerHasLine() {
  return qtrNorm[3] >= lineThreshold || qtrNorm[4] >= lineThreshold || qtrNorm[5] >= lineThreshold;
}

FollowMode chooseMode(int error) {
  if (serialStopped || killPressed()) {
    return FollowMode::Stopped;
  }

  if (!lineDetected) {
    integralError = 0.0f;
    return lastSeenError < 0 ? FollowMode::SearchLeft : FollowMode::SearchRight;
  }

  const bool leftEdgeStrong = qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
  const bool rightEdgeStrong = qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;

  if (error < -kHardTurnError || (leftEdgeStrong && error < -kCenterRecoverError)) {
    integralError = 0.0f;
    return FollowMode::HardLeft;
  }

  if (error > kHardTurnError || (rightEdgeStrong && error > kCenterRecoverError)) {
    integralError = 0.0f;
    return FollowMode::HardRight;
  }

  if (centerHasLine()) {
    return FollowMode::Follow;
  }

  return error < 0 ? FollowMode::HardLeft : FollowMode::HardRight;
}

void computeMotorCommand(FollowMode mode, int error, int *leftSpeed, int *rightSpeed) {
  switch (mode) {
    case FollowMode::Stopped:
      *leftSpeed = 0;
      *rightSpeed = 0;
      return;

    case FollowMode::SearchLeft:
      *leftSpeed = -searchTurnSpeed;
      *rightSpeed = searchTurnSpeed;
      return;

    case FollowMode::SearchRight:
      *leftSpeed = searchTurnSpeed;
      *rightSpeed = -searchTurnSpeed;
      return;

    case FollowMode::HardLeft:
      *leftSpeed = -hardTurnSpeed;
      *rightSpeed = hardTurnSpeed;
      return;

    case FollowMode::HardRight:
      *leftSpeed = hardTurnSpeed;
      *rightSpeed = -hardTurnSpeed;
      return;

    case FollowMode::Follow:
    default:
      break;
  }

  integralError += error / 1000.0f;
  integralError = constrain(integralError, -static_cast<float>(kIntegralClamp), static_cast<float>(kIntegralClamp));

  const int derivative = error - lastError;
  lastError = error;

  int correction = static_cast<int>(kp * error + ki * integralError + kd * derivative);
  correction = constrain(correction, -maxCorrection, maxCorrection);

  *leftSpeed = baseSpeed + correction;
  *rightSpeed = baseSpeed - correction;
}

void printStatus(FollowMode mode, int position, int error, int leftSpeed, int rightSpeed) {
  if (millis() - lastPrintMs < kPrintIntervalMs) return;
  lastPrintMs = millis();

  Serial.print(F("mode="));
  Serial.print(modeName(mode));
  Serial.print(F(" line="));
  Serial.print(lineDetected ? F("YES") : F("NO "));
  Serial.print(F(" active="));
  Serial.print(activeSensorCount(lineThreshold));
  Serial.print(F(" pos="));
  Serial.print(position);
  Serial.print(F(" err="));
  Serial.print(error);
  Serial.print(F(" L="));
  Serial.print(leftSpeed);
  Serial.print(F(" R="));
  Serial.print(rightSpeed);
  Serial.print(F(" raw:"));
  for (uint8_t i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(qtrRaw[i]);
  }
  Serial.print(F(" norm:"));
  for (uint8_t i = 0; i < 9; i++) {
    Serial.print(' ');
    Serial.print(qtrNorm[i]);
  }
  Serial.println();
}

void printMotoronFeedback() {
  if (millis() - lastMotoronPrintMs < kMotoronPrintIntervalMs) return;
  lastMotoronPrintMs = millis();

  const int16_t targetL = motoron.getTargetSpeed(kMotoronLeftChannel);
  const uint8_t errAfterTargetL = motoron.getLastError();
  const int16_t targetR = motoron.getTargetSpeed(kMotoronRightChannel);
  const uint8_t errAfterTargetR = motoron.getLastError();
  const int16_t currentL = motoron.getCurrentSpeed(kMotoronLeftChannel);
  const uint8_t errAfterCurrentL = motoron.getLastError();
  const int16_t currentR = motoron.getCurrentSpeed(kMotoronRightChannel);
  const uint8_t errAfterCurrentR = motoron.getLastError();
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t errAfterStatus = motoron.getLastError();
  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t errAfterVin = motoron.getLastError();

  Serial.print(F(" Motoron targetL="));
  Serial.print(targetL);
  Serial.print(F(" targetR="));
  Serial.print(targetR);
  Serial.print(F(" currentL="));
  Serial.print(currentL);
  Serial.print(F(" currentR="));
  Serial.print(currentR);
  Serial.print(F(" status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" errSeq="));
  Serial.print(errAfterTargetL);
  Serial.print(',');
  Serial.print(errAfterTargetR);
  Serial.print(',');
  Serial.print(errAfterCurrentL);
  Serial.print(',');
  Serial.print(errAfterCurrentR);
  Serial.print(',');
  Serial.print(errAfterStatus);
  Serial.print(',');
  Serial.println(errAfterVin);
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

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  pinMode(kMechanicalKillPin, INPUT_PULLUP);
  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);

  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
  }

  initializeMotoron();

  Serial.println(F("LineFollow_Test ready."));
  Serial.println(F("QTR-HD-09RC: black line should produce higher raw RC timing values."));
  Serial.println(F("Motoron: Wire1 / shield SDA1-SCL1, address 0x11, M1=left, M2=right."));
  Serial.println(F("Pull D32 to GND to stop."));
  printHelp();
  printSettings();
  initializeQtrCalibration();
}

void loop() {
  handleSerialCommands();

  readQtrRcArray();
  const int position = computeLinePosition();
  const int error = (position - 4000) * kLineErrorSign;
  const FollowMode mode = chooseMode(error);

  int leftSpeed = 0;
  int rightSpeed = 0;
  computeMotorCommand(mode, error, &leftSpeed, &rightSpeed);

  if (mode == FollowMode::Stopped) {
    stopMotors();
  } else {
    setTank(leftSpeed, rightSpeed);
  }

  printStatus(mode, position, error, leftSpeed, rightSpeed);
  printMotoronFeedback();
  delay(kLoopDelayMs);
}
