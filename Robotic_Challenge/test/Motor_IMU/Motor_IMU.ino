#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Motor_IMU
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   Test "turn this many degrees" using the ICM20948 IMU as the main angle
//   feedback, while keeping encoder feedback for debugging only.
//
// Hardware assumptions:
//   Motoron M3S550 address 0x11 on Wire1 / shield SDA1-SCL1
//   Motoron M1 = left motor, M2 = right motor
//   Left encoder C1/C2  -> D34/D35
//   Right encoder C1/C2 -> D36/D37
//   ICM20948 IMU        -> GIGA Wire bus, SDA D20 / SCL D21, address 0x68
//   Kill button         -> D32 to GND, INPUT_PULLUP
//
// Current confirmed I2C scanner map:
//   Wire1 / shield SDA1-SCL1: Motoron 0x11, shield device 0x60
//   Wire  / D20 SDA-D21 SCL: RFID 0x28, Modulino Distance 0x29,
//                            Modulino Pixels 0x36, ICM20948 IMU 0x68
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

constexpr uint8_t kImuAddress = 0x68;
constexpr uint16_t kGyroBiasSamples = 500;
constexpr uint16_t kGyroBiasSampleDelayMs = 4;

constexpr uint8_t kLeftEncoderAPin = 34;
constexpr uint8_t kLeftEncoderBPin = 35;
constexpr uint8_t kRightEncoderAPin = 36;
constexpr uint8_t kRightEncoderBPin = 37;

constexpr bool kUseKillPin = true;
constexpr uint8_t kKillPin = 32;

// Flip these if a motor runs backward relative to the intended command.
constexpr int kLeftMotorSign = 1;
constexpr int kRightMotorSign = 1;

// If positive target angles turn the robot the wrong way, flip this to -1.
constexpr int kTurnCommandSign = 1;

// If the printed yaw goes negative while the robot is physically turning in
// your intended positive direction, flip this to -1.
constexpr int kImuYawSign = 1;

// Geometry is still useful for encoder debug.
constexpr float kWheelDiameterMm = 39.0f;
constexpr float kWheelTrackMm = 165.0f;
constexpr float kMotorNoLoadRpm = 200.0f;        // Waveshare DCGM-N20-12V-EN-200RPM no-load speed.
constexpr float kEncoderCountsPerMotorRev = 7.0f; // C1/A rising-edge pulses before gearbox; 1050 per wheel rev after 1:150.
constexpr float kGearRatio = 150.0f;
constexpr float kTurnCalibration = 1.5f;

// IMU closed-loop turn control.
constexpr int kDefaultTurnMaxSpeed = 600;
constexpr int kDefaultTurnMinSpeed = 115;
constexpr int kDefaultFineTurnMaxSpeed = 180;
constexpr int kDefaultFineTurnMinSpeed = 75;
constexpr float kDefaultFineZoneDeg = 18.0f;
constexpr bool kDefaultUseFineZone = false;
constexpr float kDefaultTurnKp = 500.0f; // Runtime adjustable with Serial: p 80
constexpr float kDefaultTurnKd = 0.0f;   // Runtime adjustable with Serial: d 1.2
constexpr float kTurnToleranceDeg = 2.0f;
constexpr float kGyroStopRateDps = 10.0f;
constexpr uint32_t kTurnTimeoutMs = 50000;
constexpr bool kUseImuTurnTimeout = false; // false = no automatic IMU turn timeout.

// Turning uses a hard equal-speed rule: left and right motors always receive
// the same magnitude in opposite directions. Encoders are debug/stop fallback.

constexpr float kTestAnglesDeg[] = {90.0f, -90.0f, 180.0f};
constexpr uint32_t kStopBetweenTurnsMs = 3000;
constexpr bool kRunSequenceOnce = true;
constexpr uint32_t kRepeatPauseMs = 8000;
constexpr uint32_t kPrintIntervalMs = 120;

constexpr float kPi = 3.1415926f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;

MotoronI2C motoron(kMotoronAddress);
Adafruit_ICM20948 imu;

volatile long leftCount = 0;
volatile long rightCount = 0;

bool imuOk = false;
float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;
float turnKp = kDefaultTurnKp;
float turnKd = kDefaultTurnKd;
int turnMaxSpeed = kDefaultTurnMaxSpeed;
int turnMinSpeed = kDefaultTurnMinSpeed;
int fineTurnMaxSpeed = kDefaultFineTurnMaxSpeed;
int fineTurnMinSpeed = kDefaultFineTurnMinSpeed;
float fineZoneDeg = kDefaultFineZoneDeg;
bool useFineZone = kDefaultUseFineZone;
bool serialStopRequested = false;
bool pendingSerialTurn = false;
float pendingSerialTurnDeg = 0.0f;

void stopMotors();

long absLong(long value) {
  return value < 0 ? -value : value;
}

float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

int clampMotorSpeed(int speed) {
  if (speed > 800) return 800;
  if (speed < -800) return -800;
  return speed;
}

void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
}

long getLeftCount() {
  noInterrupts();
  const long count = leftCount;
  interrupts();
  return count;
}

long getRightCount() {
  noInterrupts();
  const long count = rightCount;
  interrupts();
  return count;
}

void resetEncoders() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
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

void printSerialHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  p 80        -> set Kp"));
  Serial.println(F("  d 1.2       -> set Kd"));
  Serial.println(F("  pd 80 1.2   -> set Kp and Kd together"));
  Serial.println(F("  max 600     -> set normal max turn speed"));
  Serial.println(F("  min 80      -> set normal min turn speed"));
  Serial.println(F("  fine off    -> disable fine-zone speed cap"));
  Serial.println(F("  fine on     -> enable fine-zone speed cap"));
  Serial.println(F("  finezone 18 -> set fine-zone size in degrees"));
  Serial.println(F("  finemax 180 -> set fine-zone max speed"));
  Serial.println(F("  finemin 75  -> set fine-zone min speed"));
  Serial.println(F("  t 90        -> run one IMU turn with current Kp/Kd"));
  Serial.println(F("  stop        -> stop motors and abort current turn"));
  Serial.println(F("  resume      -> clear serial stop flag"));
  Serial.println(F("  show        -> print current Kp/Kd"));
}

void printControlSettings() {
  Serial.print(F("[CTRL] Kp="));
  Serial.print(turnKp, 4);
  Serial.print(F(" Kd="));
  Serial.print(turnKd, 4);
  Serial.print(F(" min="));
  Serial.print(turnMinSpeed);
  Serial.print(F(" max="));
  Serial.print(turnMaxSpeed);
  Serial.print(F(" fine="));
  Serial.print(useFineZone ? F("ON") : F("OFF"));
  Serial.print(F(" fineZone="));
  Serial.print(fineZoneDeg, 1);
  Serial.print(F(" fineMin="));
  Serial.print(fineTurnMinSpeed);
  Serial.print(F(" fineMax="));
  Serial.print(fineTurnMaxSpeed);
  Serial.print(F(" timeout="));
  Serial.print(kUseImuTurnTimeout ? F("ON") : F("OFF"));
  Serial.print(F(" serialStop="));
  Serial.println(serialStopRequested ? F("YES") : F("NO"));
}

void processSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String lower = line;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printSerialHelp();
    return;
  }

  if (lower == "show") {
    printControlSettings();
    return;
  }

  if (lower == "stop") {
    serialStopRequested = true;
    pendingSerialTurn = false;
    stopMotors();
    Serial.println(F("[SERIAL] stop requested. Motors stopped."));
    return;
  }

  if (lower == "resume") {
    serialStopRequested = false;
    Serial.println(F("[SERIAL] stop flag cleared."));
    return;
  }

  if (lower.startsWith("p ") || lower.startsWith("kp ")) {
    const int firstSpace = line.indexOf(' ');
    turnKp = line.substring(firstSpace + 1).toFloat();
    printControlSettings();
    return;
  }

  if (lower.startsWith("p=") || lower.startsWith("kp=")) {
    const int equals = line.indexOf('=');
    turnKp = line.substring(equals + 1).toFloat();
    printControlSettings();
    return;
  }

  if (lower.startsWith("d ") || lower.startsWith("kd ")) {
    const int firstSpace = line.indexOf(' ');
    turnKd = line.substring(firstSpace + 1).toFloat();
    printControlSettings();
    return;
  }

  if (lower.startsWith("d=") || lower.startsWith("kd=")) {
    const int equals = line.indexOf('=');
    turnKd = line.substring(equals + 1).toFloat();
    printControlSettings();
    return;
  }

  if (lower.startsWith("pd ")) {
    String rest = line.substring(3);
    rest.trim();
    const int space = rest.indexOf(' ');
    if (space > 0) {
      turnKp = rest.substring(0, space).toFloat();
      turnKd = rest.substring(space + 1).toFloat();
      printControlSettings();
    } else {
      Serial.println(F("[SERIAL] Use: pd 80 1.2"));
    }
    return;
  }

  if (lower.startsWith("max ")) {
    const int firstSpace = line.indexOf(' ');
    turnMaxSpeed = constrain(line.substring(firstSpace + 1).toInt(), 0, 800);
    printControlSettings();
    return;
  }

  if (lower.startsWith("min ")) {
    const int firstSpace = line.indexOf(' ');
    turnMinSpeed = constrain(line.substring(firstSpace + 1).toInt(), 0, 800);
    printControlSettings();
    return;
  }

  if (lower == "fine on") {
    useFineZone = true;
    printControlSettings();
    return;
  }

  if (lower == "fine off") {
    useFineZone = false;
    printControlSettings();
    return;
  }

  if (lower.startsWith("finezone ")) {
    const int firstSpace = line.indexOf(' ');
    fineZoneDeg = line.substring(firstSpace + 1).toFloat();
    printControlSettings();
    return;
  }

  if (lower.startsWith("finemax ")) {
    const int firstSpace = line.indexOf(' ');
    fineTurnMaxSpeed = constrain(line.substring(firstSpace + 1).toInt(), 0, 800);
    printControlSettings();
    return;
  }

  if (lower.startsWith("finemin ")) {
    const int firstSpace = line.indexOf(' ');
    fineTurnMinSpeed = constrain(line.substring(firstSpace + 1).toInt(), 0, 800);
    printControlSettings();
    return;
  }

  if (lower.startsWith("t ") || lower.startsWith("turn ")) {
    const int firstSpace = line.indexOf(' ');
    pendingSerialTurnDeg = line.substring(firstSpace + 1).toFloat();
    pendingSerialTurn = true;
    serialStopRequested = false;
    Serial.print(F("[SERIAL] queued turn "));
    Serial.print(pendingSerialTurnDeg, 1);
    Serial.println(F(" deg."));
    return;
  }

  Serial.print(F("[SERIAL] Unknown command: "));
  Serial.println(line);
  printSerialHelp();
}

void handleSerialCommands() {
  static String input;

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
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

void setTurnCommand(int signedTurnSpeed, int balanceCorrection) {
  (void)balanceCorrection;
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  setTank(-command, command);
}

void printMotoronState() {
  const uint16_t status = motoron.getStatusFlags();
  const uint8_t statusErr = motoron.getLastError();
  const uint32_t vinMv = motoron.getVinVoltageMv(kMotoronReferenceMv, kMotoronVinType);
  const uint8_t vinErr = motoron.getLastError();

  Serial.print(F("Motoron status=0x"));
  Serial.print(status, HEX);
  Serial.print(F(" statusErr="));
  Serial.print(statusErr);
  Serial.print(F(" vinMv="));
  Serial.print(vinMv);
  Serial.print(F(" vinErr="));
  Serial.println(vinErr);
}

bool updateImu() {
  if (!imuOk) {
    return false;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  const uint32_t now = micros();
  float dt = (now - lastImuUpdateUs) / 1000000.0f;
  lastImuUpdateUs = now;
  if (dt < 0.0f || dt > 0.25f) {
    dt = 0.0f;
  }

  const float rawGyroZDps = gyro.gyro.z * kRadToDeg;
  gyroZDegPerSec = (rawGyroZDps - gyroZBiasDps) * kImuYawSign;
  yawDeg += gyroZDegPerSec * dt;
  return true;
}

void resetYaw() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

bool initializeImu() {
  Wire.begin();

  Serial.print(F("Starting ICM20948 at I2C address 0x"));
  Serial.println(kImuAddress, HEX);

  if (!imu.begin_I2C(kImuAddress, &Wire)) {
    Serial.println(F("ICM20948 not found. IMU turn test cannot run."));
    return false;
  }

  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;

  Serial.println(F("ICM20948 found. Keep robot still for gyro bias calibration."));
  delay(800);

  float sum = 0.0f;
  for (uint16_t i = 0; i < kGyroBiasSamples; ++i) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp, &mag);
    sum += gyro.gyro.z * kRadToDeg;
    delay(kGyroBiasSampleDelayMs);
  }

  gyroZBiasDps = sum / kGyroBiasSamples;
  resetYaw();

  Serial.print(F("Gyro Z bias = "));
  Serial.print(gyroZBiasDps, 4);
  Serial.println(F(" deg/s"));
  return true;
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

void printTurnState(const char *label, float targetDeg, float errorDeg, int command, long encoderTarget) {
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs < kPrintIntervalMs) {
    return;
  }
  lastPrintMs = millis();

  const long left = getLeftCount();
  const long right = getRightCount();
  Serial.print(label);
  Serial.print(F(" target="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" yaw="));
  Serial.print(yawDeg, 2);
  Serial.print(F(" err="));
  Serial.print(errorDeg, 2);
  Serial.print(F(" gyroZ="));
  Serial.print(gyroZDegPerSec, 1);
  Serial.print(F(" cmd="));
  Serial.print(command);
  Serial.print(F(" Kp="));
  Serial.print(turnKp, 2);
  Serial.print(F(" Kd="));
  Serial.print(turnKd, 2);
  Serial.print(F(" L="));
  Serial.print(left);
  Serial.print(F(" R="));
  Serial.print(right);
  Serial.print(F(" absAvg="));
  Serial.print((absLong(left) + absLong(right)) / 2);
  Serial.print(F(" encTarget="));
  Serial.println(encoderTarget);
}

void pauseStopped(uint32_t durationMs) {
  stopMotors();
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    handleSerialCommands();
    if (killPressed()) {
      Serial.println(F("KILL pressed during pause."));
      stopMotors();
      return;
    }
    updateImu();
    delay(20);
  }
}

void encoderOnlyTurnFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;

  Serial.print(F("Encoder fallback turn degrees="));
  Serial.print(degrees);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    if (serialStopRequested) {
      Serial.println(F("Serial stop. Encoder fallback turn aborted."));
      break;
    }
    if (killPressed()) {
      Serial.println(F("KILL pressed. Encoder fallback turn aborted."));
      break;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) {
      break;
    }
    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("Encoder fallback turn timeout."));
      break;
    }

    setTurnCommand(direction * abs(speed), 0);
    printTurnState("EncoderTurn", degrees, 0.0f, direction * abs(speed), targetCounts);
    delay(10);
  }

  stopMotors();
}

void turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, turnMaxSpeed);
    return;
  }

  const long encoderTarget = turnDegreesToEncoderCounts(targetDeg);

  Serial.println();
  Serial.print(F("IMU turn targetDeg="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" maxSpeed="));
  Serial.print(turnMaxSpeed);
  Serial.print(F(" encoderTarget="));
  Serial.print(encoderTarget);
  Serial.print(F(" Kp="));
  Serial.print(turnKp, 3);
  Serial.print(F(" Kd="));
  Serial.print(turnKd, 3);
  Serial.print(F(" timeout="));
  Serial.println(kUseImuTurnTimeout ? F("ON") : F("OFF"));

  resetEncoders();
  resetYaw();

  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    if (serialStopRequested) {
      Serial.println(F("Serial stop. IMU turn aborted."));
      break;
    }

    if (killPressed()) {
      Serial.println(F("KILL pressed. IMU turn aborted."));
      break;
    }

    updateImu();
    const float errorDeg = targetDeg - yawDeg;
    const float absError = absFloat(errorDeg);

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());

    if (absError <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("IMU turn reached target."));
      break;
    }

    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("IMU turn timeout."));
      break;
    }

    const bool inFineZone = useFineZone && absError < fineZoneDeg;
    const int activeMaxSpeed = inFineZone ? fineTurnMaxSpeed : turnMaxSpeed;
    const int activeMinSpeed = inFineZone ? fineTurnMinSpeed : turnMinSpeed;

    float commandFloat = turnKp * errorDeg - turnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, activeMinSpeed, activeMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;

    setTurnCommand(signedCommand, 0);
    printTurnState("IMUTurn", targetDeg, errorDeg, signedCommand, encoderTarget);
    delay(10);
  }

  stopMotors();
  updateImu();
  printTurnState("IMUTurn final", targetDeg, targetDeg - yawDeg, 0, encoderTarget);
}

void runTurnSequence() {
  Serial.println(F("=== Motor_IMU turn sequence ==="));
  for (uint8_t i = 0; i < sizeof(kTestAnglesDeg) / sizeof(kTestAnglesDeg[0]); ++i) {
    if (serialStopRequested) {
      Serial.println(F("Serial stop flag is active. Sequence aborted."));
      break;
    }
    turnDegreesImu(kTestAnglesDeg[i]);
    pauseStopped(kStopBetweenTurnsMs);
  }
}

void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
  }

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);

  initializeMotoron();
  initializeImu();

  Serial.println(F("Motor_IMU ready."));
  Serial.println(F("Positive target uses setTurnCommand(+): left motor reverse, right motor forward."));
  Serial.println(F("If yaw moves away from target, flip kImuYawSign or kTurnCommandSign."));
  printSerialHelp();
  printControlSettings();
  Serial.print(F("Motor spec: no-load rpm="));
  Serial.print(kMotorNoLoadRpm, 1);
  Serial.print(F(", gear ratio=1:"));
  Serial.print(kGearRatio, 0);
  Serial.print(F(", encoder PPR="));
  Serial.println(kEncoderCountsPerMotorRev, 1);
  Serial.print(F("Counts per mm estimate = "));
  Serial.println(kEncoderCountsPerMm, 4);
  printMotoronState();
  pauseStopped(2000);
}

void loop() {
  runTurnSequence();

  Serial.println(F("Motor_IMU sequence complete."));
  if (kRunSequenceOnce) {
    Serial.println(F("Sequence is configured to run once. Motors stopped forever."));
    stopMotors();
    while (true) {
      handleSerialCommands();
      if (pendingSerialTurn) {
        const float requestedTurn = pendingSerialTurnDeg;
        pendingSerialTurn = false;
        turnDegreesImu(requestedTurn);
        stopMotors();
      }
      updateImu();
      delay(50);
    }
  }

  pauseStopped(kRepeatPauseMs);
}
