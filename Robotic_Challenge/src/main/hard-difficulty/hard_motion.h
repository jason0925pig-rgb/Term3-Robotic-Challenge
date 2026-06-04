#pragma once

#include "hard_config.h"
#include "hard_forward.h"

inline void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

/**
 * Right encoder ISR using channel B for direction.
 *
 * This is attached to channel A rising edges to match the encoder constants.
 */
inline void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
}

/**
 * Read the left encoder count atomically.
 *
 * @return Snapshot of leftCount.
 */
inline long getLeftCount() {
  noInterrupts();
  const long count = leftCount;
  interrupts();
  return count;
}

/**
 * Read the right encoder count atomically.
 *
 * @return Snapshot of rightCount.
 */
inline long getRightCount() {
  noInterrupts();
  const long count = rightCount;
  interrupts();
  return count;
}

/**
 * Reset both encoder counters atomically.
 */
inline void resetEncoders() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

/**
 * Convert straight travel distance to average encoder counts.
 *
 * @param distanceMm Signed distance in millimeters; sign is ignored here.
 * @return Positive encoder target count.
 */
inline long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

/**
 * Convert in-place turn angle to encoder counts for fallback turning.
 *
 * @param degrees Signed turn angle; sign is ignored here.
 * @return Positive per-side encoder target count.
 */
inline long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

inline bool initializeImuHardware() {
  Serial.print(F("[IMU] starting ICM20948 at 0x"));
  Serial.println(kImuAddress, HEX);

  if (!imu.begin_I2C(kImuAddress, &Wire)) {
    Serial.println(F("[IMU] not found. Encoder-only fallback will be used for turns."));
    imuOk = false;
    return false;
  }

  imu.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  imu.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  imu.setMagDataRate(AK09916_MAG_DATARATE_50_HZ);
  imuOk = true;
  Serial.println(F("[IMU] found."));
  return true;
}

/**
 * Calibrate gyro Z bias while the robot is stopped.
 *
 * @return true if calibration succeeded, false if the IMU is unavailable.
 */
inline bool calibrateImuGyroBias() {
  if (!imuOk) return false;

  stopMotors();
  Serial.println(F("[IMU] calibrating gyro bias. Keep robot still."));
  delay(800);

  float sum = 0.0f;
  for (uint16_t i = 0; i < kGyroBiasSamples; ++i) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t mag;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp, &mag);
    sum += gyro.gyro.z * kRadToDeg;
    delay(kGyroBiasSampleDelayMs);
  }

  gyroZBiasDps = sum / kGyroBiasSamples;
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
  Serial.print(F("[IMU] gyro Z bias = "));
  Serial.print(gyroZBiasDps, 4);
  Serial.println(F(" deg/s"));
  return true;
}

/**
 * Integrate IMU gyro Z into relative yaw.
 *
 * @return true if IMU data was read, false if the IMU is unavailable.
 */
inline bool updateImu() {
  if (!imuOk) return false;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp, &mag);

  const uint32_t now = micros();
  float dt = (now - lastImuUpdateUs) / 1000000.0f;
  lastImuUpdateUs = now;
  if (dt < 0.0f || dt > 0.25f) dt = 0.0f;

  const float rawGyroZDps = gyro.gyro.z * kRadToDeg;
  gyroZDegPerSec = (rawGyroZDps - gyroZBiasDps) * kImuYawSign;
  yawDeg += gyroZDegPerSec * dt;
  return true;
}

/**
 * Reset relative yaw to zero.
 */
inline void resetYaw() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

/**
 * Command an in-place turn.
 *
 * @param signedTurnSpeed Positive uses the configured left-turn convention.
 */
inline void setTurnCommand(int signedTurnSpeed) {
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  setTank(-command, command);
}

/**
 * Print turn telemetry at a throttled rate.
 *
 * @param label Log prefix.
 * @param targetDeg Target relative yaw in degrees.
 * @param errorDeg Current yaw error.
 * @param command Signed motor turn command.
 * @param encoderTarget Encoder target used for fallback reference.
 */
inline void printTurnStatus(const char *label, float targetDeg, float errorDeg, int command, long encoderTarget) {
  if (millis() - lastTurnPrintMs < kTurnPrintIntervalMs) return;
  lastTurnPrintMs = millis();

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
  Serial.print(F(" L="));
  Serial.print(getLeftCount());
  Serial.print(F(" R="));
  Serial.print(getRightCount());
  Serial.print(F(" encTarget="));
  Serial.println(encoderTarget);
}

/**
 * Turn using encoders when the IMU is unavailable.
 *
 * @param degrees Signed turn angle.
 * @param speed Absolute turn speed.
 */
inline void encoderOnlyTurnFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) break;

    const long averageAbs = (absLong(getLeftCount()) + absLong(getRightCount())) / 2;
    if (averageAbs >= targetCounts) break;

    if (millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] encoder fallback timeout."));
      break;
    }

    setTurnCommand(direction * abs(speed));
    printTurnStatus("[EncoderTurn]", degrees, 0.0f, direction * abs(speed), targetCounts);
    delay(10);
  }
  stopMotors();
}

/**
 * Turn in place to a relative angle using IMU feedback.
 *
 * @param targetDeg Signed target angle; positive is configured as left.
 * @return true if completed, false if stopped or killed.
 */
inline bool turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, kTurnMaxSpeed);
    return !serialStopped;
  }

  const long encoderTarget = turnDegreesToEncoderCounts(targetDeg);
  resetEncoders();
  resetYaw();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();

  while (true) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    updateImu();
    const float errorDeg = targetDeg - yawDeg;
    if (absFloat(errorDeg) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      break;
    }

    if (kUseImuTurnTimeout && millis() - start > kTurnTimeoutMs) {
      Serial.println(F("[TURN] IMU timeout."));
      break;
    }

    float commandFloat = kTurnKp * errorDeg - kTurnKd * gyroZDegPerSec;
    int commandMagnitude = static_cast<int>(absFloat(commandFloat));
    commandMagnitude = constrain(commandMagnitude, kTurnMinSpeed, kTurnMaxSpeed);
    const int signedCommand = commandFloat >= 0.0f ? commandMagnitude : -commandMagnitude;

    setTurnCommand(signedCommand);
    printTurnStatus("[IMUTurn]", targetDeg, errorDeg, signedCommand, encoderTarget);
    delay(10);
  }

  stopMotors();
  updateImu();
  printTurnStatus("[IMUTurn final]", targetDeg, targetDeg - yawDeg, 0, encoderTarget);
  return true;
}

// ---------------------------------------------------------------------------
// Encoder distance drive
// ---------------------------------------------------------------------------

/**
 * Drive straight for a target distance using encoder balancing.
 *
 * @param distanceMm Signed distance in millimeters.
 * @param speed Absolute base speed.
 * @return true if completed or timed out, false if stopped or killed.
 */
inline bool driveDistanceMm(float distanceMm, int speed) {
  const long targetCounts = distanceMmToCounts(distanceMm);
  const int direction = distanceMm >= 0.0f ? 1 : -1;

  Serial.print(F("[DRIVE] distanceMm="));
  Serial.print(distanceMm, 1);
  Serial.print(F(" speed="));
  Serial.print(speed);
  Serial.print(F(" targetCounts="));
  Serial.println(targetCounts);

  resetEncoders();
  const uint32_t start = millis();

  while (true) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long averageAbs = (leftAbs + rightAbs) / 2;
    if (averageAbs >= targetCounts) break;

    if (millis() - start > kDriveTimeoutMs) {
      Serial.println(F("[DRIVE] timeout."));
      break;
    }

    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);

    const int base = abs(speed) * direction;
    setTank(base - correction, base + correction);
    updateImu();
    delay(10);
  }

  stopMotors();
  return true;
}


inline bool initializeImu() {
  return initializeImuHardware() && calibrateImuGyroBias();
}

inline ImuReading getImuReading() {
  updateImu();
  ImuReading reading;
  reading.ok = imuOk;
  reading.yawDeg = yawDeg;
  reading.gyroZDps = gyroZDegPerSec;
  return reading;
}
