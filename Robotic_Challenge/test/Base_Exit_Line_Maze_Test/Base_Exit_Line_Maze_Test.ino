#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Base_Exit_Line_Maze_Test
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   Follow the branched line inside the base using a classic line-maze method:
//   normal PD line follow on ordinary segments, then route-scripted 90 degree
//   turns at intersections/corners.
//
// Base route model:
//   From start, the robot reaches the first T/bifurcation.
//   Route B/left/upper:  LEFT, RIGHT, RIGHT, LEFT
//   Route A/right/lower: RIGHT, LEFT, LEFT, RIGHT
//
// Why this works better than "count black sensors" alone:
//   A wide line parallel to the robot can light the middle 5 sensors, so the
//   later T/corner events are detected using expected route direction plus
//   outer-edge evidence. The first bifurcation is the only place where this
//   sketch expects an all/near-all black bar.
//
// Hardware:
//   QTR CTRL odd/even -> D2/D3
//   QTR sensors       -> D22-D30, left to right when viewed from robot front
//   RFID              -> Wire / D20 SDA-D21 SCL, address 0x28
//   IMU ICM20948      -> Wire / D20 SDA-D21 SCL, address 0x68
//   Motoron           -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1/M2     -> left/right motor
//   Left encoder      -> D34/D35
//   Right encoder     -> D36/D37
//   Kill button       -> D32 to GND, INPUT_PULLUP
//
// Optional WiFi exit request:
//   Set USE_WIFI_EXIT_REQUEST to 1 and place secrets.h in this folder if you
//   want this sketch to send a best-effort MiniMessenger request after RFID.
//   The exact server message format may need adjustment for your challenge API.
// ---------------------------------------------------------------------------

// Set to 1 to compile the optional MiniMessenger WiFi exit-clearance hook.
#define USE_WIFI_EXIT_REQUEST 0

#if USE_WIFI_EXIT_REQUEST
#include <MiniMessenger.h>
#include "secrets.h"
#endif

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------

constexpr uint32_t kSerialBaud = 115200;  // Serial Monitor baud rate.

enum class RouteChoice {
  BaseA_Bottom,
  BaseB_Top
};

constexpr RouteChoice kDefaultRouteChoice = RouteChoice::BaseA_Bottom;  // Route used after reset/start; A is the right/lower branch.
constexpr bool kStartOnBoot = false;  // false = start in IDLE and wait for D32 button or serial "start".

constexpr float kFirstTAdvanceMm = 70.0f;  // Distance to drive after detecting the first T before turning.
constexpr float kSharpTurnAdvanceMm = 45.0f;  // Distance to drive after later hard-turn events before turning.
constexpr int kAdvanceSpeed = 300;  // Motor speed used for the short pre-turn advance.

constexpr uint32_t kDoorApproachMs = 5500;  // Time to keep following the final line toward the door after route turns.
constexpr bool kStopAfterDoorApproach = true;  // true = stop after kDoorApproachMs instead of running forever.

constexpr uint32_t kStopOverRfidMs = 1600;  // Time to remain stopped over the RFID tag before continuing.
constexpr uint32_t kRfidPollIntervalMs = 70;  // Minimum time between RFID reader polls.

// Line-following control. These are copied from LineFollow_Test as a starting
// point, with a slightly lower base speed for intersection reliability.
constexpr int kLineBaseSpeed = 340;  // Forward speed during normal PD line following.
constexpr int kLineMaxCorrection = 560;  // Maximum steering correction added/subtracted from base speed.
constexpr int kLineHardTurnSpeed = 450;  // In-place spin speed for hard-left/hard-right modes.
constexpr int kLineSearchTurnSpeed = 210;  // In-place spin speed used when all QTR sensors read white.
constexpr float kLineKp = 0.80f;  // Proportional gain for line position error.
constexpr float kLineKi = 0.0f;  // Integral gain; usually keep 0 unless steady bias needs correction.
constexpr float kLineKd = 0.08f;  // Derivative gain to damp fast line-error changes.
constexpr uint16_t kLineThreshold = 230;  // Normalized QTR value above which a sensor counts as black line.
constexpr uint16_t kStrongLineThreshold = 650;  // Strong black threshold for outer-edge hard-turn evidence.
constexpr int kHardTurnError = 2500;  // Line error magnitude that forces hard-turn mode.
constexpr int kCenterRecoverError = 900;  // Outer-edge + error threshold for declaring a hard turn.
constexpr float kLineIntegralClamp = 120.0f;  // Limit for accumulated integral error.
constexpr uint32_t kLinePrintIntervalMs = 130;  // Minimum time between line-following debug prints.
constexpr uint32_t kLoopDelayMs = 8;  // Delay at the end of each line-following loop step.

// Intersection confirmation. Stable frames prevents one noisy sample from
// firing a scripted turn.
constexpr uint8_t kFirstTMinActiveSensors = 8;  // Minimum active QTR sensors to accept the first T/bifurcation.
constexpr uint8_t kFirstTStableFrames = 5;  // Consecutive first-T frames required before committing the first turn.
constexpr float kFirstTMinTravelMm = 160.0f;  // Minimum travel from start before the first T can be accepted.
constexpr uint16_t kFirstTEdgeStrongThreshold = 650;  // Strong-black threshold required on both outer edges for first T.
constexpr uint16_t kFirstTMinTotalStrength = 5600;  // Minimum summed 9-sensor black strength for first-T confidence.
constexpr int kFirstTMaxCenterError = 900;  // Maximum centered line error allowed when accepting the first T.
constexpr uint8_t kSharpTurnStableFrames = 1;  // Later expected hard turns commit immediately when detected.
constexpr uint32_t kEventCooldownMs = 500;  // Ignore all route events briefly after a committed turn.

// Line reacquisition after an IMU turn.
constexpr int kReacquireTurnSpeed = 150;  // Slow spin speed used to find the line after a 90 degree turn.
constexpr uint32_t kReacquireTimeoutMs = 1400;  // Maximum time to actively search for the center line after turning.
constexpr uint8_t kReacquireStableFrames = 3;  // Center sensors must see line for this many frames to finish reacquire.
constexpr uint32_t kPostTurnHardIgnoreMs = 1100;  // Ignore hard-left/right readings briefly after each scripted turn.
constexpr uint8_t kPostTurnHardReleaseFrames = 4;  // Centered frames needed before re-enabling hard-turn route events.
constexpr int kPostTurnSoftErrorClamp = 650;  // Max line error used while soft-following through the post-turn ignore window.

// QTR-HD-09RC line sensor.
constexpr uint8_t kQtrCtrlOddPin = 2;  // QTR odd emitter control pin.
constexpr uint8_t kQtrCtrlEvenPin = 3;  // QTR even emitter control pin.
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};  // QTR sensor pins, left to right from robot front.
constexpr uint16_t kQtrTimeoutUs = 3000;  // Maximum RC discharge wait time per QTR reading.
constexpr uint16_t kMinUsefulCalibrationSpan = 20;  // Minimum raw max-min span needed to trust a sensor.
constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};  // Saved raw white-floor calibration.
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};  // Saved raw black-line calibration.

// Motoron and motor signs.
constexpr uint8_t kMotoronAddress = 0x11;  // I2C address of the Motoron motor controller.
constexpr uint8_t kMotoronLeftChannel = 1;  // Motoron channel connected to the left motor.
constexpr uint8_t kMotoronRightChannel = 2;  // Motoron channel connected to the right motor.
constexpr int kLeftMotorSign = 1;  // Flip to -1 if the left motor runs backward.
constexpr int kRightMotorSign = 1;  // Flip to -1 if the right motor runs backward.
constexpr int kLineErrorSign = 1;  // Flip to -1 if PD correction steers away from the line.
constexpr int kMaxMotorCommand = 800;  // Absolute Motoron command limit.

// Encoders.
constexpr uint8_t kLeftEncoderAPin = 34;  // Left encoder A/C1 pin.
constexpr uint8_t kLeftEncoderBPin = 35;  // Left encoder B/C2 direction pin.
constexpr uint8_t kRightEncoderAPin = 36;  // Right encoder A/C1 pin.
constexpr uint8_t kRightEncoderBPin = 37;  // Right encoder B/C2 direction pin.
constexpr float kWheelDiameterMm = 39.0f;  // Wheel diameter used for encoder distance conversion.
constexpr float kWheelTrackMm = 165.0f;  // Distance between left and right wheel contact lines for turn conversion.
constexpr float kEncoderCountsPerMotorRev = 7.0f;  // Encoder pulses per motor shaft revolution before gearbox.
constexpr float kGearRatio = 150.0f;  // Motor gearbox ratio.
constexpr float kDistanceCalibration = 1.0f;  // Scale factor for straight encoder distance correction.
constexpr float kTurnCalibration = 1.5f;  // Scale factor for encoder-only turn angle correction.
constexpr float kStraightCorrectionKp = 0.35f;  // Encoder balancing gain during straight distance drives.
constexpr int kMaxStraightCorrection = 90;  // Maximum encoder balancing correction during straight drives.
constexpr uint32_t kDriveTimeoutMs = 10000;  // Maximum time allowed for encoder distance drive.

// IMU yaw / turning.
constexpr uint8_t kImuAddress = 0x68;  // I2C address of the ICM20948 IMU.
constexpr uint16_t kGyroBiasSamples = 500;  // Number of gyro samples collected during startup bias calibration.
constexpr uint16_t kGyroBiasSampleDelayMs = 4;  // Delay between gyro bias samples.
constexpr int kTurnCommandSign = 1;  // Flip to -1 if positive turn command rotates the wrong physical direction.
constexpr int kImuYawSign = 1;  // Flip to -1 if yaw moves away from the target during IMU turns.
constexpr int kTurnMaxSpeed = 560;  // Maximum motor command during IMU-controlled turns.
constexpr int kTurnMinSpeed = 115;  // Minimum motor command during IMU-controlled turns.
constexpr float kTurnKp = 500.0f;  // Proportional gain from yaw error to turn command.
constexpr float kTurnKd = 0.0f;  // Derivative damping from gyro rate to turn command.
constexpr float kTurnToleranceDeg = 2.0f;  // Acceptable final yaw error for a turn.
constexpr float kGyroStopRateDps = 10.0f;  // Required low gyro rate before declaring turn complete.
constexpr bool kUseImuTurnTimeout = false;  // true = abort IMU turn after kTurnTimeoutMs.
constexpr uint32_t kTurnTimeoutMs = 50000;  // Timeout for IMU/encoder turn loops when enabled.
constexpr uint32_t kTurnPrintIntervalMs = 120;  // Minimum time between turn debug prints.

// RFID.
constexpr uint8_t kRfidAddress = 0x28;  // I2C address of the RFID2 reader.
constexpr uint8_t kRfidResetPin = 39;  // RFID reset pin.

// Mechanical kill.
constexpr bool kUseKillPin = true;  // true = use D32 as start/stop button.
constexpr uint8_t kKillPin = 32;  // Start/stop button pin, active LOW with INPUT_PULLUP.
constexpr uint32_t kKillDebounceMs = 35;  // Button debounce time before accepting a press event.

#if USE_WIFI_EXIT_REQUEST
constexpr const char *kBoardId = "Team2Robot";  // Board ID used in optional MiniMessenger registration.
constexpr uint32_t kRegisterIntervalMs = 5000;  // Interval between optional MiniMessenger registration messages.
#endif

constexpr float kPi = 3.1415926f;  // Pi constant for geometry calculations.
constexpr float kRadToDeg = 57.2957795f;  // Conversion factor from radians to degrees.
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;  // Derived wheel circumference.
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;  // Derived encoder counts per wheel revolution.
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;  // Derived encoder counts per millimeter.

// ---------------------------------------------------------------------------
// Types / globals
// ---------------------------------------------------------------------------

enum class MissionState {
  Idle,
  FollowRoute,
  StopOverRfid,
  FollowToDoor,
  Done,
  Stopped
};

enum class FollowMode {
  Follow,
  HardLeft,
  HardRight,
  SearchLeft,
  SearchRight,
  Stopped
};

enum class TurnDir {
  Left,
  Right
};

struct MotorCommand {
  int left = 0;
  int right = 0;
};

struct LineReading {
  uint16_t raw[9] = {};
  uint16_t norm[9] = {};
  bool detected = false;
  int position = 4000;
  int error = 0;
  uint8_t activeCount = 0;
  FollowMode mode = FollowMode::Stopped;
};

/**
 * Forward declaration for readLine().
 *
 * The serial command parser calls readLine() before the full function body
 * appears in this file, so this declaration keeps the build independent of
 * Arduino's automatic prototype generation.
 */
LineReading readLine();

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;

#if USE_WIFI_EXIT_REQUEST
MiniMessenger messenger;
uint32_t lastRegisterMs = 0;
#endif

uint16_t qtrRaw[9] = {};
uint16_t qtrMin[9] = {};
uint16_t qtrMax[9] = {};
uint16_t qtrNorm[9] = {};

volatile long leftCount = 0;
volatile long rightCount = 0;

RouteChoice routeChoice = kDefaultRouteChoice;
MissionState missionState = kStartOnBoot ? MissionState::FollowRoute : MissionState::Idle;

bool serialStopped = false;
bool rfidOk = false;
bool imuOk = false;
bool rfidHandled = false;

bool lastKillReading = HIGH;
bool stableKillReading = HIGH;
uint32_t lastKillChangeMs = 0;

uint8_t routeTurnIndex = 0;
uint8_t eventStableCount = 0;

String lastUid;

int lastLineError = 0;
int lastSeenLineError = 0;
float lineIntegral = 0.0f;
bool postTurnHardIgnoreActive = false;
uint32_t postTurnHardIgnoreStartMs = 0;
uint8_t postTurnHardReleaseCount = 0;

float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;

uint32_t lastLinePrintMs = 0;
uint32_t lastTurnPrintMs = 0;
uint32_t lastRfidPollMs = 0;
uint32_t lastEventMs = 0;
uint32_t stateStartMs = 0;

/**
 * Forward declaration used by post-turn route-event filtering.
 */
bool centerHasLine();

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

/**
 * Return the absolute value of a long integer.
 *
 * @param value Signed long value to convert.
 * @return Non-negative magnitude of value.
 */
long absLong(long value) {
  return value < 0 ? -value : value;
}

/**
 * Return the absolute value of a floating-point number.
 *
 * @param value Signed float value to convert.
 * @return Non-negative magnitude of value.
 */
float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

/**
 * Limit a requested Motoron speed command to the configured safe range.
 *
 * @param speed Requested motor command before limiting.
 * @return speed clamped to [-kMaxMotorCommand, kMaxMotorCommand].
 */
int clampMotorSpeed(int speed) {
  if (speed > kMaxMotorCommand) return kMaxMotorCommand;
  if (speed < -kMaxMotorCommand) return -kMaxMotorCommand;
  return speed;
}

/**
 * Read the mechanical kill input.
 *
 * @return true when the kill pin is enabled and pulled LOW by the switch.
 */
bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

/**
 * Detect one debounced kill-button press event.
 *
 * The raw kill switch is active LOW. This function returns true only once per
 * physical press, after the input has been stable for kKillDebounceMs.
 *
 * @return true on the debounced falling edge of the kill button.
 */
bool killButtonPressedEvent() {
  if (!kUseKillPin) return false;

  const bool reading = digitalRead(kKillPin);
  if (reading != lastKillReading) {
    lastKillReading = reading;
    lastKillChangeMs = millis();
  }

  if (millis() - lastKillChangeMs < kKillDebounceMs) {
    return false;
  }

  if (reading == stableKillReading) {
    return false;
  }

  stableKillReading = reading;
  return stableKillReading == LOW;
}

/**
 * Convert a mission-state enum to a flash-stored label for Serial output.
 *
 * @param state MissionState value to describe.
 * @return Printable label stored with F() to save SRAM.
 */
const __FlashStringHelper *stateName(MissionState state) {
  switch (state) {
    case MissionState::Idle: return F("IDLE");
    case MissionState::FollowRoute: return F("FOLLOW_ROUTE");
    case MissionState::StopOverRfid: return F("STOP_OVER_RFID");
    case MissionState::FollowToDoor: return F("FOLLOW_TO_DOOR");
    case MissionState::Done: return F("DONE");
    case MissionState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

/**
 * Convert a line-following mode enum to a flash-stored label.
 *
 * @param mode FollowMode value selected by the line sensor logic.
 * @return Printable label stored with F() to save SRAM.
 */
const __FlashStringHelper *followModeName(FollowMode mode) {
  switch (mode) {
    case FollowMode::Follow: return F("FOLLOW");
    case FollowMode::HardLeft: return F("HARD_LEFT");
    case FollowMode::HardRight: return F("HARD_RIGHT");
    case FollowMode::SearchLeft: return F("SEARCH_LEFT");
    case FollowMode::SearchRight: return F("SEARCH_RIGHT");
    case FollowMode::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

/**
 * Convert a scripted turn direction to a flash-stored label.
 *
 * @param dir Turn direction, left or right.
 * @return "LEFT" or "RIGHT" as a flash-stored string.
 */
const __FlashStringHelper *turnName(TurnDir dir) {
  return dir == TurnDir::Left ? F("LEFT") : F("RIGHT");
}

/**
 * Convert the selected base route to a flash-stored label.
 *
 * @param route RouteChoice value for the A/right/lower or B/left/upper branch.
 * @return Printable route label for diagnostics.
 */
const __FlashStringHelper *routeName(RouteChoice route) {
  return route == RouteChoice::BaseB_Top ? F("B_LEFT") : F("A_RIGHT");
}

/**
 * Clear the PD controller's stored error terms.
 *
 * This is called after deliberate turns and mission restarts so the derivative
 * and integral terms do not carry stale values into the next line segment.
 */
void resetLineController() {
  lineIntegral = 0.0f;
  lastLineError = 0;
}

/**
 * Check whether a line-follow mode is an aggressive hard-turn correction.
 *
 * @param mode Current line-follow mode.
 * @return true for hard-left or hard-right modes.
 */
bool isHardLineMode(FollowMode mode) {
  return mode == FollowMode::HardLeft || mode == FollowMode::HardRight;
}

/**
 * Start the short ignore window after a scripted 90 degree turn.
 *
 * This prevents an imperfect 90 degree turn from being immediately interpreted
 * as the next hard-left/right route event.
 */
void beginPostTurnHardIgnore() {
  postTurnHardIgnoreActive = true;
  postTurnHardIgnoreStartMs = millis();
  postTurnHardReleaseCount = 0;
}

/**
 * Measure travel from the current route segment start.
 *
 * Before the first turn, encoders are reset at mission start, so this is the
 * distance from start toward the first T.
 *
 * @return Average wheel travel in millimeters.
 */
float currentRouteSegmentTravelMm() {
  const long leftAbs = absLong(getLeftCount());
  const long rightAbs = absLong(getRightCount());
  const long averageCounts = (leftAbs + rightAbs) / 2;
  return averageCounts / (kEncoderCountsPerMm * kDistanceCalibration);
}

/**
 * Update the post-turn ignore state from the latest line reading.
 *
 * @param line Latest QTR line reading.
 */
void updatePostTurnHardIgnore(const LineReading &line) {
  if (!postTurnHardIgnoreActive) return;

  if (millis() - postTurnHardIgnoreStartMs >= kPostTurnHardIgnoreMs) {
    postTurnHardIgnoreActive = false;
    postTurnHardReleaseCount = 0;
    return;
  }

  if (isHardLineMode(line.mode)) {
    postTurnHardReleaseCount = 0;
    return;
  }

  if (line.detected && centerHasLine()) {
    if (postTurnHardReleaseCount < 255) postTurnHardReleaseCount++;
    if (postTurnHardReleaseCount >= kPostTurnHardReleaseFrames) {
      postTurnHardIgnoreActive = false;
      postTurnHardReleaseCount = 0;
    }
  } else {
    postTurnHardReleaseCount = 0;
  }
}

/**
 * Build a softened line reading while post-turn hard modes are being ignored.
 *
 * @param line Original QTR line reading.
 * @return Original reading, or a copy forced into normal follow mode.
 */
LineReading softenedPostTurnLine(LineReading line) {
  if (postTurnHardIgnoreActive && isHardLineMode(line.mode)) {
    line.mode = FollowMode::Follow;
    line.error = constrain(line.error, -kPostTurnSoftErrorClamp, kPostTurnSoftErrorClamp);
  }
  return line;
}

/**
 * Enter a new mission state and timestamp the transition.
 *
 * @param newState State that should become active immediately.
 */
void setState(MissionState newState) {
  missionState = newState;
  stateStartMs = millis();
  Serial.print(F("[STATE] "));
  Serial.println(stateName(newState));
}

// ---------------------------------------------------------------------------
// Motor / encoder
// ---------------------------------------------------------------------------

/**
 * Send signed tank-drive speed commands to the left and right Motoron channels.
 *
 * @param leftSpeed Requested left motor command before sign correction.
 * @param rightSpeed Requested right motor command before sign correction.
 */
void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

/**
 * Stop both drive motors.
 *
 * This wraps setTank(0, 0) so all stop paths use the same Motoron interface.
 */
void stopMotors() {
  setTank(0, 0);
}

/**
 * Interrupt service routine for the left quadrature encoder A channel.
 *
 * The ISR reads both encoder channels and increments or decrements leftCount
 * based on their phase relationship.
 */
void leftEncoderIsr() {
  const bool a = digitalRead(kLeftEncoderAPin);
  const bool b = digitalRead(kLeftEncoderBPin);
  leftCount += (a == b) ? 1 : -1;
}

/**
 * Interrupt service routine for the right quadrature encoder A channel.
 *
 * The ISR reads both encoder channels and increments or decrements rightCount
 * based on their phase relationship.
 */
void rightEncoderIsr() {
  const bool a = digitalRead(kRightEncoderAPin);
  const bool b = digitalRead(kRightEncoderBPin);
  rightCount += (a == b) ? 1 : -1;
}

/**
 * Read the left encoder count atomically.
 *
 * @return Snapshot of leftCount with interrupts briefly disabled.
 */
long getLeftCount() {
  noInterrupts();
  const long count = leftCount;
  interrupts();
  return count;
}

/**
 * Read the right encoder count atomically.
 *
 * @return Snapshot of rightCount with interrupts briefly disabled.
 */
long getRightCount() {
  noInterrupts();
  const long count = rightCount;
  interrupts();
  return count;
}

/**
 * Reset both encoder counters to zero atomically.
 *
 * This is used before distance drives and turn maneuvers.
 */
void resetEncoders() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

/**
 * Convert a straight-line travel distance to average encoder counts.
 *
 * @param distanceMm Signed distance in millimeters; sign is ignored here.
 * @return Positive encoder-count target after distance calibration.
 */
long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

/**
 * Convert an in-place turn angle to per-side encoder counts.
 *
 * @param degrees Signed turn angle in degrees; sign is ignored here.
 * @return Positive encoder-count target after turn calibration.
 */
long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

/**
 * Initialize the Motoron motor controller on Wire1.
 *
 * The function configures the I2C bus, clears reset state, disables CRC for
 * simple commands, removes acceleration limits, and stops the motors.
 */
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
  Serial.println(F("[INIT] Motoron ready."));
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------

/**
 * Print available Serial Monitor commands.
 *
 * This is a human-facing diagnostic helper and does not change robot state.
 */
void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  start | stop | resume | show | line"));
  Serial.println(F("  route a | route b | route right | route left"));
  Serial.println(F("Notes:"));
  Serial.println(F("  D32 kill button: press in IDLE/STOPPED to start; press while moving to stop."));
  Serial.println(F("  Route B/left turn script:  L R R L"));
  Serial.println(F("  Route A/right turn script: R L L R"));
}

/**
 * Print the current mission, route, RFID, and stop status.
 *
 * This is intended for quick field debugging from the Serial Monitor.
 */
void printSettings() {
  Serial.print(F("[SETTINGS] state="));
  Serial.print(stateName(missionState));
  Serial.print(F(" route="));
  Serial.print(routeName(routeChoice));
  Serial.print(F(" turnIndex="));
  Serial.print(routeTurnIndex);
  Serial.print(F("/4 rfid="));
  Serial.print(rfidHandled ? F("DONE") : F("WAIT"));
  Serial.print(F(" uid="));
  Serial.print(lastUid.length() > 0 ? lastUid : String("none"));
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
}

/**
 * Reset mission progress and begin following the selected route from start.
 *
 * It clears RFID state, route-turn index, event debouncing, and PD memory.
 */
void restartMission() {
  serialStopped = false;
  rfidHandled = false;
  lastUid = "";
  routeTurnIndex = 0;
  eventStableCount = 0;
  postTurnHardIgnoreActive = false;
  postTurnHardReleaseCount = 0;
  lastEventMs = millis();
  resetEncoders();
  resetLineController();
  setState(MissionState::FollowRoute);
}

/**
 * Handle the kill switch as a start/stop button.
 *
 * Pressing the button while Idle/Stopped/Done restarts the mission from the
 * beginning. Pressing it during motion stops the motors and enters Stopped.
 *
 * @return true when the button press requested a stop.
 */
bool handleKillButtonEvent() {
  if (!killButtonPressedEvent()) {
    return false;
  }

  if (missionState == MissionState::Idle ||
      missionState == MissionState::Stopped ||
      missionState == MissionState::Done) {
    Serial.println(F("[KILL] button press -> START"));
    restartMission();
    return false;
  }

  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.println(F("[KILL] button press -> STOP"));
  return true;
}

/**
 * Parse and execute one complete Serial Monitor command line.
 *
 * @param line Raw command text without the trailing newline. The command is
 *             trimmed and compared case-insensitively.
 */
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

  if (lower == "start" || lower == "restart") {
    restartMission();
    return;
  }

  if (lower == "resume") {
    serialStopped = false;
    resetLineController();
    if (missionState == MissionState::Stopped || missionState == MissionState::Idle) {
      setState(MissionState::FollowRoute);
    }
    Serial.println(F("[SERIAL] resumed."));
    return;
  }

  if (lower == "stop") {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    Serial.println(F("[SERIAL] stopped."));
    return;
  }

  if (lower == "line") {
    LineReading lineReading;
    lineReading = readLine();
    Serial.print(F("[LINE TEST] mode="));
    Serial.print(followModeName(lineReading.mode));
    Serial.print(F(" detected="));
    Serial.print(lineReading.detected ? F("YES") : F("NO"));
    Serial.print(F(" active="));
    Serial.print(lineReading.activeCount);
    Serial.print(F(" pos="));
    Serial.print(lineReading.position);
    Serial.print(F(" err="));
    Serial.println(lineReading.error);
    return;
  }

  if (lower.startsWith("route ")) {
    const String value = lower.substring(6);
    if (value == "a" || value == "right" || value == "bottom") {
      routeChoice = RouteChoice::BaseA_Bottom;
    } else if (value == "b" || value == "left" || value == "top") {
      routeChoice = RouteChoice::BaseB_Top;
    } else {
      Serial.println(F("[SERIAL] use: route a/right OR route b/left"));
      return;
    }
    Serial.print(F("[SERIAL] route set to "));
    Serial.println(routeName(routeChoice));
    routeTurnIndex = 0;
    eventStableCount = 0;
    postTurnHardIgnoreActive = false;
    postTurnHardReleaseCount = 0;
    return;
  }

  Serial.print(F("[SERIAL] unknown command: "));
  Serial.println(line);
}

/**
 * Accumulate Serial bytes and dispatch complete newline-terminated commands.
 *
 * The small static buffer avoids blocking the control loop while still letting
 * the user stop, restart, inspect sensors, or switch route choice.
 */
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
    if (input.length() < 90) input += c;
  }
}

// ---------------------------------------------------------------------------
// IMU / turning
// ---------------------------------------------------------------------------

/**
 * Read the IMU gyroscope and integrate yaw angle.
 *
 * Implementation notes:
 * - Uses the calibrated Z-axis gyro bias.
 * - Integrates angular velocity over elapsed microseconds.
 * - Rejects unusually large time gaps so paused code does not create yaw jumps.
 *
 * @return true when IMU data was read, false when the IMU is unavailable.
 */
bool updateImu() {
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
 * Reset the relative yaw estimate used for an in-place turn.
 *
 * This sets yaw and angular rate to zero and restarts the integration timer.
 */
void resetYaw() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

/**
 * Command an in-place turn using equal and opposite motor speeds.
 *
 * @param signedTurnSpeed Positive values use the configured left-turn
 *                        convention; negative values turn the other way.
 */
void setTurnCommand(int signedTurnSpeed) {
  const int command = clampMotorSpeed(signedTurnSpeed) * kTurnCommandSign;
  setTank(-command, command);
}

/**
 * Print periodic telemetry while executing a turn.
 *
 * @param label Prefix identifying the turn controller.
 * @param targetDeg Desired relative turn angle in degrees.
 * @param errorDeg Remaining angle error in degrees.
 * @param command Signed turn command currently sent to the motors.
 * @param encoderTarget Fallback encoder target for comparison/debugging.
 */
void printTurnStatus(const char *label, float targetDeg, float errorDeg, int command, long encoderTarget) {
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
 * Turn in place using only encoder counts.
 *
 * This fallback is used when the IMU is not detected. It estimates wheel travel
 * needed for the requested angle and spins the wheels until the average encoder
 * count reaches that target.
 *
 * @param degrees Signed turn angle in degrees; positive follows left-turn convention.
 * @param speed Absolute motor speed to use for the fallback turn.
 */
void encoderOnlyTurnFallback(float degrees, int speed) {
  const long targetCounts = turnDegreesToEncoderCounts(degrees);
  const int direction = degrees >= 0.0f ? 1 : -1;

  resetEncoders();
  const uint32_t start = millis();
  while (true) {
    handleSerialCommands();
    if (serialStopped || handleKillButtonEvent()) break;

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
 * Turn in place by a relative angle using IMU yaw feedback.
 *
 * The controller resets yaw to zero, then applies a proportional turn command
 * until the yaw error and gyro rate are both within tolerance. If the IMU is
 * unavailable, it delegates to encoderOnlyTurnFallback().
 *
 * @param targetDeg Signed target angle in degrees; positive is the configured left turn.
 * @return true if the turn routine completed, false if stopped or killed.
 */
bool turnDegreesImu(float targetDeg) {
  if (!imuOk) {
    encoderOnlyTurnFallback(targetDeg, kTurnMaxSpeed);
    return !serialStopped;
  }

  const long encoderTarget = turnDegreesToEncoderCounts(targetDeg);
  Serial.print(F("[TURN] IMU targetDeg="));
  Serial.print(targetDeg, 1);
  Serial.print(F(" encoderTarget="));
  Serial.println(encoderTarget);

  resetEncoders();
  resetYaw();
  lastTurnPrintMs = 0;
  const uint32_t start = millis();

  while (true) {
    handleSerialCommands();
    if (serialStopped || handleKillButtonEvent()) {
      stopMotors();
      return false;
    }

    updateImu();
    const float errorDeg = targetDeg - yawDeg;
    if (absFloat(errorDeg) <= kTurnToleranceDeg && absFloat(gyroZDegPerSec) <= kGyroStopRateDps) {
      Serial.println(F("[TURN] IMU target reached."));
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

/**
 * Initialize and calibrate the ICM20948 IMU.
 *
 * The robot must stay still during bias calibration. If initialization fails,
 * the sketch continues with encoder-only turn fallback.
 *
 * @return true when the IMU is found and calibrated, false otherwise.
 */
bool initializeImu() {
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

  Serial.println(F("[IMU] found. Keep robot still for gyro bias calibration."));
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
  Serial.print(F("[IMU] gyro Z bias = "));
  Serial.print(gyroZBiasDps, 4);
  Serial.println(F(" deg/s"));
  return true;
}

// ---------------------------------------------------------------------------
// Encoder distance drive
// ---------------------------------------------------------------------------

/**
 * Drive straight for a requested distance using encoder feedback.
 *
 * The function resets encoders, drives both motors forward or backward, and
 * uses a small encoder-difference correction to keep the two sides balanced.
 *
 * @param distanceMm Signed distance in millimeters; negative drives backward.
 * @param speed Absolute base speed command for the drive segment.
 * @return true if the segment completed or timed out normally, false if stopped or killed.
 */
bool driveDistanceMm(float distanceMm, int speed) {
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
    if (serialStopped || handleKillButtonEvent()) {
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

// ---------------------------------------------------------------------------
// QTR line sensor / PD follower
// ---------------------------------------------------------------------------

/**
 * Initialize the 9-channel QTR line sensor pins and saved calibration values.
 *
 * The QTR CTRL pins are enabled, sensor pins are set as inputs, and raw
 * min/max values from previous calibration are copied into qtrMin/qtrMax.
 */
void initializeQtr() {
  pinMode(kQtrCtrlOddPin, OUTPUT);
  pinMode(kQtrCtrlEvenPin, OUTPUT);
  digitalWrite(kQtrCtrlOddPin, HIGH);
  digitalWrite(kQtrCtrlEvenPin, HIGH);
  for (uint8_t i = 0; i < 9; i++) {
    pinMode(kQtrPins[i], INPUT);
    qtrMin[i] = kSavedQtrMin[i];
    qtrMax[i] = kSavedQtrMax[i];
  }
  Serial.println(F("[INIT] QTR ready with saved calibration."));
}

/**
 * Read raw RC timing values from the 9 QTR sensors.
 *
 * Implementation method:
 * - Charge each RC sensor node by driving the pin HIGH.
 * - Switch pins to input and measure discharge time until LOW.
 * - Darker surfaces generally produce longer timings on this hardware.
 */
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

/**
 * Convert raw QTR timing values to normalized 0..1000 reflectance values.
 *
 * Values are scaled using qtrMin/qtrMax. Sensors with too little calibration
 * span are treated as white/no-line to avoid divide-by-noise behavior.
 */
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

/**
 * Count how many normalized QTR sensors are above a line threshold.
 *
 * @param threshold Normalized black-line threshold in the 0..1000 range.
 * @return Number of active sensors, from 0 to 9.
 */
uint8_t activeSensorCount(uint16_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

/**
 * Check whether the line is under the center part of the sensor array.
 *
 * @return true when any of sensors 3, 4, or 5 is above kLineThreshold.
 */
bool centerHasLine() {
  return qtrNorm[3] >= kLineThreshold || qtrNorm[4] >= kLineThreshold || qtrNorm[5] >= kLineThreshold;
}

/**
 * Check whether the far-left sensors see a strong black line.
 *
 * @return true when sensor 0 or 1 exceeds kStrongLineThreshold.
 */
bool leftOuterHasStrongLine() {
  return qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
}

/**
 * Check whether the far-right sensors see a strong black line.
 *
 * @return true when sensor 7 or 8 exceeds kStrongLineThreshold.
 */
bool rightOuterHasStrongLine() {
  return qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;
}

/**
 * Compute the weighted line position from normalized QTR readings.
 *
 * Each active sensor contributes weight at positions 0, 1000, ..., 8000.
 * The centered line target is 4000. If the line is lost, the function returns
 * the last-seen side so search mode can turn in the likely direction.
 *
 * @param detectedOut Output flag set true when any sensor is above threshold.
 * @return Weighted line position, normally 0..8000, centered at 4000.
 */
int computeLinePosition(bool *detectedOut) {
  normalizeQtrValues();

  uint32_t weighted = 0;
  uint32_t sum = 0;
  bool detected = false;

  for (uint8_t i = 0; i < 9; i++) {
    const uint16_t weight = qtrNorm[i] >= kLineThreshold ? qtrNorm[i] : 0;
    if (weight > 0) {
      detected = true;
      weighted += static_cast<uint32_t>(weight) * (i * 1000);
      sum += weight;
    }
  }

  *detectedOut = detected;
  if (sum == 0) {
    return 4000 + lastSeenLineError;
  }

  const int position = static_cast<int>(weighted / sum);
  lastSeenLineError = position - 4000;
  return position;
}

/**
 * Classify the current line reading into a control mode.
 *
 * Ordinary centered readings use Follow. Lost-line readings use SearchLeft or
 * SearchRight based on the last seen error. Large errors or strong outer-edge
 * hits become HardLeft/HardRight, which the route logic can interpret as
 * expected sharp-turn events.
 *
 * @param line Current line reading with position, error, and detected flag.
 * @return FollowMode for the current control step.
 */
FollowMode chooseFollowMode(const LineReading &line) {
  if (serialStopped) return FollowMode::Stopped;

  if (!line.detected) {
    lineIntegral = 0.0f;
    return lastSeenLineError < 0 ? FollowMode::SearchLeft : FollowMode::SearchRight;
  }

  if (line.error < -kHardTurnError || (leftOuterHasStrongLine() && line.error < -kCenterRecoverError)) {
    lineIntegral = 0.0f;
    return FollowMode::HardLeft;
  }

  if (line.error > kHardTurnError || (rightOuterHasStrongLine() && line.error > kCenterRecoverError)) {
    lineIntegral = 0.0f;
    return FollowMode::HardRight;
  }

  if (centerHasLine()) return FollowMode::Follow;
  return line.error < 0 ? FollowMode::HardLeft : FollowMode::HardRight;
}

/**
 * Perform a complete line-sensor update.
 *
 * This reads raw QTR values, normalizes them, computes line position/error,
 * counts active sensors, selects a follow mode, and copies readings into a
 * LineReading struct for control and logging.
 *
 * @return Fully populated LineReading snapshot.
 */
LineReading readLine() {
  LineReading line;
  readQtrRcArray();

  bool detected = false;
  line.position = computeLinePosition(&detected);
  line.detected = detected;
  line.error = (line.position - 4000) * kLineErrorSign;
  line.activeCount = activeSensorCount(kLineThreshold);
  line.mode = chooseFollowMode(line);

  for (uint8_t i = 0; i < 9; i++) {
    line.raw[i] = qtrRaw[i];
    line.norm[i] = qtrNorm[i];
  }
  return line;
}

/**
 * Convert a line reading into left/right motor commands.
 *
 * Search and hard-turn modes produce fixed spin commands. Follow mode uses PD
 * control on line.error, with optional integral storage kept clamped.
 *
 * @param line Current line reading and selected follow mode.
 * @return MotorCommand with signed left and right speed commands.
 */
MotorCommand computeLineMotorCommand(const LineReading &line) {
  MotorCommand cmd;

  switch (line.mode) {
    case FollowMode::Stopped:
      cmd.left = 0;
      cmd.right = 0;
      return cmd;
    case FollowMode::SearchLeft:
      cmd.left = -kLineSearchTurnSpeed;
      cmd.right = kLineSearchTurnSpeed;
      return cmd;
    case FollowMode::SearchRight:
      cmd.left = kLineSearchTurnSpeed;
      cmd.right = -kLineSearchTurnSpeed;
      return cmd;
    case FollowMode::HardLeft:
      cmd.left = -kLineHardTurnSpeed;
      cmd.right = kLineHardTurnSpeed;
      return cmd;
    case FollowMode::HardRight:
      cmd.left = kLineHardTurnSpeed;
      cmd.right = -kLineHardTurnSpeed;
      return cmd;
    case FollowMode::Follow:
    default:
      break;
  }

  lineIntegral += line.error / 1000.0f;
  lineIntegral = constrain(lineIntegral, -kLineIntegralClamp, kLineIntegralClamp);

  const int derivative = line.error - lastLineError;
  lastLineError = line.error;

  int correction = static_cast<int>(kLineKp * line.error + kLineKi * lineIntegral + kLineKd * derivative);
  correction = constrain(correction, -kLineMaxCorrection, kLineMaxCorrection);

  cmd.left = kLineBaseSpeed + correction;
  cmd.right = kLineBaseSpeed - correction;
  return cmd;
}

/**
 * Apply one line-following motor command and print throttled diagnostics.
 *
 * @param line Current line reading to convert into motor output.
 * @param label Flash-stored log prefix such as "[LINE]" or "[DOOR]".
 */
void applyLineCommand(const LineReading &line, const __FlashStringHelper *label) {
  const MotorCommand cmd = computeLineMotorCommand(line);

  if (line.mode == FollowMode::Stopped) {
    stopMotors();
  } else {
    setTank(cmd.left, cmd.right);
  }

  if (millis() - lastLinePrintMs >= kLinePrintIntervalMs) {
    lastLinePrintMs = millis();
    Serial.print(label);
    Serial.print(F(" state="));
    Serial.print(stateName(missionState));
    Serial.print(F(" route="));
    Serial.print(routeName(routeChoice));
    Serial.print(F(" turn="));
    Serial.print(routeTurnIndex);
    Serial.print(F("/4 mode="));
    Serial.print(followModeName(line.mode));
    Serial.print(F(" active="));
    Serial.print(line.activeCount);
    Serial.print(F(" err="));
    Serial.print(line.error);
    Serial.print(F(" L="));
    Serial.print(cmd.left);
    Serial.print(F(" R="));
    Serial.print(cmd.right);
    Serial.print(F(" rfid="));
    Serial.println(rfidHandled ? F("DONE") : F("WAIT"));
  }

  updateImu();
  delay(kLoopDelayMs);
}

// ---------------------------------------------------------------------------
// RFID / optional exit request
// ---------------------------------------------------------------------------

/**
 * Initialize the MFRC522-compatible RFID reader on the main I2C bus.
 *
 * The version register is checked so the rest of the sketch can skip polling
 * if the reader is missing or not responding.
 */
void initializeRfid() {
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
 * Convert the last RFID UID read by the MFRC522 library into hex text.
 *
 * @return Uppercase hexadecimal UID string with leading zeroes preserved.
 */
String rfidUidToString() {
  String uid;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(rfid.uid.uidByte[i], HEX);
  }
  uid.toUpperCase();
  return uid;
}

/**
 * Poll the RFID reader for a new card/tag.
 *
 * Polling is rate-limited by kRfidPollIntervalMs to avoid excessive I2C work.
 * When a card is found, the UID is copied to uidOut and the card is halted.
 *
 * @param uidOut Output pointer that receives the UID string on success.
 * @return true when a new RFID tag was read, false otherwise.
 */
bool pollRfid(String *uidOut) {
  if (!rfidOk) return false;
  if (millis() - lastRfidPollMs < kRfidPollIntervalMs) return false;
  lastRfidPollMs = millis();

  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    *uidOut = rfidUidToString();
    rfid.PICC_HaltA();
    return true;
  }
  return false;
}

#if USE_WIFI_EXIT_REQUEST
/**
 * Handle inbound MiniMessenger messages from the challenge server.
 *
 * @param metadata MiniMessenger metadata for the message; currently unused.
 * @param payload Raw message bytes.
 * @param length Number of bytes available in payload.
 */
void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
  char msg[256];
  const size_t copyLen = length < sizeof(msg) - 1 ? length : sizeof(msg) - 1;
  memcpy(msg, payload, copyLen);
  msg[copyLen] = '\0';
  Serial.print(F("[WIFI RX] "));
  Serial.println(msg);
}

/**
 * Start MiniMessenger WiFi support for optional exit-clearance requests.
 *
 * Credentials and broker details come from secrets.h when
 * USE_WIFI_EXIT_REQUEST is enabled.
 */
void initializeWifi() {
  messenger.onMessage(onWifiMessage);
  messenger.begin(WIFI_SSID, WIFI_PASSWORD, BROKER_HOST, BROKER_PORT, GROUP_ID, kBoardId);
  Serial.println(F("[WIFI] MiniMessenger started."));
}

/**
 * Service MiniMessenger and periodically register this robot with the server.
 *
 * This must be called frequently from motion loops so networking stays alive
 * while the robot is following lines or performing maneuvers.
 */
void updateWifi() {
  messenger.loop();
  if (messenger.isConnected() && (lastRegisterMs == 0 || millis() - lastRegisterMs >= kRegisterIntervalMs)) {
    lastRegisterMs = millis();
    char reg[128];
    snprintf(reg, sizeof(reg), "type=register team_id=%s board_id=%s", GROUP_ID, kBoardId);
    messenger.sendToBoard("server", reg);
    Serial.print(F("[WIFI REGISTER] "));
    Serial.println(reg);
  }
}
#else
/**
 * No-op WiFi initializer when USE_WIFI_EXIT_REQUEST is disabled.
 */
void initializeWifi() {}

/**
 * No-op WiFi service function when USE_WIFI_EXIT_REQUEST is disabled.
 */
void updateWifi() {}
#endif

/**
 * Send or log a base-exit clearance request after stopping over the RFID tag.
 *
 * With WiFi enabled, this sends a best-effort MiniMessenger message to the
 * server. With WiFi disabled, it prints a clear diagnostic so the driver knows
 * the robot has aligned over the tag and the door can be opened manually.
 *
 * @param uid UID string from the base-exit RFID tag.
 */
void sendExitClearanceRequest(const String &uid) {
#if USE_WIFI_EXIT_REQUEST
  if (!messenger.isConnected()) {
    Serial.println(F("[EXIT API] WiFi not connected; cannot send request yet."));
    return;
  }
  char msg[180];
  snprintf(
      msg,
      sizeof(msg),
      "type=base_exit_request team_id=%s board_id=%s uid=%s",
      GROUP_ID,
      kBoardId,
      uid.c_str());
  messenger.sendToBoard("server", msg);
  Serial.print(F("[EXIT API] sent "));
  Serial.println(msg);
#else
  Serial.print(F("[EXIT API] hook not enabled. RFID UID="));
  Serial.print(uid);
  Serial.println(F(" stopped over tag; open door manually or enable USE_WIFI_EXIT_REQUEST."));
#endif
}

/**
 * Check for the base-exit RFID tag while the robot is moving.
 *
 * The tag is handled only once. On detection, the robot stops, stores the UID,
 * sends/logs the exit request, and enters StopOverRfid state.
 *
 * @return true if an RFID tag was newly handled during this call.
 */
bool checkRfidDuringMotion() {
  if (rfidHandled) return false;

  String uid;
  if (!pollRfid(&uid)) return false;

  lastUid = uid;
  rfidHandled = true;
  stopMotors();

  Serial.print(F("[RFID] Base exit tag detected UID="));
  Serial.println(lastUid);
  sendExitClearanceRequest(lastUid);
  setState(MissionState::StopOverRfid);
  return true;
}

// ---------------------------------------------------------------------------
// Route-scripted intersection handling
// ---------------------------------------------------------------------------

/**
 * Look up the scripted turn direction for the selected base route.
 *
 * @param index Zero-based route event index, expected range 0..3.
 * @return TurnDir for the A/right/lower or B/left/upper route script.
 */
TurnDir routeTurnAt(uint8_t index) {
  if (routeChoice == RouteChoice::BaseB_Top) {
    // B left/upper branch: L, R, R, L
    if (index == 0 || index == 3) return TurnDir::Left;
    return TurnDir::Right;
  }

  // A right/lower branch: R, L, L, R
  if (index == 0 || index == 3) return TurnDir::Right;
  return TurnDir::Left;
}

/**
 * Convert a route turn direction to an IMU target angle.
 *
 * @param dir Turn direction to execute.
 * @return +90 degrees for left, -90 degrees for right.
 */
float degreesForTurn(TurnDir dir) {
  return dir == TurnDir::Left ? 90.0f : -90.0f;
}

/**
 * Detect the first T-shaped bifurcation.
 *
 * The first branch is treated differently from later corners: it expects a
 * near-all-black sensor pattern with both outer edges and the center active.
 *
 * @param line Current line reading.
 * @return true when the first T intersection pattern is present.
 */
bool firstTDetected(const LineReading &line) {
  if (!line.detected) return false;
  if (currentRouteSegmentTravelMm() < kFirstTMinTravelMm) return false;
  if (line.activeCount < kFirstTMinActiveSensors) return false;
  if (!centerHasLine()) return false;
  if (abs(line.error) > kFirstTMaxCenterError) return false;

  const bool leftEdgeStrong =
      qtrNorm[0] >= kFirstTEdgeStrongThreshold || qtrNorm[1] >= kFirstTEdgeStrongThreshold;
  const bool rightEdgeStrong =
      qtrNorm[7] >= kFirstTEdgeStrongThreshold || qtrNorm[8] >= kFirstTEdgeStrongThreshold;
  if (!leftEdgeStrong || !rightEdgeStrong) return false;

  uint16_t totalStrength = 0;
  for (uint8_t i = 0; i < 9; i++) {
    totalStrength += qtrNorm[i];
  }
  return totalStrength >= kFirstTMinTotalStrength;
}

/**
 * Detect a later route event in the expected turn direction.
 *
 * This intentionally avoids using black-sensor count alone. A route event is
 * accepted only when the current line mode or outer-edge evidence agrees with
 * the next scripted direction.
 *
 * @param line Current line reading.
 * @param expected Next scripted turn direction.
 * @return true when the reading matches the expected sharp-turn evidence.
 */
bool expectedSharpTurnDetected(const LineReading &line, TurnDir expected) {
  if (!line.detected) return false;

  if (expected == TurnDir::Left) {
    return line.mode == FollowMode::HardLeft || (leftOuterHasStrongLine() && line.error < -kCenterRecoverError);
  }

  return line.mode == FollowMode::HardRight || (rightOuterHasStrongLine() && line.error > kCenterRecoverError);
}

/**
 * Debounce and validate whether the next scripted route event should fire.
 *
 * The function applies a cooldown after each committed turn, checks the correct
 * detector for the current route index, and requires several stable frames.
 *
 * @param line Current line reading.
 * @return true when the route logic should perform the next committed turn.
 */
bool routeEventReady(const LineReading &line) {
  if (routeTurnIndex >= 4) return false;
  if (millis() - lastEventMs < kEventCooldownMs) return false;
  if (postTurnHardIgnoreActive && isHardLineMode(line.mode)) {
    eventStableCount = 0;
    return false;
  }

  const TurnDir expected = routeTurnAt(routeTurnIndex);
  const bool firstRouteTurn = routeTurnIndex == 0;
  const bool eventNow = firstRouteTurn ? firstTDetected(line) : expectedSharpTurnDetected(line, expected);

  if (!eventNow) {
    eventStableCount = 0;
    return false;
  }

  if (eventStableCount < 255) eventStableCount++;
  const uint8_t requiredFrames = firstRouteTurn ? kFirstTStableFrames : kSharpTurnStableFrames;
  return eventStableCount >= requiredFrames;
}

/**
 * Re-center the robot on the line after a committed 90-degree turn.
 *
 * The robot slowly turns in the same direction until the center sensors see the
 * line for several stable frames. If this times out, the function resumes PD
 * anyway so the normal search behavior can recover.
 *
 * @param dir Direction of the turn that was just performed.
 * @return false only when stopped or killed; timeout still returns true.
 */
bool reacquireLineAfterTurn(TurnDir dir) {
  const uint32_t start = millis();
  uint8_t stable = 0;
  const int command = dir == TurnDir::Left ? kReacquireTurnSpeed : -kReacquireTurnSpeed;

  while (millis() - start < kReacquireTimeoutMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleKillButtonEvent()) {
      stopMotors();
      return false;
    }

    const LineReading line = readLine();
    if (line.detected && centerHasLine()) {
      stable++;
      stopMotors();
      if (stable >= kReacquireStableFrames) {
        Serial.println(F("[REACQUIRE] centered line found."));
        resetLineController();
        return true;
      }
    } else {
      stable = 0;
      setTurnCommand(command);
    }
    updateImu();
    delay(10);
  }

  stopMotors();
  Serial.println(F("[REACQUIRE] timeout; resuming PD/search mode."));
  resetLineController();
  return true;
}

/**
 * Execute one committed route turn from the line-maze script.
 *
 * Sequence:
 * 1. Stop briefly after detecting the route event.
 * 2. Drive forward by the tuned sensor-to-turn-axis offset.
 * 3. Turn 90 degrees using IMU or encoder fallback.
 * 4. Reacquire the line and advance the route index.
 *
 * @return true if the maneuver completed, false if stopped or killed.
 */
bool performRouteTurn() {
  if (routeTurnIndex >= 4) return true;

  const TurnDir dir = routeTurnAt(routeTurnIndex);
  const bool firstT = routeTurnIndex == 0;
  const float advanceMm = firstT ? kFirstTAdvanceMm : kSharpTurnAdvanceMm;

  stopMotors();
  delay(80);

  Serial.print(F("[ROUTE] event turnIndex="));
  Serial.print(routeTurnIndex);
  Serial.print(F(" dir="));
  Serial.print(turnName(dir));
  Serial.print(F(" trigger="));
  Serial.println(firstT ? F("FIRST_T") : F("EXPECTED_SHARP"));

  if (!driveDistanceMm(advanceMm, kAdvanceSpeed)) {
    return false;
  }

  delay(60);

  if (!turnDegreesImu(degreesForTurn(dir))) {
    return false;
  }

  delay(80);

  if (!reacquireLineAfterTurn(dir)) {
    return false;
  }

  routeTurnIndex++;
  eventStableCount = 0;
  lastEventMs = millis();
  beginPostTurnHardIgnore();

  if (routeTurnIndex >= 4) {
    Serial.println(F("[ROUTE] all scripted turns complete; following line to door."));
    setState(MissionState::FollowToDoor);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Mission update
// ---------------------------------------------------------------------------

/**
 * Run one control-loop step while navigating the scripted base route.
 *
 * The function checks RFID first, reads the line, performs a committed route
 * turn if the next event is ready, otherwise applies ordinary PD line following.
 */
void runRouteFollowStep() {
  if (checkRfidDuringMotion()) return;

  const LineReading line = readLine();
  if (routeEventReady(line)) {
    if (!performRouteTurn()) {
      serialStopped = true;
      stopMotors();
      setState(MissionState::Stopped);
    }
    return;
  }

  const LineReading followLine = softenedPostTurnLine(line);
  applyLineCommand(followLine, F("[LINE]"));
  updatePostTurnHardIgnore(line);
}

/**
 * Run one control-loop step after all route turns are complete.
 *
 * The robot keeps following the outgoing line toward the door, still watching
 * for RFID if it was missed earlier, and optionally stops after kDoorApproachMs.
 */
void runDoorFollowStep() {
  checkRfidDuringMotion();
  if (missionState == MissionState::StopOverRfid) return;

  const LineReading line = readLine();
  const LineReading followLine = softenedPostTurnLine(line);
  applyLineCommand(followLine, F("[DOOR]"));
  updatePostTurnHardIgnore(line);

  if (kStopAfterDoorApproach && millis() - stateStartMs >= kDoorApproachMs) {
    stopMotors();
    if (!rfidHandled) {
      Serial.println(F("[WARN] Door approach complete, but RFID was not detected."));
    }
    Serial.println(F("[DONE] Reached door approach timeout."));
    setState(MissionState::Done);
  }
}

/**
 * Top-level nonblocking mission state machine.
 *
 * This services serial/WiFi, checks the kill switch, then dispatches behavior
 * for Idle, FollowRoute, StopOverRfid, FollowToDoor, Done, or Stopped states.
 */
void updateMission() {
  handleSerialCommands();
  updateWifi();

  if (handleKillButtonEvent()) {
    delay(50);
    return;
  }

  switch (missionState) {
    case MissionState::Idle:
      stopMotors();
      delay(20);
      break;

    case MissionState::FollowRoute:
      if (serialStopped) {
        stopMotors();
        setState(MissionState::Stopped);
      } else {
        runRouteFollowStep();
      }
      break;

    case MissionState::StopOverRfid:
      stopMotors();
      if (millis() - stateStartMs >= kStopOverRfidMs) {
        Serial.println(F("[RFID] stop complete; continuing toward door."));
        if (routeTurnIndex >= 4) {
          setState(MissionState::FollowToDoor);
        } else {
          setState(MissionState::FollowRoute);
        }
      }
      delay(10);
      break;

    case MissionState::FollowToDoor:
      if (serialStopped) {
        stopMotors();
        setState(MissionState::Stopped);
      } else {
        runDoorFollowStep();
      }
      break;

    case MissionState::Done:
      stopMotors();
      updateImu();
      delay(50);
      break;

    case MissionState::Stopped:
      stopMotors();
      updateImu();
      delay(50);
      break;
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

/**
 * Arduino setup entry point.
 *
 * Initializes Serial, safety input, encoders, I2C buses, QTR, Motoron, RFID,
 * IMU, and optional WiFi, then enters the configured start state.
 */
void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  Serial.println(F("=== Base_Exit_Line_Maze_Test ==="));
  Serial.println(F("Line-maze route script: normal PD + committed intersection turns."));

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
    lastKillReading = digitalRead(kKillPin);
    stableKillReading = lastKillReading;
    lastKillChangeMs = millis();
  }

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, CHANGE);

  Wire.begin();
  Wire1.begin();

  initializeQtr();
  initializeMotoron();
  initializeRfid();
  initializeImu();
  initializeWifi();

  lastEventMs = millis();

  Serial.print(F("[INIT] route="));
  Serial.println(routeName(routeChoice));
  Serial.print(F("[INIT] encoder counts/mm="));
  Serial.println(kEncoderCountsPerMm, 4);
  printHelp();
  printSettings();

  if (kStartOnBoot) {
    setState(MissionState::FollowRoute);
  } else {
    setState(MissionState::Idle);
  }
}

/**
 * Arduino main loop entry point.
 *
 * Delegates each iteration to the mission state machine.
 */
void loop() {
  updateMission();
}
