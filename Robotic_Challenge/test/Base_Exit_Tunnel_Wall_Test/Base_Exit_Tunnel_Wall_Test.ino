#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include <MFRC522_I2C.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Arduino_Modulino.h>

#if __has_include("secrets.h")
#define HAS_TUNNEL_WIFI_SECRETS 1
#else
#define HAS_TUNNEL_WIFI_SECRETS 0
#endif

#ifndef USE_WIFI_AIRLOCK_REQUEST
#define USE_WIFI_AIRLOCK_REQUEST HAS_TUNNEL_WIFI_SECRETS
#endif

#if USE_WIFI_AIRLOCK_REQUEST
#include <MiniMessenger.h>
#include "secrets.h"
#endif

// ---------------------------------------------------------------------------
// Base_Exit_Tunnel_Wall_Test
// Board: Arduino GIGA R1 WiFi
//
// Purpose:
//   Full base-exit and tunnel traversal test:
//   1. Start on the base line.
//   2. Follow the selected A/right/lower or B/left/upper route inside the base.
//   3. Read the base-exit RFID tag and request airlock A.
//   4. Keep following the line until it disappears at the tunnel entry.
//   5. Start tunnel wall following using the initial IMU calibration.
//   6. As soon as the QTR array sees the grid line again, abandon wall
//      following and follow the line to the first grid RFID.
//   7. At every grid RFID: drive 75 mm to center the hole, stop, query the
//      server, plant only if fertile=true and planted=false, then continue.
//   8. After the first grid RFID turn right, then execute the fixed RFID-count
//      route: 2L,3L,1L,2R,1R,2L,1L,3R,1R,3L,1L,3R,1L+openAirlock B.
//   9. After requesting B, follow the line until six no-line frames or 12 s,
//      wall-follow the return tunnel, and abandon wall following as soon as
//      the QTR array sees the base line again.
//
// Hardware:
//   QTR CTRL odd/even -> D2/D3
//   QTR sensors       -> D22-D30, left to right when viewed from robot front
//   Front sonar       -> trig D8, echo D11 through 5V-to-3.3V level shifter
//   Left sonar        -> trig D9, echo D12 through 5V-to-3.3V level shifter
//   Right sonar       -> trig D10, echo D13 through 5V-to-3.3V level shifter
//   RFID              -> Wire / D20 SDA-D21 SCL, address 0x28
//   IMU ICM20948      -> Wire / D20 SDA-D21 SCL, address 0x68
//   Motoron           -> Wire1 / shield SDA1-SCL1, address 0x11
//   Motoron M1/M2     -> left/right motor
//   Left encoder      -> D34/D35
//   Right encoder     -> D36/D37
//   Kill button       -> D32 to GND, INPUT_PULLUP
//   Modulino Pixels   -> Wire / D20 SDA-D21 SCL, address 0x36
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Frequently tuned parameters
// ---------------------------------------------------------------------------

constexpr uint32_t kSerialBaud = 115200;  // Serial Monitor baud rate.
constexpr bool kStartOnBoot = false;  // false = start in IDLE and wait for D32 button or serial "start".

enum class RouteChoice {
  BaseA_Bottom,
  BaseB_Top
};

// Default route: A is the right/lower branch from the first base fork.
constexpr RouteChoice kDefaultRouteChoice = RouteChoice::BaseA_Bottom;  // Route used after reset/start; A is the right/lower branch.
constexpr uint8_t kRouteTurnCount = 4;  // Four committed route turns from start to the tunnel-door line.

// Distance to roll forward after a line event before making a 90 degree turn.
// Tune these for the sensor-to-wheel-axis geometry on your robot.
constexpr float kFirstTAdvanceMm = 60.0f;  // Distance to drive after detecting the first T before turning.
constexpr float kSharpTurnAdvanceMm = 60.0f;  // Distance to drive after later hard-turn events before turning.
constexpr int kRouteAdvanceSpeed = 300;  // Motor speed used for the short pre-turn advance.

// After the first outside RFID is detected, drive this extra distance so the
// robot center is over the spot. Keep 0 if the RFID reader is already centered.
constexpr float kDefaultRfidAlignOffsetMm = 0.0f;  // Extra distance after first outside RFID detection for centering.
constexpr uint32_t kStopOverAirlockTagMs = 800;  // Brief stop after reading the base-exit RFID tag.
constexpr uint8_t kAirlockRfidCheckFromTurnIndex = 2;  // Start checking base-exit RFID after this many route turns.

// Door detection with front sonar. Closed uses the low threshold; open uses
// the high threshold to provide hysteresis and avoid flickering.
constexpr float kDoorClosedThresholdMm = 170.0f;  // Front sonar distance at/below this means the door is closed.
constexpr float kDoorOpenThresholdMm = 320.0f;  // Front sonar distance at/above this means the door is open.
constexpr uint8_t kDoorStableFrames = 3;  // Consecutive sonar frames required before accepting open/closed.
constexpr bool kTreatNoEchoAsOpen = true;  // No echo usually means the door is no longer directly in front.
constexpr uint32_t kDoorPrintIntervalMs = 250;  // Minimum time between front-door sonar debug prints.

// Safety timeouts for long straight sections.
constexpr uint32_t kLineToDoorTimeoutMs = 12000;  // Maximum line-follow time from base route to tunnel entry.
constexpr uint32_t kWallTunnelTimeoutMs = 25000;  // Maximum wall-following time through the tunnel.
constexpr uint32_t kRfidSearchTimeoutMs = 20000;  // Maximum forward-search time after tunnel exit.
constexpr uint8_t kTunnelEntryNoLineFrames = 6;  // Consecutive all-white QTR frames required to declare tunnel entry.
constexpr int kTunnelEntryConfirmSpeed = 260;  // Straight speed used while confirming the base line has ended.

// Forward motion after tunnel exit while searching for the first RFID tag.
constexpr int kPostTunnelSearchSpeed = 300;  // Forward speed after exit door while searching for first RFID.
constexpr int kRfidAlignSpeed = 260;  // Encoder-drive speed for final RFID alignment offset.

// After the tunnel, the robot returns to fixed RFID-count navigation. Every
// RFID node is handled in this order: read tag, drive 75 mm to put the robot
// center over the hole, stop and wait for the server, then plant only if the
// server reports fertile=true and planted=false.
constexpr uint8_t kWallExitLineStableFrames = 2;  // Consecutive line frames needed to leave wall following.
constexpr uint8_t kReturnWallExitLineStableFrames = 1;  // Return trip: one QTR line frame immediately resumes line following.
constexpr float kGridPlantCenterOffsetMm = 90.0f;  // Drive after each grid RFID before querying/planting.
constexpr int kGridPlantOffsetSpeed = 360;  // Encoder-drive speed for the 75 mm RFID-to-hole offset.
constexpr bool kPlantIfNoServerReply = false;  // true only for offline bench tests.
constexpr uint8_t kMaxSeedsToPlant = 5;  // Keep following the route after this, but do not drop more seeds.
constexpr uint32_t kServerReplyTimeoutMs = 900;  // Time to wait for isFertileReply at each RFID node.
constexpr uint32_t kGridRfidCooldownMs = 1300;  // Ignore the same UID briefly after it was handled.
constexpr uint32_t kPostTurnRfidIgnoreMs = 1000;  // Ignore grid RFID reads briefly after each scripted grid turn.

// DS-R005 300 degree positional servo used for seed release.
constexpr uint8_t kServoPin = 33;  // Servo signal pin.
constexpr int kServoMinUs = 500;  // Pulse width for 0 degrees.
constexpr int kServoMaxUs = 2500;  // Pulse width for 300 degrees.
constexpr int kServoMinAngle = 0;  // Servo reset angle.
constexpr int kServoMaxAngle = 300;  // Servo maximum angle.
constexpr int kServoStepAngle = 60;  // Seed-release step.
constexpr uint32_t kServoMoveSettleMs = 600;  // Time to send the new target angle.
constexpr uint32_t kServoHoldAfterDropMs = 1200;  // Hold time after a seed-release step.
constexpr uint32_t kServoFrameUs = 20000;  // 50 Hz servo frame.
constexpr bool kResetServoAtStartup = true;  // Reset servo to 0 degrees when sensors initialize.

// Line-following control copied from the existing line tests, slowed slightly
// for reliable route events.
constexpr int kLineBaseSpeed = 340;  // Forward speed during normal PD line following.
constexpr int kLineMaxCorrection = 560;  // Maximum steering correction added/subtracted from base speed.
constexpr int kLineHardTurnSpeed = 450;  // In-place spin speed for hard-left/hard-right modes.
constexpr int kLineSearchTurnSpeed = 210;  // In-place spin speed when all QTR sensors read white.
constexpr float kLineKp = 0.80f;  // Proportional gain for line error.
constexpr float kLineKi = 0.0f;  // Integral gain; usually keep 0 unless a steady bias appears.
constexpr float kLineKd = 0.08f;  // Derivative gain to damp quick line-error changes.
constexpr uint16_t kLineThreshold = 230;  // Normalized QTR threshold above which a sensor sees black.
constexpr uint16_t kStrongLineThreshold = 650;  // Strong-black threshold for outer-edge hard-turn evidence.
constexpr int kHardTurnError = 2500;  // Error magnitude that forces hard-turn mode.
constexpr int kCenterRecoverError = 900;  // Outer-edge + error threshold for declaring hard turn.
constexpr float kLineIntegralClamp = 120.0f;  // Integral accumulator limit.
constexpr uint32_t kLinePrintIntervalMs = 150;  // Minimum time between line debug prints.
constexpr uint32_t kLineLoopDelayMs = 8;  // Delay at end of each line-following step.

// Intersection confirmation. The first T uses near-all-black detection; later
// events use expected left/right hard-turn evidence.
constexpr uint8_t kFirstTMinActiveSensors = 8;  // Minimum active sensors to accept the first T/bifurcation.
constexpr uint8_t kFirstTStableFrames = 5;  // Consecutive first-T frames required before committing the first turn.
constexpr float kFirstTMinTravelMm = 80.0f;  // Minimum travel from start before the first T can be accepted.
constexpr uint16_t kFirstTEdgeStrongThreshold = 650;  // Strong-black threshold required on both outer edges for first T.
constexpr uint16_t kFirstTMinTotalStrength = 5600;  // Minimum summed 9-sensor black strength for first-T confidence.
constexpr int kFirstTMaxCenterError = 900;  // Maximum centered line error allowed when accepting the first T.
constexpr uint8_t kSharpTurnStableFrames = 1;  // Later expected hard turns commit immediately when detected.
constexpr uint32_t kEventCooldownMs = 500;  // Ignore all route events briefly after a committed turn.
constexpr int kReacquireTurnSpeed = 150;  // Slow spin speed used to find the line after a turn.
constexpr uint32_t kReacquireTimeoutMs = 1400;  // Maximum time to actively reacquire line after turning.
constexpr uint8_t kReacquireStableFrames = 3;  // Center-line frames required to finish reacquisition.
constexpr uint32_t kPostTurnHardIgnoreMs = 1100;  // Ignore hard-left/right readings briefly after each scripted turn.
constexpr uint8_t kPostTurnHardReleaseFrames = 4;  // Centered frames needed before re-enabling hard-turn route events.
constexpr int kPostTurnSoftErrorClamp = 650;  // Max line error used while soft-following through the post-turn ignore window.

// Wall-following side and PID values. Use the side with the cleaner tunnel wall.
enum class WallSide {
  Left,
  Right
};

constexpr WallSide kDefaultWallSide = WallSide::Left;  // Wall side used for tunnel following by default.
constexpr float kTargetWallDistanceMm = 62.5f;  // Desired side-wall distance during tunnel wall following.
constexpr int kWallBaseSpeed = 520;  // Forward base speed during wall following.
constexpr int kWallMaxCorrection = 190;  // Maximum wall-following steering correction.
constexpr float kMaxFastSlowMotorRatio = 1.40f;  // Limit between faster and slower motor speeds.
constexpr float kWallKp = 1.0f;  // Wall-following proportional gain.
constexpr float kWallKi = 0.0f;  // Wall-following integral gain; normally keep at 0.
constexpr float kWallKd = 0.0f;  // Wall-following derivative gain.
constexpr float kWallIntegralClamp = 80.0f;  // Wall-following integral accumulator limit.
constexpr uint32_t kWallLoopDelayMs = 20;  // Delay at the end of each wall-following step.
constexpr uint32_t kWallPrintIntervalMs = 180;  // Minimum time between wall-following debug prints.
constexpr bool kStopIfNoWallEcho = false;  // true = stop if side sonar fails instead of using last valid reading.

// QTR-HD-09RC line sensor.
constexpr uint8_t kQtrCtrlOddPin = 2;  // QTR odd emitter control pin.
constexpr uint8_t kQtrCtrlEvenPin = 3;  // QTR even emitter control pin.
constexpr uint8_t kQtrPins[9] = {22, 23, 24, 25, 26, 27, 28, 29, 30};  // QTR sensor pins, left to right from robot front.
constexpr uint16_t kQtrTimeoutUs = 3000;  // Maximum RC discharge wait time per QTR reading.
constexpr uint16_t kMinUsefulCalibrationSpan = 20;  // Minimum raw max-min span needed to trust a sensor.
constexpr uint16_t kSavedQtrMin[9] = {82, 82, 82, 82, 82, 82, 82, 82, 93};  // Saved raw white-floor calibration.
constexpr uint16_t kSavedQtrMax[9] = {420, 336, 308, 301, 293, 316, 331, 341, 464};  // Saved raw black-line calibration.

// Sonar pins.
constexpr uint8_t kFrontTrigPin = 8;  // Front sonar trigger pin.
constexpr uint8_t kLeftTrigPin = 9;  // Left sonar trigger pin.
constexpr uint8_t kRightTrigPin = 10;  // Right sonar trigger pin.
constexpr uint8_t kFrontEchoPin = 11;  // Front sonar echo pin.
constexpr uint8_t kLeftEchoPin = 12;  // Left sonar echo pin.
constexpr uint8_t kRightEchoPin = 13;  // Right sonar echo pin.
constexpr uint32_t kEchoTimeoutUs = 12000;  // Sonar pulseIn timeout; roughly caps range.
constexpr float kMinValidSonarMm = 20.0f;  // Minimum accepted sonar distance.
constexpr float kMaxValidSonarMm = 900.0f;  // Maximum accepted sonar distance.

// Motoron and motor signs.
constexpr uint8_t kMotoronAddress = 0x11;  // I2C address of the Motoron motor controller.
constexpr uint8_t kMotoronLeftChannel = 1;  // Motoron channel connected to left motor.
constexpr uint8_t kMotoronRightChannel = 2;  // Motoron channel connected to right motor.
constexpr int kLeftMotorSign = 1;  // Flip to -1 if left motor runs backward.
constexpr int kRightMotorSign = 1;  // Flip to -1 if right motor runs backward.
constexpr int kLineErrorSign = 1;  // Flip to -1 if PD line correction steers away from line.
constexpr int kMaxMotorCommand = 800;  // Absolute Motoron command limit.

// Encoders.
constexpr uint8_t kLeftEncoderAPin = 34;  // Left encoder A/C1 pin.
constexpr uint8_t kLeftEncoderBPin = 35;  // Left encoder B/C2 direction pin.
constexpr uint8_t kRightEncoderAPin = 36;  // Right encoder A/C1 pin.
constexpr uint8_t kRightEncoderBPin = 37;  // Right encoder B/C2 direction pin.
constexpr float kWheelDiameterMm = 39.0f;  // Wheel diameter used for encoder distance conversion.
constexpr float kWheelTrackMm = 165.0f;  // Wheel track used for encoder-only turn conversion.
constexpr float kEncoderCountsPerMotorRev = 7.0f;  // Encoder pulses per motor shaft revolution before gearbox.
constexpr float kGearRatio = 150.0f;  // Motor gearbox ratio.
constexpr float kDistanceCalibration = 1.0f;  // Scale factor for straight encoder distance.
constexpr float kTurnCalibration = 1.5f;  // Scale factor for encoder-only turn distance.
constexpr float kStraightCorrectionKp = 0.35f;  // Encoder balancing gain during straight drives.
constexpr int kMaxStraightCorrection = 90;  // Maximum encoder balancing correction during straight drives.
constexpr uint32_t kDriveTimeoutMs = 12000;  // Maximum time allowed for an encoder distance drive.

// IMU yaw / turning.
constexpr uint8_t kImuAddress = 0x68;  // I2C address of the ICM20948 IMU.
constexpr uint16_t kGyroBiasSamples = 500;  // Number of gyro samples for bias calibration.
constexpr uint16_t kGyroBiasSampleDelayMs = 4;  // Delay between gyro bias samples.
constexpr int kTurnCommandSign = 1;  // Flip to -1 if positive turn command rotates wrong way.
constexpr int kImuYawSign = 1;  // Flip to -1 if yaw moves away from target during IMU turns.
constexpr int kTurnMaxSpeed = 560;  // Maximum motor command during IMU turns.
constexpr int kTurnMinSpeed = 115;  // Minimum motor command during IMU turns.
constexpr float kTurnKp = 500.0f;  // Proportional gain from yaw error to turn command.
constexpr float kTurnKd = 0.0f;  // Derivative damping from gyro rate to turn command.
constexpr float kTurnToleranceDeg = 2.0f;  // Acceptable final yaw error for a turn.
constexpr float kGyroStopRateDps = 10.0f;  // Required low gyro rate before declaring turn complete.
constexpr bool kUseImuTurnTimeout = false;  // true = abort IMU turn after kTurnTimeoutMs.
constexpr uint32_t kTurnTimeoutMs = 50000;  // Timeout for turn loops when enabled.
constexpr uint32_t kTurnPrintIntervalMs = 120;  // Minimum time between turn debug prints.

// RFID.
constexpr uint8_t kRfidAddress = 0x28;  // I2C address of the RFID2 reader.
constexpr uint8_t kRfidResetPin = 39;  // RFID reader reset pin.
constexpr uint32_t kRfidPollIntervalMs = 20;  // Minimum time between normal RFID polls.
constexpr uint8_t kBaseExitRfidBurstPolls = 4;  // Forced RFID polls per base-exit check while moving over the tag.
constexpr uint16_t kBaseExitRfidBurstGapMs = 5;  // Delay between forced base-exit RFID polls.

// Mechanical kill.
constexpr bool kUseKillPin = true;  // true = use D32 as start/stop button.
constexpr uint8_t kKillPin = 32;  // Start/stop button pin, active LOW with INPUT_PULLUP.
constexpr uint32_t kKillDebounceMs = 35;  // Button debounce time before accepting a press event.

#if USE_WIFI_AIRLOCK_REQUEST
constexpr const char *kBoardId = "YU7GT";  // Board ID used in register/openAirlock messages.
constexpr uint32_t kRegisterIntervalMs = 5000;  // Interval between register messages to stay online.
#endif
constexpr uint32_t kAirlockRequestRetryMs = 1000;  // Retry pending airlock request while WiFi/MQTT reconnects.
constexpr bool kRequireWifiBeforeCalibration = true;  // Wait for MiniMessenger before IMU calibration and motion.
constexpr uint32_t kWifiConnectTimeoutMs = 0;  // 0 = wait forever; nonzero aborts startup after this many ms.
constexpr uint32_t kWifiConnectPrintIntervalMs = 1000;  // Waiting-for-WiFi status print interval.
constexpr uint8_t kPixelBrightness = 70;  // Modulino Pixels brightness percent, 0..100.

constexpr float kPi = 3.1415926f;  // Pi constant for geometry calculations.
constexpr float kRadToDeg = 57.2957795f;  // Conversion factor from radians to degrees.
constexpr float kWheelCircumferenceMm = kWheelDiameterMm * kPi;  // Derived wheel circumference.
constexpr float kEncoderCountsPerWheelRev = kEncoderCountsPerMotorRev * kGearRatio;  // Derived counts per wheel revolution.
constexpr float kEncoderCountsPerMm = kEncoderCountsPerWheelRev / kWheelCircumferenceMm;  // Derived encoder counts per millimeter.

// ---------------------------------------------------------------------------
// Types / globals
// ---------------------------------------------------------------------------

enum class MissionState {
  Idle,
  FollowBaseRoute,
  FollowLineToTunnelEntry,
  WallFollowTunnel,
  WaitExitDoorOpen,
  SearchFirstRfid,
  AlignOverRfid,
  FollowLineToFirstGridRfid,
  EasyGridRoute,
  EasyDoorRequest,
  EasyAfterDoorForward,
  ReturnLineToTunnelEntry,
  ReturnWallFollowTunnel,
  ReturnFollowLineToBase,
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

struct EasyRouteStep {
  uint8_t rfidCount;
  TurnDir turnAfter;
  bool requestDoorAfterTurn;
};

// This route starts after the first grid RFID has already been handled and
// the robot has made the initial right turn.
constexpr EasyRouteStep kEasyRoute[] = {
    {2, TurnDir::Left, false},
    {3, TurnDir::Left, false},
    {1, TurnDir::Left, false},
    {2, TurnDir::Right, false},
    {1, TurnDir::Right, false},
    {2, TurnDir::Left, false},
    {1, TurnDir::Left, false},
    {3, TurnDir::Right, false},
    {1, TurnDir::Right, false},
    {3, TurnDir::Left, false},
    {1, TurnDir::Left, false},
    {3, TurnDir::Right, false},
    {1, TurnDir::Left, true},
};
constexpr uint8_t kEasyRouteLength = sizeof(kEasyRoute) / sizeof(kEasyRoute[0]);

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

struct DoorReading {
  float frontMm = -1.0f;
  bool valid = false;
  bool closed = false;
  bool open = false;
};

struct CellStatus {
  bool valid = false;
  bool fertile = false;
  bool planted = false;
};

MotoronI2C motoron(kMotoronAddress);
MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
Adafruit_ICM20948 imu;
ModulinoPixels pixels;

#if USE_WIFI_AIRLOCK_REQUEST
MiniMessenger messenger;
uint32_t lastRegisterMs = 0;
bool wifiInitialized = false;
#endif

uint16_t qtrRaw[9] = {};
uint16_t qtrMin[9] = {};
uint16_t qtrMax[9] = {};
uint16_t qtrNorm[9] = {};

volatile long leftCount = 0;
volatile long rightCount = 0;

MissionState missionState = kStartOnBoot ? MissionState::FollowBaseRoute : MissionState::Idle;
RouteChoice routeChoice = kDefaultRouteChoice;
WallSide wallSide = kDefaultWallSide;

float rfidAlignOffsetMm = kDefaultRfidAlignOffsetMm;

bool serialStopped = false;
bool rfidOk = false;
bool imuOk = false;
bool missionSensorsInitialized = false;
bool airlockRequestSent = false;
bool airlockAccepted = false;
bool encoderInterruptsAttached = false;
bool wifiSafetyEnabled = true;
bool stoppedByWifiKill = false;
bool pixelsOk = false;
String pendingAirlockUid;
uint32_t lastAirlockRequestAttemptMs = 0;

enum class PixelMode {
  Unknown,
  NormalBlue,
  RunningPurple
};

PixelMode currentPixelMode = PixelMode::Unknown;

bool lastKillReading = HIGH;
bool stableKillReading = HIGH;
uint32_t lastKillChangeMs = 0;

uint8_t routeTurnIndex = 0;
uint8_t eventStableCount = 0;
uint8_t doorClosedStableCount = 0;
uint8_t doorOpenStableCount = 0;
uint8_t wallExitLineStableCount = 0;
uint8_t returnTunnelNoLineCount = 0;
uint8_t returnWallExitLineStableCount = 0;
uint8_t easyRouteIndex = 0;
uint8_t easySegmentRfidCount = 0;
uint8_t seedsPlanted = 0;
int currentServoAngle = kServoMinAngle;

String lastUid;
String lastGridUid;
uint32_t lastGridUidMs = 0;
uint32_t lastGridTurnMs = 0;
bool easyDoorRequestSent = false;
bool waitingForCellStatus = false;
CellStatus lastCellStatus;

int lastLineError = 0;
int lastSeenLineError = 0;
float lineIntegral = 0.0f;
bool postTurnHardIgnoreActive = false;
uint32_t postTurnHardIgnoreStartMs = 0;
uint8_t postTurnHardReleaseCount = 0;
uint8_t tunnelEntryNoLineCount = 0;

float wallIntegral = 0.0f;
float lastWallErrorMm = 0.0f;
float lastValidLeftMm = -1.0f;
float lastValidRightMm = -1.0f;

float gyroZBiasDps = 0.0f;
float yawDeg = 0.0f;
float gyroZDegPerSec = 0.0f;
uint32_t lastImuUpdateUs = 0;

uint32_t lastLinePrintMs = 0;
uint32_t lastWallPrintMs = 0;
uint32_t lastDoorPrintMs = 0;
uint32_t lastTurnPrintMs = 0;
uint32_t lastRfidPollMs = 0;
uint32_t lastEventMs = 0;
uint32_t lastWallUpdateMs = 0;
uint32_t stateStartMs = 0;

// Forward declarations needed because the serial helpers appear before the
// sonar section in this sketch.
float readSonarMm(uint8_t trigPin, uint8_t echoPin);
/**
 * Forward declaration for serial/button start paths.
 */
void restartMission();

/**
 * Forward declaration for long blocking loops.
 */
bool handleStartStopButtonEvent();

/**
 * Forward declaration used before the optional WiFi section.
 */
void updateWifi();

/**
 * Forward declaration used by restartMission().
 */
bool initializeMissionSensorsForRun();

/**
 * Forward declaration used by post-turn route-event filtering.
 */
bool centerHasLine();

/**
 * Forward declarations used by first-T distance gating.
 */
long getLeftCount();
long getRightCount();


// ---------------------------------------------------------------------------
// Small helpers and labels
// ---------------------------------------------------------------------------

/**
 * Return the absolute value of a long integer.
 *
 * @param value Signed long value.
 * @return Non-negative magnitude.
 */
long absLong(long value) {
  return value < 0 ? -value : value;
}

/**
 * Return the absolute value of a float.
 *
 * @param value Signed float value.
 * @return Non-negative magnitude.
 */
float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

/**
 * Clamp a motor command to the configured Motoron command range.
 *
 * @param speed Requested motor speed.
 * @return Clamped motor speed.
 */
int clampMotorSpeed(int speed) {
  if (speed > kMaxMotorCommand) return kMaxMotorCommand;
  if (speed < -kMaxMotorCommand) return -kMaxMotorCommand;
  return speed;
}

/**
 * Check the mechanical kill switch.
 *
 * @return true when the kill pin is enabled and pulled LOW.
 */
bool killPressed() {
  return kUseKillPin && digitalRead(kKillPin) == LOW;
}

/**
 * Detect one debounced start/stop button press.
 *
 * The D32 button is active LOW. This returns true only once per physical press,
 * after the input has stayed stable for kKillDebounceMs.
 *
 * @return true on a debounced press event.
 */
bool killButtonPressedEvent() {
  if (!kUseKillPin) return false;

  const bool reading = digitalRead(kKillPin);
  if (reading != lastKillReading) {
    lastKillReading = reading;
    lastKillChangeMs = millis();
  }

  if (millis() - lastKillChangeMs < kKillDebounceMs) return false;
  if (reading == stableKillReading) return false;

  stableKillReading = reading;
  return stableKillReading == LOW;
}

/**
 * Convert mission state to a flash-stored diagnostic label.
 *
 * @param state Mission state to describe.
 * @return Printable state label.
 */
const __FlashStringHelper *stateName(MissionState state) {
  switch (state) {
    case MissionState::Idle: return F("IDLE");
    case MissionState::FollowBaseRoute: return F("FOLLOW_BASE_ROUTE");
    case MissionState::FollowLineToTunnelEntry: return F("FOLLOW_LINE_TO_TUNNEL_ENTRY");
    case MissionState::WallFollowTunnel: return F("WALL_FOLLOW_TUNNEL");
    case MissionState::WaitExitDoorOpen: return F("WAIT_EXIT_DOOR_OPEN");
    case MissionState::SearchFirstRfid: return F("SEARCH_FIRST_RFID");
    case MissionState::AlignOverRfid: return F("ALIGN_OVER_RFID");
    case MissionState::FollowLineToFirstGridRfid: return F("FOLLOW_LINE_TO_FIRST_GRID_RFID");
    case MissionState::EasyGridRoute: return F("EASY_GRID_ROUTE");
    case MissionState::EasyDoorRequest: return F("EASY_DOOR_REQUEST");
    case MissionState::EasyAfterDoorForward: return F("EASY_AFTER_DOOR_FORWARD");
    case MissionState::ReturnLineToTunnelEntry: return F("RETURN_LINE_TO_TUNNEL_ENTRY");
    case MissionState::ReturnWallFollowTunnel: return F("RETURN_WALL_FOLLOW_TUNNEL");
    case MissionState::ReturnFollowLineToBase: return F("RETURN_FOLLOW_LINE_TO_BASE");
    case MissionState::Done: return F("DONE");
    case MissionState::Stopped: return F("STOPPED");
    default: return F("UNKNOWN");
  }
}

/**
 * Convert line follow mode to a flash-stored diagnostic label.
 *
 * @param mode Follow mode chosen by the line sensor logic.
 * @return Printable mode label.
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
 * Convert turn direction to a flash-stored label.
 *
 * @param dir Turn direction.
 * @return "LEFT" or "RIGHT".
 */
const __FlashStringHelper *turnName(TurnDir dir) {
  return dir == TurnDir::Left ? F("LEFT") : F("RIGHT");
}

/**
 * Convert wall side to a flash-stored label.
 *
 * @param side Selected wall-following side.
 * @return "LEFT" or "RIGHT".
 */
const __FlashStringHelper *sideName(WallSide side) {
  return side == WallSide::Left ? F("LEFT") : F("RIGHT");
}

/**
 * Convert route choice to a flash-stored label.
 *
 * @param route Selected base route.
 * @return "A_RIGHT" or "B_LEFT".
 */
const __FlashStringHelper *routeName(RouteChoice route) {
  return route == RouteChoice::BaseA_Bottom ? F("A_RIGHT") : F("B_LEFT");
}

/**
 * Reset the line-following controller memory.
 *
 * This prevents stale integral/derivative values from affecting a new segment.
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
 * Reset the wall-following controller memory.
 *
 * This is called before starting tunnel wall following.
 */
void resetWallController() {
  wallIntegral = 0.0f;
  lastWallErrorMm = 0.0f;
  lastWallUpdateMs = millis();
}

/**
 * Clear door hysteresis counters.
 *
 * Door closed/open checks use stable-frame counters, so each new door wait
 * state should start from a clean counter.
 */
void resetDoorCounters() {
  doorClosedStableCount = 0;
  doorOpenStableCount = 0;
}

/**
 * Set all eight Modulino Pixels to one color.
 *
 * @param color Modulino color constant.
 * @param mode Logical mode used to avoid redundant I2C writes.
 */
void setAllPixels(ModulinoColor color, PixelMode mode) {
  if (!pixelsOk) return;
  if (currentPixelMode == mode) return;

  for (uint8_t i = 0; i < 8; i++) {
    pixels.set(i, color, kPixelBrightness);
  }
  pixels.show();
  currentPixelMode = mode;
}

/**
 * Blue means the robot is normal but not actively executing the base-exit run.
 */
void setPixelsNormalBlue() {
  setAllPixels(BLUE, PixelMode::NormalBlue);
}

/**
 * Purple/Violet means the robot is starting, calibrating, or actively running.
 */
void setPixelsRunningPurple() {
  setAllPixels(VIOLET, PixelMode::RunningPurple);
}

/**
 * Keep the Modulino Pixels synchronized with mission state.
 *
 * Idle/Stopped/Done are blue. All active run states are purple.
 *
 * @param state Current mission state.
 */
void updatePixelsForState(MissionState state) {
  if (state == MissionState::Idle ||
      state == MissionState::Stopped ||
      state == MissionState::Done) {
    setPixelsNormalBlue();
  } else {
    setPixelsRunningPurple();
  }
}

/**
 * Change mission state and record the transition time.
 *
 * @param newState State to enter immediately.
 */
void setState(MissionState newState) {
  missionState = newState;
  stateStartMs = millis();
  resetDoorCounters();
  updatePixelsForState(newState);
  if (newState == MissionState::FollowLineToTunnelEntry) {
    tunnelEntryNoLineCount = 0;
  }
  if (newState == MissionState::WallFollowTunnel) {
    wallExitLineStableCount = 0;
  }
  if (newState == MissionState::ReturnLineToTunnelEntry) {
    returnTunnelNoLineCount = 0;
    resetLineController();
  }
  if (newState == MissionState::ReturnWallFollowTunnel) {
    returnWallExitLineStableCount = 0;
    resetWallController();
  }
  if (newState == MissionState::FollowLineToFirstGridRfid ||
      newState == MissionState::EasyGridRoute ||
      newState == MissionState::EasyAfterDoorForward ||
      newState == MissionState::ReturnFollowLineToBase) {
    resetLineController();
  }
  Serial.print(F("[STATE] "));
  Serial.println(stateName(newState));
}

/**
 * Convert a 0-300 degree servo target to a 50 Hz pulse width.
 *
 * @param angle Servo angle in degrees.
 * @return Pulse width in microseconds.
 */
int angleToPulseUs(int angle) {
  angle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  return map(angle, kServoMinAngle, kServoMaxAngle, kServoMinUs, kServoMaxUs);
}

/**
 * Send one blocking servo PWM frame.
 *
 * @param pulseUs High pulse width in microseconds.
 */
void sendServoPulse(int pulseUs) {
  digitalWrite(kServoPin, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(kServoPin, LOW);
  delayMicroseconds(kServoFrameUs - pulseUs);
}

/**
 * Hold a servo angle by repeatedly sending PWM frames.
 *
 * @param angle Servo angle in degrees.
 * @param durationMs Duration to keep sending the command.
 */
void holdServoAngle(int angle, uint32_t durationMs) {
  const int pulseUs = angleToPulseUs(angle);
  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    sendServoPulse(pulseUs);
  }
}

/**
 * Move the seed-release servo to an absolute angle.
 *
 * @param angle Target servo angle.
 */
void moveServoToAngle(int angle) {
  currentServoAngle = constrain(angle, kServoMinAngle, kServoMaxAngle);
  Serial.print(F("[SERVO] angle="));
  Serial.print(currentServoAngle);
  Serial.print(F(" pulseUs="));
  Serial.println(angleToPulseUs(currentServoAngle));
  holdServoAngle(currentServoAngle, kServoMoveSettleMs);
}

/**
 * Drop one seed by stepping the 300 degree servo by 60 degrees.
 */
void dropOneSeed() {
  int nextAngle = currentServoAngle + kServoStepAngle;
  if (nextAngle > kServoMaxAngle) {
    Serial.println(F("[SERVO] 300deg reached; reset to 0 before next drop."));
    moveServoToAngle(kServoMinAngle);
    holdServoAngle(kServoMinAngle, 400);
    nextAngle = kServoStepAngle;
  }

  Serial.println(F("[PLANT] drop one seed."));
  moveServoToAngle(nextAngle);
  holdServoAngle(currentServoAngle, kServoHoldAfterDropMs);
}

// ---------------------------------------------------------------------------
// Motor / encoder
// ---------------------------------------------------------------------------

/**
 * Send tank-drive commands to the Motoron.
 *
 * @param leftSpeed Left motor command before clamping and sign correction.
 * @param rightSpeed Right motor command before clamping and sign correction.
 */
void setTank(int leftSpeed, int rightSpeed) {
  leftSpeed = clampMotorSpeed(leftSpeed) * kLeftMotorSign;
  rightSpeed = clampMotorSpeed(rightSpeed) * kRightMotorSign;
  motoron.setSpeed(kMotoronLeftChannel, leftSpeed);
  motoron.setSpeed(kMotoronRightChannel, rightSpeed);
}

/**
 * Stop both motors.
 */
void stopMotors() {
  setTank(0, 0);
}

/**
 * Left encoder ISR using channel B for direction.
 *
 * This is attached to channel A rising edges to match the encoder constants.
 */
void leftEncoderIsr() {
  leftCount += (digitalRead(kLeftEncoderBPin) == LOW) ? 1 : -1;
}

/**
 * Right encoder ISR using channel B for direction.
 *
 * This is attached to channel A rising edges to match the encoder constants.
 */
void rightEncoderIsr() {
  rightCount += (digitalRead(kRightEncoderBPin) == LOW) ? 1 : -1;
}

/**
 * Read the left encoder count atomically.
 *
 * @return Snapshot of leftCount.
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
 * @return Snapshot of rightCount.
 */
long getRightCount() {
  noInterrupts();
  const long count = rightCount;
  interrupts();
  return count;
}

/**
 * Reset both encoder counters atomically.
 */
void resetEncoders() {
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
long distanceMmToCounts(float distanceMm) {
  return static_cast<long>(absFloat(distanceMm) * kEncoderCountsPerMm * kDistanceCalibration);
}

/**
 * Convert in-place turn angle to encoder counts for fallback turning.
 *
 * @param degrees Signed turn angle; sign is ignored here.
 * @return Positive per-side encoder target count.
 */
long turnDegreesToEncoderCounts(float degrees) {
  const float wheelTravelMm = kPi * kWheelTrackMm * absFloat(degrees) / 360.0f;
  return static_cast<long>(wheelTravelMm * kEncoderCountsPerMm * kTurnCalibration);
}

/**
 * Initialize the Motoron motor controller.
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
 */
void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  start | restart | stop | resume | show"));
  Serial.println(F("  route a | route b | route right | route left"));
  Serial.println(F("  wall left | wall right"));
  Serial.println(F("  alignmm 0       set RFID alignment offset after tag detection"));
  Serial.println(F("  sonar           print one front/left/right sonar snapshot"));
  Serial.println(F("  D32 button: press in IDLE/STOPPED to start; press while moving to stop."));
}

/**
 * Print current tunable settings and mission progress.
 */
void printSettings() {
  Serial.print(F("[SETTINGS] state="));
  Serial.print(stateName(missionState));
  Serial.print(F(" route="));
  Serial.print(routeName(routeChoice));
  Serial.print(F(" routeTurn="));
  Serial.print(routeTurnIndex);
  Serial.print(F("/"));
  Serial.print(kRouteTurnCount);
  Serial.print(F(" airlockRfidFromTurn="));
  Serial.print(kAirlockRfidCheckFromTurnIndex);
  Serial.print(F(" wall="));
  Serial.print(sideName(wallSide));
  Serial.print(F(" alignMm="));
  Serial.print(rfidAlignOffsetMm, 1);
  Serial.print(F(" lastUid="));
  Serial.print(lastUid.length() > 0 ? lastUid : String("none"));
  Serial.print(F(" wifiConnected="));
#if USE_WIFI_AIRLOCK_REQUEST
  Serial.print(messenger.isConnected() ? F("YES") : F("NO"));
#else
  Serial.print(F("DISABLED"));
#endif
  Serial.print(F(" wifiSafety="));
  Serial.print(wifiSafetyEnabled ? F("ENABLED") : F("DISABLED"));
  Serial.print(F(" pendingAirlock="));
  Serial.print(pendingAirlockUid.length() > 0 ? pendingAirlockUid : String("none"));
  Serial.print(F(" pixels="));
  Serial.print(pixelsOk ? F("OK") : F("NOT_FOUND"));
  Serial.print(F(" stopped="));
  Serial.println(serialStopped ? F("YES") : F("NO"));
}

/**
 * Reset all mission progress and start again from the base line.
 */
void restartMission() {
  if (!wifiSafetyEnabled) {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    Serial.println(F("[WIFI SAFETY] start blocked because server safety is disabled. Wait for enable=1."));
    return;
  }

  serialStopped = false;
  stoppedByWifiKill = false;
  setPixelsRunningPurple();
  airlockRequestSent = false;
  airlockAccepted = false;
  pendingAirlockUid = "";
  lastAirlockRequestAttemptMs = 0;
  routeTurnIndex = 0;
  eventStableCount = 0;
  tunnelEntryNoLineCount = 0;
  wallExitLineStableCount = 0;
  returnTunnelNoLineCount = 0;
  returnWallExitLineStableCount = 0;
  easyRouteIndex = 0;
  easySegmentRfidCount = 0;
  seedsPlanted = 0;
  easyDoorRequestSent = false;
  waitingForCellStatus = false;
  lastCellStatus = {};
  postTurnHardIgnoreActive = false;
  postTurnHardReleaseCount = 0;
  lastUid = "";
  lastGridUid = "";
  lastGridUidMs = 0;
  lastGridTurnMs = 0;
  lastEventMs = millis();
  resetLineController();
  resetWallController();

  if (!initializeMissionSensorsForRun()) {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    return;
  }

  setState(MissionState::FollowBaseRoute);
}

/**
 * Handle the D32 button as a mission start/stop control.
 *
 * Pressing the button in Idle/Stopped/Done initializes sensors and starts a new
 * mission. Pressing it while moving stops the robot immediately.
 *
 * @return true if the press requested a stop.
 */
bool handleStartStopButtonEvent() {
  if (!killButtonPressedEvent()) return false;

  if (missionState == MissionState::Idle ||
      missionState == MissionState::Stopped ||
      missionState == MissionState::Done) {
    Serial.println(F("[BUTTON] press -> START"));
    restartMission();
    return false;
  }

  serialStopped = true;
  stopMotors();
  setState(MissionState::Stopped);
  Serial.println(F("[BUTTON] press -> STOP"));
  return true;
}

/**
 * Read and print one sonar snapshot for field debugging.
 */
void printSonarSnapshot() {
  if (!missionSensorsInitialized) {
    Serial.println(F("[SONAR] sensors are initialized after START; press D32 or send start first."));
    return;
  }

  const float front = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  delay(8);
  const float left = readSonarMm(kLeftTrigPin, kLeftEchoPin);
  delay(8);
  const float right = readSonarMm(kRightTrigPin, kRightEchoPin);

  Serial.print(F("[SONAR] frontMm="));
  Serial.print(front, 1);
  Serial.print(F(" leftMm="));
  Serial.print(left, 1);
  Serial.print(F(" rightMm="));
  Serial.println(right, 1);
}

/**
 * Parse and execute one complete Serial Monitor command.
 *
 * @param line Raw command line without the trailing newline.
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
  if (lower == "sonar") {
    printSonarSnapshot();
    return;
  }
  if (lower == "start" || lower == "restart") {
    restartMission();
    return;
  }
  if (lower == "stop") {
    serialStopped = true;
    stopMotors();
    setState(MissionState::Stopped);
    return;
  }
  if (lower == "resume") {
    restartMission();
    return;
  }
  if (lower == "wall left") {
    wallSide = WallSide::Left;
    printSettings();
    return;
  }
  if (lower == "wall right") {
    wallSide = WallSide::Right;
    printSettings();
    return;
  }
  if (lower == "route a" || lower == "route right" || lower == "route bottom") {
    routeChoice = RouteChoice::BaseA_Bottom;
    routeTurnIndex = 0;
    eventStableCount = 0;
    printSettings();
    return;
  }
  if (lower == "route b" || lower == "route left" || lower == "route top") {
    routeChoice = RouteChoice::BaseB_Top;
    routeTurnIndex = 0;
    eventStableCount = 0;
    printSettings();
    return;
  }

  const int space = lower.indexOf(' ');
  if (space <= 0) {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  const String key = lower.substring(0, space);
  const float value = line.substring(space + 1).toFloat();

  if (key == "alignmm") {
    rfidAlignOffsetMm = constrain(value, 0.0f, 300.0f);
  } else {
    Serial.print(F("[SERIAL] unknown command: "));
    Serial.println(line);
    return;
  }

  printSettings();
}

/**
 * Accumulate Serial input and dispatch newline-terminated commands.
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
// Sonar and door detection
// ---------------------------------------------------------------------------

/**
 * Read one HC-SR04 style sonar distance in millimeters.
 *
 * @param trigPin Trigger output pin.
 * @param echoPin Echo input pin.
 * @return Distance in millimeters, or -1 if no echo was received.
 */
float readSonarMm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  const uint32_t durationUs = pulseIn(echoPin, HIGH, kEchoTimeoutUs);
  if (durationUs == 0) return -1.0f;
  return durationUs * 0.1715f;
}

/**
 * Check whether a sonar distance is inside the useful range.
 *
 * @param mm Distance reading in millimeters.
 * @return true if the reading should be trusted.
 */
bool isValidSonarDistance(float mm) {
  return mm >= kMinValidSonarMm && mm <= kMaxValidSonarMm;
}

/**
 * Read the front sonar and classify door state.
 *
 * @return DoorReading with raw distance, validity, closed flag, and open flag.
 */
DoorReading readDoor() {
  DoorReading reading;
  reading.frontMm = readSonarMm(kFrontTrigPin, kFrontEchoPin);
  reading.valid = isValidSonarDistance(reading.frontMm);
  reading.closed = reading.valid && reading.frontMm <= kDoorClosedThresholdMm;
  reading.open = (reading.valid && reading.frontMm >= kDoorOpenThresholdMm) ||
                 (!reading.valid && kTreatNoEchoAsOpen);
  return reading;
}

/**
 * Update stable closed-door detection.
 *
 * @param reading Latest front sonar door reading.
 * @return true after the door has looked closed for enough consecutive frames.
 */
bool doorClosedStable(const DoorReading &reading) {
  if (reading.closed) {
    if (doorClosedStableCount < 255) doorClosedStableCount++;
  } else {
    doorClosedStableCount = 0;
  }
  return doorClosedStableCount >= kDoorStableFrames;
}

/**
 * Update stable open-door detection.
 *
 * @param reading Latest front sonar door reading.
 * @return true after the door has looked open for enough consecutive frames.
 */
bool doorOpenStable(const DoorReading &reading) {
  if (reading.open) {
    if (doorOpenStableCount < 255) doorOpenStableCount++;
  } else {
    doorOpenStableCount = 0;
  }
  return doorOpenStableCount >= kDoorStableFrames;
}

/**
 * Print throttled door sonar diagnostics.
 *
 * @param label Log prefix describing which door is being handled.
 * @param reading Latest door reading.
 */
void printDoorStatus(const __FlashStringHelper *label, const DoorReading &reading) {
  if (millis() - lastDoorPrintMs < kDoorPrintIntervalMs) return;
  lastDoorPrintMs = millis();

  Serial.print(label);
  Serial.print(F(" frontMm="));
  Serial.print(reading.frontMm, 1);
  Serial.print(F(" valid="));
  Serial.print(reading.valid ? F("YES") : F("NO"));
  Serial.print(F(" closedStable="));
  Serial.print(doorClosedStableCount);
  Serial.print(F(" openStable="));
  Serial.println(doorOpenStableCount);
}

// ---------------------------------------------------------------------------
// IMU / turning
// ---------------------------------------------------------------------------

/**
 * Initialize the ICM20948 IMU hardware.
 *
 * @return true if the IMU was found, false if encoder fallback is needed.
 */
bool initializeImuHardware() {
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
bool calibrateImuGyroBias() {
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
 * Reset relative yaw to zero.
 */
void resetYaw() {
  yawDeg = 0.0f;
  gyroZDegPerSec = 0.0f;
  lastImuUpdateUs = micros();
}

/**
 * Command an in-place turn.
 *
 * @param signedTurnSpeed Positive uses the configured left-turn convention.
 */
void setTurnCommand(int signedTurnSpeed) {
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
 * Turn using encoders when the IMU is unavailable.
 *
 * @param degrees Signed turn angle.
 * @param speed Absolute turn speed.
 */
void encoderOnlyTurnFallback(float degrees, int speed) {
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
bool turnDegreesImu(float targetDeg) {
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

// ---------------------------------------------------------------------------
// QTR line sensor / base route line following
// ---------------------------------------------------------------------------

/**
 * Initialize QTR pins and saved calibration values.
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
 * Read raw RC timing values from all QTR channels.
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
 * Normalize raw QTR timing values into 0..1000 black-line strength.
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
 * Count active QTR sensors above a threshold.
 *
 * @param threshold Normalized black-line threshold.
 * @return Number of sensors above threshold.
 */
uint8_t activeSensorCount(uint16_t threshold) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 9; i++) {
    if (qtrNorm[i] >= threshold) count++;
  }
  return count;
}

/**
 * Check whether the center sensors are over the line.
 *
 * @return true when sensors 3, 4, or 5 see black.
 */
bool centerHasLine() {
  return qtrNorm[3] >= kLineThreshold || qtrNorm[4] >= kLineThreshold || qtrNorm[5] >= kLineThreshold;
}

/**
 * Check whether the left outer sensors see a strong line.
 *
 * @return true when sensor 0 or 1 is strongly active.
 */
bool leftOuterHasStrongLine() {
  return qtrNorm[0] >= kStrongLineThreshold || qtrNorm[1] >= kStrongLineThreshold;
}

/**
 * Check whether the right outer sensors see a strong line.
 *
 * @return true when sensor 7 or 8 is strongly active.
 */
bool rightOuterHasStrongLine() {
  return qtrNorm[7] >= kStrongLineThreshold || qtrNorm[8] >= kStrongLineThreshold;
}

/**
 * Compute weighted line position from normalized QTR readings.
 *
 * @param detectedOut Output flag that becomes true if any sensor sees the line.
 * @return Weighted position from 0..8000, centered at 4000.
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
 * Select the line-following mode for the current reading.
 *
 * @param line Current line reading.
 * @return Follow/search/hard-turn mode.
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
 * Read the line sensor and build a complete LineReading snapshot.
 *
 * @return LineReading containing raw, normalized, position, error, and mode.
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
 * Convert line reading into motor commands.
 *
 * @param line Current line reading and follow mode.
 * @return Signed left/right motor command.
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
 * Apply one line-following step and print diagnostics.
 *
 * @param line Current line reading.
 * @param label Log prefix.
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
    Serial.print(F(" turn="));
    Serial.print(routeTurnIndex);
    Serial.print(F("/"));
    Serial.print(kRouteTurnCount);
    Serial.print(F(" mode="));
    Serial.print(followModeName(line.mode));
    Serial.print(F(" active="));
    Serial.print(line.activeCount);
    Serial.print(F(" err="));
    Serial.print(line.error);
    Serial.print(F(" L="));
    Serial.print(cmd.left);
    Serial.print(F(" R="));
    Serial.println(cmd.right);
  }

  updateImu();
  delay(kLineLoopDelayMs);
}

// ---------------------------------------------------------------------------
// RFID
// ---------------------------------------------------------------------------

/**
 * Initialize the RFID reader.
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
 * Convert the last read RFID UID to uppercase hex text.
 *
 * @return UID string.
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
 * Poll RFID for a tag.
 *
 * @param uidOut Output UID string when a tag is found.
 * @param force true bypasses the normal rate limit for short burst scans.
 * @return true when a tag was read.
 */
bool pollRfid(String *uidOut, bool force) {
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
bool pollRfid(String *uidOut) {
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
bool pollRfidBurst(String *uidOut, uint8_t attempts, uint16_t gapMs) {
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
void stopForWifiSafety(const __FlashStringHelper *reason) {
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
void allowWifiSafety() {
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
void parseCellStatusReply(const char *msg) {
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
void onWifiMessage(const MessageMetadata& metadata, const uint8_t* payload, size_t length) {
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
void initializeWifi() {
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
void updateWifi() {
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
bool waitForWifiBeforeCalibration() {
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
bool sendAirlockOpenRequest(const String &uid, char airlock) {
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
bool sendAirlockOpenRequest(const String &uid) {
  return sendAirlockOpenRequest(uid, 'A');
}

/**
 * @return true when MiniMessenger is connected to the server.
 */
bool serverOnline() {
  return messenger.isConnected();
}

/**
 * Ask the server whether an RFID cell is fertile and unplanted.
 *
 * @param uid RFID tag UID.
 * @param statusOut Filled with the reply when one is received.
 * @return true when an isFertileReply was received before timeout.
 */
bool queryServerForCellStatus(const String &uid, CellStatus *statusOut) {
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
void notifySeedPlanted(const String &uid) {
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
void initializeWifi() {}

/**
 * No-op WiFi service function when secrets.h is not present.
 */
void updateWifi() {}

/**
 * Abort startup when WiFi is required but secrets.h is not available.
 *
 * @return true only when WiFi is not required.
 */
bool waitForWifiBeforeCalibration() {
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
bool sendAirlockOpenRequest(const String &uid, char airlock) {
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
bool sendAirlockOpenRequest(const String &uid) {
  return sendAirlockOpenRequest(uid, 'A');
}

/**
 * @return false when MiniMessenger is disabled.
 */
bool serverOnline() {
  return false;
}

/**
 * Offline fallback for cell status queries.
 *
 * @param uid RFID tag UID.
 * @param statusOut Filled with invalid/false values.
 * @return false because no server reply can be received.
 */
bool queryServerForCellStatus(const String &uid, CellStatus *statusOut) {
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
void notifySeedPlanted(const String &uid) {
  Serial.print(F("[SERVER] WiFi disabled. Would send seedPlanted tag_id="));
  Serial.println(uid);
}
#endif

/**
 * Retry a pending base-exit airlock request without needing to see the RFID tag again.
 *
 * @return true only when the request has just been sent successfully.
 */
bool servicePendingAirlockRequest() {
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
bool checkBaseExitRfidAndRequestAirlock() {
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

// ---------------------------------------------------------------------------
// Route-scripted base movement
// ---------------------------------------------------------------------------

/**
 * Return the selected route turn direction for a route event.
 *
 * @param index Route event index, 0..3.
 * @return Turn direction for the selected A/right/lower or B/left/upper route.
 */
TurnDir routeTurnAt(uint8_t index) {
  if (routeChoice == RouteChoice::BaseA_Bottom) {
    // A right/lower route: RIGHT, LEFT, LEFT, RIGHT.
    if (index == 0 || index == 3) return TurnDir::Right;
    return TurnDir::Left;
  }

  // B left/upper route: LEFT, RIGHT, RIGHT, LEFT.
  if (index == 0 || index == 3) return TurnDir::Left;
  return TurnDir::Right;
}

/**
 * Convert route turn direction to IMU degrees.
 *
 * @param dir Turn direction.
 * @return +90 for left, -90 for right.
 */
float degreesForTurn(TurnDir dir) {
  return dir == TurnDir::Left ? 90.0f : -90.0f;
}

/**
 * Detect the first T/bifurcation using near-all-black QTR evidence.
 *
 * @param line Current line reading.
 * @return true when the first T pattern is present.
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
 * Detect later hard-turn events only in the expected route direction.
 *
 * @param line Current line reading.
 * @param expected Expected scripted turn direction.
 * @return true when current evidence matches the expected turn.
 */
bool expectedSharpTurnDetected(const LineReading &line, TurnDir expected) {
  if (!line.detected) return false;

  if (expected == TurnDir::Left) {
    return line.mode == FollowMode::HardLeft || (leftOuterHasStrongLine() && line.error < -kCenterRecoverError);
  }

  return line.mode == FollowMode::HardRight || (rightOuterHasStrongLine() && line.error > kCenterRecoverError);
}

/**
 * Debounce route-event detection.
 *
 * @param line Current line reading.
 * @return true when the next committed route turn should run.
 */
bool routeEventReady(const LineReading &line) {
  if (routeTurnIndex >= kRouteTurnCount) return false;
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
 * Reacquire the line after a committed 90 degree turn.
 *
 * @param dir Direction of the turn just completed.
 * @return false only when stopped or killed.
 */
bool reacquireLineAfterTurn(TurnDir dir) {
  const uint32_t start = millis();
  uint8_t stable = 0;
  const int command = dir == TurnDir::Left ? kReacquireTurnSpeed : -kReacquireTurnSpeed;

  while (millis() - start < kReacquireTimeoutMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    const LineReading line = readLine();
    if (line.detected && centerHasLine()) {
      stable++;
      stopMotors();
      if (stable >= kReacquireStableFrames) {
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
  resetLineController();
  return true;
}

/**
 * Execute one committed turn for the selected base route.
 *
 * @return true if completed, false if stopped or killed.
 */
bool performRouteTurn() {
  if (routeTurnIndex >= kRouteTurnCount) return true;

  const TurnDir dir = routeTurnAt(routeTurnIndex);
  const bool firstT = routeTurnIndex == 0;
  const float advanceMm = firstT ? kFirstTAdvanceMm : kSharpTurnAdvanceMm;

  stopMotors();
  delay(80);

  Serial.print(F("[ROUTE] turnIndex="));
  Serial.print(routeTurnIndex);
  Serial.print(F(" route="));
  Serial.print(routeName(routeChoice));
  Serial.print(F(" dir="));
  Serial.println(turnName(dir));

  if (!driveDistanceMm(advanceMm, kRouteAdvanceSpeed)) return false;
  delay(60);
  if (!turnDegreesImu(degreesForTurn(dir))) return false;
  delay(80);
  if (!reacquireLineAfterTurn(dir)) return false;

  routeTurnIndex++;
  eventStableCount = 0;
  lastEventMs = millis();
  beginPostTurnHardIgnore();
  return true;
}

// ---------------------------------------------------------------------------
// Wall following
// ---------------------------------------------------------------------------

/**
 * Compute the maximum correction allowed by the fast/slow motor ratio limit.
 *
 * @return Maximum correction magnitude.
 */
int maxWallCorrectionFromRatioLimit() {
  if (kMaxFastSlowMotorRatio <= 1.0f || kWallBaseSpeed <= 0) return 0;
  const float maxCorrection =
      kWallBaseSpeed * (kMaxFastSlowMotorRatio - 1.0f) / (kMaxFastSlowMotorRatio + 1.0f);
  return maxCorrection > 0.0f ? static_cast<int>(maxCorrection) : 0;
}

/**
 * Read the selected wall-following side sonar.
 *
 * @param usedFallback Output flag true when last valid reading is reused.
 * @return Wall distance in millimeters, or -1 when no usable reading exists.
 */
float selectedWallDistanceMm(bool *usedFallback) {
  const bool useLeft = wallSide == WallSide::Left;
  const uint8_t trig = useLeft ? kLeftTrigPin : kRightTrigPin;
  const uint8_t echo = useLeft ? kLeftEchoPin : kRightEchoPin;
  const float mm = readSonarMm(trig, echo);

  if (isValidSonarDistance(mm)) {
    if (useLeft) lastValidLeftMm = mm;
    else lastValidRightMm = mm;
    *usedFallback = false;
    return mm;
  }

  const float fallback = useLeft ? lastValidLeftMm : lastValidRightMm;
  if (fallback > 0.0f && !kStopIfNoWallEcho) {
    *usedFallback = true;
    return fallback;
  }

  *usedFallback = false;
  return -1.0f;
}

/**
 * Apply one wall-following PID step.
 *
 * @return true when a valid wall command was applied; false when sonar failed.
 */
bool runWallFollowStep() {
  bool usedFallback = false;
  const float distanceMm = selectedWallDistanceMm(&usedFallback);
  if (distanceMm < 0.0f) {
    stopMotors();
    Serial.println(F("[WALL] no valid wall distance."));
    delay(80);
    return false;
  }

  const uint32_t now = millis();
  float dt = (now - lastWallUpdateMs) / 1000.0f;
  lastWallUpdateMs = now;
  if (dt <= 0.0f || dt > 0.25f) dt = kWallLoopDelayMs / 1000.0f;

  const float errorMm = distanceMm - kTargetWallDistanceMm;
  wallIntegral += errorMm * dt;
  wallIntegral = constrain(wallIntegral, -kWallIntegralClamp, kWallIntegralClamp);
  const float derivative = (errorMm - lastWallErrorMm) / dt;
  lastWallErrorMm = errorMm;

  const float pid = kWallKp * errorMm + kWallKi * wallIntegral + kWallKd * derivative;
  const int ratioCorrectionLimit = maxWallCorrectionFromRatioLimit();
  int activeCorrectionLimit = kWallMaxCorrection;
  if (ratioCorrectionLimit < activeCorrectionLimit) activeCorrectionLimit = ratioCorrectionLimit;
  const int correction = constrain(static_cast<int>(pid), -activeCorrectionLimit, activeCorrectionLimit);

  // Positive turnLeftCorrection slows the left motor and speeds the right.
  const int turnLeftCorrection = wallSide == WallSide::Left ? correction : -correction;
  const int leftSpeed = kWallBaseSpeed - turnLeftCorrection;
  const int rightSpeed = kWallBaseSpeed + turnLeftCorrection;
  setTank(leftSpeed, rightSpeed);

  if (millis() - lastWallPrintMs >= kWallPrintIntervalMs) {
    lastWallPrintMs = millis();
    Serial.print(F("[WALL] side="));
    Serial.print(sideName(wallSide));
    Serial.print(F(" distMm="));
    Serial.print(distanceMm, 1);
    Serial.print(usedFallback ? F(" fallback") : F(""));
    Serial.print(F(" err="));
    Serial.print(errorMm, 1);
    Serial.print(F(" corr="));
    Serial.print(correction);
    Serial.print(F(" L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
  }

  updateImu();
  delay(kWallLoopDelayMs);
  return true;
}

// ---------------------------------------------------------------------------
// Post-tunnel RFID search
// ---------------------------------------------------------------------------

/**
 * Drive forward with encoder balancing until the first RFID tag is read.
 *
 * @return true when an RFID tag is detected, false if timeout/stop/kill occurs.
 */
bool driveForwardUntilFirstRfid() {
  resetEncoders();
  const uint32_t start = millis();
  uint32_t lastPrintMs = 0;

  while (millis() - start < kRfidSearchTimeoutMs) {
    handleSerialCommands();
    updateWifi();
    if (serialStopped || handleStartStopButtonEvent()) {
      stopMotors();
      return false;
    }

    String uid;
    if (pollRfid(&uid, true)) {
      lastUid = uid;
      stopMotors();
      Serial.print(F("[RFID] first outside tag UID="));
      Serial.println(lastUid);
      return true;
    }

    const long leftAbs = absLong(getLeftCount());
    const long rightAbs = absLong(getRightCount());
    const long diff = leftAbs - rightAbs;
    int correction = static_cast<int>(diff * kStraightCorrectionKp);
    correction = constrain(correction, -kMaxStraightCorrection, kMaxStraightCorrection);
    setTank(kPostTunnelSearchSpeed - correction, kPostTunnelSearchSpeed + correction);

    if (millis() - lastPrintMs >= 250) {
      lastPrintMs = millis();
      Serial.print(F("[RFID SEARCH] L="));
      Serial.print(getLeftCount());
      Serial.print(F(" R="));
      Serial.println(getRightCount());
    }

    updateImu();
    delay(10);
  }

  stopMotors();
  Serial.println(F("[RFID SEARCH] timeout before first RFID tag."));
  return false;
}

// ---------------------------------------------------------------------------
// Post-tunnel fixed grid route
// ---------------------------------------------------------------------------

/**
 * @return true if this UID was already handled recently.
 */
bool isDuplicateRecentGridUid(const String &uid) {
  return uid == lastGridUid && millis() - lastGridUidMs < kGridRfidCooldownMs;
}

/**
 * @return true while grid RFID reads should be ignored after a scripted turn.
 */
bool suppressGridRfidAfterTurn() {
  return lastGridTurnMs != 0 && millis() - lastGridTurnMs < kPostTurnRfidIgnoreMs;
}

/**
 * Drive from the RFID reader position to the hole-center position, then query
 * the server and plant if the cell is fertile and unplanted.
 *
 * @param uid RFID tag UID.
 * @return true when the UID was handled; false if it was a duplicate or stop.
 */
bool handleGridRfidNode(const String &uid) {
  if (isDuplicateRecentGridUid(uid)) {
    return false;
  }

  lastUid = uid;
  lastGridUid = uid;
  lastGridUidMs = millis();

  stopMotors();
  Serial.print(F("[GRID RFID] uid="));
  Serial.println(uid);
  Serial.println(F("[GRID RFID] driving 75 mm to hole center before server query."));

  if (!driveDistanceMm(kGridPlantCenterOffsetMm, kGridPlantOffsetSpeed)) {
    stopMotors();
    return false;
  }

  stopMotors();
  Serial.println(F("[GRID RFID] centered over hole; waiting for isFertileReply."));

  CellStatus status = {};
  const bool gotReply = queryServerForCellStatus(uid, &status);
  const bool eligible = gotReply ? (status.fertile && !status.planted) : kPlantIfNoServerReply;

  if (!eligible) {
    Serial.println(F("[PLANT] not eligible or no server reply; no seed dropped."));
    lastGridUidMs = millis();
    return true;
  }

  if (seedsPlanted >= kMaxSeedsToPlant) {
    Serial.println(F("[PLANT] seed limit reached; route continues without dropping."));
    lastGridUidMs = millis();
    return true;
  }

  dropOneSeed();
  seedsPlanted++;
  notifySeedPlanted(uid);
  Serial.print(F("[PLANT] seedsPlanted="));
  Serial.print(seedsPlanted);
  Serial.print(F("/"));
  Serial.println(kMaxSeedsToPlant);
  lastGridUidMs = millis();
  return true;
}

/**
 * Perform one easy-route turn and reset line controller afterwards.
 *
 * @param dir Turn direction.
 * @return true if the turn completed.
 */
bool performEasyTurn(TurnDir dir) {
  Serial.print(F("[EASY ROUTE] turn "));
  Serial.println(turnName(dir));
  stopMotors();
  delay(120);
  const bool ok = turnDegreesImu(degreesForTurn(dir));
  delay(120);
  resetLineController();
  if (ok) {
    lastGridTurnMs = millis();
  }
  return ok;
}

/**
 * Follow the rediscovered line after the tunnel until the first grid RFID.
 *
 * The first grid RFID is handled as a planting candidate, then the robot turns
 * right and starts the fixed route from "2 RFIDs, left".
 */
void updateFollowLineToFirstGridRfid() {
  String uid;
  if (pollRfid(&uid, true)) {
    if (handleGridRfidNode(uid)) {
      if (!performEasyTurn(TurnDir::Right)) {
        serialStopped = true;
        setState(MissionState::Stopped);
        return;
      }
      easyRouteIndex = 0;
      easySegmentRfidCount = 0;
      setState(MissionState::EasyGridRoute);
      return;
    }
  }

  const LineReading line = readLine();
  if (!line.detected) {
    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  applyLineCommand(line, F("[GRID FIRST]"));
  delay(kLineLoopDelayMs);
}

/**
 * Follow the hard-coded RFID-count route after the tunnel.
 */
void updateEasyGridRoute() {
  if (easyRouteIndex >= kEasyRouteLength) {
    setState(MissionState::EasyAfterDoorForward);
    return;
  }

  if (!suppressGridRfidAfterTurn()) {
    String uid;
    if (pollRfid(&uid, true)) {
      if (handleGridRfidNode(uid)) {
        easySegmentRfidCount++;
        const EasyRouteStep step = kEasyRoute[easyRouteIndex];

        Serial.print(F("[EASY ROUTE] step="));
        Serial.print(easyRouteIndex + 1);
        Serial.print(F("/"));
        Serial.print(kEasyRouteLength);
        Serial.print(F(" count="));
        Serial.print(easySegmentRfidCount);
        Serial.print(F("/"));
        Serial.println(step.rfidCount);

        if (easySegmentRfidCount >= step.rfidCount) {
          easySegmentRfidCount = 0;
          if (!performEasyTurn(step.turnAfter)) {
            serialStopped = true;
            setState(MissionState::Stopped);
            return;
          }

          easyRouteIndex++;
          if (step.requestDoorAfterTurn) {
            Serial.println(F("[EASY ROUTE] final turn complete; requesting airlock with last RFID UID."));
            setState(MissionState::EasyDoorRequest);
            return;
          }
        }
      }
    }
  }

  const LineReading line = readLine();
  if (!line.detected) {
    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  applyLineCommand(line, F("[EASY ROUTE]"));
  delay(kLineLoopDelayMs);
}

/**
 * Send the final openAirlock request after the fixed route.
 */
void updateEasyDoorRequest() {
  stopMotors();
  const String uid = lastGridUid.length() > 0 ? lastGridUid : lastUid;
  if (uid.length() == 0) {
    Serial.println(F("[AIRLOCK] no RFID UID available for return airlock B request; starting return line search anyway."));
    setState(MissionState::ReturnLineToTunnelEntry);
    return;
  }

  if (!easyDoorRequestSent) {
    easyDoorRequestSent = sendAirlockOpenRequest(uid, 'B');
  }

  if (easyDoorRequestSent) {
    Serial.println(F("[AIRLOCK] return airlock B request sent; follow line until tunnel/no-line handoff."));
    setState(MissionState::ReturnLineToTunnelEntry);
    return;
  }

  Serial.println(F("[AIRLOCK] return airlock B request not sent yet; retrying."));
  delay(kAirlockRequestRetryMs);
}

/**
 * Continue forward after the final door request. RFID nodes are still queried
 * and planted if eligible, but no more scripted turns are performed.
 */
void updateEasyAfterDoorForward() {
  String uid;
  if (pollRfid(&uid, true)) {
    handleGridRfidNode(uid);
  }

  const LineReading line = readLine();
  if (!line.detected) {
    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  applyLineCommand(line, F("[AFTER DOOR]"));
  delay(kLineLoopDelayMs);
}

/**
 * After requesting return airlock B, follow the line toward the return tunnel.
 *
 * Handoff to wall following happens when the QTR line disappears for six
 * consecutive frames, or after the same 12 second timeout used on base exit.
 */
void updateReturnLineToTunnelEntry() {
  if (millis() - stateStartMs > kLineToDoorTimeoutMs) {
    stopMotors();
    Serial.println(F("[RETURN] line-to-tunnel timeout after B request; starting return wall follow."));
    resetWallController();
    setState(MissionState::ReturnWallFollowTunnel);
    return;
  }

  const LineReading line = readLine();
  if (!line.detected) {
    if (returnTunnelNoLineCount < 255) returnTunnelNoLineCount++;
    if (returnTunnelNoLineCount >= kTunnelEntryNoLineFrames) {
      stopMotors();
      Serial.println(F("[RETURN] line disappeared for 6 frames; starting return wall follow."));
      resetWallController();
      setState(MissionState::ReturnWallFollowTunnel);
      return;
    }

    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  returnTunnelNoLineCount = 0;
  applyLineCommand(line, F("[RETURN TO TUNNEL]"));
  delay(kLineLoopDelayMs);
}

/**
 * Wall-follow back through the return tunnel.
 *
 * Unlike the outbound tunnel state, this does not go to RFID search when the
 * line reappears. It immediately resumes normal line following back to base.
 */
void updateReturnWallFollowTunnel() {
  const LineReading line = readLine();
  if (line.detected) {
    if (returnWallExitLineStableCount < 255) returnWallExitLineStableCount++;
    if (returnWallExitLineStableCount >= kReturnWallExitLineStableFrames) {
      stopMotors();
      Serial.println(F("[RETURN] QTR line found; leaving wall following and following line back to base."));
      setState(MissionState::ReturnFollowLineToBase);
      return;
    }
  } else {
    returnWallExitLineStableCount = 0;
  }

  runWallFollowStep();
}

/**
 * Follow the rediscovered line back toward base.
 */
void updateReturnFollowLineToBase() {
  const LineReading line = readLine();
  applyLineCommand(line, F("[RETURN BASE]"));
  delay(kLineLoopDelayMs);
}

// ---------------------------------------------------------------------------
// Mission state handlers
// ---------------------------------------------------------------------------

/**
 * Follow the selected scripted base route until all four route turns are complete.
 */
void updateFollowBaseRoute() {
  if (routeTurnIndex >= kAirlockRfidCheckFromTurnIndex) {
    checkBaseExitRfidAndRequestAirlock();
  }

  const LineReading line = readLine();
  if (routeEventReady(line)) {
    if (!performRouteTurn()) {
      serialStopped = true;
      setState(MissionState::Stopped);
      return;
    }

    if (routeTurnIndex >= kRouteTurnCount) {
      Serial.print(F("[ROUTE] complete route="));
      Serial.print(routeName(routeChoice));
      Serial.println(F("; following line to tunnel entry."));
      setState(MissionState::FollowLineToTunnelEntry);
      return;
    }
  } else {
    const LineReading followLine = softenedPostTurnLine(line);
    applyLineCommand(followLine, F("[BASE]"));
    updatePostTurnHardIgnore(line);
  }
}

/**
 * Follow the exit line until it disappears at the tunnel entry.
 *
 * The A door should already be opening from the RFID/server request before the
 * robot arrives, so this segment no longer uses front sonar as a collision
 * stop. The tunnel itself has no line; several all-white QTR frames are used as
 * the handoff point into wall following.
 */
void updateFollowLineToTunnelEntry() {
  if (checkBaseExitRfidAndRequestAirlock()) {
    return;
  }

  if (millis() - stateStartMs > kLineToDoorTimeoutMs) {
    stopMotors();
    Serial.println(F("[WARN] line-to-tunnel timeout; starting wall follow from current position."));
    resetWallController();
    setState(MissionState::WallFollowTunnel);
    return;
  }

  const LineReading line = readLine();
  if (!line.detected) {
    if (tunnelEntryNoLineCount < 255) tunnelEntryNoLineCount++;
    if (tunnelEntryNoLineCount >= kTunnelEntryNoLineFrames) {
      stopMotors();
      if (!airlockRequestSent) {
        Serial.println(F("[WARN] tunnel entry reached before base-exit RFID was read; airlock request not sent."));
      }
      Serial.println(F("[TUNNEL] base line ended; starting wall following with initial IMU calibration."));
      resetWallController();
      setState(MissionState::WallFollowTunnel);
      return;
    }

    setTank(kTunnelEntryConfirmSpeed, kTunnelEntryConfirmSpeed);
    updateImu();
    delay(kLineLoopDelayMs);
    return;
  }

  tunnelEntryNoLineCount = 0;
  const LineReading followLine = softenedPostTurnLine(line);
  applyLineCommand(followLine, F("[TO TUNNEL]"));
  updatePostTurnHardIgnore(line);
  checkBaseExitRfidAndRequestAirlock();
}

/**
 * Follow the tunnel wall until the front sonar detects the far door.
 */
void updateWallFollowTunnel() {
  if (millis() - stateStartMs > kWallTunnelTimeoutMs) {
    stopMotors();
    Serial.println(F("[WARN] wall-follow tunnel timeout; switching to line/RFID search from current position."));
    setState(MissionState::FollowLineToFirstGridRfid);
    return;
  }

  const LineReading exitLine = readLine();
  if (exitLine.detected) {
    if (wallExitLineStableCount < 255) wallExitLineStableCount++;
    if (wallExitLineStableCount >= kWallExitLineStableFrames) {
      stopMotors();
      Serial.println(F("[TUNNEL EXIT] QTR line found; leaving wall following and following line to first grid RFID."));
      setState(MissionState::FollowLineToFirstGridRfid);
      return;
    }
  } else {
    wallExitLineStableCount = 0;
  }

  const DoorReading door = readDoor();
  const bool closed = doorClosedStable(door);
  printDoorStatus(F("[EXIT DOOR APPROACH]"), door);
  if (closed) {
    stopMotors();
    Serial.println(F("[EXIT DOOR] closed door detected; wall following cancelled."));
    setState(MissionState::WaitExitDoorOpen);
    return;
  }

  runWallFollowStep();
}

/**
 * Stop at the far tunnel door until it opens.
 */
void updateWaitExitDoorOpen() {
  stopMotors();
  const DoorReading door = readDoor();
  const bool opened = doorOpenStable(door);
  printDoorStatus(F("[EXIT DOOR WAIT]"), door);

  if (opened) {
    Serial.println(F("[EXIT DOOR] open; following line to first grid RFID when available."));
    setState(MissionState::FollowLineToFirstGridRfid);
  }
  delay(20);
}

/**
 * Drive forward without wall following until the first outside RFID tag is read.
 */
void updateSearchFirstRfid() {
  if (driveForwardUntilFirstRfid()) {
    setState(MissionState::AlignOverRfid);
  } else {
    setState(MissionState::Stopped);
  }
}

/**
 * Drive the final alignment offset after RFID detection.
 */
void updateAlignOverRfid() {
  if (rfidAlignOffsetMm > 0.0f) {
    driveDistanceMm(rfidAlignOffsetMm, kRfidAlignSpeed);
  }
  stopMotors();
  Serial.println(F("[DONE] aligned over first outside RFID spot."));
  setState(MissionState::Done);
}

/**
 * Top-level mission state machine.
 */
void updateMission() {
  handleSerialCommands();
  updateWifi();
  servicePendingAirlockRequest();

  if (handleStartStopButtonEvent()) {
    delay(50);
    return;
  }

  if (serialStopped && missionState != MissionState::Stopped) {
    stopMotors();
    setState(MissionState::Stopped);
    return;
  }

  switch (missionState) {
    case MissionState::Idle:
      stopMotors();
      delay(20);
      break;

    case MissionState::FollowBaseRoute:
      updateFollowBaseRoute();
      break;

    case MissionState::FollowLineToTunnelEntry:
      updateFollowLineToTunnelEntry();
      break;

    case MissionState::WallFollowTunnel:
      updateWallFollowTunnel();
      break;

    case MissionState::WaitExitDoorOpen:
      updateWaitExitDoorOpen();
      break;

    case MissionState::SearchFirstRfid:
      updateSearchFirstRfid();
      break;

    case MissionState::AlignOverRfid:
      updateAlignOverRfid();
      break;

    case MissionState::FollowLineToFirstGridRfid:
      updateFollowLineToFirstGridRfid();
      break;

    case MissionState::EasyGridRoute:
      updateEasyGridRoute();
      break;

    case MissionState::EasyDoorRequest:
      updateEasyDoorRequest();
      break;

    case MissionState::EasyAfterDoorForward:
      updateEasyAfterDoorForward();
      break;

    case MissionState::ReturnLineToTunnelEntry:
      updateReturnLineToTunnelEntry();
      break;

    case MissionState::ReturnWallFollowTunnel:
      updateReturnWallFollowTunnel();
      break;

    case MissionState::ReturnFollowLineToBase:
      updateReturnFollowLineToBase();
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
 * Initialize the top Modulino Pixels used for mission status.
 */
void initializePixels() {
  Wire.begin();
  Modulino.begin(Wire);
  pixelsOk = pixels.begin();
  Serial.print(F("[LED] Modulino Pixels="));
  Serial.println(pixelsOk ? F("OK") : F("NOT FOUND"));
  setPixelsNormalBlue();
}

/**
 * Initialize sensors and calibrate IMU only after the robot is placed down.
 *
 * The robot may be hand-carried before the start button is pressed, so QTR,
 * sonar, RFID, and IMU setup are delayed until mission start. Motoron is still
 * initialized in setup so the motors can be commanded to a safe stop.
 *
 * @return true if the mission can start, false if start was cancelled.
 */
bool initializeMissionSensorsForRun() {
  stopMotors();

  if (!missionSensorsInitialized) {
    Serial.println(F("[INIT] initializing sensors after start button."));

    pinMode(kFrontTrigPin, OUTPUT);
    pinMode(kLeftTrigPin, OUTPUT);
    pinMode(kRightTrigPin, OUTPUT);
    pinMode(kFrontEchoPin, INPUT);
    pinMode(kLeftEchoPin, INPUT);
    pinMode(kRightEchoPin, INPUT);
    digitalWrite(kFrontTrigPin, LOW);
    digitalWrite(kLeftTrigPin, LOW);
    digitalWrite(kRightTrigPin, LOW);

    pinMode(kServoPin, OUTPUT);
    if (kResetServoAtStartup) {
      moveServoToAngle(kServoMinAngle);
      holdServoAngle(kServoMinAngle, 500);
    }

    pinMode(kLeftEncoderAPin, INPUT_PULLUP);
    pinMode(kLeftEncoderBPin, INPUT_PULLUP);
    pinMode(kRightEncoderAPin, INPUT_PULLUP);
    pinMode(kRightEncoderBPin, INPUT_PULLUP);
    if (!encoderInterruptsAttached) {
      attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), leftEncoderIsr, RISING);
      attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), rightEncoderIsr, RISING);
      encoderInterruptsAttached = true;
    }

    Wire.begin();
    initializeQtr();
    initializeRfid();
    initializeImuHardware();

    missionSensorsInitialized = true;
  }

  if (!waitForWifiBeforeCalibration()) {
    return false;
  }

  resetEncoders();
  resetLineController();
  resetWallController();
  lastEventMs = millis();
  lastWallUpdateMs = millis();

  if (imuOk && !calibrateImuGyroBias()) {
    return false;
  }

  return !serialStopped;
}

/**
 * Arduino setup entry point.
 */
void setup() {
  Serial.begin(kSerialBaud);
  delay(1500);

  Serial.println(F("=== Base_Exit_Tunnel_Wall_Test ==="));

  if (kUseKillPin) {
    pinMode(kKillPin, INPUT_PULLUP);
    lastKillReading = digitalRead(kKillPin);
    stableKillReading = lastKillReading;
    lastKillChangeMs = millis();
  }

  initializePixels();
  initializeWifi();

  Wire1.begin();
  initializeMotoron();

  Serial.print(F("[INIT] encoder counts/mm="));
  Serial.println(kEncoderCountsPerMm, 4);
  printHelp();
  printSettings();

  if (kStartOnBoot) {
    restartMission();
  } else {
    setState(MissionState::Idle);
  }
}

/**
 * Arduino loop entry point.
 */
void loop() {
  updateMission();
}
