#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MFRC522_I2C.h>
#include <Arduino_Modulino.h>

#if __has_include("../secrets.h")
#define HAS_TUNNEL_WIFI_SECRETS 1
#else
#define HAS_TUNNEL_WIFI_SECRETS 0
#endif

#ifndef USE_WIFI_AIRLOCK_REQUEST
#define USE_WIFI_AIRLOCK_REQUEST HAS_TUNNEL_WIFI_SECRETS
#endif

#if USE_WIFI_AIRLOCK_REQUEST
#include <MiniMessenger.h>
#include "../secrets.h"
#endif

// Hard-mode route and mission tuning. Shared motor, encoder, QTR, sonar, line PID,
// wall PID, and IMU constants come from ../constants.h.
constexpr bool kStartOnBoot = false;

enum class RouteChoice {
  BaseA_Bottom,
  BaseB_Top
};

constexpr RouteChoice kDefaultRouteChoice = RouteChoice::BaseA_Bottom;
constexpr uint8_t kRouteTurnCount = 4;
constexpr float kFirstTAdvanceMm = 52.0f;
constexpr float kSharpTurnAdvanceMm = 52.0f;
constexpr int kRouteAdvanceSpeed = 300;
constexpr float kDefaultRfidAlignOffsetMm = 0.0f;
constexpr uint32_t kStopOverAirlockTagMs = 800;
constexpr uint8_t kAirlockRfidCheckFromTurnIndex = 2;

constexpr float kDoorClosedThresholdMm = 170.0f;
constexpr float kDoorOpenThresholdMm = 320.0f;
constexpr uint8_t kDoorStableFrames = 3;
constexpr bool kTreatNoEchoAsOpen = true;
constexpr uint32_t kDoorPrintIntervalMs = 250;
constexpr uint32_t kExitDoorObstacleWaitMs = 8000;

constexpr uint32_t kLineToDoorTimeoutMs = 12000;
constexpr uint32_t kWallTunnelTimeoutMs = 25000;
constexpr uint32_t kRfidSearchTimeoutMs = 20000;
constexpr uint8_t kTunnelEntryNoLineFrames = 6;
constexpr int kTunnelEntryConfirmSpeed = 260;
constexpr int kPostTunnelSearchSpeed = 300;
constexpr int kRfidAlignSpeed = 260;

constexpr uint8_t kWallExitLineStableFrames = 2;
constexpr uint8_t kReturnWallExitLineStableFrames = 1;
constexpr float kGridPlantCenterOffsetMm = 100.0f;
constexpr int kGridPlantOffsetSpeed = 360;
constexpr bool kPlantIfNoServerReply = false;
constexpr uint8_t kMaxSeedsToPlant = 5;
constexpr uint32_t kServerReplyTimeoutMs = 900;
constexpr uint32_t kGridRfidCooldownMs = 1300;
constexpr uint32_t kPostTurnRfidIgnoreMs = 1000;

constexpr uint8_t kServoPin = 33;
constexpr int kServoMinUs = 500;
constexpr int kServoMaxUs = 2500;
constexpr int kServoMinAngle = 0;
constexpr int kServoMaxAngle = 300;
constexpr int kServoStepAngle = 60;
constexpr uint32_t kServoMoveSettleMs = 600;
constexpr uint32_t kServoHoldAfterDropMs = 1200;
constexpr uint32_t kServoFrameUs = 20000;
constexpr bool kResetServoAtStartup = true;

constexpr uint8_t kFirstTMinActiveSensors = 7;
constexpr uint8_t kFirstTStableFrames = 3;
constexpr float kFirstTMinTravelMm = 80.0f;
constexpr uint16_t kFirstTEdgeStrongThreshold = 650;
constexpr uint16_t kFirstTMinTotalStrength = 5600;
constexpr uint8_t kFirstTMiddleMinActiveSensors = 4;
constexpr uint32_t kFirstTEvidenceWindowMs = 180;
constexpr int kFirstTConfirmSpeed = 220;
constexpr uint8_t kSharpTurnStableFrames = 1;
constexpr uint32_t kEventCooldownMs = 500;
constexpr int kReacquireTurnSpeed = 150;
constexpr uint32_t kReacquireTimeoutMs = 1400;
constexpr uint8_t kReacquireStableFrames = 3;
constexpr uint32_t kPostTurnHardIgnoreMs = 1100;
constexpr uint8_t kPostTurnHardReleaseFrames = 4;
constexpr int kPostTurnSoftErrorClamp = 650;

constexpr WallSide kDefaultWallSide = WallSide::Left;
constexpr bool kStopIfNoWallEcho = false;

constexpr float kHardObstacleAheadThresholdMm = kDoorClosedThresholdMm;
constexpr float kObstacleFrontSafetyStopMm = 20.0f;
constexpr float kObstacleSameSideDistanceToleranceMm = 35.0f;
constexpr float kObstacleClearedSideDistanceIncreaseMm = 90.0f;
constexpr float kObstacleRfidCenteringForwardOffsetMm = 220.0f;
constexpr int kObstacleManoeuvreSpeed = 260;
constexpr uint32_t kObstacleMovementTimeoutMs = 12000;
constexpr uint32_t kObstacleRfidNodeDebounceMs = 900;
constexpr uint8_t kObstaclePostNodeTarget = 3;
constexpr uint8_t kObstacleMaxSidewaysGridSpaces = 4;
constexpr uint8_t kObstacleMaxPassingGridSpaces = 4;
constexpr WallSide kObstacleFallbackSwerveDirection = WallSide::Left;

constexpr uint8_t kRfidAddress = 0x28;
constexpr uint8_t kRfidResetPin = 39;
constexpr uint32_t kRfidPollIntervalMs = 20;
constexpr uint8_t kBaseExitRfidBurstPolls = 4;
constexpr uint16_t kBaseExitRfidBurstGapMs = 5;
constexpr uint32_t kKillDebounceMs = 35;

#if USE_WIFI_AIRLOCK_REQUEST
constexpr const char *kBoardId = "YU7GT";
constexpr uint32_t kRegisterIntervalMs = 5000;
#endif
constexpr uint32_t kAirlockRequestRetryMs = 1000;
constexpr bool kRequireWifiBeforeCalibration = true;
constexpr uint32_t kWifiConnectTimeoutMs = 0;
constexpr uint32_t kWifiConnectPrintIntervalMs = 1000;
constexpr uint8_t kPixelBrightness = 70;

enum class MissionState {
  Idle,
  FollowBaseRoute,
  FollowLineToTunnelEntry,
  WallFollowTunnel,
  WaitExitDoorOpen,
  ObstaclePrepareSwerve,
  ObstacleFindAlignmentTag,
  ObstacleDriveRfidCenterOffset,
  ObstacleTurnAway,
  ObstacleMoveSidewaysAroundObstacle,
  ObstacleTurnParallelToOriginal,
  ObstaclePassObstacle,
  ObstacleTurnBackTowardLine,
  ObstacleReturnToOriginalLineOffset,
  ObstacleRestoreOriginalHeading,
  ObstacleFollowLineAfterObstacle,
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

enum class TurnDir {
  Left,
  Right
};

struct EasyRouteStep {
  uint8_t rfidCount;
  TurnDir turnAfter;
  bool requestDoorAfterTurn;
};

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

MFRC522_I2C rfid(kRfidAddress, kRfidResetPin, &Wire);
ModulinoPixels pixels;

#if USE_WIFI_AIRLOCK_REQUEST
MiniMessenger messenger;
uint32_t lastRegisterMs = 0;
bool wifiInitialized = false;
#endif

MissionState missionState = kStartOnBoot ? MissionState::FollowBaseRoute : MissionState::Idle;
RouteChoice routeChoice = kDefaultRouteChoice;
WallSide wallSide = kDefaultWallSide;
float rfidAlignOffsetMm = kDefaultRfidAlignOffsetMm;

bool rfidOk = false;
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
uint32_t lastFirstTEvidenceMs = 0;
uint8_t doorClosedStableCount = 0;
uint8_t doorOpenStableCount = 0;
uint8_t wallExitLineStableCount = 0;
uint8_t returnTunnelNoLineCount = 0;
uint8_t returnWallExitLineStableCount = 0;
uint8_t easyRouteIndex = 0;
uint8_t easySegmentRfidCount = 0;
uint8_t seedsPlanted = 0;
int currentServoAngle = kServoMinAngle;

MissionState obstacleResumeState = MissionState::FollowLineToFirstGridRfid;
MissionState obstacleCenteringNextState = MissionState::ObstacleTurnAway;
WallSide obstacleSwerveDirection = kObstacleFallbackSwerveDirection;
WallSide obstacleFacingSide = WallSide::Right;
float obstacleHeadingOffsetDeg = 0.0f;
float obstacleReferenceSideMm = -1.0f;
float obstacleCurrentSideMm = -1.0f;
uint8_t obstacleGridSpacesAway = 0;
uint8_t obstacleGridSpacesPassing = 0;
uint8_t obstacleReturnGridSpaces = 0;
uint8_t obstaclePostRfidCount = 0;
bool obstacleCenteringCountsAsSidewaysStep = false;

String lastUid;
String lastGridUid;
String lastObstacleNodeUid;
uint32_t lastGridUidMs = 0;
uint32_t lastObstacleNodeUidMs = 0;
uint32_t lastGridTurnMs = 0;
bool easyDoorRequestSent = false;
bool waitingForCellStatus = false;
CellStatus lastCellStatus;

bool postTurnHardIgnoreActive = false;
uint32_t postTurnHardIgnoreStartMs = 0;
uint8_t postTurnHardReleaseCount = 0;
uint8_t tunnelEntryNoLineCount = 0;

uint32_t lastDoorPrintMs = 0;
uint32_t lastRfidPollMs = 0;
uint32_t lastEventMs = 0;
uint32_t stateStartMs = 0;
