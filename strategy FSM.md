# simple & medium

### over all strategy

```mermaid
stateDiagram-v2
    [*] --> ExitTunnel

    state "Exit tunnel" as ExitTunnel
    state "Go to A9" as GoA9
    state "Snake scan field" as SnakeScan
    state "Process current RFID" as ProcessRFID
    state "Check mission done" as CheckDone
    state "Handle rescue task" as RescueTask
    state "Go to base door" as GoBaseDoor
    state "Request base door" as RequestDoor
    state "Wait for door reply" as WaitDoor
    state "Report brightest LDR area" as ReportBrightest
    state "Enter base" as EnterBase
    state "Mission complete" as Done

    ExitTunnel --> GoA9: robot leaves tunnel
    GoA9 --> SnakeScan: arriveAt(A9)

    SnakeScan --> ProcessRFID: RFID tag detected
    ProcessRFID --> CheckDone: point handled

    CheckDone --> GoBaseDoor: at bottom right OR map known AND seedsPlanted == 5
    CheckDone --> RescueTask: strandedList not empty
    CheckDone --> SnakeScan: continue scan

    RescueTask --> SnakeScan: rescue search finished

    GoBaseDoor --> RequestDoor: arrive at base entrance
    RequestDoor --> WaitDoor: sendOpenDoorRequest()
    WaitDoor --> ReportBrightest: server confirms door open
    ReportBrightest --> EnterBase: send brightest LDR reading / grid point
    EnterBase --> Done
    Done --> [*]

    SnakeScan --> GoBaseDoor: emergency signal
    ProcessRFID --> GoBaseDoor: emergency signal
    RescueTask --> GoBaseDoor: emergency signal
```

### snake scan FSM

```mermaid
stateDiagram-v2
    [*] --> AtA9

    state "At A9" as AtA9
    state "Scan row left" as ScanLeft
    state "Move down at left edge" as DownLeft
    state "Scan row right" as ScanRight
    state "Move down at right edge" as DownRight
    state "Next RFID point" as NextPoint
    state "Sweep finished" as SweepFinished

    AtA9 --> ScanLeft: face toward A8

    ScanLeft --> NextPoint: followLineForward()
    NextPoint --> ScanLeft: next point is same row leftward
    ScanLeft --> DownLeft: reached column 1

    DownLeft --> ScanRight: moveDownOneRow()

    ScanRight --> NextPoint: followLineForward()
    NextPoint --> ScanRight: next point is same row rightward
    ScanRight --> DownRight: reached column 9

    DownRight --> ScanLeft: moveDownOneRow()

    ScanLeft --> SweepFinished: all rows scanned
    ScanRight --> SweepFinished: all rows scanned
    SweepFinished --> [*]
```

### RFID processing FSM

```mermaid
stateDiagram-v2
    [*] --> ReadTag

    state "Read RFID tag" as ReadTag
    state "Get latest map" as SyncMap
    state "Check explored status" as CheckExplored
    state "Ask server for point info" as QueryPoint
    state "Read known point info" as ReadKnown
    state "Check planting condition" as CheckPlant
    state "Plant seed" as PlantSeed
    state "Continue scanning" as ContinueScan

    ReadTag --> SyncMap: tagId available
    SyncMap --> CheckExplored: map received

    CheckExplored --> QueryPoint: point not explored
    CheckExplored --> ReadKnown: point already explored

    QueryPoint --> CheckPlant: point info received
    ReadKnown --> CheckPlant: point info loaded

    CheckPlant --> PlantSeed: fertile AND not planted AND seedsLeft > 0
    CheckPlant --> ContinueScan: not fertile OR already planted

    PlantSeed --> ContinueScan: planting finished
    ContinueScan --> [*]
```

### Rescue FSM

```mermaid
stateDiagram-v2
    [*] --> GetStrandedInfo

    state "Get stranded robot last position" as GetStrandedInfo
    state "Compare row position" as CompareRow
    state "Continue scan until same column" as ReachSameColumn
    state "Turn upward" as TurnUp
    state "Drive to stranded point" as DriveToStranded
    state "Continue scan until stranded point" as ReachStrandedPoint
    state "Search rescue target" as SearchRescue
    state "Return to scan route" as ReturnToScan

    GetStrandedInfo --> CompareRow: last position known

    CompareRow --> ReachSameColumn: stranded row is above robot
    ReachSameColumn --> TurnUp: currentColumn == strandedColumn
    TurnUp --> DriveToStranded: turnLeftOrRightToFaceUp()
    DriveToStranded --> SearchRescue: arrive near last position

    CompareRow --> ReachStrandedPoint: stranded row is below robot
    ReachStrandedPoint --> SearchRescue: arrive at last known position

    SearchRescue --> ReturnToScan: rescue search finished
    ReturnToScan --> [*]
```

# hard&harder strategy

```mermaid
stateDiagram-v2
    [*] --> EnterField

    state "Enter field from tunnel" as EnterField
    state "Turn right" as TurnRight
    state "Wall follow forward" as WallFollow
    state "Front obstacle detected?" as CheckObstacle
    state "At base return tunnel?" as CheckBaseTunnel
    state "Turn left" as TurnLeft
    state "Request base door" as RequestDoor
    state "Wait for server response" as WaitDoor
    state "Report brightest LDR area" as ReportBrightest
    state "Enter base" as EnterBase
    state "Mission complete" as Done

    EnterField --> TurnRight: field reached
    TurnRight --> WallFollow: turnRight()

    WallFollow --> CheckBaseTunnel: periodically check position / RFID / tunnel marker
    CheckBaseTunnel --> RequestDoor: at base tunnel entrance
    CheckBaseTunnel --> CheckObstacle: not at base tunnel

    CheckObstacle --> TurnLeft: frontObstacleDetected()
    CheckObstacle --> WallFollow: path clear

    TurnLeft --> WallFollow: turnLeft()

    RequestDoor --> WaitDoor: sendOpenDoorRequest()
    WaitDoor --> ReportBrightest: server confirms door open
    ReportBrightest --> EnterBase: send brightest LDR reading / grid point
    EnterBase --> Done
    Done --> [*]
```

#### Seed planting is same as simple&medium; rescue signal is ignored.
