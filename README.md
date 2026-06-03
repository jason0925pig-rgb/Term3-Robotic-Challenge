## Complete Easy-Mission State Machine

````mermaid
stateDiagram-v2
    [*] --> Init

    Init --> ExitBase
    ExitBase --> TurnRightToStartRoute
    TurnRightToStartRoute --> ReadRFID

    ReadRFID --> QueryServer: RFID detected
    ReadRFID --> Error: RFID read failed

    QueryServer --> CheckPlantingConditions: server response received
    QueryServer --> Error: server query failed

    CheckPlantingConditions --> PlantSeed: fertile && not already planted && seedsRemaining > 0
    CheckPlantingConditions --> CheckRouteComplete: do not plant

    PlantSeed --> NotifyServerSeedPlanted
    NotifyServerSeedPlanted --> CheckRouteComplete: update successful
    NotifyServerSeedPlanted --> Error: update failed

    CheckRouteComplete --> Finished: current cell is final route cell
    CheckRouteComplete --> SelectNextCell: more cells remain

    SelectNextCell --> CalculateRequiredHeading
    CalculateRequiredHeading --> TurnToNextCell
    TurnToNextCell --> SelectMovementMode

    SelectMovementMode --> FollowLineToNextRFID: next cell is in bottom half
    SelectMovementMode --> DriveStraightToNextRFID: next cell is in top half

    FollowLineToNextRFID --> ReadRFID: next RFID reached
    FollowLineToNextRFID --> Error: movement/RFID failure

    DriveStraightToNextRFID --> ReadRFID: next RFID reached
    DriveStraightToNextRFID --> Error: movement/RFID failure

    Finished --> StopRobot
    Error --> StopRobot

    StopRobot --> [*]
```
````

### A slightly cleaner version, with the planting and movement decisions made more explicit:

Easy-Mission State Machine With Decision Nodes

```mermaid
stateDiagram-v2
    [*] --> Init

    Init --> ExitBase
    ExitBase --> TurnRight
    TurnRight --> AtCell

    AtCell --> ReadRFID
    ReadRFID --> QueryServer

    QueryServer --> PlantDecision

    PlantDecision --> PlantSeed: plant allowed
    PlantDecision --> RouteDecision: plant not allowed

    PlantSeed --> UpdateServer
    UpdateServer --> RouteDecision

    RouteDecision --> Finished: final cell reached
    RouteDecision --> NextCell: route continues

    NextCell --> ComputeDirection
    ComputeDirection --> TurnToDirection
    TurnToDirection --> MovementDecision

    MovementDecision --> LineFollowMove: bottom half
    MovementDecision --> StraightMove: top half

    LineFollowMove --> AtCell: next RFID reached
    StraightMove --> AtCell: next RFID reached

    ReadRFID --> Error: RFID failed
    QueryServer --> Error: server failed
    UpdateServer --> Error: update failed
    LineFollowMove --> Error: movement failed
    StraightMove --> Error: movement failed

    Finished --> StopRobot
    Error --> StopRobot
    StopRobot --> [*]
```

```md



```
