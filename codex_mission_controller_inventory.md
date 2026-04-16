# MissionController Inventory

## Scope

This note records the current responsibility inventory for `MissionController` in
`MazeMap/MazeMap/MazeMapMissionController.cpp` and evaluates whether each area
belongs there under the current cleanup rules.

The assessment below incorporates these architectural constraints:

- `MissionController` should be the owner of mission-mode policy, not a sink for shared infrastructure.
- Logging, maneuver execution, sensor calibration, and fault handling are infrastructure and should be presented through `SharedRobotRuntime` or its composed members.
- The maze-solving stack is system-wide infrastructure: the authoritative `Maze`, maze update/fusion logic, pathfinders, maze-aware maneuver procedures, and systems that merge maze knowledge with sensor readings should not be mission-owned.
- Utility/test/diagnostic workflows should become dedicated boot-selected modes over the shared infrastructure rather than methods on the mission owner.
- `LoopController` remains the sole cadence owner; mission code may use callback swapping inside loop sessions but must not regain a public per-tick control model.

## Inventory

### 1. Boot-mode host multiplexing

- Current code:
  - `MazeMap/MazeMap/MazeMapMissionModeHost.h`
  - `BeginMissionRunMode`, `BeginManeuverFileTestMode`, `BeginCorridorRepeatabilityMode`, `BeginPositionAccuracyAuditMode`
- Why it is currently here:
  - One host object was used as the sink for mission mode plus several utility modes.
- Assessment:
  - This should not stay in its current shape.
  - `MissionController` may remain the owner of mission-run mode only.
  - Maneuver-file test, corridor repeatability, and position accuracy audit should become separate mode owners.

### 2. Top-level mission policy

- Current code:
  - `RunMissionRunMode`
  - `ExploreFullMaze`
  - `ReturnToStart`
  - `ExecuteRacingRunCycle`
- Why it is currently here:
  - This is the actual mission workflow and phase ordering.
- Assessment:
  - This should stay.
  - It is the core legitimate responsibility of the class.
  - The class should decide what the robot must do next, not re-own the shared subsystems used to do it.

### 3. Runtime references and mission-local flags

- Current code:
  - `_runtime`
  - `_goalPauseComplete`
  - `_missionComplete`
  - `_faulted`
  - `_currentCell`, `_currentDirection`, `_currentDirectionalLocation`
- Why it is currently here:
  - The class was built as the integration point over drive, sensors, maze, pathfinders, and mission progress.
- Assessment:
  - Only `_runtime` and the small mission-progress flags clearly belong here.
  - Discrete navigation state such as current cell, direction, and directional location should not remain mission-owned if maze/navigation becomes shared runtime infrastructure.

### 4. LoopController session dispatch and per-motion loop state

- Current code:
  - `BuildLoopOptions`
  - `ActiveLoopThunk`
  - `TransitionLoopPhase`
  - `RunLoopSession`
  - all `*LoopState` structs
- Why it is currently here:
  - The class needed a way to use `LoopController` without reintroducing a per-tick control API.
- Assessment:
  - The cadence-preserving pattern is correct.
  - The machinery should not stay in `MissionController` long-term.
  - It belongs in runtime-owned maneuver/navigation execution infrastructure presented through `SharedRobotRuntime` or one of its composed members.

### 5. Logging, telemetry, trace emission, and fault handling

- Current code:
  - `BeginTelemetryLog`
  - `WriteTelemetryEvent`
  - `EmitMissionControllerLine`
  - `BeginTelemetryPhase`
  - `Fail`
  - `OnRuntimeFault`
- Why it is currently here:
  - The class accumulated its own mode logging and failure plumbing while hosting multiple workflows.
- Assessment:
  - This should not stay as mission-owned behavior.
  - Logging and fault handling are explicitly shared runtime infrastructure.
  - Mission code should request logging/fault services from `SharedRobotRuntime`; it should not own parallel policy around them.

### 6. Sensor calibration and startup wall calibration

- Current code:
  - `RunStartupWallCalibration`
  - front sweep capture
  - wall-touch-based coordinate capture
  - front open-scene baseline capture
- Why it is currently here:
  - Mission startup currently performs calibration directly before exploration.
- Assessment:
  - This should not stay as mission-owned implementation.
  - Mission mode may require calibration completion, but the calibration procedure itself is shared infrastructure and should be reusable from dedicated calibration/diagnostic modes.

### 7. Front-wall characterization loading, matching, and baseline capture

- Current code:
  - `LoadPersistedFrontWallCharacterization`
  - `TryApplyFrontWallCharacterizationToObservation`
  - front characterization capture/storage helpers
- Why it is currently here:
  - Mission observation wants characterized front-wall sensing.
- Assessment:
  - This should not stay in `MissionController`.
  - It is sensing/calibration infrastructure and should be shared across mission and diagnostic modes.

### 8. Maze update, wall-belief fusion, and observation interpretation

- Current code:
  - `CaptureStationaryObservationSnapshot`
  - `ObserveCellFromSnapshot`
  - `ObserveCurrentCell`
  - `HandleSearchWallMapUpdateStop`
- Why it is currently here:
  - The class currently owns the search loop, so it also owns map updates and fusion.
- Assessment:
  - This should not stay.
  - Maze update and sensor/maze fusion are explicitly system-wide infrastructure.

### 9. Pathfinding, maze-aware search execution, and return/speed-run planning

- Current code:
  - `SearchStraightLoopTick`
  - `ExecuteSearchStraightCellsLoopDriven`
  - `ExecuteSearchPath`
  - speed-run path execution
- Why it is currently here:
  - Mission mode currently plans and executes its own navigation directly.
- Assessment:
  - This should not stay.
  - Pathfinders and maze-aware navigation procedures are shared runtime infrastructure and should be callable from mission and diagnostic modes alike.

### 10. Maneuver execution primitives

- Current code:
  - `HoldLoopTick`
  - `SettleLoopTick`
  - `ReverseStraightLoopTick`
  - `StraightLoopTick`
  - `TurnLoopTick`
  - `ArcLoopTick`
  - `SmoothTurnLoopTick`
  - `ExecuteQueuedManeuvers`
- Why it is currently here:
  - Mission mode needed to move the robot, so the executor accumulated here.
- Assessment:
  - This should not stay.
  - Maneuver execution is explicitly shared infrastructure and should be exposed through runtime-owned execution services.
  - The need for dedicated test/diagnostic modes for these behaviors is additional evidence that they are not mission-owned.

### 11. Map-grounded pose correction and wall-centering helpers

- Current code:
  - `TryResolveMapQualifiedSideWallReference`
  - `TryComputeWallGroundedCorridorErrorM`
  - `ApplyWallGroundedCorridorPoseCorrection`
  - `ApplyTurnWallEdgePoseCorrection`
- Why it is currently here:
  - These helpers were added as part of search and turn execution.
- Assessment:
  - This should not stay.
  - These are maze-aware motion/fusion procedures and belong with shared navigation/execution infrastructure.

### 12. Utility-mode audit fixtures and result writers

- Current code:
  - corridor repeatability metadata/results
  - position accuracy audit metadata/results
  - audit fixture construction helpers
  - maneuver-file test helpers
- Why it is currently here:
  - The class became the dumping ground for bring-up and audit workflows.
- Assessment:
  - This should not stay.
  - These should be separate testing/diagnostic mode owners built over shared infrastructure, not methods on the mission owner.

## Conclusion

`MissionController` should collapse toward a thin mission-policy owner.

It should remain responsible for:

- selecting and ordering mission phases,
- mission-run success/failure progression,
- mission-specific high-level decisions such as when to explore, pause at goal, return to start, or launch a speed run.

It should stop owning:

- logging,
- fault handling,
- calibration,
- maneuver execution,
- maze updates and fusion,
- pathfinding,
- maze-aware navigation execution,
- utility/test/diagnostic workflows.

Those responsibilities should move behind `SharedRobotRuntime` or its composed authoritative members so that mission mode and dedicated diagnostic modes can consume the same infrastructure instead of forking it.
