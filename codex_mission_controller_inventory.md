# MissionController Inventory

## Scope

This note records the current responsibility inventory for the mission-mode
control stack in `MazeMap/MazeMap/MissionModeController.cpp` and evaluates
whether each area belongs there under the current cleanup rules. The file now
exports direct boot-mode owners, and the utility-mode fixture/result helpers
now live in file-local support functions, but the shared
`MissionModeController` implementation still carries most of the execution
responsibilities inventoried below.

The assessment below incorporates these architectural constraints:

- `MissionController` should be the owner of mission-mode policy, not a sink for shared infrastructure.
- Logging, maneuver execution, sensor calibration, and fault handling are infrastructure and should be presented through `SharedRobotRuntime` or its composed members.
- The maze-solving stack is system-wide infrastructure: the authoritative `Maze`, maze update/fusion logic, pathfinders, maze-aware maneuver procedures, and systems that merge maze knowledge with sensor readings should not be mission-owned.
- Utility/test/diagnostic workflows should become dedicated boot-selected modes over the shared infrastructure rather than methods on the mission owner.
- `LoopController` remains the sole cadence owner; mission code may use callback swapping inside loop sessions but must not regain a public per-tick control model.
- The selected top-level mode owner must be the only code that starts the active `LoopController` session; nested `RunLoopSession(...)` launches inside mission helpers are architectural debt, not an acceptable steady state.
- Waiting for later ticks, operator input, or settle work outside a `RequestPause(...)` callback is not allowed.
- `ManeuverInstance` is the canonical maneuver execution vocabulary; peeled-off `SmoothTurnExecutionProfile`-style helpers are non-authoritative.

## Inventory

### 1. Boot-mode host multiplexing

- Current code:
  - file-local `MissionRunModeOwner`, `ManeuverFileTestModeOwner`, `CorridorRepeatabilityModeOwner`, and `PositionAccuracyAuditModeOwner`
  - shared `MissionModeController` methods `BeginMissionRunMode`, `BeginManeuverFileTestMode`, `BeginCorridorRepeatabilityMode`, and `BeginPositionAccuracyAuditMode`
- Why it is currently here:
  - The stale `IMissionModeHost` wrapper layer was removed, but the four boot-mode owners still delegate into one shared controller implementation.
- Assessment:
  - The public host-multiplexing shape has been removed.
  - The remaining issue is internal ownership: `MissionModeController` still carries mission plus utility-mode behavior.
  - Mission mode should remain the owner of mission-run policy only; maneuver-file test, corridor repeatability, and position accuracy audit should continue moving toward distinct owners over shared infrastructure.

### 2. Top-level mission policy

- Current code:
  - `RunMissionRunMode`
  - `ExploreFullMaze`
  - `ReturnToStart`
  - `ExecuteRacingRunCycle`
- Why it is currently here:
  - This is the actual mission workflow and phase ordering.
- Assessment:
  - This should stay as mission policy, but it should become callback-owned phase logic rather than outer-stack sequencing around nested loop sessions.
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
  - The top-level mode object should own the one session for the whole mode lifetime; mission helpers should contribute callbacks and state, not nested session launches.
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
  - `ManeuverInstance` should be the execution input vocabulary. `SmoothTurnExecutionProfile` and similar peeled-off geometry bags should disappear rather than being preserved behind wrappers.
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
  - file-local corridor repeatability metadata/result helpers
  - file-local position accuracy audit metadata/result helpers
  - file-local audit fixture construction helpers
  - maneuver-file test helpers
- Why it is currently here:
  - The fixed-fixture and result-formatting pieces were moved out of `MissionModeController`, but the surrounding utility workflows still run through the shared controller.
- Assessment:
  - The extracted file-local helpers are an improvement: the mission controller no longer owns the fixed utility geometry and audit result/metadata formatting bodies directly.
  - The remaining issue is workflow ownership. Corridor repeatability, position accuracy audit, and maneuver-file test still execute through `MissionModeController` instead of dedicated diagnostic owners over shared runtime infrastructure.

## Conclusion

The old boot-mode host wrapper concern is resolved, and the utility-specific
fixture/result support is no longer implemented directly on
`MissionModeController`, but the shared controller still needs to collapse
toward a thin mission-policy owner.

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

The immediate convergence targets are therefore:

- move session ownership outward to the top-level mode owner,
- reshape the mission flow into callback-owned phases such as mapping/search/racing callbacks instead of outer-stack mini-sessions,
- remove waits outside `RequestPause(...)` callbacks,
- converge maneuver execution on `ManeuverInstance`,
- and move shared motion execution toward `DriveBase`-rooted owners instead of preserving it inside `MissionModeController`.
