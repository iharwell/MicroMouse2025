# Cross-File Duplicate Responsibility Audit

Date: 2026-04-17

Scope:
- Limited to the 10 largest code files under `MazeMap/MazeMap`.
- This pass looks for duplicated or near-duplicated responsibilities across files, not just copy-paste blocks.
- Vendored code and non-`MazeMap` projects were excluded.

Top-10 files audited:
1. `MazeMap/MazeMap/MissionModeController.cpp` - 299359 bytes
2. `MazeMap/MazeMap/MazeMapRuntimeCore.h` - 184017 bytes
3. `MazeMap/MazeMap/PlantModel.cpp` - 142256 bytes
4. `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` - 121892 bytes
5. `MazeMap/MazeMap/SrUkfCore.cpp` - 105942 bytes
6. `MazeMap/MazeMap/DiagnosticController.cpp` - 79107 bytes
7. `MazeMap/MazeMap/DriveBase.h` - 62928 bytes
8. `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` - 58983 bytes
9. `MazeMap/MazeMap/Defines.h` - 51569 bytes
10. `MazeMap/MazeMap/ManeuverExecutor.cpp` - 49614 bytes

## Findings

### 1. `MissionModeController` still acts as a parallel top-level boot-mode entry surface beside `BootModeRegistry`

Evidence:
- `MazeMap/MazeMap/MissionModeController.h:20-30` exposes `Begin*/Run*` entry points for mission mode and multiple other boot-selected modes.
- `MazeMap/MazeMap/MissionModeController.cpp:63-328` defines internal implementations for `Mission`, `ManeuverFileTest`, `CorridorRepeatability`, and `PositionAccuracyAudit`.
- `MazeMap/MazeMap/MissionModeController.cpp:7365-7402` forwards those public entry points through the wrapper class.
- `MazeMap/MazeMap/BootModeRegistry.cpp:79-123` already defines the authoritative registry entries for those boot-selectable modes.

Why this is duplicate responsibility:
- `BootModeRegistry` already owns the "what top-level modes exist and how they are entered" responsibility.
- `MissionModeController` still publishes a second top-level mode catalog plus direct begin/run entry points for multiple other boot-selected modes.
- That creates two places to discover and route boot-selected modes: the registry and the controller API.

Likely canonical owner:
- `BootModeRegistry` plus the authoritative mode descriptors.
- `MissionModeController` should be limited to mission-mode behavior or become an internal implementation detail behind the registry path.

### 2. Loop-session setup, callback thunks, and continuation wiring are duplicated across controllers

Evidence:
- `MazeMap/MazeMap/DiagnosticController.cpp:98-122` runs its own begin-session / run / cleanup flow.
- `MazeMap/MazeMap/DiagnosticController.cpp:387-459` defines `BuildLoopOptions`, `ModeWorkThunk`, `AdvanceToNextStep`, and hold-phase setup.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:649-689` defines `BuildLoopOptions`, `ModeWorkThunk`, and `PauseThunk`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1397-1453` runs its own begin-session / run / cleanup flow.
- `MazeMap/MazeMap/MissionModeController.cpp:4648-4696` defines `BuildLoopOptions`, `ActiveLoopThunk`, `InterRunServicePauseThunk`, and `PrepareLoopContinuation`.
- `MazeMap/MazeMap/MissionModeController.cpp:5145-5176` runs its own loop session wrapper.

Why this is duplicate responsibility:
- Each controller owns the same framework-level machinery: callback trampolines, active-state plumbing, session startup, pause handling, and continuation installation.
- The duplication is architectural, not just stylistic. The same `LoopController` session lifecycle is being independently reimplemented three times.

Likely canonical owner:
- A single shared boot-mode/session owner, which the repository policy already points toward as `BootUtilityModeFramework` when shared infrastructure is justified.

### 3. Utility log bootstrap and phase/sample log lifecycle are split across three controllers even though runtime helpers already exist

Evidence:
- Shared helpers already exist in `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h:111-245`:
  - `WriteDiagnosticTuningEvents`
  - `WriteDiagnosticSummaryInstructions`
  - `PopulateDiagnosticLogRow`
- `MazeMap/MazeMap/MissionModeController.cpp:600-644` bootstraps telemetry logs with mode metadata, IMU metadata, bias metadata, schema setup, tuning events, and summary instructions.
- `MazeMap/MazeMap/MissionModeController.cpp:4152-4629` owns telemetry phase markers, event logging, and sample logging.
- `MazeMap/MazeMap/DiagnosticController.cpp:1153-1208` performs nearly the same log bootstrap for diagnostic mode.
- `MazeMap/MazeMap/DiagnosticController.cpp:1211-1453` owns event markers, phase markers, close logic, and sample logging.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:821-894` defines `BeginTimingLog`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1005-1071` defines `BeginMainLog`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1233-1280` defines section markers, events, and fault-dump text entries.

Why this is duplicate responsibility:
- The low-level shared logging helpers are already centralized, but the higher-level responsibility of "initialize a runtime log session, emit standard metadata, manage sections/phases, and stage per-cycle samples" is still owned independently by each mode controller.
- `MissionModeController` and `DiagnosticController` are especially close: both write nearly the same metadata keys, call the same shared helper lambdas, start the same schema, and then layer local phase/event/sample plumbing on top.
- `OpenFloorMeasurementController` adds a third logging lifecycle for timing and sectioned measurement logs instead of reusing a common boot-mode log owner with hooks for mode-specific schemas and labels.

Likely canonical owner:
- Shared runtime or `BootUtilityModeFramework` for shared log lifecycle.
- Individual modes should only supply their schema rows, labels, and mode-specific annotations.

### 4. `DiagnosticController` duplicates primitive hold/straight/turn/arc execution already owned by `ManeuverExecutor`

Evidence:
- `MazeMap/MazeMap/DiagnosticController.cpp:645-699` starts turn and arc phases.
- `MazeMap/MazeMap/DiagnosticController.cpp:1472-1835` implements `HoldPhaseTick`, `StraightPhaseTick`, `TurnPhaseTick`, and `ArcPhaseTick`.
- `MazeMap/MazeMap/ManeuverExecutor.cpp:174-368` already owns `ProceedToHoldRoutine`, `ProceedToBrakedSettleRoutine`, `ProceedToZeroVelocitySettleRoutine`, `ProceedToReverseStraightRoutine`, `ProceedToStraightRoutine`, `ProceedToTurnRoutine`, and `ProceedToArcRoutine`.
- `MazeMap/MazeMap/ManeuverExecutor.cpp:700-1055` already owns `HoldRoutineTick`, `SettleRoutineTick`, `ReverseStraightRoutineTick`, `StraightRoutineTick`, `TurnRoutineTick`, and `ArcRoutineTick`.

Why this is duplicate responsibility:
- Both files own the same primitive motion vocabulary: hold, settle, straight, turn, and arc.
- Both implement per-tick watchdogs, completion checks, speed shaping, and direct drive output generation.
- That splits primitive execution logic between a reusable executor and a mode-local controller, which guarantees drift over time.

Likely canonical owner:
- `ManeuverExecutor` for primitive execution.
- `DiagnosticController` should choose scenario steps, collect metrics, and react to completion/fault hooks rather than owning a second executor.

### 5. `OpenFloorMeasurementController` owns another primitive executor beside `ManeuverExecutor`

Evidence:
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1805-1838` starts recovery motion.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1931-2036` implements `RecoverToMarkerTick`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2280-2451` implements straight-distance and settle execution.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2454-2550` implements in-place turn execution.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2649-2785` implements smooth-turn execution and launch.
- `MazeMap/MazeMap/ManeuverExecutor.cpp:174-417` already owns proceed-to routines for hold, settle, reverse-straight, straight, turn, arc, and smooth-turn.
- `MazeMap/MazeMap/ManeuverExecutor.cpp:700-1124` already owns the corresponding tick functions.

Why this is duplicate responsibility:
- `OpenFloorMeasurementController` is not just sequencing measurement sections. It owns a second motion executor for straight, settle, turn, smooth-turn, and recovery behavior.
- The smooth-turn overlap is especially clear:
  - `OpenFloorMeasurementController.cpp:2707-2739` computes a `ManeuverPoint`, adds a yaw-rate PD correction, clamps `Omega`, and drives `_drive.PointControlVector(...)`.
  - `ManeuverExecutor.cpp:1110-1124` computes a `ManeuverPoint`, clamps `Omega`, and drives `_drive->PointControlVector(...)`.
- That is the same primitive-tracking responsibility with local divergence already starting.

Likely canonical owner:
- `ManeuverExecutor` for primitive motion execution.
- `OpenFloorMeasurementController` should own measurement sequencing, labeling, and capture hooks around the executor.

### 6. Recovery and settle orchestration are independently implemented in multiple boot-selected modes

Evidence:
- `MazeMap/MazeMap/DiagnosticController.cpp:156-159` defines explicit recovery steps: `RecoveryTurnaround`, `RecoveryReturn`, `RecoveryResetHeading`, `RecoverySettle`.
- `MazeMap/MazeMap/DiagnosticController.cpp:719-787` starts and routes the characterization recovery flow through those phases.
- `MazeMap/MazeMap/DiagnosticController.cpp:1764-1768` has recovery-specific behavior embedded inside `TurnPhaseTick`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1805-1838` starts a recovery phase.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1931-2036` implements `RecoverToMarkerTick`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2365-2451` implements section settle setup and execution.

Why this is duplicate responsibility:
- Both modes own the higher-level "recover robot to a known state, settle, then resume scripted work" workflow.
- The concrete destinations differ, but the responsibility is the same: recovery/settle orchestration with watchdogs, state bookkeeping, and phase transitions.
- This is separate from primitive motion duplication. It is duplicated workflow ownership above the primitive layer.

Likely canonical owner:
- Shared boot-mode recovery/settle helpers under the same framework that should own common session behavior.

### 7. `DriveBase` exposes a second velocity-target command-planning API parallel to `PlantModel`

Evidence:
- `MazeMap/MazeMap/DriveBase.h:320-391` publishes a large public planning surface:
  - `DeltaCommand`
  - `PointCommand`
  - `PointControlVector`
  - `PointYawRateCommand`
  - `FeedbackCommand`
- `MazeMap/MazeMap/DriveBase.h:1253-1338` privately re-derives velocity-target operating envelopes and closed-loop velocity commands.
- `MazeMap/MazeMap/DriveBase.cpp:563-638` locally instantiates `PlantModel` and directly calls `solveDriveCommandsForVelocityTarget`, `solveDriveCommands`, and related helpers instead of consuming a canonical shared owner.
- `MazeMap/MazeMap/PlantModel.cpp:2611-2735` already owns `solveDriveCommandsForVelocityTarget` and `resolveVelocityTargetAccelerations`.
- `MazeMap/MazeMap/PlantModel.cpp:2872-3210` already owns `velocityTargetTechnicalLimits` and `solveClosedLoopDriveCommandsForVelocityTarget`.

Why this is duplicate responsibility:
- `PlantModel` already owns the motion-model-side responsibility of turning velocity targets into accelerations, technical limits, and drive-command solutions.
- `DriveBase` then adds a second public vocabulary for command planning and duplicates the same problem decomposition by creating extra local `PlantModel` instances and re-deriving envelopes and targets instead of going through one authoritative shared owner.
- That makes `DriveBase` both actuator owner and alternate planner, which blurs ownership and creates two places where command-solving policy can drift.

Likely canonical owner:
- A single runtime-owned `PlantModel` for velocity-target solving and technical envelopes.
- `DriveBase` should apply resolved commands and own actuator interfacing, not become a second public planning subsystem.

## Notes on non-findings in this pass

- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` looked more like the shared helper owner than a duplicate owner. The duplication problem is that controllers still own parallel orchestration above it.
- `MazeMap/MazeMap/MazeMapRuntimeCore.h` is large, but this pass did not turn up a comparably strong duplicated-responsibility finding inside the audited top-10 set.
- `MazeMap/MazeMap/SrUkfCore.cpp` did not show an obvious top-10 cross-file ownership overlap during this pass.
- `MazeMap/MazeMap/Defines.h` is the centralized host/target boundary and should not be treated as duplication by itself unless mode-local adapters start reappearing elsewhere.

## Highest-value cleanup targets

1. Converge boot-mode entry ownership on `BootModeRegistry` and remove `MissionModeController` as a parallel boot-mode surface.
2. Converge boot-mode loop/session/logging/recovery scaffolding into one shared owner instead of keeping separate controller-local frameworks.
3. Converge primitive motion execution on `ManeuverExecutor`; keep measurement and audit controllers focused on sequencing, capture, and result handling.
4. Converge velocity-target solving and technical limits on `PlantModel`; shrink `DriveBase` back toward actuation and command application.
