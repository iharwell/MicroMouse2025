# LoopController Inventory And Convergence Notes

`LoopController` is intentionally not wired into the runtime yet. This document does two things:

1. catalogs the five existing timing-synchronized loop systems, and
2. defines what a single configurable singleton `LoopController` must provide to subsume them.

## Current Corrections

This inventory predates the current session-ownership rules. Treat the following constraints as superseding older exploratory suggestions below:

- One top-level `IApplicationMode` owns exactly one active `LoopController` session for the life of that boot-selected mode.
- Nested `RunLoopSession(...)` launch sites inside subordinate controllers are architectural bugs, not an acceptable steady state.
- While motion is active, only the installed callback executes and can steer flow, request a pause, end the session, or swap the callback.
- Waiting, sleeping, or `delay(...)` outside a granted `RequestPause(...)` callback is nonconforming.
- Mission code owns goals, replans, and phase policy; shared drive and maneuver execution infrastructure should own the actual motion primitives.
- `ManeuverInstance` is the canonical executable maneuver unit. `SmoothTurnExecutionProfile` should not remain mode-level execution vocabulary.

## Proposed Home

- `MazeMap/MazeMap/LoopController.h`
- `MazeMap/MazeMap/LoopController.cpp`

## Existing Timing Loop Systems

### `MazeMap/MazeMap/MissionModeController.cpp`

- Primary function: `TickControl(bool stationary, float& dtSeconds, SensorSnapshot& snapshot)`
- Loop shape: full control-tick owner
- Timing ownership:
- waits against `Config::kControlPeriodUs`
- services telemetry logging while waiting
- latches one tick timestamp and computes `dtSeconds`
- Tick responsibilities:
- captures mission sensors
- updates odometry through `DriveBase`
- checks estimator faults
- logs the telemetry sample for the tick
- Caller responsibilities after each tick:
- decide whether to brake, hold, turn, drive straight, or run a maneuver
- issue one command for the tick
- decide when a phase is complete or faulted
- Distinct features:
- closest thing to the canonical mission control loop
- reused by many mission behaviors including hold, settle, search straight, turn, and maneuver execution
- already follows the intended pattern where higher-level behavior makes one decision per tick on top of shared timing/capture work

### `MazeMap/MazeMap/OpenFloorMeasurementController.cpp`

- Primary function: `CaptureCycle(bool stationary, OpenFloorMeasurementCycle& cycle)`
- Loop shape: full control-tick owner with heavy measurement instrumentation
- Timing ownership:
- waits against `DiagnosticConfig::kControlPeriodUs`
- services open-floor log flushing while waiting
- latches one tick start time and sequence number
- captures detailed sub-phase timing such as encoder latch/read and UKF timing fields
- Tick responsibilities:
- snapshots drive telemetry before the sensor pipeline runs
- captures diagnostic sensors
- updates odometry through `DriveBase`
- derives measured linear/angular kinematics after capture
- checks estimator faults
- enforces the open-floor workspace guard
- Caller responsibilities after each tick:
- set section/phase labels
- decide recovery, launch, straight, yaw, smooth-turn, and loop commands
- write section markers and mode-specific fault rows
- Distinct features:
- the richest timing instrumentation in the codebase
- owns a dual logging scheme with timing and main streams
- attaches per-tick measurement metadata such as section, primitive, phase, repeat, and speed bin
- includes mode-specific workspace fault handling and recovery flows

### `MazeMap/MazeMap/DiagnosticController.cpp`

- Primary function: `TickControl(bool stationary, float& dtSeconds, uint32_t& timestampUs, DiagnosticSensorSnapshot& snapshot)`
- Loop shape: full control-tick owner
- Timing ownership:
- waits against `DiagnosticConfig::kControlPeriodUs`
- services diagnostic logging while waiting
- latches one tick timestamp and computes `dtSeconds`
- Tick responsibilities:
- captures diagnostic sensors
- updates odometry through `DriveBase`
- checks estimator faults
- enforces the diagnostic operating boundary
- Caller responsibilities after each tick:
- decide motion commands for hold, straight, turn, and other legacy diagnostic phases
- write phase-specific result events
- decide completion and fault outcomes
- Distinct features:
- older full diagnostic loop with less timing detail than open-floor
- returns both `dtSeconds` and a raw tick timestamp
- has integrated boundary enforcement rather than open-floor workspace enforcement

### `MazeMap/MazeMap/AuxMeasurementController.cpp`

- Primary function: `WaitForNextSample(uint32_t& timestampUs, uint32_t& dtUs)`
- Representative caller-owned tick bodies:
- `RunTurningTractionSweep`
- `HoldPhase`
- Loop shape: timing-gate helper with caller-owned tick body
- Timing ownership:
- waits against `AuxMeasurementConfig::kControlPeriodUs`
- services utility logging while waiting
- latches one tick timestamp and computes `dtUs`
- Tick responsibilities owned by the helper:
- only timing synchronization
- Caller responsibilities after each tick:
- capture diagnostic sensors
- update odometry through `DriveBase`
- check estimator faults
- compute traction or hold-phase behavior
- issue open-loop or closed-loop drive commands
- log the sample
- Distinct features:
- separates the timing gate from the body of the control tick
- already demonstrates that the same timing infrastructure can support both closed-loop and open-loop command paths
- used by both active sweep logic and passive hold logic

### `MazeMap/MazeMap/FrontWallCharacterizationController.cpp`

- Primary function: `WaitForNextSample(uint32_t& timestampUs, uint32_t& dtUs)`
- Representative caller-owned tick bodies:
- `HoldStationary`
- `CaptureCurve`
- Loop shape: timing-gate helper with caller-owned tick body
- Timing ownership:
- waits against `FrontWallCharacterizationConfig::kControlPeriodUs`
- latches one tick timestamp and computes `dtUs`
- Tick responsibilities owned by the helper:
- only timing synchronization
- Caller responsibilities after each tick:
- capture diagnostic sensors
- update odometry through `DriveBase`
- compute reverse-travel progress
- store characterization samples
- decide timeout/storage/collapse termination
- issue heading-corrected reverse motion or brake
- Distinct features:
- simplest timing gate in the set
- no idle log-service callback while waiting
- owns procedural sample-capture logic rather than a generic managed tick

## Common Pattern Across All Five

All five systems share the same basic timing skeleton:

- mode-local `_lastControlMicros`
- mode-local `kControlPeriodUs`
- wait-until-next-period gate
- timestamp latch
- `dt` derivation from the previous tick
- per-tick sensor/odometry/control work driven after the latch

The main differences are:

- whether the loop owner also captures sensors and updates odometry, or only gates time
- whether idle-service work runs while waiting
- whether timing instrumentation is minimal or detailed
- whether guards are workspace-based, boundary-based, or absent
- whether logging is generic telemetry, utility logging, or mode-specific measurement logging

## Singleton `LoopController` Target

`LoopController` should become the single runtime-owned authority for fixed-period control timing.

That implies:

- exactly one active loop session at a time
- the selected top-level `IApplicationMode` owns session startup and shutdown for the whole boot-selected run
- subordinate controllers and per-motion helpers must not start nested sessions
- no mode-local `_lastControlMicros` once migration is complete
- no parallel timing helpers in individual controllers
- boot modes remain declared by `BootModeRegistry`; `LoopController` does not own boot-mode selection metadata
- `LoopController` owns timing and per-tick orchestration, while mode logic owns mode decisions through callbacks

`LoopController` should be a singleton because the application only runs one active top-level boot mode per session, and the app entrypoint already selects one `IApplicationMode` and calls `Run()` once.

## Required Shared Layout

The singleton design needs four shared pieces:

1. one singleton owner,
2. one shared session settings interface,
3. one shared callback contract,
4. one shared tick/session state layout.

## Singleton Owner Responsibilities

The singleton `LoopController` should eventually own:

- the active control period
- idle-sleep configuration
- last latched tick time
- tick sequence counter
- active session metadata
- the shared tick orchestration path
- optional timing instrumentation state for modes that need it

It should not own:

- boot-mode selection policy
- maze/domain behavior
- mission logic
- utility-mode motion logic
- per-mode result formatting

## Shared Session Settings Interface

One shared settings object should configure the loop for any boot mode. A concrete future shape could be `LoopControllerSettings`, with grouped settings like the following.

### Session Identity

- `BootModeId bootModeId`
- `const BootModeDescriptor* descriptor`
- `const char* stableSessionName`

This lets the timing infrastructure know which top-level mode is using it without duplicating `BootModeRegistry`.

### Timing Settings

- `unsigned long controlPeriodUs`
- `unsigned int idleSleepUs`
- `bool resetClockOnSessionStart`
- `bool runIdleServiceWhileWaiting`
- `bool maintainTickSequence`

These fields cover the current period-wait differences across mission, diagnostic, open-floor, auxiliary measurement, and front-wall characterization.

### Execution Style Settings

- `bool managedTickPipelineEnabled`
- `bool timingGateOnlyEnabled`
- `bool applyReturnedCommandInsideLoopController`

The intent is:

- `managedTickPipelineEnabled` supports mission, open-floor, and legacy diagnostic style loops
- `timingGateOnlyEnabled` supports migration of aux/front-wall timing helpers without forcing immediate restructuring
- only one execution style should be active for a given session

### Capture And Estimation Settings

- `bool captureSensorsInsideLoopController`
- `bool updateOdometryInsideLoopController`
- `bool snapshotDriveTelemetryBeforeCapture`
- `bool deriveMeasuredKinematicsAfterCapture`
- `bool stationaryHintComesFromCallback`
- `bool checkEstimatorFaultAfterCapture`

These fields are needed because open-floor wants richer capture products than mission, while aux/front-wall currently own capture outside the timing helper.

### Guard Settings

- `bool enableWorkspaceGuard`
- `bool enableBoundaryGuard`
- `bool enableCustomGuardCallback`
- `bool brakeOnGuardFault`
- `bool brakeOnEstimatorFault`

This covers:

- mission: usually no open-floor workspace/boundary guard
- open-floor: workspace guard
- diagnostic: diagnostic boundary guard
- aux/front-wall: no built-in guard today, but custom guard support should exist

### Instrumentation Settings

- `bool recordControlTiming`
- `bool recordTickTimestamp`
- `bool recordTickSequence`
- `bool recordDriveTelemetry`
- `bool recordModeSpecificFlags`

This covers the span from front-wall's minimal timing to open-floor's detailed per-cycle instrumentation.

### Logging Integration Settings

- `bool runLogServiceWhileWaiting`
- `bool logEachTick`
- `bool logFaultsThroughLoopController`
- `bool allowModeSpecificTickLogging`

This is required because:

- mission services telemetry log flushing while waiting
- diagnostic/open-floor service their own logs while waiting
- aux measurement also services logs while waiting
- front-wall currently has no wait-time service work

## Shared Callback Infrastructure

One shared callback contract should let any boot mode plug into the timing infrastructure. This can be implemented either as a callback struct or as an interface object, but the shared responsibilities need to be the same.

A concrete future shape could be `LoopControllerCallbacks` or `ILoopControllerClient`, with the following hooks.

### Session Lifecycle Hooks

- `OnSessionBegin`
- `OnSessionEnd`
- `OnSessionFault`

These let a mode initialize mode-local state and emit session-level artifacts without making `LoopController` own mode behavior.

### Wait-State Hook

- `ServiceWhileWaiting`

This covers the current mission, diagnostic, open-floor, and auxiliary log-service work that occurs during the period wait.

### Tick Preparation Hooks

- `QueryStationaryHint`
- `OnTickStart`
- `OnPreCapture`

This allows a mode to change the stationary flag per tick and to prepare per-tick labels or mode-local state before capture.

### Capture/Odometry Hooks

- `CaptureSensors`
- `UpdateOdometry`
- `OnPostCapture`

These are required because the concrete capture products differ between mission and diagnostic/open-floor code today. The shared timing owner should not force one snapshot type onto every mode.

### Guard Hooks

- `EvaluateCustomGuard`
- `OnGuardFault`

These are needed for modes whose guard logic is not simply "workspace" or "boundary".

### Decision Hooks

- `ComputeControlDecision`
- `ApplyControlDecision`

The key rule is:

- the control loop infrastructure owns one tick,
- the boot mode provides one decision for that tick,
- the loop owner does not make multi-step mode decisions outside that callback.

This is the core requirement for avoiding the control-loop usurpation problem.

### Logging Hooks

- `OnTickLogged`
- `LogTick`
- `LogFault`

This allows shared timing while keeping mission telemetry, open-floor timing/main logs, and utility-mode result logging mode-specific.

## Shared Tick State Layout

One shared tick/session state layout should be available to the callback layer. A concrete future shape could include the following.

### `LoopControllerTick`

- `uint32_t sequence`
- `unsigned long tickStartUs`
- `uint32_t dtUs`
- `float dtSeconds`
- `bool stationary`
- `bool estimatorFault`
- `bool guardFault`
- `const char* faultReason`

### Optional Common Tick Attachments

- latched drive telemetry
- control-timing instrumentation payload
- derived measured kinematics
- tick flags

Mode-specific payload should remain mode-owned rather than bloating the shared loop owner with every mode's log schema.

## Shared Control Decision Layout

One shared decision/result type is needed so the loop owner can apply commands consistently.

A concrete future shape could be `LoopControlDecision` with:

- `ActuationKind`
- `Brake`
- `Velocity`
- `OpenLoopRaw`
- `NoCommand`
- linear speed target
- angular speed target
- left/right open-loop command values
- completion result
- fault result

That is enough to express all current loop bodies:

- mission: brake or velocity
- open-floor: brake, velocity, open-loop launch
- diagnostic: brake or velocity
- aux measurement: brake, velocity, open-loop launch/sweep
- front-wall characterization: brake or reverse velocity

## Required Tick Order

A single shared control loop needs one standard order of operations. The loop owner should follow this order for managed sessions:

1. wait until the configured period boundary
2. run `ServiceWhileWaiting` if enabled
3. latch tick start time, `dt`, and sequence
4. ask the mode for the stationary hint if configured
5. run pre-capture hooks
6. capture sensors and update odometry through the configured capture callbacks
7. run built-in estimator/guard checks
8. run custom guard callbacks if enabled
9. hand the tick to the mode for exactly one control decision
10. apply the returned command if loop-owned actuation is enabled
11. run per-tick logging hooks
12. return `continue`, `complete`, or `fault`

This ordering covers all five current systems.

## Boot Mode Integration

Boot modes should hook into `LoopController` through the existing `IApplicationMode` boundary, not through a second registry or a side manager.

The intended integration model is:

- `BootModeRegistry` continues to identify the selected top-level boot mode
- the selected `IApplicationMode::Begin()` configures the singleton `LoopController` session
- the selected `IApplicationMode::Run()` either:
- drives `LoopController` in managed-tick mode, or
- requests timing-gate ticks during migration
- when the mode completes or faults, it closes the session
- while the robot is moving, only the active callback can change loop flow or request a different phase

The boot-mode-facing shared inputs should therefore be:

- one settings object
- one callback object/interface implementation
- one runtime/service bundle of authoritative owners the callbacks already use

## Shared Runtime Bundle Needed By Callbacks

To let boot modes hook into the timing infrastructure without inventing parallel owners, the callbacks need a shared runtime bundle that references existing canonical owners.

That bundle should contain references such as:

- `SharedRobotRuntime&`
- `DriveBase&`
- mission or diagnostic sensor pipeline as appropriate
- optional `Maze*` when odometry update requires it

The bundle should not create new owners for:

- `Vehicle`
- `Maze`
- `MmLogLogger`
- pathfinders

## Feature Mapping From Existing Systems To One Configurable Loop

### Mission Needs

- managed full tick
- idle log service while waiting
- mission sensor capture
- odometry update
- estimator-fault propagation
- telemetry logging
- one decision per tick

### Open-Floor Needs

- managed full tick
- detailed timing instrumentation
- drive telemetry snapshot before capture
- derived measured kinematics after capture
- workspace guard
- per-tick mode labels owned outside the loop core
- dual log integration through mode callbacks
- open-loop and closed-loop actuation kinds

### Legacy Diagnostic Needs

- managed full tick
- idle log service while waiting
- diagnostic sensor capture
- odometry update
- estimator-fault propagation
- diagnostic boundary guard
- raw timestamp exposure

### Auxiliary Measurement Needs

- either managed full tick or timing-gate-only during migration
- idle log service while waiting
- open-loop and closed-loop actuation kinds
- caller-owned sweep/hold logic
- utility logging hooks

### Front-Wall Characterization Needs

- either managed full tick or timing-gate-only during migration
- minimal timing data
- no required wait-time service callback
- caller-owned sample storage and termination logic
- reverse-velocity actuation and braking

## Migration Boundary

The singleton `LoopController` should replace:

- mission `TickControl`
- open-floor `CaptureCycle`
- diagnostic `TickControl`
- auxiliary `WaitForNextSample`
- front-wall `WaitForNextSample`

It should not replace:

- boot-mode selection
- mode descriptors
- mode-local logging schemas
- mission/open-floor/diagnostic behavior decisions
- domain owners such as `Maze`, `Vehicle`, and `SharedRobotRuntime`

## Immediate Design Constraint

When this is implemented for real, `LoopController` must centralize timing, capture orchestration, and shared tick services, but it must not become the owner of mode behavior. The mode callback layer should make one control decision per tick and hand that decision back to the shared loop owner.

Pause callbacks should remain small and phase-specific. They should not become distribution hubs for unrelated work.
