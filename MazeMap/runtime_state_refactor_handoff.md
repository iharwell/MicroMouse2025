# Runtime State / UKF Refactor Handoff

Date: 2026-04-23

This note is for the next pass. The current assistant context is saturated, the change is large, and the user clarified several ownership rules mid-stream that materially changed the plan.

## User-stated non-negotiables

- Kill `LoopController::ModeState`.
- Kill `PoseEstimate`.
- Standardize on `VehicleState`.
- `VehicleState` must not carry control-loop diagnostic timing.
- A single whole-state timestamp is acceptable on `VehicleState`.
- `VehicleState` should represent the state variables and the actual sensor records of a single tick.
- `LoopController` owns timing and exposes `TimingDiagnostics` through getters only.
- `TimingDiagnostics` must not be passed around as a callback parameter.
- `TickServices` must remain command-only. Do not turn it into a read-getter bundle.
- Broad reuse is not a defense of redundancy.
- `DriveBase` must not own state.
- `SharedRobotRuntime` is the top-level owner of the live runtime state.
- `DriveBase` is the motor engine.
- The UKF update path must be extracted out of `DriveBase` in this same pass.
- No aggregate cache as a replacement for `PoseEstimate`. User explicitly rejected that shape:
  - if some subsystem needs yaw, that can be a private float
  - if it needs heading, that can be a private derived/private field
  - do not preserve a public or semi-public “pose bundle”

## AGENTS.md constraints relevant here

- Use delete-first / copy-delete-stitch.
- Do not leave old and new ownership in parallel.
- Do not preserve wrappers or compatibility shims.
- Preserve buildability and verification paths.
- Verify latest changes through Release tests if supported.

## Current architectural diagnosis

### The main ownership problem

The current runtime path still has three overlapping state/access layers:

1. `DriveBase` owns the UKF object and currently exposes:
   - `GetPose()`
   - `GetEstimatorStateVector()`
   - `GetVehicleState()` value helper
2. `LoopController` rebuilds a second per-tick wrapper:
   - `ObservedTickState`
   - `ModeState`
   - `PauseContext.stateEstimate`
3. Many controllers then read from `ModeState` or `PoseEstimate`.

That is exactly the redundancy the user wants removed.

### The deeper entanglement

`DriveBase` is not just reading estimator state. It is currently running the UKF update path itself inside `UpdateOdometry(...)`. The user called this out directly:

> "DriveBase is the motor engine. What is it doing with UKF updates?!"

That means this pass is not just a callback-signature migration. The estimator/update path must move out of `DriveBase`, and runtime state ownership must move up to `SharedRobotRuntime`.

## Canonical ownership after user clarifications

This is the intended end state inferred from the conversation:

- `SharedRobotRuntime`
  - owns the authoritative live runtime `VehicleState`
  - owns the authoritative runtime estimator / UKF update path
  - owns the authoritative `LoopController`
  - owns the authoritative sensor pipeline
- `LoopController`
  - owns tick cadence and timing diagnostics only
  - publishes timing only via getters
  - passes canonical runtime state to mode callbacks
- `DriveBase`
  - motor engine / low-level command generation / encoder-facing motion machinery
  - must not own runtime state
  - must not own UKF update orchestration
  - must not expose `PoseEstimate`
- `VehicleState`
  - canonical state object
  - may carry one whole-state timestamp
  - should carry per-tick sensor record(s)
  - should not carry `dt`, tick sequence, overrun, or diagnostic timing fields

## Important correction to one earlier idea

Do **not** replace `_poseCache` with a cached `VehicleState`.

The user explicitly rejected that direction. If a subsystem truly needs narrow derived pieces internally, keep those as narrow private fields, not as a second state object.

## Current code hotspots

### `LoopController`

Files:
- `MazeMap/MazeMap/LoopController.h`
- `MazeMap/MazeMap/LoopController.cpp`

Current problems:
- `ModeState` still exists.
- `PauseContext.stateEstimate` is still a full `ModeState`.
- `ObservedTickState` still carries `PoseEstimate`.
- `ProjectEstimate(...)` still projects `PoseEstimate`.
- `CurrentModeState()` is a back-channel for `Drive` and `WallTouch`.

Key current lines from local inspection:
- `LoopController.h:99` `ModeState`
- `LoopController.h:115` `PauseContext`
- `LoopController.h:140` callback typedef
- `LoopController.h:206` `ObservedTickState`
- `LoopController.h:280` `CurrentModeState()`
- `LoopController.cpp:483` `ProjectEstimate(...)`
- `LoopController.cpp:600` `ExecuteSensingUpdate(...)`
- `LoopController.cpp:719` `BuildModeState(...)`
- `LoopController.cpp:839` pause settlement still builds a bundled `ModeState`

### `DriveBase`

Files:
- `MazeMap/MazeMap/DriveBase.h`
- `MazeMap/MazeMap/DriveBase.cpp`

Current problems:
- still exposes `GetPose()`
- still owns `_poseCache`
- still owns UKF update execution in `UpdateOdometry(...)`
- still has `SyncPoseEstimate()`
- still has local state reset helpers centered on estimator ownership

Key current lines from local inspection:
- `DriveBase.h:410` `GetPose()`
- `DriveBase.h:415` `GetVehicleState()` value helper already exists
- `DriveBase.h:745` `ResetPoseEstimate(...)`
- `DriveBase.h:766` `SyncPoseEstimate()`
- `DriveBase.h:956-1048` UKF timing/update work still in `UpdateOdometry(...)`
- `DriveBase.h:1061` `_poseCache`

### `SharedRobotRuntime`

Files:
- `MazeMap/MazeMap/SharedRobotRuntime.h`
- `MazeMap/MazeMap/SharedRobotRuntime.cpp`

Current problems:
- does not yet own a top-level runtime `VehicleState`
- still constructs `DriveBase` as if drive+estimator+state are one thing

Existing runtime composition relevant to the refactor:
- `SharedRobotRuntime.cpp:425-475` constructor wires:
  - `speedVehicle`
  - `drive`
  - `driveService`
  - `controlLoop`
  - `sensors`

This is the place to install the top-level live runtime state owner.

### `Vehicle`

Files:
- `MazeMap/MazeMap/Vehicle.h`
- `MazeMap/MazeMap/Vehicle.cpp`

Current problem:
- `Vehicle` still has a private `VehicleState _state;`

User clarification after that was discovered:

> "SharedRobotRuntime. The state is top-level."

That means `Vehicle::_state` is now suspect. `Vehicle` should remain the owner of construction facts / physical facts, not the live runtime state.

## Specific user clarification that changed the plan

Originally this pass looked like:
- remove `ModeState`
- remove `PoseEstimate`
- migrate callback readers

After clarification, it became:
- remove `ModeState`
- remove `PoseEstimate`
- move state ownership to `SharedRobotRuntime`
- extract UKF update logic out of `DriveBase`
- then migrate all callback/state readers to the top-level owner

That is why this is no longer a narrow signature-only cleanup.

## Subagent findings worth preserving

Both agents have now returned. Their useful concrete recommendations are captured here, with the parts that were later superseded by user clarification called out explicitly.

### Agent result: disjoint migration slice

The cleanest early slice identified was:
- `LoopController.h/.cpp`
- `VehicleState.h`
- `DriveBase.h`
- `Drive.h/.cpp`
- `WallTouch.h/.cpp`
- `MissionRunMode.cpp`

Reason:
- it covers the callback surface and shared motion services first
- it avoids the timing-heavy measurement modes until the core ownership is fixed

Useful takeaway:
- this is still a good *consumer migration* slice once ownership is fixed at the runtime level

Important limitation:
- this slice was proposed before the user made the ownership boundary stricter:
  - `SharedRobotRuntime` owns live runtime state
  - `DriveBase` must not own state
  - UKF update extraction is in-scope this pass

So this slice is **not** sufficient as the opening move anymore. It is still useful after the top-level runtime state owner and estimator owner are established.

### Agent result: callback surface recommendation

One agent recommended replacing `ModeState` with a split callback surface:

```cpp
using ModeWorkCallback = ControlVector (*)(
    void* context,
    std::uint32_t loopEndTimeUs,
    const VehicleState& state,
    const SensorSnapshot& sensors,
    const DriveTelemetry& driveTelemetry,
    float measuredLinearSpeedMps,
    float measuredAngularSpeedRadps,
    bool estimatorHealthy,
    const char* estimatorFaultReason,
    TickServices& services);
```

That is still useful as a migration reference, but it is not fully aligned with the latest user guidance because the user wants `VehicleState` itself to provide the tick’s actual sensor records. Re-evaluate whether `SensorSnapshot` should remain a separate callback parameter or be read from `VehicleState`.

Additional useful agent recommendations from the same result:

- add whole-state timestamp access on `VehicleState`
  - `SetTimestampUs(std::uint32_t timestampUs) noexcept;`
  - `GetTimestampUs() const noexcept;`
- expose one authoritative `LoopController` timing snapshot:
  - `LastDiagnostics()`

These are aligned with the user’s “timing stays on `LoopController`” rule.

Important limitation:
- the same agent also recommended replacing `DriveBase::GetPose()` with a `DriveBase::GetVehicleState()` plus cache/sync path.
- that part is **superseded** by later user clarification:
  - `DriveBase` does not own state
  - `SharedRobotRuntime` is top-level state owner
  - no aggregate cached replacement for `PoseEstimate`

So:
- keep the signature and timing-getter guidance
- reject the `DriveBase` state-owner/cache direction

### Agent result: risky call-site map

The agents produced a useful set of concrete risky sites that should be revisited during the change:

- `LoopController.h:99`
  - `ModeState` fanout point
- `LoopController.h:140`
  - `ModeWorkCallback`
- `LoopController.h:280`
  - `CurrentModeState()`
- `LoopController.cpp:483`
  - `ProjectEstimate(...)`
- `LoopController.cpp:607`
  - still captures `observed.estimate = _runtime->Drive().GetPose();`
- `LoopController.cpp:719`
  - `BuildModeState(...)`
- `LoopController.cpp:839`
  - pause settlement still depends on bundled state

- `Drive.cpp:269`
  - `BuildFallbackModeState(...)` must be deleted, not renamed
- `Drive.cpp:498`
  - state/fallback selection in `GetNextControls(...)`
- `Drive.cpp:638`
  - `state.dtSeconds` currently drives accel ramping; should come from `LoopController` timing getters instead

- `WallTouch.cpp:162`
  - `TryGetLoopState()` / `CurrentModeState()` dependency
- `WallTouch.cpp:169`
  - estimator health / fault reads
- `WallTouch.cpp:339`
  - heading/yaw damping currently reads `state.estimate`

- timing/logging-heavy consumers that will need explicit timing migration:
  - `AuxMeasurementController.cpp:45`
  - `DiagnosticController.cpp:399`
  - `OpenFloorMeasurementController.cpp:979`
  - `ShowcasingDonutController.cpp:563`
  - `TopSpeedMeasurementMode.cpp:258`
  - `MazeMapRuntimeInfrastructure.h:107`

### Agent result: narrow helper-signature suggestions

One useful set of suggestions from the agents was to narrow helper signatures based on actual use instead of preserving a wrapper.

For `Drive`, current-use narrowing proposed:
- `HoldControls(const SensorSnapshot&, const DriveTelemetry&, bool&)`
- `LinearMotionControls(const VehicleState&, const SensorSnapshot&, const DriveTelemetry&, float dtSeconds, bool&)`
- `TurnControls(const VehicleState&, const SensorSnapshot&, float dtSeconds, bool&)`
- `TurnTransitionControls(const VehicleState&, const DriveTelemetry&, bool&)`
- `ArcControls(const DriveTelemetry&, bool&)`
- `ManeuverControls(const VehicleState&, const SensorSnapshot&, const DriveTelemetry&, bool&)`

For `WallTouch`, current-use narrowing proposed:
- `ForwardControl(const VehicleState&, float desiredSpeedMps, float yawRateBiasRadps = 0.0f) const`
- `SeekControls(const VehicleState&, const SensorSnapshot&, const DriveTelemetry&, bool&)`
- `SeatControls(const VehicleState&, bool&)`
- `SquareControls(const VehicleState&, const SensorSnapshot&, bool&)`
- `HoldBeforeReturnControls(const VehicleState&, bool&)`

These are still directionally useful because they push behavior toward real owners and away from the old wrapper.

Practical caution:
- do not apply these as a blind mechanical signature edit before runtime ownership is fixed
- state source and timing source must be correct first

### Agent result: loop timing surface

The agent recommendation to expose timing through one authoritative snapshot is still aligned:

- `LastDiagnostics()`

That snapshot is the replacement for `state.sequence`, `state.tickStartUs`, `state.dtUs`, and `state.dtSeconds`.

## Recommended migration order

### 1. Establish the top-level live runtime state owner

Likely files:
- `SharedRobotRuntime.h/.cpp`
- `Vehicle.h/.cpp`

Goal:
- add authoritative runtime `VehicleState` ownership to `SharedRobotRuntime`
- remove or de-authorize `Vehicle::_state`

Important:
- `Vehicle` should remain physical fact owner
- do not turn `Vehicle` into a runtime session owner

### 2. Extract estimator / UKF update ownership from `DriveBase`

Likely files:
- `DriveBase.h/.cpp`
- `SharedRobotRuntime.h/.cpp`
- possibly a new authoritative runtime estimator owner if needed

Goal:
- remove UKF update orchestration from `DriveBase`
- `DriveBase` should become a consumer of state / producer of low-level command effects, not the owner of estimation

Important:
- if a new owner is introduced, it must be the authoritative owner, not a wrapper around the old `DriveBase` implementation
- do not leave “DriveBase + estimator helper” in ambiguous parallel ownership

### 3. Kill `PoseEstimate`

Likely files:
- `MazeMapRuntimeCore.h`
- `RuntimeSensorSuite.h/.cpp`
- `MazeMapRuntimeSignalHelpers.h/.cpp`
- `AuxMeasurementModeSupport.h/.cpp`
- `StartupCalibration.cpp`
- `DiagnosticController.cpp`
- `TopSpeedMeasurementMode.cpp`
- `WallTouch.cpp`

Most of these only need:
- `position x/y`
- `yaw`
- derived heading
- linear/angular speeds

Those should come from `VehicleState` or narrow local derivation, not a parallel wrapper.

### 4. Kill `ModeState`

Likely files:
- `LoopController.h/.cpp`
- `Drive.h/.cpp`
- `WallTouch.h/.cpp`
- all callback consumers currently taking `const LoopController::ModeState&`

Important:
- remove `CurrentModeState()` and any back-channel state wrapper access
- move timing uses to `LoopController` getters
- move state/sensor uses to the canonical runtime state path

### 5. Migrate timing-heavy controllers after core ownership is fixed

Timing/logging heavy files include:
- `AuxMeasurementController.cpp`
- `DiagnosticController.cpp`
- `OpenFloorMeasurementController.cpp`
- `ShowcasingDonutController.cpp`
- `TopSpeedMeasurementMode.cpp`
- `MazeMapRuntimeInfrastructure.h`

These currently read:
- `state.tickStartUs`
- `state.sequence`
- `state.dtUs`
- `state.dtSeconds`

Those should migrate to `LoopController::LastDiagnostics()`.

## Concrete files likely in play

This is the consolidated file list from local inspection and subagent outputs.

Core ownership files:
- `MazeMap/MazeMap/SharedRobotRuntime.h`
- `MazeMap/MazeMap/SharedRobotRuntime.cpp`
- `MazeMap/MazeMap/Vehicle.h`
- `MazeMap/MazeMap/Vehicle.cpp`
- `MazeMap/MazeMap/VehicleState.h`
- `MazeMap/MazeMap/VehicleState.cpp`
- `MazeMap/MazeMap/LoopController.h`
- `MazeMap/MazeMap/LoopController.cpp`
- `MazeMap/MazeMap/DriveBase.h`
- `MazeMap/MazeMap/DriveBase.cpp`

Shared motion / callback-reader files:
- `MazeMap/MazeMap/Drive.h`
- `MazeMap/MazeMap/Drive.cpp`
- `MazeMap/MazeMap/WallTouch.h`
- `MazeMap/MazeMap/WallTouch.cpp`
- `MazeMap/MazeMap/MissionRunMode.cpp`
- `MazeMap/MazeMap/ManeuverExecutor.cpp`
- `MazeMap/MazeMap/ManeuverFileTestMode.cpp`
- `MazeMap/MazeMap/PositionAccuracyAuditMode.cpp`
- `MazeMap/MazeMap/CorridorRepeatabilityMode.cpp`
- `MazeMap/MazeMap/WallSensorLedCalibrationController.h`
- `MazeMap/MazeMap/WallSensorLedCalibrationController.cpp`

Pose / sensor helper files:
- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `MazeMap/MazeMap/MazeMapRuntimeSignalHelpers.h`
- `MazeMap/MazeMap/MazeMapRuntimeSignalHelpers.cpp`
- `MazeMap/MazeMap/RuntimeSensorSuite.h`
- `MazeMap/MazeMap/RuntimeSensorSuite.cpp`
- `MazeMap/MazeMap/AuxMeasurementModeSupport.h`
- `MazeMap/MazeMap/AuxMeasurementModeSupport.cpp`

Timing/logging-heavy callback consumers:
- `MazeMap/MazeMap/AuxMeasurementController.cpp`
- `MazeMap/MazeMap/DiagnosticController.cpp`
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp`
- `MazeMap/MazeMap/ShowcasingDonutController.cpp`
- `MazeMap/MazeMap/TopSpeedMeasurementMode.cpp`
- `MazeMap/MazeMap/FrontWallCharacterizationController.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`

Tests likely affected:
- `MazeMap/MazeMapTest/DriveBaseTest.cpp`
- other tests referencing application/runtime names depending on compile fallout

## Partial edits already made in this session

Only two file changes were made by this assistant:

1. `MazeMap/MazeMap/VehicleState.h`
2. `MazeMap/refactor_delete_first_reference.txt`

### `VehicleState.h` partial changes

Added:
- `#include "SensorSnapshot.h"`
- `GetPositionX()`
- `GetPositionY()`
- `GetHeadingUnit()`
- `SetSensorSnapshot(...)`
- `GetSensorSnapshot()`
- `ProjectConstantVelocity(float dtSeconds)`
- private field `SensorSnapshot _sensorSnapshot`

Intent:
- move toward user requirement that `VehicleState` represent state plus tick sensor record
- supply enough canonical behavior to avoid preserving `PoseEstimate`

Status:
- partial only
- not yet threaded through runtime owners/callers
- may still be valid in final design

### `refactor_delete_first_reference.txt`

Added as a temporary non-compiled copy of:
- `PoseEstimate`
- `LoopController::ModeState`

Intent:
- satisfy delete-first workflow when the actual delete/copy/stitch began

Status:
- harmless temporary reference
- delete it before final completion

## Important current repo state

- The worktree is already dirty in many unrelated and related files.
- Do not revert unrelated user changes.
- Read every touched file carefully before editing because this refactor area is already in motion.

## Verification expectations

Before testing:
- per repo instructions, check that active binaries reflect latest changes rather than rebuilding blindly from scratch

After implementation:
- Release host build / tests
- Teensy compile path
- compare failures against existing baseline rather than assuming zero failures is realistic

Known earlier baseline from a prior turn:
- Release unit tests: 630 total, 551 passed, 79 failed

That baseline was from before this unfinished refactor attempt and should only be used as rough context.

## Practical warning for the next pass

Do not start by just renaming types.

If the first change is “replace `ModeState` with `VehicleState` everywhere” without first moving:
- top-level runtime state ownership
- estimator ownership
- timing ownership

then the result will still be the same architecture under different names.

The user is explicitly against that.
