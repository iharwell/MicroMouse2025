# Wrapper, Parameter-Bag, and Redirection-Helper Type Audit

This is an agent-assisted inventory of types that *potentially* conflict with the repo guidance on wrapper shells, parameter bags, split owners, or redirection helpers.

Confidence scale:
- `3`: strong candidate
- `2`: plausible candidate worth review
- `1`: low-confidence candidate or possibly justified framework contract

Deliberately not flagged here:
- `BootModeDescriptor`, `BootModeRegistryEntry`, and `BootModeSelectorCondition`, because the repo guidance explicitly names that registry/descriptor split as canonical.
- `mmlog` row schemas, because the repo guidance explicitly allows them.
- File-local phase-state structs that stay inside one authoritative owner, because the guidance prefers private/file-local helpers over new public types.

## Production Candidates: Wrapper Shells and Redirection Helpers

| Type | File | Category | Confidence | Why it is suspicious |
| --- | --- | --- | --- | --- |
| `MouseUkfFacade` | `MazeMap/MazeMap/MouseUkfFacade.h` | wrapper facade | `3` | Literal facade type; most public API forwards into `SrUkfCore` while also exposing `MapEvidenceUpdater`, which suggests a composed wrapper rather than one clear owner. |
| `MissionRunMode` | `MazeMap/MazeMap/MissionRunMode.h` | wrapper shell | `3` | `Begin()` and `Run()` only forward into `MissionModeController` through a hidden `Implementation`, adding another shell around mission ownership. |
| `PositionAccuracyAuditMode` | `MazeMap/MazeMap/PositionAccuracyAuditMode.h` | wrapper shell | `3` | Same pattern as `MissionRunMode`, but forwarding into `MazeRunningAuditController`. |
| `CorridorRepeatabilityMode` | `MazeMap/MazeMap/CorridorRepeatabilityMode.h` | wrapper shell | `3` | Same forwarding shell pattern as the other boot-selected mode wrappers. |
| `MissionModeController` | `MazeMap/MazeMap/MissionModeController.h` | controller wrapper | `3` | Tiny public shell with all real behavior in a nested `Implementation`; combined with `MissionRunMode` this creates two public layers before the mode logic. |
| `MazeRunningAuditController` | `MazeMap/MazeMap/MazeRunningAuditController.h` | controller wrapper | `3` | Same controller-plus-hidden-implementation pattern, and it is itself wrapped again by two boot modes. |
| `ManeuverFileTestMode` | `MazeMap/MazeMap/ManeuverFileTestMode.h` | wrapper shell | `2` | The public type is just `Begin()`/`Run()` over a private `Implementation`; less layered than the mission/audit pairings, but still a shell. |
| `SharedRobotRuntime` | `MazeMap/MazeMap/MazeMapSharedRuntime.h` | PIMPL facade | `2` | Intended canonical owner, but the public type exposes a wide forwarding surface over a hidden `Implementation`, which can make ownership harder to trace. |
| `WallTouchRoutine` | `MazeMap/MazeMap/WallTouchRoutine.h` | wrapper shell | `3` | Packages callbacks and state locally, then delegates core per-tick wall-touch execution into `Runtime::DriveSharedWallTouchLoopTick(...)`. |
| `WallTouchRoutine::Hooks` | `MazeMap/MazeMap/WallTouchRoutine.h` | redirection helper | `2` | Function-pointer/context bundle used to bounce routine events out of the routine owner. |
| `Runtime::WallTouchObservation` | `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` | redirection helper | `2` | Repackages a subset of `SensorSnapshot` into another observation struct instead of using the canonical runtime snapshot directly. |
| `Runtime::WallTouchLoopHooks` | `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` | redirection helper | `3` | Another callback/context bag for trace, fault, completion, and pose-reset redirection around shared wall-touch execution. |
| `RuntimeSensorSuite::CaptureServices` | `MazeMap/MazeMap/RuntimeSensorSuite.h` | redirection helper | `2` | Small shell that re-exposes owner methods through stored function pointers and context. |
| `RuntimeSensorSuite::CaptureCallback` | `MazeMap/MazeMap/RuntimeSensorSuite.h` | redirection helper | `2` | Callback/context transport object for capture-time interleaving rather than a direct owner method contract. |
| `ManeuverExecutor::Hooks` | `MazeMap/MazeMap/ManeuverExecutor.h` | redirection helper | `2` | Public callback/context bag for queue-entry and sample hooks around maneuver execution. |
| `MapEvidenceUpdater` | `MazeMap/MazeMap/MapEvidenceUpdater.h` | helper owner | `3` | Holds a second edge-evidence map outside `Maze`, which looks like a helper-owned parallel maze representation. |
| `WallBeliefMap` | `MazeMap/MazeMap/WallBeliefMap.h` | helper owner | `3` | Stores authoritative-looking wall belief state in a separate map type instead of converging maze-domain state into `Maze`. |
| `WallDecisionAccumulator` | `MazeMap/MazeMap/WallObservationPipeline.h` | helper owner | `2` | Public helper class for wall-decision scoring that may be an implementation detail rather than stable public vocabulary. |
| `WallSensorPreprocessor` | `MazeMap/MazeMap/WallSensorPreprocessor.h` | helper layer | `2` | Standalone preprocessing stage between `WallSensor` and `WallObs`, which may be a split helper layer instead of behavior living with the authoritative owner. |
| `Application` | `MazeMap/MazeMap/MazeMapApplication.h` | entry wrapper | `1` | Very thin shell over active-mode resolution and runtime finalization. It may be acceptable as an entrypoint, but it is still mostly a wrapper. |

## Production Candidates: Parameter Bags and Split-Owner Support Types

| Type | File | Category | Confidence | Why it is suspicious |
| --- | --- | --- | --- | --- |
| `PlantParams` | `MazeMap/MazeMap/PlantModel.h` | parameter bag | `3` | Large public companion bag to `PlantModel`; mixes plant tuning with robot construction facts and sensor extrinsics that repo guidance wants owned by `Vehicle`. |
| `PlantPreparedParams` | `MazeMap/MazeMap/PlantModel.h` | prepared/cache bag | `3` | Public cache-like companion type that exposes `PlantModel` internals and duplicates `PlantParams` plus derived coefficients. |
| `VehiclePhysicalModel` | `MazeMap/MazeMap/Vehicle.h` | parameter bag | `3` | Public companion bag to `Vehicle` with raw fields rather than a single authoritative owner surface. |
| `ArcTrackWidthInterpolation` | `MazeMap/MazeMap/Vehicle.h` | companion bag | `2` | Nested support bag inside `VehiclePhysicalModel`; looks like peeled-off internals of `Vehicle`. |
| `MotorEncoderDrivePhysicalModel` | `MazeMap/MazeMap/MotorEncoderDrive.h` | parameter bag | `3` | Public physical-parameter bundle for one owner rather than behavior/state living on the owner. |
| `MotorEncoderDriveHardwareConfig` | `MazeMap/MazeMap/MotorEncoderDrive.h` | parameter bag | `3` | Public pin/config bag for `MotorEncoderDrive`, matching the repo's forbidden companion-config pattern. |
| `WheelControlProfile` | `MazeMap/MazeMap/WheelControlProfile.h` | tuning bag | `2` | Public profile struct used to shuttle wheel-control scales around instead of owning the policy inside the authoritative motion owner. |
| `InPlaceTurnProfile` | `MazeMap/MazeMap/InPlaceTurnProfile.h` | tuning bag | `2` | Public bag of in-place turn gains and tolerances; plausible typed config, but still a companion profile rather than behavior on the owner. |
| `SmoothTurnYawRateControllerState` | `MazeMap/MazeMap/SmoothTurnYawRateController.h` | state bag | `2` | Public state carrier for one helper function rather than hidden implementation state. |
| `WallSensor::DistanceModel` | `MazeMap/MazeMap/WallSensor.h` | companion bag | `2` | Public nested model bag for `WallSensor`; resembles peeled-off calibration internals. |
| `WallPreprocessorInput` | `MazeMap/MazeMap/WallSensorPreprocessor.h` | input bag | `2` | Public evidence bundle created mainly to feed one preprocessor owner. |
| `WallSensorPreprocessorConfig` | `MazeMap/MazeMap/WallSensorPreprocessor.h` | config bag | `3` | Public config companion for `WallSensorPreprocessor`, with many fields that look like implementation tuning rather than domain vocabulary. |
| `MapEvidenceUpdaterConfig` | `MazeMap/MazeMap/MapEvidenceUpdater.h` | config bag | `3` | Public tuning bag for one helper owner that already looks like a second maze-state system. |
| `EdgeEvidence` | `MazeMap/MazeMap/MapEvidenceUpdater.h` | state bag | `2` | Public state record for `MapEvidenceUpdater`, exposing internal accumulation state directly. |
| `WallEvidenceConfig` | `MazeMap/MazeMap/WallObservationPipeline.h` | config bag | `2` | Public configuration bundle for a pipeline helper rather than a stable cross-subsystem domain type. |
| `WallBeliefConfig` | `MazeMap/MazeMap/WallBeliefMap.h` | config bag | `3` | Public config companion for the separate wall-belief owner. |
| `WallBeliefState` | `MazeMap/MazeMap/WallBeliefMap.h` | state bag | `2` | Public companion state type exposing one owner's storage layout. |
| `WallBeliefUpdate` | `MazeMap/MazeMap/WallBeliefMap.h` | update/result bag | `2` | Public update record for `WallBeliefMap`, again exposing owner internals as a support type. |
| `OpenFloorMeasurementLabels` | `MazeMap/MazeMap/OpenFloorMeasurementLabels.h` | metadata bag | `3` | Public field bag for logging labels. It is not an allowed `mmlog` schema, but it behaves like one. |
| `OpenFloorMeasurementCycle` | `MazeMap/MazeMap/OpenFloorMeasurementCycle.h` | sample bag | `3` | Large public cycle snapshot bag used for logging and controller bookkeeping outside the designated `mmlog` schema mechanism. |
| `PositionAuditFixtureGeometry` | `MazeMap/MazeMap/AuxMeasurementModeSupport.h` | geometry bag | `3` | Public helper-owned bundle containing a `Maze` plus derived geometry, which looks like a mode-support mirror of authoritative maze facts. |
| `SearchStraightPlan` | `MazeMap/MazeMap/SearchRunPlanner.h` | plan bag | `2` | Public result bag for one planning helper; may be justified, but it follows the "support type mainly for one owner's pipeline" pattern. |
| `SearchReplanResponse` | `MazeMap/MazeMap/SearchRunPlanner.h` | response bag | `2` | Same concern as `SearchStraightPlan`: a public support type mostly serving a narrow helper pipeline. |
| `Runtime::WallTouchLoopState` | `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` | state bag | `2` | Large public/internal state bag used to shuttle wall-touch execution state across helper boundaries instead of hiding it in one owner. |

## Lower-Confidence Framework-Contract Shapes

These may be acceptable because they live inside a single framework owner, but they still match the "public support type" pattern closely enough to keep on the watch list.

| Type | File | Category | Confidence | Why it is suspicious |
| --- | --- | --- | --- | --- |
| `LoopController::SensorWorkPlan` | `MazeMap/MazeMap/LoopController.h` | config bag | `1` | Public control-loop work-plan bundle; may be justified as framework contract, but it is still a public bag for one owner. |
| `LoopController::SessionOptions` | `MazeMap/MazeMap/LoopController.h` | config bag | `1` | Public session-setup bag rather than a more constrained owner API. |
| `LoopController::PauseRequest` | `MazeMap/MazeMap/LoopController.h` | request bag | `1` | Public request carrier used to steer pause behavior through callbacks. |
| `LoopController::PauseContext` | `MazeMap/MazeMap/LoopController.h` | context bag | `1` | Public context carrier for pause callbacks. |
| `LoopController::PauseDisposition` | `MazeMap/MazeMap/LoopController.h` | response bag | `1` | Public response carrier for pause callbacks. |
| `LoopController::ModeCallbacks` | `MazeMap/MazeMap/LoopController.h` | redirection helper | `1` | Public callback/context bundle used for control transfer. |
| `LoopController::MotorPwmSink` | `MazeMap/MazeMap/LoopController.h` | redirection helper | `1` | Private adapter that routes raw motor PWM through a stored callback and context. |
| `LoopController::ObservedTickState` | `MazeMap/MazeMap/LoopController.h` | duplicated state bag | `1` | Near-duplicate of `ModeState` kept as a second internal carrier. |

## Test-Only Support Types

These are not production architecture violations, but they follow the same shape patterns and are worth separating from the production list.

| Type | File | Category | Confidence | Why it is suspicious |
| --- | --- | --- | --- | --- |
| `Mazes` | `MazeMap/MazeMapTest/MazeRef.h` | helper registry | `2` | Static helper registry that owns shared test mazes through one support type rather than direct fixture construction. |
| `InitialStationaryGyroBiasExpectation` | `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h` | expectation bag | `1` | Test-only state bag for one helper pipeline. |
| `RuntimeTuningRestoreScope` | `MazeMap/MazeMapTest/VehicleStateTest.cpp` | scope helper | `1` | Small test helper that exists only to redirect runtime-tuning cleanup. |
| `ScopedUkfRuntimeTuningRestore` | `MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp` | scope helper | `1` | Same scope-restore helper pattern as above. |
| `HistoricalMazeCase` | `MazeMap/MazeMapTest/HistoricalMazePathFinderTest.cpp` | case bag | `1` | Test catalog support bag. |
| `HistoricalMazeCatalog` | `MazeMap/MazeMapTest/HistoricalMazePathFinderTest.cpp` | helper catalog | `1` | Test-only catalog helper that groups a support pipeline into one type. |

## Review Notes

- The strongest wrapper-shell cluster is the boot-mode stack around `MissionRunMode` / `MissionModeController` and `PositionAccuracyAuditMode` / `CorridorRepeatabilityMode` / `MazeRunningAuditController`.
- The strongest parameter-bag cluster is the plant/vehicle/motor configuration surface: `PlantParams`, `PlantPreparedParams`, `VehiclePhysicalModel`, `MotorEncoderDrivePhysicalModel`, and `MotorEncoderDriveHardwareConfig`.
- The strongest redirection-helper cluster is the callback/context plumbing around `WallTouchRoutine`, `RuntimeSensorSuite`, `ManeuverExecutor`, and `LoopController`.
- The strongest split-owner/helper cluster is the maze-evidence surface: `MapEvidenceUpdater`, `WallBeliefMap`, and their companion config/state types.
