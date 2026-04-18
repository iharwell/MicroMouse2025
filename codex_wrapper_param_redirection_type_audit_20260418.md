# Wrapper / Parameter Bag / Redirection-Helper Type Audit

Date: 2026-04-18

Scope:
- `MazeMap/MazeMap`
- `MazeMap/MazeMapTest`

Method:
- Local declaration sweep across production and test code
- Manual review of the highest-signal wrapper and parameter-bag candidates
- Parallel agent passes to widen coverage; the final list below is still marked as `potential`, not a final architectural verdict

Confidence:
- `3`: strong reviewed candidate
- `2`: plausible candidate with direct naming/usage evidence
- `1`: broad-sweep candidate worth auditing because it contributes to compiled helper/state/config surface

Deliberate exclusions:
- I did not list obvious canonical owners such as `Vehicle`, `PlantModel`, `Maze`, `SharedRobotRuntime`, `BootModeRegistry`, or `BootModeDescriptor` just because they exist.
- I also did not list every tiny one-off local sample struct in tests.

## Reviewed Strong Production Hits

| Type | File | Kind | Conf | Why it is on the list |
| --- | --- | --- | --- | --- |
| `Application` | `MazeMap/MazeMap/MazeMapApplication.h` | wrapper shell | 3 | `Setup()` resolves the active mode, calls `Begin()` / `Run()`, then finalizes runtime exit. It is a very thin top-level wrapper rather than an obvious authoritative owner. |
| `MissionRunMode` | `MazeMap/MazeMap/MissionRunMode.h` | wrapper shell | 3 | Boot-selected mode shell that only forwards `Begin()` / `Run()` into `MissionModeController`. |
| `MissionRunMode::Implementation` | `MazeMap/MazeMap/MissionRunMode.cpp` | pimpl shell | 3 | Private implementation object that only owns a `MissionModeController`, making the outer mode even thinner. |
| `MissionModeController` | `MazeMap/MazeMap/MissionModeController.h` | controller layer | 3 | Large separate controller layer beneath `MissionRunMode`; likely an extra owner boundary instead of the boot-selected mode being the direct authority. |
| `CorridorRepeatabilityMode` | `MazeMap/MazeMap/CorridorRepeatabilityMode.h` | wrapper shell | 3 | Thin boot-mode shell forwarding into `MazeRunningAuditController`. |
| `CorridorRepeatabilityMode::Implementation` | `MazeMap/MazeMap/CorridorRepeatabilityMode.cpp` | pimpl shell | 3 | Holds only `MazeRunningAuditController`. |
| `PositionAccuracyAuditMode` | `MazeMap/MazeMap/PositionAccuracyAuditMode.h` | wrapper shell | 3 | Same shell pattern as `CorridorRepeatabilityMode`, just routed to a different routine on the same controller layer. |
| `PositionAccuracyAuditMode::Implementation` | `MazeMap/MazeMap/PositionAccuracyAuditMode.cpp` | pimpl shell | 3 | Holds only `MazeRunningAuditController`. |
| `MazeRunningAuditController` | `MazeMap/MazeMap/MazeRunningAuditController.h` | controller layer | 3 | Shared controller under two boot modes. Plausible extra architectural layer instead of one authoritative mode owner or framework-owned shared helpers. |
| `ManeuverFileTestMode` | `MazeMap/MazeMap/ManeuverFileTestMode.h` | wrapper shell | 3 | Another boot-selected mode class that exists largely to delegate into a hidden implementation object. |
| `MouseUkfFacade` | `MazeMap/MazeMap/MouseUkfFacade.h` | facade / wrapper | 3 | Exposes `ukf()` and `mapEvidence()` and mainly forwards to `SrUkfCore` while coordinating `MapEvidenceUpdater`. High-probability facade smell by both name and implementation shape. |
| `WallTouchRoutine` | `MazeMap/MazeMap/WallTouchRoutine.h` | routine helper shell | 3 | Wraps `Runtime::DriveSharedWallTouchLoopTick(...)`, owns callback hooks, and redirects continuation control. Strong helper/redirection candidate. |
| `WallTouchRoutine::Hooks` | `MazeMap/MazeMap/WallTouchRoutine.h` | callback redirection bag | 3 | Public callback/context bundle created to route tracing, sampling, and pose-reset behavior through the routine helper. |
| `WallDistanceCalibration` | `MazeMap/MazeMap/WallDistanceCalibration.h` | split-owner / fake entry surface | 3 | The type lives in `MazeMapRuntimeCore.h`, but `WallDistanceCalibration.h` exists as an entry header that only includes the giant runtime core header. That is explicitly close to the repo’s “entry header” / fake-surface smell. |
| `PlantParams` | `MazeMap/MazeMap/PlantModel.h` | parameter bag | 3 | Public parameter bag carrying vehicle geometry, mass, friction, motor, tire, and sensor facts that overlap heavily with `Vehicle` ownership. |
| `PlantPreparedParams` | `MazeMap/MazeMap/PlantModel.h` | prepared cache bag | 3 | Public prepared/cache struct for plant hot-path work. The repo guidelines call out companion `Params` / cache types as suspect, especially when public. |
| `VehiclePhysicalModel` | `MazeMap/MazeMap/Vehicle.h` | parameter bag | 3 | Public bag of vehicle construction facts. Even though `Vehicle` owns it, this is still a public extracted fact bag with public fields. |
| `ArcTrackWidthInterpolation` | `MazeMap/MazeMap/Vehicle.h` | companion bag | 3 | Small companion struct hanging off `VehiclePhysicalModel`, used to externalize a narrow internal representation detail. |
| `MotorEncoderDrivePhysicalModel` | `MazeMap/MazeMap/MotorEncoderDrive.h` | split physical-model owner | 3 | Separate physical-model bag for motor/drive facts, outside `Vehicle` and `PlantModel`. Plausible duplicate ownership of construction/runtime facts. |
| `MotorEncoderDriveHardwareConfig` | `MazeMap/MazeMap/MotorEncoderDrive.h` | hardware config bag | 3 | Separate pin/config owner with public fields. Plausible duplicate hardware-fact ownership instead of one canonical vehicle/platform owner. |

## Broad Production Inventory

These are weaker than the reviewed hits above, but they match the repo’s anti-patterns closely enough to warrant inspection.

| Type | File | Kind | Conf | Why it is on the list |
| --- | --- | --- | --- | --- |
| `MapEvidenceUpdaterConfig` | `MazeMap/MazeMap/MapEvidenceUpdater.h` | config bag | 2 | Public thresholds/weights bag for one owner’s internal update policy. |
| `WallEvidenceConfig` | `MazeMap/MazeMap/WallObservationPipeline.h` | config bag | 2 | Another public thresholds/weights bag for wall-decision accumulation. |
| `WallBeliefConfig` | `MazeMap/MazeMap/WallBeliefMap.h` | config bag | 2 | Public log-odds tuning bundle externalized from the owning map. |
| `WallBeliefState` | `MazeMap/MazeMap/WallBeliefMap.h` | public internal state | 2 | Public state shape for one map owner’s internal belief cell representation. |
| `WallBeliefUpdate` | `MazeMap/MazeMap/WallBeliefMap.h` | update/result bag | 2 | Public response struct exposing internal update pipeline state. |
| `WallSensorPreprocessorConfig` | `MazeMap/MazeMap/WallSensorPreprocessor.h` | config bag | 2 | Threshold/calibration bag external to the preprocessor owner. |
| `WallPreprocessorInput` | `MazeMap/MazeMap/WallSensorPreprocessor.h` | input bag | 2 | Multi-field input transport object for the preprocessor pipeline. |
| `WheelControlProfile` | `MazeMap/MazeMap/WheelControlProfile.h` | profile bag | 2 | Public wheel tuning profile with normalization helpers around it. |
| `InPlaceTurnProfile` | `MazeMap/MazeMap/InPlaceTurnProfile.h` | profile bag | 2 | Public turn-control tuning bag with helper functions operating on it. |
| `SmoothTurnYawRateControllerState` | `MazeMap/MazeMap/SmoothTurnYawRateController.h` | peeled-off controller state | 2 | Explicit controller state bag rather than state owned privately inside a controller. |
| `PositionAuditFixtureGeometry` | `MazeMap/MazeMap/AuxMeasurementModeSupport.h` | helper geometry bag | 2 | Utility-mode fixture geometry bundle created by support helpers instead of living with a single authoritative owner. |
| `FrontWallCharacterizationStorage` | `MazeMap/MazeMap/FrontWallCharacterizationStorage.h` | storage bag | 2 | Public storage/result bag carrying characterization data between systems. |
| `FrontWallCharacterizationMatch` | `MazeMap/MazeMap/FrontWallCharacterizationStorage.h` | result bag | 2 | Companion result bag tied tightly to `FrontWallCharacterizationStorage`. |
| `SearchStraightPlan` | `MazeMap/MazeMap/SearchRunPlanner.h` | planning bag | 2 | Small plan bundle returned by planner helpers rather than behavior staying inside one owner. |
| `SearchReplanResponse` | `MazeMap/MazeMap/SearchRunPlanner.h` | planning response bag | 2 | Similar peeled-off response bag. |
| `MeasurementUpdateResult` | `MazeMap/MazeMap/SrUkfCore.h` | result bag | 2 | Public filter update response type exposing internal measurement pipeline output. |
| `WallUpdateResult` | `MazeMap/MazeMap/SrUkfCore.h` | result bag | 2 | Same issue for wall updates. |
| `FrontPairUpdateResult` | `MazeMap/MazeMap/SrUkfCore.h` | result bag | 2 | Same issue for front-pair updates. |
| `ModeProcessNoiseTuning` | `MazeMap/MazeMap/SrUkfCore.h` | tuning bag | 2 | Public tuning sub-struct for core internals. |
| `RuntimeTuning` | `MazeMap/MazeMap/SrUkfCore.h` | runtime tuning bag | 2 | Public runtime tuning bundle for one filter owner. |
| `OpenFloorMeasurementLabels` | `MazeMap/MazeMap/OpenFloorMeasurementLabels.h` | label bag | 2 | Phase/section/primitive metadata bundle rather than a single authoritative measurement owner. |
| `OpenFloorMeasurementCycle` | `MazeMap/MazeMap/OpenFloorMeasurementCycle.h` | large cycle bundle | 2 | Captured cycle-state bundle aggregating timing, drivetrain, sensor, and diagnostic data. |
| `OpenFloorSectionDefinition` | `MazeMap/MazeMap/OpenFloorMeasurementSpec.h` | spec bag | 2 | Public section-definition data shape tied to one measurement subsystem. |
| `OpenFloorPrimitiveDefinition` | `MazeMap/MazeMap/OpenFloorMeasurementSpec.h` | spec bag | 2 | Public primitive-definition data shape tied to one measurement subsystem. |
| `LoopController::SensorWorkPlan` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public work-plan bag around one loop owner’s cadence/sensing responsibilities. |
| `LoopController::SessionOptions` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public session configuration bag. |
| `LoopController::SessionResult` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public session result bag. |
| `LoopController::TimingDiagnostics` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public diagnostic bundle exposing loop internals. |
| `LoopController::ModeState` | `MazeMap/MazeMap/LoopController.h` | public companion state | 2 | Large public state bundle handed to callbacks each tick. |
| `LoopController::PauseContext` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public pause-state bag. |
| `LoopController::PauseDisposition` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public pause-result bag. |
| `LoopController::PauseRequest` | `MazeMap/MazeMap/LoopController.h` | public companion bag | 2 | Public pause request object. |
| `LoopController::ModeCallbacks` | `MazeMap/MazeMap/LoopController.h` | redirection bag | 2 | Callback/context redirection bundle used to move control ownership around. |
| `LoopController::TickServices` | `MazeMap/MazeMap/LoopController.h` | helper service surface | 2 | Service/helper contract exposed to callbacks; plausible extra public support type for one owner. |
| `ManeuverExecutor::Hooks` | `MazeMap/MazeMap/ManeuverExecutor.h` | callback redirection bag | 2 | Public callback/context bundle used to redirect sampling and queue-entry events through the maneuver executor. |
| `RuntimeSensorSuite::CaptureServices` | `MazeMap/MazeMap/RuntimeSensorSuite.h` | redirection helper | 2 | Service object wrapping function pointers and context for staged capture work. |
| `RuntimeSensorSuite::CaptureCallback` | `MazeMap/MazeMap/RuntimeSensorSuite.h` | callback redirection bag | 2 | Another callback/context indirection object for one owner’s internal capture pipeline. |
| `DriveBase::CommandContext` | `MazeMap/MazeMap/DriveBase.h` | companion bag | 2 | Public/declared context type for drive commands. |
| `DriveBase::CommandTargets` | `MazeMap/MazeMap/DriveBase.h` | companion bag | 2 | Public/declared target bundle for drive command resolution. |
| `DriveBase::ClosedLoopVelocityCommand` | `MazeMap/MazeMap/DriveBase.h` | companion bag | 2 | Public/declared command transport type. |
| `DriveBase::ResolvedVelocityDriveSignal` | `MazeMap/MazeMap/DriveBase.h` | result bag | 2 | Resolved drive-signal bundle peeled out from `DriveBase`. |
| `DriveBase::WheelLaunchAssistState` | `MazeMap/MazeMap/DriveBase.h` | peeled-off state | 2 | Explicit launch-assist state bag. |

## Compiled Private State / Helper Inventory

These are mostly private or file-local, so some may be acceptable. They are still worth naming because they represent exactly the kind of compiled helper/state footprint the repo guidelines are trying to collapse.

| Type | File | Kind | Conf | Why it is on the list |
| --- | --- | --- | --- | --- |
| `MissionModeController::InterRunServicePauseLoopState` | `MazeMap/MazeMap/MissionModeController.cpp` | private state bag | 1 | Private per-phase loop state compiled into the controller layer. |
| `MissionModeController::QueuedManeuverLoopState` | `MazeMap/MazeMap/MissionModeController.cpp` | private state bag | 1 | Same. |
| `MissionModeController::StartupStationaryHoldLoopState` | `MazeMap/MazeMap/MissionModeController.cpp` | private state bag | 1 | Same. |
| `MissionModeController::ObservationCaptureLoopState` | `MazeMap/MazeMap/MissionModeController.cpp` | private state bag | 1 | Same. |
| `MissionModeController::FrontCalibrationSweepLoopState` | `MazeMap/MazeMap/MissionModeController.cpp` | private state bag | 1 | Same. |
| `MissionModeController::SearchStraightLoopState` | `MazeMap/MazeMap/MissionModeController.cpp` | private state bag | 1 | Same. |
| `MazeRunningAuditController::InterRunServicePauseLoopState` | `MazeMap/MazeMap/MazeRunningAuditController.cpp` | private state bag | 1 | Private per-phase loop state compiled into the shared audit controller layer. |
| `MazeRunningAuditController::QueuedManeuverLoopState` | `MazeMap/MazeMap/MazeRunningAuditController.cpp` | private state bag | 1 | Same. |
| `MazeRunningAuditController::StartupStationaryHoldLoopState` | `MazeMap/MazeMap/MazeRunningAuditController.cpp` | private state bag | 1 | Same. |
| `MazeRunningAuditController::FrontCalibrationSweepLoopState` | `MazeMap/MazeMap/MazeRunningAuditController.cpp` | private state bag | 1 | Same. |
| `ManeuverExecutor::HoldRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Routine-specific state extracted into named support types. |
| `ManeuverExecutor::SettleRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `ManeuverExecutor::ReverseStraightRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `ManeuverExecutor::StraightRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `ManeuverExecutor::TurnRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `ManeuverExecutor::ArcRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `ManeuverExecutor::SmoothTurnRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `ManeuverExecutor::QueueRoutineState` | `MazeMap/MazeMap/ManeuverExecutor.h` | private state bag | 1 | Same. |
| `WallTouchRoutine::SettleRoutineState` | `MazeMap/MazeMap/WallTouchRoutine.h` | private state bag | 1 | Private settle-state object paired with the routine helper shell. |
| `RuntimeSensorSuite::FilteredIrChannel` | `MazeMap/MazeMap/RuntimeSensorSuite.h` | private channel state | 1 | Private repeated state bundle for the runtime sensor helper pipeline. |
| `RuntimeSensorSuite.cpp::CaptureContext` | `MazeMap/MazeMap/RuntimeSensorSuite.cpp` | local helper bag | 1 | File-local callback/service context for capture redirection. |
| `LoopController.cpp::CaptureContext` | `MazeMap/MazeMap/LoopController.cpp` | local helper bag | 1 | File-local callback/service context for capture redirection. |
| `OpenFloorMeasurementController::PendingLog` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private helper bag | 1 | File-local logging/phase helper bag. |
| `OpenFloorMeasurementController::TimingBlockState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Private section-state object. |
| `OpenFloorMeasurementController::StaticHoldState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::RecoveryState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::LaunchPulseState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::StraightSectionState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::SectionSettleState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::TurnSectionState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::SmoothTurnState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private phase state | 1 | Same. |
| `OpenFloorMeasurementController::LaunchSequenceState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private sequence state | 1 | Same. |
| `OpenFloorMeasurementController::StraightSequenceState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private sequence state | 1 | Same. |
| `OpenFloorMeasurementController::YawSequenceState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private sequence state | 1 | Same. |
| `OpenFloorMeasurementController::SmoothSequenceState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private sequence state | 1 | Same. |
| `OpenFloorMeasurementController::LoopSequenceState` | `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | private sequence state | 1 | Same. |
| `DiagnosticController::HoldPhaseState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private phase state | 1 | Private diagnostic phase/sequence support type. |
| `DiagnosticController::StraightPhaseState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private phase state | 1 | Same. |
| `DiagnosticController::KickoffPhaseState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private phase state | 1 | Same. |
| `DiagnosticController::KickoffSweepSequenceState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private sequence state | 1 | Same. |
| `DiagnosticController::ForwardPhaseState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private phase state | 1 | Same. |
| `DiagnosticController::ForwardSweepSequenceState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private sequence state | 1 | Same. |
| `DiagnosticController::TurnPhaseState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private phase state | 1 | Same. |
| `DiagnosticController::ArcPhaseState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private phase state | 1 | Same. |
| `DiagnosticController::CharacterizationRecoveryState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private recovery state | 1 | Same. |
| `DiagnosticController::CircleSequenceState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private sequence state | 1 | Same. |
| `DiagnosticController::SquareSequenceState` | `MazeMap/MazeMap/DiagnosticController.cpp` | private sequence state | 1 | Same. |
| `FrontWallCharacterizationController::HoldPhaseState` | `MazeMap/MazeMap/FrontWallCharacterizationController.cpp` | private phase state | 1 | Private characterization helper state. |
| `FrontWallCharacterizationController::CaptureCurveState` | `MazeMap/MazeMap/FrontWallCharacterizationController.cpp` | private phase state | 1 | Same. |
| `AuxMeasurementController::HoldPhaseState` | `MazeMap/MazeMap/AuxMeasurementController.cpp` | private phase state | 1 | Private auxiliary helper state. |
| `AuxMeasurementController::TurningTractionState` | `MazeMap/MazeMap/AuxMeasurementController.cpp` | private phase state | 1 | Same. |

## Test-Only Hits

| Type | File | Kind | Conf | Why it is on the list |
| --- | --- | --- | --- | --- |
| `Mazes` | `MazeMap/MazeMapTest/MazeRef.h` | wrapper / alternate access path | 3 | Test-only static maze registry around encoded mazes; creates a parallel maze-loading surface. |
| `InitialStationaryGyroBiasExpectation` | `MazeMap/MazeMapTest/SrUkfCoreTestSupport.h` | mirrored state bag | 3 | Test support struct that mirrors estimator transition state and expectations. |
| `HistoricalMazeCatalog` | `MazeMap/MazeMapTest/HistoricalMazePathFinderTest.cpp` | helper registry | 2 | Catalog/registry layer around historical maze corpus metadata and parsed cases. |
| `HistoricalMazeCase` | `MazeMap/MazeMapTest/HistoricalMazePathFinderTest.cpp` | parameter bag | 2 | Test-case bag combining scenario metadata and maze data. |
| `RuntimeTuningRestoreScope` | `MazeMap/MazeMapTest/VehicleStateTest.cpp` | helper redirection | 2 | File-local RAII restore helper around shared runtime tuning. |
| `ScopedUkfRuntimeTuningRestore` | `MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp` | helper redirection | 2 | Duplicate restore-helper pattern for the same shared runtime tuning redirection. |
| `FaultCallbackState` | `MazeMap/MazeMapTest/MazeMapSharedRuntimeTest.cpp` | callback redirection bag | 1 | Test callback context bundle used to redirect fault reporting. |
| `StreamingFaultCallbackState` | `MazeMap/MazeMapTest/MazeMapSharedRuntimeTest.cpp` | callback redirection bag | 1 | Same pattern for streaming-fault assertions. |

## Notes

- The strongest public-shape problems appear to be:
  - boot-mode shell classes around separate controller layers,
  - public `*Params` / `*Config` / `*Profile` / `*State` / `*Update` / `*Labels` / `*Cycle` bags,
  - callback/context helper structs used to redirect control instead of letting the owner own the behavior directly.
- The biggest compiled-code bucket is the private phase/state/helper surface in the controller and measurement implementations. Those are not all equally bad, but they do match the repository’s “state bag / helper family” cleanup target.
