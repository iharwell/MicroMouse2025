# Broad Production `struct` Sweep

Date: 2026-04-18

Intent:
- This is a broader follow-up to the earlier audit.
- The bias here is intentionally simple: if production code chose `struct`, it is in scope unless it is very clearly a canonical domain/value type or a required framework contract.
- This file is therefore less curated and more useful as a raw cleanup working set.

Scope:
- `MazeMap/MazeMap` only
- production code only
- declaration form: `struct ...`

Quick take:
- The densest likely-cleanup surfaces are still the controller/mode `.cpp` files, `LoopController.h`, `DriveBase.h`, `ManeuverExecutor.h`, `PlantModel.h`, `MazeMapRuntimeInfrastructure.h`, and `MazeMapRuntimeCore.h`.
- If your working heuristic is “`struct` is already suspicious,” those files are the fastest places to keep digging.

## Struct Density By File

| File | Struct count | Dominant shape |
| --- | ---: | --- |
| `MazeMap/MazeMap/MazeMapRuntimeCore.h` | 22 | runtime transport / calibration / async helper / cache |
| `MazeMap/MazeMap/DiagnosticController.cpp` | 14 | private phase-state / sequence-state / metrics |
| `MazeMap/MazeMap/LoopController.h` | 14 | public control/session/callback carrier types |
| `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | 14 | private phase-state / sequence-state / logging |
| `MazeMap/MazeMap/DriveBase.h` | 10 | drive command/state transport |
| `MazeMap/MazeMap/ManeuverExecutor.h` | 10 | callback bags + private routine state |
| `MazeMap/MazeMap/PlantModel.h` | 9 | params / prepared params / contact/result carriers |
| `MazeMap/MazeMap/PlantModel.cpp` | 8 | private plant helper/result state |
| `MazeMap/MazeMap/VehicleState.h` | 7 | extrinsics / observation / control carriers |
| `MazeMap/MazeMap/MissionModeController.cpp` | 6 | private loop/phase state |
| `MazeMap/MazeMap/MmLog.h` | 6 | logger internals / metadata carriers |
| `MazeMap/MazeMap/SrUkfCore.h` | 5 | result + tuning carriers |
| `MazeMap/MazeMap/UKF.h` | 5 | math/helper carriers |
| `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` | 4 | wall-touch result/state/hook carriers |
| `MazeMap/MazeMap/MazeRunningAuditController.cpp` | 4 | private loop/phase state |

## High-Density Files

### `MazeMap/MazeMap/MazeMapRuntimeCore.h` (22)
Bucket: runtime transport / calibration / async helper / cache

- `MotionLimits`
- `PoseEstimate`
- `OpticalObservationTiming`
- `ImuObservationTiming`
- `WallSensorTelemetry`
- `ImuTelemetry`
- `SensorSnapshot`
- `DriveTelemetry`
- `ControlCycleTiming`
- `EncoderProgressWatchdog`
- `RawWallSensorSample`
- `WallSensorCalibrationInput`
- `RobustSignalBand`
- `WallSensorCalibrationCapture`
- `FrontCalibrationSpinSampleSet`
- `AveragedWallSensorInputWindow`
- `AveragedBackLeftImuSample`
- `FrontSignalModelCache`
- `AsyncWallSensorPairRead`
- `AsyncWallSensorSweepRead`
- `NamedCode`
- `RollingObservationVoteSummary`

### `MazeMap/MazeMap/DiagnosticController.cpp` (14)
Bucket: private phase-state / sequence-state / metrics

- `StraightPhaseMetrics`
- `TurnPhaseMetrics`
- `ArcPhaseMetrics`
- `HoldPhaseState`
- `StraightPhaseState`
- `KickoffPhaseState`
- `KickoffSweepSequenceState`
- `ForwardPhaseState`
- `ForwardSweepSequenceState`
- `TurnPhaseState`
- `ArcPhaseState`
- `CharacterizationRecoveryState`
- `CircleSequenceState`
- `SquareSequenceState`

### `MazeMap/MazeMap/LoopController.h` (14)
Bucket: public control/session/callback carrier types

- `SensorWorkPlan`
- `SessionOptions`
- `SessionResult`
- `ControlVector`
- `TimingDiagnostics`
- `MeasuredMotion`
- `ModeState`
- `PauseContext`
- `PauseDisposition`
- `PauseRequest`
- `ModeCallbacks`
- `ObservedTickState`
- `LatchedRequests`
- `MotorPwmSink`

### `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` (14)
Bucket: private phase-state / sequence-state / logging

- `PendingLog`
- `TimingBlockState`
- `StaticHoldState`
- `RecoveryState`
- `LaunchPulseState`
- `StraightSectionState`
- `SectionSettleState`
- `TurnSectionState`
- `SmoothTurnState`
- `LaunchSequenceState`
- `StraightSequenceState`
- `YawSequenceState`
- `SmoothSequenceState`
- `LoopSequenceState`

### `MazeMap/MazeMap/DriveBase.h` (10)
Bucket: drive command/state transport

- `CommandContext`
- `CommandTargets`
- `ClosedLoopVelocityCommand`
- `MeasuredKinematics`
- `EncoderCycleSample`
- `ResolvedVelocityDriveSignal`
- `WheelLaunchAssistState`

Note:
- `CommandContext`, `CommandTargets`, and `ClosedLoopVelocityCommand` appear as forward declarations and later full definitions in the same header.

### `MazeMap/MazeMap/ManeuverExecutor.h` (10)
Bucket: callback bags + private routine state

- `TurnWallEdgeTracker` (forward declaration)
- `Hooks`
- `HoldRoutineState`
- `SettleRoutineState`
- `ReverseStraightRoutineState`
- `StraightRoutineState`
- `TurnRoutineState`
- `ArcRoutineState`
- `SmoothTurnRoutineState`
- `QueueRoutineState`

### `MazeMap/MazeMap/PlantModel.h` (9)
Bucket: params / prepared params / contact/result carriers

- `ContactKinematics`
- `WheelKinematics`
- `SlipTargets`
- `ContactForce`
- `ContactForces`
- `PlantDerivatives`
- `DriveCommandSolution`
- `PlantParams`
- `PlantPreparedParams`

### `MazeMap/MazeMap/PlantModel.cpp` (8)
Bucket: private plant helper/result state

- `MotionMetrics`
- `ContactLoads`
- `PeakFrictionCoefficients`
- `SplitForceRequest`
- `RollingContactEvaluation`
- `RollingStateEvaluation`
- `ModeTransition`
- `VelocityTargetExactSolution`

### `MazeMap/MazeMap/VehicleState.h` (7)
Bucket: extrinsics / observation / control carriers

- `SensorExtrinsics`
- `ImuExtrinsics`
- `ControlInput`
- `EncoderObs`
- `ImuObservation`
- `ImuAccelObs`
- `WallObs`

### `MazeMap/MazeMap/MissionModeController.cpp` (6)
Bucket: private loop/phase state

- `InterRunServicePauseLoopState`
- `QueuedManeuverLoopState`
- `StartupStationaryHoldLoopState`
- `ObservationCaptureLoopState`
- `FrontCalibrationSweepLoopState`
- `SearchStraightLoopState`

### `MazeMap/MazeMap/MmLog.h` (6)
Bucket: logger internals / metadata carriers

- `FieldDescriptor`
- `FieldTraits` (forward declaration)
- `has_row_contract`
- `MetadataEntry`
- `MMLOG_PACKED RowName` (macro-generated declaration form)

Note:
- `has_row_contract` appears in a primary template and a specialization.

### `MazeMap/MazeMap/SrUkfCore.h` (5)
Bucket: result + tuning carriers

- `MeasurementUpdateResult`
- `WallUpdateResult`
- `FrontPairUpdateResult`
- `ModeProcessNoiseTuning`
- `RuntimeTuning`

### `MazeMap/MazeMap/UKF.h` (5)
Bucket: math/helper carriers

- `NoopUkfLoopHook`
- `UkfStorageOrder`
- `UkfFloatOps`
- `SrUkfWeights`
- `SrUkfMath`

### `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` (4)
Bucket: wall-touch result/state/hook carriers

- `WallTouchObservation`
- `WallTouchExecutionResult`
- `WallTouchLoopState`
- `WallTouchLoopHooks`

### `MazeMap/MazeMap/MazeRunningAuditController.cpp` (4)
Bucket: private loop/phase state

- `InterRunServicePauseLoopState`
- `QueuedManeuverLoopState`
- `StartupStationaryHoldLoopState`
- `FrontCalibrationSweepLoopState`

## Remaining Files With `struct` Declarations

### 3-struct files

#### `MazeMap/MazeMap/Defines.h`
Bucket: host/platform helper carriers

- `HostDigitalPinState`
- `FlexPwmPinInfo`
- `EncoderSlot`

#### `MazeMap/MazeMap/OpenFloorMeasurementSpec.h`
Bucket: measurement spec carriers

- `OpenFloorMarkerPose`
- `OpenFloorSectionDefinition`
- `OpenFloorPrimitiveDefinition`

#### `MazeMap/MazeMap/WallBeliefMap.h`
Bucket: belief config/state/update carriers

- `WallBeliefConfig`
- `WallBeliefState`
- `WallBeliefUpdate`

### 2-struct files

#### `MazeMap/MazeMap/AuxMeasurementController.cpp`
Bucket: private phase state

- `HoldPhaseState`
- `TurningTractionState`

#### `MazeMap/MazeMap/BootModeRegistry.h`
Bucket: canonical registry metadata, but still `struct`

- `BootModeSelectorCondition`
- `BootModeRegistryEntry`

#### `MazeMap/MazeMap/FrontWallCharacterizationController.cpp`
Bucket: private phase state

- `HoldPhaseState`
- `CaptureCurveState`

#### `MazeMap/MazeMap/FrontWallCharacterizationStorage.h`
Bucket: storage/result carriers

- `FrontWallCharacterizationStorage`
- `FrontWallCharacterizationMatch`

#### `MazeMap/MazeMap/LSM6DSV16X_IMU.h`
Bucket: device data carriers

- `Axes` (appears twice in the file)

#### `MazeMap/MazeMap/MapEvidenceUpdater.h`
Bucket: evidence config/state carriers

- `EdgeEvidence`
- `MapEvidenceUpdaterConfig`

#### `MazeMap/MazeMap/MazeMapRuntimeSignalHelpers.h`
Bucket: forward-declared runtime transport types

- `PoseEstimate` (forward declaration)
- `SensorSnapshot` (forward declaration)

#### `MazeMap/MazeMap/MotorEncoderDrive.h`
Bucket: physical/hardware config carriers

- `MotorEncoderDrivePhysicalModel`
- `MotorEncoderDriveHardwareConfig`

#### `MazeMap/MazeMap/RuntimeSensorSuite.h`
Bucket: callback/state carriers

- `CaptureCallback`
- `FilteredIrChannel`

#### `MazeMap/MazeMap/SearchRunPlanner.h`
Bucket: planner response carriers

- `SearchStraightPlan`
- `SearchReplanResponse`

#### `MazeMap/MazeMap/TractionLimitSweep.h`
Bucket: sweep result/command carriers

- `TurningTractionMetrics`
- `TurningLaunchCommands`

#### `MazeMap/MazeMap/Vehicle.h`
Bucket: vehicle fact carriers

- `ArcTrackWidthInterpolation`
- `VehiclePhysicalModel`

#### `MazeMap/MazeMap/WallGeometryModel.h`
Bucket: geometry prediction/state carriers

- `GeometryPrediction`
- `GeometryStateFrame`

#### `MazeMap/MazeMap/WallSensorPreprocessor.h`
Bucket: preprocessor input/config carriers

- `WallPreprocessorInput`
- `WallSensorPreprocessorConfig`

#### `MazeMap/MazeMap/WallTouchRoutine.h`
Bucket: callback/state carriers

- `Hooks`
- `SettleRoutineState`

### 1-struct files

#### `MazeMap/MazeMap/AuxMeasurementModeSupport.h`
Bucket: support geometry carrier

- `PositionAuditFixtureGeometry`

#### `MazeMap/MazeMap/BootModeDescriptor.h`
Bucket: canonical descriptor metadata, but still `struct`

- `BootModeDescriptor`

#### `MazeMap/MazeMap/DiagnosticCoverage.h`
Bucket: instruction carrier

- `DiagnosticSummaryInstruction`

#### `MazeMap/MazeMap/ImuCalibrationPolicy.h`
Bucket: tiny helper carrier

- `EncoderCountPair`

#### `MazeMap/MazeMap/InPlaceTurnProfile.h`
Bucket: profile carrier

- `InPlaceTurnProfile`

#### `MazeMap/MazeMap/LoopController.cpp`
Bucket: local callback/service context

- `CaptureContext`

#### `MazeMap/MazeMap/ManeuverPath.cpp`
Bucket: local helper/result match

- `CandidateMatch`

#### `MazeMap/MazeMap/MazeMapRuntimeSignalHelpers.cpp`
Bucket: local helper reference

- `MapQualifiedSideWallReference`

#### `MazeMap/MazeMap/OpenFloorMeasurementCycle.h`
Bucket: large measurement-cycle carrier

- `OpenFloorMeasurementCycle`

#### `MazeMap/MazeMap/OpenFloorMeasurementLabels.h`
Bucket: label/phase carrier

- `OpenFloorMeasurementLabels`

#### `MazeMap/MazeMap/RuntimeSensorSuite.cpp`
Bucket: local callback/service context

- `CaptureContext`

#### `MazeMap/MazeMap/SmoothTurnYawRateController.h`
Bucket: controller state carrier

- `SmoothTurnYawRateControllerState`

#### `MazeMap/MazeMap/TurnWallEdgeTracker.h`
Bucket: maneuver helper carrier

- `TurnWallEdgeTracker`

#### `MazeMap/MazeMap/WallObservationPipeline.h`
Bucket: evidence config carrier

- `WallEvidenceConfig`

#### `MazeMap/MazeMap/WallSensor.h`
Bucket: sensor model carrier

- `DistanceModel`

#### `MazeMap/MazeMap/WallSensorCalibration.h`
Bucket: calibration-point carrier

- `Point`

#### `MazeMap/MazeMap/WheelControlProfile.h`
Bucket: wheel-profile carrier

- `WheelControlProfile`

## Suggested Next Dig Targets

If the goal is to keep pushing the earlier “private helper/state” section outward, I would inspect these in order:

1. `DiagnosticController.cpp`
2. `OpenFloorMeasurementController.cpp`
3. `MissionModeController.cpp`
4. `MazeRunningAuditController.cpp`
5. `ManeuverExecutor.h`
6. `LoopController.h`
7. `DriveBase.h`
8. `MazeMapRuntimeInfrastructure.h`
9. `PlantModel.h`
10. `MazeMapRuntimeCore.h`

Those ten files account for the bulk of the compiled `struct` helper/state/config surface.
