# Struct-Biased Production Inventory

Date: 2026-04-18

This pass is intentionally over-inclusive.

Working rule for this file:
- if it is a production `struct`, it is in-scope by default;
- `.cpp`-local `struct`s and nested helper `struct`s are the primary focus;
- public carrier/profile/result/spec `struct`s are listed afterward because you explicitly called out `struct` itself as a strong smell indicator.

This is not a final verdict file. It is a broad inventory for cleanup triage.

## Dense Files By Production `struct` Count

| File | `struct` count |
| --- | ---: |
| `MazeMap/MazeMap/MazeMapRuntimeCore.h` | 22 |
| `MazeMap/MazeMap/DiagnosticController.cpp` | 14 |
| `MazeMap/MazeMap/LoopController.h` | 14 |
| `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` | 14 |
| `MazeMap/MazeMap/DriveBase.h` | 10 |
| `MazeMap/MazeMap/ManeuverExecutor.h` | 10 |
| `MazeMap/MazeMap/PlantModel.h` | 9 |
| `MazeMap/MazeMap/PlantModel.cpp` | 8 |
| `MazeMap/MazeMap/VehicleState.h` | 7 |
| `MazeMap/MazeMap/MissionModeController.cpp` | 6 |
| `MazeMap/MazeMap/SrUkfCore.h` | 5 |
| `MazeMap/MazeMap/UKF.h` | 5 |

## A. Compiled File-Local `struct`s In `.cpp` Files

This is the bucket you said you care about right now. Nearly all of these are compiled helper/state bags or phase/sequence carriers.

### `MazeMap/MazeMap/AuxMeasurementController.cpp` (2)
- `HoldPhaseState` (115)
- `TurningTractionState` (125)

### `MazeMap/MazeMap/DiagnosticController.cpp` (14)
- `StraightPhaseMetrics` (231)
- `TurnPhaseMetrics` (237)
- `ArcPhaseMetrics` (243)
- `HoldPhaseState` (267)
- `StraightPhaseState` (274)
- `KickoffPhaseState` (290)
- `KickoffSweepSequenceState` (303)
- `ForwardPhaseState` (308)
- `ForwardSweepSequenceState` (327)
- `TurnPhaseState` (332)
- `ArcPhaseState` (344)
- `CharacterizationRecoveryState` (364)
- `CircleSequenceState` (371)
- `SquareSequenceState` (380)

### `MazeMap/MazeMap/FrontWallCharacterizationController.cpp` (2)
- `HoldPhaseState` (146)
- `CaptureCurveState` (160)

### `MazeMap/MazeMap/LoopController.cpp` (1)
- `CaptureContext` (632)

### `MazeMap/MazeMap/ManeuverPath.cpp` (1)
- `CandidateMatch` (11)

### `MazeMap/MazeMap/MazeMapRuntimeSignalHelpers.cpp` (1)
- `MapQualifiedSideWallReference` (11)

### `MazeMap/MazeMap/MazeRunningAuditController.cpp` (4)
- `InterRunServicePauseLoopState` (218)
- `QueuedManeuverLoopState` (222)
- `StartupStationaryHoldLoopState` (230)
- `FrontCalibrationSweepLoopState` (238)

### `MazeMap/MazeMap/MissionModeController.cpp` (6)
- `InterRunServicePauseLoopState` (170)
- `QueuedManeuverLoopState` (174)
- `StartupStationaryHoldLoopState` (182)
- `ObservationCaptureLoopState` (190)
- `FrontCalibrationSweepLoopState` (201)
- `SearchStraightLoopState` (216)

### `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` (14)
- `PendingLog` (267)
- `TimingBlockState` (278)
- `StaticHoldState` (283)
- `RecoveryState` (289)
- `LaunchPulseState` (300)
- `StraightSectionState` (318)
- `SectionSettleState` (332)
- `TurnSectionState` (346)
- `SmoothTurnState` (355)
- `LaunchSequenceState` (374)
- `StraightSequenceState` (382)
- `YawSequenceState` (390)
- `SmoothSequenceState` (398)
- `LoopSequenceState` (406)

### `MazeMap/MazeMap/PlantModel.cpp` (8)
- `MotionMetrics` (38)
- `ContactLoads` (45)
- `PeakFrictionCoefficients` (56)
- `SplitForceRequest` (62)
- `RollingContactEvaluation` (70)
- `RollingStateEvaluation` (82)
- `ModeTransition` (743)
- `VelocityTargetExactSolution` (759)

### `MazeMap/MazeMap/RuntimeSensorSuite.cpp` (1)
- `CaptureContext` (213)

## B. Nested / Header Helper `struct`s

These are not file-local `.cpp` structs, but they are still helper/state/callback carriers embedded in major owners.

### `MazeMap/MazeMap/LoopController.h` (14)
- `SensorWorkPlan` (28)
- `SessionOptions` (39)
- `SessionResult` (45)
- `ControlVector` (60)
- `TimingDiagnostics` (69)
- `MeasuredMotion` (86)
- `ModeState` (92)
- `PauseContext` (108)
- `PauseDisposition` (114)
- `PauseRequest` (143)
- `ModeCallbacks` (154)
- `ObservedTickState` (196)
- `LatchedRequests` (218)
- `MotorPwmSink` (228)

### `MazeMap/MazeMap/ManeuverExecutor.h` (10)
- `TurnWallEdgeTracker` forward declaration (14)
- `Hooks` (35)
- `HoldRoutineState` (278)
- `SettleRoutineState` (286)
- `ReverseStraightRoutineState` (301)
- `StraightRoutineState` (316)
- `TurnRoutineState` (338)
- `ArcRoutineState` (348)
- `SmoothTurnRoutineState` (365)
- `QueueRoutineState` (377)

### `MazeMap/MazeMap/RuntimeSensorSuite.h` (2)
- `CaptureCallback` (29)
- `FilteredIrChannel` (72)

### `MazeMap/MazeMap/WallTouchRoutine.h` (2)
- `Hooks` (24)
- `SettleRoutineState` (71)

### `MazeMap/MazeMap/DriveBase.h` (10)
- `CommandContext` forward declaration (122)
- `CommandTargets` forward declaration (123)
- `ClosedLoopVelocityCommand` forward declaration (124)
- `MeasuredKinematics` (245)
- `EncoderCycleSample` (528)
- `CommandContext` definition (934)
- `CommandTargets` definition (955)
- `ClosedLoopVelocityCommand` definition (977)
- `ResolvedVelocityDriveSignal` (986)
- `WheelLaunchAssistState` (996)

### `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` (4)
- `WallTouchObservation` (383)
- `WallTouchExecutionResult` (449)
- `WallTouchLoopState` (460)
- `WallTouchLoopHooks` (505)

## C. Public Carrier / Parameter / Result / Spec `struct`s

This section is broader and lower-signal than A/B, but it is the right place to look if you want to collapse public carrier surfaces next.

### `MazeMap/MazeMap/AuxMeasurementModeSupport.h`
- `PositionAuditFixtureGeometry` (10)

### `MazeMap/MazeMap/BootModeDescriptor.h`
- `BootModeDescriptor` (33)

### `MazeMap/MazeMap/BootModeRegistry.h`
- `BootModeSelectorCondition` (15)
- `BootModeRegistryEntry` (32)

### `MazeMap/MazeMap/DiagnosticCoverage.h`
- `DiagnosticSummaryInstruction` (7)

### `MazeMap/MazeMap/FrontWallCharacterizationStorage.h`
- `FrontWallCharacterizationStorage` (14)
- `FrontWallCharacterizationMatch` (33)

### `MazeMap/MazeMap/ImuCalibrationPolicy.h`
- `EncoderCountPair` (8)

### `MazeMap/MazeMap/InPlaceTurnProfile.h`
- `InPlaceTurnProfile` (10)

### `MazeMap/MazeMap/MapEvidenceUpdater.h`
- `EdgeEvidence` (13)
- `MapEvidenceUpdaterConfig` (20)

### `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `MotionLimits` (573)
- `PoseEstimate` (584)
- `OpticalObservationTiming` (594)
- `ImuObservationTiming` (603)
- `WallSensorTelemetry` (610)
- `ImuTelemetry` (620)
- `SensorSnapshot` (633)
- `DriveTelemetry` (687)
- `ControlCycleTiming` (723)
- `EncoderProgressWatchdog` (740)
- `RawWallSensorSample` (818)
- `WallSensorCalibrationInput` (829)
- `RobustSignalBand` (839)
- `WallSensorCalibrationCapture` (846)
- `FrontCalibrationSpinSampleSet` (854)
- `AveragedWallSensorInputWindow` (920)
- `AveragedBackLeftImuSample` (1012)
- `FrontSignalModelCache` (2201)
- `AsyncWallSensorPairRead` (3128)
- `AsyncWallSensorSweepRead` (3148)
- `NamedCode` (3712)
- `RollingObservationVoteSummary` (4081)

### `MazeMap/MazeMap/MotorEncoderDrive.h`
- `MotorEncoderDrivePhysicalModel` (8)
- `MotorEncoderDriveHardwareConfig` (23)

### `MazeMap/MazeMap/OpenFloorMeasurementCycle.h`
- `OpenFloorMeasurementCycle` (10)

### `MazeMap/MazeMap/OpenFloorMeasurementLabels.h`
- `OpenFloorMeasurementLabels` (8)

### `MazeMap/MazeMap/OpenFloorMeasurementSpec.h`
- `OpenFloorMarkerPose` (140)
- `OpenFloorSectionDefinition` (149)
- `OpenFloorPrimitiveDefinition` (156)

### `MazeMap/MazeMap/PlantModel.h`
- `ContactKinematics` (17)
- `WheelKinematics` (24)
- `SlipTargets` (32)
- `ContactForce` (40)
- `ContactForces` (49)
- `PlantDerivatives` (92)
- `DriveCommandSolution` (108)
- `PlantParams` (142)
- `PlantPreparedParams` (245)

### `MazeMap/MazeMap/SearchRunPlanner.h`
- `SearchStraightPlan` (11)
- `SearchReplanResponse` (24)

### `MazeMap/MazeMap/SmoothTurnYawRateController.h`
- `SmoothTurnYawRateControllerState` (9)

### `MazeMap/MazeMap/SrUkfCore.h`
- `MeasurementUpdateResult` (16)
- `WallUpdateResult` (24)
- `FrontPairUpdateResult` (31)
- `ModeProcessNoiseTuning` (50)
- `RuntimeTuning` (61)

### `MazeMap/MazeMap/TractionLimitSweep.h`
- `TurningTractionMetrics` (9)
- `TurningLaunchCommands` (18)

### `MazeMap/MazeMap/TurnWallEdgeTracker.h`
- `TurnWallEdgeTracker` (5)

### `MazeMap/MazeMap/UKF.h`
- `NoopUkfLoopHook` (26)
- `UkfStorageOrder` (34)
- `UkfFloatOps` (43)
- `SrUkfWeights` (173)
- `SrUkfMath` (215)

### `MazeMap/MazeMap/Vehicle.h`
- `ArcTrackWidthInterpolation` (13)
- `VehiclePhysicalModel` (21)

### `MazeMap/MazeMap/VehicleState.h`
- `SensorExtrinsics` (27)
- `ImuExtrinsics` (35)
- `ControlInput` (44)
- `EncoderObs` (52)
- `ImuObservation` (60)
- `ImuAccelObs` (71)
- `WallObs` (79)

### `MazeMap/MazeMap/WallBeliefMap.h`
- `WallBeliefConfig` (15)
- `WallBeliefState` (25)
- `WallBeliefUpdate` (34)

### `MazeMap/MazeMap/WallGeometryModel.h`
- `GeometryPrediction` (23)
- `GeometryStateFrame` (40)

### `MazeMap/MazeMap/WallObservationPipeline.h`
- `WallEvidenceConfig` (25)

### `MazeMap/MazeMap/WallSensor.h`
- `DistanceModel` (27)

### `MazeMap/MazeMap/WallSensorCalibration.h`
- `Point` (17)

### `MazeMap/MazeMap/WallSensorPreprocessor.h`
- `WallPreprocessorInput` (11)
- `WallSensorPreprocessorConfig` (23)

### `MazeMap/MazeMap/WheelControlProfile.h`
- `WheelControlProfile` (8)

## D. Other Production `struct` Surfaces Worth A Quick Glance

These are lower-priority for the wrapper/helper question, but they are still production `struct`s.

### `MazeMap/MazeMap/Defines.h`
- `HostDigitalPinState` (244)
- `FlexPwmPinInfo` (1485)
- `EncoderSlot` (1579)

### `MazeMap/MazeMap/LSM6DSV16X_IMU.h`
- `Axes` (142)
- `Axes` (806)

### `MazeMap/MazeMap/MmLog.h`
- `FieldDescriptor` (189)
- `FieldTraits` forward declaration (232)
- `has_row_contract` (278)
- `has_row_contract` specialization (281)
- `MetadataEntry` (674)

## Quick Read

If you want the strongest next cleanup target from this file alone:
1. Start with section A.
2. Then section B.
3. Use section C only after that, because many of those are public carrier structs and some may turn out to be legitimate domain vocabulary rather than pure helper junk.

The biggest repeated pattern in section A is not just “there are many structs,” but “entire phases, sequences, routines, and callback dispatch paths are being carried around as standalone state bags.”
