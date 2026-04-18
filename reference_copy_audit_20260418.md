# Local Reference Copy Audit

Date: 2026-04-18

## Scope

- Goal: find classes and functions that copy data locally even though the authoritative data is already reachable through a canonical owner or local reference.
- Strict rule used here: count retained by-value state that duplicates data from a nearer authoritative owner.
- Excluded from the strict count: `Drive`, `Maze`, `MapEvidenceUpdater`, and `WallBeliefMap`.
- Broad candidate pool, once function-local snapshots are included: `329` heuristic by-value copy candidates across `42` files.
- Validation method: static audit plus local `sizeof` / class-layout probes compiled against current headers with the project's `C++17` settings.
- Verification status: static only. No unit tests or runtime verification were run for this audit.

## High-Confidence Retained-Copy Offenders

| Offender | Why it qualifies | Direct copied bytes | Extra cache / snapshot bytes | Total retained duplicate bytes | Evidence |
| --- | --- | ---: | ---: | ---: | --- |
| `PlantParams` | Full parameter bag materialized from `Vehicle` / `MotorEncoderDrive` facts instead of reading those owners directly. | 368 | 0 | 368 | [Vehicle.h:34](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Vehicle.h:34>), [PlantModel.h:142](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PlantModel.h:142>), [PlantModel.cpp:1664](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PlantModel.cpp:1664>) |
| `PlantPreparedParams` | Keeps a second retained representation of `PlantParams`; `raw` is an explicit full copy, and the rest is derived duplicate state. | 368 | 272 | 640 | [PlantModel.h:245](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PlantModel.h:245>), [PlantModel.cpp:1724](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PlantModel.cpp:1724>) |
| `SrUkfCore` | Retains both `_params` and `_preparedParams` even though those facts already have canonical owners; also keeps pre-predict state snapshots. | 1008 | 360 | 1368 | [SrUkfCore.h:137](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h:137>), [SrUkfCore.h:461](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h:461>), [SrUkfCore.cpp:337](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.cpp:337>) |
| `Vehicle` default progress cache | Process-wide static copies of default plant params and prepared params. | 1008 | 0 | 1008 | [Vehicle.cpp:93](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Vehicle.cpp:93>) |
| `MotorEncoderDrive` | Copies shared physical and hardware config into each instance instead of reading the shared model/config directly. | 40 | 0 | 40 per instance | [MotorEncoderDrive.h:37](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MotorEncoderDrive.h:37>), [MotorEncoderDrive.h:78](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MotorEncoderDrive.h:78>), [MotorEncoderDrive.h:231](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MotorEncoderDrive.h:231>) |
| `WallSensorPreprocessor` | Stores a by-value `_config` even though the constructor receives a `const Config&`. | 432 | 0 | 432 | [WallSensorPreprocessor.h:27](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensorPreprocessor.h:27>), [WallSensorPreprocessor.h:53](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensorPreprocessor.h:53>), [WallSensorPreprocessor.cpp:14](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensorPreprocessor.cpp:14>) |
| `TopSpeedMeasurementMode` selector cache | Copies boot-selector pins out of `BootModeRegistry` and retains them locally. | 2 | 0 | 2 | [TopSpeedMeasurementMode.h:189](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TopSpeedMeasurementMode.h:189>), [TopSpeedMeasurementMode.cpp:442](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TopSpeedMeasurementMode.cpp:442>) |
| `OpenFloorMeasurementController` selector cache | Same pattern as above for the primary-diagnostic selector pins. | 2 | 0 | 2 | [OpenFloorMeasurementController.cpp:1566](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1566>) |

Notes:

- `PlantParams` is the clearest architectural mismatch. `Vehicle` explicitly says these facts should not be copied out, then `PlantParams::Default()` immediately recopies them anyway.
- `PlantPreparedParams` and `SrUkfCore` stack additional retained duplicate state on top of that first copy.
- `MotorEncoderDrive` consumes `40` bytes of copied shared config per instance. `DriveBase` embeds two instances, so that path alone burns `80` bytes of duplicated config per `DriveBase`.

## Secondary Cache / Projection Buckets

These are real local retained copies, but they are better classified as projection/cache state than as pure "should-have-used-a-reference" violations.

| Offender | Retained local copy shape | Added-up bytes | Evidence |
| --- | --- | ---: | --- |
| `LoopController` | `_observedScratch` `484` + `_callbackModeState` `488` + `_pauseContextScratch` `492` | 1464 | [LoopController.h:93](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.h:93>), [LoopController.h:109](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.h:109>), [LoopController.h:198](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.h:198>) |
| `RuntimeSensorSuite` | Four averaging windows `4 x 240` + four filtered IR channels `4 x 8`; total object size `1064` | 992 core cache bytes, 1064 total object bytes | [RuntimeSensorSuite.h:117](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/RuntimeSensorSuite.h:117>), [RuntimeSensorSuite.h:134](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/RuntimeSensorSuite.h:134>) |
| `DriveBase` | `_poseCache` projection of estimator state | 28 | [DriveBase.h:385](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:385>), [DriveBase.h:568](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:568>) |

## Immutable Duplicate Access Paths

These are not big byte offenders, but they do create alternate authority paths to facts already owned elsewhere.

- `CoreConfig.h` republishes maze and vehicle facts through config constants instead of forcing callers to use `Maze` / `Vehicle` directly. See [CoreConfig.h:16](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CoreConfig.h:16>), [CoreConfig.h:24](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CoreConfig.h:24>), and [CoreConfig.h:127](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CoreConfig.h:127>).
- `MazeMapRuntimeCore.h` repeats the same pattern. See [MazeMapRuntimeCore.h:73](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h:73>), [MazeMapRuntimeCore.h:81](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h:81>), and [MazeMapRuntimeCore.h:184](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h:184>).

## Function-Local Copy Inventory

The file-local count explodes once temporary by-value snapshots are included. This bucket is intentionally broad and over-inclusive.

Top candidate-density files from the heuristic sweep:

| File | Candidate count |
| --- | ---: |
| [MazeRunningAuditController.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeRunningAuditController.cpp:1>) | 52 |
| [SrUkfCore.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.cpp:1>) | 32 |
| [PlantModel.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PlantModel.cpp:1>) | 31 |
| [MissionModeController.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MissionModeController.cpp:1>) | 24 |
| [UKF.h](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h:1>) | 14 |
| [MazeMapRuntimeCore.h](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h:1>) | 14 |
| [RuntimeSensorSuite.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/RuntimeSensorSuite.cpp:1>) | 12 |
| [DriveBase.h](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:1>) | 9 |
| [TopSpeedMeasurementMode.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TopSpeedMeasurementMode.cpp:1>) | 7 |
| [OpenFloorMeasurementController.cpp](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1>) | 7 |

Verified repeated local-copy families:

| Family | Count | Size per copy | Added-up bytes across sites | Representative evidence |
| --- | ---: | ---: | ---: | --- |
| `_filter.covariance()` copied into `StateMatrix` | 13 | 324 | 4212 | [SrUkfCore.cpp:1390](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.cpp:1390>) |
| `_drive.GetTelemetry()` copied into `DriveTelemetry` | 12 | 112 | 1344 | [MazeRunningAuditController.cpp:839](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeRunningAuditController.cpp:839>) |
| `_filter.state()` copied into `StateVector` | 11 | 36 | 396 | [SrUkfCore.cpp:1389](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.cpp:1389>) |
| `_drive.GetPose()` copied into `PoseEstimate` | 7 | 28 | 196 | [OpenFloorMeasurementController.cpp:1788](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1788>) |
| `_loopController.Run()` copied into `SessionResult` | 8 | 8 | 64 | [TopSpeedMeasurementMode.cpp:215](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TopSpeedMeasurementMode.cpp:215>) |

Notes:

- These totals are per-site copied volume, not simultaneous live memory.
- Many of these local copies may be intentional snapshots for before/after comparison. They still contribute materially to the "copy instead of direct owner access" pattern count.

## Verified Ad Hoc `PlantParams::Default()` Materialization Sites

These are especially weak sites because they instantiate a `368`-byte parameter bag just to read one scalar or to feed a default argument path.

- [VehicleState.cpp:42](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/VehicleState.cpp:42>) creates a full `PlantParams` copy to read `wheelRadiusM`.
- [WallGeometryModel.cpp:225](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallGeometryModel.cpp:225>), [WallGeometryModel.cpp:233](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallGeometryModel.cpp:233>), and [WallGeometryModel.cpp:252](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallGeometryModel.cpp:252>) materialize a full `PlantParams` temporary for `noHitRangeM`.
- [OpenFloorMeasurementController.cpp:741](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/OpenFloorMeasurementController.cpp:741>) does the same for `maxRangeM`.
- [DriveBase.h:95](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:95>), [MouseUkfFacade.h:14](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MouseUkfFacade.h:14>), and [SrUkfCore.h:138](</C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h:138>) use `PlantParams::Default()` in default-construction paths.

## Exclusions

- `Drive` is excluded. It is reference-based and does not match the audit rule.
- `Maze` is excluded from the strict rule. It is a denormalized cell-centric primary representation, not a class copying from a nearer hidden owner.
- `MapEvidenceUpdater` and `WallBeliefMap` are also better treated as mirrored / denormalized edge models, not strict local-reference-copy violations.

