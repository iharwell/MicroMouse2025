# Worker B Handoff Note

Worker B scope was limited to these owned files:

- `MazeMap/MazeMap/PlantModel.h`
- `MazeMap/MazeMap/PlantModel.cpp`
- `MazeMap/MazeMap/Estimator.h`
- `MazeMap/MazeMap/Estimator.cpp`
- `MazeMap/MazeMap/SharedRobotRuntime.h`
- `MazeMap/MazeMap/SharedRobotRuntime.cpp`
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `MazeMap/MazeMap/TopSpeedMeasurementMode.cpp`

Current Worker A context assumed present:

- `Vehicle` owns private `_leftMotor` and `_rightMotor`.
- `Vehicle` has private `ApplyMotorCommand(...)`, `ResetDriveEncoders()`, and `CaptureEncoderObservation(...)`.
- `Vehicle` has private scalar drive/motor constants and friendship for `PlantModel`, `LoopController`, and `RuntimeSensorSuite`.
- Public `MotorEncoderDrive` statics/factories were removed.

Worker B changes:

- `PlantModel` now has constructors:
  - default constructor
  - `explicit PlantModel(const Vehicle& vehicle)`
- `PlantModel` owns a private `_preparedParams`.
- `PlantModel::BuildParamsFromVehicle(const Vehicle&)` builds plant facts from `Vehicle` facts/private constants, not from `MotorEncoderDrive` statics.
- `PlantParams::Default()` remains, but now routes through `Vehicle`/`PlantModel` as a legacy math/test migration artifact.
- `SharedRobotRuntime` now constructs `plantModel(speedVehicle)` before `estimator`.
- `SharedRobotRuntime` now constructs `estimator(plantModel, &runtimeState)`.
- `SharedRobotRuntime` no longer passes `plantModel._preparedParams.raw` into `DriveBase`.
- `Estimator` gained `Estimator(const PlantModel&, VehicleState*)`, which binds `SrUkfCore` from the runtime `PlantModel` facts.
- `Estimator::ukf()` was already removed in the working tree and remains removed.
- `Estimator::WriteUkfDebugTextDump` was removed.
- `TopSpeedMeasurementMode` no longer calls removed motor statics for supply voltage; it uses `_drive.CurrentBatteryVoltageV()`.
- `MazeMapRuntimeInfrastructure.h` no longer uses `MotorEncoderDrive::GetSharedPhysicalModel()` for diagnostic tuning events; it emits motor-model metadata from `PlantParams::Default()`.
- `MazeMapRuntimeCore.h` no longer includes `MotorEncoderDrive.h` or owns encoder-channel constants. The old zero-arg `CaptureDriveEncoderCounts()` and `HaveDriveEncodersMovedSince()` helpers were removed. The IMU calibration helpers now require a caller-supplied encoder-count capture callback.

Important correction already applied:

- I initially introduced local encoder-channel constants in `MazeMapRuntimeCore.h`; that was rejected because Vehicle owns hardware facts. Those constants were removed.

Known remaining handoffs:

- Worker D must update `RuntimeSensorSuite.cpp` call sites that still reference removed helpers:
  - `WaitForImuCalibrationSettle(startCounts, ...)`
  - `AverageBackLeftImuSelfTestSample(...)`
  - `CaptureDriveEncoderCounts()`
  - `HaveDriveEncodersMovedSince(...)`
  RuntimeSensorSuite is already a `Vehicle` friend, so it should use/borrow the Vehicle-owned encoder capture path without adding public Vehicle accessors.
- Worker E must clean `DriveBase`. Release build currently fails because `DriveBase.h` still uses removed/noncanonical motor ownership and private motor methods, including:
  - `MotorEncoderDrive::CreateDefaultLeftDrive()`
  - `MotorEncoderDrive::CreateDefaultRightDrive()`
  - private `getEncoderCount()`
  - private `consumeEncoderCount()`
  - private `setDriveCommand(...)`
- Worker E also needs to remove/fix `DriveBase`'s remaining `PlantParams` constructor/default path. I intentionally stopped passing runtime plant raw params from `SharedRobotRuntime` to avoid widening a private PlantModel path.

Verification performed:

- `rg` over Worker B owned files found no `MotorEncoderDrive::` removed static/factory usage, no duplicated encoder-channel constants, and no direct `ReadEncoderCount(...)`.
- `git diff --check` passed for Worker B owned edits.
- Ran `codex_verify/build_and_verify_latest.cmd --no-pause`; Release build failed before tests due to non-owned `DriveBase.h` errors listed above, so unit tests did not run.
