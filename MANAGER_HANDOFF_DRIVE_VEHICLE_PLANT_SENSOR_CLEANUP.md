# Manager Handoff: Drive / Vehicle / Plant / Sensor Ownership Cleanup

Current branch/worktree contains a large dirty architectural cleanup patch. The next manager should treat it as untrusted until reviewed and built. Existing worker A-D handoffs are already present:

- `WORKER_A_HANDOFF.md`
- `WorkerB_handoff_note.md`
- `WORKER_C_HANDOFF.md`
- `WorkerD_handoff.md`

Additional handoffs created from completed/crashed worker notifications:

- `REVIEWER_PLATO_HANDOFF.md`
- `WORKER_E_TESLA_HANDOFF.md`
- `WORKER_F_GALILEO_HANDOFF.md`
- `WORKER_G_BOYLE_CRASH_HANDOFF.md`
- `REVIEWER_LOVELACE_HANDOFF.md`
- `WORKER_H_NEWTON_HANDOFF.md`

## User Corrections That Must Govern The Next Pass

1. Do not move generic `VehicleState` manipulation onto `Estimator`.
   - `Estimator` may own semantic filter reset/update operations.
   - Rejected copied convenience APIs: `SetPose`, `SetStartPoint`, `SetPoseXMeters`, `SetPoseYMeters`.
2. Do not let `DriveBase` know `PlantParams`, `PlantPreparedParams`, `PlantModel::_preparedParams`, or `PlantModel::Prepare`.
   - The earlier `friend class DriveBase` approach was explicitly rejected.
   - `DriveBase` must not use private PlantModel params by friendship.
3. `PlantModel` should be bound to the shared authoritative state/view it needs.
   - `DriveBase` should pass only motion requests to PlantModel, such as desired acceleration/yaw acceleration or desired velocity/yaw rate, plus explicit runtime scalars if truly required.
   - `DriveBase` should not pass `VehicleState` or current-state vectors into PlantModel command solves.
4. The next manager should use workers heavily. A worker crash is acceptable; coordinator context exhaustion is not.

## Current Known State

Implemented or partially implemented in dirty tree:

- `Vehicle` owns left/right `MotorEncoderDrive` and private motor/encoder primitives.
- `MotorEncoderDrive` static shared fact/factory paths were removed.
- `SharedRobotRuntime` constructs `PlantModel(speedVehicle)` beside `Vehicle` and `Estimator`.
- `RuntimeSensorSuite::Capture(...)` can publish `EncoderObs` into `SensorSnapshot`.
- `LoopController` applies selected `CommandVector` through private `Vehicle` privilege and no longer uses a public runtime PWM setter.
- `DriveBase` no longer owns `MotorEncoderDrive` according to later worker reports.
- `DriveBase` no longer has direct `PlantParams`/`PlantPreparedParams` text according to Worker H verification.
- Worker G partially added `PlantModel::AttachRuntimeState(...)` and bound-state no-param PlantModel methods before crashing.
- Worker H removed stale `VehicleState::StateVector presentState` construction from `DriveBase::ResolveRawAccelerationCommand`.

## High-Risk Areas To Review First

- `DriveBase.cpp` / `DriveBase.h`
  - Confirm it has no hardware ownership, PWM application, encoder read/reset/consume, `PlantParams`, `PlantPreparedParams`, `_preparedParams`, or `Prepare`.
  - Confirm PlantModel calls pass only request/scalar inputs, not state vectors or params.
  - Confirm any remaining `VehicleState::StateVector` work is only DriveBase-local feedback context and not PlantModel command-solving leakage.
- `PlantModel.h` / `PlantModel.cpp`
  - Confirm no public param getters or DriveBase friendship.
  - Confirm bound runtime-state API is not a wrapper/shim and is actually used by `SharedRobotRuntime`.
  - Confirm new no-param solve methods are real PlantModel domain operations, not exposed parameter conveniences.
- `Estimator.h` / `Estimator.cpp`
  - Confirm rejected convenience APIs are gone: `SetPose`, `SetStartPoint`, `SetPoseXMeters`, `SetPoseYMeters`.
  - Confirm `Estimator::ukf()` remains removed.
- `EstimatorTestSupport.h`
  - Worker E reported a remaining `DriveBase::RecordMeasurementInputs` blocker; Worker G may have fixed it before crashing. Recheck.
- Full callers
  - Recheck `OpenFloorMeasurementController`, `ShowcasingDonutController`, `DiagnosticController`, `StartupCalibration`, `ManeuverExecutor`, `WallTouch`, tests for convenience API fallout.

## Mandatory Searches Before Build

Run these from repo root:

```powershell
rg -n "ConsumeEncoderObservation|RecordMeasurementInputs|consumeEncoderCount|getEncoderCount|resetEncoderDistanceMeters|CreateDefaultLeftDrive|CreateDefaultRightDrive|GetSharedPhysicalModel|GetLeftHardwareConfig|GetRightHardwareConfig|SetMotorPwm|SetMotorPWM|SetDriveCommand|ApplyCommand|MotorSink|Estimator\(\)\.ukf|\.ukf\(" MazeMap/MazeMap MazeMap/MazeMapTest
rg -n "PlantParams|PlantPreparedParams|_preparedParams|Prepare\(" MazeMap/MazeMap/DriveBase.h MazeMap/MazeMap/DriveBase.cpp
rg -n "SetStartPoint|SetPoseXMeters|SetPoseYMeters|void SetPose|\.SetPose\(" MazeMap/MazeMap/Estimator.h MazeMap/MazeMap/Estimator.cpp MazeMap/MazeMap MazeMap/MazeMapTest
```

Acceptable hits:

- `MotorEncoderDrive` private implementation and `Vehicle` private encoder capture may contain encoder endpoint calls.
- Existing PlantModel/SrUkfCore tests may still use `PlantParams`, but `DriveBase` must not.
- `ResetPose` calls are okay; copied `SetPose` convenience calls are not.

## Verification Status

No successful full release verification has been run after the latest Worker G/H changes.

Earlier build attempts failed before tests due to DriveBase migration blockers that have since been partially addressed. The next manager should run release-mode verification only after the mandatory searches are clean.

