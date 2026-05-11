# Worker F / Galileo Handoff

Scope: caller migration outside Worker E files.

Status: completed.

## Changes Reported By Worker

- `StartupCalibration`, `ManeuverExecutor`, `WallTouch`, `DiagnosticController`, and `AuxMeasurementController` were migrated to use `SharedRobotRuntime::Estimator()` for pose/start/bias mutation.
- `StartupCalibration` and `ManeuverExecutor` no longer kept `DriveBase` members solely for estimator state.
- `OpenFloorMeasurementController` and `ShowcasingDonutController` no longer read removed `DriveTelemetry` UKF fields or `Estimator::ukf()`.
- `EstimatorTestSupport`, `SharedRuntimeTest`, and `DriveManeuverTests` were migrated to pass encoder observations through `SensorSnapshot::encoderObservation`.
- `DiagnosticCoverageTest` no longer uses removed `MotorEncoderDrive` factories or private constructors.

## Verification Reported

- Scoped `rg` checks passed for touched files.
- Broad scan only found remaining prohibited caller patterns in `DriveBaseTest.cpp`, which was Worker E scope at the time.
- `git diff --check` passed on touched files, with CRLF warnings only.
- No full build was run.

## Later Rejection / Follow-Up

- User challenged the migration of copied DriveBase state-manipulation methods onto `Estimator`.
- `Estimator` convenience APIs `SetPose`, `SetStartPoint`, `SetPoseXMeters`, and `SetPoseYMeters` were rejected.
- Worker G partially corrected this before crashing. Recheck current tree.

