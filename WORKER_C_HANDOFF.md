# Worker C Handoff

Worker C owned only these files:

- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\LoopController.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\LoopController.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\SharedRobotRuntime.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\SharedRobotRuntime.cpp`

Accepted incoming context:

- `Vehicle` privately owns `_leftMotor` and `_rightMotor`.
- `Vehicle` has private `ApplyMotorCommand(const CommandVector&)`, `ResetDriveEncoders()`, and `CaptureEncoderObservation(float)`.
- `Vehicle` friends include `LoopController` and `RuntimeSensorSuite`.
- `RuntimeSensorSuite::Capture(...)` accepts `captureEncoders` and `encoderDtSeconds`, and publishes `snapshot.encoderObservation` plus `snapshot.encoderObservationValid`.
- `PlantModel` is constructed from `speedVehicle`; `Estimator` is constructed from `plantModel` in `SharedRobotRuntime`.
- `Estimator::ukf()` is removed from public API.
- `DriveBase` remains dirty/out-of-scope for Worker C; Worker E is expected to remove/fix it.

Worker C changes made:

- Removed `LoopController::MotorPwmSink` from `LoopController.h`.
- Removed the `SetMotorPwmThunk(...)` function from `LoopController.cpp`.
- Removed public `SharedRobotRuntime::SetMotorPWM(...)` declaration and definition.
- `LoopController::AttachRuntime(...)` now only stores `_runtime`.
- `LoopController::Run()` no longer requires a motor sink to be present.
- `LoopController::ApplyControlAtTickStart(...)` now directly calls `_runtime->SpeedVehicle().ApplyMotorCommand(control);` through existing `Vehicle` friendship.
- `SharedRobotRuntime::FailActiveMode(...)` no longer calls `drive.Brake()` or `drive.UseNominalWheelControlProfile()`.
- Fault braking now goes through `controlLoop.ApplyControlAtTickStart(CommandVector::Brake());` using existing `SharedRobotRuntime` friendship into `LoopController`.
- `LoopController::RestoreSessionStartPhysicalState()` now calls `_runtime->SpeedVehicle().ResetDriveEncoders();`.
- `LoopController::RestoreSessionStartPhysicalState()` now restores estimator state through `_runtime->Estimator().RestoreSessionStartPhysicalState(...)`.
- `LoopController::RestoreSessionStartPhysicalState()` no longer calls DriveBase restore.
- `LoopController::CaptureTickState()` now requests encoders through `RuntimeSensorSuite::Capture(...)` with `captureEncoders = _options.workPlan.readEncoders` and `encoderDtSeconds = dtSeconds`.
- Encoder estimator update now uses `snapshot.encoderObservation`, guarded by `snapshot.encoderObservationValid`.
- Removed `DriveBase::ConsumeEncoderObservation(...)` usage from `LoopController`.
- Removed `estimator.ukf().params().noHitRangeM` usage from `LoopController`.
- Wall no-hit range now uses the requested temporary residual: `MazeMap::PlantParams::Default().noHitRangeM`.
- Added direct includes needed by edited files:
  - `LoopController.cpp`: `MazeMapRuntimeCore.h`, `PlantModel.h`
  - `SharedRobotRuntime.cpp`: `CommandVector.h`
- `LoopController.cpp` no longer includes `DriveBase.h`.

Remaining known references:

- In Worker C owned files, no remaining references to:
  - `MotorPwmSink`
  - `SetMotorPWM`
  - `SetMotorPwm`
  - `_motorPwmSink`
  - `DriveBase::ConsumeEncoderObservation`
  - `estimator.ukf()`
- Repo-wide `DriveBase.h` still defines `ConsumeEncoderObservation(...)`, but that file is explicitly Worker E territory.

Verification performed:

- Ran `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`.
- First run timed out at 124 seconds.
- Reran with longer timeout; compile failed before tests.
- Build log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260510_132624_153.txt`

Verification failure cause:

- Failure is in out-of-scope `DriveBase.h`, not Worker C owned files.
- `DriveBase.h` still uses private/removed `MotorEncoderDrive` APIs:
  - `CreateDefaultLeftDrive`
  - `CreateDefaultRightDrive`
  - `resetEncoderDistanceMeters`
  - `getDriveCommand`
  - `getVoltage`
  - `brake`
  - `getEncoderCount`
  - `consumeEncoderCount`
  - `setDriveCommand`
- There are also downstream `DiagnosticController.cpp` errors against `DriveBase`, likely related to the same cleanup area.

Important residual note:

- `PlantParams::Default().noHitRangeM` in `LoopController.cpp` is intentionally temporary per task instructions because no canonical no-hit accessor exists in Worker C owned files. This should be cleaned up later with PlantParams/PlantModel ownership, without restoring `Estimator::ukf()` or exposing `SrUkfCore`.

Potential coordination note:

- `SharedRobotRuntime` still owns and exposes `DriveBase& Drive()` because removing that is outside Worker C scope and likely belongs with Worker E's DriveBase cleanup.
- Worker C did not edit `DriveBase.h` despite build failures, per ownership boundary.
