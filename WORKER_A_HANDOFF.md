# Worker A Handoff Note

I was Worker A for the Vehicle/MotorEncoderDrive ownership slice. I edited only:

- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Vehicle.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Vehicle.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MotorEncoderDrive.h`

My completed work:

- Moved left/right `MotorEncoderDrive` concrete ownership into `Vehicle` as private members `_leftMotor` and `_rightMotor`.
- Moved motor/encoder construction facts into private `Vehicle` scalar constants:
  - drive nominal voltage/no-load RPM/supply voltage/resistance/torque constant/no-load current/speed constant/gear ratio
  - wheel diameter, wheel Y offset, encoder pulses per rev
  - left/right motor output pins, encoder input pins, encoder channels, motor inversion, encoder inversion
- Added private `Vehicle` primitives for later workers:
  - `void ApplyMotorCommand(const App::Internal::CommandVector& command) noexcept`
  - `void ResetDriveEncoders() noexcept`
  - `EncoderObs CaptureEncoderObservation(float dtSeconds) noexcept`
- Added narrow friend access on `Vehicle` for:
  - `MazeMap::PlantModel`
  - `MazeMap::App::Internal::LoopController`
  - `::RuntimeSensorSuite`
- Removed public static/shared-fact/factory paths from `MotorEncoderDrive`:
  - `GetSharedPhysicalModel`
  - `GetLeftHardwareConfig`
  - `GetRightHardwareConfig`
  - `CreateDefaultLeftDrive`
  - `CreateDefaultRightDrive`
  - internal `PhysicalModel` and `HardwareConfig` structs
- Narrowed `MotorEncoderDrive` so configured construction and direct endpoint operations are private, with `Vehicle` as friend:
  - direct motor PWM operations like `setDriveCommand`, `brake`, `coast`
  - encoder endpoint operations like `getEncoderCount`, `consumeEncoderCount`, reset/set encoder distance/count
  - model scalar setters/getters and pin/channel setters/getters

Known intentional compile breakpoints left for other workers:

- `PlantModel.cpp:1866` still calls removed `MotorEncoderDrive::GetSharedPhysicalModel()`.
- `MazeMapRuntimeInfrastructure.h:125` still calls removed `GetSharedPhysicalModel()`.
- `TopSpeedMeasurementMode.cpp:315` still calls removed `GetSharedPhysicalModel()`.
- `MazeMapRuntimeCore.h:199-200` still calls removed `GetLeftHardwareConfig()` / `GetRightHardwareConfig()`.
- `DriveBase.h` still directly owns/uses motor endpoints:
  - `CreateDefaultLeftDrive/CreateDefaultRightDrive`
  - encoder reset/read/consume paths
  - `brake`
  - `setDriveCommand`

Verification I ran:

- `git diff --check -- MazeMap\MazeMap\Vehicle.h MazeMap\MazeMap\Vehicle.cpp MazeMap\MazeMap\MotorEncoderDrive.h`
- It passed with only CRLF normalization warnings.
- I did not run release tests because the intentional handoff breakpoints prevent a clean build until Workers B/C/D migrate their call sites.

Coordination expectations:

- Worker B should bind `PlantModel` / `PlantParams::Default()` to `Vehicle` private constants through friendship, without adding a public parameter-bag path.
- Worker C should remove `SharedRobotRuntime::SetMotorPWM` and have `LoopController` use `Vehicle::ApplyMotorCommand(...)` through friend access.
- Worker D should move encoder capture into `RuntimeSensorSuite` using `Vehicle::CaptureEncoderObservation(...)`, and remove remaining DriveBase encoder ownership paths.
- Someone must migrate or delete the startup/IMU calibration encoder hardware reads in `MazeMapRuntimeCore.h`; those currently depend on removed public motor hardware config accessors.
- `DriveBase` still needs its motor members removed after the command and encoder paths are migrated.
