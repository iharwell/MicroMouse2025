# Runtime State Refactor Worklog

Date: 2026-04-23

## Current objective

Move runtime estimation logic out of `DriveBase` and into the newly renamed `Estimator` home, with `SharedRobotRuntime` as the top-level owner of live runtime state.

## Decisions locked in

- `MouseUkfFacade` has been renamed to `Estimator`.
- The renamed home is acceptable for estimator ownership; the old `facade` naming is not.
- `SharedRobotRuntime` remains the intended top-level owner of live `VehicleState`.
- `DriveBase` must stop owning runtime state and UKF update orchestration.

## Completed this pass

- Renamed:
  - `MazeMap/MazeMap/MouseUkfFacade.h` -> `MazeMap/MazeMap/Estimator.h`
  - `MazeMap/MazeMap/MouseUkfFacade.cpp` -> `MazeMap/MazeMap/Estimator.cpp`
- Updated core project references:
  - `MazeMap/MazeMap/DriveBase.h`
  - `MazeMap/MazeMap/MazeMap.vcxproj`
  - `MazeMap/MazeMap/MazeMap.vcxproj.filters`
- Fixed stale rename fallout required for host builds:
  - `MazeMap/MazeSimulation/MazeSimulation.cpp`
  - `Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp`
  - `tooling/run_open_floor_ukf_replay.ps1`
- Moved the runtime estimation owner to the renamed `Estimator`:
  - `Estimator` now owns estimator fault state, pose cache, reset/coordinate update logic, projected measured-kinematics logic, and the UKF runtime update path.
  - `Estimator` now synchronizes the authoritative live `VehicleState`.
- Moved production ownership into `SharedRobotRuntime`:
  - added runtime-owned `VehicleState runtimeState`
  - added runtime-owned `Estimator estimator`
  - `DriveBase` now borrows the runtime-owned estimator instead of owning its own estimator instance
- Rewired the main runtime call path:
  - `LoopController` now reads estimator pose/health from `SharedRobotRuntime::Estimator()`
  - `LoopController` now drives the estimation update through the runtime-owned `Estimator`
- Updated direct estimator mutation to use the new owner:
  - `StartupCalibration.cpp` now sets gyro bias through the estimator path instead of mutating the state vector through a `const_cast`
- Deleted dead duplicate vehicle-side runtime state:
  - removed `Vehicle::_state`
  - removed `Vehicle::GetVehicleState()` accessors
- Updated `DriveBaseTest.cpp` constructor ownership with a local estimator harness so tests construct `DriveBase` against an explicit estimator owner

## Validation status

- Canonical build+verify wrapper runs:
  - `codex_verify/build_and_verify_latest.cmd --no-pause`
  - latest log: `codex_verify/logs/build_and_verify_latest_20260423_144920_327.txt`
- Current result:
  - Teensy compile passed
  - Release host build passed for `MazeMapTest` and `MazeSimulation`
  - latest-binary freshness checks passed for Teensy and host artifacts
  - Release unit tests executed
- Release unit-test status:
  - current run: `630` total, `551` passed, `79` failed
  - prior baseline in `codex_verify/logs/verify_latest_build_20260422_162514_162.txt`: `631` total, `552` passed, `79` failed
  - the long-standing failing families observed in the current run were already present in the prior verify logs (for example the `DriveBaseOscillationPairing...`, `DriveManeuver_S180SS...`, and `WallSensorLedCalibrationMode_UsesPauseInsteadOfAdvancingLoopTicks` failures)

## Remaining cleanup after this slice

- `DriveBase` still exposes legacy read/update surface that now forwards into the estimator owner:
  - `GetPose()`
  - `GetEstimatorStateVector()`
  - `HasEstimatorFault()`
  - `GetEstimatorFaultReason()`
  - `UpdateOdometry(...)`
- Those alternate access paths should be removed in a follow-on caller-migration slice so `DriveBase` stops looking like a second estimator doorway.
- `PoseEstimate` and `LoopController::ModeState` cleanup is still outstanding beyond this ownership move.

## Constraints to preserve

- No new wrapper layer or compatibility shim.
- No replacement aggregate cache for `PoseEstimate`.
- Keep a worklog update before any context compaction.
