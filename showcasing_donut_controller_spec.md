# ShowcasingDonutController Spec

## Status

- Added as standalone staged source files only:
  - `staging/showcasing_donut/ShowcasingDonutController.h`
  - `staging/showcasing_donut/ShowcasingDonutController.cpp`
- Intentionally kept outside compiled source directories.
- Intentionally not added to the project file yet.
- Intentionally not wired into `BootModeDescriptor.h` or `BootModeRegistry.cpp` yet.

## Intended Mode Identity

- Mode name: `ShowcasingDonutController`
- Planned stable id: `showcasing_donut`
- Planned selector pins: `9/10`
- Planned category: utility mode
- Planned purpose summary:
  - Run a fixed-radius donut sweep after shared startup calibration, ramp speed until traction loss or the 4 m/s cap, then finish with a bounded flashy turn sequence.

## Runtime Shape

- Uses the shared runtime owners directly:
  - `SharedRobotRuntime`
  - `LoopController`
  - `DriveBase`
  - `Drive`
  - `StartupCalibration`
- Uses the runtime-owned fault path.
- Does not explicitly close `logging.txt` or the utility data log.
- Uses a single `LoopController` session.
- Faults immediately on jumper removal.

## Motion Plan

- Run shared startup calibration before motion.
- Hold with fan enabled to settle before the sweep.
- Execute a clockwise fixed-radius turn:
  - radius: `0.090 m`
  - initial commanded speed: `0.30 m/s`
  - speed ramp: `0.05 m/s^2`
  - command path: `DriveBase::PointControlVector(...)`
  - yaw-rate command: `v / r`
- End conditions:
  - detected traction loss from non-UKF signals only
  - reaching the `4.0 m/s` speed cap
- Finish sequence:
  - two `180 deg` in-place `Drive` turns for a full flashy spin
  - short completion hold

## Traction-Loss Detection

- Uses only non-UKF stop logic:
  - encoder velocity
  - gyro yaw rate
  - planar acceleration
- Does not use bounds detection, pose fences, or UKF-health faulting to end the run.
- Detects sustained coherence loss between the commanded circle and measured yaw/planar response.

## Open-Floor Log Compatibility

- The mode writes an open-floor-compatible `main` stream schema:
  - same field layout as the current open-floor main row
  - same enum-id vocabulary from `OpenFloorMeasurementSpec.h`
  - same metadata keys for the open-floor main-format family
- Important limitation:
  - the authoritative `OpenFloorMainRow` type is currently private to `OpenFloorMeasurementController.cpp`
  - because this task forbids editing other files, exact shared ownership of the schema could not be completed here
  - the new mode therefore carries a local same-layout row declaration so it can emit the same on-disk field contract without refactoring the existing open-floor owner

## Flash-Move Radius Bound

- The flashy finish is intentionally an in-place turn sequence.
- That keeps the vehicle at its current orbit position instead of sending it onto a larger translation path.
- With the donut orbit itself fixed at `0.090 m`, the finish stays well inside the requested `0.27 m` maximum radius from the donut center.

## Deferred Integration Work

When edits to other files are allowed, the remaining steps are:

1. Move the staged files into `MazeMap/MazeMap` only when integration is explicitly requested.
2. Add a new `BootModeId` entry in `MazeMap/MazeMap/BootModeDescriptor.h`.
3. Add a real descriptor function for `ShowcasingDonutController`.
4. Add the `BootModeRegistry` selector entry for pins `9/10`.
5. Add the new source files to the project build and then verify Release host tests and the current Teensy build path.

## Build Note

- Per request, no build or test has been run yet.
- The next step is to pause here before any build is started.
