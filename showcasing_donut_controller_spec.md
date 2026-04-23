# ShowcasingDonutController Spec

## Status

- Integrated into the compiled boot-mode graph:
  - `MazeMap/MazeMap/ShowcasingDonutController.h`
  - `MazeMap/MazeMap/ShowcasingDonutController.cpp`
- Wired into:
  - `MazeMap/MazeMap/BootModeDescriptor.h`
  - `MazeMap/MazeMap/MazeMapControllerRegistry.h`
  - `MazeMap/MazeMap/BootModeRegistry.cpp`
  - `MazeMap/MazeMap/MazeMap.vcxproj`
  - `MazeMap/MazeMap/MazeMap.vcxproj.filters`
  - `MazeMap/MazeMapTest/ApplicationTest.cpp`

## Intended Mode Identity

- Mode name: `ShowcasingDonutController`
- Planned stable id: `showcasing_donut`
- Planned selector pins: `9/10`
- Planned category: utility mode
- Planned purpose summary:
  - Run a fixed-radius donut sweep, ramp speed until traction loss or the 4 m/s cap, then finish with a bounded flashy turn sequence.

## Runtime Shape

- Uses the shared runtime owners directly:
  - `SharedRobotRuntime`
  - `LoopController`
  - `DriveBase`
  - `Drive`
- Uses the runtime-owned fault path.
- Does not explicitly close `logging.txt` or the utility data log.
- Uses a single `LoopController` session.
- Faults immediately on jumper removal.
- Uses two execution phases inside that one session:
  - one continuous donut-sweep phase
  - one flashy-turn phase

## Motion Plan

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

Integration is complete. Ongoing validation should cover:

1. Release host tests after the next clean current-artifact build.
2. The current Teensy build path after the next clean current-artifact build.
3. On-hardware confirmation that the `9/10` selector monitor faults immediately when the jumper is removed.

## Build Note

- Direct Release rebuild of `MazeMap.sln` succeeded on `2026-04-21`.
- Targeted Release boot-registry tests passed for the integrated mode:
  - `BootModeRegistry_ExposesCurrentInventory`
  - `BootModeRegistry_DescriptorsAreAuthoritative`
  - `ResolveActiveApplicationMode_UsesDescriptorEntryMode`
  - `BootModeRegistry_DefaultsToMission`
  - `BootModeRegistry_PrefersFrontWallCharacterization`
  - `BootModeRegistry_PrefersLedCalibrationOverLaterModes`
  - `BootModeRegistry_PrefersAuxiliarySelectorOverMissionModes`
  - `BootModeRegistry_PrefersManeuverFileTestOverLaterModes`
  - `BootModeRegistry_PrefersTopSpeedMeasurementOverLaterModes`
  - `BootModeRegistry_PrefersOpenFloorMeasurementOverShowcasingDonut`
  - `BootModeRegistry_SelectsShowcasingDonutWhenPins9And10AreStrapped`
- Direct Teensy compile succeeded and produced a fresh `MazeMap.ino.hex`.
- The repository wrapper `build_and_verify_latest.cmd --no-pause` still stops at its artifact-freshness gate even after successful builds.
- The full Release `MazeMapTest.dll` suite currently has unrelated existing failures in `DriveManeuver`, `PlantModelDriveCommand`, and `SrUkfCoreMotionUpdate`; those failures are outside this mode integration.
