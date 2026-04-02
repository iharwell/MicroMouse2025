# Coordinate Refactor Worklog

Last updated: 2026-04-02

Target convention:
- world/maze axes: `+X` right, `+Y` up/forward in maze coordinates
- body axes: `+X` right, `+Y` forward
- positive yaw / positive turn angle: right turn
- `MazeMap::Direction`: absolute maze direction
- `MazeMap::RelativeDirection`: direction relative to current heading

Files already inspected:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Direction.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Direction.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeLocation.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\PathFinder.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\ManeuverPathFinder.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapRuntimeCore.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\TurnCommandGeometry.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\TractionLimitSweep.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Vehicle.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\VehicleState.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MouseUkf.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapMissionController.cpp`

Files edited so far in this coordinate refactor pass:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\VehicleState.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Vehicle.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapRuntimeSensors.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MouseUkf.h`
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\UKFTest.cpp`

Completed in this recovery pass:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MouseUkf.h`
  - Rebased `HeadingUnitFromYaw()` and body-to-world rotation on `+X` right, `+Y` forward, positive yaw = clockwise.
  - Replaced plant contact geometry and tire force bookkeeping so the body frame is no longer `+X` forward / `+Y` left anywhere in the plant.
  - Flipped wheel-speed / yaw-rate sign relationships so positive yaw now means left wheel faster than right wheel.
  - Removed the plant-to-"legacy measurement" acceleration remap; the UKF plant now predicts IMU planar acceleration directly in body `(+X right, +Y forward)` axes.
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\VehicleState.h`
  - Restored IMU extrinsics naming to body-axis semantics (`accelBodyFromImu`) instead of the temporary measurement-axis alias.
  - Reworded IMU observation comments and state comments so the file now documents only `+X` right, `+Y` forward, positive yaw = clockwise semantics.
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Vehicle.cpp`
  - Converted IMU and wall-sensor extrinsic positions/directions from the old `(forward, left)` basis into the project body frame `(right, forward)`.
  - Kept the IMU accel-axis map identity, but only after restoring it to explicit body-axis semantics (`accelBodyFromImu`).
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapRuntimeSensors.h`
  - Swapped runtime IMU ingestion over to `accelBodyFromImu`, keeping the captured accel snapshots explicitly in body `(+X right, +Y forward)` axes.
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapRuntimeCore.h`
  - Rebased runtime yaw on `0 = Up/+Y` with positive yaw = clockwise/right turn.
  - Replaced shared heading-error and body-to-world rotation helpers so runtime pose math now uses body `(+X right, +Y forward)` instead of `(forward, left)`.
  - Removed the remaining hand-coded `cos/sin` sensor rotation in favor of the shared project-frame helpers.
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapRuntimeDrive.h`
  - Flipped fallback encoder yaw-rate estimation, wheel target decomposition, and wheel feedforward yaw-force split so positive yaw now means a right turn / left wheel faster.
  - Swapped midpoint odometry projection over to `HeadingUnitFromYawRad()` so drive pose integration now follows the shared `0 = Up, positive clockwise` convention.
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\Maneuver.h`
  - Corrected `CodeDegrees()` so unmirrored right-turn maneuver codes now produce positive angles under the project yaw convention instead of negating them for an old CCW runtime.

Next file queued:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapMissionController.cpp`
  - Inspected after the shared-helper and `CodeDegrees()` fixes.
  - No direct patch applied in this pass; its `Direction`/`RelativeDirection` usage and turn-target generation now resolve through the corrected shared conventions.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapStandaloneModes.cpp`
  - Replace the remaining explicit clockwise/counterclockwise angle signs and any direct wheel/yaw sign assumptions that still encode positive-left-turn behavior.

Completed after mission-controller inspection:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MazeMapStandaloneModes.cpp`
  - Flipped the remaining hard-coded clockwise/counterclockwise turn angles, circle angles, and traction-sweep direction signs so standalone diagnostics now agree with positive yaw = clockwise/right turn.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MissionStartPolicy.h`
  - Remove the remaining north-relative yaw conversion that still assumes zero-east / positive-CCW runtime angles.

Completed after standalone-mode sweep:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\MissionStartPolicy.h`
  - Rebased the start-scene east-of-north classifier onto the project yaw directly, removing the leftover `HALF_PI - yaw` conversion from the old zero-east/CCW runtime.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeSimulation\MazeSimulation.cpp`
  - Check the simulation entry points and benchmark helpers for any explicit old yaw sign assumptions or direction misuse before test repair.

Completed after startup-policy fix:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeSimulation\MazeSimulation.cpp`
  - Inspected after the shared refactor; no local coordinate patch was needed because the simulation flow already resolves motion through `Direction` and `RelativeDirection` rather than a separate yaw basis.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\UKFTest.cpp`
  - Update plant/IMU assertions for the new body-axis and yaw semantics, and repair the renamed contact-force / IMU extrinsics members.

Completed after simulation inspection:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\UKFTest.cpp`
  - Updated contact-force field references, IMU lever-arm expectations, and planar-accel prediction assertions to match the project body axes directly instead of the temporary remapped measurement axes.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\MazeMapRuntimeHelperTest.cpp`
  - Check helper-level yaw/direction tests for old zero-east / positive-CCW assumptions after the runtime-core refactor.

Completed after UKF test repair:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\MazeMapRuntimeHelperTest.cpp`
  - Inspected after the runtime-core changes; no local patch was needed because the helper tests cover logging/signal helpers rather than the yaw/body-frame convention.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\DiagnosticCoverageTest.cpp`
  - Repair direct signed-turn and runtime helper expectations that still assume clockwise/right turns are negative.

Completed after helper-test inspection:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\DiagnosticCoverageTest.cpp`
  - Flipped the `CodeDegrees()` coverage expectations so unmirrored right-turn maneuver codes are now treated as positive, matching the project yaw convention.

Next active file:
- Repository-wide compile/sign sweep
  - Search for any remaining references to removed helpers/renamed fields or old right-turn-negative expectations before rebuilding in release mode.

Current assessment:
- `Direction` / `RelativeDirection` core types already distinguish absolute vs relative correctly.
- The runtime yaw helpers and plant/body transforms are still centered on the old convention.
- Wall sensor extrinsics in `Vehicle.cpp` are still stored in the old forward-left body frame.
- The coordinate correction needs to be applied coherently across:
  - UKF plant and geometry transforms
  - runtime yaw helpers and pose transforms
  - mission/turn helpers using signed angle error
  - tests that assume the old yaw sign or old body-axis basis

Release verification after the latest rebuild:
- Ran release `vstest` against `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\x64\Release\MazeMapTest.dll`.
- Result: 372 passed, 3 failed.
- Remaining failures at this checkpoint:
  - `DiagnosticCoverageTest::ClassifyFrontCalibrationSpinHeadingFromNorthUsesOpenAndEastKnownWallBuckets`
  - `MazeMapTest::WallGeometryModelRespectsSensorExtrinsicsForFrontWallPrediction`
  - `MazeMapTest::WallGeometryModelCanIdentifyPostHits`

Completed after release-test triage:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\DiagnosticCoverageTest.cpp`
  - Updated the front-calibration spin-heading coverage to use project yaw directly (`0 = North/+Y`, positive clockwise/east-of-north) instead of the old zero-east / positive-CCW angles.
  - Rebased the open / ignore / wall sample headings from `(90, 63, 59, 0)` degrees to `(0, 27, 31, 90)` degrees for the same bucket boundaries.

Next active file:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\MazeMapTest.cpp`
  - Repair front-wall geometry expectations that still assume the old body-frame sensor extrinsics and/or old yaw basis.

Additional user constraint captured mid-pass:
- The legacy `+X=forward, +Y=left, +theta=counterclockwise` basis is banned solution-wide, not merely in the runtime/project path.
- The only allowed appearance of that legacy basis is inside failure assertions/tests that loudly reject its reintroduction.

Verification after `DiagnosticCoverageTest.cpp` update:
- Rebuilt the release outputs far enough to refresh `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\x64\Release\MazeMapTest.dll` (timestamp `2026-04-02 00:53:49` local).
- Reran `ClassifyFrontCalibrationSpinHeadingFromNorthUsesOpenAndEastKnownWallBuckets` against that refreshed release DLL.
- Result: pass.

Completed after solution-wide legacy-basis sweep found a remaining default-frame leak:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\VehicleState.h`
  - Changed the default `SensorExtrinsics.directionBody` from `(1, 0)` to `(0, 1)` so unconfigured sensors now face forward in the project body frame (`+X right, +Y forward`) instead of inheriting the banned legacy `+X forward` basis.

Completed after front-wall geometry test sweep:
- `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\MazeMapTest.cpp`
  - Rebased the front-wall prediction fixture to place the known wall on `Direction::Up`, matching `yaw = 0` => facing north/+Y in the project frame.
  - Rebased the synthetic post-hit sensor to use body-forward `(0, 1)` before applying a `+45` degree clockwise yaw offset, removing the remaining test-local `(+1, 0) = forward` assumption.

Verification after `VehicleState.h` + `MazeMapTest.cpp` updates:
- Rebuilt `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\MazeMapTest.vcxproj` in release mode.
- Verified the refreshed release test DLL at `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\x64\Release\MazeMapTest.dll` (timestamp `2026-04-02 01:03:26` local).
- Reran:
  - `WallGeometryModelRespectsSensorExtrinsicsForFrontWallPrediction`
  - `WallGeometryModelCanIdentifyPostHits`
- Result: both pass.

Next active file:
- Repository-wide source sweep
  - Search for any remaining legacy `+X forward, +Y left, +theta CCW` assumptions or absolute/relative direction mixups before the final release verification pass.

Completed after stale generated-snapshot cleanup:
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\VehicleState.h`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\Vehicle.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\MouseUkf.h`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\MazeMapRuntimeCore.h`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\MazeMapRuntimeSensors.h`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\MazeMapRuntimeDrive.h`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\MazeMapStandaloneModes.cpp`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\MissionStartPolicy.h`
- `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\arduino_build_work_20260401_234449_042\sketch\Maneuver.h`
  - Synchronized the generated Arduino build snapshot with the corrected source files so the banned legacy basis is not left behind in copied build-work sources.

Final verification:
- Whole-workspace source sweep found no remaining matches for:
  - `directionBody = Eigen::Vector2f(1.0f, 0.0f)`
  - `accelMeasurementFromImu`
  - `imuAccelMeasurementMps2`
  - `HALF_PI_F - yaw*`
  - `return Eigen::Vector2f(cosf(yaw*), sinf(yaw*))`
  - `LeftUnitFromHeading`
  - the literal banned basis strings `+X=forward`, `+Y=left`, `+theta=counterclockwise`
- Ran the full release unit-test suite against `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\x64\Release\MazeMapTest.dll`.
- Result: 375 passed, 0 failed.
