# Runtime Sensor Suite Merge Intentions

Date: 2026-04-15

User request:
- Merge `SensorSuite` and `DiagnosticSensorSuite` completely.
- Use one clearly named resulting class in its own `.h` and `.cpp`.
- Make produced results pass-by-reference only.
- Preserve all features currently offered by either class.
- Remove duplicate consumer-facing paths and force the project onto one canonical sensor track.
- Document the resulting public surface.

Key findings already verified:
- `SensorSuite.h` and `DiagnosticSensorSuite.h` are only alias headers. The real implementations are both inline inside `MazeMap/MazeMap/MazeMapRuntimeSensors.h`.
- `LoopController::CaptureSelectedTickState()` already always chooses the diagnostic path, so the mission-vs-diagnostic split is largely dead structure.
- `SharedRobotRuntime` currently owns both `missionSensors` and `diagnosticSensors`; this is the main ownership seam that must collapse.
- The existing duplication is not just the classes. There are also duplicate runtime accessors, duplicate loop-controller state fields, duplicate `DriveBase` overloads, duplicate helper overloads in `MazeMapRuntimeInfrastructure.h`, and tests that assert the duplicated ownership pattern.

Canonical direction to implement:
1. Introduce one authoritative class named `RuntimeSensorSuite` in:
   - `MazeMap/MazeMap/RuntimeSensorSuite.h`
   - `MazeMap/MazeMap/RuntimeSensorSuite.cpp`
2. Replace both `SensorSuite` and `DiagnosticSensorSuite` with that one owner.
3. Replace runtime accessors `MissionSensors()` and `DiagnosticSensors()` with one accessor:
   - `SharedRobotRuntime::Sensors()`
4. Use one authoritative snapshot type across the project:
   - keep `SensorSnapshot`
   - fold the diagnostic-only fields from `DiagnosticSensorSnapshot` into `SensorSnapshot`
   - remove `DiagnosticSensorSnapshot`
5. Make capture output pass-by-reference only:
   - remove return-by-value `Capture(...)`
   - use `Capture(..., SensorSnapshot& snapshot, ...)`
6. Keep one capture pipeline that always fills the full unified snapshot:
   - mission/control fields
   - wall sensor telemetry
   - ADC timing/config telemetry
   - IMU raw telemetry
   - accel/gyro bias fields
7. Update all consumers to the same track:
   - `LoopController`
   - `MazeMapMissionController`
   - `DiagnosticController`
   - `AuxMeasurementController`
   - `FrontWallCharacterizationController`
   - `OpenFloorMeasurementController`
   - `TopSpeedMeasurementMode`
   - `DriveBase`
   - runtime/log helpers
   - tests
8. Remove obsolete files and project entries:
   - `MazeMapRuntimeSensors.h`
   - `SensorSuite.h`
   - `DiagnosticSensorSuite.h`
   - old vcxproj/vcxproj.filters entries

Implementation notes for continuation:
- The unified `SensorSnapshot` should retain the existing mission scalar fields and gain these diagnostic fields:
  - `frontLeft`, `frontRight`, `sideLeft`, `sideRight` (`WallSensorTelemetry`)
  - `frontTiming`, `leftTiming`, `rightTiming`
  - `wallSensorAdcCfgBeforeStart`, `wallSensorAdcGcBeforeStart`
  - `wallSensorAdcCfgAfterStart`, `wallSensorAdcGcAfterStart`
  - `wallSensorAdcTargetCfg`, `wallSensorAdcIpgClockHz`
  - `imuFrontRight`, `imuBackLeft`, `imuTiming`
- The unified capture logic should start from the richer mission/control wall-observation path, then add the diagnostic telemetry population on top of it.
- `planarAccelMps2` should live directly in the unified snapshot so callers stop asking the suite to recompute it.
- `LoopController` should collapse to one `SensorSnapshot sensors` field and remove `diagnosticSensors` / `hasDiagnosticSensors`.
- `DriveBase` should collapse to one `UpdateOdometry(..., const SensorSnapshot&, ...)` track and one set of helper overloads.

Testing/verification intentions after edits:
- Check that the active binaries correspond to the latest sources before testing; do not clean/rebuild from scratch unless required for a specific problem.
- Use the project’s latest-build verification path rather than ad hoc rebuild steps.
- Run the relevant host-side Release unit tests after the refactor if the incremental build state is healthy.
- Stop immediately if `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`.

Known unrelated workspace state to avoid touching:
- `MazeMap/MazeMapTest/DriveBaseTest.cpp` already has user changes.
- `MazeMap/mazemap_header_cleanup_thread_backlog.md` is untracked.
