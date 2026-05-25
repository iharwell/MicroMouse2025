# Wall Sensor Runtime/Calibration Cleanup Work Log

## 2026-05-25 05:48:14 -05:00

- Inspected `AGENTS.md`, `git status --short`, and the active wall-sensor source graph before edits.
- Created temporary non-compiled reference copies under `.codex_tmp/wall_sensor_migration_reference`.
- Break-first deleted compiled public helper surface from `MazeMap/MazeMap/MazeMapRuntimeCore.h`:
  - `AsyncWallSensorSweepStage`, `AsyncWallSensorSweepRead`
  - `StartAsyncWallSensorSweepRead`, `ServiceAsyncWallSensorSweepRead`, `AwaitAsyncWallSensorSweepRead`, `AbortAsyncWallSensorSweepRead`
  - `WallSensorCalibrationInput`, `WallSensorCalibrationCapture`, `AveragedWallSensorInputWindow`
  - `BuildWallSensorCalibrationInput`, `SampleWallCalibrationInputRaw*`, `SampleWallCalibrationCaptureAverageRaw*`
  - `WallSensorIdName`, `WallSensorId`-based front threshold helper
- Deleted `WallSensorId` and `IsFrontWallSensor` from `MazeMap/MazeMap/WallSensorRuntimeTypes.h`.
- Stitched runtime capture into `RuntimeSensorSuite`: fixed front/IMU/left/right/finish sequencing now lives in `RuntimeSensorSuite`; per-sensor capture lifecycle remains on `WallSensor`.
- Stitched startup calibration sampling into `StartupCalibration` private procedure code with explicit side-pair and front-pair sampling.
- Moved side-wall signal distance computation onto `WallDistanceCalibration::TryComputeSideWallSignalDistanceM`.
- Replaced LED calibration `WallSensorId` selector use with explicit front/side hardware timing constants.
- Rewrote the helper-only async sweep test into a `WallSensor` capture lifecycle test.
- Verification run:
  - Command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`
  - Log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260525_053940_033.txt`
  - Teensy compile: passed.
  - Release host build: passed.
  - Release unit tests: failed. Most failures were unrelated Drive/Maneuver/Plant/Estimator assertions. One edited wall-sensor lifecycle test failed due an incorrect assertion about the final LED-off timestamp replacing the initial LED-off timestamp.
- Static check before graph refresh:
  - Command: `rg -n "AsyncWallSensorSweep|WallSensorId|WallSensorCalibrationInput|WallSensorCalibrationCapture|AveragedWallSensorInputWindow|BuildWallSensorCalibrationInput|SampleWallCalibration" MazeMap\MazeMap MazeMap\MazeMapTest Tools`
  - Result: source/test hits removed; remaining hits were stale generated `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` entries.
- Refreshed `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` with `python tooling\generate_maze_map_dependency_graph.py`.
- Static check after graph refresh:
  - Same command.
  - Result: no hits.
- Narrowed `RuntimeSensorSuite.h` off `MazeMapRuntimeCore.h`; `RuntimeSensorSuite.cpp` still includes it for non-wall runtime helpers pending compile confirmation.
- Fixed the wall-sensor lifecycle test assertion to check final `ledOffCommandUs >= adcOnSampleUs` and `observationReadyUs == ledOffCommandUs`.

## 2026-05-25 06:06:55 -05:00

- Checkpoint: migration is buildable. Latest Teensy compile and Release host build passed; Release unit tests still fail only in unrelated Drive/Maneuver/Plant/Estimator families.
- Deleted old compiled surfaces: `AsyncWallSensorSweep*`, `WallSensorCalibrationInput`, `WallSensorCalibrationCapture`, `AveragedWallSensorInputWindow`, `BuildWallSensorCalibrationInput`, `SampleWallCalibration*`, `WallSensorId`, ID-based LED timing helpers, and free `TryComputeSideWallSignalDistanceM`.
- `RuntimeSensorSuite` no longer includes `MazeMapRuntimeCore.h` or `WallSensorRuntimeTypes.h`.
- Static forbidden-symbol check: no hits.
- Last static command: `rg -n "AsyncWallSensorSweep|WallSensorId|WallSensorCalibrationInput|WallSensorCalibrationCapture|AveragedWallSensorInputWindow|BuildWallSensorCalibrationInput|SampleWallCalibration" MazeMap\MazeMap MazeMap\MazeMapTest Tools`.
- Last verification command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`; log `codex_verify\logs\build_and_verify_latest_20260525_060242_229.txt`.
- Blocker: none in the wall-sensor migration path. Unrelated failing test families remain out of scope by user instruction.

## 2026-05-25 06:27:19 -05:00

- Checkpoint after verifier-blocker fixes only.
- Temp migration references: `.codex_tmp/wall_sensor_migration_reference` deleted; only `.codex_tmp/wall_sensor_cleanup_work_log.md` remains under `.codex_tmp` for this task.
- Residual calibration free helpers: `ComputeSignalRiseAboveBaselineValue` and `IsCalibratedSideDistanceValidForControl` removed from compiled source; static check has no hits.
- `WallSensorLedCalibrationController.cpp`: anonymous namespace removed; static namespace check has no hits.
- Static forbidden-symbol check: no hits for `AsyncWallSensorSweep|WallSensorId|WallSensorCalibrationInput|WallSensorCalibrationCapture|AveragedWallSensorInputWindow|BuildWallSensorCalibrationInput|SampleWallCalibration`.
- Last verification command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`; log `codex_verify\logs\build_and_verify_latest_20260525_061757_823.txt`.
- Verification result: Teensy compile passed and Release host build passed; Release unit tests failed in unrelated Drive/Maneuver/Plant/Estimator families, with no wall-sensor failure observed. No unrelated tests were changed.

## 2026-05-25 06:44:56 -05:00

- Process correction: user flagged direct in-place editing during RuntimeSensorSuite thick-edge cleanup. Created non-compiled reference copies under .codex_tmp/wall_sensor_runtime_suite_reference before continuing copy-delete-stitch. These are temporary migration references and must be deleted before final; the work log remains the only persistent .codex_tmp artifact.

## 2026-05-25 06:53:08 -05:00

- Takeover after failed RuntimeSensorSuite hub cleanup worker.
- Re-read `AGENTS.md`, current work log, `git status --short`, and `git diff --stat`.
- Observed existing dirty wall-sensor cleanup edits and a temporary `.codex_tmp/wall_sensor_runtime_suite_reference` directory from the prior worker.
- Continuing under the stricter user-requested copy-delete-stitch protocol; any additional materially rewritten compiled file will first be copied to `.codex_tmp/wall_sensor_migration_reference_2/`, then rebuilt canonically.

## 2026-05-25 07:21:27 -05:00

- Process correction from user: section-level copy/delete/stitch is invalid for this task.
- Correct protocol now being followed for materially rewritten RuntimeSensorSuite-hub files: copy the whole compiled file to non-compiled reference, delete the active compile-path file/content, then hand-assemble a new compile-path file from the top with only intentionally retained behavior.
- User clarified that this file-level rewrite pressure is intentional; if a file is too monolithic to hand-assemble safely, that points to an ownership problem, not permission to add wrappers/helpers/managers.
- `RuntimeSensorSuite.h`, `StartupCalibration.h`, and `StartupCalibration.cpp` have been deleted and re-added through explicit `apply_patch` full-file content. `RuntimeSensorSuite.cpp` still needs the same file-level handwrite pass before final verification.

## 2026-05-25 07:28:48 -05:00

- Takeover after Kuhn crashed during compaction.
- Re-read `AGENTS.md` and `.codex_tmp/wall_sensor_cleanup_work_log.md` before inspecting source.
- Treating existing dirty RuntimeSensorSuite-hub edits as untrusted until verified against the hard ownership target.
- Continuing with the user-mandated file-level copy-delete-handwrite protocol, using `.codex_tmp/wall_sensor_migration_reference_3/` for any compiled file I materially rewrite.

## 2026-05-25 07:55:28 -05:00

- Inspected `git status --short`, `git diff --stat`, targeted diffs, current RuntimeSensorSuite/WallSensor/StartupCalibration/WallDistanceCalibration source, and existing temporary reference directories.
- Copied the whole current `MazeMap/MazeMap/RuntimeSensorSuite.h` and `MazeMap/MazeMap/RuntimeSensorSuite.cpp` to `.codex_tmp/wall_sensor_migration_reference_3/` before editing.
- Deleted both active RuntimeSensorSuite compile-path files and hand-added new full-file content through `apply_patch`.
- RuntimeSensorSuite changes:
  - removed private bag-like `EncoderCountSnapshot` and `AveragedBackLeftImuSample` nested types,
  - kept fixed wall-sensor choreography in `BeginInterlacedCapture`, `ServiceFrontWallCollection`, `CaptureInterlacedInertialSnapshot`, `ServiceLeftWallCollection`, `ServiceRightWallCollection`, and `FinishInterlacedCapture`,
  - preserved per-device LED/capture lifecycle on `WallSensor`,
  - fixed the stale side-wall window call to use the new `VehicleState` + concrete `WallSensor` member signature,
  - kept `RuntimeSensorSuite` independent of `MazeMapRuntimeCore.h` and `WallSensorRuntimeTypes.h`.
- Diff hygiene: `git diff --check` passed.
- Verification command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`.
- Verification log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260525_074631_040.txt`.
- Verification result:
  - Teensy compile passed.
  - Release host build passed.
  - Release unit tests failed in unrelated Drive/Maneuver/Plant/Estimator families; no wall-sensor/runtime helper failure appeared in the reported failures. Per instruction, no unrelated test fixes were attempted.
- Regenerated `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` from the latest Release tlogs with `python tooling\generate_maze_map_dependency_graph.py`.
- Graph refresh summary: `compiled_existing_sources=68`, `active_local_files=190`, `classes=184`, `structs=61`, `functions=507`, `edges=1771`, `weight_counts=0.1:314, 0.2:1014, 0.5:69, 1:374`.
- Required static checks after the rewrite:
  - forbidden async/calibration helper symbols: no hits,
  - `WallSensorId`: no hits,
  - RuntimeSensorSuite include/ID check: no hits,
  - anonymous namespace check on touched RuntimeSensorSuite-hub files: no hits.
- Dependency graph stale-symbol check for deleted wall-sensor APIs and `WallSensorId`: no hits.
- RuntimeSensorSuite-hub thick-edge audit:
  - removed stale thick edges to the old async sweep/read helpers, wall-sensor calibration bags, `WallSensorId`, and RuntimeSensorSuite anonymous helper nodes,
  - no noncanonical wall-sensor wrapper/helper thick edge remains directly connected to `RuntimeSensorSuite`,
  - remaining direct `weight=1` RuntimeSensorSuite graph edges are exception candidates requiring user approval rather than agent self-waiver: hardware timing/pin primitives (`delay`, `delayMicroseconds`, `micros`, `millis`, `pinMode`, `digitalRead`), telemetry structs (`ImuObservationTiming`, `ImuTelemetry`, `WallSensorTelemetry`), policy/math helpers (`ComputeCellInner*`, `ComputeGyroBiasSampleCount`, `IsAccelSelfTestDeltaValidMg`, `IsGyroSelfTestDeltaValidDps`, `IsWithinWallSegmentCenterWindowM`, `SelectUiImuSamplingProfile`, `ShouldUpdateGyroBiasFromStationarySample`, `Math::Sqrtf`), runtime logging access (`GetSharedRobotRuntime`), and graph-generator member-method nodes (`EstimateMissionGyroBiasRadps`, `IsSideWallDetectionWindowValid`, `ReadBackLeftGyroZRadpsRaw`, `SensorWorldFacing`, `SensorWorldPosition`, `TryComputeSideWallAimCoordinateM`).
- Removed temporary migration references `.codex_tmp/wall_sensor_migration_reference_2`, `.codex_tmp/wall_sensor_migration_reference_3`, and `.codex_tmp/wall_sensor_runtime_suite_reference`; `.codex_tmp` now contains only `wall_sensor_cleanup_work_log.md`.
- Blocker: Release unit-test verification remains blocked by unrelated Drive/Maneuver/Plant/Estimator failures per user instruction. No wall-sensor cleanup blocker remains.

## 2026-05-25 08:04:43 -05:00

- Verifier rejected the prior pass as architectural, not symbol-check related.
- Continuing without stopping.
- Action-needed items accepted for investigation:
  - RuntimeSensorSuite duplicated wall geometry/window helper behavior that still exists in `MazeMapRuntimeCore.h`.
  - RuntimeSensorSuite still owns side-wall signal classification/hysteresis behavior that should move to a true owner, likely `WallDistanceCalibration`.
  - RuntimeSensorSuite thick edges to geometry/window helpers and old gyro helper equivalents must be removed or recorded as user-approval exception candidates.
- Will use file-level copy-delete-handwrite for any materially rewritten compiled file in this follow-up pass.

## 2026-05-25 08:10:51 -05:00

- Takeover after Ramanujan crashed during compaction.
- Re-read checked-in `AGENTS.md` first, then `.codex_tmp/wall_sensor_cleanup_work_log.md`.
- Inspected initial `git status --short` and `git diff --stat`; current tree still has the prior wall-sensor cleanup edits plus `.codex_tmp` and untracked `WallSensor.cpp`/dependency graph tooling.
- Treating all prior RuntimeSensorSuite-hub edits as untrusted until verified against the user-provided blockers.
- Continuing under the user-mandated whole-file copy-delete-handwrite protocol. Any compiled file I materially rewrite in this pass will first be copied whole to `.codex_tmp/wall_sensor_migration_reference_4/`, then the active compile-path file will be deleted/replaced from the top.

## 2026-05-25 08:45:46 -05:00

- Copied whole active files to `.codex_tmp/wall_sensor_migration_reference_4/` before this pass: `RuntimeSensorSuite.h`, `RuntimeSensorSuite.cpp`, `WallDistanceCalibration.h`, `WallSensor.h`, `WallSensor.cpp`, `MazeMapRuntimeCore.h`, and `MazeMapRuntimeSignalHelpers.cpp`.
- Resolved the verifier's parallel-implementation blockers:
  - moved wall-sensor world-position/facing and side-wall aim/window geometry onto `WallSensor`,
  - deleted the old free `SensorWorldPosition`, `SensorWorldFacing`, `TryComputeSideWallAimCoordinateM`, and `IsSideWallDetectionWindowValid` helpers from `MazeMapRuntimeCore.h`,
  - changed runtime-core distance helpers and `MazeMapRuntimeSignalHelpers.cpp` to use the `WallSensor` geometry owner,
  - moved side-wall signal rise metrics, classifiability, transition detection, observation hit, control-range validity, and hysteresis state update policy from `RuntimeSensorSuite` into `WallDistanceCalibration`,
  - deleted the old free runtime-core gyro bias helpers `ReadBackLeftGyroZRadpsRaw` and `EstimateMissionGyroBiasRadps`,
  - deleted the unused public runtime-core IMU calibration sample/result/templates/constants and duplicate gravity constant after the read-only audit confirmed they were RuntimeSensorSuite duplicates.
- Applied the added user rule against public-field/no-method and private/internal bag structs:
  - removed the private `FrontSignalModelCache` struct in `WallDistanceCalibration`,
  - converted `WallSensor::DistanceModel` from a public-field struct into a small value class with accessors and distance behavior,
  - static search found no `struct` declarations in the touched RuntimeSensorSuite-hub files after these changes.
- Additional RuntimeSensorSuite-hub audit decisions:
  - `MazeMapRuntimeSignalHelpers.cpp` still calls `TryComputePoseAxisFromObservedWall`; this is not a direct RuntimeSensorSuite edge after graph refresh, so it is recorded as outside this hub slice rather than migrated here.
  - Remaining runtime-core wall-touch/start-cell/front-wall distance helpers are not directly connected to `RuntimeSensorSuite` after graph refresh; no unrelated WallTouch/Drive/Aux/Maneuver cleanup was started.
- Verification:
  - First post-move command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`, log `codex_verify\logs\build_and_verify_latest_20260525_082552_484.txt`; Teensy compile passed, Release host build passed, Release unit tests failed in unrelated Drive/Maneuver/Plant/Estimator families.
  - Second command after deleting runtime-core IMU duplicates: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`, log `codex_verify\logs\build_and_verify_latest_20260525_083619_399.txt`; Teensy compile passed, Release host build passed, Release unit tests failed in the same unrelated Drive/Maneuver/Plant/Estimator families.
- Regenerated `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` with `python tooling\generate_maze_map_dependency_graph.py`.
  - Graph summary: `compiled_existing_sources=68`, `active_local_files=190`, `classes=184`, `structs=57`, `functions=499`, `edges=1722`, `weight_counts=0.1:303, 0.2:989, 0.5:63, 1:367`.
- Static results:
  - `git diff --check` passed.
  - No hits for deleted old async/calibration helper surfaces or `WallSensorId`.
  - No hits for stale deleted runtime-core free helper nodes in the regenerated graph: `fn_EstimateMissionGyroBiasRadps`, `fn_ReadBackLeftGyroZRadpsRaw`, `fn_TryComputeSideWallAimCoordinateM`, `fn_SensorWorldPosition`, `fn_SensorWorldFacing`.
  - `RuntimeSensorSuite` has no `MazeMapRuntimeCore` or `WallSensorRuntimeTypes` include/dependency-name hits and no hits for the runtime-core wall geometry/window helpers named by the verifier.
  - Anonymous namespace check on touched RuntimeSensorSuite-hub files: no hits.
  - Temporary migration reference directory deleted; `.codex_tmp` contains only `wall_sensor_cleanup_work_log.md`.
- RuntimeSensorSuite-hub thick-edge audit after graph refresh:
  - Removed action-needed stale thick edges from `RuntimeSensorSuite` to old runtime-core helper/free functions: async sweep/read helpers, `WallSensorId`, wall-sensor calibration bags, `SensorWorldPosition`, `SensorWorldFacing`, `TryComputeSideWallAimCoordinateM`, old gyro-bias free helpers, runtime-core cell/window geometry helpers, and side-wall signal policy helpers.
  - Remaining direct `weight=1` `RuntimeSensorSuite` graph edges are exception candidates requiring user approval rather than agent-approved: hardware timing/pin primitives (`delay`, `delayMicroseconds`, `digitalRead`, `micros`, `millis`, `pinMode`), telemetry schemas/value types (`ImuObservationTiming`, `ImuTelemetry`, `WallSensorTelemetry`), runtime/logging access (`GetSharedRobotRuntime`), IMU policy/math helpers (`ComputeGyroBiasSampleCount`, `IsAccelSelfTestDeltaValidMg`, `IsGyroSelfTestDeltaValidDps`, `SelectUiImuSamplingProfile`, `ShouldUpdateGyroBiasFromStationarySample`, `Math::Sqrtf`).
- Blocker: Release unit-test verification is still blocked by unrelated Drive/Maneuver/Plant/Estimator failures per user instruction; no unrelated tests or assertions were weakened.

## 2026-05-25 09:01:09 -05:00

- Final verifier failed on remaining concrete blockers only.
- Accepted action-needed scope:
  - rename `RuntimeSensorSuite` member behavior still using old free-helper names `ReadBackLeftGyroZRadpsRaw` and `EstimateMissionGyroBiasRadps`,
  - remove/rename the `RuntimeSensorSuite` direct call spelling `IsSideWallDetectionWindowValid`,
  - delete the exported free `ComputeSignalRiseAboveBaseline` path from `MazeMapRuntimeSignalHelpers` and update only its direct helper test,
  - remove the non-domain public-field `NamedCode` struct from `MazeMapRuntimeCore.h`,
  - refresh or delete stale `tooling/latest_tlog_dependency_graph.gv`.
- Continuing under file-level migration-reference protocol with `.codex_tmp/wall_sensor_migration_reference_5/` for files touched in this pass.

## 2026-05-25 09:58:21 -05:00

- Takeover after Galileo interruption.
- Re-read `AGENTS.md` first, then `.codex_tmp/wall_sensor_cleanup_work_log.md`.
- Inspected initial `git status --short`, `git diff --stat`, and dirty-file list before edits.
- Continuing only within the RuntimeSensorSuite hub wall-sensor cleanup scope and the partial IMU ownership stitch.
- Treating the current `LSM6DSV16X_IMU.h` telemetry move as incomplete until reconciled against `RuntimeSensorSuite` callers and build verification.

## 2026-05-25 10:10:27 -05:00

- Finished the partial IMU telemetry ownership stitch:
  - kept `LSM6DSV16X_IMU::CaptureTelemetry(...)` as the authoritative register/timing telemetry capture owner,
  - removed `RuntimeSensorSuite::CaptureBackLeftImuTelemetry(...)` declaration and definition,
  - changed `RuntimeSensorSuite::CaptureInertialSnapshot(...)` to call `_vehicle.BackLeftImu().CaptureTelemetry(&imuTiming)` directly.
- Confirmed `tooling/generate_maze_map_dependency_graph.py` is legitimate current project tooling for regenerating `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` from MSVC tlogs; it is not an accidental helper dump.
- Diff hygiene: `git diff --check` passed, with only existing CRLF conversion warnings from the dirty files.
- Verification command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`.
- Verification log: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260525_100115_417.txt`.
- Verification result:
  - Teensy compile passed.
  - Release host build passed.
  - Latest release artifacts were verified current after the build.
  - Release unit tests failed in out-of-scope Drive/Maneuver/Plant/Estimator families; no wall-sensor, RuntimeSensorSuite, or IMU telemetry ownership failure appeared in the reported failure summary.
- Regenerated `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` with `python tooling\generate_maze_map_dependency_graph.py` after the build updated Release tlogs.
  - Graph summary: `compiled_existing_sources=68`, `cl_read_blocks=68`, `matched_blocks=68`, `active_local_files=190`, `classes=184`, `structs=55`, `functions=498`, `edges=1723`, `weight_counts=0.1:303, 0.2:989, 0.5:62, 1:369`.
- Static results:
  - no active-source/tooling hits for deleted async/calibration helper surfaces, old gyro/window helper names, `ComputeSignalRiseAboveBaseline`, or `NamedCode`,
  - no active-source/tooling hits for `WallSensorId`,
  - `RuntimeSensorSuite` has no `MazeMapRuntimeCore`, `WallSensorRuntimeTypes`, old geometry/window/gyro helper names, or `CaptureBackLeftImuTelemetry` hits,
  - no anonymous namespace hits in the touched RuntimeSensorSuite-hub files,
  - no `struct` definitions in the touched RuntimeSensorSuite-hub files inspected; the broader touched set has one existing `BootModeDescriptor` forward declaration only,
  - no stale generated-graph/tooling hits for the deleted helper names,
  - `tooling/latest_tlog_dependency_graph.gv` remains deleted.
- Deleted `.codex_tmp/wall_sensor_migration_reference_5/` after verifying the resolved path was under the workspace; `.codex_tmp` now contains only `wall_sensor_cleanup_work_log.md`.
- Remaining RuntimeSensorSuite direct `weight=1` graph edges are exception candidates requiring user approval rather than agent self-waiver: hardware timing/pin primitives (`delay`, `delayMicroseconds`, `digitalRead`, `micros`, `millis`, `pinMode`), telemetry value schemas (`ImuObservationTiming`, `ImuTelemetry`, `WallSensorTelemetry`), runtime logging access (`GetSharedRobotRuntime`), and IMU policy/math helpers (`ComputeGyroBiasSampleCount`, `IsAccelSelfTestDeltaValidMg`, `IsGyroSelfTestDeltaValidDps`, `Math::Sqrtf`, `SelectUiImuSamplingProfile`, `ShouldUpdateGyroBiasFromStationarySample`).
- Blocker: Release unit-test completion remains blocked by unrelated Drive/Maneuver/Plant/Estimator failures per user instruction. The tree is compile-buildable.

## 2026-05-25 10:42:11 -05:00

- Takeover for the rejected RuntimeSensorSuite thick-edge pass after the user ruled on remaining candidates.
- Re-read `AGENTS.md`, this work log, `git status --short`, and current RuntimeSensorSuite/StartupCalibration/IMU code before edits.
- Confirmed active `RuntimeSensorSuite.cpp` still contains direct blocking/timing calls (`delay`, `delayMicroseconds`, `millis`), direct `GetSharedRobotRuntime` logging, telemetry value names, and IMU policy/profile helper calls.
- User clarified that `millis` use reflects the wrong model and that `StartupCalibration` should own the async approach, not an overlapping side service.
- Copied the current active `RuntimeSensorSuite.h`, `RuntimeSensorSuite.cpp`, `StartupCalibration.h`, `StartupCalibration.cpp`, `LSM6DSV16X_IMU.h`, and `SensorSnapshot.h` to `.codex_tmp/runtime_sensor_suite_rejected_edges_reference/` before materially rewriting the active files.
- Verified `MazeMap::Math::Sqrtf` is the project implementation in `Defines.h`; the Teensy/ARM/VFP path uses inline `vsqrt.f32` assembly, so the user-approved condition holds.

## 2026-05-25 15:22:45 -05:00

- Takeover after Schrodinger crashed during compaction.
- Re-read `AGENTS.md`, this work log, current `git status --short`, and `git diff --stat` before editing.
- Continuing from the current dirty tree without reverting unrelated user edits.
- Applying the user rulings for remaining `RuntimeSensorSuite` direct edges: `digitalRead`, `pinMode`, `micros`, and project `Math::Sqrtf` are approved when still justified; blocking/timing waits, telemetry/policy helper ownership, and `GetSharedRobotRuntime` access from the suite remain action-needed.

## 2026-05-25 15:58:20 -05:00

- Checkpoint only, per user request.
- Exact rejected `RuntimeSensorSuite` searches:
  - `rg -n '(delayMicroseconds|\bdelay\s*\(|\bmillis\s*\()' .\MazeMap\MazeMap\RuntimeSensorSuite.cpp .\MazeMap\MazeMap\RuntimeSensorSuite.h`: no hits.
  - `rg -n '\b(ImuObservationTiming|ImuTelemetry|WallSensorTelemetry|GetSharedRobotRuntime|ComputeGyroBiasSampleCount|IsAccelSelfTestDeltaValidMg|IsGyroSelfTestDeltaValidDps|SelectUiImuSamplingProfile|ShouldUpdateGyroBiasFromStationarySample)\b' .\MazeMap\MazeMap\RuntimeSensorSuite.cpp .\MazeMap\MazeMap\RuntimeSensorSuite.h`: no hits.
- Dependency graph was regenerated after the last successful build-capable state. Current `RuntimeSensorSuite` direct `weight=1` graph edges are only the user-approved primitives: `digitalRead`, `micros`, and `pinMode`.
- Last full verification command before this checkpoint: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`; log `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260525_153518_948.txt`.
  - Teensy compile passed.
  - Release host build passed.
  - Release unit tests failed in out-of-scope Drive/Maneuver/Plant/Estimator families.
- Current active tree is not buildable as-is because the wall-sampling nonblocking conversion was interrupted after header/signature changes: `StartupCalibration.h` now declares `ResetWallSampleState`, boolean `SampleSideWallPair`, and boolean `SampleFrontWallPair`, while `StartupCalibration.cpp` still contains the old blocking `CaptureWallSensorPairBlocking` implementation and old `void` sample definitions/calls.
- Nonblocking IMU self-test/calibration migration status:
  - Implementable in this slice for IMU: `StartupCalibration` is the phase owner and can drive IMU baseline/self-test/disabled-settle/bias phases from `GetNextControls(...)`; `LSM6DSV16X_IMU` owns register configuration, telemetry capture, scaling, runtime bias storage, and self-test delta validation.
  - A broader phase-owner/user decision is needed only if wall-sensor calibration sampling must also be made nonblocking in the same slice, because that requires completing a separate multi-tick wall-sampling phase inside `StartupCalibration` rather than preserving the old blocking pair-read loop.
- Temporary non-compiled references currently remain under `.codex_tmp/runtime_sensor_suite_rejected_edges_reference` and `.codex_tmp/runtime_sensor_suite_rejected_edges_reference_3`; they must be deleted before final completion, but were left intact for checkpoint continuity.

## 2026-05-25 16:16:12 -05:00

- User correction received: wall-sensor calibration sampling is not out of scope and must be integrated into the existing `StartupCalibration` service/phase flow.
- Restoring only buildability by backing out the interrupted wall-sampling state is no longer acceptable; continuing with a coherent nonblocking wall-sensor sampling stitch.
- Saved current `StartupCalibration.cpp` and `StartupCalibration.h` into `.codex_tmp/runtime_sensor_suite_rejected_edges_reference_3/` as non-compiled references before replacing the active blocking sampling path.

## 2026-05-25 16:31:44 -05:00

- Reread `AGENTS.md` after user interruption, specifically the copy-delete-stitch, partial migration, duplicate-path, and build-size warning rules.
- Treating the active owner as `StartupCalibration`: the old compiled blocking wall-sensor pair path must be deleted from active source, not hidden or left as an alternate implementation.
- Build-size suspicion response: auditing for retained old paths and reducing duplicated sampling/averaging code before verification; any size increase must be explainable as replacing invalid blocking waits with bounded per-tick state, not preserving both paths.

## 2026-05-25 16:48:03 -05:00

- User clarification received: original `RuntimeSensorSuite` hub violations are not fixed if relocated to `LoopController` or another hub.
- Continuing the audit as an ownership check, not a graph-hiding exercise: no transport widening, no new structs/bags, no helper relocation to `LoopController`, and no extra callback/control-flow hubbing.
- Audited `LoopController` after graph regeneration:
  - The `ImuObservationTiming`, `ImuTelemetry`, and `WallSensorTelemetry` thick edges came from snapshot clearing code and were collapsed into one owner method on `SensorSnapshot` instead of four `LoopController` helper methods.
  - The remaining `delayMicroseconds` edge is the pre-existing strict-cadence scheduler wait in `LoopController::WaitUntilUs`, not a relocated wall/IMU calibration path from this slice.

## 2026-05-25 17:09:46 -05:00

- Takeover after Herschel crashed during compaction.
- Re-read checked-in `AGENTS.md`, then this work log, then inspected `git status --short`, `git diff --stat`, and the current work-log diff before any source edits.
- Continuing from the dirty tree without reverting unrelated user edits.
- Required completion target is a coherent nonblocking `StartupCalibration` service integration for wall-sensor calibration sampling, with rejected `RuntimeSensorSuite` edges kept clean and no transport widening, bags/structs, helper relocation to `LoopController`, or graph-metric-only fixes.

## 2026-05-25 17:33:59 -05:00

- Verified inherited active tree was buildable before source edits:
  - Command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`
  - Log: `codex_verify\logs\build_and_verify_latest_20260525_171130_777.txt`
  - Teensy compile passed, Release host build passed, Release unit tests failed in unrelated Drive/Maneuver/Plant/Estimator families.
- Copied whole active files to non-compiled references before this pass:
  - `.codex_tmp/runtime_sensor_suite_corridor_reference/`: `RuntimeSensorSuite.cpp`, `SensorSnapshot.h`, `MazeMapRuntimeSignalHelpers.h`, `MazeMapRuntimeSignalHelpers.cpp`, `RuntimeHelperTest.cpp`.
  - `.codex_tmp/startup_calibration_distance_reference/`: `StartupCalibration.h`, `StartupCalibration.cpp`, `MazeMapRuntimeCore.h`.
- Deleted the exported free `Runtime::ComputeCorridorError` helper and updated `RuntimeSensorSuite` to call the authoritative `SensorSnapshot::RecomputeCorridorErrorM(...)` owner method.
- Updated the direct helper test to construct `SensorSnapshot` and verify corridor-error recomputation through the canonical owner.
- Removed dead start-cell wall-distance inline helpers from `MazeMapRuntimeCore.h`: `TryDistanceToWestWall`, `TryDistanceToEastWall`, `TryDistanceToSouthWall`, `TryDistanceToNorthWall`, and `TryComputeNearestStartCellWallDistanceM`.
- Moved the remaining south-wall startup measurement geometry used by wall-sensor calibration sampling into `StartupCalibration::TryComputeDistanceToSouthStartWall(...)`.
- Verification:
  - Command: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\build_and_verify_latest.cmd --no-pause`
  - Log: `codex_verify\logs\build_and_verify_latest_20260525_172436_723.txt`
  - Teensy compile passed, Release host build passed, latest artifacts verified current, Release unit tests failed in the same unrelated Drive/Maneuver/Plant/Estimator families.
  - Firmware report changed from the inherited build's `FLASH code:263264, data:45000, headers:9172` to `FLASH code:263328, data:45000, headers:9108`; total firmware hex size stayed `892958` bytes.
- Regenerated `MazeMap/MazeMap/MazeMapClassStructDependencies.dot` with `python tooling\generate_maze_map_dependency_graph.py`.
  - Graph summary: `compiled_existing_sources=68`, `active_local_files=190`, `classes=183`, `structs=55`, `functions=492`, `edges=1682`, `weight_counts=0.1:293, 0.2:973, 0.5:62, 1:354`.
- Static/audit results:
  - `RuntimeSensorSuite` rejected-edge checks for `delayMicroseconds`, `delay(...)`, `millis(...)`, telemetry value types, `GetSharedRobotRuntime`, and rejected IMU policy/profile helpers: no hits.
  - Deleted wall-sensor helper surfaces, `WallSensorId`, `ComputeCorridorError`, and deleted start-cell distance helpers: no active-source/tooling/generated-graph hits.
  - Regenerated graph shows `RuntimeSensorSuite` direct `weight=1` edges only to user-approved `digitalRead`, `micros`, and `pinMode`.
  - `LoopController` remaining ADC-probe thick edges were confirmed present in `HEAD`, not moved from `RuntimeSensorSuite`; telemetry clearing helpers previously moved to `LoopController` are now owned by `SensorSnapshot::ClearUnavailableObservations(...)`.
- Deleted all temporary reference directories after verifying their resolved paths were under the workspace; `.codex_tmp` now contains only `wall_sensor_cleanup_work_log.md`.
- Blocker: Release unit-test completion remains blocked by unrelated Drive/Maneuver/Plant/Estimator failures per user instruction. The tree is compile-buildable.
