# Worker I / Sartre Crash Handoff

Sartre was spawned after Leibniz and Bernoulli to continue the drive/vehicle/plant/sensor cleanup. It crashed with:

`Error running remote compact task: stream disconnected before completion`

No final implementation report was produced. Treat any changes since the prior worker reports as untrusted until reviewed.

## Known Context Before Crash

- Manager handoff: `MANAGER_HANDOFF_DRIVE_VEHICLE_PLANT_SENSOR_CLEANUP.md`.
- Leibniz report:
  - Removed production `PlantParams::Default()` / `PlantModel::Prepare(...)` uses from several controllers/runtime call sites.
  - Added `SharedRobotRuntime::Plant()` canonical access.
  - Added PlantModel domain operations for open-floor metadata, diagnostic motor-model metadata, and scalar command solves.
  - DriveBase mandatory search was clean for `PlantParams|PlantPreparedParams|_preparedParams|Prepare`.
  - Release verification built but release tests failed: 640 total, 560 passed, 80 failed.
- Bernoulli read-only triage:
  - Failure clusters were DriveBase oscillation, DriveManeuver timing/heading/variation, DirectionalPathFinder, SharedRuntime fault logging, PlantModel yaw-only breakaway, and SrUkfCore pivot/grip/control behavior.

## State Observed After Crash

Targeted manager checks after Sartre crash:

- Dirty tree still has the broad cleanup patch across `MazeMap/MazeMap`, `MazeMap/MazeMapTest`, and `MazeMap/MazeSimulation`.
- Extra untracked file exists: `codex_verify/plant_probe.cpp`.
- Newest verification log is `codex_verify/logs/build_and_verify_latest_20260510_160744_634.txt`.
- That log shows a completed build/test run ending `2026-05-10 16:13:21 -05:00`.
- Latest test result improved slightly but still fails: 640 total, 562 passed, 78 failed.

## Latest Failure Summary

From `build_and_verify_latest_20260510_160744_634.txt`:

- DriveManeuver failures remain widespread, including S45LS heading/position/yaw acceleration/velocity and IP45/IP90/IP135/IP180 time acceptable.
- DirectionalPathFinder still has 3 failures:
  - `HalfStepPathFromTo_SameStartAndEndClearsEstimatedTime`
  - `HalfStepPathToNearestUnknown_ReturnsReachableUnknownFrontierCell`
  - `HalfStepPathToNearestUnknown_PrefersCurrentHeadingForSymmetricFrontiers`
- SharedRuntime fault logging still has 2 failures:
  - `SharedRuntime_FailActiveModeClosesAndDisablesRuntimeLogs`
  - `SharedRuntime_FaultCallbackCanStreamTextBeyondQueueCapacityByFlushingIncrementally`
- PlantModel still has:
  - `PlantModelSolveDriveCommandsForVelocityTargetYawOnlyRepresentativeStatesMeetBreakawayFloor`
- SrUkfCore still has 4 failures:
  - `SrUkfCorePivotScrubModeMasksEncoderYawAndAppliesSoftZeroU`
  - `SrUkfCoreNonPivotMotionLeavesPivotScrubTelemetryCleared`
  - `SrUkfCoreYawRateUpdateAppliesGripPseudoMeasurementForCredibleRollingGrip`
  - `SrUkfCoreDoesNotLetControlVectorCreateUnboundedForwardMotionWithEncoderOpposition`

## Gates Still Active

- Do not restore `Estimator::ukf()` or rejected Estimator convenience APIs.
- Do not reintroduce DriveBase knowledge of `PlantParams`, `PlantPreparedParams`, `_preparedParams`, or `Prepare`.
- Do not create public motor/PWM command side channels.
- Do not use wrappers/shims/broad public accessors to hide old ownership.
- Do not add anonymous namespaces in touched files.
- Re-run mandatory searches and `git diff --check` before final report.

