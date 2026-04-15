# Open Floor Cleanup Audit Review Addendum

Date: 2026-04-14

Scope: review of `open_floor_mode_cleanup_audit.md` against the current open-floor implementation and adjacent runtime/config owners.

Build/tests: not run. This was a source review only.

## Summary

The existing audit's four findings are supported by the current code.

I found three additional cleanup issues that were not called out in the original audit.

## Additional Findings

### 1. Active open-floor tuning still lives under the stale `DiagnosticConfig` owner

Severity: Medium

The current audit catches stale `diagnostic` naming on the mode entry points, but it misses the same ownership problem in configuration. The authoritative open-floor path still pulls its active tuning from `DiagnosticConfig`, and the legacy `DiagnosticController` explicitly documents that `DiagnosticConfig` now exists to expose open-floor dials. That leaves open-floor behavior split across a legacy-named config namespace and `OpenFloorMeasurementSpec`, which conflicts with the repo rule that configuration must have one authoritative ownership hierarchy instead of stale or copied config surfaces.

Evidence:

- `MazeMap/MazeMap/DiagnosticConfig.h:10-70` defines open-floor workspace and run parameters such as half-step size, control period, timing-capture count, static-hold time, launch/yaw/smooth/loop repeats, and recovery speed under `MazeMap::DiagnosticConfig`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:651`, `:872`, `:1054`, `:1365`, `:1458-1461`, `:1888`, `:2007`, `:2183`, `:2342`, `:2505`, `:2607`, `:2817-2828`, `:2871`, and `:2937` consume those `DiagnosticConfig` values throughout the active open-floor implementation.
- `MazeMap/MazeMap/DiagnosticController.cpp:17-19` says the legacy controller keeps its old constants private so `DiagnosticConfig` can expose the active open-floor dials.

Recommended cleanup:

- Converge the active open-floor tuning into one authoritative owner with open-floor-specific naming or a shared typed owner.
- Stop routing the active mode's configuration through a legacy `DiagnosticConfig` namespace.
- Reconcile the split between `DiagnosticConfig` and `OpenFloorMeasurementSpec` so the production parameter set is discoverable through one canonical ownership path.

### 2. Open-floor still owns shared logger session setup and metadata boilerplate

Severity: Medium

The original audit correctly calls out shared utility-mode bring-up and teardown, but it does not separately call out the logging lifecycle. `SharedRobotRuntime` already owns the single production `MmLogLogger`, `logging.txt`, log servicing, and close paths, yet `OpenFloorMeasurementController` still hand-builds both utility log sessions: file open, metadata emission, schema begin, text-log metadata bridging, failure capture, and close. That is shared runtime/framework work, not open-floor experiment logic.

Evidence:

- `MazeMap/MazeMap/MazeMapSharedRuntime.h:32-35`, `:61-67`, `:76-97`, and `:107-143` expose the runtime-owned utility log/text log API on the shared owner.
- `MazeMap/MazeMap/MazeMapSharedRuntime.cpp:413-465` owns the single `dataLogger` and `textLogFile`, and `:557-596`, `:965-1033` centralize log open/service/close behavior around that runtime-owned logger.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:820-893` and `:1004-1073` still manually assemble the timing-log and main-log sessions, including repeated metadata plumbing and schema startup.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:991-1002` and `:1314-1325` still own close/failure bookkeeping for those sessions.
- Similar duplicated mode-local logging setup remains in `MazeMap/MazeMap/AuxMeasurementController.cpp:320-366` and `MazeMap/MazeMap/FrontWallCharacterizationController.cpp:522-553`.

Recommended cleanup:

- Move shared utility log session setup/teardown/failure handling into `BootUtilityModeFramework` or one shared runtime helper.
- Keep only the row schemas and truly mode-specific metadata values local to open-floor.
- Eliminate repeated mode-local boilerplate for pairing `logging.txt` with utility `mmlog` files.

### 3. Additional dead logging scaffolding remains in the controller

Severity: Low

The original audit already catches unused includes and the dead `_vehicle` reference, but there is more dead logging scaffolding left in the file. This is minor, but it still conflicts with the repo cleanup rule to delete superseded helpers and not leave migration residue beside the canonical path.

Evidence:

- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:162-163` declares `kOpenFloorLoggerFlagOverflow` and `kOpenFloorLoggerFlagWriteFailure`, but nothing in the file uses those constants.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:501` declares `ServiceLogs()`, and `:976-989` defines it, but there are no call sites for that helper.

Recommended cleanup:

- Delete the unused logger-flag constants.
- Delete `ServiceLogs()` unless there is a concrete plan to wire it back into the canonical control/logging path.
