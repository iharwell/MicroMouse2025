# Open Floor Mode Cleanup Audit Consolidated

Date: 2026-04-14

Scope: consolidated static cleanup audit of the open-floor boot mode implementation against `AGENTS.md`, with emphasis on authoritative ownership and avoiding mode-local management of tasks already owned elsewhere.

Build/tests: not run. This was a source review only.

This document consolidates:

- `open_floor_mode_cleanup_audit.md`
- `open_floor_mode_cleanup_audit_review.md`

## Summary

The original audit was directionally correct.

After consolidating overlapping items and doing one more review pass, the audit set comes down to five substantive findings. The second pass did not uncover another distinct category beyond these, but it did strengthen the configuration finding: active open-floor tuning still lives under stale `DiagnosticConfig` ownership, and `DiagnosticConfig` still carries stale geometry baggage even though open-floor geometry is already derived in `OpenFloorMeasurementSpec`.

## Findings

### 1. Legacy `diagnostic` entry points and the old `DiagnosticController` still exist alongside the open-floor mode

Severity: High

The codebase still exposes the primary diagnostic boot mode through the stale `GetDiagnosticMode()` name even though the actual authoritative implementation is `OpenFloorMeasurementController`. At the same time, the old `DiagnosticController` remains defined and compiled. That leaves two parallel concepts for the same boot-selected responsibility and makes the canonical owner harder to discover.

Evidence:

- `MazeMap/MazeMap/MazeMapControllerRegistry.h:15-16` exports `GetDiagnosticMode()` next to `GetOpenFloorMeasurementBootModeDescriptor()`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2948-2968` says the descriptor entry point is `GetDiagnosticMode`, and `GetDiagnosticMode()` returns `OpenFloorMeasurementController`.
- `MazeMap/MazeMap/MazeMapApplication.cpp:22-27`, `:35-39`, and `:68-69` still route `BootModeId::PrimaryDiagnostic` through a `diagnostic` slot.
- `MazeMap/MazeMap/DiagnosticController.cpp:17-19` explicitly calls the old controller legacy and says it is no longer registered, but the file still defines it and the project still builds it.

Recommended cleanup:

- Rename the factory surface to `GetOpenFloorMeasurementMode()`.
- Rename the application slot from `diagnostic` to an open-floor-specific name.
- Remove `DiagnosticController.cpp` from the build in the same change if it is truly obsolete.

### 2. The open-floor mode re-owns boot selector monitoring and duplicates boot-selector metadata that already belongs to `BootModeRegistry`

Severity: High

`BootModeRegistry` already owns the primary-diagnostic selector pins, but `OpenFloorMeasurementController` rebuilds a separate selector-monitor state machine, hard-codes selector wording into log metadata, and stores the selector-removal timeout in `OpenFloorMeasurementSpec`. That spreads one boot-entry condition across the registry, the mode, and a measurement-spec header.

Evidence:

- `MazeMap/MazeMap/BootModeRegistry.cpp:104-108` declares the primary-diagnostic selector as pin pair `27U, 28U`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1615-1674` looks that entry up and then owns its own monitor lifecycle through `_primaryDiagnosticSelector*` state plus direct `PinPairStrap` calls.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:866`, `:889-892`, and `:1380-1387` hard-code `pins_27_28_shorted_at_boot` into log metadata and run-start text.
- `MazeMap/MazeMap/OpenFloorMeasurementSpec.h:274-329` places `kOpenFloorSelectorRemovalFaultDelayMs` and `HasOpenFloorSelectorRemovalFaultDelayElapsed(...)` inside the open-floor measurement spec.

Recommended cleanup:

- Move selector-removal monitoring behind a registry-owned or framework-owned interface instead of keeping a mode-local selector manager.
- Derive boot-reason strings from the registry entry rather than hard-coding pin numbers inside the mode.
- Move the selector-removal timeout out of `OpenFloorMeasurementSpec`; it belongs with boot-mode selection metadata or a shared limit owner, not with open-floor motion/measurement definitions.

### 3. Shared boot-mode lifecycle and shared log-session setup are still implemented locally in the controller

Severity: Medium

The repository already has a `BootUtilityModeFramework`, but it currently only wraps startup-trace logging. `OpenFloorMeasurementController` still owns the larger boot-mode lifecycle itself: fault-handler registration, hardware setup, drive/sensor initialization, fan policy, wall-calibration reset, teardown, and the full setup/close path for the runtime-owned log sessions. The same pattern is repeated in other boot-selected modes, which means the cleanup threshold for shared framework ownership has already been crossed.

Evidence:

- `MazeMap/MazeMap/BootUtilityModeFramework.h:3-6` and `BootUtilityModeFramework.cpp:7-35` only provide `ResetStartupTrace(...)` and `AppendStartupTrace(...)`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:820-893` and `:1004-1073` locally assemble the timing-log and main-log sessions, including file open, metadata emission, schema begin, and text-log metadata bridging.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:991-1002`, `:1314-1325`, and `:1349-1451` locally perform log close/failure bookkeeping plus mode fault registration, `SetupHardware()`, drive begin, fan policy, wall-calibration reset, sensor bring-up, and shutdown cleanup.
- Similar duplicated mode-local setup remains in `MazeMap/MazeMap/AuxMeasurementController.cpp:320-366` and `MazeMap/MazeMap/FrontWallCharacterizationController.cpp:522-553`.
- `MazeMap/MazeMap/MazeMapSharedRuntime.h:61-143` and `MazeMap/MazeMap/MazeMapSharedRuntime.cpp:557-596`, `:965-1033` already expose the one runtime-owned utility logger and text-log lifecycle on the canonical owner.

Recommended cleanup:

- Expand `BootUtilityModeFramework` or one shared runtime helper so boot-selected modes stop individually managing shared begin/fail/teardown behavior.
- Move shared log-session setup/teardown/failure handling out of `OpenFloorMeasurementController`.
- Keep only row schemas and truly open-floor-specific metadata local to the mode.

### 4. Active open-floor tuning still lives under stale and mixed `DiagnosticConfig` ownership

Severity: Medium

The active open-floor path still pulls its live tuning from `DiagnosticConfig`, and the legacy `DiagnosticController` explicitly documents that `DiagnosticConfig` now exists to expose open-floor dials. That is already a stale ownership shape. With your clarification, the recovery-speed reuse is not evidence that open-floor should have its own dedicated recovery-tuning knob again; it is evidence that shared tuning is still parked under a characterization-named constant in a legacy namespace. At the same time, open-floor geometry is already derived elsewhere in `OpenFloorMeasurementSpec`. That leaves the production parameter set split across a legacy config namespace, a measurement spec header, and stale constant names.

Evidence:

- `MazeMap/MazeMap/DiagnosticController.cpp:17-19` says the legacy controller keeps its old constants private so `DiagnosticConfig` can expose the active open-floor dials.
- `MazeMap/MazeMap/DiagnosticConfig.h:10-70` defines active open-floor parameters such as control period, timing-capture count, static-hold time, launch/yaw/smooth/loop repeats, and recovery speed under `MazeMap::DiagnosticConfig`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:651`, `:872`, `:1054`, `:1365`, `:1458-1461`, `:1888`, `:2007`, `:2183`, `:2342`, `:2505`, `:2607`, `:2817-2828`, and `:2937` consume those `DiagnosticConfig` values throughout the active open-floor implementation.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1820` and `:2871` specifically use `DiagnosticConfig::kCharacterizationRecoverySpeedMps`. If that recovery speed is intentionally shared after removing the open-floor-specific recovery tuning, it still belongs under a shared/open-floor-appropriate owner and name instead of a characterization-named constant in `DiagnosticConfig`.
- `MazeMap/MazeMap/DiagnosticConfig.h:11` still carries `kHalfStepMm`, but `MazeMap/MazeMap/OpenFloorMeasurementSpec.h:315-323` now derives open-floor half-step geometry from `Maze::GetCellDimension()` instead.

Recommended cleanup:

- Converge the active open-floor tuning into one authoritative owner with open-floor-specific naming or a shared typed owner.
- Stop routing the active mode's configuration through a legacy `DiagnosticConfig` namespace.
- If the recovery speed is intentionally shared, rename and re-home it as shared tuning rather than leaving it under a characterization-specific constant name.
- Reconcile `DiagnosticConfig` and `OpenFloorMeasurementSpec` so the production parameter set is discoverable through one canonical ownership hierarchy.

### 5. Dead and deprecated scaffolding remains in the controller

Severity: Low

There is straightforward cleanup left inside the controller itself: unused deprecated includes, an unused runtime-owned `Vehicle` reference, and dead logging helpers/constants that no longer participate in the canonical path.

Evidence:

- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:14-15` includes `RuntimeBinaryLogSupport.h` and `WallSensorLedCalibrationPhase.h`, but the file does not reference anything from either header.
- `AGENTS.md` explicitly marks `RuntimeBinaryLogSupport.h` as deprecated.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:415` declares `_vehicle`, and `:625` initializes it, but the member is never used elsewhere in the file.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:162-163` declares `kOpenFloorLoggerFlagOverflow` and `kOpenFloorLoggerFlagWriteFailure`, but nothing in the file uses those constants.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:501` declares `ServiceLogs()`, and `:976-989` defines it, but there are no call sites for that helper.

Recommended cleanup:

- Remove the unused includes.
- Drop the unused `_vehicle` member and constructor wiring.
- Delete the unused logger-flag constants.
- Delete `ServiceLogs()` unless there is a concrete plan to wire it back into the canonical control/logging path.
- Recheck the file's direct include list after those deletions so it only depends on the authoritative owners it actually uses.
