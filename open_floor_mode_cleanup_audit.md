# Open Floor Mode Cleanup Audit

Date: 2026-04-14

Scope: static cleanup audit of the open-floor boot mode implementation against `AGENTS.md`, with emphasis on avoiding ownership of tasks that belong to other objects.

Build/tests: not run. This audit was a source review only.

## Findings

### 1. Legacy `diagnostic` entry points and the old `DiagnosticController` still exist alongside the open-floor mode

Severity: High

The codebase still exposes the primary diagnostic mode through the stale `GetDiagnosticMode()` name even though the actual authoritative implementation is `OpenFloorMeasurementController`. At the same time, the old `DiagnosticController` remains defined and compiled. That leaves two parallel concepts for the same boot-selected responsibility and makes the canonical owner harder to discover.

Evidence:

- `MazeMap/MazeMap/MazeMapControllerRegistry.h:15-16` exports `GetDiagnosticMode()` next to `GetOpenFloorMeasurementBootModeDescriptor()`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2956-2968` says the descriptor entry point is `GetDiagnosticMode`, and `GetDiagnosticMode()` returns `OpenFloorMeasurementController`.
- `MazeMap/MazeMap/MazeMapApplication.cpp:22-27` and `:35-39` still name the resolved controller slot `diagnostic`, and `:68-69` routes `BootModeId::PrimaryDiagnostic` through that stale name.
- `MazeMap/MazeMap/DiagnosticController.cpp:17-19` explicitly calls the old controller legacy and says it is no longer registered, but `:42-45` still defines the class.
- `MazeMap/MazeMap/MazeMap.vcxproj:323-329` still builds both `DiagnosticController.cpp` and `OpenFloorMeasurementController.cpp`.

Recommended cleanup:

- Rename the factory surface to `GetOpenFloorMeasurementMode()`.
- Rename the application slot from `diagnostic` to an open-floor-specific name.
- Remove `DiagnosticController.cpp` from the build in the same change if it is truly obsolete.

### 2. The open-floor mode re-owns boot selector monitoring and duplicates boot-selector metadata that already belongs to `BootModeRegistry`

Severity: High

`BootModeRegistry` already owns the primary-diagnostic selector pins, but `OpenFloorMeasurementController` rebuilds a separate selector-monitor state machine, hard-codes the selector wording into log metadata, and stores the selector-removal timeout inside `OpenFloorMeasurementSpec`. That spreads one boot-entry condition across the registry, the mode, and a measurement-spec header.

Evidence:

- `MazeMap/MazeMap/BootModeRegistry.cpp:104-108` declares the primary-diagnostic selector as pin pair `27U, 28U`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1615-1674` looks that entry up and then owns its own monitor lifecycle through `_primaryDiagnosticSelector*` state plus direct `PinPairStrap` calls.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:866`, `:889-892`, and `:1380-1387` hard-code `pins_27_28_shorted_at_boot` into log metadata and run-start text.
- `MazeMap/MazeMap/OpenFloorMeasurementSpec.h:274-329` places `kOpenFloorSelectorRemovalFaultDelayMs` and `HasOpenFloorSelectorRemovalFaultDelayElapsed(...)` inside the open-floor measurement spec.

Recommended cleanup:

- Move selector-removal monitoring behind a registry-owned or framework-owned interface instead of keeping a mode-local selector manager.
- Derive boot-reason strings from the registry entry rather than hard-coding pin numbers inside the mode.
- Move the selector-removal timeout out of `OpenFloorMeasurementSpec`; it belongs with boot-mode selection metadata or a shared limit owner, not with open-floor motion/measurement definitions.

### 3. Shared utility-mode lifecycle is still implemented locally in the controller instead of converging on shared infrastructure

Severity: Medium

The repository already has a `BootUtilityModeFramework`, but it currently only wraps startup-trace logging. `OpenFloorMeasurementController` still owns the larger utility-mode lifecycle itself: mode fault registration, hardware setup, drive/sensor initialization, fan policy, wall-calibration reset, and teardown. The same pattern is repeated in other utility modes, which means the cleanup threshold for shared framework ownership has already been crossed.

Evidence:

- `MazeMap/MazeMap/BootUtilityModeFramework.h:3-6` and `BootUtilityModeFramework.cpp:7-35` only provide `ResetStartupTrace(...)` and `AppendStartupTrace(...)`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1349-1451` locally performs fault-handler registration, `SetupHardware()`, startup trace, drive begin, fan enable/disable, wall-calibration clearing, diagnostic sensor begin, and shutdown cleanup.
- `MazeMap/MazeMap/AuxMeasurementController.cpp:36-68` repeats the same setup pattern.
- `MazeMap/MazeMap/FrontWallCharacterizationController.cpp:46-71` repeats the same setup pattern again.

Recommended cleanup:

- Expand `BootUtilityModeFramework` or another single shared owner so utility modes stop individually managing the shared begin/fail/teardown path.
- Keep open-floor-specific sequencing local, but move the repeated runtime bring-up and cleanup responsibilities out of the mode.

### 4. The controller still carries dead or deprecated dependencies

Severity: Low

There is straightforward cleanup left inside the controller itself: it includes a deprecated header, pulls in an unrelated calibration-phase header, and stores a runtime-owned `Vehicle` reference that is never used.

Evidence:

- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:14-15` includes `RuntimeBinaryLogSupport.h` and `WallSensorLedCalibrationPhase.h`, but the file does not reference anything from either header.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:415` declares `_vehicle`, and `:625` initializes it, but the member is never used elsewhere in the file.
- `AGENTS.md` explicitly marks `RuntimeBinaryLogSupport.h` as deprecated.

Recommended cleanup:

- Remove the unused includes.
- Drop the unused `_vehicle` member and constructor wiring.
- Recheck the file's direct include list after those deletions so it only depends on the authoritative owners it actually uses.
