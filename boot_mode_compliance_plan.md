# Boot Mode File Compliance Plan

## Scope

- This document captures the current boot-mode layout and a staged path to align it with the repository's file, ownership, and boot-mode rules.
- The first implementation pass should be file-placement and include-hygiene work only. It should not change runtime behavior or redesign the class architecture yet.

## Post-Transplant Baseline

- The concrete standalone boot-mode controllers are now isolated into same-named implementation files:
  - `MazeMap/MazeMap/AuxMeasurementController.cpp`
  - `MazeMap/MazeMap/FrontWallCharacterizationController.cpp`
  - `MazeMap/MazeMap/WallSensorLedCalibrationController.cpp`
  - `MazeMap/MazeMap/DiagnosticController.cpp`
  - `MazeMap/MazeMap/OpenFloorMeasurementController.cpp`
- The mission-side boot-mode wrappers are now isolated into same-named files:
  - `MazeMap/MazeMap/MissionHostedModeBase.h/.cpp`
  - `MazeMap/MazeMap/MissionRunMode.h/.cpp`
  - `MazeMap/MazeMap/ManeuverFileTestMode.h/.cpp`
  - `MazeMap/MazeMap/CorridorRepeatabilityMode.h/.cpp`
  - `MazeMap/MazeMap/PositionAccuracyAuditMode.h/.cpp`
- This transplant intentionally preserves the current architecture:
  - `IApplicationMode` remains the top-level execution contract.
  - `IMissionModeHost` remains the mission-wrapper boundary.
  - `MazeMapApplication.cpp` still resolves startup precedence the same way.
  - `GetDiagnosticMode()` still resolves to `OpenFloorMeasurementController`.
- The next refactoring stages should improve ownership and shared boot-mode/session infrastructure without undoing the file split.

## Current State

- `MazeMap/MazeMap/MazeMapApplication.cpp` reads startup requests, resolves boot-mode precedence, and constructs the active mode from a mix of concrete mode objects and mission-hosted wrappers.
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` currently owns the boot-pin strap checks for the top-level modes.
- `MazeMap/MazeMap/MazeMapStandaloneModes.cpp` currently contains multiple substantive `IApplicationMode` implementations in one aggregate file:
  - `AuxMeasurementController`
  - `FrontWallCharacterizationController`
  - `WallSensorLedCalibrationController`
  - `DiagnosticController`
  - `OpenFloorMeasurementController`
- `MazeMap/MazeMap/MazeMapStandaloneModes.cpp` also contains the mode factory functions:
  - `GetAuxMeasurementMode()`
  - `GetFrontWallCharacterizationMode()`
  - `GetWallSensorLedCalibrationMode()`
  - `GetDiagnosticMode()`
- `MazeMap/MazeMap/MazeMapMissionModes.h` and `MazeMap/MazeMap/MazeMapMissionModes.cpp` currently group one shared base plus four concrete mission-side wrappers:
  - `MissionHostedModeBase`
  - `MissionRunMode`
  - `ManeuverFileTestMode`
  - `CorridorRepeatabilityMode`
  - `PositionAccuracyAuditMode`
- `MazeMap/MazeMap/MazeMapMissionController.cpp` is the current mission-side owner that implements `IMissionModeHost`.
- `MazeMap/MazeMap/MazeMapControllerRegistry.h` declares the mode accessors, but `MazeMap/MazeMap/MazeMapApplication.cpp` still redeclares those functions locally instead of using the authoritative header.

## Observed Compliance Gaps

- Multiple substantive boot/application mode classes are packed into aggregate files instead of same-named authoritative files.
- The mission-side wrapper classes are also grouped into aggregate files instead of same-named authoritative files.
- Boot-mode discovery metadata is scattered across:
  - `StartupMode`
  - `StartupModeRequests`
  - `ResolveStartupMode()`
  - inline pin-selection helpers in `MazeMapRuntimeInfrastructure.h`
  - mode-local human-readable text
- `BootModeRegistry` does not yet exist as the authoritative source of discovery metadata.
- The current `IMissionModeHost` shape encodes the mode list into `BeginXMode()` and `RunXMode()` methods, which the repository guidance explicitly rejects as the long-term design.
- `GetDiagnosticMode()` currently returns `OpenFloorMeasurementController`, while `DiagnosticController` remains defined in the same aggregate file but is not wired through the factory path. That must be resolved before architectural cleanup, because it is not clear whether `DiagnosticController` is obsolete or merely disconnected.
- `MazeMapStandaloneModes.cpp` also owns unrelated per-mode helpers and `mmlog` row declarations, which makes direct dependency ownership harder to trace.
- Several boot-selected modes still duplicate the same lifecycle responsibilities:
  - runtime fault registration and trampoline handling
  - hardware/setup entry
  - startup trace reset and trace append calls
  - sparse text-log announcements
  - drive and sensor bring-up
  - log open/flush/close and failure reporting
  - mode-local recovery and shutdown sequencing

## Target End State

- Every concrete boot-selectable mode class lives in its own same-named header/source pair.
- Shared mission-side helper classes also live in their own same-named files if they remain necessary after cleanup.
- `MazeMapApplication.cpp` depends on authoritative mode declarations only and does not locally redeclare registry/factory functions.
- `BootModeRegistry` becomes the single source of truth for boot-mode discovery metadata.
- Every boot-selectable mode has one authoritative descriptor colocated with the mode implementation or its header.
- Any remaining shared boot-mode/session infrastructure is introduced only if it clearly replaces duplicated machinery across multiple modes.
- Shared boot-mode/session infrastructure is owned in one place rather than repeated in each runtime-backed controller.

## Phase 1: File Compliance Without Architecture Change

- Status: completed by the transplant pass.

- Extract each concrete standalone mode into its own same-named authoritative implementation file in the transplant pass, with public headers deferred until a later stage actually needs them:
  - `AuxMeasurementController.cpp`
  - `FrontWallCharacterizationController.cpp`
  - `WallSensorLedCalibrationController.cpp`
  - `DiagnosticController.cpp`
  - `OpenFloorMeasurementController.cpp`
- Extract the grouped mission-side wrappers into their own authoritative files:
  - `MissionHostedModeBase.h/.cpp`
  - `MissionRunMode.h/.cpp`
  - `ManeuverFileTestMode.h/.cpp`
  - `CorridorRepeatabilityMode.h/.cpp`
  - `PositionAccuracyAuditMode.h/.cpp`
- Keep the current external behavior stable during this phase:
  - keep `IApplicationMode`
  - keep `IMissionModeHost`
  - keep `Get*Mode()` and `GetMissionModeHost()`
  - keep the current startup precedence rules
- Move each mode's local helper functions, constants, and `mmlog` row declarations into the matching mode file whenever those declarations are not genuinely shared.
- Replace the local mode-accessor forward declarations in `MazeMapApplication.cpp` with the authoritative header.
- Update `MazeMap.vcxproj` and `MazeMap.vcxproj.filters` so the build metadata points at the extracted authoritative files.
- Delete `MazeMapStandaloneModes.cpp` and `MazeMapMissionModes.h/.cpp` once all moved symbols are compiling from their new files.

## Phase 2: Boot Registry And Descriptor Compliance

- Introduce `BootModeRegistry` as the only authoritative owner of boot-mode discovery metadata.
- Define one registry entry per boot-selectable mode containing only:
  - stable mode id or display name
  - selector condition
  - reboot requirement
  - descriptor reference
- Replace the scattered `StartupModeRequests` plus `ResolveStartupMode()` chain with registry-driven resolution.
- Move boot selector pins and strap conditions out of scattered inline helpers and into `BootModeRegistry` or the pin map it owns or references.
- Add one authoritative descriptor per boot mode, colocated with the mode implementation or mode header, containing:
  - short purpose
  - primary outputs or logs
  - entry point
  - implementation file location
  - major phases or sections when meaningful
  - shared tuning used
  - explicit tuning overrides
  - expected artifacts produced
- Keep the registry focused on discovery only. It should not absorb mode behavior, logging mechanics, or runtime setup.

## Phase 3: Compliant Shared Boot-Mode Session Infrastructure

- Introduce `BootUtilityModeFramework` only for the runtime-backed boot-selected modes that genuinely share lifecycle mechanics:
  - `AuxMeasurementController`
  - `FrontWallCharacterizationController`
  - `OpenFloorMeasurementController`
  - any future measurement, audit, calibration, bring-up, or operational modes that need the same runtime/logging skeleton
- Do not force `WallSensorLedCalibrationController` into the framework unless it starts sharing enough machinery to justify it. It is currently simple enough to remain direct mode-local code.
- Define the framework's responsibilities narrowly:
  - acquire shared runtime access
  - register and route runtime fault handling
  - standardize startup trace reset and sparse text-log lifecycle
  - standardize begin/run/teardown sequencing where that sequencing is truly shared
  - expose shared helpers for safe logger bring-up, logger shutdown, and failure recording using the one runtime-owned logger architecture
  - provide common recovery and timeout helpers that respect the repository watchdog and recovery rules
- Keep the following out of the framework unless later evidence proves they are shared enough to own centrally:
  - per-mode geometry
  - per-mode measurement labels or cycle structs
  - per-mode `mmlog` row schemas
  - per-mode experiment sections
  - mode-specific tuning overrides
- Candidate authoritative files for that stage:
  - `BootUtilityModeFramework.h/.cpp`
  - `BootModeRegistry.h/.cpp`
  - one descriptor definition colocated beside each mode file
- Migration strategy for that stage:
  - start by extracting only the exact duplicated begin/fault/log/shutdown mechanics
  - move one boot-selected mode onto the shared framework
  - move the next boot-selected mode only after the first extraction proves the framework is actually reducing duplication
  - stop if the framework starts accumulating mode-specific knowledge instead of shared lifecycle mechanics
- The one runtime-owned logger rule remains unchanged:
  - the framework may reconfigure or reopen the canonical runtime-owned logger
  - the framework must not create parallel production logger owners

## Phase 4: Architectural Convergence After The File Split

- Decide the authoritative diagnostic boot mode:
  - If `OpenFloorMeasurementController` is the real replacement for the older diagnostic path, remove `DiagnosticController` in the same change that removes its last remaining references.
  - If both modes are still required, give each a distinct boot-registry entry and descriptor instead of hiding one behind the other's factory name.
- Revisit the mission-side forwarding wrappers:
  - `MissionRunMode`, `ManeuverFileTestMode`, `CorridorRepeatabilityMode`, and `PositionAccuracyAuditMode` are currently thin forwarding wrappers over `IMissionModeHost`.
  - The repository guidance rejects host interfaces that encode the mode list and rejects one forwarding wrapper per mode as the long-term shape.
  - After the file split, either move those workflows into more authoritative owners or converge on one cleaner boot-mode contract.
- Reduce or remove `MazeMapApplicationPrivate.h` as the extracted mode files gain direct includes for the owners and subsystems they actually use.

## Recommended Work Order

1. Extract the standalone boot-mode classes into same-named files while preserving existing factory entry points.
2. Extract the mission-side wrapper classes into same-named files while preserving `IMissionModeHost`.
3. Switch `MazeMapApplication.cpp` to authoritative declarations only and update Visual Studio project metadata.
4. Introduce `BootModeRegistry` and colocated descriptors without changing the mode implementations yet.
5. Extract the shared boot-mode/session lifecycle mechanics into `BootUtilityModeFramework` only where real duplication exists.
6. Resolve whether `DiagnosticController` is obsolete, disconnected, or still intended.
7. Remove the long-term noncompliant forwarding and selection shapes after registry ownership is in place.

## Verification To Run During Implementation

- Build `Release|x64` after the file split.
- Run the release-mode application tests that cover startup precedence.
- Add or update tests around boot-mode registry precedence once `BootModeRegistry` exists.
