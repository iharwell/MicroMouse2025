# Open Floor Mode Cleanup Parallel Audit Addendum

Date: 2026-04-14

Scope: fresh parallel source audit of the open-floor boot mode and adjacent owners, run independently from the earlier consolidated audit to check for missed cleanup issues.

Method: three parallel review passes covering boot/registry ownership, runtime/logging/framework ownership, and controller/config/spec ownership, followed by local verification.

Build/tests: not run. This was a source review only.

## Summary

The earlier consolidated audit still stands.

This parallel pass found five additional issues that were not called out there.

## Additional Findings

### 1. The mode rereads the live selector as though it were a boot-latched fact

Severity: Medium

`OpenFloorMeasurementController` records `pins_27_28_shorted_at_boot` by rereading the selector during `Begin()`, then uses that reread result to decide whether selector-removal monitoring should even arm. That is not the same fact as "selected at boot", and it can disable removal monitoring if the strap is removed between boot selection and mode initialization.

Evidence:

- `MazeMap/MazeMap/MazeMapApplication.cpp:79-83` resolves the selected top-level boot mode before launching the active mode.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1372` sets `_pinsLatchedAtBoot` from `MazeMap::App::IsBootModeSelectorActive(MazeMap::App::BootModeId::PrimaryDiagnostic)`, which is a live reread.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1380-1383` logs that reread value as `pins_27_28_shorted_at_boot`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1617-1620` skips selector-removal monitoring entirely when `_pinsLatchedAtBoot` is false.

Recommended cleanup:

- Preserve the boot-selected condition in one authoritative startup-owned place and pass that fact into the mode.
- Do not rederive a boot-latched fact by rereading the selector pins inside `Begin()`.

### 2. `MazeMapApplication.cpp` still maintains a parallel boot-mode identity and dispatch authority outside `BootModeRegistry`

Severity: Medium

`BootModeRegistry` is supposed to be the single source of truth for top-level boot-mode discovery and selection metadata, but application launch still keeps a second mode inventory and dispatch table. It also launches through `selectedMode.descriptor->id` instead of the registry entry's own `id`, which makes the descriptor a competing owner of mode identity.

Evidence:

- `MazeMap/MazeMap/BootModeRegistry.h:32-37` gives each registry entry its own authoritative `id` plus descriptor reference.
- `MazeMap/MazeMap/BootModeDescriptor.h:25-37` duplicates registry-owned identity in the descriptor itself via `id` and `stableId`.
- `MazeMap/MazeMap/MazeMapApplication.cpp:19-43` hard-codes a second top-level mode inventory in `ApplicationControllers`.
- `MazeMap/MazeMap/MazeMapApplication.cpp:52-73` hard-codes a second dispatch table in `ResolveApplicationMode`.
- `MazeMap/MazeMap/MazeMapApplication.cpp:82-83` resolves launch through `selectedMode.descriptor->id` instead of `selectedMode.id`.

Recommended cleanup:

- Launch from `selectedMode.id`, not `selectedMode.descriptor->id`.
- Stop duplicating registry-owned mode identity in the descriptor or, at minimum, stop consuming descriptor identity for launch decisions.
- Converge the application dispatch path so `BootModeRegistry` remains the authoritative boot-mode inventory and identity owner.

### 3. Utility-log servicing policy is now hard-wired into `LoopController`

Severity: Medium

Open-floor no longer owns log servicing directly, but that policy did not land in an appropriate shared utility-mode owner. Instead, `LoopController` now directly decides when utility-log servicing runs, which mixes framework/logging policy into the generic timing owner.

Evidence:

- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:501` declares `ServiceLogs()`, and `:976-989` defines it, but the helper is dead.
- `MazeMap/MazeMap/LoopController.cpp:641`, `:649`, `:686`, and `:694` call `_runtime->ServiceUtilityDataLog()` inside generic capture/update lambdas.
- `MazeMap/MazeMap/LoopController.cpp:821-839` also hard-wires normal and fault-path log servicing through `_runtime->ServiceUtilityDataLog()`.
- `MazeMap/MazeMap/BootUtilityModeFramework.h:3-6` and `BootUtilityModeFramework.cpp:7-35` still only cover startup-trace logging.

Recommended cleanup:

- Keep file-service implementation in `SharedRobotRuntime`, but move the policy for when utility-mode log servicing runs out of `LoopController` and behind a framework or explicit loop callback contract.
- Delete the dead mode-local `ServiceLogs()` path in open-floor as part of that convergence.

### 4. Open-floor support has leaked into shared runtime infrastructure and unrelated modes

Severity: Medium

Open-floor measurement support is no longer confined to the open-floor mode. Shared runtime infrastructure now includes open-floor headers and emits open-floor-specific schedule/bin/motion metadata for non-open-floor owners such as mission telemetry and the legacy diagnostic path. That makes open-floor support types a shared dumping ground instead of mode-local implementation detail.

Evidence:

- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h:5-7` includes `OpenFloorMeasurementCycle.h`, `OpenFloorMeasurementLabels.h`, and `OpenFloorMeasurementSpec.h` at shared-runtime scope.
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h:188-217` emits `open_floor_*` metadata inside `WriteDiagnosticTuningEvents(...)`.
- `MazeMap/MazeMap/DiagnosticController.cpp:1188-1203` consumes that shared helper for the legacy diagnostic mode.
- `MazeMap/MazeMap/MazeMapMissionController.cpp:655-660` consumes the same helper for mission telemetry.

Recommended cleanup:

- Remove open-floor-specific schedule/bin/motion metadata from shared runtime infrastructure.
- Keep shared runtime helpers limited to truly shared telemetry/runtime facts.
- Move open-floor-specific metadata emission back behind open-floor-owned code or an explicitly shared owner that is justified by more than one real consumer.

### 5. Motion limits and stop-condition formulas are still synthesized ad hoc inside `OpenFloorMeasurementController`

Severity: Medium

The controller is still the practical owner of motion-limit composition and timeout policy. It rebuilds `MotionLimits` locally from config values and embeds stop-condition formulas and minimum timeout policy directly in the mode. The repo guidance explicitly says software-induced limits and stop conditions belong in `SoftwareLimits` or another typed shared limit owner, not as scattered mode-local formulas.

Evidence:

- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:255` declares `kMinimumFailureTimeoutMs`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1454-1462` rebuilds `MotionLimits` locally from `DiagnosticConfig`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:1516-1519` applies a mode-local timeout floor in `FailureTimeoutMs(...)`.
- `MazeMap/MazeMap/OpenFloorMeasurementController.cpp:2325`, `:2331-2332`, `:2591-2598`, and `:2789-2791` thread those limits and timeout formulas through straight, turn, and smooth-turn phases.

Recommended cleanup:

- Move software-induced limits and timeout policy into `SoftwareLimits` or one typed owner shared across modes.
- Have the controller consume resolved limits instead of composing them ad hoc.

## Result

After local verification, these five findings were the additional issues that held up from the parallel pass. I did not find another distinct missed category beyond them.
