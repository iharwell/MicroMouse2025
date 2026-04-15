# Boot Mode Setup Guide

This guide is the concrete setup checklist for adding a new boot-selectable top-level mode in this repository.

It is intentionally stricter than a generic "add a class and register it" guide:

- New modes must converge on the current canonical owners.
- New modes must use `LoopController`.
- New modes should treat `TopSpeedMeasurementMode` as the closest current implementation template.
- Legacy wrapper-based or pre-`LoopController` modes are not templates for new work.

All top-level modes are boot-selected once at startup. Do not model a new top-level mode as a runtime mode machine.

## Required Mode Shape

Use one authoritative mode owner:

- `FooMode.h`
- `FooMode.cpp`

That owner should usually implement `IApplicationMode` directly and expose:

- `bool Begin()`
- `void Run()`
- `const BootModeDescriptor& GetFooBootModeDescriptor()`
- `IApplicationMode& GetFooMode()`

For a new top-level mode, the normal file touch list is:

1. Add the mode's authoritative `.h/.cpp`.
2. Add or extend the `BootModeId` enum in `MazeMap/MazeMap/BootModeDescriptor.h` if the mode needs a new stable id.
3. Add the mode getter/descriptor declarations to `MazeMap/MazeMap/MazeMapControllerRegistry.h`.
4. Add the selector entry to `MazeMap/MazeMap/BootModeRegistry.cpp`.
5. Add the application-mode mapping in `MazeMap/MazeMap/MazeMapApplication.cpp`.
6. Add or update tests for the registry, mode selection, and any mode-local logic that is unit-testable.

Keep the metadata split clean:

- `BootModeRegistry` owns selector condition, precedence, reboot requirement, and descriptor reference.
- The mode descriptor owns purpose, outputs, phases, tuning notes, entry point, and implementation location.

Do not add a new per-mode host interface, forwarding wrapper, helper manager, or duplicate registry.

## Canonical Runtime Ownership

`SharedRobotRuntime` is the production owner for the shared runtime resources that a mode may borrow:

- `Maze`
- `Vehicle` instances
- `FloodFillPathFinder`
- `ManeuverPathFinder`
- `WallBeliefMap`
- `DriveBase`
- mission and diagnostic sensor suites
- the one production `MmLogLogger`
- `logging.txt`
- the singleton `LoopController`

Mode code must borrow these resources through `SharedRobotRuntime`. Do not create:

- another `MmLogLogger`
- another `logging.txt` owner
- another `Maze`
- another production pathfinder instance
- mode-local file-service loops
- dynamic allocations for shared pathfinders

Pathfinder and locomotion rules stay repo-wide:

- Use `FloodFill` for simple navigation.
- Use `ManeuverPathfinder` only while stationary.
- In a maze, prefer the `Maneuver` classes for locomotion.
- For more manual control, generate `ManeuverInstance` objects directly and add only the small target-yaw correction that is actually needed.
- Reserve direct position/yaw control for cases that cannot reasonably be expressed through maneuvers.

Fault ownership also stays centralized:

- Register one mode cleanup callback with `SharedRobotRuntime::RegisterModeFaultHandler()` during `Begin()`.
- Route fatal mode failures through `SharedRobotRuntime::FailActiveMode()`.
- `LoopController` may detect a stop condition, but `SharedRobotRuntime` remains the fault authority.

## Logging System

### `logging.txt`

`logging.txt` is runtime infrastructure. Treat it like `Serial`:

- Write through `SharedRobotRuntime`.
- Never open it directly.
- Never close it directly.
- Never create another text log file for a mode-specific logging subsystem.

Use `logging.txt` for sparse, human-readable records:

- operator prompts
- startup trace
- phase transitions
- mode result summaries
- fault reasons
- phase markers
- control/data file association metadata

Current concrete limits and behaviors:

- Queue capacity: `4096` bytes.
- Normal runtime service budget: `512` bytes per service call.
- `AppendTextLogFormatted()` formats into a local `384`-byte line buffer before enqueue.
- `WriteTextLogMetadata()` and `WriteTextLogPhase()` format into local `256`-byte message buffers.
- The text log is opened lazily by runtime code and is closed only by runtime shutdown/success/fault paths.

Operational rules:

- Keep `logging.txt` sparse.
- Do not stream per-tick telemetry to it.
- Do not flush it in the hot path just because a mode wants "live updates".
- Use `BootUtilityModeFramework::ResetStartupTrace()` / `AppendStartupTrace()` for startup-trace records instead of inventing another startup logging path.

### `mmlog`

Structured telemetry goes through the one runtime-owned `MmLogLogger` instance.

Correct session lifecycle:

1. `OpenUtilityDataLogFile()` or `OpenUtilityDataLog()`
2. `WriteUtilityDataLogMetadata*()` zero or more times
3. `BeginUtilityDataLogSchema(row)`
4. `LogUtilityDataRow(row)` during the loop
5. `ServiceUtilityDataLog()` during loop slack, through the runtime/`LoopController`
6. `FlushUtilityDataLog()` and/or `CloseUtilityDataLog()` only at phase boundaries or shutdown

Important runtime rules:

- Only one production `MmLogLogger` instance exists.
- To switch schemas or files, close and reopen the same runtime-owned logger.
- If the mode needs unbounded logging work between phases, request a `LoopController` pause first.
- `OpenUtilityDataLogFile()` already writes `file` and `control_log_file` metadata.

Current compile-time limits:

- Primary queue: `262144` bytes.
- Sidecar queue: `4096` bytes.
- Primary service budget: `512` bytes per `service()` call.
- Sidecar service budget: `512` bytes per `service()` call.
- Maximum metadata entries: `512`.
- Maximum metadata key length: `63` characters.
- Maximum metadata value length: `95` characters.
- Maximum label length: `96` characters.
- Maximum log path length: `128` characters.
- Maximum latched error text length: `96` characters.
- Maximum row size: `512` bytes.

Teensy FIFO SDIO preallocation rules:

- Primary file preallocation: `96 MiB`.
- Sidecar file preallocation: `1 MiB`.
- Minimum allowed preallocation per file: `1 MiB`.

Mode-author guidance:

- Use `MMLOG_DEFINE_ROW(...)` for row schemas.
- Keep each row under `512` bytes or split the stream.
- Put high-rate numeric data in rows, not in text.
- Put run configuration, revision strings, control periods, sensor scales, run ids, and similar setup facts in metadata.
- If you need to distinguish logger overflow from other write failures in a summary path, call `SharedRobotRuntime::CaptureUtilityDataLogFailure()`.
- If a logging call fails, inspect `SharedRobotRuntime::LastRuntimeLogError()` and fault the mode instead of building a mode-local recovery logger.

### `logging.txt` vs `mmlog`

Use `logging.txt` for:

- "what happened"
- "what phase are we in"
- "what file was opened"
- "why did we stop"
- short operator-facing text

Use `mmlog` metadata for:

- machine-readable run configuration
- format revisions
- tuning revisions
- file-level identity
- sensor scale or calibration facts
- run ids and mode ids

Use `mmlog` rows for:

- per-tick telemetry
- repeated structured measurements
- timing samples
- section-level result records that must decode cleanly to CSV

## `LoopController` Requirements

`LoopController` is required for new modes moving forward.

Do not add a new mode-local control-period gate, `_lastControlMicros`, or custom wait loop. New modes should not add another timing helper family.

The expected pattern is:

1. Do one-time setup in `Begin()`.
2. Build `LoopController::SessionOptions`.
3. Start a session in `Run()`.
4. Supply one mode-work callback.
5. Use `TickServices` to end, fault, pause, or hand off the active callback.

Important current semantics:

- The first tick always begins in brake.
- The callback's returned command is applied on the next tick boundary, not retroactively to the current tick.
- `LastDiagnostics()` exposes the last published completed tick from the loop's two-slot timing buffer.
- `SharedRobotRuntime` owns the actual log servicing and fault path; `LoopController` just schedules the control loop around it.

Current practical constraints of the implementation:

- `controlPeriodUs` is required.
- The implementation currently supports the full encoder + IMU + estimator update path.
- `useWallUpdates=false` is the only meaningful sensor-plan reduction currently exposed for new mode authors.
- Tick capture currently comes through the diagnostic sensor path, so new modes should assume `diagnosticSensors` is the primary per-tick structured sensor surface.

Preferred control style for new modes:

- `ControlVector::OpenLoopCommand()` is highly preferred.
- Use raw open-loop drive commands unless there is a concrete reason the mode must use closed-loop target velocity.
- `VelocityCommand()` exists, but it should be the exception, not the default.

Preferred callback shape:

- Keep phase logic in small callback functions.
- Prefer `TickServices::SetNextModeWorkCallback()` to hand off between phase-local handlers instead of building a giant monolithic tick switch.
- Use `TickServices::RequestEndLoop()` for normal completion.
- Use `TickServices::Fault()` for terminal failures that should route to runtime fault handling.
- Use `TickServices::RequestPause()` when the mode must do unbounded work such as:
  - flushing or closing an `mmlog`
  - reopening the logger with a new schema
  - exporting persisted artifacts
  - running other non-real-time cleanup or setup

Pause behavior matters:

- Pause settlement brakes the robot and continues ticking until measured motion is below threshold.
- Default pause thresholds are `0.01 m/s` linear and `0.05 rad/s` angular for `2` consecutive settled ticks.
- The pause callback can resume, complete, or stop by runtime fault.
- If the pause needs a clean logging state before it runs, set `flushLogsBeforeGrant=true`.

Use loop-owned timing instead of rebuilding your own timing capture:

- If the mode needs precise tick timing, read `LastDiagnostics()`.
- Do not add another mode-local timing log format when `LoopController` already provides the required timestamps.

## File Requirements And Repo Rules

The file-level rules from `AGENTS.md` apply directly to mode work:

- A substantive mode class lives in same-named authoritative files.
- An edited non-template `.cpp` includes `pch.h` first and then its own header first.
- Headers must be self-sufficient.
- Do not rely on incidental transitive includes.
- Do not add forwarding headers or alias headers.
- Do not create a new public `Params`, `State`, `Context`, `Data`, or similar companion bag just to make the mode easier to wire.
- Keep mode-local helpers private, nested, or file-local.
- Reuse typed shared config owners instead of cloning a config namespace or constant block.
- Put shared limits in `SoftwareLimits` or another existing authoritative typed owner, not in a new mode-local constant dump.

For new mode structure specifically:

- Put the authoritative descriptor in the mode implementation or header.
- Put the selector rule only in `BootModeRegistry`.
- Do not duplicate selector conditions in comments, config namespaces, or text-log trivia.
- Do not add runtime switching between top-level modes.
- Do not add a per-mode host interface for a new utility mode.
- Do not create wrappers around `SharedRobotRuntime`, `LoopController`, or the pathfinders just to reduce edits.

## Post-Addition Audit

Before considering a new mode done, audit it against this list.

### Registry and Descriptor

- The mode has one `BootModeRegistry` entry.
- The entry contains only selection/discovery metadata.
- The descriptor is authoritative and accurate.
- A reviewer can find the selector condition in the registry and the behavior summary in the descriptor without chasing duplicate comments.

### Runtime Ownership

- The mode borrows all shared runtime resources from `SharedRobotRuntime`.
- The mode does not create another `MmLogLogger`, `logging.txt`, `Maze`, `FloodFill`, or `ManeuverPathfinder`.
- The mode does not add a parallel file-service loop.

### Loop Ownership

- The mode uses `LoopController`.
- The mode does not introduce another control-period timing helper.
- The mode uses `RequestPause()` for non-real-time phase work instead of doing unbounded file work inside the hot path.
- The command path is raw open-loop unless there is a documented reason otherwise.

### Logging

- `logging.txt` is sparse and human-readable.
- High-rate data is in `mmlog`, not text.
- All `mmlog` metadata is written before `BeginUtilityDataLogSchema()`.
- The row schema is `<= 512` bytes.
- Metadata and path lengths stay within the compile-time limits.
- Success and failure paths both leave the runtime-owned logger in a coherent state.
- The mode uses `LastRuntimeLogError()` / `CaptureUtilityDataLogFailure()` instead of inventing another error-reporting subsystem.

### Repo Rules

- No copied config namespaces.
- No public peeled-off internal bags.
- No wrapper classes that mostly forward.
- No hidden selector metadata outside `BootModeRegistry`.
- No duplicate top-level mode entry points.
- No include-order accidents.

### Verification

- Add or update unit tests for any new selection logic, descriptor wiring, or mode-local helper logic that can be host-tested.
- Verify behavior through the canonical path, not through a compatibility wrapper.
- Run the repository's latest-build verification path after code changes.
- Before testing, confirm the active binaries already correspond to the latest sources; do not rebuild from scratch unless a specific problem requires it.
- When building or verifying, use `codex_verify/build_and_verify_latest.cmd` or `codex_verify/build_and_verify_latest.ps1`.
- If `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately and do not try to recover with `Clean`, `Rebuild`, or more artifact deletion.

## Current Best Starting Point

For a new direct utility mode, start by reading:

- `MazeMap/MazeMap/TopSpeedMeasurementMode.h`
- `MazeMap/MazeMap/TopSpeedMeasurementMode.cpp`
- `MazeMap/MazeMap/LoopController.h`
- `MazeMap/MazeMap/LoopController.cpp`
- `MazeMap/MazeMap/MazeMapSharedRuntime.h`
- `MazeMap/MazeMap/MazeMapSharedRuntime.cpp`
- `MazeMap/MazeMap/BootModeRegistry.h`
- `MazeMap/MazeMap/BootModeRegistry.cpp`

Use those files as the current canonical reference, and use this guide as the checklist that keeps a new mode from sliding back into duplicate ownership or legacy wrapper patterns.
