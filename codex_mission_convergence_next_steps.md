# Mission Convergence Next Steps

This note records the next convergent cuts after the `LoopController` callback-plus-context handoff seam.

The target shape is now explicit:

- one boot-selected top-level mode owns one `LoopController` session for the life of that mode,
- reusable loop-driven infrastructure fixtures run inside that one active session,
- a fixture returns control by installing a caller-supplied return callback and return context,
- top-level modes keep policy and phase decisions, not shared drive or mapping execution.

## Current State

- `LoopController` now supports handing off to the next callback with both function and context through `SetNextModeWorkCallbacks(...)`.
- `MissionModeController` phase transitions now use that full callback handoff path.
- This is only the enabling seam. The actual shared fixtures do not exist yet.
- `MissionModeController` still owns nested `RunLoopSession(...)` launches and still owns shared execution logic that should move out.
- `ManeuverQueue` execution, simple motion primitives, and mapping/search execution are still mission-owned.
- The latest full Release verification reached the unit-test phase without reporting `HOST_INTERMEDIATE_STATE_BROKEN`.
- The latest verification then failed on the existing Release test `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists`.
- Latest log path: `C:\Users\thene\source\repos\MicroMouse2025\codex_verify\logs\build_and_verify_latest_20260416_232637_008.txt`

## Architectural Target

- The selected top-level mode owner is the only code that may call `LoopController::BeginSession(...)`, `Run()`, and `EndSession()`.
- A reusable fixture may install its own internal callback phases, but it may not start or end a session.
- A fixture must accept a caller-supplied continuation using `LoopController::ModeCallbacks`.
- A fixture must return control by installing that continuation when its work is complete.
- Do not introduce another wrapper registry, fixture manager, or generic forwarding facade around this contract unless a concrete shared need appears.
- Shared execution fixtures belong with shared runtime and drive/navigation ownership, not in mission policy code.

## Convergent Order

### 1. Treat `LoopController::ModeCallbacks` as the fixture continuation contract

- Use `LoopController::ModeCallbacks` directly as the "return control here" contract.
- A fixture should accept:
  - caller-owned fixture state/input,
  - one completion continuation,
  - any explicit result storage the caller needs.
- Do not add a second public continuation wrapper unless `ModeCallbacks` proves materially insufficient.

### 2. Extract one authoritative shared `ManeuverQueue` execution fixture

This should be the first real shared fixture.

Destination:

- one shared drive-execution owner rooted with shared runtime / drive execution ownership,
- not `MissionModeController`,
- not a mode-local helper family.

Move out of mission ownership:

- `ExecuteQueuedManeuvers`
- `ExecuteStraightProfile`
- `ExecuteTurnProfile`
- `ExecuteArcProfile`
- `ExecuteSmoothTurnProfile`
- `HoldPosition`
- `HoldBrakedUntilDriveSettles`
- `HoldZeroVelocityUntilDriveSettles`
- the loop-state structs and loop callbacks required by that execution path

Required shape:

- consumes a caller-supplied `ManeuverQueue`,
- uses `ManeuverInstance` as the execution vocabulary,
- uses `ManeuverPoint` for tracked maneuver progression,
- accepts a completion continuation as `LoopController::ModeCallbacks`,
- returns control by installing that continuation instead of ending the session,
- keeps mode-specific telemetry/logging choices outside the shared fixture unless that logging is truly shared infrastructure.

### 3. Move `ManeuverFileTestMode` onto the shared queue fixture

This should be the first caller migrated to the shared fixture.

Required cut:

- `ManeuverFileTestMode` owns the one active `LoopController` session,
- maneuver test setup still happens in the mode owner or directly owned mode-local setup code,
- the mode invokes the shared `ManeuverQueue` fixture inside that session,
- when queue execution completes, control returns to a mode-owned continuation callback,
- the mode-owned continuation performs final hold / shutdown / log completion,
- no nested `RunLoopSession(...)` launches remain in that workflow.

### 4. Extract the shared maze mapping / search fixture

After the queue fixture lands, extract the mapping/search side as another reusable fixture.

Move out of mission ownership:

- observation capture used by search mapping,
- loop-driven straight search execution,
- rolling observation stop / replan-stop handling,
- map-update / fusion work that currently rides inside the mission search executor,
- queue construction that belongs to shared search execution rather than mission policy

Required shape:

- runs inside the caller's active session,
- returns control through a caller-supplied continuation,
- reports completion or replan-needed state through caller-owned result storage,
- leaves goal choice, retry policy, and mission progression decisions in mission mode.

### 5. Convert mission run mode into a callback-owned phase graph over fixtures

Mission mode should become policy over shared fixtures.

Target shape:

- `MissionStartupCallback`
- mapping fixture launch callback
- goal / replan policy callback
- return-to-start callback
- race-queue launch callback
- inter-run service callback
- mission-complete callback

Mission mode should decide:

- what the next mission objective is,
- when to explore,
- when to replan,
- when to return to start,
- when to launch a speed run,
- when mission success or failure has occurred.

Mission mode should not own:

- the actual queue executor,
- shared straight / turn / arc / tracked-maneuver execution,
- shared mapping execution,
- nested loop-session ownership.

### 6. Delete subordinate session launch paths as each fixture migrates

- Once a workflow is converted to a fixture running inside the caller's active session, delete the old nested `RunLoopSession(...)` path in that same change.
- Do not preserve both the fixture path and the nested-session path side by side.
- Keep the migration convergent: one canonical callback-driven execution path per converted behavior.

### 7. Keep pause behavior narrow

As fixtures are introduced:

- no waits for control progress outside `RequestPause(...)` callbacks,
- no generic pause dispatch hub,
- each pause callback stays local to the fixture or mode phase that requested it,
- pauses exist only for operator input, settling, or non-real-time work.

## Verification Steps After Each Cut

1. Check timestamps to confirm edited sources are newer than active host binaries.
2. Run the smallest relevant Release unit coverage first if available.
3. Run `build_and_verify_latest.cmd --no-pause`.
4. If the script reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately.
5. Record the blocking log path in the work note before continuing later.
6. If verification reaches unit tests, record whether any failure is new or matches a known pre-existing failing test.

## Done When

This convergence stage is not done when the code merely compiles.

It is done when:

- reusable execution fixtures run inside a caller-owned active session,
- those fixtures return control through caller-supplied `LoopController::ModeCallbacks`,
- `ManeuverQueue` execution is no longer mission-owned,
- `ManeuverFileTestMode` uses the shared queue fixture and owns its one top-level session,
- mapping/search execution is exposed as a reusable fixture instead of mission-local execution,
- mission run mode reads as policy over fixtures rather than as a drive executor,
- no converted workflow still relies on nested `RunLoopSession(...)` launches.
