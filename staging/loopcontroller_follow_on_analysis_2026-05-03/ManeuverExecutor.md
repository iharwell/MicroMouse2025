# ManeuverExecutor

## Fundamental rule

Keep legitimate callback-transfer behavior, but remove invalid loop fault/completion behavior outright rather than wrapping it.

## Current dependency

`ManeuverExecutor` currently:

- transfers callback ownership through `services.SetNextModeWorkCallbacks(...)`
- faults through `services.Fault(...)`
- ends the loop through `services.RequestEndLoop()` when there is no continuation

## Final-form changes required

- Move callback-transfer behavior onto the final direct LoopController continuation surface.
- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Remove the no-continuation terminal fallback entirely. A routine with no continuation is an invariant violation, not a valid request for terminal loop/program end.

## Why minimal edits are toxic

This class is a shared routine owner. If it keeps the old helper-based loop policy behind a wrapper, that poisoned contract will keep spreading to every mode that uses it.

## Dependencies and risks

This is a central migration site because many modes depend on it for callback ownership handoff.
