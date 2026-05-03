# MissionRunMode

## Fundamental rule

Do not preserve the old `TickServices` model behind local adapters. Rewrite mission control flow directly to the final LoopController contract.

## Current dependency

`Run()` manually sequences `BeginSession(...)`, `Run()`, and `EndSession()`, while the tick path currently uses `services.Fault(...)` and `services.RequestEndLoop()`.

## Final-form changes required

- Remove mode-local ownership of the terminal `LoopController::Run()` boundary.
- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Replace `services.RequestEndLoop()` with the direct terminal loop-end request, conceptually `HaltExecutionEndProgram`.
- Update the callback path to use direct `LoopController` methods rather than `TickServices`.

## Why minimal edits are toxic

Mission mode is structurally simple. Keeping the old helper-driven fault/completion model alive here would make the repository look cleaned up while preserving the same bad contract.

## Dependencies and risks

This is a good early migration site once the final direct callback surface and infrastructure-owned `Run()` boundary are explicit.
