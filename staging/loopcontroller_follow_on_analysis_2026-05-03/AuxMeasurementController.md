# AuxMeasurementController

## Fundamental rule

Rewrite this controller directly to the final narrower `LoopController` contract. Do not preserve its current control flow behind wrappers or renamed helpers.

## Current dependency

- `Run()` open-codes the session lifecycle.
- The controller legitimately reads loop timing for logging and integration.
- The tick path depends on `services.Fault(...)` and `services.RequestEndLoop()`.

## Final-form changes required

- Keep the existing timing reads.
- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Classify each `RequestEndLoop()` site correctly:
  - terminal measurement-mode completion stays terminal loop/program end semantics
  - any future session discontinuity must use `RequestEndSession` plus `StageNextSessionState`, not terminal loop end
- Rewrite the callback path to use direct `LoopController` methods instead of `TickServices`.

## Why minimal edits are toxic

Minimal edits would keep the same measurement-state machine talking to the loop through a thinner helper surface. That preserves the wrong architecture.

## Dependencies and risks

Timing-sensitive logging must remain intact while the invalid control helpers are removed.
