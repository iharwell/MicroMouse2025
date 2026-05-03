# PositionAccuracyAuditMode

## Fundamental rule

Rewrite this mode directly to the final narrower `LoopController` contract. Do not preserve its phase-launch structure behind compatibility helpers.

## Current dependency

- `Run()` manually owns the session lifecycle.
- Phase-transition failures depend on `services.Fault(...)`.
- Final completion depends on `services.RequestEndLoop()`.

## Final-form changes required

- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Keep final completion as terminal loop/program end semantics, but through the final direct loop-owned request rather than `TickServices`.
- Rewrite the callback API usage so the mode uses direct `LoopController` methods instead of `TickServices`.

## Why minimal edits are toxic

Minimal edits would let the phase-launch sites keep speaking the same old loop mutation language through wrappers.

## Dependencies and risks

This file composes several shared services, so the direct callback/control surface has to be clear before the mode can be rewritten cleanly.
