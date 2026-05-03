# FrontWallCharacterizationController

## Fundamental rule

Rebuild this controller directly to the final pause/fault contract. Do not preserve the current structure behind compatibility helpers.

## Current dependency

- `Run()` duplicates the session lifecycle.
- The controller depends on the rejected pause surface: `PauseRequest` policy fields, `services.RequestPause(...)`, and `PauseDisposition::StopByRuntime(...)` / `Resume()`.
- It pauses to persist/export, then resumes the same measurement flow afterward.
- It also faults through `services.Fault(...)`.

## Final-form changes required

- Keep this as a true pause case because continuity is preserved across the persist/export work.
- Rebuild the pause-time export handoff around the narrower direct `LoopController` pause contract.
- Remove `PauseDisposition::StopByRuntime(...)` and caller-selected pause policy fields.
- Replace failure returns and `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Keep session control on direct `LoopController` methods instead of `TickServices`.

## Why minimal edits are toxic

This controller is pause-heavy. Any wrapper-based cleanup would leave the same invalid pause assumptions alive under different names.

## Dependencies and risks

The artifact-generation path depends on a real stationary handoff, so the pause rewrite must be real, not cosmetic.
