# CorridorRepeatabilityMode

## Fundamental rule

Rewrite the mode directly to the final narrower LoopController contract. Do not preserve a transitional wrapper.

## Current dependency

The tick path faults through `services.Fault(...)`, and sequence completion goes through `services.RequestEndLoop()`.

## Final-form changes required

- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Keep sequence completion as terminal loop/program end semantics, but through the final direct loop-owned request rather than `TickServices`.
- Keep repeatability sequencing, but remove dependence on `TickServices`.

## Why minimal edits are toxic

This mode is another simple sequencer that could look “fixed” after a local adapter pass while still carrying the same bad caller assumptions.

## Dependencies and risks

It composes `StartupCalibration` and `WallTouch`, so their remaining private-loop dependencies need to stay compatible while the mode is migrated.
