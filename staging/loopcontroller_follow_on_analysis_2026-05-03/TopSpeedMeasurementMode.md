# TopSpeedMeasurementMode

## Fundamental rule

Rewrite the experiment control flow directly to the final narrower LoopController contract. Do not preserve the old helper model behind a small patch.

## Current dependency

The tick path faults through `services.Fault(...)`, and experiment completion goes through `services.RequestEndLoop()`.

## Final-form changes required

- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Keep experiment completion as terminal loop/program end semantics, but through the final direct terminal request rather than `TickServices`.
- Keep the experiment sequencing, but remove dependence on `TickServices`.

## Why minimal edits are toxic

This mode is simple enough that a superficial migration would be tempting. That would only freeze the poisoned caller model in one more place.

## Dependencies and risks

Selector-loss handling and experiment completion logging must stay intact while the direct loop-control calls change.
