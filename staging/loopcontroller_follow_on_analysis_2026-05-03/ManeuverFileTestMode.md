# ManeuverFileTestMode

## Fundamental rule

Rewrite the scripted-test control flow directly to the final narrower LoopController contract. Do not keep the old model behind a local adapter.

## Current dependency

The tick path faults through `services.Fault(...)`, and scripted completion uses `services.RequestEndLoop()`.

## Final-form changes required

- Replace `services.Fault(...)` with direct `SharedRobotRuntime::FailActiveMode(...)`.
- Keep scripted test completion as terminal loop/program end semantics, but through the final direct loop-owned request rather than `TickServices`.
- Keep scripted test progression, but remove dependence on `TickServices`.

## Why minimal edits are toxic

This mode would be easy to “clean up” superficially, which is exactly why it is dangerous. A thin rewrite would leave the poisoned loop-control model intact.

## Dependencies and risks

It depends on `ManeuverExecutor`, so that continuation cleanup needs to happen in step.
