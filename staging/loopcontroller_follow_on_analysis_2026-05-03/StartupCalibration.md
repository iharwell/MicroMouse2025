# StartupCalibration

## Fundamental rule

Fix real private-state coupling directly. Do not preserve it behind smaller shims.

## Current dependency

`StartupCalibration` does not depend on `TickServices` or the bad pause surface, but it does reach into LoopController private state for control-period data.

## Final-form changes required

- If the final LoopController cleanup removes that private reach-in, switch directly to an explicit public LoopController accessor.
- Do not create a wrapper object just to preserve private-state dependence indirectly.

## Why minimal edits are toxic

A shim that still feeds this class hidden LoopController internals would keep the same ownership violation alive.

## Dependencies and risks

This is a secondary site compared with the heavy `TickServices` and pause consumers.
