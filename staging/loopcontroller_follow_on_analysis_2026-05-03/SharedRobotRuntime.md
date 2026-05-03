# SharedRobotRuntime

## Fundamental rule

Only use `SharedRobotRuntime` where it is already the existing direct destination. Do not widen it to absorb more LoopController behavior.

## Current dependency

`SharedRobotRuntime` already exposes `FailActiveMode(...)`, which is the existing direct fault destination callers should use once `services.Fault(...)` disappears.

## Final-form changes required

- Keep `FailActiveMode(...)` as the direct mode-fault destination.
- Update callers to use it directly where they currently rely on LoopController fault helpers.
- Do not externalize additional loop behavior here.

## Why minimal edits are toxic

Adding new runtime helper paths for pause or session completion would only relocate the bad contract.

## Dependencies and risks

Most of the work lands in consumers, not in this class.
