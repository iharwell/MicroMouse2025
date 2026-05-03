# Drive

## Fundamental rule

Do not use `Drive` as an alternate home for loop-control behavior just because it already participates in per-tick execution.

## Current dependency

`Drive` consumes legitimate loop timing and returns `LoopController::ControlVector`, but it does not depend on the invalid public control capabilities called out in the comments.

## Final-form changes required

No direct restructuring is required here from the comment set alone.

## Why minimal edits are toxic

Using `Drive` to hide removed LoopController behavior would preserve the same control assumptions under a different name.

## Dependencies and risks

Revisit this class only if the final direct LoopController timing/control surface changes materially.
