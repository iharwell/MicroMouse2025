# WallTouch

## Fundamental rule

Do not expand this class into a replacement control surface while cleaning up `LoopController`.

## Current dependency

`WallTouch` participates as a shared service and does not appear to rely on the invalid public `LoopController` capabilities that drove the wider problem.

## Final-form changes required

No direct restructuring is required here unless final LoopController cleanup removes a private dependency that this class still reaches into.

## Why minimal edits are toxic

Routing removed loop-control behavior through `WallTouch` would only create a new misuse site.

## Dependencies and risks

Keep this file stable unless a real remaining private LoopController dependency proves otherwise.
