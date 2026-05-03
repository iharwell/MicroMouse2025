# BootModeRegistry

## Fundamental rule

Do not move loop semantics into boot-mode metadata.

## Current dependency

`BootModeRegistry` is passive selector/descriptor metadata and is not one of the misuse sites that grew around invalid `LoopController` capabilities.

## Final-form changes required

No direct structural change is required here for this cleanup.

## Why minimal edits are toxic

Using the registry to coordinate loop/session cleanup would externalize the problem into the wrong owner instead of fixing the callers.

## Dependencies and risks

Keep this file out of scope unless a later task explicitly changes boot metadata itself.
