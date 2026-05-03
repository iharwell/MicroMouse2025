# MazeMapRuntimeInfrastructure

## Fundamental rule

Read-only timing consumers are not the problem. Do not widen them into new control surfaces.

## Current dependency

This helper reads timing data from `LoopController` and does not use the invalid public pause/fault/completion features.

## Final-form changes required

No direct change is required here for the comment-driven cleanup.

## Why minimal edits are toxic

There is no value in teaching this helper to proxy new loop abstractions; that would only spread the surface area further.

## Dependencies and risks

Low risk unless the legitimate timing getter shape changes.
