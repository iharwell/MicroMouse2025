# RuntimeSensorSuite

## Fundamental rule

Sensor work staying wound into `LoopController` is acceptable for this pass, so do not use this file as an extraction target.

## Current dependency

`RuntimeSensorSuite` is used by `LoopController`, but the comment set driving this cleanup does not identify it as a destination for removed public loop capabilities.

## Final-form changes required

No direct change is recommended here for this cleanup.

## Why minimal edits are toxic

Externalizing sensor orchestration here would create a new architecture move that is not required to solve the commented problems.

## Dependencies and risks

Leave this file stable unless a future task separately targets the sensing boundary.
