## DriveBase command-surface note

`DriveBase::FeedbackCommand(...)` is acting like an internal composition helper, not a sanctioned external command path.

Callers that want DriveBase-owned feedforward plus feedback should go through the public command entry points instead:

- `PointCommand(...)`
- `PointYawRateCommand(...)`
- `PointCommandWithHeadingTarget(...)`
- `DeltaCommand(...)`

The oscillation tests were updated to use only those public pathways. No production visibility change was made in this patch; this note is here so the API cleanup is explicit.
