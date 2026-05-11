# Worker G / Boyle Crash Handoff

Scope assigned:

- Remove rejected Estimator convenience APIs.
- Correct `EstimatorTestSupport.h` away from removed DriveBase measurement APIs.
- Remove `DriveBase` access to PlantModel private params/friendship.
- Shift PlantModel boundary toward bound-state domain operations with DriveBase passing requests only.

Status: crashed during compaction. Partial edits exist in the working tree and must be reviewed as untrusted.

## Intended Corrections

- Remove public Estimator convenience APIs:
  - `SetStartPoint(...)`
  - `SetPose(...)`
  - `SetPoseXMeters(...)`
  - `SetPoseYMeters(...)`
- Keep semantic estimator operations:
  - `ResetPose(...)`
  - `ResetForSessionTransition(...)`
  - `RestoreSessionStartPhysicalState(...)`
  - `SetGyroBiasZ(...)`
  - predict/update APIs
- Replace convenience API call sites with explicit semantic `ResetPose(...)` calls or report blockers.
- Remove `DriveBase` friendship with `PlantModel`.
- Remove all `DriveBase` references to `PlantParams`, `PlantPreparedParams`, `_preparedParams`, and `Prepare`.
- Add PlantModel domain operations that use PlantModel-owned internal bound facts/state, not getters or exposed parameter bags.

## Partial State Observed After Crash

- `Estimator` convenience API text appeared to be removed in a narrow search.
- `DriveBase` no longer had direct `PlantParams` / `PlantPreparedParams` / `_preparedParams` / `Prepare` hits in a narrow search.
- `PlantModel::AttachRuntimeState(...)` and bound-state methods appeared in the diff.
- `SharedRobotRuntime` appeared to call `plantModel.AttachRuntimeState(runtimeState)`.
- `DriveBase` still had stale state-vector construction in `ResolveRawAccelerationCommand`; Reviewer Lovelace found it and Worker H later removed it.

## Required Follow-Up

Treat all Worker G edits as partial and review:

- `Estimator.h` / `Estimator.cpp`
- `PlantModel.h` / `PlantModel.cpp`
- `DriveBase.h` / `DriveBase.cpp`
- caller files touched by Worker F/G

