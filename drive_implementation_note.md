## Drive implementation note

This implementation treats `Drive` as an independent shared runtime service, not as a port of `ManeuverExecutor`.
`Drive` should now be treated as the exemplar template of a solid multi-tick service in this codebase.

Key assumptions confirmed in discussion:

- Owner-level `MotionLimits`, `OperationMode`, and `CommandPDSettings` stay live on `Drive`. They are not latched per primitive.
- The active primitive spec is copied into `Drive` because `Drive` is the canonical execution owner, but storage stays union-based and compact.
- Do not widen stored spec fields.
- Reverse straight is redundant with signed linear velocity. `StartReverseStraight(...)` arms the same linear-motion branch as `StartStraight(...)`, with negative commanded speed.
- Maneuver execution uses encoder progress plus yaw-rate targets only. It does not depend on `ManeuverPoint.X`, `Y`, or `Theta`.
- `done` from `GetNextControls(...)` is advisory. Ignoring it must not make `Drive` incoherent or implicitly revoke the last explicit instruction the mode gave it.
- `Drive` is deferent to mode logic: the mode may return Drive's proposed output, override it, ignore it, or stop polling it without transferring lifecycle ownership to the service.

Internal branch storage target:

- `Hold`: `uint16_t requestedTicks`, `uint16_t remainingTicks`, `bool resetOnNonStationary`
- `LinearMotion`: distance, cruise speed, exit speed, captured target yaw, optional projected target distance, start distance, commanded speed
- `Turn`: target yaw, optional `TurnWallEdgeTracker*`
- `TurnTransition`: distance, `dCurvatureDs`, initial speed, initial yaw rate, start distance
- `Arc`: distance, curvature, initial speed, start distance
- `Maneuver`: `ManeuverInstance`, start distance

Control intent:

- `Hold` only completes on stationary ticks.
- `LinearMotion` uses signed linear velocity with heading hold; optional projected target distance overrides encoder-only completion.
- `Turn` uses in-place yaw-rate command shaping from `InPlaceTurnProfile`.
- `TurnTransition` holds the armed entry speed and advances yaw rate from encoder progress using `r = r0 + v0 * dCurvatureDs * s`.
- `Arc` holds the armed entry speed and commands `omega = v0 * curvature`.
- `Maneuver` samples `ManeuverInstance::TryGetManeuverPoint(...)` by encoder progress and feeds only `point.Velocity` and `point.Omega` into `DriveBase`.

Runtime ownership:

- `SharedRobotRuntime::Drive()` remains the canonical `DriveBase` accessor for existing callers.
- New higher-level access is exposed separately as `SharedRobotRuntime::DriveService()`.
