# LoopController Migration Review Findings

## 2026-04-13

Review pass: migration conformance against repository ownership and cleanup rules.

### Finding 1

- Priority: P2
- File: `MazeMap/MazeMap/LoopController.h`
- Lines: `132-214`
- Title: Shrink the inert `LoopController` public surface
- Details:
  `LoopController` still publishes a much broader runtime/config contract than the implementation actually honors.
  `RuntimeBundle` exposes three logger pointers that are never consumed, and `SessionConfig` advertises descriptor/session identity, dynamic capture, guard policy, instrumentation, and derived-kinematics knobs that the loop core does not use.
  That leaves the canonical owner with public migration scaffolding and inactive switches, which conflicts with the repo rule to keep one authoritative public owner and avoid support types that mostly expose unfinished internal pipeline options.

### Finding 2

- Priority: P2
- File: `MazeMap/MazeMap/OpenFloorMeasurementController.cpp`
- Lines: `1408-1417`
- Title: Open-floor still owns part of loop timing
- Details:
  The migration moved fixed-period timing into `LoopController`, but open-floor still stamps `pwmLatchUs`, `controlEndUs`, and `cycleCounterEnd` locally right before logging.
  That keeps timing ownership split between the mode and the shared loop owner, which is exactly the kind of parallel authority the repo spec is trying to eliminate.
  This controller should ideally consume the loop-owned timing data and add only mode-local labels or artifacts.

### Finding 3

- Priority: P3
- File: `MazeMap/MazeMap/LoopController.cpp`
- Lines: `759-770`
- Title: Remove or finish the dead capture helper
- Details:
  `CaptureTickStateWithResolvedSensors` is still present as a private helper that only suppresses warnings and returns `false`, and the adjacent `SupportsCaptureOptions` comment explicitly says selective capture has not been landed yet.
  Under this repo's cleanup policy, dead migration scaffolding should not stay side-by-side with the canonical path.
  Either wire this helper into the real capture flow or delete it until selective capture support is actually implemented.
