# Codex UKF Schedule Search Work Log

- Date: 2026-04-16
- Goal: Continue the interrupted UKF parameter sweep work, preserve prior sandbox progress, and converge on a better replay-tuned UKF setup for the open-floor runs.
- Constraint: Do not seed the replay UKF from the logged run state; the tested runs begin from a known stationary start.

## Progress

- Confirmed the main worktree is already dirty in unrelated files:
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\DriveBase.cpp`
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\DriveBase.h`
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\SrUkfCore.cpp`
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\SrUkfCore.h`
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\DriveBaseTest.cpp`
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\SrUkfCoreModeAndDiagnosticsTest.cpp`
  - `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMapTest\SrUkfCoreMotionUpdateTest.cpp`
- Found the interrupted sandbox copy under:
  - `C:\Users\thene\source\repos\MicroMouse2025\.tmp\ukf_schedule_search\workspace`
- Recovered the key sandbox-only changes that appear to belong to this task:
  - Runtime-tunable `SrUkfCore` parameters.
  - `OpenFloorUkfReplay` support for `--tuning <file>`.
  - `OpenFloorUkfReplay` support for `--known-stationary-seed`.
  - `OpenFloorUkfReplay` aggregate JSON metrics output for machine scoring.
  - `VehicleState` stationary thresholds wired to runtime tuning instead of frozen constants.
- Confirmed the sandbox copy did not include a completed sweep script or completed sweep output files, so the search runner still needs to be finished in the main tree.

## Next

- Port the recovered UKF replay tuning changes into the main tree without disturbing unrelated dirty files.
- Add the missing sweep driver and scoring path.
- Verify release binaries are current before running tests or replay.
- Run the sweep with the known stationary seed path and capture the best result.
