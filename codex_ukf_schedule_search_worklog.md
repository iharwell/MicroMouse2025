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
- Confirmed the substantive UKF replay/tuning work was already committed on `codex/drivebase-mission-control-rewrite`.
- Committed the remaining dirty follow-up fix on that branch:
  - `1befe3f Fix DriveBase correction command operating-state inputs`
- Detected an accidental rescue-branch commit that tracked build artifacts and the nested `.codex-main-merge` worktree pointer:
  - `ef30840 Add UKF sweep recovery log and build outputs`
- Cleaned that accidental rescue-branch commit non-destructively with a follow-up commit that removed those tracked artifact paths from the index:
  - `788df07 Remove accidental UKF sweep artifact commit contents`
- Fast-forwarded `main` to include the real UKF/DriveBase branch tip:
  - `main: 1fbf1be -> 1befe3f`
- Added the follow-up main-branch log update:
  - `028b8f8 Update UKF sweep recovery log after main merge`
- Ran the required release verification entry point from merged `main`:
  - `codex_verify\build_and_verify_latest.cmd --no-pause`
- Verification stopped immediately with the repo-mandated blocker:
  - `HOST_INTERMEDIATE_STATE_BROKEN`
  - Missing or damaged host Release intermediates were reported for `MazeMapTest` and `MazeSimulation`.
  - Per repo instructions, no `Clean`, `Rebuild`, or further artifact deletion was attempted.
- Preserved the root-worktree-only obstacle files under the ignored backup directory:
  - `C:\Users\thene\source\repos\MicroMouse2025\.tmp\root_to_main_cleanup_backup_20260416_194602`
- Removed the redundant `.codex-main-merge` worktree so the root checkout could hold `main`.
- Switched the root checkout itself onto `main`.
- Confirmed the root worktree is now:
  - branch: `main`
  - commit: `028b8f8`
  - status: clean

## Next

- Human intervention is required to repair or intentionally recreate the missing host Release intermediates before release verification or replay can continue.
- After the host incremental-build state is repaired, rerun release verification from merged `main`.
- Once verification is unblocked, run the stationary-seed UKF sweep from merged `main` and capture the tuned result.
