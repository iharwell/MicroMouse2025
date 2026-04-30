# Project Vocabulary

This document defines preferred project terminology for code, design notes, reviews, and future cleanup work.

The repository is still in architectural cleanup, so some legacy names remain. When new code or documentation needs a word, prefer the meanings in this glossary over older wrapper-era or ad hoc usage.

## Core Distinctions

- `Mode` means a boot-selected top-level application mode.
- A mode is selected once at startup from hardware selector state, typically a jumper strap on a pin pair, and is not changed without reboot.
- A mode is the top-level owner of `IApplicationMode::Begin()` / `Run()` and the active `LoopController` session for that boot.
- A boot cycle normally gets one `LoopController` session.
- A second `LoopController` session is allowed only when the vehicle state is known to be discontinuous, such as after user service or a physical lift that invalidates UKF continuity.
- `Routine` means a public high-level procedure or workflow concept.
- Modes may compose, launch, or sequence routines, but routines must never depend on Modes. Any routine-level dependency on a Mode is a strictly unacceptable architecture violation.
- A routine may span many ticks, but once launched it owns the active `LoopController` callback until completion and therefore does not span phases.
- A routine that does not explicitly pause `LoopController` must return control before the active tick deadline.
- If `LoopController` is paused, the robot must not move.
- `Routine` should not be used to hide what are really separate boot-selected Modes.
- `Phase` means a dynamic execution block inside a mode.
- Phases are useful for reasoning about mode internals and for recording transitions, but infrastructure should not need to understand phase-specific semantics.
- `section` is a reporting or workflow subdivision inside one mode or one routine. Neither phases nor sections are modes.

## Cross-Cutting Naming Terms

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `policy` | A small stateless rule or criteria helper. Use `policy` for logic that decides whether something is valid, allowed, ready, armed, complete, or eligible. A policy is not a controller, not a routine, and not an owner of substantive state. | `MazeMap/MazeMap/EncoderStallPolicy.h`, `MazeMap/MazeMap/GyroBiasUpdatePolicy.h`, `MazeMap/MazeMap/ImuCalibrationPolicy.h` |
| `physical coordinates` / `physical location` | The continuous-space position vocabulary on the robot side, typically in meters, used when ambiguity with maze-grid coordinates is possible. If the context is already clearly on the physical side, `coordinates` or `location` is sufficient. | `MazeMap/MazeMap/DriveBase.h`, `MazeMap/MazeMap/MotionTargetProjection.h`, `MazeMap/MazeMap/CellCoordinates.h`, `MazeMap/MazeMap/MazeLocation.h` |

## Top-Level Control Terms

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `mode` / `boot mode` | A top-level startup-selected application mode. It is chosen through `BootModeRegistry`, remains fixed for the session, normally owns the one `LoopController` session for that boot cycle, and is the primary owner of the active `LoopController` callback chain. A second session is only for known state discontinuity such as user service or a physical lift. | `MazeMap/MazeMap/BootModeRegistry.h`, `MazeMap/MazeMap/BootModeDescriptor.h`, `MazeMap/MazeMap/IApplicationMode.h`, `MazeMap/MazeMap/LoopController.h` |
| `mission mode` | A boot-selected mode whose scenario is maze operation such as exploration, return, and speed run. This is a scenario label, not a separate architectural class, and the mission mode may be smaller than many test modes because only tightly verified behavior should live there. | `MazeMap/MazeMap/MissionRunMode.cpp`, `AGENTS.md` |
| `utility mode` | A boot-selected mode whose scenario is measurement, audit, characterization, calibration, or bring-up. This is also a scenario label, not a lighter-weight architectural class or a different mode model. | `MazeMap/MazeMap/BootModeDescriptor.h`, `AGENTS.md`, `boot_mode_setup_guide.md` |
| `routine` | A public high-level procedure or workflow concept that lives inside a mode or shared execution owner and advances through `LoopController` callbacks. A routine is a reusable behavioral block used while constructing a mode phase; once launched it owns the callback stream until completion. Modes may use routines, but routines must never depend on Modes; if they do, that is a strictly unacceptable architecture violation. It is not a synchronous/blocking wrapper term, and it is not the right term for hidden compile-time sub-mode selection. | `MazeMap/MazeMap/AuxMeasurementConfig.h`, `MazeMap/MazeMap/AuxMeasurementController.cpp`, `MazeMap/MazeMap/ManeuverExecutor.h`, `execution_model_guide.md` |
| `ManeuverExecutorRoutine` | The high-performance racing routine that attempts to execute the maneuvers queued in a `ManeuverQueue` as aggressively as possible. This is the routine that should push the hardware the hardest once installed, subject to any explicit dial-back parameterization. | `MazeMap/MazeMap/ManeuverExecutor.h`, `MazeMap/MazeMap/ManeuverQueue.h` |
| `MappingRoutine` | The routine that constructs the internal map of the `Maze`. This is the preferred concept for map-building work that should not be mislabeled as generic `Explore...` behavior. | `MazeMap/MazeMap/MissionRunMode.cpp`, `MazeMap/MazeMap/Maze.h` |
| `phase` | A dynamic execution block inside a mode, usually a named hold, settle, launch, sweep, or internal handoff boundary. Phases are mode concepts, and routines are one way a mode may construct a phase. Infrastructure may support recording phase transitions, but should not need to understand phase-specific behavior. | `MazeMap/MazeMap/AuxMeasurementController.cpp`, `MazeMap/MazeMap/PositionAccuracyAuditMode.cpp`, `AGENTS.md`, `execution_model_guide.md` |
| `section` | A labeled piece of a measurement or audit workflow, especially when the label matters for logs, result summaries, or artifacts. Use it for report-facing subdivisions inside one mode or routine. | `MazeMap/MazeMap/BootModeDescriptor.h`, `boot_mode_infrastructure_spec.md`, `AGENTS.md` |
| `controller` | A logic owner or execution owner used by a mode. A controller is not automatically a boot mode. Current names like `DiagnosticController` or historical names like `MissionModeController` are implementation owners, not evidence of a runtime mode machine. | `MazeMap/MazeMap/TopSpeedMeasurementMode.cpp`, `MazeMap/MazeMap/OpenFloorMeasurementController.cpp` |
| `session` | One uninterrupted `LoopController` run between `BeginSession()` and completion or runtime stop. A boot cycle normally gets one such session; a second session requires known state discontinuity such as user service or a physical lift that invalidates UKF continuity. | `MazeMap/MazeMap/LoopController.h` |
| `tick` | One fixed-period control cycle inside a `LoopController` session. | `MazeMap/MazeMap/LoopController.h` |
| `callback` | The umbrella term for function-pointer-based re-entry or dynamic flow-control reconfiguration. In project usage, callbacks usually participate directly in primary control flow, lifecycle transitions, or framework-owned dispatch. `SetNextModeWorkCallbacks(...)` and `SetNextModeWorkCallback(...)` are the sanctioned control-transfer setters inside the one active session. | `MazeMap/MazeMap/LoopController.h`, `MazeMap/MazeMap/TopSpeedMeasurementMode.cpp`, `MazeMap/MazeMap/AuxMeasurementController.cpp`, `MazeMap/MazeMap/DriveBase.h` |
| `hook` | An optional, narrower callback attached to an already-owned routine or framework, typically for observation, instrumentation, or side effects rather than primary dispatch ownership. Every hook is a callback in the broad sense, but not every callback should be called a hook. | `MazeMap/MazeMap/ManeuverExecutor.h`, `MazeMap/MazeMap/ManeuverExecutor.cpp` |
| `pause` | The sanctioned escape from strict periodic cadence for blocking, no-motion calculation, and other non-real-time work. Use `RequestPause()` instead of sleeping, spinning, or waiting for another tick outside pause handling. While paused, the robot must remain motionless, and `LoopController` pause/start/stop transitions are only valid while stationary or faulting. | `MazeMap/MazeMap/LoopController.h`, `AGENTS.md`, `boot_mode_setup_guide.md` |

## Boot And Runtime Ownership Terms

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `BootModeRegistry` | The authoritative startup-mode inventory and selector table. It owns stable ids, selector conditions, reboot requirement, descriptor references, and launch precedence. | `MazeMap/MazeMap/BootModeRegistry.h`, `MazeMap/MazeMap/BootModeRegistry.cpp`, `AGENTS.md` |
| `BootModeDescriptor` | The authoritative descriptive metadata for one boot mode: category, purpose, outputs, entry point, implementation file, phases, shared tuning, overrides, and expected artifacts. | `MazeMap/MazeMap/BootModeDescriptor.h`, mode `*.cpp` descriptor definitions |
| `stableId` | The stable machine-readable identity string for a boot mode, used for metadata and human-readable identification. | `MazeMap/MazeMap/BootModeDescriptor.h`, mode descriptor definitions |
| `selector condition` | The hardware condition that selects a boot mode, usually a pin-pair strap or fallback. | `MazeMap/MazeMap/BootModeRegistry.h`, `MazeMap/MazeMap/BootModeRegistry.cpp` |
| `IApplicationMode` | The top-level runnable mode contract with explicit `Begin()` and `Run()` phases. | `MazeMap/MazeMap/IApplicationMode.h` |
| `SharedRobotRuntime` | The shared runtime composition root and owner of shared single-instance production resources such as the maze, pathfinders, `logging.txt`, the runtime `MmLogLogger`, sensors, drive, and control loop. | `MazeMap/MazeMap/SharedRobotRuntime.h`, `AGENTS.md`, `boot_mode_setup_guide.md` |
| `LoopController` | The authoritative fixed-period control-session owner. It owns cadence, tick capture timing, sensing/update timing, and the active per-tick callback dispatch. | `MazeMap/MazeMap/LoopController.h`, `loop_controller_inventory.md` |
| `TeensyLayout` | The authoritative owner of the robot's hardwired connectivity configuration derived from the circuit diagram. Use this term for the physical wiring/layout authority: pin assignments, peripheral hookups, and startup hardware-setup assumptions. `Pins` and `HardwareConfig` are subordinate hardware vocabulary, not competing owners. | `MazeMap/MazeMap/TeensyLayout.h`, `MazeMap/MazeMap/Pins.h`, `MazeMap/MazeMap/HardwareConfig.h` |
| `LSM6DSV16X_IMU` | The low-level IMU device class built from the sensor datasheet so register addresses, bitfields, and chip-facing behavior live in one authoritative place instead of being scattered as magic numbers. It is the chip-facing IMU owner, not the higher-level runtime sensing or estimation owner. | `MazeMap/MazeMap/LSM6DSV16X_IMU.h`, `MazeMap/MazeMap/Vehicle.h` |
| `search vehicle` / `speed vehicle` | Runtime-owned vehicle variants used for search-style and speed-style motion/tuning contexts. These are not different robots; they are different runtime vehicle owners exposed by `SharedRobotRuntime`. | `MazeMap/MazeMap/SharedRobotRuntime.h` |
| `Vehicle` | The authoritative owner of robot construction facts, geometry, physical dimensions, physical limits, sensor extrinsics, and other fixed robot facts. | `MazeMap/MazeMap/Vehicle.h`, `AGENTS.md` |
| `PlantModel` | The authoritative owner of the vehicle's physics model. Anything related to the physics of motion belongs here: plant equations, plant evaluation, inverse drive solves, traction/force modeling, and other motion-physics-derived quantities. | `MazeMap/MazeMap/PlantModel.h`, `AGENTS.md` |
| `Maze` | The authoritative maze representation and maze-domain behavior owner. | `MazeMap/MazeMap/Maze.h`, `AGENTS.md` |
| `WallBeliefMap` | A probabilistic wall-evidence accumulation structure. It is a wall-belief owner, not a second authoritative maze representation. | `MazeMap/MazeMap/WallBeliefMap.h` |

## Maze-Grid Vocabulary

These terms belong to the discrete maze-language side of the codebase.

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `Maze` | The authoritative maze-domain owner and the main discrete grid representation. | `MazeMap/MazeMap/Maze.h`, `AGENTS.md` |
| `Direction` | The absolute direction vocabulary in the maze grid, such as `Up`, `Down`, `Left`, and `Right`. | `MazeMap/MazeMap/Direction.h`, `MazeMap/MazeMap/Direction.cpp` |
| `RelativeDirection` | The relative direction vocabulary in the maze grid, used to describe direction changes relative to the current heading. | `MazeMap/MazeMap/Direction.h`, `MazeMap/MazeMap/Direction.cpp` |
| `CellCoordinates` | The cell-based full-step coordinate vocabulary for the maze grid. Use it when reasoning in whole cells. | `MazeMap/MazeMap/CellCoordinates.h` |
| `MazeLocation` | The complementary half-step maze-grid vocabulary for specific walls, corners, and cell centers. Use it when cell-level coordinates are not precise enough. | `MazeMap/MazeMap/MazeLocation.h` |
| `DirectionalLocation` | A maze location plus a facing direction. This is the usual discrete pose vocabulary for maze planning. | `MazeMap/MazeMap/DirectionalLocation.h` |
| `goal` | The maze goal region. `Maze::GetGoalLowerLeft()` returns the lower-left cell of the 2x2 goal block. | `MazeMap/MazeMap/Maze.h` |
| `Path` | A cell-by-cell route represented as a sequence of `CellCoordinates`. | `MazeMap/MazeMap/Path.h`, `MazeMap/MazeMap/PathFinder.h` |
| `HalfStepPath` | A higher-resolution grid path with half-cell granularity. It still belongs to the maze-grid side even though it gives finer maze resolution. | `MazeMap/MazeMap/HalfStepPath.h`, `MazeMap/MazeMap/PathFinder.h` |
| `FloodFillPathFinder` | The simple flood-fill planner used for exploration and goal routing on the cell grid. | `MazeMap/MazeMap/FloodFillPathFinder.h`, `AGENTS.md` |
| `SearchRunPlanner` | Helper vocabulary for straight-segment planning and replan responses during search-style driving. | `MazeMap/MazeMap/SearchRunPlanner.h` |
| `search run` / `exploration` | The mission-phase vocabulary for mapping and finding the goal while the maze is still being learned. This commonly uses `FloodFillPathFinder` and search straight planning. | `MazeMap/MazeMap/MissionRunMode.cpp`, `MazeMap/MazeMap/SearchRunPlanner.h`, `AGENTS.md` |

## Bridge Vocabulary Between Maze Grid And Motion

These terms translate between the discrete maze-grid language and the continuous motion side of the codebase.

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `ManeuverCode` | The symbolic id for one predefined traversal primitive, for example `S90SS` or `IP180`. | `MazeMap/MazeMap/Maneuver.h` |
| `Maneuver` | A specific, predefined, hand-optimized maze-traversal path from the catalog owned by `ManeuverSet`. It is the interface between the maze's grid language and the drive system's motion language. It is **not** a synonym for a generic move, hold, settle, reverse, straight-distance command, free turn, or free arc. If the behavior is not one of the named catalogued `ManeuverCode` paths, do not call it a maneuver. | `MazeMap/MazeMap/Maneuver.h`, `MazeMap/MazeMap/Maneuver.cpp`, `AGENTS.md` |
| `ManeuverSet` | The canonical registry and semantics owner for the predefined maneuver catalogue. | `MazeMap/MazeMap/ManeuverSet.h`, `MazeMap/MazeMap/ManeuverSet.cpp` |
| `ManeuverPath` | A path expressed as a sequence of maneuver codes rather than a sequence of cells. | `MazeMap/MazeMap/ManeuverPath.h`, `MazeMap/MazeMap/ManeuverPath.cpp` |
| `ManeuverInstance` | One realized executable use of a predefined maneuver, including a start pose and entry/exit speeds. This is the canonical execution vocabulary for maneuver-driven motion. A command expressed only as distance, angle, or hold/settle timing is not a `ManeuverInstance`; it is generic motion vocabulary unless it is first represented through the canonical maneuver owners. | `MazeMap/MazeMap/ManeuverInstance.h`, `AGENTS.md`, `MazeMap/CODEBASE_NAVIGATION.md` |
| `ManeuverPathFinder` | The planner that turns maze state into executable maneuver sequences. Use it while stationary, not during active motion. | `MazeMap/MazeMap/ManeuverPathFinder.h`, `AGENTS.md` |
| `ManeuverExecutor` | The execution-side owner that consumes maneuver-oriented vocabulary such as `ManeuverInstance`, `ManeuverQueue`, and `ManeuverPoint`, then drives maneuver execution against the lower motion-command layers. It should stay centered on maneuver execution rather than becoming a general bucket for unrelated hold, settle, reverse-straight, straight-distance, free-turn, or free-arc helpers. If an owner mainly exposes those generic motion primitives, it is not accurately described as a `ManeuverExecutor`. | `MazeMap/MazeMap/ManeuverExecutor.h`, `MazeMap/MazeMap/SharedRobotRuntime.h` |
| `speed run` | The mission-phase vocabulary for a fast goal-directed run once the relevant maze knowledge is available. This typically uses `ManeuverPathFinder` and queued maneuver execution. | `MazeMap/MazeMap/MissionRunMode.cpp`, `AGENTS.md` |

## Float-Based Motion Vocabulary

These terms belong to the continuous, float-based motion side of the codebase.

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `project body frame` | The robot/body-frame sign convention: `+X = right`, `+Y = forward/up`, and `+Yaw = clockwise`. | `AGENTS.md`, `MazeMap/MazeMap/PlantModel.h` |
| `state` | The specific values representative of the robot state used by the UKF filter. In practice this usually means the estimator state vector and closely related filter state quantities. | `MazeMap/MazeMap/VehicleState.h`, `MazeMap/MazeMap/PlantModel.h` |
| `pose` | Not a heavily used project term, but when used it means the vehicle position and heading. It is narrower than full filter state. | `MazeMap/MazeMap/VehicleState.h`, `MazeMap/MazeMap/DriveBase.h`, `MazeMap/MazeMap/LoopController.h` |
| `target` | On the robot/motion side, essentially the same idea as a setpoint. A target may be a distance, speed, acceleration, heading, yaw rate, or alpha, and is not usually a position specifically. On the maze side, `target` usually means the destination of a pathfinding search. | `MazeMap/MazeMap/DriveBase.h`, `MazeMap/MazeMap/PathFinder.h`, `MazeMap/MazeMap/SearchRunPlanner.h` |
| `physical coordinates` / `physical location` | The preferred disambiguating term for continuous-space robot-side position language when a nearby discussion may also involve `CellCoordinates`, `MazeLocation`, or other maze-grid terms. Use plain `coordinates` / `location` only when the physical-side context is already obvious. | `MazeMap/MazeMap/DriveBase.h`, `MazeMap/MazeMap/MotionTargetProjection.h`, `MazeMap/MazeMap/MazeLocation.h` |
| `PlantModel` | The authoritative owner of the vehicle's physics model. Anything related to the physics of motion belongs here: plant equations, plant evaluation, inverse drive solves, traction/force modeling, and other motion-physics-derived quantities. | `MazeMap/MazeMap/PlantModel.h`, `AGENTS.md` |
| `ManeuverPoint` | The per-point target primitive used while tracking through a maneuver. It carries pointwise position, heading, yaw rate, and velocity targets. | `MazeMap/MazeMap/Maneuver.h`, `AGENTS.md` |
| `motion primitive` / `generic motion primitive` | A non-maneuver movement or hold concept expressed directly in continuous motion terms such as a timed hold, braked settle, reverse straight, straight-distance drive, free turn, or free arc. These are real concepts, but they are not maneuvers unless they are represented through the canonical maneuver vocabulary. | `MazeMap/MazeMap/DriveBase.h`, `MazeMap/MazeMap/ManeuverInstance.h`, `AGENTS.md` |
| `DriveBase` | The concrete runtime motion owner that turns targets and control requests into wheel/motor commands while also owning odometry, estimator-facing pose state, and the underlying closed-loop wheel-control machinery. It is the destination for concrete "move in this manner right now" commands such as open-loop drive, point commands, and delta commands. It is **not** the intended home for higher-level motion-language translation, maneuver scheduling, queue ownership, or shared multi-tick routine orchestration. | `MazeMap/MazeMap/DriveBase.h`, `AGENTS.md` |
| `Drive` | The higher-level motion layer above `DriveBase` and below maneuver-planning/mode logic. It is a shared runtime service borrowed from `SharedRobotRuntime`, not a per-mode owner. Mode logic should arm it by calling `Start...` members for the active multi-tick motion primitive or maneuver, and the active mode callback should normally call a generic per-tick method such as `GetNextControls(bool& done)` when it wants Drive's current proposal. `done` is advisory: ignoring it must not make the service incoherent or implicitly revoke the last explicit instruction the mode gave it. The mode may return Drive's proposed output, override it, ignore it, or stop polling Drive altogether. Drive keeps the active primitive's execution data internally rather than pushing that state out to callers so most mode logic does not need to care which primitive is active, and it is the exemplar template of a solid multi-tick service in this codebase. | `MazeMap/MazeMap/Drive.h`, `MazeMap/MazeMap/SharedRobotRuntime.h`, `AGENTS.md` |
| `OpenLoopDriveCommand` | A raw normalized left/right drive command pair. Use it when you mean raw actuator command values, not a maneuver or target state. | `MazeMap/MazeMap/OpenLoopDriveCommand.h` |
| `PointCommand` / `PointControlVector` | Drive commands that target a desired forward speed, yaw rate, or `ManeuverPoint`. Use these when commanding a target state or target point. | `MazeMap/MazeMap/DriveBase.h` |
| `DeltaCommand` | A drive command resolved from an explicit operating point plus desired longitudinal and optionally yaw acceleration. Use this when you mean "command the delta from the current motion state." | `MazeMap/MazeMap/DriveBase.h` |

## Logging And Artifact Terms

| Term | Meaning here | Canonical anchors |
| --- | --- | --- |
| `logging.txt` | The runtime-owned sparse human-readable text log and the project's practical equivalent of `std::cout` / `Serial.write` while the robot is moving. Use it for short summaries, phase transitions, metadata association, operator-facing notes, and fault reasons. | `MazeMap/MazeMap/SharedRobotRuntime.h`, `AGENTS.md`, `boot_mode_setup_guide.md` |
| `mmlog` | The structured telemetry logging format and the family of `.mmlog` files written through the runtime-owned logger. | `MazeMap/MazeMap/MmLog.h`, `AGENTS.md`, `boot_mode_setup_guide.md` |
| `MmLogLogger` | The one production runtime-owned structured logger instance. Modes borrow it through `SharedRobotRuntime`; they do not own additional production logger instances. | `MazeMap/MazeMap/SharedRobotRuntime.h`, `AGENTS.md` |
| `telemetry` | Repeated structured measurements about state, control, timing, or sensors. Telemetry belongs in `mmlog`, not in `logging.txt`. | mode descriptors, `boot_mode_setup_guide.md` |
| `metadata` | File-level structured setup facts written ahead of row logging, such as mode id, control period, tuning facts, run id, or associated filenames. Metadata is still buffer-limited runtime infrastructure, not a place for arbitrarily large setup dumps. | `MazeMap/MazeMap/SharedRobotRuntime.h`, `boot_mode_setup_guide.md` |
| `row schema` | A typed `mmlog` record layout declared with the project row macros such as `MMLOG_DEFINE_ROW(...)`. | `MazeMap/MazeMap/MmLog.h`, mode `*.cpp` logging rows |
| `phase marker` | A sparse human-readable record of a phase transition, commonly written through `WriteTextLogPhase(...)` or equivalent transition-recording helpers. Prefer wording like `RecordPhaseTransition` when you mean support for logging the transition rather than ownership of phase semantics. | `MazeMap/MazeMap/SharedRobotRuntime.h`, `MazeMap/MazeMap/AuxMeasurementController.cpp` |
| `artifact` | A named file or durable output produced by a mode, such as a `.mmlog`, `maze.txt`, or characterization result. | `MazeMap/MazeMap/BootModeDescriptor.h`, mode descriptor definitions |
| `startup trace` | A sparse trace of mode startup and major transition notes written through shared startup-trace helpers. | `MazeMap/MazeMap/BootUtilityModeFramework.h`, `boot_mode_setup_guide.md` |

## Practical Logging Constraints

- Treat `logging.txt` as the project's console-style output channel when the robot is active. If you would normally reach for `std::cout` or `Serial.write`, this is usually the correct runtime destination.
- `logging.txt` is small and easy to overwhelm. Its queue budget is about 4 KB, so text output must stay extremely concise.
- The `mmlog` metadata path is also buffer-limited. Large metadata bursts can overflow if they are emitted without servicing the logger.
- If setup needs to emit a large metadata block that approaches the buffer limit, call the runtime log service before the buffer fills.
- If setup metadata is comfortably under the small buffer budget, an extra service call is usually unnecessary.
- Because RAM and storage are tight, prefer compact metadata keys/values and very sparse text logging.
- Do not use `logging.txt` for high-rate telemetry or verbose tracing. Put repeated structured data in `mmlog` rows instead.

## Recommended Wording

- Reserve `mode` for boot-selected top-level application modes.
- Use `phase` for a dynamic execution block inside a mode.
- Use `section` for a report-facing or workflow-facing subdivision inside a larger routine or mode.
- Use `routine` for a public high-level procedure, not for a single tick callback, a single motion primitive, or a hidden sub-mode selector.
- Describe the dependency direction as Modes using routines; never describe routines as depending on Modes.
- Use `callback` as the umbrella term for function-pointer-based dynamic flow-control reconfiguration. Use `mode-work callback` or `tick function` when you specifically mean the active `LoopController` work function.
- Use the callback setter APIs for control transfer inside the one active `LoopController` session; do not treat them as permission to start another session.
- Do not describe a second session as ordinary phase control. It is only for known state discontinuity such as user service or a physical lift that invalidates UKF continuity.
- Use `hook` for optional subordinate callbacks around an already-owned routine or framework, especially for observation or instrumentation.
- Use `policy` for stateless rule/criteria helpers, not for controllers, routines, or geometry owners.
- Use wording like `RecordPhaseTransition` when you mean logging support around phases rather than infrastructure understanding what the phase does.
- Use `Direction` for absolute maze-grid directions and `RelativeDirection` for relative maze-grid directions.
- Use `CellCoordinates` for full-step cell coordinates and `MazeLocation` for half-step wall/corner/center locations.
- Prefer `physical coordinates` or `physical location` when you need to distinguish continuous robot-side position language from maze-grid coordinates.
- Use `path` for a cell sequence, `maneuver path` for a maneuver-code sequence, `maneuver instance` for an executable segment, and `maneuver point` for a point along that segment.
- Do not use `maneuver` as a generic synonym for "movement" or "motion primitive." Timed holds, settles, reverse-straight moves, free straight-distance drives, free turns, and free arcs are generic motion vocabulary unless they are represented through the canonical maneuver owners.
- If an API or class primarily accepts raw distances, angles, or hold/settle timing rather than `ManeuverCode`, `ManeuverInstance`, `ManeuverQueue`, or `ManeuverPoint`, do not name it with `Maneuver...` vocabulary.
- Reserve `ManeuverExecutor` wording for owners whose primary responsibility is executing the canonical maneuver vocabulary. Do not use that name for a grab bag of unrelated shared motion helpers.
- Use `DriveBase` when you mean the concrete low-level runtime motion owner that produces wheel/motor commands and maintains drive/pose control state.
- Do not use `DriveBase` wording for higher-level shared motion-routine ownership, maneuver scheduling, or software-to-motion translation responsibilities.
- Treat `Drive` as shared runtime infrastructure that mode logic arms and then queries through a generic per-tick method such as `GetNextControls(bool& done)` from inside the active mode callback. It is the exemplar template of a solid multi-tick service. Do not describe it as mode-owned, do not make callers instantiate per-mode `Drive` copies, and do not push primitive execution state out to callers.
- Treat informal phrases like `runtime.Drive()` as the legacy `DriveBase` accessor name, not as the higher-level `Drive` service; use `runtime.DriveService()` when you mean the shared multi-tick service owner.
- Do not introduce new standalone `Profile` concepts as preferred vocabulary. If a so-called profile only parameterizes one owner or one algorithm, it should generally become an internal type or helper of that owner.
- Use `state` for the UKF/state-estimation values, `pose` for position-plus-heading only, and `target` for robot-side setpoints or maze-side pathfinding destinations depending on context.
- Use `TeensyLayout` for the hardwired connectivity and board-layout authority, with `Pins` and `HardwareConfig` as subordinate hardware terms rather than separate owners.
- Use `LSM6DSV16X_IMU` for the low-level chip-facing IMU driver/owner that centralizes datasheet register vocabulary and avoids scattered magic numbers.
- Use `Vehicle` for robot construction facts, `PlantModel` for motion physics, `DriveBase` for low-level motion-command production, and `Drive` for the higher-level software-to-motion translation layer and exemplar shared multi-tick service.
- Use `logging.txt` as the extremely concise console-style output channel and `mmlog` for dense machine-readable telemetry.
- Use `selector` or `selector condition` for the physical jumper or pin-pair boot-entry rule.
- Use `mission mode` and `utility mode` only as scenario labels inside the same common boot-mode model. Neither term implies a different architectural shape, and mission mode may be smaller than many test modes because verified behavior must stay tight.

## Current Legacy Exceptions And Drift

- `BootModeId::PrimaryDiagnostic` currently resolves to the open-floor measurement descriptor and implementation target. Treat that as a current naming mismatch, not as evidence that "diagnostic" and "open-floor measurement" are distinct top-level implementations today.
- The `28-29` selector currently depends on `AuxMeasurementConfig::kRoutine`, so the effective selected id can become `auxiliary_measurement`, `corridor_repeatability`, or `position_accuracy_audit`. If those are really separate public workflows rather than one mode's internal routines, this is likely architectural drift and they should become fully separated Modes.
- `profile` is not preferred standalone project vocabulary. Broader attempts to use `profile` as an umbrella term here are drift from prior AI intervention that interpreted `Maneuver` too loosely. Existing `*Profile` names remain current code names where they already exist, but they should generally be treated as owner-internal helpers or parameter bundles rather than as sanctioned first-class concepts.
- Some existing code still uses `Maneuver...` names for shared motion helpers that are not actually expressed in `ManeuverCode` / `ManeuverInstance` / `ManeuverPoint` terms. Treat that as naming and ownership drift, not as permission to broaden the meaning of `maneuver`.
- `SharedRobotRuntime::Drive()` currently returns `DriveBase`, while `SharedRobotRuntime::DriveService()` returns the higher-level multi-tick `Drive` owner. Treat the shorter accessor name as current naming drift around the legacy `DriveBase` entry point, not as a reason to blur the two owners together.
- `MissionStartPolicy.h` currently mixes genuine policy-style criteria with physical-coordinate and sampling-geometry helper logic. Treat that as current naming drift rather than as the ideal shape of a `policy` owner.
- `TeensyLayout.h` currently appears to be the intended hardware-connectivity authority, but much of the codebase still treats `Pins`, `HardwareConfig`, or direct setup helpers as the visible access path instead of explicitly recognizing `TeensyLayout` as that owner. Treat that as current drift rather than as a reason to fragment hardware ownership further.
- `MissionRunMode`, `CorridorRepeatabilityMode`, and `PositionAccuracyAuditMode` are the public boot-mode owners for the maze-running family. Shared maze-running mechanics may still exist privately behind those mode owners while convergence continues, but they should not reappear as a second public mode surface.
- Some existing docs and historical notes use looser wording around "mode" and "routine." When that wording conflicts with this glossary, prefer this glossary and the canonical source files above.
