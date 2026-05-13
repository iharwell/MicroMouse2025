# Repository Infrastructure and Support Map

This map focuses on infrastructure and support systems across the repository.

Excluded by request:

- `PlantModel*`
- `UKF*`
- `SrUkfCore*`
- feedforward-focused implementation details and replay/tuning analysis

Some mapped files still depend on excluded subsystems. In those cases, this document describes the infrastructure role only.

## Top-Level Layout

- `MazeMap/MazeMap`
  - Production library and firmware-side runtime infrastructure.
- `MazeMap/MazeMapTest`
  - Host-side unit and regression coverage for production code.
- `MazeMap/MazeSimulation`
  - Desktop simulator harness for maze and pathfinding workflows.
- `codex_verify`
  - Authoritative build, verification, upload, and artifact-freshness helpers.
- `tooling`
  - Offline analysis scripts and dependency-graph assets.
- `Maze Files`
  - Historical and test maze corpus used by simulation, tests, and planning sweeps.
- `scripts`
  - Repo-maintenance scripts.
- `staging`
  - Working notes and temporary analysis artifacts; useful context, not an authoritative owner.

## Runtime and Boot Infrastructure

### Entry and selection flow

1. `MazeMap/MazeMap/MazeMap.ino`
   - Arduino entrypoint that delegates to `MazeMap::App::Application`.
2. `MazeMap/MazeMap/Application.h` and `Application.cpp`
   - Thin application shell. `Setup()` resolves the active mode, runs it through the shared control loop, finalizes runtime logging, then halts.
3. `MazeMap/MazeMap/MazeMapApplicationRuntime.h` and `MazeMapApplication.cpp`
   - Narrow runtime boundary for resolving the selected top-level mode.
4. `MazeMap/MazeMap/BootModeRegistry.h` and `BootModeRegistry.cpp`
   - Authoritative boot-selection table: selector conditions, fallback behavior, reboot requirement, and descriptor references.
5. `MazeMap/MazeMap/BootModeDescriptor.h`
   - Descriptor schema for each boot-selected mode: stable ID, purpose, outputs, entrypoint, implementation file, phases, tuning notes, and expected artifacts.
6. `MazeMap/MazeMap/IApplicationMode.h`
   - Narrow top-level mode contract: `SetupMode()` plus `RunTick(...)`.
7. `MazeMap/MazeMap/LoopController.h` and `LoopController.cpp`
   - Strict-cadence owner of the active control session: session state, timing, callback dispatch, pause/end-session boundaries, and terminal halt.
8. `MazeMap/MazeMap/SharedRobotRuntime.h` and `SharedRobotRuntime.cpp`
   - Production composition root. Owns the shared runtime services that modes borrow instead of duplicating.

### Shared runtime owners

`SharedRobotRuntime` is the main support-system anchor. In the requested scope it owns or exposes:

- the canonical production `Maze`
- speed and search `Vehicle` owners
- the production `FloodFillPathFinder`
- the production `ManeuverPathFinder`
- the low-level `DriveBase`
- the higher-level shared `Drive` service
- `RuntimeSensorSuite`
- `StartupCalibration`
- `WallTouch`
- `ManeuverExecutor`
- the one `LoopController`
- the one production `MmLogLogger`
- the one runtime-owned `logging.txt` text log

### Boot-mode registry inventory

`BootModeRegistry.cpp` currently exposes eight selectable entries:

- `PinPair(39, 40)` -> `FrontWallCharacterization`
- `PinPair(38, 39)` -> `WallSensorLedCalibration`
- `PinPair(28, 29)` -> auxiliary selector entry, resolved by `AuxMeasurementConfig`
- `PinPair(29, 30)` -> `ManeuverFileTest`
- `PinPair(26, 27)` -> `TopSpeedMeasurement`
- `PinPair(27, 28)` -> `OpenFloorMeasurement`
- `PinPair(9, 10)` -> `ShowcasingDonut`
- fallback -> `Mission`

The auxiliary selector entry can resolve to:

- `AuxiliaryMeasurement`
- `CorridorRepeatability`
- `PositionAccuracyAudit`

### Runtime-support hubs

- `MazeMap/MazeMap/MazeMapControllerRegistry.h`
  - Declaration hub for mode getters and mode descriptors.
- `MazeMap/MazeMap/BootUtilityModeFramework.h` and `.cpp`
  - Very small shared boot helper surface today: startup-trace logging through the runtime-owned text log.
- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
  - Large mixed runtime-support header for motion-limit types, timing bundles, calibration helpers, sensor-input support types, and shared utility logic.
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` and `.cpp`
  - Runtime measurement/logging infrastructure, especially `mmlog` row schemas and log-start helpers for diagnostic and measurement modes.
- `MazeMap/MazeMap/MazeMapRuntimeSignalHelpers.h` and `.cpp`
  - Shared signal-processing helpers reused by runtime sensing and wall-based control calculations.
- `MazeMap/MazeMap/MazeMapApplicationPrivate.h`
  - Private include aggregator for app/runtime code. Useful for navigation, but not an authoritative domain owner.

## Maze, Navigation, and Maneuver Infrastructure

### Authoritative maze and path owners

- `MazeMap/MazeMap/Maze.h` and `Maze.cpp`
  - Authoritative maze representation: cells, wall state, reachability, accessibility, goal handling, legality checks, and export helpers.
- `MazeMap/MazeMap/PathFinder.h` and `PathFinder.cpp`
  - Base pathfinding contract and shared path-construction logic.
- `MazeMap/MazeMap/FloodFillPathFinder.h`
  - Canonical flood-fill planner for search-style navigation.
- `MazeMap/MazeMap/DirectionalPathFinder.h` and `.cpp`
  - Heading-aware cell planner with directional costs.
- `MazeMap/MazeMap/ManeuverPathFinder.h` and `.cpp`
  - Stationary maneuver planner that produces executable maneuver sequences.
- `MazeMap/MazeMap/ManeuverExecutor.h` and `.cpp`
  - Shared routine-style executor that installs loop callbacks to run queued maneuvers, then returns control to a continuation callback.

### Supporting maze vocabulary

- `Cell.h` and `Cell.cpp`
  - One maze cell, including wall-state storage.
- `CellCoordinates.h` and `.cpp`
  - Grid coordinates and neighbor movement helpers.
- `Direction.h` and `.cpp`
  - Direction enums, relative-direction helpers, and shared directional math.
- `MazeLocation.h` and `.cpp`
  - Location within a maze cell, with physical/logical conversions.
- `DirectionalLocation.h` and `.cpp`
  - Pairing of a location with a heading.
- `MazeMask.h` and `MazeMask.cpp`
  - Bitmask representation used by maze algorithms.
- `MaskQueue.h` and `MaskQueue.cpp`
  - Queue helpers used by flood-fill style searches.
- `WallBeliefMap.h`
  - Shared wall-belief storage used by runtime services that work with partially known mazes.

### Path and maneuver containers

- `Path.h` and `Path.cpp`
  - Fixed-capacity cell-by-cell path container.
- `PathPoint.h` and `PathPoint.cpp`
  - Small per-point path helpers.
- `HalfStepPath.h`
  - Half-cell path container used for finer-grained planning.
- `CompactPath.h` and `CompactPath.cpp`
  - Lighter-weight path container.
- `Maneuver.h` and `Maneuver.cpp`
  - Maneuver catalogue and execution geometry/target helpers.
- `ManeuverSet.h` and `ManeuverSet.cpp`
  - Legal maneuver registry and maneuver-code semantics.
- `ManeuverPath.h` and `ManeuverPath.cpp`
  - Path container expressed in maneuvers instead of raw cells.
- `ManeuverInstance.h`
  - Canonical execution vocabulary for one realized maneuver segment.
- `ManeuverQueue.h`
  - Queue used to stage maneuvers for execution.

### Navigation support helpers

- `SearchRunPlanner.h`
  - Search straight-segment planning and replan-response helpers.
- `MissionStartPolicy.h`
  - Shared mission/startup geometry and front-calibration positioning helpers.
- `MapEvidenceUpdater.h` and `.cpp`
  - Support layer that accumulates wall evidence into map state. It touches excluded estimator-adjacent surfaces but still participates in shared map support.

### Navigation call flow

- `Maze` owns the authoritative maze state.
- `FloodFillPathFinder` builds exploration/search paths from that maze.
- `DirectionalPathFinder` adds heading-aware planning when direction matters.
- `ManeuverPathFinder` turns maze/vehicle facts into maneuver-native paths while stationary.
- `ManeuverExecutor` runs queued maneuver work by borrowing the shared runtime’s `Drive` and `LoopController` instead of owning another motion stack.

## Sensing, Logging, Hardware, and Motion Support

### Build boundary and hardware surface

- `MazeMap/MazeMap/Defines.h`
  - Central host/Teensy boundary. `MazeMap::Platform` lives here: PWM, ADC, encoder, timing, delay, and Arduino-host compatibility surfaces.
- `MazeMap/MazeMap/Pins.h`
  - Board pin map for motors, encoders, IMUs, LEDs, wall sensors, and fan control.
- `MazeMap/MazeMap/HardwareConfig.h`
  - Hardware timing and electrical constants.
- `MazeMap/MazeMap/TeensyLayout.h`
  - Teensy bring-up helpers: pin configuration, SPI start, encoder init, wall-sensor ADC setup, SD mounting, and status LEDs.
- `MazeMap/MazeMap/LSM6DSV16X_IMU.h`
  - Production IMU device driver.

### Sensor and calibration infrastructure

- `MazeMap/MazeMap/RuntimeSensorSuite.h` and `.cpp`
  - Canonical runtime sensor owner. Captures wall-sensor and IMU state into one `SensorSnapshot`, maintains bias/filter state, and applies wall-distance calibration.
- `MazeMap/MazeMap/SensorSnapshot.h` and `SensorSnapshot.cpp`
  - Unified per-tick sensor payload and snapshot-combination helpers.
- `MazeMap/MazeMap/SensorTelemetryTypes.h`
  - Shared telemetry/timing structs for optical and IMU data.
- `MazeMap/MazeMap/WallSensor.h`
  - Per-sensor hardware abstraction: LED control, ADC reads, light conversion, and raw distance estimation.
- `MazeMap/MazeMap/WallSensorCalibration.h`
  - Reusable wall-sensor calibration curve type.
- `MazeMap/MazeMap/WallDistanceCalibration.h` and `.cpp`
  - Runtime-owned wall calibration model and threshold derivation logic.
- `MazeMap/MazeMap/WallSensorRuntimeTypes.h`
  - Shared wall-sensor IDs and calibration-mode helpers.
- `MazeMap/MazeMap/StartupCalibration.h` and `.cpp`
  - Shared startup calibration service used by multiple boot modes.
- `MazeMap/MazeMap/WallTouch.h` and `.cpp`
  - Shared contact-based wall seating/reseating service.

### Motion support

- `MazeMap/MazeMap/DriveBase.h`
  - Concrete low-level motion owner: motor actuation, encoder handling, odometry-facing measurement capture, and closed-loop command production.
- `MazeMap/MazeMap/Drive.h` and `.cpp`
  - Higher-level shared motion-primitive service layered over `DriveBase`; modes consult it while keeping `LoopController` callback ownership.
- `MazeMap/MazeMap/MotorEncoderDrive.h`
  - Motor + encoder hardware wrapper with shared physical-model and hardware-config access.
- `MazeMap/MazeMap/DriveTelemetry.h`
  - Per-tick drive telemetry payload returned by `DriveBase`.
  - Wheel-control scaling bundle used by runtime motion control.

### Logging support

- `MazeMap/MazeMap/MmLog.h` and `MmLog.cpp`
  - Structured binary logging subsystem and row-definition macros.
- `MazeMap/MazeMap/MazeMapRuntimeMmLog.h`
  - Thin runtime-facing `mmlog` include layer.
- `MazeMap/MazeMap/RuntimeBinaryLogSupport.h`
  - Runtime file naming and metadata helpers for `.mmlog` sessions.
- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
  - Defines runtime row schemas such as diagnostic and measurement logs and helper writers that package sensor, drive, and state data.

### Support-layer call flow

- `Defines.h` and `TeensyLayout.h` establish the hardware/platform surface.
- `RuntimeSensorSuite` begins runtime sensing and produces `SensorSnapshot` data per tick.
- `DriveBase` consumes measurements and emits concrete motion commands and `DriveTelemetry`.
- `Drive` interprets higher-level primitive requests and proposes commands through `DriveBase`.
- `SharedRobotRuntime` owns the one structured `MmLogLogger` plus the `logging.txt` text log and exposes both to modes and support helpers.

## Modes as Infrastructure Clients

These files are primarily mode implementations, but they matter as map landmarks because they are the top-level users of the shared support stack:

- `AuxMeasurementController.cpp`
- `FrontWallCharacterizationController.cpp`
- `WallSensorLedCalibrationController.h` and `.cpp`
- `OpenFloorMeasurementController.h` and `.cpp`
- `ShowcasingDonutController.h` and `.cpp`
- `MissionRunMode.cpp`
- `CorridorRepeatabilityMode.cpp`
- `PositionAccuracyAuditMode.cpp`
- `ManeuverFileTestMode.cpp`
- `TopSpeedMeasurementMode.cpp`

These modes typically:

- resolve through `BootModeRegistry`
- borrow services from `SharedRobotRuntime`
- stage `LoopController::SessionOptions`
- use `BootUtilityModeFramework` for startup trace logging when needed
- drive work through `RunTick(...)`, pause callbacks, or continuation-based routine handoffs

## Simulation, Verification, and Tooling

### Desktop simulation

- `MazeMap/MazeSimulation/MazeSimulation.cpp`
  - Main simulator executable. Drives maze/pathfinding flows against the Micromouse simulator.
- `MazeMap/MazeSimulation/Mazes.h` and `Mazes.cpp`
  - Built-in maze catalog and maze-loading helpers.
- `MazeMap/MazeSimulation/SimVehicle.h` and `SimVehicle.cpp`
  - Simulator-side vehicle adapter that executes MazeMap navigation outputs.
- `MazeMap/MazeSimulation/mmsAPI.h` and `mmsAPI.cpp`
  - Bridge to the external simulator API.

Note: `MazeSimulation.cpp` is a mixed-purpose file because it also contains an excluded open-floor UKF benchmark path.

### Host test project

- `MazeMap/MazeMapTest/MazeMapTest.vcxproj`
  - Authoritative host unit-test harness for the production library.

Key infrastructure/support coverage anchors:

- `ApplicationTest.cpp`
  - Boot registry and mode-resolution behavior.
- `SharedRuntimeTest.cpp`
  - Shared runtime ownership, drive/sensor service uniqueness, and runtime logging behavior.
- `RuntimeHelperTest.cpp`
  - Runtime support helpers, `mmlog`, signal helpers, and sensor/runtime utility paths.
- `DiagnosticCoverageTest.cpp`
  - Broad helper/policy/geometry coverage.
- `PathFinderTest.cpp`
  - Core pathfinding behavior.
- `HistoricalMazePathFinderTest.cpp`
  - Historical-maze regression sweep.
- `ManeuverPathFinderTest.cpp`
  - Maneuver-planning coverage.
- `ManeuverTest.cpp`
  - Maneuver semantics and geometry.
- `DriveBaseTest.cpp`
  - Low-level drive behavior.
- `DriveManeuverTests.cpp`
  - Maneuver-execution regressions.
- `WallMappingTest.cpp`
  - Wall and map-related support behavior.
- `MazeTest.cpp`
  - Maze mutation and lookup behavior.
- `MazeMaskTest.cpp`
  - Mask-grid helpers.
- `SearchRunPlannerTest.cpp`
  - Search straight-segment and replanning helpers.
- `LoopControllerTest.cpp`
  - Very small guardrail coverage for `LoopController`.

Gaps worth noting:

- `OpenFloorMeasurementControllerTest.cpp` is effectively a placeholder.
- `LoopControllerTest.cpp` is much narrower than the importance of `LoopController` would suggest.

### Build and verification helpers

- `codex_verify/build_and_verify_latest.ps1`
  - Authoritative build/verify orchestrator for firmware build, host Release build, artifact freshness checks, and Release unit tests.
- `codex_verify/build_and_verify_latest.cmd`
  - Thin launcher wrapper for the PowerShell helper.
- `codex_verify/build_latest.cmd`
  - Build-only wrapper.
- `codex_verify/test_latest_binaries.cmd`
  - Verify-only wrapper for latest Release artifacts when freshness gates pass.
- `codex_verify/ARDUINO_BUILD_NOTES.md`
  - Repo-local Arduino/Eigen staging and intended build entrypoints.
- `codex_verify/build_maneuver_tests.cmd` and `run_maneuver_tests.cmd`
  - Older targeted maneuver-suite helpers.
- `codex_verify/host_syntax.cmd`, `defines_host_syntax.cpp`, `defines_teensy_syntax.cpp`, `imu_header_syntax.cpp`
  - Syntax/probe checks around the centralized host/Teensy boundary and low-level headers.
- `codex_verify/upload_latest_build.ps1` and `upload_latest_build.cmd`
  - Upload/deployment helpers.

### Offline tooling

Non-excluded or partially relevant navigation/support tooling:

- `tooling/analyze_encoder_imu_disagreement.py`
  - Log analysis for encoder/IMU turn-sign disagreements.
- `tooling/open_floor_recovery.py`
  - Recovery-turn summaries and raw-sensor metrics.
- `tooling/open_floor_yaw_fft.py`
  - FFT-based yaw analysis built on recovery outputs.
- `tooling/open_floor_launch_floor.py`
  - Launch-floor and backlash-style motion summary helper.
- `tooling/latest_tlog_dependency_graph.gv`
  - Dependency-graph source for the simulator/project graph.
- `tooling/Dep Graph.png`
  - Rendered dependency graph.
- `tooling/README.md`
  - Tooling documentation hub, though much of it focuses on excluded estimator/plant/feedforward workflows.

### Other navigation aids

- `AGENTS.md`
  - Architectural rules and authoritative-owner policy for this repository.
- `MazeMap/CODEBASE_NAVIGATION.md`
  - Older broad file inventory; still useful, but it mixes in excluded subsystems and misses current drift.
- `boot_mode_infrastructure_spec.md`
  - Boot-mode architecture/spec notes.
- `boot_mode_setup_guide.md`
  - Boot-mode setup checklist and runtime expectations.
- `execution_model_guide.md`
  - `LoopController` session/callback model.
- `project_vocabulary.md`
  - Canonical terminology for runtime, modes, and navigation vocabulary.
- `Hardware.md`
  - Concise hardware summary.

## Quick Starting Points

If you need to re-orient quickly, start here:

- boot/runtime entry: `MazeMap/MazeMap/Application.cpp`
- mode selection: `MazeMap/MazeMap/BootModeRegistry.cpp`
- session lifecycle: `MazeMap/MazeMap/LoopController.h`
- runtime composition: `MazeMap/MazeMap/SharedRobotRuntime.h`
- maze owner: `MazeMap/MazeMap/Maze.h`
- maneuver planning: `MazeMap/MazeMap/ManeuverPathFinder.h`
- maneuver execution: `MazeMap/MazeMap/ManeuverExecutor.h`
- sensor pipeline: `MazeMap/MazeMap/RuntimeSensorSuite.h`
- logging: `MazeMap/MazeMap/MmLog.h`
- verification harness: `MazeMap/MazeMapTest/SharedRuntimeTest.cpp`
- build/verify orchestration: `codex_verify/build_and_verify_latest.ps1`

## Known Hotspots and Ambiguities

- `MazeMap/MazeMap/MazeMapRuntimeCore.h` is an important but broad support hub rather than a narrow owner file.
- `MazeMap/MazeMap/BootUtilityModeFramework.*` exists, but the shared framework is still minimal.
- `MazeMap/MazeMap/MazeMapApplicationPrivate.h` is an include bundle, not a domain owner.
- `MazeMap/MazeSimulation/MazeSimulation.cpp` mixes simulator responsibilities with an excluded benchmark path.
- Several mode implementations are large `.cpp`-local classes with co-located descriptors, so mode logic is easy to find but not always narrowly decomposed.
