# Boot Mode Responsibility Migration Map

## Purpose

- Identify, mode by mode, which responsibilities should move out of current boot-mode implementations.
- Name the authoritative destination for each responsibility.
- Keep boot modes high-level and scenario-focused while centralizing low-level hardware, logging, recovery, and memory-sensitive mechanics.

## Shared Destinations Used In This Map

### `BootModeRegistry`

Owns:

- boot-mode inventory
- selector conditions
- selector precedence
- reboot-required metadata
- stable launch identity

### `BootModeDescriptor`

Owns:

- purpose summary
- artifacts produced
- authoritative entry point
- implementation file location
- major phases
- shared tuning references
- explicit tuning overrides

### `BootUtilityModeFramework`

Owns shared utility-mode session mechanics:

- startup trace begin/append
- fault registration and canonical fail path
- text-log events, metadata, and phase markers
- utility-data-log session lifecycle
- logger failure capture
- timeout/watchdog normalization
- common recovery/session policy
- selected-mode identity and selector provenance for logging

### `SharedRobotRuntime`

Remains the production owner of shared resources:

- `Maze`
- shared pathfinders
- `MmLogLogger`
- `logging.txt`
- shared drive, sensor, and runtime services

### `RuntimeArtifactSupport`

Reusable runtime artifact support for:

- runtime file naming
- artifact input/output
- shared metadata formatting
- import/export helpers
- persistent storage access where a stable domain owner already exists

### `MeasurementModeCore`

Reusable execution core for measurement, calibration, and diagnostic scenarios.

Owns shared mechanics such as:

- control-cycle capture
- timed sample loops
- reusable primitive execution
- marker or pose seeding where shared
- recovery and boundary enforcement
- per-cycle telemetry logging
- section success/fault finalization

### `MissionModeCore`

Reusable execution core for mission-family modes.

Owns shared mechanics such as:

- mission-family startup preparation
- mission-family text tracing and telemetry sessions
- startup wall-calibration/session preparation
- known-maze and known-start seeding
- maneuver-queue execution
- mission-family artifact output such as maze snapshots

### `WallSensorLedHardwareService`

Authoritative low-level wall-sensor LED and calibration-jumper service.

Owns:

- LED pin setup
- LED enable/disable
- pulse generation
- calibration jumper monitoring

## Global Rules For Every Mode

For every boot-selectable mode:

- move selector metadata to `BootModeRegistry`
- move descriptive metadata to `BootModeDescriptor`
- remove direct boot-selection logic from the mode body

## Mode-By-Mode Map

### `WallSensorLedCalibration`

Keep in the mode:

- front-then-side calibration scenario
- selected frequencies
- operator-facing calibration flow

Move out:

- startup trace and text-log plumbing
  Destination: `BootUtilityModeFramework`
- direct pin setup, direct LED on/off, direct jumper probing, and square-wave toggling loops
  Destination: `WallSensorLedHardwareService`
- any future host/target boundary handling for those LED and jumper operations
  Destination: centralized runtime boundary / `WallSensorLedHardwareService`

Resulting mode shape:

- define the calibration phases and frequencies
- ask the wall-sensor LED hardware service to run those phases

### `FrontWallCharacterization`

Keep in the mode:

- characterization scenario
- collapse criteria
- stored row/schema definition
- front-wall-specific artifact semantics

Move out:

- startup trace, fault registration, and shared log-session lifecycle
  Destination: `BootUtilityModeFramework`
- reusable timed sample wait and common stationary capture loop mechanics
  Destination: `MeasurementModeCore`
- reusable reverse-motion capture mechanics, estimator-fault routing, and section finalization
  Destination: `MeasurementModeCore`
- log-file naming and export-session support
  Destination: `RuntimeArtifactSupport`
- persistent read/write/verify plumbing for stored characterization data
  Destination: stable front-wall storage owner plus `RuntimeArtifactSupport`

Resulting mode shape:

- define the capture schedule and completion criteria
- ask `MeasurementModeCore` to execute the reverse characterization scenario
- ask the artifact/storage owner to persist and export the result

### `AuxMeasurement`

Keep in the mode:

- auxiliary scenario definitions
- routine-specific event vocabulary
- aux-specific schemas
- aux-specific thresholds such as traction-slip criteria

Move out:

- startup trace, fault registration, phase markers, and log-session lifecycle
  Destination: `BootUtilityModeFramework`
- timed hold phases, common sample-loop execution, and common per-sample logging mechanics
  Destination: `MeasurementModeCore`
- reusable fan/drive/session state transitions requested by the scenario
  Destination: `MeasurementModeCore` consuming `SharedRobotRuntime`
- runtime file naming and generic artifact metadata formatting
  Destination: `RuntimeArtifactSupport`

Important architectural correction:

- `AuxMeasurementConfig::kRoutine` should not remain a hidden boot-mode remap.
  Destination: explicit architecture decision in `BootModeRegistry`

Resulting mode shape:

- define the selected auxiliary scenario
- provide routine-specific thresholds and result events
- execute through `MeasurementModeCore`

### `OpenFloorMeasurement`

Keep in the mode:

- open-floor vocabulary
- open-floor sections, markers, labels, and artifacts
- open-floor schemas
- open-floor-specific validation and fault vocabulary
- open-floor scenario schedule

Move out:

- startup trace, fault registration, and shared logger session policy
  Destination: `BootUtilityModeFramework`
- dual log-session coordination for timing/main streams
  Destination: `BootUtilityModeFramework` plus `RuntimeArtifactSupport`
- control-cycle capture plumbing
  Destination: `MeasurementModeCore`
- low-level timing capture and cycle-counter plumbing
  Destination: centralized runtime boundary / `SharedRobotRuntime`
- marker seeding, traverse-to-marker, recover-to-marker, and shared boundary enforcement
  Destination: `MeasurementModeCore`
- repeated primitive execution loops for launch, straight, yaw, smooth-turn, and loops
  Destination: `MeasurementModeCore`
- repeated section start/end/fault finalization
  Destination: `MeasurementModeCore`
- generic fault-dump session finalization mechanics
  Destination: `MeasurementModeCore` plus `BootUtilityModeFramework`

Resulting mode shape:

- define the open-floor schedule, labels, artifacts, and validations
- execute through `MeasurementModeCore`

### `DiagnosticController`

If this mode is deleted:

- remove it once `OpenFloorMeasurement` is confirmed as the authoritative diagnostic path
  Destination: deletion

If this mode is retained as a distinct scenario:

Keep in the mode:

- diagnostic section schedule
- diagnostic result-event vocabulary
- diagnostic-specific metrics and summaries
- diagnostic schema

Move out:

- startup trace, fault registration, and log-session lifecycle
  Destination: `BootUtilityModeFramework`
- timed sample loops, primitive execution, boundary enforcement, and common capture/log mechanics
  Destination: `MeasurementModeCore`
- runtime file naming and generic artifact metadata
  Destination: `RuntimeArtifactSupport`

Resulting mode shape if retained:

- define the diagnostic schedule and result summaries
- execute through `MeasurementModeCore`

### `MissionRun`

Keep in the mode:

- the top-level mission scenario: explore, return, speed run, service cycle, speed run
- mission descriptor

Move out:

- the forwarding wrapper itself
  Destination: deletion; descriptor points directly at the authoritative mission owner
- shared mission-family startup/session mechanics from `Initialize()`
  Destination: `MissionModeCore`
- mission text-log session plumbing
  Destination: `MissionModeCore`
- maze snapshot artifact plumbing
  Destination: `MissionModeCore` plus `RuntimeArtifactSupport`

Substantive mission domain behavior:

- exploration, return-to-start, and racing remain in the authoritative mission owner
  Destination: `MissionModeCore`

Resulting mode shape:

- define the top-level mission scenario
- invoke the authoritative mission execution owner directly

### `ManeuverFileTest`

Keep in the mode:

- scenario identity
- source artifact name such as `test.txt`
- mode-local completion policy

Move out:

- the forwarding wrapper itself
  Destination: deletion
- mission-family startup/session preparation
  Destination: `MissionModeCore`
- telemetry log-session lifecycle
  Destination: `MissionModeCore` plus `BootUtilityModeFramework` where shared utility behavior overlaps
- maneuver-file loading and queue import plumbing
  Destination: `RuntimeArtifactSupport` plus `MissionModeCore`
- queue execution, expected-location snapping, and telemetry phase handling
  Destination: `MissionModeCore`
- common maneuver queue metadata logging
  Destination: `MissionModeCore`

Resulting mode shape:

- declare input artifact and completion steps
- ask `MissionModeCore` to load and execute the queue

### `CorridorRepeatability`

Keep in the mode:

- corridor-repeatability scenario definition
- corridor-specific metadata
- corridor-specific pass schedule and limits

Move out:

- the forwarding wrapper itself
  Destination: deletion
- mission-family startup/session preparation
  Destination: `MissionModeCore`
- telemetry log-session setup and shutdown
  Destination: `MissionModeCore`
- runtime log-file naming
  Destination: `RuntimeArtifactSupport`
- repeated maneuver execution and shared telemetry capture
  Destination: `MissionModeCore`
- common recovery, snapping, and queue execution mechanics
  Destination: `MissionModeCore`

Resulting mode shape:

- define corridor passes and metadata
- execute through `MissionModeCore`

### `PositionAccuracyAudit`

Keep in the mode:

- audit scenario definition
- audit-specific fixture geometry
- audit-specific metadata
- audit-specific validation and result interpretation

Move out:

- the forwarding wrapper itself
  Destination: deletion
- mission-family startup/session preparation
  Destination: `MissionModeCore`
- telemetry log-session setup and shutdown
  Destination: `MissionModeCore`
- runtime log-file naming
  Destination: `RuntimeArtifactSupport`
- repeated maneuver execution, traversal, snapping, and telemetry capture
  Destination: `MissionModeCore`

Resulting mode shape:

- define the fixture, phases, and expected paths
- execute through `MissionModeCore`

## Mission-Family Shared Extraction

The following shared responsibilities are currently spread across `MissionController` and should move once for reuse by `MissionRun`, `ManeuverFileTest`, `CorridorRepeatability`, and `PositionAccuracyAudit`:

- startup/session preparation from `Initialize()`
- startup wall-calibration session handling
- mission-family text-log and telemetry-session lifecycle
- front-wall characterization loading for mission-family consumers
- known-start and known-maze seeding helpers
- maze snapshot output
- queued maneuver execution

Destination:

- `MissionModeCore`
- `RuntimeArtifactSupport` for artifact input/output pieces

## Measurement-Family Shared Extraction

The following shared responsibilities are currently spread across `FrontWallCharacterization`, `AuxMeasurement`, `OpenFloorMeasurement`, and `DiagnosticController`:

- timed sample-loop execution
- common drive/sensor capture plumbing
- common estimator-fault routing
- common phase or section finalization
- reusable recovery and boundary checks
- reusable primitive execution where a measurement scenario uses motion primitives

Destination:

- `MeasurementModeCore`
- `BootUtilityModeFramework` for session/lifecycle pieces

## Final Target Shape

After the migration:

- `BootModeRegistry` chooses the boot mode
- each mode contributes one descriptor and one high-level scenario definition
- `BootUtilityModeFramework` owns shared utility-mode session mechanics
- `MeasurementModeCore` owns shared measurement/calibration execution mechanics
- `MissionModeCore` owns shared mission-family execution mechanics
- `SharedRobotRuntime` remains the sole owner of shared production resources
- modes stop carrying low-level hardware and memory-architecture concerns
