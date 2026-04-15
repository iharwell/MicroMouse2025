# Boot Mode Infrastructure Spec

## Purpose

- Define the boot-mode architecture needed to make boot modes high-level, scenario-definition code.
- Centralize low-level hardware, logging, recovery, timing, and memory-sensitive behavior into shared authoritative owners.
- Keep the design compliant with `AGENTS.md`: one clear owner per responsibility, no forwarding-wrapper architecture, no duplicate shared subsystems.
- For the concrete authoring checklist when adding a new mode, see `boot_mode_setup_guide.md`.

## Core Requirement

After this refactor, a boot mode should mostly describe:

- what scenario it runs
- what sections or phases it contains
- what artifacts it produces
- what limits, fixtures, and mode-specific callbacks it needs

A boot mode should **not** directly own:

- boot pin reads
- low-level hardware bring-up or teardown
- logger session lifecycle
- fault-handler registration
- watchdog and timeout plumbing
- repeated recovery loops
- repeated primitive execution loops
- memory-sensitive shared resource ownership

The target is not "small wrappers." The target is "small authoritative scenario definitions over shared execution cores."

## Current Boot Mode Inventory

Current startup precedence:

1. `front_wall_characterization`
2. `wall_sensor_led_calibration`
3. `auxiliary_measurement_selector`
4. `maneuver_file_test`
5. `primary_diagnostic`
6. `mission`

Current boot-selectable entries:

| Effective id today | Selector today | Current target | Current issue |
| --- | --- | --- | --- |
| `front_wall_characterization` | pins `39-40` | `FrontWallCharacterizationController` | standalone mode |
| `wall_sensor_led_calibration` | pins `38-39` | `WallSensorLedCalibrationController` | standalone mode |
| `auxiliary_measurement_selector` | pins `28-29` | depends on `AuxMeasurementConfig::kRoutine` | hidden remap, not stable identity |
| `maneuver_file_test` | pins `29-30` | wrapper -> `MissionController` | selector is inline, wrapper architecture |
| `primary_diagnostic` | pins `27-28` | `OpenFloorMeasurementController` via `GetDiagnosticMode()` | name does not match implementation |
| `mission` | fallback | wrapper -> `MissionController` | wrapper architecture |

Current architectural problems that the infrastructure must resolve:

- selection metadata is scattered
- selector precedence is encoded in if-chains
- mode identity is ambiguous for auxiliary and diagnostic paths
- mission-side boot modes are wrappers over `IMissionModeHost`
- utility modes duplicate setup, logging, recovery, and fault plumbing

## Architectural Principles

### 1. Modes Are High-Level Only

Boot modes are scenario owners, not low-level runtime owners.

A mode may define:

- descriptor
- scenario schedule
- mode-local vocabulary
- mode-local geometry or fixtures
- mode-local schemas
- mode-local validation and callbacks

A mode should not manipulate low-level runtime architecture directly.

### 2. Low-Level Concepts Are Centralized

Hardware-facing and memory-sensitive concepts belong in shared infrastructure, not in individual modes.

That includes:

- selector-pin evaluation
- hardware setup and teardown
- logger lifetime and file naming
- shared `mmlog` session management
- startup trace plumbing
- fault routing
- watchdog policy
- recovery policy
- repeated primitive execution
- control-cycle capture
- host/target boundary behavior

### 3. Shared Runtime Ownership Stays Canonical

`SharedRobotRuntime` remains the production owner for shared single-instance resources such as:

- the production `Maze`
- the production pathfinders
- the production `MmLogLogger`
- `logging.txt`
- shared drive and sensor services

Modes and frameworks borrow these services. They do not create alternate owners.

### 4. Scenario Definition Must Be Typed

The scenario-authoring surface should be typed C++, not:

- a generic manager stack
- a second registry layer
- a string DSL
- a reflection-based dispatcher
- a public `Context` or `Params` bag that competes with real owners

## Required Infrastructure

### `BootModeRegistry`

`BootModeRegistry` is the single source of truth for top-level boot-mode discovery.

It owns only:

- stable mode id
- selector condition
- selection precedence
- reboot-required metadata
- descriptor reference

It must let consumers:

- enumerate all boot modes
- inspect precedence
- inspect selector conditions
- resolve the selected boot mode through one centralized path

It must not own:

- mode behavior
- mode-local tuning
- mode-local geometry
- logging mechanics

### `BootModeDescriptor`

Each boot mode defines one authoritative descriptor, colocated with the mode.

The descriptor owns:

- stable mode id
- mode category: mission or utility
- short purpose summary
- artifacts or logs produced
- authoritative entry point
- authoritative implementation file location
- major sections or phases for utility modes
- shared tuning relied upon
- explicit tuning overrides

This descriptive metadata must stop living in ad hoc comments and text-log strings.

### Registry-Driven Launch Path

`MazeMapApplication.cpp` should launch through the registry and descriptor only.

It should no longer own:

- `StartupModeRequests`
- the startup precedence chain
- the startup-mode switch
- hidden remap logic

### `BootUtilityModeFramework`

`BootUtilityModeFramework` is the shared session/lifecycle surface for utility modes.

It should own only shared utility-mode mechanics such as:

- startup trace begin/append
- mode fault registration
- canonical failure routing
- text-log event, metadata, and phase helpers
- utility-data-log session open/begin/service/flush/close
- logger failure capture
- timeout normalization and watchdog policy
- common recovery flow
- selected-mode identity and selector provenance for logging

It must not become:

- a second architecture
- a giant generic DSL engine
- a new owner of maze, pathfinders, or logging subsystems
- a place where mode-local schemas, geometry, or schedules get centralized

### Reusable Execution Cores

Where multiple modes share the same execution mechanics, those mechanics should move into one authoritative reusable execution core for that family.

Examples of family-level reusable features:

- stationary hold execution
- marker-based pose seeding
- traverse-to-marker
- recover-to-marker
- open-loop pulse execution
- straight-distance execution
- in-place turn execution
- smooth-turn execution
- control-cycle capture
- per-cycle telemetry logging
- structured section fault finalization
- artifact-session transitions

These should be reusable because they are common mechanics, not because the design wants another abstraction layer.

### Shared Runtime Boundary

Low-level host/target differences must remain centralized in the build boundary or authoritative runtime owners.

Modes should not grow:

- direct pin probing logic
- scattered `#ifdef` shims
- file-system naming helpers
- low-level timing capture code

## Consumer Contract

### What a Mode Author Should Be Able To Do

A mode author should be able to write high-level code that:

- declares ordered sections or phases
- declares repeats and artifacts
- uses shared markers, poses, or fixtures where appropriate
- chooses reusable primitives
- sets mode-local limits and validation hooks
- adds the small amount of mode-specific behavior that is truly unique

The code should read like a scenario definition, not like a hardware driver or control-loop harness.

### What a Mode Author Should Not Need To Do

A mode author should not need to:

- read selector pins
- open or name log files
- manage the logger lifecycle
- manually wire fault handlers
- implement standard watchdog plumbing
- hand-roll repeated recovery loops
- implement repeated control-cycle capture loops
- manage shared resource lifetime or memory layout

## What Must Move Out Of Mode Implementations

### Move into `BootModeRegistry`

- top-level boot-mode inventory
- selector-pin ownership for top-level selection
- selector evaluation
- selector precedence
- reboot-required metadata
- stable launch identity

### Move into `BootModeDescriptor`

- purpose summary
- artifact list
- authoritative entry point
- implementation file location
- major phases
- shared tuning references
- explicit overrides

### Move into `BootUtilityModeFramework`

- startup trace plumbing
- fault registration
- canonical fail path
- text-log metadata and phase writing
- utility-data-log session lifecycle
- logger failure capture
- timeout normalization
- shared recovery/session policy

### Move into Reusable Execution Cores

- common section sequencing mechanics
- repeated primitive execution loops
- repeated capture-and-log loops
- common marker navigation
- common recovery loops
- common section success/fault finalization

### Move into an Authoritative Runtime Owner

- sequential runtime log-file naming
- sibling-file naming
- shared metadata formatting helpers
- low-level access to the runtime-owned text log and `mmlog` logger

## What Must Stay In Modes

The following stay in the authoritative mode implementation:

- the mode descriptor
- the scenario schedule
- the mode's vocabulary
- mode-local geometry and fixtures
- mode-local schemas
- mode-local labels and artifact-specific metadata
- mode-local validation and callbacks

This keeps the mode authoritative without forcing it to own low-level machinery.

## Open-Floor Implication

For a large mode such as open-floor, the end-state mode file should shrink toward:

- descriptor
- open-floor-specific schedule
- open-floor-specific labels, markers, fault vocabulary, and artifacts
- open-floor-specific validation and callbacks
- one call into a reusable measurement execution core

What must leave open-floor to make that possible:

- control-cycle capture plumbing
- repeated marker traversal and recovery mechanics
- repeated primitive execution loops
- timing/main log session management
- repeated fault formatting and session finalization

Those are reusable mechanics and should not remain hidden inside one mode.

## Implementation Order

1. Introduce `BootModeRegistry` with the current inventory and current precedence.
2. Add one descriptor per current boot-selected mode.
3. Replace the current startup if-chain and switch with registry-driven launch.
4. Move selector-pin ownership into the registry or a registry-owned pin map.
5. Introduce `BootUtilityModeFramework` for shared session/lifecycle mechanics.
6. Extract the first real reusable execution core for one family of scenario-style modes.
7. Move one substantive mode onto the scenario-definition shape.
8. Remove mission-side forwarding wrappers and `IMissionModeHost` mode-list expansion.
9. Resolve the auxiliary hidden remap into an explicit architecture.
10. Resolve the disconnected `DiagnosticController` by deletion or by giving it a real distinct registry entry.

## Done When

This work is done when:

- boot discovery and precedence are owned only by `BootModeRegistry`
- every boot-selectable mode has one authoritative descriptor
- utility-mode lifecycle code is centralized in `BootUtilityModeFramework`
- shared execution mechanics live in one or a few reusable execution cores
- boot modes can be written primarily as high-level scenario definitions
- boot modes do not directly own low-level hardware or memory-architecture concerns
- `SharedRobotRuntime` remains the sole owner of shared production resources
- forwarding-wrapper boot modes are gone
- hidden boot identity remaps are gone or made explicit
