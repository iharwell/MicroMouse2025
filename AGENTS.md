# AGENTS.md

## Purpose

This repository is in active architectural cleanup. Existing code is **not** presumed acceptable merely because it compiles, links, or appears to work.

This policy is intentionally aggressive and temporary. Its job is to force convergence while the repository is being cleaned up. Replace it with a less aggressive steady-state policy once the repository is in acceptable shape.

During cleanup:

1. identify the authoritative owner,
2. move state and behavior to that owner,
3. delete redundant layers, wrappers, and parallel implementations,
4. update callers to the canonical interface,
5. remove dead code and superseded files.

Do **not** preserve nonconforming structure or compatibility shims unless the task explicitly requires staged compatibility.

Preserve buildability, host/Teensy verification paths, and explicitly required external behavior. This repository is nonfunctional in places; current behavior is **not** automatically authoritative merely because it exists.

---

## Rule Priority

Apply these rules in this order:

1. preserve authoritative ownership, buildability, and the ability to verify behavior on both host and Teensy,
2. converge shared runtime resources and mission-relevant behavior into canonical owners,
3. remove duplicate systems, wrappers, scattered redirections, and alternate access paths,
4. keep subsystem shapes centralized and traceable,
5. prefer composition over inheritance,
6. minimize churn only after the architecture is correct.

Working junk is still junk. Do not keep it just because replacing it touches many files.

---

## Canonical Owners

### Vehicle

- `Vehicle` is the root source of truth for robot construction facts, dimensions, locations, physical limits, and fixed subsystem facts.
- Those facts belong in `Vehicle` or in named composed members owned by `Vehicle`, not in copied parameter bags, helper mirrors, or mode-local extracts.
- Prefer direct derivation first. Introduce cached derived state only when a real hot path is identified and the cache remains internal, explicit, and testable.
- Do not expose companion parameter, state, or cache types merely for convenience or presumed performance.

### PlantModel

- `PlantModel` is the single source of truth for the robot motion model and shared plant equations.
- `PlantModel` must not become a second owner of vehicle facts already owned by `Vehicle`.
- Do not duplicate plant equations in controllers, helpers, facades, or mode-specific files.

### Maze

- `Maze` defines the authoritative maze representation and maze-domain behavior.
- Maze topology, wall state, reachability, accessibility, and maze-derived legality belong with `Maze`.
- Do **not** create alternate maze representations, helper mirrors, or mode-local copies.

### SharedRobotRuntime

- In production runtime, `SharedRobotRuntime` owns every shared single-instance subsystem that must not be copied.
- That includes the only production `MmLogLogger` instance, the `logging.txt` output, the canonical production `Maze` instance, the only production instances of all pathfinder implementations and variants including `FloodFill` and `ManeuverPathfinder`, and other similarly memory-constrained shared navigation/runtime subsystems.
- The production `MmLogLogger` may close one file and reopen another file with a different designated schema when required, but that still uses the same one runtime-owned logger instance. Do **not** create a second production `MmLogLogger`.
- Pathfinder ownership is intentionally absolute in production code. Do **not** create additional pathfinder instances as mode members, helper members, locals, file-statics, hidden caches, or dynamic allocations. These pathfinders are large enough that duplicate production instances are architectural and memory-allocation failures, not conveniences.
- No per-mode, per-helper, per-subsystem, or wrapper-hidden copies are allowed in production code.
- Access these subsystems through `SharedRobotRuntime`; do **not** create alternate managers, access helpers, wrappers, convenience owners, or on-demand construction paths.
- `Maze` remains the authoritative maze-domain type. `SharedRobotRuntime` owns the production instance.
- Host-side test architecture may use its own test logging system, but that does not create a second production ownership pattern.

### BootModeRegistry

- `BootModeRegistry` is the single source of truth for top-level boot-mode discovery and selection metadata.
- If it does not yet exist in acceptable form, any task touching boot-mode selection must introduce or converge on it rather than inventing another registry, selector helper, or parallel authority.
- Boot selector pins, jumpers, and startup-entry hardware conditions belong here or in the platform pin map it owns or references, not in mode tuning or config files.
- `BootModeRegistry` owns selection and discovery metadata only: stable mode identifier, selector condition, reboot requirement, and one authoritative descriptor reference.
- Human-facing purpose summaries, phase descriptions, artifact descriptions, and implementation-location details belong to the authoritative mode descriptor, not duplicated ad hoc across the registry.

### BootUtilityModeFramework

- `BootUtilityModeFramework` is the intended authoritative home for shared boot-mode execution helpers and contracts when such shared infrastructure is justified. The name is historical; it does **not** imply that mission mode is a different architectural species.
- It does **not** need to be introduced preemptively for a single mode.
- If a task must introduce or substantially extend shared boot-mode/session infrastructure, it must converge on `BootUtilityModeFramework` rather than creating another host contract, setup layer, helper family, or provisional registry/context abstraction.
- Shared setup/teardown, runtime access, logging lifecycle, failure handling, control-tick capture, and recovery helpers belong here once they are truly shared.
- Until it exists in acceptable form, keep boot-mode logic direct and local rather than inventing temporary shared layers.

### SoftwareLimits

- Software-induced limits belong in `SoftwareLimits` or in one typed configuration object owned by `SoftwareLimits` and shared across modes.
- Safety bounds, stop conditions, and shared runtime limits must use this typed ownership rather than scattered `stop if X` directives or mode-local constant bags.

---

## Cross-Build Boundary

- This project has one dual-build boundary: host testing and Teensy deployment.
- Platform-specific redirection of embedded functions to feasible host-side equivalents must be centralized in one designated build boundary, currently `defines.h` unless replaced by one explicit authoritative subsystem.
- Do **not** scatter thin host adapters, duplicate wrappers, `#ifdef` facades, or alternate platform shims throughout mode code or subsystem code.
- If the centralized boundary must grow, extend that one boundary rather than creating local redirection layers.
- Any boundary type introduced for this purpose must be minimal, centrally owned, and reused. It must not become a second architecture layered on top of the real subsystem owners.

---

## Public API and Type Shape Rules

### Default shape

For each substantive subsystem, expose **one authoritative public owner**.

Supporting public types are allowed only when they are one of the following:

1. a stable domain-vocabulary subsystem,
2. a designated `mmlog` row schema declared with the project macros, or
3. part of the single centralized host/target boundary or one centrally owned framework contract.

If callers need more operations, add domain methods to the owner or move behavior inward. Do **not** publish a second public concept merely to expose, rename, or partially extract the owner's internals.

### Forbidden default shapes

Do **not** introduce or preserve:

- `Facade` + `Core` + `Helper` + `Manager` + `Utils` around one subsystem,
- a class whose public API mostly forwards to one owned object,
- multiple overlapping public entry points into the same responsibility,
- public support-type families that mainly expose one class's internal pipeline,
- wrapper classes introduced only to reduce local edits,
- one-off helper dumps or generalized convenience layers,
- parallel systems with slightly different naming or access patterns,
- compatibility wrappers kept after the canonical pattern already exists,
- copy-and-fork architecture for new modes or subsystems,
- companion `Params`, `State`, `Context`, `Data`, or similar public structs created mainly to simplify access, refactoring, or testing for one substantive owner,
- scattered thin adapters or `#ifdef` wrappers outside the centralized cross-build boundary.

### Supporting types

A public supporting type is acceptable only if all of the following are true unless it is an `mmlog` schema or centralized framework/boundary contract:

- it is a stable domain concept rather than an implementation artifact,
- it would still make sense if the current algorithm were rewritten,
- it has broad cross-subsystem use and its own meaningful operations or ecosystem,
- it does not compete with an existing authoritative owner,
- it does not merely expose one owner's internals through a smaller surface area.

If a type mainly supports one class's implementation, keep it private, nested, or file-local in the `.cpp`.

Public fields are disallowed by default. They are acceptable only for:

- designated `mmlog` row schemas,
- true domain-vocabulary/value types with stable semantics and broad reuse.

They are **not** acceptable for peeled-off internals, parameter bags, cache exposure, or testing conveniences.

---

## Configuration and Limits

Configuration must have one authoritative ownership hierarchy.

- Do **not** create a new mode by copying an existing `*Config` namespace, file, or constant block.
- Shared settings must be owned once as typed configuration objects.
- Mode-specific settings must be compact profiles or explicit overrides.
- New modes must reuse shared typed groups and define only real deltas.
- Mode-specific safety bounds must use the shared typed `SoftwareLimits` ownership.
- Boot selector pins, jumper conditions, and startup-entry hardware conditions belong to `BootModeRegistry` or the platform pin map, not to mode tuning or config ownership.

Reject any configuration architecture in which the production parameter set cannot be instantiated, inspected, and validated directly in unit tests through its authoritative typed owner.

Do **not** use copied config namespaces, namespace aliases, scattered top-level constants, preprocessor switches, hidden file-local config state, or mode-specific forks as substitute ownership paths.

When adding a new mode:

1. do **not** copy another mode's config file or namespace,
2. reuse existing shared settings,
3. define only a compact profile for mode-specific deltas,
4. extract a shared typed group once if multiple modes truly share it,
5. update callers to use the shared owner rather than introducing another access path.

---

## Boot-Selected Execution Modes

Top-level application modes are selected only at startup by reading designated mode-select pins or jumpers. Once selected, the application stays in that one top-level mode for the session.

- Do **not** model top-level modes as a runtime mode machine.
- Do **not** implement runtime transitions between top-level modes.
- Entering a different top-level mode requires a reboot.
- Internal setup, calibration stages, sections, subtests, and recovery paths are **phases** of one mode, not separate modes.
- The selected top-level mode owner is the only place allowed to start the active `LoopController` session for that mode.
- Starting that session means the mode is now active; ending that session means the mode is shutting down.
- Subordinate controllers, helpers, and phase executors must not start nested `LoopController` sessions.
- While the session is active, only the currently installed loop callback runs and can steer flow, request a pause, end the session, or swap callbacks.
- `Routine` means a callback-driven `LoopController` procedure. Do **not** introduce synchronous/blocking routine wrappers that hide that callback ownership model.
- Any routine that does not explicitly request a pause must return control to `LoopController` before the provided deadline for that tick.
- If `LoopController` is paused, the robot must not move. Pause handling is for non-motion work only unless an explicit task says otherwise.
- Outside a `LoopController::RequestPause(...)` callback, mode code must not wait for another tick, sleep for control progress, or spin on control-state changes.

### Mode categories

- `mission mode`: a boot-selected mode whose scenario is normal maze operation. This is a scenario label, not a separate architectural class, and the mission mode may be smaller than many test or measurement modes because only heavily verified behavior should live there.
- `utility mode`: a boot-selected mode whose scenario is measurement, calibration, audit, characterization, or bring-up. This is also a scenario label, not a lighter-weight architectural class.

Both labels follow the same structural rules. A boot-selected mode may use one class per mode only when that pattern is applied consistently and the mode uses `BootUtilityModeFramework` for shared setup, runtime access, logging, and teardown where that sharing is truly justified.

### Boot-mode construction rule

A direct boot-selected mode should normally consist of:

- one `BootModeRegistry` entry,
- one authoritative descriptor,
- one implementation file or one easy-to-find mode class,
- optional small mode-specific callbacks only where `BootUtilityModeFramework` cannot express the behavior directly,
- shared use of canonical runtime, logging, control, sensing, recovery, and watchdog infrastructure.

Editing a boot-selected mode should usually require touching only:

- `BootModeRegistry`,
- the mode's authoritative implementation files,

and, when genuinely needed:

- the mode's unit test file,
- mode-local `mmlog` schema declarations in the mode header,
- shared typed tuning or `SoftwareLimits`,
- shared framework-owned descriptor or context types when genuinely reused,
- build metadata or non-code assets the mode or tooling genuinely consumes.

It should **not** require new per-mode host interfaces, config namespaces, wrappers, logger ownership patterns, mode-specific label bundles, mode-specific cycle bundles, or scattered infrastructure families.

### Registry and descriptor rule

All top-level boot-selectable modes must be declared in one authoritative `BootModeRegistry`.

For every boot mode, `BootModeRegistry` must contain only the selection/discovery metadata needed to find and enter the mode:

- stable mode identifier or display name,
- selector pins or jumper condition,
- whether reboot is required to enter a different top-level mode,
- one authoritative descriptor reference.

Each boot mode must define one authoritative descriptor, colocated with the mode implementation or its header. The descriptor owns the human-facing and implementation-facing mode description. It must contain:

- short purpose summary,
- primary outputs or logs produced,
- the implementation entry point or callable,
- authoritative implementation file location,
- when meaningful: major phases or sections,
- when meaningful: shared tuning relied upon,
- when meaningful: any explicit tuning overrides,
- when meaningful: expected artifacts produced.

Do **not** duplicate selector metadata in descriptors or descriptive metadata in the registry beyond what the registry minimally needs for discovery.

Do **not** duplicate boot conditions or mode metadata across comments, serial strings, scattered config files, or per-mode trivia.

A reviewer must be able to inspect `BootModeRegistry` to find every boot-selectable mode and its hardware entry condition, then inspect one descriptor to learn what that mode does and where its implementation lives.

### Shared framework introduction rule

`BootUtilityModeFramework` becomes authoritative only when shared boot-mode/session infrastructure is deliberately introduced or substantially extended.

If a change must introduce or substantially extend shared boot-mode/session infrastructure, define in the plan or task notes:

1. the authoritative owner,
2. the common contract and shared responsibilities,
3. which duplicated per-mode infrastructure it replaces,
4. which files become authoritative and which are expected to shrink or disappear,
5. which existing or planned modes will use it.

Do **not** create multiple provisional registries, contexts, wrappers, logger layers, or dispatch abstractions without this plan. Prefer one convergent framework introduction over iterative framework sprawl.
Introduce or extend shared boot-mode/session infrastructure only when it replaces duplicated per-mode machinery across multiple existing modes, or across one existing mode and at least one clearly planned additional mode, or when the task explicitly makes framework introduction a primary objective. Do **not** create shared framework layers opportunistically for a single mode when direct mode-local code still satisfies the canonical-owner rules.

### Mode interface, logging, and tuning

- Do **not** encode the set of modes into host interface method names such as `BeginXMode()` or `RunXMode()`.
- Do **not** create one forwarding wrapper per mode.
- If boot-selected modes use one class per mode, they must all conform to one common contract and be registered through `BootModeRegistry`.
- Pause callbacks must stay small and phase-specific. Do **not** build a catch-all pause-dispatch hub that re-routes unrelated behavior.
- In production runtime, boot-selected modes must use only the shared logging architecture: the one runtime-owned `MmLogLogger` instance for structured data, `logging.txt` for sparse human-readable text, and other runtime-owned logging objects.
- Boot-selected modes may declare `mmlog` row schemas with the designated macros, including separate schemas for different internal phases.
- Separate phase schemas are allowed only through the designated macros and only when bound through the one runtime-owned `MmLogLogger` instance by closing/reopening or otherwise reconfiguring that same logger according to the shared runtime logging architecture.
- Boot-selected modes may **not** own additional `MmLogLogger` instances, export objects, file-export objects, event-log objects, or alternate logging subsystems directly.
- Boot-selected modes should use shared runtime tuning by default.
- Allowed mode-specific parameters are limited to geometry, repetition counts, workspace limits, capture cadence, labels/metadata, safety bounds expressed through `SoftwareLimits`, and explicit documented experimental overrides.
- Any tuning override must be explicit, minimal, local, and documented with why shared mission/runtime tuning is insufficient.

---

## Cleanup Workflow and Deletion Policy

Deletion is preferred over preservation when a type, file, or subsystem is:

- redundant,
- a thin wrapper,
- a second access path to the same behavior,
- an obsolete implementation of a centralized responsibility,
- a nearly identical duplicate,
- a compatibility layer preserving a nonstandard pattern.

Do **not** leave old and new systems side by side unless the task explicitly requires staged compatibility.

### Before writing code

1. identify the authoritative owner,
2. identify all wrappers, helpers, configs, and alternate access paths involved,
3. choose the single canonical destination,
4. confirm the task boundary and `done when` condition,
5. plan caller migration,
6. plan deletion of superseded code,
7. if the task truly requires new or expanded shared boot-mode infrastructure, complete the framework-introduction rule first,
8. if host/Teensy behavior differs, decide whether the difference belongs in the centralized build boundary rather than the subsystem itself.

### During implementation

- extend the authoritative owner rather than creating a parallel type,
- move behavior inward toward the owner,
- keep non-vocabulary helpers private or file-local,
- prefer one authoritative class with private helpers over several thin cooperating classes,
- do **not** fix an ownership problem at the call site when the owner is known,
- do **not** broaden the task beyond its boundary unless convergence requires it,
- delete superseded code in the same change when feasible.

### After implementation

- remove dead declarations, includes, and obsolete files from the build,
- ensure only the canonical access path remains,
- ensure tests hit the canonical path,
- ensure configuration still resolves through authoritative typed owners,
- ensure every edited file includes its own direct dependencies,
- ensure every edited non-template `.cpp` includes its own header first after the project precompiled header if one is required.

---

## Automatic Rejection Checks

Reject the design and revise it if any of the following are true:

- a new class mostly forwards to, renames, or repackages another class,
- a subsystem ends up with multiple overlapping public entry points,
- a public type mainly represents one class's internal pipeline or peeled-off internals,
- old and new implementations remain in parallel after the change,
- a wrapper or adapter is introduced only to reduce local edits, testing effort, or mock setup,
- a new mode requires copied config, a new per-mode host interface, a forwarding wrapper, or a new logging architecture,
- a boot condition or selector rule is hidden outside `BootModeRegistry` or the platform pin map,
- selector metadata and descriptive mode metadata are duplicated instead of split cleanly between `BootModeRegistry` and the authoritative descriptor,
- shared tuning diverges without explicit experimental justification,
- the architecture implies runtime switching between top-level modes,
- a new host/Teensy redirection is scattered outside the centralized build boundary,
- public fields are introduced outside domain vocabulary or designated `mmlog` row schemas,
- inheritance grows where composition or a flatter contract would suffice,
- production code creates another `MmLogLogger` instance or another production pathfinder instance outside `SharedRobotRuntime`,
- production code dynamically allocates a large shared pathfinder instead of using the runtime-owned instance,
- shared boot-mode/session infrastructure is introduced or widened without satisfying the shared framework introduction rule,
- a file builds only because of incidental transitive includes or forwarding headers.

---

## File, Header, and Inheritance Rules

- Headers must be self-sufficient for the declarations they expose.
- Do **not** rely on incidental transitive includes, unrelated include order, or current build-graph accidents.
- Include every direct dependency needed by the file's declarations, inline definitions, templates, and constants.
- Use forward declarations only when they are truly sufficient and stable.
- Every edited non-template `.cpp` must include its own header first after the project precompiled header if one is required.
- Substantive classes must live in same-named authoritative files.
- Do **not** create or preserve alias headers, forwarding headers, or fake entry headers that only include another file while pretending to define a separate subsystem.
- Prefer private helpers, nested types, and file-local functions over fake public decomposition.
- Prefer direct concrete owners and composition.
- Allow inheritance only for narrow interface contracts, unavoidable framework/toolchain integration, or one shallow abstraction layer with obvious substitutability.
- Flatten lopsided or disparate inheritance trees that make call tracing harder than the equivalent composed design.
- Do **not** introduce multi-level categorization trees, speculative base classes, or convenience inheritance.

---

## Testing Rules

- Existing tests should not be modified merely to preserve a noncanonical design.
- Tests may be updated when needed to reach the canonical architecture or reflect an intentional behavior change.
- New architecture must preserve or improve direct test access to authoritative owners.
- Tests must be able to construct, inspect, and compare the exact parameter sets used by production code.
- Do **not** hide production behavior behind wrappers, aliases, or mode forks that tests must special-case.
- Do **not** introduce wrappers, facades, adapters, or companion structs solely to make testing easier.
- Host-side redirection of embedded functions is allowed only through the centralized cross-build boundary.
- Host-side test architecture may use its own logging system.

---

## Clarification and Scope

- Ask concise clarifying questions only when ownership, required behavior, or interface contract is genuinely ambiguous.
- Do **not** ask questions merely to avoid cleanup work.
- If the authoritative owner is clear, extend it rather than inventing a new layer.
- If a multi-step cleanup contains minor ambiguity, state the working assumption and proceed without creating a new public layer.
- Follow the explicit task request and its stated or implied `done when` condition.
- Do **not** expand the change set to nearby systems for speculative consistency or hypothetical reuse.
- If neighboring cleanup is truly required to preserve authoritative ownership, perform only the minimum extra work needed and keep the migration convergent.

---

## Project-Specific Instructions

### Operational constraints

- No watchdog timers under 60 seconds that trigger a run failure.
- Prefer recovery over fail-fast behavior.
- When runtime behavior deviates from expectation, log the condition before attempting recovery.
- The robot can sustain approximately `16.5 m/s^2` of lateral acceleration when the fan is running at `80%`.
- Plan strategies with the high-performance operating envelope in mind.
- Use the Decimus 5A project as guidance for intended style and performance envelope.
- Directional code must respect `+X = right`, `+Y = forward/up`, and `+Yaw = clockwise`.
- Do **not** introduce alternate access patterns, public fields, or companion structs based on presumed performance benefit. First establish correctness, ownership, and clean architecture. Optimize representation only when the hot path is known and the simpler encapsulated form is materially insufficient.

### Navigation and locomotion

- Use `FloodFill` for simple navigation.
- Use `ManeuverPathfinder` only while stationary.
- In a maze, locomotion should prefer the `Maneuver` classes.
- For more manual control, generate `ManeuverInstance` objects directly and follow with a small target-yaw PID if needed.
- `ManeuverInstance` is the canonical execution vocabulary for maneuver-driven motion. Do **not** peel maneuver execution back out into parallel smooth-turn profile structs or mode-local geometry bags.
- Derive maneuver execution facts from `ManeuverSet`, `ManeuverCode`, and `Maze::GetCellDimension()` at the point of use or through methods on the canonical maneuver owners.
- `ManeuverPoint` is the per-point drive primitive for higher-level motion execution.
- Mission-mode code should choose goals, replans, and phase transitions. Shared drive execution owners should own the actual motion primitives and per-tick maneuver tracking.
- `DriveBase` is the destination for concrete "move in this manner" commands. If a higher-level drive owner is introduced, it must become the authoritative motion owner rather than a forwarding facade over `DriveBase`.
- While motion is active, only the current `LoopController` callback runs. Mapping, pathfinding response, maneuver dispatch, and phase progression that must happen during motion must therefore be callback-driven or live in shared services called by that callback.
- A `Routine` in the maneuver/motion vocabulary is callback-driven work owned by the active `LoopController` flow, following the `ManeuverExecutor` pattern. Do **not** wrap motion routines in synchronous/blocking helper APIs.
- If the loop is paused, motion must be halted; pauses are not an excuse to keep driving in parallel with non-real-time work.
- Outside `LoopController::RequestPause(...)` callbacks, do **not** wait for "one more tick", sleep for control progress, or spin on control-state changes.
- Open-loop commands are appropriate for low-level tasks such as wall tapping or certain measurements.
- Reserve direct position or yaw control for tasks that cannot reasonably be expressed through maneuver-based control.

### Logging and output

- Serial-style output should be sparse and should go to `logging.txt`.
- Treat `logging.txt` as runtime infrastructure, not as a mode-owned file. Mode code should write to it through `SharedRobotRuntime`, but should never explicitly open or close it. It is expected to be available from boot through shutdown unless someone deliberately interferes with the runtime-owned logging pipeline, which is roughly equivalent to breaking `Serial`, `printf`, or `std::cout`.
- Heavy data logging should use `mmlog` and the designated macros.
- Do **not** create parallel ad hoc logging systems.

### Build

- When building, use `build_and_verify_latest.cmd` or `build_and_verify_latest.ps1`.
- If `build_and_verify_latest` reports `HOST_INTERMEDIATE_STATE_BROKEN`, stop immediately. Treat missing host-side Release intermediates as a broken incremental build state caused by prior artifact deletion; do not "fix" it with `Clean`, `Rebuild`, or more deletion. Human intervention is required.

---

## Preferred Cleanup Outcome

A good cleanup change should usually produce:

- one clear authority for the edited responsibility,
- fewer public entry points,
- fewer wrappers, helper files, and alternate access paths,
- fewer duplicate parameters or config owners,
- easier unit testing through canonical owners,
- easier discovery of every boot mode and its entry condition,
- stronger include hygiene,
- more consistent boot-mode setup, logging, and self-description.

If a change increases the number of layers, config owners, wrappers, or parallel patterns, it is probably wrong.

---

## Deprecated Files

- `RuntimeBinaryLogSupport.h`
- `CoreBinaryFileExport.h`
- `CoreBinaryFileExport.cpp`
- `OptionalRuntimeEventLog.h`
- `OptionalRuntimeEventLog.cpp`
