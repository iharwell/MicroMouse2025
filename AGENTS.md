# AGENTS.md

## Purpose

This project is in active architectural cleanup. Existing code is **not** presumed acceptable merely because it compiles, links, or appears to work.

For cleanup tasks, architectural conformance takes priority over minimizing churn.

When existing code conflicts with project architecture:

1. identify the authoritative owner,
2. move state and behavior to that owner or its proper subsystem,
3. delete redundant layers, wrappers, and parallel implementations,
4. update callers to the canonical interface,
5. remove dead code and superseded files.

Do **not** preserve nonconforming structure for compatibility unless the prompt explicitly requires compatibility.

---

## Rule Priority

Apply these rules in this order:

1. preserve authoritative ownership and source-of-truth boundaries,
2. remove duplicate systems, wrappers, and alternate access patterns,
3. keep subsystem shapes coherent and testable,
4. prefer composition over inheritance,
5. minimize churn only after the architecture is correct.

Working junk is still junk. Do not keep it just because replacing it touches many files.

---

## Authoritative Owners

### Vehicle

- Unchanging vehicle facts must be owned by `Vehicle` or by composed members owned by `Vehicle`.
- `Vehicle` is the root source of truth for robot construction facts, dimensions, locations, physical limits, and fixed subsystem facts.
- These facts should be arranged into composed members divided by physical subsystem.
- Consumers should depend only on the relevant subsystem facts rather than on the whole `Vehicle` when practical for testing.
- Derived facts that are expensive or repeatedly used should be precalculated and updated rather than recomputed ad hoc.
- Core facts must have one exclusive owner in this hierarchy. Do not create independent duplicate sources elsewhere.
- Performance-critical access must remain tightly contained inside the authoritative owner. Prefer internal caches with strict update rules over exposing public companion parameter or state types.

### PlantModel

- `PlantModel` is the single source of truth for the robot motion model and shared plant equations.
- `PlantModel` must not become a second owner of vehicle facts already owned by `Vehicle`.
- `PlantModel` should provide shared motion equations, state propagation, and inverse/forward motion relationships.
- Do not create parallel plant-equation implementations in controllers, facades, helpers, or mode-specific files.

### Maze

- `Maze` is the authority on maze layout and maze state.
- Maze topology, wall state, reachability, accessibility, and maze-derived navigation legality belong with `Maze` or with tightly related maze-domain types.

### SharedRobotRuntime

- `SharedRobotRuntime` provides the only legal logging instances in the project through `GetDataLogger()` and `GetLoggingFile()`.
- No other class may own logger or logging file instances independently.
- Additional structured log streams are allowed only when justified and must still be created and owned through `SharedRobotRuntime` or its designated shared logging subsystem rather than by per-mode classes.
- Do not create alternate logger access helpers, wrappers, managers, or utility functions outside those owned instances.

### BootModeRegistry

- `BootModeRegistry` (or the designated equivalent once introduced) is the single source of truth for top-level boot mode discovery and selection metadata.
- Boot selector pins, jumpers, and startup-entry hardware conditions belong to this registry or to the platform pin map it owns or references, not to mode tuning or configuration profiles.
- Every top-level boot mode must be discoverable from this one registry, including its selector condition, purpose summary, primary outputs or logs, implementation entry point, and the file or files where its implementation lives.
- Do not duplicate boot selection logic in scattered mode files, comments, serial-print strings, or config namespaces.

### BootUtilityModeFramework

- `BootUtilityModeFramework` (or the designated equivalent once introduced) is the single owner of shared utility-mode execution helpers and contracts.
- Shared concerns such as utility-mode setup and teardown, runtime or service access, logging lifecycle, failure handling, common control-tick capture, and common recovery helpers belong in this framework rather than inside each utility mode.
- Until this framework exists, do **not** invent multiple provisional layers. If a task requires introducing it, follow the shared infrastructure planning mandate below and create one convergent framework rather than several ad hoc helpers.
- Do **not** introduce new shared utility-mode framework types unless they replace clear duplication across multiple utility modes or are part of an explicit planned framework introduction with named intended consumers.

### Software Limits

- Software-induced limits belong in a designated class or typed configuration owner common to all modes and configurations.
- Do not scatter such limits across unrelated files or mode-specific constant bags.

---

## Reference Shapes

Use these shapes as the default target.

### Allowed Shape A: Maze-style authoritative class

A substantive subsystem should usually look like `Maze`:

- one authoritative owner of relevant state,
- one coherent public interface expressed in domain terms,
- private storage and private implementation details,
- no public forwarding shell layered on top of the real owner,
- no exposure of internals merely to let callers bypass the parent.

### Allowed Shape B: Direction-style domain vocabulary subsystem

A supporting-type subsystem is acceptable only when it behaves like `Direction`:

- a small, stable project language,
- tightly related types and operations,
- reused broadly across substantively different consumers,
- not just a decomposition of one class's internal calculations.

---

## Disallowed Default Shapes

These are forbidden by default unless the prompt explicitly justifies them and the justification survives review against these rules.

- `Facade` + `Core` + `Helper` + `Manager` + `Utils` around one subsystem.
- A new class whose public API mostly forwards to one owned object.
- Multiple public entry points into the same subsystem with overlapping responsibility.
- Public support-type families that mainly expose one class's internal pipeline.
- Wrapper classes introduced only to reduce local edits.
- One-off helper modules, utility dumps, or generalized convenience layers.
- Parallel systems that are nearly identical but have slightly different naming, access, or behavior.
- Compatibility wrappers that preserve a nonstandard pattern after the canonical pattern already exists.
- Copy-and-fork architecture for new modes or new subsystems.
- Companion `Params`, `State`, `Context`, `Data`, or similar public structs introduced mainly to simplify access, refactoring, or testing for one substantive owner.
- Test seams, adapters, or facades introduced only to make mocking or local edits easier.

---

## One Public Concept Rule

For each substantive subsystem, prefer exactly one of the following:

1. one authoritative public class, or
2. one coherent domain-vocabulary subsystem.

Do not create multiple public concepts for the same responsibility.

A class is forbidden if a substantial portion of its public API exists primarily to forward to, expose, rename, or repackage another owned object.

Do not expose owned subsystem objects through getters unless there is a documented performance or interoperability reason and no cleaner domain API exists.

---

## Supporting Type Policy

Supporting types are allowed only in these cases:

### 1. Domain vocabulary subsystems

The types collectively define a small, stable project language.

Examples of acceptable shape:

- directions,
- relative directions,
- coordinates and locations,
- maneuver codes,
- wall-state vocabulary.

### 2. Narrow transport types

A small input or output record may be used when needed for a public API boundary.

These must be:

- minimal,
- directly justified by that boundary,
- not a decomposition of one class's internal algorithm,
- not a companion container for one class's owned parameters, cached state, or derived values.

### Forbidden supporting-type patterns

Do **not** create public supporting types that mainly:

- expose intermediate calculation stages,
- mirror one class's internal processing steps,
- represent temporary solver stages,
- bundle one class's derivative bookkeeping,
- organize one implementation's private algebra into top-level public structs,
- duplicate facts already owned elsewhere.

If a type mainly supports one class's implementation, keep it:

- private,
- nested, or
- file-local in the `.cpp`.

Before creating a new supporting type, verify all of the following:

- it is a stable domain concept rather than an implementation artifact,
- it would still make sense if the current algorithm were rewritten,
- it already has, or is clearly expected to have, at least three substantively different consumers unless it is fundamental project vocabulary,
- it defines project vocabulary rather than one class's internal pipeline,
- it does not compete with an existing authoritative owner.

If any of these are false, do not create a new public supporting type.

---

## Configuration Architecture

Configuration must have one authoritative ownership hierarchy.

Do **not** create a new mode by copying an existing `*Config` namespace, file, or constant block.

Peer configuration islands such as `CoreConfig`, `DiagnosticConfig`, `RaceConfig`, `CalibrationConfig`, or similar are disallowed by default when they duplicate structure or ownership.

### Required configuration shape

- Shared settings must be owned once.
- Shared parameter groups must be represented as typed configuration objects.
- Mode-specific settings must be represented as compact mode profiles or explicit overrides.
- New modes must reuse shared typed groups and define only true deltas.
- Boot selector pins, jumper conditions, and startup-entry hardware conditions belong to `BootModeRegistry` or the platform pin map, not to mode tuning or configuration profiles.

### Configuration testability rule

Reject any configuration architecture in which the parameter set used by a production subsystem cannot be instantiated, inspected, and validated directly in unit tests through its authoritative typed owner.

Do **not** use global namespace visibility as a substitute for testability.

Disallow configuration primarily exposed through:

- copied config namespaces,
- namespace aliases used as alternate ownership paths,
- scattered top-level constants,
- preprocessor switches,
- hidden file-local configuration state,
- mode-specific forks of another config file.

Every production mode must resolve to an explicit typed configuration object or profile that tests can obtain without alternate access paths.

### Adding a new mode

When adding a new operating mode:

1. do not copy an existing mode config file or namespace,
2. identify which settings are already shared and reuse them,
3. create only a compact mode profile for values unique to that mode,
4. if the new mode and an existing mode share a parameter group, extract that group once into a shared typed object,
5. update callers to use the shared owner rather than introducing another access path.

---

## Boot-Selected Execution Modes

Top-level application modes in this project are boot-selected execution modes.

A top-level mode is chosen only during startup by reading designated mode-select pins or jumpers. Once startup selection is complete, the application remains in that one selected top-level mode for the rest of the session.

Do **not** design top-level application modes as a navigable runtime mode machine.
Do **not** implement runtime transitions from one top-level application mode to another.
Entering a different top-level application mode requires a reboot.

Within a selected top-level mode, internal steps such as setup, calibration stages, sections, subtests, and recovery paths are phases of that mode, not separate modes.
Do **not** model such internal phases as independent application modes.

### Mode terminology

Use these terms consistently:

- `boot mode`: a top-level execution path selected only at startup,
- `utility mode`: a boot-selected test, measurement, calibration, or audit workflow,
- `mission mode`: the normal operational top-level mode of the robot,
- `phase` / `section` / `subroutine`: an internal step within one boot mode,
- `recovery path`: a control path within one boot mode,
- `configuration profile`: data that tunes one boot mode or one shared subsystem.

Do **not** call internal phases or subroutines separate application modes.

### Boot utility modes vs mission mode

Top-level application modes fall into two categories:

1. `Mission mode`
   - the normal operational mode of the robot,
   - allowed to be a substantive owner with richer internal policy and behavior.

2. `Boot utility modes`
   - test, measurement, calibration, audit, and bring-up workflows selected only at startup,
   - used to tune, validate, or characterize the robot and the mission mode,
   - not intended to form an independent parallel architecture.

Boot utility modes may use one class per mode **if that pattern is applied consistently**.
A utility mode class is acceptable only when:

- it is easy to locate by name,
- it contains primarily mode-specific procedure and intent,
- it uses the shared utility-mode infrastructure for setup, runtime access, logging, and teardown,
- it does not introduce a new mode architecture, host pattern, logging system, or boot-selection mechanism.

Do **not** add per-mode infrastructure.

Mission mode may remain a more substantive owner. Utility modes should usually be lightweight and procedural. The lightweight convenience and shared-framework rules below apply to utility modes by default; mission mode is exempt except for the boot-selection, registry, logging-standard, and no-runtime-switching requirements.

### Utility-mode construction rule

A boot utility mode should normally consist of:

- one mode registration or definition entry,
- one mode class or one small local implementation block when needed for findability,
- optional small mode-specific callbacks only where the shared framework cannot express the behavior directly,
- shared use of the canonical runtime, logging, control, sensing, recovery, and watchdog infrastructure.

Do **not** create a dedicated host-method pair, config namespace, wrapper, or file cluster for a new utility mode unless the mode has genuinely unique architecture that cannot be expressed through the shared framework.

### Utility-mode convenience rule

Adding or editing a non-mission utility mode should usually require changing only:

- one central mode registry or definition file, and
- one mode implementation file when a dedicated mode class is used.

This expectation applies outside deliberate shared-framework introduction tasks covered by the planning mandate below.

Do **not** require a new utility mode to introduce:

- a dedicated host interface expansion,
- a dedicated config namespace,
- a dedicated wrapper,
- a mode-specific logger type,
- a mode-specific label bundle,
- a mode-specific cycle or state bundle,
- or multiple scattered infrastructure files,

unless the prompt explicitly justifies that extra structure. Shared framework-owned descriptor or context types are allowed when they are reused across multiple utility modes.

### Utility-mode auditability rule

Each utility mode must declare its purpose in one compact, easily audited place.

Every utility mode must provide:

- a one-sentence intent,
- the measurement, calibration, or test goal,
- the shared tuning it relies on,
- any explicit tuning overrides,
- the sequence of major phases or sections,
- the expected artifacts produced,
- and the boot condition that selects it.

A reviewer must not need to read the full procedure body to learn what the mode is for.

### Boot mode registry rule

All top-level boot-selectable modes must be declared in one authoritative boot-mode registry.

The registry must contain, for every boot mode:

- mode name,
- selector pins or jumper condition,
- whether reboot is required to enter a different top-level mode,
- short purpose summary,
- primary outputs or logs produced,
- the implementation entry point, and
- the implementation file or files.

Do **not** hide boot conditions in:

- comments,
- serial-print strings,
- scattered config files,
- or per-mode implementation details.

A reviewer must be able to find every boot-selectable mode and its hardware entry condition in one place.

### Shared infrastructure planning mandate

The shared utility-mode framework does not yet exist as a finished canonical subsystem. If a task requires introducing new shared boot-mode or utility-mode infrastructure, treat that as a deliberate architectural change rather than incidental byproduct of adding one mode.

Before creating such infrastructure, first define in the change plan or task notes:

1. the proposed authoritative owner,
2. the common mode contract and shared responsibilities,
3. which duplicated per-mode infrastructure it replaces,
4. which files become authoritative and which files are expected to shrink or disappear,
5. and which existing or planned utility modes will use it.

Do **not** begin by creating multiple provisional registries, contexts, helpers, wrappers, logger layers, or dispatch abstractions without this plan. Prefer one convergent framework introduction over iterative framework sprawl.

Only introduce new shared boot-mode or utility-mode infrastructure when:

- it clearly replaces duplicated per-mode machinery across multiple existing or planned utility modes, or
- the task explicitly calls for framework introduction as a primary objective.

Do **not** create shared framework layers opportunistically for a single mode.

### Mode interface rule

Do **not** encode the set of modes into host interface method names.

Forbidden:

- `BeginXMode()` / `RunXMode()` growth on a host interface,
- one forwarding wrapper class per mode,
- one singleton getter per mode without a central registry.

If utility modes use one class per mode, they must all conform to one common mode contract and be registered through the central boot-mode registry.

### Utility-mode logging rule

Utility modes must use the project-standard logging architecture only:

- `mmlog` for structured data,
- `logging.txt` for sparse human-readable text,
- shared runtime-owned logging instances and helpers.

Human-readable mode output should go through the shared `logging.txt` path rather than per-mode text files.

Do **not** create per-mode logging subsystems, ad hoc log frameworks, legacy binary-log support, or mode-specific logger ownership patterns.

If a utility mode needs a specialized row schema, define only the row schema and metadata within the shared logging pattern.
Do **not** introduce a separate logging architecture for that mode.

Utility modes may define one or more `mmlog` row schemas, including separate schemas for different internal phases, only when those schemas are declared with the designated `mmlog` macros and remain within the shared runtime-owned logging architecture.
A phase-specific schema is allowed; a phase-specific logging subsystem is not.

Utility modes may define row schemas and request streams, but may not own `MmLogLogger`, file-export, event-log, or other logging objects directly. Structured log streams must be created and owned by the shared runtime logging infrastructure.

If a mode needs more than one structured stream, that must be explicitly justified in the mode description and must still use the shared `mmlog` infrastructure.

### Utility-mode shared tuning rule

Utility modes exist to test, measure, calibrate, or tune the robot and mission behavior.

Therefore, utility modes must use the same shared mission and runtime tuning by default.

Do **not** create separate control, estimator, sensing, drivetrain, or logging tuning families for utility modes unless the explicit purpose of the mode is to characterize or compare those exact parameters.

Allowed utility-mode-specific parameters are limited to:

- test geometry,
- repetition counts,
- workspace limits,
- capture cadence,
- labels and metadata,
- safety bounds,
- and explicit documented experimental overrides.

If a utility mode temporarily overrides shared tuning, the override must be:

- explicit,
- minimal,
- local to the mode definition or implementation,
- and documented with why mission or shared tuning is insufficient for that experiment.

---

## Cleanup and Deletion Policy

Deletion is preferred over preservation when a type, file, or subsystem is:

- redundant,
- a thin wrapper,
- a second access path to the same behavior,
- an obsolete implementation of a centralized responsibility,
- a nearly identical duplicate of another system,
- a compatibility layer preserving a nonstandard pattern.

Do not leave old and new systems side by side.
Do not keep transitional wrappers unless the prompt explicitly requests a staged migration.
Default to completing the migration and removing the old path.
Do not preserve both the legacy path and the canonical path after the change unless the prompt explicitly requires staged compatibility.

---

## Required Workflow for Cleanup Tasks

### Before writing code

1. Identify the authoritative owner for the responsibility being changed.
2. Identify all existing classes, wrappers, helpers, configs, and access paths involved.
3. Choose the single canonical destination for the responsibility.
4. Confirm the explicit task boundaries and `done when` conditions.
5. Plan to migrate touched callers to that destination.
6. Plan deletion of superseded files, wrappers, and duplicate code.
7. If new shared boot-mode or utility-mode infrastructure is required, complete the shared infrastructure planning mandate before creating files or abstractions.

### During implementation

- Extend the authoritative type rather than creating a parallel type.
- Move behavior inward toward the authority.
- Keep helper logic private or file-local when it does not define shared domain vocabulary.
- Prefer one authoritative class with private helpers over splitting behavior across thin cooperating classes.
- Do not fix an ownership problem at the call site when the authoritative owner is known.
- Do not broaden the task beyond the stated `done when` conditions unless a concrete architectural reason requires it.
- Delete superseded code in the same change when feasible.

### After implementation

- remove dead declarations and includes,
- remove obsolete files from the build,
- ensure only the canonical access path remains,
- ensure tests target the canonical path,
- ensure configuration still resolves through authoritative typed owners,
- ensure every new or edited file explicitly includes its own direct dependencies rather than relying on incidental inclusion order,
- ensure every new or edited non-template `.cpp` includes its own header first after the project precompiled header if one is required.

---

## Automatic Rejection Conditions

Reject the design and revise it if any of the following are true:

- A new class mostly forwards calls to one owned object.
- A new class exists mainly to rename, expose, or repackage another class.
- A subsystem ends up with multiple public entry points that overlap in responsibility.
- A public supporting type mainly represents one class's internal calculation stage.
- A file introduces a second naming convention, access pattern, or ownership pattern beside an existing project-standard one.
- A wrapper is introduced only to reduce local edits.
- Old and new implementations are left in parallel after the change.
- A change preserves architectural junk because deleting it would require additional refactoring.
- A new mode is created by copying another mode's config namespace or file.
- The same conceptual parameter exists in more than one config owner.
- Shared tuning is duplicated instead of referenced from one authority.
- A namespace alias is used to create another casual access path to configuration.
- Public fields are introduced without a clear, documented performance reason.
- Inheritance is introduced where composition would suffice.
- A new utility or helper module is introduced instead of extending the authoritative owner.
- A change introduces test-only seams, wrappers, or adapters that do not belong in the production architecture.
- A change expands beyond the stated task boundary without a concrete ownership or architecture reason.
- Adding a mode requires new `BeginXMode()` / `RunXMode()` methods on a host interface.
- Adding a mode requires a new forwarding wrapper subclass.
- The architecture implies runtime switching between top-level application modes.
- Internal phases of one boot-selected workflow are modeled as separate application modes.
- A different top-level mode can be entered without reboot.
- Adding a utility mode requires a new logging architecture.
- Adding a utility mode hides its boot condition outside the central boot-mode registry.
- Boot selector pins or jumper conditions are stored in mode tuning or configuration ownership instead of `BootModeRegistry` or the platform pin map.
- Adding a utility mode makes shared tuning diverge without explicit experimental justification.
- Adding a utility mode requires a cluster of new infrastructure files rather than primarily one mode registry entry and one mode implementation file.
- Adding a utility mode makes it hard to tell, from one place, what the mode is for and what it produces.
- Multiple utility modes solve the same setup, logging, recovery, or watchdog problem in inconsistent ways.
- A utility mode owns `MmLogLogger`, file-export, or event-log objects directly instead of using shared runtime-owned logging infrastructure.
- A change introduces new shared boot-mode or utility-mode infrastructure without first naming its authoritative owner, shared contract, replacement targets, and intended consumers.
- A one-off shared boot-mode or utility-mode framework layer is introduced for a single mode when no clear multi-mode consolidation exists.
- An entry header, alias header, or forwarding header is introduced or preserved instead of giving the subsystem either real authoritative files or no dedicated files at all.
- A header or source file builds only because of incidental transitive inclusion or current include order rather than its own direct includes and forward declarations.
- A newly extracted or moved class file omits required includes and relies on unrelated headers or precompiled-header leakage to compile.
- A new or edited non-template `.cpp` does not include its own header first after the project precompiled header if one is required.

---

## Header and Include Hygiene

- Headers must be self-sufficient for the declarations they expose.
- Do **not** rely on incidental transitive includes, unrelated include order, or the current build graph to make a header compile.
- Include every direct dependency needed by a header's declarations, base classes, member objects, inline definitions, templates, and constants.
- Use forward declarations only when they are sufficient and stable for the declaration being exposed.
- If a type is used by value, as a base class, or in inline code requiring completeness, include its header rather than relying on a forward declaration.
- Every new or edited non-template `.cpp` must include its own header first after the project precompiled header if one is required.
- Source files must include the headers for the symbols they use directly. Do **not** rely on some other header in the include chain to pull them in accidentally.
- When extracting or moving a class to a new file, add the full set of required includes and forward declarations for that file as part of the same change.
- A file that only builds because another unrelated header happened to be included first is nonconforming even if the current build passes.

---

## Class, File, and API Organization

- Substantive classes must be placed in files sharing the name of that substantive class.
- Non-template classes should have a `.h` with declarations and member documentation and a `.cpp` with implementation.
- Enums and supporting types may be placed in the same file only if they are tightly related to that class or domain-vocabulary subsystem and satisfy the supporting-type rules above.
- Structs and classes are subject to the same acceptance rules.
- Reject structs and classes without documented public members, organized headers, or const-aware methods.
- Reject structs and classes with public fields unless there is a clear, documented performance reason for the type to exist.
- Public APIs should expose domain operations, not implementation staging.
- Fully abstract the internal representation from consumers.
- Offer const and non-const accessors only where both are sensible and meaningful.
- Project constants should exist as compile-time objects owned by the proper class or typed configuration owner, not as scattered top-level constants.
- Do not create additional public types or files merely to make the code look more decomposed; prefer private helpers, nested types, and file-local functions when the concept is not independently authoritative.
- Do not create or preserve entry headers, alias headers, or forwarding headers that only include another file while pretending to define a separate subsystem. If a named subsystem is real, its declarations must live in its own authoritative files. If it is not real, remove the alias header and include the canonical owner directly.

---

## Inheritance and Composition

- Strongly prefer composed types over inheritance.
- Inheritance is allowed only for true interface boundaries or when it provides substantial benefit across at least four real use sites.
- Do not introduce inheritance trees to model convenience, categorization, or speculative reuse.

---

## Testing Rules

- Existing unit tests should not be modified merely to preserve a noncanonical design.
- Existing unit tests may be updated when required to reach the canonical architecture or reflect an intentional behavior change.
- New architecture must improve or preserve direct test access to the authoritative owner.
- Tests must be able to construct, inspect, and compare the exact parameter sets used by production code.
- Do not hide production behavior behind wrappers, aliases, or mode forks that tests must special-case.
- Do not introduce wrappers, facades, adapters, or companion structs solely to make testing easier.

If a design makes the real production configuration or behavior harder to test through the canonical owner, revise the design.

---

## Clarification Policy

- Ask concise clarifying questions when ownership, required behavior, or interface contract is genuinely ambiguous.
- Do not ask clarifying questions merely to avoid cleanup work.
- If the authoritative owner is clear, proceed by extending that owner rather than inventing a new layer.
- If a multi-step cleanup still contains minor ambiguity, state the working assumption and proceed without inventing a new public layer.

---

## Task Scope and Completion Discipline

- Follow the explicit task request and its stated or implied `done when` conditions.
- Do not expand the change set to nearby systems for speculative consistency or hypothetical reuse.
- If a neighboring cleanup is truly required to preserve authoritative ownership, perform only the minimum additional work needed and keep the migration convergent.
- Do not perform unrelated renames, abstractions, or decomposition passes during a targeted cleanup.

---

## Project-Specific Instructions

- No watchdog timers under 60 seconds that trigger a run failure.
- Prefer recovery over fail-fast behavior.
- When runtime behavior deviates from expectation, log the condition before attempting recovery.
- The robot can sustain approximately `16.5 m/s^2` of lateral acceleration when the fan is running at `80%`.
- Plan strategies with the high-performance operating envelope in mind.
- Use the Decimus 5A project for guidance and reference to understand the intended style and performance envelope of this project.
- Directional code must respect the conventions `+X = right`, `+Y = forward/up`, and `+Yaw = clockwise`.
- Prefer being concise, clear, and writing understandable code **over** minimizing code churn when the existing code is nonconforming.
- Do not introduce alternate access patterns, public fields, or companion structs based on presumed performance benefit. First establish correctness, ownership, and clean architecture. Only change representation for performance when the hot path is identified and the simpler encapsulated form is materially insufficient.

### Navigation and locomotion

- The `FloodFill` pathfinder should be used for simple navigation.
- `ManeuverPathfinder` should only be used while stationary.
- Locomotion should prefer the `Maneuver` classes when in a maze.
- For more manual control, generate `ManeuverInstance` objects directly and follow with a small target-yaw PID if needed.
- Open-loop commands are appropriate for low-level tasks such as wall tapping or certain measurements.
- Direct position or yaw control should be reserved for specific tasks that cannot reasonably be done through maneuver-based control.

### Logging and output

- Serial-style output should be sparse and should go to `logging.txt`.
- Heavy datalogging should use the `mmlog` system and the appropriate macros.
- Do not create parallel ad hoc logging systems.

### Build

- When building, use `verify_latest_build.cmd` or `verify_latest_build.ps`.

---

## Preferred Coding Outcome

A good cleanup change should usually result in all of the following:

- one clear authority for the edited responsibility,
- fewer public entry points than before,
- fewer wrappers and helper files than before,
- fewer duplicate parameters or config owners than before,
- fewer alternate naming and access patterns than before,
- easier unit testing through the canonical owner than before,
- easier discovery of every boot mode and its entry condition than before,
- more consistent utility-mode setup, logging, and self-description than before.
- stronger include hygiene and less reliance on incidental transitive includes than before.

If the change increases the number of layers, config owners, access paths, wrappers, or parallel patterns, it is probably wrong.

---

## Deprecated Files

- `RuntimeBinaryLogSupport.h`
- `CoreBinaryFileExport.h`
- `CoreBinaryFileExport.cpp`
- `OptionalRuntimeEventLog.h`
- `OptionalRuntimeEventLog.cpp`
