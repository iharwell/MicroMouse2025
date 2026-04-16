# MazeMap Header Cleanup Thread Backlog

This note turns the findings in `MazeMap/mazemap_hub_header_implementation_audit.md` into thread-sized follow-up work.

The intent is not "make headers smaller" in the abstract. The intent is:

- restore authoritative ownership,
- stop distributing runtime implementation through conglomerate headers,
- delete wrapper/alias headers,
- move non-template behavior into `.cpp` files owned by the real subsystem.

## Current offenders

| Header | Current lines | Main problem |
| --- | ---: | --- |
| `MazeMap/MazeMap/MazeMapRuntimeCore.h` | 3962 | Runtime implementation dump with multiple owners collapsed into one header |
| `MazeMap/MazeMap/DriveBase.h` | 1493 | Large concrete owner with substantial inline behavior still in the header |
| `MazeMap/MazeMap/MazeMapRuntimeSensors.h` | 1246 | Two substantive runtime classes defined mostly inline |
| `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` | 1159 | Another implementation hub that also republishes other heavy headers |
| `MazeMap/MazeMap/OpenFloorMeasurementSpec.h` | 598 | Spec/data header still coupled to runtime-heavy dependencies |
| `MazeMap/MazeMap/LoopController.h` | 264 | Declaration-heavy header still leaning on heavier runtime surfaces |
| `MazeMap/MazeMap/TeensyLayout.h` | 198 | Teensy-specific implementation in a header without a real need to stay header-only |
| `MazeMap/MazeMap/RuntimeBinaryLogSupport.h` | 164 | Deprecated header-only implementation |
| `MazeMap/MazeMap/MazeMapApplicationPrivate.h` | 79 | Small file, but it is the conglomerate include sink on the PCH spine |
| `MazeMap/MazeMap/OpenFloorMeasurementCycle.h` | 25 | Data-only header that should not need heavy runtime dependencies |
| `MazeMap/MazeMap/OpenFloorMeasurementLabels.h` | 17 | Label-only header inheriting heavy dependencies transitively |
| `MazeMap/MazeMap/SensorSuite.h` | 3 | Pure wrapper header |
| `MazeMap/MazeMap/DiagnosticSensorSuite.h` | 3 | Pure wrapper header |
| `MazeMap/MazeMap/WallDistanceCalibration.h` | 3 | Pure wrapper header |

## Recommended thread split

### 1. Break the PCH distribution chain

**Headers**

- `MazeMap/MazeMap/pch.h`
- `MazeMap/MazeMap/MazeMapApplicationPrivate.h`
- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `MazeMap/MazeMap/MazeMapRuntimeSensors.h`

**Why this is a separate thread**

This is the ambient-distribution problem. Right now runtime implementation rides the `pch.h -> MazeMapApplicationPrivate.h -> MazeMapRuntimeCore.h / MazeMapRuntimeSensors.h` path.

**Thread goal**

- remove runtime implementation hubs from the PCH spine,
- stop `MazeMapApplicationPrivate.h` from acting as a private-everything umbrella,
- stop exporting `using` declarations out of that header.

**Done when**

- `pch.h` no longer makes runtime implementation ambient,
- `MazeMapApplicationPrivate.h` is either deleted or reduced to a real private interface,
- edited `.cpp` files include their own direct dependencies.

**Dependency note**

This thread should happen first or very early because it exposes the true include graph.

### 2. Decompose `MazeMapRuntimeCore.h`

**Headers**

- `MazeMap/MazeMap/MazeMapRuntimeCore.h`
- `MazeMap/MazeMap/WallDistanceCalibration.h`

**Why this is a separate thread**

`MazeMapRuntimeCore.h` is the worst implementation dump in the tree. It is large enough that "just move a little" work will not converge unless the authoritative owners are named up front.

**Thread goal**

- identify the real owners hidden inside the hub,
- move non-template logic into authoritative same-named `.cpp` files,
- delete wrapper-only surfaces like `WallDistanceCalibration.h`.

**Done when**

- no substantive runtime/calibration behavior remains stranded in `MazeMapRuntimeCore.h`,
- callers include the real owner headers directly,
- wrapper-only headers are removed from the build.

### 3. Split sensor ownership out of `MazeMapRuntimeSensors.h`

**Headers**

- `MazeMap/MazeMap/MazeMapRuntimeSensors.h`
- `MazeMap/MazeMap/SensorSuite.h`
- `MazeMap/MazeMap/DiagnosticSensorSuite.h`

**Why this is a separate thread**

This is a clean ownership seam: two substantive classes are living in one implementation-heavy header, and two tiny wrapper headers keep republishing it.

**Thread goal**

- move `SensorSuite` into authoritative `SensorSuite.*`,
- move `DiagnosticSensorSuite` into authoritative `DiagnosticSensorSuite.*`,
- delete the wrapper-only headers.

**Done when**

- the two sensor owners live in their own files,
- `MazeMapRuntimeSensors.h` disappears or becomes unnecessary,
- no wrapper header exists just to include another header.

### 4. Collapse `MazeMapRuntimeInfrastructure.h`

**Headers**

- `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h`
- `MazeMap/MazeMap/RuntimeBinaryLogSupport.h`

**Why this is a separate thread**

This header compounds several heavy dependencies and then adds another block of inline runtime behavior on top. It is a second hub, not infrastructure in the narrow sense.

**Thread goal**

- move runtime logging/helpers into their authoritative owners,
- remove deprecated binary-log support from the live architecture,
- keep only lightweight declarations in the remaining runtime infrastructure surface.

**Done when**

- `MazeMapRuntimeInfrastructure.h` is no longer an implementation hub,
- deprecated `RuntimeBinaryLogSupport.h` is removed or no longer on active paths,
- includers do not pick up `DriveBase` and OpenFloor runtime behavior transitively.

### 5. Slim the OpenFloor header chain

**Headers**

- `MazeMap/MazeMap/OpenFloorMeasurementSpec.h`
- `MazeMap/MazeMap/OpenFloorMeasurementCycle.h`
- `MazeMap/MazeMap/OpenFloorMeasurementLabels.h`
- `MazeMap/MazeMap/LoopController.h`

**Why this is a separate thread**

These are mostly data/descriptor/controller-declaration surfaces, but they still inherit runtime-heavy includes because vocabulary types are not owned cleanly.

**Thread goal**

- extract shared runtime vocabulary types into lightweight authoritative headers,
- make `Spec`, `Cycle`, `Labels`, and `LoopController` depend on those light owners instead of on runtime hubs.

**Done when**

- `OpenFloorMeasurementSpec.h` no longer includes `MazeMapRuntimeCore.h` directly,
- `OpenFloorMeasurementCycle.h` is a genuine record/data header,
- `OpenFloorMeasurementLabels.h` stays lightweight,
- `LoopController.h` only includes what its declarations actually need.

**Dependency note**

This thread will likely overlap with thread 4, but it is still worth tracking separately because the success condition is include-shape cleanup, not runtime behavior cleanup.

### 6. Continue the `DriveBase.h` migration

**Headers**

- `MazeMap/MazeMap/DriveBase.h`

**Why this is a separate thread**

`DriveBase` is a real owner, so this is not a "split into more classes" task. It is a "keep the owner, move behavior out of the header" task.

**Thread goal**

- continue moving non-template bodies into `DriveBase.cpp`,
- keep plant math in `PlantModel`,
- keep only true declarations, light accessors, and genuinely inline helpers in the header.

**Done when**

- `DriveBase.h` is mostly declarations,
- new behavior is added to `DriveBase` or `PlantModel`, not to helper/wrapper layers,
- callers still use the same authoritative `DriveBase` owner.

**Note**

Recent plant-math extraction already moved some velocity-target and wheel-motion logic out of `DriveBase`; this thread is the remainder of that cleanup.

### 7. Low-priority small-header cleanup

**Headers**

- `MazeMap/MazeMap/TeensyLayout.h`
- `MazeMap/MazeMap/PinPairStrap.h`
- `MazeMap/MazeMap/WallSensorLedCalibrationPhase.h`

**Why this is a separate thread**

These are smaller than the major hubs, so they should not block the bigger structural work. They are still worth cleaning because they keep normalizing header-resident implementation.

**Thread goal**

- move behavior into `.cpp` files or into the true subsystem owner,
- leave only stable declarations in the public header surface.

**Done when**

- these headers no longer carry runtime implementation just because it was convenient,
- no new tiny helper dumps are introduced in their place.

## Suggested order

1. Break the PCH chain.
2. Split `MazeMapRuntimeSensors.h`.
3. Decompose `MazeMapRuntimeCore.h`.
4. Collapse `MazeMapRuntimeInfrastructure.h`.
5. Slim the OpenFloor header chain.
6. Continue the `DriveBase.h` migration.
7. Finish the small-header cleanup.

## Parallelization notes

- Threads 2 and 6 can proceed in parallel once thread 1 has exposed the real includes.
- Threads 4 and 5 are related but not identical; one is runtime-owner cleanup, the other is dependency-shape cleanup.
- Thread 7 should stay low priority unless it is needed to finish one of the higher-value cleanups cleanly.

## Explicit non-goals

- Do not preserve wrapper headers for convenience.
- Do not introduce new helper/facade layers to avoid touching callers.
- Do not split `DriveBase` into fake public companions just to make the header smaller.
- Do not move code out of a hub into another hub.
