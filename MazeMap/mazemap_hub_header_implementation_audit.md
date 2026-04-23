# MazeMap Hub-Header Implementation Audit

## Method

- Source of truth: `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\x64\Release\MazeMap.tlog\CL.read.1.tlog`
- PCH exposure check: `C:\Users\thene\source\repos\MicroMouse2025\MazeMap\MazeMap\x64\Release\MazeMap.tlog\CL.command.1.tlog`
- I treated a header as a hub when either:
  - it had repeated direct fan-in in the release `tlog`, or
  - it sat on the `pch.cpp -> pch.h` spine, because the current build uses `/Yu"pch.h"` for 46 translation units.
- I flagged only places where header-resident implementation or implementation-bearing includes do not appear to exist for:
  - a real Teensy build-path requirement,
  - the centralized host/target compatibility boundary, or
  - an obvious template/`constexpr` necessity.

## Non-findings

- `UKF.h` is intentionally template-heavy and has a real host/Teensy split for CMSIS DSP.
- `WallGeometryModel.h`, `BootModeDescriptor.h`, and most of `PlantModel.h` are declaration/data heavy rather than implementation-dump headers.

## Findings

### 1. PCH spine distributes private runtime implementation broadly

| Site | Exposure | Implementation pulled in | Why this does not buy Teensy or host value |
| --- | --- | --- | --- |
| `MazeMap/MazeMap/pch.h:12` | Effective fan-in: 46 `/Yu` compiles | `MazeMapApplicationPrivate.h` | This turns one private aggregate header into a project-wide distribution point. The benefit here is PCH convenience, not host compatibility or Teensy-specific behavior. |
| `MazeMap/MazeMap/MazeMapApplicationPrivate.h:72-73` | On the same PCH spine | `MazeMapRuntimeCore.h`, `RuntimeSensorSuite.h` | Runtime declarations and implementation-heavy runtime-core content still ride through the PCH instead of through direct includes from the files that need them. |
| `MazeMap/MazeMap/MazeMapApplicationPrivate.h:13` | On the same PCH spine | `TeensyLayout.h` | `TeensyLayout.h` is gated by `ARDUINO_TEENSY41`, so the include does not help host compatibility. It also does not need to be header-only for Teensy. |

### 2. Wrapper headers that only forward to implementation hubs

| Header | Direct `tlog` fan-in | Include site | Problem |
| --- | --- | --- | --- |
| `MazeMap/MazeMap/WallDistanceCalibration.h` | 0 direct, but reachable through PCH/runtime headers | `:4` -> `MazeMapRuntimeCore.h` | Another pure wrapper. It republishes a large runtime implementation header instead of exposing an owned type directly. |

### 3. Public or semi-public hub headers that pull runtime implementation when they only need types/constants

| Header | Direct `tlog` fan-in | Include site | Why the include is heavier than necessary |
| --- | --- | --- | --- |
| `MazeMap/MazeMap/OpenFloorMeasurementSpec.h` | 6 | `:3` -> `MazeMapRuntimeCore.h` | This header is mostly enums, ids, tables, and lookup helpers. Its uses are `Direction`, `Maze::GetCellDimension()`, `DEG_TO_RAD_F`, and `MazeMap::Math`; none justify pulling a 3962-line runtime core implementation header into every includer. |
| `MazeMap/MazeMap/LoopController.h` | 7 | `:3` -> `MazeMapRuntimeCore.h` | `LoopController.h` is declaration-heavy. Its dependency on the runtime-core implementation dump signals missing type ownership, not a genuine need for header-resident runtime code. |

### 4. Hub headers that are themselves implementation dumps

| Header | Exposure | Evidence | Why the header form is not justified here |
| --- | --- | --- | --- |
| `MazeMap/MazeMap/MazeMapRuntimeCore.h` | PCH spine; effective exposure through 46 `/Yu` compiles | 3962 lines; host stub at `:56`, then large non-template runtime implementation from `:573` onward; `WallDistanceCalibration` begins at `:1355`; many free inline algorithms continue through `:4120` | A few top-of-file host fallback definitions are legitimate, but they do not justify keeping the rest of the runtime core, calibration logic, wall sensing, fan control, parsing, and geometry helpers in the header. The centralized host boundary is `Defines.h`, not this file. |
| `MazeMap/MazeMap/DriveBase.h` | Direct `tlog` fan-in: 8; also pulled by runtime infrastructure | 2226 lines; free inline helpers start at `:18`; `DriveBase` class body starts at `:203` | This is a concrete runtime owner with substantial non-template behavior. Nothing about Teensy or host support requires the whole drive subsystem to live in the header. |
| `MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h` | Direct `tlog` fan-in: 6 | Includes `DriveBase.h`, `LoopController.h`, `MazeMapRuntimeCore.h`, `MazeMapRuntimeMmLog.h`, and `OpenFloorMeasurementSpec.h` at `:3-9`; then adds inline logging and wall-touch implementation | This file compounds multiple implementation hubs and then adds another one. The header shape spreads runtime behavior instead of just declaring contracts. |
| `MazeMap/MazeMap/TeensyLayout.h` | On the PCH spine | 198 lines; inline Teensy hardware and SD setup from `:16` and `:149` onward | This code is Teensy-specific, but header residency is not what makes it usable on Teensy. It is excluded on host, so it does not serve host compatibility either. |
| `MazeMap/MazeMap/RuntimeBinaryLogSupport.h` | Direct `tlog` fan-in: 7 | 164 lines; inline helpers at `:25-160`; small `ARDUINO_TEENSY41` branch plus host fallback | The host/Teensy branch is real, but it can exist in a `.cpp` just as well. The compatibility branch does not require the utilities to be header-only. |

### 5. Small header-only utilities that still do not appear target- or host-driven

| Header | Direct `tlog` fan-in | Evidence | Note |
| --- | --- | --- | --- |
| `MazeMap/MazeMap/PinPairStrap.h` | 8 | Four inline helpers at `:5-38` | Small compared with the runtime hubs, but still header-resident implementation without an obvious Teensy or host-compatibility reason. |
| `MazeMap/MazeMap/WallSensorLedCalibrationPhase.h` | 5 | Inline state advance helper at `:12-29` | Very small issue, but still not driven by template or cross-build needs. |

## Summary

The main architectural problem is not isolated one-off inline helpers. It is the distribution chain:

`pch.h` -> `MazeMapApplicationPrivate.h` -> `MazeMapRuntimeCore.h` / `RuntimeSensorSuite.h`

That chain makes runtime implementation effectively global to the host build, and several secondary headers (`DriveBase.h`, `MazeMapRuntimeInfrastructure.h`, `OpenFloorMeasurementSpec.h`, and wrapper headers such as `WallDistanceCalibration.h`) keep republishing the same implementation-heavy surfaces. None of that appears necessary for Teensy compilation, and the host-compatibility boundary is already supposed to be centralized elsewhere.
