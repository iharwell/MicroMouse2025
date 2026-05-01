# Curated Duplicate Small-Function Implementations

This report narrows the earlier duplicate-name output to repeated sub-3-logical-line implementations that still look like real duplicated logic after removing obvious noise.

Excluded as noise:
- Common container/API vocabulary repeated in different owners, such as `clear`, `size`, `GetSize`, `Index`, `contains`, `GetCount`, and similar accessors.
- Host/test stub functions that intentionally return constants like `0`, `0.0f`, `true`, or `false`.
- Same-file overload pairs and const/non-const accessor pairs.
- Trivial one-field getters/setters on unrelated owners.

## Production Candidates

### `FiniteOrZero`

Exact body:

```cpp
return std::isfinite(value) ? value : 0.0f;
```

- [GripUtilizationMetrics.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/GripUtilizationMetrics.cpp:33): `FiniteOrZero`
- [TorqueEstimateAdapter.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TorqueEstimateAdapter.cpp:12): `FiniteOrZero`
- [UkfRobustUpdatePolicy.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UkfRobustUpdatePolicy.cpp:43): `FiniteOrZero`
- [WheelAuthorityPolicy.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WheelAuthorityPolicy.cpp:33): `FiniteOrZero`

### `IsFinitePositive`

Exact body:

```cpp
return std::isfinite(value) && (value > 0.0f);
```

- [DriveBase.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:714): `IsFinitePositive`
- [Estimator.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h:284): `IsFinitePositive`
- [LoopController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.cpp:25): `IsFinitePositive`

### `ClampMeasuredRange`

Exact body:

```cpp
return (std::clamp)(value, 0.01f, maxRangeM);
```

- [DriveBase.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:719): `ClampMeasuredRange`
- [Estimator.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h:288): `ClampMeasuredRange`

### Selector-Removed Check

Exact body:

```cpp
return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
```

- [OpenFloorMeasurementController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/OpenFloorMeasurementController.cpp:993): `OpenFloorMeasurementController::State::SelectorRemoved`
- [ShowcasingDonutController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ShowcasingDonutController.cpp:379): `ShowcasingDonutController::SelectorRemoved`
- [TopSpeedMeasurementMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TopSpeedMeasurementMode.cpp:222): `SelectorRemoved`

### Phase Marker Wrapper

Exact body:

```cpp
++_phaseId;
return _runtime.WriteTextLogPhase(_phaseId, micros(), name);
```

- [AuxMeasurementController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementController.cpp:281): `BeginPhase`
- [DiagnosticController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DiagnosticController.cpp:1276): `WritePhaseMarker`

### Text-Log Event Wrapper

Exact body:

```cpp
return _runtime.WriteTextLogEntry(micros(), type, message);
```

- [AuxMeasurementController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementController.cpp:276): `WriteEvent`
- [DiagnosticController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DiagnosticController.cpp:1271): `WriteLogEvent`

### Active-Mode Fail Wrapper

Exact body:

```cpp
return _runtime.FailActiveMode(reason);
```

- [FrontWallCharacterizationController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/FrontWallCharacterizationController.cpp:638): `Fail`
- [ManeuverFileTestMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverFileTestMode.cpp:250): `Fail`

### Wheel-Control-Profile Wrapper

Exact body:

```cpp
return BuildNominalWheelControlProfile();
```

- [AuxMeasurementController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementController.cpp:513): `BuildTurningTractionWheelControlProfile`
- [DiagnosticController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DiagnosticController.cpp:1491): `BuildDiagnosticWheelControlProfile`

### Start-Cell Center Helper

Exact body:

```cpp
return Eigen::Vector2f(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
```

- [CorridorRepeatabilityMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CorridorRepeatabilityMode.cpp:185): `StartCellCenter`
- [PositionAccuracyAuditMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PositionAccuracyAuditMode.cpp:186): `StartCellCenter`

### Coordinate/Adjacency Logic Duplicated Between `CellCoordinates` and `MazeLocation`

These are not just shared names; they are the same equality and adjacency implementations repeated across two coordinate-like types.

- [CellCoordinates.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CellCoordinates.cpp:25) and [MazeLocation.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeLocation.cpp:73): `operator==` returning `GetX()/GetY()` equality
- [CellCoordinates.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CellCoordinates.cpp:30) and [MazeLocation.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeLocation.cpp:81): `operator!=` returning `GetX()/GetY()` inequality
- [CellCoordinates.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CellCoordinates.cpp:68) and [MazeLocation.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeLocation.cpp:89): `IsUp`
- [CellCoordinates.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CellCoordinates.cpp:69) and [MazeLocation.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeLocation.cpp:91): `IsDown`
- [CellCoordinates.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CellCoordinates.cpp:70) and [MazeLocation.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeLocation.cpp:93): `IsLeft`
- [CellCoordinates.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CellCoordinates.cpp:71) and [MazeLocation.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeLocation.cpp:95): `IsRight`

## Test-Code Duplicates

### `ScopedMissionFanDuty`

Duplicated local RAII helper in two test files.

- [DriveManeuverTests.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveManeuverTests.cpp:44): constructor writes `WriteFanDutyCycle(dutyCycle)`
- [SharedRuntimeTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SharedRuntimeTest.cpp:44): constructor writes `WriteFanDutyCycle(dutyCycle)`
- [DriveManeuverTests.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveManeuverTests.cpp:50): destructor writes `WriteFanDutyCycle(previousDutyCycle)`
- [SharedRuntimeTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SharedRuntimeTest.cpp:50): destructor writes `WriteFanDutyCycle(previousDutyCycle)`

### `ScopedUkfRuntimeTuningRestore`

Duplicated local runtime-tuning restore helper in two UKF test files.

- [SrUkfCoreBiasAndStationaryTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SrUkfCoreBiasAndStationaryTest.cpp:25): destructor calls `SrUkfCore::SetRuntimeTuning(saved);`
- [SrUkfCoreModeAndDiagnosticsTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp:18): destructor calls `SrUkfCore::SetRuntimeTuning(saved);`

### Production/Test Helper Mirrors

- [DriveBase.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.cpp:491) and [DriveBaseTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveBaseTest.cpp:164): average-drive-command helper returning `0.5f * (command.leftMotorPwm + command.rightMotorPwm);`
- [DriveBase.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.cpp:497) and [DriveBaseTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveBaseTest.cpp:169): delta-drive-command helper returning `0.5f * (command.leftMotorPwm - command.rightMotorPwm);`

## Tooling Duplicates

### `strip_cpp_comments`

Exact body:

```python
text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
return re.sub(r"//.*", "", text)
```

- [analyze_open_floor_turn_bias.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:224): `strip_cpp_comments`
- [competition_feedforward.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:96): `strip_cpp_comments`
