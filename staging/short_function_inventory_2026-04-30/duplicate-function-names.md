# Duplicate Function Names Across the Sub-3 Inventory

This report is derived from the existing sub-3-logical-line inventory files in this directory.
A function name is listed here when the same exact reported name appears in more than one source file.

Excluded parser artifact:
- `alignas` was removed after spot-checking showed those entries were aligned-buffer declarations, not functions.

- Duplicate function names: 49
- Total duplicate occurrences: 136

## `clear`

- Files: 4
- Occurrences: 4
- [MazeMap/MazeMap/HalfStepPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h): [line 121](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:121)
- [MazeMap/MazeMap/ManeuverPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h): [line 92](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:92)
- [MazeMap/MazeMap/ManeuverQueue.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h): [line 75](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:75)
- [MazeMap/MazeMap/Path.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h): [line 83](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:83)

## `FiniteOrZero`

- Files: 4
- Occurrences: 4
- [MazeMap/MazeMap/GripUtilizationMetrics.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/GripUtilizationMetrics.cpp): [line 33](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/GripUtilizationMetrics.cpp:33)
- [MazeMap/MazeMap/TorqueEstimateAdapter.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TorqueEstimateAdapter.cpp): [line 12](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/TorqueEstimateAdapter.cpp:12)
- [MazeMap/MazeMap/UkfRobustUpdatePolicy.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UkfRobustUpdatePolicy.cpp): [line 43](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UkfRobustUpdatePolicy.cpp:43)
- [MazeMap/MazeMap/WheelAuthorityPolicy.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WheelAuthorityPolicy.cpp): [line 33](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WheelAuthorityPolicy.cpp:33)

## `size`

- Files: 4
- Occurrences: 7
- [MazeMap/MazeMap/CircularBuffer.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CircularBuffer.h): [line 129](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CircularBuffer.h:129), [line 130](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CircularBuffer.h:130)
- [MazeMap/MazeMap/ManeuverQueue.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h): [line 66](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:66), [line 67](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:67)
- [MazeMap/MazeMap/ManeuverSet.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverSet.h): [line 65](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverSet.h:65), [line 66](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverSet.h:66)
- [MazeMap/MazeMap/MmLog.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h): [line 313](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h:313)

## `contains`

- Files: 3
- Occurrences: 6
- [MazeMap/MazeMap/HalfStepPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h): [line 126](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:126), [line 127](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:127)
- [MazeMap/MazeMap/ManeuverPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h): [line 97](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:97), [line 98](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:98)
- [MazeMap/MazeMap/Path.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h): [line 88](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:88), [line 89](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:89)

## `Fail`

- Files: 3
- Occurrences: 3
- [MazeMap/MazeMap/DiagnosticController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DiagnosticController.cpp): [line 1496](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DiagnosticController.cpp:1496)
- [MazeMap/MazeMap/FrontWallCharacterizationController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/FrontWallCharacterizationController.cpp): [line 638](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/FrontWallCharacterizationController.cpp:638)
- [MazeMap/MazeMap/ManeuverFileTestMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverFileTestMode.cpp): [line 250](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverFileTestMode.cpp:250)

## `GetSize`

- Files: 3
- Occurrences: 6
- [MazeMap/MazeMap/HalfStepPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h): [line 58](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:58), [line 59](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:59)
- [MazeMap/MazeMap/ManeuverPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h): [line 27](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:27), [line 28](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:28)
- [MazeMap/MazeMap/Path.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h): [line 18](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:18), [line 19](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:19)

## `Index`

- Files: 3
- Occurrences: 6
- [MazeMap/MazeMap/HalfStepPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h): [line 61](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:61), [line 62](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:62)
- [MazeMap/MazeMap/ManeuverPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h): [line 30](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:30), [line 31](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:31)
- [MazeMap/MazeMap/Path.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h): [line 21](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:21), [line 22](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:22)

## `indexOf`

- Files: 3
- Occurrences: 3
- [MazeMap/MazeMap/HalfStepPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h): [line 129](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/HalfStepPath.h:129)
- [MazeMap/MazeMap/ManeuverPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h): [line 100](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:100)
- [MazeMap/MazeMap/Path.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h): [line 91](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:91)

## `IsFinitePositive`

- Files: 3
- Occurrences: 3
- [MazeMap/MazeMap/DriveBase.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h): [line 714](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:714)
- [MazeMap/MazeMap/Estimator.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h): [line 284](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h:284)
- [MazeMap/MazeMap/LoopController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.cpp): [line 25](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.cpp:25)

## `write`

- Files: 3
- Occurrences: 5
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 586](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:586), [line 793](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:793), [line 836](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:836)
- [MazeMap/MazeMap/MmLog.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h): [line 613](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h:613)
- [codex_verify/QuadEncoder.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h): [line 14](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h:14)

## `analogWrite`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 506](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:506)
- [codex_verify/Arduino.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h): [line 21](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h:21)

## `analogWriteFrequency`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 512](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:512)
- [codex_verify/Arduino.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h): [line 23](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h:23)

## `analogWriteResolution`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 509](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:509)
- [codex_verify/Arduino.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h): [line 22](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h:22)

## `begin`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 726](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:726)
- [codex_verify/SPI.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/SPI.h): [line 16](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/SPI.h:16)

## `ClampMeasuredRange`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/DriveBase.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h): [line 719](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:719)
- [MazeMap/MazeMap/Estimator.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h): [line 288](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h:288)

## `covariance`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/SrUkfCore.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h): [line 157](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h:157)
- [MazeMap/MazeMap/UKF.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h): [line 433](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h:433)

## `delay`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 401](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:401)
- [codex_verify/Arduino.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h): [line 24](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/Arduino.h:24)

## `empty`

- Files: 2
- Occurrences: 3
- [MazeMap/MazeMap/ManeuverQueue.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h): [line 70](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:70), [line 71](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:71)
- [MazeMap/MazeMap/MmLog.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h): [line 314](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h:314)

## `flush`

- Files: 2
- Occurrences: 3
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 717](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:717), [line 799](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:799)
- [MazeMap/MazeMap/MmLog.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h): [line 410](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h:410)

## `GetCount`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/RollingAverageWindow.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/RollingAverageWindow.h): [line 89](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/RollingAverageWindow.h:89)
- [MazeMap/MazeMap/WallSensorCalibration.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensorCalibration.h): [line 106](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensorCalibration.h:106)

## `GetPosition`

- Files: 2
- Occurrences: 3
- [MazeMap/MazeMap/VehicleState.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/VehicleState.h): [line 235](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/VehicleState.h:235), [line 236](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/VehicleState.h:236)
- [MazeMap/MazeMap/WallSensor.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensor.h): [line 69](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallSensor.h:69)

## `init`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 829](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:829)
- [codex_verify/QuadEncoder.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h): [line 12](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h:12)

## `IsStraightCode`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/ManeuverQueue.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h): [line 19](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:19)
- [MazeMap/MazeMap/MazeMapRuntimeCore.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h): [line 1586](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h:1586)

## `last`

- Files: 2
- Occurrences: 4
- [MazeMap/MazeMap/ManeuverPath.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h): [line 82](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:82), [line 91](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverPath.h:91)
- [MazeMap/MazeMap/Path.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h): [line 73](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:73), [line 82](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Path.h:82)

## `maxSize`

- Files: 2
- Occurrences: 4
- [MazeMap/MazeMap/CircularBuffer.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CircularBuffer.h): [line 132](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CircularBuffer.h:132), [line 133](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CircularBuffer.h:133)
- [MazeMap/MazeMap/ManeuverQueue.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h): [line 68](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:68), [line 69](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverQueue.h:69)

## `modelCycleContext`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Estimator.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h): [line 29](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Estimator.h:29)
- [MazeMap/MazeMap/SrUkfCore.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h): [line 172](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h:172)

## `operator&`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Direction.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp): [line 24](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp:24)
- [MazeMap/MazeMap/Maneuver.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h): [line 148](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h:148)

## `operator()`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/UKF.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h): [line 28](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h:28)
- [Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp](C:/Users/thene/source/repos/MicroMouse2025/Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp): [line 460](C:/Users/thene/source/repos/MicroMouse2025/Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp:460)

## `operator+`

- Files: 2
- Occurrences: 4
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 567](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:567)
- [MazeMap/MazeMap/Direction.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp): [line 52](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp:52), [line 57](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp:57), [line 62](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp:62)

## `operator==`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 572](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:572)
- [Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp](C:/Users/thene/source/repos/MicroMouse2025/Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp): [line 451](C:/Users/thene/source/repos/MicroMouse2025/Tools/OpenFloorUkfReplay/OpenFloorUkfReplay.cpp:451)

## `operatorbool`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/LoopController.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.h): [line 209](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LoopController.h:209)
- [MazeMap/MazeMap/MmLog.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h): [line 419](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MmLog.h:419)

## `operator|`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Direction.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp): [line 16](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp:16)
- [MazeMap/MazeMap/Maneuver.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h): [line 143](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h:143)

## `operator|=`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/CommandPD.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CommandPD.h): [line 37](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CommandPD.h:37)
- [MazeMap/MazeMap/Direction.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp): [line 20](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Direction.cpp:20)

## `parse_float`

- Files: 2
- Occurrences: 2
- [tooling/build_feedforward_tensor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py): [line 318](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:318)
- [tooling/open_floor_recovery.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py): [line 113](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:113)

## `parse_int`

- Files: 2
- Occurrences: 2
- [tooling/build_feedforward_tensor.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py): [line 332](C:/Users/thene/source/repos/MicroMouse2025/tooling/build_feedforward_tensor.py:332)
- [tooling/open_floor_recovery.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py): [line 117](C:/Users/thene/source/repos/MicroMouse2025/tooling/open_floor_recovery.py:117)

## `QuadEncoder`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 820](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:820)
- [codex_verify/QuadEncoder.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h): [line 10](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h:10)

## `read`

- Files: 2
- Occurrences: 3
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 715](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:715), [line 831](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:831)
- [codex_verify/QuadEncoder.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h): [line 13](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h:13)

## `Reset`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/LSM6DSV16X_IMU.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LSM6DSV16X_IMU.h): [line 816](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/LSM6DSV16X_IMU.h:816)
- [MazeMap/MazeMap/SmoothTurnYawRateController.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SmoothTurnYawRateController.h): [line 14](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SmoothTurnYawRateController.h:14)

## `RightUnitFromHeading`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/MazeMapRuntimeCore.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h): [line 550](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/MazeMapRuntimeCore.h:550)
- [MazeMap/MazeMap/WallGeometryModel.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallGeometryModel.cpp): [line 22](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/WallGeometryModel.cpp:22)

## `ScopedMissionFanDuty`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMapTest/DriveManeuverTests.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveManeuverTests.cpp): [line 44](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveManeuverTests.cpp:44)
- [MazeMap/MazeMapTest/SharedRuntimeTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SharedRuntimeTest.cpp): [line 44](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SharedRuntimeTest.cpp:44)

## `SetGyroBiasZ`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/DriveBase.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h): [line 152](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/DriveBase.h:152)
- [MazeMap/MazeMap/VehicleState.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/VehicleState.h): [line 267](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/VehicleState.h:267)

## `setInitConfig`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/Defines.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h): [line 828](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Defines.h:828)
- [codex_verify/QuadEncoder.h](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h): [line 11](C:/Users/thene/source/repos/MicroMouse2025/codex_verify/QuadEncoder.h:11)

## `StartCellCenter`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/CorridorRepeatabilityMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CorridorRepeatabilityMode.cpp): [line 185](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/CorridorRepeatabilityMode.cpp:185)
- [MazeMap/MazeMap/PositionAccuracyAuditMode.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PositionAccuracyAuditMode.cpp): [line 186](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/PositionAccuracyAuditMode.cpp:186)

## `state`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/SrUkfCore.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h): [line 152](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/SrUkfCore.h:152)
- [MazeMap/MazeMap/UKF.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h): [line 431](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/UKF.h:431)

## `strip_cpp_comments`

- Files: 2
- Occurrences: 2
- [tooling/analyze_open_floor_turn_bias.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py): [line 224](C:/Users/thene/source/repos/MicroMouse2025/tooling/analyze_open_floor_turn_bias.py:224)
- [tooling/competition_feedforward.py](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py): [line 96](C:/Users/thene/source/repos/MicroMouse2025/tooling/competition_feedforward.py:96)

## `SupportsPointTracking`

- Files: 2
- Occurrences: 3
- [MazeMap/MazeMap/Maneuver.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h): [line 220](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h:220), [line 402](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/Maneuver.h:402)
- [MazeMap/MazeMap/ManeuverInstance.h](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverInstance.h): [line 93](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/ManeuverInstance.h:93)

## `WriteEvent`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMap/AuxMeasurementController.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementController.cpp): [line 276](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementController.cpp:276)
- [MazeMap/MazeMap/AuxMeasurementModeSupport.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementModeSupport.cpp): [line 12](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMap/AuxMeasurementModeSupport.cpp:12)

## `~ScopedMissionFanDuty`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMapTest/DriveManeuverTests.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveManeuverTests.cpp): [line 48](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveManeuverTests.cpp:48)
- [MazeMap/MazeMapTest/SharedRuntimeTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SharedRuntimeTest.cpp): [line 48](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SharedRuntimeTest.cpp:48)

## `~ScopedUkfRuntimeTuningRestore`

- Files: 2
- Occurrences: 2
- [MazeMap/MazeMapTest/SrUkfCoreBiasAndStationaryTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SrUkfCoreBiasAndStationaryTest.cpp): [line 23](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SrUkfCoreBiasAndStationaryTest.cpp:23)
- [MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp): [line 16](C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/SrUkfCoreModeAndDiagnosticsTest.cpp:16)
