# Teensy Code Size Audit

Generated: 2026-04-18 17:58:16 America/Chicago

## Scope

This audit examines [MazeMap.ino.hex](/C:/Users/thene/source/repos/MicroMouse2025/codex_verify/arduino_build/firmware/MazeMap.ino.hex) through its matching symbol-bearing ELF, [MazeMap.ino.elf](/C:/Users/thene/source/repos/MicroMouse2025/codex_verify/arduino_build/firmware/MazeMap.ino.elf), from [build_latest_20260418_091620_507.txt](/C:/Users/thene/source/repos/MicroMouse2025/codex_verify/logs/build_latest_20260418_091620_507.txt).

The copied `firmware/sketch` sources were checked against the current dirty workspace. This image matches the current source snapshot apart from Arduino's injected `#line` wrappers.

The focus here is RAM1 code. RAM2 and flash are intentionally excluded.

## Reconciliation

RAM1 code closes against the compiler number:

- `.text.itcm`: `402,736`
- Named function ranges attributed: `402,674`
- Unattributed thunks/padding inside `.text.itcm`: `62`
- `.ARM.exidx`: `8`
- Total vs compiler `RAM1 code`: `402,744`

## Main Owning Objects

Main owning objects and largest first-layer members:

- `MissionModeController::Implementation`: `29,206`
  `RunStartupWallCalibration` `4,320`, `SearchStraightLoopTick` `3,964`, `ExecuteSearchStraightCellsLoopDriven` `1,860`, `InterRunServicePauseThunk` `1,812`
- `MazeRunningAuditController::Implementation`: `27,986`
  `RunSinglePositionSmoothTurnAuditPass` `4,696`, `RunStartupWallCalibration` `3,856`, `RunPositionAccuracyAuditPasses` `3,240`, `QueuedManeuverLoopTick` `2,304`
- `OpenFloorMeasurementController`: `24,636`
  `HandleRuntimeFault` `4,016`, `EmitPendingLog` `3,140`, `Begin` `2,160`, `RecoverToMarkerTick` `1,596`
- `SrUkfCore`: `23,566`
  `updateYawRateImpl` `6,816`, `updateEncoderPairImpl` `5,164`, `predictImpl` `3,700`, `applyStationaryZeroMotionConstraint` `3,022`
- `DriveBase`: `22,296`
  constructor `4,728`, `UpdatePoseEstimate<...>` `4,422`, `PointCommand` `2,986`, `ResolveRawAccelerationCommand` `2,592`, `CaptureCommandContext` `1,316`, `ResetPoseEstimate` `1,252`
- `RuntimeSensorSuite`: `10,400`
  `Capture` `4,596`, `Begin` `3,172`, `CaptureInertialSnapshot` `1,452`
- Standalone calibration helpers in `MazeMapRuntimeCore.h`:
  `CalibrateStationaryBackLeftGyroBias` `8,580`, `SampleWallCalibrationCaptureAverageRawPair` `8,180`, `SampleWallCalibrationCaptureAverageRaw` `4,960`
- `LoopController`: `7,376`
  `Run` `4,452`, `BeginSession` `652`, `ResetSessionState` `544`, `CaptureTickState` `468`
- `PlantModel`: `7,156`
  `forwardStep` `3,744`, `solveDriveCommands` `3,084`
- `ManeuverExecutor`: `6,804`
  `QueueDispatchRoutineTick` `1,924`, `StraightRoutineTick` `1,108`
- `Drive`: `5,624`
  `GetNextControls` `4,554`, `StartStraight` `784`
- `SharedRobotRuntime`: `5,336`
  `OpenUtilityDataLogFile` `1,152`, `EnsureTextLogOpen` `588`, `ServiceUtilityDataLog` `408`

## First-Layer Rollups

Useful first-layer rollups:

- Under `MazeMap`: `App` `125,634`, `SrUkfCore` `23,566`, `PlantModel` `7,156`, `mmlog` `4,218`, `ManeuverPathFinder` `4,034`, `FloodFillPathFinder` `2,106`
- Under `MazeMap::App::Internal`: `MissionModeController` `29,206`, `MazeRunningAuditController` `27,986`, `PositionAccuracyAuditMode` `8,488`, `ManeuverFileTestMode` `7,812`, `LoopController` `7,376`, `Runtime` `7,114`, `ManeuverExecutor` `6,804`, `TopSpeedMeasurementMode` `6,378`, `Drive` `5,624`, `SharedRobotRuntime` `5,336`

## External Code Buckets

External code worth noting:

- newlib/libm/printf bucket is still large; biggest pieces are `_svfprintf_r` `6,980`, `_vfiprintf_r` `3,980`, `_dtoa_r` `3,308`
- SD/SdFat is also material: `FatFile` `5,718`, `ExFatFile` `5,124`, `SDClass` `5,008`, `SdioCard` path `6,024`

## Verification Status

Release verification is not green. [verify_latest_build_20260418_093008_364.txt](/C:/Users/thene/source/repos/MicroMouse2025/codex_verify/logs/verify_latest_build_20260418_093008_364.txt) accepted this firmware as current, but `DriveBasePointCommandImuYawTrackingChangesCommandWhenYawRateErrorExists` failed at [DriveBaseTest.cpp](/C:/Users/thene/source/repos/MicroMouse2025/MazeMap/MazeMapTest/DriveBaseTest.cpp:406).
