# Logging Migration Ledger

## Moved To Direct `MmLogLogger`

- `DiagnosticController`
  - Owns `MazeMap::mmlog::MmLogLogger` directly for `DiagnosticLogRow`.
  - Verbose tuning/config detail stays in centralized `logging.txt` events to stay within stock metadata limits.
- `MissionController` telemetry modes
  - Own `MazeMap::mmlog::MmLogLogger` directly for `DiagnosticLogRow`.
  - Maneuver test, corridor repeatability, and position audit telemetry now share the same direct pattern.
- `AuxMeasurementController`
  - Owns `MazeMap::mmlog::MmLogLogger` directly for `AuxMeasurementLogRow`.
  - Routine-specific config detail stays in centralized `logging.txt` events.
- `OpenFloorMeasurementController`
  - Owns direct `MazeMap::mmlog::MmLogLogger` instances for `OpenFloorTimingRow` and `OpenFloorMainRow`.
  - Section markers and faults now write to centralized `logging.txt` without an intermediate logger class.
- Front-wall characterization export in `MazeMapStandaloneModes.cpp`
  - Row definition now uses `MMLOG_DEFINE_ROW(FrontWallCharacterizationLogRow, ...)`.
  - Writes `.mmlog` data directly through `MazeMap::mmlog::MmLogLogger`.

## Centralized Text Event Path

- Mission/application text events now use the centralized runtime control log backend in `logging.txt`.
- `OptionalRuntimeEventLog` now targets the centralized `logging.txt` file instead of a datalog sidecar adapter.
- The centralized file is kept open once opened and is only closed on write fault.

## Rejected And Removed

- `RuntimeBinaryLogFile`
- `RuntimeEventLogFile` (`MazeMapRuntimeCsvLog`)
- `RuntimeBinaryRecordSupport`
- `RuntimeInfrastructureSupport`
- `RuntimeRecordBuilder`
- `RuntimeTextBlockBuilder`
- `DiagnosticLogger`
- `AuxMeasurementLogger`
- `OpenFloorMainLogger`
- `OpenFloorTimingLogger`
- `OpenFloorMainLoggerV2` logger class and `.cpp` adapter
- `OpenFloorTimingLoggerV2` logger class and `.cpp` adapter

## Remaining

- No active logging subsystem remains on the legacy runtime-binary adapter stack.
- No logger class remains between application sources and `MazeMap::mmlog::MmLogLogger`.
- Remaining logging-specific support outside `MmLog` is limited to row-definition headers, metadata helpers, and the centralized `logging.txt` event sink.
- Teensy firmware verification is currently blocked in upstream `MmLog.cpp` because the Arduino `FsFile::flush()` API resolves to `void`, while `MmLog.cpp` tests it as a `bool` at lines 294, 383, and 386.
