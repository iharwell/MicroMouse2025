#pragma once

#include "Defines.h"
#include "BootFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "Estimator.h"
#include "FloodFillPathFinder.h"
#include "LoopController.h"
#include "ManeuverExecutor.h"
#include "ManeuverPathFinder.h"
#include "Maze.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "MmLog.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "StartupCalibration.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "WallBeliefMap.h"
#include "WallTouch.h"

namespace MazeMap::App::Internal
{
    // Canonical runtime-owned human-readable log file name.
    inline constexpr const char* kSharedRuntimeTextLogFileName = "logging.txt";

    // Fixed queue capacity reserved for the runtime-owned logging.txt ring buffer.
    inline constexpr std::size_t kSharedRuntimeTextLogQueueBytes = 4096U;
    static_assert((kSharedRuntimeTextLogQueueBytes % MazeMap::mmlog::kSdSectorBytes) == 0U);

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
    // Platform-selected file handle type used for the one runtime-owned text log file.
    //
    // On Teensy FIFO SDIO builds, the runtime text log uses the SdFat FsFile handle.
    using SharedRuntimeTextLogFileHandle = FsFile;
#elif defined(ARDUINO)
    // Platform-selected file handle type used for the one runtime-owned text log file.
    //
    // On non-FIFO Arduino builds, the runtime text log uses the Arduino File handle.
    using SharedRuntimeTextLogFileHandle = File;
#else
    // Platform-selected file handle type used for the one runtime-owned text log file.
    //
    // On host builds, the runtime text log uses a stdio FILE handle.
    using SharedRuntimeTextLogFileHandle = std::FILE*;
#endif

    // Single production owner of the heavyweight robot runtime infrastructure.
    //
    // Ownership contract:
    // SharedRobotRuntime owns the production vehicle, maze, estimator, pathfinders, sensors,
    // drive helpers, maneuver executor, one LoopController instance, and the runtime logging
    // objects that top-level modes must share instead of duplicating.
    //
    // Mode interaction model:
    // Top-level modes borrow references from this runtime during SetupMode(...) and RunTick(...)
    // rather than constructing their own parallel subsystem owners. Application infrastructure
    // resolves the active mode, then the mode and any loop-routine callbacks operate against this
    // shared runtime.
    //
    // Failure model:
    // One top-level mode may register one fault-time cleanup callback. FailActiveMode(...) runs
    // that cleanup if present, shuts down shared runtime resources, and traps execution
    // permanently. It is not a recoverable status-return API.
    class EXPORT SharedRobotRuntime final
    {
    public:
        // Optional top-level mode fault-time cleanup callback registered through
        // RegisterModeFaultHandler(...).
        //
        // Parameters:
        // `context`:
        // Caller-owned pointer supplied when the handler is registered.
        //
        // `reason`:
        // Terminal fault reason forwarded from FailActiveMode(...).
        //
        // Behavior:
        // The callback runs only on the terminal FailActiveMode(...) path, after the runtime has
        // already decided that ordinary control flow is over and before execution is trapped.
        using ModeFaultCleanupCallback = void (*)(void* context, const char* reason) noexcept;

        // Constructs the one production runtime object and wires shared services back to it.
        //
        // Behavior:
        // - Creates the canonical production subsystem owners.
        // - Attaches internal services that need a SharedRobotRuntime back-reference.
        SharedRobotRuntime();
        explicit SharedRobotRuntime(float nominalCommandPeriodSeconds);

        // Best-effort final cleanup for runtime-owned logs during object destruction.
        //
        // Behavior:
        // Closes runtime-owned log resources through the fault-style log shutdown helper. This
        // destructor does not make the runtime fault path recoverable or restartable.
        ~SharedRobotRuntime();

        // `SharedRobotRuntime(const SharedRobotRuntime&)`:
        // SharedRobotRuntime is the unique production runtime owner; copying is forbidden.
        SharedRobotRuntime(const SharedRobotRuntime&) = delete;
        // `operator=(const SharedRobotRuntime&)`:
        // SharedRobotRuntime is the unique production runtime owner; copy assignment is forbidden.
        SharedRobotRuntime& operator=(const SharedRobotRuntime&) = delete;
        // `SharedRobotRuntime(SharedRobotRuntime&&)`:
        // SharedRobotRuntime is the unique production runtime owner; moving is forbidden.
        SharedRobotRuntime(SharedRobotRuntime&&) = delete;
        // `operator=(SharedRobotRuntime&&)`:
        // SharedRobotRuntime is the unique production runtime owner; move assignment is forbidden.
        SharedRobotRuntime& operator=(SharedRobotRuntime&&) = delete;

        // `BootFramework()`:
        // Returns the runtime-owned boot lifecycle coordinator.
        //
        // Behavior:
        // Application infrastructure uses this owner to resolve and run the selected peer boot
        // mode. The framework borrows runtime resources; it does not own logs, loop state, maze
        // state, pathfinders, or other heavy subsystems.
        BootFramework& BootFramework() noexcept;

        // `Vehicle()`:
        // Returns the mutable canonical production Vehicle owner.
        //
        // Behavior:
        // Modes and shared services use this owner for robot construction facts and setup.
        MazeMap::Vehicle& Vehicle() noexcept;

        // `Vehicle() const`:
        // Returns the read-only canonical production Vehicle owner.
        //
        // Behavior:
        // Exposes canonical robot facts without permitting mutation.
        const MazeMap::Vehicle& Vehicle() const noexcept;

        // `Maze()`:
        // Returns the mutable production Maze owner.
        //
        // Behavior:
        // Exposes the one canonical runtime maze instance for authoritative maze-domain work.
        MazeMap::Maze& Maze() noexcept;

        // `Maze() const`:
        // Returns the read-only production Maze owner.
        //
        // Behavior:
        // Exposes the one canonical runtime maze instance without permitting mutation.
        const MazeMap::Maze& Maze() const noexcept;

        // `SearchPathFinder()`:
        // Returns the production FloodFill pathfinder owner used for search-style navigation.
        //
        // Behavior:
        // Exposes the one runtime-owned flood-fill planner rather than allowing modes to create
        // duplicate search planners.
        MazeMap::FloodFillPathFinder& SearchPathFinder() noexcept;

        // `SpeedPathFinder()`:
        // Returns the production ManeuverPathFinder owner used for maneuver/native planning.
        //
        // Behavior:
        // Exposes the one runtime-owned maneuver/native path planner.
        MazeMap::ManeuverPathFinder& SpeedPathFinder() noexcept;

        // `WallBeliefMap()`:
        // Returns the production WallBeliefMap owner.
        //
        // Behavior:
        // Exposes the shared wall-belief state used across runtime services.
        MazeMap::WallBeliefMap& WallBeliefMap() noexcept;

        // `OpenUtilityDataLog(...)`:
        // Selects a runtime file name and opens it as the active utility data log.
        //
        // Parameters:
        // `resolvedFileName`:
        // Caller-provided output buffer that receives the selected file name.
        //
        // `bufferSize`:
        // Capacity of `resolvedFileName`.
        //
        // `explicitFileName`:
        // Optional caller-requested exact file name. When null or empty, runtime naming fallback
        // logic selects a sequential file name from the provided format strings.
        //
        // `teensyFormat`:
        // Teensy-side sequential naming format used when `explicitFileName` is not provided.
        //
        // `hostFallback`:
        // Host-side fallback name used when `explicitFileName` is not provided.
        //
        // Return value:
        // `true` if name selection succeeded and the resulting file was opened as the active
        // utility data log. `false` if name selection failed or if OpenUtilityDataLogFile(...)
        // failed for the selected name.
        bool OpenUtilityDataLog(
            char* resolvedFileName,
            std::size_t bufferSize,
            const char* explicitFileName,
            const char* teensyFormat,
            const char* hostFallback);

        // `OpenUtilityDataLogFile(fileName)`:
        // Opens `fileName` as the active utility data log on the runtime-owned MmLogLogger.
        //
        // Parameters:
        // `fileName`:
        // Target utility-data-log file name. Null or empty names are rejected.
        //
        // Behavior:
        // - Clears LastRuntimeLogError().
        // - Closes any previously active utility data log first.
        // - Ensures the runtime-owned text log is open.
        // - Opens the runtime-owned utility logger on `fileName`.
        // - Writes standard `file` and `control_log_file` metadata rows on success.
        //
        // Return value:
        // `true` if the file was opened and the standard metadata writes succeeded.
        // `false` if the name is invalid, the text log could not be opened, the logger open
        // failed, or the required metadata writes failed.
        bool OpenUtilityDataLogFile(const char* fileName);

        // `ActiveUtilityDataLogFileName()`:
        // Returns the currently active utility-data-log file name.
        //
        // Return value:
        // Empty string when no utility data log is active; otherwise the runtime-retained active
        // file name.
        const char* ActiveUtilityDataLogFileName() const noexcept;

        // `TextLogFileName()`:
        // Returns the canonical runtime-owned human-readable text-log file name.
        //
        // Return value:
        // The fixed runtime text-log name, currently `logging.txt`.
        const char* TextLogFileName() const noexcept;

        // `LastRuntimeLogError()`:
        // Returns the latest runtime/logging error text captured by this owner.
        //
        // Return value:
        // Empty string when no current runtime/logging error text is stored.
        const char* LastRuntimeLogError() const noexcept;

        // `EnsureTextLogOpen()`:
        // Opens the one runtime-owned logging.txt file if it is not already open.
        //
        // Behavior:
        // - Returns `true` immediately if the file is already open.
        // - Creates a fresh file on first open.
        // - Reopens in append mode on later opens after a normal close.
        // - Refuses to reopen once the text log has faulted or the runtime fault path has closed
        //   runtime logs.
        //
        // Return value:
        // `true` if the text log is open on return. `false` if open failed or reopening is no
        // longer allowed because the log/runtime is fault-closed.
        bool EnsureTextLogOpen();

        // `TextLogIsOpen()`:
        // Returns whether the runtime-owned text-log file handle is presently open.
        //
        // This is a handle-state query only. It does not guarantee that the log has not faulted
        // previously or that future writes will succeed.
        //
        // Return value:
        // `true` if the underlying text-log handle is presently open. `false` otherwise.
        bool TextLogIsOpen() noexcept;

        // `AppendTextLogLine(line)`:
        // Enqueues one caller-supplied human-readable line plus newline to the runtime text-log
        // queue.
        //
        // Parameters:
        // `line`:
        // Text line to append. Null and empty strings are rejected.
        //
        // Return value:
        // `true` if the line was queued successfully.
        // `false` if the input is invalid, the text log could not be opened, or the queue
        // overflowed. Queue overflow faults the text log and closes runtime logs.
        bool AppendTextLogLine(const char* line);

        // `AppendTextLogFormatted(format, ...)`:
        // Formats one line into a fixed internal buffer and then enqueues it through
        // AppendTextLogLine(...).
        //
        // Parameters:
        // `format`:
        // printf-style format string. Null is rejected.
        //
        // Return value:
        // `true` if formatting succeeded and the resulting line was queued.
        // `false` if formatting failed or AppendTextLogLine(...) failed.
        bool AppendTextLogFormatted(const char* format, ...);

        // `WriteTextLogEntry(source, timestampUs, type, message)`:
        // Formats and enqueues one structured human-readable event line.
        //
        // Parameters:
        // `source`:
        // Optional source label. Null or empty falls back to `"runtime"`.
        //
        // `timestampUs`:
        // Timestamp to print into the line.
        //
        // `type`:
        // Optional entry type label. Null or empty falls back to `"event"`.
        //
        // `message`:
        // Optional free-form message payload.
        //
        // Return value:
        // `true` if the formatted line was queued successfully. `false` if formatting or queueing
        // failed.
        bool WriteTextLogEntry(
            const char* source,
            unsigned long timestampUs,
            const char* type,
            const char* message);

        // `WriteTextLogEntry(timestampUs, type, message)`:
        // Writes a structured text-log entry using the runtime-selected default source label.
        //
        // The default source prefers the active utility-data-log-derived source label, then the
        // registered mode-fault source label, then `"runtime"`.
        bool WriteTextLogEntry(
            unsigned long timestampUs,
            const char* type,
            const char* message);

        // `WriteTextLogMetadata(key, value)`:
        // Writes one `metadata` text-log entry using the runtime-selected default source label.
        //
        // Parameters:
        // `key`:
        // Metadata key to encode.
        //
        // `value`:
        // Metadata value to encode.
        //
        // Return value:
        // `true` if formatting and queueing succeeded. `false` otherwise.
        bool WriteTextLogMetadata(const char* key, const char* value);

        // `WriteTextLogMetadata(source, key, value)`:
        // Writes one `metadata` text-log entry with payload `key=value`.
        //
        // Parameters:
        // `source`:
        // Optional source label. Null or empty falls back to the default source-selection logic.
        //
        // `key`:
        // Metadata key to encode.
        //
        // `value`:
        // Metadata value to encode.
        //
        // Return value:
        // `true` if formatting and queueing succeeded. `false` otherwise.
        bool WriteTextLogMetadata(const char* source, const char* key, const char* value);

        // `WriteTextLogPhase(phaseId, timestampUs, name)`:
        // Writes one `phase` text-log entry using the runtime-selected default source label.
        //
        // Parameters:
        // `phaseId`:
        // Caller-defined phase identifier to print.
        //
        // `timestampUs`:
        // Timestamp to print into the line.
        //
        // `name`:
        // Optional phase name payload.
        //
        // Return value:
        // `true` if formatting and queueing succeeded. `false` otherwise.
        bool WriteTextLogPhase(
            unsigned long phaseId,
            unsigned long timestampUs,
            const char* name);

        // `WriteTextLogPhase(source, phaseId, timestampUs, name)`:
        // Writes one `phase` text-log entry with payload `phase_id=<id>;name=<name>`.
        //
        // Parameters:
        // `source`:
        // Optional source label. Null or empty falls back to the default source-selection logic.
        //
        // `phaseId`:
        // Caller-defined phase identifier to print.
        //
        // `timestampUs`:
        // Timestamp to print into the line.
        //
        // `name`:
        // Optional phase name payload.
        //
        // Return value:
        // `true` if formatting and queueing succeeded. `false` otherwise.
        bool WriteTextLogPhase(
            const char* source,
            unsigned long phaseId,
            unsigned long timestampUs,
            const char* name);

        // `FlushTextLog()`:
        // Forces the queued logging.txt bytes through the runtime-owned text-log file now.
        //
        // Behavior:
        // - Best-effort lifecycle operation.
        // - On flush failure, records LastRuntimeLogError(), faults the text log, and closes
        //   runtime logs.
        void FlushTextLog();

        // `CloseTextLog()`:
        // Flushes and closes the runtime-owned logging.txt file, then clears the text-log queue.
        //
        // Behavior:
        // - Reserved for runtime shutdown/fault handling; normal mode code should not call it.
        // - On flush/close failure, records LastRuntimeLogError(), faults the text log, closes
        //   the underlying file handle, and clears the queue anyway.
        void CloseTextLog();

        // `RegisterModeFaultHandler(callback, context, source)`:
        // Registers the one optional top-level mode fault-time cleanup callback.
        //
        // Parameters:
        // `callback`:
        // Optional cleanup callback to run on FailActiveMode(...).
        //
        // `context`:
        // Caller-owned context pointer passed back to `callback`.
        //
        // `source`:
        // Optional source label retained for later terminal fault logging.
        //
        // Return value:
        // `true` if registration succeeded.
        //
        // Failure behavior:
        // Registering more than one top-level mode fault handler is rejected because runtime
        // top-level mode switching is not supported. Host builds throw; Teensy builds assert and
        // return `false`.
        bool RegisterModeFaultHandler(
            ModeFaultCleanupCallback callback,
            void* context,
            const char* source);

        // `FailActiveMode(reason)`:
        // Terminal active-mode failure path.
        //
        // Behavior:
        // - Runs the registered mode-specific fault cleanup callback if present.
        // - Brakes shared actuation, closes shared logs, and starts the runtime fault indicator.
        // - Traps execution permanently. Ordinary control flow does not continue past this call.
        [[noreturn]] void FailActiveMode(const char* reason) noexcept;

        // `FinalizeSuccessfulModeExit()`:
        // Finalizes the one normal terminal path after LoopController::Run() returns for
        // HaltExecutionEndProgram().
        //
        // Behavior:
        // - No-op if the mode already faulted or runtime logs were fault-closed.
        // - Closes the active utility data log if open.
        // - Closes the runtime-owned text log if open.
        //
        // This is not an end-session handoff path and not a fault path.
        void FinalizeSuccessfulModeExit() noexcept;

        // `WriteUtilityDataLogMetadata(key, value)`:
        // Writes one string metadata pair to the active utility data log.
        //
        // Return value:
        // `true` if the metadata write succeeded. `false` if the logger rejected the write, in
        // which case LastRuntimeLogError() is updated and a runtime text-log failure marker is
        // emitted when possible.
        bool WriteUtilityDataLogMetadata(const char* key, const char* value);

        // `WriteUtilityDataLogMetadataUnsigned(key, value)`:
        // Writes one unsigned metadata value to the active utility data log.
        //
        // Parameters:
        // `key`:
        // Metadata key to encode.
        //
        // `value`:
        // Unsigned metadata value to encode.
        //
        // Return value:
        // `true` if the metadata write succeeded. `false` if the logger rejected the write.
        bool WriteUtilityDataLogMetadataUnsigned(const char* key, unsigned long value);

        // `WriteUtilityDataLogMetadataFloat(key, value, precision)`:
        // Writes one floating-point metadata value to the active utility data log using the
        // provided decimal precision.
        //
        // Parameters:
        // `key`:
        // Metadata key to encode.
        //
        // `value`:
        // Floating-point metadata value to encode.
        //
        // `precision`:
        // Decimal precision forwarded to the logger.
        //
        // Return value:
        // `true` if the metadata write succeeded. `false` if the logger rejected the write.
        bool WriteUtilityDataLogMetadataFloat(const char* key, float value, std::uint8_t precision);

        // `WriteUtilityDataLogAccelBiasMetadata(sensors)`:
        // Writes the runtime sensor suite's accel-bias metadata to the active utility data log.
        //
        // Parameters:
        // `sensors`:
        // Runtime sensor suite whose current accel-bias metadata should be written.
        //
        // Return value:
        // `true` if all required metadata writes succeeded. `false` otherwise.
        bool WriteUtilityDataLogAccelBiasMetadata(const RuntimeSensorSuite& sensors);

        // `BeginUtilityDataLogSchema(row)`:
        // Begins the active utility data log with the supplied schema row type.
        //
        // Parameters:
        // `row`:
        // Schema-defining row object forwarded to the runtime-owned MmLogLogger.
        //
        // Return value:
        // `true` if schema begin succeeded. `false` if the logger rejected the begin request, in
        // which case LastRuntimeLogError() is updated and a runtime text-log failure marker is
        // emitted when possible.
        template <typename Row>
        bool BeginUtilityDataLogSchema(const Row& row)
        {
            ClearLastRuntimeLogError();
            const bool ok = UtilityDataLogger().begin(row);
            if (!ok)
            {
                SetLastRuntimeLogErrorFromUtilityDataLogger("Failed to begin utility data log schema.");
                LogUtilityDataLoggerFailure("data_log_begin_failed");
            }
            return ok;
        }

        // `LogUtilityDataRow(row)`:
        // Forwards one row to the runtime-owned utility data logger.
        //
        // Return value:
        // Exact pass-through result of MmLogLogger::log(row). This helper does not add extra
        // LastRuntimeLogError() handling around that call.
        template <typename Row>
        bool LogUtilityDataRow(const Row& row)
        {
            return UtilityDataLogger().log(row);
        }

        // `ServiceUtilityDataLog()`:
        // Runs the one runtime-owned log/file service hub.
        //
        // Behavior:
        // - Clears LastRuntimeLogError() first.
        // - Returns `false` immediately once logs are fault-closed.
        // - If a text-log or utility-log transfer is already busy, returns `true` without forcing
        //   more work.
        // - Prioritizes draining full logging.txt sectors before servicing the utility logger.
        //
        // Return value:
        // `true` if no fault occurred during this service call.
        // `false` if text-log drain failed or if utility-log service failed.
        bool ServiceUtilityDataLog();

        // `FlushUtilityDataLog()`:
        // Forces the active utility data log to flush now.
        //
        // Return value:
        // `true` if flush succeeded. `false` if the logger flush failed, in which case
        // LastRuntimeLogError() is updated.
        bool FlushUtilityDataLog();

        // `CloseUtilityDataLog()`:
        // Closes the active utility data log and clears the retained active data-log identity.
        //
        // Return value:
        // `true` if close succeeded. `false` if the logger close failed, in which case
        // LastRuntimeLogError() is updated. The retained active file/source identity is cleared
        // regardless.
        bool CloseUtilityDataLog();

        // `CaptureUtilityDataLogFailure(overflowed, writeFailed)`:
        // Snapshots the utility logger's internal failure state into caller-provided booleans.
        //
        // Parameters:
        // `overflowed`:
        // Receives whether the logger recorded an internal queue/storage overflow condition.
        //
        // `writeFailed`:
        // Receives whether the logger recorded a write failure condition.
        void CaptureUtilityDataLogFailure(bool& overflowed, bool& writeFailed) const noexcept;

        // `DriveBase()`:
        // Returns the concrete low-level DriveBase owner used for raw command application and
        // measurement interpretation.
        //
        // Behavior:
        // Exposes the canonical low-level drive helper rather than a duplicate actuation owner.
        DriveBase& DriveBase() noexcept;

        // `Estimator()`:
        // Returns the production estimator owner.
        //
        // Behavior:
        // Exposes the authoritative runtime estimator instance.
        MazeMap::Estimator& Estimator() noexcept;

        // `Plant()`:
        // Returns the production plant model owner.
        MazeMap::PlantModel& Plant() noexcept;
        const MazeMap::PlantModel& Plant() const noexcept;

        // `RuntimeState()`:
        // Returns the authoritative live runtime VehicleState owner.
        //
        // Behavior:
        // Exposes the shared mutable state object updated by sensing and estimation.
        MazeMap::VehicleState& RuntimeState() noexcept;

        // `DriveService()`:
        // Returns the shared higher-level Drive service.
        //
        // Behavior:
        // Exposes the canonical multi-tick motion helper shared across top-level modes.
        MazeMap::App::Internal::Drive& DriveService() noexcept;

        // `StartupCalibrationService()`:
        // Returns the shared startup-calibration service.
        //
        // Behavior:
        // Exposes the runtime-owned startup-calibration helper.
        StartupCalibration& StartupCalibrationService() noexcept;

        // `WallTouchService()`:
        // Returns the shared wall-touch service.
        //
        // Behavior:
        // Exposes the runtime-owned wall-touch helper.
        WallTouch& WallTouchService() noexcept;

        // `ManeuverExecutorService()`:
        // Returns the shared maneuver-execution owner.
        //
        // Behavior:
        // Exposes the canonical runtime-owned maneuver executor.
        ManeuverExecutor& ManeuverExecutorService() noexcept;

        // `ControlLoop()`:
        // Returns the one production LoopController instance.
        //
        // Application infrastructure owns the terminal Run() boundary; borrowers use this
        // reference for staging, timing reads, and other non-terminal loop interaction.
        LoopController& ControlLoop() noexcept;

        // `Sensors()`:
        // Returns the shared runtime sensor-suite owner.
        //
        // Behavior:
        // Exposes the canonical runtime sensing owner used by LoopController and setup code.
        RuntimeSensorSuite& Sensors() noexcept;

    private:
        mmlog::MmLogLogger& UtilityDataLogger() noexcept;
        const mmlog::MmLogLogger& UtilityDataLogger() const noexcept;
        const char* DefaultTextLogSource() const noexcept;
        void ClearLastRuntimeLogError() noexcept;
        void SetLastRuntimeLogError(const char* message) noexcept;
        void SetLastRuntimeLogErrorFromUtilityDataLogger(const char* fallback) noexcept;
        void LogUtilityDataLoggerFailure(const char* type) noexcept;
        void CloseRuntimeLogsForFault() noexcept;

        static constexpr std::size_t kTextLogSourceLength = 64U;

        MazeMap::Vehicle vehicle;                        // Canonical production vehicle facts and devices.
        MazeMap::Maze maze;                              // Production canonical maze instance.
        MazeMap::FloodFillPathFinder searchPathFinder;   // Production flood-fill owner.
        MazeMap::ManeuverPathFinder speedPathFinder;     // Production maneuver pathfinder owner.
        MazeMap::WallBeliefMap wallBeliefMap;            // Production wall-belief owner.
        MazeMap::VehicleState runtimeState;              // Authoritative live runtime state.
        MazeMap::PlantModel plantModel;                  // Shared plant model owner.
        MazeMap::Estimator estimator;                    // Shared estimator owner.
        MazeMap::DriveBase driveBase;                    // Concrete low-level drive command owner.
        MazeMap::App::Internal::Drive driveService;      // Shared multi-tick Drive service.
        StartupCalibration startupCalibrationService;    // Shared startup-calibration service.
        WallTouch wallTouchService;                      // Shared wall-touch service.
        ManeuverExecutor maneuverExecutor;               // Shared maneuver-execution owner.
        LoopController controlLoop;                      // One production LoopController instance.
        RuntimeSensorSuite sensors;                      // Shared runtime sensing owner.
        MazeMap::App::Internal::BootFramework bootFramework; // Boot-mode lifecycle coordinator.
        mmlog::MmLogLogger dataLogger;                   // One runtime-owned utility-data logger.
        SharedRuntimeTextLogFileHandle textLogFile{};    // One runtime-owned logging.txt handle.
        mmlog::detail::ByteRing textLogQueue;            // Runtime-owned logging.txt queue/arbitration buffer.
#if !MAZEMAP_USE_RAM2_FILE_BUFFERS
        alignas(32) std::uint8_t textLogStorage[kSharedRuntimeTextLogQueueBytes]{};
#endif
        bool textLogFaulted{};                           // Whether logging.txt has faulted irrecoverably.
        bool textLogInitialized{};                       // Whether logging.txt has been opened at least once.
        bool runtimeLogsClosedForFault{};                // Whether fault-path shutdown already closed logs.
        bool modeFaultHandlerRegistered{};               // Whether the active top-level mode installed cleanup.
        bool modeFaulted{};                              // Whether FailActiveMode(...) has already taken ownership.
        ModeFaultCleanupCallback modeFaultCleanupCallback{}; // Optional top-level mode fault-time cleanup.
        void* modeFaultCleanupContext{};                 // Context paired with modeFaultCleanupCallback.
        char activeDataLogFileName[MMLOG_MAX_PATH_LENGTH + 1U]{}; // Current utility-data-log file name.
        char activeDataLogSource[kTextLogSourceLength]{};         // Default source label for runtime text logging.
        char activeModeFaultSource[kTextLogSourceLength]{};       // Fault-source label for terminal mode failures.
        char lastRuntimeLogError[MMLOG_ERROR_TEXT_LENGTH + 1U]{}; // Latest runtime/logging error text.
    };

    // `LogWallSensorAdcRegisterWrite(phase, expectedCfg, readCfg, readGc)`:
    // Emits one diagnostic record for a wall-sensor ADC register write/readback mismatch.
    //
    // Parameters:
    // `phase`:
    // Short caller-supplied label describing the setup/verification phase being logged.
    //
    // `expectedCfg`:
    // Expected ADC CFG register contents.
    //
    // `readCfg`:
    // CFG value read back from the hardware.
    //
    // `readGc`:
    // GC value read back from the hardware.
    EXPORT void LogWallSensorAdcRegisterWrite(
        const char* phase,
        std::uint32_t expectedCfg,
        std::uint32_t readCfg,
        std::uint32_t readGc) noexcept;

    // `GetSharedRobotRuntime()`:
    // Returns the process-global production SharedRobotRuntime owner.
    //
    // Behavior:
    // Application infrastructure and top-level modes use this as the canonical entry point for
    // shared runtime subsystem ownership.
    EXPORT SharedRobotRuntime& GetSharedRobotRuntime();
}

