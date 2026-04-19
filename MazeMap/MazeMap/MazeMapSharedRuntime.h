#pragma once
// Declares the shared runtime object that wires together vehicles, planners, sensors, and drive services for the app.

#include "Defines.h"
#include "Drive.h"
#include "DriveBase.h"
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
#include "WallBeliefMap.h"
#include "WallTouch.h"

namespace MazeMap::App::Internal
{
    inline constexpr const char* kSharedRuntimeTextLogFileName = "logging.txt";
    inline constexpr std::size_t kSharedRuntimeTextLogQueueBytes = 4096U;
    static_assert((kSharedRuntimeTextLogQueueBytes % MazeMap::mmlog::kSdSectorBytes) == 0U);

#if MMLOG_ENABLE_TEENSY_FIFO_SDIO
    using SharedRuntimeTextLogFileHandle = FsFile;
#elif defined(ARDUINO)
    using SharedRuntimeTextLogFileHandle = File;
#else
    using SharedRuntimeTextLogFileHandle = std::FILE*;
#endif

    // Owns the heavyweight runtime infrastructure that should be shared across all startup modes.
    // Mode implementations stay small by borrowing references from this hub instead of allocating
    // duplicate vehicle, estimator, pathfinding, and sensor state.
    class EXPORT SharedRobotRuntime final
    {
    public:
        using ModeFaultCleanupCallback = void (*)(void* context, const char* reason) noexcept;

        SharedRobotRuntime();
        ~SharedRobotRuntime();

        SharedRobotRuntime(const SharedRobotRuntime&) = delete;
        SharedRobotRuntime& operator=(const SharedRobotRuntime&) = delete;
        SharedRobotRuntime(SharedRobotRuntime&&) = delete;
        SharedRobotRuntime& operator=(SharedRobotRuntime&&) = delete;

        MazeMap::Vehicle& SpeedVehicle() noexcept;
        const MazeMap::Vehicle& SpeedVehicle() const noexcept;

        MazeMap::Vehicle& SearchVehicle() noexcept;
        const MazeMap::Vehicle& SearchVehicle() const noexcept;

        MazeMap::Maze& Maze() noexcept;
        const MazeMap::Maze& Maze() const noexcept;

        MazeMap::FloodFillPathFinder& SearchPathFinder() noexcept;
        MazeMap::ManeuverPathFinder& SpeedPathFinder() noexcept;
        MazeMap::WallBeliefMap& WallBeliefMap() noexcept;

        bool OpenUtilityDataLog(
            char* resolvedFileName,
            std::size_t bufferSize,
            const char* explicitFileName,
            const char* teensyFormat,
            const char* hostFallback);
        bool OpenUtilityDataLogFile(const char* fileName);
        const char* ActiveUtilityDataLogFileName() const noexcept;
        const char* TextLogFileName() const noexcept;
        const char* LastRuntimeLogError() const noexcept;

        // logging.txt is runtime infrastructure, not a mode-owned file. Modes may write text
        // entries and metadata through SharedRobotRuntime, but they should not manage the file
        // lifetime directly. Treat it like Serial/printf/std::cout: expected to exist for the
        // whole boot-to-shutdown session unless the runtime fault/shutdown path tears it down.
        bool EnsureTextLogOpen();
        bool TextLogIsOpen() noexcept;
        bool AppendTextLogLine(const char* line);
        bool AppendTextLogFormatted(const char* format, ...);
        bool WriteTextLogEntry(
            const char* source,
            unsigned long timestampUs,
            const char* type,
            const char* message);
        bool WriteTextLogEntry(
            unsigned long timestampUs,
            const char* type,
            const char* message);
        bool WriteTextLogMetadata(const char* key, const char* value);
        bool WriteTextLogMetadata(const char* source, const char* key, const char* value);
        bool WriteTextLogPhase(
            unsigned long phaseId,
            unsigned long timestampUs,
            const char* name);
        bool WriteTextLogPhase(
            const char* source,
            unsigned long phaseId,
            unsigned long timestampUs,
            const char* name);
        void FlushTextLog();
        // Reserved for runtime shutdown/fault handling. Normal mode code should not call this.
        void CloseTextLog();

        // Centralized top-level mode failure path: one optional mode-specific cleanup callback plus
        // one runtime-owned shutdown sequence for shared actuators and logs.
        bool RegisterModeFaultHandler(
            ModeFaultCleanupCallback callback,
            void* context,
            const char* source);
        bool FailActiveMode(const char* reason) noexcept;
        void FinalizeSuccessfulModeExit() noexcept;

        bool WriteUtilityDataLogMetadata(const char* key, const char* value);
        bool WriteUtilityDataLogMetadataUnsigned(const char* key, unsigned long value);
        bool WriteUtilityDataLogMetadataFloat(const char* key, float value, std::uint8_t precision);
        bool WriteUtilityDataLogAccelBiasMetadata(const RuntimeSensorSuite& sensors);

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

        template <typename Row>
        bool LogUtilityDataRow(const Row& row)
        {
            return UtilityDataLogger().log(row);
        }

        bool ServiceUtilityDataLog();
        bool FlushUtilityDataLog();
        bool CloseUtilityDataLog();
        void CaptureUtilityDataLogFailure(bool& overflowed, bool& writeFailed) const noexcept;

        // SharedRobotRuntime owns the production actuation hookup for the shared control loop so
        // LoopController can remain a cadence owner that only forwards raw motor PWM values.
        // `Drive()` currently returns the concrete `DriveBase` owner. It is not evidence that the
        // planned higher-level `Drive` translation layer already exists as a separate runtime owner.
        bool SetMotorPWM(float leftMotorPwm, float rightMotorPwm) noexcept;
        DriveBase& Drive() noexcept;
        MazeMap::App::Internal::Drive& DriveService() noexcept;
        StartupCalibration& StartupCalibrationService() noexcept;
        WallTouch& WallTouchService() noexcept;
        ManeuverExecutor& ManeuverExecutorService() noexcept;
        LoopController& ControlLoop() noexcept;
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

        MazeMap::Vehicle speedVehicle;
        MazeMap::Vehicle searchVehicle;
        MazeMap::Maze maze;
        MazeMap::FloodFillPathFinder searchPathFinder;
        MazeMap::ManeuverPathFinder speedPathFinder;
        MazeMap::WallBeliefMap wallBeliefMap;
        MazeMap::PlantModel plantModel;
        DriveBase drive;
        MazeMap::App::Internal::Drive driveService;
        StartupCalibration startupCalibrationService;
        WallTouch wallTouchService;
        ManeuverExecutor maneuverExecutor;
        LoopController controlLoop;
        RuntimeSensorSuite sensors;
        mmlog::MmLogLogger dataLogger;
        SharedRuntimeTextLogFileHandle textLogFile{};
        mmlog::detail::ByteRing textLogQueue;
#if !MAZEMAP_USE_RAM2_FILE_BUFFERS
        alignas(32) std::uint8_t textLogStorage[kSharedRuntimeTextLogQueueBytes]{};
#endif
        bool textLogFaulted{};
        bool textLogInitialized{};
        bool runtimeLogsClosedForFault{};
        bool modeFaultHandlerRegistered{};
        bool modeFaulted{};
        ModeFaultCleanupCallback modeFaultCleanupCallback{};
        void* modeFaultCleanupContext{};
        char activeDataLogFileName[MMLOG_MAX_PATH_LENGTH + 1U]{};
        char activeDataLogSource[kTextLogSourceLength]{};
        char activeModeFaultSource[kTextLogSourceLength]{};
        char lastRuntimeLogError[MMLOG_ERROR_TEXT_LENGTH + 1U]{};
    };

    EXPORT void LogWallSensorAdcRegisterWrite(
        const char* phase,
        std::uint32_t expectedCfg,
        std::uint32_t readCfg,
        std::uint32_t readGc) noexcept;
    EXPORT SharedRobotRuntime& GetSharedRobotRuntime();
}

