#pragma once
// Declares the shared runtime object that wires together vehicles, planners, sensors, and drive services for the app.

#include "Defines.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include "MmLog.h"

class DriveBase;
class SensorSuite;
class DiagnosticSensorSuite;
namespace MazeMap::App::Internal
{
    class LoopController;
}

namespace MazeMap
{
    class Vehicle;
    class Maze;
    class FloodFillPathFinder;
    class ManeuverPathFinder;
    class WallBeliefMap;
}

namespace MazeMap::App::Internal
{
    inline constexpr const char* kSharedRuntimeTextLogFileName = "logging.txt";

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
        void CloseTextLog();

        // Centralized top-level mode failure path: one optional mode-specific cleanup callback plus
        // one runtime-owned shutdown sequence for shared actuators and logs.
        bool RegisterModeFaultHandler(
            ModeFaultCleanupCallback callback,
            void* context,
            const char* source);
        bool FailActiveMode(const char* reason) noexcept;

        bool WriteUtilityDataLogMetadata(const char* key, const char* value);
        bool WriteUtilityDataLogMetadataUnsigned(const char* key, unsigned long value);
        bool WriteUtilityDataLogMetadataFloat(const char* key, float value, std::uint8_t precision);
        bool WriteUtilityDataLogAccelBiasMetadata(const DiagnosticSensorSuite& sensors);

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

        DriveBase& Drive() noexcept;
        LoopController& ControlLoop() noexcept;
        SensorSuite& MissionSensors() noexcept;
        DiagnosticSensorSuite& DiagnosticSensors() noexcept;

    private:
        mmlog::MmLogLogger& UtilityDataLogger() noexcept;
        const mmlog::MmLogLogger& UtilityDataLogger() const noexcept;
        const char* DefaultTextLogSource() const noexcept;
        void ClearLastRuntimeLogError() noexcept;
        void SetLastRuntimeLogError(const char* message) noexcept;
        void SetLastRuntimeLogErrorFromUtilityDataLogger(const char* fallback) noexcept;
        void LogUtilityDataLoggerFailure(const char* type) noexcept;
        void CloseRuntimeLogsForFault() noexcept;

        class Implementation;
        std::unique_ptr<Implementation> _impl;
    };

    EXPORT void LogWallSensorAdcRegisterWrite(
        const char* phase,
        std::uint32_t expectedCfg,
        std::uint32_t readCfg,
        std::uint32_t readGc) noexcept;
    EXPORT SharedRobotRuntime& GetSharedRobotRuntime();
}

