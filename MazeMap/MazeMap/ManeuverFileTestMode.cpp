#include "pch.h"
#include "ManeuverFileTestMode.h"

#include "BootFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "IApplicationMode.h"
#include "LoopController.h"
#include "ManeuverPath.h"
#include "ManeuverQueue.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"

#include <cstdio>
#include <cstring>
#include <limits>

namespace
{
    constexpr const char* kManeuverFileName = "test.txt";
    constexpr std::uint16_t kManeuverFilePostStartupHoldMs = 50U;
    constexpr std::uint16_t kManeuverFileCompletionHoldMs = 50U;

    MazeMap::DirectionalLocation ManeuverFileTestStartLocation() noexcept
    {
        return MazeMap::DirectionalLocation(
            MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)),
            MazeMap::Up);
    }

    bool LoadManeuverQueueFromSd(
        MazeMap::App::Internal::SharedRobotRuntime& runtime,
        MazeMap::Vehicle& vehicle,
        MazeMap::ManeuverQueue& queue)
    {
#if defined(ARDUINO_TEENSY41)
        File file = SD.open(kManeuverFileName, FILE_READ);
        if (!file)
        {
            (void)runtime.AppendTextLogLine("Maneuver file unavailable; skipping maneuver-file test");
            return false;
        }

        MazeMap::ManeuverPath path;
        char line[128] = {};
        std::uint16_t lineNumber = 0U;
        while (file.available())
        {
            const size_t lineLength = file.readBytesUntil('\n', line, sizeof(line) - 1U);
            line[lineLength] = '\0';
            ++lineNumber;

            if (char* hashComment = std::strchr(line, '#'); hashComment != nullptr)
            {
                *hashComment = '\0';
            }

            if (char* slashComment = std::strstr(line, "//"); slashComment != nullptr)
            {
                *slashComment = '\0';
            }

            for (char* token = std::strtok(line, ", \t\r;"); token != nullptr; token = std::strtok(nullptr, ", \t\r;"))
            {
                MazeMap::ManeuverCode code = MazeMap::MC_NONE;
                if (!TryParseManeuverCodeToken(token, code))
                {
                    char message[96] = {};
                    std::snprintf(
                        message,
                        sizeof(message),
                        "Maneuver file token issue on line %u: %s",
                        lineNumber,
                        token);
                    file.close();
                    (void)runtime.AppendTextLogLine(message);
                    return false;
                }

                if (!path.push_back(code))
                {
                    file.close();
                    (void)runtime.AppendTextLogLine(
                        "Maneuver file exceeded path capacity; skipping maneuver-file test");
                    return false;
                }
            }
        }

        file.close();

        if (path.GetSize() == 0U)
        {
            (void)runtime.AppendTextLogLine("Maneuver file did not contain any maneuvers");
            return false;
        }

        queue.clear();
        MazeMap::DirectionalLocation currentLocation = ManeuverFileTestStartLocation();
        if (!queue.push_back(path, currentLocation))
        {
            (void)runtime.AppendTextLogLine("Maneuver file could not be converted into a queue");
            return false;
        }

        queue.ComputeSpeeds(vehicle, 0.0f, 0.0f);
        return true;
#else
        (void)runtime;
        (void)vehicle;
        (void)queue;
        return false;
#endif
    }
}

namespace MazeMap::App::Internal
{
    class ManeuverFileTestMode final : public IApplicationMode
    {
    public:
        explicit ManeuverFileTestMode(SharedRobotRuntime& runtime)
            : _runtime(runtime)
            , _loopController(runtime.ControlLoop())
            , _vehicle(runtime.Vehicle())
            , _drive(runtime.DriveBase())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
        {
        }

        void SetupMode(BootFramework& framework) override
        {
            (void)framework;
            ResetState();
            (void)_runtime.AppendTextLogLine("Maneuver file test mode");
            (void)_runtime.AppendTextLogLine(
                "Load and execute the maneuver queue stored in test.txt through shared startup calibration and Drive.");

            _drive.ClearCommandEvidence();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);

            if (!LoadManeuverQueueFromSd(_runtime, _vehicle, _queue))
            {
                _runtime.FailActiveMode("Maneuver file test could not load test.txt");
            }

            (void)_runtime.AppendTextLogFormatted(
                "Loaded maneuver test queue with %u maneuvers",
                static_cast<unsigned>(_queue.size()));
            _phase = Phase::LaunchStartupCalibration;
            const auto& runtimeState = _runtime.RuntimeState();
            _loopController.StageNextSessionState(
                Config::kControlPeriodUs,
                runtimeState.GetPositionX(),
                runtimeState.GetPositionY());
        }

        CommandVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) override
        {
            (void)loopEndTimeUs;
            (void)state;

            switch (_phase)
            {
            case Phase::LaunchStartupCalibration:
                _startupCalibration.Start();
                if (!_startupCalibration.Active())
                {
                    _runtime.FailActiveMode("Maneuver file test startup calibration could not start");
                }
                else
                {
                    _phase = Phase::RunStartupCalibration;
                }
                return CommandVector::Brake();

            case Phase::RunStartupCalibration:
            {
                bool done = false;
                const CommandVector control = _startupCalibration.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchPostStartupHold;
                return CommandVector::Brake();
            }

            case Phase::LaunchPostStartupHold:
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartHold(kManeuverFilePostStartupHoldMs, true);
                _phase = Phase::RunPostStartupHold;
                return CommandVector::Brake();

            case Phase::RunPostStartupHold:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchQueueEntry;
                return CommandVector::Brake();
            }

            case Phase::LaunchQueueEntry:
                if (_queueActiveIndex >= _queue.size())
                {
                    _phase = Phase::LaunchCompletionHold;
                }
                else
                {
                    const MazeMap::ManeuverInstance& entry = _queue[_queueActiveIndex];

                    char codeName[24] = {};
                    FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
                    (void)_runtime.AppendTextLogFormatted(
                        "Maneuver %u start: %s from (%d,%d) %s",
                        static_cast<unsigned>(_queueActiveIndex),
                        codeName,
                        static_cast<int>(static_cast<MazeMap::CellCoordinates>(entry.getStart().GetLocation()).GetX()),
                        static_cast<int>(static_cast<MazeMap::CellCoordinates>(entry.getStart().GetLocation()).GetY()),
                        DirectionName(entry.getStart().GetDirection()));

                    _driveService.SetOperationMode(Drive::OperationMode::Maze);
                    _driveService.StartManeuver(entry);
                    _phase = Phase::RunQueueEntry;
                }
                return CommandVector::Brake();

            case Phase::RunQueueEntry:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                if (_queueActiveIndex >= _queue.size())
                {
                    _runtime.FailActiveMode("Maneuver file test queue entry completion index was invalid");
                }
                else
                {
                    const MazeMap::ManeuverInstance& entry = _queue[_queueActiveIndex];

                    char codeName[24] = {};
                    FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
                    (void)_runtime.AppendTextLogFormatted(
                        "Maneuver %u complete: %s at (%d,%d) %s",
                        static_cast<unsigned>(_queueActiveIndex),
                        codeName,
                        static_cast<int>(static_cast<MazeMap::CellCoordinates>(entry.getEnd().GetLocation()).GetX()),
                        static_cast<int>(static_cast<MazeMap::CellCoordinates>(entry.getEnd().GetLocation()).GetY()),
                        DirectionName(entry.getEnd().GetDirection()));

                    ++_queueActiveIndex;
                    if (_queueActiveIndex >= _queue.size())
                    {
                        _phase = Phase::LaunchCompletionHold;
                    }
                    else
                    {
                        _phase = Phase::LaunchQueueEntry;
                    }
                }
                return CommandVector::Brake();
            }

            case Phase::LaunchCompletionHold:
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartHold(kManeuverFileCompletionHoldMs, true);
                _phase = Phase::RunCompletionHold;
                return CommandVector::Brake();

            case Phase::RunCompletionHold:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::Complete;
                return CommandVector::Brake();
            }

            case Phase::Complete:
            {
                (void)_runtime.AppendTextLogLine("Maneuver file test complete");
                _startupCalibration.Cancel();
                const CommandVector stopCommand = _drive.ProposeBodyTick(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    (std::numeric_limits<float>::quiet_NaN)());
                _phase = Phase::Idle;
                loopController.HaltExecutionEndProgram();
                return stopCommand;
            }

            case Phase::Idle:
            default:
                _runtime.FailActiveMode("Maneuver file test phase was not initialized");
                return CommandVector::Brake();
            }
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            LaunchStartupCalibration,
            RunStartupCalibration,
            LaunchPostStartupHold,
            RunPostStartupHold,
            LaunchQueueEntry,
            RunQueueEntry,
            LaunchCompletionHold,
            RunCompletionHold,
            Complete
        };

        void ResetState() noexcept
        {
            _queue.clear();
            _queueActiveIndex = 0U;
            _phase = Phase::Idle;
            _startupCalibration.Cancel();
        }

        void OnModeFault(const char* reason) noexcept override
        {
            (void)reason;
            _startupCalibration.Cancel();
            _drive.ClearCommandEvidence();
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        MazeMap::ManeuverQueue _queue{};
        std::uint16_t _queueActiveIndex{};
        Phase _phase{ Phase::Idle };
    };

    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::ManeuverFileTest,
            "maneuver_file_test",
            "Load and execute test.txt after shared maze startup calibration and shared Drive execution.",
            "logging.txt; maneuver queue execution trace",
            &GetManeuverFileTestMode,
            "GetManeuverFileTestMode",
            "ManeuverFileTestMode.cpp",
            "shared startup calibration; post-startup settle; maneuver queue execution; completion settle",
            "Shared startup calibration (including WallTouch) and Drive services; ManeuverQueue speed synthesis",
            "Behavior is intentionally reduced to the canonical shared-service queue path",
            "test.txt; logging.txt",
            true,
        };
        return descriptor;
    }

    IApplicationMode& GetManeuverFileTestMode()
    {
        static ManeuverFileTestMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
