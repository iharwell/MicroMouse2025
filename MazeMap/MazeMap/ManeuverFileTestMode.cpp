#include "pch.h"
#include "ManeuverFileTestMode.h"

#include "BootUtilityModeFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "ManeuverPath.h"
#include "ManeuverQueue.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"

#include <cstdio>
#include <cstring>

namespace
{
    constexpr const char* kManeuverFileTestStableId = "maneuver_file_test";
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
        MazeMap::Vehicle& speedVehicle,
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

        queue.ComputeSpeeds(speedVehicle, 0.0f, 0.0f);
        return true;
#else
        (void)runtime;
        (void)speedVehicle;
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
            , _speedVehicle(runtime.SpeedVehicle())
            , _drive(runtime.Drive())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
        {
        }

        void SetupMode() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&ManeuverFileTestMode::TeardownOnRuntimeFault, this, kManeuverFileTestStableId))
            {
                _runtime.FailActiveMode("Maneuver file test fault handler registration failed");
            }

            if (!SetupHardware())
            {
                _runtime.FailActiveMode("Maneuver file test hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:maneuver_file_test");
            (void)_runtime.AppendTextLogLine("Maneuver file test mode");
            (void)_runtime.AppendTextLogLine(
                "Load and execute the maneuver queue stored in test.txt through shared startup calibration and Drive.");

            if (!_drive.Begin())
            {
                _runtime.FailActiveMode("Maneuver file test drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                _runtime.FailActiveMode("Maneuver file test startup bring-up failed");
            }

            if (!LoadManeuverQueueFromSd(_runtime, _speedVehicle, _queue))
            {
                _runtime.FailActiveMode("Maneuver file test could not load test.txt");
            }

            (void)_runtime.AppendTextLogFormatted(
                "Loaded maneuver test queue with %u maneuvers",
                static_cast<unsigned>(_queue.size()));
            _phase = Phase::LaunchStartupCalibration;
            _loopController.StageNextSessionState(BuildLoopOptions());
        }

        LoopController::ControlVector RunTick(
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
                return LoopController::ControlVector::Brake;

            case Phase::RunStartupCalibration:
            {
                bool done = false;
                const LoopController::ControlVector control = _startupCalibration.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchPostStartupHold;
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchPostStartupHold:
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartHold(kManeuverFilePostStartupHoldMs, true);
                _phase = Phase::RunPostStartupHold;
                return LoopController::ControlVector::Brake;

            case Phase::RunPostStartupHold:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchQueueEntry;
                return LoopController::ControlVector::Brake;
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
                return LoopController::ControlVector::Brake;

            case Phase::RunQueueEntry:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
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
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchCompletionHold:
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartHold(kManeuverFileCompletionHoldMs, true);
                _phase = Phase::RunCompletionHold;
                return LoopController::ControlVector::Brake;

            case Phase::RunCompletionHold:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::Complete;
                return LoopController::ControlVector::Brake;
            }

            case Phase::Complete:
                (void)_runtime.AppendTextLogLine("Maneuver file test complete");
                _startupCalibration.Cancel();
                _drive.Brake();
                _drive.UseNominalWheelControlProfile();
                _phase = Phase::Idle;
                loopController.HaltExecutionEndProgram();
                return LoopController::ControlVector::Brake;

            case Phase::Idle:
            default:
                _runtime.FailActiveMode("Maneuver file test phase was not initialized");
                return LoopController::ControlVector::Brake;
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

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            options.controlPeriodUs = Config::kControlPeriodUs;
            return options;
        }

        void ResetState() noexcept
        {
            _queue.clear();
            _queueActiveIndex = 0U;
            _phase = Phase::Idle;
            _startupCalibration.Cancel();
        }

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
        {
            (void)reason;
            if (context == nullptr)
            {
                return;
            }

            auto* const self = static_cast<ManeuverFileTestMode*>(context);
            self->_startupCalibration.Cancel();
            self->_drive.Brake();
            self->_drive.UseNominalWheelControlProfile();
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _speedVehicle;
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
            BootModeCategory::Utility,
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
        };
        return descriptor;
    }

    IApplicationMode& GetManeuverFileTestMode()
    {
        static ManeuverFileTestMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
