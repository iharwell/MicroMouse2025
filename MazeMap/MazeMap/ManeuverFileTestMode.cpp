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
#include "MazeMapSharedRuntime.h"
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

        bool Begin() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&ManeuverFileTestMode::HandleRuntimeFault, this, kManeuverFileTestStableId))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return Fail("Maneuver file test hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:maneuver_file_test");
            (void)_runtime.AppendTextLogLine("Maneuver file test mode");
            (void)_runtime.AppendTextLogLine("Load and execute the maneuver queue stored in test.txt.");

            if (!_drive.Begin())
            {
                return Fail("Maneuver file test drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                return Fail("Maneuver file test startup bring-up failed");
            }

            if (!LoadManeuverQueueFromSd(_runtime, _speedVehicle, _queue))
            {
                return Fail("Maneuver file test could not load test.txt");
            }

            (void)_runtime.AppendTextLogFormatted(
                "Loaded maneuver test queue with %u maneuvers",
                static_cast<unsigned>(_queue.size()));
            return true;
        }

        void Run() override
        {
            if (_faulted)
            {
                return;
            }

            _phase = Phase::LaunchStartupCalibration;

            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &ManeuverFileTestMode::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)Fail("Maneuver file test loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                _completed =
                    !_faulted &&
                    (result.status == LoopController::SessionResult::Status::Completed);
                _loopController.EndSession();
            }

            _startupCalibration.Cancel();
            _driveService.Cancel();
            _drive.Brake();
            _drive.UseNominalWheelControlProfile();

            if (_completed)
            {
                (void)_runtime.AppendTextLogLine("Maneuver file test complete");
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

        static void HandleRuntimeFault(void* context, const char* reason) noexcept
        {
            if (context != nullptr)
            {
                static_cast<ManeuverFileTestMode*>(context)->OnRuntimeFault(reason);
            }
        }

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            auto* const self = static_cast<ManeuverFileTestMode*>(context);
            if (self == nullptr)
            {
                services.Fault("Maneuver file test callback context was not installed");
                return LoopController::ControlVector::Brake;
            }

            return self->RunTick(loopEndTimeUs, state, services);
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            options.controlPeriodUs = Config::kControlPeriodUs;
            return options;
        }

        void ResetState() noexcept
        {
            _faulted = false;
            _completed = false;
            _queue.clear();
            _currentLocation = ManeuverFileTestStartLocation();
            _queueActiveIndex = 0U;
            _phase = Phase::Idle;
            _startupCalibration.Cancel();
            _driveService.Cancel();
        }

        bool Fail(const char* reason)
        {
            return _runtime.FailActiveMode(reason);
        }

        void OnRuntimeFault(const char* reason) noexcept
        {
            _faulted = true;
            (void)_runtime.AppendTextLogLine(
                (reason != nullptr && reason[0] != '\0') ?
                    reason :
                    "maneuver_file_test_fault");
        }

        bool StartHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.Cancel();
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartHold(durationMs, true);
            return _driveService.Active();
        }

        bool StartQueueEntry(const std::uint16_t index)
        {
            if (index >= _queue.size())
            {
                return false;
            }

            _queueActiveIndex = index;
            const MazeMap::ManeuverInstance& entry = _queue[_queueActiveIndex];
            _currentLocation = entry.getStart();

            char codeName[24] = {};
            FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
            (void)_runtime.AppendTextLogFormatted(
                "Maneuver %u start: %s from (%d,%d) %s",
                static_cast<unsigned>(_queueActiveIndex),
                codeName,
                static_cast<int>(static_cast<MazeMap::CellCoordinates>(_currentLocation.GetLocation()).GetX()),
                static_cast<int>(static_cast<MazeMap::CellCoordinates>(_currentLocation.GetLocation()).GetY()),
                DirectionName(_currentLocation.GetDirection()));

            _driveService.Cancel();
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartManeuver(entry);
            return _driveService.Active();
        }

        bool FinishQueueEntry()
        {
            if (_queueActiveIndex >= _queue.size())
            {
                return false;
            }

            const MazeMap::ManeuverInstance& entry = _queue[_queueActiveIndex];
            _currentLocation = entry.getEnd();

            char codeName[24] = {};
            FormatManeuverCodeName(entry.getCode(), codeName, sizeof(codeName));
            (void)_runtime.AppendTextLogFormatted(
                "Maneuver %u complete: %s at (%d,%d) %s",
                static_cast<unsigned>(_queueActiveIndex),
                codeName,
                static_cast<int>(static_cast<MazeMap::CellCoordinates>(_currentLocation.GetLocation()).GetX()),
                static_cast<int>(static_cast<MazeMap::CellCoordinates>(_currentLocation.GetLocation()).GetY()),
                DirectionName(_currentLocation.GetDirection()));

            ++_queueActiveIndex;
            return true;
        }

        LoopController::ControlVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            (void)loopEndTimeUs;
            (void)state;

            switch (_phase)
            {
            case Phase::LaunchStartupCalibration:
                _startupCalibration.Start();
                if (!_startupCalibration.Active())
                {
                    services.Fault("Maneuver file test startup calibration could not start");
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
                if (!StartHold(kManeuverFilePostStartupHoldMs))
                {
                    services.Fault("Maneuver file test post-startup hold could not start");
                }
                else
                {
                    _phase = Phase::RunPostStartupHold;
                }
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
                else if (!StartQueueEntry(_queueActiveIndex))
                {
                    services.Fault("Maneuver file test queue entry could not start");
                }
                else
                {
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

                if (!FinishQueueEntry())
                {
                    services.Fault("Maneuver file test queue entry could not complete");
                }
                else if (_queueActiveIndex >= _queue.size())
                {
                    _phase = Phase::LaunchCompletionHold;
                }
                else
                {
                    _phase = Phase::LaunchQueueEntry;
                }
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchCompletionHold:
                if (!StartHold(kManeuverFileCompletionHoldMs))
                {
                    services.Fault("Maneuver file test completion hold could not start");
                }
                else
                {
                    _phase = Phase::RunCompletionHold;
                }
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
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;

            case Phase::Idle:
            default:
                services.Fault("Maneuver file test phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _speedVehicle;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        MazeMap::ManeuverQueue _queue{};
        MazeMap::DirectionalLocation _currentLocation{ ManeuverFileTestStartLocation() };
        std::uint16_t _queueActiveIndex{};
        bool _faulted{};
        bool _completed{};
        Phase _phase{ Phase::Idle };
    };

    const BootModeDescriptor& GetManeuverFileTestBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::ManeuverFileTest,
            BootModeCategory::Utility,
            "maneuver_file_test",
            "Load and execute test.txt after shared maze startup calibration.",
            "logging.txt",
            &GetManeuverFileTestMode,
            "GetManeuverFileTestMode",
            "ManeuverFileTestMode.cpp",
            "shared startup calibration; post-startup settle; maneuver queue execution; completion settle",
            "Shared startup calibration and drive services; ManeuverQueue speed synthesis",
            "Behavior is intentionally reduced to the clean shared-service queue path",
            "test.txt",
        };
        return descriptor;
    }

    IApplicationMode& GetManeuverFileTestMode()
    {
        static ManeuverFileTestMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
