#include "pch.h"
#include "MissionRunMode.h"

#include "BootUtilityModeFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapSharedRuntime.h"
#include "StartupCalibration.h"

namespace
{
    constexpr std::uint16_t kMissionPostStartupHoldMs = 250U;
}

namespace MazeMap::App::Internal
{
    class MissionRunMode final : public IApplicationMode
    {
    public:
        explicit MissionRunMode(SharedRobotRuntime& runtime)
            : _runtime(runtime)
            , _loopController(runtime.ControlLoop())
            , _drive(runtime.Drive())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
        {
        }

        bool Begin() override
        {
            ResetState();
            if (!SetupHardware())
            {
                return _runtime.FailActiveMode("Mission hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:mission");
            (void)_runtime.AppendTextLogLine("Mission mode");
            (void)_runtime.AppendTextLogLine("Shared-service mission startup audit");

            if (!_drive.Begin())
            {
                return _runtime.FailActiveMode("Mission drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                return _runtime.FailActiveMode("Mission startup calibration bring-up failed");
            }

            return true;
        }

        void Run() override
        {
            _phase = Phase::LaunchStartupCalibration;

            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &MissionRunMode::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)_runtime.FailActiveMode("Mission loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                const bool completed =
                    (result.status == LoopController::SessionResult::Status::Completed);
                _loopController.EndSession();

                if (completed)
                {
                    (void)_runtime.AppendTextLogLine("Mission startup audit complete");
                }
            }

            _startupCalibration.Cancel();
            _driveService.Cancel();
            _drive.Brake();
            _drive.UseNominalWheelControlProfile();
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            LaunchStartupCalibration,
            RunStartupCalibration,
            LaunchPostStartupHold,
            RunPostStartupHold,
            Complete
        };

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            auto* const self = static_cast<MissionRunMode*>(context);
            if (self == nullptr)
            {
                services.Fault("Mission mode callback context was not installed");
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
            _phase = Phase::Idle;
            _startupCalibration.Cancel();
            _driveService.Cancel();
        }

        bool StartMissionHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.Cancel();
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(durationMs, true);
            return _driveService.Active();
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
                    services.Fault("Mission startup calibration could not start");
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
                if (!StartMissionHold(kMissionPostStartupHoldMs))
                {
                    services.Fault("Mission post-startup hold could not start");
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

                _phase = Phase::Complete;
                return LoopController::ControlVector::Brake;
            }

            case Phase::Complete:
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;

            case Phase::Idle:
            default:
                services.Fault("Mission mode phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        Phase _phase{ Phase::Idle };
    };

    const BootModeDescriptor& GetMissionRunBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::Mission,
            BootModeCategory::Mission,
            "mission",
            "Run the clean single-session mission startup path.",
            "logging.txt",
            &GetMissionRunMode,
            "GetMissionRunMode",
            "MissionRunMode.cpp",
            "hardware bring-up; shared startup calibration; post-startup hold",
            "SharedRobotRuntime drive and startup-calibration services",
            "Mission run flow is intentionally reduced to the clean shared-service startup path",
            "none",
        };
        return descriptor;
    }

    IApplicationMode& GetMissionRunMode()
    {
        static MissionRunMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
