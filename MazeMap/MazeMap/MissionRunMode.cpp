#include "pch.h"
#include "MissionRunMode.h"

#include "BootUtilityModeFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"

#include <limits>

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
            , _drive(runtime.DriveBase())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
        {
        }

        void SetupMode() override
        {
            ResetState();
            if (!SetupHardware())
            {
                _runtime.FailActiveMode("Mission hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:mission");
            (void)_runtime.AppendTextLogLine("Mission mode");
            (void)_runtime.AppendTextLogLine("Shared-service mission startup audit");

            _drive.ClearCommandEvidence();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                _runtime.FailActiveMode("Mission startup calibration bring-up failed");
            }

            _phase = Phase::LaunchStartupCalibration;
            _loopController.StageNextSessionState(BuildLoopOptions());
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

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            const auto& runtimeState = _runtime.RuntimeState();
            options.controlPeriodUs = Config::kControlPeriodUs;
            options.SessionStartPointX = runtimeState.GetPositionX();
            options.SessionStartPointY = runtimeState.GetPositionY();
            return options;
        }

        void ResetState() noexcept
        {
            _phase = Phase::Idle;
            _startupCalibration.Cancel();
        }

        bool StartMissionHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(durationMs, true);
            return true;
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
                    _runtime.FailActiveMode("Mission startup calibration could not start");
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
                if (!StartMissionHold(kMissionPostStartupHoldMs))
                {
                    _runtime.FailActiveMode("Mission post-startup hold could not start");
                }
                else
                {
                    _phase = Phase::RunPostStartupHold;
                }
                return CommandVector::Brake();

            case Phase::RunPostStartupHold:
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
                (void)_runtime.AppendTextLogLine("Mission startup audit complete");
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
                _runtime.FailActiveMode("Mission mode phase was not initialized");
                return CommandVector::Brake();
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
