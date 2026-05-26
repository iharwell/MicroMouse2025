#include "pch.h"
#include "TopSpeedMeasurementMode.h"

#include "BootFramework.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "DriveBase.h"
#include "IApplicationMode.h"
#include "LoopController.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"

#include <limits>

namespace
{
    constexpr const char* kTopSpeedMeasurementSelectorRemovedReason =
        "Top-speed measurement selector jumper removed";
    constexpr std::uint16_t kTopSpeedMeasurementPrelaunchHoldMs = 1000U;
    constexpr std::uint16_t kTopSpeedMeasurementCompletionHoldMs = 250U;
    constexpr float kTopSpeedMeasurementDistanceM = 2.50f;
    constexpr float kTopSpeedMeasurementAccelMps2 = 3.0f;
    constexpr float kTopSpeedMeasurementDecelMps2 = 4.0f;

    MotionLimits BuildTopSpeedLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps(vehicle.GetMaxSpeed());
        limits.SetAccelMps2(kTopSpeedMeasurementAccelMps2);
        limits.SetDecelMps2(kTopSpeedMeasurementDecelMps2);
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxYawRate());
        limits.SetAngularAccelRadps2(vehicle.GetMaxYawAccel());
        return limits;
    }
}

namespace MazeMap::App::Internal
{
    class TopSpeedMeasurementMode final : public IApplicationMode
    {
    public:
        explicit TopSpeedMeasurementMode(SharedRobotRuntime& runtime)
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
            ResetState();
            _bootFramework = &framework;
            if (!framework.IsSelectedModeSelectorInstalled())
            {
                _runtime.FailActiveMode("Top speed measurement selector pins unavailable");
            }
            (void)_runtime.AppendTextLogLine("Top speed measurement mode");
            (void)_runtime.AppendTextLogLine("Shared-service open-floor straight-line top-speed audit");

            _drive.ClearCommandEvidence();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(false);
            if (!_startupCalibration.BringUp())
            {
                _runtime.FailActiveMode("Top speed measurement startup bring-up failed");
            }

            if (!framework.IsSelectedModeSelectorInstalled())
            {
                _runtime.FailActiveMode(kTopSpeedMeasurementSelectorRemovedReason);
            }

            _phase = Phase::LaunchPrelaunchHold;
            const auto& runtimeState = _runtime.RuntimeState();
            _loopController.StageNextSessionState(
                DiagnosticConfig::kControlPeriodUs,
                runtimeState.GetPositionX(),
                runtimeState.GetPositionY(),
                LoopController::WallMask::All,
                true,
                true,
                true,
                false);
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            LaunchPrelaunchHold,
            RunPrelaunchHold,
            LaunchStraight,
            RunStraight,
            LaunchCompletionHold,
            RunCompletionHold,
            Complete
        };

        void OnModeFault(const char* reason) noexcept override
        {
            (void)reason;
            _phase = Phase::Idle;
            _startupCalibration.Cancel();
            _drive.ClearCommandEvidence();
        }

        void ResetState() noexcept
        {
            _phase = Phase::Idle;
            _peakMeasuredSpeedMps = 0.0f;
            _peakPlanarAccelMps2 = 0.0f;
            _batteryVoltageStart = 0.0f;
            _bootFramework = nullptr;
            _startupCalibration.Cancel();
        }

        bool StartHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.SetLimits(BuildTopSpeedLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(durationMs, true);
            return true;
        }

        bool StartStraightRun() noexcept
        {
            const Eigen::Vector2f heading(0.0f, 1.0f);
            const MazeMap::VehicleState& pose = _runtime.RuntimeState();
            const Eigen::Vector2f targetPosition(
                pose.GetPositionX(),
                pose.GetPositionY() + kTopSpeedMeasurementDistanceM);
            _driveService.SetLimits(BuildTopSpeedLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartStraight(
                kTopSpeedMeasurementDistanceM,
                _vehicle.GetMaxSpeed(),
                0.0f,
                &heading,
                &targetPosition);
            return true;
        }

        void UpdatePeaks(const MazeMap::VehicleState& state) noexcept
        {
            if (std::isfinite(state.GetForwardVelocity()))
            {
                _peakMeasuredSpeedMps =
                    (std::max)(_peakMeasuredSpeedMps, std::fabs(state.GetForwardVelocity()));
            }
            if (std::isfinite(state.GetSensorSnapshot().PlanarAccelerationMps2()))
            {
                _peakPlanarAccelMps2 =
                    (std::max)(_peakPlanarAccelMps2, std::fabs(state.GetSensorSnapshot().PlanarAccelerationMps2()));
            }
        }

        CommandVector PollDrive(const Phase nextPhase)
        {
            bool done = false;
            const CommandVector control = _driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = nextPhase;
            return CommandVector::Brake();
        }

        CommandVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
            LoopController& loopController) override
        {
            (void)loopEndTimeUs;

            if (_bootFramework != nullptr && !_bootFramework->IsSelectedModeSelectorInstalled())
            {
                _runtime.FailActiveMode(kTopSpeedMeasurementSelectorRemovedReason);
                return CommandVector::Brake();
            }

            UpdatePeaks(state);

            switch (_phase)
            {
            case Phase::LaunchPrelaunchHold:
                if (!StartHold(kTopSpeedMeasurementPrelaunchHoldMs))
                {
                    _runtime.FailActiveMode("Top speed measurement prelaunch hold could not start");
                }
                else
                {
                    _phase = Phase::RunPrelaunchHold;
                }
                return CommandVector::Brake();

            case Phase::RunPrelaunchHold:
                return PollDrive(Phase::LaunchStraight);

            case Phase::LaunchStraight:
                if (!StartStraightRun())
                {
                    _runtime.FailActiveMode("Top speed measurement straight run could not start");
                }
                else
                {
                    _phase = Phase::RunStraight;
                }
                return CommandVector::Brake();

            case Phase::RunStraight:
                return PollDrive(Phase::LaunchCompletionHold);

            case Phase::LaunchCompletionHold:
                if (!StartHold(kTopSpeedMeasurementCompletionHoldMs))
                {
                    _runtime.FailActiveMode("Top speed measurement completion hold could not start");
                }
                else
                {
                    _phase = Phase::RunCompletionHold;
                }
                return CommandVector::Brake();

            case Phase::RunCompletionHold:
                return PollDrive(Phase::Complete);

            case Phase::Complete:
            {
                (void)_runtime.AppendTextLogFormatted(
                    "Top speed complete: ticks=%lu peak_speed_mps=%.3f peak_planar_accel_mps2=%.3f vbat0=%.3f",
                    static_cast<unsigned long>(_loopController.LastTimingSequence()),
                    _peakMeasuredSpeedMps,
                    _peakPlanarAccelMps2,
                    _batteryVoltageStart);
                _bootFramework = nullptr;
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
                _runtime.FailActiveMode("Top speed measurement phase was not initialized");
                return CommandVector::Brake();
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        Phase _phase{ Phase::Idle };
        float _peakMeasuredSpeedMps{};
        float _peakPlanarAccelMps2{};
        float _batteryVoltageStart{};
        BootFramework* _bootFramework{};
    };

    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::TopSpeedMeasurement,
            "top_speed_measurement",
            "Run a clean open-floor straight-line top-speed measurement with shared services.",
            "logging.txt",
            &GetTopSpeedMeasurementMode,
            "GetTopSpeedMeasurementMode",
            "TopSpeedMeasurementMode.cpp",
            "shared bring-up; prelaunch hold; straight run; completion hold",
            "Shared startup calibration bring-up; shared drive service; selector presence check",
            "Behavior is intentionally reduced to the clean shared-service top-speed path",
            "none",
        };
        return descriptor;
    }

    IApplicationMode& GetTopSpeedMeasurementMode()
    {
        static TopSpeedMeasurementMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
