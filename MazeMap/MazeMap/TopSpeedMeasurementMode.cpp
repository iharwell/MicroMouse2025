#include "pch.h"
#include "TopSpeedMeasurementMode.h"

#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "PinPairStrap.h"
#include "StartupCalibration.h"

#include <limits>

namespace
{
    constexpr const char* kTopSpeedMeasurementStableId = "top_speed_measurement";
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
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxRotationalVelocity());
        limits.SetAngularAccelRadps2(vehicle.GetMaxAngularAcceleration());
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

        void SetupMode() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&TopSpeedMeasurementMode::TeardownOnRuntimeFault, this, kTopSpeedMeasurementStableId))
            {
                _runtime.FailActiveMode("Top speed measurement fault handler registration failed");
            }

            if (!SetupHardware())
            {
                _runtime.FailActiveMode("Top speed measurement hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:top_speed_measurement");
            (void)_runtime.AppendTextLogLine("Top speed measurement mode");
            (void)_runtime.AppendTextLogLine("Shared-service open-floor straight-line top-speed audit");

            _drive.ClearCommandEvidence();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(false);
            if (!_startupCalibration.BringUp())
            {
                _runtime.FailActiveMode("Top speed measurement startup bring-up failed");
            }

            ConfigureSelectorMonitor();
            if (SelectorRemoved())
            {
                _runtime.FailActiveMode(kTopSpeedMeasurementSelectorRemovedReason);
            }

            _phase = Phase::LaunchPrelaunchHold;
            _loopController.StageNextSessionState(BuildLoopOptions());
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

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
        {
            (void)reason;
            auto* const self = static_cast<TopSpeedMeasurementMode*>(context);
            if (self == nullptr)
            {
                return;
            }

            self->ReleaseSelectorMonitor();
            self->_phase = Phase::Idle;
            self->_startupCalibration.Cancel();
            self->_drive.ClearCommandEvidence();
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            const auto& runtimeState = _runtime.RuntimeState();
            options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
            options.workPlan.SetUseWallUpdates(false);
            options.SessionStartPointX = runtimeState.GetPositionX();
            options.SessionStartPointY = runtimeState.GetPositionY();
            return options;
        }

        void ResetState() noexcept
        {
            _phase = Phase::Idle;
            _peakMeasuredSpeedMps = 0.0f;
            _peakPlanarAccelMps2 = 0.0f;
            _batteryVoltageStart = 0.0f;
            _selectorDrivePin = 0U;
            _selectorSensePin = 0U;
            _selectorMonitorArmed = false;
            _startupCalibration.Cancel();
        }

        void ConfigureSelectorMonitor() noexcept
        {
            ReleaseSelectorMonitor();
            const BootModeRegistryEntry* const entry =
                FindBootModeRegistryEntry(BootModeId::TopSpeedMeasurement);
            if ((entry == nullptr) || (entry->selector.kind != BootModeSelectorKind::PinPair))
            {
                return;
            }

            _selectorDrivePin = entry->selector.pinA;
            _selectorSensePin = entry->selector.pinB;
            BeginPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
            _selectorMonitorArmed = true;
        }

        void ReleaseSelectorMonitor() noexcept
        {
            if (_selectorMonitorArmed)
            {
                EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
            }
            _selectorMonitorArmed = false;
            _selectorDrivePin = 0U;
            _selectorSensePin = 0U;
        }

        bool SelectorRemoved() const noexcept
        {
            return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
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
            if (std::isfinite(state.GetVelocity()))
            {
                _peakMeasuredSpeedMps =
                    (std::max)(_peakMeasuredSpeedMps, std::fabs(state.GetVelocity()));
            }
            if (std::isfinite(state.GetSensorSnapshot().planarAccelMps2))
            {
                _peakPlanarAccelMps2 =
                    (std::max)(_peakPlanarAccelMps2, std::fabs(state.GetSensorSnapshot().planarAccelMps2));
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

            if (SelectorRemoved())
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
                    static_cast<unsigned long>(_loopController.LastDiagnostics().sequence),
                    _peakMeasuredSpeedMps,
                    _peakPlanarAccelMps2,
                    _batteryVoltageStart);
                ReleaseSelectorMonitor();
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
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
    };

    const BootModeDescriptor& GetTopSpeedMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::TopSpeedMeasurement,
            BootModeCategory::Utility,
            "top_speed_measurement",
            "Run a clean open-floor straight-line top-speed measurement with shared services.",
            "logging.txt",
            &GetTopSpeedMeasurementMode,
            "GetTopSpeedMeasurementMode",
            "TopSpeedMeasurementMode.cpp",
            "shared bring-up; prelaunch hold; straight run; completion hold",
            "Shared startup calibration bring-up; shared drive service; selector monitor",
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
