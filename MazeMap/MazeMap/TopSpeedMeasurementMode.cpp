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
        limits.maxSpeedMps = vehicle.GetMaxSpeed();
        limits.accelMps2 = kTopSpeedMeasurementAccelMps2;
        limits.decelMps2 = kTopSpeedMeasurementDecelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
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
            , _vehicle(runtime.SpeedVehicle())
            , _drive(runtime.Drive())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
        {
        }

        bool Begin() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&TopSpeedMeasurementMode::TeardownOnRuntimeFault, this, kTopSpeedMeasurementStableId))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return _runtime.FailActiveMode("Top speed measurement hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:top_speed_measurement");
            (void)_runtime.AppendTextLogLine("Top speed measurement mode");
            (void)_runtime.AppendTextLogLine("Shared-service open-floor straight-line top-speed audit");

            if (!_drive.Begin())
            {
                return _runtime.FailActiveMode("Top speed measurement drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(false);
            if (!_startupCalibration.BringUp())
            {
                return _runtime.FailActiveMode("Top speed measurement startup bring-up failed");
            }

            ConfigureSelectorMonitor();
            if (SelectorRemoved())
            {
                return _runtime.FailActiveMode(kTopSpeedMeasurementSelectorRemovedReason);
            }

            _batteryVoltageStart = ReadBatteryVoltage();
            return true;
        }

        void Run() override
        {
            _phase = Phase::LaunchPrelaunchHold;

            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &TopSpeedMeasurementMode::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)_runtime.FailActiveMode("Top speed measurement loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                const bool completed =
                    (result.status == LoopController::SessionResult::Status::Completed);
                const std::uint32_t completedTicks = result.tickCount;
                _loopController.EndSession();

                if (completed)
                {
                    (void)_runtime.AppendTextLogFormatted(
                        "Top speed complete: ticks=%lu peak_speed_mps=%.3f peak_planar_accel_mps2=%.3f vbat0=%.3f",
                        static_cast<unsigned long>(completedTicks),
                        _peakMeasuredSpeedMps,
                        _peakPlanarAccelMps2,
                        _batteryVoltageStart);
                }
            }

            ReleaseSelectorMonitor();
            _startupCalibration.Cancel();
            _driveService.Cancel();
            _drive.Brake();
            _drive.UseNominalWheelControlProfile();
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
            self->_driveService.Cancel();
            self->_drive.Brake();
            self->_drive.UseNominalWheelControlProfile();
        }

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            auto* const self = static_cast<TopSpeedMeasurementMode*>(context);
            if (self == nullptr)
            {
                services.Fault("Top speed measurement callback context was not installed");
                return LoopController::ControlVector::Brake;
            }

            return self->RunTick(loopEndTimeUs, state, services);
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
            options.workPlan.useWallUpdates = false;
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
            _driveService.Cancel();
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
            _driveService.Cancel();
            _driveService.SetLimits(BuildTopSpeedLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(durationMs, true);
            return _driveService.Active();
        }

        bool StartStraightRun() noexcept
        {
            const Eigen::Vector2f heading(0.0f, 1.0f);
            const PoseEstimate& pose = _drive.GetPose();
            const Eigen::Vector2f targetPosition(
                pose.xMeters,
                pose.yMeters + kTopSpeedMeasurementDistanceM);
            _driveService.Cancel();
            _driveService.SetLimits(BuildTopSpeedLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartStraight(
                kTopSpeedMeasurementDistanceM,
                _vehicle.GetMaxSpeed(),
                0.0f,
                &heading,
                &targetPosition);
            return _driveService.Active();
        }

        void UpdatePeaks(const LoopController::ModeState& state) noexcept
        {
            if (std::isfinite(state.measured.linearSpeedMps))
            {
                _peakMeasuredSpeedMps =
                    (std::max)(_peakMeasuredSpeedMps, std::fabs(state.measured.linearSpeedMps));
            }
            if (std::isfinite(state.sensors.planarAccelMps2))
            {
                _peakPlanarAccelMps2 =
                    (std::max)(_peakPlanarAccelMps2, std::fabs(state.sensors.planarAccelMps2));
            }
        }

        LoopController::ControlVector PollDrive(const Phase nextPhase)
        {
            bool done = false;
            const LoopController::ControlVector control = _driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = nextPhase;
            return LoopController::ControlVector::Brake;
        }

        LoopController::ControlVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            (void)loopEndTimeUs;

            if (SelectorRemoved())
            {
                services.Fault(kTopSpeedMeasurementSelectorRemovedReason);
                return LoopController::ControlVector::Brake;
            }

            UpdatePeaks(state);

            switch (_phase)
            {
            case Phase::LaunchPrelaunchHold:
                if (!StartHold(kTopSpeedMeasurementPrelaunchHoldMs))
                {
                    services.Fault("Top speed measurement prelaunch hold could not start");
                }
                else
                {
                    _phase = Phase::RunPrelaunchHold;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunPrelaunchHold:
                return PollDrive(Phase::LaunchStraight);

            case Phase::LaunchStraight:
                if (!StartStraightRun())
                {
                    services.Fault("Top speed measurement straight run could not start");
                }
                else
                {
                    _phase = Phase::RunStraight;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunStraight:
                return PollDrive(Phase::LaunchCompletionHold);

            case Phase::LaunchCompletionHold:
                if (!StartHold(kTopSpeedMeasurementCompletionHoldMs))
                {
                    services.Fault("Top speed measurement completion hold could not start");
                }
                else
                {
                    _phase = Phase::RunCompletionHold;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunCompletionHold:
                return PollDrive(Phase::Complete);

            case Phase::Complete:
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;

            case Phase::Idle:
            default:
                services.Fault("Top speed measurement phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        float ReadBatteryVoltage() const noexcept
        {
            return MazeMap::MotorEncoderDrive::GetSharedPhysicalModel().supplyVoltageV;
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
