#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapSharedRuntime.h"
#include "PinPairStrap.h"
#include "StartupCalibration.h"

namespace
{
    constexpr const char* kPrimaryDiagnosticStableId = "primary_diagnostic";
    constexpr const char* kPrimaryDiagnosticSelectorRemovedReason =
        "Primary diagnostic selector jumper removed";
    constexpr std::uint16_t kPrimaryDiagnosticCompletionHoldMs = 250U;
    constexpr float kPrimaryDiagnosticStraightDistanceM = 0.54f;
    constexpr float kPrimaryDiagnosticCruiseSpeedMps = 0.45f;

    MotionLimits BuildPrimaryDiagnosticLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = kPrimaryDiagnosticCruiseSpeedMps;
        limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
        limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }
}

namespace MazeMap::App::Internal
{
    class OpenFloorMeasurementController final : public IApplicationMode
    {
    public:
        explicit OpenFloorMeasurementController(SharedRobotRuntime& runtime)
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
            if (!_runtime.RegisterModeFaultHandler(&OpenFloorMeasurementController::TeardownOnRuntimeFault, this, kPrimaryDiagnosticStableId))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return _runtime.FailActiveMode("Primary diagnostic hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:primary_diagnostic");
            (void)_runtime.AppendTextLogLine("Primary diagnostic mode");
            (void)_runtime.AppendTextLogLine("Shared-service open-floor straight/turn diagnostic sweep");

            if (!_drive.Begin())
            {
                return _runtime.FailActiveMode("Primary diagnostic drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(false);
            if (!_startupCalibration.BringUp())
            {
                return _runtime.FailActiveMode("Primary diagnostic startup bring-up failed");
            }

            ConfigureSelectorMonitor();
            if (SelectorRemoved())
            {
                return _runtime.FailActiveMode(kPrimaryDiagnosticSelectorRemovedReason);
            }

            return true;
        }

        void Run() override
        {
            _phase = Phase::LaunchStartupHold;

            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &OpenFloorMeasurementController::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)_runtime.FailActiveMode("Primary diagnostic loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                const bool completed =
                    (result.status == LoopController::SessionResult::Status::Completed);
                _loopController.EndSession();

                if (completed)
                {
                    (void)_runtime.AppendTextLogLine("Primary diagnostic complete");
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
            LaunchStartupHold,
            RunStartupHold,
            LaunchNorthStraight,
            RunNorthStraight,
            LaunchClockwiseTurn,
            RunClockwiseTurn,
            LaunchEastStraight,
            RunEastStraight,
            LaunchFaceNorth,
            RunFaceNorth,
            LaunchCompletionHold,
            RunCompletionHold,
            Complete
        };

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
        {
            (void)reason;
            auto* const self = static_cast<OpenFloorMeasurementController*>(context);
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
            auto* const self = static_cast<OpenFloorMeasurementController*>(context);
            if (self == nullptr)
            {
                services.Fault("Primary diagnostic callback context was not installed");
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
                FindBootModeRegistryEntry(BootModeId::PrimaryDiagnostic);
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
            _driveService.SetLimits(BuildPrimaryDiagnosticLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartHold(durationMs, true);
            return _driveService.Active();
        }

        bool StartStraight(const Eigen::Vector2f& heading) noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildPrimaryDiagnosticLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartStraight(
                kPrimaryDiagnosticStraightDistanceM,
                kPrimaryDiagnosticCruiseSpeedMps,
                0.0f,
                &heading);
            return _driveService.Active();
        }

        bool StartTurn(const float angleRad) noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildPrimaryDiagnosticLimits(_vehicle));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartTurn(angleRad);
            return _driveService.Active();
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
            (void)state;

            if (SelectorRemoved())
            {
                services.Fault(kPrimaryDiagnosticSelectorRemovedReason);
                return LoopController::ControlVector::Brake;
            }

            switch (_phase)
            {
            case Phase::LaunchStartupHold:
                if (!StartHold(DiagnosticConfig::kStartupSettleMs))
                {
                    services.Fault("Primary diagnostic startup hold could not start");
                }
                else
                {
                    _phase = Phase::RunStartupHold;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunStartupHold:
                return PollDrive(Phase::LaunchNorthStraight);

            case Phase::LaunchNorthStraight:
                if (!StartStraight(Eigen::Vector2f(0.0f, 1.0f)))
                {
                    services.Fault("Primary diagnostic northbound straight could not start");
                }
                else
                {
                    _phase = Phase::RunNorthStraight;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunNorthStraight:
                return PollDrive(Phase::LaunchClockwiseTurn);

            case Phase::LaunchClockwiseTurn:
                if (!StartTurn(0.5f * PI_F))
                {
                    services.Fault("Primary diagnostic clockwise turn could not start");
                }
                else
                {
                    _phase = Phase::RunClockwiseTurn;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunClockwiseTurn:
                return PollDrive(Phase::LaunchEastStraight);

            case Phase::LaunchEastStraight:
                if (!StartStraight(Eigen::Vector2f(1.0f, 0.0f)))
                {
                    services.Fault("Primary diagnostic eastbound straight could not start");
                }
                else
                {
                    _phase = Phase::RunEastStraight;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunEastStraight:
                return PollDrive(Phase::LaunchFaceNorth);

            case Phase::LaunchFaceNorth:
                if (!StartTurn(-0.5f * PI_F))
                {
                    services.Fault("Primary diagnostic face-north turn could not start");
                }
                else
                {
                    _phase = Phase::RunFaceNorth;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunFaceNorth:
                return PollDrive(Phase::LaunchCompletionHold);

            case Phase::LaunchCompletionHold:
                if (!StartHold(kPrimaryDiagnosticCompletionHoldMs))
                {
                    services.Fault("Primary diagnostic completion hold could not start");
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
                services.Fault("Primary diagnostic phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        Phase _phase{ Phase::Idle };
        std::uint8_t _selectorDrivePin{};
        std::uint8_t _selectorSensePin{};
        bool _selectorMonitorArmed{};
    };

    IApplicationMode& GetDiagnosticMode();

    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::PrimaryDiagnostic,
            BootModeCategory::Utility,
            "primary_diagnostic",
            "Run the clean shared-service open-floor diagnostic sweep for primary diagnostic mode.",
            "logging.txt",
            &GetDiagnosticMode,
            "GetDiagnosticMode",
            "OpenFloorMeasurementController.cpp",
            "shared bring-up; startup hold; north straight; clockwise turn; east straight; face north; completion hold",
            "DiagnosticConfig linear limits; shared startup-calibration bring-up; shared drive service",
            "Behavior is intentionally reduced to the clean shared-service open-floor sweep",
            "none",
        };
        return descriptor;
    }

    IApplicationMode& GetDiagnosticMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}
