#include "pch.h"
#include "CorridorRepeatabilityMode.h"

#include "AuxMeasurementConfig.h"
#include "BootUtilityModeFramework.h"
#include "CommandPD.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "MazeMapSharedRuntime.h"
#include "StartupCalibration.h"
#include "WallTouch.h"

namespace
{
    constexpr const char* kCorridorStableId = "corridor_repeatability";

    MotionLimits BuildCorridorDriveLimits(const MazeMap::Vehicle& vehicle, const float cruiseSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = cruiseSpeedMps;
        limits.accelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }

    MotionLimits BuildWallTouchLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = 0.35f;
        limits.accelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2;
        limits.decelMps2 = AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }
}

namespace MazeMap::App::Internal
{
    class CorridorRepeatabilityMode final : public IApplicationMode
    {
    public:
        explicit CorridorRepeatabilityMode(SharedRobotRuntime& runtime)
            : _runtime(runtime)
            , _loopController(runtime.ControlLoop())
            , _vehicle(runtime.SpeedVehicle())
            , _drive(runtime.Drive())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
            , _wallTouch(runtime.WallTouchService())
        {
        }

        bool Begin() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&CorridorRepeatabilityMode::HandleRuntimeFault, this, kCorridorStableId))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return Fail("Corridor repeatability hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:corridor_repeatability");
            (void)_runtime.AppendTextLogLine("Corridor repeatability mode");
            (void)_runtime.AppendTextLogLine("Single-session shared-service corridor passes");

            if (!_drive.Begin())
            {
                return Fail("Corridor repeatability drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                return Fail("Corridor repeatability startup bring-up failed");
            }

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
            callbacks.onModeWork = &CorridorRepeatabilityMode::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)Fail("Corridor repeatability loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                _completed =
                    !_faulted &&
                    (result.status == LoopController::SessionResult::Status::Completed);
                _loopController.EndSession();
            }

            _wallTouch.Cancel();
            _startupCalibration.Cancel();
            _driveService.Cancel();
            _drive.Brake();
            _drive.UseNominalWheelControlProfile();

            if (_completed)
            {
                (void)_runtime.AppendTextLogLine("Corridor repeatability complete");
            }
        }

    private:
        enum class Phase : std::uint8_t
        {
            Idle,
            LaunchStartupCalibration,
            RunStartupCalibration,
            LaunchStartHold,
            RunStartHold,
            LaunchOutbound,
            RunOutbound,
            LaunchFarTouch,
            RunFarTouch,
            LaunchTurnHome,
            RunTurnHome,
            LaunchReturn,
            RunReturn,
            LaunchFaceNorth,
            RunFaceNorth,
            AdvanceSpeed,
            Complete
        };

        static void HandleRuntimeFault(void* context, const char* reason) noexcept
        {
            if (context != nullptr)
            {
                static_cast<CorridorRepeatabilityMode*>(context)->OnRuntimeFault(reason);
            }
        }

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const LoopController::ModeState& state,
            LoopController::TickServices& services)
        {
            auto* const self = static_cast<CorridorRepeatabilityMode*>(context);
            if (self == nullptr)
            {
                services.Fault("Corridor repeatability callback context was not installed");
                return LoopController::ControlVector::Brake;
            }

            return self->RunTick(loopEndTimeUs, state, services);
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            options.controlPeriodUs = AuxMeasurementConfig::kControlPeriodUs;
            return options;
        }

        void ResetState() noexcept
        {
            _faulted = false;
            _completed = false;
            _speedIndex = 0U;
            _phase = Phase::Idle;
            _wallTouch.Cancel();
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
                    "corridor_repeatability_fault");
        }

        float OutboundDistanceM() const noexcept
        {
            return
                Config::kCellSizeM *
                static_cast<float>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U);
        }

        Eigen::Vector2f StartCellCenter() const noexcept
        {
            return Eigen::Vector2f(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
        }

        Eigen::Vector2f FarCellCenter() const noexcept
        {
            return Eigen::Vector2f(
                0.5f * Config::kCellSizeM,
                (0.5f * Config::kCellSizeM) + OutboundDistanceM());
        }

        bool StartDriveHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildCorridorDriveLimits(_vehicle, AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartHold(durationMs, true);
            return _driveService.Active();
        }

        bool StartDriveStraight(
            const float distanceM,
            const float cruiseSpeedMps,
            const Eigen::Vector2f& heading,
            const Eigen::Vector2f& targetPosition) noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildCorridorDriveLimits(_vehicle, cruiseSpeedMps));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartStraight(distanceM, cruiseSpeedMps, 0.0f, &heading, &targetPosition);
            return _driveService.Active();
        }

        bool StartTurnAround() noexcept
        {
            _driveService.Cancel();
            _driveService.SetLimits(BuildCorridorDriveLimits(_vehicle, AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartTurn(PI_F);
            return _driveService.Active();
        }

        bool StartWallTouch() noexcept
        {
            _wallTouch.Cancel();
            _wallTouch.SetLimits(BuildWallTouchLimits(_vehicle));
            _wallTouch.SetTrackingCommandPD(
                MazeMap::CommandPD::StateWheelOmegaPD |
                MazeMap::CommandPD::IMUYaw);
            _wallTouch.SetAllowPassThroughNoWall(false);
            _wallTouch.Start(
                MazeMap::CellCoordinates(
                    0U,
                    static_cast<std::uint8_t>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U)),
                MazeMap::Up);
            return _wallTouch.Active();
        }

        LoopController::ControlVector PollStartupCalibration()
        {
            bool done = false;
            const LoopController::ControlVector control = _startupCalibration.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = Phase::LaunchStartHold;
            return LoopController::ControlVector::Brake;
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

        LoopController::ControlVector PollWallTouch(const Phase nextPhase)
        {
            bool done = false;
            const LoopController::ControlVector control = _wallTouch.GetNextControls(done);
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

            switch (_phase)
            {
            case Phase::LaunchStartupCalibration:
                _startupCalibration.Start();
                if (!_startupCalibration.Active())
                {
                    services.Fault("Corridor repeatability startup calibration could not start");
                }
                else
                {
                    _phase = Phase::RunStartupCalibration;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunStartupCalibration:
                return PollStartupCalibration();

            case Phase::LaunchStartHold:
                if (!StartDriveHold(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs))
                {
                    services.Fault("Corridor repeatability start hold could not start");
                }
                else
                {
                    _phase = Phase::RunStartHold;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunStartHold:
                return PollDrive(Phase::LaunchOutbound);

            case Phase::LaunchOutbound:
                if (!StartDriveStraight(
                        OutboundDistanceM(),
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex],
                        DirectionToUnitVector(MazeMap::Up),
                        FarCellCenter()))
                {
                    services.Fault("Corridor repeatability outbound drive could not start");
                }
                else
                {
                    _phase = Phase::RunOutbound;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunOutbound:
                return PollDrive(Phase::LaunchFarTouch);

            case Phase::LaunchFarTouch:
                if (!StartWallTouch())
                {
                    services.Fault("Corridor repeatability wall touch could not start");
                }
                else
                {
                    _phase = Phase::RunFarTouch;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunFarTouch:
                return PollWallTouch(Phase::LaunchTurnHome);

            case Phase::LaunchTurnHome:
                if (!StartTurnAround())
                {
                    services.Fault("Corridor repeatability turn-home could not start");
                }
                else
                {
                    _phase = Phase::RunTurnHome;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunTurnHome:
                return PollDrive(Phase::LaunchReturn);

            case Phase::LaunchReturn:
                if (!StartDriveStraight(
                        OutboundDistanceM(),
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex],
                        DirectionToUnitVector(MazeMap::Down),
                        StartCellCenter()))
                {
                    services.Fault("Corridor repeatability return drive could not start");
                }
                else
                {
                    _phase = Phase::RunReturn;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunReturn:
                return PollDrive(Phase::LaunchFaceNorth);

            case Phase::LaunchFaceNorth:
                if (!StartTurnAround())
                {
                    services.Fault("Corridor repeatability face-north turn could not start");
                }
                else
                {
                    _phase = Phase::RunFaceNorth;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunFaceNorth:
                return PollDrive(Phase::AdvanceSpeed);

            case Phase::AdvanceSpeed:
                ++_speedIndex;
                _phase =
                    (_speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount) ?
                        Phase::LaunchStartHold :
                        Phase::Complete;
                return LoopController::ControlVector::Brake;

            case Phase::Complete:
                services.RequestEndLoop();
                return LoopController::ControlVector::Brake;

            case Phase::Idle:
            default:
                services.Fault("Corridor repeatability phase was not initialized");
                return LoopController::ControlVector::Brake;
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        WallTouch& _wallTouch;
        bool _faulted{};
        bool _completed{};
        std::uint8_t _speedIndex{};
        Phase _phase{ Phase::Idle };
    };

    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::CorridorRepeatability,
            BootModeCategory::Utility,
            "corridor_repeatability",
            "Run clean single-session corridor repeatability passes.",
            "logging.txt",
            &GetCorridorRepeatabilityMode,
            "GetCorridorRepeatabilityMode",
            "CorridorRepeatabilityMode.cpp",
            "shared startup calibration; start settle; outbound drive; north-wall touch; return drive",
            "AuxMeasurementConfig corridor speeds; shared runtime drive and wall-touch services",
            "Structured telemetry export is temporarily removed during the execution-model cleanup",
            "none",
        };
        return descriptor;
    }

    IApplicationMode& GetCorridorRepeatabilityMode()
    {
        static CorridorRepeatabilityMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
