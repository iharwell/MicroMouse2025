#include "pch.h"
#include "PositionAccuracyAuditMode.h"

#include "AuxMeasurementConfig.h"
#include "AuxMeasurementModeSupport.h"
#include "BootUtilityModeFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"
#include "WallTouch.h"

namespace
{
    constexpr const char* kPositionAuditStableId = "position_accuracy_audit";
    constexpr float kPositionAuditWallTouchSpeedMps = 0.35f;

    MotionLimits BuildPositionWallTouchLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits =
            MazeMap::App::Internal::AuxMeasurementModeSupport::PositionAccuracyAuditStraightLimits(
                kPositionAuditWallTouchSpeedMps);
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }
}

namespace MazeMap::App::Internal
{
    class PositionAccuracyAuditMode final : public IApplicationMode
    {
    public:
        explicit PositionAccuracyAuditMode(SharedRobotRuntime& runtime)
            : _runtime(runtime)
            , _loopController(runtime.ControlLoop())
            , _vehicle(runtime.Vehicle())
            , _drive(runtime.Drive())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
            , _wallTouch(runtime.WallTouchService())
        {
        }

        void SetupMode() override
        {
            ResetState();
            if (!_runtime.RegisterModeFaultHandler(&PositionAccuracyAuditMode::TeardownOnRuntimeFault, this, kPositionAuditStableId))
            {
                _runtime.FailActiveMode("Position accuracy audit fault handler registration failed");
            }

            if (!SetupHardware())
            {
                _runtime.FailActiveMode("Position accuracy audit hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:position_accuracy_audit");
            (void)_runtime.AppendTextLogLine("Position accuracy audit mode");
            (void)_runtime.AppendTextLogLine("Single-session shared-service straight audit");
            _fixture = AuxMeasurementModeSupport::BuildPositionAuditFixtureGeometry();

            if (!_drive.Begin())
            {
                _runtime.FailActiveMode("Position accuracy audit drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                _runtime.FailActiveMode("Position accuracy audit startup bring-up failed");
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

        static void TeardownOnRuntimeFault(void* context, const char* reason) noexcept
        {
            (void)reason;
            auto* const self = static_cast<PositionAccuracyAuditMode*>(context);
            if (self == nullptr)
            {
                return;
            }

            self->_phase = Phase::Idle;
            self->_wallTouch.Cancel();
            self->_startupCalibration.Cancel();
            self->_drive.Brake();
            self->_drive.UseNominalWheelControlProfile();
        }

        LoopController::SessionOptions BuildLoopOptions() const noexcept
        {
            LoopController::SessionOptions options{};
            const auto& runtimeState = _runtime.RuntimeState();
            options.controlPeriodUs = AuxMeasurementConfig::kControlPeriodUs;
            options.SessionStartPointX = runtimeState.GetPositionX();
            options.SessionStartPointY = runtimeState.GetPositionY();
            return options;
        }

        void ResetState() noexcept
        {
            _speedIndex = 0U;
            _phase = Phase::Idle;
            _fixture = {};
            _wallTouch.Cancel();
            _startupCalibration.Cancel();
        }

        float OutboundDistanceM() const noexcept
        {
            return _fixture.outDistanceM;
        }

        Eigen::Vector2f StartCellCenter() const noexcept
        {
            return Eigen::Vector2f(0.5f * Config::kCellSizeM, 0.5f * Config::kCellSizeM);
        }

        Eigen::Vector2f FarCellCenter() const noexcept
        {
            return Eigen::Vector2f(0.5f * Config::kCellSizeM, _fixture.farCellCenterYM);
        }

        bool StartDriveHold(const std::uint16_t durationMs) noexcept
        {
            _driveService.SetLimits(
                AuxMeasurementModeSupport::PositionAccuracyAuditStraightLimits(
                    AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[_speedIndex]));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartHold(durationMs, true);
            return true;
        }

        bool StartDriveStraight(
            const float distanceM,
            const float cruiseSpeedMps,
            const Eigen::Vector2f& heading,
            const Eigen::Vector2f& targetPosition) noexcept
        {
            _driveService.SetLimits(AuxMeasurementModeSupport::PositionAccuracyAuditStraightLimits(cruiseSpeedMps));
            _driveService.SetOperationMode(Drive::OperationMode::Maze);
            _driveService.StartStraight(distanceM, cruiseSpeedMps, 0.0f, &heading, &targetPosition);
            return true;
        }

        bool StartTurnAround() noexcept
        {
            _driveService.SetLimits(AuxMeasurementModeSupport::PositionAccuracyAuditTurnLimits());
            _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            _driveService.StartTurn(PI_F);
            return true;
        }

        bool StartWallTouch() noexcept
        {
            _wallTouch.Cancel();
            _wallTouch.SetLimits(BuildPositionWallTouchLimits(_vehicle));
            _wallTouch.SetAllowPassThroughNoWall(false);
            _wallTouch.Start(
                MazeMap::CellCoordinates(
                    0U,
                    static_cast<std::uint8_t>(_fixture.northCorridorCellCount - 1U)),
                MazeMap::Up);
            return _wallTouch.Active();
        }

        CommandVector PollStartupCalibration()
        {
            bool done = false;
            const CommandVector control = _startupCalibration.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _phase = Phase::LaunchStartHold;
            return CommandVector::Brake();
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

        CommandVector PollWallTouch(const Phase nextPhase)
        {
            bool done = false;
            const CommandVector control = _wallTouch.GetNextControls(done);
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
            (void)state;

            switch (_phase)
            {
            case Phase::LaunchStartupCalibration:
                _startupCalibration.Start();
                if (!_startupCalibration.Active())
                {
                    _runtime.FailActiveMode("Position accuracy audit startup calibration could not start");
                }
                else
                {
                    _phase = Phase::RunStartupCalibration;
                }
                return CommandVector::Brake();

            case Phase::RunStartupCalibration:
                return PollStartupCalibration();

            case Phase::LaunchStartHold:
                if (!StartDriveHold(AuxMeasurementConfig::kPositionAuditStartSettleMs))
                {
                    _runtime.FailActiveMode("Position accuracy audit start hold could not start");
                }
                else
                {
                    _phase = Phase::RunStartHold;
                }
                return CommandVector::Brake();

            case Phase::RunStartHold:
                return PollDrive(Phase::LaunchOutbound);

            case Phase::LaunchOutbound:
                if (!StartDriveStraight(
                        OutboundDistanceM(),
                        AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[_speedIndex],
                        DirectionToUnitVector(MazeMap::Up),
                        FarCellCenter()))
                {
                    _runtime.FailActiveMode("Position accuracy audit outbound drive could not start");
                }
                else
                {
                    _phase = Phase::RunOutbound;
                }
                return CommandVector::Brake();

            case Phase::RunOutbound:
                return PollDrive(Phase::LaunchFarTouch);

            case Phase::LaunchFarTouch:
                if (!StartWallTouch())
                {
                    _runtime.FailActiveMode("Position accuracy audit wall touch could not start");
                }
                else
                {
                    _phase = Phase::RunFarTouch;
                }
                return CommandVector::Brake();

            case Phase::RunFarTouch:
                return PollWallTouch(Phase::LaunchTurnHome);

            case Phase::LaunchTurnHome:
                if (!StartTurnAround())
                {
                    _runtime.FailActiveMode("Position accuracy audit turn-home could not start");
                }
                else
                {
                    _phase = Phase::RunTurnHome;
                }
                return CommandVector::Brake();

            case Phase::RunTurnHome:
                return PollDrive(Phase::LaunchReturn);

            case Phase::LaunchReturn:
                if (!StartDriveStraight(
                        OutboundDistanceM(),
                        AuxMeasurementConfig::kPositionAuditStraightSpeedsMps[_speedIndex],
                        DirectionToUnitVector(MazeMap::Down),
                        StartCellCenter()))
                {
                    _runtime.FailActiveMode("Position accuracy audit return drive could not start");
                }
                else
                {
                    _phase = Phase::RunReturn;
                }
                return CommandVector::Brake();

            case Phase::RunReturn:
                return PollDrive(Phase::LaunchFaceNorth);

            case Phase::LaunchFaceNorth:
                if (!StartTurnAround())
                {
                    _runtime.FailActiveMode("Position accuracy audit face-north turn could not start");
                }
                else
                {
                    _phase = Phase::RunFaceNorth;
                }
                return CommandVector::Brake();

            case Phase::RunFaceNorth:
                return PollDrive(Phase::AdvanceSpeed);

            case Phase::AdvanceSpeed:
                ++_speedIndex;
                _phase =
                    (_speedIndex < AuxMeasurementConfig::kPositionAuditStraightSpeedCount) ?
                        Phase::LaunchStartHold :
                        Phase::Complete;
                return CommandVector::Brake();

            case Phase::Complete:
                (void)_runtime.AppendTextLogLine("Position accuracy audit complete");
                _wallTouch.Cancel();
                _startupCalibration.Cancel();
                _drive.Brake();
                _drive.UseNominalWheelControlProfile();
                _phase = Phase::Idle;
                loopController.HaltExecutionEndProgram();
                return CommandVector::Brake();

            case Phase::Idle:
            default:
                _runtime.FailActiveMode("Position accuracy audit phase was not initialized");
                return CommandVector::Brake();
            }
        }

        SharedRobotRuntime& _runtime;
        LoopController& _loopController;
        MazeMap::Vehicle& _vehicle;
        DriveBase& _drive;
        Drive& _driveService;
        StartupCalibration& _startupCalibration;
        WallTouch& _wallTouch;
        AuxMeasurementModeSupport::PositionAuditFixtureGeometry _fixture{};
        std::uint8_t _speedIndex{};
        Phase _phase{ Phase::Idle };
    };

    const BootModeDescriptor& GetPositionAccuracyAuditBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::PositionAccuracyAudit,
            BootModeCategory::Utility,
            "position_accuracy_audit",
            "Run clean single-session shared-service position audit passes.",
            "logging.txt",
            &GetPositionAccuracyAuditMode,
            "GetPositionAccuracyAuditMode",
            "PositionAccuracyAuditMode.cpp",
            "shared startup calibration; start settle; outbound drive; north-wall touch; return drive; face north; speed advance",
            "AuxMeasurementConfig straight-speed points; shared runtime drive and wall-touch services; shared audit fixture geometry",
            "Smooth-turn audit sections remain intentionally absent from this mode's shared-service path",
            "none",
        };
        return descriptor;
    }

    IApplicationMode& GetPositionAccuracyAuditMode()
    {
        static PositionAccuracyAuditMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
