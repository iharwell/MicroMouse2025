#include "pch.h"
#include "PositionAccuracyAuditMode.h"

#include "AuxMeasurementConfig.h"
#include "AuxMeasurementModeSupport.h"
#include "BootFramework.h"
#include "Drive.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeCore.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"
#include "WallTouch.h"

#include <limits>

namespace
{
    constexpr float kPositionAuditWallTouchSpeedMps = 0.35f;

    MotionLimits BuildPositionWallTouchLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits =
            MazeMap::App::Internal::AuxMeasurementModeSupport::PositionAccuracyAuditStraightLimits(
                kPositionAuditWallTouchSpeedMps);
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxYawRate());
        limits.SetAngularAccelRadps2(vehicle.GetMaxYawAccel());
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
            , _drive(runtime.DriveBase())
            , _driveService(runtime.DriveService())
            , _startupCalibration(runtime.StartupCalibrationService())
            , _wallTouch(runtime.WallTouchService())
        {
        }

        void SetupMode(BootFramework& framework) override
        {
            (void)framework;
            ResetState();
            (void)_runtime.AppendTextLogLine("Position accuracy audit mode");
            (void)_runtime.AppendTextLogLine("Single-session shared-service straight audit");
            _fixture = AuxMeasurementModeSupport::BuildPositionAuditFixtureGeometry();

            _drive.ClearCommandEvidence();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                _runtime.FailActiveMode("Position accuracy audit startup bring-up failed");
            }

            _phase = Phase::LaunchStartupCalibration;
            const auto& runtimeState = _runtime.RuntimeState();
            _loopController.StageNextSessionState(
                AuxMeasurementConfig::kControlPeriodUs,
                runtimeState.GetPositionX(),
                runtimeState.GetPositionY());
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

        void OnModeFault(const char* reason) noexcept override
        {
            (void)reason;
            _phase = Phase::Idle;
            _wallTouch.Cancel();
            _startupCalibration.Cancel();
            _drive.ClearCommandEvidence();
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
            {
                (void)_runtime.AppendTextLogLine("Position accuracy audit complete");
                _wallTouch.Cancel();
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
