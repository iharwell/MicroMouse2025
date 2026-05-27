#include "pch.h"
#include "CorridorRepeatabilityMode.h"

#include "AuxMeasurementConfig.h"
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
    MotionLimits BuildCorridorMotionLimits(const MazeMap::Vehicle& vehicle, const float maxSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.SetMaxSpeedMps(maxSpeedMps);
        limits.SetAccelMps2(AuxMeasurementConfig::kCorridorRepeatabilityAccelMps2);
        limits.SetDecelMps2(AuxMeasurementConfig::kCorridorRepeatabilityDecelMps2);
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxYawRate());
        limits.SetAngularAccelRadps2(vehicle.GetMaxYawAccel());
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
            (void)_runtime.AppendTextLogLine("Corridor repeatability mode");
            (void)_runtime.AppendTextLogLine("Single-session shared-service corridor passes");

            _drive.ClearCommandEvidence();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);

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
            _wallTouch.Cancel();
            _startupCalibration.Cancel();
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
                    _runtime.FailActiveMode("Corridor repeatability startup calibration could not start");
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

                _phase = Phase::LaunchStartHold;
                return CommandVector::Brake();
            }

            case Phase::LaunchStartHold:
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartHold(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs, true);
                _phase = Phase::RunStartHold;
                return CommandVector::Brake();

            case Phase::RunStartHold:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchOutbound;
                return CommandVector::Brake();
            }

            case Phase::LaunchOutbound:
            {
                const Eigen::Vector2f heading = DirectionToUnitVector(MazeMap::Up);
                const Eigen::Vector2f targetPosition = FarCellCenter();
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartStraight(
                    OutboundDistanceM(),
                    AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex],
                    0.0f,
                    &heading,
                    &targetPosition);
                _phase = Phase::RunOutbound;
                return CommandVector::Brake();
            }

            case Phase::RunOutbound:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchFarTouch;
                return CommandVector::Brake();
            }

            case Phase::LaunchFarTouch:
                _wallTouch.Cancel();
                _wallTouch.SetLimits(BuildCorridorMotionLimits(_vehicle, 0.35f));
                _wallTouch.SetAllowPassThroughNoWall(false);
                _wallTouch.Start(
                    MazeMap::CellCoordinates(
                        0U,
                        static_cast<std::uint8_t>(AuxMeasurementConfig::kCorridorRepeatabilityRowCellCount - 1U)),
                    MazeMap::Up);
                if (!_wallTouch.Active())
                {
                    _runtime.FailActiveMode("Corridor repeatability wall touch could not start");
                }
                else
                {
                    _phase = Phase::RunFarTouch;
                }
                return CommandVector::Brake();

            case Phase::RunFarTouch:
            {
                bool done = false;
                const CommandVector control = _wallTouch.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchTurnHome;
                return CommandVector::Brake();
            }

            case Phase::LaunchTurnHome:
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                _driveService.StartTurn(PI_F);
                _phase = Phase::RunTurnHome;
                return CommandVector::Brake();

            case Phase::RunTurnHome:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchReturn;
                return CommandVector::Brake();
            }

            case Phase::LaunchReturn:
            {
                const Eigen::Vector2f heading = DirectionToUnitVector(MazeMap::Down);
                const Eigen::Vector2f targetPosition = StartCellCenter();
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartStraight(
                    OutboundDistanceM(),
                    AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex],
                    0.0f,
                    &heading,
                    &targetPosition);
                _phase = Phase::RunReturn;
                return CommandVector::Brake();
            }

            case Phase::RunReturn:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchFaceNorth;
                return CommandVector::Brake();
            }

            case Phase::LaunchFaceNorth:
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                _driveService.StartTurn(PI_F);
                _phase = Phase::RunFaceNorth;
                return CommandVector::Brake();

            case Phase::RunFaceNorth:
            {
                bool done = false;
                const CommandVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::AdvanceSpeed;
                return CommandVector::Brake();
            }

            case Phase::AdvanceSpeed:
                ++_speedIndex;
                _phase =
                    (_speedIndex < AuxMeasurementConfig::kCorridorRepeatabilitySpeedCount) ?
                        Phase::LaunchStartHold :
                        Phase::Complete;
                return CommandVector::Brake();

            case Phase::Complete:
            {
                (void)_runtime.AppendTextLogLine("Corridor repeatability complete");
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
                _runtime.FailActiveMode("Corridor repeatability phase was not initialized");
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
        std::uint8_t _speedIndex{};
        Phase _phase{ Phase::Idle };
    };

    const BootModeDescriptor& GetCorridorRepeatabilityBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::CorridorRepeatability,
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
            true,
        };
        return descriptor;
    }

    IApplicationMode& GetCorridorRepeatabilityMode()
    {
        static CorridorRepeatabilityMode mode(GetSharedRobotRuntime());
        return mode;
    }
}
