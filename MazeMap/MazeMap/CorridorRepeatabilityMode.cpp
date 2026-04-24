#include "pch.h"
#include "CorridorRepeatabilityMode.h"

#include "AuxMeasurementConfig.h"
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
    constexpr const char* kCorridorStableId = "corridor_repeatability";

    MotionLimits BuildCorridorMotionLimits(const MazeMap::Vehicle& vehicle, const float maxSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = maxSpeedMps;
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
            if (!_runtime.RegisterModeFaultHandler(&CorridorRepeatabilityMode::TeardownOnRuntimeFault, this, kCorridorStableId))
            {
                return false;
            }

            if (!SetupHardware())
            {
                return _runtime.FailActiveMode("Corridor repeatability hardware setup failed");
            }

            (void)BootUtilityModeFramework::ResetStartupTrace("mode:corridor_repeatability");
            (void)_runtime.AppendTextLogLine("Corridor repeatability mode");
            (void)_runtime.AppendTextLogLine("Single-session shared-service corridor passes");

            if (!_drive.Begin())
            {
                return _runtime.FailActiveMode("Corridor repeatability drive base init failed");
            }
            _drive.UseNominalWheelControlProfile();

            _startupCalibration.Cancel();
            _startupCalibration.SetIsInMaze(true);
            if (!_startupCalibration.BringUp())
            {
                return _runtime.FailActiveMode("Corridor repeatability startup bring-up failed");
            }

            return true;
        }

        void Run() override
        {
            _phase = Phase::LaunchStartupCalibration;

            LoopController::ModeCallbacks callbacks{};
            callbacks.onModeWork = &CorridorRepeatabilityMode::ModeWorkThunk;
            callbacks.context = this;
            if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
            {
                (void)_runtime.FailActiveMode("Corridor repeatability loop session start failed");
            }
            else
            {
                const LoopController::SessionResult result = _loopController.Run();
                const bool completed =
                    (result.status == LoopController::SessionResult::Status::Completed);
                _loopController.EndSession();

                if (completed)
                {
                    (void)_runtime.AppendTextLogLine("Corridor repeatability complete");
                }
            }

            _wallTouch.Cancel();
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
            auto* const self = static_cast<CorridorRepeatabilityMode*>(context);
            if (self == nullptr)
            {
                return;
            }

            self->_phase = Phase::Idle;
            self->_wallTouch.Cancel();
            self->_startupCalibration.Cancel();
            self->_driveService.Cancel();
            self->_drive.Brake();
            self->_drive.UseNominalWheelControlProfile();
        }

        static LoopController::ControlVector ModeWorkThunk(
            void* context,
            std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
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
            _speedIndex = 0U;
            _phase = Phase::Idle;
            _wallTouch.Cancel();
            _startupCalibration.Cancel();
            _driveService.Cancel();
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

        LoopController::ControlVector RunTick(
            const std::uint32_t loopEndTimeUs,
            const MazeMap::VehicleState& state,
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

            case Phase::LaunchStartHold:
                _driveService.Cancel();
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::Maze);
                _driveService.StartHold(AuxMeasurementConfig::kCorridorRepeatabilityStartSettleMs, true);
                if (!_driveService.Active())
                {
                    services.Fault("Corridor repeatability start hold could not start");
                }
                else
                {
                    _phase = Phase::RunStartHold;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunStartHold:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchOutbound;
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchOutbound:
            {
                const Eigen::Vector2f heading = DirectionToUnitVector(MazeMap::Up);
                const Eigen::Vector2f targetPosition = FarCellCenter();
                _driveService.Cancel();
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
                if (!_driveService.Active())
                {
                    services.Fault("Corridor repeatability outbound drive could not start");
                }
                else
                {
                    _phase = Phase::RunOutbound;
                }
                return LoopController::ControlVector::Brake;
            }

            case Phase::RunOutbound:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchFarTouch;
                return LoopController::ControlVector::Brake;
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
                    services.Fault("Corridor repeatability wall touch could not start");
                }
                else
                {
                    _phase = Phase::RunFarTouch;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunFarTouch:
            {
                bool done = false;
                const LoopController::ControlVector control = _wallTouch.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchTurnHome;
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchTurnHome:
                _driveService.Cancel();
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                _driveService.StartTurn(PI_F);
                if (!_driveService.Active())
                {
                    services.Fault("Corridor repeatability turn-home could not start");
                }
                else
                {
                    _phase = Phase::RunTurnHome;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunTurnHome:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchReturn;
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchReturn:
            {
                const Eigen::Vector2f heading = DirectionToUnitVector(MazeMap::Down);
                const Eigen::Vector2f targetPosition = StartCellCenter();
                _driveService.Cancel();
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
                if (!_driveService.Active())
                {
                    services.Fault("Corridor repeatability return drive could not start");
                }
                else
                {
                    _phase = Phase::RunReturn;
                }
                return LoopController::ControlVector::Brake;
            }

            case Phase::RunReturn:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::LaunchFaceNorth;
                return LoopController::ControlVector::Brake;
            }

            case Phase::LaunchFaceNorth:
                _driveService.Cancel();
                _driveService.SetLimits(
                    BuildCorridorMotionLimits(
                        _vehicle,
                        AuxMeasurementConfig::kCorridorRepeatabilitySpeedsMps[_speedIndex]));
                _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                _driveService.StartTurn(PI_F);
                if (!_driveService.Active())
                {
                    services.Fault("Corridor repeatability face-north turn could not start");
                }
                else
                {
                    _phase = Phase::RunFaceNorth;
                }
                return LoopController::ControlVector::Brake;

            case Phase::RunFaceNorth:
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (!done)
                {
                    return control;
                }

                _phase = Phase::AdvanceSpeed;
                return LoopController::ControlVector::Brake;
            }

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
