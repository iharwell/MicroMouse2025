#include "pch.h"
#include "OpenFloorMeasurementController.h"

#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DiagnosticConfig.h"
#include "DriveBase.h"
#include "ManeuverQueue.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SharedRobotRuntime.h"
#include "SigmaPointSetSimplex.h"
#include "StartupCalibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace
{
    using MazeMap::App::Internal::Drive;
    using MazeMap::App::Internal::LoopController;
    using MazeMap::App::Internal::OpenFloorMeasurementController;
    using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
    using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;

    constexpr const char* kOpenFloorMeasurementStableId = "open_floor_measurement";
    constexpr const char* kOpenFloorMeasurementSelectorRemovedReason =
        "Open-floor measurement selector jumper removed";
    constexpr MazeMap::ManeuverCode kOpenFloorMeasurementSpeedChangeStraightCode = MazeMap::S1;
    constexpr MazeMap::OpenFloorPrimitiveId kOpenFloorMeasurementSpeedChangeStraightPrimitive =
        MazeMap::OpenFloorPrimitiveId::Str1;
    constexpr MazeMap::OpenFloorPrimitiveId kOpenFloorMeasurementLoopStraightPrimitive =
        MazeMap::OpenFloorPrimitiveId::Str2;
    constexpr std::uint16_t kOpenFloorStaticHoldWithLaunchSettleMs =
        DiagnosticConfig::kStaticHoldMs +
        static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs);

    struct OpenFloorCompiledManeuverDefinition final
    {
        MazeMap::ManeuverCode code{};
        MazeMap::OpenFloorPrimitiveId primitiveId{ MazeMap::OpenFloorPrimitiveId::None };
    };

    constexpr std::array<OpenFloorCompiledManeuverDefinition, 26U> kOpenFloorMeasurementSmoothCycle = {
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135LS, MazeMap::OpenFloorPrimitiveId::S135ls },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S90SD, MazeMap::OpenFloorPrimitiveId::S90sd },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S90SD_M, MazeMap::OpenFloorPrimitiveId::S90sdM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135LD_M, MazeMap::OpenFloorPrimitiveId::S135ldM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135LS_M, MazeMap::OpenFloorPrimitiveId::S135lsM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135LD, MazeMap::OpenFloorPrimitiveId::S135ld },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135SS, MazeMap::OpenFloorPrimitiveId::S135ss },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45LD, MazeMap::OpenFloorPrimitiveId::S45ld },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135SS_M, MazeMap::OpenFloorPrimitiveId::S135ssM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45LD_M, MazeMap::OpenFloorPrimitiveId::S45ldM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S180LS_M, MazeMap::OpenFloorPrimitiveId::S180lsM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45LS, MazeMap::OpenFloorPrimitiveId::S45ls },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135SD, MazeMap::OpenFloorPrimitiveId::S135sd },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45LS_M, MazeMap::OpenFloorPrimitiveId::S45lsM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S135SD_M, MazeMap::OpenFloorPrimitiveId::S135sdM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45SS_M, MazeMap::OpenFloorPrimitiveId::S45ssM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45SD_M, MazeMap::OpenFloorPrimitiveId::S45sdM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S90LS_M, MazeMap::OpenFloorPrimitiveId::S90lsM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S180LS, MazeMap::OpenFloorPrimitiveId::S180ls },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45SS, MazeMap::OpenFloorPrimitiveId::S45ss },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S45SD, MazeMap::OpenFloorPrimitiveId::S45sd },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S90SS, MazeMap::OpenFloorPrimitiveId::S90ss },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S90LS, MazeMap::OpenFloorPrimitiveId::S90ls },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S180SS_M, MazeMap::OpenFloorPrimitiveId::S180ssM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S90SS_M, MazeMap::OpenFloorPrimitiveId::S90ssM },
        OpenFloorCompiledManeuverDefinition{ MazeMap::S180SS, MazeMap::OpenFloorPrimitiveId::S180ss },
    };

    MotionLimits BuildOpenFloorMeasurementLimits(
        const MazeMap::Vehicle& vehicle,
        const float maxSpeedMps) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = maxSpeedMps;
        limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
        limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }

    MazeMap::DirectionalLocation OpenFloorMeasurementSmoothQueueStartLocation(
        const std::uint8_t speedIndex) noexcept
    {
        return MazeMap::DirectionalLocation(
            2U,
            static_cast<std::uint8_t>(3U + speedIndex),
            MazeMap::Up);
    }

    bool BuildOpenFloorMeasurementSmoothQueue(
        MazeMap::Vehicle& vehicle,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float initialEntrySpeedMps,
        MazeMap::ManeuverQueue& queue,
        float& exitBoundarySpeedMps)
    {
        queue.clear();
        exitBoundarySpeedMps = 0.0f;

        MazeMap::DirectionalLocation current = OpenFloorMeasurementSmoothQueueStartLocation(speedIndex);
        if (!queue.push_back(kOpenFloorMeasurementSpeedChangeStraightCode, current))
        {
            return false;
        }
        current = queue.back().getEnd();

        for (const OpenFloorCompiledManeuverDefinition& entry : kOpenFloorMeasurementSmoothCycle)
        {
            if (!queue.push_back(entry.code, current))
            {
                queue.clear();
                exitBoundarySpeedMps = 0.0f;
                return false;
            }
            current = queue.back().getEnd();
        }

        const bool isLastSpeedBin =
            (speedIndex + 1U) >= MazeMap::kOpenFloorSmoothSpeedBinsMps.size();
        if (isLastSpeedBin && !queue.push_back(kOpenFloorMeasurementSpeedChangeStraightCode, current))
        {
            queue.clear();
            exitBoundarySpeedMps = 0.0f;
            return false;
        }

        const float finalExitSpeedMps = isLastSpeedBin ? 0.0f : cruiseSpeedMps;
        queue.ComputeSpeeds(vehicle, initialEntrySpeedMps, finalExitSpeedMps);
        if (queue.empty())
        {
            return false;
        }

        exitBoundarySpeedMps = queue.back().getExitSpeed();
        return true;
    }

    MazeMap::DirectionalLocation OpenFloorMeasurementLoopQueueStartLocation(const bool clockwise) noexcept
    {
        return clockwise ?
            MazeMap::DirectionalLocation(3U, 3U, MazeMap::Up) :
            MazeMap::DirectionalLocation(7U, 3U, MazeMap::Up);
    }

    bool BuildOpenFloorMeasurementLoopQueue(
        MazeMap::Vehicle& vehicle,
        const bool clockwise,
        MazeMap::ManeuverQueue& queue)
    {
        queue.clear();

        MazeMap::DirectionalLocation current = OpenFloorMeasurementLoopQueueStartLocation(clockwise);
        const MazeMap::ManeuverCode turnCode = clockwise ? MazeMap::IP90 : MazeMap::IP90_M;
        for (std::uint8_t side = 0U; side < 4U; ++side)
        {
            if (!queue.push_back(MazeMap::S2, current))
            {
                queue.clear();
                return false;
            }
            current = queue.back().getEnd();

            if (!queue.push_back(turnCode, current))
            {
                queue.clear();
                return false;
            }
            current = queue.back().getEnd();
        }

        queue.ComputeSpeeds(vehicle, 0.0f, 0.0f);
        return !queue.empty();
    }

    bool WriteOpenFloorV62Metadata(MazeMap::App::Internal::SharedRobotRuntime& runtime)
    {
        const MazeMap::PlantModel::PreparedParams prepared =
            MazeMap::PlantModel::Prepare(MazeMap::PlantParams::Default());
        return
            runtime.WriteUtilityDataLogMetadata("ukfver", "v6.2") &&
            runtime.WriteUtilityDataLogMetadata("ukfset", "splx") &&
            runtime.WriteUtilityDataLogMetadataUnsigned(
                "nx",
                static_cast<unsigned long>(MazeMap::VehicleState::kDimension)) &&
            runtime.WriteUtilityDataLogMetadataUnsigned(
                "nsig",
                static_cast<unsigned long>(
                    MazeMap::SigmaPointSetSimplex::ActiveSigmaCountForDimension(
                        MazeMap::VehicleState::kDimension))) &&
            runtime.WriteUtilityDataLogMetadataFloat("re_m", prepared.wheelRadiusM, 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("we_m", prepared.trackWidthM, 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("imu_x", prepared.imuPositionBodyM.x(), 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("imu_y", prepared.imuPositionBodyM.y(), 6) &&
            runtime.WriteUtilityDataLogMetadataFloat("jw_kgm2", prepared.wheelInertiaKgM2, 9);
    }

    void ApplyControlTimingToTimingRow(
        const ControlCycleTiming& timing,
        OpenFloorTimingRow& row) noexcept
    {
        row.control_start_us = timing.controlStartUs;
        row.control_end_us = timing.controlEndUs;
        row.pwm_latch_us = timing.pwmLatchUs;
        row.encoder_latch_us = timing.encoderLatchUs;
        row.encoder_read_done_us = timing.encoderReadDoneUs;
        row.ukf_predict_start_us = timing.ukfPredictStartUs;
        row.ukf_predict_end_us = timing.ukfPredictEndUs;
        row.ukf_predict_duration_us = timing.ukfPredictDurationUs;
        row.ukf_update_start_us = timing.ukfUpdateStartUs;
        row.ukf_update_end_us = timing.ukfUpdateEndUs;
        row.ukf_update_duration_us = timing.ukfUpdateDurationUs;
        row.cycle_counter_start = timing.cycleCounterStart;
        row.cycle_counter_end = timing.cycleCounterEnd;
    }
}

namespace MazeMap::App::Internal
{
    OpenFloorMeasurementController::OpenFloorMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.Sensors())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
    }

    OpenFloorMeasurementController::~OpenFloorMeasurementController() = default;

    void OpenFloorMeasurementController::ActiveSegmentExecution::Reset() noexcept
    {
        _state = std::monostate{};
    }

    void OpenFloorMeasurementController::ActiveSegmentExecution::Rebind(
        const CompiledSegment& segment) noexcept
    {
        if (std::holds_alternative<CompiledSegment::HoldPlan>(segment._plan))
        {
            if (!std::holds_alternative<HoldExecution>(_state))
            {
                _state = HoldExecution{};
            }
            return;
        }

        if (std::holds_alternative<CompiledSegment::WheelCommandProfilePlan>(segment._plan))
        {
            if (!std::holds_alternative<WheelCommandProfileExecution>(_state))
            {
                _state = WheelCommandProfileExecution{};
            }
            return;
        }

        if (std::holds_alternative<CompiledSegment::StraightPlan>(segment._plan))
        {
            if (!std::holds_alternative<StraightExecution>(_state))
            {
                _state = StraightExecution{};
            }
            return;
        }

        if (std::holds_alternative<CompiledSegment::TurnPlan>(segment._plan))
        {
            if (!std::holds_alternative<TurnExecution>(_state))
            {
                _state = TurnExecution{};
            }
            return;
        }

        if (!std::holds_alternative<ManeuverExecution>(_state))
        {
            _state = ManeuverExecution{};
        }
    }

    OpenFloorMeasurementController::CompiledSegment OpenFloorMeasurementController::CompiledSegment::Hold(
        const LoggedRowIdentity identity,
        const std::uint16_t durationMs) noexcept
    {
        return CompiledSegment(identity, 0U, HoldPlan{ durationMs });
    }

    OpenFloorMeasurementController::CompiledSegment
        OpenFloorMeasurementController::CompiledSegment::WheelCommandProfile(
            const LoggedRowIdentity identity,
            const std::uint16_t durationMs,
            const float leftCommand,
            const float rightCommand,
            const std::uint16_t settlingHoldMs) noexcept
    {
        return CompiledSegment(
            identity,
            settlingHoldMs,
            WheelCommandProfilePlan{ durationMs, leftCommand, rightCommand });
    }

    OpenFloorMeasurementController::CompiledSegment OpenFloorMeasurementController::CompiledSegment::Straight(
        const LoggedRowIdentity identity,
        const float distanceM,
        const float speedMps,
        const std::uint16_t settlingHoldMs) noexcept
    {
        return CompiledSegment(identity, settlingHoldMs, StraightPlan{ distanceM, speedMps });
    }

    OpenFloorMeasurementController::CompiledSegment OpenFloorMeasurementController::CompiledSegment::Turn(
        const LoggedRowIdentity identity,
        const float yawRad,
        const float maxOmegaRadps,
        const std::uint16_t settlingHoldMs) noexcept
    {
        return CompiledSegment(identity, settlingHoldMs, TurnPlan{ yawRad, maxOmegaRadps });
    }

    OpenFloorMeasurementController::CompiledSegment OpenFloorMeasurementController::CompiledSegment::Maneuver(
        const LoggedRowIdentity identity,
        const std::uint16_t maneuverIndex,
        const float speedMps,
        const std::uint16_t settlingHoldMs) noexcept
    {
        return CompiledSegment(identity, settlingHoldMs, ManeuverPlan{ maneuverIndex, speedMps });
    }

    const OpenFloorMeasurementController::LoggedRowIdentity&
        OpenFloorMeasurementController::CompiledSegment::RowIdentity() const noexcept
    {
        return _rowIdentity;
    }

    OpenFloorMeasurementController::BatteryPhaseId
        OpenFloorMeasurementController::CompiledSegment::PhaseId() const noexcept
    {
        return _rowIdentity.phaseId;
    }

    const char* OpenFloorMeasurementController::CompiledSegment::FaultReasonText() const noexcept
    {
        switch (_rowIdentity.phaseId)
        {
        case BatteryPhaseId::Timing:
            return "timing";
        case BatteryPhaseId::Static:
            return "static";
        case BatteryPhaseId::Launch:
            return "launch";
        case BatteryPhaseId::Straight:
            return "straight";
        case BatteryPhaseId::Yaw:
            return "yaw";
        case BatteryPhaseId::Smooth:
            return "smooth";
        case BatteryPhaseId::LoopClockwise:
            return "clockwise loop";
        case BatteryPhaseId::LoopCounterClockwise:
            return "counter-clockwise loop";
        default:
            return "unknown";
        }
    }

    LoopController::ControlVector OpenFloorMeasurementController::CompiledSegment::TickExecution(
        OpenFloorMeasurementController& controller,
        ActiveSegmentExecution& execution,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done) const
    {
        (void)state;
        execution.Rebind(*this);

        if (const auto* const holdPlan = std::get_if<HoldPlan>(&_plan))
        {
            return TickHoldExecution(
                controller,
                holdPlan->durationMs,
                std::get<ActiveSegmentExecution::HoldExecution>(execution._state),
                done);
        }
        if (const auto* const wheelPlan = std::get_if<WheelCommandProfilePlan>(&_plan))
        {
            return TickWheelCommandProfileExecution(
                controller,
                *wheelPlan,
                std::get<ActiveSegmentExecution::WheelCommandProfileExecution>(execution._state),
                done);
        }
        if (const auto* const straightPlan = std::get_if<StraightPlan>(&_plan))
        {
            return TickStraightExecution(
                controller,
                services,
                *straightPlan,
                std::get<ActiveSegmentExecution::StraightExecution>(execution._state),
                done);
        }
        if (const auto* const turnPlan = std::get_if<TurnPlan>(&_plan))
        {
            return TickTurnExecution(
                controller,
                services,
                *turnPlan,
                std::get<ActiveSegmentExecution::TurnExecution>(execution._state),
                done);
        }

        return TickManeuverExecution(
            controller,
            services,
            std::get<ManeuverPlan>(_plan),
            std::get<ActiveSegmentExecution::ManeuverExecution>(execution._state),
            done);
    }

    OpenFloorMeasurementController::CompiledSegment::CompiledSegment(
        const LoggedRowIdentity identity,
        const std::uint16_t settlingHoldMs,
        const Plan& plan) noexcept
        : _rowIdentity(identity)
        , _settlingHoldMs(settlingHoldMs)
        , _plan(plan)
    {
    }

    LoopController::ControlVector OpenFloorMeasurementController::CompiledSegment::TickHoldExecution(
        OpenFloorMeasurementController& controller,
        const std::uint16_t durationMs,
        ActiveSegmentExecution::HoldExecution& execution,
        bool& done) const
    {
        if (!execution.started)
        {
            execution = {};
            execution.started = true;
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(durationMs, false);
        }

        const LoopController::ControlVector candidateControl = controller._driveService.GetNextControls(done);
        if (done)
        {
            execution = {};
            return LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        }

        return candidateControl;
    }

    LoopController::ControlVector
        OpenFloorMeasurementController::CompiledSegment::TickWheelCommandProfileExecution(
            OpenFloorMeasurementController& controller,
            const WheelCommandProfilePlan& plan,
            ActiveSegmentExecution::WheelCommandProfileExecution& execution,
            bool& done) const
    {
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (execution.settling)
        {
            const LoopController::ControlVector control =
                TickHoldExecution(controller, _settlingHoldMs, execution.settlingHold, done);
            if (done)
            {
                execution = {};
            }
            return control;
        }

        if (!execution.started)
        {
            execution = {};
            execution.started = true;
            execution.deadlineMs = millis() + plan.durationMs;
        }

        if (static_cast<long>(execution.deadlineMs - millis()) <= 0)
        {
            if (_settlingHoldMs == 0U)
            {
                done = true;
                execution = {};
            }
            else
            {
                execution.settling = true;
                execution.settlingHold = {};
                done = false;
            }
            return stopControl;
        }

        done = false;
        return LoopController::ControlVector::RawMotorPwm(plan.leftCommand, plan.rightCommand);
    }

    LoopController::ControlVector OpenFloorMeasurementController::CompiledSegment::TickStraightExecution(
        OpenFloorMeasurementController& controller,
        LoopController::TickServices& services,
        const StraightPlan& plan,
        ActiveSegmentExecution::StraightExecution& execution,
        bool& done) const
    {
        (void)services;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (execution.settling)
        {
            const LoopController::ControlVector control =
                TickHoldExecution(controller, _settlingHoldMs, execution.settlingHold, done);
            if (done)
            {
                execution = {};
            }
            return control;
        }

        if (!execution.started)
        {
            execution = {};
            execution.started = true;
            controller._driveService.SetLimits(
                BuildOpenFloorMeasurementLimits(controller._vehicle, std::fabs(plan.speedMps)));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartStraight(plan.distanceM, plan.speedMps, 0.0f);
            execution.startDistanceM = controller._drive.GetAverageDistanceMeters();
            execution.totalDistanceM = plan.distanceM;
        }

        const LoopController::ControlVector candidateControl = controller._driveService.GetNextControls(done);
        if (done)
        {
            if (_settlingHoldMs == 0U)
            {
                execution = {};
                return stopControl;
            }

            execution.settling = true;
            execution.settlingHold = {};
            done = false;
            return stopControl;
        }

        return candidateControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::CompiledSegment::TickTurnExecution(
        OpenFloorMeasurementController& controller,
        LoopController::TickServices& services,
        const TurnPlan& plan,
        ActiveSegmentExecution::TurnExecution& execution,
        bool& done) const
    {
        (void)services;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (execution.settling)
        {
            const LoopController::ControlVector control =
                TickHoldExecution(controller, _settlingHoldMs, execution.settlingHold, done);
            if (done)
            {
                execution = {};
            }
            return control;
        }

        if (!execution.started)
        {
            execution = {};
            execution.started = true;

            MotionLimits limits = BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f);
            limits.maxAngularSpeedRadps = plan.maxOmegaRadps;
            controller._driveService.SetLimits(limits);
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartTurn(plan.yawRad);
            execution.targetYawRad =
                WrapAngleRad(controller._runtime.RuntimeState().GetOrientation() + plan.yawRad);
            execution.targetMagnitudeRad = std::fabs(plan.yawRad);
        }

        const LoopController::ControlVector candidateControl = controller._driveService.GetNextControls(done);
        if (done)
        {
            if (_settlingHoldMs == 0U)
            {
                execution = {};
                return stopControl;
            }

            execution.settling = true;
            execution.settlingHold = {};
            done = false;
            return stopControl;
        }

        return candidateControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::CompiledSegment::TickManeuverExecution(
        OpenFloorMeasurementController& controller,
        LoopController::TickServices& services,
        const ManeuverPlan& plan,
        ActiveSegmentExecution::ManeuverExecution& execution,
        bool& done) const
    {
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (execution.settling)
        {
            const LoopController::ControlVector control =
                TickHoldExecution(controller, _settlingHoldMs, execution.settlingHold, done);
            if (done)
            {
                execution = {};
            }
            return control;
        }

        if (!execution.started)
        {
            const MazeMap::ManeuverInstance* const maneuver =
                controller._mainStage.CompiledManeuverAt(plan.maneuverIndex);
            if (maneuver == nullptr)
            {
                services.Fault("Open-floor compiled maneuver segment referenced an invalid maneuver");
                done = true;
                execution = {};
                return LoopController::ControlVector::Brake;
            }

            execution = {};
            execution.started = true;
            controller._driveService.SetLimits(
                BuildOpenFloorMeasurementLimits(controller._vehicle, plan.speedMps));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartManeuver(*maneuver);
            execution.startDistanceM = controller._drive.GetAverageDistanceMeters();
            execution.totalDistanceM = maneuver->GetTravelDistanceMeters();
            execution.targetYawRad =
                WrapAngleRad(
                    controller._runtime.RuntimeState().GetOrientation() +
                    (static_cast<float>(MazeMap::CodeDegrees(maneuver->getCode())) * DEG_TO_RAD_F));
            execution.targetMagnitudeRad =
                std::fabs(static_cast<float>(MazeMap::CodeDegrees(maneuver->getCode())) * DEG_TO_RAD_F);
        }

        const LoopController::ControlVector candidateControl = controller._driveService.GetNextControls(done);
        if (done)
        {
            if (_settlingHoldMs == 0U)
            {
                execution = {};
                return stopControl;
            }

            execution.settling = true;
            execution.settlingHold = {};
            done = false;
            return stopControl;
        }

        return candidateControl;
    }

    void OpenFloorMeasurementController::TimingStage::Reset() noexcept
    {
        _tickIndex = 0U;
        _logOpen = false;
        _pendingRow.reset();
    }

    bool OpenFloorMeasurementController::TimingStage::Begin(OpenFloorMeasurementController& controller)
    {
        Reset();
        if (!controller._runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorTimingFileName))
        {
            return false;
        }
        if (!controller._runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorTimingStreamType)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("boot_reason", MazeMap::kOpenFloorBootReason)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!WriteOpenFloorV62Metadata(controller._runtime)) return false;

        Runtime::OpenFloorTimingRow row{};
        if (!controller._runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _logOpen = true;
        return true;
    }

    LoopController::ControlVector OpenFloorMeasurementController::TimingStage::Tick(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (!FlushPending(controller, "Open-floor measurement timing log write failed", &services))
        {
            return stopControl;
        }
        if (controller.SelectorRemoved())
        {
            services.Fault(kOpenFloorMeasurementSelectorRemovedReason);
            return stopControl;
        }
        if (controller._runtime.Estimator().HasFault())
        {
            services.Fault("Estimator fault during timing capture");
            return stopControl;
        }

        Runtime::OpenFloorTimingRow row{};
        controller.PopulateTimingRowFromState(state, row);
        StageRow(row);
        ++_tickIndex;
        if (CaptureComplete())
        {
            controller._pauseAction = PauseAction::TimingToMain;

            LoopController::PauseRequest request{};
            request.onPauseGranted = &OpenFloorMeasurementController::PauseThunk;
            request.reason = "open_floor_timing_to_main";
            request.flushLogsBeforeGrant = true;
            request.resetClockOnResume = true;
            services.RequestPause(request);
        }

        return stopControl;
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::TimingStage::CompleteTimingToMainHandoff(
        OpenFloorMeasurementController& controller,
        MainStage& mainStage,
        const LoopController::PauseContext& pause)
    {
        (void)pause;
        if (controller._pauseAction != PauseAction::TimingToMain)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement pause granted without a pending timing transition");
        }
        if (!FlushPending(
                controller,
                "Open-floor measurement timing log write failed during timing transition",
                nullptr))
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement timing log write failed during timing transition");
        }

        _logOpen = false;
        if (!mainStage.Begin(controller))
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement main log setup failed after timing capture");
        }

        controller._activeStageTick = &OpenFloorMeasurementController::MainStageTick;
        controller._pauseAction = PauseAction::None;
        return LoopController::PauseDisposition::Resume();
    }

    void OpenFloorMeasurementController::TimingStage::FinalizeCompletedRun(
        OpenFloorMeasurementController& controller) noexcept
    {
        if (_logOpen)
        {
            (void)FlushPending(controller, "Open-floor measurement timing log write failed", nullptr);
            _logOpen = false;
        }
    }

    bool OpenFloorMeasurementController::TimingStage::FlushPending(
        OpenFloorMeasurementController& controller,
        const char* failureReason,
        LoopController::TickServices* const services)
    {
        if (!_pendingRow.has_value())
        {
            return true;
        }

        Runtime::OpenFloorTimingRow& row = *_pendingRow;
        ApplyControlTimingToTimingRow(controller._loopController.LastDiagnostics().controlTiming, row);
        if (!controller._runtime.LogUtilityDataRow(row))
        {
            if ((services != nullptr) && (failureReason != nullptr))
            {
                services->Fault(failureReason);
            }
            return false;
        }

        _pendingRow.reset();
        return true;
    }

    void OpenFloorMeasurementController::TimingStage::StageRow(const Runtime::OpenFloorTimingRow& row)
    {
        _pendingRow = row;
    }

    bool OpenFloorMeasurementController::TimingStage::CaptureComplete() const noexcept
    {
        return _tickIndex >= DiagnosticConfig::kTimingCaptureCycles;
    }

    void OpenFloorMeasurementController::MainStage::Reset() noexcept
    {
        _planSize = 0U;
        _maneuverCount = 0U;
        _nextSegmentIndex = 0U;
        _logOpen = false;
        _completionPending = false;
        _activeExecution.Reset();
        _estimatorFaultReason[0] = '\0';
        _pendingRow.reset();
    }

    bool OpenFloorMeasurementController::MainStage::CompilePlan(OpenFloorMeasurementController& controller)
    {
        Reset();

        const auto makeIdentity =
            [](const BatteryPhaseId phaseId,
               const OpenFloorPrimitiveId primitiveId,
               const OpenFloorSpeedBin speedBin,
               const std::uint16_t repeatIndex) noexcept
        {
            return LoggedRowIdentity(phaseId, primitiveId, speedBin, repeatIndex);
        };

        if (!AppendSegment(
                CompiledSegment::Hold(
                    makeIdentity(
                        BatteryPhaseId::Static,
                        OpenFloorPrimitiveId::StaticHold,
                        OpenFloorSpeedBin::None,
                        1U),
                    kOpenFloorStaticHoldWithLaunchSettleMs)))
        {
            return false;
        }

        std::uint16_t launchRepeatIndex = 0U;
        for (const float magnitude : MazeMap::kOpenFloorLaunchDriveMagnitudes)
        {
            for (std::uint8_t repeat = 0U; repeat < MazeMap::kOpenFloorLaunchRepeatsPerMagnitude; ++repeat)
            {
                for (const float sign : { 1.0f, -1.0f })
                {
                    if (!AppendSegment(
                            CompiledSegment::WheelCommandProfile(
                                makeIdentity(
                                    BatteryPhaseId::Launch,
                                    OpenFloorPrimitiveId::OpenLoopLaunch,
                                    OpenFloorSpeedBin::None,
                                    ++launchRepeatIndex),
                                static_cast<std::uint16_t>(MazeMap::kOpenFloorLaunchPulseMs),
                                magnitude * sign,
                                magnitude * sign,
                                static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs))))
                    {
                        return false;
                    }
                }
            }
        }

        std::uint16_t straightRepeatIndex = 0U;
        const float straightDistanceM = MazeMap::OpenFloorStrEquivalentDistanceMeters(4U);
        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size(); ++speedIndex)
        {
            const float speedMps = MazeMap::kOpenFloorStraightSpeedBinsMps[speedIndex];
            const OpenFloorSpeedBin speedBin = OpenFloorMeasurementController::SpeedBinForIndex(speedIndex);
            for (std::uint8_t repeat = 0U; repeat < MazeMap::kOpenFloorStraightRepeatsPerSpeed; ++repeat)
            {
                for (const float direction : { 1.0f, -1.0f })
                {
                    if (!AppendSegment(
                            CompiledSegment::Straight(
                                makeIdentity(
                                    BatteryPhaseId::Straight,
                                    OpenFloorPrimitiveId::Str4,
                                    speedBin,
                                    ++straightRepeatIndex),
                                straightDistanceM,
                                direction * speedMps,
                                static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs))))
                    {
                        return false;
                    }
                }
            }
        }

        if (MazeMap::kOpenFloorYawPrimitiveIds.size() != MazeMap::kOpenFloorYawNominalAnglesRad.size())
        {
            return false;
        }

        std::uint16_t yawRepeatIndex = 0U;
        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorYawOmegaBinsRadps.size(); ++speedIndex)
        {
            const OpenFloorSpeedBin speedBin = OpenFloorMeasurementController::SpeedBinForIndex(speedIndex);
            for (std::uint8_t repeat = 0U; repeat < DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed; ++repeat)
            {
                for (std::size_t primitiveIndex = 0U;
                     primitiveIndex < MazeMap::kOpenFloorYawPrimitiveIds.size();
                     ++primitiveIndex)
                {
                    if (!AppendSegment(
                            CompiledSegment::Turn(
                                makeIdentity(
                                    BatteryPhaseId::Yaw,
                                    MazeMap::kOpenFloorYawPrimitiveIds[primitiveIndex],
                                    speedBin,
                                    ++yawRepeatIndex),
                                MazeMap::kOpenFloorYawNominalAnglesRad[primitiveIndex],
                                MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex],
                                static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs))))
                    {
                        return false;
                    }
                }
            }
        }

        float entryBoundarySpeedMps = 0.0f;
        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size(); ++speedIndex)
        {
            MazeMap::ManeuverQueue queue{};
            float exitBoundarySpeedMps = 0.0f;
            const bool isLastSpeedBin = (speedIndex + 1U) == MazeMap::kOpenFloorSmoothSpeedBinsMps.size();
            if (!BuildOpenFloorMeasurementSmoothQueue(
                    controller._vehicle,
                    static_cast<std::uint8_t>(speedIndex),
                    MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex],
                    entryBoundarySpeedMps,
                    queue,
                    exitBoundarySpeedMps))
            {
                return false;
            }

            const OpenFloorSpeedBin speedBin = OpenFloorMeasurementController::SpeedBinForIndex(speedIndex);
            for (std::size_t entryIndex = 0U; entryIndex < queue.size(); ++entryIndex)
            {
                const MazeMap::ManeuverInstance& maneuver = queue[static_cast<std::uint16_t>(entryIndex)];
                const bool isLastQueueEntry = (entryIndex + 1U) == queue.size();
                const bool isClosingSpeedChange = isLastSpeedBin && isLastQueueEntry;
                const OpenFloorPrimitiveId primitiveId =
                    ((entryIndex == 0U) || isClosingSpeedChange) ?
                        kOpenFloorMeasurementSpeedChangeStraightPrimitive :
                        kOpenFloorMeasurementSmoothCycle[entryIndex - 1U].primitiveId;

                std::uint16_t maneuverIndex = 0U;
                if (!StoreCompiledManeuver(maneuver, maneuverIndex))
                {
                    return false;
                }

                if (!AppendSegment(
                        CompiledSegment::Maneuver(
                            makeIdentity(BatteryPhaseId::Smooth, primitiveId, speedBin, 1U),
                            maneuverIndex,
                            MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex],
                            (isLastSpeedBin && isLastQueueEntry) ?
                                static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs) :
                                0U)))
                {
                    return false;
                }
            }

            entryBoundarySpeedMps = exitBoundarySpeedMps;
        }

        for (const bool clockwise : { true, false })
        {
            MazeMap::ManeuverQueue queue{};
            if (!BuildOpenFloorMeasurementLoopQueue(controller._vehicle, clockwise, queue))
            {
                return false;
            }

            const BatteryPhaseId phaseId =
                clockwise ? BatteryPhaseId::LoopClockwise : BatteryPhaseId::LoopCounterClockwise;
            const OpenFloorPrimitiveId turnPrimitiveId =
                clockwise ? OpenFloorPrimitiveId::Ip90 : OpenFloorPrimitiveId::Ip90M;
            const std::uint16_t finalSettlingHoldMs =
                clockwise ? static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs) : 0U;
            for (std::uint16_t repeatIndex = 1U; repeatIndex <= DiagnosticConfig::kLoopRepeats; ++repeatIndex)
            {
                for (std::size_t entryIndex = 0U; entryIndex < queue.size(); ++entryIndex)
                {
                    std::uint16_t maneuverIndex = 0U;
                    if (!StoreCompiledManeuver(queue[static_cast<std::uint16_t>(entryIndex)], maneuverIndex))
                    {
                        return false;
                    }

                    const bool isLastEntry = (entryIndex + 1U) == queue.size();
                    if (!AppendSegment(
                            CompiledSegment::Maneuver(
                                makeIdentity(
                                    phaseId,
                                    ((entryIndex % 2U) == 0U) ?
                                        kOpenFloorMeasurementLoopStraightPrimitive :
                                        turnPrimitiveId,
                                    OpenFloorSpeedBin::Low,
                                    repeatIndex),
                                maneuverIndex,
                                MazeMap::kOpenFloorStraightSpeedBinsMps[0],
                                ((repeatIndex == DiagnosticConfig::kLoopRepeats) && isLastEntry) ?
                                    finalSettlingHoldMs :
                                    0U)))
                    {
                        return false;
                    }
                }
            }
        }

        return _planSize > 0U;
    }

    bool OpenFloorMeasurementController::MainStage::Begin(OpenFloorMeasurementController& controller)
    {
        _nextSegmentIndex = 0U;
        _completionPending = (_planSize == 0U);
        _activeExecution.Reset();
        _pendingRow.reset();
        _estimatorFaultReason[0] = '\0';

        if (!controller._runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorMainFileName))
        {
            return false;
        }
        if (!controller._runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorMainStreamType)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("boot_reason", MazeMap::kOpenFloorBootReason)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!WriteOpenFloorV62Metadata(controller._runtime)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("fan_duty_cycle", GetMissionFanDutyCycle(), 3)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", controller._sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", controller._sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", controller._sensors.GetGyroBiasRadps(), 6)) return false;
        if (!controller._runtime.WriteUtilityDataLogAccelBiasMetadata(controller._sensors)) return false;

        Runtime::OpenFloorMainRow row{};
        if (!controller._runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _logOpen = true;
        return true;
    }

    LoopController::ControlVector OpenFloorMeasurementController::MainStage::Tick(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (!FlushPending(controller, &services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }

        const CompiledSegment* const segment = ActiveSegment();
        if (_completionPending || (segment == nullptr))
        {
            services.RequestEndLoop();
            return stopControl;
        }
        if (CheckFault(controller, services))
        {
            return stopControl;
        }

        bool done = false;
        const LoopController::ControlVector control =
            segment->TickExecution(controller, _activeExecution, state, services, done);
        Runtime::OpenFloorMainRow row{};
        controller.PopulateMainRowFromState(segment->RowIdentity(), state, row);
        StageRow(row);

        if (done)
        {
            Advance();
            return stopControl;
        }

        return control;
    }

    void OpenFloorMeasurementController::MainStage::FinalizeCompletedRun(
        OpenFloorMeasurementController& controller) noexcept
    {
        if (_logOpen)
        {
            (void)FlushPending(controller, nullptr, "Open-floor measurement main log write failed");
            _logOpen = false;
        }
    }

    bool OpenFloorMeasurementController::MainStage::AppendSegment(const CompiledSegment& segment)
    {
        if (_planSize >= _plan.size())
        {
            return false;
        }

        _plan[_planSize++] = segment;
        return true;
    }

    bool OpenFloorMeasurementController::MainStage::StoreCompiledManeuver(
        const MazeMap::ManeuverInstance& maneuver,
        std::uint16_t& maneuverIndex)
    {
        if (_maneuverCount >= _maneuvers.size())
        {
            return false;
        }

        maneuverIndex = _maneuverCount;
        _maneuvers[_maneuverCount++] = maneuver;
        return true;
    }

    const MazeMap::ManeuverInstance* OpenFloorMeasurementController::MainStage::CompiledManeuverAt(
        const std::uint16_t maneuverIndex) const noexcept
    {
        return (maneuverIndex < _maneuverCount) ? &_maneuvers[maneuverIndex] : nullptr;
    }

    bool OpenFloorMeasurementController::MainStage::FlushPending(
        OpenFloorMeasurementController& controller,
        LoopController::TickServices* const services,
        const char* const failureReason)
    {
        if (!_pendingRow.has_value())
        {
            return true;
        }

        Runtime::OpenFloorMainRow& row = *_pendingRow;
        row.encoder_timestamp_us = controller._loopController.LastDiagnostics().controlTiming.encoderReadDoneUs;
        if (!controller._runtime.LogUtilityDataRow(row))
        {
            if ((services != nullptr) && (failureReason != nullptr))
            {
                services->Fault(failureReason);
            }
            return false;
        }

        _pendingRow.reset();
        return true;
    }

    void OpenFloorMeasurementController::MainStage::StageRow(const Runtime::OpenFloorMainRow& row)
    {
        _pendingRow = row;
    }

    bool OpenFloorMeasurementController::MainStage::CheckFault(
        OpenFloorMeasurementController& controller,
        LoopController::TickServices& services)
    {
        if (controller.SelectorRemoved())
        {
            services.Fault(kOpenFloorMeasurementSelectorRemovedReason);
            return true;
        }
        if (!controller._runtime.Estimator().HasFault())
        {
            return false;
        }

        const CompiledSegment* const segment = ActiveSegment();
        if (segment == nullptr)
        {
            services.Fault("Estimator fault during open-floor main stage");
            return true;
        }

        std::snprintf(
            _estimatorFaultReason,
            sizeof(_estimatorFaultReason),
            "Estimator fault during %s phase %u",
            segment->FaultReasonText(),
            static_cast<unsigned>(static_cast<std::uint8_t>(segment->PhaseId())));
        services.Fault(_estimatorFaultReason);
        return true;
    }

    void OpenFloorMeasurementController::MainStage::Advance() noexcept
    {
        ++_nextSegmentIndex;
        _activeExecution.Reset();
        if (_nextSegmentIndex >= _planSize)
        {
            _completionPending = true;
        }
    }

    const OpenFloorMeasurementController::CompiledSegment*
        OpenFloorMeasurementController::MainStage::ActiveSegment() const noexcept
    {
        return (_nextSegmentIndex < _planSize) ? &_plan[_nextSegmentIndex] : nullptr;
    }

    bool OpenFloorMeasurementController::Begin()
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(
                &OpenFloorMeasurementController::TeardownOnRuntimeFault,
                this,
                kOpenFloorMeasurementStableId))
        {
            return false;
        }
        if (!SetupHardware())
        {
            return _runtime.FailActiveMode("Open-floor measurement hardware setup failed");
        }

        (void)BootUtilityModeFramework::ResetStartupTrace("mode:open_floor_measurement");
        (void)_runtime.AppendTextLogLine("Open-floor measurement mode");
        (void)_runtime.AppendTextLogLine(
            "Open-floor battery: timing -> static -> launch -> straight -> yaw -> smooth -> loop cw -> loop ccw");

        if (!_drive.Begin())
        {
            return _runtime.FailActiveMode("Open-floor measurement drive base init failed");
        }
        _drive.UseNominalWheelControlProfile();

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        if (!_startupCalibration.BringUp())
        {
            return _runtime.FailActiveMode("Open-floor measurement startup bring-up failed");
        }
        SetMissionLevelFanEnabled(true);

        ConfigureSelectorMonitor();
        if (SelectorRemoved())
        {
            return _runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
        }

        if (!_mainStage.CompilePlan(*this))
        {
            return _runtime.FailActiveMode("Open-floor measurement plan compilation failed");
        }
        if (!_timingStage.Begin(*this))
        {
            return _runtime.FailActiveMode("Open-floor measurement timing log setup failed");
        }

        return true;
    }

    void OpenFloorMeasurementController::Run()
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &OpenFloorMeasurementController::ModeWorkThunk;
        callbacks.context = this;

        bool completed = false;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            (void)_runtime.FailActiveMode("Open-floor measurement loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            completed = (result.status == LoopController::SessionResult::Status::Completed);
            _loopController.EndSession();
        }

        if (completed)
        {
            _mainStage.FinalizeCompletedRun(*this);
            _timingStage.FinalizeCompletedRun(*this);
        }

        ReleaseSelectorMonitor();

        if (completed)
        {
            (void)_runtime.AppendTextLogLine("Open-floor measurement complete");
        }
    }

    void OpenFloorMeasurementController::TeardownOnRuntimeFault(void* context, const char* reason) noexcept
    {
        (void)reason;
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        if (self == nullptr)
        {
            return;
        }

        self->ReleaseSelectorMonitor();
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::PauseThunk(
        void* context,
        const LoopController::PauseContext& pause)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        return (self != nullptr) ?
            self->OnPauseGranted(pause) :
            LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement pause callback context was null");
    }

    LoopController::ControlVector OpenFloorMeasurementController::ModeWorkThunk(
        void* context,
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        if (self == nullptr)
        {
            services.Fault("Open-floor measurement callback context was not installed");
            return LoopController::ControlVector::Brake;
        }

        return (self->*self->_activeStageTick)(loopEndTimeUs, state, services);
    }

    LoopController::SessionOptions OpenFloorMeasurementController::BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.workPlan.useWallUpdates = false;
        return options;
    }

    void OpenFloorMeasurementController::ResetState() noexcept
    {
        _startupCalibration.Cancel();
        ReleaseSelectorMonitor();
        _activeStageTick = &OpenFloorMeasurementController::TimingStageTick;
        _pauseAction = PauseAction::None;
        _timingStage.Reset();
        _mainStage.Reset();
    }

    void OpenFloorMeasurementController::PopulateTimingRowFromState(
        const MazeMap::VehicleState& state,
        Runtime::OpenFloorTimingRow& row) const noexcept
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        row.mono_time_us = _loopController.CurrentTickStartUs();
        row.control_tick_sequence = _loopController.CurrentTickSequence();
        row.dt_us = _loopController.CurrentTickDtUs();
        row.phase_id = static_cast<std::uint32_t>(static_cast<std::uint8_t>(BatteryPhaseId::Timing));
        row.imu_drdy_us = sensors.imuTiming.drdyUs;
        row.imu_read_start_us = sensors.imuTiming.readStartUs;
        row.imu_read_done_us = sensors.imuTiming.readDoneUs;
        row.front_led_on_us = sensors.frontTiming.ledOnCommandUs;
        row.front_adc_on_us = sensors.frontTiming.adcOnSampleUs;
        row.front_led_off_us = sensors.frontTiming.ledOffCommandUs;
        row.front_adc_off_us = sensors.frontTiming.adcOffSampleUs;
        row.front_ready_us = sensors.frontTiming.observationReadyUs;
        row.left_led_on_us = sensors.leftTiming.ledOnCommandUs;
        row.left_adc_on_us = sensors.leftTiming.adcOnSampleUs;
        row.left_led_off_us = sensors.leftTiming.ledOffCommandUs;
        row.left_adc_off_us = sensors.leftTiming.adcOffSampleUs;
        row.left_ready_us = sensors.leftTiming.observationReadyUs;
        row.right_led_on_us = sensors.rightTiming.ledOnCommandUs;
        row.right_adc_on_us = sensors.rightTiming.adcOnSampleUs;
        row.right_led_off_us = sensors.rightTiming.ledOffCommandUs;
        row.right_adc_off_us = sensors.rightTiming.adcOffSampleUs;
        row.right_ready_us = sensors.rightTiming.observationReadyUs;
        row.wall_adc_cfg_before_start = sensors.wallSensorAdcCfgBeforeStart;
        row.wall_adc_gc_before_start = sensors.wallSensorAdcGcBeforeStart;
        row.wall_adc_cfg_after_start = sensors.wallSensorAdcCfgAfterStart;
        row.wall_adc_gc_after_start = sensors.wallSensorAdcGcAfterStart;
        row.wall_adc_target_cfg = sensors.wallSensorAdcTargetCfg;
        row.wall_adc_ipg_clock_hz = sensors.wallSensorAdcIpgClockHz;
    }

    void OpenFloorMeasurementController::PopulateMainRowFromState(
        const LoggedRowIdentity& identity,
        const MazeMap::VehicleState& state,
        Runtime::OpenFloorMainRow& row) const
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
        const MazeMap::VehicleState::DriveCommandState& commandState = state.GetDriveCommandState();
        const MazeMap::DriveCommandPair appliedDriveCommand = state.GetAppliedDriveCommand();
        const MazeMap::VehicleState::StateVector estimatorState = state.GetStateVector();
        const MazeMap::PlantPreparedParams& prepared = _runtime.Estimator().ukf().preparedParams();
        const float wheelRadiusM =
            (std::isfinite(prepared.wheelRadiusM) && (prepared.wheelRadiusM > 0.0f)) ?
                prepared.wheelRadiusM :
                0.0f;
        const float trackWidthM =
            (std::isfinite(prepared.trackWidthM) && (prepared.trackWidthM > 0.0f)) ?
                prepared.trackWidthM :
                0.0f;
        const float leftWheelVelocityMps = wheelRadiusM * state.GetWheelSpeedLeft();
        const float rightWheelVelocityMps = wheelRadiusM * state.GetWheelSpeedRight();
        const float measuredLinearSpeedMps = 0.5f * (leftWheelVelocityMps + rightWheelVelocityMps);
        const float measuredAngularSpeedRadps =
            std::isfinite(sensors.gyroRadps) ?
                sensors.gyroRadps :
                ((trackWidthM > 0.0f) ?
                    ((leftWheelVelocityMps - rightWheelVelocityMps) / trackWidthM) :
                    0.0f);

        row.master_time_us = _loopController.CurrentTickStartUs();
        row.control_tick_sequence = _loopController.CurrentTickSequence();
        row.dt_us = _loopController.CurrentTickDtUs();
        row.phase_id = static_cast<std::uint8_t>(identity.phaseId);
        row.primitive_id = static_cast<std::uint8_t>(identity.primitiveId);
        row.speed_bin = static_cast<std::uint8_t>(identity.speedBin);
        row.repeat_index = identity.repeatIndex;
        row.mode_flags = commandState.modeFlags;
        row.saturation_flags = commandState.saturationFlags;
        row.ukf_mode_id = driveTelemetry.ukfModeId;
        row.ukf_yaw_valid_for_feedforward = driveTelemetry.ukfYawValidForFeedforward;
        row.bias_update_enabled = driveTelemetry.ukfBiasUpdateEnabled;
        row.gyro_bias_anchor_radps = driveTelemetry.ukfGyroBiasAnchorRadps;
        row.yaw_consistency_lp_radps = driveTelemetry.ukfYawConsistencyLowPassRadps;
        row.yaw_window_mismatch_rad = driveTelemetry.ukfYawWindowMismatchRad;
        row.nhc_sigma_mps = driveTelemetry.ukfNhcSigmaMps;
        row.nhc_residual_mps = driveTelemetry.ukfNhcResidualMps;
        row.nhc_residual_sigma = driveTelemetry.ukfNhcResidualSigma;
        row.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
        row.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
        row.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
        row.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
        row.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
        row.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
        row.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
        row.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
        row.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
        row.measured_linear_speed_mps = measuredLinearSpeedMps;
        row.measured_angular_speed_radps = measuredAngularSpeedRadps;
        row.cmd_linear_mps = commandState.commandedLinearSpeedMps;
        row.cmd_angular_radps = commandState.commandedAngularSpeedRadps;
        row.left_drive_command = appliedDriveCommand.left;
        row.right_drive_command = appliedDriveCommand.right;
        row.left_feedforward_command = commandState.feedforward.left;
        row.right_feedforward_command = commandState.feedforward.right;
        row.left_feedback_command = commandState.feedback.left;
        row.right_feedback_command = commandState.feedback.right;
        row.left_target_velocity_mps = commandState.leftTargetVelocityMps;
        row.right_target_velocity_mps = commandState.rightTargetVelocityMps;
        row.left_launch_assist_floor = commandState.leftLaunchAssistFloor;
        row.right_launch_assist_floor = commandState.rightLaunchAssistFloor;
        row.encoder_timestamp_us = 0U;
        row.left_encoder_count = driveTelemetry.leftEncoderCount;
        row.right_encoder_count = driveTelemetry.rightEncoderCount;
        row.left_encoder_omega_radps = driveTelemetry.leftEncoderOmegaRadps;
        row.right_encoder_omega_radps = driveTelemetry.rightEncoderOmegaRadps;
        row.left_encoder_distance_m = driveTelemetry.leftDistanceM;
        row.right_encoder_distance_m = driveTelemetry.rightDistanceM;
        row.left_encoder_velocity_mps = driveTelemetry.leftVelocityMps;
        row.right_encoder_velocity_mps = driveTelemetry.rightVelocityMps;
        row.imu_timestamp_us = sensors.imuTiming.readDoneUs;
        row.imu_status = sensors.imuBackLeft.status;
        row.accel_bias_valid = sensors.accelBiasValid ? 1U : 0U;
        row.imu_gyro_x = sensors.imuBackLeft.gyroX;
        row.imu_gyro_y = sensors.imuBackLeft.gyroY;
        row.imu_gyro_z = sensors.imuBackLeft.gyroZ;
        row.imu_accel_x = sensors.imuBackLeft.accelX;
        row.imu_accel_y = sensors.imuBackLeft.accelY;
        row.imu_accel_z = sensors.imuBackLeft.accelZ;
        row.imu_temp = sensors.imuBackLeft.temp;
        row.gyro_raw_radps = sensors.gyroRawRadps;
        row.gyro_bias_radps = sensors.gyroBiasRadps;
        row.gyro_radps = sensors.gyroRadps;
        row.accel_body_x_mps2 = sensors.accelBodyXMps2;
        row.accel_body_y_mps2 = sensors.accelBodyYMps2;
        row.planar_accel_mps2 = sensors.planarAccelMps2;
        row.front_timestamp_us = sensors.frontTiming.observationReadyUs;
        row.left_timestamp_us = sensors.leftTiming.observationReadyUs;
        row.right_timestamp_us = sensors.rightTiming.observationReadyUs;
    }

    void OpenFloorMeasurementController::ConfigureSelectorMonitor() noexcept
    {
        ReleaseSelectorMonitor();
        const BootModeRegistryEntry* const entry =
            FindBootModeRegistryEntry(BootModeId::OpenFloorMeasurement);
        if ((entry == nullptr) || (entry->selector.kind != BootModeSelectorKind::PinPair))
        {
            return;
        }

        _selectorDrivePin = entry->selector.pinA;
        _selectorSensePin = entry->selector.pinB;
        BeginPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        _selectorMonitorArmed = true;
    }

    void OpenFloorMeasurementController::ReleaseSelectorMonitor() noexcept
    {
        if (_selectorMonitorArmed)
        {
            EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        }
        _selectorMonitorArmed = false;
        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
    }

    bool OpenFloorMeasurementController::SelectorRemoved() const noexcept
    {
        return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::OnPauseGranted(
        const LoopController::PauseContext& pause)
    {
        return _timingStage.CompleteTimingToMainHandoff(*this, _mainStage, pause);
    }

    LoopController::ControlVector OpenFloorMeasurementController::TimingStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        return _timingStage.Tick(*this, state, services);
    }

    LoopController::ControlVector OpenFloorMeasurementController::MainStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        return _mainStage.Tick(*this, state, services);
    }

    OpenFloorSpeedBin OpenFloorMeasurementController::SpeedBinForIndex(const std::size_t speedIndex) noexcept
    {
        return (speedIndex == 0U) ? OpenFloorSpeedBin::Low :
            (speedIndex == 1U) ? OpenFloorSpeedBin::Medium :
            OpenFloorSpeedBin::High;
    }

    IApplicationMode& GetOpenFloorMeasurementMode();

    const BootModeDescriptor& GetOpenFloorMeasurementBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::OpenFloorMeasurement,
            BootModeCategory::Utility,
            "open_floor_measurement",
            "Run the ordered open-floor measurement battery with timing, static, launch, straight, yaw, smooth, and closed maneuver loops.",
            "open_floor_timing.mmlog, open_floor_main.mmlog",
            &GetOpenFloorMeasurementMode,
            "GetOpenFloorMeasurementMode",
            "OpenFloorMeasurementController.cpp",
            "timing capture; static hold; launch PWM pulses; straight drive tests; yaw drive tests; smooth maneuver sweep; clockwise closed maneuver loop; counter-clockwise closed maneuver loop",
            "DiagnosticConfig linear limits; OpenFloorMeasurementSpec speed bins; shared startup calibration; shared drive service",
            "Inter-phase 500 ms brake holds; launch and straight samples insert 250 ms brake holds between motions; smooth phase uses the current hand-picked closed maneuver sequence; loop phases are maneuver-driven",
            "open_floor_timing.mmlog, open_floor_main.mmlog",
        };
        return descriptor;
    }

    IApplicationMode& GetOpenFloorMeasurementMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}
