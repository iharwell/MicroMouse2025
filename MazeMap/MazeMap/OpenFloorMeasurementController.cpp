#include "pch.h"
#include "OpenFloorMeasurementController.h"

#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DriveBase.h"
#include "ManeuverQueue.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SharedRobotRuntime.h"
#include "SigmaPointSetSimplex.h"
#include "StartupCalibration.h"

#include <cmath>

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
    constexpr std::array<MazeMap::ManeuverCode, 26U> kOpenFloorMeasurementSmoothCycleCodes = {
        MazeMap::S135LS,
        MazeMap::S90SD,
        MazeMap::S90SD_M,
        MazeMap::S135LD_M,
        MazeMap::S135LS_M,
        MazeMap::S135LD,
        MazeMap::S135SS,
        MazeMap::S45LD,
        MazeMap::S135SS_M,
        MazeMap::S45LD_M,
        MazeMap::S180LS_M,
        MazeMap::S45LS,
        MazeMap::S135SD,
        MazeMap::S45LS_M,
        MazeMap::S135SD_M,
        MazeMap::S45SS_M,
        MazeMap::S45SD_M,
        MazeMap::S90LS_M,
        MazeMap::S180LS,
        MazeMap::S45SS,
        MazeMap::S45SD,
        MazeMap::S90SS,
        MazeMap::S90LS,
        MazeMap::S180SS_M,
        MazeMap::S90SS_M,
        MazeMap::S180SS,
    };

    constexpr std::array<MazeMap::OpenFloorPrimitiveId, 26U> kOpenFloorMeasurementSmoothCyclePrimitives = {
        MazeMap::OpenFloorPrimitiveId::S135ls,
        MazeMap::OpenFloorPrimitiveId::S90sd,
        MazeMap::OpenFloorPrimitiveId::S90sdM,
        MazeMap::OpenFloorPrimitiveId::S135ldM,
        MazeMap::OpenFloorPrimitiveId::S135lsM,
        MazeMap::OpenFloorPrimitiveId::S135ld,
        MazeMap::OpenFloorPrimitiveId::S135ss,
        MazeMap::OpenFloorPrimitiveId::S45ld,
        MazeMap::OpenFloorPrimitiveId::S135ssM,
        MazeMap::OpenFloorPrimitiveId::S45ldM,
        MazeMap::OpenFloorPrimitiveId::S180lsM,
        MazeMap::OpenFloorPrimitiveId::S45ls,
        MazeMap::OpenFloorPrimitiveId::S135sd,
        MazeMap::OpenFloorPrimitiveId::S45lsM,
        MazeMap::OpenFloorPrimitiveId::S135sdM,
        MazeMap::OpenFloorPrimitiveId::S45ssM,
        MazeMap::OpenFloorPrimitiveId::S45sdM,
        MazeMap::OpenFloorPrimitiveId::S90lsM,
        MazeMap::OpenFloorPrimitiveId::S180ls,
        MazeMap::OpenFloorPrimitiveId::S45ss,
        MazeMap::OpenFloorPrimitiveId::S45sd,
        MazeMap::OpenFloorPrimitiveId::S90ss,
        MazeMap::OpenFloorPrimitiveId::S90ls,
        MazeMap::OpenFloorPrimitiveId::S180ssM,
        MazeMap::OpenFloorPrimitiveId::S90ssM,
        MazeMap::OpenFloorPrimitiveId::S180ss,
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

    LoopController::ControlVector StopControlVector() noexcept
    {
        return LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
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

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::StaticMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Static;
    }

    void OpenFloorMeasurementController::StaticMeasurementPhase::Reset() noexcept
    {
    }

    bool OpenFloorMeasurementController::StaticMeasurementPhase::Compile(
        OpenFloorMeasurementController& controller,
        MainStage& stage)
    {
        (void)controller;
        return stage.RegisterUnrepeatedCase(
            *this,
            0U,
            OpenFloorPrimitiveId::StaticHold,
            OpenFloorSpeedBin::None);
    }

    void OpenFloorMeasurementController::StaticMeasurementPhase::BeginCase(
        OpenFloorMeasurementController& controller,
        const std::uint16_t caseIndex)
    {
        (void)caseIndex;
        controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartHold(DiagnosticConfig::kStaticHoldMs, false);
    }

    LoopController::ControlVector OpenFloorMeasurementController::StaticMeasurementPhase::TickCase(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
        return done ? StopControlVector() : control;
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::LaunchMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Launch;
    }

    void OpenFloorMeasurementController::LaunchMeasurementPhase::Reset() noexcept
    {
        _caseCount = 0U;
        _activeLeftCommand = 0.0f;
        _activeRightCommand = 0.0f;
        _deadlineMs = 0U;
    }

    bool OpenFloorMeasurementController::LaunchMeasurementPhase::Compile(
        OpenFloorMeasurementController& controller,
        MainStage& stage)
    {
        (void)controller;
        Reset();

        for (const float magnitude : MazeMap::kOpenFloorLaunchDriveMagnitudes)
        {
            for (std::uint8_t repeat = 0U; repeat < MazeMap::kOpenFloorLaunchRepeatsPerMagnitude; ++repeat)
            {
                for (const float sign : { 1.0f, -1.0f })
                {
                    if (_caseCount >= _leftCommands.size())
                    {
                        return false;
                    }

                    _leftCommands[_caseCount] = magnitude * sign;
                    _rightCommands[_caseCount] = magnitude * sign;
                    if (!stage.RegisterSequentialCase(
                            *this,
                            _caseCount,
                            OpenFloorPrimitiveId::OpenLoopLaunch,
                            OpenFloorSpeedBin::None))
                    {
                        return false;
                    }

                    ++_caseCount;
                }
            }
        }

        return _caseCount > 0U;
    }

    void OpenFloorMeasurementController::LaunchMeasurementPhase::BeginCase(
        OpenFloorMeasurementController& controller,
        const std::uint16_t caseIndex)
    {
        (void)controller;
        _activeLeftCommand = _leftCommands[caseIndex];
        _activeRightCommand = _rightCommands[caseIndex];
        _deadlineMs = millis() + static_cast<std::uint32_t>(MazeMap::kOpenFloorLaunchPulseMs);
    }

    LoopController::ControlVector OpenFloorMeasurementController::LaunchMeasurementPhase::TickCase(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)controller;
        (void)state;
        (void)services;

        if (static_cast<long>(_deadlineMs - millis()) <= 0)
        {
            done = true;
            return StopControlVector();
        }

        done = false;
        return LoopController::ControlVector::RawMotorPwm(_activeLeftCommand, _activeRightCommand);
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::StraightMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Straight;
    }

    void OpenFloorMeasurementController::StraightMeasurementPhase::Reset() noexcept
    {
        _speedsMps.fill(0.0f);
        _caseCount = 0U;
    }

    bool OpenFloorMeasurementController::StraightMeasurementPhase::Compile(
        OpenFloorMeasurementController& controller,
        MainStage& stage)
    {
        (void)controller;
        Reset();

        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorStraightSpeedBinsMps.size(); ++speedIndex)
        {
            const float speedMps = MazeMap::kOpenFloorStraightSpeedBinsMps[speedIndex];
            const OpenFloorSpeedBin speedBin = OpenFloorMeasurementController::SpeedBinForIndex(speedIndex);
            for (std::uint8_t repeat = 0U; repeat < MazeMap::kOpenFloorStraightRepeatsPerSpeed; ++repeat)
            {
                for (const float direction : { 1.0f, -1.0f })
                {
                    if (_caseCount >= _speedsMps.size())
                    {
                        return false;
                    }

                    _speedsMps[_caseCount] = direction * speedMps;
                    if (!stage.RegisterSequentialCase(*this, _caseCount, OpenFloorPrimitiveId::Str4, speedBin))
                    {
                        return false;
                    }

                    ++_caseCount;
                }
            }
        }

        return _caseCount > 0U;
    }

    void OpenFloorMeasurementController::StraightMeasurementPhase::BeginCase(
        OpenFloorMeasurementController& controller,
        const std::uint16_t caseIndex)
    {
        const float speedMps = _speedsMps[caseIndex];
        controller._driveService.SetLimits(
            BuildOpenFloorMeasurementLimits(controller._vehicle, std::fabs(speedMps)));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartStraight(
            MazeMap::OpenFloorStrEquivalentDistanceMeters(4U),
            speedMps,
            0.0f);
    }

    LoopController::ControlVector OpenFloorMeasurementController::StraightMeasurementPhase::TickCase(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
        return done ? StopControlVector() : control;
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::YawMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Yaw;
    }

    void OpenFloorMeasurementController::YawMeasurementPhase::Reset() noexcept
    {
        _yawRad.fill(0.0f);
        _maxOmegaRadps.fill(0.0f);
        _caseCount = 0U;
    }

    bool OpenFloorMeasurementController::YawMeasurementPhase::Compile(
        OpenFloorMeasurementController& controller,
        MainStage& stage)
    {
        (void)controller;
        Reset();

        if (MazeMap::kOpenFloorYawPrimitiveIds.size() != MazeMap::kOpenFloorYawNominalAnglesRad.size())
        {
            return false;
        }

        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorYawOmegaBinsRadps.size(); ++speedIndex)
        {
            const OpenFloorSpeedBin speedBin = OpenFloorMeasurementController::SpeedBinForIndex(speedIndex);
            for (std::uint8_t repeat = 0U; repeat < DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed; ++repeat)
            {
                for (std::size_t primitiveIndex = 0U;
                     primitiveIndex < MazeMap::kOpenFloorYawPrimitiveIds.size();
                     ++primitiveIndex)
                {
                    if (_caseCount >= _yawRad.size())
                    {
                        return false;
                    }

                    _yawRad[_caseCount] = MazeMap::kOpenFloorYawNominalAnglesRad[primitiveIndex];
                    _maxOmegaRadps[_caseCount] = MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex];
                    if (!stage.RegisterSequentialCase(
                            *this,
                            _caseCount,
                            MazeMap::kOpenFloorYawPrimitiveIds[primitiveIndex],
                            speedBin))
                    {
                        return false;
                    }

                    ++_caseCount;
                }
            }
        }

        return _caseCount > 0U;
    }

    void OpenFloorMeasurementController::YawMeasurementPhase::BeginCase(
        OpenFloorMeasurementController& controller,
        const std::uint16_t caseIndex)
    {
        MotionLimits limits = BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f);
        limits.maxAngularSpeedRadps = _maxOmegaRadps[caseIndex];
        controller._driveService.SetLimits(limits);
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartTurn(_yawRad[caseIndex]);
    }

    LoopController::ControlVector OpenFloorMeasurementController::YawMeasurementPhase::TickCase(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
        return done ? StopControlVector() : control;
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::SmoothMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Smooth;
    }

    void OpenFloorMeasurementController::SmoothMeasurementPhase::Reset() noexcept
    {
        _speedLimitsMps.fill(0.0f);
        _caseCount = 0U;
    }

    bool OpenFloorMeasurementController::SmoothMeasurementPhase::Compile(
        OpenFloorMeasurementController& controller,
        MainStage& stage)
    {
        Reset();

        float entryBoundarySpeedMps = 0.0f;
        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size(); ++speedIndex)
        {
            MazeMap::ManeuverQueue queue{};
            float exitBoundarySpeedMps = 0.0f;
            if (!BuildQueue(
                    controller._vehicle,
                    static_cast<std::uint8_t>(speedIndex),
                    MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex],
                    entryBoundarySpeedMps,
                    queue,
                    exitBoundarySpeedMps))
            {
                return false;
            }

            const bool isLastSpeedBin = (speedIndex + 1U) == MazeMap::kOpenFloorSmoothSpeedBinsMps.size();
            const OpenFloorSpeedBin speedBin = OpenFloorMeasurementController::SpeedBinForIndex(speedIndex);
            for (std::size_t entryIndex = 0U; entryIndex < queue.size(); ++entryIndex)
            {
                if (_caseCount >= _maneuvers.size())
                {
                    return false;
                }

                const bool isLastQueueEntry = (entryIndex + 1U) == queue.size();
                const bool isClosingSpeedChange = isLastSpeedBin && isLastQueueEntry;
                const OpenFloorPrimitiveId primitiveId =
                    ((entryIndex == 0U) || isClosingSpeedChange) ?
                        kOpenFloorMeasurementSpeedChangeStraightPrimitive :
                        kOpenFloorMeasurementSmoothCyclePrimitives[entryIndex - 1U];

                _maneuvers[_caseCount] = queue[static_cast<std::uint16_t>(entryIndex)];
                _speedLimitsMps[_caseCount] = MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex];
                if (!stage.RegisterUnrepeatedCase(*this, _caseCount, primitiveId, speedBin))
                {
                    return false;
                }

                ++_caseCount;
            }

            entryBoundarySpeedMps = exitBoundarySpeedMps;
        }

        return _caseCount > 0U;
    }

    void OpenFloorMeasurementController::SmoothMeasurementPhase::BeginCase(
        OpenFloorMeasurementController& controller,
        const std::uint16_t caseIndex)
    {
        controller._driveService.SetLimits(
            BuildOpenFloorMeasurementLimits(controller._vehicle, _speedLimitsMps[caseIndex]));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartManeuver(_maneuvers[caseIndex]);
    }

    LoopController::ControlVector OpenFloorMeasurementController::SmoothMeasurementPhase::TickCase(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
        return done ? StopControlVector() : control;
    }

    bool OpenFloorMeasurementController::SmoothMeasurementPhase::BuildQueue(
        MazeMap::Vehicle& vehicle,
        const std::uint8_t speedIndex,
        const float cruiseSpeedMps,
        const float initialEntrySpeedMps,
        MazeMap::ManeuverQueue& queue,
        float& exitBoundarySpeedMps) const
    {
        queue.clear();
        exitBoundarySpeedMps = 0.0f;

        MazeMap::DirectionalLocation current(
            2U,
            static_cast<std::uint8_t>(3U + speedIndex),
            MazeMap::Up);
        if (!queue.push_back(kOpenFloorMeasurementSpeedChangeStraightCode, current))
        {
            return false;
        }
        current = queue.back().getEnd();

        for (std::size_t entryIndex = 0U; entryIndex < kOpenFloorMeasurementSmoothCycleCodes.size(); ++entryIndex)
        {
            if (!queue.push_back(kOpenFloorMeasurementSmoothCycleCodes[entryIndex], current))
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

    OpenFloorMeasurementController::LoopMeasurementPhase::LoopMeasurementPhase(
        const MeasurementPhaseId phaseId,
        const bool clockwise) noexcept
        : _phaseId(phaseId)
        , _clockwise(clockwise)
    {
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::LoopMeasurementPhase::PhaseId() const noexcept
    {
        return _phaseId;
    }

    void OpenFloorMeasurementController::LoopMeasurementPhase::Reset() noexcept
    {
        _caseCount = 0U;
    }

    bool OpenFloorMeasurementController::LoopMeasurementPhase::Compile(
        OpenFloorMeasurementController& controller,
        MainStage& stage)
    {
        Reset();

        MazeMap::ManeuverQueue queue{};
        if (!BuildQueue(controller._vehicle, queue))
        {
            return false;
        }
        if (!stage.BeginRepeatedSequence(DiagnosticConfig::kLoopRepeats))
        {
            return false;
        }

        const OpenFloorPrimitiveId turnPrimitiveId =
            _clockwise ? OpenFloorPrimitiveId::Ip90 : OpenFloorPrimitiveId::Ip90M;
        for (std::size_t entryIndex = 0U; entryIndex < queue.size(); ++entryIndex)
        {
            if (_caseCount >= _maneuvers.size())
            {
                return false;
            }

            _maneuvers[_caseCount] = queue[static_cast<std::uint16_t>(entryIndex)];
            if (!stage.RegisterRepeatedSequenceCase(
                    *this,
                    _caseCount,
                    ((entryIndex % 2U) == 0U) ? kOpenFloorMeasurementLoopStraightPrimitive : turnPrimitiveId,
                    OpenFloorSpeedBin::Low))
            {
                return false;
            }

            ++_caseCount;
        }

        return stage.EndRepeatedSequence();
    }

    void OpenFloorMeasurementController::LoopMeasurementPhase::BeginCase(
        OpenFloorMeasurementController& controller,
        const std::uint16_t caseIndex)
    {
        controller._driveService.SetLimits(
            BuildOpenFloorMeasurementLimits(
                controller._vehicle,
                MazeMap::kOpenFloorStraightSpeedBinsMps[0]));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartManeuver(_maneuvers[caseIndex]);
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopMeasurementPhase::TickCase(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
        return done ? StopControlVector() : control;
    }

    bool OpenFloorMeasurementController::LoopMeasurementPhase::BuildQueue(
        MazeMap::Vehicle& vehicle,
        MazeMap::ManeuverQueue& queue) const
    {
        queue.clear();

        MazeMap::DirectionalLocation current =
            _clockwise ?
                MazeMap::DirectionalLocation(3U, 3U, MazeMap::Up) :
                MazeMap::DirectionalLocation(7U, 3U, MazeMap::Up);
        const MazeMap::ManeuverCode turnCode = _clockwise ? MazeMap::IP90 : MazeMap::IP90_M;
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

    OpenFloorMeasurementController::MainStage::ScheduledWork
        OpenFloorMeasurementController::MainStage::ScheduledWork::PhaseCase(
            MainMeasurementPhase& phase,
            const std::uint16_t caseIndex,
            const MainRowLabel& labels) noexcept
    {
        ScheduledWork work{};
        work._kind = Kind::PhaseCase;
        work._phase = &phase;
        work._labels = labels;
        work._caseIndex = caseIndex;
        work._holdDurationMs = 0U;
        return work;
    }

    OpenFloorMeasurementController::MainStage::ScheduledWork
        OpenFloorMeasurementController::MainStage::ScheduledWork::Hold(
            MainMeasurementPhase& phase,
            const std::uint16_t caseIndex,
            const MainRowLabel& labels,
            const std::uint16_t durationMs) noexcept
    {
        ScheduledWork work{};
        work._kind = Kind::Hold;
        work._phase = &phase;
        work._labels = labels;
        work._caseIndex = caseIndex;
        work._holdDurationMs = durationMs;
        return work;
    }

    bool OpenFloorMeasurementController::MainStage::ScheduledWork::IsHold() const noexcept
    {
        return _kind == Kind::Hold;
    }

    OpenFloorMeasurementController::MainMeasurementPhase&
        OpenFloorMeasurementController::MainStage::ScheduledWork::Phase() const noexcept
    {
        return *_phase;
    }

    std::uint16_t OpenFloorMeasurementController::MainStage::ScheduledWork::CaseIndex() const noexcept
    {
        return _caseIndex;
    }

    const OpenFloorMeasurementController::MainRowLabel&
        OpenFloorMeasurementController::MainStage::ScheduledWork::Labels() const noexcept
    {
        return _labels;
    }

    std::uint16_t OpenFloorMeasurementController::MainStage::ScheduledWork::HoldDurationMs() const noexcept
    {
        return _holdDurationMs;
    }

    void OpenFloorMeasurementController::MainStage::Reset() noexcept
    {
        _scheduledWorkCount = 0U;
        _nextWorkIndex = 0U;
        _logOpen = false;
        _completionPending = false;
        _activeWorkStarted = false;
        _activePhase = nullptr;
        _activeCaseIndex = 0U;
        _pendingRow.reset();

        ClearCompileState();
        _staticPhase.Reset();
        _launchPhase.Reset();
        _straightPhase.Reset();
        _yawPhase.Reset();
        _smoothPhase.Reset();
        _clockwiseLoopPhase.Reset();
        _counterClockwiseLoopPhase.Reset();
    }

    bool OpenFloorMeasurementController::MainStage::CompilePlan(OpenFloorMeasurementController& controller)
    {
        Reset();

        if (!CompilePhase(
                controller,
                _staticPhase,
                0U,
                static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs)))
        {
            return false;
        }
        if (!CompilePhase(
                controller,
                _launchPhase,
                static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs),
                0U))
        {
            return false;
        }
        if (!CompilePhase(
                controller,
                _straightPhase,
                static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs),
                0U))
        {
            return false;
        }
        if (!CompilePhase(
                controller,
                _yawPhase,
                static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs),
                0U))
        {
            return false;
        }
        if (!CompilePhase(
                controller,
                _smoothPhase,
                0U,
                static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs)))
        {
            return false;
        }
        if (!CompilePhase(
                controller,
                _clockwiseLoopPhase,
                0U,
                static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs)))
        {
            return false;
        }
        if (!CompilePhase(controller, _counterClockwiseLoopPhase, 0U, 0U))
        {
            return false;
        }

        return _scheduledWorkCount > 0U;
    }

    bool OpenFloorMeasurementController::MainStage::Begin(OpenFloorMeasurementController& controller)
    {
        _nextWorkIndex = 0U;
        _completionPending = (_scheduledWorkCount == 0U);
        _activeWorkStarted = false;
        _activePhase = nullptr;
        _activeCaseIndex = 0U;
        _pendingRow.reset();

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
        if (!FlushPending(controller, &services, "Open-floor measurement main log write failed"))
        {
            return StopControlVector();
        }

        const ScheduledWork* const work = ActiveWork();
        if (_completionPending || (work == nullptr))
        {
            services.RequestEndLoop();
            return StopControlVector();
        }
        if (CheckFault(controller, services))
        {
            return StopControlVector();
        }

        if (!_activeWorkStarted)
        {
            _activePhase = &work->Phase();
            _activeCaseIndex = work->CaseIndex();
            _activeLabels = work->Labels();
            _activeWorkStarted = true;

            if (work->IsHold())
            {
                controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
                controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                controller._driveService.StartHold(work->HoldDurationMs(), false);
            }
            else
            {
                work->Phase().BeginCase(controller, work->CaseIndex());
            }
        }

        bool done = false;
        LoopController::ControlVector control = StopControlVector();
        if (work->IsHold())
        {
            control = controller._driveService.GetNextControls(done);
        }
        else
        {
            control = work->Phase().TickCase(controller, state, services, done);
        }

        Runtime::OpenFloorMainRow row{};
        controller.PopulateMainRowFromState(_activeLabels, state, row);
        StageRow(row);

        if (done)
        {
            Advance();
            return StopControlVector();
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

    bool OpenFloorMeasurementController::MainStage::RegisterUnrepeatedCase(
        MainMeasurementPhase& phase,
        const std::uint16_t caseIndex,
        const OpenFloorPrimitiveId primitiveId,
        const OpenFloorSpeedBin speedBin)
    {
        if ((_compilingPhase != &phase) || _sequenceActive)
        {
            return false;
        }

        return AppendScheduledCase(phase, caseIndex, MainRowLabel(phase.PhaseId(), primitiveId, speedBin, 1U));
    }

    bool OpenFloorMeasurementController::MainStage::RegisterSequentialCase(
        MainMeasurementPhase& phase,
        const std::uint16_t caseIndex,
        const OpenFloorPrimitiveId primitiveId,
        const OpenFloorSpeedBin speedBin)
    {
        if ((_compilingPhase != &phase) || _sequenceActive)
        {
            return false;
        }

        return AppendScheduledCase(
            phase,
            caseIndex,
            MainRowLabel(phase.PhaseId(), primitiveId, speedBin, ++_compilingSequentialRepeatIndex));
    }

    bool OpenFloorMeasurementController::MainStage::BeginRepeatedSequence(
        const std::uint16_t repeatCount) noexcept
    {
        if ((_compilingPhase == nullptr) || _sequenceActive || (repeatCount == 0U))
        {
            return false;
        }

        _sequenceActive = true;
        _sequenceRepeatCount = repeatCount;
        _sequenceSize = 0U;
        return true;
    }

    bool OpenFloorMeasurementController::MainStage::RegisterRepeatedSequenceCase(
        MainMeasurementPhase& phase,
        const std::uint16_t caseIndex,
        const OpenFloorPrimitiveId primitiveId,
        const OpenFloorSpeedBin speedBin)
    {
        if ((_compilingPhase != &phase) || !_sequenceActive || (_sequenceSize >= _sequenceScratch.size()))
        {
            return false;
        }

        _sequenceScratch[_sequenceSize++] =
            ScheduledWork::PhaseCase(phase, caseIndex, MainRowLabel(phase.PhaseId(), primitiveId, speedBin, 0U));
        return true;
    }

    bool OpenFloorMeasurementController::MainStage::EndRepeatedSequence()
    {
        if (!_sequenceActive || (_compilingPhase == nullptr) || (_sequenceSize == 0U))
        {
            return false;
        }

        for (std::uint16_t repeatIndex = 1U; repeatIndex <= _sequenceRepeatCount; ++repeatIndex)
        {
            for (std::uint16_t entryIndex = 0U; entryIndex < _sequenceSize; ++entryIndex)
            {
                const ScheduledWork& prototype = _sequenceScratch[entryIndex];
                if (!AppendScheduledCase(
                        prototype.Phase(),
                        prototype.CaseIndex(),
                        prototype.Labels().WithRepeatIndex(repeatIndex)))
                {
                    return false;
                }
            }
        }

        _sequenceActive = false;
        _sequenceRepeatCount = 0U;
        _sequenceSize = 0U;
        return true;
    }

    bool OpenFloorMeasurementController::MainStage::CompilePhase(
        OpenFloorMeasurementController& controller,
        MainMeasurementPhase& phase,
        const std::uint16_t interCaseHoldMs,
        const std::uint16_t interPhaseHoldMs)
    {
        phase.Reset();
        _compilingPhase = &phase;
        _compilingInterCaseHoldMs = interCaseHoldMs;
        _compilingInterPhaseHoldMs = interPhaseHoldMs;
        _compilingSequentialRepeatIndex = 0U;
        _compilingPhaseHasCases = false;
        _lastCompiledPhase = nullptr;
        _lastCompiledCaseIndex = 0U;
        _sequenceActive = false;
        _sequenceRepeatCount = 0U;
        _sequenceSize = 0U;

        const bool compiled = phase.Compile(controller, *this);
        if (!compiled || _sequenceActive)
        {
            ClearCompileState();
            return false;
        }
        if (_compilingPhaseHasCases && (_compilingInterPhaseHoldMs > 0U))
        {
            if (!AppendHold(
                    *_lastCompiledPhase,
                    _lastCompiledCaseIndex,
                    _lastCompiledLabels,
                    _compilingInterPhaseHoldMs))
            {
                ClearCompileState();
                return false;
            }
        }

        ClearCompileState();
        return true;
    }

    void OpenFloorMeasurementController::MainStage::ClearCompileState() noexcept
    {
        _compilingPhase = nullptr;
        _compilingInterCaseHoldMs = 0U;
        _compilingInterPhaseHoldMs = 0U;
        _compilingSequentialRepeatIndex = 0U;
        _compilingPhaseHasCases = false;
        _lastCompiledPhase = nullptr;
        _lastCompiledCaseIndex = 0U;
        _lastCompiledLabels = MainRowLabel{};
        _sequenceActive = false;
        _sequenceRepeatCount = 0U;
        _sequenceSize = 0U;
    }

    bool OpenFloorMeasurementController::MainStage::AppendScheduledCase(
        MainMeasurementPhase& phase,
        const std::uint16_t caseIndex,
        const MainRowLabel& labels)
    {
        if (_scheduledWorkCount >= _scheduledWork.size())
        {
            return false;
        }

        _scheduledWork[_scheduledWorkCount++] = ScheduledWork::PhaseCase(phase, caseIndex, labels);
        _compilingPhaseHasCases = true;
        _lastCompiledPhase = &phase;
        _lastCompiledCaseIndex = caseIndex;
        _lastCompiledLabels = labels;

        if (_compilingInterCaseHoldMs > 0U)
        {
            return AppendHold(phase, caseIndex, labels, _compilingInterCaseHoldMs);
        }

        return true;
    }

    bool OpenFloorMeasurementController::MainStage::AppendHold(
        MainMeasurementPhase& phase,
        const std::uint16_t caseIndex,
        const MainRowLabel& labels,
        const std::uint16_t durationMs)
    {
        if (_scheduledWorkCount >= _scheduledWork.size())
        {
            return false;
        }

        _scheduledWork[_scheduledWorkCount++] = ScheduledWork::Hold(phase, caseIndex, labels, durationMs);
        return true;
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

        services.Fault("Estimator fault during open-floor main stage");
        return true;
    }

    void OpenFloorMeasurementController::MainStage::Advance() noexcept
    {
        ++_nextWorkIndex;
        _activeWorkStarted = false;
        _activePhase = nullptr;
        _activeCaseIndex = 0U;
        if (_nextWorkIndex >= _scheduledWorkCount)
        {
            _completionPending = true;
        }
    }

    const OpenFloorMeasurementController::MainStage::ScheduledWork*
        OpenFloorMeasurementController::MainStage::ActiveWork() const noexcept
    {
        return (_nextWorkIndex < _scheduledWorkCount) ? &_scheduledWork[_nextWorkIndex] : nullptr;
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
        row.phase_id = static_cast<std::uint32_t>(static_cast<std::uint8_t>(MeasurementPhaseId::Timing));
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
        const MainRowLabel& identity,
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
        row.phase_id = static_cast<std::uint8_t>(identity.PhaseId());
        row.primitive_id = static_cast<std::uint8_t>(identity.PrimitiveId());
        row.speed_bin = static_cast<std::uint8_t>(identity.SpeedBin());
        row.repeat_index = identity.RepeatIndex();
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
