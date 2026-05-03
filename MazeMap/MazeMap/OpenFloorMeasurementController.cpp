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
    constexpr MazeMap::ManeuverCode kOpenFloorMeasurementLoopStraightCode = MazeMap::S2;
    constexpr std::array<MazeMap::ManeuverCode, 26U> kOpenFloorMeasurementSmoothCycleCodes = { {
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
    } };

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

    const char* OpenFloorMeasurementController::StaticMeasurementPhase::Name() const noexcept
    {
        return "static";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::StaticMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Static;
    }

    void OpenFloorMeasurementController::StaticMeasurementPhase::Reset() noexcept
    {
    }

    bool OpenFloorMeasurementController::StaticMeasurementPhase::Prepare(
        OpenFloorMeasurementController& controller)
    {
        (void)controller;
        Reset();
        return true;
    }

    std::uint16_t
        OpenFloorMeasurementController::StaticMeasurementPhase::MaxPrimitiveCount() const noexcept
    {
        return 1U;
    }

    std::uint8_t OpenFloorMeasurementController::StaticMeasurementPhase::MaxSpeedBinCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::StaticMeasurementPhase::MaxRepeatCount() const noexcept
    {
        return 1U;
    }

    bool OpenFloorMeasurementController::StaticMeasurementPhase::IsSlotActive(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex) const noexcept
    {
        return (primitiveIndex == 0U) && (speedBinIndex == 0U) && (repeatIndex == 0U);
    }

    OpenFloorMeasurementController::MainMeasurementPhase::SlotIdentity
        OpenFloorMeasurementController::StaticMeasurementPhase::DescribeSlot(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedBinIndex;
        (void)repeatIndex;
        return SlotIdentity(MazeMap::MC_NONE, MazeMap::kOpenFloorSpeedBinLogIdNone);
    }

    void OpenFloorMeasurementController::StaticMeasurementPhase::BeginSlot(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex)
    {
        (void)primitiveIndex;
        (void)speedBinIndex;
        (void)repeatIndex;

        const unsigned long totalHoldMs =
            static_cast<unsigned long>(DiagnosticConfig::kStaticHoldMs) +
            MazeMap::kOpenFloorInterPhaseHoldMs;
        controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartHold(totalHoldMs, false);
    }

    LoopController::ControlVector OpenFloorMeasurementController::StaticMeasurementPhase::TickActiveSlot(
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

    const char* OpenFloorMeasurementController::LaunchMeasurementPhase::Name() const noexcept
    {
        return "launch";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::LaunchMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Launch;
    }

    void OpenFloorMeasurementController::LaunchMeasurementPhase::Reset() noexcept
    {
        _activeLeftCommand = 0.0f;
        _activeRightCommand = 0.0f;
        _deadlineMs = 0U;
        _executionState = PhaseExecutionState::Motion;
    }

    bool OpenFloorMeasurementController::LaunchMeasurementPhase::Prepare(
        OpenFloorMeasurementController& controller)
    {
        (void)controller;
        Reset();
        return true;
    }

    std::uint16_t
        OpenFloorMeasurementController::LaunchMeasurementPhase::MaxPrimitiveCount() const noexcept
    {
        return 1U;
    }

    std::uint8_t OpenFloorMeasurementController::LaunchMeasurementPhase::MaxSpeedBinCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::LaunchMeasurementPhase::MaxRepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(kLaunchPhaseSlotCount);
    }

    bool OpenFloorMeasurementController::LaunchMeasurementPhase::IsSlotActive(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex) const noexcept
    {
        return
            (primitiveIndex == 0U) &&
            (speedBinIndex == 0U) &&
            (repeatIndex < static_cast<std::uint16_t>(kLaunchPhaseSlotCount));
    }

    OpenFloorMeasurementController::MainMeasurementPhase::SlotIdentity
        OpenFloorMeasurementController::LaunchMeasurementPhase::DescribeSlot(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedBinIndex;
        (void)repeatIndex;
        return SlotIdentity(MazeMap::MC_NONE, MazeMap::kOpenFloorSpeedBinLogIdNone);
    }

    void OpenFloorMeasurementController::LaunchMeasurementPhase::BeginSlot(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex)
    {
        (void)controller;
        (void)primitiveIndex;
        (void)speedBinIndex;

        const std::uint16_t slotsPerMagnitude =
            static_cast<std::uint16_t>(MazeMap::kOpenFloorLaunchRepeatsPerMagnitude * 2U);
        const std::uint16_t magnitudeIndex = repeatIndex / slotsPerMagnitude;
        const std::uint16_t slotWithinMagnitude = repeatIndex % slotsPerMagnitude;
        const float sign = ((slotWithinMagnitude % 2U) == 0U) ? 1.0f : -1.0f;
        const float magnitude = MazeMap::kOpenFloorLaunchDriveMagnitudes[magnitudeIndex];

        _activeLeftCommand = magnitude * sign;
        _activeRightCommand = magnitude * sign;
        _deadlineMs = millis() + static_cast<std::uint32_t>(MazeMap::kOpenFloorLaunchPulseMs);
        _executionState = PhaseExecutionState::Motion;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LaunchMeasurementPhase::TickActiveSlot(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        switch (_executionState)
        {
        case PhaseExecutionState::Motion:
            if (static_cast<long>(_deadlineMs - millis()) > 0)
            {
                done = false;
                return LoopController::ControlVector::RawMotorPwm(_activeLeftCommand, _activeRightCommand);
            }

            _executionState = PhaseExecutionState::PendingHold;
            done = false;
            return StopControlVector();

        case PhaseExecutionState::PendingHold:
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            _executionState = PhaseExecutionState::Hold;
            done = false;
            return StopControlVector();

        case PhaseExecutionState::Hold:
        default:
            const LoopController::ControlVector holdControl = controller._driveService.GetNextControls(done);
            return done ? StopControlVector() : holdControl;
        }
    }

    const char* OpenFloorMeasurementController::StraightMeasurementPhase::Name() const noexcept
    {
        return "straight";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::StraightMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Straight;
    }

    void OpenFloorMeasurementController::StraightMeasurementPhase::Reset() noexcept
    {
        _activeSpeedMps = 0.0f;
        _executionState = PhaseExecutionState::Motion;
    }

    bool OpenFloorMeasurementController::StraightMeasurementPhase::Prepare(
        OpenFloorMeasurementController& controller)
    {
        (void)controller;
        Reset();
        return true;
    }

    std::uint16_t
        OpenFloorMeasurementController::StraightMeasurementPhase::MaxPrimitiveCount() const noexcept
    {
        return 1U;
    }

    std::uint8_t
        OpenFloorMeasurementController::StraightMeasurementPhase::MaxSpeedBinCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorStraightSpeedBinsMps.size());
    }

    std::uint16_t
        OpenFloorMeasurementController::StraightMeasurementPhase::MaxRepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(kStraightPhaseRepeatSlotCount);
    }

    bool OpenFloorMeasurementController::StraightMeasurementPhase::IsSlotActive(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex) const noexcept
    {
        return
            (primitiveIndex == 0U) &&
            (speedBinIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorStraightSpeedBinsMps.size())) &&
            (repeatIndex < static_cast<std::uint16_t>(kStraightPhaseRepeatSlotCount));
    }

    OpenFloorMeasurementController::MainMeasurementPhase::SlotIdentity
        OpenFloorMeasurementController::StraightMeasurementPhase::DescribeSlot(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)repeatIndex;
        return SlotIdentity(MazeMap::S4, OpenFloorMeasurementController::SpeedBinForIndex(speedBinIndex));
    }

    void OpenFloorMeasurementController::StraightMeasurementPhase::BeginSlot(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex)
    {
        (void)primitiveIndex;

        const float speedMagnitudeMps = MazeMap::kOpenFloorStraightSpeedBinsMps[speedBinIndex];
        const float direction = ((repeatIndex % 2U) == 0U) ? 1.0f : -1.0f;
        _activeSpeedMps = direction * speedMagnitudeMps;

        controller._driveService.SetLimits(
            BuildOpenFloorMeasurementLimits(controller._vehicle, std::fabs(_activeSpeedMps)));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartStraight(
            MazeMap::OpenFloorStrEquivalentDistanceMeters(4U),
            _activeSpeedMps,
            0.0f);
        _executionState = PhaseExecutionState::Motion;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StraightMeasurementPhase::TickActiveSlot(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        switch (_executionState)
        {
        case PhaseExecutionState::Motion:
        {
            const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _executionState = PhaseExecutionState::PendingHold;
            done = false;
            return StopControlVector();
        }

        case PhaseExecutionState::PendingHold:
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            _executionState = PhaseExecutionState::Hold;
            done = false;
            return StopControlVector();

        case PhaseExecutionState::Hold:
        default:
            const LoopController::ControlVector holdControl = controller._driveService.GetNextControls(done);
            return done ? StopControlVector() : holdControl;
        }
    }

    const char* OpenFloorMeasurementController::YawMeasurementPhase::Name() const noexcept
    {
        return "yaw";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::YawMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Yaw;
    }

    void OpenFloorMeasurementController::YawMeasurementPhase::Reset() noexcept
    {
        _activeYawRad = 0.0f;
        _activeMaxOmegaRadps = 0.0f;
        _executionState = PhaseExecutionState::Motion;
    }

    bool OpenFloorMeasurementController::YawMeasurementPhase::Prepare(
        OpenFloorMeasurementController& controller)
    {
        (void)controller;
        Reset();
        return MazeMap::kOpenFloorYawManeuverCodes.size() == MazeMap::kOpenFloorYawNominalAnglesRad.size();
    }

    std::uint16_t OpenFloorMeasurementController::YawMeasurementPhase::MaxPrimitiveCount() const noexcept
    {
        return static_cast<std::uint16_t>(MazeMap::kOpenFloorYawManeuverCodes.size());
    }

    std::uint8_t OpenFloorMeasurementController::YawMeasurementPhase::MaxSpeedBinCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorYawOmegaBinsRadps.size());
    }

    std::uint16_t OpenFloorMeasurementController::YawMeasurementPhase::MaxRepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed);
    }

    bool OpenFloorMeasurementController::YawMeasurementPhase::IsSlotActive(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex) const noexcept
    {
        return
            (primitiveIndex < static_cast<std::uint16_t>(MazeMap::kOpenFloorYawManeuverCodes.size())) &&
            (speedBinIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorYawOmegaBinsRadps.size())) &&
            (repeatIndex < static_cast<std::uint16_t>(DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed));
    }

    OpenFloorMeasurementController::MainMeasurementPhase::SlotIdentity
        OpenFloorMeasurementController::YawMeasurementPhase::DescribeSlot(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex) const noexcept
    {
        (void)repeatIndex;
        return SlotIdentity(
            MazeMap::kOpenFloorYawManeuverCodes[primitiveIndex],
            OpenFloorMeasurementController::SpeedBinForIndex(speedBinIndex));
    }

    void OpenFloorMeasurementController::YawMeasurementPhase::BeginSlot(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex)
    {
        (void)repeatIndex;

        _activeYawRad = MazeMap::kOpenFloorYawNominalAnglesRad[primitiveIndex];
        _activeMaxOmegaRadps = MazeMap::kOpenFloorYawOmegaBinsRadps[speedBinIndex];

        MotionLimits limits = BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f);
        limits.maxAngularSpeedRadps = _activeMaxOmegaRadps;
        controller._driveService.SetLimits(limits);
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartTurn(_activeYawRad);
        _executionState = PhaseExecutionState::Motion;
    }

    LoopController::ControlVector OpenFloorMeasurementController::YawMeasurementPhase::TickActiveSlot(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        switch (_executionState)
        {
        case PhaseExecutionState::Motion:
        {
            const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            _executionState = PhaseExecutionState::PendingHold;
            done = false;
            return StopControlVector();
        }

        case PhaseExecutionState::PendingHold:
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            _executionState = PhaseExecutionState::Hold;
            done = false;
            return StopControlVector();

        case PhaseExecutionState::Hold:
        default:
            const LoopController::ControlVector holdControl = controller._driveService.GetNextControls(done);
            return done ? StopControlVector() : holdControl;
        }
    }

    const char* OpenFloorMeasurementController::SmoothMeasurementPhase::Name() const noexcept
    {
        return "smooth";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::SmoothMeasurementPhase::PhaseId() const noexcept
    {
        return MeasurementPhaseId::Smooth;
    }

    void OpenFloorMeasurementController::SmoothMeasurementPhase::Reset() noexcept
    {
        _speedLimitsMps.fill(0.0f);
        _primitiveCounts.fill(0U);
        _activePostSlotHoldMs = 0U;
        _executionState = PhaseExecutionState::Motion;
    }

    bool OpenFloorMeasurementController::SmoothMeasurementPhase::Prepare(
        OpenFloorMeasurementController& controller)
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
            if (queue.empty() || (queue.size() > kSmoothPhaseMaxPrimitiveCount))
            {
                return false;
            }

            _primitiveCounts[speedIndex] = static_cast<std::uint8_t>(queue.size());
            for (std::uint16_t primitiveIndex = 0U;
                 primitiveIndex < static_cast<std::uint16_t>(queue.size());
                 ++primitiveIndex)
            {
                const std::size_t offset =
                    SlotOffset(static_cast<std::uint8_t>(speedIndex), primitiveIndex);
                _maneuvers[offset] = queue[primitiveIndex];
                _speedLimitsMps[offset] = MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex];
            }

            entryBoundarySpeedMps = exitBoundarySpeedMps;
        }

        return true;
    }

    std::uint16_t
        OpenFloorMeasurementController::SmoothMeasurementPhase::MaxPrimitiveCount() const noexcept
    {
        return static_cast<std::uint16_t>(kSmoothPhaseMaxPrimitiveCount);
    }

    std::uint8_t OpenFloorMeasurementController::SmoothMeasurementPhase::MaxSpeedBinCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorSmoothSpeedBinsMps.size());
    }

    std::uint16_t OpenFloorMeasurementController::SmoothMeasurementPhase::MaxRepeatCount() const noexcept
    {
        return 1U;
    }

    bool OpenFloorMeasurementController::SmoothMeasurementPhase::IsSlotActive(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex) const noexcept
    {
        return
            (speedBinIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorSmoothSpeedBinsMps.size())) &&
            (repeatIndex == 0U) &&
            (primitiveIndex < _primitiveCounts[speedBinIndex]);
    }

    OpenFloorMeasurementController::MainMeasurementPhase::SlotIdentity
        OpenFloorMeasurementController::SmoothMeasurementPhase::DescribeSlot(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex) const noexcept
    {
        (void)repeatIndex;

        if (primitiveIndex == 0U)
        {
            return SlotIdentity(
                kOpenFloorMeasurementSpeedChangeStraightCode,
                OpenFloorMeasurementController::SpeedBinForIndex(speedBinIndex));
        }
        if (IsLastActiveSlot(primitiveIndex, speedBinIndex) &&
            (_primitiveCounts[speedBinIndex] > static_cast<std::uint8_t>(kSmoothPhaseCyclePrimitiveCount + 1U)))
        {
            return SlotIdentity(
                kOpenFloorMeasurementSpeedChangeStraightCode,
                OpenFloorMeasurementController::SpeedBinForIndex(speedBinIndex));
        }

        return SlotIdentity(
            kOpenFloorMeasurementSmoothCycleCodes[primitiveIndex - 1U],
            OpenFloorMeasurementController::SpeedBinForIndex(speedBinIndex));
    }

    void OpenFloorMeasurementController::SmoothMeasurementPhase::BeginSlot(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex)
    {
        (void)repeatIndex;

        const std::size_t offset = SlotOffset(speedBinIndex, primitiveIndex);
        controller._driveService.SetLimits(
            BuildOpenFloorMeasurementLimits(controller._vehicle, _speedLimitsMps[offset]));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartManeuver(_maneuvers[offset]);
        _activePostSlotHoldMs = IsLastActiveSlot(primitiveIndex, speedBinIndex) ?
            static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs) :
            0U;
        _executionState = PhaseExecutionState::Motion;
    }

    LoopController::ControlVector OpenFloorMeasurementController::SmoothMeasurementPhase::TickActiveSlot(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        switch (_executionState)
        {
        case PhaseExecutionState::Motion:
        {
            const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }
            if (_activePostSlotHoldMs == 0U)
            {
                return StopControlVector();
            }

            _executionState = PhaseExecutionState::PendingHold;
            done = false;
            return StopControlVector();
        }

        case PhaseExecutionState::PendingHold:
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(_activePostSlotHoldMs, false);
            _executionState = PhaseExecutionState::Hold;
            done = false;
            return StopControlVector();

        case PhaseExecutionState::Hold:
        default:
            const LoopController::ControlVector holdControl = controller._driveService.GetNextControls(done);
            return done ? StopControlVector() : holdControl;
        }
    }

    bool OpenFloorMeasurementController::SmoothMeasurementPhase::IsLastActiveSlot(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex) const noexcept
    {
        return
            ((static_cast<std::size_t>(speedBinIndex) + 1U) == MazeMap::kOpenFloorSmoothSpeedBinsMps.size()) &&
            ((primitiveIndex + 1U) == _primitiveCounts[speedBinIndex]);
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

    const char* OpenFloorMeasurementController::LoopMeasurementPhase::Name() const noexcept
    {
        return _clockwise ? "loop_clockwise" : "loop_counter_clockwise";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::LoopMeasurementPhase::PhaseId() const noexcept
    {
        return _phaseId;
    }

    void OpenFloorMeasurementController::LoopMeasurementPhase::Reset() noexcept
    {
        _activePostSlotHoldMs = 0U;
        _executionState = PhaseExecutionState::Motion;
    }

    bool OpenFloorMeasurementController::LoopMeasurementPhase::Prepare(
        OpenFloorMeasurementController& controller)
    {
        Reset();

        MazeMap::ManeuverQueue queue{};
        if (!BuildQueue(controller._vehicle, queue) || (queue.size() != _maneuvers.size()))
        {
            return false;
        }

        for (std::uint16_t primitiveIndex = 0U;
             primitiveIndex < static_cast<std::uint16_t>(_maneuvers.size());
             ++primitiveIndex)
        {
            _maneuvers[primitiveIndex] = queue[primitiveIndex];
        }

        return true;
    }

    std::uint16_t OpenFloorMeasurementController::LoopMeasurementPhase::MaxPrimitiveCount() const noexcept
    {
        return static_cast<std::uint16_t>(kLoopPhasePrimitiveCount);
    }

    std::uint8_t OpenFloorMeasurementController::LoopMeasurementPhase::MaxSpeedBinCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::LoopMeasurementPhase::MaxRepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(DiagnosticConfig::kLoopRepeats);
    }

    bool OpenFloorMeasurementController::LoopMeasurementPhase::IsSlotActive(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex) const noexcept
    {
        return
            (primitiveIndex < static_cast<std::uint16_t>(kLoopPhasePrimitiveCount)) &&
            (speedBinIndex == 0U) &&
            (repeatIndex < static_cast<std::uint16_t>(DiagnosticConfig::kLoopRepeats));
    }

    OpenFloorMeasurementController::MainMeasurementPhase::SlotIdentity
        OpenFloorMeasurementController::LoopMeasurementPhase::DescribeSlot(
            const std::uint16_t primitiveIndex,
            const std::uint8_t speedBinIndex,
            const std::uint16_t repeatIndex) const noexcept
    {
        (void)speedBinIndex;
        (void)repeatIndex;

        const MazeMap::ManeuverCode turnCode = _clockwise ? MazeMap::IP90 : MazeMap::IP90_M;
        return SlotIdentity(
            ((primitiveIndex % 2U) == 0U) ? kOpenFloorMeasurementLoopStraightCode : turnCode,
            MazeMap::kOpenFloorSpeedBinLogIdLow);
    }

    void OpenFloorMeasurementController::LoopMeasurementPhase::BeginSlot(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedBinIndex,
        const std::uint16_t repeatIndex)
    {
        (void)speedBinIndex;

        controller._driveService.SetLimits(
            BuildOpenFloorMeasurementLimits(
                controller._vehicle,
                MazeMap::kOpenFloorStraightSpeedBinsMps[0U]));
        controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        controller._driveService.StartManeuver(_maneuvers[primitiveIndex]);
        _activePostSlotHoldMs =
            (_clockwise &&
                ((primitiveIndex + 1U) == static_cast<std::uint16_t>(kLoopPhasePrimitiveCount)) &&
                ((repeatIndex + 1U) == static_cast<std::uint16_t>(DiagnosticConfig::kLoopRepeats))) ?
            static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs) :
            0U;
        _executionState = PhaseExecutionState::Motion;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopMeasurementPhase::TickActiveSlot(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        bool& done)
    {
        (void)state;
        (void)services;

        switch (_executionState)
        {
        case PhaseExecutionState::Motion:
        {
            const LoopController::ControlVector control = controller._driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }
            if (_activePostSlotHoldMs == 0U)
            {
                return StopControlVector();
            }

            _executionState = PhaseExecutionState::PendingHold;
            done = false;
            return StopControlVector();
        }

        case PhaseExecutionState::PendingHold:
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(_activePostSlotHoldMs, false);
            _executionState = PhaseExecutionState::Hold;
            done = false;
            return StopControlVector();

        case PhaseExecutionState::Hold:
        default:
            const LoopController::ControlVector holdControl = controller._driveService.GetNextControls(done);
            return done ? StopControlVector() : holdControl;
        }
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

    void OpenFloorMeasurementController::MainStage::Reset() noexcept
    {
        ResetCursor();
        _logOpen = false;
        _completionPending = false;
        _slotStarted = false;
        _activeLabels = MainRowLabel{};
        _pendingRow.reset();

        for (MainMeasurementPhase* const phase : _phases)
        {
            if (phase != nullptr)
            {
                phase->Reset();
            }
        }
    }

    bool OpenFloorMeasurementController::MainStage::PreparePhases(
        OpenFloorMeasurementController& controller)
    {
        Reset();

        for (MainMeasurementPhase* const phase : _phases)
        {
            if ((phase == nullptr) ||
                !phase->Prepare(controller) ||
                (phase->MaxPrimitiveCount() == 0U) ||
                (phase->MaxSpeedBinCount() == 0U) ||
                (phase->MaxRepeatCount() == 0U))
            {
                return false;
            }
        }

        return true;
    }

    bool OpenFloorMeasurementController::MainStage::Begin(OpenFloorMeasurementController& controller)
    {
        _completionPending = false;
        _slotStarted = false;
        _activeLabels = MainRowLabel{};
        _pendingRow.reset();
        if (!MoveToFirstActiveSlot())
        {
            _completionPending = true;
            return false;
        }

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

        if (_completionPending)
        {
            services.RequestEndLoop();
            return StopControlVector();
        }
        if (CheckFault(controller, services))
        {
            return StopControlVector();
        }

        if (!_slotStarted)
        {
            _activeLabels = BuildActiveLabels();
            ActivePhase().BeginSlot(
                controller,
                _activePrimitiveIndex,
                _activeSpeedBinIndex,
                _activeRepeatIndex);
            _slotStarted = true;
        }

        bool done = false;
        const LoopController::ControlVector control =
            ActivePhase().TickActiveSlot(controller, state, services, done);

        Runtime::OpenFloorMainRow row{};
        controller.PopulateMainRowFromState(_activeLabels, state, row);
        StageRow(row);

        if (done)
        {
            _slotStarted = false;
            _completionPending = !MoveToNextActiveSlot();
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

    OpenFloorMeasurementController::MainMeasurementPhase&
        OpenFloorMeasurementController::MainStage::ActivePhase() const noexcept
    {
        return *_phases[_activePhaseIndex];
    }

    void OpenFloorMeasurementController::MainStage::ResetCursor() noexcept
    {
        _activePhaseIndex = 0U;
        _activePrimitiveIndex = 0U;
        _activeSpeedBinIndex = 0U;
        _activeRepeatIndex = 0U;
    }

    bool OpenFloorMeasurementController::MainStage::MoveToFirstActiveSlot() noexcept
    {
        ResetCursor();
        return SeekActiveSlotFromCurrent();
    }

    bool OpenFloorMeasurementController::MainStage::MoveToNextActiveSlot() noexcept
    {
        return AdvanceCursorOneStep() && SeekActiveSlotFromCurrent();
    }

    bool OpenFloorMeasurementController::MainStage::SeekActiveSlotFromCurrent() noexcept
    {
        while (_activePhaseIndex < _phases.size())
        {
            if (ActivePhase().IsSlotActive(_activePrimitiveIndex, _activeSpeedBinIndex, _activeRepeatIndex))
            {
                return true;
            }
            if (!AdvanceCursorOneStep())
            {
                return false;
            }
        }

        return false;
    }

    bool OpenFloorMeasurementController::MainStage::AdvanceCursorOneStep() noexcept
    {
        while (_activePhaseIndex < _phases.size())
        {
            MainMeasurementPhase& phase = ActivePhase();

            ++_activeRepeatIndex;
            if (_activeRepeatIndex < phase.MaxRepeatCount())
            {
                return true;
            }

            _activeRepeatIndex = 0U;
            ++_activeSpeedBinIndex;
            if (_activeSpeedBinIndex < phase.MaxSpeedBinCount())
            {
                return true;
            }

            _activeSpeedBinIndex = 0U;
            ++_activePrimitiveIndex;
            if (_activePrimitiveIndex < phase.MaxPrimitiveCount())
            {
                return true;
            }

            _activePrimitiveIndex = 0U;
            ++_activePhaseIndex;
            if (_activePhaseIndex < _phases.size())
            {
                return true;
            }
        }

        return false;
    }

    OpenFloorMeasurementController::MainRowLabel
        OpenFloorMeasurementController::MainStage::BuildActiveLabels() const noexcept
    {
        const MainMeasurementPhase::SlotIdentity identity =
            ActivePhase().DescribeSlot(_activePrimitiveIndex, _activeSpeedBinIndex, _activeRepeatIndex);
        return MainRowLabel(
            ActivePhase().PhaseId(),
            identity.PrimitiveCode(),
            identity.SpeedBinLogId(),
            static_cast<std::uint16_t>(_activeRepeatIndex + 1U));
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

        if (!_mainStage.PreparePhases(*this))
        {
            return _runtime.FailActiveMode("Open-floor measurement phase preparation failed");
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
        row.primitive_id = static_cast<std::uint8_t>(identity.PrimitiveCode());
        row.speed_bin = identity.SpeedBinLogId();
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

    std::uint8_t OpenFloorMeasurementController::SpeedBinForIndex(const std::size_t speedIndex) noexcept
    {
        return MazeMap::OpenFloorSpeedBinLogIdForIndex(speedIndex);
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
