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
        , _staticRegime(runtime)
        , _launchRegime(runtime)
        , _straightRegime(runtime)
        , _yawRegime(runtime)
        , _smoothRegime(runtime)
        , _clockwiseLoopRegime(runtime, MeasurementPhaseId::LoopClockwise, true)
        , _counterClockwiseLoopRegime(runtime, MeasurementPhaseId::LoopCounterClockwise, false)
        , _mainRegimes{
            &_staticRegime,
            &_launchRegime,
            &_straightRegime,
            &_yawRegime,
            &_smoothRegime,
            &_clockwiseLoopRegime,
            &_counterClockwiseLoopRegime,
        }
        , _mainStage(_mainRegimes.data(), _mainRegimes.size())
    {
    }

    OpenFloorMeasurementController::~OpenFloorMeasurementController() = default;

    OpenFloorMeasurementController::StaticMeasurementRegime::StaticMeasurementRegime(
        SharedRobotRuntime& runtime) noexcept
        : _runtime(runtime)
    {
    }

    const char* OpenFloorMeasurementController::StaticMeasurementRegime::Title() const noexcept
    {
        return "static";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::StaticMeasurementRegime::LogId() const noexcept
    {
        return MeasurementPhaseId::Static;
    }

    std::uint16_t
        OpenFloorMeasurementController::StaticMeasurementRegime::PrimitiveCount() const noexcept
    {
        return 1U;
    }

    std::uint8_t OpenFloorMeasurementController::StaticMeasurementRegime::SpeedCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::StaticMeasurementRegime::RepeatCount() const noexcept
    {
        return 1U;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::StaticMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex) const noexcept
    {
        (void)primitiveIndex;
        return MazeMap::MC_NONE;
    }

    float OpenFloorMeasurementController::StaticMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedIndex;
        return 0.0f;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StaticMeasurementRegime::Tick(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        (void)primitiveIndex;
        (void)speedIndex;

        auto& driveService = _runtime.DriveService();
        if (_needsStart)
        {
            const unsigned long totalHoldMs =
                static_cast<unsigned long>(DiagnosticConfig::kStaticHoldMs) +
                MazeMap::kOpenFloorInterPhaseHoldMs;
            driveService.SetLimits(BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartHold(totalHoldMs, false);
            _needsStart = false;
        }

        const LoopController::ControlVector control = driveService.GetNextControls(done);
        if (done)
        {
            _needsStart = true;
        }
        return control;
    }

    OpenFloorMeasurementController::LaunchMeasurementRegime::LaunchMeasurementRegime(
        SharedRobotRuntime& runtime) noexcept
        : _runtime(runtime)
    {
    }

    const char* OpenFloorMeasurementController::LaunchMeasurementRegime::Title() const noexcept
    {
        return "launch";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::LaunchMeasurementRegime::LogId() const noexcept
    {
        return MeasurementPhaseId::Launch;
    }

    std::uint16_t
        OpenFloorMeasurementController::LaunchMeasurementRegime::PrimitiveCount() const noexcept
    {
        return 1U;
    }

    std::uint8_t OpenFloorMeasurementController::LaunchMeasurementRegime::SpeedCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::LaunchMeasurementRegime::RepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(kLaunchPhaseSlotCount);
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::LaunchMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex) const noexcept
    {
        (void)primitiveIndex;
        return MazeMap::MC_NONE;
    }

    float OpenFloorMeasurementController::LaunchMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedIndex;
        return 0.0f;
    }

    LoopController::ControlVector OpenFloorMeasurementController::LaunchMeasurementRegime::Tick(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        (void)primitiveIndex;
        (void)speedIndex;

        auto& driveService = _runtime.DriveService();
        if (_needsStart)
        {
            const std::uint16_t slotsPerMagnitude =
                static_cast<std::uint16_t>(MazeMap::kOpenFloorLaunchRepeatsPerMagnitude * 2U);
            const std::uint16_t magnitudeIndex = _repeatIndex / slotsPerMagnitude;
            const std::uint16_t slotWithinMagnitude = _repeatIndex % slotsPerMagnitude;
            const float sign = ((slotWithinMagnitude % 2U) == 0U) ? 1.0f : -1.0f;
            const float magnitude = MazeMap::kOpenFloorLaunchDriveMagnitudes[magnitudeIndex];

            _activeLeftCommand = magnitude * sign;
            _activeRightCommand = magnitude * sign;
            _deadlineMs = millis() + static_cast<std::uint32_t>(MazeMap::kOpenFloorLaunchPulseMs);
            _holdActive = false;
            _needsStart = false;
        }

        if (!_holdActive)
        {
            if (static_cast<long>(_deadlineMs - millis()) > 0)
            {
                done = false;
                return LoopController::ControlVector::RawMotorPwm(
                    _activeLeftCommand,
                    _activeRightCommand);
            }

            driveService.SetLimits(BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            _holdActive = true;
        }

        const LoopController::ControlVector holdControl = driveService.GetNextControls(done);
        if (done)
        {
            _repeatIndex = static_cast<std::uint16_t>((_repeatIndex + 1U) % RepeatCount());
            _needsStart = true;
            _holdActive = false;
        }
        return holdControl;
    }

    OpenFloorMeasurementController::StraightMeasurementRegime::StraightMeasurementRegime(
        SharedRobotRuntime& runtime) noexcept
        : _runtime(runtime)
    {
    }

    const char* OpenFloorMeasurementController::StraightMeasurementRegime::Title() const noexcept
    {
        return "straight";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::StraightMeasurementRegime::LogId() const noexcept
    {
        return MeasurementPhaseId::Straight;
    }

    std::uint16_t
        OpenFloorMeasurementController::StraightMeasurementRegime::PrimitiveCount() const noexcept
    {
        return 1U;
    }

    std::uint8_t OpenFloorMeasurementController::StraightMeasurementRegime::SpeedCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorStraightSpeedBinsMps.size());
    }

    std::uint16_t OpenFloorMeasurementController::StraightMeasurementRegime::RepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(kStraightPhaseRepeatSlotCount);
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::StraightMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex) const noexcept
    {
        (void)primitiveIndex;
        return MazeMap::S4;
    }

    float OpenFloorMeasurementController::StraightMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        return
            (speedIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorStraightSpeedBinsMps.size())) ?
                MazeMap::kOpenFloorStraightSpeedBinsMps[speedIndex] :
                0.0f;
    }

    LoopController::ControlVector OpenFloorMeasurementController::StraightMeasurementRegime::Tick(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        (void)primitiveIndex;

        auto& driveService = _runtime.DriveService();
        if ((!_selectionValid) || (_activeSpeedIndex != speedIndex))
        {
            _selectionValid = true;
            _activeSpeedIndex = speedIndex;
            _repeatIndex = 0U;
            _needsStart = true;
            _holdActive = false;
        }

        if (_needsStart)
        {
            const float speedMagnitudeMps = MazeMap::kOpenFloorStraightSpeedBinsMps[_activeSpeedIndex];
            const float direction = ((_repeatIndex % 2U) == 0U) ? 1.0f : -1.0f;
            const float activeSpeedMps = direction * speedMagnitudeMps;

            driveService.SetLimits(
                BuildOpenFloorMeasurementLimits(
                    _runtime.SpeedVehicle(),
                    std::fabs(activeSpeedMps)));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartStraight(
                MazeMap::OpenFloorStrEquivalentDistanceMeters(4U),
                activeSpeedMps,
                0.0f);
            _holdActive = false;
            _needsStart = false;
        }

        if (_holdActive)
        {
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _repeatIndex = static_cast<std::uint16_t>((_repeatIndex + 1U) % RepeatCount());
                _needsStart = true;
                _holdActive = false;
            }
            return control;
        }

        if (driveService.IsEffectivelyComplete())
        {
            driveService.SetLimits(BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            _holdActive = true;
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _repeatIndex = static_cast<std::uint16_t>((_repeatIndex + 1U) % RepeatCount());
                _needsStart = true;
                _holdActive = false;
            }
            return control;
        }

        const LoopController::ControlVector control = driveService.GetNextControls(done);
        if (done)
        {
            done = false;
        }
        return control;
    }

    OpenFloorMeasurementController::YawMeasurementRegime::YawMeasurementRegime(
        SharedRobotRuntime& runtime) noexcept
        : _runtime(runtime)
    {
    }

    const char* OpenFloorMeasurementController::YawMeasurementRegime::Title() const noexcept
    {
        return "yaw";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::YawMeasurementRegime::LogId() const noexcept
    {
        return MeasurementPhaseId::Yaw;
    }

    std::uint16_t OpenFloorMeasurementController::YawMeasurementRegime::PrimitiveCount() const noexcept
    {
        return
            (MazeMap::kOpenFloorYawManeuverCodes.size() == MazeMap::kOpenFloorYawNominalAnglesRad.size()) ?
                static_cast<std::uint16_t>(MazeMap::kOpenFloorYawManeuverCodes.size()) :
                0U;
    }

    std::uint8_t OpenFloorMeasurementController::YawMeasurementRegime::SpeedCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorYawOmegaBinsRadps.size());
    }

    std::uint16_t OpenFloorMeasurementController::YawMeasurementRegime::RepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed);
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::YawMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex) const noexcept
    {
        return
            (primitiveIndex < PrimitiveCount()) ?
                MazeMap::kOpenFloorYawManeuverCodes[primitiveIndex] :
                MazeMap::MC_NONE;
    }

    float OpenFloorMeasurementController::YawMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        return
            (speedIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorYawOmegaBinsRadps.size())) ?
                MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex] :
                0.0f;
    }

    LoopController::ControlVector OpenFloorMeasurementController::YawMeasurementRegime::Tick(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        auto& driveService = _runtime.DriveService();
        if ((!_selectionValid) ||
            (_activePrimitiveIndex != primitiveIndex) ||
            (_activeSpeedIndex != speedIndex))
        {
            _selectionValid = true;
            _activePrimitiveIndex = primitiveIndex;
            _activeSpeedIndex = speedIndex;
            _needsStart = true;
            _holdActive = false;
        }

        if (_needsStart)
        {
            const float activeYawRad = MazeMap::kOpenFloorYawNominalAnglesRad[_activePrimitiveIndex];
            const float activeMaxOmegaRadps = MazeMap::kOpenFloorYawOmegaBinsRadps[_activeSpeedIndex];

            MotionLimits limits = BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f);
            limits.maxAngularSpeedRadps = activeMaxOmegaRadps;
            driveService.SetLimits(limits);
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartTurn(activeYawRad);
            _holdActive = false;
            _needsStart = false;
        }

        if (_holdActive)
        {
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _needsStart = true;
                _holdActive = false;
            }
            return control;
        }

        if (driveService.IsEffectivelyComplete())
        {
            driveService.SetLimits(BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            _holdActive = true;
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _needsStart = true;
                _holdActive = false;
            }
            return control;
        }

        const LoopController::ControlVector control = driveService.GetNextControls(done);
        if (done)
        {
            done = false;
        }
        return control;
    }

    OpenFloorMeasurementController::SmoothMeasurementRegime::SmoothMeasurementRegime(
        SharedRobotRuntime& runtime)
        : _runtime(runtime)
    {
        _primitiveCodes.fill(MazeMap::MC_NONE);
        _speedLimitsMps.fill(0.0f);
        _speedBinValues.fill(0.0f);

        float entryBoundarySpeedMps = 0.0f;
        std::uint16_t primitiveIndex = 0U;
        for (std::size_t speedIndex = 0U; speedIndex < MazeMap::kOpenFloorSmoothSpeedBinsMps.size(); ++speedIndex)
        {
            MazeMap::ManeuverQueue queue{};
            float exitBoundarySpeedMps = 0.0f;
            if (!BuildQueue(
                    _runtime.SpeedVehicle(),
                    static_cast<std::uint8_t>(speedIndex),
                    MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex],
                    entryBoundarySpeedMps,
                    queue,
                    exitBoundarySpeedMps) ||
                queue.empty() ||
                ((primitiveIndex + static_cast<std::uint16_t>(queue.size())) >
                    static_cast<std::uint16_t>(_maneuvers.size())))
            {
                _valid = false;
                break;
            }

            const bool isLastSpeedBin =
                (speedIndex + 1U) >= MazeMap::kOpenFloorSmoothSpeedBinsMps.size();
            const float speedBinValue = MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex];
            const std::uint16_t queueSize = static_cast<std::uint16_t>(queue.size());
            for (std::uint16_t queuePrimitiveIndex = 0U;
                 queuePrimitiveIndex < queueSize;
                 ++queuePrimitiveIndex)
            {
                const bool isClosingStraight =
                    isLastSpeedBin &&
                    (queueSize > static_cast<std::uint16_t>(kSmoothPhasePrimitiveCountPerSpeed)) &&
                    ((queuePrimitiveIndex + 1U) == queueSize);
                _primitiveCodes[primitiveIndex] =
                    (queuePrimitiveIndex == 0U) || isClosingStraight ?
                        kOpenFloorMeasurementSpeedChangeStraightCode :
                        kOpenFloorMeasurementSmoothCycleCodes[queuePrimitiveIndex - 1U];
                _maneuvers[primitiveIndex] = queue[queuePrimitiveIndex];
                _speedLimitsMps[primitiveIndex] = MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex];
                _speedBinValues[primitiveIndex] = speedBinValue;
                ++primitiveIndex;
            }

            entryBoundarySpeedMps = exitBoundarySpeedMps;
        }

        _primitiveCount = _valid ? primitiveIndex : 0U;
        if (_primitiveCount != static_cast<std::uint16_t>(_maneuvers.size()))
        {
            _valid = false;
            _primitiveCount = 0U;
        }
    }

    const char* OpenFloorMeasurementController::SmoothMeasurementRegime::Title() const noexcept
    {
        return "smooth";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::SmoothMeasurementRegime::LogId() const noexcept
    {
        return MeasurementPhaseId::Smooth;
    }

    std::uint16_t OpenFloorMeasurementController::SmoothMeasurementRegime::PrimitiveCount() const noexcept
    {
        return _valid ? _primitiveCount : 0U;
    }

    std::uint8_t OpenFloorMeasurementController::SmoothMeasurementRegime::SpeedCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::SmoothMeasurementRegime::RepeatCount() const noexcept
    {
        return 1U;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::SmoothMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex) const noexcept
    {
        return
            (primitiveIndex < PrimitiveCount()) ?
                _primitiveCodes[primitiveIndex] :
                MazeMap::MC_NONE;
    }

    float OpenFloorMeasurementController::SmoothMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)speedIndex;
        return
            (primitiveIndex < PrimitiveCount()) ?
                _speedBinValues[primitiveIndex] :
                0.0f;
    }

    LoopController::ControlVector OpenFloorMeasurementController::SmoothMeasurementRegime::Tick(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        (void)speedIndex;

        if (!_valid)
        {
            _runtime.FailActiveMode("Open-floor measurement smooth regime initialization failed");
            done = false;
            return StopControlVector();
        }

        auto& driveService = _runtime.DriveService();
        if ((!_selectionValid) || (_activePrimitiveIndex != primitiveIndex))
        {
            _selectionValid = true;
            _activePrimitiveIndex = primitiveIndex;
            _needsStart = true;
            _holdActive = false;
        }

        if (_needsStart)
        {
            driveService.SetLimits(
                BuildOpenFloorMeasurementLimits(
                    _runtime.SpeedVehicle(),
                    _speedLimitsMps[_activePrimitiveIndex]));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartManeuver(_maneuvers[_activePrimitiveIndex]);
            _activePostSlotHoldMs = IsLastPrimitive(_activePrimitiveIndex) ?
                static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs) :
                0U;
            _holdActive = false;
            _needsStart = false;
        }

        if (_holdActive)
        {
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _holdActive = false;
                _needsStart = true;
            }
            return control;
        }

        if ((_activePostSlotHoldMs > 0U) && driveService.IsEffectivelyComplete())
        {
            driveService.SetLimits(BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartHold(_activePostSlotHoldMs, false);
            _holdActive = true;
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _holdActive = false;
                _needsStart = true;
            }
            return control;
        }

        const LoopController::ControlVector control = driveService.GetNextControls(done);
        if (done)
        {
            if (_activePostSlotHoldMs > 0U)
            {
                done = false;
            }
            else
            {
                _needsStart = true;
            }
        }
        return control;
    }

    bool OpenFloorMeasurementController::SmoothMeasurementRegime::IsLastPrimitive(
        const std::uint16_t primitiveIndex) const noexcept
    {
        return (primitiveIndex + 1U) == PrimitiveCount();
    }

    bool OpenFloorMeasurementController::SmoothMeasurementRegime::BuildQueue(
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
            (static_cast<std::size_t>(speedIndex) + 1U) >= MazeMap::kOpenFloorSmoothSpeedBinsMps.size();
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

    OpenFloorMeasurementController::LoopMeasurementRegime::LoopMeasurementRegime(
        SharedRobotRuntime& runtime,
        const MeasurementPhaseId phaseId,
        const bool clockwise)
        : _runtime(runtime)
        , _phaseId(phaseId)
        , _clockwise(clockwise)
    {
        MazeMap::ManeuverQueue queue{};
        if (!BuildQueue(_runtime.SpeedVehicle(), queue) || (queue.size() != _maneuvers.size()))
        {
            _valid = false;
            return;
        }

        for (std::uint16_t primitiveIndex = 0U;
             primitiveIndex < static_cast<std::uint16_t>(_maneuvers.size());
             ++primitiveIndex)
        {
            _maneuvers[primitiveIndex] = queue[primitiveIndex];
        }
    }

    const char* OpenFloorMeasurementController::LoopMeasurementRegime::Title() const noexcept
    {
        return _clockwise ? "loop_clockwise" : "loop_counter_clockwise";
    }

    OpenFloorMeasurementController::MeasurementPhaseId
        OpenFloorMeasurementController::LoopMeasurementRegime::LogId() const noexcept
    {
        return _phaseId;
    }

    std::uint16_t OpenFloorMeasurementController::LoopMeasurementRegime::PrimitiveCount() const noexcept
    {
        return _valid ? static_cast<std::uint16_t>(kLoopPhasePrimitiveCount) : 0U;
    }

    std::uint8_t OpenFloorMeasurementController::LoopMeasurementRegime::SpeedCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::LoopMeasurementRegime::RepeatCount() const noexcept
    {
        return static_cast<std::uint16_t>(DiagnosticConfig::kLoopRepeats);
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::LoopMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex) const noexcept
    {
        const MazeMap::ManeuverCode turnCode = _clockwise ? MazeMap::IP90 : MazeMap::IP90_M;
        return
            (primitiveIndex < PrimitiveCount()) ?
                (((primitiveIndex % 2U) == 0U) ? kOpenFloorMeasurementLoopStraightCode : turnCode) :
                MazeMap::MC_NONE;
    }

    float OpenFloorMeasurementController::LoopMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedIndex;
        return MazeMap::kOpenFloorStraightSpeedBinsMps[0U];
    }

    LoopController::ControlVector OpenFloorMeasurementController::LoopMeasurementRegime::Tick(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        (void)speedIndex;

        if (!_valid)
        {
            _runtime.FailActiveMode("Open-floor measurement loop regime initialization failed");
            done = false;
            return StopControlVector();
        }

        auto& driveService = _runtime.DriveService();
        if ((!_selectionValid) || (_activePrimitiveIndex != primitiveIndex))
        {
            _selectionValid = true;
            _activePrimitiveIndex = primitiveIndex;
            _repeatIndex = 0U;
            _needsStart = true;
            _holdActive = false;
        }

        if (_needsStart)
        {
            driveService.SetLimits(
                BuildOpenFloorMeasurementLimits(
                    _runtime.SpeedVehicle(),
                    MazeMap::kOpenFloorStraightSpeedBinsMps[0U]));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartManeuver(_maneuvers[_activePrimitiveIndex]);
            _activePostSlotHoldMs =
                (_clockwise &&
                    ((_activePrimitiveIndex + 1U) == PrimitiveCount()) &&
                    ((_repeatIndex + 1U) == RepeatCount())) ?
                static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs) :
                0U;
            _holdActive = false;
            _needsStart = false;
        }

        if (_holdActive)
        {
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _repeatIndex = static_cast<std::uint16_t>((_repeatIndex + 1U) % RepeatCount());
                _holdActive = false;
                _needsStart = true;
            }
            return control;
        }

        if ((_activePostSlotHoldMs > 0U) && driveService.IsEffectivelyComplete())
        {
            driveService.SetLimits(BuildOpenFloorMeasurementLimits(_runtime.SpeedVehicle(), 0.0f));
            driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            driveService.StartHold(_activePostSlotHoldMs, false);
            _holdActive = true;
            const LoopController::ControlVector control = driveService.GetNextControls(done);
            if (done)
            {
                _repeatIndex = static_cast<std::uint16_t>((_repeatIndex + 1U) % RepeatCount());
                _holdActive = false;
                _needsStart = true;
            }
            return control;
        }

        const LoopController::ControlVector control = driveService.GetNextControls(done);
        if (done)
        {
            if (_activePostSlotHoldMs > 0U)
            {
                done = false;
            }
            else
            {
                _repeatIndex = static_cast<std::uint16_t>((_repeatIndex + 1U) % RepeatCount());
                _needsStart = true;
            }
        }
        return control;
    }

    bool OpenFloorMeasurementController::LoopMeasurementRegime::BuildQueue(
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
        return queue.size() == kLoopPhasePrimitiveCount;
    }

    void OpenFloorMeasurementController::TimingStage::Reset() noexcept
    {
        _tickIndex = 0U;
        _logOpen = false;
        _bufferedRow.reset();
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
        LoopController& loopController)
    {
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (!WriteBufferedRow(controller, "Open-floor measurement timing log write failed"))
        {
            controller._runtime.FailActiveMode("Open-floor measurement timing log write failed");
            return stopControl;
        }
        if (controller.SelectorRemoved())
        {
            controller._runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
            return stopControl;
        }
        if (controller._runtime.Estimator().HasFault())
        {
            controller._runtime.FailActiveMode("Estimator fault during timing capture");
            return stopControl;
        }

        Runtime::OpenFloorTimingRow row{};
        controller.PopulateTimingRowFromState(state, row);
        BufferRow(row);
        ++_tickIndex;
        if (CaptureComplete())
        {
            controller._sessionBoundaryAction = SessionBoundaryAction::TimingToMain;
            loopController.RequestEndSession(
                &OpenFloorMeasurementController::TimingToMainEndSessionThunk,
                &controller);
        }

        return stopControl;
    }

    void OpenFloorMeasurementController::TimingStage::CompleteTimingToMainSessionTransition(
        OpenFloorMeasurementController& controller,
        MainStage& mainStage,
        LoopController& loopController)
    {
        if (controller._sessionBoundaryAction != SessionBoundaryAction::TimingToMain)
        {
            controller._runtime.FailActiveMode(
                "Open-floor measurement end-session callback ran without a pending timing transition");
        }
        if (!WriteBufferedRow(controller, "Open-floor measurement timing log write failed during timing transition"))
        {
            controller._runtime.FailActiveMode(
                "Open-floor measurement timing log write failed during timing transition");
        }

        _logOpen = false;
        if (!mainStage.Begin(controller))
        {
            controller._runtime.FailActiveMode(
                "Open-floor measurement main log setup failed after timing capture");
        }

        controller._activeStageTick = &OpenFloorMeasurementController::MainStageTick;
        controller._sessionBoundaryAction = SessionBoundaryAction::None;
        loopController.StageNextSessionState(controller.BuildLoopOptions());
    }

    void OpenFloorMeasurementController::TimingStage::FinalizeCompletedRun(
        OpenFloorMeasurementController& controller) noexcept
    {
        if (_logOpen)
        {
            (void)WriteBufferedRow(controller, "Open-floor measurement timing log write failed");
            _logOpen = false;
        }
    }

    bool OpenFloorMeasurementController::TimingStage::WriteBufferedRow(
        OpenFloorMeasurementController& controller,
        const char* const failureReason)
    {
        if (!_bufferedRow.has_value())
        {
            return true;
        }

        Runtime::OpenFloorTimingRow& row = *_bufferedRow;
        ApplyControlTimingToTimingRow(controller._loopController.LastDiagnostics().controlTiming, row);
        if (!controller._runtime.LogUtilityDataRow(row))
        {
            (void)failureReason;
            return false;
        }

        _bufferedRow.reset();
        return true;
    }

    void OpenFloorMeasurementController::TimingStage::BufferRow(const Runtime::OpenFloorTimingRow& row)
    {
        _bufferedRow = row;
    }

    bool OpenFloorMeasurementController::TimingStage::CaptureComplete() const noexcept
    {
        return _tickIndex >= DiagnosticConfig::kTimingCaptureCycles;
    }

    OpenFloorMeasurementController::MainStage::MainStage(
        MainMeasurementRegime* const* regimes,
        const std::size_t regimeCount) noexcept
        : _regimes(regimes)
        , _regimeCount(regimeCount)
    {
    }

    void OpenFloorMeasurementController::MainStage::Reset() noexcept
    {
        ResetIndices();
        _logOpen = false;
        _completionPending = false;
        _bufferedRow.reset();
    }

    bool OpenFloorMeasurementController::MainStage::Begin(OpenFloorMeasurementController& controller)
    {
        Reset();
        if ((_regimes == nullptr) || (_regimeCount == 0U))
        {
            return false;
        }

        for (std::size_t regimeIndex = 0U; regimeIndex < _regimeCount; ++regimeIndex)
        {
            MainMeasurementRegime* const regime = _regimes[regimeIndex];
            if ((regime == nullptr) ||
                (regime->Title() == nullptr) ||
                (regime->PrimitiveCount() == 0U) ||
                (regime->SpeedCount() == 0U) ||
                (regime->RepeatCount() == 0U))
            {
                return false;
            }
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
        LoopController& loopController)
    {
        if (!WriteBufferedRow(controller, "Open-floor measurement main log write failed"))
        {
            controller._runtime.FailActiveMode("Open-floor measurement main log write failed");
            return StopControlVector();
        }

        if (_completionPending)
        {
            controller.FinalizeSuccessfulRun();
            loopController.HaltExecutionEndProgram();
            return StopControlVector();
        }
        if (CheckFault(controller))
        {
            return StopControlVector();
        }

        MainMeasurementRegime& regime = ActiveRegime();
        bool done = false;
        const LoopController::ControlVector control =
            regime.Tick(_activePrimitiveIndex, _activeSpeedIndex, done);

        Runtime::OpenFloorMainRow row{};
        controller.PopulateMainRowFromState(
            regime.LogId(),
            regime.PrimitiveCode(_activePrimitiveIndex),
            regime.SpeedBinValue(_activePrimitiveIndex, _activeSpeedIndex),
            static_cast<std::uint16_t>(_activeRepeatIndex + 1U),
            state,
            row);
        BufferRow(row);

        if (done)
        {
            _completionPending = !AdvanceIndices();
        }

        return control;
    }

    void OpenFloorMeasurementController::MainStage::FinalizeCompletedRun(
        OpenFloorMeasurementController& controller) noexcept
    {
        if (_logOpen)
        {
            (void)WriteBufferedRow(controller, "Open-floor measurement main log write failed");
            _logOpen = false;
        }
    }

    OpenFloorMeasurementController::MainMeasurementRegime&
        OpenFloorMeasurementController::MainStage::ActiveRegime() const noexcept
    {
        return *_regimes[_activeRegimeIndex];
    }

    void OpenFloorMeasurementController::MainStage::ResetIndices() noexcept
    {
        _activeRegimeIndex = 0U;
        _activePrimitiveIndex = 0U;
        _activeSpeedIndex = 0U;
        _activeRepeatIndex = 0U;
    }

    bool OpenFloorMeasurementController::MainStage::AdvanceIndices() noexcept
    {
        ++_activeRepeatIndex;
        if (_activeRepeatIndex < ActiveRegime().RepeatCount())
        {
            return true;
        }

        _activeRepeatIndex = 0U;
        ++_activeSpeedIndex;
        if (_activeSpeedIndex < ActiveRegime().SpeedCount())
        {
            return true;
        }

        _activeSpeedIndex = 0U;
        ++_activePrimitiveIndex;
        if (_activePrimitiveIndex < ActiveRegime().PrimitiveCount())
        {
            return true;
        }

        _activePrimitiveIndex = 0U;
        ++_activeRegimeIndex;
        return _activeRegimeIndex < _regimeCount;
    }

    bool OpenFloorMeasurementController::MainStage::WriteBufferedRow(
        OpenFloorMeasurementController& controller,
        const char* const failureReason)
    {
        if (!_bufferedRow.has_value())
        {
            return true;
        }

        Runtime::OpenFloorMainRow& row = *_bufferedRow;
        row.encoder_timestamp_us = controller._loopController.LastDiagnostics().controlTiming.encoderReadDoneUs;
        if (!controller._runtime.LogUtilityDataRow(row))
        {
            (void)failureReason;
            return false;
        }

        _bufferedRow.reset();
        return true;
    }

    void OpenFloorMeasurementController::MainStage::BufferRow(const Runtime::OpenFloorMainRow& row)
    {
        _bufferedRow = row;
    }

    bool OpenFloorMeasurementController::MainStage::CheckFault(
        OpenFloorMeasurementController& controller)
    {
        if (controller.SelectorRemoved())
        {
            controller._runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
            return true;
        }
        if (!controller._runtime.Estimator().HasFault())
        {
            return false;
        }

        controller._runtime.FailActiveMode("Estimator fault during open-floor main stage");
        return true;
    }

    void OpenFloorMeasurementController::SetupMode()
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(
                &OpenFloorMeasurementController::TeardownOnRuntimeFault,
                this,
                kOpenFloorMeasurementStableId))
        {
            _runtime.FailActiveMode("Open-floor measurement fault handler registration failed");
        }
        if (!SetupHardware())
        {
            _runtime.FailActiveMode("Open-floor measurement hardware setup failed");
        }

        (void)BootUtilityModeFramework::ResetStartupTrace("mode:open_floor_measurement");
        (void)_runtime.AppendTextLogLine("Open-floor measurement mode");
        (void)_runtime.AppendTextLogLine(
            "Open-floor battery: timing -> static -> launch -> straight -> yaw -> smooth -> loop cw -> loop ccw");

        if (!_drive.Begin())
        {
            _runtime.FailActiveMode("Open-floor measurement drive base init failed");
        }
        _drive.UseNominalWheelControlProfile();

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        if (!_startupCalibration.BringUp())
        {
            _runtime.FailActiveMode("Open-floor measurement startup bring-up failed");
        }
        SetMissionLevelFanEnabled(true);

        ConfigureSelectorMonitor();
        if (SelectorRemoved())
        {
            _runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
        }

        if (!_timingStage.Begin(*this))
        {
            _runtime.FailActiveMode("Open-floor measurement timing log setup failed");
        }

        _loopController.StageNextSessionState(BuildLoopOptions());
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

    void OpenFloorMeasurementController::TimingToMainEndSessionThunk(
        void* context,
        LoopController& loopController)
    {
        auto* const self = static_cast<OpenFloorMeasurementController*>(context);
        if (self == nullptr)
        {
            GetSharedRobotRuntime().FailActiveMode(
                "Open-floor measurement end-session callback context was null");
        }

        self->CompleteTimingToMainEndSession(loopController);
    }

    LoopController::ControlVector OpenFloorMeasurementController::RunTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        return (this->*_activeStageTick)(loopEndTimeUs, state, loopController);
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
        _sessionBoundaryAction = SessionBoundaryAction::None;
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
        const MeasurementPhaseId phaseId,
        const MazeMap::ManeuverCode primitiveCode,
        const float speedBinValue,
        const std::uint16_t repeatIndex,
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
        row.phase_id = static_cast<std::uint8_t>(phaseId);
        row.primitive_id = static_cast<std::uint8_t>(primitiveCode);
        row.speed_bin = speedBinValue;
        row.repeat_index = repeatIndex;
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

    void OpenFloorMeasurementController::CompleteTimingToMainEndSession(
        LoopController& loopController)
    {
        _timingStage.CompleteTimingToMainSessionTransition(*this, _mainStage, loopController);
    }

    void OpenFloorMeasurementController::FinalizeSuccessfulRun() noexcept
    {
        _mainStage.FinalizeCompletedRun(*this);
        _timingStage.FinalizeCompletedRun(*this);
        ReleaseSelectorMonitor();
        (void)_runtime.AppendTextLogLine("Open-floor measurement complete");
    }

    LoopController::ControlVector OpenFloorMeasurementController::TimingStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        return _timingStage.Tick(*this, state, loopController);
    }

    LoopController::ControlVector OpenFloorMeasurementController::MainStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        return _mainStage.Tick(*this, state, loopController);
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
