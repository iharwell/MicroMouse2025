#include "pch.h"
#include "OpenFloorMeasurementController.h"

#include "BootFramework.h"
#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "DriveBase.h"
#include "DriveTelemetry.h"
#include "ManeuverQueue.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SharedRobotRuntime.h"
#include "StartupCalibration.h"
#include "Vehicle.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace MazeMap
{
    using MazeMap::App::Internal::CommandVector;
    using MazeMap::App::Internal::Drive;
    using MazeMap::App::Internal::LoopController;
    using MazeMap::App::Internal::OpenFloorMeasurementController;
    using MazeMap::App::Internal::Runtime::OpenFloorMainRow;
    using MazeMap::App::Internal::Runtime::OpenFloorTimingRow;

    constexpr const char* kOpenFloorMeasurementSelectorRemovedReason =
        "Open-floor measurement selector jumper removed";
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
        limits.SetMaxSpeedMps(maxSpeedMps);
        limits.SetAccelMps2(DiagnosticConfig::kStraightAccelMps2);
        limits.SetDecelMps2(DiagnosticConfig::kStraightDecelMps2);
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxYawRate());
        limits.SetAngularAccelRadps2(vehicle.GetMaxYawAccel());
        return limits;
    }

    MotionLimits BuildOpenFloorMeasurementModeLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        return BuildOpenFloorMeasurementLimits(vehicle, std::numeric_limits<float>::infinity());
    }

    void ResetOpenFloorModeLimits(
        Drive& driveService,
        const MazeMap::Vehicle& vehicle) noexcept
    {
        driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        driveService.SetLimits(BuildOpenFloorMeasurementModeLimits(vehicle));
    }

    CommandVector StopControlVector() noexcept
    {
        return CommandVector(0.0f, 0.0f);
    }

    bool WriteOpenFloorV62Metadata(MazeMap::App::Internal::SharedRobotRuntime& runtime)
    {
        return
            runtime.WriteUtilityDataLogMetadata("ukfver", "v6.2") &&
            runtime.WriteUtilityDataLogMetadata("ukfset", "splx") &&
            runtime.Plant().WriteOpenFloorPlantMetadata(
                [&runtime](const char* key, float value, std::uint8_t precision) -> bool
                {
                    return runtime.WriteUtilityDataLogMetadataFloat(key, value, precision);
                });
    }

    void ApplyLoopTimingToTimingRow(
        const LoopController& loopController,
        OpenFloorTimingRow& row) noexcept
    {
        row.mono_time_us = loopController.LastTimingTickStartUs();
        row.control_tick_sequence = loopController.LastTimingSequence();
        row.dt_us = loopController.LastTimingDtUs();
        row.control_start_us = loopController.LastTimingTickStartUs();
        row.control_end_us = loopController.LastTimingTickFinalizeUs();
        row.pwm_latch_us = loopController.LastTimingCommandAppliedUs();
        row.encoder_latch_us = loopController.LastTimingEncoderLatchUs();
        row.encoder_read_done_us = loopController.LastTimingEncoderReadDoneUs();
        row.estimator_predict_start_us = loopController.LastTimingEstimatorPredictStartUs();
        row.estimator_predict_end_us = loopController.LastTimingEstimatorPredictEndUs();
        row.estimator_predict_duration_us = loopController.LastTimingEstimatorPredictDurationUs();
        row.estimator_update_start_us = loopController.LastTimingEstimatorUpdateStartUs();
        row.estimator_update_end_us = loopController.LastTimingEstimatorUpdateEndUs();
        row.estimator_update_duration_us = loopController.LastTimingEstimatorUpdateDurationUs();
        row.cycle_counter_start = loopController.LastTimingCycleCounterStart();
        row.cycle_counter_end = loopController.LastTimingCycleCounterEnd();
    }
}

namespace MazeMap::App::Internal
{
    OpenFloorMeasurementController::MainStage
        OpenFloorMeasurementController::BuildRegisteredMainStage() noexcept
    {
        static MainMeasurementRegime* const regimes[] = {
            &StaticMeasurementRegime::SharedInstance(),
            &LaunchMeasurementRegime::SharedInstance(),
            &YawLaunchMeasurementRegime::SharedInstance(),
            &StraightMeasurementRegime::SharedInstance(),
            &YawMeasurementRegime::SharedInstance(),
            &SmoothMeasurementRegime::SharedInstance(),
            &LoopMeasurementRegime::ClockwiseSharedInstance(),
            &LoopMeasurementRegime::CounterClockwiseSharedInstance(),
        };
        return MainStage(regimes, sizeof(regimes) / sizeof(regimes[0]));
    }

    bool OpenFloorMeasurementController::MainMeasurementRegime::EnsureReady(
        OpenFloorMeasurementController& controller)
    {
        (void)controller;
        return true;
    }

    OpenFloorMeasurementController::OpenFloorMeasurementController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.Vehicle())
        , _sensors(runtime.Sensors())
        , _drive(runtime.DriveBase())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
        , _mainStage(BuildRegisteredMainStage())
    {
    }

    OpenFloorMeasurementController::~OpenFloorMeasurementController() = default;

    OpenFloorMeasurementController::StaticMeasurementRegime&
        OpenFloorMeasurementController::StaticMeasurementRegime::SharedInstance() noexcept
    {
        static StaticMeasurementRegime regime{};
        return regime;
    }

    const char* OpenFloorMeasurementController::StaticMeasurementRegime::Title() const noexcept
    {
        return "static";
    }

    OpenFloorMeasurementController::MeasurementLogId
        OpenFloorMeasurementController::StaticMeasurementRegime::LogId() const noexcept
    {
        return kLogId;
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
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedIndex;
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

    CommandVector OpenFloorMeasurementController::StaticMeasurementRegime::Tick(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        (void)primitiveIndex;
        (void)speedIndex;

        Drive& driveService = controller._driveService;
        if (_needsStart)
        {
            const unsigned long totalHoldMs =
                static_cast<unsigned long>(DiagnosticConfig::kStaticHoldMs) +
                MazeMap::kOpenFloorInterPhaseHoldMs;
            driveService.StartHold(totalHoldMs, false);
            _needsStart = false;
        }

        const CommandVector control = driveService.GetNextControls(done);
        if (done)
        {
            _needsStart = true;
        }
        return control;
    }

    OpenFloorMeasurementController::LaunchMeasurementRegime&
        OpenFloorMeasurementController::LaunchMeasurementRegime::SharedInstance() noexcept
    {
        static LaunchMeasurementRegime regime{};
        return regime;
    }

    const char* OpenFloorMeasurementController::LaunchMeasurementRegime::Title() const noexcept
    {
        return "launch";
    }

    OpenFloorMeasurementController::MeasurementLogId
        OpenFloorMeasurementController::LaunchMeasurementRegime::LogId() const noexcept
    {
        return kLogId;
    }

    std::uint16_t
        OpenFloorMeasurementController::LaunchMeasurementRegime::PrimitiveCount() const noexcept
    {
        return 2U;
    }

    std::uint8_t OpenFloorMeasurementController::LaunchMeasurementRegime::SpeedCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorLaunchDriveMagnitudeCount);
    }

    std::uint16_t OpenFloorMeasurementController::LaunchMeasurementRegime::RepeatCount() const noexcept
    {
        return MazeMap::kOpenFloorLaunchRepeatsPerMagnitude;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::LaunchMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedIndex;
        return MazeMap::MC_NONE;
    }

    float OpenFloorMeasurementController::LaunchMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        if (speedIndex >= SpeedCount())
        {
            return 0.0f;
        }

        const float magnitude = MazeMap::kOpenFloorLaunchDriveMagnitudes[speedIndex];
        if (primitiveIndex == 0U)
        {
            return magnitude;
        }
        if (primitiveIndex == 1U)
        {
            return -magnitude;
        }
        return 0.0f;
    }

    CommandVector OpenFloorMeasurementController::LaunchMeasurementRegime::Tick(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        Drive& driveService = controller._driveService;
        const float driveCommand = SpeedBinValue(primitiveIndex, speedIndex);
        if (_tickCounter < kPulseTickCount)
        {
            ++_tickCounter;
            done = false;
            return CommandVector(driveCommand, driveCommand);
        }

        if (_tickCounter == kPulseTickCount)
        {
            driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            ++_tickCounter;
        }

        const CommandVector holdControl = driveService.GetNextControls(done);
        if (done)
        {
            _tickCounter = 0U;
        }
        return holdControl;
    }

    OpenFloorMeasurementController::StraightMeasurementRegime&
        OpenFloorMeasurementController::StraightMeasurementRegime::SharedInstance() noexcept
    {
        static StraightMeasurementRegime regime{};
        return regime;
    }

    const char* OpenFloorMeasurementController::StraightMeasurementRegime::Title() const noexcept
    {
        return "straight";
    }

    OpenFloorMeasurementController::MeasurementLogId
        OpenFloorMeasurementController::StraightMeasurementRegime::LogId() const noexcept
    {
        return kLogId;
    }

    std::uint16_t
        OpenFloorMeasurementController::StraightMeasurementRegime::PrimitiveCount() const noexcept
    {
        return 2U;
    }

    std::uint8_t OpenFloorMeasurementController::StraightMeasurementRegime::SpeedCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorStraightSpeedBinsMps.size());
    }

    std::uint16_t OpenFloorMeasurementController::StraightMeasurementRegime::RepeatCount() const noexcept
    {
        return MazeMap::kOpenFloorStraightRepeatsPerSpeed;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::StraightMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        (void)speedIndex;
        return MazeMap::S4;
    }

    float OpenFloorMeasurementController::StraightMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        if (speedIndex >= SpeedCount())
        {
            return 0.0f;
        }

        const std::uint8_t magnitudeIndex = static_cast<std::uint8_t>(speedIndex);
        const float magnitude = MazeMap::kOpenFloorStraightSpeedBinsMps[magnitudeIndex];
        return (primitiveIndex) ? magnitude : -magnitude;
    }

    CommandVector OpenFloorMeasurementController::StraightMeasurementRegime::Tick(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        done = false;
        Drive& driveService = controller._driveService;
        if (driveService.IsEffectivelyComplete())
        {
            if (_donesDetected == 0U)
            {
                // We just entered the regime motion.
                const float activeSpeedMps = SpeedBinValue(primitiveIndex, speedIndex);
                driveService.StartStraight(
                    MazeMap::OpenFloorStrEquivalentDistanceMeters(4U),
                    activeSpeedMps,
                    0.0f);
            }
			else if (_donesDetected == 1U)
            {
                driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs - 1, false);
            }
            else
            {
                // Final hold complete.
                _donesDetected = 0;
                return driveService.GetNextControls(done);
            }
            _donesDetected++;
        }
        bool doneLocal;
        return driveService.GetNextControls(doneLocal);
    }

    OpenFloorMeasurementController::YawMeasurementRegime&
        OpenFloorMeasurementController::YawMeasurementRegime::SharedInstance() noexcept
    {
        static YawMeasurementRegime regime{};
        return regime;
    }

    const char* OpenFloorMeasurementController::YawMeasurementRegime::Title() const noexcept
    {
        return "yaw";
    }

    OpenFloorMeasurementController::MeasurementLogId
        OpenFloorMeasurementController::YawMeasurementRegime::LogId() const noexcept
    {
        return kLogId;
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
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorYawRateBinsRadps.size());
    }

    std::uint16_t OpenFloorMeasurementController::YawMeasurementRegime::RepeatCount() const noexcept
    {
        return kRepeatCount;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::YawMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)speedIndex;
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
            (speedIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorYawRateBinsRadps.size())) ?
                MazeMap::kOpenFloorYawRateBinsRadps[speedIndex] :
                0.0f;
    }

    CommandVector OpenFloorMeasurementController::YawMeasurementRegime::Tick(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        done = false;
        Drive& driveService = controller._driveService;
        if (driveService.IsEffectivelyComplete())
        {
            if (_donesDetected == 0U)
            {
                MotionLimits limits = BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f);
                limits.SetMaxAngularSpeedRadps(SpeedBinValue(primitiveIndex, speedIndex));
                driveService.SetLimits(limits);
                driveService.StartTurn(MazeMap::kOpenFloorYawNominalAnglesRad[primitiveIndex]);
            }
            else if (_donesDetected == 1U)
            {
                driveService.StartHold(MazeMap::kOpenFloorPostSegmentHoldMs, false);
            }
            else
            {
                _donesDetected = 0U;
                return driveService.GetNextControls(done);
            }
            ++_donesDetected;
        }

        bool doneLocal = false;
        return driveService.GetNextControls(doneLocal);
    }

    OpenFloorMeasurementController::SmoothMeasurementRegime&
        OpenFloorMeasurementController::SmoothMeasurementRegime::SharedInstance() noexcept
    {
        static SmoothMeasurementRegime regime{};
        return regime;
    }

    OpenFloorMeasurementController::SmoothMeasurementRegime::SmoothMeasurementRegime() noexcept
    {
        _primitiveCodes.fill(MazeMap::MC_NONE);
        _primitiveCodes[0] = kOpenFloorMeasurementLoopStraightCode;
        for (std::size_t primitiveIndex = 0U;
             primitiveIndex < kOpenFloorMeasurementSmoothCycleCodes.size();
             ++primitiveIndex)
        {
            _primitiveCodes[primitiveIndex + 1U] = kOpenFloorMeasurementSmoothCycleCodes[primitiveIndex];
        }

        _primitiveCount = static_cast<std::uint16_t>(_primitiveCodes.size());
        if ((static_cast<std::size_t>(_primitiveCount) * kOpenFloorSmoothSpeedBinsMps.size()) !=
            _maneuvers.size())
        {
            _primitiveCount = 0U;
        }
    }

    const char* OpenFloorMeasurementController::SmoothMeasurementRegime::Title() const noexcept
    {
        return "smooth";
    }

    OpenFloorMeasurementController::MeasurementLogId
        OpenFloorMeasurementController::SmoothMeasurementRegime::LogId() const noexcept
    {
        return kLogId;
    }

    std::uint16_t OpenFloorMeasurementController::SmoothMeasurementRegime::PrimitiveCount() const noexcept
    {
        return _primitiveCount;
    }

    std::uint8_t OpenFloorMeasurementController::SmoothMeasurementRegime::SpeedCount() const noexcept
    {
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorSmoothSpeedBinsMps.size());
    }

    std::uint16_t OpenFloorMeasurementController::SmoothMeasurementRegime::RepeatCount() const noexcept
    {
        return 1U;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::SmoothMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        if (primitiveIndex >= PrimitiveCount())
        {
            return MazeMap::MC_NONE;
        }
        return static_cast<MazeMap::ManeuverCode>(
            static_cast<int>(_primitiveCodes[primitiveIndex]) -
            (((speedIndex == 0U) && (primitiveIndex == 0U)) ? 1 : 0));
    }

    float OpenFloorMeasurementController::SmoothMeasurementRegime::SpeedBinValue(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)primitiveIndex;
        return
            (speedIndex < SpeedCount()) ?
                MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex] :
                0.0f;
    }

    bool OpenFloorMeasurementController::SmoothMeasurementRegime::EnsureReady(
        OpenFloorMeasurementController& controller)
    {
        _stage = 0U;
        return (PrimitiveCount() != 0U) && EnsureInitialized(controller._vehicle);
    }

    CommandVector OpenFloorMeasurementController::SmoothMeasurementRegime::Tick(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        done = false;

        if (!_maneuversInitialized || (_configuredVehicle != &controller._vehicle))
        {
            controller._runtime.FailActiveMode(
                "Open-floor measurement smooth regime initialization failed");
            done = false;
            return StopControlVector();
        }

        Drive& driveService = controller._driveService;
        if (driveService.IsEffectivelyComplete())
        {
            if (_stage == 0U)
            {
                driveService.SetLimits(
                    BuildOpenFloorMeasurementLimits(
                        controller._vehicle,
                        SpeedBinValue(primitiveIndex, speedIndex)));
                driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                driveService.StartManeuver(_maneuvers[ManeuverOffset(primitiveIndex, speedIndex)]);
                _stage = 1U;
            }
            else if ((_stage == 1U) && controller.ActiveMainStageSlotEndsRegime())
            {
                driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
                driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                driveService.StartHold(MazeMap::kOpenFloorInterPhaseHoldMs, false);
                _stage = 2U;
            }
            else
            {
                _stage = 0U;
                return driveService.GetNextControls(done);
            }
        }

        return driveService.GetNextControls(done);
    }

    bool OpenFloorMeasurementController::SmoothMeasurementRegime::EnsureInitialized(
        MazeMap::Vehicle& vehicle)
    {
        if (_maneuversInitialized && (_configuredVehicle == &vehicle))
        {
            return true;
        }
        if (PrimitiveCount() == 0U)
        {
            return false;
        }

        MazeMap::DirectionalLocation current(2U, 3U, MazeMap::Up);
        float entryBoundarySpeedMps = 0.0f;
        for (std::uint8_t speedIndex = 0U; speedIndex < SpeedCount(); ++speedIndex)
        {
            MazeMap::ManeuverQueue queue{};
            float exitBoundarySpeedMps = 0.0f;
            MazeMap::DirectionalLocation endLocation{};
            if (!BuildQueue(
                    vehicle,
                    speedIndex,
                    current,
                    MazeMap::kOpenFloorSmoothSpeedBinsMps[speedIndex],
                    entryBoundarySpeedMps,
                    queue,
                    exitBoundarySpeedMps,
                    endLocation) ||
                queue.empty() ||
                (queue.size() != PrimitiveCount()))
            {
                _configuredVehicle = nullptr;
                _maneuversInitialized = false;
                return false;
            }

            for (std::uint16_t primitiveIndex = 0U; primitiveIndex < PrimitiveCount(); ++primitiveIndex)
            {
                _maneuvers[ManeuverOffset(primitiveIndex, speedIndex)] = queue[primitiveIndex];
            }

            current = endLocation;
            entryBoundarySpeedMps = exitBoundarySpeedMps;
        }

        _maneuversInitialized = true;
        _configuredVehicle = _maneuversInitialized ? &vehicle : nullptr;
        return _maneuversInitialized;
    }

    std::size_t OpenFloorMeasurementController::SmoothMeasurementRegime::ManeuverOffset(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        return
            (static_cast<std::size_t>(speedIndex) * static_cast<std::size_t>(PrimitiveCount())) +
            static_cast<std::size_t>(primitiveIndex);
    }

    bool OpenFloorMeasurementController::SmoothMeasurementRegime::BuildQueue(
        MazeMap::Vehicle& vehicle,
        const std::uint8_t speedIndex,
        const MazeMap::DirectionalLocation& startLocation,
        const float cruiseSpeedMps,
        const float initialEntrySpeedMps,
        MazeMap::ManeuverQueue& queue,
        float& exitBoundarySpeedMps,
        MazeMap::DirectionalLocation& endLocation) const
    {
        queue.clear();
        exitBoundarySpeedMps = 0.0f;
        endLocation = startLocation;

        MazeMap::DirectionalLocation current = startLocation;
        for (std::uint16_t primitiveIndex = 0U; primitiveIndex < PrimitiveCount(); ++primitiveIndex)
        {
            if (!queue.push_back(PrimitiveCode(primitiveIndex, speedIndex), current))
            {
                queue.clear();
                exitBoundarySpeedMps = 0.0f;
                return false;
            }
            current = queue.back().getEnd();
        }

        queue.ComputeSpeeds(vehicle, initialEntrySpeedMps, cruiseSpeedMps);
        if (queue.empty())
        {
            return false;
        }

        exitBoundarySpeedMps = queue.back().getExitSpeed();
        endLocation = queue.back().getEnd();
        return true;
    }

    OpenFloorMeasurementController::LoopMeasurementRegime&
        OpenFloorMeasurementController::LoopMeasurementRegime::ClockwiseSharedInstance() noexcept
    {
        static LoopMeasurementRegime regime{ 6U, true };
        return regime;
    }

    OpenFloorMeasurementController::LoopMeasurementRegime&
        OpenFloorMeasurementController::LoopMeasurementRegime::CounterClockwiseSharedInstance() noexcept
    {
        static LoopMeasurementRegime regime{ 7U, false };
        return regime;
    }

    OpenFloorMeasurementController::LoopMeasurementRegime::LoopMeasurementRegime(
        const MeasurementLogId logId,
        const bool clockwise) noexcept
        : _logId(logId)
        , _clockwise(clockwise)
    {
    }

    const char* OpenFloorMeasurementController::LoopMeasurementRegime::Title() const noexcept
    {
        return _clockwise ? "loop_clockwise" : "loop_counter_clockwise";
    }

    OpenFloorMeasurementController::MeasurementLogId
        OpenFloorMeasurementController::LoopMeasurementRegime::LogId() const noexcept
    {
        return _logId;
    }

    std::uint16_t OpenFloorMeasurementController::LoopMeasurementRegime::PrimitiveCount() const noexcept
    {
        return static_cast<std::uint16_t>(kLoopPhasePrimitiveCount);
    }

    std::uint8_t OpenFloorMeasurementController::LoopMeasurementRegime::SpeedCount() const noexcept
    {
        return 1U;
    }

    std::uint16_t OpenFloorMeasurementController::LoopMeasurementRegime::RepeatCount() const noexcept
    {
        return kRepeatCount;
    }

    MazeMap::ManeuverCode OpenFloorMeasurementController::LoopMeasurementRegime::PrimitiveCode(
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex) const noexcept
    {
        (void)speedIndex;
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

    bool OpenFloorMeasurementController::LoopMeasurementRegime::EnsureReady(
        OpenFloorMeasurementController& controller)
    {
        _stage = 0U;
        return EnsureInitialized(controller._vehicle);
    }

    CommandVector OpenFloorMeasurementController::LoopMeasurementRegime::Tick(
        OpenFloorMeasurementController& controller,
        const std::uint16_t primitiveIndex,
        const std::uint8_t speedIndex,
        bool& done)
    {
        done = false;

        if (!_maneuversInitialized || (_configuredVehicle != &controller._vehicle))
        {
            controller._runtime.FailActiveMode(
                "Open-floor measurement loop regime initialization failed");
            done = false;
            return StopControlVector();
        }

        Drive& driveService = controller._driveService;
        if (driveService.IsEffectivelyComplete())
        {
            if (_stage == 0U)
            {
                driveService.SetLimits(
                    BuildOpenFloorMeasurementLimits(
                        controller._vehicle,
                        SpeedBinValue(primitiveIndex, speedIndex)));
                driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                driveService.StartManeuver(_maneuvers[primitiveIndex]);
                _stage = 1U;
            }
            else if ((_stage == 1U) && controller.ActiveMainStageSlotEndsRegime())
            {
                driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
                driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                driveService.StartHold(MazeMap::kOpenFloorInterPhaseHoldMs, false);
                _stage = 2U;
            }
            else
            {
                _stage = 0U;
                return driveService.GetNextControls(done);
            }
        }

        return driveService.GetNextControls(done);
    }

    bool OpenFloorMeasurementController::LoopMeasurementRegime::EnsureInitialized(
        MazeMap::Vehicle& vehicle)
    {
        if (_maneuversInitialized && (_configuredVehicle == &vehicle))
        {
            return true;
        }

        MazeMap::ManeuverQueue queue{};
        if (!BuildQueue(vehicle, queue) || (queue.size() != _maneuvers.size()))
        {
            _configuredVehicle = nullptr;
            _maneuversInitialized = false;
            return false;
        }

        for (std::uint16_t primitiveIndex = 0U;
             primitiveIndex < static_cast<std::uint16_t>(_maneuvers.size());
             ++primitiveIndex)
        {
            _maneuvers[primitiveIndex] = queue[primitiveIndex];
        }

        _configuredVehicle = &vehicle;
        _maneuversInitialized = true;
        return true;
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

    bool OpenFloorMeasurementController::TimingStage::OpenTimingLog(
        OpenFloorMeasurementController& controller)
    {
        _tickIndex = 0U;
        _bufferedRow.reset();
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

        return true;
    }

    CommandVector OpenFloorMeasurementController::TimingStage::Tick(
        OpenFloorMeasurementController& controller,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        const CommandVector stopControl = CommandVector(0.0f, 0.0f);
        if (controller._startupCalibration.Active())
        {
            bool calibrationDone = false;
            const CommandVector calibrationControl = controller._startupCalibration.GetNextControls(calibrationDone);
            if (!calibrationDone)
            {
                return calibrationControl;
            }
        }

        if (!WriteBufferedRow(controller, "Open-floor measurement timing log write failed"))
        {
            controller._runtime.FailActiveMode("Open-floor measurement timing log write failed");
            return stopControl;
        }
        if (controller._bootFramework != nullptr &&
            !controller._bootFramework->IsSelectedModeSelectorInstalled())
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
            loopController.RequestEndSession(
                +[](void* const context, LoopController& boundaryLoopController)
                {
                    auto* const self = static_cast<OpenFloorMeasurementController*>(context);
                    if (self == nullptr)
                    {
                        GetSharedRobotRuntime().FailActiveMode(
                            "Open-floor measurement end-session callback context was null");
                        return;
                    }

                    if (!self->_timingStage.WriteBufferedRow(
                            *self,
                            "Open-floor measurement timing log write failed during timing-to-main handoff"))
                    {
                        self->_runtime.FailActiveMode(
                            "Open-floor measurement timing log write failed during timing-to-main handoff");
                    }
                    if (!self->_mainStage.OpenMainLog(*self))
                    {
                        self->_runtime.FailActiveMode(
                            "Open-floor measurement main log setup failed after timing capture");
                    }

                    self->_activeStageTick = &OpenFloorMeasurementController::MainStageTick;
                    boundaryLoopController.StageNextSessionState(
                        DiagnosticConfig::kControlPeriodUs,
                        self->_sessionStartPointX,
                        self->_sessionStartPointY,
                        LoopController::WallMask::All,
                        true,
                        true,
                        true,
                        false);
                },
                &controller);
        }

        return stopControl;
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
        ApplyLoopTimingToTimingRow(controller._loopController, row);
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

    bool OpenFloorMeasurementController::MainStage::OpenMainLog(
        OpenFloorMeasurementController& controller)
    {
        _activeRegimeIndex = 0U;
        _activePrimitiveIndex = 0U;
        _activeSpeedIndex = 0U;
        _activeRepeatIndex = 0U;
        _completionPending = false;
        _bufferedRow.reset();
        if ((_regimes == nullptr) || (_regimeCount == 0U))
        {
            return false;
        }

        for (std::size_t regimeIndex = 0U; regimeIndex < _regimeCount; ++regimeIndex)
        {
            MainMeasurementRegime* const regime = _regimes[regimeIndex];
            if ((regime == nullptr) ||
                (regime->Title() == nullptr) ||
                !regime->EnsureReady(controller) ||
                (regime->PrimitiveCount() == 0U) ||
                (regime->SpeedCount() == 0U) ||
                (regime->RepeatCount() == 0U) ||
                (regime->LogId() == kTimingLogId))
            {
                return false;
            }

            for (std::size_t priorRegimeIndex = 0U; priorRegimeIndex < regimeIndex; ++priorRegimeIndex)
            {
                if (_regimes[priorRegimeIndex]->LogId() == regime->LogId())
                {
                    return false;
                }
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
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("fan_duty_cycle", controller._vehicle.GetFanDuty(), 3)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", controller._sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", controller._sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!controller._runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", controller._sensors.GetGyroBiasRadps(), 6)) return false;
        if (!controller._runtime.WriteUtilityDataLogAccelBiasMetadata(controller._sensors)) return false;
        for (std::size_t regimeIndex = 0U; regimeIndex < _regimeCount; ++regimeIndex)
        {
            MainMeasurementRegime* const regime = _regimes[regimeIndex];
            char nameKey[32] = {};
            char phaseIdKey[36] = {};
            const int nameKeyLength = std::snprintf(
                nameKey,
                sizeof(nameKey),
                "phase_battery_%02u_name",
                static_cast<unsigned>(regimeIndex));
            const int phaseIdKeyLength = std::snprintf(
                phaseIdKey,
                sizeof(phaseIdKey),
                "phase_battery_%02u_phase_id",
                static_cast<unsigned>(regimeIndex));
            if ((nameKeyLength <= 0) ||
                (nameKeyLength >= static_cast<int>(sizeof(nameKey))) ||
                (phaseIdKeyLength <= 0) ||
                (phaseIdKeyLength >= static_cast<int>(sizeof(phaseIdKey))) ||
                !controller._runtime.WriteUtilityDataLogMetadata(nameKey, regime->Title()) ||
                !controller._runtime.WriteUtilityDataLogMetadataUnsigned(
                    phaseIdKey,
                    static_cast<unsigned long>(regime->LogId())))
            {
                return false;
            }
        }

        Runtime::OpenFloorMainRow row{};
        if (!controller._runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        return true;
    }

    bool OpenFloorMeasurementController::MainStage::IsCurrentSlotLastInRegime() const noexcept
    {
        const MainMeasurementRegime& regime = ActiveRegime();
        return
            ((_activePrimitiveIndex + 1U) == regime.PrimitiveCount()) &&
            ((_activeRepeatIndex + 1U) == regime.RepeatCount()) &&
            ((_activeSpeedIndex + 1U) == regime.SpeedCount());
    }

    CommandVector OpenFloorMeasurementController::MainStage::Tick(
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
        const CommandVector control =
            regime.Tick(controller, _activePrimitiveIndex, _activeSpeedIndex, done);

        Runtime::OpenFloorMainRow row{};
        controller.PopulateMainRowFromState(
            regime.LogId(),
            regime.PrimitiveCode(_activePrimitiveIndex, _activeSpeedIndex),
            regime.SpeedBinValue(_activePrimitiveIndex, _activeSpeedIndex),
            static_cast<std::uint16_t>(_activeRepeatIndex + 1U),
            state,
            row);
        BufferRow(row);

        if (done)
        {
            if (IsCurrentSlotLastInRegime())
            {
                ResetOpenFloorModeLimits(controller._driveService, controller._vehicle);
            }
            _completionPending = !AdvanceIndices();
        }

        return control;
    }

    OpenFloorMeasurementController::MainMeasurementRegime&
        OpenFloorMeasurementController::MainStage::ActiveRegime() const noexcept
    {
        return *_regimes[_activeRegimeIndex];
    }

    bool OpenFloorMeasurementController::MainStage::AdvanceIndices() noexcept
    {
        MainMeasurementRegime& regime = ActiveRegime();
        ++_activePrimitiveIndex;
        if (_activePrimitiveIndex < regime.PrimitiveCount())
        {
            return true;
        }

        _activePrimitiveIndex = 0U;
        ++_activeRepeatIndex;
        if (_activeRepeatIndex < regime.RepeatCount())
        {
            return true;
        }

        _activeRepeatIndex = 0U;
        ++_activeSpeedIndex;
        if (_activeSpeedIndex < regime.SpeedCount())
        {
            return true;
        }

        _activeSpeedIndex = 0U;
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
        row.master_time_us = controller._loopController.LastTimingTickStartUs();
        row.control_tick_sequence = controller._loopController.LastTimingSequence();
        row.dt_us = controller._loopController.LastTimingDtUs();
        row.encoder_timestamp_us = controller._loopController.LastTimingEncoderReadDoneUs();
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
        if (controller._bootFramework != nullptr &&
            !controller._bootFramework->IsSelectedModeSelectorInstalled())
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

    void OpenFloorMeasurementController::SetupMode(BootFramework& framework)
    {
        _bootFramework = &framework;
        (void)_runtime.AppendTextLogLine("Open-floor measurement mode");
        (void)_runtime.AppendTextLogLine(
            "Open-floor battery: timing capture plus the registered main-regime battery");

        _drive.ClearCommandEvidence();
        _activeStageTick = &OpenFloorMeasurementController::TimingStageTick;

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        _startupCalibration.Start();
        _vehicle.SetFanDuty(Config::kRacingFanDutyCycle);
        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService.SetLimits(BuildOpenFloorMeasurementModeLimits(_vehicle));

        if (!framework.IsSelectedModeSelectorInstalled())
        {
            _runtime.FailActiveMode("Open-floor measurement selector pins unavailable");
        }
        if (!framework.IsSelectedModeSelectorInstalled())
        {
            _runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
        }

        if (!_timingStage.OpenTimingLog(*this))
        {
            _runtime.FailActiveMode("Open-floor measurement timing log setup failed");
            return;
        }

        const auto& runtimeState = _runtime.RuntimeState();
        _sessionStartPointX = runtimeState.GetPositionX();
        _sessionStartPointY = runtimeState.GetPositionY();
        _loopController.StageNextSessionState(
            DiagnosticConfig::kControlPeriodUs,
            _sessionStartPointX,
            _sessionStartPointY,
            LoopController::WallMask::All,
            true,
            true,
            true,
            false);
    }

    void OpenFloorMeasurementController::OnModeFault(const char* reason) noexcept
    {
        (void)reason;
    }

    CommandVector OpenFloorMeasurementController::RunTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        return (this->*_activeStageTick)(loopEndTimeUs, state, loopController);
    }

    void OpenFloorMeasurementController::PopulateTimingRowFromState(
        const MazeMap::VehicleState& state,
        Runtime::OpenFloorTimingRow& row) const noexcept
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        row.phase_id = static_cast<std::uint32_t>(kTimingLogId);
        row.imu_drdy_us = sensors.ImuTiming().drdyUs;
        row.imu_read_start_us = sensors.ImuTiming().readStartUs;
        row.imu_read_done_us = sensors.ImuTiming().readDoneUs;
        row.front_led_on_us = sensors.FrontTiming().ledOnCommandUs;
        row.front_adc_on_us = sensors.FrontTiming().adcOnSampleUs;
        row.front_led_off_us = sensors.FrontTiming().ledOffCommandUs;
        row.front_adc_off_us = sensors.FrontTiming().adcOffSampleUs;
        row.front_ready_us = sensors.FrontTiming().observationReadyUs;
        row.left_led_on_us = sensors.LeftTiming().ledOnCommandUs;
        row.left_adc_on_us = sensors.LeftTiming().adcOnSampleUs;
        row.left_led_off_us = sensors.LeftTiming().ledOffCommandUs;
        row.left_adc_off_us = sensors.LeftTiming().adcOffSampleUs;
        row.left_ready_us = sensors.LeftTiming().observationReadyUs;
        row.right_led_on_us = sensors.RightTiming().ledOnCommandUs;
        row.right_adc_on_us = sensors.RightTiming().adcOnSampleUs;
        row.right_led_off_us = sensors.RightTiming().ledOffCommandUs;
        row.right_adc_off_us = sensors.RightTiming().adcOffSampleUs;
        row.right_ready_us = sensors.RightTiming().observationReadyUs;
    }

    void OpenFloorMeasurementController::PopulateMainRowFromState(
        const MeasurementLogId phaseId,
        const MazeMap::ManeuverCode primitiveCode,
        const float speedBinValue,
        const std::uint16_t repeatIndex,
        const MazeMap::VehicleState& state,
        Runtime::OpenFloorMainRow& row) const
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = _drive.LastTelemetry();
        const auto& currentCommand = state.GetCurrentCommand();
        const float leftWheelVelocityMps =
            MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedLeft());
        const float rightWheelVelocityMps =
            MazeMap::Vehicle::WheelLinearVelocityFromWheelSpeed(state.GetWheelSpeedRight());
        const float measuredLinearSpeedMps =
            MazeMap::Vehicle::BodyForwardVelocityFromWheelLinear(leftWheelVelocityMps, rightWheelVelocityMps);
        const float measuredYawRateFromWheelsRadps =
            MazeMap::Vehicle::BodyYawRateFromWheelLinear(leftWheelVelocityMps, rightWheelVelocityMps);
        const float measuredYawRateRadps =
            std::isfinite(sensors.YawRateRadps()) ?
                sensors.YawRateRadps() :
                measuredYawRateFromWheelsRadps;

        row.phase_id = static_cast<std::uint8_t>(phaseId);
        row.primitive_id = static_cast<std::uint8_t>(primitiveCode);
        row.speed_bin = speedBinValue;
        row.repeat_index = repeatIndex;
        row.mode_flags = driveTelemetry.commandKindFlags;
        row.SetVehicleState(state);
        row.measured_linear_speed_mps = measuredLinearSpeedMps;
        row.measured_yaw_rate_radps = measuredYawRateRadps;
        row.cmd_linear_mps = driveTelemetry.requestedForwardMps;
        row.cmd_yaw_rate_radps = driveTelemetry.requestedYawRateRadps;
        row.left_drive_command = currentCommand.LeftCommand();
        row.right_drive_command = currentCommand.RightCommand();
        row.left_plant_command = driveTelemetry.leftPlantCommand;
        row.right_plant_command = driveTelemetry.rightPlantCommand;
        row.left_command_residual = driveTelemetry.leftDriveCommand - driveTelemetry.leftPlantCommand;
        row.right_command_residual = driveTelemetry.rightDriveCommand - driveTelemetry.rightPlantCommand;
        row.left_target_velocity_mps = MazeMap::Vehicle::LeftWheelLinearVelocityFromBody(
            driveTelemetry.requestedForwardMps,
            driveTelemetry.requestedYawRateRadps);
        row.right_target_velocity_mps = MazeMap::Vehicle::RightWheelLinearVelocityFromBody(
            driveTelemetry.requestedForwardMps,
            driveTelemetry.requestedYawRateRadps);
        row.encoder_timestamp_us = 0U;
        row.left_encoder_count = static_cast<std::int32_t>(sensors.LeftEncoderTotalCounts());
        row.right_encoder_count = static_cast<std::int32_t>(sensors.RightEncoderTotalCounts());
        row.left_encoder_wheel_speed_radps = sensors.EncoderObservation().LeftWheelSpeedRadps();
        row.right_encoder_wheel_speed_radps = sensors.EncoderObservation().RightWheelSpeedRadps();
        row.left_encoder_distance_m = sensors.LeftEncoderDistanceM();
        row.right_encoder_distance_m = sensors.RightEncoderDistanceM();
        row.left_encoder_velocity_mps = sensors.EncoderObservation().LeftVelocityMps();
        row.right_encoder_velocity_mps = sensors.EncoderObservation().RightVelocityMps();
        row.imu_timestamp_us = sensors.ImuTiming().readDoneUs;
        row.imu_status = sensors.BackLeftImuTelemetry().status;
        row.accel_bias_valid = sensors.AccelerationBiasValid() ? 1U : 0U;
        row.imu_gyro_x = sensors.BackLeftImuTelemetry().gyroX;
        row.imu_gyro_y = sensors.BackLeftImuTelemetry().gyroY;
        row.imu_gyro_z = sensors.BackLeftImuTelemetry().gyroZ;
        row.imu_accel_x = sensors.BackLeftImuTelemetry().accelX;
        row.imu_accel_y = sensors.BackLeftImuTelemetry().accelY;
        row.imu_accel_z = sensors.BackLeftImuTelemetry().accelZ;
        row.imu_temp = sensors.BackLeftImuTelemetry().temp;
        row.gyro_raw_radps = sensors.RawYawRateRadps();
        row.gyro_bias_radps = sensors.YawRateBiasRadps();
        row.gyro_radps = sensors.YawRateRadps();
        row.accel_body_right_mps2 = sensors.BodyRightAccelerationMps2();
        row.accel_body_forward_mps2 = sensors.BodyForwardAccelerationMps2();
        row.planar_accel_mps2 = sensors.PlanarAccelerationMps2();
        row.front_timestamp_us = sensors.FrontTiming().observationReadyUs;
        row.left_timestamp_us = sensors.LeftTiming().observationReadyUs;
        row.right_timestamp_us = sensors.RightTiming().observationReadyUs;
        row.front_left_wall_ambient_adc = _vehicle.FrontLeftWallSensor().LatestAmbientAdcCode();
        row.front_left_wall_lit_adc = _vehicle.FrontLeftWallSensor().LatestLitAdcCode();
        row.front_right_wall_ambient_adc = _vehicle.FrontRightWallSensor().LatestAmbientAdcCode();
        row.front_right_wall_lit_adc = _vehicle.FrontRightWallSensor().LatestLitAdcCode();
        row.side_left_wall_ambient_adc = _vehicle.SideLeftWallSensor().LatestAmbientAdcCode();
        row.side_left_wall_lit_adc = _vehicle.SideLeftWallSensor().LatestLitAdcCode();
        row.side_right_wall_ambient_adc = _vehicle.SideRightWallSensor().LatestAmbientAdcCode();
        row.side_right_wall_lit_adc = _vehicle.SideRightWallSensor().LatestLitAdcCode();
    }

    void OpenFloorMeasurementController::FinalizeSuccessfulRun() noexcept
    {
        _bootFramework = nullptr;
        (void)_runtime.AppendTextLogLine("Open-floor measurement complete");
    }

    bool OpenFloorMeasurementController::ActiveMainStageSlotEndsRegime() const noexcept
    {
        return _mainStage.IsCurrentSlotLastInRegime();
    }

    CommandVector OpenFloorMeasurementController::TimingStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;
        return _timingStage.Tick(*this, state, loopController);
    }

    CommandVector OpenFloorMeasurementController::MainStageTick(
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
            "open_floor_measurement",
            "Run open-floor timing capture followed by the registered main-regime measurement battery.",
            "open_floor_timing.mmlog, open_floor_main.mmlog",
            &GetOpenFloorMeasurementMode,
            "GetOpenFloorMeasurementMode",
            "OpenFloorMeasurementController.cpp",
            "timing capture; registered main-regime battery",
            "DiagnosticConfig linear limits; OpenFloorMeasurementSpec speed bins; shared startup calibration; shared drive service",
            "Inter-phase 500 ms brake holds; launch and straight samples insert 250 ms brake holds between motions; loop-style regimes remain maneuver-driven",
            "open_floor_timing.mmlog, open_floor_main.mmlog",
            true,
        };
        return descriptor;
    }

    IApplicationMode& GetOpenFloorMeasurementMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}
