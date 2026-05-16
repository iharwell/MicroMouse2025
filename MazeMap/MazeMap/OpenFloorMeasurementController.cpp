#include "pch.h"
#include "OpenFloorMeasurementController.h"

#include "MazeMapApplicationPrivate.h"
#include "BootModeDescriptor.h"
#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DriveBase.h"
#include "DriveTelemetry.h"
#include "ManeuverQueue.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SharedRobotRuntime.h"
#include "SigmaPointSetSimplex.h"
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

    constexpr const char* kOpenFloorMeasurementStableId = "open_floor_measurement";
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
        limits.SetMaxAngularSpeedRadps(vehicle.GetMaxRotationalVelocity());
        limits.SetAngularAccelRadps2(vehicle.GetMaxAngularAcceleration());
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
            runtime.WriteUtilityDataLogMetadataUnsigned(
                "nx",
                static_cast<unsigned long>(MazeMap::VehicleState::kDimension)) &&
            runtime.WriteUtilityDataLogMetadataUnsigned(
                "nsig",
                static_cast<unsigned long>(
                    MazeMap::SigmaPointSetSimplex::ActiveSigmaCountForDimension(
                        MazeMap::VehicleState::kDimension))) &&
            runtime.Plant().WriteOpenFloorUkfMetadata(
                [&runtime](const char* key, float value, std::uint8_t precision) -> bool
                {
                    return runtime.WriteUtilityDataLogMetadataFloat(key, value, precision);
                });
    }

    void ApplyLoopTimingToTimingRow(
        const LoopController::TimingDiagnostics& timing,
        OpenFloorTimingRow& row) noexcept
    {
        row.mono_time_us = timing.tickStartUs;
        row.control_tick_sequence = timing.sequence;
        row.dt_us = timing.dtUs;
        row.control_start_us = timing.tickStartUs;
        row.control_end_us = timing.tickFinalizeUs;
        row.pwm_latch_us = timing.commandAppliedUs;
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
        return static_cast<std::uint8_t>(MazeMap::kOpenFloorYawOmegaBinsRadps.size());
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
            (speedIndex < static_cast<std::uint8_t>(MazeMap::kOpenFloorYawOmegaBinsRadps.size())) ?
                MazeMap::kOpenFloorYawOmegaBinsRadps[speedIndex] :
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
                    boundaryLoopController.StageNextSessionState(self->BuildLoopOptions());
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
        ApplyLoopTimingToTimingRow(controller._loopController.LastDiagnostics(), row);
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
        const LoopController::TimingDiagnostics& timing = controller._loopController.LastDiagnostics();
        row.master_time_us = timing.tickStartUs;
        row.control_tick_sequence = timing.sequence;
        row.dt_us = timing.dtUs;
        row.encoder_timestamp_us = timing.encoderReadDoneUs;
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
            "Open-floor battery: timing capture plus the registered main-regime battery");

        _drive.ClearCommandEvidence();

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(false);
        if (!_startupCalibration.BringUp())
        {
            _runtime.FailActiveMode("Open-floor measurement startup bring-up failed");
        }
        _vehicle.SetFanDuty(Config::kRacingFanDutyCycle);
        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService.SetLimits(BuildOpenFloorMeasurementModeLimits(_vehicle));

        ConfigureSelectorMonitor();
        if (SelectorRemoved())
        {
            _runtime.FailActiveMode(kOpenFloorMeasurementSelectorRemovedReason);
        }

        if (!_timingStage.OpenTimingLog(*this))
        {
            _runtime.FailActiveMode("Open-floor measurement timing log setup failed");
        }

        const auto& runtimeState = _runtime.RuntimeState();
        _sessionStartPointX = runtimeState.GetPositionX();
        _sessionStartPointY = runtimeState.GetPositionY();
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

    CommandVector OpenFloorMeasurementController::RunTick(
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
        options.workPlan.SetUseWallUpdates(false);
        options.SessionStartPointX = _sessionStartPointX;
        options.SessionStartPointY = _sessionStartPointY;
        return options;
    }

    void OpenFloorMeasurementController::PopulateTimingRowFromState(
        const MazeMap::VehicleState& state,
        Runtime::OpenFloorTimingRow& row) const noexcept
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        row.phase_id = static_cast<std::uint32_t>(kTimingLogId);
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
        const float leftWheelVelocityMps =
            MazeMap::Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedLeft());
        const float rightWheelVelocityMps =
            MazeMap::Vehicle::WheelLinearVelocityFromOmega(state.GetWheelSpeedRight());
        const float measuredLinearSpeedMps =
            MazeMap::Vehicle::BodyForwardVelocityFromWheelLinear(leftWheelVelocityMps, rightWheelVelocityMps);
        const float measuredAngularSpeedFromWheelsRadps =
            MazeMap::Vehicle::BodyYawRateFromWheelLinear(leftWheelVelocityMps, rightWheelVelocityMps);
        const float measuredAngularSpeedRadps =
            std::isfinite(sensors.gyroRadps) ?
                sensors.gyroRadps :
                measuredAngularSpeedFromWheelsRadps;

        row.phase_id = static_cast<std::uint8_t>(phaseId);
        row.primitive_id = static_cast<std::uint8_t>(primitiveCode);
        row.speed_bin = speedBinValue;
        row.repeat_index = repeatIndex;
        row.mode_flags = driveTelemetry.commandKindFlags;
        row.saturation_flags = driveTelemetry.solverFailureFlags;
        row.SetVehicleState(state);
        row.measured_linear_speed_mps = measuredLinearSpeedMps;
        row.measured_angular_speed_radps = measuredAngularSpeedRadps;
        row.cmd_linear_mps = driveTelemetry.requestedForwardMps;
        row.cmd_angular_radps = driveTelemetry.requestedYawRateRadps;
        row.left_drive_command = driveTelemetry.leftDriveCommand;
        row.right_drive_command = driveTelemetry.rightDriveCommand;
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
        row.left_encoder_count = static_cast<std::int32_t>(sensors.leftEncoderTotalCounts);
        row.right_encoder_count = static_cast<std::int32_t>(sensors.rightEncoderTotalCounts);
        row.left_encoder_omega_radps = sensors.encoderObservation.omegaLeftRadps;
        row.right_encoder_omega_radps = sensors.encoderObservation.omegaRightRadps;
        row.left_encoder_distance_m = sensors.leftEncoderDistanceM;
        row.right_encoder_distance_m = sensors.rightEncoderDistanceM;
        row.left_encoder_velocity_mps = sensors.encoderObservation.leftVelocityMps;
        row.right_encoder_velocity_mps = sensors.encoderObservation.rightVelocityMps;
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

    void OpenFloorMeasurementController::FinalizeSuccessfulRun() noexcept
    {
        ReleaseSelectorMonitor();
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
            BootModeCategory::Utility,
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
        };
        return descriptor;
    }

    IApplicationMode& GetOpenFloorMeasurementMode()
    {
        static OpenFloorMeasurementController mode(GetSharedRobotRuntime());
        return mode;
    }
}
