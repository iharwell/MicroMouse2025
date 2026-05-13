#include "pch.h"
#include "ShowcasingDonutController.h"

#include "BootModeRegistry.h"
#include "BootUtilityModeFramework.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "DriveBase.h"
#include "DriveTelemetry.h"
#include "MazeMapApplicationPrivate.h"
#include "SharedRobotRuntime.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "SensorSnapshot.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "WallObservationPipeline.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr const char* kShowcasingDonutStableId = "showcasing_donut";
    constexpr const char* kShowcasingDonutModeVariant = "showcasing_donut";
    constexpr const char* kShowcasingDonutBootReason = "pins9_10";
    constexpr const char* kShowcasingDonutSelectorRemovedReason =
        "Showcasing donut selector jumper removed";
    constexpr const char* kShowcasingDonutMainLogHostFallback = "showcasing_donut_main.mmlog";

    constexpr float kShowcasingDonutRadiusM = 0.090f;
    constexpr float kShowcasingDonutInitialSpeedMps = 0.30f;
    constexpr float kShowcasingDonutSpeedRampMps2 = 0.05f;
    constexpr float kShowcasingDonutSpeedCapMps = 4.0f;
    constexpr float kShowcasingDonutFlashTurnAngleRad = PI_F;
    constexpr std::uint8_t kShowcasingDonutFlashTurnCount = 2U;
    constexpr float kShowcasingDonutCenterRadiusLimitM = 0.27f;

    constexpr float kTractionLossMinEncoderSpeedMps = 0.35f;
    constexpr float kTractionLossMinExpectedPlanarAccelMps2 = 1.0f;
    constexpr float kTractionLossYawCoherenceFloor = 0.70f;
    constexpr float kTractionLossPlanarCoherenceFloor = 0.65f;
    constexpr float kTractionLossConfirmS = 0.15f;

    constexpr std::uint16_t kOpenFloorMeasurementFlagAbortMarker = 1u << 0;
    constexpr std::uint16_t kOpenFloorMeasurementFlagEstimatorFault = 1u << 2;
    constexpr std::uint16_t kOpenFloorMeasurementFlagFanEnabled = 1u << 3;
    constexpr std::uint16_t kOpenFloorMeasurementFlagEncoderValid = 1u << 4;
    constexpr std::uint16_t kOpenFloorMeasurementFlagImuValid = 1u << 5;
    constexpr std::uint16_t kOpenFloorMeasurementFlagAccelBiasValid = 1u << 6;
    constexpr std::uint16_t kOpenFloorMeasurementFlagFrontLeftObsValid = 1u << 7;
    constexpr std::uint16_t kOpenFloorMeasurementFlagFrontRightObsValid = 1u << 8;
    constexpr std::uint16_t kOpenFloorMeasurementFlagLeftObsValid = 1u << 9;
    constexpr std::uint16_t kOpenFloorMeasurementFlagRightObsValid = 1u << 10;
    constexpr std::uint8_t kShowcasingDonutDirectionLogIdNone = 0U;
    constexpr std::uint8_t kShowcasingDonutDirectionLogIdClockwise = 1U;

    MotionLimits BuildShowcasingDonutLimits(const MazeMap::Vehicle& vehicle) noexcept
    {
        MotionLimits limits{};
        limits.maxSpeedMps = (std::min)(vehicle.GetMaxSpeed(), kShowcasingDonutSpeedCapMps);
        limits.accelMps2 = vehicle.GetMaxForwardAcceleration();
        limits.decelMps2 = vehicle.GetMaxForwardAcceleration();
        limits.maxAngularSpeedRadps = vehicle.GetMaxRotationalVelocity();
        limits.angularAccelRadps2 = vehicle.GetMaxAngularAcceleration();
        return limits;
    }

    std::uint16_t BuildOpenFloorMeasurementFlags(
        const bool abortMarker,
        const bool estimatorFault,
        const MazeMap::App::Internal::Runtime::ShowcasingDonutMainRow& row,
        const bool encoderValid,
        const bool imuValid,
        const MazeMap::WallObs& frontLeftObs,
        const MazeMap::WallObs& frontRightObs,
        const MazeMap::WallObs& leftObs,
        const MazeMap::WallObs& rightObs) noexcept
    {
        std::uint16_t flags = 0U;
        if (abortMarker)
        {
            flags |= kOpenFloorMeasurementFlagAbortMarker;
        }
        if (estimatorFault)
        {
            flags |= kOpenFloorMeasurementFlagEstimatorFault;
        }
        if (row.fan_duty_cycle > 0.0f)
        {
            flags |= kOpenFloorMeasurementFlagFanEnabled;
        }
        if (encoderValid)
        {
            flags |= kOpenFloorMeasurementFlagEncoderValid;
        }
        if (imuValid)
        {
            flags |= kOpenFloorMeasurementFlagImuValid;
        }
        if (row.accel_bias_valid != 0U)
        {
            flags |= kOpenFloorMeasurementFlagAccelBiasValid;
        }
        if (frontLeftObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagFrontLeftObsValid;
        }
        if (frontRightObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagFrontRightObsValid;
        }
        if (leftObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagLeftObsValid;
        }
        if (rightObs.valid)
        {
            flags |= kOpenFloorMeasurementFlagRightObsValid;
        }
        return flags;
    }
}

namespace MazeMap::App::Internal
{
    ShowcasingDonutController::ShowcasingDonutController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.Vehicle())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
    {
    }

    void ShowcasingDonutController::SetupMode()
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(&ShowcasingDonutController::TeardownOnRuntimeFault, this, kShowcasingDonutStableId))
        {
            _runtime.FailActiveMode("Showcasing donut fault handler registration failed");
        }
        if (!SetupHardware())
        {
            _runtime.FailActiveMode("Showcasing donut hardware setup failed");
        }

        (void)BootUtilityModeFramework::ResetStartupTrace("mode:showcasing_donut");
        (void)_runtime.AppendTextLogLine("Showcasing donut mode");
        (void)_runtime.AppendTextLogLine("Fixed-radius donut sweep followed by flashy in-place turns, open-floor main-schema logging");

        if (!_drive.Begin())
        {
            _runtime.FailActiveMode("Showcasing donut drive base init failed");
        }

        ConfigureSelectorMonitor();
        if (!_selectorMonitorArmed)
        {
            _runtime.FailActiveMode("Showcasing donut selector pins unavailable");
        }
        if (SelectorRemoved())
        {
            _runtime.FailActiveMode(kShowcasingDonutSelectorRemovedReason);
        }

        if (!BeginMainLog())
        {
            _runtime.FailActiveMode("Showcasing donut main log setup failed");
        }
        if (!BeginDonutSweep())
        {
            _runtime.FailActiveMode("Showcasing donut sweep could not start");
        }

        _phase = Phase::DonutSweep;
        SetMissionLevelFanEnabled(true);
        (void)_runtime.AppendTextLogFormatted(
            "Showcasing donut sweep: radius=%.3f start_speed=%.3f ramp=%.3f speed_cap=%.3f center_limit=%.3f",
            kShowcasingDonutRadiusM,
            kShowcasingDonutInitialSpeedMps,
            kShowcasingDonutSpeedRampMps2,
            kShowcasingDonutSpeedCapMps,
            kShowcasingDonutCenterRadiusLimitM);
        _loopController.StageNextSessionState(BuildLoopOptions());
    }

    void ShowcasingDonutController::TeardownOnRuntimeFault(void* context, const char* reason) noexcept
    {
        (void)reason;
        auto* const self = static_cast<ShowcasingDonutController*>(context);
        if (self == nullptr)
        {
            return;
        }

        self->ReleaseSelectorMonitor();
        self->_phase = Phase::Idle;
        self->_drive.Brake();
        SetMissionLevelFanEnabled(false);
    }

    LoopController::SessionOptions ShowcasingDonutController::BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        const auto& runtimeState = _runtime.RuntimeState();
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.workPlan.SetUseWallUpdates(false);
        options.SessionStartPointX = runtimeState.GetPositionX();
        options.SessionStartPointY = runtimeState.GetPositionY();
        return options;
    }

    void ShowcasingDonutController::ResetState() noexcept
    {
        ReleaseSelectorMonitor();
        _phase = Phase::Idle;
        _endReason = EndReason::None;
        _logFileName[0] = '\0';
        _mainLogOpen = false;
        _bufferedMainRow = {};
        _bufferedMainRowValid = false;
        _commandedSpeedMps = 0.0f;
        _peakCommandedSpeedMps = 0.0f;
        _peakEncoderSpeedMps = 0.0f;
        _peakYawRateRadps = 0.0f;
        _peakPlanarAccelMps2 = 0.0f;
        _flashTurnsStarted = 0U;
        _tractionLoss = {};
    }

    bool ShowcasingDonutController::BeginMainLog()
    {
        if (!_runtime.OpenUtilityDataLog(
                _logFileName,
                sizeof(_logFileName),
                nullptr,
                "donut%03u.mmlog",
                kShowcasingDonutMainLogHostFallback))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("mode_variant", kShowcasingDonutModeVariant)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorMainStreamType)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("boot_reason", kShowcasingDonutBootReason)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("selector_drive_pin", _selectorDrivePin)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("selector_sense_pin", _selectorSensePin)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("turn_radius_m", kShowcasingDonutRadiusM, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("turn_start_speed_mps", kShowcasingDonutInitialSpeedMps, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("turn_ramp_mps2", kShowcasingDonutSpeedRampMps2, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("turn_speed_cap_mps", kShowcasingDonutSpeedCapMps, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("center_radius_limit_m", kShowcasingDonutCenterRadiusLimitM, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("finish_turn_angle_rad", kShowcasingDonutFlashTurnAngleRad, 6)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("finish_turn_count", kShowcasingDonutFlashTurnCount)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _runtime.Sensors().GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _runtime.Sensors().GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _runtime.Sensors().GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_runtime.Sensors())) return false;

        Runtime::ShowcasingDonutMainRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _mainLogOpen = true;
        return true;
    }

    bool ShowcasingDonutController::WriteBufferedMainRow()
    {
        if (!_bufferedMainRowValid)
        {
            return true;
        }

        const LoopController::TimingDiagnostics& timing = _loopController.LastDiagnostics();
        _bufferedMainRow.master_time_us = timing.tickStartUs;
        _bufferedMainRow.control_tick_sequence = timing.sequence;
        _bufferedMainRow.dt_us = timing.dtUs;
        _bufferedMainRow.encoder_timestamp_us = timing.encoderReadDoneUs;
        if (!_runtime.LogUtilityDataRow(_bufferedMainRow))
        {
            return false;
        }

        _bufferedMainRow = {};
        _bufferedMainRowValid = false;
        return true;
    }

    void ShowcasingDonutController::UpdatePeaks(const MazeMap::VehicleState& state) noexcept
    {
        if (std::isfinite(_commandedSpeedMps))
        {
            _peakCommandedSpeedMps = (std::max)(_peakCommandedSpeedMps, _commandedSpeedMps);
        }

        const float encoderSpeedMps = EncoderAverageSpeedMps(state);
        if (std::isfinite(encoderSpeedMps))
        {
            _peakEncoderSpeedMps = (std::max)(_peakEncoderSpeedMps, encoderSpeedMps);
        }
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        if (std::isfinite(sensors.gyroRadps))
        {
            _peakYawRateRadps = (std::max)(_peakYawRateRadps, std::fabs(sensors.gyroRadps));
        }
        if (std::isfinite(sensors.planarAccelMps2))
        {
            _peakPlanarAccelMps2 = (std::max)(_peakPlanarAccelMps2, std::fabs(sensors.planarAccelMps2));
        }
    }

    void ShowcasingDonutController::ConfigureSelectorMonitor() noexcept
    {
        ReleaseSelectorMonitor();
        const BootModeRegistryEntry* const entry =
            FindBootModeRegistryEntry(BootModeId::ShowcasingDonut);
        if ((entry == nullptr) || (entry->selector.kind != BootModeSelectorKind::PinPair))
        {
            return;
        }

        _selectorDrivePin = entry->selector.pinA;
        _selectorSensePin = entry->selector.pinB;
        BeginPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        _selectorMonitorArmed = true;
    }

    void ShowcasingDonutController::ReleaseSelectorMonitor() noexcept
    {
        if (_selectorMonitorArmed)
        {
            EndPinPairStrapMonitor(_selectorDrivePin, _selectorSensePin);
        }
        _selectorMonitorArmed = false;
        _selectorDrivePin = 0U;
        _selectorSensePin = 0U;
    }

    bool ShowcasingDonutController::SelectorRemoved() const noexcept
    {
        return _selectorMonitorArmed && !IsPinPairStrapMonitorClosed(_selectorSensePin);
    }

    bool ShowcasingDonutController::BeginDonutSweep() noexcept
    {
        _commandedSpeedMps = kShowcasingDonutInitialSpeedMps;
        _endReason = EndReason::None;
        _flashTurnsStarted = 0U;
        _tractionLoss = {};
        return true;
    }

    bool ShowcasingDonutController::BeginFlashTurn(const float angleRad) noexcept
    {
        _driveService.SetLimits(BuildShowcasingDonutLimits(_vehicle));
        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService.StartTurn(angleRad);
        return true;
    }

    bool ShowcasingDonutController::TractionLossDetected(const MazeMap::VehicleState& state) noexcept
    {
        const float encoderSpeedMps = EncoderAverageSpeedMps(state);
        const float expectedYawRateRadps = std::fabs(CommandedYawRateRadps());
        const float expectedPlanarAccelMps2 = std::fabs(_commandedSpeedMps * expectedYawRateRadps);
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        const float measuredYawRateRadps = std::fabs(sensors.gyroRadps);
        const float measuredPlanarAccelMps2 = std::fabs(sensors.planarAccelMps2);

        const float yawCoherence =
            (expectedYawRateRadps > 1.0e-5f) && std::isfinite(measuredYawRateRadps) ?
                (measuredYawRateRadps / expectedYawRateRadps) :
                1.0f;
        const float planarCoherence =
            (expectedPlanarAccelMps2 > 1.0e-5f) && std::isfinite(measuredPlanarAccelMps2) ?
                (measuredPlanarAccelMps2 / expectedPlanarAccelMps2) :
                1.0f;

        _tractionLoss.lastYawCoherence = yawCoherence;
        _tractionLoss.lastPlanarCoherence = planarCoherence;

        if (!(std::isfinite(encoderSpeedMps) && (encoderSpeedMps >= kTractionLossMinEncoderSpeedMps)) ||
            !(std::isfinite(expectedPlanarAccelMps2) && (expectedPlanarAccelMps2 >= kTractionLossMinExpectedPlanarAccelMps2)))
        {
            _tractionLoss.mismatchDurationS = 0.0f;
            return false;
        }

        const bool mismatch =
            !std::isfinite(yawCoherence) ||
            !std::isfinite(planarCoherence) ||
            (yawCoherence < kTractionLossYawCoherenceFloor) ||
            (planarCoherence < kTractionLossPlanarCoherenceFloor);

        if (mismatch)
        {
            const float dtSeconds = static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f;
            _tractionLoss.mismatchDurationS += (dtSeconds > 0.0f) ? dtSeconds : 0.0f;
        }
        else
        {
            _tractionLoss.mismatchDurationS = 0.0f;
        }

        return _tractionLoss.mismatchDurationS >= kTractionLossConfirmS;
    }

    bool ShowcasingDonutController::EndConditionReached(const MazeMap::VehicleState& state)
    {
        if (_endReason != EndReason::None)
        {
            return true;
        }

        if ((_commandedSpeedMps >= kShowcasingDonutSpeedCapMps) ||
            (EncoderAverageSpeedMps(state) >= kShowcasingDonutSpeedCapMps))
        {
            RecordRequestedEnd(EndReason::SpeedCap);
            return true;
        }

        if (TractionLossDetected(state))
        {
            RecordRequestedEnd(EndReason::TractionLoss);
            return true;
        }

        return false;
    }

    void ShowcasingDonutController::RecordRequestedEnd(const EndReason reason)
    {
        if (_endReason != EndReason::None)
        {
            return;
        }

        _endReason = reason;
        (void)_runtime.AppendTextLogFormatted(
            "Showcasing donut ending: reason=%s yaw_coherence=%.3f planar_coherence=%.3f",
            EndReasonText(reason),
            _tractionLoss.lastYawCoherence,
            _tractionLoss.lastPlanarCoherence);
    }

    const char* ShowcasingDonutController::EndReasonText(const EndReason reason) const noexcept
    {
        switch (reason)
        {
        case EndReason::TractionLoss:
            return "traction_loss";
        case EndReason::SpeedCap:
            return "speed_cap";
        case EndReason::None:
        default:
            return "none";
        }
    }

    bool ShowcasingDonutController::LogCurrentSample(
        const LogLabels& labels,
        const MazeMap::VehicleState& state,
        const bool abortMarker)
    {
        if (!WriteBufferedMainRow())
        {
            return false;
        }

        PopulateMainRow(labels, state, abortMarker, _bufferedMainRow);
        _bufferedMainRowValid = true;
        return true;
    }

    void ShowcasingDonutController::PopulateMainRow(
        const LogLabels& labels,
        const MazeMap::VehicleState& state,
        const bool abortMarker,
        Runtime::ShowcasingDonutMainRow& row) const
    {
        row = {};
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
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
        const bool encoderValid = driveTelemetry.encoderObservationValid;
        const bool imuValid = std::isfinite(sensors.gyroRawRadps);
        const float maxRangeM = MazeMap::kDefaultWallObservationMaxRangeM;

        MazeMap::WallObs frontLeftObs{};
        MazeMap::WallObs frontRightObs{};
        MazeMap::BuildFrontWallObservations(
            sensors.frontWallObservationValid,
            sensors.frontWall,
            sensors.frontWallUsesFallbackDetection,
            sensors.frontWallUsesCharacterizationDetection,
            sensors.frontLeftDistanceM,
            sensors.frontRightDistanceM,
            maxRangeM,
            frontLeftObs,
            frontRightObs);
        const MazeMap::WallObs leftObs =
            MazeMap::BuildSideWallObservation(
                sensors.leftDistanceValidForControl,
                sensors.leftTransitionDetected,
                sensors.leftWallObservation,
                sensors.sideLeftDistanceM,
                maxRangeM);
        const MazeMap::WallObs rightObs =
            MazeMap::BuildSideWallObservation(
                sensors.rightDistanceValidForControl,
                sensors.rightTransitionDetected,
                sensors.rightWallObservation,
                sensors.sideRightDistanceM,
                maxRangeM);
        row.section_id = labels.sectionId;
        row.primitive_id = labels.primitiveId;
        row.primitive_family = static_cast<std::uint8_t>(
            MazeMap::OpenFloorPrimitiveFamilyForManeuverCode(static_cast<MazeMap::ManeuverCode>(labels.primitiveId)));
        row.direction_id = labels.directionId;
        row.phase_id = labels.phaseId;
        row.speed_bin = labels.speedBin;
        row.start_marker_id = labels.startMarkerId;
        row.repeat_index = labels.repeatIndex;
        row.progress_norm = labels.progressNorm;
        row.mode_flags = driveTelemetry.modeFlags;
        row.clipping_flags = 0U;
        row.saturation_flags = driveTelemetry.saturationFlags;
        row.watchdog_flags = 0U;
        row.ukf_state_px_m = state.GetPositionX();
        row.ukf_state_py_m = state.GetPositionY();
        row.ukf_state_psi_rad = state.GetOrientation();
        row.ukf_state_u_mps = state.GetVelocity();
        row.ukf_state_v_mps = state.GetLateralVelocity();
        row.ukf_state_r_radps = state.GetRotationalVelocity();
        row.ukf_state_omega_l_radps = state.GetWheelSpeedLeft();
        row.ukf_state_omega_r_radps = state.GetWheelSpeedRight();
        row.ukf_state_bgz_radps = state.GetGyroBiasZ();
        row.measured_linear_speed_mps = measuredLinearSpeedMps;
        row.measured_angular_speed_radps = measuredAngularSpeedRadps;
        row.cmd_linear_mps = driveTelemetry.commandedLinearSpeedMps;
        row.cmd_angular_radps = driveTelemetry.commandedAngularSpeedRadps;
        row.left_drive_command = driveTelemetry.leftDriveCommand;
        row.right_drive_command = driveTelemetry.rightDriveCommand;
        row.left_feedforward_command = driveTelemetry.leftFeedforwardCommand;
        row.right_feedforward_command = driveTelemetry.rightFeedforwardCommand;
        row.left_feedback_command = driveTelemetry.leftFeedbackCommand;
        row.right_feedback_command = driveTelemetry.rightFeedbackCommand;
        row.left_target_velocity_mps = driveTelemetry.leftTargetVelocityMps;
        row.right_target_velocity_mps = driveTelemetry.rightTargetVelocityMps;
        row.left_launch_assist_floor = driveTelemetry.leftLaunchAssistFloor;
        row.right_launch_assist_floor = driveTelemetry.rightLaunchAssistFloor;
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
        row.imu_interrupt_high = sensors.imuBackLeft.interruptHigh ? 1U : 0U;
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
        row.front_left_obs_class = static_cast<std::uint8_t>(frontLeftObs.cls);
        row.front_right_obs_class = static_cast<std::uint8_t>(frontRightObs.cls);
        row.left_obs_class = static_cast<std::uint8_t>(leftObs.cls);
        row.right_obs_class = static_cast<std::uint8_t>(rightObs.cls);
        row.front_left_obs_rho_m = frontLeftObs.rho;
        row.front_right_obs_rho_m = frontRightObs.rho;
        row.left_obs_rho_m = leftObs.rho;
        row.right_obs_rho_m = rightObs.rho;
        row.front_left_obs_confidence = frontLeftObs.confidence;
        row.front_right_obs_confidence = frontRightObs.confidence;
        row.left_obs_confidence = leftObs.confidence;
        row.right_obs_confidence = rightObs.confidence;
        row.fan_duty_cycle = GetMissionFanDutyCycle();
        row.measurement_flags = BuildOpenFloorMeasurementFlags(
            abortMarker,
            _runtime.Estimator().HasFault(),
            row,
            encoderValid,
            imuValid,
            frontLeftObs,
            frontRightObs,
            leftObs,
            rightObs);
    }

    ShowcasingDonutController::LogLabels ShowcasingDonutController::CurrentLabels() const noexcept
    {
        LogLabels labels{};
        labels.repeatIndex = 0U;

        switch (_phase)
        {
        case Phase::DonutSweep:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec60LoopCw);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::MC_NONE);
            labels.directionId = kShowcasingDonutDirectionLogIdClockwise;
            labels.phaseId = static_cast<std::uint8_t>(Phase::DonutSweep);
            labels.speedBin = SpeedBinForSpeed(_commandedSpeedMps);
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::CW);
            labels.progressNorm =
                (kShowcasingDonutSpeedCapMps > kShowcasingDonutInitialSpeedMps) ?
                    (std::clamp)(
                        (_commandedSpeedMps - kShowcasingDonutInitialSpeedMps) /
                            (kShowcasingDonutSpeedCapMps - kShowcasingDonutInitialSpeedMps),
                        0.0f,
                        1.0f) :
                    0.0f;
            break;

        case Phase::FlashyMoves:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec40Yaw);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::IP180);
            labels.directionId = kShowcasingDonutDirectionLogIdClockwise;
            labels.phaseId = static_cast<std::uint8_t>(Phase::FlashyMoves);
            labels.speedBin = MazeMap::kOpenFloorSpeedBinLogIdNone;
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm =
                (kShowcasingDonutFlashTurnCount > 0U) ?
                    (std::clamp)(
                        static_cast<float>(_flashTurnsStarted) / static_cast<float>(kShowcasingDonutFlashTurnCount),
                        0.0f,
                        1.0f) :
                    1.0f;
            break;

        case Phase::Complete:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec10Static);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::MC_NONE);
            labels.directionId = kShowcasingDonutDirectionLogIdNone;
            labels.phaseId = static_cast<std::uint8_t>(Phase::Complete);
            labels.speedBin = MazeMap::kOpenFloorSpeedBinLogIdNone;
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm = (_phase == Phase::Complete) ? 1.0f : 0.0f;
            break;

        case Phase::Idle:
        default:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec10Static);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::MC_NONE);
            labels.directionId = kShowcasingDonutDirectionLogIdNone;
            labels.phaseId = static_cast<std::uint8_t>(Phase::Idle);
            labels.speedBin = MazeMap::kOpenFloorSpeedBinLogIdNone;
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm = 0.0f;
            break;
        }

        return labels;
    }

    std::uint8_t ShowcasingDonutController::SpeedBinForSpeed(const float speedMps) const noexcept
    {
        return MazeMap::OpenFloorSpeedBinLogIdForMagnitudeMps(speedMps);
    }

    float ShowcasingDonutController::EncoderAverageSpeedMps(const MazeMap::VehicleState& state) const noexcept
    {
        (void)state;
        const DriveTelemetry driveTelemetry = _drive.GetTelemetry();
        const float leftSpeedMps = std::fabs(driveTelemetry.leftVelocityMps);
        const float rightSpeedMps = std::fabs(driveTelemetry.rightVelocityMps);
        if (!(std::isfinite(leftSpeedMps) && std::isfinite(rightSpeedMps)))
        {
            return 0.0f;
        }
        return 0.5f * (leftSpeedMps + rightSpeedMps);
    }

    float ShowcasingDonutController::CommandedYawRateRadps() const noexcept
    {
        return (kShowcasingDonutRadiusM > 1.0e-5f) ? (_commandedSpeedMps / kShowcasingDonutRadiusM) : 0.0f;
    }

    CommandVector ShowcasingDonutController::RunTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController& loopController)
    {
        (void)loopEndTimeUs;

        UpdatePeaks(state);

        if (_mainLogOpen && !WriteBufferedMainRow())
        {
            _runtime.FailActiveMode("Showcasing donut main log write failed");
            return CommandVector::Brake();
        }

        if (SelectorRemoved())
        {
            _runtime.FailActiveMode(kShowcasingDonutSelectorRemovedReason);
            return CommandVector::Brake();
        }

        if (_mainLogOpen && !LogCurrentSample(CurrentLabels(), state, false))
        {
            _runtime.FailActiveMode("Showcasing donut main log write failed");
            return CommandVector::Brake();
        }

        switch (_phase)
        {
        case Phase::DonutSweep:
        {
            const float dtSeconds =
                (std::max)(0.0f, static_cast<float>(_loopController.LastDiagnostics().dtUs) * 1.0e-6f);
            _commandedSpeedMps =
                (std::min)(kShowcasingDonutSpeedCapMps, _commandedSpeedMps + (kShowcasingDonutSpeedRampMps2 * dtSeconds));
            if (EndConditionReached(state))
            {
                (void)_runtime.AppendTextLogLine("Showcasing donut flashy turns");
                _phase = Phase::FlashyMoves;
                return CommandVector::Brake();
            }

            const CommandVector control = _drive.PointControlVector(
                _commandedSpeedMps,
                CommandedYawRateRadps(),
                MazeMap::CommandPD::EncoderVelocity);
            return control;
        }

        case Phase::FlashyMoves:
        {
            bool done = false;
            const CommandVector control = _driveService.GetNextControls(done);
            if (!done)
            {
                return control;
            }

            if (_flashTurnsStarted >= kShowcasingDonutFlashTurnCount)
            {
                _phase = Phase::Complete;
                return CommandVector::Brake();
            }

            if (!BeginFlashTurn(kShowcasingDonutFlashTurnAngleRad))
            {
                _runtime.FailActiveMode("Showcasing donut flashy turn could not start");
                return CommandVector::Brake();
            }

            ++_flashTurnsStarted;
            return CommandVector::Brake();
        }

        case Phase::Complete:
            loopController.RequestEndSession(
                +[](void* const context, LoopController& boundaryLoopController)
                {
                    auto* const self = static_cast<ShowcasingDonutController*>(context);
                    if (self == nullptr)
                    {
                        GetSharedRobotRuntime().FailActiveMode(
                            "Showcasing donut completion callback context was null");
                        return;
                    }

                    if (!self->WriteBufferedMainRow())
                    {
                        self->_runtime.FailActiveMode("Showcasing donut main log write failed");
                    }
                    self->ReleaseSelectorMonitor();
                    self->_drive.Brake();
                    SetMissionLevelFanEnabled(false);
                    (void)self->_runtime.AppendTextLogFormatted(
                        "Showcasing donut complete: reason=%s log=%s peak_cmd_mps=%.3f peak_enc_mps=%.3f peak_yaw_radps=%.3f peak_planar_accel_mps2=%.3f",
                        self->EndReasonText(self->_endReason),
                        self->_logFileName,
                        self->_peakCommandedSpeedMps,
                        self->_peakEncoderSpeedMps,
                        self->_peakYawRateRadps,
                        self->_peakPlanarAccelMps2);
                    self->_phase = Phase::Idle;
                    boundaryLoopController.HaltExecutionEndProgram();
                },
                this);
            return CommandVector::Brake();

        case Phase::Idle:
        default:
            _runtime.FailActiveMode("Showcasing donut phase was not initialized");
            return CommandVector::Brake();
        }
    }

    IApplicationMode& GetShowcasingDonutMode()
    {
        static ShowcasingDonutController mode(GetSharedRobotRuntime());
        return mode;
    }

    const BootModeDescriptor& GetShowcasingDonutBootModeDescriptor()
    {
        static constexpr BootModeDescriptor descriptor{
            BootModeId::ShowcasingDonut,
            BootModeCategory::Utility,
            "showcasing_donut",
            "Execute a fixed-radius clockwise donut sweep, ramp speed until traction loss or the 4 m/s cap, then finish with bounded flashy turns.",
            "logging.txt, donutNNN.mmlog",
            &GetShowcasingDonutMode,
            "GetShowcasingDonutMode",
            "ShowcasingDonutController.cpp",
            "fixed-radius donut sweep; bounded flashy turns",
            "DiagnosticConfig control period; OpenFloorMeasurementSpec log vocabulary; shared drive and drive-base services",
            "Fixed 0.090 m clockwise turn radius; 0.30 m/s initial speed; 0.05 m/s^2 speed ramp; 4.0 m/s end cap; selector removal faults the run; finish stays within 0.27 m of donut center",
            "donutNNN.mmlog",
        };
        return descriptor;
    }
}
