#include "pch.h"
#include "ShowcasingDonutController.h"

#include "BootUtilityModeFramework.h"
#include "DiagnosticConfig.h"
#include "Drive.h"
#include "DriveBase.h"
#include "MazeMapApplicationPrivate.h"
#include "MazeMapSharedRuntime.h"
#include "OpenFloorMeasurementSpec.h"
#include "PinPairStrap.h"
#include "PlantModel.h"
#include "RuntimeSensorSuite.h"
#include "StartupCalibration.h"
#include "VehicleState.h"

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

    constexpr std::uint8_t kShowcasingDonutSelectorDrivePin = 9U;
    constexpr std::uint8_t kShowcasingDonutSelectorSensePin = 10U;

    constexpr float kShowcasingDonutRadiusM = 0.090f;
    constexpr float kShowcasingDonutInitialSpeedMps = 0.30f;
    constexpr float kShowcasingDonutSpeedRampMps2 = 0.05f;
    constexpr float kShowcasingDonutSpeedCapMps = 4.0f;
    constexpr std::uint16_t kShowcasingDonutFanSpinupHoldMs = 1000U;
    constexpr std::uint16_t kShowcasingDonutCompletionHoldMs = 250U;
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
        , _vehicle(runtime.SpeedVehicle())
        , _drive(runtime.Drive())
        , _driveService(runtime.DriveService())
        , _startupCalibration(runtime.StartupCalibrationService())
    {
    }

    bool ShowcasingDonutController::Begin()
    {
        ResetState();
        if (!_runtime.RegisterModeFaultHandler(&ShowcasingDonutController::TeardownOnRuntimeFault, this, kShowcasingDonutStableId))
        {
            return false;
        }
        if (!SetupHardware())
        {
            return _runtime.FailActiveMode("Showcasing donut hardware setup failed");
        }

        (void)BootUtilityModeFramework::ResetStartupTrace("mode:showcasing_donut");
        (void)_runtime.AppendTextLogLine("Showcasing donut mode");
        (void)_runtime.AppendTextLogLine("Shared startup calibration, fixed-radius donut sweep, open-floor main-schema logging");

        if (!_drive.Begin())
        {
            return _runtime.FailActiveMode("Showcasing donut drive base init failed");
        }
        _drive.UseNominalWheelControlProfile();

        _startupCalibration.Cancel();
        _startupCalibration.SetIsInMaze(true);
        if (!_startupCalibration.BringUp())
        {
            return _runtime.FailActiveMode("Showcasing donut startup bring-up failed");
        }

        ConfigureSelectorMonitor();
        if (SelectorRemoved())
        {
            return _runtime.FailActiveMode(kShowcasingDonutSelectorRemovedReason);
        }

        if (!BeginMainLog())
        {
            return _runtime.FailActiveMode("Showcasing donut main log setup failed");
        }

        return true;
    }

    void ShowcasingDonutController::Run()
    {
        _phase = Phase::LaunchStartupCalibration;

        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &ShowcasingDonutController::ModeWorkThunk;
        callbacks.context = this;

        bool completed = false;
        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            (void)_runtime.FailActiveMode("Showcasing donut loop session start failed");
        }
        else
        {
            const LoopController::SessionResult result = _loopController.Run();
            completed = (result.status == LoopController::SessionResult::Status::Completed);
            _loopController.EndSession();
        }

        ReleaseSelectorMonitor();
        _startupCalibration.Cancel();
        _driveService.Cancel();
        _drive.Brake();
        _drive.UseNominalWheelControlProfile();
        SetMissionLevelFanEnabled(false);

        if (completed)
        {
            (void)_runtime.AppendTextLogFormatted(
                "Showcasing donut complete: reason=%s log=%s peak_cmd_mps=%.3f peak_enc_mps=%.3f peak_yaw_radps=%.3f peak_planar_accel_mps2=%.3f",
                EndReasonText(_endReason),
                _logFileName,
                _peakCommandedSpeedMps,
                _peakEncoderSpeedMps,
                _peakYawRateRadps,
                _peakPlanarAccelMps2);
        }
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
        self->_startupCalibration.Cancel();
        self->_driveService.Cancel();
        self->_drive.Brake();
        self->_drive.UseNominalWheelControlProfile();
        SetMissionLevelFanEnabled(false);
    }

    LoopController::ControlVector ShowcasingDonutController::ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<ShowcasingDonutController*>(context);
        if (self == nullptr)
        {
            services.Fault("Showcasing donut callback context was not installed");
            return LoopController::ControlVector::Brake;
        }

        return self->RunTick(loopEndTimeUs, state, services);
    }

    LoopController::SessionOptions ShowcasingDonutController::BuildLoopOptions() const noexcept
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        options.workPlan.useWallUpdates = false;
        return options;
    }

    void ShowcasingDonutController::ResetState() noexcept
    {
        ReleaseSelectorMonitor();
        _startupCalibration.Cancel();
        _driveService.Cancel();
        _phase = Phase::Idle;
        _endReason = EndReason::None;
        _logFileName[0] = '\0';
        _mainLogOpen = false;
        _commandedSpeedMps = 0.0f;
        _sweepElapsedS = 0.0f;
        _peakCommandedSpeedMps = 0.0f;
        _peakEncoderSpeedMps = 0.0f;
        _peakYawRateRadps = 0.0f;
        _peakPlanarAccelMps2 = 0.0f;
        _flashTurnsRemaining = 0U;
        _flashTurnTargetYawRad = 0.0f;
        _flashTurnMagnitudeRad = 0.0f;
        _appliedCommandTelemetry = BuildBrakeTelemetry();
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

    ShowcasingDonutController::CommandTelemetrySample ShowcasingDonutController::BuildCommandTelemetry(
        const LoopController::ControlVector& control) const noexcept
    {
        CommandTelemetrySample sample{};
        sample.linearCommandMps = _drive.GetLastLinearCommandMps();
        sample.angularCommandRadps = _drive.GetLastAngularCommandRadps();

        if (std::isfinite(control.leftMotorPwm) && std::isfinite(control.rightMotorPwm))
        {
            sample.driveTelemetry = _drive.GetGeneratedTelemetry(control);
            return sample;
        }

        return BuildBrakeTelemetry();
    }

    ShowcasingDonutController::CommandTelemetrySample ShowcasingDonutController::BuildBrakeTelemetry() const noexcept
    {
        CommandTelemetrySample sample{};
        sample.driveTelemetry = {};
        sample.linearCommandMps = 0.0f;
        sample.angularCommandRadps = 0.0f;
        sample.driveTelemetry.modeFlags = ::DriveBase::kModeBraking;
        return sample;
    }

    void ShowcasingDonutController::UpdatePeaks(const LoopController::ModeState& state) noexcept
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
        if (std::isfinite(state.sensors.gyroRadps))
        {
            _peakYawRateRadps = (std::max)(_peakYawRateRadps, std::fabs(state.sensors.gyroRadps));
        }
        if (std::isfinite(state.sensors.planarAccelMps2))
        {
            _peakPlanarAccelMps2 = (std::max)(_peakPlanarAccelMps2, std::fabs(state.sensors.planarAccelMps2));
        }
    }

    void ShowcasingDonutController::ConfigureSelectorMonitor() noexcept
    {
        ReleaseSelectorMonitor();
        _selectorDrivePin = kShowcasingDonutSelectorDrivePin;
        _selectorSensePin = kShowcasingDonutSelectorSensePin;
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

    bool ShowcasingDonutController::StartSharedStartupCalibration()
    {
        _startupCalibration.Start();
        return _startupCalibration.Active();
    }

    bool ShowcasingDonutController::StartHold(const std::uint16_t durationMs) noexcept
    {
        _driveService.Cancel();
        _driveService.SetLimits(BuildShowcasingDonutLimits(_vehicle));
        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _driveService.StartHold(durationMs, true);
        return _driveService.Active();
    }

    bool ShowcasingDonutController::BeginDonutSweep() noexcept
    {
        _driveService.Cancel();
        _commandedSpeedMps = kShowcasingDonutInitialSpeedMps;
        _sweepElapsedS = 0.0f;
        _endReason = EndReason::None;
        _tractionLoss = {};
        return true;
    }

    bool ShowcasingDonutController::BeginFlashTurn(const float angleRad) noexcept
    {
        _driveService.Cancel();
        _driveService.SetLimits(BuildShowcasingDonutLimits(_vehicle));
        _driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
        _flashTurnTargetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        _flashTurnMagnitudeRad = std::fabs(angleRad);
        _driveService.StartTurn(angleRad);
        return _driveService.Active();
    }

    bool ShowcasingDonutController::TractionLossDetected(const LoopController::ModeState& state) noexcept
    {
        const float encoderSpeedMps = EncoderAverageSpeedMps(state);
        const float expectedYawRateRadps = std::fabs(CommandedYawRateRadps());
        const float expectedPlanarAccelMps2 = std::fabs(_commandedSpeedMps * expectedYawRateRadps);
        const float measuredYawRateRadps = std::fabs(state.sensors.gyroRadps);
        const float measuredPlanarAccelMps2 = std::fabs(state.sensors.planarAccelMps2);

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
            _tractionLoss.mismatchDurationS += (state.dtSeconds > 0.0f) ? state.dtSeconds : 0.0f;
        }
        else
        {
            _tractionLoss.mismatchDurationS = 0.0f;
        }

        return _tractionLoss.mismatchDurationS >= kTractionLossConfirmS;
    }

    bool ShowcasingDonutController::EndConditionReached(const LoopController::ModeState& state)
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
        const LoopController::ModeState& state,
        const bool abortMarker)
    {
        Runtime::ShowcasingDonutMainRow row{};
        PopulateMainRow(labels, state, abortMarker, row);
        return _runtime.LogUtilityDataRow(row);
    }

    void ShowcasingDonutController::PopulateMainRow(
        const LogLabels& labels,
        const LoopController::ModeState& state,
        const bool abortMarker,
        Runtime::ShowcasingDonutMainRow& row) const
    {
        const bool encoderValid = state.driveTelemetry.encoderObservationValid;
        const bool imuValid = std::isfinite(state.sensors.gyroRawRadps);
        const float maxRangeM = MazeMap::PlantParams::Default().noHitRangeM;

        MazeMap::WallObs frontLeftObs{};
        MazeMap::WallObs frontRightObs{};
        ::DriveBase::BuildLoggedFrontPairObservations(
            state.sensors,
            maxRangeM,
            frontLeftObs,
            frontRightObs);
        const MazeMap::WallObs leftObs =
            ::DriveBase::BuildLoggedLeftSideObservation(state.sensors, maxRangeM);
        const MazeMap::WallObs rightObs =
            ::DriveBase::BuildLoggedRightSideObservation(state.sensors, maxRangeM);
        const MazeMap::VehicleState::StateVector estimatorState = _drive.GetEstimatorStateVector();

        row.master_time_us = state.tickStartUs;
        row.control_tick_sequence = state.sequence;
        row.dt_us = state.dtUs;
        row.section_id = labels.sectionId;
        row.primitive_id = labels.primitiveId;
        row.primitive_family = static_cast<std::uint8_t>(
            MazeMap::OpenFloorPrimitiveFamilyForId(static_cast<MazeMap::OpenFloorPrimitiveId>(labels.primitiveId)));
        row.direction_id = labels.directionId;
        row.phase_id = labels.phaseId;
        row.speed_bin = labels.speedBin;
        row.start_marker_id = labels.startMarkerId;
        row.repeat_index = labels.repeatIndex;
        row.progress_norm = labels.progressNorm;
        row.mode_flags = _appliedCommandTelemetry.driveTelemetry.modeFlags;
        row.clipping_flags = 0U;
        row.saturation_flags = _appliedCommandTelemetry.driveTelemetry.saturationFlags;
        row.watchdog_flags = 0U;
        row.ukf_mode_id = state.driveTelemetry.ukfModeId;
        row.ukf_yaw_valid_for_feedforward = state.driveTelemetry.ukfYawValidForFeedforward;
        row.bias_update_enabled = state.driveTelemetry.ukfBiasUpdateEnabled;
        row.ukf_state_px_m = estimatorState(MazeMap::VehicleState::kPx);
        row.ukf_state_py_m = estimatorState(MazeMap::VehicleState::kPy);
        row.ukf_state_psi_rad = estimatorState(MazeMap::VehicleState::kPsi);
        row.ukf_state_u_mps = estimatorState(MazeMap::VehicleState::kU);
        row.ukf_state_v_mps = estimatorState(MazeMap::VehicleState::kV);
        row.ukf_state_r_radps = estimatorState(MazeMap::VehicleState::kR);
        row.ukf_state_omega_l_radps = estimatorState(MazeMap::VehicleState::kOmegaL);
        row.ukf_state_omega_r_radps = estimatorState(MazeMap::VehicleState::kOmegaR);
        row.ukf_state_bgz_radps = estimatorState(MazeMap::VehicleState::kBgz);
        row.gyro_bias_anchor_radps = state.driveTelemetry.ukfGyroBiasAnchorRadps;
        row.yaw_consistency_lp_radps = state.driveTelemetry.ukfYawConsistencyLowPassRadps;
        row.yaw_window_mismatch_rad = state.driveTelemetry.ukfYawWindowMismatchRad;
        row.nhc_sigma_mps = state.driveTelemetry.ukfNhcSigmaMps;
        row.nhc_residual_mps = state.driveTelemetry.ukfNhcResidualMps;
        row.nhc_residual_sigma = state.driveTelemetry.ukfNhcResidualSigma;
        row.measured_linear_speed_mps = state.measured.linearSpeedMps;
        row.measured_angular_speed_radps = state.measured.angularSpeedRadps;
        row.cmd_linear_mps = _appliedCommandTelemetry.linearCommandMps;
        row.cmd_angular_radps = _appliedCommandTelemetry.angularCommandRadps;
        row.left_drive_command = _appliedCommandTelemetry.driveTelemetry.leftDriveCommand;
        row.right_drive_command = _appliedCommandTelemetry.driveTelemetry.rightDriveCommand;
        row.left_feedforward_command = _appliedCommandTelemetry.driveTelemetry.leftFeedforwardCommand;
        row.right_feedforward_command = _appliedCommandTelemetry.driveTelemetry.rightFeedforwardCommand;
        row.left_feedback_command = _appliedCommandTelemetry.driveTelemetry.leftFeedbackCommand;
        row.right_feedback_command = _appliedCommandTelemetry.driveTelemetry.rightFeedbackCommand;
        row.left_target_velocity_mps = _appliedCommandTelemetry.driveTelemetry.leftTargetVelocityMps;
        row.right_target_velocity_mps = _appliedCommandTelemetry.driveTelemetry.rightTargetVelocityMps;
        row.left_launch_assist_floor = _appliedCommandTelemetry.driveTelemetry.leftLaunchAssistFloor;
        row.right_launch_assist_floor = _appliedCommandTelemetry.driveTelemetry.rightLaunchAssistFloor;
        row.encoder_timestamp_us = 0U;
        row.left_encoder_count = state.driveTelemetry.leftEncoderCount;
        row.right_encoder_count = state.driveTelemetry.rightEncoderCount;
        row.left_encoder_omega_radps = state.driveTelemetry.leftEncoderOmegaRadps;
        row.right_encoder_omega_radps = state.driveTelemetry.rightEncoderOmegaRadps;
        row.left_encoder_distance_m = state.driveTelemetry.leftDistanceM;
        row.right_encoder_distance_m = state.driveTelemetry.rightDistanceM;
        row.left_encoder_velocity_mps = state.driveTelemetry.leftVelocityMps;
        row.right_encoder_velocity_mps = state.driveTelemetry.rightVelocityMps;
        row.imu_timestamp_us = state.sensors.imuTiming.readDoneUs;
        row.imu_status = state.sensors.imuBackLeft.status;
        row.imu_interrupt_high = state.sensors.imuBackLeft.interruptHigh ? 1U : 0U;
        row.accel_bias_valid = state.sensors.accelBiasValid ? 1U : 0U;
        row.imu_gyro_x = state.sensors.imuBackLeft.gyroX;
        row.imu_gyro_y = state.sensors.imuBackLeft.gyroY;
        row.imu_gyro_z = state.sensors.imuBackLeft.gyroZ;
        row.imu_accel_x = state.sensors.imuBackLeft.accelX;
        row.imu_accel_y = state.sensors.imuBackLeft.accelY;
        row.imu_accel_z = state.sensors.imuBackLeft.accelZ;
        row.imu_temp = state.sensors.imuBackLeft.temp;
        row.gyro_raw_radps = state.sensors.gyroRawRadps;
        row.gyro_bias_radps = state.sensors.gyroBiasRadps;
        row.gyro_radps = state.sensors.gyroRadps;
        row.accel_body_x_mps2 = state.sensors.accelBodyXMps2;
        row.accel_body_y_mps2 = state.sensors.accelBodyYMps2;
        row.planar_accel_mps2 = state.sensors.planarAccelMps2;
        row.front_timestamp_us = state.sensors.frontTiming.observationReadyUs;
        row.left_timestamp_us = state.sensors.leftTiming.observationReadyUs;
        row.right_timestamp_us = state.sensors.rightTiming.observationReadyUs;
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
            !state.estimatorHealthy,
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
        case Phase::LaunchStartupCalibration:
        case Phase::RunStartupCalibration:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec10Static);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveId::StaticHold);
            labels.directionId = static_cast<std::uint8_t>(MazeMap::OpenFloorDirectionId::None);
            labels.phaseId = static_cast<std::uint8_t>(MazeMap::OpenFloorPhaseId::Startup);
            labels.speedBin = static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::None);
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm = _startupCalibration.Active() ? 0.0f : 1.0f;
            break;

        case Phase::LaunchFanSpinupHold:
        case Phase::RunFanSpinupHold:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec10Static);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveId::StaticHold);
            labels.directionId = static_cast<std::uint8_t>(MazeMap::OpenFloorDirectionId::None);
            labels.phaseId = static_cast<std::uint8_t>(MazeMap::OpenFloorPhaseId::Hold);
            labels.speedBin = static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::None);
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm = 0.0f;
            break;

        case Phase::LaunchDonutSweep:
        case Phase::RunDonutSweep:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec60LoopCw);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveId::None);
            labels.directionId = static_cast<std::uint8_t>(MazeMap::OpenFloorDirectionId::Clockwise);
            labels.phaseId = static_cast<std::uint8_t>(MazeMap::OpenFloorPhaseId::Accel);
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

        case Phase::LaunchFlashTurn:
        case Phase::RunFlashTurn:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec40Yaw);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveId::Ip180);
            labels.directionId = static_cast<std::uint8_t>(MazeMap::OpenFloorDirectionId::Clockwise);
            labels.phaseId = static_cast<std::uint8_t>(MazeMap::OpenFloorPhaseId::SteadyRotation);
            labels.speedBin = static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::None);
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm =
                (_flashTurnMagnitudeRad > 1.0e-5f) ?
                    (std::clamp)(
                        1.0f - (std::fabs(AngleErrorRad(_flashTurnTargetYawRad, _drive.GetPose().yawRad)) / _flashTurnMagnitudeRad),
                        0.0f,
                        1.0f) :
                    0.0f;
            break;

        case Phase::LaunchCompletionHold:
        case Phase::RunCompletionHold:
        case Phase::Complete:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec10Static);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveId::StaticHold);
            labels.directionId = static_cast<std::uint8_t>(MazeMap::OpenFloorDirectionId::None);
            labels.phaseId = static_cast<std::uint8_t>(MazeMap::OpenFloorPhaseId::Stop);
            labels.speedBin = static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::None);
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm = (_phase == Phase::Complete) ? 1.0f : 0.0f;
            break;

        case Phase::Idle:
        default:
            labels.sectionId = static_cast<std::uint8_t>(MazeMap::OpenFloorSectionId::Sec10Static);
            labels.primitiveId = static_cast<std::uint8_t>(MazeMap::OpenFloorPrimitiveId::StaticHold);
            labels.directionId = static_cast<std::uint8_t>(MazeMap::OpenFloorDirectionId::None);
            labels.phaseId = static_cast<std::uint8_t>(MazeMap::OpenFloorPhaseId::Idle);
            labels.speedBin = static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::None);
            labels.startMarkerId = static_cast<std::uint8_t>(MazeMap::OpenFloorMarkerId::C);
            labels.progressNorm = 0.0f;
            break;
        }

        return labels;
    }

    std::uint8_t ShowcasingDonutController::SpeedBinForSpeed(const float speedMps) const noexcept
    {
        const float magnitudeMps = std::fabs(speedMps);
        if (!(std::isfinite(magnitudeMps) && (magnitudeMps >= MazeMap::kOpenFloorStraightSpeedBinsMps[0])))
        {
            return static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::None);
        }
        if (magnitudeMps < MazeMap::kOpenFloorStraightSpeedBinsMps[1])
        {
            return static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::Low);
        }
        if (magnitudeMps < MazeMap::kOpenFloorStraightSpeedBinsMps[2])
        {
            return static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::Medium);
        }
        return static_cast<std::uint8_t>(MazeMap::OpenFloorSpeedBin::High);
    }

    float ShowcasingDonutController::EncoderAverageSpeedMps(const LoopController::ModeState& state) const noexcept
    {
        const float leftSpeedMps = std::fabs(state.driveTelemetry.leftVelocityMps);
        const float rightSpeedMps = std::fabs(state.driveTelemetry.rightVelocityMps);
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

    LoopController::ControlVector ShowcasingDonutController::RunTick(
        const std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;

        UpdatePeaks(state);

        if (SelectorRemoved())
        {
            (void)LogCurrentSample(CurrentLabels(), state, true);
            services.Fault(kShowcasingDonutSelectorRemovedReason);
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;
        }

        if (_mainLogOpen && !LogCurrentSample(CurrentLabels(), state, false))
        {
            services.Fault("Showcasing donut main log write failed");
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;
        }

        switch (_phase)
        {
        case Phase::LaunchStartupCalibration:
            if (!StartSharedStartupCalibration())
            {
                services.Fault("Showcasing donut startup calibration could not start");
            }
            else
            {
                (void)_runtime.AppendTextLogLine("Showcasing donut startup calibration");
                _phase = Phase::RunStartupCalibration;
            }
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;

        case Phase::RunStartupCalibration:
        {
            bool done = false;
            const LoopController::ControlVector control = _startupCalibration.GetNextControls(done);
            if (done)
            {
                _phase = Phase::LaunchFanSpinupHold;
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }

            _appliedCommandTelemetry = BuildCommandTelemetry(control);
            return control;
        }

        case Phase::LaunchFanSpinupHold:
            SetMissionLevelFanEnabled(true);
            if (!StartHold(kShowcasingDonutFanSpinupHoldMs))
            {
                services.Fault("Showcasing donut fan-spinup hold could not start");
            }
            else
            {
                (void)_runtime.AppendTextLogLine("Showcasing donut fan spinup");
                _phase = Phase::RunFanSpinupHold;
            }
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;

        case Phase::RunFanSpinupHold:
            if (!_driveService.Active())
            {
                services.Fault("Showcasing donut fan-spinup hold was not active");
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }
            else
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (done)
                {
                    _phase = Phase::LaunchDonutSweep;
                    _appliedCommandTelemetry = BuildBrakeTelemetry();
                    return LoopController::ControlVector::Brake;
                }

                _appliedCommandTelemetry = BuildCommandTelemetry(control);
                return control;
            }

        case Phase::LaunchDonutSweep:
            if (!BeginDonutSweep())
            {
                services.Fault("Showcasing donut sweep could not start");
            }
            else
            {
                (void)_runtime.AppendTextLogFormatted(
                    "Showcasing donut sweep: radius=%.3f start_speed=%.3f ramp=%.3f speed_cap=%.3f center_limit=%.3f",
                    kShowcasingDonutRadiusM,
                    kShowcasingDonutInitialSpeedMps,
                    kShowcasingDonutSpeedRampMps2,
                    kShowcasingDonutSpeedCapMps,
                    kShowcasingDonutCenterRadiusLimitM);
                _phase = Phase::RunDonutSweep;
            }
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;

        case Phase::RunDonutSweep:
        {
            _sweepElapsedS += (state.dtSeconds > 0.0f) ? state.dtSeconds : 0.0f;
            if (EndConditionReached(state))
            {
                _phase = Phase::LaunchFlashTurn;
                _flashTurnsRemaining = kShowcasingDonutFlashTurnCount;
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }

            const float dtSeconds = (state.dtSeconds > 0.0f) ? state.dtSeconds : 0.0f;
            _commandedSpeedMps =
                (std::min)(kShowcasingDonutSpeedCapMps, _commandedSpeedMps + (kShowcasingDonutSpeedRampMps2 * dtSeconds));
            const LoopController::ControlVector control = _drive.PointControlVector(
                _commandedSpeedMps,
                CommandedYawRateRadps(),
                MazeMap::CommandPD::StateWheelOmegaPD);
            _appliedCommandTelemetry = BuildCommandTelemetry(control);
            return control;
        }

        case Phase::LaunchFlashTurn:
            if (_flashTurnsRemaining == 0U)
            {
                _phase = Phase::LaunchCompletionHold;
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }
            if (!BeginFlashTurn(kShowcasingDonutFlashTurnAngleRad))
            {
                _phase = Phase::LaunchCompletionHold;
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }
            --_flashTurnsRemaining;
            _phase = Phase::RunFlashTurn;
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;

        case Phase::RunFlashTurn:
            if (!_driveService.Active())
            {
                services.Fault("Showcasing donut flash turn was not active");
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }
            else
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (done)
                {
                    _phase = (_flashTurnsRemaining > 0U) ? Phase::LaunchFlashTurn : Phase::LaunchCompletionHold;
                    _appliedCommandTelemetry = BuildBrakeTelemetry();
                    return LoopController::ControlVector::Brake;
                }

                _appliedCommandTelemetry = BuildCommandTelemetry(control);
                return control;
            }

        case Phase::LaunchCompletionHold:
            if (!StartHold(kShowcasingDonutCompletionHoldMs))
            {
                services.Fault("Showcasing donut completion hold could not start");
            }
            else
            {
                _phase = Phase::RunCompletionHold;
            }
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;

        case Phase::RunCompletionHold:
            if (!_driveService.Active())
            {
                services.Fault("Showcasing donut completion hold was not active");
                _appliedCommandTelemetry = BuildBrakeTelemetry();
                return LoopController::ControlVector::Brake;
            }
            else
            {
                bool done = false;
                const LoopController::ControlVector control = _driveService.GetNextControls(done);
                if (done)
                {
                    _phase = Phase::Complete;
                    _appliedCommandTelemetry = BuildBrakeTelemetry();
                    return LoopController::ControlVector::Brake;
                }

                _appliedCommandTelemetry = BuildCommandTelemetry(control);
                return control;
            }

        case Phase::Complete:
            services.RequestEndLoop();
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;

        case Phase::Idle:
        default:
            services.Fault("Showcasing donut phase was not initialized");
            _appliedCommandTelemetry = BuildBrakeTelemetry();
            return LoopController::ControlVector::Brake;
        }
    }

    IApplicationMode& GetShowcasingDonutMode()
    {
        static ShowcasingDonutController mode(GetSharedRobotRuntime());
        return mode;
    }
}
