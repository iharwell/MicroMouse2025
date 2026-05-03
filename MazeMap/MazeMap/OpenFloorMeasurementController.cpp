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
    const OpenFloorMeasurementController::SegmentExecutor
        OpenFloorMeasurementController::kHoldSegmentExecutor{ &OpenFloorMeasurementController::ExecuteHoldSegment };
    const OpenFloorMeasurementController::SegmentExecutor
        OpenFloorMeasurementController::kWheelCommandProfileExecutor{
            &OpenFloorMeasurementController::ExecuteWheelCommandProfileSegment };
    const OpenFloorMeasurementController::SegmentExecutor
        OpenFloorMeasurementController::kDrivePrimitiveExecutor{
            &OpenFloorMeasurementController::ExecuteDrivePrimitiveSegment };

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

        if (!CompileMainPlan())
        {
            return _runtime.FailActiveMode("Open-floor measurement plan compilation failed");
        }

        if (!BeginTimingLog())
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

        if (completed && _mainStage.logOpen && _mainStage.pendingSampleValid)
        {
            _mainStage.pendingRow.encoder_timestamp_us =
                _loopController.LastDiagnostics().controlTiming.encoderReadDoneUs;
            if (_runtime.LogUtilityDataRow(_mainStage.pendingRow))
            {
                _mainStage.pendingSampleValid = false;
                _mainStage.pendingRow = {};
            }
        }

        if (completed && _timingStage.logOpen && _timingStage.pendingSampleValid)
        {
            ApplyControlTimingToTimingRow(_loopController.LastDiagnostics().controlTiming, _timingStage.pendingRow);
            if (_runtime.LogUtilityDataRow(_timingStage.pendingRow))
            {
                _timingStage.pendingSampleValid = false;
                _timingStage.pendingRow = {};
            }
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
        _timingStage = {};
        _mainStage = {};
    }

    bool OpenFloorMeasurementController::BeginTimingLog()
    {
        if (!_runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorTimingFileName))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorTimingStreamType)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("boot_reason", MazeMap::kOpenFloorBootReason)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!WriteOpenFloorV62Metadata(_runtime)) return false;

        OpenFloorTimingRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _timingStage.logOpen = true;
        return true;
    }

    void OpenFloorMeasurementController::StagePendingTimingSample(const OpenFloorTimingRow& row) noexcept
    {
        _timingStage.pendingRow = row;
        _timingStage.pendingSampleValid = true;
    }

    bool OpenFloorMeasurementController::BeginMainLog()
    {
        if (!_runtime.OpenUtilityDataLogFile(MazeMap::kOpenFloorMainFileName))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", MazeMap::kOpenFloorSelectedRoutineName)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("stream_type", MazeMap::kOpenFloorMainStreamType)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_version", MazeMap::kOpenFloorFormatVersion)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("revisions", MazeMap::kOpenFloorRevisionBundle)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("imu_setup", MazeMap::kOpenFloorImuSetup)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("boot_reason", MazeMap::kOpenFloorBootReason)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", MazeMap::kOpenFloorLogFormatSpec)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", MazeMap::kOpenFloorEndianness)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        if (!WriteOpenFloorV62Metadata(_runtime)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("fan_duty_cycle", GetMissionFanDutyCycle(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;

        OpenFloorMainRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        _mainStage.logOpen = true;
        return true;
    }

    bool OpenFloorMeasurementController::CommitPendingMainSample(
        LoopController::TickServices& services,
        const char* failureReason)
    {
        if (!_mainStage.pendingSampleValid)
        {
            return true;
        }

        _mainStage.pendingRow.encoder_timestamp_us =
            _loopController.LastDiagnostics().controlTiming.encoderReadDoneUs;
        if (!_runtime.LogUtilityDataRow(_mainStage.pendingRow))
        {
            services.Fault(failureReason);
            return false;
        }

        _mainStage.pendingSampleValid = false;
        _mainStage.pendingRow = {};
        return true;
    }

    void OpenFloorMeasurementController::StagePendingMainSample(const OpenFloorMainRow& row) noexcept
    {
        _mainStage.pendingRow = row;
        _mainStage.pendingSampleValid = true;
    }

    void OpenFloorMeasurementController::PopulateTimingRowFromState(
        const MazeMap::VehicleState& state,
        OpenFloorTimingRow& row) const noexcept
    {
        const SensorSnapshot& sensors = state.GetSensorSnapshot();
        row.mono_time_us = _loopController.CurrentTickStartUs();
        row.control_tick_sequence = _loopController.CurrentTickSequence();
        row.dt_us = _loopController.CurrentTickDtUs();
        row.phase_id = static_cast<std::uint32_t>(MazeMap::OpenFloorSectionId::Sec00Timing);
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
        const SegmentIdentity& identity,
        const MazeMap::VehicleState& state,
        OpenFloorMainRow& row) const
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

    bool OpenFloorMeasurementController::CompileMainPlan()
    {
        if (!ResetMainPlan())
        {
            return false;
        }

        MainSegment staticHold{};
        staticHold.executor = &kHoldSegmentExecutor;
        staticHold.identity = SegmentIdentity(
            OpenFloorSectionId::Sec10Static,
            OpenFloorPrimitiveId::StaticHold,
            OpenFloorSpeedBin::None,
            1U);
        staticHold.hold.durationMs = kOpenFloorStaticHoldWithLaunchSettleMs;
        const WheelCommandSweepDefinition launchSweep{
            OpenFloorSectionId::Sec20Launch,
            OpenFloorPrimitiveId::OpenLoopLaunch,
            MazeMap::kOpenFloorLaunchDriveMagnitudes.data(),
            MazeMap::kOpenFloorLaunchDriveMagnitudes.size(),
            static_cast<std::uint16_t>(MazeMap::kOpenFloorLaunchPulseMs),
            MazeMap::kOpenFloorLaunchRepeatsPerMagnitude,
            1.0f,
            1.0f,
            true,
            static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs),
        };
        const StraightSweepDefinition straightSweep{
            OpenFloorSectionId::Sec30Straight,
            OpenFloorPrimitiveId::Str4,
            MazeMap::kOpenFloorStraightSpeedBinsMps.data(),
            MazeMap::kOpenFloorStraightSpeedBinsMps.size(),
            MazeMap::OpenFloorStrEquivalentDistanceMeters(4U),
            MazeMap::kOpenFloorStraightRepeatsPerSpeed,
            true,
            static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs),
        };
        const TurnSweepDefinition yawSweep{
            OpenFloorSectionId::Sec40Yaw,
            MazeMap::kOpenFloorYawPrimitiveIds.data(),
            MazeMap::kOpenFloorYawNominalAnglesRad.data(),
            MazeMap::kOpenFloorYawPrimitiveIds.size(),
            MazeMap::kOpenFloorYawOmegaBinsRadps.data(),
            MazeMap::kOpenFloorYawOmegaBinsRadps.size(),
            DiagnosticConfig::kYawRepeatsPerPrimitiveSpeed,
            static_cast<std::uint16_t>(MazeMap::kOpenFloorPostSegmentHoldMs),
        };
        const ManeuverQueueDefinition smoothQueue{
            ManeuverQueueKind::SmoothSweep,
            OpenFloorSectionId::Sec50Smooth,
            MazeMap::kOpenFloorSmoothSpeedBinsMps.data(),
            MazeMap::kOpenFloorSmoothSpeedBinsMps.size(),
            false,
            1U,
            static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs),
        };
        const ManeuverQueueDefinition clockwiseLoopQueue{
            ManeuverQueueKind::Loop,
            OpenFloorSectionId::Sec60LoopCw,
            nullptr,
            0U,
            true,
            DiagnosticConfig::kLoopRepeats,
            static_cast<std::uint16_t>(MazeMap::kOpenFloorInterPhaseHoldMs),
        };
        const ManeuverQueueDefinition counterClockwiseLoopQueue{
            ManeuverQueueKind::Loop,
            OpenFloorSectionId::Sec70LoopCcw,
            nullptr,
            0U,
            false,
            DiagnosticConfig::kLoopRepeats,
            0U,
        };

        return
            AppendSegment(staticHold) &&
            CompileWheelCommandSweep(launchSweep) &&
            CompileStraightSweep(straightSweep) &&
            CompileTurnSweep(yawSweep) &&
            CompileManeuverQueue(smoothQueue) &&
            CompileManeuverQueue(clockwiseLoopQueue) &&
            CompileManeuverQueue(counterClockwiseLoopQueue);
    }

    bool OpenFloorMeasurementController::ResetMainPlan() noexcept
    {
        _mainStage = {};
        return true;
    }

    bool OpenFloorMeasurementController::AppendSegment(const MainSegment& segment)
    {
        if (_mainStage.planSize >= _mainStage.plan.size())
        {
            return false;
        }

        _mainStage.plan[_mainStage.planSize++] = segment;
        return true;
    }

    bool OpenFloorMeasurementController::AppendCompiledManeuverSegment(
        MainSegment segment,
        const MazeMap::ManeuverInstance& maneuver)
    {
        std::uint16_t maneuverIndex = 0U;
        if (!StoreCompiledManeuver(maneuver, maneuverIndex))
        {
            return false;
        }

        segment.drivePrimitive.kind = DrivePrimitiveKind::Maneuver;
        segment.drivePrimitive.maneuver.maneuverIndex = maneuverIndex;
        return AppendSegment(segment);
    }

    bool OpenFloorMeasurementController::StoreCompiledManeuver(
        const MazeMap::ManeuverInstance& maneuver,
        std::uint16_t& maneuverIndex)
    {
        if (_mainStage.maneuverCount >= _mainStage.maneuvers.size())
        {
            return false;
        }

        maneuverIndex = _mainStage.maneuverCount;
        _mainStage.maneuvers[_mainStage.maneuverCount++] = maneuver;
        return true;
    }

    bool OpenFloorMeasurementController::CompileWheelCommandSweep(
        const WheelCommandSweepDefinition& definition)
    {
        if ((definition.magnitudes == nullptr) || (definition.magnitudeCount == 0U))
        {
            return false;
        }

        const std::size_t signCount = definition.alternateSign ? 2U : 1U;
        std::uint16_t repeatIndex = 0U;
        for (std::size_t magnitudeIndex = 0U; magnitudeIndex < definition.magnitudeCount; ++magnitudeIndex)
        {
            const float magnitude = definition.magnitudes[magnitudeIndex];
            for (std::uint8_t repeat = 0U; repeat < definition.repeatsPerMagnitude; ++repeat)
            {
                for (std::size_t signIndex = 0U; signIndex < signCount; ++signIndex)
                {
                    const float sign = (signIndex == 0U) ? 1.0f : -1.0f;
                    MainSegment segment{};
                    segment.executor = &kWheelCommandProfileExecutor;
                    segment.identity = SegmentIdentity(
                        definition.phaseId,
                        definition.primitiveId,
                        OpenFloorSpeedBin::None,
                        ++repeatIndex);
                    segment.settlingHoldMs = definition.settlingHoldMs;
                    segment.wheelCommandProfile.durationMs = definition.durationMs;
                    segment.wheelCommandProfile.leftCommand = magnitude * sign * definition.leftScale;
                    segment.wheelCommandProfile.rightCommand = magnitude * sign * definition.rightScale;
                    if (!AppendSegment(segment))
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool OpenFloorMeasurementController::CompileStraightSweep(const StraightSweepDefinition& definition)
    {
        if ((definition.speedsMps == nullptr) || (definition.speedCount == 0U))
        {
            return false;
        }

        const std::size_t directionCount = definition.alternateDirection ? 2U : 1U;
        std::uint16_t repeatIndex = 0U;
        for (std::size_t speedIndex = 0U; speedIndex < definition.speedCount; ++speedIndex)
        {
            const float speedMps = definition.speedsMps[speedIndex];
            const OpenFloorSpeedBin speedBin = SpeedBinForIndex(speedIndex);
            for (std::uint8_t repeat = 0U; repeat < definition.repeatsPerSpeed; ++repeat)
            {
                for (std::size_t directionIndex = 0U; directionIndex < directionCount; ++directionIndex)
                {
                    const float direction = (directionIndex == 0U) ? 1.0f : -1.0f;
                    MainSegment segment{};
                    segment.executor = &kDrivePrimitiveExecutor;
                    segment.identity = SegmentIdentity(
                        definition.phaseId,
                        definition.primitiveId,
                        speedBin,
                        ++repeatIndex);
                    segment.settlingHoldMs = definition.settlingHoldMs;
                    segment.drivePrimitive.kind = DrivePrimitiveKind::Straight;
                    segment.drivePrimitive.straight.distanceM = definition.distanceM;
                    segment.drivePrimitive.straight.speedMps = direction * speedMps;
                    if (!AppendSegment(segment))
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool OpenFloorMeasurementController::CompileTurnSweep(const TurnSweepDefinition& definition)
    {
        if ((definition.primitiveIds == nullptr) ||
            (definition.nominalAnglesRad == nullptr) ||
            (definition.primitiveCount == 0U) ||
            (definition.omegaBinsRadps == nullptr) ||
            (definition.omegaBinCount == 0U))
        {
            return false;
        }

        std::uint16_t repeatIndex = 0U;
        for (std::size_t speedIndex = 0U; speedIndex < definition.omegaBinCount; ++speedIndex)
        {
            const OpenFloorSpeedBin speedBin = SpeedBinForIndex(speedIndex);
            for (std::uint8_t repeat = 0U; repeat < definition.repeatsPerOmegaBin; ++repeat)
            {
                for (std::size_t primitiveIndex = 0U; primitiveIndex < definition.primitiveCount; ++primitiveIndex)
                {
                    MainSegment segment{};
                    segment.executor = &kDrivePrimitiveExecutor;
                    segment.identity = SegmentIdentity(
                        definition.phaseId,
                        definition.primitiveIds[primitiveIndex],
                        speedBin,
                        ++repeatIndex);
                    segment.settlingHoldMs = definition.settlingHoldMs;
                    segment.drivePrimitive.kind = DrivePrimitiveKind::Turn;
                    segment.drivePrimitive.turn.yawRad = definition.nominalAnglesRad[primitiveIndex];
                    segment.drivePrimitive.turn.maxOmegaRadps = definition.omegaBinsRadps[speedIndex];
                    if (!AppendSegment(segment))
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    bool OpenFloorMeasurementController::CompileManeuverQueue(
        const ManeuverQueueDefinition& definition)
    {
        switch (definition.kind)
        {
        case ManeuverQueueKind::SmoothSweep:
        {
            if ((definition.speedBinsMps == nullptr) || (definition.speedCount == 0U))
            {
                return false;
            }

            float entryBoundarySpeedMps = 0.0f;
            for (std::size_t speedIndex = 0U; speedIndex < definition.speedCount; ++speedIndex)
            {
                MazeMap::ManeuverQueue queue{};
                float exitBoundarySpeedMps = 0.0f;
                const bool isLastSpeedBin = (speedIndex + 1U) == definition.speedCount;
                if (!BuildOpenFloorMeasurementSmoothQueue(
                        _vehicle,
                        static_cast<std::uint8_t>(speedIndex),
                        definition.speedBinsMps[speedIndex],
                        entryBoundarySpeedMps,
                        queue,
                        exitBoundarySpeedMps))
                {
                    return false;
                }

                const OpenFloorSpeedBin speedBin = SpeedBinForIndex(speedIndex);
                for (std::size_t entryIndex = 0U; entryIndex < queue.size(); ++entryIndex)
                {
                    const MazeMap::ManeuverInstance& maneuver =
                        queue[static_cast<std::uint16_t>(entryIndex)];
                    const bool isLastQueueEntry = (entryIndex + 1U) == queue.size();
                    const bool isClosingSpeedChange = isLastSpeedBin && isLastQueueEntry;
                    const OpenFloorPrimitiveId primitiveId =
                        (entryIndex == 0U) || isClosingSpeedChange ?
                            kOpenFloorMeasurementSpeedChangeStraightPrimitive :
                            kOpenFloorMeasurementSmoothCycle[entryIndex - 1U].primitiveId;
                    MainSegment segment{};
                    segment.executor = &kDrivePrimitiveExecutor;
                    segment.identity = SegmentIdentity(definition.phaseId, primitiveId, speedBin, 1U);
                    segment.settlingHoldMs =
                        (isLastSpeedBin && isLastQueueEntry) ? definition.settlingHoldMs : 0U;
                    segment.drivePrimitive.maneuver.speedMps = definition.speedBinsMps[speedIndex];
                    if (!AppendCompiledManeuverSegment(segment, maneuver))
                    {
                        return false;
                    }
                }

                entryBoundarySpeedMps = exitBoundarySpeedMps;
            }

            return true;
        }
        case ManeuverQueueKind::Loop:
        {
            MazeMap::ManeuverQueue queue{};
            if (!BuildOpenFloorMeasurementLoopQueue(_vehicle, definition.clockwise, queue))
            {
                return false;
            }

            for (std::uint16_t repeatIndex = 1U; repeatIndex <= definition.repeatCount; ++repeatIndex)
            {
                for (std::size_t entryIndex = 0U; entryIndex < queue.size(); ++entryIndex)
                {
                    const MazeMap::ManeuverInstance& maneuver =
                        queue[static_cast<std::uint16_t>(entryIndex)];
                    const bool isLastEntry = (entryIndex + 1U) == queue.size();
                    MainSegment segment{};
                    segment.executor = &kDrivePrimitiveExecutor;
                    segment.identity = SegmentIdentity(
                        definition.phaseId,
                        ((entryIndex % 2U) == 0U) ?
                            kOpenFloorMeasurementLoopStraightPrimitive :
                            (definition.clockwise ? OpenFloorPrimitiveId::Ip90 : OpenFloorPrimitiveId::Ip90M),
                        OpenFloorSpeedBin::Low,
                        repeatIndex);
                    segment.settlingHoldMs =
                        ((repeatIndex == definition.repeatCount) && isLastEntry) ? definition.settlingHoldMs : 0U;
                    segment.drivePrimitive.maneuver.speedMps = MazeMap::kOpenFloorStraightSpeedBinsMps[0];
                    if (!AppendCompiledManeuverSegment(segment, maneuver))
                    {
                        return false;
                    }
                }
            }

            return true;
        }
        default:
            return false;
        }
    }

    bool OpenFloorMeasurementController::CheckFault(
        LoopController::TickServices& services,
        const bool mainStage)
    {
        if (SelectorRemoved())
        {
            services.Fault(kOpenFloorMeasurementSelectorRemovedReason);
            return true;
        }
        if (!_runtime.Estimator().HasFault())
        {
            return false;
        }
        if (!mainStage)
        {
            services.Fault("Estimator fault during timing capture");
            return true;
        }

        std::snprintf(
            _mainStage.estimatorFaultReason,
            sizeof(_mainStage.estimatorFaultReason),
            "Estimator fault during open-floor phase %u",
            static_cast<unsigned>(
                (ActiveMainSegment() != nullptr) ?
                    ActiveMainSegment()->identity.phaseId :
                    OpenFloorSectionId::Sec00Timing));
        services.Fault(_mainStage.estimatorFaultReason);
        return true;
    }

    void OpenFloorMeasurementController::AdvanceMainSegment() noexcept
    {
        ++_mainStage.nextSegmentIndex;
        _mainStage.activeRuntime = {};
        if (_mainStage.nextSegmentIndex >= _mainStage.planSize)
        {
            _mainStage.completionPending = true;
        }
    }

    const OpenFloorMeasurementController::MainSegment* OpenFloorMeasurementController::ActiveMainSegment() const noexcept
    {
        return (_mainStage.nextSegmentIndex < _mainStage.planSize) ?
            &_mainStage.plan[_mainStage.nextSegmentIndex] :
            nullptr;
    }

    LoopController::PauseDisposition OpenFloorMeasurementController::OnPauseGranted(
        const LoopController::PauseContext& pause)
    {
        (void)pause;
        if (_pauseAction != PauseAction::TimingToMain)
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement pause granted without a pending timing transition");
        }

        if (_timingStage.pendingSampleValid)
        {
            ApplyControlTimingToTimingRow(_loopController.LastDiagnostics().controlTiming, _timingStage.pendingRow);
            if (!_runtime.LogUtilityDataRow(_timingStage.pendingRow))
            {
                return LoopController::PauseDisposition::StopByRuntime(
                    "Open-floor measurement timing log write failed during timing transition");
            }
            _timingStage.pendingSampleValid = false;
            _timingStage.pendingRow = {};
        }

        _timingStage.logOpen = false;
        if (!_mainStage.logOpen && !BeginMainLog())
        {
            return LoopController::PauseDisposition::StopByRuntime(
                "Open-floor measurement main log setup failed after timing capture");
        }

        _activeStageTick = &OpenFloorMeasurementController::MainStageTick;
        _pauseAction = PauseAction::None;
        return LoopController::PauseDisposition::Resume();
    }

    LoopController::ControlVector OpenFloorMeasurementController::TimingStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (_timingStage.pendingSampleValid)
        {
            ApplyControlTimingToTimingRow(_loopController.LastDiagnostics().controlTiming, _timingStage.pendingRow);
            if (!_runtime.LogUtilityDataRow(_timingStage.pendingRow))
            {
                services.Fault("Open-floor measurement timing log write failed");
                return stopControl;
            }

            _timingStage.pendingSampleValid = false;
            _timingStage.pendingRow = {};
        }
        if (CheckFault(services, false))
        {
            return stopControl;
        }

        OpenFloorTimingRow row{};
        PopulateTimingRowFromState(state, row);
        StagePendingTimingSample(row);
        ++_timingStage.tickIndex;
        if (_timingStage.tickIndex >= DiagnosticConfig::kTimingCaptureCycles)
        {
            _pauseAction = PauseAction::TimingToMain;

            LoopController::PauseRequest request{};
            request.onPauseGranted = &OpenFloorMeasurementController::PauseThunk;
            request.reason = "open_floor_timing_to_main";
            request.flushLogsBeforeGrant = true;
            request.resetClockOnResume = true;
            services.RequestPause(request);
        }

        return stopControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::MainStageTick(
        const std::uint32_t loopEndTimeUs,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        if (!CommitPendingMainSample(services, "Open-floor measurement main log write failed"))
        {
            return stopControl;
        }

        const MainSegment* const segment = ActiveMainSegment();
        if (_mainStage.completionPending || (segment == nullptr))
        {
            services.RequestEndLoop();
            return stopControl;
        }

        if (CheckFault(services, true))
        {
            return stopControl;
        }

        SegmentTickResult result{};
        const LoopController::ControlVector control =
            segment->executor->tick(*this, _mainStage.activeRuntime, *segment, state, services, result);
        OpenFloorMainRow row{};
        PopulateMainRowFromState(segment->identity, state, row);
        StagePendingMainSample(row);

        if (result.done)
        {
            AdvanceMainSegment();
            return stopControl;
        }

        return control;
    }

    LoopController::ControlVector OpenFloorMeasurementController::ExecuteHoldSegment(
        OpenFloorMeasurementController& controller,
        SegmentRuntime& runtime,
        const MainSegment& segment,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        SegmentTickResult& result)
    {
        (void)state;
        (void)services;
        HoldSegmentRuntime& holdRuntime = runtime.hold;

        if (!holdRuntime.started)
        {
            holdRuntime = {};
            holdRuntime.started = true;
            controller._driveService.SetLimits(BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f));
            controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
            controller._driveService.StartHold(segment.hold.durationMs, false);
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = controller._driveService.GetNextControls(done);
        result.done = done;
        if (done)
        {
            holdRuntime = {};
            return LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);
        }

        return candidateControl;
    }

    LoopController::ControlVector OpenFloorMeasurementController::ExecuteWheelCommandProfileSegment(
        OpenFloorMeasurementController& controller,
        SegmentRuntime& runtime,
        const MainSegment& segment,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        SegmentTickResult& result)
    {
        WheelCommandProfileRuntime& wheelRuntime = runtime.wheelCommandProfile;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);

        if (wheelRuntime.settling)
        {
            MainSegment settlingSegment = segment;
            settlingSegment.hold.durationMs = segment.settlingHoldMs;
            const LoopController::ControlVector control =
                ExecuteHoldSegment(controller, runtime, settlingSegment, state, services, result);
            if (result.done)
            {
                wheelRuntime = {};
            }
            return control;
        }

        if (!wheelRuntime.started)
        {
            wheelRuntime = {};
            wheelRuntime.started = true;
            wheelRuntime.deadlineMs = millis() + segment.wheelCommandProfile.durationMs;
        }

        if (static_cast<long>(wheelRuntime.deadlineMs - millis()) <= 0)
        {
            if (segment.settlingHoldMs == 0U)
            {
                result.done = true;
                wheelRuntime = {};
            }
            else
            {
                wheelRuntime.settling = true;
                runtime.hold = {};
            }
            return stopControl;
        }

        return LoopController::ControlVector::RawMotorPwm(
            segment.wheelCommandProfile.leftCommand,
            segment.wheelCommandProfile.rightCommand);
    }

    LoopController::ControlVector OpenFloorMeasurementController::ExecuteDrivePrimitiveSegment(
        OpenFloorMeasurementController& controller,
        SegmentRuntime& runtime,
        const MainSegment& segment,
        const MazeMap::VehicleState& state,
        LoopController::TickServices& services,
        SegmentTickResult& result)
    {
        (void)state;
        DrivePrimitiveRuntime& driveRuntime = runtime.drivePrimitive;
        const DrivePrimitivePayload& payload = segment.drivePrimitive;
        const LoopController::ControlVector stopControl = LoopController::ControlVector::RawMotorPwm(0.0f, 0.0f);

        if (driveRuntime.settling)
        {
            MainSegment settlingSegment = segment;
            settlingSegment.hold.durationMs = segment.settlingHoldMs;
            const LoopController::ControlVector control =
                ExecuteHoldSegment(controller, runtime, settlingSegment, state, services, result);
            if (result.done)
            {
                driveRuntime = {};
            }
            return control;
        }

        if (!driveRuntime.started)
        {
            driveRuntime = {};
            driveRuntime.started = true;

            switch (payload.kind)
            {
            case DrivePrimitiveKind::Straight:
                controller._driveService.SetLimits(
                    BuildOpenFloorMeasurementLimits(controller._vehicle, std::fabs(payload.straight.speedMps)));
                controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                controller._driveService.StartStraight(
                    payload.straight.distanceM,
                    payload.straight.speedMps,
                    0.0f);
                driveRuntime.straight.startDistanceM = controller._drive.GetAverageDistanceMeters();
                driveRuntime.straight.totalDistanceM = payload.straight.distanceM;
                break;
            case DrivePrimitiveKind::Turn:
            {
                MotionLimits limits = BuildOpenFloorMeasurementLimits(controller._vehicle, 0.0f);
                limits.maxAngularSpeedRadps = payload.turn.maxOmegaRadps;
                controller._driveService.SetLimits(
                    limits);
                controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                controller._driveService.StartTurn(payload.turn.yawRad);
                driveRuntime.turn.targetYawRad = WrapAngleRad(
                    controller._runtime.RuntimeState().GetOrientation() + payload.turn.yawRad);
                driveRuntime.turn.targetMagnitudeRad = std::fabs(payload.turn.yawRad);
                break;
            }
            case DrivePrimitiveKind::Maneuver:
            {
                if (payload.maneuver.maneuverIndex >= controller._mainStage.maneuverCount)
                {
                    services.Fault("Open-floor compiled maneuver segment referenced an invalid maneuver");
                    result.done = true;
                    return LoopController::ControlVector::Brake;
                }

                const MazeMap::ManeuverInstance& maneuver =
                    controller._mainStage.maneuvers[payload.maneuver.maneuverIndex];
                controller._driveService.SetLimits(
                    BuildOpenFloorMeasurementLimits(controller._vehicle, payload.maneuver.speedMps));
                controller._driveService.SetOperationMode(Drive::OperationMode::OpenFloor);
                controller._driveService.StartManeuver(maneuver);
                driveRuntime.maneuver.startDistanceM = controller._drive.GetAverageDistanceMeters();
                driveRuntime.maneuver.totalDistanceM = maneuver.GetTravelDistanceMeters();
                driveRuntime.maneuver.targetYawRad = WrapAngleRad(
                    controller._runtime.RuntimeState().GetOrientation() +
                    (static_cast<float>(MazeMap::CodeDegrees(maneuver.getCode())) * DEG_TO_RAD_F));
                driveRuntime.maneuver.targetMagnitudeRad =
                    std::fabs(static_cast<float>(MazeMap::CodeDegrees(maneuver.getCode())) * DEG_TO_RAD_F);
                break;
            }
            default:
                services.Fault("Open-floor drive segment kind was invalid");
                result.done = true;
                return LoopController::ControlVector::Brake;
            }
        }

        bool done = false;
        const LoopController::ControlVector candidateControl = controller._driveService.GetNextControls(done);
        result.done = done;
        if (done)
        {
            if (segment.settlingHoldMs == 0U)
            {
                driveRuntime = {};
                return stopControl;
            }

            driveRuntime.settling = true;
            runtime.hold = {};
            result.done = false;
            return stopControl;
        }

        return candidateControl;
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
            "Inter-phase 500 ms brake holds; launch and straight samples insert 250 ms brake holds between motions; smooth phase uses the current hand-picked closed maneuver sequence; loop sections are maneuver-driven",
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
