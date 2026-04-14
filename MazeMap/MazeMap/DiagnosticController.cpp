#include "pch.h"
#include "MazeMapApplicationPrivate.h"
#include "DriveBase.h"
#include "LoopController.h"
#include "MazeMapRuntimeInfrastructure.h"
#include "MazeMapRuntimeMmLog.h"
#include "MazeMapSharedRuntime.h"
#include "OpenFloorMeasurementSpec.h"
#include "RuntimeBinaryLogSupport.h"
#include "WallSensorLedCalibrationPhase.h"

using MazeMap::App::Internal::GetSharedRobotRuntime;
using MazeMap::App::Internal::SharedRobotRuntime;

namespace
{
    // This legacy controller is no longer registered as the primary diagnostic mode; keep
    // its old sweep constants private so DiagnosticConfig only exposes active open-floor dials.
    namespace LegacyDiagnosticConfig
    {
        constexpr float kShortStraightDistanceM = 0.18f;
        constexpr float kLongStraightDistanceM = 0.27f;
        constexpr float kSquareLegDistanceM = 0.18f;
        constexpr float kArcHalfCircleDistanceM = 0.20f;
        constexpr float kSlowStraightSpeedMps = 0.4f;
        constexpr float kCircleMediumSpeedMps = 0.6f;
        constexpr float kFastStraightSpeedMps = 0.8f;
        constexpr float kKickoffSweepMoveThresholdM = 0.03f;
        constexpr float kKickoffSweepMoveThresholdMps = 0.03f;
        constexpr uint16_t kKickoffSweepPulseMs = 250U;
        constexpr float kForwardSweepKickoffDriveCommand = 0.35f;
        constexpr uint16_t kForwardSweepKickoffMs = 120U;
        constexpr float kForwardSweepMinDriveCommand = 0.10f;
        constexpr float kForwardSweepMaxDriveCommand = 0.40f;
        constexpr float kForwardSweepStepDriveCommand = 0.01f;
        constexpr uint16_t kForwardSweepHoldMs = 220U;
        constexpr float kForwardSweepCarryThresholdMps = 0.05f;
        constexpr float kForwardSweepCarryThresholdM = 0.180f;
    }
}

class DiagnosticController : public IApplicationMode
{
public:
    explicit DiagnosticController(SharedRobotRuntime& runtime)
        : _runtime(runtime)
        , _loopController(runtime.ControlLoop())
        , _vehicle(runtime.SpeedVehicle())
        , _sensors(runtime.DiagnosticSensors())
        , _drive(runtime.Drive())
        , _startX(0.0f)
        , _startY(0.0f)
        , _faulted(false)
        , _phaseId(0UL)
        , _sampleCount(0UL)
    {
        _logFileName[0] = '\0';
    }

    bool Begin() override
    {
        if (!_runtime.RegisterModeFaultHandler(&DiagnosticController::HandleRuntimeFault, this, "diagnostic"))
        {
            return false;
        }

        if (!SetupHardware())
        {
            return Fail("Hardware setup failed");
        }
        ResetStartupTrace("mode:primary_diagnostic");
        (void)_runtime.AppendTextLogLine("Micromouse diagnostic setup");
        if (!_drive.Begin())
        {
            return Fail("Drive base init failed");
        }
        _drive.SetWheelControlProfile(BuildDiagnosticWheelControlProfile());
        SetMissionLevelFanEnabled(true);
        gWallDistanceCalibration.Clear();
        if (!_sensors.Begin(DiagnosticConfig::kControlPeriodUs))
        {
            return Fail("Diagnostic sensor init failed");
        }
        if (!BeginLog())
        {
            return Fail("Diagnostic log open failed");
        }

        _drive.SetStartPoint(MazeMap::DirectionalLocation(MazeMap::MazeLocation::CellCenter(MazeMap::CellCoordinates(0, 0)), MazeMap::Up));
        _startX = _drive.GetPose().xMeters;
        _startY = _drive.GetPose().yMeters;

        return true;
    }

    void Run() override
    {
        if (_faulted)
        {
            _loopController.EndSession();
            return;
        }

        bool ok = true;
        ok = ok && HoldPhase("startup_settle", DiagnosticConfig::kStartupSettleMs, true);
        ok = ok && HoldPhase("baseline_idle", DiagnosticConfig::kBaselineHoldMs, true);
        ok = ok && ExecuteKickoffSweep();
        ok = ok && ExecuteForwardSweep();
        ok = ok && HoldPhase("characterization_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteTurnPhase("turn_cw_90_1", HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_1", -HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_cw_90_2", HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_90_2", -HALF_PI_F);
        ok = ok && ExecuteTurnPhase("turn_cw_180", PI_F);
        ok = ok && ExecuteTurnPhase("turn_ccw_180", -PI_F);
        ok = ok && HoldPhase("turn_sweep_settle", DiagnosticConfig::kInterTestHoldMs, true);
        float shortReturnDistanceM = LegacyDiagnosticConfig::kShortStraightDistanceM;
        ok = ok && ExecuteStraightPhase("straight_short_forward", LegacyDiagnosticConfig::kShortStraightDistanceM, LegacyDiagnosticConfig::kSlowStraightSpeedMps, &shortReturnDistanceM);
        shortReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(LegacyDiagnosticConfig::kShortStraightDistanceM, shortReturnDistanceM);
        ok = ok && ExecuteTurnPhase("straight_short_turnaround", PI_F);
        ok = ok && ExecuteStraightPhase("straight_short_return", shortReturnDistanceM, LegacyDiagnosticConfig::kSlowStraightSpeedMps);
        ok = ok && ExecuteTurnPhase("straight_short_reset_heading", PI_F);
        ok = ok && HoldPhase("straight_short_settle", DiagnosticConfig::kInterTestHoldMs, true);
        float longReturnDistanceM = LegacyDiagnosticConfig::kLongStraightDistanceM;
        ok = ok && ExecuteStraightPhase("straight_long_forward", LegacyDiagnosticConfig::kLongStraightDistanceM, LegacyDiagnosticConfig::kFastStraightSpeedMps, &longReturnDistanceM);
        longReturnDistanceM = MazeMap::SelectDiagnosticReturnDistanceM(LegacyDiagnosticConfig::kLongStraightDistanceM, longReturnDistanceM);
        ok = ok && ExecuteTurnPhase("straight_long_turnaround", PI_F);
        ok = ok && ExecuteStraightPhase("straight_long_return", longReturnDistanceM, LegacyDiagnosticConfig::kFastStraightSpeedMps);
        ok = ok && ExecuteTurnPhase("straight_long_reset_heading", PI_F);
        ok = ok && HoldPhase("straight_long_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteCircleSpeedSweep("slow", LegacyDiagnosticConfig::kSlowStraightSpeedMps);
        ok = ok && ExecuteCircleSpeedSweep("medium", LegacyDiagnosticConfig::kCircleMediumSpeedMps);
        ok = ok && ExecuteCircleSpeedSweep("fast", LegacyDiagnosticConfig::kFastStraightSpeedMps);
        ok = ok && ExecuteSquareLoop("square_cw", HALF_PI_F);
        ok = ok && HoldPhase("square_cw_settle", DiagnosticConfig::kInterTestHoldMs, true);
        ok = ok && ExecuteSquareLoop("square_ccw", -HALF_PI_F);
        ok = ok && HoldPhase("final_idle", DiagnosticConfig::kBaselineHoldMs / 2U, true);

        _drive.Brake();
        _drive.UseNominalWheelControlProfile();

        if (ok)
        {
            (void)_runtime.AppendTextLogFormatted("Diagnostic complete, log saved to %s", GetLogFileName());
            (void)_runtime.AppendTextLogLine("Use the # event,summary lines in the log header to map phases to tunables.");
        }

        CloseLog();
        SetMissionLevelFanEnabled(false);
    }

private:
    using LoopController = MazeMap::App::Internal::LoopController;
    using PhaseFn = LoopController::ControlVector (DiagnosticController::*)(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services);

    static void HandleRuntimeFault(void* context, const char* reason) noexcept
    {
        if (context == nullptr)
        {
            return;
        }

        static_cast<DiagnosticController*>(context)->OnRuntimeFault(reason);
    }

    SharedRobotRuntime& _runtime;
    LoopController& _loopController;
    struct StraightPhaseMetrics
    {
        float peakSpeedMps = 0.0f;
        float maxHeadingErrorRad = 0.0f;
    };

    struct TurnPhaseMetrics
    {
        float peakOmegaRadps = 0.0f;
        float maxYawErrorRad = 0.0f;
    };

    struct ArcPhaseMetrics
    {
        float peakSpeedMps = 0.0f;
        float peakOmegaRadps = 0.0f;
        float maxHeadingErrorRad = 0.0f;
        float durationSeconds = 0.0f;
        float omegaIntegralRad = 0.0f;
        float speedIntegralMpsSeconds = 0.0f;
        float planarAccelIntegralMps2Seconds = 0.0f;
        float peakPlanarAccelMps2 = 0.0f;
    };

    MazeMap::Vehicle& _vehicle;
    DiagnosticSensorSuite& _sensors;
    DriveBase& _drive;
    char _logFileName[64];
    float _startX;
    float _startY;
    bool _faulted;
    unsigned long _phaseId;
    unsigned long _sampleCount;
    PhaseFn _phaseFn{};

    struct HoldPhaseState final
    {
        bool stationary{};
        unsigned long deadlineMs{};
        bool started{};
    } _holdPhaseState{};

    struct StraightPhaseState final
    {
        const char* phaseName{};
        float distanceM{};
        float cruiseSpeedMps{};
        float startDistanceM{};
        Eigen::Vector2f targetHeading = Eigen::Vector2f(0.0f, 1.0f);
        float commandedSpeedMps{};
        float traveledM{};
        unsigned long timeoutMs{};
        EncoderProgressWatchdog translationWatchdog{};
        StraightPhaseMetrics metrics{};
        float* outTraveledDistanceM{};
        bool started{};
    } _straightPhaseState{};

    struct KickoffPhaseState final
    {
        char label[24]{};
        float driveCommand{};
        float startDistanceM{};
        unsigned long pulseDeadlineMs{};
        unsigned long settleDeadlineMs{};
        float travelLimitM{};
        float maxSpeedMps{};
        bool travelLimited{};
        unsigned long travelLimitSettleDeadlineMs{};
        bool started{};
    } _kickoffPhaseState{};

    struct ForwardPhaseState final
    {
        char label[24]{};
        float forwardDriveCommand{};
        float startDistanceM{};
        unsigned long kickoffDeadlineMs{};
        unsigned long holdDeadlineMs{};
        unsigned long settleDeadlineMs{};
        float travelLimitM{};
        float maxSpeedMps{};
        float holdStartDistanceM{};
        float holdEndDistanceM{};
        float holdElapsedSeconds{};
        bool holdStarted{};
        bool holdComplete{};
        bool travelLimited{};
        unsigned long travelLimitSettleDeadlineMs{};
        bool started{};
    } _forwardPhaseState{};

    struct TurnPhaseState final
    {
        const char* phaseName{};
        float angleRad{};
        float targetYawRad{};
        MazeMap::InPlaceTurnProfile turnProfile{};
        unsigned long timeoutMs{};
        TurnPhaseMetrics metrics{};
        bool started{};
    } _turnPhaseState{};

    struct ArcPhaseState final
    {
        const char* phaseName{};
        float distanceM{};
        float angleRad{};
        float cruiseSpeedMps{};
        ArcPhaseMetrics* outMetrics{};
        MotionLimits limits{};
        float startDistanceM{};
        float startYawRad{};
        float targetYawRad{};
        float curvature{};
        float commandedSpeedMps{};
        float traveledM{};
        unsigned long timeoutMs{};
        EncoderProgressWatchdog translationWatchdog{};
        ArcPhaseMetrics metrics{};
        bool started{};
    } _arcPhaseState{};

    LoopController::SessionOptions BuildLoopOptions() const
    {
        LoopController::SessionOptions options{};
        options.controlPeriodUs = DiagnosticConfig::kControlPeriodUs;
        return options;
    }

    static LoopController::ControlVector ModeWorkThunk(
        void* context,
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        auto* const self = static_cast<DiagnosticController*>(context);
        if ((self == nullptr) || (self->_phaseFn == nullptr))
        {
            services.Fault("Diagnostic phase callback was not installed");
            return LoopController::ControlVector::BrakeCommand();
        }

        if (!state.estimatorHealthy)
        {
            services.Fault((state.faultReason != nullptr) ? state.faultReason : "Diagnostic estimator fault");
            return LoopController::ControlVector::BrakeCommand();
        }

        if (!self->IsWithinBoundary(state.estimate))
        {
            services.Fault("Diagnostic boundary exceeded");
            return LoopController::ControlVector::BrakeCommand();
        }

        return (self->*self->_phaseFn)(loopEndTimeUs, state, services);
    }

    bool RunPhaseSession(PhaseFn phase)
    {
        LoopController::ModeCallbacks callbacks{};
        callbacks.onModeWork = &DiagnosticController::ModeWorkThunk;
        callbacks.context = this;
        _phaseFn = phase;

        if (!_loopController.BeginSession(BuildLoopOptions(), callbacks))
        {
            _phaseFn = nullptr;
            return Fail("Diagnostic loop session start failed");
        }

        const LoopController::SessionResult result = _loopController.Run();
        _phaseFn = nullptr;
        return (result.status == LoopController::SessionResult::Status::Completed) && !_faulted;
    }

    bool BeginLog()
    {
        _phaseId = 0UL;
        _sampleCount = 0UL;
        if (!_runtime.OpenUtilityDataLog(
                _logFileName,
                sizeof(_logFileName),
                nullptr,
                "diag%03u.mmlog",
                "diagnostic_log.mmlog"))
        {
            return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadata("mode", "diagnostic")) return false;
        if (!_runtime.WriteUtilityDataLogMetadataUnsigned("control_period_us", DiagnosticConfig::kControlPeriodUs)) return false;
        {
            const unsigned long imuSampleRateHz = MazeMap::GetUiImuSampleRateHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if (imuSampleRateHz > 0UL && !_runtime.WriteUtilityDataLogMetadataUnsigned("imu_sample_rate_hz", imuSampleRateHz)) return false;
        }
        {
            const float imuAccelLpf2CutoffHz = MazeMap::GetUiAccelLpf2CutoffHzForControlPeriodUs(
                DiagnosticConfig::kControlPeriodUs,
                Config::kMissionRuntimeAccelFilterFreq);
            if (imuAccelLpf2CutoffHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_lpf2_cutoff_hz", imuAccelLpf2CutoffHz, 3)) return false;
        }
        {
            const float imuGyroLpf1ReferenceHz = MazeMap::GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(DiagnosticConfig::kControlPeriodUs);
            if (imuGyroLpf1ReferenceHz > 0.0f && !_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_lpf1_cut213_datasheet_ref_hz", imuGyroLpf1ReferenceHz, 3)) return false;
        }
        if (!_runtime.WriteUtilityDataLogMetadataFloat("boundary_half_span_m", DiagnosticConfig::kBoundaryHalfSpanM, 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_gyro_mdps_per_lsb", _sensors.GetGyroSensitivityMdpsPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("imu_accel_mg_per_lsb", _sensors.GetAccelSensitivityMgPerLsb(), 3)) return false;
        if (!_runtime.WriteUtilityDataLogMetadataFloat("mission_gyro_bias_estimate_radps", _sensors.GetGyroBiasRadps(), 6)) return false;
        if (!_runtime.WriteUtilityDataLogAccelBiasMetadata(_sensors)) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("format_spec", "micromouse_logging_spec_rev_g")) return false;
        if (!_runtime.WriteUtilityDataLogMetadata("endianness", "little")) return false;
        if (!MazeMap::App::Internal::Runtime::WriteDiagnosticTuningEvents(
                [this](const char* type, const char* message) -> bool
                {
                    return _runtime.WriteTextLogEntry(micros(), type, message);
                })) return false;

        DiagnosticLogRow row{};
        if (!_runtime.BeginUtilityDataLogSchema(row))
        {
            return false;
        }

        if (!_runtime.WriteTextLogMetadata("file", _runtime.TextLogFileName())) return false;
        if (!_runtime.WriteTextLogMetadata("data_file", _logFileName)) return false;
        if (!_runtime.WriteTextLogMetadata("mode", "diagnostic")) return false;
        return MazeMap::App::Internal::Runtime::WriteDiagnosticSummaryInstructions(
            [this](const char* type, const char* message) -> bool
            {
                return _runtime.WriteTextLogEntry(micros(), type, message);
            });
    }

    bool WriteLogEvent(const char* type, const char* message)
    {
        return _runtime.WriteTextLogEntry(micros(), type, message);
    }

    bool WritePhaseMarker(const char* name)
    {
        ++_phaseId;
        return _runtime.WriteTextLogPhase(_phaseId, micros(), name);
    }

    void ServiceLog()
    {
        (void)_runtime.ServiceUtilityDataLog();
    }

    void CloseLog()
    {
        (void)_runtime.CloseUtilityDataLog();
        _runtime.CloseTextLog();
    }

    const char* GetLogFileName() const
    {
        return _logFileName;
    }

    bool WriteStraightResult(
        const char* phaseName,
        float distanceM,
        float cruiseSpeedMps,
        float traveledM,
        const Eigen::Vector2f& targetHeading,
        const StraightPhaseMetrics& metrics)
    {
        char message[192] = {};
        const float stopErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG_F * HeadingErrorRad(targetHeading, _drive.GetPose().headingUnit);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;distance_m=%.3f;cruise_mps=%.3f;peak_speed_mps=%.3f;max_heading_err_deg=%.2f;stop_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            cruiseSpeedMps,
            metrics.peakSpeedMps,
            RAD_TO_DEG_F * metrics.maxHeadingErrorRad,
            stopErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format straight diagnostic result");
        }
        return WriteEventOrFail("straight_result", message, "Failed to write straight diagnostic result");
    }

    bool WriteTurnResult(const char* phaseName, float angleRad, const TurnPhaseMetrics& metrics, float targetYawRad)
    {
        char message[176] = {};
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;angle_deg=%.1f;peak_omega_radps=%.3f;peak_yaw_err_deg=%.2f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            RAD_TO_DEG_F * angleRad,
            metrics.peakOmegaRadps,
            RAD_TO_DEG_F * metrics.maxYawErrorRad,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format turn diagnostic result");
        }
        return WriteEventOrFail("turn_result", message, "Failed to write turn diagnostic result");
    }

    bool WriteArcResult(
        const char* phaseName,
        float distanceM,
        float angleRad,
        float traveledM,
        float targetYawRad,
        const ArcPhaseMetrics& metrics)
    {
        char message[192] = {};
        const float distanceErrorM = traveledM - distanceM;
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(targetYawRad, _drive.GetPose().yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;dist_m=%.3f;ang_deg=%.1f;peak_w_radps=%.3f;max_head_err_deg=%.2f;dist_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            distanceM,
            RAD_TO_DEG_F * angleRad,
            metrics.peakOmegaRadps,
            RAD_TO_DEG_F * metrics.maxHeadingErrorRad,
            distanceErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format arc diagnostic result");
        }
        return WriteEventOrFail("arc_result", message, "Failed to write arc diagnostic result");
    }

    bool WriteCircleResult(
        const char* phaseName,
        float cruiseSpeedMps,
        const DriveTelemetry& startTelemetry,
        const ArcPhaseMetrics& metrics)
    {
        const DriveTelemetry endTelemetry = _drive.GetTelemetry();
        const long leftCountDelta = static_cast<long>(endTelemetry.leftEncoderCount - startTelemetry.leftEncoderCount);
        const long rightCountDelta = static_cast<long>(endTelemetry.rightEncoderCount - startTelemetry.rightEncoderCount);
        const float leftDistanceDeltaM = endTelemetry.leftDistanceM - startTelemetry.leftDistanceM;
        const float rightDistanceDeltaM = endTelemetry.rightDistanceM - startTelemetry.rightDistanceM;
        const float averageOmegaRadps = (metrics.durationSeconds > 0.0f) ? (metrics.omegaIntegralRad / metrics.durationSeconds) : 0.0f;
        const float averageSpeedMps = (metrics.durationSeconds > 0.0f) ? (metrics.speedIntegralMpsSeconds / metrics.durationSeconds) : 0.0f;
        const float effectiveTrackWidthM =
            MazeMap::Vehicle::GetEffectiveTrackWidthForMotion(averageSpeedMps, averageOmegaRadps);
        const float encoderYawDeg =
            (effectiveTrackWidthM > 0.0f)
            ? (RAD_TO_DEG_F * ((rightDistanceDeltaM - leftDistanceDeltaM) / effectiveTrackWidthM))
            : 0.0f;
        const float estimatedLateralAccelMps2 = std::fabs(averageSpeedMps * averageOmegaRadps);
        const float averageLateralAccelMps2 = (metrics.durationSeconds > 0.0f) ? (metrics.planarAccelIntegralMps2Seconds / metrics.durationSeconds) : 0.0f;

        char message[256] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cruise_mps=%.3f;l_cnt=%ld;r_cnt=%ld;enc_yaw_deg=%.1f;avg_speed_mps=%.3f;avg_omega_radps=%.3f;est_lat_mps2=%.3f;avg_lat_mps2=%.3f;peak_lat_mps2=%.3f",
            (phaseName != nullptr) ? phaseName : "",
            cruiseSpeedMps,
            leftCountDelta,
            rightCountDelta,
            encoderYawDeg,
            averageSpeedMps,
            averageOmegaRadps,
            estimatedLateralAccelMps2,
            averageLateralAccelMps2,
            metrics.peakPlanarAccelMps2);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format circle diagnostic result");
        }
        return WriteEventOrFail("circle_result", message, "Failed to write circle diagnostic result");
    }

    bool WriteClosureResult(const char* type, const char* phaseName, const PoseEstimate& startPose, const char* failMessage)
    {
        char message[160] = {};
        const PoseEstimate& pose = _drive.GetPose();
        const float deltaXM = pose.xMeters - startPose.xMeters;
        const float deltaYM = pose.yMeters - startPose.yMeters;
        const float closureErrorM = std::sqrt((deltaXM * deltaXM) + (deltaYM * deltaYM));
        const float finalYawErrorDeg = RAD_TO_DEG_F * AngleErrorRad(startPose.yawRad, pose.yawRad);
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;closure_err_m=%.4f;final_yaw_err_deg=%.2f",
            (phaseName != nullptr) ? phaseName : "",
            closureErrorM,
            finalYawErrorDeg);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format diagnostic closure result");
        }
        return WriteEventOrFail(type, message, failMessage);
    }

    static void AccumulateArcMetrics(ArcPhaseMetrics& total, const ArcPhaseMetrics& segment)
    {
        total.peakSpeedMps = (std::max)(total.peakSpeedMps, segment.peakSpeedMps);
        total.peakOmegaRadps = (std::max)(total.peakOmegaRadps, segment.peakOmegaRadps);
        total.maxHeadingErrorRad = (std::max)(total.maxHeadingErrorRad, segment.maxHeadingErrorRad);
        total.durationSeconds += segment.durationSeconds;
        total.omegaIntegralRad += segment.omegaIntegralRad;
        total.speedIntegralMpsSeconds += segment.speedIntegralMpsSeconds;
        total.planarAccelIntegralMps2Seconds += segment.planarAccelIntegralMps2Seconds;
        total.peakPlanarAccelMps2 = (std::max)(total.peakPlanarAccelMps2, segment.peakPlanarAccelMps2);
    }

    static MotionLimits DiagnosticLimits(float maxSpeedMps)
    {
        MotionLimits limits{};
        limits.maxSpeedMps = maxSpeedMps;
        limits.accelMps2 = DiagnosticConfig::kStraightAccelMps2;
        limits.decelMps2 = DiagnosticConfig::kStraightDecelMps2;
        limits.maxAngularSpeedRadps = DiagnosticConfig::kTurnMaxOmegaRadps;
        limits.angularAccelRadps2 = DiagnosticConfig::kTurnAccelRadps2;
        return limits;
    }

    bool WriteEventOrFail(const char* type, const char* message, const char* failMessage)
    {
        if (WriteLogEvent(type, message))
        {
            return true;
        }

        return Fail(failMessage);
    }

    static void BuildDriveCommandLabel(const char* prefix, float driveCommand, char* buffer, size_t bufferSize)
    {
        const unsigned drivePercent = static_cast<unsigned>((100.0f * driveCommand) + 0.5f);
        snprintf(buffer, bufferSize, "%s_%03u", (prefix != nullptr) ? prefix : "cmd", drivePercent);
    }

    static MazeMap::WheelControlProfile BuildDiagnosticWheelControlProfile()
    {
        return BuildNominalWheelControlProfile();
    }

    bool Fail(const char* message)
    {
        return _runtime.FailActiveMode(message);
    }

    void OnRuntimeFault(const char* message) noexcept
    {
        _faulted = true;
        (void)WriteLogEvent("fault", message);
    }

    bool StartPhase(const char* name)
    {
        (void)_runtime.AppendTextLogFormatted("Diagnostic phase: %s", (name != nullptr) ? name : "unknown");
        if (WritePhaseMarker(name))
        {
            return true;
        }
        return Fail("Failed to write diagnostic phase marker");
    }

    bool IsWithinBoundary(const PoseEstimate& pose) const
    {
        return (std::fabs(pose.xMeters - _startX) <= DiagnosticConfig::kBoundaryHalfSpanM) &&
            (std::fabs(pose.yMeters - _startY) <= DiagnosticConfig::kBoundaryHalfSpanM);
    }

    bool LogSample(bool stationary, uint32_t timestampUs, float dtSeconds, const DiagnosticSensorSnapshot& snapshot)
    {
        const DriveTelemetry telemetry = _drive.GetTelemetry();
        const uint32_t dtUs = static_cast<uint32_t>(dtSeconds * 1.0e6f);
        DiagnosticLogRow row{};
        MazeMap::App::Internal::Runtime::PopulateDiagnosticLogRow(
            row,
            _sampleCount,
            _phaseId,
            stationary,
            timestampUs,
            dtUs,
            _drive.GetPose(),
            _drive,
            telemetry,
            snapshot);
        if (_runtime.LogUtilityDataRow(row))
        {
            ++_sampleCount;
            return true;
        }
        return Fail("Failed to write diagnostic sample");
    }

    bool HoldPhase(const char* phaseName, uint16_t durationMs, bool stationary)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _holdPhaseState = HoldPhaseState{};
        _holdPhaseState.stationary = stationary;
        _holdPhaseState.deadlineMs = millis() + durationMs;
        return RunPhaseSession(&DiagnosticController::HoldPhaseTick);
    }

    LoopController::ControlVector HoldPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        if (!LogSample(_holdPhaseState.stationary, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::BrakeCommand();
        }

        if (static_cast<long>(_holdPhaseState.deadlineMs - millis()) <= 0)
        {
            services.RequestEndLoop();
        }

        return LoopController::ControlVector::BrakeCommand();
    }

    bool ExecuteStraightPhase(const char* phaseName, float distanceM, float cruiseSpeedMps, float* outTraveledDistanceM = nullptr)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _straightPhaseState = StraightPhaseState{};
        _straightPhaseState.phaseName = phaseName;
        _straightPhaseState.distanceM = distanceM;
        _straightPhaseState.cruiseSpeedMps = cruiseSpeedMps;
        _straightPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _straightPhaseState.targetHeading = _drive.GetPose().headingUnit;
        _straightPhaseState.timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (6000.0f * distanceM));
        _straightPhaseState.translationWatchdog.Reset(0.0f, millis());
        _straightPhaseState.outTraveledDistanceM = outTraveledDistanceM;
        if (!RunPhaseSession(&DiagnosticController::StraightPhaseTick))
        {
            return false;
        }

        if (outTraveledDistanceM != nullptr)
        {
            *outTraveledDistanceM = _straightPhaseState.traveledM;
        }

        return WriteStraightResult(
            phaseName,
            distanceM,
            cruiseSpeedMps,
            _straightPhaseState.traveledM,
            _straightPhaseState.targetHeading,
            _straightPhaseState.metrics);
    }

    LoopController::ControlVector StraightPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const MotionLimits limits = DiagnosticLimits(_straightPhaseState.cruiseSpeedMps);
        _straightPhaseState.traveledM =
            std::fabs(_drive.GetAverageDistanceMeters() - _straightPhaseState.startDistanceM);
        const float remainingM = (std::max)(0.0f, _straightPhaseState.distanceM - _straightPhaseState.traveledM);
        _straightPhaseState.metrics.peakSpeedMps = (std::max)(
            _straightPhaseState.metrics.peakSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        if ((remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
            {
                services.Fault("Failed to write diagnostic sample");
                return LoopController::ControlVector::BrakeCommand();
            }

            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }
        if (_straightPhaseState.translationWatchdog.Stalled(
                _straightPhaseState.traveledM,
                _straightPhaseState.commandedSpeedMps,
                remainingM,
                millis()))
        {
            services.Fault("Straight diagnostic encoder progress stalled");
            return LoopController::ControlVector::BrakeCommand();
        }
        if (static_cast<long>(_straightPhaseState.timeoutMs - millis()) <= 0)
        {
            services.Fault("Straight diagnostic phase timed out");
            return LoopController::ControlVector::BrakeCommand();
        }

        const float accelLimitedSpeedMps = (std::min)(
            limits.maxSpeedMps,
            _straightPhaseState.commandedSpeedMps + (limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps = ReachableSpeedWithBoundary(0.0f, remainingM, limits.decelMps2);
        _straightPhaseState.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float headingErrorRad = HeadingErrorRad(_straightPhaseState.targetHeading, state.estimate.headingUnit);
        _straightPhaseState.metrics.maxHeadingErrorRad = (std::max)(
            _straightPhaseState.metrics.maxHeadingErrorRad,
            std::fabs(headingErrorRad));
        float angularCommandRadps =
            (Config::kStraightHeadingKp * headingErrorRad) -
            (Config::kStraightYawD * state.estimate.angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -limits.maxAngularSpeedRadps,
            limits.maxAngularSpeedRadps);

        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::BrakeCommand();
        }

        return LoopController::ControlVector::VelocityCommand(
            _straightPhaseState.commandedSpeedMps,
            angularCommandRadps);
    }

    bool RecoverCharacterizationSample(const char* label, float traveledDistanceM)
    {
        char phaseName[48] = {};
        if (traveledDistanceM <= LegacyDiagnosticConfig::kKickoffSweepMoveThresholdM)
        {
            snprintf(phaseName, sizeof(phaseName), "%s_settle", label);
            return HoldPhase(phaseName, DiagnosticConfig::kCharacterizationSettleMs, true);
        }

        snprintf(phaseName, sizeof(phaseName), "%s_turnaround", label);
        if (!ExecuteTurnPhase(phaseName, PI_F))
        {
            return false;
        }

        // Recover characterization samples with the same forward-drive path used elsewhere in diagnostics.
        // This avoids the poorly controlled reverse leg that can drift far past the available space.
        snprintf(phaseName, sizeof(phaseName), "%s_return", label);
        if (!ExecuteStraightPhase(phaseName, traveledDistanceM, DiagnosticConfig::kCharacterizationRecoverySpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "%s_reset_heading", label);
        if (!ExecuteTurnPhase(phaseName, PI_F))
        {
            return false;
        }

        const PoseEstimate& pose = _drive.GetPose();
        _drive.SetPose(pose.xMeters, pose.yMeters, DirectionToYawRad(MazeMap::Up));

        snprintf(phaseName, sizeof(phaseName), "%s_settle", label);
        return HoldPhase(phaseName, DiagnosticConfig::kCharacterizationSettleMs, true);
    }

    bool ExecuteKickoffCharacterizationSample(float driveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("kickoff", driveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _kickoffPhaseState = KickoffPhaseState{};
        snprintf(_kickoffPhaseState.label, sizeof(_kickoffPhaseState.label), "%s", label);
        _kickoffPhaseState.driveCommand = driveCommand;
        _kickoffPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _kickoffPhaseState.pulseDeadlineMs = millis() + LegacyDiagnosticConfig::kKickoffSweepPulseMs;
        _kickoffPhaseState.settleDeadlineMs = _kickoffPhaseState.pulseDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        _kickoffPhaseState.travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        if (!RunPhaseSession(&DiagnosticController::KickoffCharacterizationTick))
        {
            return false;
        }

        _drive.Brake();
        const float traveledDistanceM = std::fabs(_drive.GetAverageDistanceMeters() - _kickoffPhaseState.startDistanceM);
        const bool moved =
            (traveledDistanceM >= LegacyDiagnosticConfig::kKickoffSweepMoveThresholdM) ||
            (_kickoffPhaseState.maxSpeedMps >= LegacyDiagnosticConfig::kKickoffSweepMoveThresholdMps);

        char message[192] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;cmd=%.2f;dist_m=%.4f;max_speed_mps=%.3f;moved=%u;travel_limited=%u",
            label,
            driveCommand,
            traveledDistanceM,
            _kickoffPhaseState.maxSpeedMps,
            moved ? 1U : 0U,
            _kickoffPhaseState.travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format kickoff characterization result");
        }
        if (!WriteEventOrFail("kickoff_result", message, "Failed to write kickoff characterization result"))
        {
            return false;
        }

        return RecoverCharacterizationSample(label, traveledDistanceM);
    }

    LoopController::ControlVector KickoffCharacterizationTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const unsigned long nowMs = millis();
        const float traveledDistanceM =
            std::fabs(_drive.GetAverageDistanceMeters() - _kickoffPhaseState.startDistanceM);
        if (!_kickoffPhaseState.travelLimited &&
            (_kickoffPhaseState.travelLimitM > 0.0f) &&
            (traveledDistanceM >= _kickoffPhaseState.travelLimitM))
        {
            _kickoffPhaseState.travelLimited = true;
            _kickoffPhaseState.travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
        }

        const bool pulseActive =
            !_kickoffPhaseState.travelLimited &&
            (static_cast<long>(_kickoffPhaseState.pulseDeadlineMs - nowMs) > 0);
        const LoopController::ControlVector command =
            _kickoffPhaseState.travelLimited ?
            LoopController::ControlVector::BrakeCommand() :
            pulseActive ?
            LoopController::ControlVector::OpenLoopCommand(
                _kickoffPhaseState.driveCommand,
                _kickoffPhaseState.driveCommand) :
            LoopController::ControlVector::BrakeCommand();

        _kickoffPhaseState.maxSpeedMps = (std::max)(
            _kickoffPhaseState.maxSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::BrakeCommand();
        }

        if (_kickoffPhaseState.travelLimited &&
            (static_cast<long>(_kickoffPhaseState.travelLimitSettleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }

        if (!_kickoffPhaseState.travelLimited &&
            !pulseActive &&
            (static_cast<long>(_kickoffPhaseState.settleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }

        return command;
    }

    bool ExecuteForwardCharacterizationSample(float forwardDriveCommand)
    {
        char label[24] = {};
        char phaseName[48] = {};
        BuildDriveCommandLabel("forward", forwardDriveCommand, label, sizeof(label));
        snprintf(phaseName, sizeof(phaseName), "%s_probe", label);
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _forwardPhaseState = ForwardPhaseState{};
        snprintf(_forwardPhaseState.label, sizeof(_forwardPhaseState.label), "%s", label);
        _forwardPhaseState.forwardDriveCommand = forwardDriveCommand;
        _forwardPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _forwardPhaseState.kickoffDeadlineMs = millis() + LegacyDiagnosticConfig::kForwardSweepKickoffMs;
        _forwardPhaseState.holdDeadlineMs = _forwardPhaseState.kickoffDeadlineMs + LegacyDiagnosticConfig::kForwardSweepHoldMs;
        _forwardPhaseState.settleDeadlineMs = _forwardPhaseState.holdDeadlineMs + DiagnosticConfig::kCharacterizationSettleMs;
        _forwardPhaseState.travelLimitM = MazeMap::ComputeDiagnosticCharacterizationTravelLimitM(
            DiagnosticConfig::kBoundaryHalfSpanM,
            DiagnosticConfig::kCharacterizationBoundaryReserveM);
        if (!RunPhaseSession(&DiagnosticController::ForwardCharacterizationTick))
        {
            return false;
        }

        _drive.Brake();
        if (!_forwardPhaseState.holdComplete)
        {
            _forwardPhaseState.holdEndDistanceM = _drive.GetAverageDistanceMeters();
        }

        const float totalDistanceM =
            std::fabs(_drive.GetAverageDistanceMeters() - _forwardPhaseState.startDistanceM);
        const float holdDistanceM =
            _forwardPhaseState.holdStarted ?
            std::fabs(_forwardPhaseState.holdEndDistanceM - _forwardPhaseState.holdStartDistanceM) :
            0.0f;
        const float averageHoldSpeedMps =
            (_forwardPhaseState.holdElapsedSeconds > 0.0f) ?
            (holdDistanceM / _forwardPhaseState.holdElapsedSeconds) :
            0.0f;
        const bool carried =
            (averageHoldSpeedMps >= LegacyDiagnosticConfig::kForwardSweepCarryThresholdMps) ||
            (holdDistanceM >= LegacyDiagnosticConfig::kForwardSweepCarryThresholdM);

        char message[224] = {};
        const int length = snprintf(
            message,
            sizeof(message),
            "%s;kickoff=%.2f;hold=%.2f;hold_dist_m=%.4f;hold_avg_speed_mps=%.3f;total_dist_m=%.4f;max_speed_mps=%.3f;carried=%u;travel_limited=%u",
            label,
            LegacyDiagnosticConfig::kForwardSweepKickoffDriveCommand,
            forwardDriveCommand,
            holdDistanceM,
            averageHoldSpeedMps,
            totalDistanceM,
            _forwardPhaseState.maxSpeedMps,
            carried ? 1U : 0U,
            _forwardPhaseState.travelLimited ? 1U : 0U);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return Fail("Failed to format forward characterization result");
        }
        if (!WriteEventOrFail("forward_result", message, "Failed to write forward characterization result"))
        {
            return false;
        }

        return RecoverCharacterizationSample(label, totalDistanceM);
    }

    LoopController::ControlVector ForwardCharacterizationTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const unsigned long nowMs = millis();
        const float traveledDistanceM =
            std::fabs(_drive.GetAverageDistanceMeters() - _forwardPhaseState.startDistanceM);
        if (!_forwardPhaseState.travelLimited &&
            (_forwardPhaseState.travelLimitM > 0.0f) &&
            (traveledDistanceM >= _forwardPhaseState.travelLimitM))
        {
            _forwardPhaseState.travelLimited = true;
            _forwardPhaseState.travelLimitSettleDeadlineMs = nowMs + DiagnosticConfig::kCharacterizationSettleMs;
            if (_forwardPhaseState.holdStarted && !_forwardPhaseState.holdComplete)
            {
                _forwardPhaseState.holdComplete = true;
                _forwardPhaseState.holdEndDistanceM = _drive.GetAverageDistanceMeters();
            }
        }

        LoopController::ControlVector command = LoopController::ControlVector::BrakeCommand();
        if (_forwardPhaseState.travelLimited)
        {
            command = LoopController::ControlVector::BrakeCommand();
        }
        else if (static_cast<long>(_forwardPhaseState.kickoffDeadlineMs - nowMs) > 0)
        {
            command = LoopController::ControlVector::OpenLoopCommand(
                LegacyDiagnosticConfig::kForwardSweepKickoffDriveCommand,
                LegacyDiagnosticConfig::kForwardSweepKickoffDriveCommand);
        }
        else if (static_cast<long>(_forwardPhaseState.holdDeadlineMs - nowMs) > 0)
        {
            if (!_forwardPhaseState.holdStarted)
            {
                _forwardPhaseState.holdStarted = true;
                _forwardPhaseState.holdStartDistanceM = _drive.GetAverageDistanceMeters();
            }
            _forwardPhaseState.holdElapsedSeconds += state.dtSeconds;
            command = LoopController::ControlVector::OpenLoopCommand(
                _forwardPhaseState.forwardDriveCommand,
                _forwardPhaseState.forwardDriveCommand);
        }
        else if (!_forwardPhaseState.holdComplete)
        {
            _forwardPhaseState.holdComplete = true;
            _forwardPhaseState.holdEndDistanceM = _drive.GetAverageDistanceMeters();
        }

        _forwardPhaseState.maxSpeedMps = (std::max)(
            _forwardPhaseState.maxSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::BrakeCommand();
        }

        if (_forwardPhaseState.travelLimited &&
            (static_cast<long>(_forwardPhaseState.travelLimitSettleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }

        if (!_forwardPhaseState.travelLimited &&
            _forwardPhaseState.holdComplete &&
            (static_cast<long>(_forwardPhaseState.settleDeadlineMs - nowMs) <= 0) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }

        return command;
    }

    bool ExecuteKickoffSweep()
    {
        if (!HoldPhase("kickoff_sweep_prep", DiagnosticConfig::kCharacterizationSettleMs, true))
        {
            return false;
        }

        for (float driveCommand : MazeMap::kOpenFloorLaunchDriveMagnitudes)
        {
            if (!ExecuteKickoffCharacterizationSample(driveCommand))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteForwardSweep()
    {
        if (!HoldPhase("forward_sweep_prep", DiagnosticConfig::kCharacterizationSettleMs, true))
        {
            return false;
        }

        for (float driveCommand = LegacyDiagnosticConfig::kForwardSweepMinDriveCommand;
            driveCommand <= (LegacyDiagnosticConfig::kForwardSweepMaxDriveCommand + 0.0001f);
            driveCommand += LegacyDiagnosticConfig::kForwardSweepStepDriveCommand)
        {
            if (!ExecuteForwardCharacterizationSample(driveCommand))
            {
                return false;
            }
        }

        return true;
    }

    bool ExecuteTurnPhase(const char* phaseName, float angleRad)
    {
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _turnPhaseState = TurnPhaseState{};
        _turnPhaseState.phaseName = phaseName;
        _turnPhaseState.angleRad = angleRad;
        _turnPhaseState.targetYawRad = WrapAngleRad(_drive.GetPose().yawRad + angleRad);
        _turnPhaseState.turnProfile = BuildSharedInPlaceTurnProfile(_vehicle);
        _turnPhaseState.timeoutMs = millis() + 3000UL;
        if (!RunPhaseSession(&DiagnosticController::TurnPhaseTick))
        {
            return false;
        }

        return WriteTurnResult(
            phaseName,
            angleRad,
            _turnPhaseState.metrics,
            _turnPhaseState.targetYawRad);
    }

    LoopController::ControlVector TurnPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        const float errorRad = AngleErrorRad(_turnPhaseState.targetYawRad, state.estimate.yawRad);
        const float remainingRad = std::fabs(errorRad);
        _turnPhaseState.metrics.maxYawErrorRad = (std::max)(_turnPhaseState.metrics.maxYawErrorRad, remainingRad);
        _turnPhaseState.metrics.peakOmegaRadps = (std::max)(
            _turnPhaseState.metrics.peakOmegaRadps,
            std::fabs(state.estimate.angularSpeedRadps));
        if (MazeMap::IsInPlaceTurnComplete(errorRad, state.estimate.angularSpeedRadps, _turnPhaseState.turnProfile))
        {
            if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
            {
                services.Fault("Failed to write diagnostic sample");
                return LoopController::ControlVector::BrakeCommand();
            }

            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }
        if (static_cast<long>(_turnPhaseState.timeoutMs - millis()) <= 0)
        {
            services.Fault("Turn diagnostic phase timed out");
            return LoopController::ControlVector::BrakeCommand();
        }

        float angularCommandRadps = 0.0f;
        if (!MazeMap::TryComputeInPlaceTurnCommandRadps(
                errorRad,
                state.estimate.angularSpeedRadps,
                _turnPhaseState.turnProfile,
                angularCommandRadps))
        {
            services.Fault("Turn diagnostic phase profile became invalid");
            return LoopController::ControlVector::BrakeCommand();
        }

        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::BrakeCommand();
        }

        return LoopController::ControlVector::VelocityCommand(0.0f, angularCommandRadps);
    }

    bool ExecuteArcPhase(const char* phaseName, float distanceM, float angleRad, float cruiseSpeedMps, ArcPhaseMetrics* outMetrics = nullptr)
    {
        if (distanceM <= 0.0f)
        {
            return Fail("Diagnostic arc distance must be positive");
        }
        if (!StartPhase(phaseName))
        {
            return false;
        }

        _arcPhaseState = ArcPhaseState{};
        _arcPhaseState.phaseName = phaseName;
        _arcPhaseState.distanceM = distanceM;
        _arcPhaseState.angleRad = angleRad;
        _arcPhaseState.cruiseSpeedMps = cruiseSpeedMps;
        _arcPhaseState.outMetrics = outMetrics;
        _arcPhaseState.limits = DiagnosticLimits(cruiseSpeedMps);
        _arcPhaseState.startDistanceM = _drive.GetAverageDistanceMeters();
        _arcPhaseState.startYawRad = _drive.GetPose().yawRad;
        _arcPhaseState.targetYawRad = WrapAngleRad(_arcPhaseState.startYawRad + angleRad);
        _arcPhaseState.curvature = angleRad / distanceM;
        _arcPhaseState.timeoutMs = millis() + static_cast<unsigned long>(2500.0f + (5000.0f * distanceM));
        _arcPhaseState.translationWatchdog.Reset(0.0f, millis());
        if (!RunPhaseSession(&DiagnosticController::ArcPhaseTick))
        {
            return false;
        }

        if (outMetrics != nullptr)
        {
            *outMetrics = _arcPhaseState.metrics;
        }
        return WriteArcResult(
            phaseName,
            distanceM,
            angleRad,
            _arcPhaseState.traveledM,
            _arcPhaseState.targetYawRad,
            _arcPhaseState.metrics);
    }

    LoopController::ControlVector ArcPhaseTick(
        std::uint32_t loopEndTimeUs,
        const LoopController::ModeState& state,
        LoopController::TickServices& services)
    {
        (void)loopEndTimeUs;
        _arcPhaseState.traveledM = std::fabs(_drive.GetAverageDistanceMeters() - _arcPhaseState.startDistanceM);
        const float remainingM = (std::max)(0.0f, _arcPhaseState.distanceM - _arcPhaseState.traveledM);
        _arcPhaseState.metrics.peakSpeedMps = (std::max)(
            _arcPhaseState.metrics.peakSpeedMps,
            std::fabs(state.estimate.linearSpeedMps));
        _arcPhaseState.metrics.peakOmegaRadps = (std::max)(
            _arcPhaseState.metrics.peakOmegaRadps,
            std::fabs(state.estimate.angularSpeedRadps));
        _arcPhaseState.metrics.durationSeconds += state.dtSeconds;
        _arcPhaseState.metrics.omegaIntegralRad += state.estimate.angularSpeedRadps * state.dtSeconds;
        _arcPhaseState.metrics.speedIntegralMpsSeconds += std::fabs(state.estimate.linearSpeedMps) * state.dtSeconds;
        const float planarAccelMps2 = _sensors.GetPlanarAccelMps2(state.diagnosticSensors);
        _arcPhaseState.metrics.planarAccelIntegralMps2Seconds += planarAccelMps2 * state.dtSeconds;
        _arcPhaseState.metrics.peakPlanarAccelMps2 = (std::max)(
            _arcPhaseState.metrics.peakPlanarAccelMps2,
            planarAccelMps2);
        if ((remainingM <= Config::kDistanceToleranceM) &&
            (std::fabs(state.estimate.linearSpeedMps) <= Config::kSpeedToleranceMps))
        {
            if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
            {
                services.Fault("Failed to write diagnostic sample");
                return LoopController::ControlVector::BrakeCommand();
            }

            services.RequestEndLoop();
            return LoopController::ControlVector::BrakeCommand();
        }
        if (_arcPhaseState.translationWatchdog.Stalled(
                _arcPhaseState.traveledM,
                _arcPhaseState.commandedSpeedMps,
                remainingM,
                millis()))
        {
            services.Fault("Arc diagnostic encoder progress stalled");
            return LoopController::ControlVector::BrakeCommand();
        }
        if (static_cast<long>(_arcPhaseState.timeoutMs - millis()) <= 0)
        {
            services.Fault("Arc diagnostic phase timed out");
            return LoopController::ControlVector::BrakeCommand();
        }

        const float accelLimitedSpeedMps = (std::min)(
            _arcPhaseState.cruiseSpeedMps,
            _arcPhaseState.commandedSpeedMps + (_arcPhaseState.limits.accelMps2 * state.dtSeconds));
        const float decelLimitedSpeedMps =
            ReachableSpeedWithBoundary(0.0f, remainingM, _arcPhaseState.limits.decelMps2);
        _arcPhaseState.commandedSpeedMps = (std::min)(accelLimitedSpeedMps, decelLimitedSpeedMps);

        const float progress = (std::clamp)(_arcPhaseState.traveledM / _arcPhaseState.distanceM, 0.0f, 1.0f);
        const float phaseTargetYawRad = WrapAngleRad(_arcPhaseState.startYawRad + (_arcPhaseState.angleRad * progress));
        const float headingErrorRad = AngleErrorRad(phaseTargetYawRad, state.estimate.yawRad);
        _arcPhaseState.metrics.maxHeadingErrorRad = (std::max)(
            _arcPhaseState.metrics.maxHeadingErrorRad,
            std::fabs(headingErrorRad));
        float angularCommandRadps =
            (_arcPhaseState.curvature * _arcPhaseState.commandedSpeedMps) +
            (Config::kArcHeadingKp * headingErrorRad) -
            (Config::kArcYawD * state.estimate.angularSpeedRadps);
        angularCommandRadps = (std::clamp)(
            angularCommandRadps,
            -_arcPhaseState.limits.maxAngularSpeedRadps,
            _arcPhaseState.limits.maxAngularSpeedRadps);

        if (!LogSample(false, state.tickStartUs, state.dtSeconds, state.diagnosticSensors))
        {
            services.Fault("Failed to write diagnostic sample");
            return LoopController::ControlVector::BrakeCommand();
        }

        return LoopController::ControlVector::VelocityCommand(
            _arcPhaseState.commandedSpeedMps,
            angularCommandRadps);
    }

    bool ExecuteArcCircle(const char* namePrefix, float halfCircleAngleRad, float halfCircleDistanceM, float cruiseSpeedMps)
    {
        char phaseName[48] = {};
        const PoseEstimate startPose = _drive.GetPose();
        const DriveTelemetry startTelemetry = _drive.GetTelemetry();
        ArcPhaseMetrics totalMetrics{};
        ArcPhaseMetrics segmentMetrics{};

        snprintf(phaseName, sizeof(phaseName), "%s_half_1", (namePrefix != nullptr) ? namePrefix : "arc_circle");
        if (!ExecuteArcPhase(phaseName, halfCircleDistanceM, halfCircleAngleRad, cruiseSpeedMps, &segmentMetrics))
        {
            return false;
        }
        AccumulateArcMetrics(totalMetrics, segmentMetrics);

        snprintf(phaseName, sizeof(phaseName), "%s_half_2", (namePrefix != nullptr) ? namePrefix : "arc_circle");
        if (!ExecuteArcPhase(phaseName, halfCircleDistanceM, halfCircleAngleRad, cruiseSpeedMps, &segmentMetrics))
        {
            return false;
        }
        AccumulateArcMetrics(totalMetrics, segmentMetrics);

        if (!WriteCircleResult(namePrefix, cruiseSpeedMps, startTelemetry, totalMetrics))
        {
            return false;
        }

        return WriteClosureResult("arc_circle_result", namePrefix, startPose, "Failed to write arc circle diagnostic result");
    }

    bool ExecuteCircleSpeedSweep(const char* speedLabel, float cruiseSpeedMps)
    {
        char phaseName[48] = {};

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, PI_F, LegacyDiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_cw_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_ccw_%s", (speedLabel != nullptr) ? speedLabel : "speed");
        if (!ExecuteArcCircle(phaseName, -PI_F, LegacyDiagnosticConfig::kArcHalfCircleDistanceM, cruiseSpeedMps))
        {
            return false;
        }

        snprintf(phaseName, sizeof(phaseName), "circle_%s_settle", (speedLabel != nullptr) ? speedLabel : "speed");
        return HoldPhase(phaseName, DiagnosticConfig::kInterTestHoldMs, true);
    }

    bool ExecuteSquareLoop(const char* namePrefix, float turnAngleRad)
    {
        char phaseName[48] = {};
        const PoseEstimate startPose = _drive.GetPose();
        for (uint8_t leg = 0; leg < 4U; ++leg)
        {
            snprintf(phaseName, sizeof(phaseName), "%s_leg_%u", namePrefix, static_cast<unsigned>(leg + 1U));
            if (!ExecuteStraightPhase(phaseName, LegacyDiagnosticConfig::kSquareLegDistanceM, LegacyDiagnosticConfig::kSlowStraightSpeedMps))
            {
                return false;
            }

            snprintf(phaseName, sizeof(phaseName), "%s_turn_%u", namePrefix, static_cast<unsigned>(leg + 1U));
            if (!ExecuteTurnPhase(phaseName, turnAngleRad))
            {
                return false;
            }
        }

        return WriteClosureResult("square_result", namePrefix, startPose, "Failed to write square diagnostic result");
    }
};

