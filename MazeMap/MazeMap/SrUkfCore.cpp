#include "pch.h"
#include "SrUkfCore.h"

#include "PlanarAccelLateralUpdate.h"
#include "SoftOdometryAid.h"
#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    using CommandVector = MazeMap::App::Internal::CommandVector;

    constexpr std::array<const char*, MazeMap::VehicleState::kDimension> kUkfStateFieldNames = {
        "px_m",
        "py_m",
        "psi_rad",
        "u_mps",
        "v_mps",
        "r_radps",
        "omega_l_radps",
        "omega_r_radps",
        "bgz_radps"
    };

    constexpr std::array<const char*, 3> kUkfImuNoiseFieldNames = {
        "gyro_z_radps",
        "accel_body_x_mps2",
        "accel_body_y_mps2"
    };

    constexpr std::array<const char*, 2> kUkfFrontNoiseFieldNames = {
        "front_left_range_m",
        "front_right_range_m"
    };

    constexpr std::array<const char*, 1> kUkfSideNoiseFieldNames = {
        "side_range_m"
    };

    constexpr float kStationaryCertificationDwellS = 0.150f;
    constexpr float kStationaryCandidateMaxLinearCommandMps = 0.03f;
    constexpr float kStationaryCandidateMaxAngularCommandRadps = 0.15f;
    constexpr float kStationaryCandidateMaxDriveCommand = 0.08f;
    constexpr float kStationaryCandidateMaxEncoderOmegaRadps = 1.0f;
    constexpr float kStationaryCandidateMaxCorrectedGyroRadps = 0.12f;
    constexpr float kStationaryCandidateMaxAccelMps2 = 1.0f;

    constexpr float kCommandSignFlipWindowS = 0.025f;
    constexpr float kStationaryExitLaunchWindowS = 0.060f;
    constexpr float kLaunchHoldS = 0.030f;
    constexpr float kLaunchLowSpeedThresholdMps = 0.25f;
    constexpr float kLaunchDriveCommandDeltaThreshold = 0.75f;

    constexpr float kInconsistentHoldS = 0.080f;
    constexpr float kYawConsistencyLowPassTauS = 0.025f;
    constexpr float kYawConsistencyLowPassThresholdRadps = 0.08f;
    constexpr float kYawConsistencyExceedDwellS = 0.025f;
    constexpr float kYawWindowDurationS = 0.080f;
    constexpr float kYawWindowMismatchThresholdRad = 0.03f;
    constexpr float kNhcResidualTripSigma = 3.0f;

    constexpr float kNhcMinimumEnableForwardSpeedMps = 0.08f;
    constexpr float kNhcDisableForwardSpeedMps = 0.05f;
    constexpr float kNhcMaxDriveCommandDelta = 0.60f;
    constexpr float kRecoveryNhcReenableDelayS = 0.025f;

    constexpr float kRecoveryYawRateStdFloorRadps = 0.030f;
    constexpr float kYawValidityBiasDeltaMaxRadps = 0.02f;
    constexpr float kGyroBiasCovarianceFloorRadps2 = 1.0e-9f;
    constexpr float kStationaryBodyDecayTauS = 0.075f;
    constexpr float kStationaryWheelDecayTauS = 0.050f;

    using ModeProcessNoiseConfig = MazeMap::SrUkfCore::ModeProcessNoiseTuning;

    constexpr ModeProcessNoiseConfig kStationaryCertifiedProcessNoise{
        0.006f,
        0.000f,
        0.010f,
        0.050f,
        0.010f,
        0.0f
    };

    constexpr ModeProcessNoiseConfig kLaunchOrReversalProcessNoise{
        0.020f,
        0.000f,
        0.050f,
        0.40f,
        0.030f,
        0.006f
    };

    constexpr ModeProcessNoiseConfig kGripLinearProcessNoise{
        0.012f,
        0.000f,
        0.025f,
        0.400f,
        0.020f,
        0.006f
    };

    constexpr ModeProcessNoiseConfig kInconsistentOrSaturatedProcessNoise{
        0.030f,
        0.002f,
        0.070f,
        0.50f,
        0.050f,
        0.020f
    };

    MazeMap::SrUkfCore::RuntimeTuning g_runtimeTuning = MazeMap::SrUkfCore::BuildDefaultRuntimeTuning();

    const MazeMap::SrUkfCore::RuntimeTuning& Tuning() noexcept
    {
        return g_runtimeTuning;
    }

    const ModeProcessNoiseConfig& GetModeProcessNoiseConfig(const MazeMap::SrUkfCore::OperatingMode mode) noexcept
    {
        switch (mode)
        {
        case MazeMap::SrUkfCore::OperatingMode::StationaryCertified:
            return Tuning().stationaryCertifiedProcessNoise;
        case MazeMap::SrUkfCore::OperatingMode::LaunchOrReversalTransient:
            return Tuning().launchOrReversalProcessNoise;
        case MazeMap::SrUkfCore::OperatingMode::InconsistentOrSaturated:
            return Tuning().inconsistentOrSaturatedProcessNoise;
        case MazeMap::SrUkfCore::OperatingMode::GripLinear:
        default:
            return Tuning().gripLinearProcessNoise;
        }
    }

    float Clamp01(float value) noexcept
    {
        if (!std::isfinite(value))
        {
            return 0.0f;
        }

        return (std::clamp)(value, 0.0f, 1.0f);
    }

    float ClampScale(float value) noexcept
    {
        if (!std::isfinite(value))
        {
            return 1.0f;
        }

        return (std::max)(0.25f, (std::min)(value, 8.0f));
    }

    float PositiveOr(float value, float fallback) noexcept
    {
        return (std::isfinite(value) && (value > 0.0f)) ? value : fallback;
    }

    float BlendTowards(float prior, float target, float dtS, float tauS) noexcept
    {
        if (!(std::isfinite(dtS) && (dtS > 0.0f)) || !(std::isfinite(tauS) && (tauS > 0.0f)))
        {
            return Clamp01(prior);
        }

        const float alpha = Clamp01(dtS / tauS);
        return Clamp01(prior + ((target - prior) * alpha));
    }

    float ResolveStationaryDecayAlpha(float dtS, float tauS) noexcept
    {
        if (!(std::isfinite(dtS) && (dtS > 0.0f) && std::isfinite(tauS) && (tauS > 0.0f)))
        {
            return 0.0f;
        }

        return (std::clamp)(std::exp(-dtS / tauS), 0.0f, 1.0f);
    }

    float PrecursorSeverity(float utilization) noexcept
    {
        if (!std::isfinite(utilization))
        {
            return 0.0f;
        }

        return Clamp01((utilization - 0.75f) / 0.30f);
    }

    float SmoothStep01(float value) noexcept
    {
        const float clamped = Clamp01(value);
        return clamped * clamped * (3.0f - (2.0f * clamped));
    }

    float ResolveSeverity(float anomalySeverity, float memorySeverity) noexcept
    {
        return Clamp01((std::max)(Clamp01(anomalySeverity), Clamp01(memorySeverity)));
    }

    float FeedforwardEdgeStrength(float memorySeverity, float recoverySeverity) noexcept
    {
        return Clamp01((0.5f * Clamp01(memorySeverity)) + (0.5f * Clamp01(recoverySeverity)));
    }

    float RecoveryPenalty(bool inRecovery, float recoverySeverity) noexcept
    {
        return inRecovery ? (0.5f + (0.5f * Clamp01(recoverySeverity))) : 0.0f;
    }

    bool IsLeftLateralDirection(const MazeMap::RelativeDirection side) noexcept
    {
        return side == MazeMap::RelativeDirection::Left90;
    }

    bool IsRightLateralDirection(const MazeMap::RelativeDirection side) noexcept
    {
        return side == MazeMap::RelativeDirection::Right90;
    }

    float GyroMeasurementVarianceRadps2() noexcept
    {
        const float sigmaRadps = Tuning().imuYawRateSigmaRadps;
        if (std::isfinite(sigmaRadps) && (sigmaRadps > 0.0f))
        {
            return sigmaRadps * sigmaRadps;
        }

        return MazeMap::SrUkfCore::kImuYawRateVarianceRadps2;
    }

    float GyroBiasProcessVarianceRadps2ForMode(const MazeMap::SrUkfCore::OperatingMode mode) noexcept
    {
        return
            (mode == MazeMap::SrUkfCore::OperatingMode::StationaryCertified) ?
            MazeMap::SrUkfCore::kGyroBiasProcessVarianceStationaryRadps2PerSample :
            MazeMap::SrUkfCore::kGyroBiasProcessVarianceMovingRadps2PerSample;
    }

    float GyroBiasProcessSquareRootForMode(const MazeMap::SrUkfCore::OperatingMode mode) noexcept
    {
        return MazeMap::Math::Sqrtf(GyroBiasProcessVarianceRadps2ForMode(mode));
    }

    Eigen::Vector2f HeadingUnitFromYaw(float yaw) noexcept
    {
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(yaw, s, c);
        return Eigen::Vector2f(s, c);
    }

    template <typename... Args>
    bool AppendFormattedFragment(
        char* buffer,
        const std::size_t bufferSize,
        std::size_t& used,
        const char* format,
        Args... args) noexcept
    {
        if (buffer == nullptr || format == nullptr || used >= bufferSize)
        {
            return false;
        }

        const int length = std::snprintf(buffer + used, bufferSize - used, format, args...);
        if (length <= 0 || static_cast<std::size_t>(length) >= (bufferSize - used))
        {
            return false;
        }

        used += static_cast<std::size_t>(length);
        return true;
    }

    inline bool EmitDebugTextMessage(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* message) noexcept
    {
        return
            sink != nullptr &&
            type != nullptr &&
            type[0] != '\0' &&
            message != nullptr &&
            sink(context, type, message);
    }

    template <typename... Args>
    bool EmitDebugTextLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* format,
        Args... args) noexcept
    {
        char message[300] = {};
        const int length = std::snprintf(message, sizeof(message), format, args...);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }

        return EmitDebugTextMessage(context, sink, type, message);
    }

    template <std::size_t N, typename Reader>
    bool EmitNamedFloatValuesLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* prefix,
        const std::array<const char*, N>& fieldNames,
        Reader&& reader) noexcept
    {
        char message[300] = {};
        std::size_t used = 0U;
        if (prefix != nullptr && prefix[0] != '\0' &&
            !AppendFormattedFragment(message, sizeof(message), used, "%s", prefix))
        {
            return false;
        }

        for (std::size_t index = 0; index < N; ++index)
        {
            if (!AppendFormattedFragment(
                    message,
                    sizeof(message),
                    used,
                    "%s%s=%.9g",
                    (used != 0U) ? ";" : "",
                    fieldNames[index],
                    static_cast<double>(reader(index))))
            {
                return false;
            }
        }

        return EmitDebugTextMessage(context, sink, type, message);
    }

    template <std::size_t N, typename MatrixType>
    bool EmitNamedMatrixRowLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const std::array<const char*, N>& fieldNames,
        const MatrixType& matrix,
        const int row) noexcept
    {
        char prefix[48] = {};
        const int prefixLength = std::snprintf(
            prefix,
            sizeof(prefix),
            "row=%s",
            fieldNames[static_cast<std::size_t>(row)]);
        if (prefixLength <= 0 || prefixLength >= static_cast<int>(sizeof(prefix)))
        {
            return false;
        }

        return EmitNamedFloatValuesLine(
            context,
            sink,
            type,
            prefix,
            fieldNames,
            [&](const std::size_t column) noexcept
            {
                return matrix(row, static_cast<int>(column));
            });
    }
}

namespace MazeMap
{
    SrUkfCore::RuntimeTuning SrUkfCore::BuildDefaultRuntimeTuning() noexcept
    {
        RuntimeTuning tuning{};
        tuning.generalEncoderLinearSpeedSigmaMps = kGeneralEncoderLinearSpeedSigmaMps;
        tuning.generalEncoderYawRateSigmaRadps = kGeneralEncoderYawRateSigmaRadps;
        tuning.stationaryEncoderVelocitySigmaMps = kStationaryEncoderVelocitySigmaMps;
        tuning.encoderPairNisThreshold = kEncoderPairNisThreshold;
        tuning.imuYawRateSigmaRadps = kImuYawRateSigmaRadps;
        tuning.imuAccelSigmaMps2 = kImuAccelSigmaMps2;
        tuning.pivotScrubMaxCommandLinearMps = kPivotScrubMaxCommandLinearMps;
        tuning.pivotScrubMinCommandAngularRadps = kPivotScrubMinCommandAngularRadps;
        tuning.pivotScrubYawConsistencyThresholdRadps = kPivotScrubYawConsistencyThresholdRadps;
        tuning.pivotScrubYawWindowMismatchThresholdRad = kPivotScrubYawWindowMismatchThresholdRad;
        tuning.pivotScrubZeroUSigmaMps = kPivotScrubZeroUSigmaMps;
        tuning.stationaryCertificationDwellS = kStationaryCertificationDwellS;
        tuning.stationaryCandidateMaxLinearCommandMps = kStationaryCandidateMaxLinearCommandMps;
        tuning.stationaryCandidateMaxAngularCommandRadps = kStationaryCandidateMaxAngularCommandRadps;
        tuning.stationaryCandidateMaxDriveCommand = kStationaryCandidateMaxDriveCommand;
        tuning.stationaryCandidateMaxEncoderOmegaRadps = kStationaryCandidateMaxEncoderOmegaRadps;
        tuning.stationaryCandidateMaxCorrectedGyroRadps = kStationaryCandidateMaxCorrectedGyroRadps;
        tuning.stationaryCandidateMaxAccelMps2 = kStationaryCandidateMaxAccelMps2;
        tuning.commandSignFlipWindowS = kCommandSignFlipWindowS;
        tuning.stationaryExitLaunchWindowS = kStationaryExitLaunchWindowS;
        tuning.launchHoldS = kLaunchHoldS;
        tuning.launchLowSpeedThresholdMps = kLaunchLowSpeedThresholdMps;
        tuning.launchDriveCommandDeltaThreshold = kLaunchDriveCommandDeltaThreshold;
        tuning.inconsistentHoldS = kInconsistentHoldS;
        tuning.yawConsistencyLowPassTauS = kYawConsistencyLowPassTauS;
        tuning.yawConsistencyLowPassThresholdRadps = kYawConsistencyLowPassThresholdRadps;
        tuning.yawConsistencyExceedDwellS = kYawConsistencyExceedDwellS;
        tuning.yawWindowDurationS = kYawWindowDurationS;
        tuning.yawWindowMismatchThresholdRad = kYawWindowMismatchThresholdRad;
        tuning.nhcResidualTripSigma = kNhcResidualTripSigma;
        tuning.nhcMinimumEnableForwardSpeedMps = kNhcMinimumEnableForwardSpeedMps;
        tuning.nhcDisableForwardSpeedMps = kNhcDisableForwardSpeedMps;
        tuning.nhcMaxDriveCommandDelta = kNhcMaxDriveCommandDelta;
        tuning.recoveryNhcReenableDelayS = kRecoveryNhcReenableDelayS;
        tuning.nhcBaseSigmaMps = 0.005f;
        tuning.nhcSpeedSlopePerMps = 0.05f;
        tuning.nhcMinimumSigmaMps = 0.005f;
        tuning.nhcMaximumSigmaMps = 0.040f;
        tuning.recoveryYawRateStdFloorRadps = kRecoveryYawRateStdFloorRadps;
        tuning.yawValidityBiasDeltaMaxRadps = kYawValidityBiasDeltaMaxRadps;
        tuning.stationaryCertifiedProcessNoise = kStationaryCertifiedProcessNoise;
        tuning.launchOrReversalProcessNoise = kLaunchOrReversalProcessNoise;
        tuning.gripLinearProcessNoise = kGripLinearProcessNoise;
        tuning.inconsistentOrSaturatedProcessNoise = kInconsistentOrSaturatedProcessNoise;
        return tuning;
    }

    SrUkfCore::RuntimeTuning SrUkfCore::GetRuntimeTuning() noexcept
    {
        return g_runtimeTuning;
    }

    void SrUkfCore::SetRuntimeTuning(const RuntimeTuning& tuning) noexcept
    {
        g_runtimeTuning = tuning;
    }

    void SrUkfCore::ResetRuntimeTuning() noexcept
    {
        g_runtimeTuning = BuildDefaultRuntimeTuning();
    }

    SrUkfCore::SrUkfCore(
        const PlantParams& params,
        const PlantModel& plantModel) noexcept
        : _plantModel(plantModel)
        , _geometryModel()
        , _params(params)
        , _preparedParams(PlantModel::Prepare(_params))
        , _filter()
        , _frozenAppliedTorque()
        , _frozenGripUtilization()
        , _frozenSchedule()
        , _transientContactMemory()
        , _regripRecovery()
        , _lastControl()
        , _lastEncoderObs()
        , _lastEncoderDtSeconds(0.0f)
        , _prePredictState(StateVector::Zero())
        , _prePredictCovariance(StateMatrix::Zero())
        , _havePredictionReference(false)
        , _acceptedEncoderUpdateSincePredict(false)
        , _sqrtProcessNoiseDensity(StateMatrix::Zero())
        , _sqrtImuNoise(Eigen::Matrix<float, 3, 3>::Identity())
        , _sqrtFrontNoise(Eigen::Matrix<float, 2, 2>::Identity())
        , _sqrtSideNoise(Eigen::Matrix<float, 1, 1>::Identity())
        , _operatingMode(OperatingMode::GripLinear)
        , _gyroBiasAnchorRadps(0.0f)
        , _gyroBiasAnchorVarianceRadps2(kGyroBiasInitialVarianceUnseededRadps2)
        , _initialStationaryGyroBiasPhaseExited(false)
        , _initialStationaryGyroBiasSeedApplied(false)
        , _initialStationaryGyroBiasSampleOrdinal(0U)
        , _initialStationaryGyroBiasCollectedSeedSamples(0U)
        , _initialStationaryGyroBiasSeedAccumRadps(0.0)
        , _commandedLinearMps(0.0f)
        , _commandedAngularRadps(0.0f)
        , _saturationFlags(0U)
        , _leftLaunchAssistFloor(0.0f)
        , _rightLaunchAssistFloor(0.0f)
        , _accelBodyXMps2(0.0f)
        , _accelBodyYMps2(0.0f)
        , _stationaryCandidateDwellS(0.0f)
        , _stationaryCandidatePoseReferenceState(StateVector::Zero())
        , _stationaryCandidatePoseReferenceCovariance(StateMatrix::Zero())
        , _stationaryCandidatePoseReferenceValid(false)
        , _stationaryCertified(false)
        , _timeSinceStationaryExitS(std::numeric_limits<float>::infinity())
        , _timeSinceCommandSignFlipS(std::numeric_limits<float>::infinity())
        , _previousAverageDriveCommandSign(0.0f)
        , _launchHoldRemainingS(0.0f)
        , _inconsistentHoldRemainingS(0.0f)
        , _nhcReenableDelayRemainingS(0.0f)
        , _yawConsistencyLowPassRadps(0.0f)
        , _yawWindowMismatchRad(0.0f)
        , _yawConsistencyExceedDwellS(0.0f)
        , _nhcSigmaMps(ComputeNonholonomicSigmaMps(0.0f))
        , _nhcResidualMps(0.0f)
        , _nhcResidualSigma(0.0f)
        , _nonholonomicConstraintEnabled(false)
        , _yawValidForFeedforward(true)
        , _biasUpdateEnabled(false)
        , _pivotScrubMode(false)
        , _pivotScrubEncoderBodyUpdateSkipped(false)
        , _pivotScrubZeroUSoftApplied(false)
        , _pivotScrubEncoderWheelDeltaPsiRad(0.0f)
        , _pivotScrubEncoderWheelDeltaRRadps(0.0f)
        , _pivotScrubEncoderWheelDeltaOmegaLRadps(0.0f)
        , _pivotScrubEncoderWheelDeltaOmegaRRadps(0.0f)
        , _pivotScrubEncoderWheelMaskedDeltaNorm(0.0f)
        , _pivotScrubZeroUInnovationMps(0.0f)
        , _pivotScrubZeroUDeltaMps(0.0f)
        , _pivotScrubGyroDeltaPsiRad(0.0f)
        , _pivotScrubGyroDeltaRRadps(0.0f)
        , _pivotScrubGyroDeltaBgzRadps(0.0f)
        , _pivotScrubGyroDeltaOmegaLRadps(0.0f)
        , _pivotScrubGyroDeltaOmegaRRadps(0.0f)
        , _pivotScrubGyroMaskedDeltaNorm(0.0f)
        , _directWheelUpdateBodyStateInvariant(false)
        , _releaseInflationApplied(false)
        , _lastClosureResidualLeftMps(0.0f)
        , _lastClosureResidualRightMps(0.0f)
        , _lastGyroMeasurementRadps(0.0f)
        , _lastGyroInnovationRadps(0.0f)
        , _lastGyroNis(0.0f)
        , _lastForwardAccelInnovationMps2(0.0f)
        , _lastForwardAccelNis(0.0f)
        , _lastLateralAccelInnovationMps2(0.0f)
        , _lastLateralAccelNis(0.0f)
        , _lastClosureLeftInnovationMps(0.0f)
        , _lastClosureLeftNis(0.0f)
        , _lastClosureRightInnovationMps(0.0f)
        , _lastClosureRightNis(0.0f)
        , _lastLateralPseudoInnovationMps(0.0f)
        , _lastLateralPseudoNis(0.0f)
        , _yawWindowDtSeconds{}
        , _yawWindowUkfIntegralRad{}
        , _yawWindowGyroIntegralRad{}
        , _yawWindowHead(0U)
        , _yawWindowSize(0U)
        , _yawWindowSpanS(0.0f)
    {
        _filter.setStateNormalizer(&VehicleState::NormalizeStateVector);
        _filter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);

        StateVector initialState = StateVector::Zero();
        const StateMatrix initialCovariance = BuildDefaultInitialCovariance();
        _filter.setState(initialState, initialCovariance);

        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _prePredictState = initialState;

        _sqrtImuNoise(0, 0) = Tuning().imuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = Tuning().imuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = Tuning().imuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
        const ModeProcessNoiseConfig& initialModeNoise = GetModeProcessNoiseConfig(_operatingMode);
        (void)_filter.floorVariance(VehicleState::kR, initialModeNoise.stdRMin * initialModeNoise.stdRMin);
        (void)_filter.floorVariance(VehicleState::kV, initialModeNoise.stdVMin * initialModeNoise.stdVMin);
        (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        _gyroBiasAnchorRadps = initialState(VehicleState::kBgz);
        _gyroBiasAnchorVarianceRadps2 = _filter.variance(VehicleState::kBgz);
        _prePredictCovariance = _filter.covariance();
        refreshFrozenPolicyState(
            0.0f,
            App::Internal::CommandVector(0.0f, 0.0f),
            0.80f,
            0.0f);
    }

    void SrUkfCore::setRuntimeContext(
        float commandedLinearMps,
        float commandedAngularRadps,
        std::uint16_t saturationFlags,
        float leftLaunchAssistFloor,
        float rightLaunchAssistFloor,
        bool accelBiasValid,
        float accelBodyXMps2,
        float accelBodyYMps2) noexcept
    {
        _commandedLinearMps = commandedLinearMps;
        _commandedAngularRadps = commandedAngularRadps;
        _saturationFlags = saturationFlags;
        _leftLaunchAssistFloor = leftLaunchAssistFloor;
        _rightLaunchAssistFloor = rightLaunchAssistFloor;
        _accelBodyXMps2 = accelBiasValid ? accelBodyXMps2 : std::numeric_limits<float>::quiet_NaN();
        _accelBodyYMps2 = accelBiasValid ? accelBodyYMps2 : std::numeric_limits<float>::quiet_NaN();
    }

    float SrUkfCore::resolveYawRateForFeedforward(float yawRateRawRadps) const noexcept
    {
        return _yawValidForFeedforward ? _filter.state()(VehicleState::kR) : correctedYawRateRadps(yawRateRawRadps);
    }

    DriveCommandSolution SrUkfCore::solveAlignedDriveCommands(
        const StateVector& currentState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        PlantModel::FeedforwardRequest request{};
        request.currentState = currentState;
        request.hasCurrentState = true;
        request.currentForwardSpeedMps = currentState(VehicleState::kU);
        request.currentLateralSpeedMps = currentState(VehicleState::kV);
        request.currentYawRateRadps = currentState(VehicleState::kR);
        request.currentLeftWheelSpeedRadps = currentState(VehicleState::kOmegaL);
        request.currentRightWheelSpeedRadps = currentState(VehicleState::kOmegaR);
        request.desiredLongitudinalAccelMps2 = desiredLongitudinalAccelMps2;
        request.desiredYawAccelRadps2 = desiredYawAccelRadps2;
        request.fanDutyCycle = fanDutyCycle;
        request.batteryVoltageV = batteryVoltageV;
        request.reserveUsage = 1.0f;
        request.closedLoopReserveMode = false;
        return _plantModel.solveFeedforwardCanonical(request, _preparedParams, buildFeedforwardPolicyModifiers());
    }

    DriveCommandSolution SrUkfCore::solveAlignedDriveCommands(
        float forwardVelocityMps,
        float desiredLongitudinalAccelMps2,
        float yawRateRadps,
        float desiredYawAccelRadps2,
        float fanDutyCycle,
        float batteryVoltageV) const noexcept
    {
        PlantModel::FeedforwardRequest request{};
        request.currentForwardSpeedMps = forwardVelocityMps;
        request.currentLateralSpeedMps = 0.0f;
        request.currentYawRateRadps = yawRateRadps;
        request.currentLeftWheelSpeedRadps =
            (forwardVelocityMps + (_preparedParams.halfTrackWidthM * yawRateRadps)) * _preparedParams.invWheelRadiusM;
        request.currentRightWheelSpeedRadps =
            (forwardVelocityMps - (_preparedParams.halfTrackWidthM * yawRateRadps)) * _preparedParams.invWheelRadiusM;
        request.desiredLongitudinalAccelMps2 = desiredLongitudinalAccelMps2;
        request.desiredYawAccelRadps2 = desiredYawAccelRadps2;
        request.fanDutyCycle = fanDutyCycle;
        request.batteryVoltageV = batteryVoltageV;
        request.reserveUsage = 1.0f;
        request.closedLoopReserveMode = false;
        return _plantModel.solveFeedforwardCanonical(request, _preparedParams, buildFeedforwardPolicyModifiers());
    }

    DriveCommandSolution SrUkfCore::solveAlignedDriveCommandsForVelocityTarget(
        const StateVector& currentState,
        float targetForwardVelocityMps,
        float targetYawRateRadps,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS) const noexcept
    {
        float desiredLongitudinalAccelMps2 = 0.0f;
        float desiredYawAccelRadps2 = 0.0f;
        _plantModel.ComputeBodyAction(
            currentState(VehicleState::kU),
            targetForwardVelocityMps,
            currentState(VehicleState::kR),
            targetYawRateRadps,
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
            responseTimeS,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
        PlantModel::FeedforwardRequest request{};
        request.currentState = currentState;
        request.hasCurrentState = true;
        request.currentForwardSpeedMps = currentState(VehicleState::kU);
        request.currentLateralSpeedMps = currentState(VehicleState::kV);
        request.currentYawRateRadps = currentState(VehicleState::kR);
        request.currentLeftWheelSpeedRadps = currentState(VehicleState::kOmegaL);
        request.currentRightWheelSpeedRadps = currentState(VehicleState::kOmegaR);
        request.desiredLongitudinalAccelMps2 = desiredLongitudinalAccelMps2;
        request.desiredYawAccelRadps2 = desiredYawAccelRadps2;
        request.fanDutyCycle = fanDutyCycle;
        request.batteryVoltageV = batteryVoltageV;
        request.reserveUsage = 1.0f;
        request.closedLoopReserveMode = false;
        request.hasVelocityTargets = true;
        request.targetForwardVelocityMps = targetForwardVelocityMps;
        request.targetYawRateRadps = targetYawRateRadps;
        return _plantModel.solveFeedforwardCanonical(request, _preparedParams, buildFeedforwardPolicyModifiers());
    }

    DriveCommandSolution SrUkfCore::solveAlignedDriveCommandsForVelocityTarget(
        float currentForwardVelocityMps,
        float targetForwardVelocityMps,
        float currentYawRateRadps,
        float targetYawRateRadps,
        float fanDutyCycle,
        float batteryVoltageV,
        float responseTimeS) const noexcept
    {
        float desiredLongitudinalAccelMps2 = 0.0f;
        float desiredYawAccelRadps2 = 0.0f;
        _plantModel.ComputeBodyAction(
            currentForwardVelocityMps,
            targetForwardVelocityMps,
            currentYawRateRadps,
            targetYawRateRadps,
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
            responseTimeS,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2);
        PlantModel::FeedforwardRequest request{};
        request.currentForwardSpeedMps = currentForwardVelocityMps;
        request.currentLateralSpeedMps = 0.0f;
        request.currentYawRateRadps = currentYawRateRadps;
        request.currentLeftWheelSpeedRadps =
            (currentForwardVelocityMps + (_preparedParams.halfTrackWidthM * currentYawRateRadps)) * _preparedParams.invWheelRadiusM;
        request.currentRightWheelSpeedRadps =
            (currentForwardVelocityMps - (_preparedParams.halfTrackWidthM * currentYawRateRadps)) * _preparedParams.invWheelRadiusM;
        request.desiredLongitudinalAccelMps2 = desiredLongitudinalAccelMps2;
        request.desiredYawAccelRadps2 = desiredYawAccelRadps2;
        request.fanDutyCycle = fanDutyCycle;
        request.batteryVoltageV = batteryVoltageV;
        request.reserveUsage = 1.0f;
        request.closedLoopReserveMode = false;
        request.hasVelocityTargets = true;
        request.targetForwardVelocityMps = targetForwardVelocityMps;
        request.targetYawRateRadps = targetYawRateRadps;
        return _plantModel.solveFeedforwardCanonical(request, _preparedParams, buildFeedforwardPolicyModifiers());
    }

    void SrUkfCore::alignedVelocityTargetTechnicalLimits(
        const StateVector& currentState,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2,
        float fanDutyCycle,
        float reserveUsage) const noexcept
    {
        maxLongitudinalAccelMps2 = 0.0f;
        maxYawAccelRadps2 = 0.0f;
        constexpr float kLargeRequestedAccelMagnitude = 1.0e6f;
        const PlantModel::FeedforwardEnvelopeModifiers policyModifiers = buildFeedforwardPolicyModifiers();

        const DriveCommandSolution positiveLongitudinal =
            [&]() noexcept
            {
                PlantModel::FeedforwardRequest request{};
                request.currentState = currentState;
                request.hasCurrentState = true;
                request.currentForwardSpeedMps = currentState(VehicleState::kU);
                request.currentLateralSpeedMps = currentState(VehicleState::kV);
                request.currentYawRateRadps = currentState(VehicleState::kR);
                request.currentLeftWheelSpeedRadps = currentState(VehicleState::kOmegaL);
                request.currentRightWheelSpeedRadps = currentState(VehicleState::kOmegaR);
                request.desiredLongitudinalAccelMps2 = kLargeRequestedAccelMagnitude;
                request.desiredYawAccelRadps2 = 0.0f;
                request.fanDutyCycle = fanDutyCycle;
                request.batteryVoltageV = 0.0f;
                request.reserveUsage = reserveUsage;
                request.closedLoopReserveMode = true;
                return _plantModel.solveFeedforwardCanonical(request, _preparedParams, policyModifiers);
            }();
        const DriveCommandSolution negativeLongitudinal =
            [&]() noexcept
            {
                PlantModel::FeedforwardRequest request{};
                request.currentState = currentState;
                request.hasCurrentState = true;
                request.currentForwardSpeedMps = currentState(VehicleState::kU);
                request.currentLateralSpeedMps = currentState(VehicleState::kV);
                request.currentYawRateRadps = currentState(VehicleState::kR);
                request.currentLeftWheelSpeedRadps = currentState(VehicleState::kOmegaL);
                request.currentRightWheelSpeedRadps = currentState(VehicleState::kOmegaR);
                request.desiredLongitudinalAccelMps2 = -kLargeRequestedAccelMagnitude;
                request.desiredYawAccelRadps2 = 0.0f;
                request.fanDutyCycle = fanDutyCycle;
                request.batteryVoltageV = 0.0f;
                request.reserveUsage = reserveUsage;
                request.closedLoopReserveMode = true;
                return _plantModel.solveFeedforwardCanonical(request, _preparedParams, policyModifiers);
            }();
        const DriveCommandSolution positiveYaw =
            [&]() noexcept
            {
                PlantModel::FeedforwardRequest request{};
                request.currentState = currentState;
                request.hasCurrentState = true;
                request.currentForwardSpeedMps = currentState(VehicleState::kU);
                request.currentLateralSpeedMps = currentState(VehicleState::kV);
                request.currentYawRateRadps = currentState(VehicleState::kR);
                request.currentLeftWheelSpeedRadps = currentState(VehicleState::kOmegaL);
                request.currentRightWheelSpeedRadps = currentState(VehicleState::kOmegaR);
                request.desiredLongitudinalAccelMps2 = 0.0f;
                request.desiredYawAccelRadps2 = kLargeRequestedAccelMagnitude;
                request.fanDutyCycle = fanDutyCycle;
                request.batteryVoltageV = 0.0f;
                request.reserveUsage = reserveUsage;
                request.closedLoopReserveMode = true;
                return _plantModel.solveFeedforwardCanonical(request, _preparedParams, policyModifiers);
            }();
        const DriveCommandSolution negativeYaw =
            [&]() noexcept
            {
                PlantModel::FeedforwardRequest request{};
                request.currentState = currentState;
                request.hasCurrentState = true;
                request.currentForwardSpeedMps = currentState(VehicleState::kU);
                request.currentLateralSpeedMps = currentState(VehicleState::kV);
                request.currentYawRateRadps = currentState(VehicleState::kR);
                request.currentLeftWheelSpeedRadps = currentState(VehicleState::kOmegaL);
                request.currentRightWheelSpeedRadps = currentState(VehicleState::kOmegaR);
                request.desiredLongitudinalAccelMps2 = 0.0f;
                request.desiredYawAccelRadps2 = -kLargeRequestedAccelMagnitude;
                request.fanDutyCycle = fanDutyCycle;
                request.batteryVoltageV = 0.0f;
                request.reserveUsage = reserveUsage;
                request.closedLoopReserveMode = true;
                return _plantModel.solveFeedforwardCanonical(request, _preparedParams, policyModifiers);
            }();

        const float positiveLongitudinalLimitMps2 = std::fabs(positiveLongitudinal.commandedLongitudinalAccelMps2);
        const float negativeLongitudinalLimitMps2 = std::fabs(negativeLongitudinal.commandedLongitudinalAccelMps2);
        if (std::isfinite(positiveLongitudinalLimitMps2) && std::isfinite(negativeLongitudinalLimitMps2))
        {
            maxLongitudinalAccelMps2 = (std::min)(positiveLongitudinalLimitMps2, negativeLongitudinalLimitMps2);
        }

        const float positiveYawLimitRadps2 = std::fabs(positiveYaw.commandedYawAccelRadps2);
        const float negativeYawLimitRadps2 = std::fabs(negativeYaw.commandedYawAccelRadps2);
        if (std::isfinite(positiveYawLimitRadps2) && std::isfinite(negativeYawLimitRadps2))
        {
            maxYawAccelRadps2 = (std::min)(positiveYawLimitRadps2, negativeYawLimitRadps2);
        }
    }

    FeedforwardAuditResult SrUkfCore::evaluateAlignedFeedforwardOffline(
        const StateVector& currentState,
        float desiredLongitudinalAccelMps2,
        float desiredYawAccelRadps2,
        float fanDutyCycle,
        float batteryVoltageV,
        float reserveUsage,
        float dtS) const noexcept
    {
        return _plantModel.evaluateFeedforwardOfflineInternal(
            currentState,
            desiredLongitudinalAccelMps2,
            desiredYawAccelRadps2,
            _preparedParams,
            fanDutyCycle,
            batteryVoltageV,
            reserveUsage,
            dtS,
            buildFeedforwardPolicyModifiers());
    }

    SrUkfCore::StateVector SrUkfCore::IntegrateStationaryHoldState(
        const StateVector& currentState,
        float dtS) noexcept
    {
        StateVector nextState = currentState;
        const float bodyDecayAlpha = ResolveStationaryDecayAlpha(dtS, kStationaryBodyDecayTauS);
        const float wheelDecayAlpha = ResolveStationaryDecayAlpha(dtS, kStationaryWheelDecayTauS);
        nextState(VehicleState::kPx) = currentState(VehicleState::kPx);
        nextState(VehicleState::kPy) = currentState(VehicleState::kPy);
        nextState(VehicleState::kPsi) = VehicleState::NormalizeAngle(currentState(VehicleState::kPsi));
        nextState(VehicleState::kU) = bodyDecayAlpha * currentState(VehicleState::kU);
        nextState(VehicleState::kV) = bodyDecayAlpha * currentState(VehicleState::kV);
        nextState(VehicleState::kR) = bodyDecayAlpha * currentState(VehicleState::kR);
        nextState(VehicleState::kOmegaL) = wheelDecayAlpha * currentState(VehicleState::kOmegaL);
        nextState(VehicleState::kOmegaR) = wheelDecayAlpha * currentState(VehicleState::kOmegaR);
        nextState(VehicleState::kBgz) = currentState(VehicleState::kBgz);
        VehicleState::NormalizeStateVector(nextState);
        return nextState;
    }

    float SrUkfCore::ComputeNonholonomicSigmaMps(float absForwardSpeedMps) noexcept
    {
        const float resolvedForwardSpeedMps =
            (std::isfinite(absForwardSpeedMps) && (absForwardSpeedMps > 0.0f)) ?
            absForwardSpeedMps :
            0.0f;
        const float sigmaMps =
            MazeMap::Math::Sqrtf(
                (Tuning().nhcBaseSigmaMps * Tuning().nhcBaseSigmaMps) +
                ((Tuning().nhcSpeedSlopePerMps * resolvedForwardSpeedMps) *
                    (Tuning().nhcSpeedSlopePerMps * resolvedForwardSpeedMps)));
        return (std::clamp)(sigmaMps, Tuning().nhcMinimumSigmaMps, Tuning().nhcMaximumSigmaMps);
    }

    bool SrUkfCore::IsStationaryCandidate(
        const App::Internal::CommandVector& control,
        float commandedLinearMps,
        float commandedAngularRadps,
        const EncoderObs& observation,
        float gyroRawRadps,
        float gyroBiasAnchorRadps,
        float accelBodyXMps2,
        float accelBodyYMps2,
        std::uint16_t saturationFlags) noexcept
    {
        return
            std::isfinite(commandedLinearMps) &&
            std::fabs(commandedLinearMps) < Tuning().stationaryCandidateMaxLinearCommandMps &&
            std::isfinite(commandedAngularRadps) &&
            std::fabs(commandedAngularRadps) < Tuning().stationaryCandidateMaxAngularCommandRadps &&
            std::isfinite(control.LeftMotorPwm()) &&
            std::fabs(control.LeftMotorPwm()) < Tuning().stationaryCandidateMaxDriveCommand &&
            std::isfinite(control.RightMotorPwm()) &&
            std::fabs(control.RightMotorPwm()) < Tuning().stationaryCandidateMaxDriveCommand &&
            std::isfinite(observation.omegaLeftRadps) &&
            std::fabs(observation.omegaLeftRadps) < Tuning().stationaryCandidateMaxEncoderOmegaRadps &&
            std::isfinite(observation.omegaRightRadps) &&
            std::fabs(observation.omegaRightRadps) < Tuning().stationaryCandidateMaxEncoderOmegaRadps &&
            std::isfinite(gyroRawRadps) &&
            std::isfinite(gyroBiasAnchorRadps) &&
            std::fabs(gyroRawRadps - gyroBiasAnchorRadps) < Tuning().stationaryCandidateMaxCorrectedGyroRadps &&
            std::isfinite(accelBodyXMps2) &&
            std::fabs(accelBodyXMps2) < Tuning().stationaryCandidateMaxAccelMps2 &&
            std::isfinite(accelBodyYMps2) &&
            std::fabs(accelBodyYMps2) < Tuning().stationaryCandidateMaxAccelMps2 &&
            (saturationFlags == 0U);
    }

    bool SrUkfCore::HasLaunchOrReversalTrigger(
        float forwardSpeedMps,
        float leftDriveCommand,
        float rightDriveCommand,
        float leftLaunchAssistFloor,
        float rightLaunchAssistFloor,
        bool recentCommandSignFlip,
        bool recentStationaryExit) noexcept
    {
        return
            recentCommandSignFlip ||
            (leftLaunchAssistFloor > 0.0f) ||
            (rightLaunchAssistFloor > 0.0f) ||
            (std::isfinite(forwardSpeedMps) &&
                (std::fabs(forwardSpeedMps) < Tuning().launchLowSpeedThresholdMps) &&
                std::isfinite(leftDriveCommand) &&
                std::isfinite(rightDriveCommand) &&
                (std::fabs(leftDriveCommand - rightDriveCommand) > Tuning().launchDriveCommandDeltaThreshold)) ||
            recentStationaryExit;
    }

    bool SrUkfCore::HasInconsistentOrSaturatedTrigger(
        std::uint16_t saturationFlags,
        float yawConsistencyLowPassRadps,
        float yawWindowMismatchRad,
        bool nhcEnabled,
        float nhcResidualSigma) noexcept
    {
        return
            (saturationFlags != 0U) ||
            (std::isfinite(yawConsistencyLowPassRadps) &&
                (yawConsistencyLowPassRadps > Tuning().yawConsistencyLowPassThresholdRadps)) ||
            (std::isfinite(yawWindowMismatchRad) &&
                (yawWindowMismatchRad > Tuning().yawWindowMismatchThresholdRad)) ||
            (nhcEnabled &&
                std::isfinite(nhcResidualSigma) &&
                (std::fabs(nhcResidualSigma) > Tuning().nhcResidualTripSigma));
    }

    SrUkfCore::OperatingMode SrUkfCore::ClassifyOperatingMode(
        bool stationaryCertified,
        bool launchOrReversalActive,
        bool inconsistentOrSaturatedActive) noexcept
    {
        if (stationaryCertified)
        {
            return OperatingMode::StationaryCertified;
        }
        if (inconsistentOrSaturatedActive)
        {
            return OperatingMode::InconsistentOrSaturated;
        }
        if (launchOrReversalActive)
        {
            return OperatingMode::LaunchOrReversalTransient;
        }
        return OperatingMode::GripLinear;
    }

    bool SrUkfCore::IsYawValidForFeedforward(
        OperatingMode mode,
        float bgzRadps,
        float gyroBiasAnchorRadps,
        float yawConsistencyLowPassRadps,
        bool nhcEnabled,
        float lateralVelocityMps,
        float nhcSigmaMps) noexcept
    {
        if (mode == OperatingMode::InconsistentOrSaturated)
        {
            return false;
        }
        if (!std::isfinite(bgzRadps) ||
            !std::isfinite(gyroBiasAnchorRadps) ||
            (std::fabs(bgzRadps - gyroBiasAnchorRadps) >= Tuning().yawValidityBiasDeltaMaxRadps))
        {
            return false;
        }
        if (!std::isfinite(yawConsistencyLowPassRadps) ||
            (yawConsistencyLowPassRadps >= Tuning().yawConsistencyLowPassThresholdRadps))
        {
            return false;
        }
        if (nhcEnabled)
        {
            if (!(std::isfinite(nhcSigmaMps) && (nhcSigmaMps > 0.0f)) ||
                !std::isfinite(lateralVelocityMps) ||
                (std::fabs(lateralVelocityMps) >= (Tuning().nhcResidualTripSigma * nhcSigmaMps)))
            {
                return false;
            }
        }
        return true;
    }

    void SrUkfCore::ProjectMaskedStateAndSquareRootCovariance(
        const StateVector& priorState,
        const StateMatrix& priorSqrtCovariance,
        const StateVector& updatedState,
        const StateMatrix& updatedSqrtCovariance,
        const int* allowedIndices,
        std::size_t allowedCount,
        StateVector& projectedState,
        StateMatrix& projectedSqrtCovariance) noexcept
    {
        projectedState = priorState;
        projectedSqrtCovariance = priorSqrtCovariance;
        if ((allowedIndices == nullptr) || (allowedCount == 0U))
        {
            return;
        }

        std::array<bool, VehicleState::kDimension> allowedMask{};
        for (std::size_t index = 0; index < allowedCount; ++index)
        {
            const int stateIndex = allowedIndices[index];
            if ((stateIndex < 0) || (stateIndex >= VehicleState::kDimension))
            {
                return;
            }
            allowedMask[static_cast<std::size_t>(stateIndex)] = true;
            projectedState(stateIndex) = updatedState(stateIndex);
        }

        std::array<int, VehicleState::kDimension> sortedAllowedIndices{};
        int sortedAllowedCount = 0;
        std::array<int, VehicleState::kDimension> sortedBlockedIndices{};
        int sortedBlockedCount = 0;
        for (int stateIndex = 0; stateIndex < VehicleState::kDimension; ++stateIndex)
        {
            if (allowedMask[static_cast<std::size_t>(stateIndex)])
            {
                sortedAllowedIndices[static_cast<std::size_t>(sortedAllowedCount)] = stateIndex;
                ++sortedAllowedCount;
            }
            else
            {
                sortedBlockedIndices[static_cast<std::size_t>(sortedBlockedCount)] = stateIndex;
                ++sortedBlockedCount;
            }
        }

        const auto buildCovarianceSubmatrix =
            [](const StateMatrix& sqrtCovariance,
                const std::array<int, VehicleState::kDimension>& sortedIndices,
                int indexCount) noexcept
            {
                Eigen::MatrixXf covariance = Eigen::MatrixXf::Zero(indexCount, indexCount);
                for (int row = 0; row < indexCount; ++row)
                {
                    const int globalRow = sortedIndices[static_cast<std::size_t>(row)];
                    for (int col = 0; col <= row; ++col)
                    {
                        const int globalCol = sortedIndices[static_cast<std::size_t>(col)];
                        const int limit = (std::min)(globalRow, globalCol);
                        float covarianceValue = 0.0f;
                        for (int inner = 0; inner <= limit; ++inner)
                        {
                            covarianceValue += sqrtCovariance(globalRow, inner) * sqrtCovariance(globalCol, inner);
                        }
                        covariance(row, col) = covarianceValue;
                        covariance(col, row) = covarianceValue;
                    }
                }
                return covariance;
            };
        const auto factorCovarianceSubmatrix =
            [&buildCovarianceSubmatrix](
                const StateMatrix& sqrtCovariance,
                const std::array<int, VehicleState::kDimension>& sortedIndices,
                int indexCount,
                Eigen::MatrixXf& sqrtSubmatrix) noexcept
            {
                if (indexCount <= 0)
                {
                    sqrtSubmatrix.resize(0, 0);
                    return true;
                }

                const Eigen::MatrixXf covariance = buildCovarianceSubmatrix(sqrtCovariance, sortedIndices, indexCount);
                Eigen::LLT<Eigen::MatrixXf> llt;
                llt.compute(covariance);
                if (llt.info() == Eigen::Success)
                {
                    sqrtSubmatrix = llt.matrixL();
                    return true;
                }

                float jitter = 1.0e-9f;
                const float scale = (std::max)(1.0f, covariance.diagonal().cwiseAbs().maxCoeff());
                for (int attempt = 0; attempt < 12; ++attempt)
                {
                    Eigen::MatrixXf regularized = covariance;
                    regularized.diagonal().array() += jitter * scale;
                    llt.compute(regularized);
                    if (llt.info() == Eigen::Success)
                    {
                        sqrtSubmatrix = llt.matrixL();
                        return true;
                    }
                    jitter *= 10.0f;
                }

                return false;
            };
        const auto scatterSquareRootSubmatrix =
            [](const Eigen::MatrixXf& sqrtSubmatrix,
                const std::array<int, VehicleState::kDimension>& sortedIndices,
                int indexCount,
                StateMatrix& destination) noexcept
            {
                for (int row = 0; row < indexCount; ++row)
                {
                    const int globalRow = sortedIndices[static_cast<std::size_t>(row)];
                    for (int col = 0; col <= row; ++col)
                    {
                        const int globalCol = sortedIndices[static_cast<std::size_t>(col)];
                        destination(globalRow, globalCol) = sqrtSubmatrix(row, col);
                    }
                }
            };

        Eigen::MatrixXf allowedSqrtCovariance;
        Eigen::MatrixXf blockedSqrtCovariance;
        if (factorCovarianceSubmatrix(
                updatedSqrtCovariance,
                sortedAllowedIndices,
                sortedAllowedCount,
                allowedSqrtCovariance) &&
            factorCovarianceSubmatrix(
                priorSqrtCovariance,
                sortedBlockedIndices,
                sortedBlockedCount,
                blockedSqrtCovariance))
        {
            projectedSqrtCovariance.setZero();
            scatterSquareRootSubmatrix(
                blockedSqrtCovariance,
                sortedBlockedIndices,
                sortedBlockedCount,
                projectedSqrtCovariance);
            scatterSquareRootSubmatrix(
                allowedSqrtCovariance,
                sortedAllowedIndices,
                sortedAllowedCount,
                projectedSqrtCovariance);
        }
        else
        {
            const StateMatrix priorCovariance = priorSqrtCovariance * priorSqrtCovariance.transpose();
            const StateMatrix updatedCovariance = updatedSqrtCovariance * updatedSqrtCovariance.transpose();
            StateMatrix projectedCovariance = priorCovariance;
            for (int row = 0; row < VehicleState::kDimension; ++row)
            {
                for (int col = 0; col < VehicleState::kDimension; ++col)
                {
                    const bool rowAllowed = allowedMask[static_cast<std::size_t>(row)];
                    const bool colAllowed = allowedMask[static_cast<std::size_t>(col)];
                    if (rowAllowed && colAllowed)
                    {
                        projectedCovariance(row, col) = updatedCovariance(row, col);
                    }
                    else if (rowAllowed != colAllowed)
                    {
                        projectedCovariance(row, col) = 0.0f;
                    }
                }
            }

            StateMatrix fallbackSqrtCovariance = priorSqrtCovariance;
            if (SrUkfMath<VehicleState::kDimension>::FactorCovariance(projectedCovariance, fallbackSqrtCovariance))
            {
                projectedSqrtCovariance = fallbackSqrtCovariance;
            }
        }

        VehicleState::NormalizeStateVector(projectedState);
    }

    bool SrUkfCore::IsPivotScrubCandidate(
        const EncoderObs& observation,
        float commandedLinearMps,
        float commandedAngularRadps,
        float yawConsistencyLowPassRadps,
        float yawWindowMismatchRad,
        const PlantParams& params) noexcept
    {
        if (!std::isfinite(commandedLinearMps) ||
            !std::isfinite(commandedAngularRadps) ||
            !std::isfinite(observation.omegaLeftRadps) ||
            !std::isfinite(observation.omegaRightRadps))
        {
            return false;
        }

        if ((std::fabs(commandedLinearMps) > Tuning().pivotScrubMaxCommandLinearMps) ||
            (std::fabs(commandedAngularRadps) < Tuning().pivotScrubMinCommandAngularRadps))
        {
            return false;
        }

        const float measuredLinearSpeedMps =
            std::fabs(ComputeMeasuredLinearSpeedMps(observation, params));
        if (!std::isfinite(measuredLinearSpeedMps) ||
            (measuredLinearSpeedMps > Tuning().pivotScrubMaxCommandLinearMps))
        {
            return false;
        }

        const bool wheelOpposing = (observation.omegaLeftRadps * observation.omegaRightRadps) < 0.0f;
        if (!wheelOpposing)
        {
            return false;
        }

        bool yawConflictKnown = false;
        bool yawConflict = false;
        if (std::isfinite(yawConsistencyLowPassRadps))
        {
            yawConflictKnown = true;
            yawConflict =
                yawConflict ||
                (std::fabs(yawConsistencyLowPassRadps) > Tuning().pivotScrubYawConsistencyThresholdRadps);
        }
        if (std::isfinite(yawWindowMismatchRad))
        {
            yawConflictKnown = true;
            yawConflict =
                yawConflict ||
                (std::fabs(yawWindowMismatchRad) > Tuning().pivotScrubYawWindowMismatchThresholdRad);
        }

        return yawConflictKnown && yawConflict;
    }

    void SrUkfCore::resetPivotScrubTelemetry() noexcept
    {
        _pivotScrubMode = false;
        _pivotScrubEncoderBodyUpdateSkipped = false;
        _pivotScrubZeroUSoftApplied = false;
        _pivotScrubEncoderWheelDeltaPsiRad = 0.0f;
        _pivotScrubEncoderWheelDeltaRRadps = 0.0f;
        _pivotScrubEncoderWheelDeltaOmegaLRadps = 0.0f;
        _pivotScrubEncoderWheelDeltaOmegaRRadps = 0.0f;
        _pivotScrubEncoderWheelMaskedDeltaNorm = 0.0f;
        _pivotScrubZeroUInnovationMps = 0.0f;
        _pivotScrubZeroUDeltaMps = 0.0f;
        _pivotScrubGyroDeltaPsiRad = 0.0f;
        _pivotScrubGyroDeltaRRadps = 0.0f;
        _pivotScrubGyroDeltaBgzRadps = 0.0f;
        _pivotScrubGyroDeltaOmegaLRadps = 0.0f;
        _pivotScrubGyroDeltaOmegaRRadps = 0.0f;
        _pivotScrubGyroMaskedDeltaNorm = 0.0f;
    }

    bool SrUkfCore::WriteDebugTextDump(void* context, DebugTextSink sink) const noexcept
    {
        if (sink == nullptr)
        {
            return false;
        }

        if (!EmitNamedFloatValuesLine(
                context,
                sink,
                "ukf_dump_state",
                nullptr,
                kUkfStateFieldNames,
                [&](const std::size_t index) noexcept
                {
                    return _filter.state()(static_cast<int>(index));
                }))
        {
            return false;
        }

        const StateMatrix covariance = _filter.covariance();
        for (int row = 0; row < VehicleState::kDimension; ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_covariance_row",
                    kUkfStateFieldNames,
                    covariance,
                    row))
            {
                return false;
            }
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_prediction_reference",
                "have_prediction_reference=%s",
                _havePredictionReference ? "true" : "false"))
        {
            return false;
        }

        if (!EmitNamedFloatValuesLine(
                context,
                sink,
                "ukf_dump_pre_predict_state",
                nullptr,
                kUkfStateFieldNames,
                [&](const std::size_t index) noexcept
                {
                    return _prePredictState(static_cast<int>(index));
                }))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_last_control",
                "left_motor_pwm=%.9g;right_motor_pwm=%.9g;fan_duty_cycle=%.9g;battery_voltage_v=%.9g",
                static_cast<double>(_lastControl.LeftMotorPwm()),
                static_cast<double>(_lastControl.RightMotorPwm()),
                static_cast<double>(_lastFanDutyCycle),
                static_cast<double>(_lastBatteryVoltageV)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_last_encoder_obs",
                "total_left_counts=%ld;total_right_counts=%ld;omega_left_radps=%.9g;omega_right_radps=%.9g",
                static_cast<long>(_lastEncoderObs.totalLeftCounts),
                static_cast<long>(_lastEncoderObs.totalRightCounts),
                static_cast<double>(_lastEncoderObs.omegaLeftRadps),
                static_cast<double>(_lastEncoderObs.omegaRightRadps)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_mass_geometry",
                "mass_kg=%.9g;effective_longitudinal_mass_kg=%.9g;yaw_inertia_kg_m2=%.9g;track_width_m=%.9g;contact_patch_longitudinal_offset_m=%.9g;wheel_radius_m=%.9g;equivalent_wheel_inertia_kg_m2=%.9g",
                static_cast<double>(_params.massKg),
                static_cast<double>(_params.effectiveLongitudinalMassKg),
                static_cast<double>(_params.yawInertiaKgM2),
                static_cast<double>(_params.trackWidthM),
                static_cast<double>(_params.contactPatchLongitudinalOffsetM),
                static_cast<double>(_params.wheelRadiusM),
                static_cast<double>(_params.equivalentWheelInertiaKgM2)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_drive_electrical",
                "supply_voltage_v=%.9g;drive_resistance_ohms=%.9g;torque_constant_nm_per_a=%.9g;speed_constant_radps_per_volt=%.9g;no_load_current_a=%.9g;motor_current_limit_a=%.9g;gear_ratio=%.9g;encoder_counts_per_motor_rev=%u",
                static_cast<double>(_params.supplyVoltageV),
                static_cast<double>(_params.driveResistanceOhms),
                static_cast<double>(_params.torqueConstantNmPerA),
                static_cast<double>(_params.speedConstantRadpsPerVolt),
                static_cast<double>(_params.noLoadCurrentA),
                static_cast<double>(_params.motorCurrentLimitA),
                static_cast<double>(_params.gearRatio),
                static_cast<unsigned>(_params.encoderCountsPerMotorRev)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_tire_friction",
                "drivetrain_efficiency=%.9g;rolling_friction_torque_nm=%.9g;viscous_friction_nm_per_radps=%.9g;longitudinal_tire_stiffness_n=%.9g;cornering_stiffness_front_n_per_rad=%.9g;cornering_stiffness_rear_n_per_rad=%.9g;mu_front=%.9g;mu_rear=%.9g;front_load_fraction=%.9g",
                static_cast<double>(_params.drivetrainEfficiency),
                static_cast<double>(_params.rollingFrictionTorqueNm),
                static_cast<double>(_params.viscousFrictionNmPerRadps),
                static_cast<double>(_params.longitudinalTireStiffnessN),
                static_cast<double>(_params.corneringStiffnessFrontNPerRad),
                static_cast<double>(_params.corneringStiffnessRearNPerRad),
                static_cast<double>(_params.muFront),
                static_cast<double>(_params.muRear),
                static_cast<double>(_params.frontLoadFraction)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_static_friction",
                "static_friction_torque_nm=%.9g;static_friction_max_speed_mps=%.9g",
                static_cast<double>(_params.staticFrictionTorqueNm),
                static_cast<double>(_params.staticFrictionMaxSpeedMps)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_misc",
                "velocity_epsilon_mps=%.9g;force_epsilon_n=%.9g;fan_downforce_at_full_duty_n=%.9g;no_hit_range_m=%.9g",
                static_cast<double>(_params.velocityEpsilonMps),
                static_cast<double>(_params.forceEpsilonN),
                static_cast<double>(_params.fanDownforceAtFullDutyN),
                static_cast<double>(_params.noHitRangeM)))
        {
            return false;
        }

        for (std::size_t index = 0; index < _params.contactPositionsBodyM.size(); ++index)
        {
            const Eigen::Vector2f& position = _params.contactPositionsBodyM[index];
            if (!EmitDebugTextLine(
                    context,
                    sink,
                    "ukf_dump_contact_position",
                    "index=%u;x_m=%.9g;y_m=%.9g",
                    static_cast<unsigned>(index),
                    static_cast<double>(position.x()),
                    static_cast<double>(position.y())))
            {
                return false;
            }
        }

        auto emitSensorMount =
            [&](const char* type, const SensorMount& sensor) noexcept
            {
                const Eigen::Matrix2f& bodyFromSensor = sensor.bodyFromSensor();
                return EmitDebugTextLine(
                    context,
                    sink,
                    type,
                    "position_x_m=%.9g;position_y_m=%.9g;body_from_sensor_00=%.9g;body_from_sensor_01=%.9g;body_from_sensor_10=%.9g;body_from_sensor_11=%.9g;clockwise_yaw_sign=%.9g",
                    static_cast<double>(sensor.positionBodyM().x()),
                    static_cast<double>(sensor.positionBodyM().y()),
                    static_cast<double>(bodyFromSensor(0, 0)),
                    static_cast<double>(bodyFromSensor(0, 1)),
                    static_cast<double>(bodyFromSensor(1, 0)),
                    static_cast<double>(bodyFromSensor(1, 1)),
                    static_cast<double>(sensor.clockwiseYawSign()));
            };
        if (!emitSensorMount("ukf_dump_sensor_front_left", _params.frontLeftSensor) ||
            !emitSensorMount("ukf_dump_sensor_front_right", _params.frontRightSensor) ||
            !emitSensorMount("ukf_dump_sensor_side_left", _params.sideLeftSensor) ||
            !emitSensorMount("ukf_dump_sensor_side_right", _params.sideRightSensor))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_imu_mount",
                "position_x_m=%.9g;position_y_m=%.9g;body_from_sensor_00=%.9g;body_from_sensor_01=%.9g;body_from_sensor_10=%.9g;body_from_sensor_11=%.9g;clockwise_yaw_sign=%.9g",
                static_cast<double>(_params.backLeftImuMount.positionBodyM().x()),
                static_cast<double>(_params.backLeftImuMount.positionBodyM().y()),
                static_cast<double>(_params.backLeftImuMount.bodyFromSensor()(0, 0)),
                static_cast<double>(_params.backLeftImuMount.bodyFromSensor()(0, 1)),
                static_cast<double>(_params.backLeftImuMount.bodyFromSensor()(1, 0)),
                static_cast<double>(_params.backLeftImuMount.bodyFromSensor()(1, 1)),
                static_cast<double>(_params.backLeftImuMount.clockwiseYawSign())))
        {
            return false;
        }

        for (int row = 0; row < VehicleState::kDimension; ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_process_noise_sqrt_row",
                    kUkfStateFieldNames,
                    _sqrtProcessNoiseDensity,
                    row))
            {
                return false;
            }
        }

        for (int row = 0; row < static_cast<int>(kUkfImuNoiseFieldNames.size()); ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_imu_noise_sqrt_row",
                    kUkfImuNoiseFieldNames,
                    _sqrtImuNoise,
                    row))
            {
                return false;
            }
        }

        for (int row = 0; row < static_cast<int>(kUkfFrontNoiseFieldNames.size()); ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_front_noise_sqrt_row",
                    kUkfFrontNoiseFieldNames,
                    _sqrtFrontNoise,
                    row))
            {
                return false;
            }
        }

        if (!EmitNamedMatrixRowLine(
                context,
                sink,
                "ukf_dump_side_noise_sqrt_row",
                kUkfSideNoiseFieldNames,
                _sqrtSideNoise,
                0))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_mode",
                "mode_id=%u;stationary_certified=%s;bias_update_enabled=%s;nhc_enabled=%s;yaw_valid_for_feedforward=%s",
                static_cast<unsigned>(operatingModeId()),
                _stationaryCertified ? "true" : "false",
                _biasUpdateEnabled ? "true" : "false",
                _nonholonomicConstraintEnabled ? "true" : "false",
                _yawValidForFeedforward ? "true" : "false"))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_pivot_scrub",
                "pivot_scrub_mode=%s;encoder_body_update_skipped=%s;zero_u_soft_applied=%s",
                _pivotScrubMode ? "true" : "false",
                _pivotScrubEncoderBodyUpdateSkipped ? "true" : "false",
                _pivotScrubZeroUSoftApplied ? "true" : "false"))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_pivot_scrub_encoder",
                "delta_psi_rad=%.9g;delta_r_radps=%.9g;delta_omega_l_radps=%.9g;delta_omega_r_radps=%.9g;masked_delta_norm=%.9g",
                static_cast<double>(_pivotScrubEncoderWheelDeltaPsiRad),
                static_cast<double>(_pivotScrubEncoderWheelDeltaRRadps),
                static_cast<double>(_pivotScrubEncoderWheelDeltaOmegaLRadps),
                static_cast<double>(_pivotScrubEncoderWheelDeltaOmegaRRadps),
                static_cast<double>(_pivotScrubEncoderWheelMaskedDeltaNorm)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_pivot_scrub_zero_u",
                "innovation_mps=%.9g;delta_mps=%.9g",
                static_cast<double>(_pivotScrubZeroUInnovationMps),
                static_cast<double>(_pivotScrubZeroUDeltaMps)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_pivot_scrub_gyro",
                "delta_psi_rad=%.9g;delta_r_radps=%.9g;delta_bgz_radps=%.9g;delta_omega_l_radps=%.9g;delta_omega_r_radps=%.9g;masked_delta_norm=%.9g",
                static_cast<double>(_pivotScrubGyroDeltaPsiRad),
                static_cast<double>(_pivotScrubGyroDeltaRRadps),
                static_cast<double>(_pivotScrubGyroDeltaBgzRadps),
                static_cast<double>(_pivotScrubGyroDeltaOmegaLRadps),
                static_cast<double>(_pivotScrubGyroDeltaOmegaRRadps),
                static_cast<double>(_pivotScrubGyroMaskedDeltaNorm)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_consistency",
                "gyro_bias_anchor_radps=%.9g;yaw_consistency_lp_radps=%.9g;yaw_window_mismatch_rad=%.9g;nhc_sigma_mps=%.9g;nhc_residual_mps=%.9g;nhc_residual_sigma=%.9g",
                static_cast<double>(_gyroBiasAnchorRadps),
                static_cast<double>(_yawConsistencyLowPassRadps),
                static_cast<double>(_yawWindowMismatchRad),
                static_cast<double>(_nhcSigmaMps),
                static_cast<double>(_nhcResidualMps),
                static_cast<double>(_nhcResidualSigma)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_frozen_policy_state",
                "dt_s=%.9g;exact_stationary_lock=%s;closure_residual_left_mps=%.9g;closure_residual_right_mps=%.9g;longitudinal_closure_severity=%.9g;differential_closure_severity=%.9g;lateral_accel_severity=%.9g;yaw_consistency_severity=%.9g;left_pre_projection_utilization=%.9g;right_pre_projection_utilization=%.9g;left_memory=%.9g;right_memory=%.9g;left_recovery_score=%.9g;right_recovery_score=%.9g;left_recovery_time_remaining_s=%.9g;right_recovery_time_remaining_s=%.9g;left_holdoff_active=%s;right_holdoff_active=%s;left_applied_bank_torque_nm=%.9g;right_applied_bank_torque_nm=%.9g",
                static_cast<double>(_frozenDtS),
                _frozenSchedule.exactStationaryLock ? "true" : "false",
                static_cast<double>(_lastClosureResidualLeftMps),
                static_cast<double>(_lastClosureResidualRightMps),
                static_cast<double>(_frozenGripUtilization.longitudinalClosureSeverity),
                static_cast<double>(_frozenGripUtilization.differentialClosureSeverity),
                static_cast<double>(_frozenGripUtilization.lateralAccelerationSeverity),
                static_cast<double>(_frozenGripUtilization.yawConsistencySeverity),
                static_cast<double>(_frozenGripUtilization.leftBankPreProjectionUtilization),
                static_cast<double>(_frozenGripUtilization.rightBankPreProjectionUtilization),
                static_cast<double>(_transientContactMemory.leftBankMemory),
                static_cast<double>(_transientContactMemory.rightBankMemory),
                static_cast<double>(_regripRecovery.leftBankRecoveryScore),
                static_cast<double>(_regripRecovery.rightBankRecoveryScore),
                static_cast<double>(_regripRecovery.leftBankRecoveryTimeRemainingS),
                static_cast<double>(_regripRecovery.rightBankRecoveryTimeRemainingS),
                _frozenSchedule.leftBankHoldoffActive ? "true" : "false",
                _frozenSchedule.rightBankHoldoffActive ? "true" : "false",
                static_cast<double>(_frozenAppliedTorque.leftAppliedBankTorqueNm),
                static_cast<double>(_frozenAppliedTorque.rightAppliedBankTorqueNm)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_schedule_scales",
                "forward_process_noise_scale=%.9g;lateral_process_noise_scale=%.9g;yaw_rate_process_noise_scale=%.9g;left_wheel_process_noise_scale=%.9g;right_wheel_process_noise_scale=%.9g;left_closure_covariance_scale=%.9g;right_closure_covariance_scale=%.9g;lateral_pseudo_covariance_scale=%.9g;launch_hold_remaining_s=%.9g;inconsistent_hold_remaining_s=%.9g;stationary_candidate_dwell_s=%.9g;time_since_stationary_exit_s=%.9g;nhc_reenable_delay_remaining_s=%.9g",
                static_cast<double>(_frozenSchedule.forwardSpeedProcessNoiseScale),
                static_cast<double>(_frozenSchedule.lateralSpeedProcessNoiseScale),
                static_cast<double>(_frozenSchedule.yawRateProcessNoiseScale),
                static_cast<double>(_frozenSchedule.leftWheelSpeedProcessNoiseScale),
                static_cast<double>(_frozenSchedule.rightWheelSpeedProcessNoiseScale),
                static_cast<double>(_frozenSchedule.closureCovarianceScaleLeft),
                static_cast<double>(_frozenSchedule.closureCovarianceScaleRight),
                static_cast<double>(_frozenSchedule.lateralPseudoMeasurementCovarianceScale),
                static_cast<double>(_launchHoldRemainingS),
                static_cast<double>(_inconsistentHoldRemainingS),
                static_cast<double>(_stationaryCandidateDwellS),
                static_cast<double>(_timeSinceStationaryExitS),
                static_cast<double>(_nhcReenableDelayRemainingS)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_update_metrics",
                "direct_wheel_body_invariant=%s;release_inflation_applied=%s;gyro_innovation_radps=%.9g;gyro_nis=%.9g;forward_accel_innovation_mps2=%.9g;forward_accel_nis=%.9g;lateral_accel_innovation_mps2=%.9g;lateral_accel_nis=%.9g;closure_left_innovation_mps=%.9g;closure_left_nis=%.9g;closure_right_innovation_mps=%.9g;closure_right_nis=%.9g;lateral_pseudo_innovation_mps=%.9g;lateral_pseudo_nis=%.9g;soft_odometry_enabled=%s",
                _directWheelUpdateBodyStateInvariant ? "true" : "false",
                _releaseInflationApplied ? "true" : "false",
                static_cast<double>(_lastGyroInnovationRadps),
                static_cast<double>(_lastGyroNis),
                static_cast<double>(_lastForwardAccelInnovationMps2),
                static_cast<double>(_lastForwardAccelNis),
                static_cast<double>(_lastLateralAccelInnovationMps2),
                static_cast<double>(_lastLateralAccelNis),
                static_cast<double>(_lastClosureLeftInnovationMps),
                static_cast<double>(_lastClosureLeftNis),
                static_cast<double>(_lastClosureRightInnovationMps),
                static_cast<double>(_lastClosureRightNis),
                static_cast<double>(_lastLateralPseudoInnovationMps),
                static_cast<double>(_lastLateralPseudoNis),
                SoftOdometryAid::kFeatureEnabled ? "true" : "false"))
        {
            return false;
        }

        const StateMatrix covarianceDiagonal = _filter.covariance();
        return EmitDebugTextLine(
            context,
            sink,
            "ukf_dump_covariance_diagonals",
            "forward_speed_var=%.9g;lateral_speed_var=%.9g;yaw_rate_var=%.9g;left_wheel_speed_var=%.9g;right_wheel_speed_var=%.9g;gyro_bias_var=%.9g",
            static_cast<double>(covarianceDiagonal(VehicleState::kU, VehicleState::kU)),
            static_cast<double>(covarianceDiagonal(VehicleState::kV, VehicleState::kV)),
            static_cast<double>(covarianceDiagonal(VehicleState::kR, VehicleState::kR)),
            static_cast<double>(covarianceDiagonal(VehicleState::kOmegaL, VehicleState::kOmegaL)),
            static_cast<double>(covarianceDiagonal(VehicleState::kOmegaR, VehicleState::kOmegaR)),
            static_cast<double>(covarianceDiagonal(VehicleState::kBgz, VehicleState::kBgz)));
    }

    SrUkfCore::StateMatrix SrUkfCore::BuildDefaultInitialCovariance() noexcept
    {
        StateMatrix covariance = StateMatrix::Identity() * 1.0e-3f;
        covariance(VehicleState::kPx, VehicleState::kPx) = 1.0e-5f;
        covariance(VehicleState::kPy, VehicleState::kPy) = 1.0e-5f;
        covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = 0.25f;
        covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = 0.25f;
        covariance(VehicleState::kBgz, VehicleState::kBgz) = kGyroBiasInitialVarianceUnseededRadps2;
        return covariance;
    }

    SrUkfCore::StateMatrix SrUkfCore::BuildProcessNoiseSquareRootForMode(const OperatingMode mode) noexcept
    {
        const ModeProcessNoiseConfig& config = GetModeProcessNoiseConfig(mode);
        StateMatrix sqrtNoise = StateMatrix::Zero();
        sqrtNoise.diagonal() <<
            0.0f,
            0.0f,
            0.0f,
            config.sigmaUSqrtQ,
            config.sigmaVSqrtQ,
            config.sigmaRSqrtQ,
            config.sigmaOmegaSqrtQ,
            config.sigmaOmegaSqrtQ,
            0.0f;
        return sqrtNoise;
    }

    bool SrUkfCore::reset(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
        _filter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);
        _gyroBiasAnchorRadps = std::isfinite(state(VehicleState::kBgz)) ? state(VehicleState::kBgz) : 0.0f;
        _gyroBiasAnchorVarianceRadps2 = covariance(VehicleState::kBgz, VehicleState::kBgz);
        _initialStationaryGyroBiasPhaseExited = false;
        _initialStationaryGyroBiasSeedApplied = false;
        _initialStationaryGyroBiasSampleOrdinal = 0U;
        _initialStationaryGyroBiasCollectedSeedSamples = 0U;
        _initialStationaryGyroBiasSeedAccumRadps = 0.0;
        _commandedLinearMps = 0.0f;
        _commandedAngularRadps = 0.0f;
        _saturationFlags = 0U;
        _leftLaunchAssistFloor = 0.0f;
        _rightLaunchAssistFloor = 0.0f;
        _accelBodyXMps2 = 0.0f;
        _accelBodyYMps2 = 0.0f;
        _stationaryCandidateDwellS = 0.0f;
        _stationaryCandidatePoseReferenceState = _filter.state();
        _stationaryCandidatePoseReferenceCovariance = _filter.covariance();
        _stationaryCandidatePoseReferenceValid = false;
        _stationaryCertified = false;
        _timeSinceStationaryExitS = std::numeric_limits<float>::infinity();
        _timeSinceCommandSignFlipS = std::numeric_limits<float>::infinity();
        _previousAverageDriveCommandSign = 0.0f;
        _launchHoldRemainingS = 0.0f;
        _inconsistentHoldRemainingS = 0.0f;
        _nhcReenableDelayRemainingS = 0.0f;
        _yawConsistencyLowPassRadps = 0.0f;
        _yawWindowMismatchRad = 0.0f;
        _yawConsistencyExceedDwellS = 0.0f;
        _nhcSigmaMps = ComputeNonholonomicSigmaMps(std::fabs(state(VehicleState::kU)));
        _nhcResidualMps = state(VehicleState::kV);
        _nhcResidualSigma = 0.0f;
        _nonholonomicConstraintEnabled = false;
        _operatingMode = OperatingMode::GripLinear;
        _yawValidForFeedforward = true;
        _biasUpdateEnabled = false;
        _yawWindowDtSeconds.fill(0.0f);
        _yawWindowUkfIntegralRad.fill(0.0f);
        _yawWindowGyroIntegralRad.fill(0.0f);
        _yawWindowHead = 0U;
        _yawWindowSize = 0U;
        _yawWindowSpanS = 0.0f;
        resetPivotScrubTelemetry();
        _directWheelUpdateBodyStateInvariant = false;
        _releaseInflationApplied = false;
        _lastClosureResidualLeftMps = 0.0f;
        _lastClosureResidualRightMps = 0.0f;
        _lastGyroMeasurementRadps = 0.0f;
        _lastGyroInnovationRadps = 0.0f;
        _lastGyroNis = 0.0f;
        _lastForwardAccelInnovationMps2 = 0.0f;
        _lastForwardAccelNis = 0.0f;
        _lastLateralAccelInnovationMps2 = 0.0f;
        _lastLateralAccelNis = 0.0f;
        _lastClosureLeftInnovationMps = 0.0f;
        _lastClosureLeftNis = 0.0f;
        _lastClosureRightInnovationMps = 0.0f;
        _lastClosureRightNis = 0.0f;
        _lastLateralPseudoInnovationMps = 0.0f;
        _lastLateralPseudoNis = 0.0f;
        _sqrtImuNoise(0, 0) = Tuning().imuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = Tuning().imuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = Tuning().imuAccelSigmaMps2;
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _lastControl = App::Internal::CommandVector(0.0f, 0.0f);
        _lastFanDutyCycle = 0.80f;
        _lastBatteryVoltageV = 0.0f;
        _lastEncoderObs = EncoderObs{};
        _lastEncoderDtSeconds = 0.0f;
        _prePredictState = _filter.state();
        _havePredictionReference = false;
        _acceptedEncoderUpdateSincePredict = false;
        const ModeProcessNoiseConfig& resetModeNoise = GetModeProcessNoiseConfig(_operatingMode);
        (void)_filter.floorVariance(VehicleState::kR, resetModeNoise.stdRMin * resetModeNoise.stdRMin);
        (void)_filter.floorVariance(VehicleState::kV, resetModeNoise.stdVMin * resetModeNoise.stdVMin);
        (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        _gyroBiasAnchorVarianceRadps2 = _filter.variance(VehicleState::kBgz);
        _stationaryCandidatePoseReferenceCovariance = _filter.covariance();
        _prePredictCovariance = _stationaryCandidatePoseReferenceCovariance;
        _transientContactMemory = {};
        _regripRecovery = {};
        refreshFrozenPolicyState(
            0.0f,
            App::Internal::CommandVector(0.0f, 0.0f),
            0.80f,
            0.0f);
        return true;
    }

    bool SrUkfCore::setState(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
        _filter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);
        _gyroBiasAnchorRadps = std::isfinite(state(VehicleState::kBgz)) ? state(VehicleState::kBgz) : 0.0f;
        _gyroBiasAnchorVarianceRadps2 = covariance(VehicleState::kBgz, VehicleState::kBgz);
        _initialStationaryGyroBiasPhaseExited = false;
        _initialStationaryGyroBiasSeedApplied = false;
        _initialStationaryGyroBiasSampleOrdinal = 0U;
        _initialStationaryGyroBiasCollectedSeedSamples = 0U;
        _initialStationaryGyroBiasSeedAccumRadps = 0.0;
        _commandedLinearMps = 0.0f;
        _commandedAngularRadps = 0.0f;
        _saturationFlags = 0U;
        _leftLaunchAssistFloor = 0.0f;
        _rightLaunchAssistFloor = 0.0f;
        _accelBodyXMps2 = 0.0f;
        _accelBodyYMps2 = 0.0f;
        _stationaryCandidateDwellS = 0.0f;
        _stationaryCandidatePoseReferenceState = _filter.state();
        _stationaryCandidatePoseReferenceCovariance = _filter.covariance();
        _stationaryCandidatePoseReferenceValid = false;
        _stationaryCertified = false;
        _timeSinceStationaryExitS = std::numeric_limits<float>::infinity();
        _timeSinceCommandSignFlipS = std::numeric_limits<float>::infinity();
        _previousAverageDriveCommandSign = 0.0f;
        _launchHoldRemainingS = 0.0f;
        _inconsistentHoldRemainingS = 0.0f;
        _nhcReenableDelayRemainingS = 0.0f;
        _yawConsistencyLowPassRadps = 0.0f;
        _yawWindowMismatchRad = 0.0f;
        _yawConsistencyExceedDwellS = 0.0f;
        _nhcSigmaMps = ComputeNonholonomicSigmaMps(std::fabs(state(VehicleState::kU)));
        _nhcResidualMps = state(VehicleState::kV);
        _nhcResidualSigma = 0.0f;
        _nonholonomicConstraintEnabled = false;
        _operatingMode = OperatingMode::GripLinear;
        _yawValidForFeedforward = true;
        _biasUpdateEnabled = false;
        _yawWindowDtSeconds.fill(0.0f);
        _yawWindowUkfIntegralRad.fill(0.0f);
        _yawWindowGyroIntegralRad.fill(0.0f);
        _yawWindowHead = 0U;
        _yawWindowSize = 0U;
        _yawWindowSpanS = 0.0f;
        resetPivotScrubTelemetry();
        _directWheelUpdateBodyStateInvariant = false;
        _releaseInflationApplied = false;
        _lastClosureResidualLeftMps = 0.0f;
        _lastClosureResidualRightMps = 0.0f;
        _lastGyroMeasurementRadps = 0.0f;
        _lastGyroInnovationRadps = 0.0f;
        _lastGyroNis = 0.0f;
        _lastForwardAccelInnovationMps2 = 0.0f;
        _lastForwardAccelNis = 0.0f;
        _lastLateralAccelInnovationMps2 = 0.0f;
        _lastLateralAccelNis = 0.0f;
        _lastClosureLeftInnovationMps = 0.0f;
        _lastClosureLeftNis = 0.0f;
        _lastClosureRightInnovationMps = 0.0f;
        _lastClosureRightNis = 0.0f;
        _lastLateralPseudoInnovationMps = 0.0f;
        _lastLateralPseudoNis = 0.0f;
        _sqrtImuNoise(0, 0) = Tuning().imuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = Tuning().imuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = Tuning().imuAccelSigmaMps2;
        _lastControl = App::Internal::CommandVector(0.0f, 0.0f);
        _lastFanDutyCycle = 0.80f;
        _lastBatteryVoltageV = 0.0f;
        _lastEncoderObs = EncoderObs{};
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _lastEncoderDtSeconds = 0.0f;
        _prePredictState = _filter.state();
        _havePredictionReference = false;
        _acceptedEncoderUpdateSincePredict = false;
        const ModeProcessNoiseConfig& setStateModeNoise = GetModeProcessNoiseConfig(_operatingMode);
        (void)_filter.floorVariance(VehicleState::kR, setStateModeNoise.stdRMin * setStateModeNoise.stdRMin);
        (void)_filter.floorVariance(VehicleState::kV, setStateModeNoise.stdVMin * setStateModeNoise.stdVMin);
        (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        _gyroBiasAnchorVarianceRadps2 = _filter.variance(VehicleState::kBgz);
        _stationaryCandidatePoseReferenceCovariance = _filter.covariance();
        _prePredictCovariance = _stationaryCandidatePoseReferenceCovariance;
        _transientContactMemory = {};
        _regripRecovery = {};
        refreshFrozenPolicyState(
            0.0f,
            App::Internal::CommandVector(0.0f, 0.0f),
            0.80f,
            0.0f);
        return true;
    }

    void SrUkfCore::InvokeLoopHook(void* context, LoopHookInvoker loopHook) noexcept
    {
        if (context != nullptr && loopHook != nullptr)
        {
            loopHook(context);
        }
    }

    Eigen::Matrix<float, 2, 2> SrUkfCore::ComputeGeneralEncoderPairCovarianceRadps(
        const PlantParams& params) noexcept
    {
        Eigen::Matrix<float, 2, 2> covariance = Eigen::Matrix<float, 2, 2>::Zero();
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            covariance(0, 0) = 1.0f;
            covariance(1, 1) = 1.0f;
            return covariance;
        }

        const float trackWidthM =
            (params.trackWidthM > 0.0f && std::isfinite(params.trackWidthM)) ?
            params.trackWidthM :
            Vehicle::GetPhysicalModel().trackWidthM;
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float varianceUMps2 =
            Tuning().generalEncoderLinearSpeedSigmaMps * Tuning().generalEncoderLinearSpeedSigmaMps;
        const float varianceYawRateRadps2 =
            Tuning().generalEncoderYawRateSigmaRadps * Tuning().generalEncoderYawRateSigmaRadps;
        const float varianceWheelLinearMps2 =
            varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float covarianceWheelLinearMps2 =
            varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float invWheelRadius2 = 1.0f / (params.wheelRadiusM * params.wheelRadiusM);
        covariance(0, 0) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 1) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(0, 1) = covarianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 0) = covariance(0, 1);
        return covariance;
    }

    Eigen::Matrix<float, 2, 2> SrUkfCore::ComputeGeneralEncoderPairSqrtNoise(
        const PlantParams& params) noexcept
    {
        const Eigen::Matrix<float, 2, 2> covariance = ComputeGeneralEncoderPairCovarianceRadps(params);
        const Eigen::LLT<Eigen::Matrix<float, 2, 2>> llt(covariance);
        if (llt.info() == Eigen::Success)
        {
            return llt.matrixL();
        }

        Eigen::Matrix<float, 2, 2> fallback = Eigen::Matrix<float, 2, 2>::Zero();
        fallback(0, 0) = 1.0f;
        fallback(1, 1) = 1.0f;
        return fallback;
    }

    float SrUkfCore::ComputeStationaryEncoderOmegaSigmaRadps(const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            return 1.0f;
        }

        return Tuning().stationaryEncoderVelocitySigmaMps / params.wheelRadiusM;
    }

    Eigen::Matrix<float, 2, 2> SrUkfCore::ComputeEncoderPairSqrtNoise(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            Eigen::Matrix<float, 2, 2> sqrtNoise = Eigen::Matrix<float, 2, 2>::Zero();
            const float sigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(params);
            sqrtNoise(0, 0) = sigmaRadps;
            sqrtNoise(1, 1) = sigmaRadps;
            return sqrtNoise;
        }

        return ComputeGeneralEncoderPairSqrtNoise(params);
    }

    float SrUkfCore::ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            return std::numeric_limits<float>::infinity();
        }

        return Tuning().encoderPairNisThreshold;
    }

    SrUkfCore::GripUtilizationSnapshot SrUkfCore::buildGripUtilizationSnapshot(
        const StateVector& currentState,
        const AppliedTorqueEstimate& appliedTorque,
        float leftClosureResidualMps,
        float rightClosureResidualMps,
        float fanDutyCycle) const noexcept
    {
        GripUtilizationSnapshot snapshot{};
        const PlantDerivatives derivatives =
            _plantModel.forwardStepFromAppliedBankTorques(
                currentState,
                appliedTorque.leftAppliedBankTorqueNm,
                appliedTorque.rightAppliedBankTorqueNm,
                _preparedParams,
                fanDutyCycle);
        snapshot.leftBankPreProjectionUtilization =
            (std::max)(
                0.0f,
                std::isfinite(derivatives.contactForces.LeftBankMaxPreProjectionUtilization()) ?
                    derivatives.contactForces.LeftBankMaxPreProjectionUtilization() :
                    0.0f);
        snapshot.rightBankPreProjectionUtilization =
            (std::max)(
                0.0f,
                std::isfinite(derivatives.contactForces.RightBankMaxPreProjectionUtilization()) ?
                    derivatives.contactForces.RightBankMaxPreProjectionUtilization() :
                    0.0f);

        const float closureSigmaMps = (std::max)(Tuning().generalEncoderLinearSpeedSigmaMps, 1.0e-3f);
        const float longitudinalClosureSeverity =
            (std::fabs(leftClosureResidualMps + rightClosureResidualMps) * 0.5f) / closureSigmaMps;
        const float differentialClosureSeverity =
            std::fabs(leftClosureResidualMps - rightClosureResidualMps) / closureSigmaMps;
        const float lateralAccelerationSeverity =
            (std::isfinite(_accelBodyXMps2) && (_preparedParams.raw.combinedAccelPeakMps2 > 0.0f)) ?
            (std::fabs(_accelBodyXMps2) / _preparedParams.raw.combinedAccelPeakMps2) :
            0.0f;
        const float yawConsistencySeverity =
            (Tuning().yawConsistencyLowPassThresholdRadps > 0.0f) ?
            (std::fabs(_yawConsistencyLowPassRadps) / Tuning().yawConsistencyLowPassThresholdRadps) :
            0.0f;
        const float leftBankAnomalySeverity = std::fabs(leftClosureResidualMps) / closureSigmaMps;
        const float rightBankAnomalySeverity = std::fabs(rightClosureResidualMps) / closureSigmaMps;

        snapshot.longitudinalClosureSeverity =
            Clamp01(
                (std::max)(
                    longitudinalClosureSeverity,
                    std::fabs(derivatives.longitudinalAccelMps2) /
                        PositiveOr(_preparedParams.raw.combinedAccelSustainedMps2, 1.0f)));
        snapshot.differentialClosureSeverity =
            Clamp01(
                (std::max)(
                    differentialClosureSeverity,
                    std::fabs(derivatives.yawAccelRadps2) /
                        PositiveOr(_preparedParams.raw.combinedAccelNominalMps2, 1.0f)));
        snapshot.lateralAccelerationSeverity =
            Clamp01(
                (std::max)(
                    lateralAccelerationSeverity,
                    std::fabs(derivatives.lateralAccelMps2) /
                        PositiveOr(_preparedParams.raw.combinedAccelPeakMps2, 1.0f)));
        snapshot.yawConsistencySeverity =
            Clamp01(
                (std::max)(
                    yawConsistencySeverity,
                    std::fabs(currentState(VehicleState::kR)) /
                        PositiveOr(_preparedParams.stopExitYawRateRadps, 1.0f)));

        snapshot.leftBankAnomalySeverity =
            Clamp01(
                (std::max)(
                    (std::max)(
                        leftBankAnomalySeverity,
                        PrecursorSeverity(snapshot.leftBankPreProjectionUtilization)),
                    appliedTorque.leftCurrentLimited ? 1.0f : 0.0f));
        snapshot.rightBankAnomalySeverity =
            Clamp01(
                (std::max)(
                    (std::max)(
                        rightBankAnomalySeverity,
                        PrecursorSeverity(snapshot.rightBankPreProjectionUtilization)),
                    appliedTorque.rightCurrentLimited ? 1.0f : 0.0f));

        if (!appliedTorque.batteryVoltageAvailable)
        {
            snapshot.leftBankAnomalySeverity =
                Clamp01((std::max)(snapshot.leftBankAnomalySeverity, 0.1f));
            snapshot.rightBankAnomalySeverity =
                Clamp01((std::max)(snapshot.rightBankAnomalySeverity, 0.1f));
        }

        return snapshot;
    }

    SrUkfCore::TransientContactMemoryState SrUkfCore::AdvanceTransientContactMemory(
        const TransientContactMemoryState& previousState,
        const GripUtilizationSnapshot& utilization,
        float dtS) noexcept
    {
        TransientContactMemoryState next{};
        const float leftInput =
            (std::max)(
                Clamp01(utilization.leftBankAnomalySeverity),
                Clamp01(utilization.leftBankPreProjectionUtilization));
        const float rightInput =
            (std::max)(
                Clamp01(utilization.rightBankAnomalySeverity),
                Clamp01(utilization.rightBankPreProjectionUtilization));

        constexpr float kRiseTauS = 1.0f / 8.0f;
        constexpr float kDecayTauS = 1.0f / 1.25f;

        next.leftBankMemory =
            (leftInput >= previousState.leftBankMemory) ?
            BlendTowards(previousState.leftBankMemory, leftInput, dtS, kRiseTauS) :
            BlendTowards(previousState.leftBankMemory, leftInput, dtS, kDecayTauS);
        next.rightBankMemory =
            (rightInput >= previousState.rightBankMemory) ?
            BlendTowards(previousState.rightBankMemory, rightInput, dtS, kRiseTauS) :
            BlendTowards(previousState.rightBankMemory, rightInput, dtS, kDecayTauS);
        return next;
    }

    SrUkfCore::RegripRecoveryState SrUkfCore::AdvanceRegripRecovery(
        const RegripRecoveryState& prior,
        const GripUtilizationSnapshot& utilization,
        const TransientContactMemoryState& memory,
        float dtS) noexcept
    {
        RegripRecoveryState next = prior;
        const float leftSeverity = ResolveSeverity(utilization.leftBankAnomalySeverity, memory.leftBankMemory);
        const float rightSeverity = ResolveSeverity(utilization.rightBankAnomalySeverity, memory.rightBankMemory);
        constexpr float kHoldoffThreshold = 0.700f;
        constexpr float kReleaseThreshold = 0.350f;
        constexpr float kHoldoffDwellS = 0.080f;
        constexpr float kScoreRiseTauS = 0.050f;
        constexpr float kScoreDecayTauS = 0.400f;

        const bool leftHoldoffTrigger = leftSeverity >= Clamp01(kHoldoffThreshold);
        const bool rightHoldoffTrigger = rightSeverity >= Clamp01(kHoldoffThreshold);

        next.leftBankRecoveryScore =
            leftHoldoffTrigger ?
            BlendTowards(prior.leftBankRecoveryScore, leftSeverity, dtS, kScoreRiseTauS) :
            BlendTowards(prior.leftBankRecoveryScore, 0.0f, dtS, kScoreDecayTauS);
        next.rightBankRecoveryScore =
            rightHoldoffTrigger ?
            BlendTowards(prior.rightBankRecoveryScore, rightSeverity, dtS, kScoreRiseTauS) :
            BlendTowards(prior.rightBankRecoveryScore, 0.0f, dtS, kScoreDecayTauS);

        if (leftHoldoffTrigger)
        {
            next.leftBankInRecovery = true;
            next.leftBankRecoveryTimeRemainingS = (std::max)(0.0f, kHoldoffDwellS);
        }
        else
        {
            next.leftBankRecoveryTimeRemainingS =
                (std::isfinite(dtS) && (dtS > 0.0f)) ?
                (std::max)(0.0f, prior.leftBankRecoveryTimeRemainingS - dtS) :
                Clamp01(prior.leftBankRecoveryTimeRemainingS);
            if ((next.leftBankRecoveryTimeRemainingS <= 0.0f) &&
                (next.leftBankRecoveryScore <= Clamp01(kReleaseThreshold)))
            {
                next.leftBankInRecovery = false;
            }
        }

        if (rightHoldoffTrigger)
        {
            next.rightBankInRecovery = true;
            next.rightBankRecoveryTimeRemainingS = (std::max)(0.0f, kHoldoffDwellS);
        }
        else
        {
            next.rightBankRecoveryTimeRemainingS =
                (std::isfinite(dtS) && (dtS > 0.0f)) ?
                (std::max)(0.0f, prior.rightBankRecoveryTimeRemainingS - dtS) :
                Clamp01(prior.rightBankRecoveryTimeRemainingS);
            if ((next.rightBankRecoveryTimeRemainingS <= 0.0f) &&
                (next.rightBankRecoveryScore <= Clamp01(kReleaseThreshold)))
            {
                next.rightBankInRecovery = false;
            }
        }

        next.leftBankRecoveryScore = Clamp01(next.leftBankRecoveryScore);
        next.rightBankRecoveryScore = Clamp01(next.rightBankRecoveryScore);
        return next;
    }

    bool SrUkfCore::IsHoldoffActive(const RegripRecoveryState& state) noexcept
    {
        return IsHoldoffActiveLeft(state) || IsHoldoffActiveRight(state);
    }

    bool SrUkfCore::IsHoldoffActiveLeft(const RegripRecoveryState& state) noexcept
    {
        return state.leftBankInRecovery && (state.leftBankRecoveryTimeRemainingS > 0.0f);
    }

    bool SrUkfCore::IsHoldoffActiveRight(const RegripRecoveryState& state) noexcept
    {
        return state.rightBankInRecovery && (state.rightBankRecoveryTimeRemainingS > 0.0f);
    }

    SrUkfCore::RobustUpdateSchedule SrUkfCore::buildFrozenSchedule(
        const GripUtilizationSnapshot& utilization,
        const TransientContactMemoryState& memory,
        const RegripRecoveryState& regrip,
        bool exactStationaryLock,
        bool lowSpeedLaunchWindowActive,
        bool inconsistencyWindowActive) const noexcept
    {
        RobustUpdateSchedule schedule{};
        schedule.exactStationaryLock = exactStationaryLock;
        schedule.planarAccelForwardUpdateEnabled =
            !exactStationaryLock;
        schedule.planarAccelLateralUpdateEnabled =
            PlanarAccelLateralUpdate::kFeatureEnabled && !exactStationaryLock;
        schedule.softOdometryEnabled =
            SoftOdometryAid::kFeatureEnabled && !exactStationaryLock;

        const float leftSeverity =
            (std::max)(Clamp01(utilization.leftBankAnomalySeverity), Clamp01(memory.leftBankMemory));
        const float rightSeverity =
            (std::max)(Clamp01(utilization.rightBankAnomalySeverity), Clamp01(memory.rightBankMemory));
        const float meanMemory = 0.5f * (Clamp01(memory.leftBankMemory) + Clamp01(memory.rightBankMemory));
        const float meanRecovery =
            0.5f * (Clamp01(regrip.leftBankRecoveryScore) + Clamp01(regrip.rightBankRecoveryScore));
        const float motionSeverity =
            (std::max)(
                Clamp01(utilization.longitudinalClosureSeverity),
                (std::max)(
                    Clamp01(utilization.differentialClosureSeverity),
                    (std::max)(
                        Clamp01(utilization.lateralAccelerationSeverity),
                        Clamp01(utilization.yawConsistencySeverity))));

        schedule.closureCovarianceScaleLeft =
            ClampScale(1.0f + (0.75f * leftSeverity) + (0.50f * meanMemory) + (0.50f * meanRecovery));
        schedule.closureCovarianceScaleRight =
            ClampScale(1.0f + (0.75f * rightSeverity) + (0.50f * meanMemory) + (0.50f * meanRecovery));
        schedule.lateralPseudoMeasurementCovarianceScale =
            ClampScale(1.0f + (0.60f * motionSeverity) + (0.25f * meanMemory) + (0.25f * meanRecovery));

        schedule.forwardSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.80f * Clamp01(utilization.longitudinalClosureSeverity)) + (0.30f * meanMemory));
        schedule.lateralSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.80f * Clamp01(utilization.lateralAccelerationSeverity)) + (0.30f * meanMemory));
        schedule.yawRateProcessNoiseScale =
            ClampScale(
                1.0f +
                (0.80f * Clamp01(utilization.differentialClosureSeverity)) +
                (0.30f * Clamp01(utilization.yawConsistencySeverity)));
        schedule.leftWheelSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.90f * leftSeverity) + (0.20f * meanRecovery));
        schedule.rightWheelSpeedProcessNoiseScale =
            ClampScale(1.0f + (0.90f * rightSeverity) + (0.20f * meanRecovery));

        schedule.leftBankHoldoffActive = IsHoldoffActiveLeft(regrip);
        schedule.rightBankHoldoffActive = IsHoldoffActiveRight(regrip);
        if (lowSpeedLaunchWindowActive)
        {
            schedule.lateralPseudoMeasurementCovarianceScale =
                (std::min)(schedule.lateralPseudoMeasurementCovarianceScale, 0.50f);
        }
        if (inconsistencyWindowActive || schedule.leftBankHoldoffActive || schedule.rightBankHoldoffActive)
        {
            schedule.lateralPseudoMeasurementCovarianceScale =
                ClampScale((2.0f * schedule.lateralPseudoMeasurementCovarianceScale) + 1.0f);
        }

        const bool severeTwoBankEdge =
            schedule.leftBankHoldoffActive &&
            schedule.rightBankHoldoffActive &&
            (meanMemory >= 0.60f) &&
            (meanRecovery >= 0.60f) &&
            ((std::max)(
                std::isfinite(utilization.leftBankPreProjectionUtilization) ? utilization.leftBankPreProjectionUtilization : 0.0f,
                std::isfinite(utilization.rightBankPreProjectionUtilization) ? utilization.rightBankPreProjectionUtilization : 0.0f) >= 1.0f);
        if (severeTwoBankEdge)
        {
            schedule.lateralPseudoMeasurementCovarianceScale = 64.0f;
        }

        return schedule;
    }

    float SrUkfCore::closurePseudoMeasurementScale(RelativeDirection side) const noexcept
    {
        if (!IsLeftLateralDirection(side) && !IsRightLateralDirection(side))
        {
            return 1.0f;
        }

        const bool leftSide = IsLeftLateralDirection(side);
        const float anomalySeverity =
            leftSide ?
            Clamp01(_frozenGripUtilization.leftBankAnomalySeverity) :
            Clamp01(_frozenGripUtilization.rightBankAnomalySeverity);
        const float strength =
            leftSide ?
            FeedforwardEdgeStrength(_transientContactMemory.leftBankMemory, _regripRecovery.leftBankRecoveryScore) :
            FeedforwardEdgeStrength(_transientContactMemory.rightBankMemory, _regripRecovery.rightBankRecoveryScore);
        const float recoveryPenalty =
            leftSide ?
            RecoveryPenalty(_regripRecovery.leftBankInRecovery, _regripRecovery.leftBankRecoveryScore) :
            RecoveryPenalty(_regripRecovery.rightBankInRecovery, _regripRecovery.rightBankRecoveryScore);
        const float holdoffPenalty =
            leftSide ?
            (_frozenSchedule.leftBankHoldoffActive ? 1.0f : 0.0f) :
            (_frozenSchedule.rightBankHoldoffActive ? 1.0f : 0.0f);
        return 1.0f + (0.75f * anomalySeverity) + (0.50f * strength) + recoveryPenalty + holdoffPenalty;
    }

    float SrUkfCore::lateralPseudoMeasurementScale() const noexcept
    {
        const float utilizationSeverity =
            (std::max)(
                Clamp01(_frozenGripUtilization.lateralAccelerationSeverity),
                Clamp01(_frozenGripUtilization.yawConsistencySeverity));
        const float leftStrength =
            FeedforwardEdgeStrength(_transientContactMemory.leftBankMemory, _regripRecovery.leftBankRecoveryScore);
        const float rightStrength =
            FeedforwardEdgeStrength(_transientContactMemory.rightBankMemory, _regripRecovery.rightBankRecoveryScore);
        const float memorySeverity = 0.5f * (leftStrength + rightStrength);
        const float recoverySeverity =
            0.5f *
            (Clamp01(_regripRecovery.leftBankRecoveryScore) + Clamp01(_regripRecovery.rightBankRecoveryScore));
        return 1.0f + (0.75f * utilizationSeverity) + (0.35f * memorySeverity) + (0.35f * recoverySeverity);
    }

    PlantModel::FeedforwardEnvelopeModifiers SrUkfCore::buildFeedforwardPolicyModifiers() const noexcept
    {
        PlantModel::FeedforwardEnvelopeModifiers modifiers{};
        const float leftStrength =
            FeedforwardEdgeStrength(_transientContactMemory.leftBankMemory, _regripRecovery.leftBankRecoveryScore);
        const float rightStrength =
            FeedforwardEdgeStrength(_transientContactMemory.rightBankMemory, _regripRecovery.rightBankRecoveryScore);
        const float leftUtilization =
            std::isfinite(_frozenGripUtilization.leftBankPreProjectionUtilization) ?
            _frozenGripUtilization.leftBankPreProjectionUtilization :
            0.0f;
        const float rightUtilization =
            std::isfinite(_frozenGripUtilization.rightBankPreProjectionUtilization) ?
            _frozenGripUtilization.rightBankPreProjectionUtilization :
            0.0f;

        const float leftSaturationRegion = SmoothStep01((leftUtilization - 0.65f) / 0.35f);
        float leftUtilizationScale =
            1.0f -
            (0.20f * leftStrength * leftSaturationRegion) -
            (0.10f * Clamp01(_transientContactMemory.leftBankMemory) * leftSaturationRegion);
        if (_frozenSchedule.leftBankHoldoffActive)
        {
            leftUtilizationScale -= 0.05f * leftStrength;
        }
        if (_regripRecovery.leftBankInRecovery)
        {
            leftUtilizationScale -= 0.10f * (0.5f + (0.5f * leftStrength));
        }

        const float rightSaturationRegion = SmoothStep01((rightUtilization - 0.65f) / 0.35f);
        float rightUtilizationScale =
            1.0f -
            (0.20f * rightStrength * rightSaturationRegion) -
            (0.10f * Clamp01(_transientContactMemory.rightBankMemory) * rightSaturationRegion);
        if (_frozenSchedule.rightBankHoldoffActive)
        {
            rightUtilizationScale -= 0.05f * rightStrength;
        }
        if (_regripRecovery.rightBankInRecovery)
        {
            rightUtilizationScale -= 0.10f * (0.5f + (0.5f * rightStrength));
        }

        float leftCapacityScale =
            1.0f -
            (0.10f * Clamp01(_transientContactMemory.leftBankMemory)) -
            (0.08f * leftStrength);
        if (_frozenSchedule.leftBankHoldoffActive)
        {
            leftCapacityScale -= 0.04f;
        }
        if (_regripRecovery.leftBankInRecovery)
        {
            leftCapacityScale -= 0.08f;
        }

        float rightCapacityScale =
            1.0f -
            (0.10f * Clamp01(_transientContactMemory.rightBankMemory)) -
            (0.08f * rightStrength);
        if (_frozenSchedule.rightBankHoldoffActive)
        {
            rightCapacityScale -= 0.04f;
        }
        if (_regripRecovery.rightBankInRecovery)
        {
            rightCapacityScale -= 0.08f;
        }

        modifiers.leftUtilizationScale = (std::clamp)(leftUtilizationScale, 0.60f, 1.0f);
        modifiers.rightUtilizationScale = (std::clamp)(rightUtilizationScale, 0.60f, 1.0f);
        modifiers.leftCapacityScale = (std::clamp)(leftCapacityScale, 0.65f, 1.0f);
        modifiers.rightCapacityScale = (std::clamp)(rightCapacityScale, 0.65f, 1.0f);
        return modifiers;
    }

    void SrUkfCore::refreshFrozenPolicyState(
        float dtSeconds,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV) noexcept
    {
        const StateVector& currentState = _filter.state();
        const StateVector& torqueEstimateState =
            _havePredictionReference ?
            _prePredictState :
            currentState;
        const AppliedTorqueEstimate appliedTorque =
            _plantModel.estimateAppliedTorque(
                torqueEstimateState,
                control,
                _preparedParams,
                batteryVoltageV);

        const PlantModel::WheelOnlyMeasurementPrediction wheelReference =
            _plantModel.predictWheelOnlyMeasurement(currentState, _preparedParams);
        const float effectiveWheelRadiusM =
            (std::isfinite(_preparedParams.wheelRadiusM) && (_preparedParams.wheelRadiusM > 0.0f)) ?
            _preparedParams.wheelRadiusM :
            0.0f;
        const float leftClosureResidualMps =
            effectiveWheelRadiusM * (_lastEncoderObs.omegaLeftRadps - wheelReference.leftWheelSpeedRadps);
        const float rightClosureResidualMps =
            effectiveWheelRadiusM * (_lastEncoderObs.omegaRightRadps - wheelReference.rightWheelSpeedRadps);
        _lastClosureResidualLeftMps = leftClosureResidualMps;
        _lastClosureResidualRightMps = rightClosureResidualMps;
        GripUtilizationSnapshot utilization =
            buildGripUtilizationSnapshot(
                currentState,
                appliedTorque,
                leftClosureResidualMps,
                rightClosureResidualMps,
                fanDutyCycle);
        TransientContactMemoryState memory =
            AdvanceTransientContactMemory(
                _transientContactMemory,
                utilization,
                dtSeconds);
        RegripRecoveryState regrip =
            AdvanceRegripRecovery(
                _regripRecovery,
                utilization,
                memory,
                dtSeconds);

        const float correctedYawRateForStationaryLockRadps =
            std::isfinite(_lastGyroMeasurementRadps) ?
            this->correctedYawRateRadps(_lastGyroMeasurementRadps) :
            (std::isfinite(currentState(VehicleState::kR)) ? currentState(VehicleState::kR) : 0.0f);
        const bool stationaryEvidenceReady =
            _stationaryCertified &&
            controlCommandsAreEffectivelyZero() &&
            HasExactZeroWheelObservation(_lastEncoderObs) &&
            (std::fabs(correctedYawRateForStationaryLockRadps) <= Tuning().stationaryCandidateMaxCorrectedGyroRadps) &&
            (std::fabs(currentState(VehicleState::kU)) <= Tuning().stationaryCandidateMaxLinearCommandMps) &&
            (std::fabs(currentState(VehicleState::kV)) <= Tuning().stationaryCandidateMaxLinearCommandMps) &&
            (std::fabs(_accelBodyXMps2) <= Tuning().stationaryCandidateMaxAccelMps2) &&
            (std::fabs(_accelBodyYMps2) <= Tuning().stationaryCandidateMaxAccelMps2) &&
            (_inconsistentHoldRemainingS <= 0.0f);
        if (stationaryEvidenceReady &&
            (utilization.leftBankPreProjectionUtilization <= 0.25f) &&
            (utilization.rightBankPreProjectionUtilization <= 0.25f))
        {
            memory = {};
            regrip = {};
        }
        const bool exactStationaryLock =
            stationaryEvidenceReady &&
            !IsHoldoffActive(regrip);
        const bool lowSpeedLaunchWindowActive =
            ((_launchHoldRemainingS > 0.0f) ||
             (_timeSinceStationaryExitS <= Tuning().stationaryExitLaunchWindowS)) &&
            (std::fabs(currentState(VehicleState::kU)) <= Tuning().launchLowSpeedThresholdMps);
        const RobustUpdateSchedule schedule =
            buildFrozenSchedule(
                utilization,
                memory,
                regrip,
                exactStationaryLock,
                lowSpeedLaunchWindowActive,
                _inconsistentHoldRemainingS > 0.0f);

        _transientContactMemory = memory;
        _regripRecovery = regrip;
        _frozenAppliedTorque = appliedTorque;
        _frozenGripUtilization = utilization;
        _frozenSchedule = schedule;
        _frozenDtS = (std::isfinite(dtSeconds) && (dtSeconds > 0.0f)) ? dtSeconds : 0.0f;
    }

    bool SrUkfCore::applyClosurePseudoMeasurements(
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        if (!_acceptedEncoderUpdateSincePredict)
        {
            return false;
        }

        const float effectiveWheelRadiusM = _preparedParams.wheelRadiusM;
        const float halfTrackWidthM = _preparedParams.halfTrackWidthM;
        if (!(std::isfinite(effectiveWheelRadiusM) && (effectiveWheelRadiusM > 0.0f)) ||
            !std::isfinite(halfTrackWidthM))
        {
            return false;
        }

        bool anyAccepted = false;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const float baseSigmaMps =
            (std::max)(Tuning().generalEncoderLinearSpeedSigmaMps, 1.0e-3f);
        const float leftScale =
            _frozenSchedule.closureCovarianceScaleLeft *
            closurePseudoMeasurementScale(RelativeDirection::Left90);
        const float rightScale =
            _frozenSchedule.closureCovarianceScaleRight *
            closurePseudoMeasurementScale(RelativeDirection::Right90);

        const auto applySideUpdate =
            [this, halfTrackWidthM, &invokeLoop, baseSigmaMps](
                float measuredLinearSpeedMps,
                float covarianceScale,
                float yawSign,
                float& innovationStorageMps,
                float& nisStorage) noexcept
        {
            Eigen::Matrix<float, 1, 1> measurement;
            measurement << measuredLinearSpeedMps;
            innovationStorageMps =
                measuredLinearSpeedMps -
                (_filter.state()(VehicleState::kU) +
                    (yawSign * halfTrackWidthM * _filter.state()(VehicleState::kR)));
            Eigen::Matrix<float, 1, 1> sqrtNoise;
            sqrtNoise(0, 0) =
                baseSigmaMps *
                MazeMap::Math::Sqrtf((std::max)(covarianceScale, 0.25f));
            const bool accepted = _filter.Update<1>(
                measurement,
                sqrtNoise,
                25.0f,
                [halfTrackWidthM, yawSign](const StateVector& sigmaPoint) noexcept
                {
                    Eigen::Matrix<float, 1, 1> prediction;
                    prediction <<
                        sigmaPoint(VehicleState::kU) +
                        (yawSign * halfTrackWidthM * sigmaPoint(VehicleState::kR));
                    return prediction;
                },
                invokeLoop);
            nisStorage = _filter.lastNis();
            return accepted;
        };

        anyAccepted =
            applySideUpdate(
                effectiveWheelRadiusM * _lastEncoderObs.omegaLeftRadps,
                leftScale,
                1.0f,
                _lastClosureLeftInnovationMps,
                _lastClosureLeftNis) ||
            anyAccepted;
        anyAccepted =
            applySideUpdate(
                effectiveWheelRadiusM * _lastEncoderObs.omegaRightRadps,
                rightScale,
                -1.0f,
                _lastClosureRightInnovationMps,
                _lastClosureRightNis) ||
            anyAccepted;
        return anyAccepted;
    }

    bool SrUkfCore::applyAdaptiveLateralPseudoMeasurement(
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        if (_frozenSchedule.exactStationaryLock)
        {
            updateNonholonomicDiagnostics(false);
            return false;
        }

        const float covarianceScale =
            _frozenSchedule.lateralPseudoMeasurementCovarianceScale *
            lateralPseudoMeasurementScale();
        if (!(std::isfinite(covarianceScale) && (covarianceScale > 0.0f)) || (covarianceScale >= 64.0f))
        {
            updateNonholonomicDiagnostics(false);
            return false;
        }

        Eigen::Matrix<float, 1, 1> measurement;
        measurement << 0.0f;
        _lastLateralPseudoInnovationMps = -_filter.state()(VehicleState::kV);
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) =
            ComputeNonholonomicSigmaMps(std::fabs(_filter.state()(VehicleState::kU))) *
            MazeMap::Math::Sqrtf((std::max)(covarianceScale, 0.25f));
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const bool accepted = _filter.Update<1>(
            measurement,
            sqrtNoise,
            25.0f,
            [](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction << sigmaPoint(VehicleState::kV);
                return prediction;
            },
            invokeLoop);
        _lastLateralPseudoNis = _filter.lastNis();
        updateNonholonomicDiagnostics(accepted);
        return accepted;
    }

    bool SrUkfCore::predict(
        float dt,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV) noexcept
    {
        return predictImpl(dt, control, fanDutyCycle, batteryVoltageV, nullptr, nullptr);
    }

    bool SrUkfCore::predictImpl(
        float dt,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        _lastControl = control;
        _lastFanDutyCycle = fanDutyCycle;
        _lastBatteryVoltageV = batteryVoltageV;
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return true;
        }

        _prePredictState = _filter.state();
        _prePredictCovariance = _filter.covariance();
        _havePredictionReference = true;
        _acceptedEncoderUpdateSincePredict = false;
        _lastEncoderDtSeconds = dt;
        resetPivotScrubTelemetry();
        _directWheelUpdateBodyStateInvariant = false;
        _releaseInflationApplied = false;
        _lastGyroMeasurementRadps = 0.0f;
        _lastGyroInnovationRadps = 0.0f;
        _lastGyroNis = 0.0f;
        _lastForwardAccelInnovationMps2 = 0.0f;
        _lastForwardAccelNis = 0.0f;
        _lastLateralAccelInnovationMps2 = 0.0f;
        _lastLateralAccelNis = 0.0f;
        _lastClosureLeftInnovationMps = 0.0f;
        _lastClosureLeftNis = 0.0f;
        _lastClosureRightInnovationMps = 0.0f;
        _lastClosureRightNis = 0.0f;
        _lastLateralPseudoInnovationMps = 0.0f;
        _lastLateralPseudoNis = 0.0f;
        updateCommandSignFlipWindow(dt);
        updateOperatingMode(dt);
        const bool wasExactStationaryLock = _frozenSchedule.exactStationaryLock;
        refreshFrozenPolicyState(dt, control, fanDutyCycle, batteryVoltageV);
        if (wasExactStationaryLock && !_frozenSchedule.exactStationaryLock)
        {
            const Eigen::Matrix<float, 2, 2> encoderCovariance =
                ComputeGeneralEncoderPairCovarianceRadps(_params);
            const float wheelVarianceFloorRadps2 =
                (std::max)(encoderCovariance(0, 0), encoderCovariance(1, 1));
            const float forwardVarianceFloorMps2 =
                Tuning().generalEncoderLinearSpeedSigmaMps * Tuning().generalEncoderLinearSpeedSigmaMps;
            const float lateralSigmaFloorMps =
                ComputeNonholonomicSigmaMps(std::fabs(_filter.state()(VehicleState::kU)));
            const float lateralVarianceFloorMps2 = lateralSigmaFloorMps * lateralSigmaFloorMps;
            const float yawSigmaFloorRadps =
                (std::max)(Tuning().generalEncoderYawRateSigmaRadps, Tuning().recoveryYawRateStdFloorRadps);
            const float yawVarianceFloorRadps2 = yawSigmaFloorRadps * yawSigmaFloorRadps;

            (void)_filter.floorVariance(VehicleState::kU, forwardVarianceFloorMps2);
            (void)_filter.floorVariance(VehicleState::kV, lateralVarianceFloorMps2);
            (void)_filter.floorVariance(VehicleState::kR, yawVarianceFloorRadps2);
            (void)_filter.floorVariance(VehicleState::kOmegaL, wheelVarianceFloorRadps2);
            (void)_filter.floorVariance(VehicleState::kOmegaR, wheelVarianceFloorRadps2);
            _releaseInflationApplied = true;
        }
        updateProcessNoiseForMode();
        _sqrtProcessNoiseDensity(VehicleState::kU, VehicleState::kU) *=
            MazeMap::Math::Sqrtf((std::max)(_frozenSchedule.forwardSpeedProcessNoiseScale, 1.0f));
        _sqrtProcessNoiseDensity(VehicleState::kV, VehicleState::kV) *=
            MazeMap::Math::Sqrtf((std::max)(_frozenSchedule.lateralSpeedProcessNoiseScale, 1.0f));
        _sqrtProcessNoiseDensity(VehicleState::kR, VehicleState::kR) *=
            MazeMap::Math::Sqrtf((std::max)(_frozenSchedule.yawRateProcessNoiseScale, 1.0f));
        _sqrtProcessNoiseDensity(VehicleState::kOmegaL, VehicleState::kOmegaL) *=
            MazeMap::Math::Sqrtf((std::max)(_frozenSchedule.leftWheelSpeedProcessNoiseScale, 1.0f));
        _sqrtProcessNoiseDensity(VehicleState::kOmegaR, VehicleState::kOmegaR) *=
            MazeMap::Math::Sqrtf((std::max)(_frozenSchedule.rightWheelSpeedProcessNoiseScale, 1.0f));
        StateMatrix predictProcessNoiseSquareRoot = _sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dt);
        // `kBgz` follows the explicit per-sample author tuning, not the density-scaled mode table.
        predictProcessNoiseSquareRoot(VehicleState::kBgz, VehicleState::kBgz) =
            GyroBiasProcessSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(predictProcessNoiseSquareRoot);
        Eigen::Matrix<float, 3, 1> filterCommandVector;
        filterCommandVector << control.LeftMotorPwm(), control.RightMotorPwm(), fanDutyCycle;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const PlantModel::FeedforwardEnvelopeModifiers predictModifiers = buildFeedforwardPolicyModifiers();
        const bool predicted = _filter.Predict(
            dt,
            filterCommandVector,
            [this, predictModifiers](const StateVector& sigmaPoint, const Eigen::Matrix<float, 3, 1>&, float sigmaDt) noexcept
            {
                if (_frozenSchedule.exactStationaryLock)
                {
                    return IntegrateStationaryHoldState(sigmaPoint, sigmaDt);
                }

                return _plantModel.integrateAppliedBankTorques(
                    sigmaPoint,
                    _frozenAppliedTorque.leftAppliedBankTorqueNm,
                    _frozenAppliedTorque.rightAppliedBankTorqueNm,
                    _preparedParams,
                    _lastFanDutyCycle,
                    sigmaDt,
                    predictModifiers.leftUtilizationScale,
                    predictModifiers.rightUtilizationScale,
                    predictModifiers.leftCapacityScale,
                    predictModifiers.rightCapacityScale);
            },
            invokeLoop);
        if (predicted)
        {
            const ModeProcessNoiseConfig& modeNoise = GetModeProcessNoiseConfig(_operatingMode);
            (void)_filter.floorVariance(VehicleState::kR, modeNoise.stdRMin * modeNoise.stdRMin);
            (void)_filter.floorVariance(VehicleState::kV, modeNoise.stdVMin * modeNoise.stdVMin);
            (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
            _gyroBiasAnchorRadps = _filter.state()(VehicleState::kBgz);
            _gyroBiasAnchorVarianceRadps2 = _filter.variance(VehicleState::kBgz);
        }
        return predicted;
    }

    MeasurementUpdateResult SrUkfCore::updateEncoderPair(
        const EncoderObs& observation,
        float dt,
        bool updateYaw) noexcept
    {
        return updateEncoderPairImpl(observation, dt, updateYaw, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updateEncoderPairImpl(
        const EncoderObs& observation,
        float dt,
        bool updateYaw,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        (void)updateYaw;
        MeasurementUpdateResult result{};
        result.attempted = true;

        const EncoderObs measured = observation;
        const StateVector priorState = _filter.state();
        const StateMatrix priorSqrtCovariance = _filter.sqrtCovariance();
        const float measuredWheelVarianceRadps2 =
            ComputeMeasuredWheelVarianceRadps2(measured, _params);
        _acceptedEncoderUpdateSincePredict = false;
        _lastEncoderDtSeconds =
            (std::isfinite(dt) && (dt > 0.0f)) ?
            dt :
            0.0f;
        _pivotScrubMode = IsPivotScrubCandidate(
            measured,
            _commandedLinearMps,
            _commandedAngularRadps,
            _yawConsistencyLowPassRadps,
            _yawWindowMismatchRad,
            _params);
        _pivotScrubEncoderBodyUpdateSkipped = _pivotScrubMode;
        _lastEncoderObs = measured;
        const bool wasExactStationaryLock = _frozenSchedule.exactStationaryLock;
        refreshFrozenPolicyState(_lastEncoderDtSeconds, _lastControl, _lastFanDutyCycle, _lastBatteryVoltageV);
        if (wasExactStationaryLock && !_frozenSchedule.exactStationaryLock)
        {
            const Eigen::Matrix<float, 2, 2> encoderCovariance =
                ComputeGeneralEncoderPairCovarianceRadps(_params);
            const float wheelVarianceFloorRadps2 =
                (std::max)(encoderCovariance(0, 0), encoderCovariance(1, 1));
            const float forwardVarianceFloorMps2 =
                Tuning().generalEncoderLinearSpeedSigmaMps * Tuning().generalEncoderLinearSpeedSigmaMps;
            const float lateralSigmaFloorMps =
                ComputeNonholonomicSigmaMps(std::fabs(_filter.state()(VehicleState::kU)));
            const float lateralVarianceFloorMps2 = lateralSigmaFloorMps * lateralSigmaFloorMps;
            const float yawSigmaFloorRadps =
                (std::max)(Tuning().generalEncoderYawRateSigmaRadps, Tuning().recoveryYawRateStdFloorRadps);
            const float yawVarianceFloorRadps2 = yawSigmaFloorRadps * yawSigmaFloorRadps;

            (void)_filter.floorVariance(VehicleState::kU, forwardVarianceFloorMps2);
            (void)_filter.floorVariance(VehicleState::kV, lateralVarianceFloorMps2);
            (void)_filter.floorVariance(VehicleState::kR, yawVarianceFloorRadps2);
            (void)_filter.floorVariance(VehicleState::kOmegaL, wheelVarianceFloorRadps2);
            (void)_filter.floorVariance(VehicleState::kOmegaR, wheelVarianceFloorRadps2);
            _releaseInflationApplied = true;
        }

        Eigen::Matrix<float, 2, 1> z;
        z << measured.omegaLeftRadps, measured.omegaRightRadps;
        const Eigen::Matrix<float, 2, 2> sqrtEncoderNoise = ComputeEncoderPairSqrtNoise(measured, _params);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _filter.Update<2>(
            z,
            sqrtEncoderNoise,
            ComputeEncoderPairNisThreshold(measured),
            [](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 2, 1> prediction;
                prediction << sigmaPoint(VehicleState::kOmegaL), sigmaPoint(VehicleState::kOmegaR);
                return prediction;
            },
            invokeLoop);
        const float encoderMeasurementNis = _filter.lastNis();
        if (result.accepted)
        {
            _acceptedEncoderUpdateSincePredict = true;
            if (!_frozenSchedule.exactStationaryLock)
            {
                constexpr std::array<int, 2> kWheelOnlyIndices = {
                    VehicleState::kOmegaL,
                    VehicleState::kOmegaR
                };
                const StateVector updatedState = _filter.state();
                const StateMatrix updatedSqrtCovariance = _filter.sqrtCovariance();
                StateVector projectedState = priorState;
                StateMatrix projectedSqrtCovariance = priorSqrtCovariance;
                ProjectMaskedStateAndSquareRootCovariance(
                    priorState,
                    priorSqrtCovariance,
                    updatedState,
                    updatedSqrtCovariance,
                    kWheelOnlyIndices.data(),
                    kWheelOnlyIndices.size(),
                    projectedState,
                    projectedSqrtCovariance);
                _filter.setStateSquareRootCovariance(projectedState, projectedSqrtCovariance);
                applyWheelSpeedConstraint(measured, measuredWheelVarianceRadps2);
            }
            const StateVector& postEncoderState = _filter.state();
            _pivotScrubEncoderWheelDeltaPsiRad =
                postEncoderState(VehicleState::kPsi) - priorState(VehicleState::kPsi);
            _pivotScrubEncoderWheelDeltaRRadps =
                postEncoderState(VehicleState::kR) - priorState(VehicleState::kR);
            _pivotScrubEncoderWheelDeltaOmegaLRadps =
                postEncoderState(VehicleState::kOmegaL) - priorState(VehicleState::kOmegaL);
            _pivotScrubEncoderWheelDeltaOmegaRRadps =
                postEncoderState(VehicleState::kOmegaR) - priorState(VehicleState::kOmegaR);
            const float encoderMaskedDeltaNorm =
                MazeMap::Math::Sqrtf(
                    (_pivotScrubEncoderWheelDeltaOmegaLRadps * _pivotScrubEncoderWheelDeltaOmegaLRadps) +
                    (_pivotScrubEncoderWheelDeltaOmegaRRadps * _pivotScrubEncoderWheelDeltaOmegaRRadps));
            _pivotScrubEncoderWheelMaskedDeltaNorm =
                std::isfinite(encoderMaskedDeltaNorm) ? encoderMaskedDeltaNorm : 0.0f;
        }
        else
        {
            _acceptedEncoderUpdateSincePredict = true;
            applyWheelSpeedConstraint(measured, measuredWheelVarianceRadps2);
            result.accepted = true;
        }
        if (_frozenSchedule.exactStationaryLock)
        {
            applyStationaryZeroMotionConstraint(_lastGyroMeasurementRadps);
        }
        const StateVector& constrainedState = _filter.state();
        _directWheelUpdateBodyStateInvariant =
            _frozenSchedule.exactStationaryLock ||
            ((std::fabs(constrainedState(VehicleState::kPx) - priorState(VehicleState::kPx)) <= 1.0e-6f) &&
             (std::fabs(constrainedState(VehicleState::kPy) - priorState(VehicleState::kPy)) <= 1.0e-6f) &&
             (std::fabs(constrainedState(VehicleState::kPsi) - priorState(VehicleState::kPsi)) <= 1.0e-6f) &&
             (std::fabs(constrainedState(VehicleState::kU) - priorState(VehicleState::kU)) <= 1.0e-6f) &&
             (std::fabs(constrainedState(VehicleState::kV) - priorState(VehicleState::kV)) <= 1.0e-6f) &&
             (std::fabs(constrainedState(VehicleState::kR) - priorState(VehicleState::kR)) <= 1.0e-6f) &&
             (std::fabs(constrainedState(VehicleState::kBgz) - priorState(VehicleState::kBgz)) <= 1.0e-6f));
        _pivotScrubZeroUSoftApplied = false;
        _pivotScrubZeroUInnovationMps = 0.0f;
        _pivotScrubZeroUDeltaMps = 0.0f;
        updateStationaryCertification(_lastGyroMeasurementRadps);
        const OperatingMode previousMode = _operatingMode;
        updateOperatingMode(_lastEncoderDtSeconds);
        if ((previousMode == OperatingMode::LaunchOrReversalTransient) &&
            (_operatingMode != OperatingMode::LaunchOrReversalTransient) &&
            (_saturationFlags == 0U))
        {
            (void)_filter.floorVariance(
                VehicleState::kR,
                Tuning().recoveryYawRateStdFloorRadps * Tuning().recoveryYawRateStdFloorRadps);
            _nhcReenableDelayRemainingS = Tuning().recoveryNhcReenableDelayS;
        }
        const ModeProcessNoiseConfig& encoderModeNoise = GetModeProcessNoiseConfig(_operatingMode);
        (void)_filter.floorVariance(VehicleState::kR, encoderModeNoise.stdRMin * encoderModeNoise.stdRMin);
        (void)_filter.floorVariance(VehicleState::kV, encoderModeNoise.stdVMin * encoderModeNoise.stdVMin);
        (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        result.nis = encoderMeasurementNis;
        return result;
    }

    MeasurementUpdateResult SrUkfCore::updateYawRate(float yawRateRadps) noexcept
    {
        return updateYawRateImpl(yawRateRadps, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updateYawRateImpl(
        float yawRateRadps,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = std::isfinite(yawRateRadps);
        if (!result.attempted)
        {
            return result;
        }

        const StateVector priorState = _filter.state();
        const StateMatrix priorSqrtCovariance = _filter.sqrtCovariance();
        _lastGyroMeasurementRadps = yawRateRadps;
        _lastGyroInnovationRadps =
            yawRateRadps - (priorState(VehicleState::kR) + priorState(VehicleState::kBgz));
        Eigen::Matrix<float, 1, 1> z;
        z << yawRateRadps;
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) = _sqrtImuNoise(0, 0);
        const bool shouldApplyStationaryConstraint =
            _acceptedEncoderUpdateSincePredict &&
            HasExactZeroWheelObservation(_lastEncoderObs);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _filter.Update<1>(
            z,
            sqrtNoise,
            std::numeric_limits<float>::infinity(),
            [](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction << sigmaPoint(VehicleState::kR) + sigmaPoint(VehicleState::kBgz);
                return prediction;
            },
            invokeLoop);
        const float yawMeasurementNis = _filter.lastNis();
        _lastGyroNis = yawMeasurementNis;
        if (result.accepted)
        {
            const StateVector updatedState = _filter.state();
            const StateMatrix updatedSqrtCovariance = _filter.sqrtCovariance();
            StateVector projectedState = priorState;
            StateMatrix projectedSqrtCovariance = priorSqrtCovariance;
            constexpr std::array<int, 7> kAllowedIndices = {
                VehicleState::kPx,
                VehicleState::kPy,
                VehicleState::kPsi,
                VehicleState::kU,
                VehicleState::kV,
                VehicleState::kR,
                VehicleState::kBgz
            };

            ProjectMaskedStateAndSquareRootCovariance(
                priorState,
                priorSqrtCovariance,
                updatedState,
                updatedSqrtCovariance,
                kAllowedIndices.data(),
                kAllowedIndices.size(),
                projectedState,
                projectedSqrtCovariance);
            _filter.setStateSquareRootCovariance(projectedState, projectedSqrtCovariance);

            updateInitialStationaryGyroBias(yawRateRadps, shouldApplyStationaryConstraint);
            updateStationaryCertification(yawRateRadps);
            updateYawConsistencyMetrics(yawRateRadps, priorState(VehicleState::kR));
            if (_frozenSchedule.exactStationaryLock)
            {
                applyStationaryZeroMotionConstraint(yawRateRadps);
            }
            const OperatingMode previousMode = _operatingMode;
            updateOperatingMode(_lastEncoderDtSeconds);
            if ((previousMode == OperatingMode::LaunchOrReversalTransient) &&
                (_operatingMode != OperatingMode::LaunchOrReversalTransient) &&
                (_saturationFlags == 0U))
            {
                (void)_filter.floorVariance(
                    VehicleState::kR,
                    Tuning().recoveryYawRateStdFloorRadps * Tuning().recoveryYawRateStdFloorRadps);
                _nhcReenableDelayRemainingS = Tuning().recoveryNhcReenableDelayS;
            }
            const ModeProcessNoiseConfig& yawModeNoise = GetModeProcessNoiseConfig(_operatingMode);
            (void)_filter.floorVariance(VehicleState::kR, yawModeNoise.stdRMin * yawModeNoise.stdRMin);
            (void)_filter.floorVariance(VehicleState::kV, yawModeNoise.stdVMin * yawModeNoise.stdVMin);
            (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        }
        result.nis = yawMeasurementNis;
        return result;
    }

    MeasurementUpdateResult SrUkfCore::updatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        return updatePlanarAccelImpl(observation, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updatePlanarAccelImpl(
        const ImuAccelObs& observation,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = observation.valid || _acceptedEncoderUpdateSincePredict;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };

        bool accelAccepted = false;
        if (observation.valid && _frozenSchedule.planarAccelForwardUpdateEnabled)
        {
            Eigen::Matrix<float, 1, 1> measurement;
            measurement << observation.accelBodyYMps2;
            _lastForwardAccelInnovationMps2 =
                observation.accelBodyYMps2 -
                _plantModel.imuPlanarAcceleration(
                    _filter.state(),
                    _lastControl,
                    _lastFanDutyCycle,
                    _lastBatteryVoltageV,
                    _preparedParams)(1);
            Eigen::Matrix<float, 1, 1> sqrtNoise;
            sqrtNoise(0, 0) = _sqrtImuNoise(2, 2);
            const bool forwardAccelAccepted = _filter.Update<1>(
                measurement,
                sqrtNoise,
                25.0f,
                [this](const StateVector& sigmaPoint) noexcept
                {
                    Eigen::Matrix<float, 1, 1> prediction;
                    prediction << _plantModel.imuPlanarAcceleration(
                        sigmaPoint,
                        _lastControl,
                        _lastFanDutyCycle,
                        _lastBatteryVoltageV,
                        _preparedParams)(1);
                    return prediction;
                },
                invokeLoop);
            _lastForwardAccelNis = _filter.lastNis();
            accelAccepted = forwardAccelAccepted || accelAccepted;
        }

        if (observation.valid &&
            _frozenSchedule.planarAccelLateralUpdateEnabled &&
            PlanarAccelLateralUpdate::kFeatureEnabled)
        {
            Eigen::Matrix<float, 1, 1> measurement;
            measurement << observation.accelBodyXMps2;
            _lastLateralAccelInnovationMps2 =
                observation.accelBodyXMps2 -
                _plantModel.imuPlanarAcceleration(
                    _filter.state(),
                    _lastControl,
                    _lastFanDutyCycle,
                    _lastBatteryVoltageV,
                    _preparedParams)(0);
            Eigen::Matrix<float, 1, 1> sqrtNoise;
            sqrtNoise(0, 0) = _sqrtImuNoise(1, 1);
            const bool lateralAccelAccepted = _filter.Update<1>(
                measurement,
                sqrtNoise,
                25.0f,
                [this](const StateVector& sigmaPoint) noexcept
                {
                    Eigen::Matrix<float, 1, 1> prediction;
                    prediction << _plantModel.imuPlanarAcceleration(
                        sigmaPoint,
                        _lastControl,
                        _lastFanDutyCycle,
                        _lastBatteryVoltageV,
                        _preparedParams)(0);
                    return prediction;
                },
                invokeLoop);
            _lastLateralAccelNis = _filter.lastNis();
            accelAccepted = lateralAccelAccepted || accelAccepted;
        }

        const bool closureAccepted = applyClosurePseudoMeasurements(loopHookContext, loopHook);
        const bool lateralAccepted = applyAdaptiveLateralPseudoMeasurement(loopHookContext, loopHook);
        bool stationaryApplied = false;
        if (_frozenSchedule.exactStationaryLock)
        {
            applyStationaryZeroMotionConstraint(0.0f);
            stationaryApplied = true;
        }

        const OperatingMode previousMode = _operatingMode;
        updateOperatingMode(_lastEncoderDtSeconds);
        if ((previousMode == OperatingMode::LaunchOrReversalTransient) &&
            (_operatingMode != OperatingMode::LaunchOrReversalTransient) &&
            (_saturationFlags == 0U))
        {
            (void)_filter.floorVariance(
                VehicleState::kR,
                Tuning().recoveryYawRateStdFloorRadps * Tuning().recoveryYawRateStdFloorRadps);
            _nhcReenableDelayRemainingS = Tuning().recoveryNhcReenableDelayS;
        }
        const ModeProcessNoiseConfig& accelModeNoise = GetModeProcessNoiseConfig(_operatingMode);
        (void)_filter.floorVariance(VehicleState::kR, accelModeNoise.stdRMin * accelModeNoise.stdRMin);
        (void)_filter.floorVariance(VehicleState::kV, accelModeNoise.stdVMin * accelModeNoise.stdVMin);
        (void)_filter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        _gyroBiasAnchorRadps = _filter.state()(VehicleState::kBgz);
        _gyroBiasAnchorVarianceRadps2 = _filter.variance(VehicleState::kBgz);
        _yawValidForFeedforward = IsYawValidForFeedforward(
            _operatingMode,
            _filter.state()(VehicleState::kBgz),
            _gyroBiasAnchorRadps,
            _yawConsistencyLowPassRadps,
            _nonholonomicConstraintEnabled,
            _filter.state()(VehicleState::kV),
            _nhcSigmaMps);
        result.accepted = accelAccepted || closureAccepted || lateralAccepted || stationaryApplied;
        return result;
    }

    bool SrUkfCore::HasExactZeroWheelObservation(const EncoderObs& observation) noexcept
    {
        return (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
    }

    float SrUkfCore::ComputeMeasuredLinearSpeedMps(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            return 0.0f;
        }

        return 0.5f * params.wheelRadiusM * (observation.omegaLeftRadps + observation.omegaRightRadps);
    }

    float SrUkfCore::ComputeMeasuredLinearSpeedVarianceMps2(const EncoderObs& observation) noexcept
    {
        const float sigmaMps =
            HasExactZeroWheelObservation(observation) ?
            Tuning().stationaryEncoderVelocitySigmaMps :
            Tuning().generalEncoderLinearSpeedSigmaMps;
        return sigmaMps * sigmaMps;
    }

    float SrUkfCore::ComputeMeasuredYawRateRadps(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) ||
            !std::isfinite(params.wheelRadiusM) ||
            !(params.trackWidthM > 0.0f) ||
            !std::isfinite(params.trackWidthM))
        {
            return 0.0f;
        }

        return params.wheelRadiusM * (observation.omegaLeftRadps - observation.omegaRightRadps) / params.trackWidthM;
    }

    float SrUkfCore::ComputeMeasuredYawRateVarianceRadps2(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) ||
            !std::isfinite(params.wheelRadiusM) ||
            !(params.trackWidthM > 0.0f) ||
            !std::isfinite(params.trackWidthM))
        {
            return 1.0f;
        }

        const Eigen::Matrix<float, 2, 2> wheelCovarianceRadps2 =
            HasExactZeroWheelObservation(observation) ?
            (Eigen::Matrix<float, 2, 2>::Identity() *
                (ComputeStationaryEncoderOmegaSigmaRadps(params) *
                 ComputeStationaryEncoderOmegaSigmaRadps(params))) :
            ComputeGeneralEncoderPairCovarianceRadps(params);
        const float yawScale = params.wheelRadiusM / params.trackWidthM;
        const float variance =
            (yawScale * yawScale) *
            (wheelCovarianceRadps2(0, 0) +
             wheelCovarianceRadps2(1, 1) -
             (2.0f * wheelCovarianceRadps2(0, 1)));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    float SrUkfCore::ComputeMeasuredWheelVarianceRadps2(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if (HasExactZeroWheelObservation(observation))
        {
            const float stationarySigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(params);
            return stationarySigmaRadps * stationarySigmaRadps;
        }

        const Eigen::Matrix<float, 2, 2> covariance = ComputeGeneralEncoderPairCovarianceRadps(params);
        const float variance = (std::max)(covariance(0, 0), covariance(1, 1));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    bool SrUkfCore::controlCommandsAreEffectivelyZero() const noexcept
    {
        return
            (std::fabs(_lastControl.LeftMotorPwm()) <= 1.0e-6f) &&
            (std::fabs(_lastControl.RightMotorPwm()) <= 1.0e-6f) &&
            (std::fabs(_commandedLinearMps) <= Tuning().stationaryCandidateMaxLinearCommandMps) &&
            (std::fabs(_commandedAngularRadps) <= Tuning().stationaryCandidateMaxAngularCommandRadps);
    }

    bool SrUkfCore::applyGripLateralVelocityConstraint(
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        updateNonholonomicDiagnostics(shouldEnableNonholonomicConstraint());
        if (!_nonholonomicConstraintEnabled)
        {
            return false;
        }

        Eigen::Matrix<float, 1, 1> z;
        z << 0.0f;
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) = _nhcSigmaMps;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const bool accepted = _filter.Update<1>(
            z,
            sqrtNoise,
            std::numeric_limits<float>::infinity(),
            [](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction << sigmaPoint(VehicleState::kV);
                return prediction;
            },
            invokeLoop);
        updateNonholonomicDiagnostics(accepted && _nonholonomicConstraintEnabled);
        return accepted;
    }

    void SrUkfCore::updateNonholonomicDiagnostics(bool constraintEnabled) noexcept
    {
        _nonholonomicConstraintEnabled = constraintEnabled;
        _nhcSigmaMps = ComputeNonholonomicSigmaMps(std::fabs(_filter.state()(VehicleState::kU)));
        _nhcResidualMps = _filter.state()(VehicleState::kV);
        _nhcResidualSigma =
            (_nhcSigmaMps > 0.0f) ?
            (_nhcResidualMps / _nhcSigmaMps) :
            0.0f;
    }

    void SrUkfCore::applyWheelSpeedConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept
    {
        if (_frozenSchedule.exactStationaryLock)
        {
            return;
        }

        StateVector constrainedState = _filter.state();
        constrainedState(VehicleState::kOmegaL) = measured.omegaLeftRadps;
        constrainedState(VehicleState::kOmegaR) = measured.omegaRightRadps;
        VehicleState::NormalizeStateVector(constrainedState);

        StateMatrix constrainedCovariance = _filter.covariance();
        constrainedCovariance.row(VehicleState::kOmegaL).setZero();
        constrainedCovariance.col(VehicleState::kOmegaL).setZero();
        constrainedCovariance.row(VehicleState::kOmegaR).setZero();
        constrainedCovariance.col(VehicleState::kOmegaR).setZero();
        const float constrainedVariance = (std::max)(wheelVarianceRadps2, 1.0e-12f);
        constrainedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = constrainedVariance;
        constrainedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = constrainedVariance;
        _filter.setState(constrainedState, constrainedCovariance);
    }

    void SrUkfCore::applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept
    {
        (void)yawRateRadps;
        const bool hasPoseReference =
            _stationaryCandidatePoseReferenceValid ||
            _havePredictionReference;
        const StateVector& poseReferenceState =
            _stationaryCandidatePoseReferenceValid ?
            _stationaryCandidatePoseReferenceState :
            _prePredictState;
        const StateMatrix& poseReferenceCovariance = _prePredictCovariance;
        VehicleState constrainedState;
        constrainedState.SetPosition(Eigen::Vector2f(_filter.state()(VehicleState::kPx), _filter.state()(VehicleState::kPy)));
        constrainedState.SetOrientation(_filter.state()(VehicleState::kPsi));
        constrainedState.SetVelocity(_filter.state()(VehicleState::kU));
        constrainedState.SetLateralVelocity(_filter.state()(VehicleState::kV));
        constrainedState.SetRotationalVelocity(_filter.state()(VehicleState::kR));
        constrainedState.SetWheelSpeedLeft(_filter.state()(VehicleState::kOmegaL));
        constrainedState.SetWheelSpeedRight(_filter.state()(VehicleState::kOmegaR));
        constrainedState.SetGyroBiasZ(_filter.state()(VehicleState::kBgz));
        constrainedState.SetCovariance(_filter.covariance());
        constrainedState.ApplyStationaryZeroMotionConstraint(
            true,
            hasPoseReference,
            poseReferenceState,
            poseReferenceCovariance);
        StateVector constrainedStateVector = StateVector::Zero();
        constrainedStateVector(VehicleState::kPx) = constrainedState.GetPositionX();
        constrainedStateVector(VehicleState::kPy) = constrainedState.GetPositionY();
        constrainedStateVector(VehicleState::kPsi) = constrainedState.GetOrientation();
        constrainedStateVector(VehicleState::kU) = constrainedState.GetVelocity();
        constrainedStateVector(VehicleState::kV) = constrainedState.GetLateralVelocity();
        constrainedStateVector(VehicleState::kR) = constrainedState.GetRotationalVelocity();
        constrainedStateVector(VehicleState::kOmegaL) = constrainedState.GetWheelSpeedLeft();
        constrainedStateVector(VehicleState::kOmegaR) = constrainedState.GetWheelSpeedRight();
        constrainedStateVector(VehicleState::kBgz) = constrainedState.GetGyroBiasZ();
        VehicleState::NormalizeStateVector(constrainedStateVector);
        _filter.setStateSquareRootCovariance(
            constrainedStateVector,
            constrainedState.GetSqrtCovariance());
        updateNonholonomicDiagnostics(false);
    }

    void SrUkfCore::updateCommandSignFlipWindow(float dtSeconds) noexcept
    {
        const float averageDriveCommand = _lastControl.Average();
        const float averageDriveCommandSign =
            (std::fabs(averageDriveCommand) > 1.0e-4f) ?
            ((averageDriveCommand > 0.0f) ? 1.0f : -1.0f) :
            0.0f;
        if ((averageDriveCommandSign != 0.0f) &&
            (_previousAverageDriveCommandSign != 0.0f) &&
            (averageDriveCommandSign != _previousAverageDriveCommandSign))
        {
            _timeSinceCommandSignFlipS = 0.0f;
        }
        else if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            _timeSinceCommandSignFlipS += dtSeconds;
        }
        _previousAverageDriveCommandSign =
            (averageDriveCommandSign != 0.0f) ?
            averageDriveCommandSign :
            _previousAverageDriveCommandSign;
    }

    void SrUkfCore::updateStationaryCertification(float yawRateRadps) noexcept
    {
        const bool stationaryCandidate = IsStationaryCandidate(
            _lastControl,
            _commandedLinearMps,
            _commandedAngularRadps,
            _lastEncoderObs,
            yawRateRadps,
            _gyroBiasAnchorRadps,
            _accelBodyXMps2,
            _accelBodyYMps2,
            _saturationFlags);
        if (stationaryCandidate)
        {
            if (_stationaryCandidateDwellS <= 0.0f)
            {
                _stationaryCandidatePoseReferenceState = _prePredictState;
                _stationaryCandidatePoseReferenceCovariance = _prePredictCovariance;
                _stationaryCandidatePoseReferenceValid = _havePredictionReference;
            }
            _stationaryCandidateDwellS += _lastEncoderDtSeconds;
        }
        else
        {
            _stationaryCandidateDwellS = 0.0f;
            _stationaryCandidatePoseReferenceValid = false;
        }
        _stationaryCertified = _stationaryCandidateDwellS >= Tuning().stationaryCertificationDwellS;
    }

    void SrUkfCore::pushYawWindowContribution(
        float dtSeconds,
        float ukfYawRateRadps,
        float gyroYawRateRadps) noexcept
    {
        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)))
        {
            return;
        }

        if (_yawWindowSize == _yawWindowDtSeconds.size())
        {
            _yawWindowSpanS -= _yawWindowDtSeconds[_yawWindowHead];
            _yawWindowHead = (_yawWindowHead + 1U) % _yawWindowDtSeconds.size();
            --_yawWindowSize;
        }

        const std::size_t tail = (_yawWindowHead + _yawWindowSize) % _yawWindowDtSeconds.size();
        _yawWindowDtSeconds[tail] = dtSeconds;
        _yawWindowUkfIntegralRad[tail] = ukfYawRateRadps * dtSeconds;
        _yawWindowGyroIntegralRad[tail] = gyroYawRateRadps * dtSeconds;
        _yawWindowSpanS += dtSeconds;
        ++_yawWindowSize;

        while ((_yawWindowSize > 0U) && (_yawWindowSpanS > Tuning().yawWindowDurationS))
        {
            _yawWindowSpanS -= _yawWindowDtSeconds[_yawWindowHead];
            _yawWindowHead = (_yawWindowHead + 1U) % _yawWindowDtSeconds.size();
            --_yawWindowSize;
        }

        float ukfIntegralRad = 0.0f;
        float gyroIntegralRad = 0.0f;
        for (std::size_t index = 0; index < _yawWindowSize; ++index)
        {
            const std::size_t current = (_yawWindowHead + index) % _yawWindowDtSeconds.size();
            ukfIntegralRad += _yawWindowUkfIntegralRad[current];
            gyroIntegralRad += _yawWindowGyroIntegralRad[current];
        }
        _yawWindowMismatchRad = std::fabs(ukfIntegralRad - gyroIntegralRad);
    }

    void SrUkfCore::updateYawConsistencyMetrics(float yawRateRadps, float ukfYawRateRadps) noexcept
    {
        const float correctedGyroRadps = correctedYawRateRadps(yawRateRadps);
        const float comparisonYawRateRadps =
            std::isfinite(ukfYawRateRadps) ?
            ukfYawRateRadps :
            _filter.state()(VehicleState::kR);
        const float yawResidualRadps = std::fabs(comparisonYawRateRadps - correctedGyroRadps);
        const float alpha =
            (std::isfinite(_lastEncoderDtSeconds) && (_lastEncoderDtSeconds > 0.0f)) ?
            (std::clamp)(
                _lastEncoderDtSeconds / (Tuning().yawConsistencyLowPassTauS + _lastEncoderDtSeconds),
                0.0f,
                1.0f) :
            1.0f;
        _yawConsistencyLowPassRadps += alpha * (yawResidualRadps - _yawConsistencyLowPassRadps);
        pushYawWindowContribution(_lastEncoderDtSeconds, comparisonYawRateRadps, correctedGyroRadps);
        if (_yawConsistencyLowPassRadps > Tuning().yawConsistencyLowPassThresholdRadps)
        {
            _yawConsistencyExceedDwellS += _lastEncoderDtSeconds;
        }
        else
        {
            _yawConsistencyExceedDwellS = 0.0f;
        }
    }

    void SrUkfCore::updateOperatingMode(float dtSeconds) noexcept
    {
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            if (_timeSinceStationaryExitS < std::numeric_limits<float>::infinity())
            {
                _timeSinceStationaryExitS += dtSeconds;
            }
            if (_nhcReenableDelayRemainingS > 0.0f)
            {
                _nhcReenableDelayRemainingS = (std::max)(0.0f, _nhcReenableDelayRemainingS - dtSeconds);
            }
        }

        const bool recentCommandSignFlip =
            (_timeSinceCommandSignFlipS <= Tuning().commandSignFlipWindowS);
        const bool recentStationaryExit =
            (_timeSinceStationaryExitS <= Tuning().stationaryExitLaunchWindowS);
        const bool launchTrigger = HasLaunchOrReversalTrigger(
            _filter.state()(VehicleState::kU),
            _lastControl.LeftMotorPwm(),
            _lastControl.RightMotorPwm(),
            _leftLaunchAssistFloor,
            _rightLaunchAssistFloor,
            recentCommandSignFlip,
            recentStationaryExit);
        if (launchTrigger)
        {
            _launchHoldRemainingS = Tuning().launchHoldS;
        }
        else if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            _launchHoldRemainingS = (std::max)(0.0f, _launchHoldRemainingS - dtSeconds);
        }

        const bool inconsistencyTrigger = HasInconsistentOrSaturatedTrigger(
            _saturationFlags,
            (_yawConsistencyExceedDwellS >= Tuning().yawConsistencyExceedDwellS) ? _yawConsistencyLowPassRadps : 0.0f,
            _yawWindowMismatchRad,
            _nonholonomicConstraintEnabled,
            _nhcResidualSigma);
        if (inconsistencyTrigger)
        {
            _inconsistentHoldRemainingS = Tuning().inconsistentHoldS;
        }
        else if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            _inconsistentHoldRemainingS = (std::max)(0.0f, _inconsistentHoldRemainingS - dtSeconds);
        }

        const OperatingMode previousMode = _operatingMode;
        _operatingMode = ClassifyOperatingMode(
            _stationaryCertified,
            _launchHoldRemainingS > 0.0f,
            _inconsistentHoldRemainingS > 0.0f);
        if ((previousMode == OperatingMode::StationaryCertified) &&
            (_operatingMode != OperatingMode::StationaryCertified))
        {
            _timeSinceStationaryExitS = 0.0f;
        }
        if (_operatingMode == OperatingMode::StationaryCertified)
        {
            _timeSinceStationaryExitS = std::numeric_limits<float>::infinity();
        }
    }

    void SrUkfCore::updateProcessNoiseForMode() noexcept
    {
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
    }

    bool SrUkfCore::shouldEnableNonholonomicConstraint() const noexcept
    {
        const float forwardSpeedMps = std::fabs(_filter.state()(VehicleState::kU));
        const float driveDelta = 2.0f * std::fabs(_lastControl.Differential());
        return
            !_stationaryCertified &&
            (_operatingMode == OperatingMode::GripLinear) &&
            (_saturationFlags == 0U) &&
            (forwardSpeedMps >= Tuning().nhcMinimumEnableForwardSpeedMps) &&
            (driveDelta <= Tuning().nhcMaxDriveCommandDelta) &&
            (_timeSinceCommandSignFlipS > Tuning().commandSignFlipWindowS) &&
            (_nhcReenableDelayRemainingS <= 0.0f);
    }

    float SrUkfCore::correctedYawRateRadps(float yawRateRawRadps) const noexcept
    {
        if (!std::isfinite(yawRateRawRadps))
        {
            return 0.0f;
        }

        const float gyroBiasZRadps =
            std::isfinite(_filter.state()(VehicleState::kBgz)) ?
            _filter.state()(VehicleState::kBgz) :
            _gyroBiasAnchorRadps;
        return yawRateRawRadps - gyroBiasZRadps;
    }

    void SrUkfCore::updateInitialStationaryGyroBias(
        float yawRateRadps,
        bool stationaryZeroMotionCandidate) noexcept
    {
        _biasUpdateEnabled = false;
        if (_initialStationaryGyroBiasPhaseExited)
        {
            return;
        }

        const bool startupStationaryTick =
            stationaryZeroMotionCandidate &&
            controlCommandsAreEffectivelyZero() &&
            (_saturationFlags == 0U);
        if (!startupStationaryTick)
        {
            _initialStationaryGyroBiasPhaseExited = true;
            return;
        }

        if (!std::isfinite(yawRateRadps))
        {
            return;
        }

        if (_initialStationaryGyroBiasSampleOrdinal < (std::numeric_limits<std::uint16_t>::max)())
        {
            ++_initialStationaryGyroBiasSampleOrdinal;
        }

        if ((_initialStationaryGyroBiasSampleOrdinal >= kInitialStationaryGyroBiasSeedStartSample) &&
            (_initialStationaryGyroBiasSampleOrdinal <= kInitialStationaryGyroBiasSeedEndSample))
        {
            _initialStationaryGyroBiasSeedAccumRadps += static_cast<double>(yawRateRadps);
            if (_initialStationaryGyroBiasCollectedSeedSamples < (std::numeric_limits<std::uint16_t>::max)())
            {
                ++_initialStationaryGyroBiasCollectedSeedSamples;
            }
        }

        const float measurementVarianceRadps2 = GyroMeasurementVarianceRadps2();
        if (!_initialStationaryGyroBiasSeedApplied)
        {
            if ((_initialStationaryGyroBiasSampleOrdinal >= kInitialStationaryGyroBiasSeedEndSample) &&
                (_initialStationaryGyroBiasCollectedSeedSamples > 0U))
            {
                _gyroBiasAnchorRadps = static_cast<float>(
                    _initialStationaryGyroBiasSeedAccumRadps /
                    static_cast<double>(_initialStationaryGyroBiasCollectedSeedSamples));
                _gyroBiasAnchorVarianceRadps2 = kGyroBiasInitialVarianceUnseededRadps2;
                if (!std::isfinite(_gyroBiasAnchorVarianceRadps2) ||
                    !(_gyroBiasAnchorVarianceRadps2 > 0.0f))
                {
                    _gyroBiasAnchorVarianceRadps2 = kGyroBiasInitialVarianceUnseededRadps2;
                }
                _initialStationaryGyroBiasSeedApplied = true;
                _biasUpdateEnabled = true;
            }
            return;
        }

        const float priorVarianceRadps2 =
            (std::isfinite(_gyroBiasAnchorVarianceRadps2) && (_gyroBiasAnchorVarianceRadps2 > 0.0f)) ?
            _gyroBiasAnchorVarianceRadps2 :
            kGyroBiasInitialVarianceUnseededRadps2;
        const float predictedVarianceRadps2 =
            priorVarianceRadps2 +
            kGyroBiasProcessVarianceStationaryRadps2PerSample;
        const float innovationVarianceRadps2 = predictedVarianceRadps2 + measurementVarianceRadps2;
        if (!(std::isfinite(predictedVarianceRadps2) && std::isfinite(innovationVarianceRadps2)) ||
            !(innovationVarianceRadps2 > 0.0f))
        {
            return;
        }

        const float kalmanGain = (std::clamp)(predictedVarianceRadps2 / innovationVarianceRadps2, 0.0f, 1.0f);
        _gyroBiasAnchorRadps += kalmanGain * (yawRateRadps - _gyroBiasAnchorRadps);
        _gyroBiasAnchorVarianceRadps2 = (1.0f - kalmanGain) * predictedVarianceRadps2;
        if (!std::isfinite(_gyroBiasAnchorVarianceRadps2) || !(_gyroBiasAnchorVarianceRadps2 > 0.0f))
        {
            _gyroBiasAnchorVarianceRadps2 = kGyroBiasInitialVarianceUnseededRadps2;
        }
        _biasUpdateEnabled = true;
    }

    float SrUkfCore::wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept
    {
        const float normalizedConfidence = (std::clamp)(confidence, 0.0f, 1.0f);
        return minimumNoise + ((1.0f - normalizedConfidence) * 0.020f);
    }

    float SrUkfCore::wallPredictionForSensor(
        const StateVector& sigmaPoint,
        const SensorMount& sensor,
        const Maze& maze) const noexcept
    {
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint);
        const GeometryPrediction prediction = _geometryModel.predictRay(frame, sensor, maze, _params.noHitRangeM);
        return prediction.hit ? prediction.rangeM : _params.noHitRangeM;
    }

    Eigen::Matrix<float, 2, 1> SrUkfCore::frontPairPredictionForState(
        const StateVector& sigmaPoint,
        const Maze& maze) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint);
        const GeometryPrediction leftPrediction =
            _geometryModel.predictRay(frame, _params.frontLeftSensor, maze, _params.noHitRangeM);
        const GeometryPrediction rightPrediction =
            _geometryModel.predictRay(frame, _params.frontRightSensor, maze, _params.noHitRangeM);
        prediction(0) = leftPrediction.hit ? leftPrediction.rangeM : _params.noHitRangeM;
        prediction(1) = rightPrediction.hit ? rightPrediction.rangeM : _params.noHitRangeM;
        return prediction;
    }

    FrontPairUpdateResult SrUkfCore::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const Maze& maze) noexcept
    {
        FrontPairUpdateResult result{};
        result.leftPrediction = _geometryModel.predictRay(_filter.state(), _params.frontLeftSensor, maze, _params.noHitRangeM);
        result.rightPrediction = _geometryModel.predictRay(_filter.state(), _params.frontRightSensor, maze, _params.noHitRangeM);
        result.filter.attempted = left.valid && right.valid;
        if (!result.filter.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 2, 2> sqrtNoise = _sqrtFrontNoise;
        sqrtNoise(0, 0) = wallNoiseFromConfidence(left.confidence, _sqrtFrontNoise(0, 0));
        sqrtNoise(1, 1) = wallNoiseFromConfidence(right.confidence, _sqrtFrontNoise(1, 1));

        Eigen::Matrix<float, 2, 1> z;
        z << left.rho, right.rho;
        result.filter.accepted = _filter.Update<2>(
            z,
            sqrtNoise,
            9.21034f,
            [this, &maze](const StateVector& sigmaPoint) noexcept
            {
                return frontPairPredictionForState(sigmaPoint, maze);
            });
        result.filter.nis = _filter.lastNis();
        return result;
    }

    WallUpdateResult SrUkfCore::updateSideSensor(
        RelativeDirection which,
        const WallObs& observation,
        const Maze& maze) noexcept
    {
        WallUpdateResult result{};
        const bool isLeft = which == RelativeDirection::Left90;
        const bool isRight = which == RelativeDirection::Right90;
        if (!isLeft && !isRight)
        {
            return result;
        }

        const SensorMount& sensor = isLeft ? _params.sideLeftSensor : _params.sideRightSensor;
        result.prediction = _geometryModel.predictRay(_filter.state(), sensor, maze, _params.noHitRangeM);
        result.filter.attempted = observation.valid;
        if (!result.filter.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 1, 1> sqrtNoise = _sqrtSideNoise;
        sqrtNoise(0, 0) = wallNoiseFromConfidence(observation.confidence, _sqrtSideNoise(0, 0));

        Eigen::Matrix<float, 1, 1> z;
        z << observation.rho;
        result.filter.accepted = _filter.Update<1>(
            z,
            sqrtNoise,
            7.87944f,
            [this, &maze, &sensor](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction(0) = wallPredictionForSensor(sigmaPoint, sensor, maze);
                return prediction;
            });
        result.filter.nis = _filter.lastNis();
        return result;
    }
}


