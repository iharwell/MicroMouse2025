#include "pch.h"
#include "SrUkfCore.h"

#include "PlanarAccelLateralUpdate.h"
#include "SoftOdometryAid.h"
#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>

namespace
{
    using CommandVector = MazeMap::App::Internal::CommandVector;

    MazeMap::WallGeometryModel::GeometryStateFrame BuildWallGeometryFrame(
        const MazeMap::WallGeometryModel& geometryModel,
        const MazeMap::VehicleState::StateVector& state) noexcept
    {
        return geometryModel.buildStateFrame(
            Eigen::Vector2f(
                state(MazeMap::VehicleState::kPx),
                state(MazeMap::VehicleState::kPy)),
            state(MazeMap::VehicleState::kPsi));
    }

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
    constexpr float kGyroBiasCovarianceFloorRadps2 = 1.0e-9f;
    constexpr float kStationaryBodyDecayTauS = 0.075f;
    constexpr float kStationaryWheelDecayTauS = 0.050f;
    constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.021187f;
    constexpr float kGeneralEncoderYawRateSigmaRadps = 0.111268f;
    constexpr float kStationaryEncoderVelocitySigmaMps = 0.002936f;
    constexpr float kEncoderPairNisThreshold = 13.81551f;
    constexpr float kImuYawRateVarianceRadps2 = 1.2e-6f;
    constexpr float kImuYawRateSigmaRadps = 0.0010954451f;
    constexpr float kGyroBiasProcessVarianceMovingRadps2PerSample = 0.0f;
    constexpr float kGyroBiasProcessVarianceStationaryRadps2PerSample = 3.0e-16f;
    constexpr float kGyroBiasInitialVarianceUnseededRadps2 = 3.05e-4f;
    constexpr float kImuAccelSigmaMps2 = 0.569900f;
    constexpr float kPivotScrubMaxCommandLinearMps = 0.03f;
    constexpr float kPivotScrubMinCommandAngularRadps = 1.0f;
    constexpr float kPivotScrubYawConsistencyThresholdRadps = 0.03f;
    constexpr float kPivotScrubYawWindowMismatchThresholdRad = 0.003f;
    constexpr float kPivotScrubZeroUSigmaMps = 0.06f;
    constexpr float kNhcBaseSigmaMps = 0.005f;
    constexpr float kNhcSpeedSlopePerMps = 0.05f;
    constexpr float kNhcMinimumSigmaMps = 0.005f;
    constexpr float kNhcMaximumSigmaMps = 0.040f;
    constexpr std::uint16_t kInitialStationaryGyroBiasSeedStartSample = 50U;
    constexpr std::uint16_t kInitialStationaryGyroBiasSeedEndSample = 150U;
    constexpr std::uint16_t kInitialStationaryGyroBiasSeedSampleCount =
        static_cast<std::uint16_t>(
            (kInitialStationaryGyroBiasSeedEndSample - kInitialStationaryGyroBiasSeedStartSample) + 1U);

    class ModeProcessNoise
    {
    public:
        constexpr explicit ModeProcessNoise(std::array<float, 6> values) noexcept
            : _values(values)
        {
        }

        constexpr float SigmaUSqrtQ() const noexcept { return _values[0]; }
        constexpr float SigmaVSqrtQ() const noexcept { return _values[1]; }
        constexpr float SigmaRSqrtQ() const noexcept { return _values[2]; }
        constexpr float SigmaOmegaSqrtQ() const noexcept { return _values[3]; }
        constexpr float StdRMin() const noexcept { return _values[4]; }
        constexpr float StdVMin() const noexcept { return _values[5]; }

    private:
        std::array<float, 6> _values{};
    };

    constexpr ModeProcessNoise kStationaryCertifiedProcessNoise{std::array<float, 6>{
        0.006f,
        0.000f,
        0.010f,
        0.050f,
        0.010f,
        0.0f
    }};

    constexpr ModeProcessNoise kLaunchOrReversalProcessNoise{std::array<float, 6>{
        0.020f,
        0.000f,
        0.050f,
        0.40f,
        0.030f,
        0.006f
    }};

    constexpr ModeProcessNoise kGripLinearProcessNoise{std::array<float, 6>{
        0.012f,
        0.000f,
        0.025f,
        0.400f,
        0.020f,
        0.006f
    }};

    constexpr ModeProcessNoise kInconsistentOrSaturatedProcessNoise{std::array<float, 6>{
        0.030f,
        0.002f,
        0.070f,
        0.50f,
        0.050f,
        0.020f
    }};

    const ModeProcessNoise& GetModeProcessNoise(const std::uint8_t modeId) noexcept
    {
        switch (modeId)
        {
        case 0U:
            return kStationaryCertifiedProcessNoise;
        case 1U:
            return kLaunchOrReversalProcessNoise;
        case 3U:
            return kInconsistentOrSaturatedProcessNoise;
        case 2U:
        default:
            return kGripLinearProcessNoise;
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

    float ContactRecoveryStrength(float memorySeverity, float recoverySeverity) noexcept
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
        const float sigmaRadps = kImuYawRateSigmaRadps;
        if (std::isfinite(sigmaRadps) && (sigmaRadps > 0.0f))
        {
            return sigmaRadps * sigmaRadps;
        }

        return kImuYawRateVarianceRadps2;
    }

    float GyroBiasProcessVarianceRadps2ForMode(const std::uint8_t modeId) noexcept
    {
        return
            (modeId == 0U) ?
            kGyroBiasProcessVarianceStationaryRadps2PerSample :
            kGyroBiasProcessVarianceMovingRadps2PerSample;
    }

    float GyroBiasProcessSquareRootForMode(const std::uint8_t modeId) noexcept
    {
        return MazeMap::Math::Sqrtf(GyroBiasProcessVarianceRadps2ForMode(modeId));
    }

    bool EmitDebugTextLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* format,
        ...) noexcept
    {
        if (sink == nullptr || type == nullptr || type[0] == '\0' || format == nullptr)
        {
            return false;
        }

        std::va_list args;
        va_start(args, format);
        const bool ok = sink(context, type, format, args);
        va_end(args);
        return ok;
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
        for (std::size_t index = 0; index < N; ++index)
        {
            const char* const resolvedPrefix = (prefix != nullptr) ? prefix : "";
            const char* const separator = (resolvedPrefix[0] != '\0') ? ";" : "";
            if (!EmitDebugTextLine(
                    context,
                    sink,
                    type,
                    "%s%s%s=%.9g",
                    resolvedPrefix,
                    separator,
                    fieldNames[index],
                    static_cast<double>(reader(index))))
            {
                return false;
            }
        }

        return true;
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
        const char* const rowName = fieldNames[static_cast<std::size_t>(row)];
        for (std::size_t column = 0U; column < N; ++column)
        {
            if (!EmitDebugTextLine(
                    context,
                    sink,
                    type,
                    "row=%s;%s=%.9g",
                    rowName,
                    fieldNames[column],
                    static_cast<double>(matrix(row, static_cast<int>(column)))))
            {
                return false;
            }
        }

        return true;
    }
}

namespace MazeMap
{
    SrUkfCore::SrUkfCore(const PlantModel& plantModel, VehicleState& runtimeState) noexcept
        : _plantModel(plantModel)
        , _runtimeState(runtimeState)
        , _geometryModel()
        , _workingFilter(runtimeState._state, runtimeState._sqrtCovariance)
        , _frozenLeftAppliedBankTorqueNm(0.0f)
        , _frozenRightAppliedBankTorqueNm(0.0f)
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
        _workingFilter.setStateNormalizer(&VehicleState::NormalizeStateVector);
        _workingFilter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);

        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _workingFilter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);

        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
        _gyroBiasAnchorRadps = _workingFilter.state()(VehicleState::kBgz);
        _gyroBiasAnchorVarianceRadps2 = _workingFilter.variance(VehicleState::kBgz);
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
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
        bool accelBiasValid,
        float accelBodyXMps2,
        float accelBodyYMps2) noexcept
    {
        _commandedLinearMps = commandedLinearMps;
        _commandedAngularRadps = commandedAngularRadps;
        _saturationFlags = saturationFlags;
        _accelBodyXMps2 = accelBiasValid ? accelBodyXMps2 : std::numeric_limits<float>::quiet_NaN();
        _accelBodyYMps2 = accelBiasValid ? accelBodyYMps2 : std::numeric_limits<float>::quiet_NaN();
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
                (kNhcBaseSigmaMps * kNhcBaseSigmaMps) +
                ((kNhcSpeedSlopePerMps * resolvedForwardSpeedMps) *
                    (kNhcSpeedSlopePerMps * resolvedForwardSpeedMps)));
        return (std::clamp)(sigmaMps, kNhcMinimumSigmaMps, kNhcMaximumSigmaMps);
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
            std::fabs(commandedLinearMps) < kStationaryCandidateMaxLinearCommandMps &&
            std::isfinite(commandedAngularRadps) &&
            std::fabs(commandedAngularRadps) < kStationaryCandidateMaxAngularCommandRadps &&
            std::isfinite(control.LeftMotorPwm()) &&
            std::fabs(control.LeftMotorPwm()) < kStationaryCandidateMaxDriveCommand &&
            std::isfinite(control.RightMotorPwm()) &&
            std::fabs(control.RightMotorPwm()) < kStationaryCandidateMaxDriveCommand &&
            std::isfinite(observation.omegaLeftRadps) &&
            std::fabs(observation.omegaLeftRadps) < kStationaryCandidateMaxEncoderOmegaRadps &&
            std::isfinite(observation.omegaRightRadps) &&
            std::fabs(observation.omegaRightRadps) < kStationaryCandidateMaxEncoderOmegaRadps &&
            std::isfinite(gyroRawRadps) &&
            std::isfinite(gyroBiasAnchorRadps) &&
            std::fabs(gyroRawRadps - gyroBiasAnchorRadps) < kStationaryCandidateMaxCorrectedGyroRadps &&
            std::isfinite(accelBodyXMps2) &&
            std::fabs(accelBodyXMps2) < kStationaryCandidateMaxAccelMps2 &&
            std::isfinite(accelBodyYMps2) &&
            std::fabs(accelBodyYMps2) < kStationaryCandidateMaxAccelMps2 &&
            (saturationFlags == 0U);
    }

    bool SrUkfCore::HasLaunchOrReversalTrigger(
        float forwardSpeedMps,
        float leftDriveCommand,
        float rightDriveCommand,
        bool recentCommandSignFlip,
        bool recentStationaryExit) noexcept
    {
        return
            recentCommandSignFlip ||
            (std::isfinite(forwardSpeedMps) &&
                (std::fabs(forwardSpeedMps) < kLaunchLowSpeedThresholdMps) &&
                std::isfinite(leftDriveCommand) &&
                std::isfinite(rightDriveCommand) &&
                (std::fabs(leftDriveCommand - rightDriveCommand) > kLaunchDriveCommandDeltaThreshold)) ||
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
                (yawConsistencyLowPassRadps > kYawConsistencyLowPassThresholdRadps)) ||
            (std::isfinite(yawWindowMismatchRad) &&
                (yawWindowMismatchRad > kYawWindowMismatchThresholdRad)) ||
            (nhcEnabled &&
                std::isfinite(nhcResidualSigma) &&
                (std::fabs(nhcResidualSigma) > kNhcResidualTripSigma));
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
        float yawWindowMismatchRad) const noexcept
    {
        if (!std::isfinite(commandedLinearMps) ||
            !std::isfinite(commandedAngularRadps) ||
            !std::isfinite(observation.omegaLeftRadps) ||
            !std::isfinite(observation.omegaRightRadps))
        {
            return false;
        }

        if ((std::fabs(commandedLinearMps) > kPivotScrubMaxCommandLinearMps) ||
            (std::fabs(commandedAngularRadps) < kPivotScrubMinCommandAngularRadps))
        {
            return false;
        }

        const float measuredLinearSpeedMps =
            std::fabs(_plantModel.measuredLinearSpeedMps(observation));
        if (!std::isfinite(measuredLinearSpeedMps) ||
            (measuredLinearSpeedMps > kPivotScrubMaxCommandLinearMps))
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
                (std::fabs(yawConsistencyLowPassRadps) > kPivotScrubYawConsistencyThresholdRadps);
        }
        if (std::isfinite(yawWindowMismatchRad))
        {
            yawConflictKnown = true;
            yawConflict =
                yawConflict ||
                (std::fabs(yawWindowMismatchRad) > kPivotScrubYawWindowMismatchThresholdRad);
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
                    return _workingFilter.state()(static_cast<int>(index));
                }))
        {
            return false;
        }

        const StateMatrix covariance = _workingFilter.covariance();
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

        if (!_plantModel.WriteUkfPlantDebugTextDump(context, sink))
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
                "mode_id=%u;stationary_certified=%s;bias_update_enabled=%s;nhc_enabled=%s",
                static_cast<unsigned>(static_cast<std::uint8_t>(_operatingMode)),
                _stationaryCertified ? "true" : "false",
                _biasUpdateEnabled ? "true" : "false",
                _nonholonomicConstraintEnabled ? "true" : "false"))
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
                static_cast<double>(_frozenLeftAppliedBankTorqueNm),
                static_cast<double>(_frozenRightAppliedBankTorqueNm)))
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

        const StateMatrix covarianceDiagonal = _workingFilter.covariance();
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
        return VehicleState::DefaultInitialCovariance();
    }

    SrUkfCore::StateMatrix SrUkfCore::BuildProcessNoiseSquareRootForMode(const OperatingMode mode) noexcept
    {
        const ModeProcessNoise& config = GetModeProcessNoise(static_cast<std::uint8_t>(mode));
        StateMatrix sqrtNoise = StateMatrix::Zero();
        sqrtNoise.diagonal() <<
            0.0f,
            0.0f,
            0.0f,
            config.SigmaUSqrtQ(),
            config.SigmaVSqrtQ(),
            config.SigmaRSqrtQ(),
            config.SigmaOmegaSqrtQ(),
            config.SigmaOmegaSqrtQ(),
            0.0f;
        return sqrtNoise;
    }

    bool SrUkfCore::reset(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _workingFilter.setState(state, covariance);
        _workingFilter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);
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
        _accelBodyXMps2 = 0.0f;
        _accelBodyYMps2 = 0.0f;
        _runtimeState.SetLongitudinalAcceleration(0.0f);
        _runtimeState.SetLateralAcceleration(0.0f);
        _runtimeState.SetYawAcceleration(0.0f);
        _stationaryCandidateDwellS = 0.0f;
        _stationaryCandidatePoseReferenceState = _workingFilter.state();
        _stationaryCandidatePoseReferenceCovariance = _workingFilter.covariance();
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
        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _workingFilter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _lastControl = App::Internal::CommandVector(0.0f, 0.0f);
        _lastFanDutyCycle = 0.80f;
        _lastBatteryVoltageV = 0.0f;
        _lastEncoderObs = EncoderObs{};
        _lastEncoderDtSeconds = 0.0f;
        _prePredictState = _workingFilter.state();
        _havePredictionReference = false;
        _acceptedEncoderUpdateSincePredict = false;
        const ModeProcessNoise& resetModeNoise =
            GetModeProcessNoise(static_cast<std::uint8_t>(_operatingMode));
        (void)_workingFilter.floorVariance(VehicleState::kR, resetModeNoise.StdRMin() * resetModeNoise.StdRMin());
        (void)_workingFilter.floorVariance(VehicleState::kV, resetModeNoise.StdVMin() * resetModeNoise.StdVMin());
        (void)_workingFilter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        _gyroBiasAnchorVarianceRadps2 = _workingFilter.variance(VehicleState::kBgz);
        _stationaryCandidatePoseReferenceCovariance = _workingFilter.covariance();
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

    float SrUkfCore::ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            return std::numeric_limits<float>::infinity();
        }

        return kEncoderPairNisThreshold;
    }

    SrUkfCore::GripUtilizationSnapshot SrUkfCore::buildGripUtilizationSnapshot(
        const StateVector& currentState,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float leftClosureResidualMps,
        float rightClosureResidualMps,
        float fanDutyCycle) const noexcept
    {
        GripUtilizationSnapshot snapshot{};
        const PlantDerivatives derivatives =
            _plantModel.forwardStepFromAppliedBankTorques(
                currentState,
                leftAppliedBankTorqueNm,
                rightAppliedBankTorqueNm,
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

        const float closureSigmaMps = (std::max)(kGeneralEncoderLinearSpeedSigmaMps, 1.0e-3f);
        const float longitudinalClosureSeverity =
            (std::fabs(leftClosureResidualMps + rightClosureResidualMps) * 0.5f) / closureSigmaMps;
        const float differentialClosureSeverity =
            std::fabs(leftClosureResidualMps - rightClosureResidualMps) / closureSigmaMps;
        const float lateralAccelerationSeverity =
            std::isfinite(_accelBodyXMps2) ?
            _plantModel.peakCombinedAccelerationUsage(_accelBodyXMps2) :
            0.0f;
        const float yawConsistencySeverity =
            (kYawConsistencyLowPassThresholdRadps > 0.0f) ?
            (std::fabs(_yawConsistencyLowPassRadps) / kYawConsistencyLowPassThresholdRadps) :
            0.0f;
        const float leftBankAnomalySeverity = std::fabs(leftClosureResidualMps) / closureSigmaMps;
        const float rightBankAnomalySeverity = std::fabs(rightClosureResidualMps) / closureSigmaMps;

        snapshot.longitudinalClosureSeverity =
            Clamp01(
                (std::max)(
                    longitudinalClosureSeverity,
                    _plantModel.sustainedCombinedAccelerationUsage(derivatives.longitudinalAccelMps2)));
        snapshot.differentialClosureSeverity =
            Clamp01(
                (std::max)(
                    differentialClosureSeverity,
                    _plantModel.nominalCombinedAccelerationUsage(derivatives.yawAccelRadps2)));
        snapshot.lateralAccelerationSeverity =
            Clamp01(
                (std::max)(
                    lateralAccelerationSeverity,
                    _plantModel.peakCombinedAccelerationUsage(derivatives.lateralAccelMps2)));
        snapshot.yawConsistencySeverity =
            Clamp01(
                (std::max)(
                    yawConsistencySeverity,
                    _plantModel.stopExitYawRateUsage(currentState(VehicleState::kR))));

        snapshot.leftBankAnomalySeverity =
            Clamp01(
                (std::max)(
                    leftBankAnomalySeverity,
                    PrecursorSeverity(snapshot.leftBankPreProjectionUtilization)));
        snapshot.rightBankAnomalySeverity =
            Clamp01(
                (std::max)(
                    rightBankAnomalySeverity,
                    PrecursorSeverity(snapshot.rightBankPreProjectionUtilization)));

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
            ContactRecoveryStrength(_transientContactMemory.leftBankMemory, _regripRecovery.leftBankRecoveryScore) :
            ContactRecoveryStrength(_transientContactMemory.rightBankMemory, _regripRecovery.rightBankRecoveryScore);
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
            ContactRecoveryStrength(_transientContactMemory.leftBankMemory, _regripRecovery.leftBankRecoveryScore);
        const float rightStrength =
            ContactRecoveryStrength(_transientContactMemory.rightBankMemory, _regripRecovery.rightBankRecoveryScore);
        const float memorySeverity = 0.5f * (leftStrength + rightStrength);
        const float recoverySeverity =
            0.5f *
            (Clamp01(_regripRecovery.leftBankRecoveryScore) + Clamp01(_regripRecovery.rightBankRecoveryScore));
        return 1.0f + (0.75f * utilizationSeverity) + (0.35f * memorySeverity) + (0.35f * recoverySeverity);
    }

    void SrUkfCore::refreshFrozenPolicyState(
        float dtSeconds,
        const App::Internal::CommandVector& control,
        float fanDutyCycle,
        float batteryVoltageV) noexcept
    {
        const StateVector& currentState = _workingFilter.state();
        const StateVector& torqueEstimateState =
            _havePredictionReference ?
            _prePredictState :
            currentState;
        float leftAppliedBankTorqueNm = 0.0f;
        float rightAppliedBankTorqueNm = 0.0f;
        _plantModel.resolveAppliedBankTorques(
            torqueEstimateState,
            control,
            batteryVoltageV,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm);

        const float measuredLeftWheelVelocityMps =
            Vehicle::WheelLinearVelocityFromOmega(_lastEncoderObs.omegaLeftRadps);
        const float measuredRightWheelVelocityMps =
            Vehicle::WheelLinearVelocityFromOmega(_lastEncoderObs.omegaRightRadps);
        const Eigen::Vector2f predictedWheelVelocityMps =
            _plantModel.wheelLinearVelocityFromBodyState(currentState);
        const float leftClosureResidualMps =
            measuredLeftWheelVelocityMps - predictedWheelVelocityMps(0);
        const float rightClosureResidualMps =
            measuredRightWheelVelocityMps - predictedWheelVelocityMps(1);
        _lastClosureResidualLeftMps = leftClosureResidualMps;
        _lastClosureResidualRightMps = rightClosureResidualMps;
        GripUtilizationSnapshot utilization =
            buildGripUtilizationSnapshot(
                currentState,
                leftAppliedBankTorqueNm,
                rightAppliedBankTorqueNm,
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
            (std::fabs(correctedYawRateForStationaryLockRadps) <= kStationaryCandidateMaxCorrectedGyroRadps) &&
            (std::fabs(currentState(VehicleState::kU)) <= kStationaryCandidateMaxLinearCommandMps) &&
            (std::fabs(currentState(VehicleState::kV)) <= kStationaryCandidateMaxLinearCommandMps) &&
            (std::fabs(_accelBodyXMps2) <= kStationaryCandidateMaxAccelMps2) &&
            (std::fabs(_accelBodyYMps2) <= kStationaryCandidateMaxAccelMps2) &&
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
             (_timeSinceStationaryExitS <= kStationaryExitLaunchWindowS)) &&
            (std::fabs(currentState(VehicleState::kU)) <= kLaunchLowSpeedThresholdMps);
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
        _frozenLeftAppliedBankTorqueNm = leftAppliedBankTorqueNm;
        _frozenRightAppliedBankTorqueNm = rightAppliedBankTorqueNm;
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

        bool anyAccepted = false;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const float baseSigmaMps =
            (std::max)(kGeneralEncoderLinearSpeedSigmaMps, 1.0e-3f);
        const float leftScale =
            _frozenSchedule.closureCovarianceScaleLeft *
            closurePseudoMeasurementScale(RelativeDirection::Left90);
        const float rightScale =
            _frozenSchedule.closureCovarianceScaleRight *
            closurePseudoMeasurementScale(RelativeDirection::Right90);

        const auto applySideUpdate =
            [this, &invokeLoop, baseSigmaMps](
                float measuredLinearSpeedMps,
                float covarianceScale,
                int wheelIndex,
                float& innovationStorageMps,
                float& nisStorage) noexcept
        {
            Eigen::Matrix<float, 1, 1> measurement;
            measurement << measuredLinearSpeedMps;
            const Eigen::Vector2f predictedWheelVelocityMps =
                _plantModel.wheelLinearVelocityFromBodyState(_workingFilter.state());
            innovationStorageMps =
                measuredLinearSpeedMps -
                predictedWheelVelocityMps(wheelIndex);
            Eigen::Matrix<float, 1, 1> sqrtNoise;
            sqrtNoise(0, 0) =
                baseSigmaMps *
                MazeMap::Math::Sqrtf((std::max)(covarianceScale, 0.25f));
            const bool accepted = _workingFilter.Update<1>(
                measurement,
                sqrtNoise,
                25.0f,
                [this, wheelIndex](const StateVector& sigmaPoint) noexcept
                {
                    Eigen::Matrix<float, 1, 1> prediction;
                    prediction << _plantModel.wheelLinearVelocityFromBodyState(sigmaPoint)(wheelIndex);
                    return prediction;
                },
                invokeLoop);
            nisStorage = _workingFilter.lastNis();
            return accepted;
        };

        const float measuredLeftWheelVelocityMps =
            Vehicle::WheelLinearVelocityFromOmega(_lastEncoderObs.omegaLeftRadps);
        const float measuredRightWheelVelocityMps =
            Vehicle::WheelLinearVelocityFromOmega(_lastEncoderObs.omegaRightRadps);
        anyAccepted =
            applySideUpdate(
                measuredLeftWheelVelocityMps,
                leftScale,
                0,
                _lastClosureLeftInnovationMps,
                _lastClosureLeftNis) ||
            anyAccepted;
        anyAccepted =
            applySideUpdate(
                measuredRightWheelVelocityMps,
                rightScale,
                1,
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
        _lastLateralPseudoInnovationMps = -_workingFilter.state()(VehicleState::kV);
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) =
            ComputeNonholonomicSigmaMps(std::fabs(_workingFilter.state()(VehicleState::kU))) *
            MazeMap::Math::Sqrtf((std::max)(covarianceScale, 0.25f));
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const bool accepted = _workingFilter.Update<1>(
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
        _lastLateralPseudoNis = _workingFilter.lastNis();
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

        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
        const PlantDerivatives presentDerivatives =
            _plantModel.forwardStep(control, fanDutyCycle, batteryVoltageV);
        _runtimeState.SetLongitudinalAcceleration(presentDerivatives.longitudinalAccelMps2);
        _runtimeState.SetLateralAcceleration(presentDerivatives.lateralAccelMps2);
        _runtimeState.SetYawAcceleration(presentDerivatives.yawAccelRadps2);
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
                _plantModel.encoderPairCovarianceRadps(
                    kGeneralEncoderLinearSpeedSigmaMps,
                    kGeneralEncoderYawRateSigmaRadps);
            const float wheelVarianceFloorRadps2 =
                (std::max)(encoderCovariance(0, 0), encoderCovariance(1, 1));
            const float forwardVarianceFloorMps2 =
                kGeneralEncoderLinearSpeedSigmaMps * kGeneralEncoderLinearSpeedSigmaMps;
            const float lateralSigmaFloorMps =
                ComputeNonholonomicSigmaMps(std::fabs(_workingFilter.state()(VehicleState::kU)));
            const float lateralVarianceFloorMps2 = lateralSigmaFloorMps * lateralSigmaFloorMps;
            const float yawSigmaFloorRadps =
                (std::max)(kGeneralEncoderYawRateSigmaRadps, kRecoveryYawRateStdFloorRadps);
            const float yawVarianceFloorRadps2 = yawSigmaFloorRadps * yawSigmaFloorRadps;

            (void)_workingFilter.floorVariance(VehicleState::kU, forwardVarianceFloorMps2);
            (void)_workingFilter.floorVariance(VehicleState::kV, lateralVarianceFloorMps2);
            (void)_workingFilter.floorVariance(VehicleState::kR, yawVarianceFloorRadps2);
            (void)_workingFilter.floorVariance(VehicleState::kOmegaL, wheelVarianceFloorRadps2);
            (void)_workingFilter.floorVariance(VehicleState::kOmegaR, wheelVarianceFloorRadps2);
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
            GyroBiasProcessSquareRootForMode(static_cast<std::uint8_t>(_operatingMode));
        _workingFilter.setProcessNoiseSquareRoot(predictProcessNoiseSquareRoot);
        Eigen::Matrix<float, 3, 1> filterCommandVector;
        filterCommandVector << control.LeftMotorPwm(), control.RightMotorPwm(), fanDutyCycle;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const bool predicted = _workingFilter.Predict(
            dt,
            filterCommandVector,
            [this](const StateVector& sigmaPoint, const Eigen::Matrix<float, 3, 1>&, float sigmaDt) noexcept
            {
                if (_frozenSchedule.exactStationaryLock)
                {
                    return IntegrateStationaryHoldState(sigmaPoint, sigmaDt);
                }

                return _plantModel.integrateAppliedBankTorques(
                    sigmaPoint,
                    _frozenLeftAppliedBankTorqueNm,
                    _frozenRightAppliedBankTorqueNm,
                    _lastFanDutyCycle,
                    sigmaDt);
            },
            invokeLoop);
        if (predicted)
        {
            const ModeProcessNoise& modeNoise =
                GetModeProcessNoise(static_cast<std::uint8_t>(_operatingMode));
            (void)_workingFilter.floorVariance(VehicleState::kR, modeNoise.StdRMin() * modeNoise.StdRMin());
            (void)_workingFilter.floorVariance(VehicleState::kV, modeNoise.StdVMin() * modeNoise.StdVMin());
            (void)_workingFilter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
            _gyroBiasAnchorRadps = _workingFilter.state()(VehicleState::kBgz);
            _gyroBiasAnchorVarianceRadps2 = _workingFilter.variance(VehicleState::kBgz);
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
        const StateVector priorState = _workingFilter.state();
        const StateMatrix priorSqrtCovariance = _workingFilter.sqrtCovariance();
        const float measuredWheelVarianceRadps2 =
            _plantModel.measuredWheelVarianceRadps2(
                measured,
                kStationaryEncoderVelocitySigmaMps,
                kGeneralEncoderLinearSpeedSigmaMps,
                kGeneralEncoderYawRateSigmaRadps);
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
            _yawWindowMismatchRad);
        _pivotScrubEncoderBodyUpdateSkipped = _pivotScrubMode;
        _lastEncoderObs = measured;
        const bool wasExactStationaryLock = _frozenSchedule.exactStationaryLock;
        refreshFrozenPolicyState(_lastEncoderDtSeconds, _lastControl, _lastFanDutyCycle, _lastBatteryVoltageV);
        if (wasExactStationaryLock && !_frozenSchedule.exactStationaryLock)
        {
            const Eigen::Matrix<float, 2, 2> encoderCovariance =
                _plantModel.encoderPairCovarianceRadps(
                    kGeneralEncoderLinearSpeedSigmaMps,
                    kGeneralEncoderYawRateSigmaRadps);
            const float wheelVarianceFloorRadps2 =
                (std::max)(encoderCovariance(0, 0), encoderCovariance(1, 1));
            const float forwardVarianceFloorMps2 =
                kGeneralEncoderLinearSpeedSigmaMps * kGeneralEncoderLinearSpeedSigmaMps;
            const float lateralSigmaFloorMps =
                ComputeNonholonomicSigmaMps(std::fabs(_workingFilter.state()(VehicleState::kU)));
            const float lateralVarianceFloorMps2 = lateralSigmaFloorMps * lateralSigmaFloorMps;
            const float yawSigmaFloorRadps =
                (std::max)(kGeneralEncoderYawRateSigmaRadps, kRecoveryYawRateStdFloorRadps);
            const float yawVarianceFloorRadps2 = yawSigmaFloorRadps * yawSigmaFloorRadps;

            (void)_workingFilter.floorVariance(VehicleState::kU, forwardVarianceFloorMps2);
            (void)_workingFilter.floorVariance(VehicleState::kV, lateralVarianceFloorMps2);
            (void)_workingFilter.floorVariance(VehicleState::kR, yawVarianceFloorRadps2);
            (void)_workingFilter.floorVariance(VehicleState::kOmegaL, wheelVarianceFloorRadps2);
            (void)_workingFilter.floorVariance(VehicleState::kOmegaR, wheelVarianceFloorRadps2);
            _releaseInflationApplied = true;
        }

        Eigen::Matrix<float, 2, 1> z;
        z << measured.omegaLeftRadps, measured.omegaRightRadps;
        const Eigen::Matrix<float, 2, 2> sqrtEncoderNoise =
            _plantModel.encoderPairSqrtNoise(
                measured,
                kStationaryEncoderVelocitySigmaMps,
                kGeneralEncoderLinearSpeedSigmaMps,
                kGeneralEncoderYawRateSigmaRadps);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _workingFilter.Update<2>(
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
        const float encoderMeasurementNis = _workingFilter.lastNis();
        if (result.accepted)
        {
            _acceptedEncoderUpdateSincePredict = true;
            if (!_frozenSchedule.exactStationaryLock)
            {
                constexpr std::array<int, 2> kWheelOnlyIndices = {
                    VehicleState::kOmegaL,
                    VehicleState::kOmegaR
                };
                const StateVector updatedState = _workingFilter.state();
                const StateMatrix updatedSqrtCovariance = _workingFilter.sqrtCovariance();
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
                _workingFilter.setStateSquareRootCovariance(projectedState, projectedSqrtCovariance);
                applyWheelSpeedConstraint(measured, measuredWheelVarianceRadps2);
            }
            const StateVector& postEncoderState = _workingFilter.state();
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
        const StateVector& constrainedState = _workingFilter.state();
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
            (void)_workingFilter.floorVariance(
                VehicleState::kR,
                kRecoveryYawRateStdFloorRadps * kRecoveryYawRateStdFloorRadps);
            _nhcReenableDelayRemainingS = kRecoveryNhcReenableDelayS;
        }
        const ModeProcessNoise& encoderModeNoise =
            GetModeProcessNoise(static_cast<std::uint8_t>(_operatingMode));
        (void)_workingFilter.floorVariance(VehicleState::kR, encoderModeNoise.StdRMin() * encoderModeNoise.StdRMin());
        (void)_workingFilter.floorVariance(VehicleState::kV, encoderModeNoise.StdVMin() * encoderModeNoise.StdVMin());
        (void)_workingFilter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
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

        const StateVector priorState = _workingFilter.state();
        const StateMatrix priorSqrtCovariance = _workingFilter.sqrtCovariance();
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
        result.accepted = _workingFilter.Update<1>(
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
        const float yawMeasurementNis = _workingFilter.lastNis();
        _lastGyroNis = yawMeasurementNis;
        if (result.accepted)
        {
            const StateVector updatedState = _workingFilter.state();
            const StateMatrix updatedSqrtCovariance = _workingFilter.sqrtCovariance();
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
            _workingFilter.setStateSquareRootCovariance(projectedState, projectedSqrtCovariance);

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
                (void)_workingFilter.floorVariance(
                    VehicleState::kR,
                    kRecoveryYawRateStdFloorRadps * kRecoveryYawRateStdFloorRadps);
                _nhcReenableDelayRemainingS = kRecoveryNhcReenableDelayS;
            }
            const ModeProcessNoise& yawModeNoise =
                GetModeProcessNoise(static_cast<std::uint8_t>(_operatingMode));
            (void)_workingFilter.floorVariance(VehicleState::kR, yawModeNoise.StdRMin() * yawModeNoise.StdRMin());
            (void)_workingFilter.floorVariance(VehicleState::kV, yawModeNoise.StdVMin() * yawModeNoise.StdVMin());
            (void)_workingFilter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
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
                    _workingFilter.state(),
                    _lastControl,
                    _lastFanDutyCycle,
                    _lastBatteryVoltageV)(1);
            Eigen::Matrix<float, 1, 1> sqrtNoise;
            sqrtNoise(0, 0) = _sqrtImuNoise(2, 2);
            const bool forwardAccelAccepted = _workingFilter.Update<1>(
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
                        _lastBatteryVoltageV)(1);
                    return prediction;
                },
                invokeLoop);
            _lastForwardAccelNis = _workingFilter.lastNis();
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
                    _workingFilter.state(),
                    _lastControl,
                    _lastFanDutyCycle,
                    _lastBatteryVoltageV)(0);
            Eigen::Matrix<float, 1, 1> sqrtNoise;
            sqrtNoise(0, 0) = _sqrtImuNoise(1, 1);
            const bool lateralAccelAccepted = _workingFilter.Update<1>(
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
                        _lastBatteryVoltageV)(0);
                    return prediction;
                },
                invokeLoop);
            _lastLateralAccelNis = _workingFilter.lastNis();
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
            (void)_workingFilter.floorVariance(
                VehicleState::kR,
                kRecoveryYawRateStdFloorRadps * kRecoveryYawRateStdFloorRadps);
            _nhcReenableDelayRemainingS = kRecoveryNhcReenableDelayS;
        }
        const ModeProcessNoise& accelModeNoise =
            GetModeProcessNoise(static_cast<std::uint8_t>(_operatingMode));
        (void)_workingFilter.floorVariance(VehicleState::kR, accelModeNoise.StdRMin() * accelModeNoise.StdRMin());
        (void)_workingFilter.floorVariance(VehicleState::kV, accelModeNoise.StdVMin() * accelModeNoise.StdVMin());
        (void)_workingFilter.floorVariance(VehicleState::kBgz, kGyroBiasCovarianceFloorRadps2);
        _gyroBiasAnchorRadps = _workingFilter.state()(VehicleState::kBgz);
        _gyroBiasAnchorVarianceRadps2 = _workingFilter.variance(VehicleState::kBgz);
        result.accepted = accelAccepted || closureAccepted || lateralAccepted || stationaryApplied;
        return result;
    }

    bool SrUkfCore::HasExactZeroWheelObservation(const EncoderObs& observation) noexcept
    {
        return (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
    }

    bool SrUkfCore::controlCommandsAreEffectivelyZero() const noexcept
    {
        return
            (std::fabs(_lastControl.LeftMotorPwm()) <= 1.0e-6f) &&
            (std::fabs(_lastControl.RightMotorPwm()) <= 1.0e-6f) &&
            (std::fabs(_commandedLinearMps) <= kStationaryCandidateMaxLinearCommandMps) &&
            (std::fabs(_commandedAngularRadps) <= kStationaryCandidateMaxAngularCommandRadps);
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
        const bool accepted = _workingFilter.Update<1>(
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
        _nhcSigmaMps = ComputeNonholonomicSigmaMps(std::fabs(_workingFilter.state()(VehicleState::kU)));
        _nhcResidualMps = _workingFilter.state()(VehicleState::kV);
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

        StateVector constrainedState = _workingFilter.state();
        constrainedState(VehicleState::kOmegaL) = measured.omegaLeftRadps;
        constrainedState(VehicleState::kOmegaR) = measured.omegaRightRadps;
        VehicleState::NormalizeStateVector(constrainedState);

        StateMatrix constrainedCovariance = _workingFilter.covariance();
        constrainedCovariance.row(VehicleState::kOmegaL).setZero();
        constrainedCovariance.col(VehicleState::kOmegaL).setZero();
        constrainedCovariance.row(VehicleState::kOmegaR).setZero();
        constrainedCovariance.col(VehicleState::kOmegaR).setZero();
        const float constrainedVariance = (std::max)(wheelVarianceRadps2, 1.0e-12f);
        constrainedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = constrainedVariance;
        constrainedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = constrainedVariance;
        _workingFilter.setState(constrainedState, constrainedCovariance);
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
        constrainedState.SetPosition(Eigen::Vector2f(_workingFilter.state()(VehicleState::kPx), _workingFilter.state()(VehicleState::kPy)));
        constrainedState.SetOrientation(_workingFilter.state()(VehicleState::kPsi));
        constrainedState.SetVelocity(_workingFilter.state()(VehicleState::kU));
        constrainedState.SetLateralVelocity(_workingFilter.state()(VehicleState::kV));
        constrainedState.SetRotationalVelocity(_workingFilter.state()(VehicleState::kR));
        constrainedState.SetWheelSpeedLeft(_workingFilter.state()(VehicleState::kOmegaL));
        constrainedState.SetWheelSpeedRight(_workingFilter.state()(VehicleState::kOmegaR));
        constrainedState.SetGyroBiasZ(_workingFilter.state()(VehicleState::kBgz));
        constrainedState.SetCovariance(_workingFilter.covariance());
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
        _workingFilter.setStateSquareRootCovariance(
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
        _stationaryCertified = _stationaryCandidateDwellS >= kStationaryCertificationDwellS;
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

        while ((_yawWindowSize > 0U) && (_yawWindowSpanS > kYawWindowDurationS))
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
            _workingFilter.state()(VehicleState::kR);
        const float yawResidualRadps = std::fabs(comparisonYawRateRadps - correctedGyroRadps);
        const float alpha =
            (std::isfinite(_lastEncoderDtSeconds) && (_lastEncoderDtSeconds > 0.0f)) ?
            (std::clamp)(
                _lastEncoderDtSeconds / (kYawConsistencyLowPassTauS + _lastEncoderDtSeconds),
                0.0f,
                1.0f) :
            1.0f;
        _yawConsistencyLowPassRadps += alpha * (yawResidualRadps - _yawConsistencyLowPassRadps);
        pushYawWindowContribution(_lastEncoderDtSeconds, comparisonYawRateRadps, correctedGyroRadps);
        if (_yawConsistencyLowPassRadps > kYawConsistencyLowPassThresholdRadps)
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
            (_timeSinceCommandSignFlipS <= kCommandSignFlipWindowS);
        const bool recentStationaryExit =
            (_timeSinceStationaryExitS <= kStationaryExitLaunchWindowS);
        const bool launchTrigger = HasLaunchOrReversalTrigger(
            _workingFilter.state()(VehicleState::kU),
            _lastControl.LeftMotorPwm(),
            _lastControl.RightMotorPwm(),
            recentCommandSignFlip,
            recentStationaryExit);
        if (launchTrigger)
        {
            _launchHoldRemainingS = kLaunchHoldS;
        }
        else if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            _launchHoldRemainingS = (std::max)(0.0f, _launchHoldRemainingS - dtSeconds);
        }

        const bool inconsistencyTrigger = HasInconsistentOrSaturatedTrigger(
            _saturationFlags,
            (_yawConsistencyExceedDwellS >= kYawConsistencyExceedDwellS) ? _yawConsistencyLowPassRadps : 0.0f,
            _yawWindowMismatchRad,
            _nonholonomicConstraintEnabled,
            _nhcResidualSigma);
        if (inconsistencyTrigger)
        {
            _inconsistentHoldRemainingS = kInconsistentHoldS;
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
        const float forwardSpeedMps = std::fabs(_workingFilter.state()(VehicleState::kU));
        const float driveDelta = 2.0f * std::fabs(_lastControl.Differential());
        return
            !_stationaryCertified &&
            (_operatingMode == OperatingMode::GripLinear) &&
            (_saturationFlags == 0U) &&
            (forwardSpeedMps >= kNhcMinimumEnableForwardSpeedMps) &&
            (driveDelta <= kNhcMaxDriveCommandDelta) &&
            (_timeSinceCommandSignFlipS > kCommandSignFlipWindowS) &&
            (_nhcReenableDelayRemainingS <= 0.0f);
    }

    float SrUkfCore::correctedYawRateRadps(float yawRateRawRadps) const noexcept
    {
        if (!std::isfinite(yawRateRawRadps))
        {
            return 0.0f;
        }

        const float gyroBiasZRadps =
            std::isfinite(_workingFilter.state()(VehicleState::kBgz)) ?
            _workingFilter.state()(VehicleState::kBgz) :
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
        const float noHitRangeM = _plantModel.wallObservationNoHitRangeM();
        const WallGeometryModel::GeometryStateFrame frame = BuildWallGeometryFrame(_geometryModel, sigmaPoint);
        const GeometryPrediction prediction = _geometryModel.predictRay(frame, sensor, maze, noHitRangeM);
        return prediction.hit ? prediction.rangeM : noHitRangeM;
    }

    Eigen::Matrix<float, 2, 1> SrUkfCore::frontPairPredictionForState(
        const StateVector& sigmaPoint,
        const Maze& maze) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const float noHitRangeM = _plantModel.wallObservationNoHitRangeM();
        const SensorMount frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
        const SensorMount frontRightSensor = Vehicle::GetFrontRightSensorMount();
        const WallGeometryModel::GeometryStateFrame frame = BuildWallGeometryFrame(_geometryModel, sigmaPoint);
        const GeometryPrediction leftPrediction =
            _geometryModel.predictRay(frame, frontLeftSensor, maze, noHitRangeM);
        const GeometryPrediction rightPrediction =
            _geometryModel.predictRay(frame, frontRightSensor, maze, noHitRangeM);
        prediction(0) = leftPrediction.hit ? leftPrediction.rangeM : noHitRangeM;
        prediction(1) = rightPrediction.hit ? rightPrediction.rangeM : noHitRangeM;
        return prediction;
    }

    FrontPairUpdateResult SrUkfCore::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const Maze& maze) noexcept
    {
        FrontPairUpdateResult result{};
        const float noHitRangeM = _plantModel.wallObservationNoHitRangeM();
        const SensorMount frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
        const SensorMount frontRightSensor = Vehicle::GetFrontRightSensorMount();
        const WallGeometryModel::GeometryStateFrame frame = BuildWallGeometryFrame(_geometryModel, _workingFilter.state());
        result.leftPrediction = _geometryModel.predictRay(frame, frontLeftSensor, maze, noHitRangeM);
        result.rightPrediction = _geometryModel.predictRay(frame, frontRightSensor, maze, noHitRangeM);
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
        result.filter.accepted = _workingFilter.Update<2>(
            z,
            sqrtNoise,
            9.21034f,
            [this, &maze](const StateVector& sigmaPoint) noexcept
            {
                return frontPairPredictionForState(sigmaPoint, maze);
            });
        result.filter.nis = _workingFilter.lastNis();
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

        const float noHitRangeM = _plantModel.wallObservationNoHitRangeM();
        const SensorMount sensor = isLeft ? Vehicle::GetSideLeftSensorMount() : Vehicle::GetSideRightSensorMount();
        result.prediction = _geometryModel.predictRay(
            BuildWallGeometryFrame(_geometryModel, _workingFilter.state()),
            sensor,
            maze,
            noHitRangeM);
        result.filter.attempted = observation.valid;
        if (!result.filter.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 1, 1> sqrtNoise = _sqrtSideNoise;
        sqrtNoise(0, 0) = wallNoiseFromConfidence(observation.confidence, _sqrtSideNoise(0, 0));

        Eigen::Matrix<float, 1, 1> z;
        z << observation.rho;
        result.filter.accepted = _workingFilter.Update<1>(
            z,
            sqrtNoise,
            7.87944f,
            [this, &maze, &sensor](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction(0) = wallPredictionForSensor(sigmaPoint, sensor, maze);
                return prediction;
            });
        result.filter.nis = _workingFilter.lastNis();
        return result;
    }
}


