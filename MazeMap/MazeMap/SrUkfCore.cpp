#include "pch.h"
#include "SrUkfCore.h"

#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
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

    constexpr float kStationaryCertificationDwellS = 0.080f;
    constexpr float kStationaryCandidateMaxLinearCommandMps = 0.03f;
    constexpr float kStationaryCandidateMaxAngularCommandRadps = 0.15f;
    constexpr float kStationaryCandidateMaxDriveCommand = 0.08f;
    constexpr float kStationaryCandidateMaxEncoderOmegaRadps = 1.0f;
    constexpr float kStationaryCandidateMaxCorrectedGyroRadps = 0.06f;
    constexpr float kStationaryCandidateMaxAccelMps2 = 0.6f;

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

    constexpr float kMovingGyroBiasStdCapRadps = 0.020f;
    constexpr float kRecoveryYawRateStdFloorRadps = 0.030f;
    constexpr float kYawValidityBiasDeltaMaxRadps = 0.02f;

    struct ModeProcessNoiseConfig
    {
        float sigmaUSqrtQ = 0.0f;
        float sigmaVSqrtQ = 0.0f;
        float sigmaRSqrtQ = 0.0f;
        float sigmaOmegaSqrtQ = 0.0f;
        float sigmaBgzSqrtQ = 0.0f;
        float stdRMin = 0.0f;
        float stdVMin = 0.0f;
    };

    constexpr ModeProcessNoiseConfig kStationaryCertifiedProcessNoise{
        0.006f,
        0.000f,
        0.010f,
        0.050f,
        0.00010f,
        0.010f,
        0.0f
    };

    constexpr ModeProcessNoiseConfig kLaunchOrReversalProcessNoise{
        0.020f,
        0.000f,
        0.050f,
        0.250f,
        0.000001f,
        0.030f,
        0.006f
    };

    constexpr ModeProcessNoiseConfig kGripLinearProcessNoise{
        0.012f,
        0.000f,
        0.025f,
        0.100f,
        0.000001f,
        0.020f,
        0.006f
    };

    constexpr ModeProcessNoiseConfig kInconsistentOrSaturatedProcessNoise{
        0.030f,
        0.002f,
        0.070f,
        0.350f,
        0.000001f,
        0.050f,
        0.020f
    };

    const ModeProcessNoiseConfig& GetModeProcessNoiseConfig(const MazeMap::SrUkfCore::OperatingMode mode) noexcept
    {
        switch (mode)
        {
        case MazeMap::SrUkfCore::OperatingMode::StationaryCertified:
            return kStationaryCertifiedProcessNoise;
        case MazeMap::SrUkfCore::OperatingMode::LaunchOrReversalTransient:
            return kLaunchOrReversalProcessNoise;
        case MazeMap::SrUkfCore::OperatingMode::InconsistentOrSaturated:
            return kInconsistentOrSaturatedProcessNoise;
        case MazeMap::SrUkfCore::OperatingMode::GripLinear:
        default:
            return kGripLinearProcessNoise;
        }
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
    SrUkfCore::SrUkfCore(
        const PlantParams& params,
        const PlantModel& plantModel) noexcept
        : _plantModel(plantModel)
        , _geometryModel()
        , _params(params)
        , _preparedParams(PlantModel::Prepare(_params))
        , _filter()
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
        , _gyroBiasAnchorVarianceRadps2(0.01f)
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
        , _yawWindowDtSeconds{}
        , _yawWindowUkfIntegralRad{}
        , _yawWindowGyroIntegralRad{}
        , _yawWindowHead(0U)
        , _yawWindowSize(0U)
        , _yawWindowSpanS(0.0f)
    {
        _filter.setStateNormalizer(&VehicleState::NormalizeStateVector);

        StateVector initialState = StateVector::Zero();
        const StateMatrix initialCovariance = BuildDefaultInitialCovariance();
        _filter.setState(initialState, initialCovariance);

        _gyroBiasAnchorRadps = initialState(VehicleState::kBgz);
        _gyroBiasAnchorVarianceRadps2 = initialCovariance(VehicleState::kBgz, VehicleState::kBgz);
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _prePredictState = initialState;
        _prePredictCovariance = _filter.covariance();

        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
        synchronizeGyroBiasStateToAnchor(true, kMovingGyroBiasStdCapRadps, GetModeProcessNoiseConfig(_operatingMode).stdRMin);
        enforceVarianceFloors(_operatingMode);
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

    float SrUkfCore::ComputeNonholonomicSigmaMps(float absForwardSpeedMps) noexcept
    {
        const float resolvedForwardSpeedMps =
            (std::isfinite(absForwardSpeedMps) && (absForwardSpeedMps > 0.0f)) ?
            absForwardSpeedMps :
            0.0f;
        const float sigmaMps =
            MazeMap::Math::Sqrtf(
                (0.005f * 0.005f) +
                ((0.05f * resolvedForwardSpeedMps) * (0.05f * resolvedForwardSpeedMps)));
        return (std::clamp)(sigmaMps, 0.005f, 0.040f);
    }

    bool SrUkfCore::IsStationaryCandidate(
        const ControlInput& control,
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
            std::isfinite(control.leftMotorCommand) &&
            std::fabs(control.leftMotorCommand) < kStationaryCandidateMaxDriveCommand &&
            std::isfinite(control.rightMotorCommand) &&
            std::fabs(control.rightMotorCommand) < kStationaryCandidateMaxDriveCommand &&
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
            (std::fabs(bgzRadps - gyroBiasAnchorRadps) >= kYawValidityBiasDeltaMaxRadps))
        {
            return false;
        }
        if (!std::isfinite(yawConsistencyLowPassRadps) ||
            (yawConsistencyLowPassRadps >= kYawConsistencyLowPassThresholdRadps))
        {
            return false;
        }
        if (nhcEnabled)
        {
            if (!(std::isfinite(nhcSigmaMps) && (nhcSigmaMps > 0.0f)) ||
                !std::isfinite(lateralVelocityMps) ||
                (std::fabs(lateralVelocityMps) >= (kNhcResidualTripSigma * nhcSigmaMps)))
            {
                return false;
            }
        }
        return true;
    }

    void SrUkfCore::ProjectMaskedStateAndCovariance(
        const StateVector& priorState,
        const StateMatrix& priorCovariance,
        const StateVector& updatedState,
        const StateMatrix& updatedCovariance,
        const int* allowedIndices,
        std::size_t allowedCount,
        StateVector& projectedState,
        StateMatrix& projectedCovariance) noexcept
    {
        projectedState = priorState;
        projectedCovariance = priorCovariance;
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

        if ((std::fabs(commandedLinearMps) > kPivotScrubMaxCommandLinearMps) ||
            (std::fabs(commandedAngularRadps) < kPivotScrubMinCommandAngularRadps))
        {
            return false;
        }

        const float measuredLinearSpeedMps =
            std::fabs(ComputeMeasuredLinearSpeedMps(observation, params));
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
                "left_motor_command=%.9g;right_motor_command=%.9g;fan_duty_cycle=%.9g;battery_voltage_v=%.9g",
                static_cast<double>(_lastControl.leftMotorCommand),
                static_cast<double>(_lastControl.rightMotorCommand),
                static_cast<double>(_lastControl.fanDutyCycle),
                static_cast<double>(_lastControl.batteryVoltageV)))
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

        auto emitSensorExtrinsics =
            [&](const char* type, const SensorExtrinsics& sensor) noexcept
            {
                return EmitDebugTextLine(
                    context,
                    sink,
                    type,
                    "position_x_m=%.9g;position_y_m=%.9g;direction_x=%.9g;direction_y=%.9g;yaw_offset_rad=%.9g",
                    static_cast<double>(sensor.positionBodyM.x()),
                    static_cast<double>(sensor.positionBodyM.y()),
                    static_cast<double>(sensor.directionBody.x()),
                    static_cast<double>(sensor.directionBody.y()),
                    static_cast<double>(sensor.yawOffsetRad));
            };
        if (!emitSensorExtrinsics("ukf_dump_sensor_front_left", _params.frontLeftSensor) ||
            !emitSensorExtrinsics("ukf_dump_sensor_front_right", _params.frontRightSensor) ||
            !emitSensorExtrinsics("ukf_dump_sensor_side_left", _params.sideLeftSensor) ||
            !emitSensorExtrinsics("ukf_dump_sensor_side_right", _params.sideRightSensor))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_imu_extrinsics",
                "position_x_m=%.9g;position_y_m=%.9g;accel_body_from_imu_00=%.9g;accel_body_from_imu_01=%.9g;accel_body_from_imu_10=%.9g;accel_body_from_imu_11=%.9g;gyro_z_sign=%.9g",
                static_cast<double>(_params.imu.positionBodyM.x()),
                static_cast<double>(_params.imu.positionBodyM.y()),
                static_cast<double>(_params.imu.accelBodyFromImu(0, 0)),
                static_cast<double>(_params.imu.accelBodyFromImu(0, 1)),
                static_cast<double>(_params.imu.accelBodyFromImu(1, 0)),
                static_cast<double>(_params.imu.accelBodyFromImu(1, 1)),
                static_cast<double>(_params.imu.gyroZSign)))
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

        return EmitDebugTextLine(
            context,
            sink,
            "ukf_dump_consistency",
            "gyro_bias_anchor_radps=%.9g;yaw_consistency_lp_radps=%.9g;yaw_window_mismatch_rad=%.9g;nhc_sigma_mps=%.9g;nhc_residual_mps=%.9g;nhc_residual_sigma=%.9g",
            static_cast<double>(_gyroBiasAnchorRadps),
            static_cast<double>(_yawConsistencyLowPassRadps),
            static_cast<double>(_yawWindowMismatchRad),
            static_cast<double>(_nhcSigmaMps),
            static_cast<double>(_nhcResidualMps),
            static_cast<double>(_nhcResidualSigma));
    }

    SrUkfCore::StateMatrix SrUkfCore::BuildDefaultInitialCovariance() noexcept
    {
        StateMatrix covariance = StateMatrix::Identity() * 1.0e-3f;
        covariance(VehicleState::kPx, VehicleState::kPx) = 1.0e-5f;
        covariance(VehicleState::kPy, VehicleState::kPy) = 1.0e-5f;
        covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = 0.25f;
        covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = 0.25f;
        covariance(VehicleState::kBgz, VehicleState::kBgz) = 0.01f;
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
            config.sigmaBgzSqrtQ;
        return sqrtNoise;
    }

    bool SrUkfCore::reset(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
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
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _lastControl = ControlInput{};
        _lastEncoderObs = EncoderObs{};
        _lastEncoderDtSeconds = 0.0f;
        _prePredictState = _filter.state();
        _prePredictCovariance = _filter.covariance();
        _havePredictionReference = false;
        _acceptedEncoderUpdateSincePredict = false;
        synchronizeGyroBiasStateToAnchor(true, kMovingGyroBiasStdCapRadps, GetModeProcessNoiseConfig(_operatingMode).stdRMin);
        enforceVarianceFloors(_operatingMode);
        return true;
    }

    bool SrUkfCore::setState(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
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
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRootForMode(_operatingMode);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _lastEncoderDtSeconds = 0.0f;
        _prePredictState = _filter.state();
        _prePredictCovariance = _filter.covariance();
        _havePredictionReference = false;
        _acceptedEncoderUpdateSincePredict = false;
        synchronizeGyroBiasStateToAnchor(true, kMovingGyroBiasStdCapRadps, GetModeProcessNoiseConfig(_operatingMode).stdRMin);
        enforceVarianceFloors(_operatingMode);
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
        const float varianceUMps2 = kGeneralEncoderLinearSpeedSigmaMps * kGeneralEncoderLinearSpeedSigmaMps;
        const float varianceYawRateRadps2 = kGeneralEncoderYawRateSigmaRadps * kGeneralEncoderYawRateSigmaRadps;
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

        return kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;
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

        return kEncoderPairNisThreshold;
    }

    bool SrUkfCore::predict(float dt, const ControlInput& control) noexcept
    {
        return predictImpl(dt, control, nullptr, nullptr);
    }

    bool SrUkfCore::predictImpl(
        float dt,
        const ControlInput& control,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        _lastControl = control;
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return true;
        }

        _prePredictState = _filter.state();
        _prePredictCovariance = _filter.covariance();
        _havePredictionReference = true;
        _acceptedEncoderUpdateSincePredict = false;
        _lastEncoderDtSeconds = 0.0f;
        resetPivotScrubTelemetry();
        updateCommandSignFlipWindow(dt);
        updateOperatingMode(dt);
        updateProcessNoiseForMode();
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dt));
        Eigen::Matrix<float, 3, 1> controlVector;
        controlVector << control.leftMotorCommand, control.rightMotorCommand, control.fanDutyCycle;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        const bool predicted = _filter.Predict(
            dt,
            controlVector,
            [this, &control](const StateVector& sigmaPoint, const Eigen::Matrix<float, 3, 1>&, float sigmaDt) noexcept
            {
                return _plantModel.integrate(sigmaPoint, control, sigmaDt, _preparedParams);
            },
            invokeLoop);
        if (predicted)
        {
            synchronizeGyroBiasStateToAnchor(
                true,
                kMovingGyroBiasStdCapRadps,
                GetModeProcessNoiseConfig(_operatingMode).stdRMin);
            enforceVarianceFloors(_operatingMode);
        }
        return predicted;
    }

    MeasurementUpdateResult SrUkfCore::updateEncoderPair(const EncoderObs& observation, float dt) noexcept
    {
        return updateEncoderPairImpl(observation, dt, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updateEncoderPairImpl(
        const EncoderObs& observation,
        float dt,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = true;

        const EncoderObs measured = observation;
        const StateVector priorState = _filter.state();
        const StateMatrix priorCovariance = _filter.covariance();
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
            _lastEncoderObs = measured;
            _acceptedEncoderUpdateSincePredict = true;
            if (_pivotScrubMode)
            {
                applyPivotScrubEncoderWheelConstraint(
                    priorState,
                    priorCovariance,
                    measured,
                    ComputeMeasuredWheelVarianceRadps2(measured, _params));
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
                applyWheelRateConstraint(measured, ComputeMeasuredWheelVarianceRadps2(measured, _params));
            }
        }
        else
        {
            _lastEncoderObs = measured;
            _acceptedEncoderUpdateSincePredict = true;
            applyWheelSpeedConstraint(measured, ComputeMeasuredWheelVarianceRadps2(measured, _params));
            result.accepted = true;
        }
        if (_pivotScrubMode)
        {
            const StateVector zeroUPriorState = _filter.state();
            const StateMatrix zeroUPriorCovariance = _filter.covariance();
            Eigen::Matrix<float, 1, 1> zeroUObservation;
            zeroUObservation << 0.0f;
            Eigen::Matrix<float, 1, 1> zeroUSqrtNoise;
            zeroUSqrtNoise(0, 0) = kPivotScrubZeroUSigmaMps;
            const bool zeroUAccepted = _filter.Update<1>(
                zeroUObservation,
                zeroUSqrtNoise,
                std::numeric_limits<float>::infinity(),
                [](const StateVector& sigmaPoint) noexcept
                {
                    Eigen::Matrix<float, 1, 1> prediction;
                    prediction << sigmaPoint(VehicleState::kU);
                    return prediction;
                },
                invokeLoop);
            if (zeroUAccepted)
            {
                applyPivotScrubZeroUConstraint(
                    zeroUPriorState,
                    zeroUPriorCovariance,
                    kPivotScrubZeroUSigmaMps);
            }
            _pivotScrubZeroUSoftApplied = zeroUAccepted;
            const StateVector& zeroUPostState = _filter.state();
            _pivotScrubZeroUInnovationMps = std::isfinite(zeroUPriorState(VehicleState::kU)) ?
                -zeroUPriorState(VehicleState::kU) :
                0.0f;
            _pivotScrubZeroUDeltaMps = zeroUPostState(VehicleState::kU) - zeroUPriorState(VehicleState::kU);
        }
        result.nis = encoderMeasurementNis;
        return result;
    }

    MeasurementUpdateResult SrUkfCore::updateImuMerged(const ImuMergedObs& observation) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = observation.valid;
        if (!observation.valid)
        {
            return result;
        }

        const MeasurementUpdateResult yawResult = updateYawRate(observation.gyroZRadps);
        result.accepted = yawResult.accepted;
        result.nis = yawResult.nis;
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
        const StateMatrix priorCovariance = _filter.covariance();
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
            [this](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction << sigmaPoint(VehicleState::kR) + _gyroBiasAnchorRadps;
                return prediction;
            },
            invokeLoop);
        const float yawMeasurementNis = _filter.lastNis();
        if (result.accepted)
        {
            if (!_pivotScrubMode)
            {
                const StateVector updatedState = _filter.state();
                const StateMatrix updatedCovariance = _filter.covariance();
                StateVector projectedState = priorState;
                StateMatrix projectedCovariance = priorCovariance;
                constexpr std::array<int, 6> kAllowedIndices = {
                    VehicleState::kPx,
                    VehicleState::kPy,
                    VehicleState::kPsi,
                    VehicleState::kU,
                    VehicleState::kV,
                    VehicleState::kR
                };

                // Preserve the encoder-owned wheel-rate states across the gyro update.
                ProjectMaskedStateAndCovariance(
                    priorState,
                    priorCovariance,
                    updatedState,
                    updatedCovariance,
                    kAllowedIndices.data(),
                    kAllowedIndices.size(),
                    projectedState,
                    projectedCovariance);
                _filter.setState(projectedState, projectedCovariance);
            }
            updateInitialStationaryGyroBias(yawRateRadps, shouldApplyStationaryConstraint);
            if (_pivotScrubMode)
            {
                applyPivotScrubGyroConstraint(priorState, priorCovariance, yawRateRadps);
                const StateVector& postState = _filter.state();
                _pivotScrubGyroDeltaPsiRad = postState(VehicleState::kPsi) - priorState(VehicleState::kPsi);
                _pivotScrubGyroDeltaRRadps = postState(VehicleState::kR) - priorState(VehicleState::kR);
                _pivotScrubGyroDeltaBgzRadps = postState(VehicleState::kBgz) - priorState(VehicleState::kBgz);
                _pivotScrubGyroDeltaOmegaLRadps = postState(VehicleState::kOmegaL) - priorState(VehicleState::kOmegaL);
                _pivotScrubGyroDeltaOmegaRRadps = postState(VehicleState::kOmegaR) - priorState(VehicleState::kOmegaR);
                _pivotScrubGyroMaskedDeltaNorm = MazeMap::Math::Sqrtf(
                    (_pivotScrubGyroDeltaPsiRad * _pivotScrubGyroDeltaPsiRad) +
                    (_pivotScrubGyroDeltaRRadps * _pivotScrubGyroDeltaRRadps) +
                    (_pivotScrubGyroDeltaBgzRadps * _pivotScrubGyroDeltaBgzRadps));
            }
            updateStationaryCertification(yawRateRadps);
            updateYawConsistencyMetrics(yawRateRadps);
            const OperatingMode previousMode = _operatingMode;
            updateOperatingMode(_lastEncoderDtSeconds);
            if (shouldApplyStationaryConstraint &&
                _stationaryCertified)
            {
                applyStationaryZeroMotionConstraint(yawRateRadps);
            }
            else if (_pivotScrubMode)
            {
                updateNonholonomicDiagnostics(false);
            }
            else if (_acceptedEncoderUpdateSincePredict && !HasExactZeroWheelObservation(_lastEncoderObs))
            {
                (void)applyGripLateralVelocityConstraint(loopHookContext, loopHook);
            }
            else
            {
                updateNonholonomicDiagnostics(false);
            }
            updateOperatingMode(0.0f);
            sanitizeLaunchRecoveryIfNeeded(previousMode, _operatingMode);
            synchronizeGyroBiasStateToAnchor(
                _operatingMode != OperatingMode::StationaryCertified,
                kMovingGyroBiasStdCapRadps,
                GetModeProcessNoiseConfig(_operatingMode).stdRMin);
            enforceVarianceFloors(_operatingMode);
            _yawValidForFeedforward = IsYawValidForFeedforward(
                _operatingMode,
                _filter.state()(VehicleState::kBgz),
                _gyroBiasAnchorRadps,
                _yawConsistencyLowPassRadps,
                _nonholonomicConstraintEnabled,
                _filter.state()(VehicleState::kV),
                _nhcSigmaMps);
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
        (void)observation;
        (void)loopHookContext;
        (void)loopHook;

        // Intentionally disabled while the second IMU is offline. With only one IMU online,
        // this planar-accel update feeds yaw-related state through the lever-arm model and has
        // produced edge-case estimator failures. Do not reintroduce this path unless the
        // multi-IMU measurement model is restored and revalidated end-to-end.
        MeasurementUpdateResult result{};
        result.accepted = true;
        return result;
    }

    bool SrUkfCore::HasExactZeroWheelObservation(const EncoderObs& observation) noexcept
    {
        return (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
    }

    float SrUkfCore::ComputeDistancePerEncoderCountM(const PlantParams& params) noexcept
    {
        const float encoderCountsPerWheelRev = params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev);
        if (!(params.wheelRadiusM > 0.0f) ||
            !std::isfinite(params.wheelRadiusM) ||
            !(encoderCountsPerWheelRev > 0.0f) ||
            !std::isfinite(encoderCountsPerWheelRev))
        {
            return 0.0f;
        }

        return (2.0f * PI_F * params.wheelRadiusM) / encoderCountsPerWheelRev;
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
            kStationaryEncoderVelocitySigmaMps :
            kGeneralEncoderLinearSpeedSigmaMps;
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
            (std::fabs(_lastControl.leftMotorCommand) <= 1.0e-6f) &&
            (std::fabs(_lastControl.rightMotorCommand) <= 1.0e-6f) &&
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

    void SrUkfCore::anchorPoseToEncoderDelta(
        StateVector& anchoredState,
        const EncoderObs& measured) const noexcept
    {
        if (!_havePredictionReference)
        {
            return;
        }

        const float distancePerCountM = ComputeDistancePerEncoderCountM(_params);
        const float trackWidthM = _params.trackWidthM;
        if (!(distancePerCountM > 0.0f) ||
            !std::isfinite(distancePerCountM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return;
        }

        const float leftDistanceM = static_cast<float>(measured.totalLeftCounts) * distancePerCountM;
        const float rightDistanceM = static_cast<float>(measured.totalRightCounts) * distancePerCountM;
        const float forwardDistanceM = 0.5f * (leftDistanceM + rightDistanceM);
        const float deltaYawRad = (leftDistanceM - rightDistanceM) / trackWidthM;
        const float referenceYawRad = _prePredictState(VehicleState::kPsi);
        const float translationYawRad = VehicleState::NormalizeAngle(referenceYawRad + (0.5f * deltaYawRad));
        const Eigen::Vector2f heading = HeadingUnitFromYaw(translationYawRad);

        anchoredState(VehicleState::kPx) = _prePredictState(VehicleState::kPx) + (forwardDistanceM * heading.x());
        anchoredState(VehicleState::kPy) = _prePredictState(VehicleState::kPy) + (forwardDistanceM * heading.y());
        anchoredState(VehicleState::kPsi) = VehicleState::NormalizeAngle(referenceYawRad + deltaYawRad);
    }

    void SrUkfCore::applyWheelRateConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept
    {
        StateVector anchoredState = _filter.state();
        anchorPoseToEncoderDelta(anchoredState, measured);
        anchoredState(VehicleState::kU) = ComputeMeasuredLinearSpeedMps(measured, _params);
        anchoredState(VehicleState::kOmegaL) = measured.omegaLeftRadps;
        anchoredState(VehicleState::kOmegaR) = measured.omegaRightRadps;
        VehicleState::NormalizeStateVector(anchoredState);

        StateMatrix anchoredCovariance = _filter.covariance();
        anchoredCovariance.row(VehicleState::kU).setZero();
        anchoredCovariance.col(VehicleState::kU).setZero();
        anchoredCovariance.row(VehicleState::kR).setZero();
        anchoredCovariance.col(VehicleState::kR).setZero();
        anchoredCovariance.row(VehicleState::kOmegaL).setZero();
        anchoredCovariance.col(VehicleState::kOmegaL).setZero();
        anchoredCovariance.row(VehicleState::kOmegaR).setZero();
        anchoredCovariance.col(VehicleState::kOmegaR).setZero();
        anchoredState(VehicleState::kR) = ComputeMeasuredYawRateRadps(measured, _params);
        anchoredCovariance(VehicleState::kU, VehicleState::kU) =
            (std::max)(ComputeMeasuredLinearSpeedVarianceMps2(measured), 1.0e-12f);
        anchoredCovariance(VehicleState::kR, VehicleState::kR) =
            (std::max)(ComputeMeasuredYawRateVarianceRadps2(measured, _params), 1.0e-12f);
        const float constrainedVariance = (std::max)(wheelVarianceRadps2, 1.0e-12f);
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = constrainedVariance;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = constrainedVariance;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    void SrUkfCore::applyPivotScrubEncoderWheelConstraint(
        const StateVector& priorState,
        const StateMatrix& priorCovariance,
        const EncoderObs& measured,
        float wheelVarianceRadps2) noexcept
    {
        (void)measured;
        (void)wheelVarianceRadps2;
        const StateVector updatedState = _filter.state();
        const StateMatrix updatedCovariance = _filter.covariance();
        StateVector projectedState = priorState;
        StateMatrix projectedCovariance = priorCovariance;
        constexpr std::array<int, 2> kAllowedIndices = {
            VehicleState::kOmegaL,
            VehicleState::kOmegaR
        };
        ProjectMaskedStateAndCovariance(
            priorState,
            priorCovariance,
            updatedState,
            updatedCovariance,
            kAllowedIndices.data(),
            kAllowedIndices.size(),
            projectedState,
            projectedCovariance);
        _filter.setState(projectedState, projectedCovariance);
    }

    void SrUkfCore::applyPivotScrubZeroUConstraint(
        const StateVector& priorState,
        const StateMatrix& priorCovariance,
        float sigmaMps) noexcept
    {
        (void)sigmaMps;
        const StateVector updatedState = _filter.state();
        const StateMatrix updatedCovariance = _filter.covariance();
        StateVector projectedState = priorState;
        StateMatrix projectedCovariance = priorCovariance;
        constexpr std::array<int, 1> kAllowedIndices = {
            VehicleState::kU
        };
        ProjectMaskedStateAndCovariance(
            priorState,
            priorCovariance,
            updatedState,
            updatedCovariance,
            kAllowedIndices.data(),
            kAllowedIndices.size(),
            projectedState,
            projectedCovariance);
        _filter.setState(projectedState, projectedCovariance);
    }

    void SrUkfCore::applyPivotScrubGyroConstraint(
        const StateVector& priorState,
        const StateMatrix& priorCovariance,
        float yawRateRadps) noexcept
    {
        (void)yawRateRadps;
        const StateVector updatedState = _filter.state();
        const StateMatrix updatedCovariance = _filter.covariance();
        StateVector projectedState = priorState;
        StateMatrix projectedCovariance = priorCovariance;
        constexpr std::array<int, 3> kAllowedIndices = {
            VehicleState::kPsi,
            VehicleState::kR,
            VehicleState::kBgz
        };
        ProjectMaskedStateAndCovariance(
            priorState,
            priorCovariance,
            updatedState,
            updatedCovariance,
            kAllowedIndices.data(),
            kAllowedIndices.size(),
            projectedState,
            projectedCovariance);
        _filter.setState(projectedState, projectedCovariance);
    }

    void SrUkfCore::applyWheelSpeedConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept
    {
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
        VehicleState constrainedState;
        constrainedState.SetStateVector(_filter.state());
        constrainedState.SetCovariance(_filter.covariance());
        constrainedState.ApplyStationaryZeroMotionConstraint(
            _lastEncoderObs,
            true,
            _havePredictionReference,
            _prePredictState,
            _prePredictCovariance,
            ComputeDistancePerEncoderCountM(_params),
            _params.trackWidthM);
        constrainedState.SetGyroBiasZ(_gyroBiasAnchorRadps);
        constrainedState.SetGyroBiasZVar(
            _gyroBiasAnchorVarianceRadps2);
        _filter.setStateSquareRootCovariance(
            constrainedState.GetStateVector(),
            constrainedState.GetSqrtCovariance());
        updateNonholonomicDiagnostics(false);
    }

    void SrUkfCore::updateCommandSignFlipWindow(float dtSeconds) noexcept
    {
        const float averageDriveCommand = 0.5f * (_lastControl.leftMotorCommand + _lastControl.rightMotorCommand);
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
            _stationaryCandidateDwellS += _lastEncoderDtSeconds;
        }
        else
        {
            _stationaryCandidateDwellS = 0.0f;
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

    void SrUkfCore::updateYawConsistencyMetrics(float yawRateRadps) noexcept
    {
        const float correctedGyroRadps = correctedYawRateRadps(yawRateRadps);
        const float yawResidualRadps = std::fabs(_filter.state()(VehicleState::kR) - correctedGyroRadps);
        const float alpha =
            (std::isfinite(_lastEncoderDtSeconds) && (_lastEncoderDtSeconds > 0.0f)) ?
            (std::clamp)(_lastEncoderDtSeconds / (kYawConsistencyLowPassTauS + _lastEncoderDtSeconds), 0.0f, 1.0f) :
            1.0f;
        _yawConsistencyLowPassRadps += alpha * (yawResidualRadps - _yawConsistencyLowPassRadps);
        pushYawWindowContribution(_lastEncoderDtSeconds, _filter.state()(VehicleState::kR), correctedGyroRadps);
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
            _filter.state()(VehicleState::kU),
            _lastControl.leftMotorCommand,
            _lastControl.rightMotorCommand,
            _leftLaunchAssistFloor,
            _rightLaunchAssistFloor,
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
        const float forwardSpeedMps = std::fabs(_filter.state()(VehicleState::kU));
        const float driveDelta = std::fabs(_lastControl.leftMotorCommand - _lastControl.rightMotorCommand);
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
        return std::isfinite(yawRateRawRadps) ? (yawRateRawRadps - _gyroBiasAnchorRadps) : 0.0f;
    }

    void SrUkfCore::ZeroGyroBiasDynamicCrossCovariances(StateMatrix& covariance) noexcept
    {
        constexpr std::array<int, 6> kDynamicIndices = {
            VehicleState::kPsi,
            VehicleState::kU,
            VehicleState::kV,
            VehicleState::kR,
            VehicleState::kOmegaL,
            VehicleState::kOmegaR
        };
        for (const int index : kDynamicIndices)
        {
            covariance(VehicleState::kBgz, index) = 0.0f;
            covariance(index, VehicleState::kBgz) = 0.0f;
        }
    }

    float SrUkfCore::ComputeStationaryGyroBiasBlendFactor(float dtSeconds) noexcept
    {
        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)) ||
            !(std::isfinite(kStationaryGyroBiasTimeConstantS) && (kStationaryGyroBiasTimeConstantS > 0.0f)))
        {
            return 0.0f;
        }
        return (std::clamp)(
            1.0f - std::exp(-dtSeconds / kStationaryGyroBiasTimeConstantS),
            0.0f,
            1.0f);
    }

    float SrUkfCore::ComputeStationaryGyroBiasWalkProcessVarianceRadps2(
        float dtSeconds,
        float measurementVarianceRadps2) noexcept
    {
        if (!(std::isfinite(measurementVarianceRadps2) && (measurementVarianceRadps2 > 0.0f)))
        {
            return 0.0f;
        }

        const float alpha = ComputeStationaryGyroBiasBlendFactor(dtSeconds);
        if (!(alpha > 0.0f) || !(alpha < 1.0f))
        {
            return 0.0f;
        }

        // Exact discrete random-walk variance that yields the target exponential averaging gain.
        return ((alpha * alpha) / (1.0f - alpha)) * measurementVarianceRadps2;
    }

    float SrUkfCore::ComputeStationaryGyroBiasWalkPosteriorVarianceRadps2(
        float dtSeconds,
        float measurementVarianceRadps2) noexcept
    {
        if (!(std::isfinite(measurementVarianceRadps2) && (measurementVarianceRadps2 > 0.0f)))
        {
            return 0.0f;
        }

        return ComputeStationaryGyroBiasBlendFactor(dtSeconds) * measurementVarianceRadps2;
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

        const float measurementVarianceRadps2 = kImuYawRateSigmaRadps * kImuYawRateSigmaRadps;
        if (!_initialStationaryGyroBiasSeedApplied)
        {
            if ((_initialStationaryGyroBiasSampleOrdinal >= kInitialStationaryGyroBiasSeedEndSample) &&
                (_initialStationaryGyroBiasCollectedSeedSamples > 0U))
            {
                _gyroBiasAnchorRadps = static_cast<float>(
                    _initialStationaryGyroBiasSeedAccumRadps /
                    static_cast<double>(_initialStationaryGyroBiasCollectedSeedSamples));
                _gyroBiasAnchorVarianceRadps2 =
                    ComputeStationaryGyroBiasWalkPosteriorVarianceRadps2(
                        _lastEncoderDtSeconds,
                        measurementVarianceRadps2);
                if (!std::isfinite(_gyroBiasAnchorVarianceRadps2) ||
                    !(_gyroBiasAnchorVarianceRadps2 > 0.0f))
                {
                    _gyroBiasAnchorVarianceRadps2 = 1.0e-8f;
                }
                _initialStationaryGyroBiasSeedApplied = true;
                _biasUpdateEnabled = true;
            }
            return;
        }

        const float priorVarianceRadps2 =
            (std::isfinite(_gyroBiasAnchorVarianceRadps2) && (_gyroBiasAnchorVarianceRadps2 > 0.0f)) ?
            _gyroBiasAnchorVarianceRadps2 :
            1.0e-8f;
        const float predictedVarianceRadps2 =
            priorVarianceRadps2 +
            ComputeStationaryGyroBiasWalkProcessVarianceRadps2(
                _lastEncoderDtSeconds,
                measurementVarianceRadps2);
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
            _gyroBiasAnchorVarianceRadps2 = 1.0e-8f;
        }
        _biasUpdateEnabled = true;
    }

    void SrUkfCore::synchronizeGyroBiasStateToAnchor(
        bool zeroDynamicCrossCovariances,
        float maxGyroBiasStdRadps,
        float minimumYawRateStdRadps) noexcept
    {
        StateVector state = _filter.state();
        StateMatrix covariance = _filter.covariance();
        state(VehicleState::kBgz) = _gyroBiasAnchorRadps;
        if (zeroDynamicCrossCovariances)
        {
            ZeroGyroBiasDynamicCrossCovariances(covariance);
        }
        const float cappedBiasVarianceRadps2 =
            (std::max)(maxGyroBiasStdRadps, 1.0e-4f) * (std::max)(maxGyroBiasStdRadps, 1.0e-4f);
        if (!(std::isfinite(_gyroBiasAnchorVarianceRadps2) && (_gyroBiasAnchorVarianceRadps2 > 0.0f)))
        {
            _gyroBiasAnchorVarianceRadps2 = 1.0e-8f;
        }
        _gyroBiasAnchorVarianceRadps2 = (std::min)(_gyroBiasAnchorVarianceRadps2, cappedBiasVarianceRadps2);

        // Preserve the internal stationary-walk variance exactly; only the published UKF state
        // covariance keeps the numeric floor used elsewhere in the estimator.
        covariance(VehicleState::kBgz, VehicleState::kBgz) =
            (std::max)(_gyroBiasAnchorVarianceRadps2, 1.0e-8f);
        covariance(VehicleState::kR, VehicleState::kR) =
            (std::max)(
                covariance(VehicleState::kR, VehicleState::kR),
                (std::max)(minimumYawRateStdRadps, 1.0e-4f) * (std::max)(minimumYawRateStdRadps, 1.0e-4f));
        _filter.setState(state, covariance);
    }

    void SrUkfCore::enforceVarianceFloors(OperatingMode mode) noexcept
    {
        const ModeProcessNoiseConfig& config = GetModeProcessNoiseConfig(mode);
        StateMatrix covariance = _filter.covariance();
        covariance(VehicleState::kR, VehicleState::kR) =
            (std::max)(covariance(VehicleState::kR, VehicleState::kR), config.stdRMin * config.stdRMin);
        covariance(VehicleState::kV, VehicleState::kV) =
            (std::max)(covariance(VehicleState::kV, VehicleState::kV), config.stdVMin * config.stdVMin);
        covariance(VehicleState::kBgz, VehicleState::kBgz) =
            (std::max)(covariance(VehicleState::kBgz, VehicleState::kBgz), 1.0e-8f);
        _filter.setState(_filter.state(), covariance);
    }

    void SrUkfCore::sanitizeLaunchRecoveryIfNeeded(OperatingMode previousMode, OperatingMode newMode) noexcept
    {
        if ((previousMode != OperatingMode::LaunchOrReversalTransient) ||
            (newMode == OperatingMode::LaunchOrReversalTransient) ||
            (_saturationFlags != 0U))
        {
            return;
        }

        StateVector state = _filter.state();
        StateMatrix covariance = _filter.covariance();
        state(VehicleState::kBgz) = _gyroBiasAnchorRadps;
        ZeroGyroBiasDynamicCrossCovariances(covariance);
        covariance(VehicleState::kBgz, VehicleState::kBgz) = _gyroBiasAnchorVarianceRadps2;
        covariance(VehicleState::kR, VehicleState::kR) =
            (std::max)(
                covariance(VehicleState::kR, VehicleState::kR),
                kRecoveryYawRateStdFloorRadps * kRecoveryYawRateStdFloorRadps);
        _filter.setState(state, covariance);
        _nhcReenableDelayRemainingS = kRecoveryNhcReenableDelayS;
    }

    float SrUkfCore::wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept
    {
        const float normalizedConfidence = (std::clamp)(confidence, 0.0f, 1.0f);
        return minimumNoise + ((1.0f - normalizedConfidence) * 0.020f);
    }

    float SrUkfCore::wallPredictionForSensor(
        const StateVector& sigmaPoint,
        const SensorExtrinsics& sensor,
        const LocalMapView& map) const noexcept
    {
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint, map);
        const GeometryPrediction prediction = _geometryModel.predictRay(frame, sensor, map);
        return prediction.hit ? prediction.rangeM : map.noHitRangeM;
    }

    Eigen::Matrix<float, 2, 1> SrUkfCore::frontPairPredictionForState(
        const StateVector& sigmaPoint,
        const LocalMapView& map) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint, map);
        const GeometryPrediction leftPrediction = _geometryModel.predictRay(frame, _params.frontLeftSensor, map);
        const GeometryPrediction rightPrediction = _geometryModel.predictRay(frame, _params.frontRightSensor, map);
        prediction(0) = leftPrediction.hit ? leftPrediction.rangeM : map.noHitRangeM;
        prediction(1) = rightPrediction.hit ? rightPrediction.rangeM : map.noHitRangeM;
        return prediction;
    }

    FrontPairUpdateResult SrUkfCore::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const LocalMapView& map) noexcept
    {
        FrontPairUpdateResult result{};
        result.leftPrediction = _geometryModel.predictRay(_filter.state(), _params.frontLeftSensor, map);
        result.rightPrediction = _geometryModel.predictRay(_filter.state(), _params.frontRightSensor, map);
        result.filter.attempted = left.valid && right.valid && map.IsValid();
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
            [this, &map](const StateVector& sigmaPoint) noexcept
            {
                return frontPairPredictionForState(sigmaPoint, map);
            });
        result.filter.nis = _filter.lastNis();
        return result;
    }

    WallUpdateResult SrUkfCore::updateSideSensor(
        Side which,
        const WallObs& observation,
        const LocalMapView& map) noexcept
    {
        WallUpdateResult result{};
        const SensorExtrinsics& sensor =
            (which == Side::Left) ? _params.sideLeftSensor : _params.sideRightSensor;
        result.prediction = _geometryModel.predictRay(_filter.state(), sensor, map);
        result.filter.attempted = observation.valid && map.IsValid();
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
            [this, &map, &sensor](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction(0) = wallPredictionForSensor(sigmaPoint, sensor, map);
                return prediction;
            });
        result.filter.nis = _filter.lastNis();
        return result;
    }
}
