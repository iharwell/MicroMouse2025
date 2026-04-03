#include "pch.h"
#include "SrUkfCore.h"

#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    Eigen::Vector2f HeadingUnitFromYaw(float yaw) noexcept
    {
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(yaw, s, c);
        return Eigen::Vector2f(s, c);
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
        , _filter()
        , _lastControl()
        , _lastEncoderObs()
        , _prePredictState(StateVector::Zero())
        , _havePredictionReference(false)
        , _sqrtProcessNoiseDensity(StateMatrix::Zero())
        , _sqrtImuNoise(Eigen::Matrix<float, 3, 3>::Identity())
        , _sqrtFrontNoise(Eigen::Matrix<float, 2, 2>::Identity())
        , _sqrtSideNoise(Eigen::Matrix<float, 1, 1>::Identity())
    {
        _filter.setStateNormalizer(&VehicleState::NormalizeStateVector);

        StateVector initialState = StateVector::Zero();
        StateMatrix initialCovariance = StateMatrix::Identity() * 1.0e-3f;
        initialCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = 0.25f;
        initialCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = 0.25f;
        initialCovariance(VehicleState::kBgz, VehicleState::kBgz) = 0.01f;
        _filter.setState(initialState, initialCovariance);

        StateMatrix processNoise = StateMatrix::Zero();
        processNoise.diagonal() <<
            0.0f,
            0.0f,
            0.0f,
            3.0e-4f,
            3.0e-4f,
            6.0e-4f,
            4.0e-2f,
            4.0e-2f,
            2.0e-5f;
        _sqrtProcessNoiseDensity = StateMatrix::Zero();
        _sqrtProcessNoiseDensity.diagonal() = processNoise.diagonal().cwiseSqrt();
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _prePredictState = initialState;

        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
    }

    bool SrUkfCore::reset(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
        _lastControl = ControlInput{};
        _lastEncoderObs = EncoderObs{};
        _prePredictState = state;
        _havePredictionReference = false;
        return true;
    }

    bool SrUkfCore::setState(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
        _prePredictState = state;
        _havePredictionReference = false;
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
        _havePredictionReference = true;
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dt));
        Eigen::Matrix<float, 3, 1> controlVector;
        controlVector << control.leftMotorCommand, control.rightMotorCommand, control.fanDutyCycle;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        return _filter.Predict(
            dt,
            controlVector,
            [this, &control](const StateVector& sigmaPoint, const Eigen::Matrix<float, 3, 1>&, float sigmaDt) noexcept
            {
                return _plantModel.integrate(sigmaPoint, control, sigmaDt, _params);
            },
            invokeLoop);
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
        (void)dt;
        MeasurementUpdateResult result{};
        result.attempted = true;

        const EncoderObs measured = observation;
        _lastEncoderObs = measured;

        Eigen::Matrix<float, 2, 1> z;
        z << measured.omegaLeftRadps, measured.omegaRightRadps;
        if (HasExactZeroWheelObservation(measured))
        {
            applyWheelRateConstraint(measured, ComputeMeasuredWheelVarianceRadps2(measured, _params));
            result.accepted = true;
            result.nis = 0.0f;
            return result;
        }

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
        if (result.accepted)
        {
            applyWheelRateConstraint(measured, ComputeMeasuredWheelVarianceRadps2(measured, _params));
        }
        result.nis = _filter.lastNis();
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
        const ImuAccelObs accelObservation{
            observation.valid,
            observation.accelBodyXMps2,
            observation.accelBodyYMps2
        };
        const MeasurementUpdateResult accelResult = updatePlanarAccel(accelObservation);
        result.accepted = yawResult.accepted && accelResult.accepted;
        result.nis = accelResult.attempted ? accelResult.nis : yawResult.nis;
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

        Eigen::Matrix<float, 1, 1> z;
        z << yawRateRadps;
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) = _sqrtImuNoise(0, 0);
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
        if (result.accepted)
        {
            if (HasExactZeroWheelObservation(_lastEncoderObs) &&
                (controlCommandsAreEffectivelyZero() ||
                 (std::fabs(yawRateRadps) <= (3.0f * kImuYawRateSigmaRadps))))
            {
                applyStationaryZeroMotionConstraint(yawRateRadps);
            }
        }
        result.nis = _filter.lastNis();
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
        result.attempted =
            observation.valid &&
            std::isfinite(observation.accelBodyXMps2) &&
            std::isfinite(observation.accelBodyYMps2);
        if (!result.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 2, 1> z;
        z << observation.accelBodyXMps2, observation.accelBodyYMps2;
        Eigen::Matrix<float, 2, 2> sqrtNoise = Eigen::Matrix<float, 2, 2>::Zero();
        sqrtNoise(0, 0) = _sqrtImuNoise(1, 1);
        sqrtNoise(1, 1) = _sqrtImuNoise(2, 2);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _filter.Update<2>(
            z,
            sqrtNoise,
            9.21034f,
            [this](const StateVector& sigmaPoint) noexcept
            {
                return accelPredictionForState(sigmaPoint);
            },
            invokeLoop);
        result.nis = _filter.lastNis();
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
            (std::fabs(_lastControl.rightMotorCommand) <= 1.0e-6f);
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
        anchoredCovariance.row(VehicleState::kOmegaL).setZero();
        anchoredCovariance.col(VehicleState::kOmegaL).setZero();
        anchoredCovariance.row(VehicleState::kOmegaR).setZero();
        anchoredCovariance.col(VehicleState::kOmegaR).setZero();
        anchoredCovariance(VehicleState::kU, VehicleState::kU) =
            (std::max)(ComputeMeasuredLinearSpeedVarianceMps2(measured), 1.0e-12f);
        const float constrainedVariance = (std::max)(wheelVarianceRadps2, 1.0e-12f);
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = constrainedVariance;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = constrainedVariance;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    void SrUkfCore::applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept
    {
        StateVector anchoredState = _filter.state();
        anchoredState(VehicleState::kU) = 0.0f;
        anchoredState(VehicleState::kV) = 0.0f;
        anchoredState(VehicleState::kR) = 0.0f;
        anchoredState(VehicleState::kOmegaL) = 0.0f;
        anchoredState(VehicleState::kOmegaR) = 0.0f;
        anchoredState(VehicleState::kBgz) = yawRateRadps;
        VehicleState::NormalizeStateVector(anchoredState);

        StateMatrix anchoredCovariance = _filter.covariance();
        const float stationarySigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(_params);
        const float wheelVarianceRadps2 = (std::max)(stationarySigmaRadps * stationarySigmaRadps, 1.0e-12f);
        const float linearVarianceMps2 =
            (std::max)(kStationaryEncoderVelocitySigmaMps * kStationaryEncoderVelocitySigmaMps, 1.0e-12f);
        const float yawVarianceRadps2 =
            (std::max)(kImuYawRateSigmaRadps * kImuYawRateSigmaRadps, 1.0e-12f);
        const float gyroBiasVarianceRadps2 = (std::max)(yawVarianceRadps2, 1.0e-8f);

        const std::array<int, 5> constrainedIndices = {
            VehicleState::kU,
            VehicleState::kV,
            VehicleState::kR,
            VehicleState::kOmegaL,
            VehicleState::kOmegaR
        };
        for (const int index : constrainedIndices)
        {
            anchoredCovariance.row(index).setZero();
            anchoredCovariance.col(index).setZero();
        }
        anchoredCovariance.row(VehicleState::kBgz).setZero();
        anchoredCovariance.col(VehicleState::kBgz).setZero();

        anchoredCovariance(VehicleState::kU, VehicleState::kU) = linearVarianceMps2;
        anchoredCovariance(VehicleState::kV, VehicleState::kV) = linearVarianceMps2;
        anchoredCovariance(VehicleState::kR, VehicleState::kR) = yawVarianceRadps2;
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelVarianceRadps2;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelVarianceRadps2;
        anchoredCovariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasVarianceRadps2;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    float SrUkfCore::wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept
    {
        const float normalizedConfidence = (std::clamp)(confidence, 0.0f, 1.0f);
        return minimumNoise + ((1.0f - normalizedConfidence) * 0.020f);
    }

    Eigen::Matrix<float, 2, 1> SrUkfCore::accelPredictionForState(const StateVector& sigmaPoint) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const Eigen::Vector2f accel = _plantModel.imuPlanarAcceleration(sigmaPoint, _lastControl, _params);
        prediction << accel.x(), accel.y();
        return prediction;
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
