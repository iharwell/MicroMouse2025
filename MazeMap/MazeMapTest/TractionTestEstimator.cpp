#include "pch.h"
#include "TractionTestEstimator.h"

#include "..\MazeMap\Defines.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
    // Test-only access to the PlantModel methods that production Estimator is friended to use.
    // This keeps the harness behavior-equivalent without widening production headers.
    template <typename Tag, typename Tag::type Member>
    struct PrivatePlantMemberAccessor final
    {
        friend typename Tag::type AccessPrivatePlantMember(Tag) noexcept { return Member; }
    };

    struct PredictStateFromCommandReferenceTag final
    {
        using type = MazeMap::TractionTestEstimator::StateVector (MazeMap::PlantModel::*)(
            const MazeMap::TractionTestEstimator::StateVector&,
            const MazeMap::TractionTestEstimator::StateVector&,
            const MazeMap::App::Internal::CommandVector&,
            float,
            const SensorSnapshot::EncoderObs*) const noexcept;
        friend type AccessPrivatePlantMember(PredictStateFromCommandReferenceTag) noexcept;
    };

    struct PlantActivityForStateTag final
    {
        using type = void (MazeMap::PlantModel::*)(
            const MazeMap::TractionTestEstimator::StateVector&,
            const MazeMap::App::Internal::CommandVector&,
            const SensorSnapshot::EncoderObs*,
            float&,
            float&,
            float&,
            float&,
            float&,
            float&,
            float&) const noexcept;
        friend type AccessPrivatePlantMember(PlantActivityForStateTag) noexcept;
    };

    struct BackLeftImuPlanarAccelerationForStateTag final
    {
        using type = Eigen::Vector2f (MazeMap::PlantModel::*)(
            const MazeMap::TractionTestEstimator::StateVector&,
            const MazeMap::App::Internal::CommandVector&,
            const SensorSnapshot::EncoderObs*) const noexcept;
        friend type AccessPrivatePlantMember(BackLeftImuPlanarAccelerationForStateTag) noexcept;
    };

    struct EncoderPairCovarianceRadpsTag final
    {
        using type = Eigen::Matrix2f (MazeMap::PlantModel::*)(
            float,
            float) const noexcept;
        friend type AccessPrivatePlantMember(EncoderPairCovarianceRadpsTag) noexcept;
    };

    template struct PrivatePlantMemberAccessor<
        PredictStateFromCommandReferenceTag,
        &MazeMap::PlantModel::predictStateFromCommandReference>;
    template struct PrivatePlantMemberAccessor<
        PlantActivityForStateTag,
        &MazeMap::PlantModel::plantActivityForState>;
    template struct PrivatePlantMemberAccessor<
        BackLeftImuPlanarAccelerationForStateTag,
        &MazeMap::PlantModel::backLeftImuPlanarAccelerationForState>;
    template struct PrivatePlantMemberAccessor<
        EncoderPairCovarianceRadpsTag,
        &MazeMap::PlantModel::encoderPairCovarianceRadps>;
}

namespace MazeMap
{
    TractionTestEstimator::TractionTestEstimator(
        const Vehicle& vehicle,
        const PlantModel& plantModel,
        VehicleState& runtimeState,
        const Hooks& hooks) noexcept
        : _vehicle(vehicle)
        , _plantModel(plantModel)
        , _runtimeState(runtimeState)
        , _hooks(hooks)
        , _state(BuildRuntimeStateVector(runtimeState))
        , _workingFilter(_state, _sqrtCovariance)
    {
        static_assert(VehicleState::kDimension == 9, "TractionTestEstimator mirrors the current 9-state estimator.");
        _workingFilter.setStateNormalizer(&TractionTestEstimator::NormalizeState);
        _workingFilter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);

        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRoot();
        _workingFilter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _workingFilter.setState(_state, BuildDefaultInitialCovariance());

        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
        _runtimeState.SetCurrentCommand(_lastControl);
        _lastFanDutyCycle = _vehicle.GetFanDuty();
        _lastBatteryVoltageV = _vehicle.GetBatteryVoltage();
        PublishFilterStateToRuntime();
    }

    TractionTestEstimator::CovarianceMatrix TractionTestEstimator::BuildDefaultInitialCovariance() noexcept
    {
        CovarianceMatrix covariance = CovarianceMatrix::Identity() * 1.0e-3f;
        covariance(kPx, kPx) = 1.0e-5f;
        covariance(kPy, kPy) = 1.0e-5f;
        covariance(kDeltaAf, kDeltaAf) = 0.25f;
        covariance(kDeltaAr, kDeltaAr) = 0.25f;
        covariance(kDeltaYawAccel, kDeltaYawAccel) = 0.25f;
        return covariance;
    }

    TractionTestEstimator::StateVector TractionTestEstimator::BuildRuntimeStateVector(
        const VehicleState& runtimeState) noexcept
    {
        StateVector state = StateVector::Zero();
        state(kPx) = runtimeState.GetPositionX();
        state(kPy) = runtimeState.GetPositionY();
        state(kHeading) = runtimeState.GetHeading();
        state(kVf) = runtimeState.GetForwardVelocity();
        state(kVr) = runtimeState.GetRightwardVelocity();
        state(kYawRate) = runtimeState.GetYawRate();
        state(kDeltaAf) = runtimeState.GetForwardAccelerationResidual();
        state(kDeltaAr) = runtimeState.GetRightwardAccelerationResidual();
        state(kDeltaYawAccel) = runtimeState.GetYawAccelResidual();
        NormalizeState(state);
        return state;
    }

    void TractionTestEstimator::NormalizeState(StateVector& state) noexcept
    {
        state(kHeading) = NormalizeAngle(state(kHeading));
    }

    TractionTestEstimator::CovarianceMatrix TractionTestEstimator::BuildProcessNoiseSquareRoot() noexcept
    {
        CovarianceMatrix sqrtNoise = CovarianceMatrix::Zero();
        sqrtNoise.diagonal() <<
            0.0f,
            0.0f,
            0.0f,
            0.012f,
            0.006f,
            0.025f,
            0.0f,
            0.0f,
            0.0f;
        return sqrtNoise;
    }

    float TractionTestEstimator::DriveCommandActivityIndex(const CommandVector& control) noexcept
    {
        if (!control.IsFinite())
        {
            return 1.0f;
        }

        return
            (std::clamp)(
                (std::max)(std::fabs(control.LeftCommand()), std::fabs(control.RightCommand())),
                0.0f,
                1.0f);
    }

    bool TractionTestEstimator::Reset(
        const StateVector& state,
        const CovarianceMatrix& covariance) noexcept
    {
        StateVector resolvedState = state;
        NormalizeState(resolvedState);
        _workingFilter.setState(resolvedState, covariance);
        _workingFilter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);
        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRoot();
        _workingFilter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);

        _runtimeState.SetForwardAcceleration(0.0f);
        _runtimeState.SetRightAcceleration(0.0f);
        _runtimeState.SetYawAccel(0.0f);
        _stationaryCandidateDwellS = 0.0f;
        _stationaryCertified = false;
        _lastYawRateMeasurementRadps = 0.0f;
        _lastYawRateInnovationRadps = 0.0f;
        _lastYawRateNis = 0.0f;
        _lastForwardAccelInnovationMps2 = 0.0f;
        _lastForwardAccelNis = 0.0f;
        _lastRightAccelInnovationMps2 = 0.0f;
        _lastRightAccelNis = 0.0f;
        _lastUpdateAttempted = false;
        _lastUpdateAccepted = false;
        _lastUpdateNis = 0.0f;
        _lastControl = CommandVector(0.0f, 0.0f);
        _runtimeState.SetCurrentCommand(_lastControl);
        _lastFanDutyCycle = _vehicle.GetFanDuty();
        _lastBatteryVoltageV = _vehicle.GetBatteryVoltage();
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
        _predictionUsesEncoderInput = false;
        _predictionEncoderInput = SensorSnapshot{}.EncoderObservation();
        _predictionEncoderDtSeconds = 0.0f;
        (void)_workingFilter.floorVariance(kVf, kMinimumVelocityVariance);
        (void)_workingFilter.floorVariance(kVr, kMinimumVelocityVariance);
        (void)_workingFilter.floorVariance(kYawRate, kMinimumYawRateVariance);
        _prePredictCovariance = _workingFilter.covariance();
        PublishFilterStateToRuntime();
        return true;
    }

    bool TractionTestEstimator::ResetPose(
        const float xMeters,
        const float yMeters,
        const float headingRad) noexcept
    {
        StateVector state = StateVector::Zero();
        state(kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(kHeading) = NormalizeAngle(headingRad);
        return Reset(state, BuildDefaultInitialCovariance());
    }

    void TractionTestEstimator::ProjectMaskedStateAndSquareRootCovariance(
        const StateVector& priorState,
        const CovarianceMatrix& priorSqrtCovariance,
        const StateVector& updatedState,
        const CovarianceMatrix& updatedSqrtCovariance,
        const int* allowedIndices,
        const std::size_t allowedCount,
        StateVector& projectedState,
        CovarianceMatrix& projectedSqrtCovariance) noexcept
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

        const CovarianceMatrix priorCovariance = priorSqrtCovariance * priorSqrtCovariance.transpose();
        const CovarianceMatrix updatedCovariance = updatedSqrtCovariance * updatedSqrtCovariance.transpose();
        CovarianceMatrix projectedCovariance = priorCovariance;
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

        CovarianceMatrix fallbackSqrtCovariance = priorSqrtCovariance;
        if (UKF<VehicleState::kDimension, 3>::FactorCovariance(projectedCovariance, fallbackSqrtCovariance))
        {
            projectedSqrtCovariance = fallbackSqrtCovariance;
        }
        NormalizeState(projectedState);
    }

    TractionTestEstimator::StateVector TractionTestEstimator::CallPredictState(
        const StateVector& sigmaPoint,
        const float dtSeconds) const noexcept
    {
        const PredictionInput input{
            _prePredictState,
            sigmaPoint,
            _lastControl,
            dtSeconds,
            _predictionUsesEncoderInput ? &_predictionEncoderInput : nullptr
        };
        if (_hooks.predictState != nullptr)
        {
            return _hooks.predictState(_hooks.context, _plantModel, input);
        }

        const auto member = AccessPrivatePlantMember(PredictStateFromCommandReferenceTag{});
        return (_plantModel.*member)(
            input.prePredictState,
            input.sigmaPoint,
            input.control,
            input.dtSeconds,
            input.encoderInput);
    }

    TractionTestEstimator::PlantActivity TractionTestEstimator::CallPlantActivity(
        const StateVector& state) const noexcept
    {
        const MeasurementInput input{
            state,
            _lastControl,
            _predictionUsesEncoderInput ? &_predictionEncoderInput : nullptr
        };
        if (_hooks.plantActivity != nullptr)
        {
            return _hooks.plantActivity(_hooks.context, _plantModel, input);
        }

        PlantActivity activity{};
        const auto member = AccessPrivatePlantMember(PlantActivityForStateTag{});
        (_plantModel.*member)(
            input.sigmaPoint,
            input.control,
            input.encoderInput,
            activity.forwardAccelMps2,
            activity.rightAccelMps2,
            activity.yawAccelRadps2,
            activity.maxContactRelativeSpeedMps,
            activity.maxContactUtilization,
            activity.maxContactSaturation,
            activity.totalNormalLoadN);
        return activity;
    }

    Eigen::Vector2f TractionTestEstimator::CallBackLeftImuPlanarAcceleration(
        const StateVector& state) const noexcept
    {
        const MeasurementInput input{
            state,
            _lastControl,
            _predictionUsesEncoderInput ? &_predictionEncoderInput : nullptr
        };
        if (_hooks.backLeftImuPlanarAcceleration != nullptr)
        {
            return _hooks.backLeftImuPlanarAcceleration(_hooks.context, _plantModel, input);
        }

        const auto member = AccessPrivatePlantMember(BackLeftImuPlanarAccelerationForStateTag{});
        return (_plantModel.*member)(input.sigmaPoint, input.control, input.encoderInput);
    }

    Eigen::Matrix2f TractionTestEstimator::CallEncoderPairCovarianceRadps(
        const float linearSpeedSigmaMps,
        const float yawRateSigmaRadps) const noexcept
    {
        if (_hooks.encoderPairCovarianceRadps != nullptr)
        {
            return _hooks.encoderPairCovarianceRadps(
                _hooks.context,
                _plantModel,
                linearSpeedSigmaMps,
                yawRateSigmaRadps);
        }

        const auto member = AccessPrivatePlantMember(EncoderPairCovarianceRadpsTag{});
        return (_plantModel.*member)(linearSpeedSigmaMps, yawRateSigmaRadps);
    }

    float TractionTestEstimator::CallYawRateMeasurement(const StateVector& sigmaPoint) const noexcept
    {
        if (_hooks.yawRateMeasurement != nullptr)
        {
            return _hooks.yawRateMeasurement(_hooks.context, sigmaPoint);
        }

        return sigmaPoint(kYawRate);
    }

    TractionTestEstimator::StateVector TractionTestEstimator::PredictSigmaPoint(
        void* context,
        const StateVector& sigmaPoint,
        const Eigen::Matrix<float, 3, 1>& control,
        const float dtSeconds) noexcept
    {
        (void)control;
        TractionTestEstimator* const estimator = static_cast<TractionTestEstimator*>(context);
        if (estimator == nullptr)
        {
            return sigmaPoint;
        }

        return estimator->CallPredictState(sigmaPoint, dtSeconds);
    }

    bool TractionTestEstimator::Predict(
        const float dtSeconds,
        const CommandVector& control) noexcept
    {
        _lastControl = control;
        _runtimeState.SetCurrentCommand(_lastControl);
        _lastFanDutyCycle = _vehicle.GetFanDuty();
        _lastBatteryVoltageV = _vehicle.GetBatteryVoltage();
        if (!(std::isfinite(dtSeconds) && (dtSeconds > 0.0f)))
        {
            return true;
        }

        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        const SensorSnapshot::EncoderObs& encoderInput = snapshot.EncoderObservation();
        const bool useEncoderInput =
            snapshot.EncoderObservationValid() &&
            std::isfinite(encoderInput.LeftWheelSpeedRadps()) &&
            std::isfinite(encoderInput.RightWheelSpeedRadps());
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();

        _predictionUsesEncoderInput = useEncoderInput;
        _predictionEncoderInput = useEncoderInput ? encoderInput : SensorSnapshot{}.EncoderObservation();
        _predictionEncoderDtSeconds = useEncoderInput ? dtSeconds : 0.0f;

        const PlantActivity presentActivity = CallPlantActivity(_prePredictState);
        _runtimeState.SetForwardAcceleration(presentActivity.forwardAccelMps2);
        _runtimeState.SetRightAcceleration(presentActivity.rightAccelMps2);
        _runtimeState.SetYawAccel(presentActivity.yawAccelRadps2);
        _lastYawRateMeasurementRadps = 0.0f;
        _lastYawRateInnovationRadps = 0.0f;
        _lastYawRateNis = 0.0f;
        _lastForwardAccelInnovationMps2 = 0.0f;
        _lastForwardAccelNis = 0.0f;
        _lastRightAccelInnovationMps2 = 0.0f;
        _lastRightAccelNis = 0.0f;

        const float forwardAccelerationResidualDecayAlpha =
            PlantModel::forwardAccelerationResidualDecayAlpha(dtSeconds);
        const float rightAccelerationResidualDecayAlpha =
            PlantModel::rightAccelerationResidualDecayAlpha(dtSeconds);
        const float yawAccelerationResidualDecayAlpha =
            PlantModel::yawAccelerationResidualDecayAlpha(dtSeconds);

        CovarianceMatrix predictProcessNoiseSquareRoot =
            _sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dtSeconds);
        const float timingForwardSpeedMps =
            useEncoderInput ?
            std::fabs(_plantModel.measuredLinearSpeedMps(snapshot)) :
            std::fabs(_prePredictState(kVf));
        const float timingYawRateRadps =
            useEncoderInput ?
            std::fabs(_plantModel.measuredYawRateRadps(snapshot)) :
            std::fabs(_prePredictState(kYawRate));
        predictProcessNoiseSquareRoot(kPx, kPx) =
            (std::max)(predictProcessNoiseSquareRoot(kPx, kPx), timingForwardSpeedMps * kTimingUncertaintySeconds);
        predictProcessNoiseSquareRoot(kPy, kPy) =
            (std::max)(predictProcessNoiseSquareRoot(kPy, kPy), timingForwardSpeedMps * kTimingUncertaintySeconds);
        predictProcessNoiseSquareRoot(kHeading, kHeading) =
            (std::max)(predictProcessNoiseSquareRoot(kHeading, kHeading), timingYawRateRadps * kTimingUncertaintySeconds);
        CovarianceMatrix predictProcessNoiseCovariance =
            predictProcessNoiseSquareRoot * predictProcessNoiseSquareRoot.transpose();

        if (useEncoderInput)
        {
            Eigen::Matrix2f encoderInputCovarianceRadps2 =
                CallEncoderPairCovarianceRadps(
                    kGeneralEncoderLinearSpeedSigmaMps,
                    kGeneralEncoderYawRateSigmaRadps);
            encoderInputCovarianceRadps2 =
                0.5f * (encoderInputCovarianceRadps2 + encoderInputCovarianceRadps2.transpose());
            if (encoderInputCovarianceRadps2.allFinite() &&
                (encoderInputCovarianceRadps2(0, 0) > 0.0f) &&
                (encoderInputCovarianceRadps2(1, 1) > 0.0f))
            {
                Eigen::Matrix<float, VehicleState::kDimension, 2> encoderInputJacobian =
                    Eigen::Matrix<float, VehicleState::kDimension, 2>::Zero();
                const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();

                for (int wheelColumn = 0; wheelColumn < 2; ++wheelColumn)
                {
                    const float wheelVarianceRadps2 =
                        (std::max)(0.0f, encoderInputCovarianceRadps2(wheelColumn, wheelColumn));
                    const float deltaWheelSpeedRadps =
                        (std::max)(1.0e-3f, 0.05f * MazeMap::Math::Sqrtf(wheelVarianceRadps2));
                    SensorSnapshot::EncoderObs plusEncoderInput = encoderInput;
                    SensorSnapshot::EncoderObs minusEncoderInput = encoderInput;
                    if (wheelColumn == 0)
                    {
                        plusEncoderInput.SetLeftWheelSpeedRadps(
                            plusEncoderInput.LeftWheelSpeedRadps() + deltaWheelSpeedRadps);
                        minusEncoderInput.SetLeftWheelSpeedRadps(
                            minusEncoderInput.LeftWheelSpeedRadps() - deltaWheelSpeedRadps);
                        if (std::isfinite(wheelRadiusM) &&
                            (wheelRadiusM > 0.0f) &&
                            std::isfinite(plusEncoderInput.LeftVelocityMps()) &&
                            std::isfinite(minusEncoderInput.LeftVelocityMps()))
                        {
                            plusEncoderInput.SetLeftVelocityMps(
                                plusEncoderInput.LeftVelocityMps() + (deltaWheelSpeedRadps * wheelRadiusM));
                            minusEncoderInput.SetLeftVelocityMps(
                                minusEncoderInput.LeftVelocityMps() - (deltaWheelSpeedRadps * wheelRadiusM));
                        }
                    }
                    else
                    {
                        plusEncoderInput.SetRightWheelSpeedRadps(
                            plusEncoderInput.RightWheelSpeedRadps() + deltaWheelSpeedRadps);
                        minusEncoderInput.SetRightWheelSpeedRadps(
                            minusEncoderInput.RightWheelSpeedRadps() - deltaWheelSpeedRadps);
                        if (std::isfinite(wheelRadiusM) &&
                            (wheelRadiusM > 0.0f) &&
                            std::isfinite(plusEncoderInput.RightVelocityMps()) &&
                            std::isfinite(minusEncoderInput.RightVelocityMps()))
                        {
                            plusEncoderInput.SetRightVelocityMps(
                                plusEncoderInput.RightVelocityMps() + (deltaWheelSpeedRadps * wheelRadiusM));
                            minusEncoderInput.SetRightVelocityMps(
                                minusEncoderInput.RightVelocityMps() - (deltaWheelSpeedRadps * wheelRadiusM));
                        }
                    }

                    const bool savedUsesEncoderInput = _predictionUsesEncoderInput;
                    const SensorSnapshot::EncoderObs savedEncoderInput = _predictionEncoderInput;
                    _predictionUsesEncoderInput = true;
                    _predictionEncoderInput = plusEncoderInput;
                    StateVector plusState = CallPredictState(_prePredictState, dtSeconds);
                    _predictionEncoderInput = minusEncoderInput;
                    StateVector minusState = CallPredictState(_prePredictState, dtSeconds);
                    _predictionUsesEncoderInput = savedUsesEncoderInput;
                    _predictionEncoderInput = savedEncoderInput;

                    StateVector stateDelta = plusState - minusState;
                    stateDelta(kHeading) = NormalizeAngle(plusState(kHeading) - minusState(kHeading));
                    encoderInputJacobian.col(wheelColumn) = stateDelta * (0.5f / deltaWheelSpeedRadps);
                }

                const CovarianceMatrix encoderInputProcessCovariance =
                    encoderInputJacobian *
                    encoderInputCovarianceRadps2 *
                    encoderInputJacobian.transpose();
                if (encoderInputProcessCovariance.allFinite())
                {
                    predictProcessNoiseCovariance += encoderInputProcessCovariance;
                }
            }
        }

        const float absForwardSpeedMps = std::fabs(_prePredictState(kVf));
        const float absYawRateRadps = std::fabs(_prePredictState(kYawRate));
        const float halfTrackWidthM =
            0.5f * (std::max)(0.0f, std::fabs(_vehicle.GetTrackWidth()));
        const float yawSurfaceSpeedMps = halfTrackWidthM * absYawRateRadps;
        const float contactRelativeSpeedReferenceMps =
            (std::max)(kContactSpeedScheduleReferenceMps, absForwardSpeedMps + yawSurfaceSpeedMps);
        const float contactRelativeSpeedUtilization =
            (std::clamp)(
                presentActivity.maxContactRelativeSpeedMps / contactRelativeSpeedReferenceMps,
                0.0f,
                1.0f);
        const float yawInducedContactSpeedFraction =
            (std::clamp)(
                yawSurfaceSpeedMps /
                    (yawSurfaceSpeedMps + absForwardSpeedMps + kContactSpeedScheduleReferenceMps),
                0.0f,
                1.0f);
        const float utilization =
            (std::clamp)(presentActivity.maxContactUtilization, 0.0f, 1.0f);
        const float driveSaturationIndex = DriveCommandActivityIndex(control);
        const float encoderFaultIndicator = useEncoderInput ? 0.0f : 1.0f;
        const float sigmaDeltaAfSs =
            kResidualForwardBaseSigmaSsMps2 +
            (kResidualForwardUtilSigmaSsMps2 * utilization * utilization) +
            (0.010f * driveSaturationIndex * driveSaturationIndex) +
            (kResidualEncoderFaultSigmaSs * encoderFaultIndicator);
        const float sigmaDeltaArSs =
            kResidualRightBaseSigmaSsMps2 +
            (kResidualRightUtilSigmaSsMps2 * utilization * utilization) +
            (kResidualRightContactRelativeSpeedSigmaSsMps2 * contactRelativeSpeedUtilization) +
            (kResidualYawInducedContactSpeedSigmaSs * yawInducedContactSpeedFraction) +
            (kResidualEncoderFaultSigmaSs * encoderFaultIndicator);
        const float sigmaDeltaYawAccelSs =
            kResidualYawAccelBaseSigmaSsRadps2 +
            (kResidualYawAccelUtilSigmaSsRadps2 * utilization * utilization) +
            (kResidualYawAccelContactRelativeSpeedSigmaSsRadps2 * contactRelativeSpeedUtilization) +
            (kResidualYawInducedContactSpeedSigmaSs * yawInducedContactSpeedFraction) +
            (kResidualEncoderFaultSigmaSs * encoderFaultIndicator);
        const float qDeltaAf =
            sigmaDeltaAfSs * sigmaDeltaAfSs *
            (1.0f - (forwardAccelerationResidualDecayAlpha * forwardAccelerationResidualDecayAlpha));
        const float qDeltaAr =
            sigmaDeltaArSs * sigmaDeltaArSs *
            (1.0f - (rightAccelerationResidualDecayAlpha * rightAccelerationResidualDecayAlpha));
        const float qDeltaYawAccel =
            sigmaDeltaYawAccelSs * sigmaDeltaYawAccelSs *
            (1.0f - (yawAccelerationResidualDecayAlpha * yawAccelerationResidualDecayAlpha));
        Eigen::Matrix<float, VehicleState::kDimension, 3> residualSensitivity =
            Eigen::Matrix<float, VehicleState::kDimension, 3>::Zero();
        const float headingForResidualMap = NormalizeAngle(_prePredictState(kHeading));
        float residualMapSin = 0.0f;
        float residualMapCos = 0.0f;
        sin_cosf(headingForResidualMap, residualMapSin, residualMapCos);
        const float dt2 = dtSeconds * dtSeconds;
        const float dt3 = dt2 * dtSeconds;
        residualSensitivity(kPx, 0) = residualMapSin * dt2;
        residualSensitivity(kPy, 0) = residualMapCos * dt2;
        residualSensitivity(kVf, 0) = dtSeconds;
        residualSensitivity(kDeltaAf, 0) = 1.0f;
        residualSensitivity(kPx, 1) = residualMapCos * dt2;
        residualSensitivity(kPy, 1) = -residualMapSin * dt2;
        residualSensitivity(kVr, 1) = dtSeconds;
        residualSensitivity(kDeltaAr, 1) = 1.0f;
        residualSensitivity(kPx, 2) =
            ((-_prePredictState(kVr) * residualMapSin) +
             (_prePredictState(kVf) * residualMapCos)) * dt3;
        residualSensitivity(kPy, 2) =
            ((-_prePredictState(kVr) * residualMapCos) -
             (_prePredictState(kVf) * residualMapSin)) * dt3;
        residualSensitivity(kHeading, 2) = dt2;
        residualSensitivity(kYawRate, 2) = dtSeconds;
        residualSensitivity(kDeltaYawAccel, 2) = 1.0f;
        Eigen::Matrix3f residualInnovationCovariance = Eigen::Matrix3f::Zero();
        residualInnovationCovariance(0, 0) = (std::max)(0.0f, qDeltaAf);
        residualInnovationCovariance(1, 1) = (std::max)(0.0f, qDeltaAr);
        residualInnovationCovariance(2, 2) = (std::max)(0.0f, qDeltaYawAccel);
        predictProcessNoiseCovariance +=
            residualSensitivity *
            residualInnovationCovariance *
            residualSensitivity.transpose();
        CovarianceMatrix mappedProcessNoiseSquareRoot = predictProcessNoiseSquareRoot;
        if (UKF<VehicleState::kDimension, 3>::FactorCovariance(
                predictProcessNoiseCovariance,
                mappedProcessNoiseSquareRoot))
        {
            predictProcessNoiseSquareRoot = mappedProcessNoiseSquareRoot;
        }
        _workingFilter.setProcessNoiseSquareRoot(predictProcessNoiseSquareRoot);

        Eigen::Matrix<float, 3, 1> filterCommandVector;
        filterCommandVector << control.LeftCommand(), control.RightCommand(), _lastFanDutyCycle;
        const bool predicted = _workingFilter.Predict(
            dtSeconds,
            filterCommandVector,
            this,
            &TractionTestEstimator::PredictSigmaPoint);
        if (predicted)
        {
            (void)_workingFilter.floorVariance(kVf, kMinimumVelocityVariance);
            (void)_workingFilter.floorVariance(kVr, kMinimumVelocityVariance);
            (void)_workingFilter.floorVariance(kYawRate, kMinimumYawRateVariance);
            PublishFilterStateToRuntime();
        }
        return predicted;
    }

    float TractionTestEstimator::YawRateMeasurementVarianceRadps2(
        const float yawRateMeasurementRadps) const noexcept
    {
        const float sigmaRadps = _sqrtImuNoise(0, 0);
        float varianceRadps2 = 0.0f;
        if (std::isfinite(sigmaRadps) && (sigmaRadps > 0.0f))
        {
            varianceRadps2 = sigmaRadps * sigmaRadps;
        }
        else
        {
            varianceRadps2 = kImuYawRateVarianceRadps2;
        }

        const float gyroScaleRadpsPerLsb =
            _vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() * 0.001f * DEG_TO_RAD_F;
        if (std::isfinite(gyroScaleRadpsPerLsb) && (gyroScaleRadpsPerLsb > 0.0f))
        {
            varianceRadps2 += (gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f;
        }

        if (std::isfinite(yawRateMeasurementRadps))
        {
            const float gyroScaleToleranceSigmaRadps =
                std::fabs(yawRateMeasurementRadps) *
                kImuGyroSensitivityToleranceFraction /
                MazeMap::Math::Sqrtf(3.0f);
            varianceRadps2 += gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps;
        }

        return varianceRadps2;
    }

    Eigen::Matrix<float, 1, 1> TractionTestEstimator::YawRateMeasurementForState(
        void* context,
        const StateVector& sigmaPoint) noexcept
    {
        TractionTestEstimator* const estimator = static_cast<TractionTestEstimator*>(context);
        Eigen::Matrix<float, 1, 1> prediction;
        prediction << ((estimator != nullptr) ? estimator->CallYawRateMeasurement(sigmaPoint) : 0.0f);
        return prediction;
    }

    bool TractionTestEstimator::UpdateYawRate(const float yawRateRadps) noexcept
    {
        _lastUpdateAttempted = std::isfinite(yawRateRadps);
        _lastUpdateAccepted = false;
        _lastUpdateNis = 0.0f;
        if (!_lastUpdateAttempted)
        {
            return false;
        }

        const StateVector priorState = _workingFilter.state();
        const CovarianceMatrix priorSqrtCovariance = _workingFilter.sqrtCovariance();
        _lastYawRateMeasurementRadps = yawRateRadps;
        _lastYawRateInnovationRadps = yawRateRadps - CallYawRateMeasurement(priorState);

        Eigen::Matrix<float, 1, 1> measurement;
        measurement << yawRateRadps;
        float yawRateMeasurementVariance = YawRateMeasurementVarianceRadps2(yawRateRadps);
        const float gyroFilterCutoffHz = Vehicle::GetBackLeftImuRuntimeGyroLpfCutoffHz();
        if (std::isfinite(gyroFilterCutoffHz) && (gyroFilterCutoffHz > 0.0f))
        {
            const float yawAccelRadps2 =
                std::isfinite(_runtimeState.GetYawAccel()) ?
                _runtimeState.GetYawAccel() :
                0.0f;
            const float filterPhaseDelaySeconds = 1.0f / (2.0f * PI_F * gyroFilterCutoffHz);
            const float filterPhaseSigmaRadps = std::fabs(yawAccelRadps2) * filterPhaseDelaySeconds;
            yawRateMeasurementVariance += filterPhaseSigmaRadps * filterPhaseSigmaRadps;
        }
        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        std::uint32_t imuTimingDeltaUs = 0U;
        if ((snapshot.ImuTiming().readDoneUs > snapshot.ImuTiming().drdyUs) &&
            ((snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().drdyUs) <= kMaxCredibleImuTimingDeltaUs))
        {
            imuTimingDeltaUs = snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().drdyUs;
        }
        if ((snapshot.ImuTiming().readDoneUs > snapshot.ImuTiming().readStartUs) &&
            ((snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().readStartUs) <= kMaxCredibleImuTimingDeltaUs))
        {
            imuTimingDeltaUs =
                (std::max)(imuTimingDeltaUs, snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().readStartUs);
        }
        if (imuTimingDeltaUs > 0U)
        {
            const float imuTimingSeconds =
                (std::max)(
                    kTimingUncertaintySeconds,
                    1.0e-6f * static_cast<float>(imuTimingDeltaUs));
            const float yawAccelRadps2 =
                std::isfinite(_runtimeState.GetYawAccel()) ?
                _runtimeState.GetYawAccel() :
                0.0f;
            const float timingSigmaRadps = std::fabs(yawAccelRadps2) * imuTimingSeconds;
            yawRateMeasurementVariance += timingSigmaRadps * timingSigmaRadps;
        }
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, yawRateMeasurementVariance));
        _lastUpdateAccepted = _workingFilter.Update<1>(
            measurement,
            sqrtNoise,
            std::numeric_limits<float>::infinity(),
            this,
            &TractionTestEstimator::YawRateMeasurementForState);
        _lastUpdateNis = _workingFilter.lastNis();
        _lastYawRateNis = _lastUpdateNis;
        if (_lastUpdateAccepted)
        {
            constexpr std::array<int, 1> kAllowedYawRateIndex = { {
                kYawRate
            } };
            StateVector projectedState = priorState;
            CovarianceMatrix projectedSqrtCovariance = priorSqrtCovariance;
            ProjectMaskedStateAndSquareRootCovariance(
                priorState,
                priorSqrtCovariance,
                _workingFilter.state(),
                _workingFilter.sqrtCovariance(),
                kAllowedYawRateIndex.data(),
                kAllowedYawRateIndex.size(),
                projectedState,
                projectedSqrtCovariance);
            _workingFilter.setStateSquareRootCovariance(projectedState, projectedSqrtCovariance);

            updateStationaryCertification(yawRateRadps);
            PublishFilterStateToRuntime();
        }
        return _lastUpdateAccepted;
    }

    Eigen::Matrix<float, 1, 1> TractionTestEstimator::ForwardAccelMeasurementForState(
        void* context,
        const StateVector& sigmaPoint) noexcept
    {
        TractionTestEstimator* const estimator = static_cast<TractionTestEstimator*>(context);
        Eigen::Matrix<float, 1, 1> prediction;
        prediction << ((estimator != nullptr) ?
            estimator->CallBackLeftImuPlanarAcceleration(sigmaPoint)(1) :
            0.0f);
        return prediction;
    }

    Eigen::Matrix<float, 1, 1> TractionTestEstimator::RightAccelMeasurementForState(
        void* context,
        const StateVector& sigmaPoint) noexcept
    {
        TractionTestEstimator* const estimator = static_cast<TractionTestEstimator*>(context);
        Eigen::Matrix<float, 1, 1> prediction;
        prediction << ((estimator != nullptr) ?
            estimator->CallBackLeftImuPlanarAcceleration(sigmaPoint)(0) :
            0.0f);
        return prediction;
    }

    bool TractionTestEstimator::UpdatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        _lastUpdateAttempted = observation.IsValid();
        _lastUpdateAccepted = false;
        _lastUpdateNis = 0.0f;
        if (!_lastUpdateAttempted)
        {
            return false;
        }

        Eigen::Matrix<float, 1, 1> forwardMeasurement;
        forwardMeasurement << observation.AccelBodyForwardMps2();
        const float accelSensorVariance = _sqrtImuNoise(2, 2) * _sqrtImuNoise(2, 2);
        float rFilterVariance = 0.0f;
        float rImpactVariance = 0.0f;
        const PlantActivity measurementActivity = CallPlantActivity(_workingFilter.state());
        const float absForwardSpeedMps = std::fabs(_workingFilter.state()(kVf));
        const float absYawRateRadps = std::fabs(_workingFilter.state()(kYawRate));
        const float yawInducedContactSpeedMps =
            0.5f * (std::max)(0.0f, std::fabs(_vehicle.GetTrackWidth())) * absYawRateRadps;
        const float contactRelativeSpeedReferenceMps =
            (std::max)(
                kContactSpeedScheduleReferenceMps,
                absForwardSpeedMps + yawInducedContactSpeedMps);
        const float contactRelativeSpeedUtilization =
            (std::clamp)(
                measurementActivity.maxContactRelativeSpeedMps / contactRelativeSpeedReferenceMps,
                0.0f,
                1.0f);
        const float forceLimitActivity =
            (std::clamp)(measurementActivity.maxContactUtilization - 1.0f, 0.0f, 1.0f);
        const float nominalGroundLoadN =
            (std::max)(0.0f, _vehicle.GetMass() * GRAVITY_MPS2);
        const float groundUse =
            (nominalGroundLoadN > 0.0f) ?
            (std::clamp)(1.0f - (measurementActivity.totalNormalLoadN / nominalGroundLoadN), 0.0f, 1.0f) :
            0.0f;
        const float driveSaturation =
            (std::clamp)(
                (std::max)(DriveCommandActivityIndex(_lastControl), measurementActivity.maxContactSaturation),
                0.0f,
                1.0f);
        const float yawRateRadps =
            std::isfinite(_workingFilter.state()(kYawRate)) ?
            _workingFilter.state()(kYawRate) :
            0.0f;
        const float yawAccelRadps2 =
            std::isfinite(measurementActivity.yawAccelRadps2) ?
            measurementActivity.yawAccelRadps2 :
            (std::isfinite(_runtimeState.GetYawAccel()) ? _runtimeState.GetYawAccel() : 0.0f);
        const float forwardAccelMps2 =
            std::isfinite(measurementActivity.forwardAccelMps2) ?
            measurementActivity.forwardAccelMps2 :
            (std::isfinite(_runtimeState.GetForwardAcceleration()) ?
                _runtimeState.GetForwardAcceleration() :
                0.0f);
        const float rightAccelMps2 =
            std::isfinite(measurementActivity.rightAccelMps2) ?
            measurementActivity.rightAccelMps2 :
            (std::isfinite(_runtimeState.GetRightAcceleration()) ?
                _runtimeState.GetRightAcceleration() :
                0.0f);
        const float speedMps =
            MazeMap::Math::Sqrtf(
                (_workingFilter.state()(kVf) * _workingFilter.state()(kVf)) +
                (_workingFilter.state()(kVr) * _workingFilter.state()(kVr)));
        const Eigen::Vector2f imuLeverArmBodyM = Vehicle::GetBackLeftImuMount().positionBodyM();
        const float imuLeverArmM =
            MazeMap::Math::Sqrtf(
                (imuLeverArmBodyM.x() * imuLeverArmBodyM.x()) +
                (imuLeverArmBodyM.y() * imuLeverArmBodyM.y()));
        const float planarAccelMps2 =
            MazeMap::Math::Sqrtf(
                (forwardAccelMps2 * forwardAccelMps2) +
                (rightAccelMps2 * rightAccelMps2));
        const float accelFiniteJerkMps3 =
            (std::fabs(yawRateRadps) * planarAccelMps2) +
            (std::fabs(yawAccelRadps2) * speedMps) +
            (2.0f * std::fabs(yawRateRadps) * std::fabs(yawAccelRadps2) * imuLeverArmM);
        rImpactVariance +=
            (kAccelContactRelativeSpeedSigmaMps2 * contactRelativeSpeedUtilization) *
            (kAccelContactRelativeSpeedSigmaMps2 * contactRelativeSpeedUtilization);
        rImpactVariance +=
            (kAccelForceLimitSigmaMps2 * forceLimitActivity) *
            (kAccelForceLimitSigmaMps2 * forceLimitActivity);
        rImpactVariance +=
            (kAccelGroundUseSigmaMps2 * groundUse) *
            (kAccelGroundUseSigmaMps2 * groundUse);
        rImpactVariance +=
            (kAccelSaturationSigmaMps2 * driveSaturation) *
            (kAccelSaturationSigmaMps2 * driveSaturation);
        const float impactEventActivity =
            (std::clamp)(
                (std::max)(
                    (std::max)(contactRelativeSpeedUtilization, forceLimitActivity),
                    (std::max)(groundUse, driveSaturation)),
                0.0f,
                1.0f);
        const float impactEventDtSeconds =
            (std::isfinite(_predictionEncoderDtSeconds) && (_predictionEncoderDtSeconds > 0.0f)) ?
            (std::max)(_predictionEncoderDtSeconds, kTimingUncertaintySeconds) :
            kTimingUncertaintySeconds;
        const float impactEventSigmaMps2 =
            accelFiniteJerkMps3 * impactEventDtSeconds * impactEventActivity;
        rImpactVariance += impactEventSigmaMps2 * impactEventSigmaMps2;
        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        std::uint32_t imuTimingDeltaUs = 0U;
        if ((snapshot.ImuTiming().readDoneUs > snapshot.ImuTiming().drdyUs) &&
            ((snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().drdyUs) <= kMaxCredibleImuTimingDeltaUs))
        {
            imuTimingDeltaUs = snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().drdyUs;
        }
        if ((snapshot.ImuTiming().readDoneUs > snapshot.ImuTiming().readStartUs) &&
            ((snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().readStartUs) <= kMaxCredibleImuTimingDeltaUs))
        {
            imuTimingDeltaUs =
                (std::max)(imuTimingDeltaUs, snapshot.ImuTiming().readDoneUs - snapshot.ImuTiming().readStartUs);
        }
        if (imuTimingDeltaUs > 0U)
        {
            const float imuTimingSeconds =
                (std::max)(
                    kTimingUncertaintySeconds,
                    1.0e-6f * static_cast<float>(imuTimingDeltaUs));
            const float timingSigmaMps2 = accelFiniteJerkMps3 * imuTimingSeconds;
            rFilterVariance += timingSigmaMps2 * timingSigmaMps2;
        }
        const bool imuTelemetryPresent =
            snapshot.AccelerationBiasValid() &&
            ((snapshot.BackLeftImuTelemetry().status != 0U) ||
             (snapshot.ImuTiming().drdyUs != 0U) ||
             (snapshot.ImuTiming().readStartUs != 0U) ||
             (snapshot.ImuTiming().readDoneUs != 0U));
        if (imuTelemetryPresent && ((snapshot.BackLeftImuTelemetry().status & kImuStatusAccelDataReadyMask) == 0U))
        {
            const float staleSigmaMps2 = accelFiniteJerkMps3 * impactEventDtSeconds;
            rFilterVariance += staleSigmaMps2 * staleSigmaMps2;
        }
        if ((snapshot.BackLeftImuTelemetry().status & kImuStatusTimestampOverflowMask) != 0U)
        {
            const float overflowSigmaMps2 =
                accelFiniteJerkMps3 *
                (1.0e-6f * static_cast<float>(kMaxCredibleImuTimingDeltaUs));
            rFilterVariance += overflowSigmaMps2 * overflowSigmaMps2;
        }
        const float accelMeasurementVariance =
            accelSensorVariance +
            (std::max)(0.0f, rFilterVariance) +
            (std::max)(0.0f, rImpactVariance);
        _lastForwardAccelInnovationMps2 =
            observation.AccelBodyForwardMps2() -
            CallBackLeftImuPlanarAcceleration(_workingFilter.state())(1);
        Eigen::Matrix<float, 1, 1> forwardNoise;
        forwardNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, accelMeasurementVariance));
        const bool forwardAccepted = _workingFilter.Update<1>(
            forwardMeasurement,
            forwardNoise,
            25.0f,
            this,
            &TractionTestEstimator::ForwardAccelMeasurementForState);
        _lastForwardAccelNis = _workingFilter.lastNis();

        Eigen::Matrix<float, 1, 1> rightMeasurement;
        rightMeasurement << observation.AccelBodyRightMps2();
        _lastRightAccelInnovationMps2 =
            observation.AccelBodyRightMps2() -
            CallBackLeftImuPlanarAcceleration(_workingFilter.state())(0);
        Eigen::Matrix<float, 1, 1> rightNoise;
        rightNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, accelMeasurementVariance));
        const bool rightAccepted = _workingFilter.Update<1>(
            rightMeasurement,
            rightNoise,
            25.0f,
            this,
            &TractionTestEstimator::RightAccelMeasurementForState);
        _lastRightAccelNis = _workingFilter.lastNis();

        _lastUpdateAccepted = forwardAccepted || rightAccepted;
        _lastUpdateNis = forwardAccepted ? _lastForwardAccelNis : _lastRightAccelNis;
        if (_lastUpdateAccepted)
        {
            PublishFilterStateToRuntime();
        }
        return _lastUpdateAccepted;
    }

    bool TractionTestEstimator::controlCommandsAreEffectivelyZero() const noexcept
    {
        if (!_lastControl.IsFinite())
        {
            return true;
        }

        return
            (std::fabs(_lastControl.LeftCommand()) <= kStationaryCandidateMaxDriveCommand) &&
            (std::fabs(_lastControl.RightCommand()) <= kStationaryCandidateMaxDriveCommand);
    }

    bool TractionTestEstimator::isStationaryCandidate(const float yawRateRadps) const noexcept
    {
        return
            _predictionUsesEncoderInput &&
            controlCommandsAreEffectivelyZero() &&
            std::isfinite(_predictionEncoderInput.LeftWheelSpeedRadps()) &&
            std::fabs(_predictionEncoderInput.LeftWheelSpeedRadps()) < kStationaryCandidateMaxEncoderWheelSpeedRadps &&
            std::isfinite(_predictionEncoderInput.RightWheelSpeedRadps()) &&
            std::fabs(_predictionEncoderInput.RightWheelSpeedRadps()) < kStationaryCandidateMaxEncoderWheelSpeedRadps &&
            std::isfinite(yawRateRadps) &&
            std::fabs(yawRateRadps) < kStationaryCandidateMaxYawRateRadps;
    }

    void TractionTestEstimator::updateStationaryCertification(const float yawRateRadps) noexcept
    {
        if (isStationaryCandidate(yawRateRadps))
        {
            _stationaryCandidateDwellS +=
                (std::isfinite(_predictionEncoderDtSeconds) && (_predictionEncoderDtSeconds > 0.0f)) ?
                _predictionEncoderDtSeconds :
                0.0f;
        }
        else
        {
            _stationaryCandidateDwellS = 0.0f;
        }
        _stationaryCertified = _stationaryCandidateDwellS >= kStationaryCertificationDwellS;
    }

    void TractionTestEstimator::PublishFilterStateToRuntime() noexcept
    {
        const StateVector& state = _workingFilter.state();
        _runtimeState.SetPosition(Eigen::Vector2f(state(kPx), state(kPy)));
        _runtimeState.SetHeading(state(kHeading));
        _runtimeState.SetForwardVelocity(state(kVf));
        _runtimeState.SetRightwardVelocity(state(kVr));
        _runtimeState.SetYawRate(state(kYawRate));
        _runtimeState.SetForwardAccelerationResidual(state(kDeltaAf));
        _runtimeState.SetRightwardAccelerationResidual(state(kDeltaAr));
        _runtimeState.SetYawAccelResidual(state(kDeltaYawAccel));
    }
}
