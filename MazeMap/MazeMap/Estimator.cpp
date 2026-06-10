#include "pch.h"
#include "Estimator.h"

#include "Defines.h"
#include "SensorSnapshot.h"
#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <limits>

namespace MazeMap
{
    float Estimator::YawRateMeasurementVarianceRadps2(float yawRateMeasurementRadps) const noexcept
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

    bool Estimator::EmitDebugTextLine(
        void* context,
        bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept,
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

    Estimator::Estimator(
        const Vehicle& vehicle,
        const PlantModel& plantModel,
        VehicleState& runtimeState) noexcept
        : _plantModel(plantModel)
        , _vehicle(vehicle)
        , _runtimeState(runtimeState)
        , _workingFilter(runtimeState._state, runtimeState._sqrtCovariance)
    {
        _workingFilter.setStateNormalizer(&Estimator::NormalizeState);
        _workingFilter.setSigmaPointStrategy(UKF<VehicleState::kDimension, 3>::SigmaPointStrategy::Simplex);

        _sqrtProcessNoiseDensity = BuildProcessNoiseSquareRoot();
        _workingFilter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);

        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
        _runtimeState.SetCurrentCommand(_lastControl);
    }

    Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> Estimator::BuildDefaultInitialCovariance() noexcept
    {
        return VehicleState::DefaultInitialCovariance();
    }

    void Estimator::NormalizeState(Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept
    {
        state(VehicleState::kHeading) = NormalizeAngle(state(VehicleState::kHeading));
    }

    WallGeometryModel::GeometryStateFrame Estimator::BuildWallGeometryFrame(
        float pX,
        float pY,
        const Eigen::Vector2f& headingUnit) noexcept
    {
        WallGeometryModel::GeometryStateFrame frame{};
        frame.positionWorldM = Eigen::Vector2f(pX, pY);
        frame.heading = headingUnit;
        frame.centerCell = WallGeometryModel::WorldToCell(pX, pY);
        return frame;
    }

    float Estimator::DriveCommandActivityIndex(const App::Internal::CommandVector& control) noexcept
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

    Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> Estimator::BuildProcessNoiseSquareRoot() noexcept
    {
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> sqrtNoise = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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

    bool Estimator::reset(const Eigen::Matrix<float, VehicleState::kDimension, 1>& state, const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& covariance) noexcept
    {
        Eigen::Matrix<float, VehicleState::kDimension, 1> resolvedState = state;
        resolvedState(VehicleState::kHeading) = NormalizeAngle(resolvedState(VehicleState::kHeading));
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
        _lastSideWallPrediction = GeometryPrediction{};
        _lastFrontLeftWallPrediction = GeometryPrediction{};
        _lastFrontRightWallPrediction = GeometryPrediction{};
        _lastControl = App::Internal::CommandVector(0.0f, 0.0f);
        _runtimeState.SetCurrentCommand(_lastControl);
        _lastFanDutyCycle = runtimeFanDuty();
        _lastBatteryVoltageV = runtimeBatteryVoltage();
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
        _havePredictionReference = false;
        _predictionUsesEncoderInput = false;
        _predictionEncoderInput = SensorSnapshot{}.EncoderObservation();
        _predictionEncoderDtSeconds = 0.0f;
        (void)_workingFilter.floorVariance(VehicleState::kVf, kMinimumVelocityVariance);
        (void)_workingFilter.floorVariance(VehicleState::kVr, kMinimumVelocityVariance);
        (void)_workingFilter.floorVariance(VehicleState::kYawRate, kMinimumYawRateVariance);
        _prePredictCovariance = _workingFilter.covariance();
        return true;
    }

    void Estimator::ProjectMaskedStateAndSquareRootCovariance(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& priorState,
        const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& priorSqrtCovariance,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& updatedState,
        const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& updatedSqrtCovariance,
        const int* allowedIndices,
        std::size_t allowedCount,
        Eigen::Matrix<float, VehicleState::kDimension, 1>& projectedState,
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& projectedSqrtCovariance) noexcept
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

        const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> priorCovariance = priorSqrtCovariance * priorSqrtCovariance.transpose();
        const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> updatedCovariance = updatedSqrtCovariance * updatedSqrtCovariance.transpose();
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> projectedCovariance = priorCovariance;
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

        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> fallbackSqrtCovariance = priorSqrtCovariance;
        if (UKF<VehicleState::kDimension, 3>::FactorCovariance(projectedCovariance, fallbackSqrtCovariance))
        {
            projectedSqrtCovariance = fallbackSqrtCovariance;
        }
        projectedState(VehicleState::kHeading) = NormalizeAngle(projectedState(VehicleState::kHeading));
    }

    bool Estimator::WriteDebugTextDump(void* context, bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept) const noexcept
    {
        if (sink == nullptr)
        {
            return false;
        }

        for (std::size_t index = 0U; index < kEstimatorStateFieldNames.size(); ++index)
        {
            if (!EmitDebugTextLine(
                    context,
                    sink,
                    "estimator_dump_state",
                    "%s=%.9g",
                    kEstimatorStateFieldNames[index],
                    static_cast<double>(_workingFilter.state()(static_cast<int>(index)))))
            {
                return false;
            }
        }

        const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = _workingFilter.covariance();
        for (int row = 0; row < VehicleState::kDimension; ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "estimator_dump_covariance_row",
                    kEstimatorStateFieldNames,
                    covariance,
                    row))
            {
                return false;
            }
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "estimator_dump_prediction_reference",
                "have_prediction_reference=%s",
                _havePredictionReference ? "true" : "false"))
        {
            return false;
        }

        for (std::size_t index = 0U; index < kEstimatorStateFieldNames.size(); ++index)
        {
            if (!EmitDebugTextLine(
                    context,
                    sink,
                    "estimator_dump_pre_predict_state",
                    "%s=%.9g",
                    kEstimatorStateFieldNames[index],
                    static_cast<double>(_prePredictState(static_cast<int>(index)))))
            {
                return false;
            }
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "estimator_dump_last_control",
                "left_motor_pwm=%.9g;right_motor_pwm=%.9g;fan_duty_cycle=%.9g;battery_voltage_v=%.9g",
                static_cast<double>(_lastControl.LeftCommand()),
                static_cast<double>(_lastControl.RightCommand()),
                static_cast<double>(_lastFanDutyCycle),
                static_cast<double>(_lastBatteryVoltageV)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "estimator_dump_prediction_encoder_input",
                "total_left_counts=%ld;total_right_counts=%ld;left_wheel_speed_radps=%.9g;right_wheel_speed_radps=%.9g",
                static_cast<long>(_predictionEncoderInput.TotalLeftCounts()),
                static_cast<long>(_predictionEncoderInput.TotalRightCounts()),
                static_cast<double>(_predictionEncoderInput.LeftWheelSpeedRadps()),
                static_cast<double>(_predictionEncoderInput.RightWheelSpeedRadps())))
        {
            return false;
        }

        for (int row = 0; row < VehicleState::kDimension; ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "estimator_dump_process_noise_sqrt_row",
                    kEstimatorStateFieldNames,
                    _sqrtProcessNoiseDensity,
                    row))
            {
                return false;
            }
        }

        for (int row = 0; row < static_cast<int>(kEstimatorImuNoiseFieldNames.size()); ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "estimator_dump_imu_noise_sqrt_row",
                    kEstimatorImuNoiseFieldNames,
                    _sqrtImuNoise,
                    row))
            {
                return false;
            }
        }

        for (int row = 0; row < static_cast<int>(kEstimatorFrontNoiseFieldNames.size()); ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "estimator_dump_front_noise_sqrt_row",
                    kEstimatorFrontNoiseFieldNames,
                    _sqrtFrontNoise,
                    row))
            {
                return false;
            }
        }

        if (!EmitNamedMatrixRowLine(
                context,
                sink,
                "estimator_dump_side_noise_sqrt_row",
                kEstimatorSideNoiseFieldNames,
                _sqrtSideNoise,
                0))
        {
            return false;
        }

        return EmitDebugTextLine(
            context,
            sink,
            "estimator_dump_update_metrics",
            "last_update_attempted=%s;last_update_accepted=%s;last_update_nis=%.9g;yaw_rate_innovation_radps=%.9g;yaw_rate_nis=%.9g;forward_accel_innovation_mps2=%.9g;forward_accel_nis=%.9g;right_accel_innovation_mps2=%.9g;right_accel_nis=%.9g;stationary_certified=%s;prediction_encoder_input=%s",
            _lastUpdateAttempted ? "true" : "false",
            _lastUpdateAccepted ? "true" : "false",
            static_cast<double>(_lastUpdateNis),
            static_cast<double>(_lastYawRateInnovationRadps),
            static_cast<double>(_lastYawRateNis),
            static_cast<double>(_lastForwardAccelInnovationMps2),
            static_cast<double>(_lastForwardAccelNis),
            static_cast<double>(_lastRightAccelInnovationMps2),
            static_cast<double>(_lastRightAccelNis),
            _stationaryCertified ? "true" : "false",
            _predictionUsesEncoderInput ? "true" : "false");
    }

    bool Estimator::predict(
        float dt,
        const App::Internal::CommandVector& control) noexcept
    {
        return predictImpl(dt, control);
    }

    float Estimator::runtimeFanDuty() const noexcept
    {
        return _vehicle.GetFanDuty();
    }

    float Estimator::runtimeBatteryVoltage() const noexcept
    {
        return _vehicle.GetBatteryVoltage();
    }

    Eigen::Matrix<float, VehicleState::kDimension, 1> Estimator::PredictSigmaPoint(
        void* context,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint,
        const Eigen::Matrix<float, 3, 1>& control,
        float dtS) noexcept
    {
        (void)control;
        Estimator* const core = static_cast<Estimator*>(context);
        if (core == nullptr)
        {
            return sigmaPoint;
        }

        return core->_plantModel.predictStateFromCommandReference(
            core->_prePredictState,
            sigmaPoint,
            core->_lastControl,
            dtS,
            core->_predictionUsesEncoderInput ? &core->_predictionEncoderInput : nullptr);

    }

    bool Estimator::predictImpl(
        float dt,
        const App::Internal::CommandVector& control) noexcept
    {
        return predictWithInterleavedSensorService(
            dt,
            control,
            nullptr,
            nullptr);
    }

    bool Estimator::predictWithInterleavedSensorService(
        float dt,
        const App::Internal::CommandVector& control,
        void* loopHookContext,
        void (*loopHook)(void*) noexcept) noexcept
    {
        const float fanDutyCycle = runtimeFanDuty();
        const float batteryVoltageV = runtimeBatteryVoltage();
        _lastControl = control;
        _runtimeState.SetCurrentCommand(_lastControl);
        _lastFanDutyCycle = fanDutyCycle;
        _lastBatteryVoltageV = batteryVoltageV;
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return true;
        }

        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        const SensorSnapshot::EncoderObs& encoderInput = snapshot.EncoderObservation();
        const bool useEncoderInput = true;
        _prePredictState = _workingFilter.state();
        _prePredictCovariance = _workingFilter.covariance();
        float presentForwardAccelMps2 = 0.0f;
        float presentRightAccelMps2 = 0.0f;
        float presentYawAccelRadps2 = 0.0f;
        float maxContactRelativeVelocityMps = 0.0f;
        float maxContactUtilization = 0.0f;
        float maxContactSaturation = 0.0f;
        float totalNormalLoadN = 0.0f;
        _plantModel.plantActivityForState(
            _prePredictState,
            control,
            useEncoderInput ? &encoderInput : nullptr,
            presentForwardAccelMps2,
            presentRightAccelMps2,
            presentYawAccelRadps2,
            maxContactRelativeVelocityMps,
            maxContactUtilization,
            maxContactSaturation,
            totalNormalLoadN);
        _runtimeState.SetForwardAcceleration(presentForwardAccelMps2);
        _runtimeState.SetRightAcceleration(presentRightAccelMps2);
        _runtimeState.SetYawAccel(presentYawAccelRadps2);
        (void)maxContactSaturation;
        (void)totalNormalLoadN;
        _havePredictionReference = true;
        _lastYawRateMeasurementRadps = 0.0f;
        _lastYawRateInnovationRadps = 0.0f;
        _lastYawRateNis = 0.0f;
        _lastForwardAccelInnovationMps2 = 0.0f;
        _lastForwardAccelNis = 0.0f;
        _lastRightAccelInnovationMps2 = 0.0f;
        _lastRightAccelNis = 0.0f;

        _predictionUsesEncoderInput = useEncoderInput;
        _predictionEncoderInput = useEncoderInput ? encoderInput : SensorSnapshot{}.EncoderObservation();
        _predictionEncoderDtSeconds = useEncoderInput ? dt : 0.0f;
        const float forwardAccelerationResidualDecayAlpha =
            PlantModel::forwardAccelerationResidualDecayAlpha(dt);
        const float rightAccelerationResidualDecayAlpha =
            PlantModel::rightAccelerationResidualDecayAlpha(dt);
        const float yawAccelerationResidualDecayAlpha =
            PlantModel::yawAccelerationResidualDecayAlpha(dt);

        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> predictProcessNoiseSquareRoot =
            _sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dt);
        const float timingForwardSpeedMps =
            useEncoderInput ?
            std::fabs(_plantModel.measuredLinearSpeedMps(snapshot)) :
            std::fabs(_prePredictState(VehicleState::kVf));
        const float timingYawRateRadps =
            useEncoderInput ?
            std::fabs(_plantModel.measuredYawRateRadps(snapshot)) :
            std::fabs(_prePredictState(VehicleState::kYawRate));
        predictProcessNoiseSquareRoot(VehicleState::kPx, VehicleState::kPx) =
            (std::max)(
                predictProcessNoiseSquareRoot(VehicleState::kPx, VehicleState::kPx),
                timingForwardSpeedMps * kTimingUncertaintySeconds);
        predictProcessNoiseSquareRoot(VehicleState::kPy, VehicleState::kPy) =
            (std::max)(
                predictProcessNoiseSquareRoot(VehicleState::kPy, VehicleState::kPy),
                timingForwardSpeedMps * kTimingUncertaintySeconds);
        predictProcessNoiseSquareRoot(VehicleState::kHeading, VehicleState::kHeading) =
            (std::max)(
                predictProcessNoiseSquareRoot(VehicleState::kHeading, VehicleState::kHeading),
                timingYawRateRadps * kTimingUncertaintySeconds);
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> predictProcessNoiseCovariance =
            predictProcessNoiseSquareRoot * predictProcessNoiseSquareRoot.transpose();

        if (useEncoderInput)
        {
            // The encoder snapshot carries wheel speeds only; PlantModel owns the matching wheel-speed covariance model.
            Eigen::Matrix<float, 2, 2> encoderInputCovarianceRadps2 =
                _plantModel.encoderPairCovarianceRadps(
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
                        (std::max)(
                            1.0e-3f,
                            0.05f * MazeMap::Math::Sqrtf(wheelVarianceRadps2));
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

                    Eigen::Matrix<float, VehicleState::kDimension, 1> plusState =
                        _plantModel.predictStateFromCommandReference(
                            _prePredictState,
                            _prePredictState,
                            control,
                            dt,
                            &plusEncoderInput);

                    Eigen::Matrix<float, VehicleState::kDimension, 1> minusState =
                        _plantModel.predictStateFromCommandReference(
                            _prePredictState,
                            _prePredictState,
                            control,
                            dt,
                            &minusEncoderInput);

                    Eigen::Matrix<float, VehicleState::kDimension, 1> stateDelta = plusState - minusState;
                    stateDelta(VehicleState::kHeading) =
                        NormalizeAngle(plusState(VehicleState::kHeading) - minusState(VehicleState::kHeading));
                    encoderInputJacobian.col(wheelColumn) =
                        stateDelta * (0.5f / deltaWheelSpeedRadps);
                }

                const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> encoderInputProcessCovariance =
                    encoderInputJacobian *
                    encoderInputCovarianceRadps2 *
                    encoderInputJacobian.transpose();
                if (encoderInputProcessCovariance.allFinite())
                {
                    predictProcessNoiseCovariance += encoderInputProcessCovariance;
                }
            }
        }

        const float absForwardSpeedMps =
            std::fabs(_prePredictState(VehicleState::kVf));
        const float absYawRateRadps =
            std::fabs(_prePredictState(VehicleState::kYawRate));
        const float halfTrackWidthM =
            0.5f * (std::max)(0.0f, std::fabs(_vehicle.GetTrackWidth()));
        const float yawSurfaceSpeedMps = halfTrackWidthM * absYawRateRadps;
        const float contactRelativeSpeedReferenceMps =
            (std::max)(kContactSpeedScheduleReferenceMps, absForwardSpeedMps + yawSurfaceSpeedMps);
        const float contactRelativeSpeedUtilization =
            (std::clamp)(maxContactRelativeVelocityMps / contactRelativeSpeedReferenceMps, 0.0f, 1.0f);
        const float yawInducedContactSpeedFraction =
            (std::clamp)(
                yawSurfaceSpeedMps /
                    (yawSurfaceSpeedMps + absForwardSpeedMps + kContactSpeedScheduleReferenceMps),
                0.0f,
                1.0f);
        const float utilization =
            (std::clamp)(maxContactUtilization, 0.0f, 1.0f);
        const float driveSaturationIndex = DriveCommandActivityIndex(control);
        const float encoderFaultIndicator =
            useEncoderInput ? 0.0f : 1.0f;
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
        const float headingForResidualMap =
            NormalizeAngle(_prePredictState(VehicleState::kHeading));
        float residualMapSin = 0.0f;
        float residualMapCos = 0.0f;
        sin_cosf(headingForResidualMap, residualMapSin, residualMapCos);
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        residualSensitivity(VehicleState::kPx, 0) = residualMapSin * dt2;
        residualSensitivity(VehicleState::kPy, 0) = residualMapCos * dt2;
        residualSensitivity(VehicleState::kVf, 0) = dt;
        residualSensitivity(VehicleState::kDeltaAf, 0) = 1.0f;
        residualSensitivity(VehicleState::kPx, 1) = residualMapCos * dt2;
        residualSensitivity(VehicleState::kPy, 1) = -residualMapSin * dt2;
        residualSensitivity(VehicleState::kVr, 1) = dt;
        residualSensitivity(VehicleState::kDeltaAr, 1) = 1.0f;
        residualSensitivity(VehicleState::kPx, 2) =
            ((-_prePredictState(VehicleState::kVr) * residualMapSin) +
             (_prePredictState(VehicleState::kVf) * residualMapCos)) * dt3;
        residualSensitivity(VehicleState::kPy, 2) =
            ((-_prePredictState(VehicleState::kVr) * residualMapCos) -
             (_prePredictState(VehicleState::kVf) * residualMapSin)) * dt3;
        residualSensitivity(VehicleState::kHeading, 2) = dt2;
        residualSensitivity(VehicleState::kYawRate, 2) = dt;
        residualSensitivity(VehicleState::kDeltaYawAccel, 2) = 1.0f;
        Eigen::Matrix<float, 3, 3> residualInnovationCovariance =
            Eigen::Matrix<float, 3, 3>::Zero();
        residualInnovationCovariance(0, 0) = (std::max)(0.0f, qDeltaAf);
        residualInnovationCovariance(1, 1) = (std::max)(0.0f, qDeltaAr);
        residualInnovationCovariance(2, 2) = (std::max)(0.0f, qDeltaYawAccel);
        predictProcessNoiseCovariance +=
            residualSensitivity *
            residualInnovationCovariance *
            residualSensitivity.transpose();
        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> mappedProcessNoiseSquareRoot = predictProcessNoiseSquareRoot;
        if (UKF<VehicleState::kDimension, 3>::FactorCovariance(
                predictProcessNoiseCovariance,
                mappedProcessNoiseSquareRoot))
        {
            predictProcessNoiseSquareRoot = mappedProcessNoiseSquareRoot;
        }
        _workingFilter.setProcessNoiseSquareRoot(predictProcessNoiseSquareRoot);

        Eigen::Matrix<float, 3, 1> filterCommandVector;
        filterCommandVector << control.LeftCommand(), control.RightCommand(), fanDutyCycle;
        const bool predicted = _workingFilter.Predict(
            dt,
            filterCommandVector,
            this,
            &Estimator::PredictSigmaPoint,
            loopHookContext,
            loopHook);
        if (predicted)
        {
            (void)_workingFilter.floorVariance(VehicleState::kVf, kMinimumVelocityVariance);
            (void)_workingFilter.floorVariance(VehicleState::kVr, kMinimumVelocityVariance);
            (void)_workingFilter.floorVariance(VehicleState::kYawRate, kMinimumYawRateVariance);
        }
        return predicted;
    }

    bool Estimator::updateYawRate(float yawRateRadps) noexcept
    {
        return updateYawRateImpl(yawRateRadps);
    }

    Eigen::Matrix<float, 1, 1> Estimator::YawRateMeasurementForState(
        void* context,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept
    {
        Eigen::Matrix<float, 1, 1> prediction;
        (void)context;
        prediction << sigmaPoint(VehicleState::kYawRate);
        return prediction;
    }

    bool Estimator::updateYawRateImpl(float yawRateRadps) noexcept
    {
        _lastUpdateAttempted = std::isfinite(yawRateRadps);
        _lastUpdateAccepted = false;
        _lastUpdateNis = 0.0f;
        if (!_lastUpdateAttempted)
        {
            return false;
        }

        const Eigen::Matrix<float, VehicleState::kDimension, 1> priorState = _workingFilter.state();
        const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> priorSqrtCovariance = _workingFilter.sqrtCovariance();
        _lastYawRateMeasurementRadps = yawRateRadps;
        _lastYawRateInnovationRadps = yawRateRadps - priorState(VehicleState::kYawRate);

        Eigen::Matrix<float, 1, 1> measurement_vec;
        measurement_vec << yawRateRadps;
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        float yawRateMeasurementVariance =
            YawRateMeasurementVarianceRadps2(yawRateRadps);
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
        sqrtNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, yawRateMeasurementVariance));
        _lastUpdateAccepted = _workingFilter.Update<1>(
            measurement_vec,
            sqrtNoise,
            std::numeric_limits<float>::infinity(),
            this,
            &Estimator::YawRateMeasurementForState);
        _lastUpdateNis = _workingFilter.lastNis();
        _lastYawRateNis = _lastUpdateNis;
        if (_lastUpdateAccepted)
        {
            constexpr std::array<int, 1> kAllowedYawRateIndex = { {
                VehicleState::kYawRate
            } };
            Eigen::Matrix<float, VehicleState::kDimension, 1> projectedState = priorState;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> projectedSqrtCovariance = priorSqrtCovariance;
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
        }
        return _lastUpdateAccepted;
    }

    bool Estimator::updatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        return updatePlanarAccelImpl(observation);
    }

    Eigen::Matrix<float, 1, 1> Estimator::ForwardAccelMeasurementForState(
        void* context,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept
    {
        Estimator* const core = static_cast<Estimator*>(context);
        Eigen::Matrix<float, 1, 1> prediction;
        prediction << ((core != nullptr) ?
            core->_plantModel.backLeftImuPlanarAccelerationForState(
                sigmaPoint,
                core->_lastControl,
                core->_predictionUsesEncoderInput ? &core->_predictionEncoderInput : nullptr)(1) :
            0.0f);
        return prediction;
    }

    Eigen::Matrix<float, 1, 1> Estimator::RightAccelMeasurementForState(
        void* context,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept
    {
        Estimator* const core = static_cast<Estimator*>(context);
        Eigen::Matrix<float, 1, 1> prediction;
        prediction << ((core != nullptr) ?
            core->_plantModel.backLeftImuPlanarAccelerationForState(
                sigmaPoint,
                core->_lastControl,
                core->_predictionUsesEncoderInput ? &core->_predictionEncoderInput : nullptr)(0) :
            0.0f);
        return prediction;
    }

    bool Estimator::updatePlanarAccelImpl(const ImuAccelObs& observation) noexcept
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
        float measurementForwardAccelMps2 = 0.0f;
        float measurementRightAccelMps2 = 0.0f;
        float measurementYawAccelRadps2 = 0.0f;
        float maxContactRelativeSpeedMps = 0.0f;
        float maxContactUtilization = 0.0f;
        float maxContactSaturation = 0.0f;
        float totalNormalLoadN = 0.0f;
        _plantModel.plantActivityForState(
            _workingFilter.state(),
            _lastControl,
            _predictionUsesEncoderInput ? &_predictionEncoderInput : nullptr,
            measurementForwardAccelMps2,
            measurementRightAccelMps2,
            measurementYawAccelRadps2,
            maxContactRelativeSpeedMps,
            maxContactUtilization,
            maxContactSaturation,
            totalNormalLoadN);
        const float absForwardSpeedMps =
            std::fabs(_workingFilter.state()(VehicleState::kVf));
        const float absYawRateRadps =
            std::fabs(_workingFilter.state()(VehicleState::kYawRate));
        const float yawInducedContactSpeedMps =
            0.5f * (std::max)(0.0f, std::fabs(_vehicle.GetTrackWidth())) * absYawRateRadps;
        const float contactRelativeSpeedReferenceMps =
            (std::max)(
                kContactSpeedScheduleReferenceMps,
                absForwardSpeedMps + yawInducedContactSpeedMps);
        const float contactRelativeSpeedUtilization =
            (std::clamp)(maxContactRelativeSpeedMps / contactRelativeSpeedReferenceMps, 0.0f, 1.0f);
        const float forceLimitActivity =
            (std::clamp)(maxContactUtilization - 1.0f, 0.0f, 1.0f);
        const float nominalGroundLoadN =
            (std::max)(0.0f, _vehicle.GetMass() * GRAVITY_MPS2);
        const float groundUse =
            (nominalGroundLoadN > 0.0f) ?
            (std::clamp)(1.0f - (totalNormalLoadN / nominalGroundLoadN), 0.0f, 1.0f) :
            0.0f;
        const float driveSaturation =
            (std::clamp)(
                (std::max)(DriveCommandActivityIndex(_lastControl), maxContactSaturation),
                0.0f,
                1.0f);
        const float yawRateRadps =
            std::isfinite(_workingFilter.state()(VehicleState::kYawRate)) ?
            _workingFilter.state()(VehicleState::kYawRate) :
            0.0f;
        const float yawAccelRadps2 =
            std::isfinite(measurementYawAccelRadps2) ?
            measurementYawAccelRadps2 :
            (std::isfinite(_runtimeState.GetYawAccel()) ? _runtimeState.GetYawAccel() : 0.0f);
        const float forwardAccelMps2 =
            std::isfinite(measurementForwardAccelMps2) ?
            measurementForwardAccelMps2 :
            (std::isfinite(_runtimeState.GetForwardAcceleration()) ?
                _runtimeState.GetForwardAcceleration() :
                0.0f);
        const float rightAccelMps2 =
            std::isfinite(measurementRightAccelMps2) ?
            measurementRightAccelMps2 :
            (std::isfinite(_runtimeState.GetRightAcceleration()) ?
                _runtimeState.GetRightAcceleration() :
                0.0f);
        const float speedMps =
            MazeMap::Math::Sqrtf(
                (_workingFilter.state()(VehicleState::kVf) * _workingFilter.state()(VehicleState::kVf)) +
                (_workingFilter.state()(VehicleState::kVr) * _workingFilter.state()(VehicleState::kVr)));
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
            _plantModel.backLeftImuPlanarAccelerationForState(
                _workingFilter.state(),
                _lastControl,
                _predictionUsesEncoderInput ? &_predictionEncoderInput : nullptr)(1);
        Eigen::Matrix<float, 1, 1> forwardNoise;
        forwardNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, accelMeasurementVariance));
        const bool forwardAccepted = _workingFilter.Update<1>(
            forwardMeasurement,
            forwardNoise,
            25.0f,
            this,
            &Estimator::ForwardAccelMeasurementForState);
        _lastForwardAccelNis = _workingFilter.lastNis();

        Eigen::Matrix<float, 1, 1> rightMeasurement;
        rightMeasurement << observation.AccelBodyRightMps2();
        _lastRightAccelInnovationMps2 =
            observation.AccelBodyRightMps2() -
            _plantModel.backLeftImuPlanarAccelerationForState(
                _workingFilter.state(),
                _lastControl,
                _predictionUsesEncoderInput ? &_predictionEncoderInput : nullptr)(0);
        Eigen::Matrix<float, 1, 1> rightNoise;
        rightNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, accelMeasurementVariance));
        const bool rightAccepted = _workingFilter.Update<1>(
            rightMeasurement,
            rightNoise,
            25.0f,
            this,
            &Estimator::RightAccelMeasurementForState);
        _lastRightAccelNis = _workingFilter.lastNis();

        _lastUpdateAccepted = forwardAccepted || rightAccepted;
        _lastUpdateNis = forwardAccepted ? _lastForwardAccelNis : _lastRightAccelNis;
        return _lastUpdateAccepted;
    }

    bool Estimator::controlCommandsAreEffectivelyZero() const noexcept
    {
        if (!_lastControl.IsFinite())
        {
            return true;
        }

        return
            (std::fabs(_lastControl.LeftCommand()) <= kStationaryCandidateMaxDriveCommand) &&
            (std::fabs(_lastControl.RightCommand()) <= kStationaryCandidateMaxDriveCommand);
    }

    bool Estimator::isStationaryCandidate(float yawRateRadps) const noexcept
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

    void Estimator::updateStationaryCertification(float yawRateRadps) noexcept
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

    float Estimator::wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept
    {
        const float normalizedConfidence = (std::clamp)(confidence, 0.0f, 1.0f);
        return minimumNoise + ((1.0f - normalizedConfidence) * 0.020f);
    }

    float Estimator::wallPredictionForSensor(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint,
        const SensorMount& sensor,
        const Maze& maze,
        float noHitRangeM) const noexcept
    {
        const float headingRad = sigmaPoint(VehicleState::kHeading);
        const WallGeometryModel::GeometryStateFrame frame = BuildWallGeometryFrame(
            sigmaPoint(VehicleState::kPx),
            sigmaPoint(VehicleState::kPy),
            Eigen::Vector2f(std::sin(headingRad), std::cos(headingRad)));
        const GeometryPrediction prediction = _geometryModel.predictRay(frame, sensor, maze, noHitRangeM);
        return prediction.hit ? prediction.rangeM : noHitRangeM;
    }

    Eigen::Matrix<float, 2, 1> Estimator::frontPairPredictionForState(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint,
        const Maze& maze) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const SensorMount frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
        const SensorMount frontRightSensor = Vehicle::GetFrontRightSensorMount();
        const float frontLeftNoHitRangeM = _vehicle.FrontLeftWallSensor().GetNoHitRangeM();
        const float frontRightNoHitRangeM = _vehicle.FrontRightWallSensor().GetNoHitRangeM();
        const float headingRad = sigmaPoint(VehicleState::kHeading);
        const WallGeometryModel::GeometryStateFrame frame = BuildWallGeometryFrame(
            sigmaPoint(VehicleState::kPx),
            sigmaPoint(VehicleState::kPy),
            Eigen::Vector2f(std::sin(headingRad), std::cos(headingRad)));
        const GeometryPrediction leftPrediction =
            _geometryModel.predictRay(frame, frontLeftSensor, maze, frontLeftNoHitRangeM);
        const GeometryPrediction rightPrediction =
            _geometryModel.predictRay(frame, frontRightSensor, maze, frontRightNoHitRangeM);
        prediction(0) = leftPrediction.hit ? leftPrediction.rangeM : frontLeftNoHitRangeM;
        prediction(1) = rightPrediction.hit ? rightPrediction.rangeM : frontRightNoHitRangeM;
        return prediction;
    }

    Eigen::Matrix<float, 2, 1> Estimator::FrontPairMeasurementForState(
        void* context,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept
    {
        Estimator* const core = static_cast<Estimator*>(context);
        if ((core == nullptr) || (core->_measurementMaze == nullptr))
        {
            return Eigen::Matrix<float, 2, 1>::Zero();
        }

        return core->frontPairPredictionForState(sigmaPoint, *core->_measurementMaze);
    }

    Eigen::Matrix<float, 1, 1> Estimator::SideMeasurementForState(
        void* context,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& sigmaPoint) noexcept
    {
        Estimator* const core = static_cast<Estimator*>(context);
        Eigen::Matrix<float, 1, 1> prediction;
        if ((core == nullptr) || (core->_measurementMaze == nullptr))
        {
            prediction(0) = 0.0f;
            return prediction;
        }

        prediction(0) = core->wallPredictionForSensor(
            sigmaPoint,
            core->_measurementSensor,
            *core->_measurementMaze,
            core->_measurementNoHitRangeM);
        return prediction;
    }

    bool Estimator::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const Maze& maze,
        bool freezeMapMutation,
        const MapEvidenceUpdater::Config& evidenceConfig) noexcept
    {
        _lastUpdateAttempted = false;
        _lastUpdateAccepted = false;
        _lastUpdateNis = 0.0f;
        const SensorMount frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
        const SensorMount frontRightSensor = Vehicle::GetFrontRightSensorMount();
        const float frontLeftNoHitRangeM = _vehicle.FrontLeftWallSensor().GetNoHitRangeM();
        const float frontRightNoHitRangeM = _vehicle.FrontRightWallSensor().GetNoHitRangeM();
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = _workingFilter.state();
        const float headingRad = state(VehicleState::kHeading);
        const WallGeometryModel::GeometryStateFrame frame = BuildWallGeometryFrame(
            state(VehicleState::kPx),
            state(VehicleState::kPy),
            Eigen::Vector2f(std::sin(headingRad), std::cos(headingRad)));
        _lastFrontLeftWallPrediction = _geometryModel.predictRay(frame, frontLeftSensor, maze, frontLeftNoHitRangeM);
        _lastFrontRightWallPrediction = _geometryModel.predictRay(frame, frontRightSensor, maze, frontRightNoHitRangeM);
        _lastUpdateAttempted =
            left.IsValid() &&
            right.IsValid();
        if (!_lastUpdateAttempted)
        {
            return false;
        }

        Eigen::Matrix<float, 2, 2> sqrtNoise = _sqrtFrontNoise;
        float leftWallVariance =
            wallNoiseFromConfidence(left.Confidence(), _sqrtFrontNoise(0, 0)) *
            wallNoiseFromConfidence(left.Confidence(), _sqrtFrontNoise(0, 0));
        float rightWallVariance =
            wallNoiseFromConfidence(right.Confidence(), _sqrtFrontNoise(1, 1)) *
            wallNoiseFromConfidence(right.Confidence(), _sqrtFrontNoise(1, 1));
        if (std::isfinite(left.MeasurementNoiseSigmaM()) && (left.MeasurementNoiseSigmaM() > 0.0f))
        {
            leftWallVariance += left.MeasurementNoiseSigmaM() * left.MeasurementNoiseSigmaM();
        }
        if (std::isfinite(right.MeasurementNoiseSigmaM()) && (right.MeasurementNoiseSigmaM() > 0.0f))
        {
            rightWallVariance += right.MeasurementNoiseSigmaM() * right.MeasurementNoiseSigmaM();
        }
        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        std::uint32_t frontEffectiveTimeUs = snapshot.FrontTiming().adcOnSampleUs;
        if (snapshot.FrontTiming().adcOffSampleUs > snapshot.FrontTiming().adcOnSampleUs)
        {
            frontEffectiveTimeUs =
                snapshot.FrontTiming().adcOnSampleUs +
                ((snapshot.FrontTiming().adcOffSampleUs - snapshot.FrontTiming().adcOnSampleUs) / 2U);
        }
        if ((snapshot.FrontTiming().observationReadyUs > frontEffectiveTimeUs) &&
            ((snapshot.FrontTiming().observationReadyUs - frontEffectiveTimeUs) <= kMaxCredibleWallTimingDeltaUs))
        {
            const float wallTimingSeconds =
                1.0e-6f * static_cast<float>(snapshot.FrontTiming().observationReadyUs - frontEffectiveTimeUs);
            const float yawRateRadps = state(VehicleState::kYawRate);
            const Eigen::Vector2f frontLeftVelocityBodyMps(
                state(VehicleState::kVr) + (frontLeftSensor.positionBodyM().y() * yawRateRadps),
                state(VehicleState::kVf) - (frontLeftSensor.positionBodyM().x() * yawRateRadps));
            const Eigen::Vector2f frontRightVelocityBodyMps(
                state(VehicleState::kVr) + (frontRightSensor.positionBodyM().y() * yawRateRadps),
                state(VehicleState::kVf) - (frontRightSensor.positionBodyM().x() * yawRateRadps));
            const float leftTimingSigmaM =
                std::fabs(frontLeftSensor.SensorForwardBody().dot(frontLeftVelocityBodyMps)) * wallTimingSeconds;
            const float rightTimingSigmaM =
                std::fabs(frontRightSensor.SensorForwardBody().dot(frontRightVelocityBodyMps)) * wallTimingSeconds;
            leftWallVariance += leftTimingSigmaM * leftTimingSigmaM;
            rightWallVariance += rightTimingSigmaM * rightTimingSigmaM;
        }
        sqrtNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, leftWallVariance));
        sqrtNoise(1, 1) = MazeMap::Math::Sqrtf((std::max)(0.0f, rightWallVariance));

        Eigen::Matrix<float, 2, 1> measurement_vec;
        measurement_vec << left.Rho(), right.Rho();
        _measurementMaze = &maze;
        _lastUpdateAccepted = _workingFilter.Update<2>(
            measurement_vec,
            sqrtNoise,
            9.21034f,
            this,
            &Estimator::FrontPairMeasurementForState);
        _measurementMaze = nullptr;
        _lastUpdateNis = _workingFilter.lastNis();
        if (_lastUpdateAccepted && !freezeMapMutation)
        {
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& updatedState = _workingFilter.state();
            const Direction leftDirection = dominantDirectionForSensor(frontLeftSensor, updatedState);
            const Direction rightDirection = dominantDirectionForSensor(frontRightSensor, updatedState);
            const CellCoordinates leftCell = estimateSensorCell(frontLeftSensor, updatedState);
            const CellCoordinates rightCell = estimateSensorCell(frontRightSensor, updatedState);
            _mapEvidence.Apply(
                leftCell,
                leftDirection,
                left,
                _lastFrontLeftWallPrediction,
                evidenceConfig,
                freezeMapMutation);
            _mapEvidence.Apply(
                rightCell,
                rightDirection,
                right,
                _lastFrontRightWallPrediction,
                evidenceConfig,
                freezeMapMutation);
        }
        return _lastUpdateAccepted;
    }

    bool Estimator::updateSideSensor(
        RelativeDirection which,
        const WallObs& observation,
        const Maze& maze,
        bool freezeMapMutation,
        const MapEvidenceUpdater::Config& evidenceConfig) noexcept
    {
        _lastUpdateAttempted = false;
        _lastUpdateAccepted = false;
        _lastUpdateNis = 0.0f;
        _lastSideWallPrediction = GeometryPrediction{};
        const bool isLeft = which == RelativeDirection::Left90;
        const bool isRight = which == RelativeDirection::Right90;
        if (!isLeft && !isRight)
        {
            return false;
        }

        const SensorMount sensor = isLeft ? Vehicle::GetSideLeftSensorMount() : Vehicle::GetSideRightSensorMount();
        const float noHitRangeM =
            isLeft ?
            _vehicle.SideLeftWallSensor().GetNoHitRangeM() :
            _vehicle.SideRightWallSensor().GetNoHitRangeM();
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = _workingFilter.state();
        const float headingRad = state(VehicleState::kHeading);
        _lastSideWallPrediction = _geometryModel.predictRay(
            BuildWallGeometryFrame(
                state(VehicleState::kPx),
                state(VehicleState::kPy),
                Eigen::Vector2f(std::sin(headingRad), std::cos(headingRad))),
            sensor,
            maze,
            noHitRangeM);
        _lastUpdateAttempted = observation.IsValid();
        if (!_lastUpdateAttempted)
        {
            return false;
        }

        Eigen::Matrix<float, 1, 1> sqrtNoise = _sqrtSideNoise;
        float wallVariance =
            wallNoiseFromConfidence(observation.Confidence(), _sqrtSideNoise(0, 0)) *
            wallNoiseFromConfidence(observation.Confidence(), _sqrtSideNoise(0, 0));
        if (std::isfinite(observation.MeasurementNoiseSigmaM()) && (observation.MeasurementNoiseSigmaM() > 0.0f))
        {
            wallVariance += observation.MeasurementNoiseSigmaM() * observation.MeasurementNoiseSigmaM();
        }
        const SensorSnapshot& snapshot = _runtimeState.GetSensorSnapshot();
        const OpticalObservationTiming& timing = isLeft ? snapshot.LeftTiming() : snapshot.RightTiming();
        std::uint32_t effectiveTimeUs = timing.adcOnSampleUs;
        if (timing.adcOffSampleUs > timing.adcOnSampleUs)
        {
            effectiveTimeUs = timing.adcOnSampleUs + ((timing.adcOffSampleUs - timing.adcOnSampleUs) / 2U);
        }
        if ((timing.observationReadyUs > effectiveTimeUs) &&
            ((timing.observationReadyUs - effectiveTimeUs) <= kMaxCredibleWallTimingDeltaUs))
        {
            const float wallTimingSeconds =
                1.0e-6f * static_cast<float>(timing.observationReadyUs - effectiveTimeUs);
            const Eigen::Vector2f sensorVelocityBodyMps(
                state(VehicleState::kVr) + (sensor.positionBodyM().y() * state(VehicleState::kYawRate)),
                state(VehicleState::kVf) - (sensor.positionBodyM().x() * state(VehicleState::kYawRate)));
            const float timingSigmaM =
                std::fabs(sensor.SensorForwardBody().dot(sensorVelocityBodyMps)) * wallTimingSeconds;
            wallVariance += timingSigmaM * timingSigmaM;
        }
        sqrtNoise(0, 0) = MazeMap::Math::Sqrtf((std::max)(0.0f, wallVariance));

        Eigen::Matrix<float, 1, 1> measurement_vec;
        measurement_vec << observation.Rho();
        _measurementMaze = &maze;
        _measurementSensor = sensor;
        _measurementNoHitRangeM = noHitRangeM;
        _lastUpdateAccepted = _workingFilter.Update<1>(
            measurement_vec,
            sqrtNoise,
            7.87944f,
            this,
            &Estimator::SideMeasurementForState);
        _measurementMaze = nullptr;
        _lastUpdateNis = _workingFilter.lastNis();
        if (_lastUpdateAccepted && !freezeMapMutation)
        {
            const SensorMount& evidenceSensor =
                (which == RelativeDirection::Left90) ?
                Vehicle::GetSideLeftSensorMount() :
                Vehicle::GetSideRightSensorMount();
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& updatedState = _workingFilter.state();
            const Direction direction = dominantDirectionForSensor(evidenceSensor, updatedState);
            const CellCoordinates cell = estimateSensorCell(evidenceSensor, updatedState);
            _mapEvidence.Apply(
                cell,
                direction,
                observation,
                _lastSideWallPrediction,
                evidenceConfig,
                freezeMapMutation);
        }
        return _lastUpdateAccepted;
    }
    Direction Estimator::dominantDirectionForSensor(
        const SensorMount& sensor,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const float heading = state(VehicleState::kHeading);
        const WallGeometryModel::GeometryStateFrame frame = Estimator::BuildWallGeometryFrame(
            state(VehicleState::kPx),
            state(VehicleState::kPy),
            Eigen::Vector2f(std::sin(heading), std::cos(heading)));
        const Eigen::Vector2f directionWorld =
            geometryModel.sensorDirectionWorld(frame, sensor);
        const float x = directionWorld.x();
        const float y = directionWorld.y();
        if (std::fabs(x) >= std::fabs(y))
        {
            return (x >= 0.0f) ? Direction::Right : Direction::Left;
        }
        return (y >= 0.0f) ? Direction::Up : Direction::Down;
    }

    CellCoordinates Estimator::estimateSensorCell(
        const SensorMount& sensor,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const float heading = state(VehicleState::kHeading);
        const WallGeometryModel::GeometryStateFrame frame = Estimator::BuildWallGeometryFrame(
            state(VehicleState::kPx),
            state(VehicleState::kPy),
            Eigen::Vector2f(std::sin(heading), std::cos(heading)));
        const Eigen::Vector2f sensorPositionWorld =
            geometryModel.sensorOriginWorld(frame, sensor);
        return WallGeometryModel::WorldToCell(sensorPositionWorld.x(), sensorPositionWorld.y());
    }

    void Estimator::ClearFault() noexcept
    {
        _faulted = false;
        _faultReason[0] = '\0';
    }

    bool Estimator::ResetPose(float xMeters, float yMeters, float headingRad) noexcept
    {
        Eigen::Matrix<float, VehicleState::kDimension, 1> state = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
        state(VehicleState::kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(VehicleState::kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(VehicleState::kHeading) = WrapAngleRad(headingRad);
        ClearFault();
        _mapEvidence.Reset();
        return reset(state, BuildDefaultInitialCovariance());
    }

    bool Estimator::ResetForSessionTransition(float xMeters, float yMeters, float headingRad) noexcept
    {
        Eigen::Matrix<float, VehicleState::kDimension, 1> state = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
        state(VehicleState::kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(VehicleState::kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(VehicleState::kHeading) = WrapAngleRad(headingRad);

        ClearFault();
        const bool ok = reset(state, BuildDefaultInitialCovariance());
        if (!ok)
        {
            TriggerFault("session_transition_reset_failed");
            return false;
        }

        ResetRuntimeMetadata();
        return true;
    }

    bool Estimator::RestoreSessionStartPhysicalState(float xMeters, float yMeters, float headingRad) noexcept
    {
        // Sorting note: this owns only the estimator state restore. Encoder consumption,
        // controller reset, and braking remain DriveBase concerns and are not translated here.
        return ResetForSessionTransition(xMeters, yMeters, headingRad);
    }

    void Estimator::ResetRuntimeMetadata() noexcept
    {
        _runtimeState.SetTime(0.0f);
        _runtimeState.SetTimestampUs(0U);
        _runtimeState.SetForwardAcceleration(0.0f);
        _runtimeState.SetRightAcceleration(0.0f);
        _runtimeState.SetYawAccel(0.0f);
        _runtimeState.SetSensorSnapshot(SensorSnapshot{});
    }

    void Estimator::TriggerFault(const char* reason) noexcept
    {
        if (_faulted)
        {
            return;
        }

        _faulted = true;
        std::snprintf(
            _faultReason,
            sizeof(_faultReason),
            "%s",
            (reason != nullptr && reason[0] != '\0') ? reason : "estimator_failure");
    }
}
