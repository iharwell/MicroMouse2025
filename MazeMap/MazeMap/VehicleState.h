#pragma once

#include "Defines.h"
#include "CommandVector.h"
#include "EigenCompat.h"
#include "MmLog.h"
#include "SensorSnapshot.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MazeMap
{
    class Estimator;
    class PlantModel;

    class EXPORT VehicleState
    {

        enum Index : int
        {
            kPx = 0,
            kPy = 1,
            kHeading = 2,
            kVf = 3,
            kVr = 4,
            kYawRate = 5,
            kDeltaAf = 6,
            kDeltaAr = 7,
            kDeltaYawAccel = 8
        };
    public:
        static constexpr int kDimension = 9;

        // World pose uses +X right and +Y forward.
        // Body velocity uses kVf = body-forward speed, kVr = body-right speed, and kYawRate = clockwise yaw rate.
        // The final states are colored acceleration residuals: kDeltaAf, kDeltaAr, and kDeltaYawAccel.

        VehicleState() noexcept
            : _state(Eigen::Matrix<float, kDimension, 1>::Zero())
            , _sqrtCovariance(Eigen::Matrix<float, kDimension, kDimension>::Zero())
            , _gyroBiasZRadps(0.0f)
            , _gyroBiasZVarianceRadps2(3.05e-4f)
            , _time(0.0f)
            , _timestampUs(0U)
        {
            SetCovariance(DefaultInitialCovariance());
        }

        bool IsStationary() const noexcept;

        void SetPosition(const Eigen::Vector2f& position) noexcept { _state(kPx) = position.x(); _state(kPy) = position.y(); }
        float GetPositionX() const noexcept { return _state(kPx); }
        float GetPositionX() noexcept { return const_cast<const VehicleState*>(this)->GetPositionX(); }
        float GetPositionY() const noexcept { return _state(kPy); }
        float GetPositionY() noexcept { return const_cast<const VehicleState*>(this)->GetPositionY(); }
        Eigen::Vector2f GetPosition() const noexcept { return Eigen::Vector2f(_state(kPx), _state(kPy)); }
        Eigen::Vector2f GetPosition() noexcept { return const_cast<const VehicleState*>(this)->GetPosition(); }

        void SetForwardVelocity(float forwardVelocityMps) noexcept { _state(kVf) = forwardVelocityMps; }
        float GetForwardVelocity() const noexcept { return _state(kVf); }
        float GetForwardVelocity() noexcept { return const_cast<const VehicleState*>(this)->GetForwardVelocity(); }

        void SetRightwardVelocity(float rightwardVelocityMps) noexcept { _state(kVr) = rightwardVelocityMps; }
        float GetRightwardVelocity() const noexcept { return _state(kVr); }
        float GetRightwardVelocity() noexcept { return const_cast<const VehicleState*>(this)->GetRightwardVelocity(); }

        void SetHeading(float headingRad) noexcept { _state(kHeading) = NormalizeAngle(headingRad); }
        float GetHeading() const noexcept { return _state(kHeading); }
        float GetHeading() noexcept { return const_cast<const VehicleState*>(this)->GetHeading(); }
        Eigen::Vector2f GetHeadingUnit() const noexcept
        {
            return Eigen::Vector2f(std::sin(_state(kHeading)), std::cos(_state(kHeading)));
        }
        Eigen::Vector2f GetHeadingUnit() noexcept { return const_cast<const VehicleState*>(this)->GetHeadingUnit(); }

        void SetYawRate(float yawRateRadps) noexcept { _state(kYawRate) = yawRateRadps; }
        float GetYawRate() const noexcept { return _state(kYawRate); }
        float GetYawRate() noexcept { return const_cast<const VehicleState*>(this)->GetYawRate(); }

        void SetForwardAcceleration(float accelerationMps2) noexcept
        {
            _forwardAccelerationMps2 = accelerationMps2;
        }
        float GetForwardAcceleration() const noexcept { return _forwardAccelerationMps2; }
        float GetForwardAcceleration() noexcept { return const_cast<const VehicleState*>(this)->GetForwardAcceleration(); }

        void SetRightAcceleration(float accelerationMps2) noexcept
        {
            _rightAccelerationMps2 = accelerationMps2;
        }
        float GetRightAcceleration() const noexcept { return _rightAccelerationMps2; }
        float GetRightAcceleration() noexcept { return const_cast<const VehicleState*>(this)->GetRightAcceleration(); }

        void SetYawAccel(float accelerationRadps2) noexcept
        {
            _yawAccelRadps2 = accelerationRadps2;
        }
        float GetYawAccel() const noexcept { return _yawAccelRadps2; }
        float GetYawAccel() noexcept { return const_cast<const VehicleState*>(this)->GetYawAccel(); }

        void SetForwardAccelerationResidual(float accelerationMps2) noexcept { _state(kDeltaAf) = accelerationMps2; }
        float GetForwardAccelerationResidual() const noexcept { return _state(kDeltaAf); }
        float GetForwardAccelerationResidual() noexcept { return const_cast<const VehicleState*>(this)->GetForwardAccelerationResidual(); }

        void SetRightwardAccelerationResidual(float accelerationMps2) noexcept { _state(kDeltaAr) = accelerationMps2; }
        float GetRightwardAccelerationResidual() const noexcept { return _state(kDeltaAr); }
        float GetRightwardAccelerationResidual() noexcept { return const_cast<const VehicleState*>(this)->GetRightwardAccelerationResidual(); }

        void SetYawAccelResidual(float accelerationRadps2) noexcept { _state(kDeltaYawAccel) = accelerationRadps2; }
        float GetYawAccelResidual() const noexcept { return _state(kDeltaYawAccel); }
        float GetYawAccelResidual() noexcept { return const_cast<const VehicleState*>(this)->GetYawAccelResidual(); }

        void SetWheelSpeedLeft(float wheelSpeedRadps) noexcept
        {
            MazeMap::EncoderObs encoderObservation = _sensorSnapshot.EncoderObservation();
            encoderObservation.SetLeftWheelSpeedRadps(wheelSpeedRadps);
            _sensorSnapshot.SetEncoderObservation(encoderObservation);
        }
        float GetWheelSpeedLeft() const noexcept { return _sensorSnapshot.EncoderObservation().LeftWheelSpeedRadps(); }
        float GetWheelSpeedLeft() noexcept { return const_cast<const VehicleState*>(this)->GetWheelSpeedLeft(); }

        void SetWheelSpeedRight(float wheelSpeedRadps) noexcept
        {
            MazeMap::EncoderObs encoderObservation = _sensorSnapshot.EncoderObservation();
            encoderObservation.SetRightWheelSpeedRadps(wheelSpeedRadps);
            _sensorSnapshot.SetEncoderObservation(encoderObservation);
        }
        float GetWheelSpeedRight() const noexcept { return _sensorSnapshot.EncoderObservation().RightWheelSpeedRadps(); }
        float GetWheelSpeedRight() noexcept { return const_cast<const VehicleState*>(this)->GetWheelSpeedRight(); }

        void SetGyroBiasZ(float gyroBiasRadps) noexcept { _gyroBiasZRadps = gyroBiasRadps; }
        float GetGyroBiasZ() const noexcept { return _gyroBiasZRadps; }
        float GetGyroBiasZ() noexcept { return const_cast<const VehicleState*>(this)->GetGyroBiasZ(); }

        void SetGyroBiasZVar(float gyroBiasVarianceRadps2) noexcept
        {
            _gyroBiasZVarianceRadps2 = (std::max)(0.0f, gyroBiasVarianceRadps2);
        }
        float GetGyroBiasZVar() const noexcept { return _gyroBiasZVarianceRadps2; }
        float GetGyroBiasZVar() noexcept { return const_cast<const VehicleState*>(this)->GetGyroBiasZVar(); }

        void SetTime(float time) noexcept { _time = time; }
        float GetTime() const noexcept { return _time; }
        float GetTime() noexcept { return const_cast<const VehicleState*>(this)->GetTime(); }

        void SetTimestampUs(std::uint32_t timestampUs) noexcept { _timestampUs = timestampUs; }
        std::uint32_t GetTimestampUs() const noexcept { return _timestampUs; }
        std::uint32_t GetTimestampUs() noexcept { return const_cast<const VehicleState*>(this)->GetTimestampUs(); }

        void SetSensorSnapshot(const ::SensorSnapshot& sensorSnapshot) noexcept { _sensorSnapshot = sensorSnapshot; }
        const ::SensorSnapshot& GetSensorSnapshot() const noexcept { return _sensorSnapshot; }

        void SetCurrentCommand(const App::Internal::CommandVector& currentCommand) noexcept { _currentCommand = currentCommand; }
        const App::Internal::CommandVector& GetCurrentCommand() const noexcept { return _currentCommand; }
        App::Internal::CommandVector& GetCurrentCommand() noexcept { return _currentCommand; }

        void SetVarianceValues(float xVar, float yVar, float forwardVelocityVar, float headingVar, float yawRateVar)
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kPx, kPx) = (std::max)(0.0f, xVar);
            covariance(kPy, kPy) = (std::max)(0.0f, yVar);
            covariance(kVf, kVf) = (std::max)(0.0f, forwardVelocityVar);
            covariance(kHeading, kHeading) = (std::max)(0.0f, headingVar);
            covariance(kYawRate, kYawRate) = (std::max)(0.0f, yawRateVar);
            SetCovariance(covariance);
        }

        void SetPositionVar(const Eigen::Vector2f& positionVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kPx, kPx) = (std::max)(0.0f, positionVariance.x());
            covariance(kPy, kPy) = (std::max)(0.0f, positionVariance.y());
            SetCovariance(covariance);
        }

        Eigen::Vector2f GetPositionVar() const noexcept
        {
            const Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            return Eigen::Vector2f(covariance(kPx, kPx), covariance(kPy, kPy));
        }

        Eigen::Vector2f GetPositionVar() noexcept { return const_cast<const VehicleState*>(this)->GetPositionVar(); }

        void SetForwardVelocityVar(float forwardVelocityVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kVf, kVf) = (std::max)(0.0f, forwardVelocityVariance);
            SetCovariance(covariance);
        }

        float GetForwardVelocityVar() const noexcept { return GetCovariance()(kVf, kVf); }
        float GetForwardVelocityVar() noexcept { return const_cast<const VehicleState*>(this)->GetForwardVelocityVar(); }

        void SetRightwardVelocityVar(float rightwardVelocityVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kVr, kVr) = (std::max)(0.0f, rightwardVelocityVariance);
            SetCovariance(covariance);
        }

        float GetRightwardVelocityVar() const noexcept { return GetCovariance()(kVr, kVr); }
        float GetRightwardVelocityVar() noexcept { return const_cast<const VehicleState*>(this)->GetRightwardVelocityVar(); }

        void SetHeadingVar(float headingVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kHeading, kHeading) = (std::max)(0.0f, headingVariance);
            SetCovariance(covariance);
        }

        float GetHeadingVar() const noexcept { return GetCovariance()(kHeading, kHeading); }
        float GetHeadingVar() noexcept { return const_cast<const VehicleState*>(this)->GetHeadingVar(); }

        void SetYawRateVar(float yawRateVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kYawRate, kYawRate) = (std::max)(0.0f, yawRateVariance);
            SetCovariance(covariance);
        }

        float GetYawRateVar() const noexcept { return GetCovariance()(kYawRate, kYawRate); }
        float GetYawRateVar() noexcept { return const_cast<const VehicleState*>(this)->GetYawRateVar(); }

        void SetForwardAccelerationResidualVar(float residualVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kDeltaAf, kDeltaAf) = (std::max)(0.0f, residualVariance);
            SetCovariance(covariance);
        }

        float GetForwardAccelerationResidualVar() const noexcept { return GetCovariance()(kDeltaAf, kDeltaAf); }
        float GetForwardAccelerationResidualVar() noexcept { return const_cast<const VehicleState*>(this)->GetForwardAccelerationResidualVar(); }

        void SetRightwardAccelerationResidualVar(float residualVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kDeltaAr, kDeltaAr) = (std::max)(0.0f, residualVariance);
            SetCovariance(covariance);
        }

        float GetRightwardAccelerationResidualVar() const noexcept { return GetCovariance()(kDeltaAr, kDeltaAr); }
        float GetRightwardAccelerationResidualVar() noexcept { return const_cast<const VehicleState*>(this)->GetRightwardAccelerationResidualVar(); }

        void SetYawAccelResidualVar(float residualVariance) noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = GetCovariance();
            covariance(kDeltaYawAccel, kDeltaYawAccel) = (std::max)(0.0f, residualVariance);
            SetCovariance(covariance);
        }

        float GetYawAccelResidualVar() const noexcept { return GetCovariance()(kDeltaYawAccel, kDeltaYawAccel); }
        float GetYawAccelResidualVar() noexcept { return const_cast<const VehicleState*>(this)->GetYawAccelResidualVar(); }

    private:
        friend class Estimator;
        friend class PlantModel;

        static Eigen::Matrix<float, kDimension, kDimension> DefaultInitialCovariance() noexcept
        {
            Eigen::Matrix<float, kDimension, kDimension> covariance = Eigen::Matrix<float, kDimension, kDimension>::Identity() * 1.0e-3f;
            covariance(kPx, kPx) = 1.0e-5f;
            covariance(kPy, kPy) = 1.0e-5f;
            covariance(kDeltaAf, kDeltaAf) = 0.25f;
            covariance(kDeltaAr, kDeltaAr) = 0.25f;
            covariance(kDeltaYawAccel, kDeltaYawAccel) = 0.25f;
            return covariance;
        }

        const Eigen::Matrix<float, kDimension, kDimension>& GetSqrtCovariance() const noexcept { return _sqrtCovariance; }
        void SetSqrtCovariance(const Eigen::Matrix<float, kDimension, kDimension>& sqrtCovariance) noexcept { _sqrtCovariance = sqrtCovariance; }

        Eigen::Matrix<float, kDimension, kDimension> GetCovariance() const noexcept { return _sqrtCovariance * _sqrtCovariance.transpose(); }
        void SetCovariance(const Eigen::Matrix<float, kDimension, kDimension>& covariance) noexcept
        {
            Eigen::LLT<Eigen::Matrix<float, kDimension, kDimension>> llt;
            llt.compute(covariance);
            if (llt.info() == Eigen::Success)
            {
                _sqrtCovariance = llt.matrixL();
                return;
            }
            const Eigen::Matrix<float, kDimension, kDimension> symmetric = 0.5f * (covariance + covariance.transpose());
            llt.compute(symmetric);
            if (llt.info() == Eigen::Success)
            {
                _sqrtCovariance = llt.matrixL();
                return;
            }
        }

        const Eigen::Matrix<float, kDimension, 1>& GetStateVector() const noexcept { return _state; }

        void SetStateVector(const Eigen::Matrix<float, kDimension, 1>& state) noexcept
        {
            _state = state;
            _state(kHeading) = NormalizeAngle(_state(kHeading));
        }

        Eigen::Matrix<float, kDimension, 1> _state;
        Eigen::Matrix<float, kDimension, kDimension> _sqrtCovariance;
        float _gyroBiasZRadps;
        float _gyroBiasZVarianceRadps2;
        float _forwardAccelerationMps2 = 0.0f;
        float _rightAccelerationMps2 = 0.0f;
        float _yawAccelRadps2 = 0.0f;
        float _time;
        std::uint32_t _timestampUs;
        ::SensorSnapshot _sensorSnapshot;
        App::Internal::CommandVector _currentCommand{};
    };

#define VEHICLE_STATE_LOG_FIELDS(X) \
    X(float, px_m) \
    X(float, py_m) \
    X(float, heading_rad) \
    X(float, vf_mps) \
    X(float, vr_mps) \
    X(float, yaw_rate_radps) \
    X(float, delta_af_mps2) \
    X(float, delta_ar_mps2) \
    X(float, delta_yaw_accel_radps2) \
    X(float, gyro_bias_z_radps)

    MMLOG_DEFINE_PRIVATE_ENTRY_WITH_BODY(
        VehicleStateLogEntry,
        VEHICLE_STATE_LOG_FIELDS,
        void Set(const VehicleState& state) noexcept
        {
            px_m = state.GetPositionX();
            py_m = state.GetPositionY();
            heading_rad = state.GetHeading();
            vf_mps = state.GetForwardVelocity();
            vr_mps = state.GetRightwardVelocity();
            yaw_rate_radps = state.GetYawRate();
            delta_af_mps2 = state.GetForwardAccelerationResidual();
            delta_ar_mps2 = state.GetRightwardAccelerationResidual();
            delta_yaw_accel_radps2 = state.GetYawAccelResidual();
            gyro_bias_z_radps = state.GetGyroBiasZ();
        });

#undef VEHICLE_STATE_LOG_FIELDS
}
