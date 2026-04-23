#pragma once

#include "Defines.h"
#include "EigenCompat.h"
#include "SensorSnapshot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace MazeMap
{
    enum class Side : uint8_t
    {
        Left = 0U,
        Right = 1U
    };

    enum class ObsClass : uint8_t
    {
        WallLike = 0U,
        PostLike = 1U,
        OpenLike = 2U,
        Ambiguous = 3U
    };

    struct SensorExtrinsics
    {
        Eigen::Vector2f positionBodyM = Eigen::Vector2f::Zero();
        // Body frame is +X right, +Y forward, so an unconfigured sensor faces forward.
        Eigen::Vector2f directionBody = Eigen::Vector2f(0.0f, 1.0f);
        float yawOffsetRad = 0.0f;
    };

    struct ImuExtrinsics
    {
        Eigen::Vector2f positionBodyM = Eigen::Vector2f::Zero();
        // Accelerometer samples are mapped into the project body frame:
        // +X right, +Y forward.
        Eigen::Matrix2f accelBodyFromImu = Eigen::Matrix2f::Identity();
        float gyroZSign = 1.0f;
    };

    struct ControlInput
    {
        float leftMotorCommand = 0.0f;
        float rightMotorCommand = 0.0f;
        float fanDutyCycle = 0.80f;
        float batteryVoltageV = 0.0f;
    };

    struct EncoderObs
    {
        int32_t totalLeftCounts = 0;
        int32_t totalRightCounts = 0;
        float omegaLeftRadps = 0.0f;
        float omegaRightRadps = 0.0f;
    };

    struct ImuObservation
    {
        bool valid = false;
        float gyroZRadps = 0.0f;
        // Project body-frame acceleration: +X right, +Y forward.
        float accelBodyXMps2 = 0.0f;
        float accelBodyYMps2 = 0.0f;
    };

    using ImuMergedObs = ImuObservation;

    struct ImuAccelObs
    {
        bool valid = false;
        // Project body-frame acceleration: +X right, +Y forward.
        float accelBodyXMps2 = 0.0f;
        float accelBodyYMps2 = 0.0f;
    };

    struct WallObs
    {
        bool valid = false;
        float rho = 0.0f;
        float confidence = 0.0f;
        ObsClass cls = ObsClass::Ambiguous;
    };

    class EXPORT VehicleState
    {
    public:
        static constexpr int kDimension = 9;
        using StateVector = Eigen::Matrix<float, kDimension, 1>;
        using StateMatrix = Eigen::Matrix<float, kDimension, kDimension>;

        // World pose uses +X right and +Y forward.
        // Body velocity uses kV = body-right speed, kU = body-forward speed, and kR = clockwise yaw rate.

        enum Index : int
        {
            kPx = 0,
            kPy = 1,
            kPsi = 2,
            kU = 3,
            kV = 4,
            kR = 5,
            kOmegaL = 6,
            kOmegaR = 7,
            kBgz = 8
        };


        VehicleState() noexcept
            : _state(StateVector::Zero())
            , _sqrtCovariance(StateMatrix::Identity() * 1.0e-3f)
            , _time(0.0f)
            , _control()
        {
        }

        const StateVector& GetStateVector() const noexcept { return _state; }

        void SetStateVector(const StateVector& state) noexcept
        {
            _state = state;
            _state(kPsi) = NormalizeAngle(_state(kPsi));
        }

        const StateMatrix& GetSqrtCovariance() const noexcept { return _sqrtCovariance; }
        void SetSqrtCovariance(const StateMatrix& sqrtCovariance) noexcept { _sqrtCovariance = sqrtCovariance; }

        StateMatrix GetCovariance() const noexcept { return _sqrtCovariance * _sqrtCovariance.transpose(); }
        void SetCovariance(const StateMatrix& covariance) noexcept
        {
            Eigen::LLT<StateMatrix> llt;
            llt.compute(covariance);
            if (llt.info() == Eigen::Success)
            {
                _sqrtCovariance = llt.matrixL();
                return;
            }
            const StateMatrix symmetric = 0.5f * (covariance + covariance.transpose());
            llt.compute(symmetric);
            if (llt.info() == Eigen::Success)
            {
                _sqrtCovariance = llt.matrixL();
                return;
            }
        }

        bool IsStationary() const noexcept;

        // Applies the stationary zero-motion constraint while preserving the caller-provided pose anchor block.
        // Gyro bias remains untouched.
        void ApplyStationaryZeroMotionConstraint(
            bool resetLateralVelocity,
            bool hasPoseReference,
            const StateVector& poseReferenceState,
            const StateMatrix& poseReferenceCovariance) noexcept;

        void SetPosition(const Eigen::Vector2f& position) noexcept { _state(kPx) = position.x(); _state(kPy) = position.y(); }
        float GetPositionX() const noexcept { return _state(kPx); }
        float GetPositionX() noexcept { return const_cast<const VehicleState*>(this)->GetPositionX(); }
        float GetPositionY() const noexcept { return _state(kPy); }
        float GetPositionY() noexcept { return const_cast<const VehicleState*>(this)->GetPositionY(); }
        Eigen::Vector2f GetPosition() const noexcept { return Eigen::Vector2f(_state(kPx), _state(kPy)); }
        Eigen::Vector2f GetPosition() noexcept { return const_cast<const VehicleState*>(this)->GetPosition(); }

        void SetVelocity(float velocity) noexcept { _state(kU) = velocity; }
        float GetVelocity() const noexcept { return _state(kU); }
        float GetVelocity() noexcept { return const_cast<const VehicleState*>(this)->GetVelocity(); }

        void SetLateralVelocity(float velocity) noexcept { _state(kV) = velocity; }
        float GetLateralVelocity() const noexcept { return _state(kV); }
        float GetLateralVelocity() noexcept { return const_cast<const VehicleState*>(this)->GetLateralVelocity(); }

        void SetOrientation(float orientation) noexcept { _state(kPsi) = NormalizeAngle(orientation); }
        float GetOrientation() const noexcept { return _state(kPsi); }
        float GetOrientation() noexcept { return const_cast<const VehicleState*>(this)->GetOrientation(); }
        Eigen::Vector2f GetHeadingUnit() const noexcept
        {
            return Eigen::Vector2f(std::sin(_state(kPsi)), std::cos(_state(kPsi)));
        }
        Eigen::Vector2f GetHeadingUnit() noexcept { return const_cast<const VehicleState*>(this)->GetHeadingUnit(); }

        void SetRotationalVelocity(float rotationalVelocity) noexcept { _state(kR) = rotationalVelocity; }
        float GetRotationalVelocity() const noexcept { return _state(kR); }
        float GetRotationalVelocity() noexcept { return const_cast<const VehicleState*>(this)->GetRotationalVelocity(); }

        void SetWheelSpeedLeft(float wheelSpeedRadps) noexcept { _state(kOmegaL) = wheelSpeedRadps; }
        float GetWheelSpeedLeft() const noexcept { return _state(kOmegaL); }
        float GetWheelSpeedLeft() noexcept { return const_cast<const VehicleState*>(this)->GetWheelSpeedLeft(); }

        void SetWheelSpeedRight(float wheelSpeedRadps) noexcept { _state(kOmegaR) = wheelSpeedRadps; }
        float GetWheelSpeedRight() const noexcept { return _state(kOmegaR); }
        float GetWheelSpeedRight() noexcept { return const_cast<const VehicleState*>(this)->GetWheelSpeedRight(); }

        void SetGyroBiasZ(float gyroBiasRadps) noexcept { _state(kBgz) = gyroBiasRadps; }
        float GetGyroBiasZ() const noexcept { return _state(kBgz); }
        float GetGyroBiasZ() noexcept { return const_cast<const VehicleState*>(this)->GetGyroBiasZ(); }

        void SetTime(float time) noexcept { _time = time; }
        float GetTime() const noexcept { return _time; }
        float GetTime() noexcept { return const_cast<const VehicleState*>(this)->GetTime(); }

        void SetMotorDriveL(float motorDriveL) noexcept { _control.leftMotorCommand = motorDriveL; }
        float GetMotorDriveL() const noexcept { return _control.leftMotorCommand; }
        float GetMotorDriveL() noexcept { return const_cast<const VehicleState*>(this)->GetMotorDriveL(); }

        void SetMotorDriveR(float motorDriveR) noexcept { _control.rightMotorCommand = motorDriveR; }
        float GetMotorDriveR() const noexcept { return _control.rightMotorCommand; }
        float GetMotorDriveR() noexcept { return const_cast<const VehicleState*>(this)->GetMotorDriveR(); }

        void SetFanDutyCycle(float fanDutyCycle) noexcept { _control.fanDutyCycle = fanDutyCycle; }
        float GetFanDutyCycle() const noexcept { return _control.fanDutyCycle; }
        float GetFanDutyCycle() noexcept { return const_cast<const VehicleState*>(this)->GetFanDutyCycle(); }

        void SetControlInput(const ControlInput& control) noexcept { _control = control; }
        const ControlInput& GetControlInput() const noexcept { return _control; }
        const ControlInput& GetControlInput() noexcept { return const_cast<const VehicleState*>(this)->GetControlInput(); }

        void SetSensorSnapshot(const ::SensorSnapshot& sensorSnapshot) noexcept { _sensorSnapshot = sensorSnapshot; }
        const ::SensorSnapshot& GetSensorSnapshot() const noexcept { return _sensorSnapshot; }
        ::SensorSnapshot& GetSensorSnapshot() noexcept { return _sensorSnapshot; }

        VehicleState ProjectConstantVelocity(float dtSeconds) const noexcept
        {
            VehicleState projected = *this;
            if (!std::isfinite(dtSeconds) || (dtSeconds <= 0.0f))
            {
                return projected;
            }

            const float linearSpeedMps = std::isfinite(GetVelocity()) ? GetVelocity() : 0.0f;
            const float yawRateRadps = std::isfinite(GetRotationalVelocity()) ? GetRotationalVelocity() : 0.0f;
            const float midYawRad = NormalizeAngle(GetOrientation() + (0.5f * yawRateRadps * dtSeconds));
            const Eigen::Vector2f midHeading(std::sin(midYawRad), std::cos(midYawRad));
            projected.SetPosition(Eigen::Vector2f(
                GetPositionX() + (linearSpeedMps * midHeading.x() * dtSeconds),
                GetPositionY() + (linearSpeedMps * midHeading.y() * dtSeconds)));
            projected.SetOrientation(GetOrientation() + (yawRateRadps * dtSeconds));
            projected.SetTime(GetTime() + dtSeconds);
            return projected;
        }

        void SetVarianceValues(float xVar, float yVar, float velocityVar, float orientationVar, float rotVelocityVar)
        {
            StateMatrix covariance = GetCovariance();
            covariance(kPx, kPx) = (std::max)(0.0f, xVar);
            covariance(kPy, kPy) = (std::max)(0.0f, yVar);
            covariance(kU, kU) = (std::max)(0.0f, velocityVar);
            covariance(kPsi, kPsi) = (std::max)(0.0f, orientationVar);
            covariance(kR, kR) = (std::max)(0.0f, rotVelocityVar);
            SetCovariance(covariance);
        }

        void SetPositionVar(const Eigen::Vector2f& positionVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kPx, kPx) = (std::max)(0.0f, positionVariance.x());
            covariance(kPy, kPy) = (std::max)(0.0f, positionVariance.y());
            SetCovariance(covariance);
        }

        Eigen::Vector2f GetPositionVar() const noexcept
        {
            const StateMatrix covariance = GetCovariance();
            return Eigen::Vector2f(covariance(kPx, kPx), covariance(kPy, kPy));
        }

        Eigen::Vector2f GetPositionVar() noexcept { return const_cast<const VehicleState*>(this)->GetPositionVar(); }

        void SetVelocityVar(float velocityVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kU, kU) = (std::max)(0.0f, velocityVariance);
            SetCovariance(covariance);
        }

        float GetVelocityVar() const noexcept { return GetCovariance()(kU, kU); }
        float GetVelocityVar() noexcept { return const_cast<const VehicleState*>(this)->GetVelocityVar(); }

        void SetOrientationVar(float orientationVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kPsi, kPsi) = (std::max)(0.0f, orientationVariance);
            SetCovariance(covariance);
        }

        float GetOrientationVar() const noexcept { return GetCovariance()(kPsi, kPsi); }
        float GetOrientationVar() noexcept { return const_cast<const VehicleState*>(this)->GetOrientationVar(); }

        void SetRotationalVelocityVar(float rotationalVelocityVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kR, kR) = (std::max)(0.0f, rotationalVelocityVariance);
            SetCovariance(covariance);
        }

        float GetRotationalVelocityVar() const noexcept { return GetCovariance()(kR, kR); }
        float GetRotationalVelocityVar() noexcept { return const_cast<const VehicleState*>(this)->GetRotationalVelocityVar(); }

        void SetLWheelSpeedVar(float motorDriveLVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kOmegaL, kOmegaL) = (std::max)(0.0f, motorDriveLVariance);
            SetCovariance(covariance);
        }

        float GetLWheelSpeedVar() const noexcept { return GetCovariance()(kOmegaL, kOmegaL); }
        float GetLWheelSpeedVar() noexcept { return const_cast<const VehicleState*>(this)->GetLWheelSpeedVar(); }

        void SetRWheelSpeedVar(float motorDriveRVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kOmegaR, kOmegaR) = (std::max)(0.0f, motorDriveRVariance);
            SetCovariance(covariance);
        }

        float GetRWheelSpeedVar() const noexcept { return GetCovariance()(kOmegaR, kOmegaR); }
        float GetRWheelSpeedVar() noexcept { return const_cast<const VehicleState*>(this)->GetRWheelSpeedVar(); }

        void SetGyroBiasZVar(float gyroBiasVariance) noexcept
        {
            StateMatrix covariance = GetCovariance();
            covariance(kBgz, kBgz) = (std::max)(0.0f, gyroBiasVariance);
            SetCovariance(covariance);
        }

        float GetGyroBiasZVar() const noexcept { return GetCovariance()(kBgz, kBgz); }

        static float NormalizeAngle(float angleRad) noexcept
        {
            if (!std::isfinite(angleRad))
            {
                return 0.0f;
            }

            while (angleRad > PI_F)
            {
                angleRad -= TWO_PI_F;
            }
            while (angleRad <= -PI_F)
            {
                angleRad += TWO_PI_F;
            }
            return angleRad;
        }

        static void NormalizeStateVector(StateVector& state) noexcept
        {
            state(kPsi) = NormalizeAngle(state(kPsi));
        }

    private:
        static bool BuildConstrainedLowerTriangularSquareRoot(
            const StateMatrix& covariance,
            const std::array<bool, kDimension>& exactZeroMask,
            StateMatrix& sqrtCovariance) noexcept;

        StateVector _state;
        StateMatrix _sqrtCovariance;
        float _time;
        ControlInput _control;
        ::SensorSnapshot _sensorSnapshot;
    };
}
