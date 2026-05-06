#include "pch.h"
#include "SensorMount.h"

#include <cmath>

namespace
{
    Eigen::Vector2f DefaultForwardBody() noexcept
    {
        return Eigen::Vector2f(0.0f, 1.0f);
    }

    Eigen::Vector2f NormalizeOrDefault(const Eigen::Vector2f& vector) noexcept
    {
        const float norm = vector.norm();
        if (!std::isfinite(norm) || (norm <= 1.0e-6f))
        {
            return DefaultForwardBody();
        }

        return vector / norm;
    }

    Eigen::Vector2f RightFromForward(const Eigen::Vector2f& forwardBody) noexcept
    {
        return Eigen::Vector2f(forwardBody.y(), -forwardBody.x());
    }

    Eigen::Matrix2f BuildBodyFromSensor(const Eigen::Vector2f& forwardBody) noexcept
    {
        const Eigen::Vector2f normalizedForward = NormalizeOrDefault(forwardBody);
        const Eigen::Vector2f rightBody = RightFromForward(normalizedForward);

        Eigen::Matrix2f bodyFromSensor = Eigen::Matrix2f::Identity();
        bodyFromSensor.col(0) = rightBody;
        bodyFromSensor.col(1) = normalizedForward;
        return bodyFromSensor;
    }

}

namespace MazeMap
{
    SensorMount::SensorMount() noexcept = default;

    SensorMount::SensorMount(
        const Eigen::Vector2f& positionBodyM,
        const Eigen::Matrix2f& bodyFromSensor,
        const float clockwiseYawSign) noexcept
        : _positionBodyM(positionBodyM)
        , _bodyFromSensor(bodyFromSensor)
        , _clockwiseYawSign(std::signbit(clockwiseYawSign) ? -1.0f : 1.0f)
    {
    }

    SensorMount SensorMount::FromForwardDirectionBody(
        const Eigen::Vector2f& positionBodyM,
        const Eigen::Vector2f& forwardBody,
        const float clockwiseYawSign) noexcept
    {
        return SensorMount(positionBodyM, BuildBodyFromSensor(forwardBody), clockwiseYawSign);
    }

    Eigen::Vector2f SensorMount::TransformPlanarVectorToBody(const Eigen::Vector2f& sensorVector) const noexcept
    {
        return _bodyFromSensor * sensorVector;
    }

    Eigen::Vector2f SensorMount::SensorForwardBody() const noexcept
    {
        return TransformPlanarVectorToBody(DefaultForwardBody());
    }

    float SensorMount::TransformClockwiseYawRateToBody(const float sensorYawRateRadps) const noexcept
    {
        return _clockwiseYawSign * sensorYawRateRadps;
    }
}
