#pragma once

#include "Defines.h"
#include "EigenCompat.h"

namespace MazeMap
{
    // Rigid planar mounting relationship between one sensor's local frame and the vehicle body frame.
    // This one concept owns both wall-sensor and IMU placement because both are just fixed sensor mounts.
    class EXPORT SensorMount final
    {
    public:
        SensorMount() noexcept;
        SensorMount(
            const Eigen::Vector2f& positionBodyM,
            const Eigen::Matrix2f& bodyFromSensor,
            float clockwiseYawSign = 1.0f) noexcept;

        static SensorMount FromForwardDirectionBody(
            const Eigen::Vector2f& positionBodyM,
            const Eigen::Vector2f& forwardBody,
            float clockwiseYawSign = 1.0f) noexcept;

        const Eigen::Vector2f& positionBodyM() const noexcept { return _positionBodyM; }
        const Eigen::Matrix2f& bodyFromSensor() const noexcept { return _bodyFromSensor; }
        float clockwiseYawSign() const noexcept { return _clockwiseYawSign; }

        Eigen::Vector2f TransformPlanarVectorToBody(const Eigen::Vector2f& sensorVector) const noexcept;
        Eigen::Vector2f SensorForwardBody() const noexcept;
        float TransformClockwiseYawRateToBody(float sensorYawRateRadps) const noexcept;

    private:
        Eigen::Vector2f _positionBodyM = Eigen::Vector2f::Zero();
        Eigen::Matrix2f _bodyFromSensor = Eigen::Matrix2f::Identity();
        float _clockwiseYawSign = 1.0f;
    };
}
