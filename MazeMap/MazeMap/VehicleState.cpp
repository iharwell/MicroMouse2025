#include "pch.h"
#include "VehicleState.h"

#include "PlantModel.h"

namespace
{
    constexpr float kUkfStationaryLinearSpeedThresholdMps = 1.76e-6f;
    constexpr float kUkfStationaryYawRateThresholdRadps = 3.0f * 0.0028f;

    Eigen::Vector2f HeadingUnitFromYaw(float yawRad) noexcept
    {
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(yawRad, s, c);
        return Eigen::Vector2f(s, c);
    }

    bool CanAnchorPoseFromEncoderReference(
        bool hasPoseReference,
        float distancePerEncoderCountM,
        float trackWidthM) noexcept
    {
        return
            hasPoseReference &&
            std::isfinite(distancePerEncoderCountM) &&
            (distancePerEncoderCountM > 0.0f) &&
            std::isfinite(trackWidthM) &&
            (trackWidthM > 0.0f);
    }

    float UkfStationaryWheelSpeedThresholdRadps() noexcept
    {
        const MazeMap::PlantParams params = MazeMap::PlantParams::Default();
        if (!std::isfinite(params.wheelRadiusM) || !(params.wheelRadiusM > 0.0f))
        {
            return 0.0f;
        }

        return kUkfStationaryLinearSpeedThresholdMps / params.wheelRadiusM;
    }
}

namespace MazeMap
{
    bool VehicleState::IsStationary() const noexcept
    {
        const float wheelSpeedThresholdRadps = UkfStationaryWheelSpeedThresholdRadps();
        return
            std::isfinite(_state(kU)) &&
            std::isfinite(_state(kV)) &&
            std::isfinite(_state(kR)) &&
            std::isfinite(_state(kOmegaL)) &&
            std::isfinite(_state(kOmegaR)) &&
            (std::fabs(_state(kU)) <= kUkfStationaryLinearSpeedThresholdMps) &&
            (std::fabs(_state(kV)) <= kUkfStationaryLinearSpeedThresholdMps) &&
            (std::fabs(_state(kR)) <= kUkfStationaryYawRateThresholdRadps) &&
            (std::fabs(_state(kOmegaL)) <= wheelSpeedThresholdRadps) &&
            (std::fabs(_state(kOmegaR)) <= wheelSpeedThresholdRadps);
    }

    bool VehicleState::BuildConstrainedLowerTriangularSquareRoot(
        const StateMatrix& covariance,
        const std::array<bool, kDimension>& exactZeroMask,
        StateMatrix& sqrtCovariance) noexcept
    {
        sqrtCovariance.setZero();

        std::array<int, kDimension> activeIndices{};
        int activeCount = 0;
        for (int index = 0; index < kDimension; ++index)
        {
            if (!exactZeroMask[static_cast<std::size_t>(index)])
            {
                activeIndices[static_cast<std::size_t>(activeCount)] = index;
                ++activeCount;
            }
        }

        if (activeCount == 0)
        {
            return true;
        }

        Eigen::MatrixXf activeCovariance = Eigen::MatrixXf::Zero(activeCount, activeCount);
        for (int row = 0; row < activeCount; ++row)
        {
            const int globalRow = activeIndices[static_cast<std::size_t>(row)];
            for (int col = 0; col < activeCount; ++col)
            {
                const int globalCol = activeIndices[static_cast<std::size_t>(col)];
                activeCovariance(row, col) = covariance(globalRow, globalCol);
            }
        }

        activeCovariance = 0.5f * (activeCovariance + activeCovariance.transpose());
        if (!activeCovariance.allFinite())
        {
            return false;
        }

        const Eigen::LLT<Eigen::MatrixXf> llt(activeCovariance);
        if (llt.info() != Eigen::Success)
        {
            return false;
        }

        const Eigen::MatrixXf activeSqrt = llt.matrixL();
        for (int row = 0; row < activeCount; ++row)
        {
            const int globalRow = activeIndices[static_cast<std::size_t>(row)];
            for (int col = 0; col <= row; ++col)
            {
                const int globalCol = activeIndices[static_cast<std::size_t>(col)];
                sqrtCovariance(globalRow, globalCol) = activeSqrt(row, col);
            }
        }

        return true;
    }

    void VehicleState::ApplyStationaryZeroMotionConstraint(
        const EncoderObs& encoderObservation,
        float yawRateRadps,
        bool resetLateralVelocity,
        bool hasPoseReference,
        const StateVector& poseReferenceState,
        const StateMatrix& poseReferenceCovariance,
        float distancePerEncoderCountM,
        float trackWidthM) noexcept
    {
        StateVector constrainedState = _state;
        if (CanAnchorPoseFromEncoderReference(hasPoseReference, distancePerEncoderCountM, trackWidthM))
        {
            const float leftDistanceM =
                static_cast<float>(encoderObservation.totalLeftCounts) * distancePerEncoderCountM;
            const float rightDistanceM =
                static_cast<float>(encoderObservation.totalRightCounts) * distancePerEncoderCountM;
            const float forwardDistanceM = 0.5f * (leftDistanceM + rightDistanceM);
            const float deltaYawRad = (leftDistanceM - rightDistanceM) / trackWidthM;
            const float referenceYawRad = poseReferenceState(kPsi);
            const float translationYawRad = NormalizeAngle(referenceYawRad + (0.5f * deltaYawRad));
            const Eigen::Vector2f heading = HeadingUnitFromYaw(translationYawRad);
            constrainedState(kPx) = poseReferenceState(kPx) + (forwardDistanceM * heading.x());
            constrainedState(kPy) = poseReferenceState(kPy) + (forwardDistanceM * heading.y());
            constrainedState(kPsi) = NormalizeAngle(referenceYawRad + deltaYawRad);
        }

        constrainedState(kU) = 0.0f;
        if (resetLateralVelocity)
        {
            constrainedState(kV) = 0.0f;
        }
        constrainedState(kR) = 0.0f;
        constrainedState(kOmegaL) = 0.0f;
        constrainedState(kOmegaR) = 0.0f;
        constrainedState(kBgz) = yawRateRadps;
        NormalizeStateVector(constrainedState);

        StateMatrix constrainedCovariance = GetCovariance();
        if (hasPoseReference)
        {
            constexpr std::array<int, 3> poseIndices = { kPx, kPy, kPsi };
            for (const int row : poseIndices)
            {
                for (const int col : poseIndices)
                {
                    constrainedCovariance(row, col) = poseReferenceCovariance(row, col);
                }
            }
        }

        const std::array<int, 4> constrainedIndices = { kU, kR, kOmegaL, kOmegaR };
        for (const int index : constrainedIndices)
        {
            constrainedCovariance.row(index).setZero();
            constrainedCovariance.col(index).setZero();
        }
        constrainedCovariance.row(kBgz).setZero();
        constrainedCovariance.col(kBgz).setZero();
        if (resetLateralVelocity)
        {
            constrainedCovariance.row(kV).setZero();
            constrainedCovariance.col(kV).setZero();
        }

        std::array<bool, kDimension> exactZeroMask = {};
        exactZeroMask[kU] = true;
        if (resetLateralVelocity)
        {
            exactZeroMask[kV] = true;
        }
        exactZeroMask[kR] = true;
        exactZeroMask[kOmegaL] = true;
        exactZeroMask[kOmegaR] = true;
        exactZeroMask[kBgz] = true;

        StateMatrix constrainedSqrt = StateMatrix::Zero();
        if (BuildConstrainedLowerTriangularSquareRoot(
                constrainedCovariance,
                exactZeroMask,
                constrainedSqrt))
        {
            _state = constrainedState;
            _sqrtCovariance = constrainedSqrt;
            return;
        }

        for (int index = 0; index < kDimension; ++index)
        {
            if (exactZeroMask[static_cast<std::size_t>(index)])
            {
                constrainedCovariance(index, index) = (std::max)(constrainedCovariance(index, index), 1.0e-12f);
            }
        }
        SetStateVector(constrainedState);
        SetCovariance(constrainedCovariance);
    }
}
