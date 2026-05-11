#include "pch.h"
#include "VehicleState.h"

#include "Vehicle.h"

namespace
{
    float UkfStationaryLinearSpeedThresholdMps() noexcept
    {
        return 0.002936f;
    }

    float UkfStationaryYawRateThresholdRadps() noexcept
    {
        return 3.0f * 0.0010954451f;
    }

    float UkfStationaryWheelSpeedThresholdRadps() noexcept
    {
        constexpr float wheelRadiusM = MazeMap::Vehicle::GetDriveWheelRadiusM();
        if (!std::isfinite(wheelRadiusM) || !(wheelRadiusM > 0.0f))
        {
            return 0.0f;
        }

        return UkfStationaryLinearSpeedThresholdMps() / wheelRadiusM;
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
            (std::fabs(_state(kU)) <= UkfStationaryLinearSpeedThresholdMps()) &&
            (std::fabs(_state(kV)) <= UkfStationaryLinearSpeedThresholdMps()) &&
            (std::fabs(_state(kR)) <= UkfStationaryYawRateThresholdRadps()) &&
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
        bool resetLateralVelocity,
        bool hasPoseReference,
        const StateVector& poseReferenceState,
        const StateMatrix& poseReferenceCovariance) noexcept
    {
        StateVector constrainedState = _state;
        if (hasPoseReference)
        {
            constrainedState(kPx) = poseReferenceState(kPx);
            constrainedState(kPy) = poseReferenceState(kPy);
            constrainedState(kPsi) = poseReferenceState(kPsi);
        }

        constrainedState(kU) = 0.0f;
        if (resetLateralVelocity)
        {
            constrainedState(kV) = 0.0f;
        }
        constrainedState(kR) = 0.0f;
        constrainedState(kOmegaL) = 0.0f;
        constrainedState(kOmegaR) = 0.0f;
        NormalizeStateVector(constrainedState);

        StateMatrix constrainedCovariance = GetCovariance();
        if (hasPoseReference)
        {
            constexpr std::array<int, 3> poseIndices = { kPx, kPy, kPsi };
            for (const int poseIndex : poseIndices)
            {
                constrainedCovariance.row(poseIndex).setZero();
                constrainedCovariance.col(poseIndex).setZero();
            }
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
