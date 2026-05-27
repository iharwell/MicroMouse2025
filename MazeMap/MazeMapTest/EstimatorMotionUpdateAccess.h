#pragma once

#include "EstimatorFilterTestSupport.h"

#include <Eigen/Core>
#include <cmath>
#include <limits>

namespace MazeMap
{
    class EstimatorMotionUpdateTest final
    {
    public:
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix =
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;

        static bool Reset(
            Estimator& core,
            const StateVector& state,
            const CovarianceMatrix& covariance) noexcept
        {
            return core.reset(state, covariance);
        }

        static const StateVector& WorkingState(const Estimator& core) noexcept
        {
            return core.workingState();
        }

        static CovarianceMatrix WorkingCovariance(const Estimator& core) noexcept
        {
            return core.workingCovariance();
        }

        static CovarianceMatrix BuildDefaultInitialCovariance() noexcept
        {
            return Estimator::BuildDefaultInitialCovariance();
        }

        static bool LastUpdateAttempted(const Estimator& core) noexcept
        {
            return core.LastUpdateAttempted();
        }

        static bool LastUpdateAccepted(const Estimator& core) noexcept
        {
            return core.LastUpdateAccepted();
        }

        static float LastUpdateNis(const Estimator& core) noexcept
        {
            return core.LastUpdateNis();
        }
    };

    namespace EstimatorMotionUpdateSupport
    {
        constexpr float kZeroVelocityToleranceMps = 0.008f;
        constexpr int kStationarySplitCommandPredictSteps = 500;
        constexpr float kDefaultEstimatorDtSeconds = 0.001f;

        using CommandVector = App::Internal::CommandVector;
        using StateVector = Eigen::Matrix<float, VehicleState::kDimension, 1>;
        using CovarianceMatrix =
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>;

        struct IndexedDifference final
        {
            float maxAbs = 0.0f;
            int row = -1;
            int col = -1;
            int sampleIndex = -1;
        };

        struct CovarianceEntry final
        {
            int row = 0;
            int col = 0;
        };

        inline void RecordDifference(
            IndexedDifference& difference,
            const float value,
            const int row,
            const int col = -1,
            const int sampleIndex = -1) noexcept
        {
            const float absValue = std::fabs(value);
            if (absValue > difference.maxAbs)
            {
                difference.maxAbs = absValue;
                difference.row = row;
                difference.col = col;
                difference.sampleIndex = sampleIndex;
            }
        }

        inline void RecordDifference(
            IndexedDifference& difference,
            const IndexedDifference& candidate,
            const int sampleIndex = -1) noexcept
        {
            if (candidate.maxAbs > difference.maxAbs)
            {
                difference = candidate;
                difference.sampleIndex = sampleIndex;
            }
        }

        inline StateVector NanState()
        {
            return StateVector::Constant(std::numeric_limits<float>::quiet_NaN());
        }

        inline CovarianceMatrix NanCovariance()
        {
            return CovarianceMatrix::Constant(std::numeric_limits<float>::quiet_NaN());
        }

        template <int Count>
        IndexedDifference MaxStateDifference(
            const StateVector& expected,
            const StateVector& actual,
            const int (&indices)[Count]) noexcept
        {
            IndexedDifference difference;
            for (int index = 0; index < Count; ++index)
            {
                const int row = indices[index];
                RecordDifference(difference, actual(row) - expected(row), row);
            }
            return difference;
        }

        template <int Count>
        IndexedDifference MaxCovarianceDifference(
            const CovarianceMatrix& expected,
            const CovarianceMatrix& actual,
            const CovarianceEntry (&entries)[Count]) noexcept
        {
            IndexedDifference difference;
            for (int index = 0; index < Count; ++index)
            {
                const CovarianceEntry entry = entries[index];
                RecordDifference(
                    difference,
                    actual(entry.row, entry.col) - expected(entry.row, entry.col),
                    entry.row,
                    entry.col);
            }
            return difference;
        }

        inline IndexedDifference MaxBodyStateDifference(
            const Eigen::Vector2f& expected,
            const StateVector& actual) noexcept
        {
            IndexedDifference difference;
            RecordDifference(difference, actual(3) - expected(0), 3);
            RecordDifference(difference, actual(5) - expected(1), 5);
            return difference;
        }

        inline IndexedDifference MaxBodyCovarianceDifference(
            const Eigen::Matrix2f& expected,
            const CovarianceMatrix& actual) noexcept
        {
            IndexedDifference difference;
            RecordDifference(difference, actual(3, 3) - expected(0, 0), 3, 3);
            RecordDifference(difference, actual(3, 5) - expected(0, 1), 3, 5);
            RecordDifference(difference, actual(5, 3) - expected(1, 0), 5, 3);
            RecordDifference(difference, actual(5, 5) - expected(1, 1), 5, 5);
            return difference;
        }

        inline IndexedDifference MaxUnmeasuredStateDifference(
            const StateVector& expected,
            const StateVector& actual) noexcept
        {
            constexpr int kUnmeasuredIndices[] = { 0, 1, 2, 4, 6, 7, 8 };
            return MaxStateDifference(expected, actual, kUnmeasuredIndices);
        }

        inline IndexedDifference MaxAccelResidualDifference(
            const StateVector& expected,
            const StateVector& actual) noexcept
        {
            constexpr int kResidualIndices[] = { 6, 7, 8 };
            return MaxStateDifference(expected, actual, kResidualIndices);
        }

        inline IndexedDifference MaxResidualVarianceDifference(
            const CovarianceMatrix& expected,
            const CovarianceMatrix& actual) noexcept
        {
            constexpr CovarianceEntry kResidualVarianceEntries[] = {
                { 6, 6 },
                { 7, 7 },
                { 8, 8 },
            };
            return MaxCovarianceDifference(expected, actual, kResidualVarianceEntries);
        }

        inline IndexedDifference MaxBodyResidualCrossCovariance(
            const CovarianceMatrix& covariance) noexcept
        {
            IndexedDifference difference;
            constexpr CovarianceEntry kBodyResidualEntries[] = {
                { 3, 6 },
                { 3, 7 },
                { 3, 8 },
                { 5, 6 },
                { 5, 7 },
                { 5, 8 },
            };
            for (const CovarianceEntry entry : kBodyResidualEntries)
            {
                RecordDifference(
                    difference,
                    covariance(entry.row, entry.col),
                    entry.row,
                    entry.col);
            }
            return difference;
        }

        inline void PublishStateToRuntime(
            VehicleState& runtimeState,
            const StateVector& state) noexcept
        {
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                state(3),
                state(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            runtimeState.SetPosition(Eigen::Vector2f(state(0), state(1)));
            runtimeState.SetHeading(state(2));
            runtimeState.SetForwardVelocity(state(3));
            runtimeState.SetRightwardVelocity(state(4));
            runtimeState.SetYawRate(state(5));
            runtimeState.SetForwardAccelerationResidual(state(6));
            runtimeState.SetRightwardAccelerationResidual(state(7));
            runtimeState.SetYawAccelResidual(state(8));
            runtimeState.SetWheelSpeedLeft(leftWheelSpeedRadps);
            runtimeState.SetWheelSpeedRight(rightWheelSpeedRadps);
        }

        inline CovarianceMatrix BuildTightInitialCovariance()
        {
            CovarianceMatrix covariance = CovarianceMatrix::Zero();
            covariance(0, 0) = 0.001f * 0.001f;
            covariance(1, 1) = 0.001f * 0.001f;
            covariance(2, 2) = 0.01f * 0.01f;
            covariance(3, 3) = 0.005f * 0.005f;
            covariance(4, 4) = 0.005f * 0.005f;
            covariance(5, 5) = 1.0f * 1.0f;
            covariance(6, 6) = 0.05f * 0.05f;
            covariance(7, 7) = 0.05f * 0.05f;
            covariance(8, 8) = 0.02f * 0.02f;
            return covariance;
        }

        inline StateVector BuildRestingState(const float forwardVelocityMps = 0.0f)
        {
            StateVector state = StateVector::Zero();
            state(1) = 0.09f;
            state(3) = forwardVelocityMps;
            return state;
        }
    }
}
