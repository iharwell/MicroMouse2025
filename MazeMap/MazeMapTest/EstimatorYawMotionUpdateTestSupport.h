#pragma once

#include "EstimatorMotionUpdateAccess.h"

#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        inline IndexedDifference MaxYawResidualCrossCovariance(
            const CovarianceMatrix& covariance) noexcept
        {
            IndexedDifference difference;
            constexpr CovarianceEntry kYawResidualEntries[] = {
                { 5, 6 },
                { 6, 5 },
                { 5, 7 },
                { 7, 5 },
                { 5, 8 },
                { 8, 5 },
            };
            for (const CovarianceEntry entry : kYawResidualEntries)
            {
                RecordDifference(
                    difference,
                    covariance(entry.row, entry.col),
                    entry.row,
                    entry.col);
            }
            return difference;
        }

        inline IndexedDifference MaxYawResidualRowCovariance(
            const CovarianceMatrix& covariance) noexcept
        {
            IndexedDifference difference;
            constexpr CovarianceEntry kYawResidualEntries[] = {
                { 5, 6 },
                { 5, 7 },
                { 5, 8 },
            };
            for (const CovarianceEntry entry : kYawResidualEntries)
            {
                RecordDifference(
                    difference,
                    covariance(entry.row, entry.col),
                    entry.row,
                    entry.col);
            }
            return difference;
        }

        struct YawRateExpectation final
        {
            float yawRateRadps = std::numeric_limits<float>::quiet_NaN();
            float yawVarianceRadps2 = std::numeric_limits<float>::quiet_NaN();
            float nis = std::numeric_limits<float>::quiet_NaN();
        };

        YawRateExpectation ComputeYawRateExpectation(
            EstimatorTestRuntime& runtime,
            const StateVector& before,
            const CovarianceMatrix& beforeCovariance,
            float observedYawRateRadps);

        struct MovingPredictResidualScenario final
        {
            bool resetAccepted = false;
            bool predictAccepted = false;
            StateVector initialState = StateVector::Zero();
            StateVector predictedState = NanState();
        };

        struct LaunchEncoderSample final
        {
            float dtSeconds = 0.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            float correctedYawRateRadps = 0.0f;
        };

        inline constexpr int kLaunchEncoderSampleCount = 10;

        struct LaunchEncoderScenario final
        {
            bool resetAccepted = false;
            int completedEncoderSamples = 0;
            int completedYawSamples = 0;
            const wchar_t* firstIncompleteOperation = nullptr;
            IndexedDifference encoderBodyStateDifference{};
            IndexedDifference encoderUnmeasuredStateDifference{};
            IndexedDifference encoderBodyCovarianceDifference{};
            IndexedDifference encoderCrossCovarianceDifference{};
            IndexedDifference encoderResidualVarianceDifference{};
            IndexedDifference yawRateDifference{};
            IndexedDifference yawUnmeasuredStateDifference{};
            IndexedDifference yawVarianceDifference{};
            IndexedDifference yawCrossCovarianceDifference{};
            IndexedDifference yawNisDifference{};
        };

        struct YawRateUpdateScenario final
        {
            bool resetAccepted = false;
            bool updateAttempted = false;
            bool updateReturnedAccepted = false;
            bool updateRecordedAccepted = false;
            float observedYawRateRadps = std::numeric_limits<float>::quiet_NaN();
            StateVector beforeState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforeCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
            YawRateExpectation expectation{};
            float actualNis = std::numeric_limits<float>::quiet_NaN();
        };

        MovingPredictResidualScenario RunMovingPredictResidualScenario();
        LaunchEncoderScenario RunLaunchEncoderScenario();
        YawRateUpdateScenario RunYawResidualCrossScenario();
        YawRateUpdateScenario RunCorrectedYawRateScenario();
    }
}
