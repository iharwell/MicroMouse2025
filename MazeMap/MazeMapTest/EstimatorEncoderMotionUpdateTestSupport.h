#pragma once

#include "EstimatorMotionUpdateAccess.h"

#include <Eigen/Core>
#include <limits>

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        struct EncoderPairExpectation final
        {
            Eigen::Vector2f bodyState = Eigen::Vector2f::Constant(
                std::numeric_limits<float>::quiet_NaN());
            Eigen::Matrix2f bodyCovariance = Eigen::Matrix2f::Constant(
                std::numeric_limits<float>::quiet_NaN());
            float nis = std::numeric_limits<float>::quiet_NaN();
        };

        EncoderPairExpectation ComputeEncoderPairExpectation(
            PlantModel& plant,
            const EncoderObs& encoder,
            const StateVector& priorState,
            const CovarianceMatrix& priorCovariance,
            float linearSpeedSigmaMps,
            float yawRateSigmaRadps);

        struct EncoderPairUpdateScenario final
        {
            bool resetAccepted = false;
            bool predictAccepted = false;
            bool updateAttempted = false;
            bool updateReturnedAccepted = false;
            bool updateRecordedAccepted = false;
            StateVector initialState = StateVector::Zero();
            StateVector priorState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforePredictCovariance = NanCovariance();
            CovarianceMatrix priorCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
            EncoderPairExpectation expectation{};
            float actualNis = std::numeric_limits<float>::quiet_NaN();
            float initialYawRateVarianceRadps2 = std::numeric_limits<float>::quiet_NaN();
            float forwardContactForceN = std::numeric_limits<float>::quiet_NaN();
            float rightContactForceN = std::numeric_limits<float>::quiet_NaN();
            float runtimeForwardAccelerationMps2 = std::numeric_limits<float>::quiet_NaN();
            float runtimeRightAccelerationMps2 = std::numeric_limits<float>::quiet_NaN();
            float runtimeForwardVelocityMps = std::numeric_limits<float>::quiet_NaN();
            float runtimeRightVelocityMps = std::numeric_limits<float>::quiet_NaN();
            float runtimeYawRateRadps = std::numeric_limits<float>::quiet_NaN();
            float expectedMeasuredLinearSpeedMps = std::numeric_limits<float>::quiet_NaN();
            float measuredLinearSpeedMps = std::numeric_limits<float>::quiet_NaN();
            float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN();
            float measuredYawRateVarianceRadps2 = std::numeric_limits<float>::quiet_NaN();
            float measuredWheelVarianceRadps2 = std::numeric_limits<float>::quiet_NaN();
        };

        struct LatestEncoderScenario final
        {
            bool firstPredictAccepted = false;
            bool firstUpdateAttempted = false;
            bool firstUpdateReturnedAccepted = false;
            bool firstUpdateRecordedAccepted = false;
            bool secondPredictAccepted = false;
            bool secondUpdateAttempted = false;
            bool secondUpdateReturnedAccepted = false;
            bool secondUpdateRecordedAccepted = false;
            StateVector beforeSecondState = NanState();
            StateVector afterSecondState = NanState();
            CovarianceMatrix beforeSecondCovariance = NanCovariance();
            CovarianceMatrix afterSecondCovariance = NanCovariance();
            EncoderPairExpectation expectation{};
            float secondActualNis = std::numeric_limits<float>::quiet_NaN();
        };

        struct DirectEncoderScenario final
        {
            bool resetAccepted = false;
            bool updateAttempted = false;
            bool updateReturnedAccepted = false;
            bool updateRecordedAccepted = false;
            StateVector beforeState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforeCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
            EncoderPairExpectation expectation{};
            float actualNis = std::numeric_limits<float>::quiet_NaN();
        };

        struct RejectedEncoderScenario final
        {
            bool resetAccepted = false;
            bool updateAttempted = false;
            bool updateReturnedAccepted = true;
            bool updateRecordedAccepted = true;
            float nis = std::numeric_limits<float>::quiet_NaN();
            StateVector beforeState = NanState();
            StateVector afterState = NanState();
            CovarianceMatrix beforeCovariance = NanCovariance();
            CovarianceMatrix afterCovariance = NanCovariance();
        };

        EncoderPairUpdateScenario RunZeroVelocityEncoderPairScenario();
        EncoderPairUpdateScenario RunMovingEncoderPairScenario();
        LatestEncoderScenario RunLatestEncoderScenario();
        DirectEncoderScenario RunDirectEncoderScenario();
        RejectedEncoderScenario RunRejectedEncoderScenario();
    }
}
