#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorEncoderMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorMovingEncoderUpdateTest)
    {
    public:
        TEST_METHOD(MovingEncoderResetAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected the moving encoder scenario");
        }

        TEST_METHOD(MovingEncoderMeasuredLinearSpeedMatchesCounts)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();

            Assert::AreEqual(scenario.expectedMeasuredLinearSpeedMps, scenario.measuredLinearSpeedMps, 1.0e-6f, L"measured linear speed");
        }

        TEST_METHOD(MovingEncoderMeasuredYawRateIsZero)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();

            Assert::AreEqual(0.0f, scenario.measuredYawRateRadps, 1.0e-6f, L"measured yaw rate");
        }

        TEST_METHOD(MovingEncoderYawRateVarianceIsPositive)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const std::wstring message =
                std::wstring(L"measured yaw rate variance") +
                L" actual=" +
                std::to_wstring(scenario.measuredYawRateVarianceRadps2) +
                L", limit=" +
                std::to_wstring(0.0f);

            Assert::IsTrue(scenario.measuredYawRateVarianceRadps2 > 0.0f, message.c_str());
        }

        TEST_METHOD(MovingEncoderWheelVarianceIsPositive)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const std::wstring message =
                std::wstring(L"measured wheel variance") +
                L" actual=" +
                std::to_wstring(scenario.measuredWheelVarianceRadps2) +
                L", limit=" +
                std::to_wstring(0.0f);

            Assert::IsTrue(scenario.measuredWheelVarianceRadps2 > 0.0f, message.c_str());
        }

        TEST_METHOD(MovingEncoderPredictAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();

            Assert::IsTrue(scenario.predictAccepted, L"predict rejected the moving encoder scenario");
        }

        TEST_METHOD(MovingEncoderUpdateAccepted)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"moving encoder update was rejected");
        }

        TEST_METHOD(MovingEncoderBodyStateMatchesModel)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const IndexedDifference difference =
                MaxBodyStateDifference(scenario.expectation.bodyState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"moving encoder body state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-6f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(MovingEncoderKeepsUnmeasuredState)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const IndexedDifference difference =
                MaxUnmeasuredStateDifference(scenario.priorState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"moving encoder unmeasured state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-6f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(MovingEncoderBodyCovarianceMatchesModel)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const IndexedDifference difference =
                MaxBodyCovarianceDifference(
                    scenario.expectation.bodyCovariance,
                    scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"moving encoder body covariance") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-7f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-7f, message.c_str());
        }

        TEST_METHOD(MovingEncoderDoesNotIncreaseYawRateVariance)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const std::wstring message =
                std::wstring(L"after yaw variance=") +
                std::to_wstring(scenario.afterCovariance(5, 5)) +
                L", prior=" +
                std::to_wstring(scenario.priorCovariance(5, 5));

            Assert::IsTrue(scenario.afterCovariance(5, 5) <= (scenario.priorCovariance(5, 5) + 1.0e-9f), message.c_str());
        }

        TEST_METHOD(MovingEncoderClearsBodyResidualCrossCovariance)
        {
            const EncoderPairUpdateScenario scenario = RunMovingEncoderPairScenario();
            const IndexedDifference difference =
                MaxBodyResidualCrossCovariance(scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"moving body/residual covariance") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-8f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-8f, message.c_str());
        }

    };
}
