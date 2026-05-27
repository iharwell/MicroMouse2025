#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorBiasedYawRateUpdateTest)
    {
    public:
        TEST_METHOD(BiasedYawRateUpdateAccepted)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"biased yaw update was rejected");
        }

        TEST_METHOD(BiasedYawRateNisMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();

            Assert::AreEqual(scenario.expectation.nis, scenario.actualNis, 1.0e-4f, L"biased yaw NIS");
        }

        TEST_METHOD(BiasedYawRateMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();

            Assert::AreEqual(scenario.expectation.yawRateRadps, scenario.afterState(5), 1.0e-6f, L"biased yaw rate");
        }

        TEST_METHOD(BiasedYawVarianceMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();

            Assert::AreEqual(scenario.expectation.yawVarianceRadps2, scenario.afterCovariance(5, 5), 1.0e-7f, L"biased yaw variance");
        }

        TEST_METHOD(BiasedYawKeepsGyroBias)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();

            Assert::AreEqual(scenario.beforeGyroBiasRadps, scenario.afterGyroBiasRadps, 1.0e-6f, L"gyro bias after biased yaw update");
        }

        TEST_METHOD(BiasedYawKeepsUnmeasuredState)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();
            constexpr int kUnmeasuredIndices[] = { 0, 1, 2, 3, 4, 6, 7, 8 };
            const IndexedDifference difference =
                MaxStateDifference(scenario.beforeState, scenario.afterState, kUnmeasuredIndices);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"biased yaw unmeasured state") +
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

        TEST_METHOD(BiasedYawClearsResidualCrossCovariance)
        {
            const YawRateUpdateScenario scenario = RunBiasedYawRateScenario();
            const IndexedDifference difference =
                MaxYawResidualRowCovariance(scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"biased yaw/residual covariance") +
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
