#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorYawResidualCrossCovarianceTest)
    {
    public:
        TEST_METHOD(YawResidualCrossResetAccepted)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected yaw residual cross scenario");
        }

        TEST_METHOD(YawResidualCrossUpdateAccepted)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"yaw update was rejected");
        }

        TEST_METHOD(YawResidualCrossNisMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();

            Assert::AreEqual(scenario.expectation.nis, scenario.actualNis, 1.0e-4f, L"yaw update NIS");
        }

        TEST_METHOD(YawResidualCrossRateMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();

            Assert::AreEqual(scenario.expectation.yawRateRadps, scenario.afterState(5), 1.0e-6f, L"yaw rate after update");
        }

        TEST_METHOD(YawResidualCrossVarianceMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();

            Assert::AreEqual(scenario.expectation.yawVarianceRadps2, scenario.afterCovariance(5, 5), 1.0e-7f, L"yaw variance after update");
        }

        TEST_METHOD(YawResidualCrossKeepsResidualState)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();
            const IndexedDifference difference =
                MaxAccelResidualDifference(scenario.beforeState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"yaw residual state") +
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

        TEST_METHOD(YawResidualCrossKeepsResidualVariance)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();
            const IndexedDifference difference =
                MaxResidualVarianceDifference(scenario.beforeCovariance, scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"yaw residual variance") +
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

        TEST_METHOD(YawResidualCrossClearsResidualCovariance)
        {
            const YawRateUpdateScenario scenario = RunYawResidualCrossScenario();
            const IndexedDifference difference =
                MaxYawResidualCrossCovariance(scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"yaw/residual covariance") +
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

    };
}
