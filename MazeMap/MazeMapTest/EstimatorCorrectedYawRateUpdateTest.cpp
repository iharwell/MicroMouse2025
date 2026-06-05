#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorCorrectedYawRateUpdateTest)
    {
    public:
        TEST_METHOD(CorrectedYawRateUpdateAccepted)
        {
            const YawRateUpdateScenario scenario = RunCorrectedYawRateScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"corrected yaw update was rejected");
        }

        TEST_METHOD(CorrectedYawRateNisMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunCorrectedYawRateScenario();

            Assert::AreEqual(scenario.expectation.nis, scenario.actualNis, 1.0e-4f, L"corrected yaw NIS");
        }

        TEST_METHOD(CorrectedYawRateMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunCorrectedYawRateScenario();

            Assert::AreEqual(scenario.expectation.yawRateRadps, scenario.afterState(5), 1.0e-6f, L"corrected yaw rate");
        }

        TEST_METHOD(CorrectedYawVarianceMatchesModel)
        {
            const YawRateUpdateScenario scenario = RunCorrectedYawRateScenario();

            Assert::AreEqual(scenario.expectation.yawVarianceRadps2, scenario.afterCovariance(5, 5), 1.0e-7f, L"corrected yaw variance");
        }

        TEST_METHOD(CorrectedYawKeepsUnmeasuredState)
        {
            const YawRateUpdateScenario scenario = RunCorrectedYawRateScenario();
            constexpr int kUnmeasuredIndices[] = { 0, 1, 2, 3, 4, 6, 7, 8 };
            const IndexedDifference difference =
                MaxStateDifference(scenario.beforeState, scenario.afterState, kUnmeasuredIndices);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"corrected yaw unmeasured state") +
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

        TEST_METHOD(CorrectedYawClearsResidualCrossCovariance)
        {
            const YawRateUpdateScenario scenario = RunCorrectedYawRateScenario();
            const IndexedDifference difference =
                MaxYawResidualRowCovariance(scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"corrected yaw/residual covariance") +
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
