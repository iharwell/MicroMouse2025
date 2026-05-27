#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorEncoderMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorDirectEncoderUpdateTest)
    {
    public:
        TEST_METHOD(DirectEncoderResetAccepted)
        {
            const DirectEncoderScenario scenario = RunDirectEncoderScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected the direct encoder scenario");
        }

        TEST_METHOD(DirectEncoderUpdateAccepted)
        {
            const DirectEncoderScenario scenario = RunDirectEncoderScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"direct encoder update was rejected");
        }

        TEST_METHOD(DirectEncoderBodyStateMatchesModel)
        {
            const DirectEncoderScenario scenario = RunDirectEncoderScenario();
            const IndexedDifference difference =
                MaxBodyStateDifference(scenario.expectation.bodyState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"direct encoder body state") +
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

        TEST_METHOD(DirectEncoderKeepsUnmeasuredState)
        {
            const DirectEncoderScenario scenario = RunDirectEncoderScenario();
            const IndexedDifference difference =
                MaxUnmeasuredStateDifference(scenario.beforeState, scenario.afterState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"direct encoder unmeasured state") +
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

        TEST_METHOD(DirectEncoderBodyCovarianceMatchesModel)
        {
            const DirectEncoderScenario scenario = RunDirectEncoderScenario();
            const IndexedDifference difference =
                MaxBodyCovarianceDifference(
                    scenario.expectation.bodyCovariance,
                    scenario.afterCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"direct encoder body covariance") +
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

    };
}
