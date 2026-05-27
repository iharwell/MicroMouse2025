#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorEncoderMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorLatestEncoderUpdateTest)
    {
    public:
        TEST_METHOD(LatestEncoderFirstPredictAccepted)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();

            Assert::IsTrue(scenario.firstPredictAccepted, L"first predict was rejected");
        }

        TEST_METHOD(LatestEncoderFirstUpdateAccepted)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();

            Assert::IsTrue(scenario.firstUpdateReturnedAccepted, L"first encoder update was rejected");
        }

        TEST_METHOD(LatestEncoderSecondPredictAccepted)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();

            Assert::IsTrue(scenario.secondPredictAccepted, L"second predict was rejected");
        }

        TEST_METHOD(LatestEncoderSecondUpdateAccepted)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();

            Assert::IsTrue(scenario.secondUpdateReturnedAccepted, L"second encoder update was rejected");
        }

        TEST_METHOD(LatestEncoderBodyStateMatchesLatestMeasurement)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();
            const IndexedDifference difference =
                MaxBodyStateDifference(scenario.expectation.bodyState, scenario.afterSecondState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"latest encoder body state") +
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

        TEST_METHOD(LatestEncoderKeepsUnmeasuredState)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();
            const IndexedDifference difference =
                MaxUnmeasuredStateDifference(scenario.beforeSecondState, scenario.afterSecondState);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"latest encoder unmeasured state") +
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

        TEST_METHOD(LatestEncoderBodyCovarianceMatchesModel)
        {
            const LatestEncoderScenario scenario = RunLatestEncoderScenario();
            const IndexedDifference difference =
                MaxBodyCovarianceDifference(
                    scenario.expectation.bodyCovariance,
                    scenario.afterSecondCovariance);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"latest encoder body covariance") +
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
