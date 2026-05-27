#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorEncoderMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorRejectedEncoderUpdateTest)
    {
    public:
        TEST_METHOD(RejectedEncoderResetAccepted)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected the rejected-encoder scenario");
        }

        TEST_METHOD(RejectedEncoderUpdateAttempted)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();

            Assert::IsTrue(scenario.updateAttempted, L"rejected encoder update was not attempted");
        }

        TEST_METHOD(RejectedEncoderUpdateReturnsRejected)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();

            Assert::IsFalse(scenario.updateReturnedAccepted, L"encoder outlier update returned accepted");
        }

        TEST_METHOD(RejectedEncoderUpdateRecordedRejected)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();

            Assert::IsFalse(scenario.updateRecordedAccepted, L"encoder outlier update recorded accepted");
        }

        TEST_METHOD(RejectedEncoderNisExceedsThreshold)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();
            const std::wstring message =
                std::wstring(L"NIS=") +
                std::to_wstring(scenario.nis) +
                L", threshold=" +
                std::to_wstring(kEstimatorTestEncoderPairNisThreshold);

            Assert::IsTrue(scenario.nis > kEstimatorTestEncoderPairNisThreshold, message.c_str());
        }

        TEST_METHOD(RejectedEncoderKeepsState)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();
            constexpr int kStateIndices[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
            const IndexedDifference difference =
                MaxStateDifference(scenario.beforeState, scenario.afterState, kStateIndices);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"rejected encoder state") +
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

        TEST_METHOD(RejectedEncoderKeepsCovariance)
        {
            const RejectedEncoderScenario scenario = RunRejectedEncoderScenario();
            IndexedDifference difference;
            for (int row = 0; row < VehicleState::kDimension; ++row)
            {
                for (int col = 0; col < VehicleState::kDimension; ++col)
                {
                    RecordDifference(
                        difference,
                        scenario.afterCovariance(row, col) - scenario.beforeCovariance(row, col),
                        row,
                        col);
                }
            }
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"rejected encoder covariance") +
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
