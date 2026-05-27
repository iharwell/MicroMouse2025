#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorLaunchEncoderYawUpdateTest)
    {
    public:
        TEST_METHOD(LaunchEncoderResetAccepted)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected the launch encoder scenario");
        }

        TEST_METHOD(LaunchEncoderBodyStateMatchesModel)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.encoderBodyStateDifference;
            std::wstring message =
                std::wstring(L"launch encoder body state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-5f);
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

            Assert::IsTrue(scenario.encoderBodyStateDifference.maxAbs <= 1.0e-5f, message.c_str());
        }

        TEST_METHOD(LaunchEncoderKeepsUnmeasuredState)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.encoderUnmeasuredStateDifference;
            std::wstring message =
                std::wstring(L"launch encoder unmeasured state") +
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

            Assert::IsTrue(scenario.encoderUnmeasuredStateDifference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LaunchEncoderBodyCovarianceMatchesModel)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.encoderBodyCovarianceDifference;
            std::wstring message =
                std::wstring(L"launch encoder body covariance") +
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

            Assert::IsTrue(scenario.encoderBodyCovarianceDifference.maxAbs <= 1.0e-7f, message.c_str());
        }

        TEST_METHOD(LaunchEncoderClearsBodyResidualCrossCovariance)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.encoderCrossCovarianceDifference;
            std::wstring message =
                std::wstring(L"launch body/residual covariance") +
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

            Assert::IsTrue(scenario.encoderCrossCovarianceDifference.maxAbs <= 1.0e-8f, message.c_str());
        }

        TEST_METHOD(LaunchEncoderKeepsResidualVariance)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.encoderResidualVarianceDifference;
            std::wstring message =
                std::wstring(L"launch residual variance") +
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

            Assert::IsTrue(scenario.encoderResidualVarianceDifference.maxAbs <= 1.0e-8f, message.c_str());
        }

        TEST_METHOD(LaunchYawRateMatchesModel)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.yawRateDifference;
            std::wstring message =
                std::wstring(L"launch yaw rate") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-5f);
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

            Assert::IsTrue(scenario.yawRateDifference.maxAbs <= 1.0e-5f, message.c_str());
        }

        TEST_METHOD(LaunchYawKeepsUnmeasuredState)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.yawUnmeasuredStateDifference;
            std::wstring message =
                std::wstring(L"launch yaw unmeasured state") +
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

            Assert::IsTrue(scenario.yawUnmeasuredStateDifference.maxAbs <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LaunchYawVarianceMatchesModel)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.yawVarianceDifference;
            std::wstring message =
                std::wstring(L"launch yaw variance") +
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

            Assert::IsTrue(scenario.yawVarianceDifference.maxAbs <= 1.0e-7f, message.c_str());
        }

        TEST_METHOD(LaunchYawClearsResidualCrossCovariance)
        {
            const LaunchEncoderScenario scenario = RunLaunchEncoderScenario();
            const IndexedDifference& messageDifference = scenario.yawCrossCovarianceDifference;
            std::wstring message =
                std::wstring(L"launch yaw/residual covariance") +
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

            Assert::IsTrue(scenario.yawCrossCovarianceDifference.maxAbs <= 1.0e-8f, message.c_str());
        }

    };
}
