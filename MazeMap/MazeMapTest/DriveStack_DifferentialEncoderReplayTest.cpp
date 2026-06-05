#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_EstimatorReplayTestSupport.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackEstimatorReplayTestSupport;

    TEST_CLASS(DriveStack_DifferentialEncoderReplayTest)
    {
    public:
        TEST_METHOD(LeftCountsGreaterThanRight)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=count_order"
                << L"\nleft=" << result.leftEncoderTotalCounts
                << L"\nright=" << result.rightEncoderTotalCounts
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                result.leftEncoderTotalCounts > result.rightEncoderTotalCounts,
                message.str().c_str());
        }

        TEST_METHOD(YawSignPositive)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << result.yawRad
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.yawRad > 0.0f, message.str().c_str());
        }

        TEST_METHOD(YawRateMatchesEncoderDifferential)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << result.actualEncoderYawRateRadps
                << L"\nactual=" << result.yawRateRadps
                << L"\ntolerance=" << kYawRateToleranceRadps;

            Assert::AreEqual(
                result.actualEncoderYawRateRadps,
                result.yawRateRadps,
                kYawRateToleranceRadps,
                message.str().c_str());
        }

        TEST_METHOD(YawMatchesEncoderIntegration)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=yaw_rad"
                << L"\nexpected=" << result.expectedYawRad
                << L"\nactual=" << result.yawRad
                << L"\ntolerance=" << kYawToleranceRad;

            Assert::AreEqual(
                result.expectedYawRad,
                result.yawRad,
                kYawToleranceRad,
                message.str().c_str());
        }

        TEST_METHOD(EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunDifferentialEncoderReplay();
            std::wstringstream message;
            message << L"EST40_LEFT_RIGHT_MAPPING"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }
    };
}
