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

    TEST_CLASS(DriveStack_GyroOnlyReplayTest)
    {
    public:
        TEST_METHOD(YawSignPositive)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << result.yawRad
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.yawRad > 0.0f, message.str().c_str());
        }

        TEST_METHOD(YawMatchesGyroIntegration)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
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

        TEST_METHOD(YawRateMatchesGyro)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << result.expectedYawRateRadps
                << L"\nactual=" << result.yawRateRadps
                << L"\ntolerance=" << kYawRateToleranceRadps;

            Assert::AreEqual(
                result.expectedYawRateRadps,
                result.yawRateRadps,
                kYawRateToleranceRadps,
                message.str().c_str());
        }

        TEST_METHOD(ForwardVelocityStaysZero)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=forward_velocity_mps"
                << L"\nexpected=0"
                << L"\nactual=" << result.velocityMps
                << L"\ntolerance=" << kVelocityToleranceMps;

            Assert::AreEqual(
                0.0f,
                result.velocityMps,
                kVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunGyroOnlyReplay();
            std::wstringstream message;
            message << L"EST40_GYRO_SIGN"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }
    };
}
