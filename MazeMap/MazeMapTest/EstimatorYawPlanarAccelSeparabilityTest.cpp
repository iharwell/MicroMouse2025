#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorModeAndDiagnosticsTestSupport.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorModeAndDiagnosticsTestSupport;

    TEST_CLASS(EstimatorYawPlanarAccelSeparabilityTest)
    {
    public:
        TEST_METHOD(SequentialYawUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_yaw_attempted", result.sequentialYawAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialYawAttempted, message.c_str());
        }

        TEST_METHOD(SequentialYawUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_yaw_accepted", result.sequentialYawAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialYawAccepted, message.c_str());
        }

        TEST_METHOD(SequentialPlanarAccelUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_accel_attempted", result.sequentialAccelAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialAccelAttempted, message.c_str());
        }

        TEST_METHOD(SequentialPlanarAccelUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"sequential_accel_accepted", result.sequentialAccelAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.sequentialAccelAccepted, message.c_str());
        }

        TEST_METHOD(MergedYawUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_yaw_attempted", result.mergedYawAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedYawAttempted, message.c_str());
        }

        TEST_METHOD(MergedYawUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_yaw_accepted", result.mergedYawAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedYawAccepted, message.c_str());
        }

        TEST_METHOD(MergedPlanarAccelUpdateIsAttempted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_accel_attempted", result.mergedAccelAttempted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedAccelAttempted, message.c_str());
        }

        TEST_METHOD(MergedPlanarAccelUpdateIsAccepted)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = BoolMessage(L"merged_accel_accepted", result.mergedAccelAccepted, L"true", ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.mergedAccelAccepted, message.c_str());
        }

        TEST_METHOD(SequentialAndMergedUpdatesProduceSameState)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = LimitMessage(L"state_max_abs_delta", result.stateMaxAbsDelta, L"<=", 1.0e-6f, ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.stateMaxAbsDelta <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(SequentialAndMergedUpdatesProduceSameCovariance)
        {
            const YawPlanarAccelResult result = RunYawPlanarAccelScenario();
            const std::wstring message = LimitMessage(L"covariance_max_abs_delta", result.covarianceMaxAbsDelta, L"<=", 1.0e-6f, ScenarioPairMessage(result.mergedPrimeStatus, result.sequentialPrimeStatus, L"merged", L"sequential").c_str());
            Assert::IsTrue(result.covarianceMaxAbsDelta <= 1.0e-6f, message.c_str());
        }

    };
}
