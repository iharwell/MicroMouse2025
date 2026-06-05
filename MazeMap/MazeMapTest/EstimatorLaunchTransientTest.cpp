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

    TEST_CLASS(EstimatorLaunchTransientTest)
    {
    public:
        TEST_METHOD(KeepsLateralVelocityNonzero)
        {
            const LaunchTransientResult result = RunLaunchTransientScenario();
            const std::wstring message = LimitMessage(L"lateral_velocity_mps", std::fabs(result.lateralVelocityMps), L">", 0.10f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.lateralVelocityMps) > 0.10f, message.c_str());
        }

    };
}
