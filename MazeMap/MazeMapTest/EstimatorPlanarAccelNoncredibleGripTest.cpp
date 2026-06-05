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

    TEST_CLASS(EstimatorPlanarAccelNoncredibleGripTest)
    {
    public:
        TEST_METHOD(AttemptsUpdate)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const std::wstring message = BoolMessage(L"planar_accel_attempted", result.planarAccelAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.planarAccelAttempted, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocityLarge)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const float actual = std::fabs(result.finalLateralVelocityMps);
            const std::wstring message = LimitMessage(L"abs_lateral_velocity_mps", actual, L">", 0.10f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(actual > 0.10f, message.c_str());
        }

        TEST_METHOD(LateralVarianceIsFinite)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const std::wstring message = ValueMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalLateralVarianceMps2), message.c_str());
        }

        TEST_METHOD(KeepsLateralVarianceLoose)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(2.0f, 8.0f, 0.001f * 0.001f);
            const float limit = 0.020f * 0.020f;
            const std::wstring message = LimitMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, L">=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.finalLateralVarianceMps2 >= limit, message.c_str());
        }

    };
}
