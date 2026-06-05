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

    TEST_CLASS(EstimatorPlanarAccelCredibleGripTest)
    {
    public:
        TEST_METHOD(AttemptsUpdate)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = BoolMessage(L"planar_accel_attempted", result.planarAccelAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.planarAccelAttempted, message.c_str());
        }

        TEST_METHOD(AcceptsUpdate)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = BoolMessage(L"planar_accel_accepted", result.planarAccelAccepted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.planarAccelAccepted, message.c_str());
        }

        TEST_METHOD(LateralVelocityIsFinite)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = ValueMessage(L"lateral_velocity_mps", result.finalLateralVelocityMps, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalLateralVelocityMps), message.c_str());
        }

        TEST_METHOD(DampsLateralVelocityBelowThreshold)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const float actual = std::fabs(result.finalLateralVelocityMps);
            const std::wstring message = LimitMessage(L"abs_lateral_velocity_mps", actual, L"<", 0.05f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(actual < 0.05f, message.c_str());
        }

        TEST_METHOD(ReducesLateralVelocityMagnitude)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const float actual = std::fabs(result.finalLateralVelocityMps);
            const float limit = std::fabs(result.initialLateralVelocityMps);
            const std::wstring message = LimitMessage(L"abs_lateral_velocity_mps", actual, L"< initial", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(actual < limit, message.c_str());
        }

        TEST_METHOD(LateralVarianceIsFinite)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = ValueMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalLateralVarianceMps2), message.c_str());
        }

        TEST_METHOD(ReducesLateralVariance)
        {
            const PlanarAccelGripResult result = RunPlanarAccelGripScenario(1.0f, 1.0f, 0.30f * 0.30f);
            const std::wstring message = LimitMessage(L"lateral_velocity_variance_mps2", result.finalLateralVarianceMps2, L"< initial", result.initialLateralVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.finalLateralVarianceMps2 < result.initialLateralVarianceMps2, message.c_str());
        }

    };
}
