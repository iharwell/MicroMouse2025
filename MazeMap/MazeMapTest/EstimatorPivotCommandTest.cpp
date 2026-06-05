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

    TEST_CLASS(EstimatorPivotCommandTest)
    {
    public:
        TEST_METHOD(KeepsPoseX)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(0), result.stateAfterEncoder(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsPoseY)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(1), result.stateAfterEncoder(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsHeading)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(2), result.stateAfterEncoder(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocity)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateBeforeEncoder(4), result.stateAfterEncoder(4), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(MovesForwardVelocityTowardEncoder)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const float afterError = std::fabs(result.stateAfterEncoder(3) - result.measuredForwardSpeedMps);
            const float beforeError = std::fabs(result.stateBeforeEncoder(3) - result.measuredForwardSpeedMps);
            const std::wstring message = LimitMessage(L"forward_velocity_error_mps", afterError, L"< before", beforeError, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(afterError < beforeError, message.c_str());
        }

        TEST_METHOD(MovesYawRateTowardEncoder)
        {
            const PivotCommandResult result = RunPivotCommandScenario();
            const float afterError = std::fabs(result.stateAfterEncoder(5) - result.measuredYawRateRadps);
            const float beforeError = std::fabs(result.stateBeforeEncoder(5) - result.measuredYawRateRadps);
            const std::wstring message = LimitMessage(L"yaw_rate_error_radps", afterError, L"< before", beforeError, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(afterError < beforeError, message.c_str());
        }

    };
}
