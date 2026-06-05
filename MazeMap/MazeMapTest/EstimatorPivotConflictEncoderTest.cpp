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

    TEST_CLASS(EstimatorPivotConflictEncoderTest)
    {
    public:
        TEST_METHOD(CopiesLeftWheelSpeedToRuntime)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.pivotEncoderLeftWheelSpeedRadps, result.runtimeLeftWheelSpeedRadps, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(CopiesRightWheelSpeedToRuntime)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.pivotEncoderRightWheelSpeedRadps, result.runtimeRightWheelSpeedRadps, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(UpdateIsAttempted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_encoder_attempted", result.pivotEncoderAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotEncoderAttempted, message.c_str());
        }

        TEST_METHOD(UpdateIsAccepted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_encoder_accepted", result.pivotEncoderAccepted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotEncoderAccepted, message.c_str());
        }

        TEST_METHOD(NisIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.pivotEncoderNis, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesPoseXUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(0), result.stateAfterEncoder(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesPoseYUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(1), result.stateAfterEncoder(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesHeadingUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(2), result.stateAfterEncoder(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesForwardVelocityUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(3), result.stateAfterEncoder(3), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesLateralVelocityUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(4), result.stateAfterEncoder(4), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesYawRateUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(5), result.stateAfterEncoder(5), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesForwardAccelResidualUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(6), result.stateAfterEncoder(6), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesRightwardAccelResidualUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(7), result.stateAfterEncoder(7), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesYawAccelResidualUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterPredict(8), result.stateAfterEncoder(8), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(LeavesCovarianceUnchanged)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const float delta = MaxAbsDelta(result.covarianceAfterPredict, result.covarianceAfterEncoder);
            const std::wstring message = LimitMessage(L"covariance_max_abs_delta", delta, L"<=", 1.0e-7f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(delta <= 1.0e-7f, message.c_str());
        }

    };
}
