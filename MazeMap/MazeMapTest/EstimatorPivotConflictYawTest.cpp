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

    TEST_CLASS(EstimatorPivotConflictYawTest)
    {
    public:
        TEST_METHOD(UpdateIsAttempted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_yaw_attempted", result.pivotYawAttempted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotYawAttempted, message.c_str());
        }

        TEST_METHOD(UpdateIsAccepted)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = BoolMessage(L"pivot_yaw_accepted", result.pivotYawAccepted, L"true", ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.pivotYawAccepted, message.c_str());
        }

        TEST_METHOD(NisMatchesOracle)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.expectedYawNis, result.pivotYawNis, 1.0e-4f, message.c_str());
        }

        TEST_METHOD(KeepsPoseX)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(0), result.stateAfterPivot(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsPoseY)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(1), result.stateAfterPivot(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsHeading)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(2), result.stateAfterPivot(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsForwardVelocity)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(3), result.stateAfterPivot(3), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocity)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.stateAfterEncoder(4), result.stateAfterPivot(4), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(RateUsesGyro)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.expectedYawRateRadps, result.stateAfterPivot(5), 1.0e-5f, message.c_str());
        }

        TEST_METHOD(VarianceMatchesOracle)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.expectedYawVarianceRadps2, result.covarianceAfterPivot(5, 5), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CovarianceForwardAccelCrossTermIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.covarianceAfterPivot(5, 6), 1.0e-8f, message.c_str());
        }

        TEST_METHOD(CovarianceRightwardAccelCrossTermIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.covarianceAfterPivot(5, 7), 1.0e-8f, message.c_str());
        }

        TEST_METHOD(CovarianceYawAccelCrossTermIsZero)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.covarianceAfterPivot(5, 8), 1.0e-8f, message.c_str());
        }

        TEST_METHOD(RateIsCloserToGyroThanEncoder)
        {
            const PivotConflictResult result = RunPivotConflictScenario();
            const float gyroError = std::fabs(result.stateAfterPivot(5) - result.gyroCorrectedYawRateRadps);
            const float encoderError = std::fabs(result.stateAfterPivot(5) - result.encoderDerivedYawRateRadps);
            const std::wstring message = LimitMessage(L"gyro_error_radps", gyroError, L"< encoder_error", encoderError, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(gyroError < encoderError, message.c_str());
        }

    };
}
