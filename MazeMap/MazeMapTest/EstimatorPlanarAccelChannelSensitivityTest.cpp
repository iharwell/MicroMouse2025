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

    TEST_CLASS(EstimatorPlanarAccelChannelSensitivityTest)
    {
    public:
        TEST_METHOD(BaselineUpdateIsAttempted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"baseline_attempted", result.baselineAttempted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.baselineAttempted, message.c_str());
        }

        TEST_METHOD(BaselineUpdateIsAccepted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"baseline_accepted", result.baselineAccepted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.baselineAccepted, message.c_str());
        }

        TEST_METHOD(RightPerturbedUpdateIsAttempted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"right_attempted", result.rightAttempted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightAttempted, message.c_str());
        }

        TEST_METHOD(RightPerturbedUpdateIsAccepted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"right_accepted", result.rightAccepted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightAccepted, message.c_str());
        }

        TEST_METHOD(ForwardPerturbedUpdateIsAttempted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"forward_attempted", result.forwardAttempted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardAttempted, message.c_str());
        }

        TEST_METHOD(ForwardPerturbedUpdateIsAccepted)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = BoolMessage(L"forward_accepted", result.forwardAccepted, L"true", ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardAccepted, message.c_str());
        }

        TEST_METHOD(RightPerturbationChangesState)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"right_state_max_abs_delta", result.rightStateMaxAbsDelta, L">", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightStateMaxAbsDelta > 1.0e-6f, message.c_str());
        }

        TEST_METHOD(RightPerturbationKeepsCovarianceSame)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"right_covariance_max_abs_delta", result.rightCovarianceMaxAbsDelta, L"<=", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.rightCovarianceMaxAbsDelta <= 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ForwardPerturbationChangesState)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"forward_state_max_abs_delta", result.forwardStateMaxAbsDelta, L">", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardStateMaxAbsDelta > 1.0e-6f, message.c_str());
        }

        TEST_METHOD(ForwardPerturbationKeepsCovarianceSame)
        {
            const PlanarAccelChannelResult result = RunPlanarAccelChannelScenario();
            const std::wstring message = LimitMessage(L"forward_covariance_max_abs_delta", result.forwardCovarianceMaxAbsDelta, L"<=", 1.0e-6f, ScenarioTripleMessage(result.baselinePrimeStatus, result.rightPrimeStatus, result.forwardPrimeStatus, L"baseline", L"right", L"forward").c_str());
            Assert::IsTrue(result.forwardCovarianceMaxAbsDelta <= 1.0e-6f, message.c_str());
        }
    };
}
