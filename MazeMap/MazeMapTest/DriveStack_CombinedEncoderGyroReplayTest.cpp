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

    TEST_CLASS(DriveStack_CombinedEncoderGyroReplayTest)
    {
    public:
        TEST_METHOD(PositionXFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_x_m"
                << L"\nactual=" << result.positionXM
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(result.positionXM), message.str().c_str());
        }

        TEST_METHOD(PositionYFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_y_m"
                << L"\nactual=" << result.positionYM
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(result.positionYM), message.str().c_str());
        }

        TEST_METHOD(YawFinite)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << result.yawRad
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(result.yawRad), message.str().c_str());
        }

        TEST_METHOD(PositionXPositive)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_x_m"
                << L"\nactual=" << result.positionXM
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.positionXM > 0.0f, message.str().c_str());
        }

        TEST_METHOD(PositionYPositive)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_y_m"
                << L"\nactual=" << result.positionYM
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.positionYM > 0.0f, message.str().c_str());
        }

        TEST_METHOD(PositionXMatchesArc)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_x_m"
                << L"\nexpected=" << result.expectedArcXM
                << L"\nactual=" << result.positionXM
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                result.expectedArcXM,
                result.positionXM,
                0.010f,
                message.str().c_str());
        }

        TEST_METHOD(PositionYMatchesArc)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=position_y_m"
                << L"\nexpected=" << result.expectedArcYM
                << L"\nactual=" << result.positionYM
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                result.expectedArcYM,
                result.positionYM,
                0.010f,
                message.str().c_str());
        }

        TEST_METHOD(YawMatchesGyroIntegration)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
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

        TEST_METHOD(VelocityMatchesEncoderAverage)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=velocity_mps"
                << L"\nexpected=" << result.expectedVelocityMps
                << L"\nactual=" << result.velocityMps
                << L"\ntolerance=" << kVelocityToleranceMps;

            Assert::AreEqual(
                result.expectedVelocityMps,
                result.velocityMps,
                kVelocityToleranceMps,
                message.str().c_str());
        }

        TEST_METHOD(YawRateMatchesGyro)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
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

        TEST_METHOD(EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunCombinedReplay();
            std::wstringstream message;
            message << L"EST40_REPLAY_COHERENCE"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }
    };
}
