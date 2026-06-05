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

    TEST_CLASS(DriveStack_ForwardEncoderReplayTest)
    {
    public:
        TEST_METHOD(LeftCountsPositive)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=left_total_counts"
                << L"\nactual=" << result.leftEncoderTotalCounts
                << L"\ncriterion=actual>0";

            Assert::IsTrue(result.leftEncoderTotalCounts > 0, message.str().c_str());
        }

        TEST_METHOD(LeftRightCountsMatch)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=equal_forward_counts"
                << L"\nexpected_left=" << result.leftEncoderTotalCounts
                << L"\nactual_right=" << result.rightEncoderTotalCounts
                << L"\ncriterion=left==right";

            Assert::AreEqual(
                result.leftEncoderTotalCounts,
                result.rightEncoderTotalCounts,
                message.str().c_str());
        }

        TEST_METHOD(LeftDistanceMatchesCounts)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=left_encoder_distance_m"
                << L"\nexpected=" << result.expectedForwardM
                << L"\nactual=" << result.leftEncoderDistanceM
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                result.expectedForwardM,
                result.leftEncoderDistanceM,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(PositionYMatchesEncoderDistance)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=position_y_m"
                << L"\nexpected=" << result.expectedForwardM
                << L"\nactual=" << result.positionYM
                << L"\ntolerance=" << kForwardToleranceM;

            Assert::AreEqual(
                result.expectedForwardM,
                result.positionYM,
                kForwardToleranceM,
                message.str().c_str());
        }

        TEST_METHOD(PositionXStaysNearZero)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=position_x_m"
                << L"\nexpected=0"
                << L"\nactual=" << result.positionXM
                << L"\ntolerance=" << kForwardToleranceM;

            Assert::AreEqual(
                0.0f,
                result.positionXM,
                kForwardToleranceM,
                message.str().c_str());
        }

        TEST_METHOD(YawStaysNearZero)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=yaw_rad"
                << L"\nexpected=0"
                << L"\nactual=" << result.yawRad
                << L"\ntolerance=" << kYawToleranceRad;

            Assert::AreEqual(
                0.0f,
                result.yawRad,
                kYawToleranceRad,
                message.str().c_str());
        }

        TEST_METHOD(VelocityMatchesForward)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
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

        TEST_METHOD(EstimatorFaultClear)
        {
            const ReplayScenarioResult result = RunForwardEncoderReplay();
            std::wstringstream message;
            message << L"EST40_ENCODER_SIGN"
                << L"\nfield=estimator_fault"
                << L"\nexpected=false"
                << L"\nactual=" << result.estimatorFault;

            Assert::IsFalse(result.estimatorFault, message.str().c_str());
        }
    };
}
