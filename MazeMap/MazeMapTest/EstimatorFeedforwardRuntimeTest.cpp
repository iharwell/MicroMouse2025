#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorRuntimeMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorFeedforwardRuntimeTest)
    {
    public:
        TEST_METHOD(FeedforwardRuntimeResetAccepted)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected feedforward runtime scenario");
        }

        TEST_METHOD(FeedforwardRuntimePredictAccepted)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::IsTrue(scenario.predictAccepted, L"predict rejected feedforward runtime scenario");
        }

        TEST_METHOD(FeedforwardRuntimeStoresLeftWheelSpeedBeforeUpdate)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.LeftWheelSpeedRadps(),
                scenario.runtimeLeftWheelSpeedBeforeUpdateRadps,
                1.0e-6f,
                L"left wheel speed before encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeStoresRightWheelSpeedBeforeUpdate)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.RightWheelSpeedRadps(),
                scenario.runtimeRightWheelSpeedBeforeUpdateRadps,
                1.0e-6f,
                L"right wheel speed before encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeUpdateAttempted)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::IsTrue(scenario.updateAttempted, L"feedforward encoder update was not attempted");
        }

        TEST_METHOD(FeedforwardRuntimeUpdateAccepted)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::IsTrue(scenario.updateReturnedAccepted, L"feedforward encoder update was rejected");
        }

        TEST_METHOD(FeedforwardRuntimeUpdateRecordedAccepted)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::IsTrue(scenario.updateRecordedAccepted, L"feedforward encoder update was not recorded accepted");
        }

        TEST_METHOD(FeedforwardRuntimeNisMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.nis, scenario.actualNis, 1.0e-4f, L"feedforward encoder NIS");
        }

        TEST_METHOD(FeedforwardRuntimeForwardVelocityMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.bodyState(0), scenario.afterState(3), 1.0e-6f, L"forward velocity after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeYawRateMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.bodyState(1), scenario.afterState(5), 1.0e-6f, L"yaw rate after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsXPosition)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(0), scenario.afterState(0), 1.0e-6f, L"x position after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsYPosition)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(1), scenario.afterState(1), 1.0e-6f, L"y position after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsHeading)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(2), scenario.afterState(2), 1.0e-6f, L"heading after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsRightVelocity)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(4), scenario.afterState(4), 1.0e-6f, L"right velocity after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsForwardResidual)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(6), scenario.afterState(6), 1.0e-6f, L"forward residual after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsRightResidual)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(7), scenario.afterState(7), 1.0e-6f, L"right residual after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsYawAccelResidual)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.beforeEncoderState(8), scenario.afterState(8), 1.0e-6f, L"yaw accel residual after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeForwardVelocityCovarianceMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.bodyCovariance(0, 0), scenario.afterCovariance(3, 3), 1.0e-7f, L"forward velocity covariance after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeForwardYawCovarianceMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.bodyCovariance(0, 1), scenario.afterCovariance(3, 5), 1.0e-7f, L"forward/yaw covariance after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeYawForwardCovarianceMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.bodyCovariance(1, 0), scenario.afterCovariance(5, 3), 1.0e-7f, L"yaw/forward covariance after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeYawRateCovarianceMatchesModel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(scenario.expectation.bodyCovariance(1, 1), scenario.afterCovariance(5, 5), 1.0e-7f, L"yaw rate covariance after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeClearsForwardForwardResidualCovariance)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(0.0f, scenario.afterCovariance(3, 6), 1.0e-8f, L"forward velocity/forward residual covariance");
        }

        TEST_METHOD(FeedforwardRuntimeClearsForwardRightResidualCovariance)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(0.0f, scenario.afterCovariance(3, 7), 1.0e-8f, L"forward velocity/right residual covariance");
        }

        TEST_METHOD(FeedforwardRuntimeClearsForwardYawResidualCovariance)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(0.0f, scenario.afterCovariance(3, 8), 1.0e-8f, L"forward velocity/yaw residual covariance");
        }

        TEST_METHOD(FeedforwardRuntimeClearsYawForwardResidualCovariance)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(0.0f, scenario.afterCovariance(5, 6), 1.0e-8f, L"yaw rate/forward residual covariance");
        }

        TEST_METHOD(FeedforwardRuntimeClearsYawRightResidualCovariance)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(0.0f, scenario.afterCovariance(5, 7), 1.0e-8f, L"yaw rate/right residual covariance");
        }

        TEST_METHOD(FeedforwardRuntimeClearsYawYawResidualCovariance)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(0.0f, scenario.afterCovariance(5, 8), 1.0e-8f, L"yaw rate/yaw residual covariance");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsLeftWheelSpeedAfterUpdate)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.LeftWheelSpeedRadps(),
                scenario.runtimeLeftWheelSpeedAfterUpdateRadps,
                1.0e-6f,
                L"left wheel speed after encoder update");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsRightWheelSpeedAfterUpdate)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.RightWheelSpeedRadps(),
                scenario.runtimeRightWheelSpeedAfterUpdateRadps,
                1.0e-6f,
                L"right wheel speed after encoder update");
        }

    };
}
