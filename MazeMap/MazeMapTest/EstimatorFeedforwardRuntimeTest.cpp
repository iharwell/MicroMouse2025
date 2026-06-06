#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorRuntimeMotionUpdateTestSupport.h"

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

        TEST_METHOD(FeedforwardRuntimeEncoderForwardVelocityMatchesPublishedTravel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.travelForwardVelocityMps,
                scenario.measuredForwardVelocityMps,
                1.0e-6f,
                L"feedforward encoder forward velocity from published travel");
        }

        TEST_METHOD(FeedforwardRuntimeEncoderYawRateMatchesPublishedTravel)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.travelYawRateRadps,
                scenario.measuredYawRateRadps,
                1.0e-6f,
                L"feedforward encoder yaw rate from published travel");
        }

        TEST_METHOD(FeedforwardRuntimeStoresLeftWheelSpeedBeforePredict)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.LeftWheelSpeedRadps(),
                scenario.runtimeLeftWheelSpeedBeforePredictRadps,
                1.0e-6f,
                L"left wheel speed before predict");
        }

        TEST_METHOD(FeedforwardRuntimeStoresRightWheelSpeedBeforePredict)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.RightWheelSpeedRadps(),
                scenario.runtimeRightWheelSpeedBeforePredictRadps,
                1.0e-6f,
                L"right wheel speed before predict");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsLeftWheelSpeedAfterPredict)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.LeftWheelSpeedRadps(),
                scenario.runtimeLeftWheelSpeedAfterPredictRadps,
                1.0e-6f,
                L"left wheel speed after predict");
        }

        TEST_METHOD(FeedforwardRuntimeKeepsRightWheelSpeedAfterPredict)
        {
            const FeedforwardRuntimeScenario scenario = RunFeedforwardRuntimeScenario();

            Assert::AreEqual(
                scenario.encoder.RightWheelSpeedRadps(),
                scenario.runtimeRightWheelSpeedAfterPredictRadps,
                1.0e-6f,
                L"right wheel speed after predict");
        }
    };
}
