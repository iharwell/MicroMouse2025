#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorYawMotionUpdateTestSupport.h"

#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;

    TEST_CLASS(EstimatorLaunchEncoderPredictionTest)
    {
    public:
        TEST_METHOD(LaunchEncoderResetAccepted)
        {
            const LaunchEncoderPredictionScenario scenario =
                RunLaunchEncoderPredictionScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected the launch encoder scenario");
        }

        TEST_METHOD(LaunchEncoderPredictsAllSamples)
        {
            const LaunchEncoderPredictionScenario scenario =
                RunLaunchEncoderPredictionScenario();
            const std::wstring message =
                std::wstring(L"completed_predict_samples actual=") +
                std::to_wstring(scenario.completedPredictSamples) +
                L" expected=" +
                std::to_wstring(kLaunchEncoderSampleCount) +
                L" first_incomplete=" +
                ((scenario.firstIncompleteOperation != nullptr) ?
                    scenario.firstIncompleteOperation :
                    L"none");

            Assert::AreEqual(kLaunchEncoderSampleCount, scenario.completedPredictSamples, message.c_str());
        }

        TEST_METHOD(LaunchEncoderPredictionRecordsEncoderInput)
        {
            const LaunchEncoderPredictionScenario scenario =
                RunLaunchEncoderPredictionScenario();
            const std::wstring message =
                std::wstring(L"prediction_encoder_input actual=") +
                (scenario.predictionEncoderInputObserved ? L"true" : L"false");

            Assert::IsTrue(scenario.predictionEncoderInputObserved, message.c_str());
        }

        TEST_METHOD(LaunchEncoderPredictionDumpsLatestWheelSpeeds)
        {
            const LaunchEncoderPredictionScenario scenario =
                RunLaunchEncoderPredictionScenario();
            Assert::AreEqual(
                scenario.finalEncoder.LeftWheelSpeedRadps(),
                scenario.dumpLeftWheelSpeedRadps,
                1.0e-5f,
                L"prediction encoder dump left wheel speed");
            Assert::AreEqual(
                scenario.finalEncoder.RightWheelSpeedRadps(),
                scenario.dumpRightWheelSpeedRadps,
                1.0e-5f,
                L"prediction encoder dump right wheel speed");
        }

        TEST_METHOD(LaunchEncoderPredictionWheelSpeedsMatchPublishedTravel)
        {
            const LaunchEncoderPredictionScenario scenario =
                RunLaunchEncoderPredictionScenario();

            Assert::IsTrue(
                scenario.measuredForwardSpeedDifference.maxAbs <= 1.0e-6f,
                L"launch encoder published forward speed differs from published travel");
            Assert::IsTrue(
                scenario.measuredYawRateDifference.maxAbs <= 1.0e-6f,
                L"launch encoder published yaw rate differs from published travel");
        }
    };
}
