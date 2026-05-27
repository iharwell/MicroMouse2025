#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorControlMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorSplitCommandPredictTest)
    {
    public:
        TEST_METHOD(SplitCommandSequenceCompletes)
        {
            const StationarySplitCommandPrediction prediction =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            std::wstring message =
                std::wstring(L"completed steps=") +
                std::to_wstring(prediction.completedSteps) +
                L"/" +
                std::to_wstring(kStationarySplitCommandPredictSteps);
            if (prediction.firstIncompleteOperation != nullptr)
            {
                message += L", first incomplete operation=";
                message += prediction.firstIncompleteOperation;
            }
            Assert::AreEqual(
                kStationarySplitCommandPredictSteps,
                prediction.completedSteps,
                message.c_str());
        }

        TEST_METHOD(SplitCommandKeepsZeroForwardVelocity)
        {
            const StationarySplitCommandPrediction prediction =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            std::wstring message =
                std::wstring(L"completed steps=") +
                std::to_wstring(prediction.completedSteps) +
                L"/" +
                std::to_wstring(kStationarySplitCommandPredictSteps);
            if (prediction.firstIncompleteOperation != nullptr)
            {
                message += L", first incomplete operation=";
                message += prediction.firstIncompleteOperation;
            }

            Assert::AreEqual(
                0.0f,
                prediction.state(3),
                kZeroVelocityToleranceMps,
                message.c_str());
        }

        TEST_METHOD(SplitCommandProducesPositiveYawRate)
        {
            const StationarySplitCommandPrediction prediction =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            std::wstring message =
                std::wstring(L"predicted yaw rate radps=") +
                std::to_wstring(prediction.state(5)) +
                L", " +
                L"completed steps=" +
                std::to_wstring(prediction.completedSteps) +
                L"/" +
                std::to_wstring(kStationarySplitCommandPredictSteps);
            if (prediction.firstIncompleteOperation != nullptr)
            {
                message += L", first incomplete operation=";
                message += prediction.firstIncompleteOperation;
            }

            Assert::IsTrue(prediction.state(5) > 0.0f, message.c_str());
        }

        TEST_METHOD(SplitCommandTurnsClockwise)
        {
            const StationarySplitCommandPrediction prediction =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            std::wstring message =
                std::wstring(L"predicted heading rad=") +
                std::to_wstring(prediction.state(2)) +
                L", " +
                L"completed steps=" +
                std::to_wstring(prediction.completedSteps) +
                L"/" +
                std::to_wstring(kStationarySplitCommandPredictSteps);
            if (prediction.firstIncompleteOperation != nullptr)
            {
                message += L", first incomplete operation=";
                message += prediction.firstIncompleteOperation;
            }

            Assert::IsTrue(prediction.state(2) > 0.0f, message.c_str());
        }

        TEST_METHOD(SplitCommandKeepsZeroForwardPoseDrift)
        {
            const StationarySplitCommandPrediction prediction =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            std::wstring message =
                std::wstring(L"completed steps=") +
                std::to_wstring(prediction.completedSteps) +
                L"/" +
                std::to_wstring(kStationarySplitCommandPredictSteps);
            if (prediction.firstIncompleteOperation != nullptr)
            {
                message += L", first incomplete operation=";
                message += prediction.firstIncompleteOperation;
            }

            Assert::AreEqual(
                0.0f,
                prediction.state(1),
                0.01f,
                message.c_str());
        }

    };
}
