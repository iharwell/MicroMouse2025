#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorControlMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorSplitDrivePredictTest)
    {
    public:
        TEST_METHOD(SplitDrivePredictResetAccepted)
        {
            const SplitDrivePredictScenario scenario = RunSplitDrivePredictScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected split-drive predict scenario");
        }

        TEST_METHOD(SplitDrivePredictCompletes)
        {
            const SplitDrivePredictScenario scenario = RunSplitDrivePredictScenario();
            const std::wstring message =
                std::wstring(L"first incomplete operation=") +
                ((scenario.firstIncompleteOperation != nullptr) ? scenario.firstIncompleteOperation : L"none");

            Assert::AreEqual(75, scenario.completedSteps, message.c_str());
        }

        TEST_METHOD(SplitDrivePredictAdvancesForward)
        {
            const SplitDrivePredictScenario scenario = RunSplitDrivePredictScenario();
            const std::wstring message =
                std::wstring(L"before y=") +
                std::to_wstring(scenario.initialState(1)) +
                L", after y=" +
                std::to_wstring(scenario.state(1));

            Assert::IsTrue(scenario.state(1) > scenario.initialState(1), message.c_str());
        }

        TEST_METHOD(SplitDrivePredictKeepsForwardVelocityPositive)
        {
            const SplitDrivePredictScenario scenario = RunSplitDrivePredictScenario();
            const std::wstring message =
                std::wstring(L"split-drive forward velocity") +
                L" actual=" +
                std::to_wstring(scenario.state(3)) +
                L", limit=" +
                std::to_wstring(0.0f);

            Assert::IsTrue(scenario.state(3) > 0.0f, message.c_str());
        }

        TEST_METHOD(SplitDrivePredictTurnsCounterClockwise)
        {
            const SplitDrivePredictScenario scenario = RunSplitDrivePredictScenario();
            const std::wstring message =
                std::wstring(L"heading rad=") + std::to_wstring(scenario.state(2));

            Assert::IsTrue(scenario.state(2) < 0.0f, message.c_str());
        }

        TEST_METHOD(SplitDrivePredictDriftsLeft)
        {
            const SplitDrivePredictScenario scenario = RunSplitDrivePredictScenario();
            const std::wstring message =
                std::wstring(L"x m=") + std::to_wstring(scenario.state(0));

            Assert::IsTrue(scenario.state(0) < 0.0f, message.c_str());
        }

    };
}
