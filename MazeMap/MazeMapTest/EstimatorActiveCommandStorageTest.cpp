#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorControlMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorActiveCommandStorageTest)
    {
    public:
        TEST_METHOD(ResetPoseAcceptsActiveCommandStorageScenario)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::IsTrue(scenario.resetPoseAccepted, L"ResetPose rejected the active-command storage scenario");
        }

        TEST_METHOD(ResetPoseClearsLeftCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::AreEqual(
                0.0f,
                scenario.commandAfterReset.LeftCommand(),
                0.0f,
                L"left command after ResetPose");
        }

        TEST_METHOD(ResetPoseClearsRightCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::AreEqual(
                0.0f,
                scenario.commandAfterReset.RightCommand(),
                0.0f,
                L"right command after ResetPose");
        }

        TEST_METHOD(PredictAcceptsActiveCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::IsTrue(scenario.firstPredictAccepted, L"predict rejected activeCommand");
        }

        TEST_METHOD(PredictStoresActiveLeftCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::AreEqual(
                scenario.activeCommand.LeftCommand(),
                scenario.commandAfterFirstPredict.LeftCommand(),
                0.0f,
                L"left command after activeCommand predict");
        }

        TEST_METHOD(PredictStoresActiveRightCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::AreEqual(
                scenario.activeCommand.RightCommand(),
                scenario.commandAfterFirstPredict.RightCommand(),
                0.0f,
                L"right command after activeCommand predict");
        }

        TEST_METHOD(PredictAcceptsNextCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::IsTrue(scenario.secondPredictAccepted, L"predict rejected nextCommand");
        }

        TEST_METHOD(PredictStoresNextLeftCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::AreEqual(
                scenario.nextCommand.LeftCommand(),
                scenario.commandAfterSecondPredict.LeftCommand(),
                0.0f,
                L"left command after nextCommand predict");
        }

        TEST_METHOD(PredictStoresNextRightCommand)
        {
            const ActiveCommandStorageScenario scenario =
                RunActiveCommandStorageScenario();

            Assert::AreEqual(
                scenario.nextCommand.RightCommand(),
                scenario.commandAfterSecondPredict.RightCommand(),
                0.0f,
                L"right command after nextCommand predict");
        }
    };
}
