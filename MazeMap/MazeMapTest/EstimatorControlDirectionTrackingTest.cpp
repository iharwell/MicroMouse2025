#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorControlMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorControlDirectionTrackingTest)
    {
    public:
        TEST_METHOD(ControlDirectionsCompleteTracking)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(false);
            const std::wstring message =
                std::wstring(L"first incomplete operation=") +
                ((scenario.firstIncompleteOperation != nullptr) ? scenario.firstIncompleteOperation : L"none");

            Assert::AreEqual(3000, scenario.completedTrackingSteps, message.c_str());
        }

        TEST_METHOD(ControlDirectionsBuildForwardVelocity)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(false);
            const std::wstring message =
                std::wstring(L"forward velocity") +
                L" actual=" +
                std::to_wstring(scenario.state(3)) +
                L", limit=" +
                std::to_wstring(0.8f);

            Assert::IsTrue(scenario.state(3) > 0.8f, message.c_str());
        }

        TEST_METHOD(ControlDirectionsKeepLateralVelocitySmall)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(false);
            const std::wstring message =
                std::wstring(L"lateral velocity") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(4))) +
                L", limit=" +
                std::to_wstring(0.01f);

            Assert::IsTrue(std::fabs(scenario.state(4)) < 0.01f, message.c_str());
        }

        TEST_METHOD(ControlDirectionsKeepYawRateSmall)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(false);
            const std::wstring message =
                std::wstring(L"yaw rate") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(5))) +
                L", limit=" +
                std::to_wstring(0.5f);

            Assert::IsTrue(std::fabs(scenario.state(5)) < 0.05f, message.c_str());
        }

        TEST_METHOD(StationaryThenControlCompletesTracking)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(true);
            const std::wstring message =
                std::wstring(L"first incomplete operation=") +
                ((scenario.firstIncompleteOperation != nullptr) ? scenario.firstIncompleteOperation : L"none");

            Assert::AreEqual(3000, scenario.completedTrackingSteps, message.c_str());
        }

        TEST_METHOD(StationaryThenControlBuildsForwardVelocity)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(true);
            const std::wstring message =
                std::wstring(L"forward velocity after stationary") +
                L" actual=" +
                std::to_wstring(scenario.state(3)) +
                L", limit=" +
                std::to_wstring(0.8f);

            Assert::IsTrue(scenario.state(3) > 0.8f, message.c_str());
        }

        TEST_METHOD(StationaryThenControlKeepsLateralVelocitySmall)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(true);
            const std::wstring message =
                std::wstring(L"lateral velocity after stationary") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(4))) +
                L", limit=" +
                std::to_wstring(0.01f);

            Assert::IsTrue(std::fabs(scenario.state(4)) < 0.01f, message.c_str());
        }

        TEST_METHOD(StationaryThenControlKeepsYawRateSmall)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(true);
            const std::wstring message =
                std::wstring(L"yaw rate after stationary") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(5))) +
                L", limit=" +
                std::to_wstring(0.05f);

            Assert::IsTrue(std::fabs(scenario.state(5)) < 0.05f, message.c_str());
        }

        TEST_METHOD(StationaryThenControlCompletesStationary)
        {
            const ControlDirectionScenario scenario = RunControlDirectionScenario(true);
            const std::wstring message =
                std::wstring(L"first incomplete operation=") +
                ((scenario.firstIncompleteOperation != nullptr) ? scenario.firstIncompleteOperation : L"none");

            Assert::AreEqual(3000, scenario.completedStationarySteps, message.c_str());
        }

    };
}
