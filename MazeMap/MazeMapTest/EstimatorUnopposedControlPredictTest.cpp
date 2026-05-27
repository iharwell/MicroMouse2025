#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorControlMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorUnopposedControlPredictTest)
    {
    public:
        TEST_METHOD(UnopposedControlSequenceCompletes)
        {
            const IterativeMotionScenario scenario = RunUnopposedControlScenario();
            const std::wstring message =
                std::wstring(L"first incomplete operation=") +
                ((scenario.firstIncompleteOperation != nullptr) ? scenario.firstIncompleteOperation : L"none");

            Assert::AreEqual(200, scenario.completedSteps, message.c_str());
        }

        TEST_METHOD(UnopposedControlMovesForward)
        {
            const IterativeMotionScenario scenario = RunUnopposedControlScenario();
            const std::wstring message =
                std::wstring(L"unopposed forward pose") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(1))) +
                L", limit=" +
                std::to_wstring(1.0e-2f);

            Assert::IsTrue(std::fabs(scenario.state(1)) > 1.0e-2f, message.c_str());
        }

        TEST_METHOD(UnopposedControlBuildsForwardVelocity)
        {
            const IterativeMotionScenario scenario = RunUnopposedControlScenario();
            const std::wstring message =
                std::wstring(L"unopposed forward velocity") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(3))) +
                L", limit=" +
                std::to_wstring(1.0e-2f);

            Assert::IsTrue(std::fabs(scenario.state(3)) > 1.0e-2f, message.c_str());
        }

    };
}
