#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorControlMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorOpposedControlUpdateTest)
    {
    public:
        TEST_METHOD(OpposedControlSequenceCompletes)
        {
            const IterativeMotionScenario scenario = RunOpposedControlScenario();
            const std::wstring message =
                std::wstring(L"first incomplete operation=") +
                ((scenario.firstIncompleteOperation != nullptr) ? scenario.firstIncompleteOperation : L"none");

            Assert::AreEqual(200, scenario.completedSteps, message.c_str());
        }

        TEST_METHOD(OpposedControlKeepsForwardVelocityBounded)
        {
            const IterativeMotionScenario scenario = RunOpposedControlScenario();
            const std::wstring message =
                std::wstring(L"opposed forward velocity") +
                L" actual=" +
                std::to_wstring(std::fabs(scenario.state(3))) +
                L", limit=" +
                std::to_wstring(1.0e-4f);

            Assert::IsTrue(std::fabs(scenario.state(3)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(OpposedControlKeepsResidualsZero)
        {
            const IterativeMotionScenario scenario = RunOpposedControlScenario();
            const IndexedDifference difference =
                MaxAccelResidualDifference(StateVector::Zero(), scenario.state);
            const IndexedDifference& messageDifference = difference;
            std::wstring message =
                std::wstring(L"opposed residual state") +
                L" max_abs=" +
                std::to_wstring(messageDifference.maxAbs) +
                L", limit=" +
                std::to_wstring(1.0e-6f);
            if (messageDifference.sampleIndex >= 0)
            {
                message += L", sample=";
                message += std::to_wstring(messageDifference.sampleIndex);
            }
            if (messageDifference.row >= 0)
            {
                message += L", row=";
                message += std::to_wstring(messageDifference.row);
            }
            if (messageDifference.col >= 0)
            {
                message += L", col=";
                message += std::to_wstring(messageDifference.col);
            }

            Assert::IsTrue(difference.maxAbs <= 1.0e-6f, message.c_str());
        }

    };
}
