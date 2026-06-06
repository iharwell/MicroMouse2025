#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorRuntimeMotionUpdateTestSupport.h"
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorMotionUpdateSupport;
    TEST_CLASS(EstimatorTorqueRefreshTest)
    {
    public:
        TEST_METHOD(TorqueRefreshResetAccepted)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();

            Assert::IsTrue(scenario.resetAccepted, L"reset rejected torque-refresh scenario");
        }

        TEST_METHOD(TorqueRefreshFirstPredictAccepted)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();

            Assert::IsTrue(scenario.firstPredictAccepted, L"first predict rejected torque-refresh scenario");
        }

        TEST_METHOD(TorqueRefreshFirstControlResetAccepted)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();

            Assert::IsTrue(scenario.firstControlResetAccepted, L"first-control reset rejected torque-refresh scenario");
        }

        TEST_METHOD(TorqueRefreshSecondControlResetAccepted)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();

            Assert::IsTrue(scenario.secondControlResetAccepted, L"second-control reset rejected torque-refresh scenario");
        }

        TEST_METHOD(TorqueRefreshFirstControlPredictAccepted)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();

            Assert::IsTrue(scenario.firstControlPredictAccepted, L"first-control predict rejected torque-refresh scenario");
        }

        TEST_METHOD(TorqueRefreshSecondControlPredictAccepted)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();

            Assert::IsTrue(scenario.secondControlPredictAccepted, L"second-control predict rejected torque-refresh scenario");
        }

        TEST_METHOD(TorqueRefreshUsesCurrentControl)
        {
            const TorqueRefreshScenario scenario = RunTorqueRefreshScenario();
            const std::wstring message =
                std::wstring(L"control response delta") +
                L" actual=" +
                std::to_wstring(scenario.responseDelta) +
                L", limit=" +
                std::to_wstring(1.0e-6f);

            Assert::IsTrue(scenario.responseDelta > 1.0e-6f, message.c_str());
        }

    };
}
