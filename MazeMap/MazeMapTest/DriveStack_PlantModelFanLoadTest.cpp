#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelFanLoadTest)
    {
    public:
        TEST_METHOD(NoFanContactNormalSumMatchesConfiguredLoad)
        {
            TestRuntime runtime(0.0f);
            runtime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float expectedLoadN = runtime.vehicle.GetMass() * GRAVITY_MPS2;
            const float actualLoadN = runtime.plant.totalContactNormalLoadN();
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_normal_sum_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << actualLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=0";

            Assert::AreEqual(
                expectedLoadN,
                actualLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(FanOnContactNormalSumMatchesConfiguredLoad)
        {
            TestRuntime runtime(0.80f);
            runtime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float expectedLoadN =
                (runtime.vehicle.GetMass() * GRAVITY_MPS2) +
                (0.80f * runtime.vehicle.GetFanDownforceAtFullDuty());
            const float actualLoadN = runtime.plant.totalContactNormalLoadN();
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_normal_sum_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << actualLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=0.8";

            Assert::AreEqual(
                expectedLoadN,
                actualLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(FanDutyIncreasesTotalContactNormalLoad)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            fanOffRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            fanOnRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float fanOffLoadN =
                fanOffRuntime.plant.totalContactNormalLoadN();
            const float fanOnLoadN =
                fanOnRuntime.plant.totalContactNormalLoadN();
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=total_contact_normal_load_n"
                << L"\nfan_off=" << fanOffLoadN
                << L"\nfan_on=" << fanOnLoadN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnLoadN > fanOffLoadN,
                message.str().c_str());
        }

        TEST_METHOD(Contact0NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            fanOffRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            fanOnRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float fanOffNormalLoadN =
                fanOffRuntime.plant.contactNormalLoadN(0U);
            const float fanOnNormalLoadN =
                fanOnRuntime.plant.contactNormalLoadN(0U);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_0_normal_force_n"
                << L"\nfan_off=" << fanOffNormalLoadN
                << L"\nfan_on=" << fanOnNormalLoadN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnNormalLoadN > fanOffNormalLoadN,
                message.str().c_str());
        }

        TEST_METHOD(Contact1NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            fanOffRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            fanOnRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float fanOffNormalLoadN =
                fanOffRuntime.plant.contactNormalLoadN(1U);
            const float fanOnNormalLoadN =
                fanOnRuntime.plant.contactNormalLoadN(1U);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_1_normal_force_n"
                << L"\nfan_off=" << fanOffNormalLoadN
                << L"\nfan_on=" << fanOnNormalLoadN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnNormalLoadN > fanOffNormalLoadN,
                message.str().c_str());
        }

        TEST_METHOD(Contact2NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            fanOffRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            fanOnRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float fanOffNormalLoadN =
                fanOffRuntime.plant.contactNormalLoadN(2U);
            const float fanOnNormalLoadN =
                fanOnRuntime.plant.contactNormalLoadN(2U);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_2_normal_force_n"
                << L"\nfan_off=" << fanOffNormalLoadN
                << L"\nfan_on=" << fanOnNormalLoadN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnNormalLoadN > fanOffNormalLoadN,
                message.str().c_str());
        }

        TEST_METHOD(Contact3NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            fanOffRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            fanOnRuntime.runtimeState = MakeRollingState(0.75f, 0.0f);
            const float fanOffNormalLoadN =
                fanOffRuntime.plant.contactNormalLoadN(3U);
            const float fanOnNormalLoadN =
                fanOnRuntime.plant.contactNormalLoadN(3U);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_3_normal_force_n"
                << L"\nfan_off=" << fanOffNormalLoadN
                << L"\nfan_on=" << fanOnNormalLoadN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnNormalLoadN > fanOffNormalLoadN,
                message.str().c_str());
        }

    };
}
