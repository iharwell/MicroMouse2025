#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelWheelYawSignTest)
    {
    public:
        TEST_METHOD(PositiveYawMakesLeftWheelLinearVelocityFaster)
        {
            const float leftWheelMps =
                Vehicle::LeftWheelLinearVelocityFromBody(0.40f, 2.0f);
            const float rightWheelMps =
                Vehicle::RightWheelLinearVelocityFromBody(0.40f, 2.0f);
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=wheel_linear_velocity_mps"
                << L"\nleft=" << leftWheelMps
                << L"\nright=" << rightWheelMps
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                leftWheelMps > rightWheelMps,
                message.str().c_str());
        }

        TEST_METHOD(EncoderYawMeasurementPreservesClockwisePositiveSign)
        {
            TestRuntime runtime;
            EncoderObs observation{};
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                0.40f,
                2.0f,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            observation.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            observation.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            const float actualYawRateRadps = runtime.plant.measuredYawRateRadps(observation);
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=measured_yaw_rate_radps"
                << L"\nactual=" << actualYawRateRadps
                << L"\ncriterion=actual>0"
                << L"\nleft_wheel_speed_radps=" << observation.LeftWheelSpeedRadps()
                << L"\nright_wheel_speed_radps=" << observation.RightWheelSpeedRadps();

            Assert::IsTrue(
                actualYawRateRadps > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(PositiveTargetYawRateRequestsFasterLeftWheel)
        {
            const float leftWheelMps =
                Vehicle::LeftWheelLinearVelocityFromBody(0.40f, 2.0f);
            const float rightWheelMps =
                Vehicle::RightWheelLinearVelocityFromBody(0.40f, 2.0f);
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=target_wheel_linear_velocity_mps"
                << L"\nleft=" << leftWheelMps
                << L"\nright=" << rightWheelMps
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                leftWheelMps > rightWheelMps,
                message.str().c_str());
        }

        TEST_METHOD(PositiveYawAccelRaisesLeftCommand)
        {
            const PositiveYawAccelerationStep step =
                IntegratePositiveYawAcceleration();
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=command_differential"
                << L"\nactual=" << step.command.Differential()
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << step.command.LeftCommand()
                << L"\nright_command=" << step.command.RightCommand();

            Assert::IsTrue(
                step.command.Differential() > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(PositiveYawAccelIncreasesYawRate)
        {
            const PositiveYawAccelerationStep step =
                IntegratePositiveYawAcceleration();
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=yaw_rate_after_step_radps"
                << L"\ninitial=" << step.initial.GetYawRate()
                << L"\nactual=" << step.integrated.GetYawRate()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                step.integrated.GetYawRate() > step.initial.GetYawRate(),
                message.str().c_str());
        }

    };
}
