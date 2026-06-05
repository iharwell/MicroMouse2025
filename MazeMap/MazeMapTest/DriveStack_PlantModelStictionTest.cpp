#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelStictionTest)
    {
    public:
        TEST_METHOD(SubthresholdDriveDoesNotAccelerateLeftWheelAtRest)
        {
            TestRuntime runtime;
            const VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            const float wheelAccelRadps2 =
                (integrated.GetWheelSpeedLeft() - state.GetWheelSpeedLeft()) / kDirectDtSeconds;
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=left_wheel_accel_radps2"
                << L"\nexpected=0"
                << L"\nactual=" << wheelAccelRadps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                wheelAccelRadps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(SubthresholdDriveDoesNotAccelerateRightWheelAtRest)
        {
            TestRuntime runtime;
            const VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            const float wheelAccelRadps2 =
                (integrated.GetWheelSpeedRight() - state.GetWheelSpeedRight()) / kDirectDtSeconds;
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=right_wheel_accel_radps2"
                << L"\nexpected=0"
                << L"\nactual=" << wheelAccelRadps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                wheelAccelRadps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftPositionX)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetPositionX();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=position_x_m"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetPositionX()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetPositionX(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftPositionY)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetPositionY();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=position_y_m"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetPositionY()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetPositionY(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftYaw)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetHeading();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=yaw_rad"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetHeading()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetHeading(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftForwardVelocity)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetForwardVelocity();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=forward_velocity_mps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetForwardVelocity()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetForwardVelocity(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftLateralVelocity)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetRightwardVelocity();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=lateral_velocity_mps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetRightwardVelocity()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetRightwardVelocity(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftYawRate)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetYawRate();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetYawRate()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetYawRate(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftLeftWheelSpeed)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetWheelSpeedLeft();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetWheelSpeedLeft()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetWheelSpeedLeft(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(DirectIntegrationDoesNotDriftRightWheelSpeed)
        {
            TestRuntime runtime;
            VehicleState state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state.GetWheelSpeedRight();
            for (int tick = 0; tick < 100; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.25f, 0.25f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state.GetWheelSpeedRight()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state.GetWheelSpeedRight(),
                1.0e-6f,
                message.str().c_str());
        }

    };
}
