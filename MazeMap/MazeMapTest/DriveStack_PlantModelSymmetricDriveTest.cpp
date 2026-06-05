#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelSymmetricDriveTest)
    {
    public:
        TEST_METHOD(LeftRightForwardForceSymmetry)
        {
            TestRuntime runtime;
            const VehicleState state = MakeRollingState(0.80f, 0.0f);
            runtime.runtimeState = state;
            const App::Internal::CommandVector command = MakeCommand(0.45f, 0.45f);
            const float leftForceN =
                runtime.plant.leftBankForwardContactForceN(command);
            const float rightForceN =
                runtime.plant.rightBankForwardContactForceN(command);
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=bank_forward_force_n"
                << L"\nexpected_left=" << leftForceN
                << L"\nactual_right=" << rightForceN
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                leftForceN,
                rightForceN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(WheelAccelerationSymmetry)
        {
            TestRuntime runtime;
            const VehicleState state = MakeRollingState(0.80f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(MakeCommand(0.45f, 0.45f), kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            const float leftWheelAccelRadps2 =
                (integrated.GetWheelSpeedLeft() - state.GetWheelSpeedLeft()) / kDirectDtSeconds;
            const float rightWheelAccelRadps2 =
                (integrated.GetWheelSpeedRight() - state.GetWheelSpeedRight()) / kDirectDtSeconds;
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=wheel_accel_radps2"
                << L"\nexpected_left=" << leftWheelAccelRadps2
                << L"\nactual_right=" << rightWheelAccelRadps2
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                leftWheelAccelRadps2,
                rightWheelAccelRadps2,
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(LeftRightWheelSpeedSymmetryAfterStep)
        {
            TestRuntime runtime;
            const VehicleState state = MakeRollingState(0.80f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(MakeCommand(0.45f, 0.45f), kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=wheel_speed_radps"
                << L"\nexpected_left=" << integrated.GetWheelSpeedLeft()
                << L"\nactual_right=" << integrated.GetWheelSpeedRight()
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                integrated.GetWheelSpeedLeft(),
                integrated.GetWheelSpeedRight(),
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(WheelSpeedSymmetryPersistsAcrossTicks)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.80f, 0.0f);
            for (int tick = 0; tick < 10; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.45f, 0.45f), kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=wheel_speed_radps"
                << L"\nexpected_left=" << state.GetWheelSpeedLeft()
                << L"\nactual_right=" << state.GetWheelSpeedRight()
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                state.GetWheelSpeedLeft(),
                state.GetWheelSpeedRight(),
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(NoYawAccelerationBias)
        {
            TestRuntime runtime;
            const VehicleState state = MakeRollingState(0.80f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(MakeCommand(0.45f, 0.45f), kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=yaw_accel_radps2"
                << L"\nexpected=0"
                << L"\nactual=" << integrated.GetYawAccel()
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                0.0f,
                integrated.GetYawAccel(),
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(NoLateralAccelerationBias)
        {
            TestRuntime runtime;
            const VehicleState state = MakeRollingState(0.80f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(MakeCommand(0.45f, 0.45f), kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=lateral_accel_mps2"
                << L"\nexpected=0"
                << L"\nactual=" << integrated.GetRightAcceleration()
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                0.0f,
                integrated.GetRightAcceleration(),
                1.0e-4f,
                message.str().c_str());
        }

    };
}
