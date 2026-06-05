#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelNumericStabilityTest)
    {
    public:
        TEST_METHOD(PositionXStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetPositionX()))
                {
                    break;
                }
            }
            const float actual = state.GetPositionX();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=position_x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(PositionYStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetPositionY()))
                {
                    break;
                }
            }
            const float actual = state.GetPositionY();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=position_y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(YawStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetHeading()))
                {
                    break;
                }
            }
            const float actual = state.GetHeading();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(ForwardVelocityStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetForwardVelocity()))
                {
                    break;
                }
            }
            const float actual = state.GetForwardVelocity();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LateralVelocityStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetRightwardVelocity()))
                {
                    break;
                }
            }
            const float actual = state.GetRightwardVelocity();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(YawRateStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetYawRate()))
                {
                    break;
                }
            }
            const float actual = state.GetYawRate();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LeftWheelSpeedStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetWheelSpeedLeft()))
                {
                    break;
                }
            }
            const float actual = state.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(RightWheelSpeedStaysFiniteUnderPlausibleHighCommand)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (!std::isfinite(state.GetWheelSpeedRight()))
                {
                    break;
                }
            }
            const float actual = state.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(HeadingDoesNotGoBelowNegativePi)
        {
            const float actual = HighCommandFirstBelowMinimumOrFinalHeading();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\nminimum=" << -PI_F
                << L"\ncriterion=actual>=minimum";

            Assert::IsTrue(
                actual >= -PI_F,
                message.str().c_str());
        }

        TEST_METHOD(HeadingDoesNotExceedPi)
        {
            const float actual = HighCommandFirstAboveMaximumOrFinalHeading();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\nmaximum=" << PI_F
                << L"\ncriterion=actual<=maximum";

            Assert::IsTrue(
                actual <= PI_F,
                message.str().c_str());
        }

        TEST_METHOD(FinalForwardVelocityStaysWithinPlausibleBounds)
        {
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
			Vehicle vehicle = Vehicle();
			PlantModel plant = PlantModel(vehicle, state);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                plant.integrate(command, kDirectDtSeconds);
            }
            const float actual = state.GetForwardVelocity();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<20";

            Assert::IsTrue(
                std::fabs(actual) < 20.0f,
                message.str().c_str());
        }

        TEST_METHOD(FinalYawRateStaysWithinPlausibleBounds)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            const float actual = state.GetYawRate();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<200";

            Assert::IsTrue(
                std::fabs(actual) < 200.0f,
                message.str().c_str());
        }

        TEST_METHOD(FinalLeftWheelSpeedStaysWithinPlausibleBounds)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            const float actual = state.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<3000";

            Assert::IsTrue(
                std::fabs(actual) < 3000.0f,
                message.str().c_str());
        }

        TEST_METHOD(FinalRightWheelSpeedStaysWithinPlausibleBounds)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);
            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }
            const float actual = state.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<3000";

            Assert::IsTrue(
                std::fabs(actual) < 3000.0f,
                message.str().c_str());
        }

    };
}
