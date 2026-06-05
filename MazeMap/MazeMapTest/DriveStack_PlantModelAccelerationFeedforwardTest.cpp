#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelAccelerationFeedforwardTest)
    {
    public:
        TEST_METHOD(ForwardAccelerationLeftCommandIsFinite)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    0.80f,
                    0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=forward_accel_left_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)";

            Assert::IsTrue(
                std::isfinite(command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(ForwardAccelerationRightCommandIsFinite)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    0.80f,
                    0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=forward_accel_right_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(right)";

            Assert::IsTrue(
                std::isfinite(command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(ReverseAccelerationLeftCommandIsFinite)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    -0.80f,
                    0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=reverse_accel_left_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)";

            Assert::IsTrue(
                std::isfinite(command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(ReverseAccelerationRightCommandIsFinite)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    -0.80f,
                    0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=reverse_accel_right_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(right)";

            Assert::IsTrue(
                std::isfinite(command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(ClockwiseYawLeftCommandIsFinite)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    0.0f,
                    8.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=yaw_accel_left_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)";

            Assert::IsTrue(
                std::isfinite(command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(ClockwiseYawRightCommandIsFinite)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    0.0f,
                    8.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=yaw_accel_right_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(right)";

            Assert::IsTrue(
                std::isfinite(command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(PositiveForwardAccelerationCommandsMoreAverageThanReverse)
        {
            TestRuntime runtime;
            const VehicleState state = MakeRollingState(0.60f, 0.0f);
            const App::Internal::CommandVector forward =
                SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
            const App::Internal::CommandVector reverse =
                SolveAccelerationFeedforwardAt(runtime, state, -0.80f, 0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=average_command"
                << L"\nforward_average=" << forward.Average()
                << L"\nreverse_average=" << reverse.Average()
                << L"\ncriterion=forward_average>reverse_average";

            Assert::IsTrue(
                forward.Average() > reverse.Average(),
                message.str().c_str());
        }

        TEST_METHOD(PositiveClockwiseYawCommandsLeftGreaterThanRight)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector clockwise =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(0.60f, 0.0f),
                    0.0f,
                    8.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=clockwise_command_differential"
                << L"\nactual=" << clockwise.Differential()
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << clockwise.LeftCommand()
                << L"\nright_command=" << clockwise.RightCommand();

            Assert::IsTrue(
                clockwise.Differential() > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(LongRunForwardSolveLeftCommandStaysFinite)
        {
            const App::Internal::CommandVector command =
                LongRunForwardFirstNonFiniteLeftOrFinalSolveCommand();
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_solve_left_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)";

            Assert::IsTrue(
                std::isfinite(command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(LongRunForwardSolveRightCommandStaysFinite)
        {
            const App::Internal::CommandVector command =
                LongRunForwardFirstNonFiniteRightOrFinalSolveCommand();
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_solve_right_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(right)";

            Assert::IsTrue(
                std::isfinite(command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(LongRunPositionXStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_position_x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunPositionYStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_position_y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunYawStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_yaw_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunForwardVelocityStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunLateralVelocityStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunYawRateStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunLeftWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunRightWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
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
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(LongRunPositiveAccelerationIncreasesForwardVelocity)
        {
            const float actualDelta = LongRunForwardVelocityDelta();
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_velocity_delta_mps"
                << L"\nactual=" << actualDelta
                << L"\ncriterion=actual>0.05";

            Assert::IsTrue(
                actualDelta > 0.05f,
                message.str().c_str());
        }
    };
}
