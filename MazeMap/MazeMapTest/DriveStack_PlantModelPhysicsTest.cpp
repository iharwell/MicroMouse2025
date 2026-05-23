#include "pch.h"
#include "CppUnitTest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>

#include "..\MazeMap\CommandVector.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\EigenCompat.h"
#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\Maze.h"
#include "..\MazeMap\SensorMount.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"
#include "..\MazeMap\PlantModel.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kDirectDtSeconds = 0.001f;

        struct TestRuntime final
        {
            Vehicle vehicle{};
            VehicleState runtimeState{};
            PlantModel plant;

            explicit TestRuntime(float fanDuty = 0.80f) noexcept
                : plant(vehicle, runtimeState)
            {
                vehicle.SetFanDuty(fanDuty);
            }
        };

        VehicleState MakeState(
            float xM,
            float yM,
            float yawRad,
            float forwardVelocityMps,
            float lateralVelocityMps,
            float yawRateRadps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps) noexcept
        {
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(xM, yM));
            state.SetHeading(yawRad);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(lateralVelocityMps);
            state.SetYawRate(yawRateRadps);
            state.SetWheelSpeedLeft(leftWheelSpeedRadps);
            state.SetWheelSpeedRight(rightWheelSpeedRadps);
            return state;
        }

        VehicleState MakeRollingState(
            float forwardVelocityMps,
            float yawRateRadps,
            float lateralVelocityMps = 0.0f,
            float yawRad = 0.0f) noexcept
        {
            return MakeState(
                0.0f,
                0.0f,
                yawRad,
                forwardVelocityMps,
                lateralVelocityMps,
                yawRateRadps,
                Vehicle::WheelSpeedFromLinearVelocity(
                    Vehicle::LeftWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps)),
                Vehicle::WheelSpeedFromLinearVelocity(
                    Vehicle::RightWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps)));
        }

        App::Internal::CommandVector MakeCommand(float left, float right) noexcept
        {
            App::Internal::CommandVector command{};
            command.SetLeftCommand(left);
            command.SetRightCommand(right);
            return command;
        }

        App::Internal::CommandVector SolveAccelerationFeedforwardAt(
            TestRuntime& runtime,
            const VehicleState& state,
            float forwardAccelMps2,
            float yawAccelRadps2) noexcept
        {
            runtime.runtimeState = state;
            return runtime.plant.ComputeFeedforward(forwardAccelMps2, yawAccelRadps2);
        }

        float HighCommandFirstOutOfRangeOrFinalHeading()
        {
            TestRuntime runtime;
            VehicleState state =
                MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);

            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (state.GetHeading() < -PI_F || state.GetHeading() > PI_F)
                {
                    return state.GetHeading();
                }
            }

            return state.GetHeading();
        }

        App::Internal::CommandVector LongRunForwardFirstNonFiniteOrFinalSolveCommand()
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            App::Internal::CommandVector command{};

            for (int tick = 0; tick < 500; ++tick)
            {
                command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                if (!command.IsFinite())
                {
                    return command;
                }
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }

            return command;
        }

        float LongRunForwardVelocityDelta()
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            const float initialForwardMps = state.GetForwardVelocity();

            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }

            return state.GetForwardVelocity() - initialForwardMps;
        }
    }

    TEST_CLASS(DriveStack_PlantModelPhysicsTest)
    {
    public:
        TEST_METHOD(AxisConvention_HeadingZeroForwardMovesWorldPositiveY)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector coast{};
            const VehicleState state =
                MakeRollingState(0.50f, 0.0f, 0.0f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(coast, 0.004f);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_y_m"
                << L"\ninitial=" << state.GetPositionY()
                << L"\nactual=" << integrated.GetPositionY()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated.GetPositionY() > state.GetPositionY(),
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_HeadingZeroForwardDoesNotDriftWorldX)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector coast{};
            const VehicleState state =
                MakeRollingState(0.50f, 0.0f, 0.0f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(coast, 0.004f);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_x_m"
                << L"\nexpected=" << state.GetPositionX()
                << L"\nactual=" << integrated.GetPositionX()
                << L"\ntolerance=2e-4";

            Assert::AreEqual(
                state.GetPositionX(),
                integrated.GetPositionX(),
                2.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_HeadingRightForwardMovesWorldPositiveX)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector coast{};
            const VehicleState state =
                MakeRollingState(0.50f, 0.0f, 0.0f, 0.5f * PI_F);
            runtime.runtimeState = state;
            runtime.plant.integrate(coast, 0.004f);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_x_m"
                << L"\ninitial=" << state.GetPositionX()
                << L"\nactual=" << integrated.GetPositionX()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated.GetPositionX() > state.GetPositionX(),
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_HeadingRightForwardDoesNotDriftWorldY)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector coast{};
            const VehicleState state =
                MakeRollingState(0.50f, 0.0f, 0.0f, 0.5f * PI_F);
            runtime.runtimeState = state;
            runtime.plant.integrate(coast, 0.004f);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_y_m"
                << L"\nexpected=" << state.GetPositionY()
                << L"\nactual=" << integrated.GetPositionY()
                << L"\ntolerance=2e-4";

            Assert::AreEqual(
                state.GetPositionY(),
                integrated.GetPositionY(),
                2.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_BodyPositiveLateralMovesWorldPositiveX)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector coast{};
            const VehicleState state =
                MakeState(0.0f, 0.0f, 0.0f, 0.0f, 0.30f, 0.0f, 0.0f, 0.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(coast, 0.004f);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_x_m"
                << L"\ninitial=" << state.GetPositionX()
                << L"\nactual=" << integrated.GetPositionX()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated.GetPositionX() > state.GetPositionX(),
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_PositiveYawRateIncreasesClockwiseYaw)
        {
            TestRuntime runtime;
            const App::Internal::CommandVector coast{};
            const VehicleState state =
                MakeRollingState(0.25f, 1.20f);
            runtime.runtimeState = state;
            runtime.plant.integrate(coast, 0.004f);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=yaw_rad"
                << L"\ninitial=" << state.GetHeading()
                << L"\nactual=" << integrated.GetHeading()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated.GetHeading() > state.GetHeading(),
                message.str().c_str());
        }

        TEST_METHOD(WheelYawSign_PositiveYawMakesLeftWheelLinearVelocityFaster)
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

        TEST_METHOD(WheelYawSign_EncoderYawMeasurementPreservesClockwisePositiveSign)
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

        TEST_METHOD(WheelYawSign_PositiveTargetYawRateRequestsFasterLeftWheel)
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

        TEST_METHOD(WheelYawSign_PositiveTargetYawAccelerationRequestsMoreLeftAcceleration)
        {
            TestRuntime runtime;
            const VehicleState state =
                MakeRollingState(0.40f, 2.0f);
            const App::Internal::CommandVector command =
                runtime.plant.ComputeFeedforward(0.0f, 5.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(command, kDirectDtSeconds);
            const VehicleState integrated = runtime.runtimeState;
            std::wstringstream commandMessage;
            commandMessage << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=command_differential"
                << L"\nactual=" << command.Differential()
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand();
            std::wstringstream accelMessage;
            accelMessage << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=yaw_rate_after_step_radps"
                << L"\ninitial=" << state.GetYawRate()
                << L"\nactual=" << integrated.GetYawRate()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                command.Differential() > 0.0f,
                commandMessage.str().c_str());
            Assert::IsTrue(
                integrated.GetYawRate() > state.GetYawRate(),
                accelMessage.str().c_str());
        }

        TEST_METHOD(SymmetricDrive_LeftRightForwardForceSymmetry)
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

        TEST_METHOD(SymmetricDrive_WheelAccelerationSymmetry)
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

        TEST_METHOD(SymmetricDrive_LeftRightWheelSpeedSymmetryAfterStep)
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

        TEST_METHOD(SymmetricDrive_WheelSpeedSymmetryPersistsAcrossTicks)
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

        TEST_METHOD(SymmetricDrive_NoYawAccelerationBias)
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

        TEST_METHOD(SymmetricDrive_NoLateralAccelerationBias)
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

        TEST_METHOD(Stiction_SubthresholdDriveDoesNotAccelerateLeftWheelAtRest)
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

        TEST_METHOD(Stiction_SubthresholdDriveDoesNotAccelerateRightWheelAtRest)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftPositionX)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftPositionY)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftYaw)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftForwardVelocity)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftLateralVelocity)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftYawRate)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftLeftWheelSpeed)
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

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftRightWheelSpeed)
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

        TEST_METHOD(FanLoad_NoFanContactNormalSumMatchesConfiguredLoad)
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

        TEST_METHOD(FanLoad_FanOnContactNormalSumMatchesConfiguredLoad)
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

        TEST_METHOD(FanLoad_FanDutyIncreasesTotalContactNormalLoad)
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

        TEST_METHOD(FanLoad_Contact0NormalIncreasesWithFanDuty)
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

        TEST_METHOD(FanLoad_Contact1NormalIncreasesWithFanDuty)
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

        TEST_METHOD(FanLoad_Contact2NormalIncreasesWithFanDuty)
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

        TEST_METHOD(FanLoad_Contact3NormalIncreasesWithFanDuty)
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

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangePositionX)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetPositionX() - initial.GetPositionX();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangePositionY)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetPositionY() - initial.GetPositionY();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeYaw)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetHeading() - initial.GetHeading();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_delta_rad"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeForwardVelocity)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetForwardVelocity() - initial.GetForwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_delta_mps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeLateralVelocity)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetRightwardVelocity() - initial.GetRightwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=lateral_velocity_delta_mps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeYawRate)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetYawRate() - initial.GetYawRate();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_delta_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeLeftWheelSpeed)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetWheelSpeedLeft() - initial.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_wheel_speed_delta_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeRightWheelSpeed)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetWheelSpeedRight() - initial.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_wheel_speed_delta_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_NegativeDtDoesNotChangePositionX)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), -0.001f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetPositionX() - initial.GetPositionX();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=-0.001";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_NonFiniteDtDoesNotChangePositionX)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), (std::numeric_limits<float>::quiet_NaN)());
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta = actual.GetPositionX() - initial.GetPositionX();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=nan";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepPositionXStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetPositionX();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_m"
                << L"\ninitial=" << initial.GetPositionX()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetPositionX(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepPositionYStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetPositionY();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_m"
                << L"\ninitial=" << initial.GetPositionY()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetPositionY(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepYawStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetHeading();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rad"
                << L"\ninitial=" << initial.GetHeading()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetHeading(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepForwardVelocityStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetForwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_mps"
                << L"\ninitial=" << initial.GetForwardVelocity()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetForwardVelocity(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepLateralVelocityStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetRightwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=lateral_velocity_mps"
                << L"\ninitial=" << initial.GetRightwardVelocity()
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<abs(initial)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::fabs(actual) < std::fabs(initial.GetRightwardVelocity()),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepYawRateStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetYawRate();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_radps"
                << L"\ninitial=" << initial.GetYawRate()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetYawRate(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepLeftWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_wheel_speed_radps"
                << L"\ninitial=" << initial.GetWheelSpeedLeft()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetWheelSpeedLeft(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepRightWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = state.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_wheel_speed_radps"
                << L"\ninitial=" << initial.GetWheelSpeedRight()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initial.GetWheelSpeedRight(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepPositionXStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetPositionX();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_m"
                << L"\ninitial=" << initial.GetPositionX()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetPositionX(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepPositionYStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetPositionY();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_m"
                << L"\ninitial=" << initial.GetPositionY()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetPositionY(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepYawStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetHeading();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rad"
                << L"\ninitial=" << initial.GetHeading()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetHeading(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepForwardVelocityStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetForwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_mps"
                << L"\ninitial=" << initial.GetForwardVelocity()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetForwardVelocity(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepLateralVelocityStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetRightwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=lateral_velocity_mps"
                << L"\ninitial=" << initial.GetRightwardVelocity()
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<abs(initial)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::fabs(actual) < std::fabs(initial.GetRightwardVelocity()),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepYawRateStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetYawRate();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_radps"
                << L"\ninitial=" << initial.GetYawRate()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetYawRate(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepLeftWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_wheel_speed_radps"
                << L"\ninitial=" << initial.GetWheelSpeedLeft()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetWheelSpeedLeft(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepRightWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            VehicleState state = initial;
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                state = runtime.runtimeState;
            }
            const float actual = state.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_wheel_speed_radps"
                << L"\ninitial=" << initial.GetWheelSpeedRight()
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initial.GetWheelSpeedRight(),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_PositionXRemainsCloseBetweenSingleStepAndSubsteps)
        {
            TestRuntime runtime;
            runtime.runtimeState = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState singleStep = runtime.runtimeState;
            VehicleState substeps = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = substeps;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                substeps = runtime.runtimeState;
            }
            const float actualDelta = singleStep.GetPositionX() - substeps.GetPositionX();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_single_minus_substeps_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.003";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                3.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_PositionYRemainsCloseBetweenSingleStepAndSubsteps)
        {
            TestRuntime runtime;
            runtime.runtimeState = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState singleStep = runtime.runtimeState;
            VehicleState substeps = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = substeps;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                substeps = runtime.runtimeState;
            }
            const float actualDelta = singleStep.GetPositionY() - substeps.GetPositionY();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_single_minus_substeps_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.003";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                3.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ForwardVelocityRemainsCloseBetweenSingleStepAndSubsteps)
        {
            TestRuntime runtime;
            runtime.runtimeState = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState singleStep = runtime.runtimeState;
            VehicleState substeps = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = substeps;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                substeps = runtime.runtimeState;
            }
            const float actualDelta = singleStep.GetForwardVelocity() - substeps.GetForwardVelocity();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_single_minus_substeps_mps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.08";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                8.0e-2f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_YawRateRemainsCloseBetweenSingleStepAndSubsteps)
        {
            TestRuntime runtime;
            runtime.runtimeState = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState singleStep = runtime.runtimeState;
            VehicleState substeps = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            for (int step = 0; step < 4; ++step)
            {
                runtime.runtimeState = substeps;
                runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.001f);
                substeps = runtime.runtimeState;
            }
            const float actualDelta =
                singleStep.GetYawRate() - substeps.GetYawRate();
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_single_minus_substeps_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.8";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                8.0e-1f,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_PositionXStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_PositionYStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_YawStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_ForwardVelocityStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_LateralVelocityStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_YawRateStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_LeftWheelSpeedStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_RightWheelSpeedStaysFiniteUnderPlausibleHighCommand)
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

        TEST_METHOD(NumericStability_HeadingStaysNormalizedUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstOutOfRangeOrFinalHeading();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\nminimum=" << -PI_F
                << L"\nmaximum=" << PI_F
                << L"\ncriterion=minimum<=actual<=maximum";

            Assert::IsTrue(
                actual >= -PI_F && actual <= PI_F,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_FinalForwardVelocityStaysWithinPlausibleBounds)
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

        TEST_METHOD(NumericStability_FinalYawRateStaysWithinPlausibleBounds)
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

        TEST_METHOD(NumericStability_FinalLeftWheelSpeedStaysWithinPlausibleBounds)
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

        TEST_METHOD(NumericStability_FinalRightWheelSpeedStaysWithinPlausibleBounds)
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

        TEST_METHOD(AccelerationFeedforward_ForwardAccelerationCommandIsFinite)
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
                << L"\nfield=forward_accel_feedforward_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ReverseAccelerationCommandIsFinite)
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
                << L"\nfield=reverse_accel_feedforward_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ClockwiseYawCommandIsFinite)
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
                << L"\nfield=yaw_accel_feedforward_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_PositiveForwardAccelerationCommandsMoreAverageThanReverse)
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

        TEST_METHOD(AccelerationFeedforward_PositiveClockwiseYawCommandsLeftGreaterThanRight)
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

        TEST_METHOD(AccelerationFeedforward_LongRunForwardSolveCommandStaysFinite)
        {
            const App::Internal::CommandVector command =
                LongRunForwardFirstNonFiniteOrFinalSolveCommand();
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_solve_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositionXStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunPositionYStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunYawStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunForwardVelocityStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunLateralVelocityStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunYawRateStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunLeftWheelSpeedStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunRightWheelSpeedStaysFinite)
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

        TEST_METHOD(AccelerationFeedforward_LongRunPositiveAccelerationIncreasesForwardVelocity)
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
