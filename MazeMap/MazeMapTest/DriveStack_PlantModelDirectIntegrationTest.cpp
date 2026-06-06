#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    namespace
    {
        float LeftBodyWheelSpeedRadps(const VehicleState& state) noexcept
        {
            return Vehicle::WheelSpeedFromLinearVelocity(
                Vehicle::LeftWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate()));
        }

        float RightBodyWheelSpeedRadps(const VehicleState& state) noexcept
        {
            return Vehicle::WheelSpeedFromLinearVelocity(
                Vehicle::RightWheelLinearVelocityFromBody(
                    state.GetForwardVelocity(),
                    state.GetYawRate()));
        }
    }

    TEST_CLASS(DriveStack_PlantModelDirectIntegrationTest)
    {
    public:
        TEST_METHOD(ZeroDtDoesNotChangePositionX)
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

        TEST_METHOD(ZeroDtDoesNotChangePositionY)
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

        TEST_METHOD(ZeroDtDoesNotChangeYaw)
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

        TEST_METHOD(ZeroDtDoesNotChangeForwardVelocity)
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

        TEST_METHOD(ZeroDtDoesNotChangeLateralVelocity)
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

        TEST_METHOD(ZeroDtDoesNotChangeYawRate)
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

        TEST_METHOD(ZeroDtDoesNotChangeLeftWheelSpeed)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta =
                LeftBodyWheelSpeedRadps(actual) - LeftBodyWheelSpeedRadps(initial);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_body_wheel_speed_delta_radps"
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

        TEST_METHOD(ZeroDtDoesNotChangeRightWheelSpeed)
        {
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            TestRuntime runtime;
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.0f);
            const VehicleState actual = runtime.runtimeState;
            const float actualDelta =
                RightBodyWheelSpeedRadps(actual) - RightBodyWheelSpeedRadps(initial);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_body_wheel_speed_delta_radps"
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

        TEST_METHOD(NegativeDtDoesNotChangePositionX)
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

        TEST_METHOD(NonFiniteDtDoesNotChangePositionX)
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

        TEST_METHOD(SingleStepPositionXStaysFinite)
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

        TEST_METHOD(SingleStepPositionYStaysFinite)
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

        TEST_METHOD(SingleStepYawStaysFinite)
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

        TEST_METHOD(SingleStepForwardVelocityStaysFinite)
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

        TEST_METHOD(SingleStepLateralVelocityStaysFinite)
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

        TEST_METHOD(SingleStepYawRateStaysFinite)
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

        TEST_METHOD(SingleStepLeftWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = LeftBodyWheelSpeedRadps(state);
            const float initialLeftWheelSpeedRadps = LeftBodyWheelSpeedRadps(initial);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_body_wheel_speed_radps"
                << L"\ninitial=" << initialLeftWheelSpeedRadps
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initialLeftWheelSpeedRadps,
                message.str().c_str());
        }

        TEST_METHOD(SingleStepRightWheelSpeedStaysFinite)
        {
            TestRuntime runtime;
            const VehicleState initial = MakeRollingState(0.70f, 1.50f, 0.04f, 0.20f);
            runtime.runtimeState = initial;
            runtime.plant.integrate(MakeCommand(0.42f, 0.31f), 0.004f);
            const VehicleState state = runtime.runtimeState;
            const float actual = RightBodyWheelSpeedRadps(state);
            const float initialRightWheelSpeedRadps = RightBodyWheelSpeedRadps(initial);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_body_wheel_speed_radps"
                << L"\ninitial=" << initialRightWheelSpeedRadps
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                actual > initialRightWheelSpeedRadps,
                message.str().c_str());
        }

        TEST_METHOD(SubstepPositionXStaysFinite)
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

        TEST_METHOD(SubstepPositionYStaysFinite)
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

        TEST_METHOD(SubstepYawStaysFinite)
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

        TEST_METHOD(SubstepForwardVelocityStaysFinite)
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

        TEST_METHOD(SubstepLateralVelocityStaysFinite)
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

        TEST_METHOD(SubstepYawRateStaysFinite)
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

        TEST_METHOD(SubstepLeftWheelSpeedStaysFinite)
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
            const float actual = LeftBodyWheelSpeedRadps(state);
            const float initialLeftWheelSpeedRadps = LeftBodyWheelSpeedRadps(initial);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_body_wheel_speed_radps"
                << L"\ninitial=" << initialLeftWheelSpeedRadps
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initialLeftWheelSpeedRadps,
                message.str().c_str());
        }

        TEST_METHOD(SubstepRightWheelSpeedStaysFinite)
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
            const float actual = RightBodyWheelSpeedRadps(state);
            const float initialRightWheelSpeedRadps = RightBodyWheelSpeedRadps(initial);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_body_wheel_speed_radps"
                << L"\ninitial=" << initialRightWheelSpeedRadps
                << L"\nactual=" << actual
                << L"\ncriterion=actual>initial"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                actual > initialRightWheelSpeedRadps,
                message.str().c_str());
        }

        TEST_METHOD(PositionXRemainsCloseBetweenSingleStepAndSubsteps)
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

        TEST_METHOD(PositionYRemainsCloseBetweenSingleStepAndSubsteps)
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

        TEST_METHOD(ForwardVelocityRemainsCloseBetweenSingleStepAndSubsteps)
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

        TEST_METHOD(YawRateRemainsCloseBetweenSingleStepAndSubsteps)
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

    };
}
