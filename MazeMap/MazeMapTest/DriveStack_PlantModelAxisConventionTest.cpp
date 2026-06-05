#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_PlantModelPhysicsTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackPlantModelPhysicsTestSupport;

    TEST_CLASS(DriveStack_PlantModelAxisConventionTest)
    {
    public:
        TEST_METHOD(HeadingZeroForwardMovesWorldPositiveY)
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

        TEST_METHOD(HeadingZeroForwardDoesNotDriftWorldX)
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

        TEST_METHOD(HeadingRightForwardMovesWorldPositiveX)
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

        TEST_METHOD(HeadingRightForwardDoesNotDriftWorldY)
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

        TEST_METHOD(BodyPositiveLateralMovesWorldPositiveX)
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

        TEST_METHOD(PositiveYawRateIncreasesClockwiseYaw)
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

    };
}
