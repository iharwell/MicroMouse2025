#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelDynamicsTestSupport.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace PlantModelDynamicsTestSupport;
    TEST_CLASS(PlantModelHeadingNormalizationTest)
    {
    public:
        TEST_METHOD(IntegrateHeadingDoesNotExceedPi)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(PI_F - 0.01f);
            state.SetForwardVelocity(0.5f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(6.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(0.5f));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(0.5f));
            PlantModel plant(vehicle, state);

            plant.integrate(App::Internal::CommandVector{}, 0.01f);
            std::wstringstream message;
            message << L"IntegrateHeadingDoesNotExceedPi"
                << L"\nactual=" << state.GetHeading()
                << L"\nmaximum=" << PI_F
                << L"\ncriterion=actual<=maximum";

            Assert::IsTrue(state.GetHeading() <= PI_F, message.str().c_str());
        }

        TEST_METHOD(IntegrateHeadingDoesNotGoBelowNegativePi)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.09f));
            state.SetHeading(PI_F - 0.01f);
            state.SetForwardVelocity(0.5f);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(6.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(0.5f));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(0.5f));
            PlantModel plant(vehicle, state);

            plant.integrate(App::Internal::CommandVector{}, 0.01f);
            std::wstringstream message;
            message << L"IntegrateHeadingDoesNotGoBelowNegativePi"
                << L"\nactual=" << state.GetHeading()
                << L"\nminimum=" << -PI_F
                << L"\ncriterion=actual>=minimum";

            Assert::IsTrue(state.GetHeading() >= -PI_F, message.str().c_str());
        }

    };
}
