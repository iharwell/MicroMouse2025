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
    TEST_CLASS(PlantModelDriveBasicsTest)
    {
    public:
        TEST_METHOD(LeftDriveWheelInertiaUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.leftDriveEquivalentWheelInertiaKgM2();
            std::wstringstream message;
            message << L"LeftDriveWheelInertiaUsesConstructionValue"
                << L"\nexpected=1.177e-6"
                << L"\nactual=" << actual
                << L"\ntolerance=1e-10";

            Assert::AreEqual(
                1.177e-6f,
                actual,
                1.0e-10f,
                message.str().c_str());
        }

        TEST_METHOD(RightDriveWheelInertiaUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.rightDriveEquivalentWheelInertiaKgM2();
            std::wstringstream message;
            message << L"RightDriveWheelInertiaUsesConstructionValue"
                << L"\nexpected=1.177e-6"
                << L"\nactual=" << actual
                << L"\ntolerance=1e-10";

            Assert::AreEqual(
                1.177e-6f,
                actual,
                1.0e-10f,
                message.str().c_str());
        }

        TEST_METHOD(LeftDriveLongitudinalTireStiffnessUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.leftDriveLongitudinalTireStiffnessN();
            std::wstringstream message;
            message << L"LeftDriveLongitudinalTireStiffnessUsesConstructionValue"
                << L"\nexpected=4.12"
                << L"\nactual=" << actual
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                4.12f,
                actual,
                0.01f,
                message.str().c_str());
        }

        TEST_METHOD(RightDriveLongitudinalTireStiffnessUsesConstructionValue)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);

            const float actual = plant.rightDriveLongitudinalTireStiffnessN();
            std::wstringstream message;
            message << L"RightDriveLongitudinalTireStiffnessUsesConstructionValue"
                << L"\nexpected=4.12"
                << L"\nactual=" << actual
                << L"\ntolerance=0.01";

            Assert::AreEqual(
                4.12f,
                actual,
                0.01f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricDrivePreservesYawRate)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = 2.5f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);
            const float initialYawRateRadps = state.GetYawRate();

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.55f);
            control.SetRightCommand(0.55f);

            plant.integrate(control, 0.001f);
            const float actualYawRateRadps = state.GetYawRate();
            std::wstringstream message;
            message << L"SymmetricDrivePreservesYawRate"
                << L"\ninitial_yaw_rate_radps=" << initialYawRateRadps
                << L"\nactual_yaw_rate_radps=" << actualYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initialYawRateRadps,
                actualYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(BrakeCommandAddsElectricalRetardingForceAtLowForwardSpeed)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = 0.09f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector coastCommand{};
            const App::Internal::CommandVector brakeCommand = App::Internal::CommandVector::Brake();
            const float coastForceN = plant.totalForwardContactForceN(coastCommand);
            const float brakeForceN = plant.totalForwardContactForceN(brakeCommand);
            const float addedRetardingForceN = coastForceN - brakeForceN;

            std::wstringstream message;
            message << L"BrakeCommandAddsElectricalRetardingForceAtLowForwardSpeed"
                << L"\ncoast_force_n=" << coastForceN
                << L"\nbrake_force_n=" << brakeForceN
                << L"\nadded_retarding_force_n=" << addedRetardingForceN
                << L"\ncriterion=added_retarding_force_n>0.02";

            Assert::IsTrue(
                std::isfinite(brakeForceN) && (addedRetardingForceN > 0.02f),
                message.str().c_str());
        }

        TEST_METHOD(BrakeCommandAddsElectricalRetardingForceAtLowReverseSpeed)
        {
            Vehicle vehicle;
            vehicle.SetFanDuty(0.80f);

            constexpr float forwardVelocityMps = -0.09f;
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(0.0f, 0.0f));
            state.SetHeading(0.0f);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(0.0f);
            state.SetYawRate(0.0f);
            state.SetWheelSpeedLeft(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            state.SetWheelSpeedRight(Vehicle::WheelSpeedFromLinearVelocity(forwardVelocityMps));
            PlantModel plant(vehicle, state);

            const App::Internal::CommandVector coastCommand{};
            const App::Internal::CommandVector brakeCommand = App::Internal::CommandVector::Brake();
            const float coastForceN = plant.totalForwardContactForceN(coastCommand);
            const float brakeForceN = plant.totalForwardContactForceN(brakeCommand);
            const float addedRetardingForceN = brakeForceN - coastForceN;

            std::wstringstream message;
            message << L"BrakeCommandAddsElectricalRetardingForceAtLowReverseSpeed"
                << L"\ncoast_force_n=" << coastForceN
                << L"\nbrake_force_n=" << brakeForceN
                << L"\nadded_retarding_force_n=" << addedRetardingForceN
                << L"\ncriterion=added_retarding_force_n>0.02";

            Assert::IsTrue(
                std::isfinite(brakeForceN) && (addedRetardingForceN > 0.02f),
                message.str().c_str());
        }

    };
}
