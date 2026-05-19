#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelTestSupport.h"
#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\Vehicle.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kAccelerationToleranceMps2 = 0.10f;
        constexpr float kYawAccelerationToleranceRadps2 = 10.0f * kPi / 180.0f;

        bool IsFiniteControlVector(const App::Internal::CommandVector& control) noexcept
        {
            return
                std::isfinite(control.LeftCommand()) &&
                std::isfinite(control.RightCommand());
        }

        float IntegratedRateOfChange(
            const VehicleState::StateVector& before,
            const VehicleState::StateVector& after,
            const int stateIndex,
            const float dtSeconds) noexcept
        {
			Assert::AreEqual(0.001f, dtSeconds, 1.0e-6f, L"Unexpected time step. Should be 1ms.");
            return (after(stateIndex) - before(stateIndex)) / dtSeconds;
        }
    }

    TEST_CLASS(PlantModelDriveCommandTest)
    {
    public:

        static constexpr float dtSeconds = 0.001f;
        TEST_METHOD(PlantModelAccelerationFeedforwardZeroRequestReturnsZeroCommand)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(0.0f, 0.0f);

            Assert::AreEqual(0.0f, control.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(0.0f, control.RightCommand(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsFiniteSymmetricCommandForForwardRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(1.0f, 0.0f);

            Assert::IsTrue(IsFiniteControlVector(control));
            Assert::AreEqual(control.LeftCommand(), control.RightCommand(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsSplitCommandForYawRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(0.0f, 8.0f);

            Assert::IsTrue(IsFiniteControlVector(control));
            Assert::IsTrue(std::fabs(control.LeftCommand() - control.RightCommand()) > 1.0e-4f);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsFiniteOutputForCombinedRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(1.25f, 3.75f);

            Assert::IsTrue(IsFiniteControlVector(control));
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardIgnoresObservedWheelMismatchForForwardRequest)
        {
            Vehicle vehicle;
            VehicleState state;
            PlantModel plant(vehicle, state);
            state.SetVelocity(0.35f);
            state.SetRotationalVelocity(0.0f);
            state.SetWheelSpeedLeft(-30.0f);
            state.SetWheelSpeedRight(70.0f);

            const App::Internal::CommandVector control =
                plant.ComputeFeedforward(4.0f, 0.0f);

            Assert::IsTrue(IsFiniteControlVector(control));
            Assert::AreEqual(control.LeftCommand(), control.RightCommand(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardAccountsForForwardVelocityBackEmf)
        {
            Vehicle vehicle;
            VehicleState restState;
            restState.SetVelocity(0.25f);

            // We deliberately set the wheel speeds higher on the slow state to ensure back-emf is not looking at the wheel speeds.
			float s_left = 0.0f, s_right = 0.0f;
            
			vehicle.WheelOmegasFromBodyVelocity(
				restState.GetVelocity(),
				restState.GetRotationalVelocity(),
				s_left,
				s_right);
            restState.SetWheelSpeedLeft(s_left);
            restState.SetWheelSpeedRight(s_right);
            PlantModel slowPlant(vehicle, restState);
            const App::Internal::CommandVector slowControl =
                slowPlant.ComputeFeedforward(4.0f, 0.0f);

            VehicleState movingState;
            movingState.SetVelocity(0.75f);
			movingState.SetWheelSpeedLeft(0.0f);
            movingState.SetWheelSpeedRight(0.0f);
            PlantModel movingPlant(vehicle, movingState);
            const App::Internal::CommandVector movingControl =
                movingPlant.ComputeFeedforward(4.0f, 0.0f);

            Assert::IsTrue(IsFiniteControlVector(slowControl));
            Assert::IsTrue(IsFiniteControlVector(movingControl));
			Assert::AreNotEqual(slowControl.LeftCommand(), movingControl.LeftCommand(), 1.0e-5f);
            Assert::AreNotEqual(slowControl.RightCommand(), movingControl.RightCommand(), 1.0e-5f);
            Assert::IsTrue(movingControl.Average() > slowControl.Average());
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardForLongitudinalRequestIncreasesForwardVelocity)
        {
            constexpr float requestedAccelMps2 = 4.0f;
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 0.20f;
            state(VehicleState::kOmegaL) = state(VehicleState::kU) / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = state(VehicleState::kU) / params.wheelRadiusM;

            runtime.runtimeState.SetVelocity(state(VehicleState::kU));
            runtime.runtimeState.SetWheelSpeedLeft(state(VehicleState::kOmegaL));
            runtime.runtimeState.SetWheelSpeedRight(state(VehicleState::kOmegaR));
            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(requestedAccelMps2, 0.0f);
            const PlantDerivatives derivatives =
                plant.forwardStep(state, command, params);
            const VehicleState::StateVector integrated =
                plant.integrate(state, command, dtSeconds, params);
            const float integratedForwardAccelMps2 =
                IntegratedRateOfChange(state, integrated, VehicleState::kU, dtSeconds);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(derivatives.longitudinalAccelMps2 > 0.0f);
            Assert::AreEqual(
                requestedAccelMps2,
                derivatives.longitudinalAccelMps2,
                kAccelerationToleranceMps2);
            Assert::AreEqual(
                requestedAccelMps2,
                integratedForwardAccelMps2,
                kAccelerationToleranceMps2);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardForClockwiseYawRequestIncreasesYawRate)
        {
            constexpr float requestedYawAccelRadps2 = 25.0f;
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = VehicleState::StateVector::Zero();

            const App::Internal::CommandVector command =
                plant.ComputeFeedforward(0.0f, requestedYawAccelRadps2);
            const PlantDerivatives derivatives =
                plant.forwardStep(state, command, params);
            const VehicleState::StateVector integrated =
                plant.integrate(state, command, dtSeconds, params);
            const float integratedYawAccelRadps2 =
                IntegratedRateOfChange(state, integrated, VehicleState::kR, dtSeconds);

            Assert::IsTrue(IsFiniteControlVector(command));
            Assert::IsTrue(derivatives.yawAccelRadps2 > 0.0f);
            Assert::AreEqual(
                requestedYawAccelRadps2,
                derivatives.yawAccelRadps2,
                kYawAccelerationToleranceRadps2);
            Assert::AreEqual(
                requestedYawAccelRadps2,
                integratedYawAccelRadps2,
                kYawAccelerationToleranceRadps2);
        }

        TEST_METHOD(PlantModelWheelProjectionRoundTripsVehicleBodyVelocity)
        {
            PlantModelTestRuntime runtime;
            constexpr float forwardMps = 1.0f;
            constexpr float yawRateRadps = 2.0f;
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = forwardMps;
            state(VehicleState::kR) = yawRateRadps;
            const Eigen::Vector2f wheelVelocityMps =
                runtime.plant.wheelLinearVelocityFromBodyState(state);
            EncoderObs observation{};
            observation.omegaLeftRadps = Vehicle::WheelOmegaFromLinearVelocity(wheelVelocityMps.x());
            observation.omegaRightRadps = Vehicle::WheelOmegaFromLinearVelocity(wheelVelocityMps.y());

            Assert::AreEqual(forwardMps, runtime.plant.measuredLinearSpeedMps(observation), 1.0e-6f);
            Assert::AreEqual(yawRateRadps, runtime.plant.measuredYawRateRadps(observation), 1.0e-6f);
            Assert::IsTrue(wheelVelocityMps.x() > wheelVelocityMps.y());
        }

        TEST_METHOD(PlantModelVelocityTargetTechnicalLimitsReportFinitePositiveEnvelope)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            float maxLongitudinalAccelMps2 = 0.0f;
            float maxYawAccelRadps2 = 0.0f;

            plant.velocityTargetTechnicalLimits(
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2);

            Assert::IsTrue(std::isfinite(maxLongitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(maxYawAccelRadps2));
            Assert::IsTrue(maxLongitudinalAccelMps2 > 0.0f);
            Assert::IsTrue(maxYawAccelRadps2 > 0.0f);
        }

    };
}
