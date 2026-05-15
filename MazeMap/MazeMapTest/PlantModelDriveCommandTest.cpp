#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelTestSupport.h"
#include "..\MazeMap\Vehicle.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        bool IsFiniteControlVector(const App::Internal::CommandVector& control) noexcept
        {
            return
                std::isfinite(control.LeftMotorPwm()) &&
                std::isfinite(control.RightMotorPwm());
        }
    }

    TEST_CLASS(PlantModelDriveCommandTest)
    {
    public:
        TEST_METHOD(PlantModelAccelerationFeedforwardZeroRequestReturnsZeroCommand)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.solveAccelerationFeedforward(0.0f, 0.0f);

            Assert::AreEqual(0.0f, control.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.0f, control.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelSteadyStateFeedforwardReturnsFiniteSymmetricCommandForForwardTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.solveSteadyStateFeedforward(0.75f, 0.0f);

            Assert::IsTrue(IsFiniteControlVector(control));
            Assert::AreEqual(control.LeftMotorPwm(), control.RightMotorPwm(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelSteadyStateFeedforwardReturnsSplitCommandForYawTarget)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.solveSteadyStateFeedforward(0.0f, 2.0f);

            Assert::IsTrue(IsFiniteControlVector(control));
            Assert::IsTrue(std::fabs(control.LeftMotorPwm() - control.RightMotorPwm()) > 1.0e-4f);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsFiniteOutputForCombinedRequest)
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant(vehicle, runtimeState);
            const App::Internal::CommandVector control =
                plant.solveAccelerationFeedforward(1.25f, 3.75f);

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
                plant.solveAccelerationFeedforward(4.0f, 0.0f);

            Assert::IsTrue(IsFiniteControlVector(control));
            Assert::AreEqual(control.LeftMotorPwm(), control.RightMotorPwm(), 1.0e-5f);
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
                slowPlant.solveAccelerationFeedforward(4.0f, 0.0f);

            VehicleState movingState;
            movingState.SetVelocity(0.75f);
			movingState.SetWheelSpeedLeft(0.0f);
			movingState.SetWheelSpeedRight(0.0f);
            PlantModel movingPlant(vehicle, movingState);
            const App::Internal::CommandVector movingControl =
                movingPlant.solveAccelerationFeedforward(4.0f, 0.0f);

            Assert::IsTrue(IsFiniteControlVector(slowControl));
            Assert::IsTrue(IsFiniteControlVector(movingControl));
			Assert::AreNotEqual(slowControl.LeftMotorPwm(), movingControl.LeftMotorPwm(), 1.0e-5f);
            Assert::AreNotEqual(slowControl.RightMotorPwm(), movingControl.RightMotorPwm(), 1.0e-5f);
            Assert::IsTrue(movingControl.Average() > slowControl.Average());
        }

        TEST_METHOD(PlantModelComputeBodyActionUsesLongitudinalLimitForPureSpeedChange)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            float desiredLongitudinalAccelMps2 = 0.0f;
            plant.ComputeBodyAction(
                0.20f,
                1.20f,
                0.0f,
                4.0f,
                0.025f,
                desiredLongitudinalAccelMps2);

            Assert::AreEqual(4.0f, desiredLongitudinalAccelMps2, 1.0e-6f);
        }

        TEST_METHOD(PlantModelComputeBodyActionFromYawRateUsesYawAccelLimitWhenRelevant)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            float desiredYawAccelRadps2 = 0.0f;
            plant.ComputeBodyActionFromYawRate(
                0.0f,
                0.0f,
                4.0f,
                25.0f,
                0.025f,
                desiredYawAccelRadps2);

            Assert::AreEqual(25.0f, desiredYawAccelRadps2, 1.0e-6f);
        }

        TEST_METHOD(PlantModelResolveWheelMotionTargetsUsesEffectiveTrackWidthAndWheelRadius)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            float leftTargetVelocityMps = 0.0f;
            float rightTargetVelocityMps = 0.0f;
            float leftTargetAccelMps2 = 0.0f;
            float rightTargetAccelMps2 = 0.0f;
            float leftTargetOmegaRadps = 0.0f;
            float rightTargetOmegaRadps = 0.0f;

            plant.resolveWheelMotionTargets(
                1.0f,
                2.0f,
                0.5f,
                3.0f,
                prepared,
                leftTargetVelocityMps,
                rightTargetVelocityMps,
                leftTargetAccelMps2,
                rightTargetAccelMps2,
                leftTargetOmegaRadps,
                rightTargetOmegaRadps);

            Assert::AreEqual(leftTargetVelocityMps / prepared.wheelRadiusM, leftTargetOmegaRadps, 1.0e-5f);
            Assert::AreEqual(rightTargetVelocityMps / prepared.wheelRadiusM, rightTargetOmegaRadps, 1.0e-5f);
            Assert::IsTrue(leftTargetVelocityMps != rightTargetVelocityMps);
            Assert::IsTrue(leftTargetAccelMps2 != rightTargetAccelMps2);
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
