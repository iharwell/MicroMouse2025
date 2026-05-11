#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelTestSupport.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        bool IsFiniteSolution(const DriveCommandSolution& solution) noexcept
        {
            return
                std::isfinite(solution.control.LeftMotorPwm()) &&
                std::isfinite(solution.control.RightMotorPwm()) &&
                std::isfinite(solution.fanDutyCycle) &&
                std::isfinite(solution.batteryVoltageV) &&
                std::isfinite(solution.leftContactForceN) &&
                std::isfinite(solution.rightContactForceN);
        }
    }

    TEST_CLASS(PlantModelDriveCommandTest)
    {
    public:
        TEST_METHOD(PlantModelAccelerationFeedforwardZeroRequestReturnsZeroCommand)
        {
            PlantModel plant;
            const DriveCommandSolution solution =
                plant.solveAccelerationFeedforward(0.0f, 0.0f, 0.80f, PlantParams::Default().supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(solution.converged);
            Assert::AreEqual(0.0f, solution.control.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.0f, solution.control.RightMotorPwm(), 1.0e-6f);
        }

        TEST_METHOD(PlantModelSteadyStateFeedforwardReturnsFiniteSymmetricCommandForForwardTarget)
        {
            PlantModel plant;
            const DriveCommandSolution solution =
                plant.solveSteadyStateFeedforward(0.75f, 0.0f, 0.80f, PlantParams::Default().supplyVoltageV);

            Assert::IsTrue(IsFiniteSolution(solution));
            Assert::AreEqual(solution.control.LeftMotorPwm(), solution.control.RightMotorPwm(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelSteadyStateFeedforwardReturnsSplitCommandForYawTarget)
        {
            PlantModel plant;
            const DriveCommandSolution solution =
                plant.solveSteadyStateFeedforward(0.0f, 2.0f, 0.80f, PlantParams::Default().supplyVoltageV);

            Assert::IsTrue(IsFiniteSolution(solution));
            Assert::IsTrue(std::fabs(solution.control.LeftMotorPwm() - solution.control.RightMotorPwm()) > 1.0e-4f);
        }

        TEST_METHOD(PlantModelAccelerationFeedforwardReturnsFiniteOutputForCombinedRequest)
        {
            PlantModel plant;
            const DriveCommandSolution solution =
                plant.solveAccelerationFeedforward(1.25f, 3.75f, 0.80f, PlantParams::Default().supplyVoltageV);

            Assert::IsTrue(IsFiniteSolution(solution));
            Assert::IsTrue(solution.fanDutyCycle > 0.0f);
            Assert::IsTrue(solution.batteryVoltageV > 0.0f);
        }

        TEST_METHOD(PlantModelComputeBodyActionUsesLongitudinalLimitForPureSpeedChange)
        {
            PlantModel plant;
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
            PlantModel plant;
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
            PlantModel plant;
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
            PlantModel plant;
            float maxLongitudinalAccelMps2 = 0.0f;
            float maxYawAccelRadps2 = 0.0f;

            plant.velocityTargetTechnicalLimits(
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2,
                0.80f);

            Assert::IsTrue(std::isfinite(maxLongitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(maxYawAccelRadps2));
            Assert::IsTrue(maxLongitudinalAccelMps2 > 0.0f);
            Assert::IsTrue(maxYawAccelRadps2 > 0.0f);
        }
    };
}
