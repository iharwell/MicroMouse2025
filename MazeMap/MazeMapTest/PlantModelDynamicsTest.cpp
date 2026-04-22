#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "PlantModelTestSupport.h"
#include "..\MazeMap\EstimatorPredictModel.h"
#include "..\MazeMap\GripUtilizationMetrics.h"
#include "..\MazeMap\TorqueEstimateAdapter.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kZeroLinearVelocityToleranceMps = 0.008f;

        PlantDerivatives EvaluateEstimatorAppliedBankTorqueStep(
            const VehicleState::StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            const PlantModel::PreparedParams& prepared,
            float fanDutyCycle,
            const ModelCycleContext* cycleContext = nullptr) noexcept
        {
            EstimatorPredictModel predictModel;
            EstimatorPredictModel::PredictInput predictInput{};
            predictInput.currentState = state;
            predictInput.leftAppliedBankTorqueNm = leftAppliedBankTorqueNm;
            predictInput.rightAppliedBankTorqueNm = rightAppliedBankTorqueNm;
            predictInput.fanDutyCycle = fanDutyCycle;
            predictInput.cycleContext = cycleContext;
            return predictModel.EvaluateStep(predictInput, prepared);
        }

        PlantDerivatives EvaluateEstimatorAppliedBankTorqueStep(
            const VehicleState::StateVector& state,
            float leftAppliedBankTorqueNm,
            float rightAppliedBankTorqueNm,
            const PlantParams& params,
            float fanDutyCycle,
            const ModelCycleContext* cycleContext = nullptr) noexcept
        {
            return EvaluateEstimatorAppliedBankTorqueStep(
                state,
                leftAppliedBankTorqueNm,
                rightAppliedBankTorqueNm,
                PlantModel::Prepare(params),
                fanDutyCycle,
                cycleContext);
        }
    }

    TEST_CLASS(PlantModelDynamicsTest)
    {
    public:
        TEST_METHOD(PlantModelSymmetricDriveDoesNotCreateYawBias)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.5f;
            state(VehicleState::kOmegaL) = state(VehicleState::kU) / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = state(VehicleState::kU) / params.wheelRadiusM;

            ControlInput control;
            control.leftMotorCommand = 0.55f;
            control.rightMotorCommand = 0.55f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kR)));
            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kR), 1.0e-4f);
        }

        TEST_METHOD(PlantModelForwardStepFromAppliedBankTorquesMatchesZeroCommandAtRest)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = VehicleState::StateVector::Zero();

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives commanded = plant.forwardStep(state, control, params);
            const PlantDerivatives applied =
                EvaluateEstimatorAppliedBankTorqueStep(state, 0.0f, 0.0f, params, control.fanDutyCycle);

            AssertPlantDerivativesNear(commanded, applied, 1.0e-6f);
            Assert::IsTrue(std::isfinite(applied.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(applied.yawAccelRadps2));
        }

        TEST_METHOD(PlantModelForwardStepFromAppliedBankTorquesIsContinuousAcrossZeroSpeedCrossing)
        {
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector negativeState = VehicleState::StateVector::Zero();
            VehicleState::StateVector positiveState = VehicleState::StateVector::Zero();
            negativeState(VehicleState::kU) = -1.0e-4f;
            positiveState(VehicleState::kU) = 1.0e-4f;

            const PlantDerivatives negative =
                EvaluateEstimatorAppliedBankTorqueStep(negativeState, 0.0f, 0.0f, params, 0.80f);
            const PlantDerivatives positive =
                EvaluateEstimatorAppliedBankTorqueStep(positiveState, 0.0f, 0.0f, params, 0.80f);

            Assert::IsTrue(std::isfinite(negative.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(positive.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(negative.yawAccelRadps2));
            Assert::IsTrue(std::isfinite(positive.yawAccelRadps2));
            Assert::IsTrue(std::fabs(negative.longitudinalAccelMps2 - positive.longitudinalAccelMps2) <= 1.0e-3f);
            Assert::IsTrue(std::fabs(negative.yawAccelRadps2 - positive.yawAccelRadps2) <= 1.0e-3f);
        }

        TEST_METHOD(PlantModelForwardStepFromAppliedBankTorquesUsesFrozenCycleEdgeShapeContext)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 0.15f;
            state(VehicleState::kV) = 0.08f;
            state(VehicleState::kR) = 2.5f;
            state(VehicleState::kOmegaL) = 24.0f;
            state(VehicleState::kOmegaR) = 18.0f;

            const PlantDerivatives baseline =
                EvaluateEstimatorAppliedBankTorqueStep(state, 0.0f, 0.0f, prepared, 0.80f);

            ModelCycleContext cycleContext{};
            cycleContext.utilization.leftBankPreProjectionUtilization = 1.20f;
            cycleContext.utilization.rightBankPreProjectionUtilization = 1.15f;
            cycleContext.memory.leftBankMemory = 1.0f;
            cycleContext.memory.rightBankMemory = 1.0f;
            cycleContext.regrip.leftBankInRecovery = true;
            cycleContext.regrip.rightBankInRecovery = true;
            cycleContext.regrip.leftBankRecoveryScore = 1.0f;
            cycleContext.regrip.rightBankRecoveryScore = 1.0f;
            cycleContext.regrip.leftBankRecoveryTimeRemainingS = 0.05f;
            cycleContext.regrip.rightBankRecoveryTimeRemainingS = 0.05f;
            cycleContext.schedule.leftEdgeShapeStrength = 1.0f;
            cycleContext.schedule.rightEdgeShapeStrength = 1.0f;
            cycleContext.schedule.leftBankHoldoffActive = true;
            cycleContext.schedule.rightBankHoldoffActive = true;

            const PlantDerivatives edgeShaped =
                EvaluateEstimatorAppliedBankTorqueStep(state, 0.0f, 0.0f, prepared, 0.80f, &cycleContext);

            Assert::IsTrue(
                std::fabs(edgeShaped.contactForces.LeftBankForwardForceN()) <
                std::fabs(baseline.contactForces.LeftBankForwardForceN()));
            Assert::IsTrue(
                std::fabs(edgeShaped.contactForces.RightBankForwardForceN()) <
                std::fabs(baseline.contactForces.RightBankForwardForceN()));
        }

        TEST_METHOD(GripUtilizationMetricsRetainsPreProjectionUtilizationAboveUnity)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 0.05f;
            state(VehicleState::kOmegaL) = 45.0f;
            state(VehicleState::kOmegaR) = 43.0f;

            AppliedTorqueEstimate appliedTorque{};
            const GripUtilizationSnapshot snapshot =
                GripUtilizationMetrics::Compute(state, appliedTorque, prepared);

            Assert::IsTrue(snapshot.leftBankPreProjectionUtilization > 1.0f);
            Assert::IsTrue(snapshot.rightBankPreProjectionUtilization > 1.0f);
            Assert::IsTrue(snapshot.leftBankAnomalySeverity <= 1.0f);
            Assert::IsTrue(snapshot.rightBankAnomalySeverity <= 1.0f);
        }

        TEST_METHOD(PlantModelForwardStepFromAppliedBankTorquesUsesPhysicalTorqueThresholdsAtRest)
        {
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = VehicleState::StateVector::Zero();

            const PlantDerivatives derivatives =
                EvaluateEstimatorAppliedBankTorqueStep(state, 0.02f, 0.02f, params, 0.80f);

            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kOmegaR)));
            Assert::IsTrue(derivatives.regime != MotionRegime::StoppedHold);
            Assert::IsTrue(derivatives.stateDot(VehicleState::kOmegaL) > 0.0f);
            Assert::IsTrue(derivatives.stateDot(VehicleState::kOmegaR) > 0.0f);
        }

        TEST_METHOD(GripUtilizationMetricsPreservePreProjectionUtilizationAboveUnity)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 0.05f;
            state(VehicleState::kOmegaL) = 55.0f;
            state(VehicleState::kOmegaR) = 55.0f;

            AppliedTorqueEstimate appliedTorque{};
            appliedTorque.batteryVoltageAvailable = true;
            const GripUtilizationSnapshot snapshot =
                GripUtilizationMetrics::Compute(state, appliedTorque, prepared);

            Assert::IsTrue(snapshot.leftBankPreProjectionUtilization > 1.0f);
            Assert::IsTrue(snapshot.rightBankPreProjectionUtilization > 1.0f);
            Assert::IsTrue(snapshot.leftBankAnomalySeverity > 0.0f);
            Assert::IsTrue(snapshot.rightBankAnomalySeverity > 0.0f);

            const ContactForces forces = plant.tireForces(state, prepared);
            for (const ContactForce& force : forces.contacts)
            {
                Assert::IsTrue(force.saturation >= 0.0f);
                Assert::IsTrue(force.saturation <= 1.0f);
            }
        }

        TEST_METHOD(PlantModelPreparedOverloadsMatchRawOverloads)
        {
            PlantModel plant;
            PlantParams params = PlantParams::Default();
            params.corneringStiffnessFrontNPerRad = 19.5f;
            params.corneringStiffnessRearNPerRad = 17.25f;
            params.staticFrictionTorqueNm = 0.012f;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            const VehicleState::StateVector state = BuildUkfState(
                0.18f,
                0.27f,
                0.14f,
                1.85f,
                -0.21f,
                2.7f,
                60.0f,
                47.0f,
                -0.03f);

            ControlInput control{};
            control.leftMotorCommand = 0.38f;
            control.rightMotorCommand = 0.23f;
            control.fanDutyCycle = 0.72f;
            control.batteryVoltageV = params.supplyVoltageV - 0.35f;

            const WheelKinematics rawKinematics = plant.wheelKinematics(state, params);
            const WheelKinematics preparedKinematics = plant.wheelKinematics(state, prepared);
            AssertWheelKinematicsNear(rawKinematics, preparedKinematics, 1.0e-6f);

            const SlipTargets rawSlip = plant.slipTargets(state, rawKinematics, params);
            const SlipTargets preparedSlip = plant.slipTargets(state, preparedKinematics, prepared);
            AssertSlipTargetsNear(rawSlip, preparedSlip, 1.0e-6f);
            AssertSlipTargetsNear(plant.slipTargets(state, params), plant.slipTargets(state, prepared), 1.0e-6f);

            AssertContactForcesNear(
                plant.tireForces(state, params),
                plant.tireForces(state, prepared),
                1.0e-6f);
            AssertContactForcesNear(
                plant.tireForces(state, control, params),
                plant.tireForces(state, control, prepared),
                1.0e-6f);

            AssertPlantDerivativesNear(
                plant.forwardStep(state, control, params),
                plant.forwardStep(state, control, prepared),
                1.0e-6f);

            Assert::AreEqual(
                plant.imuPlanarAcceleration(state, control, params).x(),
                plant.imuPlanarAcceleration(state, control, prepared).x(),
                1.0e-6f);
            Assert::AreEqual(
                plant.imuPlanarAcceleration(state, control, params).y(),
                plant.imuPlanarAcceleration(state, control, prepared).y(),
                1.0e-6f);

            AssertStateVectorNear(
                plant.integrate(state, control, 0.0015f, params),
                plant.integrate(state, control, 0.0015f, prepared),
                1.0e-6f);

            AssertDriveCommandSolutionNear(
                plant.solveDriveCommands(state, 1.25f, 3.75f, params, control.fanDutyCycle, control.batteryVoltageV),
                plant.solveDriveCommands(state, 1.25f, 3.75f, prepared, control.fanDutyCycle, control.batteryVoltageV),
                1.0e-6f);
            AssertDriveCommandSolutionNear(
                plant.solveDriveCommands(
                    state(VehicleState::kU),
                    1.25f,
                    state(VehicleState::kR),
                    3.75f,
                    params,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                plant.solveDriveCommands(
                    state(VehicleState::kU),
                    1.25f,
                    state(VehicleState::kR),
                    3.75f,
                    prepared,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                1.0e-6f);

            AssertDriveCommandSolutionNear(
                plant.solveDriveCommandsForVelocityTarget(state, 2.05f, 2.95f, params, control.fanDutyCycle, control.batteryVoltageV),
                plant.solveDriveCommandsForVelocityTarget(state, 2.05f, 2.95f, prepared, control.fanDutyCycle, control.batteryVoltageV),
                1.0e-6f);
            AssertDriveCommandSolutionNear(
                plant.solveDriveCommandsForVelocityTarget(
                    state(VehicleState::kU),
                    2.05f,
                    state(VehicleState::kR),
                    2.95f,
                    params,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                plant.solveDriveCommandsForVelocityTarget(
                    state(VehicleState::kU),
                    2.05f,
                    state(VehicleState::kR),
                    2.95f,
                    prepared,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                1.0e-6f);

            float rawMaxLongitudinalAccelMps2 = 0.0f;
            float rawMaxYawAccelRadps2 = 0.0f;
            float preparedMaxLongitudinalAccelMps2 = 0.0f;
            float preparedMaxYawAccelRadps2 = 0.0f;
            plant.velocityTargetTechnicalLimits(
                state,
                params,
                rawMaxLongitudinalAccelMps2,
                rawMaxYawAccelRadps2,
                control.fanDutyCycle);
            plant.velocityTargetTechnicalLimits(
                state,
                prepared,
                preparedMaxLongitudinalAccelMps2,
                preparedMaxYawAccelRadps2,
                control.fanDutyCycle);
            Assert::AreEqual(rawMaxLongitudinalAccelMps2, preparedMaxLongitudinalAccelMps2, 1.0e-6f);
            Assert::AreEqual(rawMaxYawAccelRadps2, preparedMaxYawAccelRadps2, 1.0e-6f);

            rawMaxLongitudinalAccelMps2 = 0.0f;
            rawMaxYawAccelRadps2 = 0.0f;
            preparedMaxLongitudinalAccelMps2 = 0.0f;
            preparedMaxYawAccelRadps2 = 0.0f;
            plant.velocityTargetTechnicalLimits(
                state(VehicleState::kU),
                state(VehicleState::kR),
                params,
                rawMaxLongitudinalAccelMps2,
                rawMaxYawAccelRadps2,
                control.fanDutyCycle);
            plant.velocityTargetTechnicalLimits(
                state(VehicleState::kU),
                state(VehicleState::kR),
                prepared,
                preparedMaxLongitudinalAccelMps2,
                preparedMaxYawAccelRadps2,
                control.fanDutyCycle);
            Assert::AreEqual(rawMaxLongitudinalAccelMps2, preparedMaxLongitudinalAccelMps2, 1.0e-6f);
            Assert::AreEqual(rawMaxYawAccelRadps2, preparedMaxYawAccelRadps2, 1.0e-6f);

            AssertDriveCommandSolutionNear(
                plant.solveTractionLimitedDriveCommands(state, 1.25f, 3.75f, params, control.fanDutyCycle, control.batteryVoltageV),
                plant.solveTractionLimitedDriveCommands(state, 1.25f, 3.75f, prepared, control.fanDutyCycle, control.batteryVoltageV),
                1.0e-6f);
            AssertDriveCommandSolutionNear(
                plant.solveTractionLimitedDriveCommands(
                    state(VehicleState::kU),
                    1.25f,
                    state(VehicleState::kR),
                    3.75f,
                    params,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                plant.solveTractionLimitedDriveCommands(
                    state(VehicleState::kU),
                    1.25f,
                    state(VehicleState::kR),
                    3.75f,
                    prepared,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                1.0e-6f);

            AssertDriveCommandSolutionNear(
                plant.solveTractionLimitedDriveCommandsForVelocityTarget(
                    state,
                    2.05f,
                    2.95f,
                    params,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                plant.solveTractionLimitedDriveCommandsForVelocityTarget(
                    state,
                    2.05f,
                    2.95f,
                    prepared,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                1.0e-6f);
            AssertDriveCommandSolutionNear(
                plant.solveTractionLimitedDriveCommandsForVelocityTarget(
                    state(VehicleState::kU),
                    2.05f,
                    state(VehicleState::kR),
                    2.95f,
                    params,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                plant.solveTractionLimitedDriveCommandsForVelocityTarget(
                    state(VehicleState::kU),
                    2.05f,
                    state(VehicleState::kR),
                    2.95f,
                    prepared,
                    control.fanDutyCycle,
                    control.batteryVoltageV),
                1.0e-6f);

            Assert::AreEqual(
                plant.driveTorqueFromCommand(control.leftMotorCommand, state(VehicleState::kOmegaL), control.batteryVoltageV, params),
                plant.driveTorqueFromCommand(control.leftMotorCommand, state(VehicleState::kOmegaL), control.batteryVoltageV, prepared),
                1.0e-6f);
            Assert::AreEqual(
                plant.driveCommandFromTorque(0.021f, state(VehicleState::kOmegaR), control.batteryVoltageV, params),
                plant.driveCommandFromTorque(0.021f, state(VehicleState::kOmegaR), control.batteryVoltageV, prepared),
                1.0e-6f);
            Assert::AreEqual(
                plant.driveFrictionTorque(state(VehicleState::kOmegaL), 0.018f, params),
                plant.driveFrictionTorque(state(VehicleState::kOmegaL), 0.018f, prepared),
                1.0e-6f);
        }

        TEST_METHOD(PlantModelExactRestHoldKeepsMotionStateAtZero)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float zeroWheelSpeedToleranceRadps = kZeroLinearVelocityToleranceMps / params.wheelRadiusM;
            VehicleState::StateVector state = BuildUkfState(
                0.03f,
                0.09f,
                0.21f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.12f);

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;
            for (int step = 0; step < 1000; ++step)
            {
                state = AdvancePlantPredictionState(plant, prepared, state, control, dt);
            }

            Assert::AreEqual(0.03f, state(VehicleState::kPx), 1.0e-7f);
            Assert::AreEqual(0.09f, state(VehicleState::kPy), 1.0e-7f);
            Assert::AreEqual(0.21f, state(VehicleState::kPsi), 1.0e-7f);
            Assert::AreEqual(0.12f, state(VehicleState::kBgz), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kU), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state(VehicleState::kV), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelSmallStationaryPerturbationsSnapBackToRest)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float zeroWheelSpeedToleranceRadps = kZeroLinearVelocityToleranceMps / params.wheelRadiusM;
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.05f,
                0.8f,
                -0.7f);

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;
            for (int step = 0; step < 250; ++step)
            {
                state = AdvancePlantPredictionState(plant, prepared, state, control, dt);
            }

            Assert::AreEqual(0.0f, state(VehicleState::kU), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state(VehicleState::kV), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.001f,
                0.05f,
                0.8f,
                -0.7f);
            VehicleState::StateVector state = initialState;

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;
            for (int step = 0; step < 25; ++step)
            {
                state = AdvancePlantPredictionState(plant, prepared, state, control, dt);
            }

            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < params.stopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < params.stopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < params.stopEnterYawRateRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < params.stopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < params.stopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < std::fabs(initialState(VehicleState::kR)));
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < std::fabs(initialState(VehicleState::kOmegaL)));
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < std::fabs(initialState(VehicleState::kOmegaR)));
        }

        TEST_METHOD(PlantModelComputesFiniteAlgebraicSlipAndForces)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kV) = 0.15f;
            state(VehicleState::kR) = 1.8f;
            state(VehicleState::kOmegaL) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.65f;
            control.rightMotorCommand = 0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const WheelKinematics kinematics = plant.wheelKinematics(state, params);
            const SlipTargets slip = plant.slipTargets(state, kinematics, params);
            const ContactForces forces = plant.tireForces(state, control, params);

            Assert::IsTrue(std::isfinite(slip.kappaLeft));
            Assert::IsTrue(std::isfinite(slip.kappaRight));
            for (float ratio : slip.lateralRatio)
            {
                Assert::IsTrue(std::isfinite(ratio));
            }

            for (const ContactForce& force : forces.contacts)
            {
                Assert::IsTrue(std::isfinite(force.rightForceN));
                Assert::IsTrue(std::isfinite(force.forwardForceN));
                Assert::IsTrue(force.saturation >= 0.0f);
                Assert::IsTrue(force.saturation <= 1.0f);
                Assert::IsTrue(std::isfinite(force.preProjectionUtilization));
                Assert::IsTrue(force.preProjectionUtilization >= 0.0f);
                Assert::IsTrue(force.preProjectionUtilization >= force.saturation);
            }
        }

        TEST_METHOD(GripUtilizationMetricsExposeRawPreProjectionUtilizationFromAppliedTorques)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const PlantModel plant;

            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.25f,
                0.08f,
                2.8f,
                14.0f,
                10.0f,
                0.0f);

            ControlInput control{};
            control.leftMotorCommand = 1.0f;
            control.rightMotorCommand = 0.92f;
            control.fanDutyCycle = 0.30f;
            control.batteryVoltageV = params.supplyVoltageV;

            const AppliedTorqueEstimate appliedTorque =
                TorqueEstimateAdapter::Estimate(
                    plant,
                    state,
                    control,
                    prepared,
                    control.batteryVoltageV);
            const PlantDerivatives predicted =
                EvaluateEstimatorAppliedBankTorqueStep(
                    state,
                    appliedTorque.leftAppliedBankTorqueNm,
                    appliedTorque.rightAppliedBankTorqueNm,
                    prepared,
                    control.fanDutyCycle);

            GripUtilizationInputs inputs{};
            inputs.fanDutyCycle = control.fanDutyCycle;
            const GripUtilizationSnapshot snapshot =
                GripUtilizationMetrics::Compute(
                    state,
                    appliedTorque,
                    prepared,
                    inputs);

            Assert::AreEqual(
                predicted.contactForces.LeftBankMaxPreProjectionUtilization(),
                snapshot.leftBankPreProjectionUtilization,
                1.0e-6f);
            Assert::AreEqual(
                predicted.contactForces.RightBankMaxPreProjectionUtilization(),
                snapshot.rightBankPreProjectionUtilization,
                1.0e-6f);
            Assert::IsTrue(snapshot.leftBankPreProjectionUtilization >= 0.0f);
            Assert::IsTrue(snapshot.rightBankPreProjectionUtilization >= 0.0f);
            Assert::IsTrue(
                snapshot.leftBankPreProjectionUtilization > 1.0f ||
                snapshot.rightBankPreProjectionUtilization > 1.0f);
        }

        TEST_METHOD(PlantModelImuAccelerationIncludesLeverArmTerms)
        {
            PlantModel plant;
            PlantParams zeroLeverParams = PlantParams::Default();
            zeroLeverParams.imu.positionBodyM = Eigen::Vector2f::Zero();
            PlantParams leverParams = zeroLeverParams;
            leverParams.imu.positionBodyM = Eigen::Vector2f(0.020f, -0.010f);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.4f;
            state(VehicleState::kV) = 0.2f;
            state(VehicleState::kR) = 5.0f;
            state(VehicleState::kOmegaL) = 0.9f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.1f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.30f;
            control.rightMotorCommand = 0.55f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = zeroLeverParams.supplyVoltageV;

            const PlantDerivatives zeroLever = plant.forwardStep(state, control, zeroLeverParams);
            const PlantDerivatives withLever = plant.forwardStep(state, control, leverParams);

            const float expectedDeltaX =
                (withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.y()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.x());
            const float expectedDeltaY =
                (-withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.x()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.y());

            Assert::AreEqual(
                zeroLever.imuAccelBodyMps2.x() + expectedDeltaX,
                withLever.imuAccelBodyMps2.x(),
                1.0e-5f);
            Assert::AreEqual(
                zeroLever.imuAccelBodyMps2.y() + expectedDeltaY,
                withLever.imuAccelBodyMps2.y(),
                1.0e-5f);
        }

        TEST_METHOD(PlantModelPredictsImuAccelerationInProjectBodyAxes)
        {
            PlantModel plant;
            PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.1f;
            state(VehicleState::kV) = -0.3f;
            state(VehicleState::kR) = 4.0f;
            state(VehicleState::kOmegaL) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.25f;
            control.rightMotorCommand = 0.45f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            const Eigen::Vector2f predictedMeasurement = plant.imuPlanarAcceleration(state, control, params);

            Assert::AreEqual(derivatives.imuAccelBodyMps2.x(), predictedMeasurement.x(), 1.0e-5f);
            Assert::AreEqual(derivatives.imuAccelBodyMps2.y(), predictedMeasurement.y(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelSymmetricPositiveDriveFromRestMovesForward)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);

            ControlInput control{};
            control.leftMotorCommand = 0.50f;
            control.rightMotorCommand = 0.50f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 25;
            for (int step = 0; step < kSteps; ++step)
            {
                state = AdvancePlantPredictionState(plant, prepared, state, control, dt);
            }

            const float totalTimeS = dt * static_cast<float>(kSteps);
            const float averageAccelMps2 = state(VehicleState::kU) / totalTimeS;

            Assert::IsTrue(std::isfinite(state.sum()));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPy) > 0.09f);
            Assert::IsTrue(averageAccelMps2 > 0.0f);
            Assert::IsTrue(averageAccelMps2 < 60.0f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 0.002f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 0.02f);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 0.10f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPsi)) < 0.01f);
        }

        TEST_METHOD(PlantModelStaticFrictionHoldsSubthresholdDriveAtRest)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float staticWindowRadps = params.staticFrictionMaxSpeedMps / params.wheelRadiusM;

            ControlInput control{};
            control.leftMotorCommand = 0.25f;
            control.rightMotorCommand = 0.25f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const VehicleState::StateVector atRest = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            const PlantDerivatives derivativesAtRest = plant.forwardStep(atRest, control, params);
            Assert::AreEqual(0.0f, derivativesAtRest.stateDot(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(0.0f, derivativesAtRest.stateDot(VehicleState::kOmegaR), 1.0e-6f);

            Assert::AreEqual(
                params.staticFrictionTorqueNm,
                plant.driveFrictionTorque(0.5f * staticWindowRadps, 1.0f, params),
                1.0e-6f);
            Assert::AreEqual(
                -params.staticFrictionTorqueNm,
                plant.driveFrictionTorque(-0.5f * staticWindowRadps, -1.0f, params),
                1.0e-6f);
            Assert::AreEqual(
                params.rollingFrictionTorqueNm,
                plant.driveFrictionTorque(1.1f * staticWindowRadps, 1.0f, params),
                1.0e-6f);
            Assert::AreEqual(
                -params.rollingFrictionTorqueNm,
                plant.driveFrictionTorque(-1.1f * staticWindowRadps, -1.0f, params),
                1.0e-6f);
        }

        TEST_METHOD(PlantModelIntegrateSingleLargeStepRemainsFiniteAndSymmetric)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            ControlInput control{};
            control.leftMotorCommand = 0.45f;
            control.rightMotorCommand = 0.45f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.004f;
            const VehicleState::StateVector integrated = plant.integrate(state, control, dt, params);

            Assert::IsTrue(std::isfinite(integrated.sum()));
            Assert::IsTrue(std::isfinite(integrated(VehicleState::kU)));
            Assert::IsTrue(std::isfinite(integrated(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(integrated(VehicleState::kOmegaR)));
            Assert::IsTrue(std::fabs(integrated(VehicleState::kOmegaL) - integrated(VehicleState::kOmegaR)) < 1.0f);
            Assert::IsTrue(std::fabs(integrated(VehicleState::kPx)) < 0.005f);
            Assert::IsTrue(std::fabs(integrated(VehicleState::kR)) < 0.10f);
        }

        TEST_METHOD(PlantModelIntegratePreservesHeadingNormalization)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                PI_F - 0.01f,
                0.5f,
                0.0f,
                6.0f,
                0.5f / params.wheelRadiusM,
                0.5f / params.wheelRadiusM);

            ControlInput control{};
            control.batteryVoltageV = params.supplyVoltageV;
            const VehicleState::StateVector integrated = plant.integrate(state, control, 0.01f, params);

            Assert::IsTrue(integrated(VehicleState::kPsi) <= PI_F);
            Assert::IsTrue(integrated(VehicleState::kPsi) >= -PI_F);
        }

    };
}
