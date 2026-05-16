#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "PlantModelTestSupport.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kZeroLinearVelocityToleranceMps = 0.008f;
    }

    TEST_CLASS(PlantModelDynamicsTest)
    {
    public:
        TEST_METHOD(PlantModelDefaultParamsUseDriveWheelConstructionValues)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            Assert::AreEqual(2.4e-7f, params.equivalentWheelInertiaKgM2, 1.0e-10f);
            Assert::AreEqual(4.12f, params.longitudinalTireStiffnessN, 0.01f);
            Assert::AreEqual(0.223f, params.corneringStiffnessFrontNPerRad, 0.001f);
            Assert::AreEqual(0.223f, params.corneringStiffnessRearNPerRad, 0.001f);
            Assert::AreEqual(params.equivalentWheelInertiaKgM2, prepared.wheelInertiaKgM2, 1.0e-10f);
            Assert::AreEqual(params.longitudinalTireStiffnessN, prepared.longitudinalTireStiffnessN, 0.01f);
            Assert::AreEqual(2.0f * params.corneringStiffnessFrontNPerRad, prepared.frontCorneringStiffnessAxleNPerRad, 0.01f);
            Assert::AreEqual(2.0f * params.corneringStiffnessRearNPerRad, prepared.rearCorneringStiffnessAxleNPerRad, 0.01f);
        }

        TEST_METHOD(PlantModelSymmetricDriveDoesNotCreateYawBias)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.5f;
            state(VehicleState::kOmegaL) = state(VehicleState::kU) / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = state(VehicleState::kU) / params.wheelRadiusM;

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.55f);
            control.SetRightCommand(0.55f);

            const PlantDerivatives derivatives =
                plant.forwardStep(state, control, params);
            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kR)));
            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kR), 1.0e-4f);
        }

        TEST_METHOD(PlantModelTireForcesRetainPreProjectionUtilizationAboveUnity)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 0.05f;
            state(VehicleState::kOmegaL) = 45.0f;
            state(VehicleState::kOmegaR) = 43.0f;

            const ContactForces forces = plant.tireForces(state, prepared);

            Assert::IsTrue(forces.LeftBankMaxPreProjectionUtilization() > 1.0f);
            Assert::IsTrue(forces.RightBankMaxPreProjectionUtilization() > 1.0f);
        }

        TEST_METHOD(PlantModelTireForcesPreservePreProjectionUtilizationAboveUnity)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 0.05f;
            state(VehicleState::kOmegaL) = 55.0f;
            state(VehicleState::kOmegaR) = 55.0f;

            const ContactForces forces = plant.tireForces(state, prepared);

            Assert::IsTrue(forces.LeftBankMaxPreProjectionUtilization() > 1.0f);
            Assert::IsTrue(forces.RightBankMaxPreProjectionUtilization() > 1.0f);
            for (const ContactForce& force : forces.contacts)
            {
                Assert::IsTrue(force.saturation >= 0.0f);
                Assert::IsTrue(force.saturation <= 1.0f);
            }
        }

        TEST_METHOD(PlantModelPreparedOverloadsMatchRawOverloads)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.38f);
            control.SetRightCommand(0.23f);
            const float controlFanDutyCycle = 0.72f;
            runtime.vehicle.SetFanDuty(controlFanDutyCycle);

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

            float rawMaxLongitudinalAccelMps2 = 0.0f;
            float rawMaxYawAccelRadps2 = 0.0f;
            float preparedMaxLongitudinalAccelMps2 = 0.0f;
            float preparedMaxYawAccelRadps2 = 0.0f;
            plant.velocityTargetTechnicalLimits(
                state,
                params,
                rawMaxLongitudinalAccelMps2,
                rawMaxYawAccelRadps2);
            plant.velocityTargetTechnicalLimits(
                state,
                prepared,
                preparedMaxLongitudinalAccelMps2,
                preparedMaxYawAccelRadps2);
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
                rawMaxYawAccelRadps2);
            plant.velocityTargetTechnicalLimits(
                state(VehicleState::kU),
                state(VehicleState::kR),
                prepared,
                preparedMaxLongitudinalAccelMps2,
                preparedMaxYawAccelRadps2);
            Assert::AreEqual(rawMaxLongitudinalAccelMps2, preparedMaxLongitudinalAccelMps2, 1.0e-6f);
            Assert::AreEqual(rawMaxYawAccelRadps2, preparedMaxYawAccelRadps2, 1.0e-6f);

            Assert::AreEqual(
                plant.driveFrictionTorque(state(VehicleState::kOmegaL), 0.018f, params),
                plant.driveFrictionTorque(state(VehicleState::kOmegaL), 0.018f, prepared),
                1.0e-6f);
        }

        TEST_METHOD(PlantModelFeedforwardDoesNotUsePrediction)
        {
            constexpr uint32_t numFeedforward = 100000;
            constexpr uint32_t numForwardStep = 80000;
			constexpr uint32_t numIntegrate = 75000;

            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            constexpr float dtSeconds = 0.001f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            auto state = BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f,
                    0.0f,
                    0.0f,
                    1.0f / params.wheelRadiusM,
                    1.0f / params.wheelRadiusM,
                    0.0f);
            float forwardStepAccumulator = 0.0f;
            float feedforwardAccumulator = 0.0f;

			auto startTime1 = std::chrono::high_resolution_clock::now();
            for (int tick = 0; tick < numForwardStep; ++tick)
            {
                const PlantDerivatives derivatives =
                    plant.forwardStep(
                        state,
                        control,
                        params);
                forwardStepAccumulator += derivatives.longitudinalAccelMps2 + derivatives.yawAccelRadps2;
            }
			auto durationForwardStep = std::chrono::high_resolution_clock::now() - startTime1;

            startTime1 = std::chrono::high_resolution_clock::now();
            for (int tick = 0; tick < numIntegrate; ++tick)
            {
                state =
                    plant.integrate(
                        state,
                        control,
                        dtSeconds,
                        params);
            }
            auto durationIntegrate = std::chrono::high_resolution_clock::now() - startTime1;
            startTime1 = std::chrono::high_resolution_clock::now();
            for (int tick = 0; tick < numFeedforward; ++tick)
            {
				const App::Internal::CommandVector command = plant.ComputeFeedforward(0.0f, 0.0f);
                feedforwardAccumulator += command.LeftCommand() + command.RightCommand();
            }
            auto durationFeedforward = std::chrono::high_resolution_clock::now() - startTime1;
            auto ss = std::wstringstream();
            ss << "feedforward: " << durationFeedforward.count() << "  integrate: " << durationIntegrate.count() << "  forwardstep: " << durationForwardStep.count();
            Assert::IsTrue(std::isfinite(forwardStepAccumulator));
            Assert::IsTrue(std::isfinite(feedforwardAccumulator));
            Assert::IsTrue(durationFeedforward < durationIntegrate, ss.str().c_str());
            Assert::IsTrue(durationIntegrate < durationForwardStep, ss.str().c_str());
        }

        TEST_METHOD(PlantModelLateralTireForcePlateausAtSustainedLimitAcrossTicks)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            constexpr float dtSeconds = 0.001f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            auto buildState = [&params](const float lateralVelocityMps) {
                return BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    1.0f,
                    lateralVelocityMps,
                    0.0f,
                    1.0f / params.wheelRadiusM,
                    1.0f / params.wheelRadiusM,
                    0.0f);
            };

            auto maxLateralAccelAcrossTicks =
                [&](VehicleState::StateVector state) {
                    float maxObservedAbsAccelMps2 = 0.0f;
                    for (int tick = 0; tick < 5; ++tick)
                    {
                        const PlantDerivatives derivatives =
                            plant.forwardStep(
                                state,
                                control,
                                params);
                        maxObservedAbsAccelMps2 =
                            (std::max)(maxObservedAbsAccelMps2, std::fabs(derivatives.lateralAccelMps2));
                        state =
                            plant.integrate(
                                state,
                                control,
                                dtSeconds,
                                params);
                    }
                    return maxObservedAbsAccelMps2;
                };

            const float highSlipAccelMps2 = maxLateralAccelAcrossTicks(buildState(1.25f));
            const float extremeSlipAccelMps2 = maxLateralAccelAcrossTicks(buildState(3.50f));

            Assert::IsTrue(highSlipAccelMps2 <= params.combinedAccelSustainedMps2 + 0.20f);
            Assert::IsTrue(extremeSlipAccelMps2 <= params.combinedAccelSustainedMps2 + 0.20f);
            Assert::IsTrue(extremeSlipAccelMps2 >= highSlipAccelMps2 - 0.40f);
        }

        TEST_METHOD(PlantModelInPlacePivotScrubStopsYawFromYawInertiaAcrossTicks)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            PlantParams params = PlantParams::Default();
            params.longitudinalTireStiffnessN = 0.0f;
            params.corneringStiffnessFrontNPerRad = 0.0f;
            params.corneringStiffnessRearNPerRad = 0.0f;
            params.yawRateDampingNmsPerRad = 0.0f;
            params.pivotScrubRollingYawMomentNm = 0.11f;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.0f);
            control.SetRightCommand(0.0f);

            constexpr float dtSeconds = 0.001f;
            constexpr float initialYawRateRadps = 3.0f;
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kR) = initialYawRateRadps;

            const float expectedYawDecelRadps2 =
                params.pivotScrubRollingYawMomentNm / params.yawInertiaKgM2;
            const int expectedStopTicks =
                static_cast<int>(std::ceil(initialYawRateRadps / (expectedYawDecelRadps2 * dtSeconds)));
            int observedStopTicks = 0;
            for (; observedStopTicks < 10; ++observedStopTicks)
            {
                const PlantDerivatives derivatives =
                    plant.forwardStep(
                        state,
                        control,
                        params);
                if (state(VehicleState::kR) <= params.stopEnterYawRateRadps)
                {
                    break;
                }

                Assert::AreEqual(
                    -expectedYawDecelRadps2,
                    derivatives.yawAccelRadps2,
                    1.0e-3f);
                state =
                    plant.integrate(
                        state,
                        control,
                        dtSeconds,
                        params);
            }

            const int stopTickError =
                (observedStopTicks > expectedStopTicks) ?
                (observedStopTicks - expectedStopTicks) :
                (expectedStopTicks - observedStopTicks);
            Assert::IsTrue(stopTickError <= 1);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) <= params.stopEnterYawRateRadps);
        }

        TEST_METHOD(PlantModelExactRestHoldKeepsMotionStateAtZero)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 1000; ++step)
            {
                state = AdvancePlantPredictionState(
                    plant,
                    prepared,
                    state,
                    control, dt);
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
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 250; ++step)
            {
                state = AdvancePlantPredictionState(
                    plant,
                    prepared,
                    state,
                    control, dt);
            }

            Assert::AreEqual(0.0f, state(VehicleState::kU), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state(VehicleState::kV), kZeroLinearVelocityToleranceMps);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-7f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), zeroWheelSpeedToleranceRadps);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), zeroWheelSpeedToleranceRadps);
        }

        TEST_METHOD(PlantModelPreparedNearZeroLateralPerturbationsSnapBackToRest)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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

            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            for (int step = 0; step < 25; ++step)
            {
                state = AdvancePlantPredictionState(
                    plant,
                    prepared,
                    state,
                    control, dt);
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
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kV) = 0.15f;
            state(VehicleState::kR) = 1.8f;
            state(VehicleState::kOmegaL) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.65f);
            control.SetRightCommand(0.60f);

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

        TEST_METHOD(ContactForcesExposeRawPreProjectionUtilization)
        {
            PlantModelTestRuntime runtime;
            const PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);

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

            const ContactForces forces = plant.tireForces(state, prepared);

            Assert::AreEqual(
                (std::max)(
                    forces.contacts[0].preProjectionUtilization,
                    forces.contacts[2].preProjectionUtilization),
                forces.LeftBankMaxPreProjectionUtilization(),
                1.0e-6f);
            Assert::AreEqual(
                (std::max)(
                    forces.contacts[1].preProjectionUtilization,
                    forces.contacts[3].preProjectionUtilization),
                forces.RightBankMaxPreProjectionUtilization(),
                1.0e-6f);
            Assert::IsTrue(forces.LeftBankMaxPreProjectionUtilization() >= 0.0f);
            Assert::IsTrue(forces.RightBankMaxPreProjectionUtilization() >= 0.0f);
            Assert::IsTrue(
                forces.LeftBankMaxPreProjectionUtilization() > 1.0f ||
                forces.RightBankMaxPreProjectionUtilization() > 1.0f);
        }

        TEST_METHOD(PlantModelImuAccelerationIncludesLeverArmTerms)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            PlantParams zeroLeverParams = PlantParams::Default();
            zeroLeverParams.backLeftImuMount = SensorMount(Eigen::Vector2f::Zero(), Eigen::Matrix2f::Identity());
            PlantParams leverParams = zeroLeverParams;
            leverParams.backLeftImuMount = SensorMount(Eigen::Vector2f(0.020f, -0.010f), Eigen::Matrix2f::Identity());

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.4f;
            state(VehicleState::kV) = 0.2f;
            state(VehicleState::kR) = 5.0f;
            state(VehicleState::kOmegaL) = 0.9f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.1f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.30f);
            control.SetRightCommand(0.55f);

            const PlantDerivatives zeroLever =
                plant.forwardStep(state, control, zeroLeverParams);
            const PlantDerivatives withLever =
                plant.forwardStep(state, control, leverParams);

            const float expectedDeltaX =
                (withLever.stateDot(VehicleState::kR) * leverParams.backLeftImuMount.positionBodyM().y()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.backLeftImuMount.positionBodyM().x());
            const float expectedDeltaY =
                (-withLever.stateDot(VehicleState::kR) * leverParams.backLeftImuMount.positionBodyM().x()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.backLeftImuMount.positionBodyM().y());

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
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.1f;
            state(VehicleState::kV) = -0.3f;
            state(VehicleState::kR) = 4.0f;
            state(VehicleState::kOmegaL) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);

            App::Internal::CommandVector control;
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.45f);

            const PlantDerivatives derivatives =
                plant.forwardStep(state, control, params);
            const Eigen::Vector2f predictedMeasurement =
                plant.imuPlanarAcceleration(
                    state,
                    control,
                    params);

            Assert::AreEqual(derivatives.imuAccelBodyMps2.x(), predictedMeasurement.x(), 1.0e-5f);
            Assert::AreEqual(derivatives.imuAccelBodyMps2.y(), predictedMeasurement.y(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelSymmetricPositiveDriveFromRestMovesForward)
        {
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.50f);
            control.SetRightCommand(0.50f);

            constexpr float dt = 0.002f;
            constexpr int kSteps = 25;
            for (int step = 0; step < kSteps; ++step)
            {
                state = AdvancePlantPredictionState(
                    plant,
                    prepared,
                    state,
                    control, dt);
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
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
            const PlantParams params = PlantParams::Default();
            const float staticWindowRadps = params.staticFrictionMaxSpeedMps / params.wheelRadiusM;

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.25f);

            const VehicleState::StateVector atRest = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            const PlantDerivatives derivativesAtRest =
                plant.forwardStep(atRest, control, params);
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
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.45f);
            control.SetRightCommand(0.45f);

            constexpr float dt = 0.004f;
            const VehicleState::StateVector integrated =
                plant.integrate(state, control, dt, params);

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
            PlantModelTestRuntime runtime;
            PlantModel& plant = runtime.plant;
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

            App::Internal::CommandVector control{};
            const VehicleState::StateVector integrated =
                plant.integrate(state, control, 0.01f, params);

            Assert::IsTrue(integrated(VehicleState::kPsi) <= PI_F);
            Assert::IsTrue(integrated(VehicleState::kPsi) >= -PI_F);
        }

    };
}






