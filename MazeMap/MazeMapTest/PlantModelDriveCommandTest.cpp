#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorTestSupport.h"
#include "PlantModelTestSupport.h"

#include <algorithm>
#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kPredictDtSeconds = 0.001f;
        constexpr int kFeedforwardPredictSteps = 1000;
        constexpr float kFeedforwardRelativeTolerance = 0.10f;

        struct FeedforwardPredictHorizon
        {
            int steps;
            const wchar_t* label;
        };

        constexpr FeedforwardPredictHorizon kExtendedFeedforwardPredictHorizons[] =
        {
            { 500, L"500 ms" },
            { 550, L"550 ms" },
            { 750, L"750 ms" },
            { 990, L"990 ms" },
            { 1950, L"1950 ms" },
        };

        VehicleState::StateVector BuildRollingUkfState(
            float forwardVelocityMps,
            float yawRateRadps,
            const PlantParams& params) noexcept
        {
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            return BuildUkfState(
                0.0f,
                0.0f,
                0.0f,
                forwardVelocityMps,
                0.0f,
                yawRateRadps,
                (forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM,
                (forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM,
                0.0f);
        }

        VehicleState::StateVector RunPlantPredictLoop(
            const PlantModel& plant,
            const PlantModel::PreparedParams& prepared,
            const VehicleState::StateVector& initialState,
            const ControlInput& control,
            int steps)
        {
            VehicleState::StateVector state = initialState;
            for (int step = 0; step < steps; ++step)
            {
                state = plant.integrate(state, control, kPredictDtSeconds, prepared);
            }

            return state;
        }

        template <typename Solver>
        VehicleState::StateVector RunPlantPredictLoopWithResolvedFeedforward(
            const PlantModel& plant,
            const PlantModel::PreparedParams& prepared,
            const VehicleState::StateVector& initialState,
            int steps,
            Solver&& solveFeedforward)
        {
            VehicleState::StateVector state = initialState;
            for (int step = 0; step < steps; ++step)
            {
                const DriveCommandSolution solution = solveFeedforward(state);
                state = plant.integrate(state, solution.control, kPredictDtSeconds, prepared);
            }

            return state;
        }

        void AssertPredictStateNearTarget(
            const VehicleState::StateVector& state,
            float expectedForwardVelocityMps,
            float expectedYawRateRadps,
            float forwardToleranceMps,
            float yawToleranceRadps,
            float maxLateralVelocityMps)
        {
            Assert::AreEqual(expectedForwardVelocityMps, state(VehicleState::kU), forwardToleranceMps);
            Assert::AreEqual(expectedYawRateRadps, state(VehicleState::kR), yawToleranceRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) <= maxLateralVelocityMps);
        }

        float RelativeTolerance(float expectedValue, float minimumTolerance) noexcept
        {
            return (std::max)(minimumTolerance, std::fabs(expectedValue) * kFeedforwardRelativeTolerance);
        }

        void AssertPredictStateNearTargetWithContext(
            const VehicleState::StateVector& state,
            float expectedForwardVelocityMps,
            float expectedYawRateRadps,
            float forwardToleranceMps,
            float yawToleranceRadps,
            float maxLateralVelocityMps,
            const std::wstring& context)
        {
            const float forwardVelocityMps = state(VehicleState::kU);
            const float yawRateRadps = state(VehicleState::kR);
            const float lateralVelocityMps = state(VehicleState::kV);
            if ((std::fabs(forwardVelocityMps - expectedForwardVelocityMps) > forwardToleranceMps) ||
                (std::fabs(yawRateRadps - expectedYawRateRadps) > yawToleranceRadps) ||
                (std::fabs(lateralVelocityMps) > maxLateralVelocityMps))
            {
                const std::wstring message =
                    context +
                    (std::wstring(L": predicted U,R,V = ") +
                        std::to_wstring(forwardVelocityMps) + L"," +
                        std::to_wstring(yawRateRadps) + L"," +
                        std::to_wstring(lateralVelocityMps) +
                        L" expected U,R = " +
                        std::to_wstring(expectedForwardVelocityMps) + L"," +
                        std::to_wstring(expectedYawRateRadps) +
                        L" tolerances U,R,V = " +
                        std::to_wstring(forwardToleranceMps) + L"," +
                        std::to_wstring(yawToleranceRadps) + L"," +
                        std::to_wstring(maxLateralVelocityMps));
                Assert::Fail(message.c_str());
            }
        }

        template <typename Solver>
        void AssertVelocityTargetFeedforwardAcrossHorizons(
            const PlantModel& plant,
            const PlantModel::PreparedParams& prepared,
            const PlantParams& params,
            float targetForwardVelocityMps,
            float targetYawRateRadps,
            Solver&& solveFeedforward,
            const wchar_t* scenarioLabel)
        {
            const auto& solver = solveFeedforward;
            for (const FeedforwardPredictHorizon& horizon : kExtendedFeedforwardPredictHorizons)
            {
                const VehicleState::StateVector predictedState =
                    RunPlantPredictLoopWithResolvedFeedforward(
                        plant,
                        prepared,
                        BuildRollingUkfState(0.0f, 0.0f, params),
                        horizon.steps,
                        solver);
                AssertPredictStateNearTargetWithContext(
                    predictedState,
                    targetForwardVelocityMps,
                    targetYawRateRadps,
                    RelativeTolerance(targetForwardVelocityMps, 0.01f),
                    RelativeTolerance(targetYawRateRadps, 0.02f),
                    0.04f,
                    std::wstring(scenarioLabel) + L" @ " + horizon.label);
            }
        }
    }

    TEST_CLASS(PlantModelDriveCommandTest)
    {
    public:
        TEST_METHOD(PlantModelSolveDriveCommandsZeroRequestReturnsZeroCommand)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(0.0f, 0.0f, 0.0f, 0.0f, params, 0.80f, params.supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(solution.converged);
            Assert::AreEqual(0.0f, solution.control.leftMotorCommand, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.control.rightMotorCommand, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.leftWheelTorqueNm, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.rightWheelTorqueNm, 1.0e-6f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsIncludesWheelInertiaAndFriction)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(1.8f, 3.5f, 1.5f, 14.0f, params, 0.80f, params.supplyVoltageV);

            const float expectedLeftTorqueNm =
                solution.leftContactTorqueNm +
                (params.equivalentWheelInertiaKgM2 * solution.leftWheelAccelRadps2) +
                plant.driveFrictionTorque(
                    solution.leftWheelSpeedRadps,
                    solution.leftContactTorqueNm + (params.equivalentWheelInertiaKgM2 * solution.leftWheelAccelRadps2),
                    params);
            const float expectedRightTorqueNm =
                solution.rightContactTorqueNm +
                (params.equivalentWheelInertiaKgM2 * solution.rightWheelAccelRadps2) +
                plant.driveFrictionTorque(
                    solution.rightWheelSpeedRadps,
                    solution.rightContactTorqueNm + (params.equivalentWheelInertiaKgM2 * solution.rightWheelAccelRadps2),
                    params);

            Assert::AreEqual(expectedLeftTorqueNm, solution.leftWheelTorqueNm, 1.0e-6f);
            Assert::AreEqual(expectedRightTorqueNm, solution.rightWheelTorqueNm, 1.0e-6f);
            Assert::IsTrue(std::fabs(solution.leftWheelTorqueNm - solution.leftContactTorqueNm) > 1.0e-4f);
            Assert::IsTrue(std::fabs(solution.rightWheelTorqueNm - solution.rightContactTorqueNm) > 1.0e-4f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsReturnsBodyConsistentOperatingPointAtModerateCombinedTarget)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float forwardVelocityMps = 2.1f;
            constexpr float desiredLongitudinalAccelMps2 = 1.2f;
            constexpr float yawRateRadps = 4.0f;
            constexpr float desiredYawAccelRadps2 = 4.5f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    desiredYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;

            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);
            const float expectedLeftWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 + (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;
            const float expectedRightWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 - (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;

            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(achieved.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(achieved.yawAccelRadps2));
            Assert::AreEqual(desiredLongitudinalAccelMps2, solution.commandedLongitudinalAccelMps2, 0.05f);
            Assert::AreEqual(desiredYawAccelRadps2, solution.commandedYawAccelRadps2, 0.20f);
            Assert::AreEqual(expectedLeftWheelAccelRadps2, solution.leftWheelAccelRadps2, 1.0e-5f);
            Assert::AreEqual(expectedRightWheelAccelRadps2, solution.rightWheelAccelRadps2, 1.0e-5f);
            Assert::IsTrue(std::isfinite(achieved.stateDot(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(achieved.stateDot(VehicleState::kOmegaR)));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    targetForwardVelocityMps,
                    0.0f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoop(
                    plant,
                    prepared,
                    BuildRollingUkfState(targetForwardVelocityMps, 0.0f, params),
                    solution.control,
                    kFeedforwardPredictSteps);

            Assert::IsFalse(solution.tractionLimited);
            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                0.0f,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                0.02f,
                0.02f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsStateFeedforwardHoldsOperatingPointInPredict)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            const VehicleState::StateVector operatingState =
                BuildRollingUkfState(targetForwardVelocityMps, 0.0f, params);
            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    operatingState,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                     params.supplyVoltageV);

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoop(
                    plant,
                    prepared,
                    operatingState,
                    solution.control,
                    kFeedforwardPredictSteps);

            Assert::IsFalse(solution.tractionLimited);
            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                0.0f,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                0.02f,
                0.02f);
        }

        TEST_METHOD(PlantModelSolveClosedLoopDriveCommandsReducedFeedforwardHoldsOperatingPointInPredict)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            const DriveCommandSolution solution =
                plant.solveClosedLoopDriveCommands(
                    targetForwardVelocityMps,
                    0.0f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoop(
                    plant,
                    prepared,
                    BuildRollingUkfState(targetForwardVelocityMps, 0.0f, params),
                    solution.control,
                    kFeedforwardPredictSteps);

            Assert::IsFalse(solution.tractionLimited);
            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                0.0f,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                0.02f,
                0.02f);
        }

        TEST_METHOD(PlantModelSolveClosedLoopDriveCommandsStateFeedforwardHoldsOperatingPointInPredict)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            const VehicleState::StateVector operatingState =
                BuildRollingUkfState(targetForwardVelocityMps, 0.0f, params);
            const DriveCommandSolution solution =
                plant.solveClosedLoopDriveCommands(
                    operatingState,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                     params.supplyVoltageV);

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoop(
                    plant,
                    prepared,
                    operatingState,
                    solution.control,
                    kFeedforwardPredictSteps);

            Assert::IsFalse(solution.tractionLimited);
            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                0.0f,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                0.02f,
                0.02f);
        }

        TEST_METHOD(PlantModelResolveVelocityTargetAccelerationsUsesLongitudinalLimitForPureSpeedChange)
        {
            PlantModel plant;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.resolveVelocityTargetAccelerations(
                0.0f,
                0.30f,
                0.0f,
                0.0f,
                9.0f,
                400.0f,
                PlantModel::kDefaultVelocityTargetResponseTimeS,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);

            Assert::AreEqual(9.0f, desiredLongitudinalAccelMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, desiredYawAccelRadps2, 1.0e-6f);
        }

        TEST_METHOD(PlantModelResolveVelocityTargetAccelerationsUsesYawLimitForPureYawChange)
        {
            PlantModel plant;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.resolveVelocityTargetAccelerations(
                0.0f,
                0.0f,
                0.0f,
                8.0f,
                9.0f,
                400.0f,
                PlantModel::kDefaultVelocityTargetResponseTimeS,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);

            Assert::AreEqual(0.0f, desiredLongitudinalAccelMps2, 1.0e-6f);
            Assert::AreEqual(400.0f, desiredYawAccelRadps2, 1.0e-4f);
        }

        TEST_METHOD(PlantModelResolveVelocityTargetAccelerationsBalancesCombinedRequestsToSharedArrivalScale)
        {
            PlantModel plant;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.resolveVelocityTargetAccelerations(
                0.0f,
                0.06f,
                0.0f,
                8.0f,
                9.0f,
                400.0f,
                PlantModel::kDefaultVelocityTargetResponseTimeS,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);

            Assert::AreEqual(3.0f, desiredLongitudinalAccelMps2, 1.0e-4f);
            Assert::AreEqual(400.0f, desiredYawAccelRadps2, 1.0e-4f);
        }

        TEST_METHOD(PlantModelResolveWheelMotionTargetsUsesEffectiveTrackWidthAndWheelRadius)
        {
            PlantModel plant;
            const PlantModel::PreparedParams params = PlantModel::Prepare(PlantParams::Default());
            constexpr float targetForwardVelocityMps = 0.30f;
            constexpr float targetYawRateRadps = (0.30f / 0.063f);
            constexpr float targetLongitudinalAccelMps2 = 3.0f;
            constexpr float targetYawAccelRadps2 = 40.0f;
            float leftTargetVelocityMps = 0.0f;
            float rightTargetVelocityMps = 0.0f;
            float leftTargetAccelMps2 = 0.0f;
            float rightTargetAccelMps2 = 0.0f;
            float leftTargetOmegaRadps = 0.0f;
            float rightTargetOmegaRadps = 0.0f;

            plant.resolveWheelMotionTargets(
                targetForwardVelocityMps,
                targetYawRateRadps,
                targetLongitudinalAccelMps2,
                targetYawAccelRadps2,
                params,
                leftTargetVelocityMps,
                rightTargetVelocityMps,
                leftTargetAccelMps2,
                rightTargetAccelMps2,
                leftTargetOmegaRadps,
                rightTargetOmegaRadps);

            const float effectiveTrackWidthM =
                Vehicle::GetEffectiveTrackWidthForMotion(
                    targetForwardVelocityMps,
                    targetYawRateRadps);
            const float expectedLeftTargetVelocityMps =
                targetForwardVelocityMps + (0.5f * effectiveTrackWidthM * targetYawRateRadps);
            const float expectedRightTargetVelocityMps =
                targetForwardVelocityMps - (0.5f * effectiveTrackWidthM * targetYawRateRadps);
            const float expectedLeftTargetAccelMps2 =
                targetLongitudinalAccelMps2 + (0.5f * effectiveTrackWidthM * targetYawAccelRadps2);
            const float expectedRightTargetAccelMps2 =
                targetLongitudinalAccelMps2 - (0.5f * effectiveTrackWidthM * targetYawAccelRadps2);

            Assert::AreEqual(expectedLeftTargetVelocityMps, leftTargetVelocityMps, 1.0e-6f);
            Assert::AreEqual(expectedRightTargetVelocityMps, rightTargetVelocityMps, 1.0e-6f);
            Assert::AreEqual(expectedLeftTargetAccelMps2, leftTargetAccelMps2, 1.0e-6f);
            Assert::AreEqual(expectedRightTargetAccelMps2, rightTargetAccelMps2, 1.0e-6f);
            Assert::AreEqual(expectedLeftTargetVelocityMps * params.invWheelRadiusM, leftTargetOmegaRadps, 1.0e-6f);
            Assert::AreEqual(expectedRightTargetVelocityMps * params.invWheelRadiusM, rightTargetOmegaRadps, 1.0e-6f);
        }

        TEST_METHOD(PlantModelVelocityTargetTechnicalLimitsReportReachableEnvelope)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            float maxLongitudinalAccelMps2 = 0.0f;
            float maxYawAccelRadps2 = 0.0f;

            plant.velocityTargetTechnicalLimits(
                0.0f,
                0.0f,
                params,
                maxLongitudinalAccelMps2,
                maxYawAccelRadps2);

            Assert::IsTrue(maxLongitudinalAccelMps2 > 0.0f);
            Assert::IsTrue(maxYawAccelRadps2 > 0.0f);

            const DriveCommandSolution longitudinalSolution =
                plant.solveDriveCommands(
                    0.0f,
                    0.95f * maxLongitudinalAccelMps2,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);
            const DriveCommandSolution yawSolution =
                plant.solveDriveCommands(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.95f * maxYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Assert::IsFalse(longitudinalSolution.tractionLimited);
            Assert::IsFalse(yawSolution.tractionLimited);
            Assert::AreEqual(
                0.95f * maxLongitudinalAccelMps2,
                longitudinalSolution.commandedLongitudinalAccelMps2,
                0.10f);
            Assert::AreEqual(
                0.95f * maxYawAccelRadps2,
                yawSolution.commandedYawAccelRadps2,
                0.50f);
        }

        TEST_METHOD(PlantModelVelocityTargetTechnicalLimitsShrinkUnderCorneringLoad)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            float straightLongitudinalAccelMps2 = 0.0f;
            float straightYawAccelRadps2 = 0.0f;
            float loadedLongitudinalAccelMps2 = 0.0f;
            float loadedYawAccelRadps2 = 0.0f;

            plant.velocityTargetTechnicalLimits(
                0.0f,
                0.0f,
                params,
                straightLongitudinalAccelMps2,
                straightYawAccelRadps2);
            plant.velocityTargetTechnicalLimits(
                2.0f,
                4.0f,
                params,
                loadedLongitudinalAccelMps2,
                loadedYawAccelRadps2);

            Assert::IsTrue(loadedLongitudinalAccelMps2 < straightLongitudinalAccelMps2);
            Assert::IsTrue(loadedYawAccelRadps2 < straightYawAccelRadps2);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetUsesCanonicalDefaultResponseTime)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(1.20f, 1.215f, 0.40f, 0.420f, params);
            const DriveCommandSolution explicitDefaultSolution =
                plant.solveDriveCommandsForVelocityTarget(
                    1.20f,
                    1.215f,
                    0.40f,
                    0.420f,
                    params,
                    0.80f,
                    0.0f,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            Assert::AreEqual(static_cast<int>(explicitDefaultSolution.tractionLimited), static_cast<int>(solution.tractionLimited));
            Assert::AreEqual(explicitDefaultSolution.commandedLongitudinalAccelMps2, solution.commandedLongitudinalAccelMps2, 1.0e-6f);
            Assert::AreEqual(explicitDefaultSolution.commandedYawAccelRadps2, solution.commandedYawAccelRadps2, 1.0e-6f);
            Assert::AreEqual(explicitDefaultSolution.control.leftMotorCommand, solution.control.leftMotorCommand, 1.0e-6f);
            Assert::AreEqual(explicitDefaultSolution.control.rightMotorCommand, solution.control.rightMotorCommand, 1.0e-6f);
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) <= 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetReducedFeedforwardReachesTargetWithinTenPercentAfterOneSecond)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoopWithResolvedFeedforward(
                    plant,
                    prepared,
                    BuildRollingUkfState(0.0f, 0.0f, params),
                    kFeedforwardPredictSteps,
                    [&](const VehicleState::StateVector& state)
                    {
                        return plant.solveDriveCommandsForVelocityTarget(
                            state(VehicleState::kU),
                            targetForwardVelocityMps,
                            state(VehicleState::kR),
                            targetYawRateRadps,
                            params,
                            0.80f,
                            params.supplyVoltageV,
                            responseTimeS);
                    });

            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                RelativeTolerance(targetYawRateRadps, 0.02f),
                0.04f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetStateFeedforwardReachesTargetWithinTenPercentAfterOneSecond)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoopWithResolvedFeedforward(
                    plant,
                    prepared,
                    BuildRollingUkfState(0.0f, 0.0f, params),
                    kFeedforwardPredictSteps,
                    [&](const VehicleState::StateVector& state)
                    {
                        return plant.solveDriveCommandsForVelocityTarget(
                            state,
                            targetForwardVelocityMps,
                            targetYawRateRadps,
                            params,
                            0.80f,
                            params.supplyVoltageV,
                            responseTimeS);
                    });

            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                RelativeTolerance(targetYawRateRadps, 0.02f),
                0.04f);
        }

        TEST_METHOD(PlantModelSolveClosedLoopDriveCommandsForVelocityTargetReducedFeedforwardReachesTargetWithinTenPercentAfterOneSecond)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoopWithResolvedFeedforward(
                    plant,
                    prepared,
                    BuildRollingUkfState(0.0f, 0.0f, params),
                    kFeedforwardPredictSteps,
                    [&](const VehicleState::StateVector& state)
                    {
                        return plant.solveClosedLoopDriveCommandsForVelocityTarget(
                            state(VehicleState::kU),
                            targetForwardVelocityMps,
                            state(VehicleState::kR),
                            targetYawRateRadps,
                            params,
                            0.80f,
                            params.supplyVoltageV,
                            responseTimeS);
                    });

            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                RelativeTolerance(targetYawRateRadps, 0.02f),
                0.04f);
        }

        TEST_METHOD(PlantModelSolveClosedLoopDriveCommandsForVelocityTargetStateFeedforwardReachesTargetWithinTenPercentAfterOneSecond)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            const VehicleState::StateVector predictedState =
                RunPlantPredictLoopWithResolvedFeedforward(
                    plant,
                    prepared,
                    BuildRollingUkfState(0.0f, 0.0f, params),
                    kFeedforwardPredictSteps,
                    [&](const VehicleState::StateVector& state)
                    {
                        return plant.solveClosedLoopDriveCommandsForVelocityTarget(
                            state,
                            targetForwardVelocityMps,
                            targetYawRateRadps,
                            params,
                            0.80f,
                            params.supplyVoltageV,
                            responseTimeS);
                    });

            AssertPredictStateNearTarget(
                predictedState,
                targetForwardVelocityMps,
                targetYawRateRadps,
                RelativeTolerance(targetForwardVelocityMps, 0.01f),
                RelativeTolerance(targetYawRateRadps, 0.02f),
                0.04f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetReducedFeedforwardReachesTargetWithinTenPercentAcrossExtendedHorizons)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            AssertVelocityTargetFeedforwardAcrossHorizons(
                plant,
                prepared,
                params,
                targetForwardVelocityMps,
                targetYawRateRadps,
                [&](const VehicleState::StateVector& state)
                {
                    return plant.solveDriveCommandsForVelocityTarget(
                        state(VehicleState::kU),
                        targetForwardVelocityMps,
                        state(VehicleState::kR),
                        targetYawRateRadps,
                        params,
                        0.80f,
                        params.supplyVoltageV,
                        responseTimeS);
                },
                L"Reduced open-loop velocity target");
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetStateFeedforwardReachesTargetWithinTenPercentAcrossExtendedHorizons)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            AssertVelocityTargetFeedforwardAcrossHorizons(
                plant,
                prepared,
                params,
                targetForwardVelocityMps,
                targetYawRateRadps,
                [&](const VehicleState::StateVector& state)
                {
                    return plant.solveDriveCommandsForVelocityTarget(
                        state,
                        targetForwardVelocityMps,
                        targetYawRateRadps,
                        params,
                        0.80f,
                        params.supplyVoltageV,
                        responseTimeS);
                },
                L"State open-loop velocity target");
        }

        TEST_METHOD(PlantModelSolveClosedLoopDriveCommandsForVelocityTargetReducedFeedforwardReachesTargetWithinTenPercentAcrossExtendedHorizons)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            AssertVelocityTargetFeedforwardAcrossHorizons(
                plant,
                prepared,
                params,
                targetForwardVelocityMps,
                targetYawRateRadps,
                [&](const VehicleState::StateVector& state)
                {
                    return plant.solveClosedLoopDriveCommandsForVelocityTarget(
                        state(VehicleState::kU),
                        targetForwardVelocityMps,
                        state(VehicleState::kR),
                        targetYawRateRadps,
                        params,
                        0.80f,
                        params.supplyVoltageV,
                        responseTimeS);
                },
                L"Reduced closed-loop velocity target");
        }

        TEST_METHOD(PlantModelSolveClosedLoopDriveCommandsForVelocityTargetStateFeedforwardReachesTargetWithinTenPercentAcrossExtendedHorizons)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 0.60f;
            constexpr float responseTimeS = 0.10f;

            AssertVelocityTargetFeedforwardAcrossHorizons(
                plant,
                prepared,
                params,
                targetForwardVelocityMps,
                targetYawRateRadps,
                [&](const VehicleState::StateVector& state)
                {
                    return plant.solveClosedLoopDriveCommandsForVelocityTarget(
                        state,
                        targetForwardVelocityMps,
                        targetYawRateRadps,
                        params,
                        0.80f,
                        params.supplyVoltageV,
                        responseTimeS);
                },
                L"State closed-loop velocity target");
        }

        TEST_METHOD(PlantModelClosedLoopVelocityTargetKeepsTenPercentTractionReserveWhenLimited)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float aggressiveResponseTimeS = 0.005f;
            const DriveCommandSolution tractionLimited =
                plant.solveDriveCommandsForVelocityTarget(
                    0.0f,
                    0.20f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    0.0f,
                    aggressiveResponseTimeS);
            const DriveCommandSolution reserved =
                plant.solveClosedLoopDriveCommandsForVelocityTarget(
                    0.0f,
                    0.20f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    0.0f,
                    aggressiveResponseTimeS);

            Assert::IsTrue(tractionLimited.tractionLimited);
            Assert::IsFalse(reserved.tractionLimited);
            Assert::IsTrue(reserved.commandedLongitudinalAccelMps2 < tractionLimited.commandedLongitudinalAccelMps2);
            Assert::AreEqual(
                0.90f * tractionLimited.commandedLongitudinalAccelMps2,
                reserved.commandedLongitudinalAccelMps2,
                0.25f);
        }

        TEST_METHOD(PlantModelClosedLoopVelocityTargetKeepsPositiveYawFeedforwardDirectionAtNonzeroYawRate)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveClosedLoopDriveCommandsForVelocityTarget(
                    0.0f,
                    0.0f,
                    0.35f,
                    3.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Logger::WriteMessage(
                (std::wstring(L"positive yaw left/right command, wheel speeds, predicted accel = ") +
                    std::to_wstring(solution.control.leftMotorCommand) + L"," +
                    std::to_wstring(solution.control.rightMotorCommand) + L"," +
                    std::to_wstring(solution.leftWheelSpeedRadps) + L"," +
                    std::to_wstring(solution.rightWheelSpeedRadps) + L"," +
                    std::to_wstring(solution.commandedYawAccelRadps2)).c_str());
            Assert::IsTrue(std::isfinite(solution.commandedYawAccelRadps2));
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(solution.leftWheelSpeedRadps));
            Assert::IsTrue(std::isfinite(solution.rightWheelSpeedRadps));
            Assert::IsTrue(solution.leftWheelSpeedRadps > solution.rightWheelSpeedRadps);
            if (!(solution.control.leftMotorCommand > 0.0f) ||
                !(solution.control.rightMotorCommand < 0.0f))
            {
                Assert::Fail(
                    (std::wstring(L"positive yaw left/right command, wheel speeds, predicted accel = ") +
                        std::to_wstring(solution.control.leftMotorCommand) + L"," +
                        std::to_wstring(solution.control.rightMotorCommand) + L"," +
                        std::to_wstring(solution.leftWheelSpeedRadps) + L"," +
                        std::to_wstring(solution.rightWheelSpeedRadps) + L"," +
                        std::to_wstring(solution.commandedYawAccelRadps2)).c_str());
            }
        }

        TEST_METHOD(PlantModelClosedLoopVelocityTargetKeepsNegativeYawFeedforwardDirectionAtNonzeroYawRate)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveClosedLoopDriveCommandsForVelocityTarget(
                    0.0f,
                    0.0f,
                    -0.35f,
                    -3.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Logger::WriteMessage(
                (std::wstring(L"negative yaw left/right command, wheel speeds, predicted accel = ") +
                    std::to_wstring(solution.control.leftMotorCommand) + L"," +
                    std::to_wstring(solution.control.rightMotorCommand) + L"," +
                    std::to_wstring(solution.leftWheelSpeedRadps) + L"," +
                    std::to_wstring(solution.rightWheelSpeedRadps) + L"," +
                    std::to_wstring(solution.commandedYawAccelRadps2)).c_str());
            Assert::IsTrue(std::isfinite(solution.commandedYawAccelRadps2));
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(solution.leftWheelSpeedRadps));
            Assert::IsTrue(std::isfinite(solution.rightWheelSpeedRadps));
            Assert::IsTrue(solution.leftWheelSpeedRadps < solution.rightWheelSpeedRadps);
            if (!(solution.control.leftMotorCommand < 0.0f) ||
                !(solution.control.rightMotorCommand > 0.0f))
            {
                Assert::Fail(
                    (std::wstring(L"negative yaw left/right command, wheel speeds, predicted accel = ") +
                        std::to_wstring(solution.control.leftMotorCommand) + L"," +
                        std::to_wstring(solution.control.rightMotorCommand) + L"," +
                        std::to_wstring(solution.leftWheelSpeedRadps) + L"," +
                        std::to_wstring(solution.rightWheelSpeedRadps) + L"," +
                        std::to_wstring(solution.commandedYawAccelRadps2)).c_str());
            }
        }

        TEST_METHOD(PlantModelSolveDriveCommandsValidationUsesFullCurrentState)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            PlantModel::StateVector currentState = PlantModel::StateVector::Zero();
            currentState(VehicleState::kU) = 0.60f;
            currentState(VehicleState::kV) = 0.25f;
            currentState(VehicleState::kR) = 0.35f;

            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float rollingLeftWheelSpeedRadps =
                (currentState(VehicleState::kU) + (halfTrackWidthM * currentState(VehicleState::kR))) / params.wheelRadiusM;
            const float rollingRightWheelSpeedRadps =
                (currentState(VehicleState::kU) - (halfTrackWidthM * currentState(VehicleState::kR))) / params.wheelRadiusM;
            currentState(VehicleState::kOmegaL) = rollingLeftWheelSpeedRadps + 2.0f;
            currentState(VehicleState::kOmegaR) = rollingRightWheelSpeedRadps - 2.0f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    currentState,
                    0.0f,
                    8.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            PlantModel::StateVector validationState = currentState;
            validationState(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            validationState(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achievedFull = plant.forwardStep(validationState, solution.control, params);

            PlantModel::StateVector zeroLateralValidationState = validationState;
            zeroLateralValidationState(VehicleState::kV) = 0.0f;
            const PlantDerivatives achievedZeroLateral =
                plant.forwardStep(zeroLateralValidationState, solution.control, params);

            Assert::IsTrue(std::isfinite(solution.commandedYawAccelRadps2));
            Assert::IsTrue(std::isfinite(achievedFull.yawAccelRadps2));
            Assert::IsTrue(std::isfinite(achievedZeroLateral.yawAccelRadps2));
            Assert::IsTrue(std::fabs(achievedFull.yawAccelRadps2 - achievedZeroLateral.yawAccelRadps2) > 0.01f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsCompensatesYawRateDampingWhenNotTractionLimited)
        {
            PlantModel plant;
            PlantParams params = PlantParams::Default();
            params.yawRateDampingNmsPerRad = 0.012f;

            constexpr float forwardVelocityMps = 0.6f;
            constexpr float desiredLongitudinalAccelMps2 = 0.0f;
            constexpr float yawRateRadps = 1.4f;
            constexpr float desiredYawAccelRadps2 = 18.0f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    desiredYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            VehicleState::StateVector validationState = VehicleState::StateVector::Zero();
            validationState(VehicleState::kU) = forwardVelocityMps;
            validationState(VehicleState::kR) = yawRateRadps;
            validationState(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            validationState(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achieved = plant.forwardStep(validationState, solution.control, params);
            const float expectedLeftWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 + (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;
            const float expectedRightWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 - (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;

            Assert::IsFalse(solution.tractionLimited);
            Assert::AreEqual(desiredYawAccelRadps2, solution.commandedYawAccelRadps2, 0.05f);
            Assert::AreEqual(expectedLeftWheelAccelRadps2, solution.leftWheelAccelRadps2, 1.0e-5f);
            Assert::AreEqual(expectedRightWheelAccelRadps2, solution.rightWheelAccelRadps2, 1.0e-5f);
            Assert::IsTrue(std::isfinite(achieved.yawAccelRadps2));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsTractionLimitedYawClampMatchesFixedSplitCapacity)
        {
            PlantModel plant;
            PlantParams params = PlantParams::Default();
            params.frontLongitudinalForceSplit = 0.95f;

            constexpr float fanDutyCycle = 0.80f;
            constexpr float desiredYawAccelRadps2 = 2000.0f;
            const float forceEpsilonN =
                (std::isfinite(params.forceEpsilonN) && (params.forceEpsilonN > 0.0f)) ?
                params.forceEpsilonN :
                1.0e-4f;

            PlantModel::StateVector currentState = PlantModel::StateVector::Zero();
            currentState(VehicleState::kU) = 0.9f;
            currentState(VehicleState::kV) = 0.0f;
            currentState(VehicleState::kR) = 0.0f;

            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            currentState(VehicleState::kOmegaL) =
                (currentState(VehicleState::kU) + (halfTrackWidthM * currentState(VehicleState::kR))) / params.wheelRadiusM;
            currentState(VehicleState::kOmegaR) =
                (currentState(VehicleState::kU) - (halfTrackWidthM * currentState(VehicleState::kR))) / params.wheelRadiusM;

            ControlInput baselineControl{};
            baselineControl.fanDutyCycle = fanDutyCycle;
            const ContactForces baselineForces = plant.tireForces(currentState, baselineControl, params);
            const float baselineFrontRightForceN =
                baselineForces.contacts[0].rightForceN +
                baselineForces.contacts[1].rightForceN;
            const float baselineRearRightForceN =
                baselineForces.contacts[2].rightForceN +
                baselineForces.contacts[3].rightForceN;
            const float baselineYawMomentNm =
                std::fabs(params.contactPatchLongitudinalOffsetM) *
                (baselineFrontRightForceN - baselineRearRightForceN);

            const float totalNormalLoadN = params.TotalNormalLoadN(fanDutyCycle);
            const float envelopeMu =
                (std::isfinite(params.combinedAccelPeakMps2) &&
                 (params.combinedAccelPeakMps2 > 0.0f) &&
                 std::isfinite(params.massKg) &&
                 (params.massKg > 0.0f)) ?
                ((params.combinedAccelPeakMps2 * params.massKg) / (std::max)(totalNormalLoadN, forceEpsilonN)) :
                0.0f;
            const float peakFront =
                (std::isfinite(params.muFrontPeak) && (params.muFrontPeak > 0.0f)) ?
                params.muFrontPeak :
                ((envelopeMu > 0.0f) ? envelopeMu : (std::max)(0.0f, params.muFront));
            const float peakRear =
                (std::isfinite(params.muRearPeak) && (params.muRearPeak > 0.0f)) ?
                params.muRearPeak :
                ((envelopeMu > 0.0f) ? envelopeMu : (std::max)(0.0f, params.muRear));
            const float flPeakForceN = peakFront * params.FrontWheelLoadN(fanDutyCycle);
            const float frPeakForceN = peakFront * params.FrontWheelLoadN(fanDutyCycle);
            const float rlPeakForceN = peakRear * params.RearWheelLoadN(fanDutyCycle);
            const float rrPeakForceN = peakRear * params.RearWheelLoadN(fanDutyCycle);
            const float flForwardCapacityN =
                MazeMap::Math::Sqrtf((std::max)(
                    0.0f,
                    (flPeakForceN * flPeakForceN) - (baselineForces.contacts[0].rightForceN * baselineForces.contacts[0].rightForceN)));
            const float frForwardCapacityN =
                MazeMap::Math::Sqrtf((std::max)(
                    0.0f,
                    (frPeakForceN * frPeakForceN) - (baselineForces.contacts[1].rightForceN * baselineForces.contacts[1].rightForceN)));
            const float rlForwardCapacityN =
                MazeMap::Math::Sqrtf((std::max)(
                    0.0f,
                    (rlPeakForceN * rlPeakForceN) - (baselineForces.contacts[2].rightForceN * baselineForces.contacts[2].rightForceN)));
            const float rrForwardCapacityN =
                MazeMap::Math::Sqrtf((std::max)(
                    0.0f,
                    (rrPeakForceN * rrPeakForceN) - (baselineForces.contacts[3].rightForceN * baselineForces.contacts[3].rightForceN)));
            const float lambdaFront = params.frontLongitudinalForceSplit;
            const float lambdaRear = 1.0f - lambdaFront;
            const auto fixedSplitBankForwardCapacityN =
                [&](float frontCapacityN, float rearCapacityN) -> float
            {
                float capacityN = (std::numeric_limits<float>::infinity)();
                if (lambdaFront > forceEpsilonN)
                {
                    capacityN = (std::min)(capacityN, frontCapacityN / lambdaFront);
                }
                if (lambdaRear > forceEpsilonN)
                {
                    capacityN = (std::min)(capacityN, rearCapacityN / lambdaRear);
                }

                return std::isfinite(capacityN) ? (std::max)(0.0f, capacityN) : 0.0f;
            };
            const float leftBankForwardCapacityN =
                fixedSplitBankForwardCapacityN(flForwardCapacityN, rlForwardCapacityN);
            const float rightBankForwardCapacityN =
                fixedSplitBankForwardCapacityN(frForwardCapacityN, rrForwardCapacityN);
            const float legacyLeftSummedCapacityN = flForwardCapacityN + rlForwardCapacityN;
            const float legacyRightSummedCapacityN = frForwardCapacityN + rrForwardCapacityN;

            const float yawDampingNmPerRadps = (std::max)(0.0f, params.yawRateDampingNmsPerRad);
            const float totalYawMomentCommandNm =
                (params.yawInertiaKgM2 * desiredYawAccelRadps2) +
                (yawDampingNmPerRadps * currentState(VehicleState::kR));
            const float longitudinalYawMomentCommandNm = totalYawMomentCommandNm - baselineYawMomentNm;
            const float leftBankForceUnclippedN = longitudinalYawMomentCommandNm / params.trackWidthM;
            const float rightBankForceUnclippedN = -longitudinalYawMomentCommandNm / params.trackWidthM;

            float tractionScale = 1.0f;
            tractionScale =
                (std::min)(
                    tractionScale,
                    leftBankForwardCapacityN / (std::max)(std::fabs(leftBankForceUnclippedN), forceEpsilonN));
            tractionScale =
                (std::min)(
                    tractionScale,
                    rightBankForwardCapacityN / (std::max)(std::fabs(rightBankForceUnclippedN), forceEpsilonN));

            const float requestedLongitudinalYawMomentNm = tractionScale * longitudinalYawMomentCommandNm;
            const float minLongitudinalYawMomentNm =
                (std::max)(
                    params.trackWidthM * (-leftBankForwardCapacityN),
                    params.trackWidthM * (-rightBankForwardCapacityN));
            const float maxLongitudinalYawMomentNm =
                (std::min)(
                    params.trackWidthM * leftBankForwardCapacityN,
                    params.trackWidthM * rightBankForwardCapacityN);
            const float refinedLongitudinalYawMomentNm =
                (minLongitudinalYawMomentNm <= maxLongitudinalYawMomentNm) ?
                (std::clamp)(
                    requestedLongitudinalYawMomentNm,
                    minLongitudinalYawMomentNm,
                    maxLongitudinalYawMomentNm) :
                requestedLongitudinalYawMomentNm;
            const float achievedYawMomentNm = baselineYawMomentNm + refinedLongitudinalYawMomentNm;
            const float expectedYawAccelRadps2 =
                (achievedYawMomentNm - (yawDampingNmPerRadps * currentState(VehicleState::kR))) / params.yawInertiaKgM2;
            float legacyTractionScale = 1.0f;
            legacyTractionScale =
                (std::min)(
                    legacyTractionScale,
                    legacyLeftSummedCapacityN / (std::max)(std::fabs(leftBankForceUnclippedN), forceEpsilonN));
            legacyTractionScale =
                (std::min)(
                    legacyTractionScale,
                    legacyRightSummedCapacityN / (std::max)(std::fabs(rightBankForceUnclippedN), forceEpsilonN));
            const float legacyRequestedLongitudinalYawMomentNm =
                legacyTractionScale * longitudinalYawMomentCommandNm;
            const float legacyMinLongitudinalYawMomentNm =
                (std::max)(
                    params.trackWidthM * (-legacyLeftSummedCapacityN),
                    params.trackWidthM * (-legacyRightSummedCapacityN));
            const float legacyMaxLongitudinalYawMomentNm =
                (std::min)(
                    params.trackWidthM * legacyLeftSummedCapacityN,
                    params.trackWidthM * legacyRightSummedCapacityN);
            const float legacyRefinedLongitudinalYawMomentNm =
                (legacyMinLongitudinalYawMomentNm <= legacyMaxLongitudinalYawMomentNm) ?
                (std::clamp)(
                    legacyRequestedLongitudinalYawMomentNm,
                    legacyMinLongitudinalYawMomentNm,
                    legacyMaxLongitudinalYawMomentNm) :
                legacyRequestedLongitudinalYawMomentNm;
            const float legacyAchievedYawMomentNm = baselineYawMomentNm + legacyRefinedLongitudinalYawMomentNm;
            const float legacyExpectedYawAccelRadps2 =
                (legacyAchievedYawMomentNm - (yawDampingNmPerRadps * currentState(VehicleState::kR))) / params.yawInertiaKgM2;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    currentState,
                    0.0f,
                    desiredYawAccelRadps2,
                    params,
                    fanDutyCycle,
                    params.supplyVoltageV);
            const DriveCommandSolution repeatSolution =
                plant.solveDriveCommands(
                    currentState,
                    0.0f,
                    desiredYawAccelRadps2,
                    params,
                    fanDutyCycle,
                    params.supplyVoltageV);

            Logger::WriteMessage(
                (std::wstring(L"traction-limited fixed expected/actual caps = ") +
                    std::to_wstring(expectedYawAccelRadps2) + L"," +
                    std::to_wstring(solution.commandedYawAccelRadps2) + L"," +
                    std::to_wstring(leftBankForwardCapacityN) + L"," +
                    std::to_wstring(rightBankForwardCapacityN) + L"," +
                    std::to_wstring(legacyLeftSummedCapacityN) + L"," +
                    std::to_wstring(legacyRightSummedCapacityN)).c_str());
            Assert::IsTrue(solution.tractionLimited);
            Assert::IsTrue(
                ((legacyLeftSummedCapacityN - leftBankForwardCapacityN) > 1.0e-4f) ||
                ((legacyRightSummedCapacityN - rightBankForwardCapacityN) > 1.0e-4f));
            Assert::AreEqual(expectedYawAccelRadps2, solution.commandedYawAccelRadps2, 1.0e-3f);
            Assert::AreEqual(solution.commandedYawAccelRadps2, repeatSolution.commandedYawAccelRadps2, 1.0e-6f);
            Assert::AreEqual(solution.control.leftMotorCommand, repeatSolution.control.leftMotorCommand, 1.0e-6f);
            Assert::AreEqual(solution.control.rightMotorCommand, repeatSolution.control.rightMotorCommand, 1.0e-6f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsDoesNotTractionLimitWellInsideNominalEnvelope)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float forwardVelocityMps = 2.0f;
            const float yawRateRadps = 4.0f;
            const float desiredLongitudinalAccelMps2 = 1.0f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsSupportsHistoricalThreeMeterPerSecondEnvelope)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            // Earlier pre-UKF testing reached about 3 m/s in roughly 50 mm of real travel. Keep the plant
            // drag/friction model from making that known envelope unreachable even though the current D: CSV
            // launch capture only reaches the lower post-UKF retrofit speed.
            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    3.0f,
                    1.0f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            Assert::IsFalse(solution.tractionLimited);
            Assert::IsTrue(solution.converged);
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) < 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) < 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsBeyondNominalReturnsClippedFiniteSolution)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(2.0f, params.combinedAccelNominalMps2 + 2.0f, 0.0f, 0.0f, params, 0.80f, params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);

            Assert::IsTrue(solution.tractionLimited);
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(achieved.longitudinalAccelMps2));
            Assert::IsTrue(std::fabs(achieved.longitudinalAccelMps2) <= (params.combinedAccelPeakMps2 + 1.0f));
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetTractionLimitsExplicitAggressiveStep)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float aggressiveResponseTimeS = 0.005f;
            const float requestedLongitudinalAccelMps2 =
                (0.20f - 0.0f) / aggressiveResponseTimeS;
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    0.0f,
                    0.20f,
                    0.0f,
                    0.0f,
                    params,
                    0.80f,
                    0.0f,
                    aggressiveResponseTimeS);

            Assert::IsTrue(solution.tractionLimited);
            Assert::IsTrue(solution.commandedLongitudinalAccelMps2 < requestedLongitudinalAccelMps2);
            Assert::IsTrue(std::fabs(solution.commandedLongitudinalAccelMps2) <= (params.combinedAccelPeakMps2 + 1.0f));
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) <= 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsBeyondPeakRemainsStable)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(2.0f, params.combinedAccelPeakMps2 + 5.0f, 0.0f, 0.0f, params, 0.80f, params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);

            Assert::IsTrue(solution.tractionLimited);
            Assert::IsTrue(std::isfinite(achieved.longitudinalAccelMps2));
            Assert::IsTrue(std::isfinite(achieved.yawAccelRadps2));
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) <= 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsNearZeroSpeedTurnRemainsFinite)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(0.005f, 0.0f, 0.0f, 20.0f, params, 0.80f, params.supplyVoltageV);

            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::isfinite(solution.leftWheelSpeedRadps));
            Assert::IsTrue(std::isfinite(solution.rightWheelSpeedRadps));
            Assert::IsTrue(std::isfinite(solution.leftWheelTorqueNm));
            Assert::IsTrue(std::isfinite(solution.rightWheelTorqueNm));
        }

        TEST_METHOD(PlantModelDriveCommandInverseMatchesForwardMotorModel)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float wheelSpeedRadps = 145.0f;
            const float desiredWheelTorqueNm = 0.0035f;

            const float command =
                plant.driveCommandFromTorque(
                    desiredWheelTorqueNm,
                    wheelSpeedRadps,
                    params.supplyVoltageV,
                    params);
            const float reconstructedTorqueNm =
                plant.driveTorqueFromCommand(
                    command,
                    wheelSpeedRadps,
                    params.supplyVoltageV,
                    params);

            Assert::IsTrue(std::isfinite(command));
            Assert::IsTrue(std::fabs(command) <= 1.0f);
            Assert::AreEqual(desiredWheelTorqueNm, reconstructedTorqueNm, 1.0e-4f);
        }
    };
}
