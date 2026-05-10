#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelTestSupport.h"
#include "SrUkfCoreTestSupport.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
            const App::Internal::CommandVector& control,
            int steps)
        {
            SrUkfCore core(prepared.raw, plant);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));
            SyntheticEncoderRemainderState syntheticEncoderState{};
            const float commandedLinearMps = initialState(VehicleState::kU);
            const float commandedAngularRadps = initialState(VehicleState::kR);
            for (int step = 0; step < steps; ++step)
            {
                RunPredictionMatchingCycle(
                    core,
                    control,
                    prepared.raw,
                    kPredictDtSeconds,
                    syntheticEncoderState,
                    commandedLinearMps,
                    commandedAngularRadps,
                    0.80f,
                    prepared.raw.supplyVoltageV);
            }

            return core.state();
        }

        DriveCommandSolution SolveStationaryYawOnlyVelocityTargetCommand(
            PlantModel& plant,
            const PlantParams& params,
            const float targetYawRateRadps = 3.0f)
        {
            return plant.solveDriveCommandsForVelocityTarget(
                BuildRollingUkfState(0.0f, 0.0f, params),
                0.0f,
                targetYawRateRadps,
                params,
                0.80f,
                params.supplyVoltageV,
                PlantModel::kDefaultVelocityTargetResponseTimeS);
        }

        PlantParams BuildPivotScrubTestParams() noexcept
        {
            PlantParams params = PlantParams::Default();
            params.pivotScrubBreakawayYawMomentNm = 0.12f;
            params.pivotScrubRollingYawMomentNm = 0.03f;
            params.pivotScrubMaxForwardSpeedMps = 0.03f;
            params.pivotScrubMinCommandYawRateRadps = 1.0f;
            params.pivotScrubBreakawayYawRateRadps = 2.0f;
            params.pivotScrubBreakawayYawRateBandRadps = 1.0f;
            return params;
        }

        float ComputeInPlaceTurnCommandSplitForYawAccelLimit(
            PlantModel& plant,
            const PlantParams& params,
            float targetYawRateRadps,
            float yawAccelLimitRadps2)
        {
            float desiredYawAccelRadps2 = 0.0f;
            plant.ComputeBodyActionFromYawRate(
                0.0f,
                0.0f,
                targetYawRateRadps,
                yawAccelLimitRadps2,
                PlantModel::kDefaultVelocityTargetResponseTimeS,
                desiredYawAccelRadps2);

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    0.0f,
                    0.0f,
                    0.0f,
                    desiredYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            return std::fabs(solution.control.LeftMotorPwm() - solution.control.RightMotorPwm());
        }

        void AssertCanonicalAllocationMatchesSpec(const DriveCommandSolution& solution)
        {
            const float differentialAbsMaxN =
                (std::min)(solution.leftForceLimitN, solution.rightForceLimitN);
            const float expectedDifferentialForceN =
                (std::clamp)(
                    solution.requestedDifferentialForceN,
                    -differentialAbsMaxN,
                    differentialAbsMaxN);
            const float expectedCommonForceMinN =
                (std::max)(
                    -solution.leftForceLimitN + expectedDifferentialForceN,
                    -solution.rightForceLimitN - expectedDifferentialForceN);
            const float expectedCommonForceMaxN =
                (std::min)(
                    solution.leftForceLimitN + expectedDifferentialForceN,
                    solution.rightForceLimitN - expectedDifferentialForceN);
            const float expectedCommonForceN =
                (expectedCommonForceMinN <= expectedCommonForceMaxN) ?
                (std::clamp)(
                    solution.requestedCommonForceN,
                    expectedCommonForceMinN,
                    expectedCommonForceMaxN) :
                0.0f;

            Assert::AreEqual(expectedDifferentialForceN, solution.commandedDifferentialForceN, 1.0e-5f);
            Assert::AreEqual(expectedCommonForceN, solution.commandedCommonForceN, 1.0e-5f);
            Assert::AreEqual(
                expectedCommonForceN - expectedDifferentialForceN,
                solution.leftContactForceN,
                1.0e-5f);
            Assert::AreEqual(
                expectedCommonForceN + expectedDifferentialForceN,
                solution.rightContactForceN,
                1.0e-5f);
        }

        float ComputeTargetArrivalTimeS(
            float currentValue,
            float targetValue,
            float appliedAcceleration) noexcept
        {
            if (std::fabs(appliedAcceleration) <= 1.0e-6f)
            {
                return (std::numeric_limits<float>::infinity)();
            }

            return std::fabs((targetValue - currentValue) / appliedAcceleration);
        }

        template <typename Solver>
        VehicleState::StateVector RunPlantPredictLoopWithResolvedFeedforward(
            const PlantModel& plant,
            const PlantModel::PreparedParams& prepared,
            const VehicleState::StateVector& initialState,
            float commandedLinearMps,
            float commandedAngularRadps,
            int steps,
            Solver&& solveFeedforward)
        {
            SrUkfCore core(prepared.raw, plant);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));
            SyntheticEncoderRemainderState syntheticEncoderState{};
            for (int step = 0; step < steps; ++step)
            {
                const DriveCommandSolution solution = solveFeedforward(core.state());
                RunPredictionMatchingCycle(
                    core,
                    solution.control,
                    prepared.raw,
                    kPredictDtSeconds,
                    syntheticEncoderState,
                    commandedLinearMps,
                    commandedAngularRadps,
                    solution.fanDutyCycle,
                    solution.batteryVoltageV);
            }

            return core.state();
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
                Assert::IsTrue(false, message.c_str());
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
                        targetForwardVelocityMps,
                        targetYawRateRadps,
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
            Assert::AreEqual(0.0f, solution.control.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.0f, solution.control.RightMotorPwm(), 1.0e-6f);
            Assert::AreEqual(0.0f, solution.leftWheelTorqueNm, 1.0e-6f);
            Assert::AreEqual(0.0f, solution.rightWheelTorqueNm, 1.0e-6f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetStationaryYawOnlyRequestProducesZeroAverageCommand)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                SolveStationaryYawOnlyVelocityTargetCommand(plant, params);
            const float averageCommand =
                0.5f * (solution.control.LeftMotorPwm() + solution.control.RightMotorPwm());

            Assert::AreEqual(0.0f, averageCommand, 1.0e-6f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetYawOnlyRepresentativeStatesMeetBreakawayFloor)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float breakawayDriveCommand =
                std::fabs(
                    plant.driveCommandFromTorque(
                        params.staticFrictionTorqueNm,
                        0.0f,
                        params.supplyVoltageV,
                        params));

            struct RepresentativeYawSample final
            {
                float forwardVelocityMps;
                float yawRateRadps;
                float leftWheelSpeedRadps;
                float rightWheelSpeedRadps;
                float targetYawRateRadps;
            };

            const RepresentativeYawSample samples[] = {
                // 2026-04-21 05:59:46, master_time_us=94152347
                { -0.037369605f, 1.82710910f, 11.6418180f, -34.9254532f, 5.22920513f },
                // 2026-04-21 05:59:46, master_time_us=94169347
                { -0.115910612f, 2.00860476f, 11.1761456f, -35.8567963f, 4.93123722f },
            };

            for (const RepresentativeYawSample& sample : samples)
            {
                PlantModel::StateVector currentState = PlantModel::StateVector::Zero();
                currentState(VehicleState::kU) = sample.forwardVelocityMps;
                currentState(VehicleState::kR) = sample.yawRateRadps;
                currentState(VehicleState::kOmegaL) = sample.leftWheelSpeedRadps;
                currentState(VehicleState::kOmegaR) = sample.rightWheelSpeedRadps;

                const DriveCommandSolution solution =
                    plant.solveDriveCommandsForVelocityTarget(
                        currentState,
                        0.0f,
                        sample.targetYawRateRadps,
                        params,
                        0.80f,
                        params.supplyVoltageV,
                        PlantModel::kDefaultVelocityTargetResponseTimeS);
                const float weakerBankCommand =
                    (std::min)(
                        std::fabs(solution.control.LeftMotorPwm()),
                        std::fabs(solution.control.RightMotorPwm()));

                Assert::IsFalse(solution.tractionLimited);
                Assert::IsTrue(solution.control.LeftMotorPwm() * solution.control.RightMotorPwm() < 0.0f);
                Assert::IsTrue(weakerBankCommand >= (breakawayDriveCommand - 1.0e-3f));
            }
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
            const PlantDerivatives achieved =
                plant.forwardStep(state, solution.control, solution.fanDutyCycle, solution.batteryVoltageV, params);
            const float expectedLeftWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 + (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;
            const float expectedRightWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 - (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;

            std::wstring failure;
            if (!std::isfinite(solution.control.LeftMotorPwm()) ||
                !std::isfinite(solution.control.RightMotorPwm()))
            {
                failure =
                    std::wstring(L"non-finite motor command left/right=") +
                    std::to_wstring(solution.control.LeftMotorPwm()) + L"," +
                    std::to_wstring(solution.control.RightMotorPwm());
            }
            else if (!std::isfinite(achieved.longitudinalAccelMps2) || !std::isfinite(achieved.yawAccelRadps2))
            {
                failure =
                    std::wstring(L"non-finite operating-point accel Udd/Rdd=") +
                    std::to_wstring(achieved.longitudinalAccelMps2) + L"," +
                    std::to_wstring(achieved.yawAccelRadps2);
            }
            else if (std::fabs(solution.commandedLongitudinalAccelMps2 - desiredLongitudinalAccelMps2) > 0.05f)
            {
                failure =
                    std::wstring(L"reported longitudinal accel mismatch desired/reported=") +
                    std::to_wstring(desiredLongitudinalAccelMps2) + L"," +
                    std::to_wstring(solution.commandedLongitudinalAccelMps2);
            }
            else if (std::fabs(solution.commandedYawAccelRadps2 - desiredYawAccelRadps2) > 0.20f)
            {
                failure =
                    std::wstring(L"reported yaw accel mismatch desired/reported=") +
                    std::to_wstring(desiredYawAccelRadps2) + L"," +
                    std::to_wstring(solution.commandedYawAccelRadps2);
            }
            else if (std::fabs(achieved.longitudinalAccelMps2 - solution.commandedLongitudinalAccelMps2) >
                RelativeTolerance(solution.commandedLongitudinalAccelMps2, 0.10f))
            {
                failure =
                    std::wstring(L"forwardStep longitudinal accel mismatch reported/achieved=") +
                    std::to_wstring(solution.commandedLongitudinalAccelMps2) + L"," +
                    std::to_wstring(achieved.longitudinalAccelMps2);
            }
            else if (std::fabs(achieved.yawAccelRadps2 - solution.commandedYawAccelRadps2) >
                RelativeTolerance(solution.commandedYawAccelRadps2, 0.20f))
            {
                failure =
                    std::wstring(L"forwardStep yaw accel mismatch reported/achieved=") +
                    std::to_wstring(solution.commandedYawAccelRadps2) + L"," +
                    std::to_wstring(achieved.yawAccelRadps2) +
                    std::wstring(L" wheelSpeedL/R=") +
                    std::to_wstring(solution.leftWheelSpeedRadps) + L"," +
                    std::to_wstring(solution.rightWheelSpeedRadps) +
                    std::wstring(L" contactForceL/R=") +
                    std::to_wstring(solution.leftContactForceN) + L"," +
                    std::to_wstring(solution.rightContactForceN) +
                    std::wstring(L" achievedBankForceL/R=") +
                    std::to_wstring(achieved.contactForces.LeftBankForwardForceN()) + L"," +
                    std::to_wstring(achieved.contactForces.RightBankForwardForceN()) +
                    std::wstring(L" achievedFrontRearRight=") +
                    std::to_wstring(
                        achieved.contactForces.contacts[0].rightForceN +
                        achieved.contactForces.contacts[1].rightForceN) +
                    L"," +
                    std::to_wstring(
                        achieved.contactForces.contacts[2].rightForceN +
                        achieved.contactForces.contacts[3].rightForceN);
            }
            else if (std::fabs(solution.leftWheelAccelRadps2 - expectedLeftWheelAccelRadps2) > 1.0e-3f)
            {
                failure =
                    std::wstring(L"left wheel accel mismatch expected/reported=") +
                    std::to_wstring(expectedLeftWheelAccelRadps2) + L"," +
                    std::to_wstring(solution.leftWheelAccelRadps2);
            }
            else if (std::fabs(solution.rightWheelAccelRadps2 - expectedRightWheelAccelRadps2) > 1.0e-3f)
            {
                failure =
                    std::wstring(L"right wheel accel mismatch expected/reported=") +
                    std::to_wstring(expectedRightWheelAccelRadps2) + L"," +
                    std::to_wstring(solution.rightWheelAccelRadps2);
            }
            else if (!std::isfinite(achieved.stateDot(VehicleState::kOmegaL)) ||
                !std::isfinite(achieved.stateDot(VehicleState::kOmegaR)))
            {
                failure =
                    std::wstring(L"non-finite wheel acceleration stateDot omegaL/omegaR=") +
                    std::to_wstring(achieved.stateDot(VehicleState::kOmegaL)) + L"," +
                    std::to_wstring(achieved.stateDot(VehicleState::kOmegaR));
            }

            if (!failure.empty())
            {
                Assert::IsTrue(failure.empty(), failure.c_str());
            }
        }

        TEST_METHOD(PlantModelComputeBodyActionUsesLongitudinalLimitForPureSpeedChange)
        {
            PlantModel plant;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.ComputeBodyAction(
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

        TEST_METHOD(PlantModelComputeBodyActionFromYawRateUsesYawAccelLimitWhenRelevant)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float tightYawAccelLimitRadps2 = 120.0f;
            constexpr float looseYawAccelLimitRadps2 = 240.0f;
            const float targetYawRateRadps =
                2.0f * looseYawAccelLimitRadps2 * PlantModel::kDefaultVelocityTargetResponseTimeS;
            const float tightCommandSplit =
                ComputeInPlaceTurnCommandSplitForYawAccelLimit(
                    plant,
                    params,
                    targetYawRateRadps,
                    tightYawAccelLimitRadps2);
            const float looseCommandSplit =
                ComputeInPlaceTurnCommandSplitForYawAccelLimit(
                    plant,
                    params,
                    targetYawRateRadps,
                    looseYawAccelLimitRadps2);
            const std::wstring message =
                std::wstring(L"tight split=") + std::to_wstring(tightCommandSplit) +
                L" loose split=" + std::to_wstring(looseCommandSplit) +
                L" target_yaw_rate=" + std::to_wstring(targetYawRateRadps);

            Assert::IsTrue(looseCommandSplit > (tightCommandSplit + 1.0e-4f), message.c_str());
        }

        TEST_METHOD(PlantModelComputeBodyActionCombinedTargetsShareArrivalTime)
        {
            PlantModel plant;
            constexpr float currentForwardVelocityMps = 0.0f;
            constexpr float currentYawRateRadps = 0.0f;
            constexpr float longitudinalAccelLimitMps2 = 9.0f;
            constexpr float yawAccelLimitRadps2 = 400.0f;
            constexpr float responseTimeS = PlantModel::kDefaultVelocityTargetResponseTimeS;
            const float targetForwardVelocityMps = 1.2f * longitudinalAccelLimitMps2 * responseTimeS;
            const float targetYawRateRadps = 0.8f * yawAccelLimitRadps2 * responseTimeS;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.ComputeBodyAction(
                currentForwardVelocityMps,
                targetForwardVelocityMps,
                currentYawRateRadps,
                targetYawRateRadps,
                longitudinalAccelLimitMps2,
                yawAccelLimitRadps2,
                responseTimeS,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);
            const float forwardArrivalTimeS =
                ComputeTargetArrivalTimeS(
                    currentForwardVelocityMps,
                    targetForwardVelocityMps,
                    desiredLongitudinalAccelMps2);
            const float yawArrivalTimeS =
                ComputeTargetArrivalTimeS(
                    currentYawRateRadps,
                    targetYawRateRadps,
                    desiredYawAccelRadps2);
            const std::wstring message =
                std::wstring(L"forward_arrival_s=") + std::to_wstring(forwardArrivalTimeS) +
                L" yaw_arrival_s=" + std::to_wstring(yawArrivalTimeS) +
                L" long_accel=" + std::to_wstring(desiredLongitudinalAccelMps2) +
                L" yaw_accel=" + std::to_wstring(desiredYawAccelRadps2);

            Assert::IsTrue(std::fabs(forwardArrivalTimeS - yawArrivalTimeS) <= 1.0e-5f, message.c_str());
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
            Assert::AreEqual(explicitDefaultSolution.control.LeftMotorPwm(), solution.control.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(explicitDefaultSolution.control.RightMotorPwm(), solution.control.RightMotorPwm(), 1.0e-6f);
            Assert::IsTrue(std::fabs(solution.control.LeftMotorPwm()) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.RightMotorPwm()) <= 1.0f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetReportsReturnedControlPredictionAtOperatingPoint)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector operatingState =
                BuildRollingUkfState(0.12f, 0.25f, params);
            const DriveCommandSolution solution =
                plant.solveDriveCommandsForVelocityTarget(
                    operatingState,
                    0.20f,
                    0.60f,
                    params,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);
            VehicleState::StateVector validationState = operatingState;
            validationState(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            validationState(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;
            const PlantDerivatives achieved =
                plant.forwardStep(
                    validationState,
                    solution.control,
                    solution.fanDutyCycle,
                    solution.batteryVoltageV,
                    params);

            std::wstring failure;
            if (solution.tractionLimited)
            {
                failure = L"velocity-target operating-point prediction unexpectedly reported tractionLimited=true";
            }
            else if (!std::isfinite(achieved.longitudinalAccelMps2) || !std::isfinite(achieved.yawAccelRadps2))
            {
                failure =
                    std::wstring(L"non-finite operating-point accel Udd/Rdd=") +
                    std::to_wstring(achieved.longitudinalAccelMps2) + L"," +
                    std::to_wstring(achieved.yawAccelRadps2);
            }
            else if (std::fabs(achieved.longitudinalAccelMps2 - solution.commandedLongitudinalAccelMps2) >
                RelativeTolerance(solution.commandedLongitudinalAccelMps2, 0.05f))
            {
                failure =
                    std::wstring(L"velocity-target longitudinal prediction mismatch reported/forwardStep=") +
                    std::to_wstring(solution.commandedLongitudinalAccelMps2) + L"," +
                    std::to_wstring(achieved.longitudinalAccelMps2);
            }
            else if (std::fabs(achieved.yawAccelRadps2 - solution.commandedYawAccelRadps2) >
                RelativeTolerance(solution.commandedYawAccelRadps2, 0.10f))
            {
                failure =
                    std::wstring(L"velocity-target yaw prediction mismatch reported/forwardStep=") +
                    std::to_wstring(solution.commandedYawAccelRadps2) + L"," +
                    std::to_wstring(achieved.yawAccelRadps2);
            }

            if (!failure.empty())
            {
                Assert::IsTrue(failure.empty(), failure.c_str());
            }
        }

        TEST_METHOD(PlantModelClosedLoopFeedforwardUsesExactDifferentialFirstAllocator)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state = BuildRollingUkfState(0.35f, 0.50f, params);
            const float reserveUsage = 0.25f;

            const DriveCommandSolution solution =
                plant.solveClosedLoopDriveCommands(
                    state,
                    50.0f,
                    200.0f,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    reserveUsage);

            Assert::IsTrue(std::isfinite(solution.commandedCommonForceN));
            Assert::IsTrue(std::isfinite(solution.commandedDifferentialForceN));
            AssertCanonicalAllocationMatchesSpec(solution);
        }

        TEST_METHOD(PlantModelTractionLimitedWrapperMatchesCanonicalClosedLoopReserveSemantics)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state = BuildRollingUkfState(0.35f, 0.50f, params);
            const float reserveUsage = 0.35f;

            const DriveCommandSolution canonical =
                plant.solveClosedLoopDriveCommands(
                    state,
                    12.0f,
                    65.0f,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    reserveUsage);
            const DriveCommandSolution legacyWrapper =
                plant.solveTractionLimitedDriveCommands(
                    state,
                    12.0f,
                    65.0f,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    reserveUsage);

            AssertDriveCommandSolutionNear(canonical, legacyWrapper, 1.0e-6f);
        }

        TEST_METHOD(PlantModelVelocityTargetWrappersStayThinOutsidePivotScrubRegime)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state = BuildRollingUkfState(0.28f, 0.42f, params);
            const float targetForwardVelocityMps = 0.52f;
            const float targetYawRateRadps = 0.83f;
            const float responseTimeS = PlantModel::kDefaultVelocityTargetResponseTimeS;
            float expectedLongitudinalAccelMps2 = 0.0f;
            float expectedYawAccelRadps2 = 0.0f;

            plant.ComputeBodyAction(
                state(VehicleState::kU),
                targetForwardVelocityMps,
                state(VehicleState::kR),
                targetYawRateRadps,
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)(),
                responseTimeS,
                expectedLongitudinalAccelMps2,
                expectedYawAccelRadps2);

            const DriveCommandSolution expectedOpenLoop =
                plant.solveDriveCommands(
                    state,
                    expectedLongitudinalAccelMps2,
                    expectedYawAccelRadps2,
                    prepared,
                    0.80f,
                    params.supplyVoltageV);
            const DriveCommandSolution actualOpenLoop =
                plant.solveDriveCommandsForVelocityTarget(
                    state,
                    targetForwardVelocityMps,
                    targetYawRateRadps,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    responseTimeS);
            AssertDriveCommandSolutionNear(expectedOpenLoop, actualOpenLoop, 1.0e-6f);

            const float reserveUsage = 0.40f;
            const DriveCommandSolution expectedClosedLoop =
                plant.solveClosedLoopDriveCommands(
                    state,
                    expectedLongitudinalAccelMps2,
                    expectedYawAccelRadps2,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    reserveUsage);
            const DriveCommandSolution actualClosedLoop =
                plant.solveClosedLoopDriveCommandsForVelocityTarget(
                    state,
                    targetForwardVelocityMps,
                    targetYawRateRadps,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    responseTimeS,
                    reserveUsage);
            AssertDriveCommandSolutionNear(expectedClosedLoop, actualClosedLoop, 1.0e-6f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsForVelocityTargetReducesPivotScrubAfterYawBreakaway)
        {
            PlantModel plant;
            const PlantParams params = BuildPivotScrubTestParams();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const float targetYawRateRadps = 8.0f;

            const auto measureAddedDifferentialForceN =
                [&](const VehicleState::StateVector& state)
                {
                    float desiredLongitudinalAccelMps2 = 0.0f;
                    float desiredYawAccelRadps2 = 0.0f;
                    plant.ComputeBodyAction(
                        state(VehicleState::kU),
                        0.0f,
                        state(VehicleState::kR),
                        targetYawRateRadps,
                        (std::numeric_limits<float>::max)(),
                        (std::numeric_limits<float>::max)(),
                        PlantModel::kDefaultVelocityTargetResponseTimeS,
                        desiredLongitudinalAccelMps2,
                        desiredYawAccelRadps2);

                    const DriveCommandSolution baseline =
                        plant.solveDriveCommands(
                            state,
                            desiredLongitudinalAccelMps2,
                            desiredYawAccelRadps2,
                            prepared,
                            0.80f,
                            params.supplyVoltageV);
                    const DriveCommandSolution actual =
                        plant.solveDriveCommandsForVelocityTarget(
                            state,
                            0.0f,
                            targetYawRateRadps,
                            prepared,
                            0.80f,
                            params.supplyVoltageV,
                            PlantModel::kDefaultVelocityTargetResponseTimeS);
                    return std::fabs(actual.requestedDifferentialForceN - baseline.requestedDifferentialForceN);
                };

            const VehicleState::StateVector stuckState = BuildRollingUkfState(0.0f, 0.0f, params);
            const VehicleState::StateVector rollingState =
                BuildRollingUkfState(
                    0.0f,
                    params.pivotScrubBreakawayYawRateRadps + params.pivotScrubBreakawayYawRateBandRadps + 1.0f,
                    params);

            const float stuckAddedDifferentialForceN = measureAddedDifferentialForceN(stuckState);
            const float rollingAddedDifferentialForceN = measureAddedDifferentialForceN(rollingState);

            Assert::IsTrue(stuckAddedDifferentialForceN > (rollingAddedDifferentialForceN + 0.5f));
        }

        TEST_METHOD(PlantModelVelocityTargetPivotScrubStaysOffForForwardDominantCommand)
        {
            PlantModel plant;
            const PlantParams params = BuildPivotScrubTestParams();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state = BuildRollingUkfState(0.0f, 0.0f, params);
            constexpr float targetForwardVelocityMps = 0.20f;
            constexpr float targetYawRateRadps = 2.0f;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.ComputeBodyAction(
                state(VehicleState::kU),
                targetForwardVelocityMps,
                state(VehicleState::kR),
                targetYawRateRadps,
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)(),
                PlantModel::kDefaultVelocityTargetResponseTimeS,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);

            const DriveCommandSolution expected =
                plant.solveDriveCommands(
                    state,
                    desiredLongitudinalAccelMps2,
                    desiredYawAccelRadps2,
                    prepared,
                    0.80f,
                    params.supplyVoltageV);
            const DriveCommandSolution actual =
                plant.solveDriveCommandsForVelocityTarget(
                    state,
                    targetForwardVelocityMps,
                    targetYawRateRadps,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            AssertDriveCommandSolutionNear(expected, actual, 1.0e-6f);
        }

        TEST_METHOD(PlantModelVelocityTargetPivotScrubAppliesForNonZeroForwardVelocity)
        {
            PlantModel plant;
            PlantParams params = BuildPivotScrubTestParams();
            params.pivotScrubBreakawayYawMomentNm = 0.11f;
            params.pivotScrubRollingYawMomentNm = 0.11f;
            params.pivotScrubMaxForwardSpeedMps = 0.12f;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state = BuildRollingUkfState(0.10f, 0.0f, params);
            constexpr float targetForwardVelocityMps = 0.10f;
            constexpr float targetYawRateRadps = 6.0f;
            float desiredLongitudinalAccelMps2 = 0.0f;
            float desiredYawAccelRadps2 = 0.0f;

            plant.ComputeBodyAction(
                state(VehicleState::kU),
                targetForwardVelocityMps,
                state(VehicleState::kR),
                targetYawRateRadps,
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)(),
                PlantModel::kDefaultVelocityTargetResponseTimeS,
                desiredLongitudinalAccelMps2,
                desiredYawAccelRadps2);

            const DriveCommandSolution baseline =
                plant.solveDriveCommands(
                    state,
                    desiredLongitudinalAccelMps2,
                    desiredYawAccelRadps2,
                    prepared,
                    0.80f,
                    params.supplyVoltageV);
            const DriveCommandSolution actual =
                plant.solveDriveCommandsForVelocityTarget(
                    state,
                    targetForwardVelocityMps,
                    targetYawRateRadps,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    PlantModel::kDefaultVelocityTargetResponseTimeS);

            Assert::IsTrue(
                std::fabs(actual.requestedDifferentialForceN - baseline.requestedDifferentialForceN) > 0.5f);
        }

        TEST_METHOD(PlantModelEvaluateFeedforwardOfflineReportsFiniteOutputs)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state = BuildRollingUkfState(0.22f, 0.31f, params);

            const FeedforwardAuditResult audit =
                plant.evaluateFeedforwardOffline(
                    state,
                    8.5f,
                    35.0f,
                    prepared,
                    0.80f,
                    params.supplyVoltageV,
                    0.75f,
                    0.001f);

            Assert::IsTrue(audit.issued.valid);
            Assert::IsTrue(std::isfinite(audit.predictedForwardAccelMps2));
            Assert::IsTrue(std::isfinite(audit.predictedYawAccelRadps2));
            Assert::IsTrue(std::isfinite(audit.forwardAccelResidualMps2));
            Assert::IsTrue(std::isfinite(audit.yawAccelResidualRadps2));
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
            const PlantDerivatives achievedFull =
                plant.forwardStep(
                    validationState,
                    solution.control,
                    solution.fanDutyCycle,
                    solution.batteryVoltageV,
                    params);

            PlantModel::StateVector zeroLateralValidationState = validationState;
            zeroLateralValidationState(VehicleState::kV) = 0.0f;
            const PlantDerivatives achievedZeroLateral =
                plant.forwardStep(
                    zeroLateralValidationState,
                    solution.control,
                    solution.fanDutyCycle,
                    solution.batteryVoltageV,
                    params);

            std::wstring failure;
            if (!std::isfinite(solution.commandedYawAccelRadps2))
            {
                failure =
                    std::wstring(L"reported yaw accel was not finite: ") +
                    std::to_wstring(solution.commandedYawAccelRadps2);
            }
            else if (!std::isfinite(achievedFull.yawAccelRadps2) ||
                !std::isfinite(achievedZeroLateral.yawAccelRadps2))
            {
                failure =
                    std::wstring(L"validation yaw accel was not finite full/zero-lateral=") +
                    std::to_wstring(achievedFull.yawAccelRadps2) + L"," +
                    std::to_wstring(achievedZeroLateral.yawAccelRadps2);
            }
            else if (std::fabs(achievedFull.yawAccelRadps2 - achievedZeroLateral.yawAccelRadps2) <= 0.01f)
            {
                failure =
                    std::wstring(L"validation state lost lateral-velocity effect full/zero-lateral yaw accel=") +
                    std::to_wstring(achievedFull.yawAccelRadps2) + L"," +
                    std::to_wstring(achievedZeroLateral.yawAccelRadps2);
            }

            if (!failure.empty())
            {
                Assert::IsTrue(failure.empty(), failure.c_str());
            }
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
            const PlantDerivatives achieved =
                plant.forwardStep(
                    validationState,
                    solution.control,
                    solution.fanDutyCycle,
                    solution.batteryVoltageV,
                    params);
            const float expectedLeftWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 + (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;
            const float expectedRightWheelAccelRadps2 =
                (solution.commandedLongitudinalAccelMps2 - (0.5f * params.trackWidthM * solution.commandedYawAccelRadps2)) / params.wheelRadiusM;

            std::wstring failure;
            if (solution.tractionLimited)
            {
                failure = L"yaw damping compensation unexpectedly reported tractionLimited=true";
            }
            else if (std::fabs(solution.commandedYawAccelRadps2 - desiredYawAccelRadps2) > 0.05f)
            {
                failure =
                    std::wstring(L"reported yaw accel mismatch desired/reported=") +
                    std::to_wstring(desiredYawAccelRadps2) + L"," +
                    std::to_wstring(solution.commandedYawAccelRadps2);
            }
            else if (std::fabs(solution.leftWheelAccelRadps2 - expectedLeftWheelAccelRadps2) > 1.0e-3f)
            {
                failure =
                    std::wstring(L"left wheel accel mismatch expected/reported=") +
                    std::to_wstring(expectedLeftWheelAccelRadps2) + L"," +
                    std::to_wstring(solution.leftWheelAccelRadps2);
            }
            else if (std::fabs(solution.rightWheelAccelRadps2 - expectedRightWheelAccelRadps2) > 1.0e-3f)
            {
                failure =
                    std::wstring(L"right wheel accel mismatch expected/reported=") +
                    std::to_wstring(expectedRightWheelAccelRadps2) + L"," +
                    std::to_wstring(solution.rightWheelAccelRadps2);
            }
            else if (!std::isfinite(achieved.yawAccelRadps2))
            {
                failure =
                    std::wstring(L"forwardStep yaw accel was not finite: ") +
                    std::to_wstring(achieved.yawAccelRadps2);
            }
            else if (std::fabs(achieved.yawAccelRadps2 - solution.commandedYawAccelRadps2) >
                RelativeTolerance(solution.commandedYawAccelRadps2, 0.20f))
            {
                failure =
                    std::wstring(L"forwardStep yaw accel mismatch reported/achieved=") +
                    std::to_wstring(solution.commandedYawAccelRadps2) + L"," +
                    std::to_wstring(achieved.yawAccelRadps2) +
                    std::wstring(L" wheelSpeedL/R=") +
                    std::to_wstring(solution.leftWheelSpeedRadps) + L"," +
                    std::to_wstring(solution.rightWheelSpeedRadps) +
                    std::wstring(L" contactForceL/R=") +
                    std::to_wstring(solution.leftContactForceN) + L"," +
                    std::to_wstring(solution.rightContactForceN) +
                    std::wstring(L" achievedBankForceL/R=") +
                    std::to_wstring(achieved.contactForces.LeftBankForwardForceN()) + L"," +
                    std::to_wstring(achieved.contactForces.RightBankForwardForceN()) +
                    std::wstring(L" achievedFrontRearRight=") +
                    std::to_wstring(
                        achieved.contactForces.contacts[0].rightForceN +
                        achieved.contactForces.contacts[1].rightForceN) +
                    L"," +
                    std::to_wstring(
                        achieved.contactForces.contacts[2].rightForceN +
                        achieved.contactForces.contacts[3].rightForceN);
            }

            if (!failure.empty())
            {
                Assert::IsTrue(failure.empty(), failure.c_str());
            }
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

            App::Internal::CommandVector baselineControl{};
            const float baselineControlFanDutyCycle = fanDutyCycle;
            const ContactForces baselineForces =
                plant.tireForces(
                    currentState,
                    baselineControl,
                    baselineControlFanDutyCycle,
                    params);
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
            Assert::AreEqual(solution.control.LeftMotorPwm(), repeatSolution.control.LeftMotorPwm(), 1.0e-6f);
            Assert::AreEqual(solution.control.RightMotorPwm(), repeatSolution.control.RightMotorPwm(), 1.0e-6f);
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
            Assert::IsTrue(std::isfinite(solution.control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(solution.control.RightMotorPwm()));
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
            Assert::IsTrue(std::fabs(solution.control.LeftMotorPwm()) < 1.0f);
            Assert::IsTrue(std::fabs(solution.control.RightMotorPwm()) < 1.0f);
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
            const PlantDerivatives achieved =
                plant.forwardStep(state, solution.control, solution.fanDutyCycle, solution.batteryVoltageV, params);

            std::wstring failure;
            if (!solution.tractionLimited)
            {
                failure = L"beyond-peak request should be traction-limited but reported tractionLimited=false";
            }
            else if (!std::isfinite(achieved.longitudinalAccelMps2) || !std::isfinite(achieved.yawAccelRadps2))
            {
                failure =
                    std::wstring(L"forwardStep accel was not finite Udd/Rdd=") +
                    std::to_wstring(achieved.longitudinalAccelMps2) + L"," +
                    std::to_wstring(achieved.yawAccelRadps2);
            }
            else if (std::fabs(solution.control.LeftMotorPwm()) > 1.0f ||
                std::fabs(solution.control.RightMotorPwm()) > 1.0f)
            {
                failure =
                    std::wstring(L"motor command exceeded unit bounds left/right=") +
                    std::to_wstring(solution.control.LeftMotorPwm()) + L"," +
                    std::to_wstring(solution.control.RightMotorPwm());
            }

            if (!failure.empty())
            {
                Assert::IsTrue(failure.empty(), failure.c_str());
            }
        }

        TEST_METHOD(PlantModelSolveDriveCommandsNearZeroSpeedTurnRemainsFinite)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const DriveCommandSolution solution =
                plant.solveDriveCommands(0.005f, 0.0f, 0.0f, 20.0f, params, 0.80f, params.supplyVoltageV);

            Assert::IsTrue(std::isfinite(solution.control.LeftMotorPwm()));
            Assert::IsTrue(std::isfinite(solution.control.RightMotorPwm()));
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




