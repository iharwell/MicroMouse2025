#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\CoreConfig.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using CommandVector = MazeMap::App::Internal::CommandVector;

        constexpr float kNaN = (std::numeric_limits<float>::quiet_NaN)();
        constexpr float kInf = (std::numeric_limits<float>::infinity)();
        constexpr float kDtSeconds = 0.004f;

        VehicleState::StateVector CaptureRuntimeState(const VehicleState& runtimeState)
        {
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kPx) = runtimeState.GetPositionX();
            state(VehicleState::kPy) = runtimeState.GetPositionY();
            state(VehicleState::kPsi) = runtimeState.GetOrientation();
            state(VehicleState::kU) = runtimeState.GetVelocity();
            state(VehicleState::kV) = runtimeState.GetLateralVelocity();
            state(VehicleState::kR) = runtimeState.GetRotationalVelocity();
            state(VehicleState::kOmegaL) = runtimeState.GetWheelSpeedLeft();
            state(VehicleState::kOmegaR) = runtimeState.GetWheelSpeedRight();
            state(VehicleState::kBgz) = runtimeState.GetGyroBiasZ();
            VehicleState::NormalizeStateVector(state);
            return state;
        }

        void ApplyStateVectorToRuntime(VehicleState& runtimeState, const VehicleState::StateVector& state)
        {
            runtimeState.SetPosition(Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)));
            runtimeState.SetOrientation(state(VehicleState::kPsi));
            runtimeState.SetVelocity(state(VehicleState::kU));
            runtimeState.SetLateralVelocity(state(VehicleState::kV));
            runtimeState.SetRotationalVelocity(state(VehicleState::kR));
            runtimeState.SetWheelSpeedLeft(state(VehicleState::kOmegaL));
            runtimeState.SetWheelSpeedRight(state(VehicleState::kOmegaR));
            runtimeState.SetGyroBiasZ(state(VehicleState::kBgz));
        }

        void SetRollingWheelState(Vehicle& vehicle, VehicleState& runtimeState)
        {
            float leftOmegaRadps = 0.0f;
            float rightOmegaRadps = 0.0f;
            vehicle.WheelOmegasFromBodyVelocity(
                runtimeState.GetVelocity(),
                runtimeState.GetRotationalVelocity(),
                leftOmegaRadps,
                rightOmegaRadps);
            runtimeState.SetWheelSpeedLeft(leftOmegaRadps);
            runtimeState.SetWheelSpeedRight(rightOmegaRadps);
        }

        bool IsFlagSet(const std::uint16_t flags, const std::uint16_t flag) noexcept
        {
            return (flags & flag) != 0U;
        }

        struct DriveBasePlantHarness final
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant;
            DriveBase drive;
            PlantModel::PreparedParams prepared;
            VehicleState::StateVector truthState;

            DriveBasePlantHarness() noexcept
                : vehicle()
                , runtimeState()
                , plant(vehicle, runtimeState)
                , drive(plant, runtimeState, MazeMap::Config::kDriveBasePDCluster)
                , prepared(PlantModel::Prepare(PlantParams::Default()))
                , truthState(VehicleState::StateVector::Zero())
            {
                vehicle.SetFanDuty(0.80f);
                truthState = CaptureRuntimeState(runtimeState);
            }

            CommandVector ProposeAndIntegrate(
                float targetForwardMps,
                float targetYawRateRadps,
                float targetForwardAccelMps2,
                float targetYawAccelRadps2,
                float targetYawRad)
            {
                const CommandVector command =
                    drive.ProposeBodyTick(
                        targetForwardMps,
                        targetYawRateRadps,
                        targetForwardAccelMps2,
                        targetYawAccelRadps2,
                        targetYawRad);
                truthState = plant.integrate(truthState, command, kDtSeconds, prepared);
                ApplyStateVectorToRuntime(runtimeState, truthState);
                runtimeState.SetCurrentCommand(command);
                runtimeState.SetTime(runtimeState.GetTime() + kDtSeconds);
                return command;
            }
        };

        struct BoundarySnapshotScenario final
        {
            CommandVector command{};
            DriveTelemetry telemetry{};
            CommandVector expectedPlantCommand{};

            BoundarySnapshotScenario()
            {
                DriveBasePlantHarness harness;
                harness.runtimeState.SetPosition(Eigen::Vector2f(0.012f, -0.034f));
                harness.runtimeState.SetOrientation(0.21f);
                harness.runtimeState.SetVelocity(0.37f);
                harness.runtimeState.SetLateralVelocity(-0.025f);
                harness.runtimeState.SetRotationalVelocity(0.18f);
                SetRollingWheelState(harness.vehicle, harness.runtimeState);

                command = harness.drive.ProposeBodyTick(0.55f, 0.25f, 1.10f, 2.50f, 0.40f);
                telemetry = harness.drive.LastTelemetry();
                expectedPlantCommand =
                    harness.plant.ComputeFeedforward(
                        telemetry.composedForwardAccelMps2,
                        telemetry.composedYawAccelRadps2);
            }
        };

        struct FeedbackScenario final
        {
            DriveTelemetry telemetry{};

            FeedbackScenario()
            {
                DriveBasePlantHarness harness;
                harness.runtimeState.SetVelocity(0.20f);
                harness.runtimeState.SetRotationalVelocity(-0.15f);
                harness.runtimeState.SetOrientation(0.10f);
                SetRollingWheelState(harness.vehicle, harness.runtimeState);

                (void)harness.drive.ProposeBodyTick(0.80f, 0.25f, 0.30f, 0.40f, 0.18f);
                telemetry = harness.drive.LastTelemetry();
            }
        };

        struct ClampScenario final
        {
            CommandVector command{};
            DriveTelemetry telemetry{};

            ClampScenario()
            {
                DriveBasePlantHarness harness;
                command = harness.drive.ProposeBodyTick(kNaN, kNaN, 1000000.0f, 0.0f, kNaN);
                telemetry = harness.drive.LastTelemetry();
            }
        };

        struct SolverFailureScenario final
        {
            CommandVector command{};
            DriveTelemetry telemetry{};

            SolverFailureScenario()
            {
                DriveBasePlantHarness harness;
                command = harness.drive.ProposeBodyTick(kNaN, kNaN, 0.0f, 0.0f, kInf);
                telemetry = harness.drive.LastTelemetry();
            }
        };

        struct AccelerationLongRunScenario final
        {
            static constexpr int kAccelerationTicks = 80;
            static constexpr int kMinimumDiagnosticTicks = 20;

            bool commandsFinite = true;
            bool leftCommandsClamped = true;
            bool rightCommandsClamped = true;
            bool commandEvidenceValid = true;
            bool accelerationObjectiveUnchanged = true;
            float initialVelocityMps = 0.0f;
            float velocityAtMinimumHorizonMps = 0.0f;
            float finalVelocityMps = 0.0f;
            float minimumVelocityMps = 0.0f;

            AccelerationLongRunScenario()
            {
                DriveBasePlantHarness harness;
                initialVelocityMps = harness.runtimeState.GetVelocity();
                velocityAtMinimumHorizonMps = initialVelocityMps;
                minimumVelocityMps = initialVelocityMps;

                for (int tick = 0; tick < kAccelerationTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kNaN, 0.0f, 2.0f, 0.0f, kNaN);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();

                    commandsFinite = commandsFinite && command.IsFinite();
                    leftCommandsClamped = leftCommandsClamped && std::fabs(command.LeftCommand()) <= 1.0f;
                    rightCommandsClamped = rightCommandsClamped && std::fabs(command.RightCommand()) <= 1.0f;
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    accelerationObjectiveUnchanged =
                        accelerationObjectiveUnchanged &&
                        std::fabs(telemetry.composedForwardAccelMps2 - 2.0f) <= 1.0e-6f;

                    minimumVelocityMps = (std::min)(minimumVelocityMps, harness.runtimeState.GetVelocity());
                    if (tick + 1 == kMinimumDiagnosticTicks)
                    {
                        velocityAtMinimumHorizonMps = harness.runtimeState.GetVelocity();
                    }
                }

                finalVelocityMps = harness.runtimeState.GetVelocity();
            }
        };

        struct VelocityLongRunScenario final
        {
            static constexpr int kVelocityTicks = 1500;
            static constexpr float kTargetForwardMps = 0.5f;

            bool commandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedForwardPreserved = true;
            bool requestedYawRatePreserved = true;
            float minimumVelocityMps = 0.0f;
            float maximumVelocityMps = 0.0f;
            float finalVelocityMps = 0.0f;
            float finalRequestedVelMps = 0.0f;
            float finalRequestedAccelMps2 = 0.0f;
            VelocityLongRunScenario()
            {
                DriveBasePlantHarness harness;
                harness.runtimeState.SetVelocity(0.1f);
                SetRollingWheelState(harness.vehicle, harness.runtimeState);
                harness.truthState = CaptureRuntimeState(harness.runtimeState);

                minimumVelocityMps = harness.runtimeState.GetVelocity();
                maximumVelocityMps = harness.runtimeState.GetVelocity();

                for (int tick = 0; tick < kVelocityTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kTargetForwardMps, 0.0f, kNaN, 0.0f, kNaN);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();

                    commandsFinite = commandsFinite && command.IsFinite();
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    requestedForwardPreserved =
                        requestedForwardPreserved &&
                        std::fabs(telemetry.requestedForwardMps - kTargetForwardMps) <= 1.0e-6f;
                    requestedYawRatePreserved =
                        requestedYawRatePreserved &&
                        std::fabs(telemetry.requestedYawRateRadps) <= 1.0e-6f;

                    const float velocityMps = harness.runtimeState.GetVelocity();
                    minimumVelocityMps = (std::min)(minimumVelocityMps, velocityMps);
                    maximumVelocityMps = (std::max)(maximumVelocityMps, velocityMps);
                }
				finalRequestedVelMps = harness.drive.LastTelemetry().requestedForwardMps;
				finalRequestedAccelMps2 = harness.drive.LastTelemetry().composedForwardAccelMps2;
                finalVelocityMps = harness.runtimeState.GetVelocity();
            }
        };

        float AbsYawErrorRad(const float targetYawRad, const float actualYawRad) noexcept
        {
            return std::fabs(VehicleState::NormalizeAngle(targetYawRad - actualYawRad));
        }

        struct YawRateLongRunScenario final
        {
            static constexpr int kYawRateTicks = 500;
            static constexpr float kTargetYawRateRadps = 1.20f;

            bool commandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedYawRatePreserved = true;
            bool forwardVelocityStayedBounded = true;
            float initialYawRateRadps = 0.0f;
            float maximumYawRateRadps = 0.0f;
            float finalYawRateRadps = 0.0f;
            float finalYawRad = 0.0f;

            YawRateLongRunScenario()
            {
                DriveBasePlantHarness harness;
                initialYawRateRadps = harness.runtimeState.GetRotationalVelocity();

                for (int tick = 0; tick < kYawRateTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kNaN, kTargetYawRateRadps, kNaN, 0.0f, kNaN);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();

                    commandsFinite = commandsFinite && command.IsFinite();
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    requestedYawRatePreserved =
                        requestedYawRatePreserved &&
                        std::fabs(telemetry.requestedYawRateRadps - kTargetYawRateRadps) <= 1.0e-6f;
                    forwardVelocityStayedBounded =
                        forwardVelocityStayedBounded &&
                        std::fabs(harness.runtimeState.GetVelocity()) < 0.10f;
                    maximumYawRateRadps =
                        (std::max)(maximumYawRateRadps, harness.runtimeState.GetRotationalVelocity());
                }

                finalYawRateRadps = harness.runtimeState.GetRotationalVelocity();
                finalYawRad = harness.runtimeState.GetOrientation();
            }
        };

        struct HeadingHoldLongRunScenario final
        {
            static constexpr int kHeadingTicks = 900;
            static constexpr float kTargetYawRad = 0.0f;
            static constexpr float kInitialYawRad = 0.35f;

            bool commandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedHeadingPreserved = true;
            bool headingErrorStayedBounded = true;
            float initialHeadingErrorRad = 0.0f;
            float finalHeadingErrorRad = 0.0f;
            float finalYawRateRadps = 0.0f;
            float maxAbsHeadingErrorRad = 0.0f;

            HeadingHoldLongRunScenario()
            {
                DriveBasePlantHarness harness;
                harness.runtimeState.SetOrientation(kInitialYawRad);
                harness.truthState = CaptureRuntimeState(harness.runtimeState);
                initialHeadingErrorRad = AbsYawErrorRad(kTargetYawRad, harness.runtimeState.GetOrientation());
                maxAbsHeadingErrorRad = initialHeadingErrorRad;

                for (int tick = 0; tick < kHeadingTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kNaN, 0.0f, kNaN, 0.0f, kTargetYawRad);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();
                    const float headingErrorRad =
                        AbsYawErrorRad(kTargetYawRad, harness.runtimeState.GetOrientation());

                    commandsFinite = commandsFinite && command.IsFinite();
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    requestedHeadingPreserved =
                        requestedHeadingPreserved &&
                        std::fabs(telemetry.requestedYawRad - kTargetYawRad) <= 1.0e-6f;
                    maxAbsHeadingErrorRad = (std::max)(maxAbsHeadingErrorRad, headingErrorRad);
                    headingErrorStayedBounded =
                        headingErrorStayedBounded &&
                        (headingErrorRad <= initialHeadingErrorRad + 0.08f);
                }

                finalHeadingErrorRad = AbsYawErrorRad(kTargetYawRad, harness.runtimeState.GetOrientation());
                finalYawRateRadps = harness.runtimeState.GetRotationalVelocity();
            }
        };
    }

    TEST_CLASS(DriveStack_DriveBasePlantBoundaryTest)
    {
    public:
        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_CommandIsFinite)
        {
            const BoundarySnapshotScenario scenario;

            Assert::IsTrue(scenario.command.IsFinite(), L"DRV30_BOUNDARY_SNAPSHOT command is not finite");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_LeftCommandIsClamped)
        {
            const BoundarySnapshotScenario scenario;

            Assert::IsTrue(std::fabs(scenario.command.LeftCommand()) <= 1.0f, L"DRV30_BOUNDARY_SNAPSHOT left command is not clamped");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_RightCommandIsClamped)
        {
            const BoundarySnapshotScenario scenario;

            Assert::IsTrue(std::fabs(scenario.command.RightCommand()) <= 1.0f, L"DRV30_BOUNDARY_SNAPSHOT right command is not clamped");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_BodyProposalEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.commandKindFlags, DriveTelemetry::kCommandKindBodyProposal),
                L"DRV30_BOUNDARY_SNAPSHOT missing body proposal evidence");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_ProposalSequenceEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryProposalSequenceValid),
                L"DRV30_BOUNDARY_SNAPSHOT missing proposal sequence evidence");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_CommandEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid),
                L"DRV30_BOUNDARY_SNAPSHOT missing command evidence");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_LeftPlantCommandMatchesAccelerationFeedforward)
        {
            const BoundarySnapshotScenario scenario;

            Assert::AreEqual(
                scenario.expectedPlantCommand.LeftCommand(),
                scenario.telemetry.leftPlantCommand,
                1.0e-6f,
                L"DRV30_BOUNDARY_SNAPSHOT left PlantModel command differs from acceleration feedforward");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_RightPlantCommandMatchesAccelerationFeedforward)
        {
            const BoundarySnapshotScenario scenario;

            Assert::AreEqual(
                scenario.expectedPlantCommand.RightCommand(),
                scenario.telemetry.rightPlantCommand,
                1.0e-6f,
                L"DRV30_BOUNDARY_SNAPSHOT right PlantModel command differs from acceleration feedforward");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_LeftDriveCommandMatchesReturnedCommand)
        {
            const BoundarySnapshotScenario scenario;

            Assert::AreEqual(
                scenario.command.LeftCommand(),
                scenario.telemetry.leftDriveCommand,
                1.0e-6f,
                L"DRV30_BOUNDARY_SNAPSHOT left drive command differs from returned command");
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_RightDriveCommandMatchesReturnedCommand)
        {
            const BoundarySnapshotScenario scenario;

            Assert::AreEqual(
                scenario.command.RightCommand(),
                scenario.telemetry.rightDriveCommand,
                1.0e-6f,
                L"DRV30_BOUNDARY_SNAPSHOT right drive command differs from returned command");
        }

        TEST_METHOD(VelocityFeedback_RequestedForwardMpsIsPreserved)
        {
            const FeedbackScenario scenario;

            Assert::AreEqual(0.80f, scenario.telemetry.requestedForwardMps, 1.0e-6f, L"DRV30_FEEDBACK_DOUBLE_APPLY requested forward target was rewritten");
        }

        TEST_METHOD(VelocityFeedback_RequestedYawRateRadpsIsPreserved)
        {
            const FeedbackScenario scenario;

            Assert::AreEqual(0.25f, scenario.telemetry.requestedYawRateRadps, 1.0e-6f, L"DRV30_FEEDBACK_DOUBLE_APPLY requested yaw-rate target was rewritten");
        }

        TEST_METHOD(VelocityFeedback_RequestedForwardAccelMps2IsPreserved)
        {
            const FeedbackScenario scenario;

            Assert::AreEqual(0.30f, scenario.telemetry.requestedForwardAccelMps2, 1.0e-6f, L"DRV30_FEEDBACK_DOUBLE_APPLY requested forward acceleration was rewritten");
        }

        TEST_METHOD(VelocityFeedback_RequestedYawAccelRadps2IsPreserved)
        {
            const FeedbackScenario scenario;

            Assert::AreEqual(0.40f, scenario.telemetry.requestedYawAccelRadps2, 1.0e-6f, L"DRV30_FEEDBACK_DOUBLE_APPLY requested yaw acceleration was rewritten");
        }

        TEST_METHOD(VelocityFeedback_RequestedYawRadIsPreserved)
        {
            const FeedbackScenario scenario;

            Assert::AreEqual(0.18f, scenario.telemetry.requestedYawRad, 1.0e-6f, L"DRV30_FEEDBACK_DOUBLE_APPLY requested yaw target was rewritten");
        }

        TEST_METHOD(VelocityFeedback_ComposedForwardAccelerationUsesProductionPDOnce)
        {
            const FeedbackScenario scenario;
            const float expected =
                0.30f +
                Config::kDriveBasePDCluster.VelocityStatePD.Compute(0.80f - 0.20f, 0.0f);

            Assert::AreEqual(
                expected,
                scenario.telemetry.composedForwardAccelMps2,
                1.0e-6f,
                L"DRV30_FEEDBACK_DOUBLE_APPLY forward feedback did not compose exactly once");
        }

        TEST_METHOD(VelocityFeedback_ComposedYawAccelerationUsesProductionPDOnce)
        {
            const FeedbackScenario scenario;
            const float expected =
                0.40f +
                Config::kDriveBasePDCluster.YawRateStatePD.Compute(0.25f - -0.15f, 0.0f) +
                Config::kDriveBasePDCluster.HeadingStatePD.Compute(0.18f - 0.10f, 0.25f - -0.15f);

            Assert::AreEqual(
                expected,
                scenario.telemetry.composedYawAccelRadps2,
                1.0e-6f,
                L"DRV30_FEEDBACK_DOUBLE_APPLY yaw feedback did not compose exactly once");
        }

        TEST_METHOD(ClampEvidence_CommandIsFinite)
        {
            const ClampScenario scenario;

            Assert::IsTrue(scenario.command.IsFinite(), L"DRV30_TELEMETRY_EVIDENCE clamp command is invalid");
        }

        TEST_METHOD(ClampEvidence_LeftCommandIsClamped)
        {
            const ClampScenario scenario;

            Assert::IsTrue(std::fabs(scenario.command.LeftCommand()) <= 1.0f, L"DRV30_TELEMETRY_EVIDENCE left clamp command is out of range");
        }

        TEST_METHOD(ClampEvidence_RightCommandIsClamped)
        {
            const ClampScenario scenario;

            Assert::IsTrue(std::fabs(scenario.command.RightCommand()) <= 1.0f, L"DRV30_TELEMETRY_EVIDENCE right clamp command is out of range");
        }

        TEST_METHOD(ClampEvidence_PlantVsDriveClampEvidenceIsVisible)
        {
            const ClampScenario scenario;

            Assert::IsTrue(
                (std::fabs(scenario.telemetry.leftPlantCommand - scenario.telemetry.leftDriveCommand) > 1.0e-5f) ||
                (std::fabs(scenario.telemetry.rightPlantCommand - scenario.telemetry.rightDriveCommand) > 1.0e-5f),
                L"DRV30_TELEMETRY_EVIDENCE plant-vs-drive clamp evidence is not visible");
        }

        TEST_METHOD(ClampEvidence_PlantCommandTelemetryFlagIsSet)
        {
            const ClampScenario scenario;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryPlantCommandValid),
                L"DRV30_TELEMETRY_EVIDENCE missing plant command evidence");
        }

        TEST_METHOD(SolverFailureEvidence_LeftCommandFallsBackToZero)
        {
            const SolverFailureScenario scenario;

            Assert::AreEqual(0.0f, scenario.command.LeftCommand(), 1.0e-6f, L"DRV30_TELEMETRY_EVIDENCE unsupported scalar did not zero left command");
        }

        TEST_METHOD(SolverFailureEvidence_RightCommandFallsBackToZero)
        {
            const SolverFailureScenario scenario;

            Assert::AreEqual(0.0f, scenario.command.RightCommand(), 1.0e-6f, L"DRV30_TELEMETRY_EVIDENCE unsupported scalar did not zero right command");
        }

        TEST_METHOD(SolverFailureEvidence_CommandKindFlagIsSet)
        {
            const SolverFailureScenario scenario;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.commandKindFlags, DriveTelemetry::kCommandKindSolverFailureEvidence),
                L"DRV30_TELEMETRY_EVIDENCE unsupported scalar did not publish solver failure evidence");
        }

        TEST_METHOD(SolverFailureEvidence_UnsupportedScalarFlagIsSet)
        {
            const SolverFailureScenario scenario;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.solverFailureFlags, DriveTelemetry::kSolverFailureUnsupportedScalarIntent),
                L"DRV30_TELEMETRY_EVIDENCE unsupported scalar flag is missing");
        }

        TEST_METHOD(AccelerationTargetLongRun_CommandsRemainFinite)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(scenario.commandsFinite, L"DRV30_ACCEL_TARGET_LONG_RUN command became invalid");
        }

        TEST_METHOD(AccelerationTargetLongRun_LeftCommandsRemainClamped)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(scenario.leftCommandsClamped, L"DRV30_ACCEL_TARGET_LONG_RUN left command exceeded clamp range");
        }

        TEST_METHOD(AccelerationTargetLongRun_RightCommandsRemainClamped)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(scenario.rightCommandsClamped, L"DRV30_ACCEL_TARGET_LONG_RUN right command exceeded clamp range");
        }

        TEST_METHOD(AccelerationTargetLongRun_CommandEvidenceRemainsVisible)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(scenario.commandEvidenceValid, L"DRV30_ACCEL_TARGET_LONG_RUN missing command evidence during run");
        }

        TEST_METHOD(AccelerationTargetLongRun_AccelerationObjectiveIsPreserved)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(scenario.accelerationObjectiveUnchanged, L"DRV30_ACCEL_TARGET_LONG_RUN acceleration objective changed at DriveBase boundary");
        }

        TEST_METHOD(AccelerationTargetLongRun_TwentyTickResponseTrendsForward)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(
                scenario.velocityAtMinimumHorizonMps > scenario.initialVelocityMps + 0.005f,
                L"DRV30_ACCEL_TARGET_LONG_RUN 20-tick response did not trend forward");
        }

        TEST_METHOD(AccelerationTargetLongRun_FinalResponseAccumulatesForwardVelocity)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(
                scenario.finalVelocityMps > scenario.initialVelocityMps + 0.05f,
                L"DRV30_ACCEL_TARGET_LONG_RUN final response did not accumulate forward velocity");
        }

        TEST_METHOD(AccelerationTargetLongRun_ResponseDoesNotDivergeOppositeRequest)
        {
            const AccelerationLongRunScenario scenario;

            Assert::IsTrue(
                scenario.minimumVelocityMps > -0.01f,
                L"DRV30_ACCEL_TARGET_LONG_RUN response diverged opposite the requested acceleration");
        }

        TEST_METHOD(VelocityTargetLongRun_CommandsRemainFinite)
        {
            const VelocityLongRunScenario scenario;

            Assert::IsTrue(scenario.commandsFinite, L"DRV30_VELOCITY_TARGET_LONG_RUN command became invalid");
        }

        TEST_METHOD(VelocityTargetLongRun_CommandEvidenceRemainsVisible)
        {
            const VelocityLongRunScenario scenario;

            Assert::IsTrue(scenario.commandEvidenceValid, L"DRV30_VELOCITY_TARGET_LONG_RUN command evidence became invalid");
        }

        TEST_METHOD(VelocityTargetLongRun_RequestedForwardTargetIsPreserved)
        {
            const VelocityLongRunScenario scenario;

            Assert::IsTrue(scenario.requestedForwardPreserved, L"DRV30_VELOCITY_TARGET_LONG_RUN target velocity was rewritten");
        }

        TEST_METHOD(VelocityTargetLongRun_RequestedYawRateTargetIsPreserved)
        {
            const VelocityLongRunScenario scenario;

            Assert::IsTrue(scenario.requestedYawRatePreserved, L"DRV30_VELOCITY_TARGET_LONG_RUN target yaw rate was rewritten");
        }

        TEST_METHOD(VelocityTargetLongRun_ResponseApproachesPositiveTarget)
        {
            const VelocityLongRunScenario scenario;
            auto ss = std::wstringstream();
            ss << "DRV30_VELOCITY_TARGET_LONG_RUN  " << scenario.finalVelocityMps << "m/s did not approach the positive target.\n";
            ss << "Final Requested Velocity: " << scenario.finalRequestedVelMps << "\n";
            ss << "Final Requested Acceleration: " << scenario.finalRequestedAccelMps2 << "\n";
            Assert::IsTrue(
                scenario.finalVelocityMps > 0.20f,
                ss.str().c_str());
        }

        TEST_METHOD(VelocityTargetLongRun_FinalVelocitySettlesNearTarget)
        {
            const VelocityLongRunScenario scenario;

            Assert::AreEqual(
                VelocityLongRunScenario::kTargetForwardMps, scenario.finalVelocityMps, 0.15f,
                L"DRV30_VELOCITY_TARGET_LONG_RUN final velocity did not settle near target");
        }

        TEST_METHOD(VelocityTargetLongRun_ResponseDoesNotDivergeWithWrongSign)
        {
            const VelocityLongRunScenario scenario;

            Assert::IsTrue(
                scenario.minimumVelocityMps > -0.02f,
                L"DRV30_VELOCITY_TARGET_LONG_RUN response diverged with the wrong sign");
        }

        TEST_METHOD(VelocityTargetLongRun_ResponseDoesNotOvershootBeyondDiagnosticTolerance)
        {
            const VelocityLongRunScenario scenario;

            Assert::IsTrue(
                scenario.maximumVelocityMps < VelocityLongRunScenario::kTargetForwardMps + 0.30f,
                L"DRV30_VELOCITY_TARGET_LONG_RUN response overshot beyond diagnostic tolerance");
        }

        TEST_METHOD(YawRateTargetLongRun_CommandsRemainFinite)
        {
            const YawRateLongRunScenario scenario;

            Assert::IsTrue(scenario.commandsFinite, L"DRV30_YAW_RATE_CLOSED_LOOP command became invalid");
        }

        TEST_METHOD(YawRateTargetLongRun_CommandEvidenceRemainsVisible)
        {
            const YawRateLongRunScenario scenario;

            Assert::IsTrue(scenario.commandEvidenceValid, L"DRV30_YAW_RATE_CLOSED_LOOP command evidence became invalid");
        }

        TEST_METHOD(YawRateTargetLongRun_RequestedYawRateTargetIsPreserved)
        {
            const YawRateLongRunScenario scenario;

            Assert::IsTrue(scenario.requestedYawRatePreserved, L"DRV30_YAW_RATE_CLOSED_LOOP target yaw rate was rewritten");
        }

        TEST_METHOD(YawRateTargetLongRun_ResponseBuildsClockwiseYawRate)
        {
            const YawRateLongRunScenario scenario;

            Assert::IsTrue(
                scenario.maximumYawRateRadps > scenario.initialYawRateRadps + 0.35f,
                L"DRV30_YAW_RATE_CLOSED_LOOP plant integration did not build clockwise yaw rate");
        }

        TEST_METHOD(YawRateTargetLongRun_FinalYawRateSettlesNearTarget)
        {
            const YawRateLongRunScenario scenario;

            Assert::AreEqual(
                YawRateLongRunScenario::kTargetYawRateRadps,
                scenario.finalYawRateRadps,
                0.45f,
                L"DRV30_YAW_RATE_CLOSED_LOOP final yaw rate did not settle near target");
        }

        TEST_METHOD(YawRateTargetLongRun_DoesNotCreateLargeForwardDrift)
        {
            const YawRateLongRunScenario scenario;

            Assert::IsTrue(
                scenario.forwardVelocityStayedBounded,
                L"DRV30_YAW_RATE_CLOSED_LOOP yaw-rate hold created unexpected forward drift");
        }

        TEST_METHOD(HeadingHoldLongRun_CommandsRemainFinite)
        {
            const HeadingHoldLongRunScenario scenario;

            Assert::IsTrue(scenario.commandsFinite, L"DRV30_HEADING_CLOSED_LOOP command became invalid");
        }

        TEST_METHOD(HeadingHoldLongRun_CommandEvidenceRemainsVisible)
        {
            const HeadingHoldLongRunScenario scenario;

            Assert::IsTrue(scenario.commandEvidenceValid, L"DRV30_HEADING_CLOSED_LOOP command evidence became invalid");
        }

        TEST_METHOD(HeadingHoldLongRun_RequestedHeadingTargetIsPreserved)
        {
            const HeadingHoldLongRunScenario scenario;

            Assert::IsTrue(scenario.requestedHeadingPreserved, L"DRV30_HEADING_CLOSED_LOOP target heading was rewritten");
        }

        TEST_METHOD(HeadingHoldLongRun_PhysicalPlantReducesHeadingError)
        {
            const HeadingHoldLongRunScenario scenario;

            Assert::IsTrue(
                scenario.finalHeadingErrorRad < scenario.initialHeadingErrorRad * 0.45f,
                L"DRV30_HEADING_CLOSED_LOOP plant integration did not reduce heading error");
        }

        TEST_METHOD(HeadingHoldLongRun_HeadingErrorStaysBoundedDuringCorrection)
        {
            const HeadingHoldLongRunScenario scenario;

            Assert::IsTrue(
                scenario.headingErrorStayedBounded,
                L"DRV30_HEADING_CLOSED_LOOP heading error diverged before settling");
        }

        TEST_METHOD(HeadingHoldLongRun_FinalYawRateIsDamped)
        {
            const HeadingHoldLongRunScenario scenario;

            Assert::IsTrue(
                std::fabs(scenario.finalYawRateRadps) < 0.80f,
                L"DRV30_HEADING_CLOSED_LOOP final yaw rate did not damp after heading correction");
        }
    };
}
