#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>
#include <cstdint>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using CommandVector = MazeMap::App::Internal::CommandVector;

        constexpr float kNaN = (std::numeric_limits<float>::quiet_NaN)();
        constexpr float kInf = (std::numeric_limits<float>::infinity)();

        struct DriveBaseHarness final
        {
            Vehicle vehicle;
            VehicleState runtimeState;
            PlantModel plant;
            PDCluster feedbackTuning;
            DriveBase drive;

            explicit DriveBaseHarness(const PDCluster& cluster = PDCluster{}) noexcept
                : vehicle()
                , runtimeState()
                , plant(vehicle, runtimeState)
                , feedbackTuning(cluster)
                , drive(plant, runtimeState, feedbackTuning)
            {
            }
        };

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

        void AssertFlagSet(const std::uint16_t flags, const std::uint16_t flag)
        {
            Assert::IsTrue((flags & flag) != 0U);
        }

        void AssertFlagClear(const std::uint16_t flags, const std::uint16_t flag)
        {
            Assert::IsTrue((flags & flag) == 0U);
        }

        void AssertFiniteCommand(const CommandVector& command)
        {
            Assert::IsTrue(std::isfinite(command.LeftCommand()));
            Assert::IsTrue(std::isfinite(command.RightCommand()));
        }

        void AssertCommandMatchesTelemetry(
            const CommandVector& command,
            const DriveTelemetry& telemetry)
        {
            Assert::AreEqual(telemetry.leftDriveCommand, command.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(telemetry.rightDriveCommand, command.RightCommand(), 1.0e-6f);
        }

        void AssertMatchesPlantSolve(
            const DriveBaseHarness& harness,
            const DriveTelemetry& telemetry,
            const VehicleState::StateVector& stateAtProposal)
        {
            const PlantModelBodyActionSolveResult expected =
                harness.plant.SolveBodyActionInverse(
                    stateAtProposal,
                    telemetry.requestedForwardMps,
                    telemetry.requestedYawRateRadps,
                    telemetry.requestedForwardAccelMps2,
                    telemetry.requestedYawAccelRadps2,
                    telemetry.composedForwardAccelMps2,
                    telemetry.composedYawAccelRadps2);

            Assert::AreEqual(expected.plantEvaluationId, telemetry.plantEvaluationId);
            Assert::AreEqual(expected.command.LeftCommand(), telemetry.leftPlantCommand, 1.0e-6f);
            Assert::AreEqual(expected.command.RightCommand(), telemetry.rightPlantCommand, 1.0e-6f);
        }
    }

    TEST_CLASS(DriveBaseTest)
    {
    public:
        TEST_METHOD(ClearCommandEvidenceMarksLastTelemetryStale)
        {
            DriveBaseHarness harness;

            (void)harness.drive.ProposeBodyTick(0.25f, 0.0f, 1.0f, 0.0f, kNaN);
            AssertFlagSet(harness.drive.LastTelemetry().commandKindFlags, DriveTelemetry::kCommandKindBodyProposal);

            harness.drive.ClearCommandEvidence();
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFlagSet(telemetry.commandKindFlags, DriveTelemetry::kCommandKindStaleEvidence);
            AssertFlagClear(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::AreEqual(0U, telemetry.proposalSequenceId);
        }

        TEST_METHOD(ProposeBodyTickPublishesFreshCommandEvidence)
        {
            DriveBaseHarness harness;
            const VehicleState::StateVector stateAtProposal = CaptureRuntimeState(harness.runtimeState);

            const CommandVector command =
                harness.drive.ProposeBodyTick(0.40f, 0.25f, 1.5f, 2.0f, 0.10f);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            AssertCommandMatchesTelemetry(command, telemetry);
            AssertFlagSet(telemetry.commandKindFlags, DriveTelemetry::kCommandKindBodyProposal);
            AssertFlagClear(telemetry.commandKindFlags, DriveTelemetry::kCommandKindStaleEvidence);
            AssertFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryProposalSequenceValid);
            AssertFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryPlantEvaluationValid);
            AssertFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::AreEqual(0.40f, telemetry.requestedForwardMps, 1.0e-6f);
            Assert::AreEqual(0.25f, telemetry.requestedYawRateRadps, 1.0e-6f);
            Assert::AreEqual(1.5f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(2.0f, telemetry.requestedYawAccelRadps2, 1.0e-6f);
            Assert::AreEqual(0.10f, telemetry.requestedYawRad, 1.0e-6f);
            AssertMatchesPlantSolve(harness, telemetry, stateAtProposal);
        }

        TEST_METHOD(NaNObjectivesDisableFeedbackAndRemainInactiveInTelemetry)
        {
            PDCluster cluster;
            cluster.VelocityStatePD.SetGains(100.0f, 0.0f);
            cluster.YawRateStatePD.SetGains(100.0f, 0.0f);
            cluster.HeadingStatePD.SetGains(100.0f, 0.0f);
            DriveBaseHarness harness(cluster);
            harness.runtimeState.SetVelocity(0.5f);
            harness.runtimeState.SetRotationalVelocity(1.0f);
            harness.runtimeState.SetOrientation(0.25f);
            const VehicleState::StateVector stateAtProposal = CaptureRuntimeState(harness.runtimeState);

            (void)harness.drive.ProposeBodyTick(kNaN, kNaN, kNaN, kNaN, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarForwardVelocityInactive);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarYawRateInactive);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarForwardAccelInactive);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarYawAccelInactive);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarYawInactive);
            AssertFlagSet(telemetry.feedbackBranchFlags, DriveTelemetry::kFeedbackForwardVelocityInactive);
            AssertFlagSet(telemetry.feedbackBranchFlags, DriveTelemetry::kFeedbackYawRateInactive);
            AssertFlagSet(telemetry.feedbackBranchFlags, DriveTelemetry::kFeedbackHeadingInactive);
            Assert::IsTrue(std::isnan(telemetry.composedForwardAccelMps2));
            Assert::IsTrue(std::isnan(telemetry.composedYawAccelRadps2));
            AssertMatchesPlantSolve(harness, telemetry, stateAtProposal);
        }

        TEST_METHOD(ForwardVelocityFeedbackChangesOnlyComposedForwardAcceleration)
        {
            PDCluster cluster;
            cluster.VelocityStatePD.SetGains(3.0f, 0.0f);
            DriveBaseHarness harness(cluster);
            harness.runtimeState.SetVelocity(0.20f);
            const VehicleState::StateVector stateAtProposal = CaptureRuntimeState(harness.runtimeState);

            (void)harness.drive.ProposeBodyTick(1.0f, kNaN, 0.50f, kNaN, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::AreEqual(1.0f, telemetry.requestedForwardMps, 1.0e-6f);
            Assert::AreEqual(0.50f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(0.50f + ((1.0f - 0.20f) * 3.0f), telemetry.composedForwardAccelMps2, 1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRateRadps));
            Assert::IsTrue(std::isnan(telemetry.composedYawAccelRadps2));
            AssertMatchesPlantSolve(harness, telemetry, stateAtProposal);
        }

        TEST_METHOD(YawRateAndHeadingFeedbackComposeYawAcceleration)
        {
            PDCluster cluster;
            cluster.YawRateStatePD.SetGains(2.0f, 0.0f);
            cluster.HeadingStatePD.SetGains(5.0f, 0.0f);
            DriveBaseHarness harness(cluster);
            harness.runtimeState.SetRotationalVelocity(0.40f);
            harness.runtimeState.SetOrientation(0.25f);
            const VehicleState::StateVector stateAtProposal = CaptureRuntimeState(harness.runtimeState);

            (void)harness.drive.ProposeBodyTick(kNaN, 1.0f, kNaN, 0.30f, 0.75f);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::AreEqual(1.0f, telemetry.requestedYawRateRadps, 1.0e-6f);
            Assert::AreEqual(0.75f, telemetry.requestedYawRad, 1.0e-6f);
            Assert::AreEqual(
                0.30f + ((1.0f - 0.40f) * 2.0f) + ((0.75f - 0.25f) * 5.0f),
                telemetry.composedYawAccelRadps2,
                1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.composedForwardAccelMps2));
            AssertMatchesPlantSolve(harness, telemetry, stateAtProposal);
        }

        TEST_METHOD(InfiniteAccelerationIntentReachesPlantModelAsMaximizeObjective)
        {
            PDCluster cluster;
            cluster.VelocityStatePD.SetGains(100.0f, 0.0f);
            cluster.YawRateStatePD.SetGains(100.0f, 0.0f);
            DriveBaseHarness harness(cluster);
            const VehicleState::StateVector stateAtProposal = CaptureRuntimeState(harness.runtimeState);

            const CommandVector command =
                harness.drive.ProposeBodyTick(2.0f, -3.0f, kInf, -kInf, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarForwardAccelMaximize);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarYawAccelMaximize);
            AssertFlagSet(telemetry.feedbackBranchFlags, DriveTelemetry::kFeedbackForwardSuppressedForMaximize);
            AssertFlagSet(telemetry.feedbackBranchFlags, DriveTelemetry::kFeedbackYawSuppressedForMaximize);
            Assert::IsTrue(std::isinf(telemetry.composedForwardAccelMps2));
            Assert::IsFalse(std::signbit(telemetry.composedForwardAccelMps2));
            Assert::IsTrue(std::isinf(telemetry.composedYawAccelRadps2));
            Assert::IsTrue(std::signbit(telemetry.composedYawAccelRadps2));
            AssertMatchesPlantSolve(harness, telemetry, stateAtProposal);
        }

        TEST_METHOD(CommandClampingIsRecordedAsPlantCommandAndFinalCommandEvidence)
        {
            DriveBaseHarness harness;

            const CommandVector command =
                harness.drive.ProposeBodyTick(kNaN, kNaN, 1000000.0f, 0.0f, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            Assert::IsTrue(command.LeftCommand() <= 1.0f);
            Assert::IsTrue(command.RightCommand() <= 1.0f);
            Assert::IsTrue(command.LeftCommand() >= -1.0f);
            Assert::IsTrue(command.RightCommand() >= -1.0f);
            Assert::IsTrue(
                (std::fabs(telemetry.leftPlantCommand - telemetry.leftDriveCommand) > 1.0e-5f) ||
                (std::fabs(telemetry.rightPlantCommand - telemetry.rightDriveCommand) > 1.0e-5f));
            AssertCommandMatchesTelemetry(command, telemetry);
        }

        TEST_METHOD(InfiniteHeadingObjectiveIsUnsupportedAndProducesNeutralFailureEvidence)
        {
            DriveBaseHarness harness;

            const CommandVector command =
                harness.drive.ProposeBodyTick(kNaN, kNaN, 0.0f, 0.0f, kInf);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::AreEqual(0.0f, command.LeftCommand(), 1.0e-6f);
            Assert::AreEqual(0.0f, command.RightCommand(), 1.0e-6f);
            AssertFlagSet(telemetry.scalarIntentFlags, DriveTelemetry::kScalarYawMaximizeUnsupported);
            AssertFlagSet(telemetry.solverFailureFlags, DriveTelemetry::kSolverFailureUnsupportedScalarIntent);
            AssertFlagSet(telemetry.commandKindFlags, DriveTelemetry::kCommandKindSolverFailureEvidence);
            AssertCommandMatchesTelemetry(command, telemetry);
        }

        TEST_METHOD(RepeatedProposalsAdvanceLastTelemetrySequence)
        {
            DriveBaseHarness harness;

            (void)harness.drive.ProposeBodyTick(kNaN, kNaN, 0.25f, 0.0f, kNaN);
            const std::uint32_t firstSequence = harness.drive.LastTelemetry().proposalSequenceId;
            (void)harness.drive.ProposeBodyTick(kNaN, kNaN, 0.50f, 0.0f, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::IsTrue(firstSequence > 0U);
            Assert::AreEqual(firstSequence + 1U, telemetry.proposalSequenceId);
            Assert::AreEqual(0.50f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
        }
    };
}
