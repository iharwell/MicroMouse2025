#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\CoreConfig.h"
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

            explicit DriveBaseHarness(const PDCluster& cluster = Config::kDriveBasePDCluster) noexcept
                : vehicle()
                , runtimeState()
                , plant(vehicle, runtimeState)
                , feedbackTuning(cluster)
                , drive(plant, runtimeState, feedbackTuning)
            {
            }
        };

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

        void AssertMatchesPlantFeedforward(
            const DriveBaseHarness& harness,
            const DriveTelemetry& telemetry)
        {
            const CommandVector expected =
                harness.plant.ComputeFeedforward(
                    telemetry.composedForwardAccelMps2,
                    telemetry.composedYawAccelRadps2);

            AssertFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryPlantCommandValid);
            Assert::AreEqual(expected.LeftCommand(), telemetry.leftPlantCommand, 1.0e-6f);
            Assert::AreEqual(expected.RightCommand(), telemetry.rightPlantCommand, 1.0e-6f);
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

            const CommandVector command =
                harness.drive.ProposeBodyTick(0.40f, 0.25f, 1.5f, 2.0f, 0.10f);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            AssertCommandMatchesTelemetry(command, telemetry);
            AssertFlagSet(telemetry.commandKindFlags, DriveTelemetry::kCommandKindBodyProposal);
            AssertFlagClear(telemetry.commandKindFlags, DriveTelemetry::kCommandKindStaleEvidence);
            AssertFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryProposalSequenceValid);
            AssertFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::AreEqual(0.40f, telemetry.requestedForwardMps, 1.0e-6f);
            Assert::AreEqual(0.25f, telemetry.requestedYawRateRadps, 1.0e-6f);
            Assert::AreEqual(1.5f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(2.0f, telemetry.requestedYawAccelRadps2, 1.0e-6f);
            Assert::AreEqual(0.10f, telemetry.requestedYawRad, 1.0e-6f);
            AssertMatchesPlantFeedforward(harness, telemetry);
        }

        TEST_METHOD(NaNObjectivesRemainInactiveInTelemetry)
        {
            DriveBaseHarness harness;
            harness.runtimeState.SetForwardVelocity(0.5f);
            harness.runtimeState.SetYawRate(1.0f);
            harness.runtimeState.SetHeading(0.25f);

            (void)harness.drive.ProposeBodyTick(kNaN, kNaN, kNaN, kNaN, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::IsTrue(std::isnan(telemetry.requestedForwardMps));
            Assert::IsTrue(std::isnan(telemetry.requestedYawRateRadps));
            Assert::IsTrue(std::isnan(telemetry.requestedForwardAccelMps2));
            Assert::IsTrue(std::isnan(telemetry.requestedYawAccelRadps2));
            Assert::IsTrue(std::isnan(telemetry.requestedYawRad));
            Assert::IsTrue(std::isnan(telemetry.composedForwardAccelMps2));
            Assert::IsTrue(std::isnan(telemetry.composedYawAccelRadps2));
            AssertMatchesPlantFeedforward(harness, telemetry);
        }

        TEST_METHOD(ForwardVelocityFeedbackChangesOnlyComposedForwardAcceleration)
        {
            DriveBaseHarness harness;
            harness.runtimeState.SetForwardVelocity(0.20f);

            (void)harness.drive.ProposeBodyTick(1.0f, kNaN, 0.50f, kNaN, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::AreEqual(1.0f, telemetry.requestedForwardMps, 1.0e-6f);
            Assert::AreEqual(0.50f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(
                0.50f + harness.feedbackTuning.VelocityStatePD.Compute(1.0f - 0.20f, 0.0f),
                telemetry.composedForwardAccelMps2,
                1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRateRadps));
            Assert::IsTrue(std::isnan(telemetry.composedYawAccelRadps2));
            AssertMatchesPlantFeedforward(harness, telemetry);
        }

        TEST_METHOD(YawRateAndHeadingFeedbackComposeYawAcceleration)
        {
            DriveBaseHarness harness;
            harness.runtimeState.SetYawRate(0.40f);
            harness.runtimeState.SetHeading(0.25f);

            (void)harness.drive.ProposeBodyTick(kNaN, 1.0f, kNaN, 0.30f, 0.75f);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            Assert::AreEqual(1.0f, telemetry.requestedYawRateRadps, 1.0e-6f);
            Assert::AreEqual(0.75f, telemetry.requestedYawRad, 1.0e-6f);
            Assert::AreEqual(
                0.30f +
                    harness.feedbackTuning.YawRateStatePD.Compute(1.0f - 0.40f, 0.0f) +
                    harness.feedbackTuning.HeadingStatePD.Compute(0.75f - 0.25f, 1.0f - 0.40f),
                telemetry.composedYawAccelRadps2,
                1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.composedForwardAccelMps2));
            AssertMatchesPlantFeedforward(harness, telemetry);
        }

        TEST_METHOD(InfiniteAccelerationIntentReachesPlantModelAsMaximizeObjective)
        {
            DriveBaseHarness harness;

            const CommandVector command =
                harness.drive.ProposeBodyTick(2.0f, -3.0f, kInf, -kInf, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            Assert::AreEqual(2.0f, telemetry.requestedForwardMps, 1.0e-6f);
            Assert::AreEqual(-3.0f, telemetry.requestedYawRateRadps, 1.0e-6f);
            Assert::IsTrue(telemetry.requestedForwardAccelMps2 == kInf);
            Assert::IsTrue(telemetry.requestedYawAccelRadps2 == -kInf);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRad));
            Assert::IsTrue(std::isinf(telemetry.composedForwardAccelMps2));
            Assert::IsFalse(std::signbit(telemetry.composedForwardAccelMps2));
            Assert::IsTrue(std::isinf(telemetry.composedYawAccelRadps2));
            Assert::IsTrue(std::signbit(telemetry.composedYawAccelRadps2));
            AssertMatchesPlantFeedforward(harness, telemetry);
        }

        TEST_METHOD(InfiniteForwardVelocityIntentIsRecordedInDriveTelemetry)
        {
            DriveBaseHarness harness;

            const CommandVector command =
                harness.drive.ProposeBodyTick(kInf, kNaN, 0.0f, 0.0f, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            Assert::IsTrue(telemetry.requestedForwardMps == kInf);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRateRadps));
            Assert::AreEqual(0.0f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, telemetry.requestedYawAccelRadps2, 1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRad));
            AssertFlagClear(telemetry.commandKindFlags, DriveTelemetry::kCommandKindSolverFailureEvidence);
            Assert::IsTrue(std::isinf(telemetry.requestedForwardMps));
        }

        TEST_METHOD(InfiniteYawRateIntentIsRecordedInDriveTelemetry)
        {
            DriveBaseHarness harness;

            const CommandVector command =
                harness.drive.ProposeBodyTick(kNaN, -kInf, 0.0f, 0.0f, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();

            AssertFiniteCommand(command);
            Assert::IsTrue(std::isnan(telemetry.requestedForwardMps));
            Assert::IsTrue(telemetry.requestedYawRateRadps == -kInf);
            Assert::AreEqual(0.0f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, telemetry.requestedYawAccelRadps2, 1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRad));
            AssertFlagClear(telemetry.commandKindFlags, DriveTelemetry::kCommandKindSolverFailureEvidence);
            Assert::IsTrue(std::isinf(telemetry.requestedYawRateRadps));
        }

        TEST_METHOD(CommandClampingIsRecordedAsPlantCommandAndFinalCommandEvidence)
        {
            DriveBaseHarness harness;

            const CommandVector command =
                harness.drive.ProposeBodyTick(kNaN, kNaN, 1000000.0f, 0.0f, kNaN);
            const DriveTelemetry& telemetry = harness.drive.LastTelemetry();


            Assert::IsTrue(std::isnan(telemetry.requestedForwardMps));
            Assert::IsTrue(std::isnan(telemetry.requestedYawRateRadps));
            Assert::AreEqual(1000000.0f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, telemetry.requestedYawAccelRadps2, 1.0e-6f);
            Assert::IsTrue(std::isnan(telemetry.requestedYawRad));

            AssertFiniteCommand(command);
            Assert::IsTrue(command.LeftCommand() <= 1.0f);
            Assert::IsTrue(command.RightCommand() <= 1.0f);
            Assert::IsTrue(command.LeftCommand() >= -1.0f);
            Assert::IsTrue(command.RightCommand() >= -1.0f);
            Assert::IsTrue(
                (std::fabs(telemetry.leftPlantCommand - telemetry.leftDriveCommand) > 1.0e-5f) ||
                (std::fabs(telemetry.rightPlantCommand - telemetry.rightDriveCommand) > 1.0e-5f));
            AssertMatchesPlantFeedforward(harness, telemetry);
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

            Assert::IsTrue(std::isnan(telemetry.requestedForwardMps));
            Assert::IsTrue(std::isnan(telemetry.requestedYawRateRadps));
            Assert::AreEqual(0.0f, telemetry.requestedForwardAccelMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, telemetry.requestedYawAccelRadps2, 1.0e-6f);
            Assert::IsTrue(telemetry.requestedYawRad == kInf);

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
