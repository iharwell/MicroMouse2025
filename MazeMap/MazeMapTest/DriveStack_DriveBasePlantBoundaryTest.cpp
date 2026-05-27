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
        constexpr float kDtSeconds = 0.001f;

        void SetRollingWheelState(Vehicle& vehicle, VehicleState& runtimeState)
        {
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            vehicle.WheelSpeedsFromBodyVelocity(
                runtimeState.GetForwardVelocity(),
                runtimeState.GetYawRate(),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            runtimeState.SetWheelSpeedLeft(leftWheelSpeedRadps);
            runtimeState.SetWheelSpeedRight(rightWheelSpeedRadps);
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

            DriveBasePlantHarness() noexcept
                : vehicle()
                , runtimeState()
                , plant(vehicle, runtimeState)
                , drive(plant, runtimeState, MazeMap::Config::kDriveBasePDCluster)
            {
                vehicle.SetFanDuty(0.80f);
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
                plant.integrate(command, kDtSeconds);
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
                harness.runtimeState.SetHeading(0.21f);
                harness.runtimeState.SetForwardVelocity(0.37f);
                harness.runtimeState.SetRightwardVelocity(-0.025f);
                harness.runtimeState.SetYawRate(0.18f);
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
                harness.runtimeState.SetForwardVelocity(0.20f);
                harness.runtimeState.SetYawRate(-0.15f);
                harness.runtimeState.SetHeading(0.10f);
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

            bool leftCommandsFinite = true;
            bool rightCommandsFinite = true;
            bool leftCommandsClamped = true;
            bool rightCommandsClamped = true;
            bool commandEvidenceValid = true;
            bool accelerationObjectiveUnchanged = true;
            float initialVelocityMps = 0.0f;
            float velocityAtMinimumHorizonMps = 0.0f;
            float finalVelocityMps = 0.0f;
            float minimumVelocityMps = 0.0f;
            float maxAbsLeftCommand = 0.0f;
            float maxAbsRightCommand = 0.0f;
            float finalLeftCommand = 0.0f;
            float finalRightCommand = 0.0f;
            float finalComposedForwardAccelMps2 = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;

            AccelerationLongRunScenario()
            {
                DriveBasePlantHarness harness;
                initialVelocityMps = harness.runtimeState.GetForwardVelocity();
                velocityAtMinimumHorizonMps = initialVelocityMps;
                minimumVelocityMps = initialVelocityMps;

                for (int tick = 0; tick < kAccelerationTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kNaN, 0.0f, 2.0f, 0.0f, kNaN);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();

                    leftCommandsFinite = leftCommandsFinite && std::isfinite(command.LeftCommand());
                    rightCommandsFinite = rightCommandsFinite && std::isfinite(command.RightCommand());
                    leftCommandsClamped = leftCommandsClamped && std::fabs(command.LeftCommand()) <= 1.0f;
                    rightCommandsClamped = rightCommandsClamped && std::fabs(command.RightCommand()) <= 1.0f;
                    maxAbsLeftCommand = (std::max)(maxAbsLeftCommand, std::fabs(command.LeftCommand()));
                    maxAbsRightCommand = (std::max)(maxAbsRightCommand, std::fabs(command.RightCommand()));
                    finalLeftCommand = command.LeftCommand();
                    finalRightCommand = command.RightCommand();
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    accelerationObjectiveUnchanged =
                        accelerationObjectiveUnchanged &&
                        std::fabs(telemetry.composedForwardAccelMps2 - 2.0f) <= 1.0e-6f;
                    finalComposedForwardAccelMps2 = telemetry.composedForwardAccelMps2;
                    finalTelemetryValidFlags = telemetry.telemetryValidFlags;

                    minimumVelocityMps = (std::min)(minimumVelocityMps, harness.runtimeState.GetForwardVelocity());
                    if (tick + 1 == kMinimumDiagnosticTicks)
                    {
                        velocityAtMinimumHorizonMps = harness.runtimeState.GetForwardVelocity();
                    }
                }

                finalVelocityMps = harness.runtimeState.GetForwardVelocity();
            }
        };

        struct VelocityLongRunScenario final
        {
            static constexpr int kVelocityTicks = 1500;
            static constexpr float kTargetForwardMps = 0.5f;

            bool leftCommandsFinite = true;
            bool rightCommandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedForwardPreserved = true;
            bool requestedYawRatePreserved = true;
            float minimumVelocityMps = 0.0f;
            float maximumVelocityMps = 0.0f;
            float finalVelocityMps = 0.0f;
            float finalLeftCommand = 0.0f;
            float finalRightCommand = 0.0f;
            float finalRequestedVelMps = 0.0f;
            float finalRequestedAccelMps2 = 0.0f;
            float finalRequestedYawRateRadps = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;
            VelocityLongRunScenario()
            {
                DriveBasePlantHarness harness;
                harness.runtimeState.SetForwardVelocity(0.1f);
                SetRollingWheelState(harness.vehicle, harness.runtimeState);

                minimumVelocityMps = harness.runtimeState.GetForwardVelocity();
                maximumVelocityMps = harness.runtimeState.GetForwardVelocity();

                for (int tick = 0; tick < kVelocityTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kTargetForwardMps, 0.0f, kNaN, 0.0f, kNaN);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();

                    leftCommandsFinite = leftCommandsFinite && std::isfinite(command.LeftCommand());
                    rightCommandsFinite = rightCommandsFinite && std::isfinite(command.RightCommand());
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    requestedForwardPreserved =
                        requestedForwardPreserved &&
                        std::fabs(telemetry.requestedForwardMps - kTargetForwardMps) <= 1.0e-6f;
                    requestedYawRatePreserved =
                        requestedYawRatePreserved &&
                        std::fabs(telemetry.requestedYawRateRadps) <= 1.0e-6f;

                    const float velocityMps = harness.runtimeState.GetForwardVelocity();
                    finalLeftCommand = command.LeftCommand();
                    finalRightCommand = command.RightCommand();
                    minimumVelocityMps = (std::min)(minimumVelocityMps, velocityMps);
                    maximumVelocityMps = (std::max)(maximumVelocityMps, velocityMps);
                }
				finalRequestedVelMps = harness.drive.LastTelemetry().requestedForwardMps;
				finalRequestedAccelMps2 = harness.drive.LastTelemetry().composedForwardAccelMps2;
                finalRequestedYawRateRadps = harness.drive.LastTelemetry().requestedYawRateRadps;
                finalTelemetryValidFlags = harness.drive.LastTelemetry().telemetryValidFlags;
                finalVelocityMps = harness.runtimeState.GetForwardVelocity();
            }
        };

        float AbsYawErrorRad(const float targetYawRad, const float actualYawRad) noexcept
        {
            return std::fabs(NormalizeAngle(targetYawRad - actualYawRad));
        }

        struct YawRateLongRunScenario final
        {
            static constexpr int kYawRateTicks = 500;
            static constexpr float kTargetYawRateRadps = 2.20f;

            bool leftCommandsFinite = true;
            bool rightCommandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedYawRatePreserved = true;
            bool forwardVelocityStayedBounded = true;
            float initialYawRateRadps = 0.0f;
            float maxAbsForwardVelocityMps = 0.0f;
            float maximumYawRateRadps = 0.0f;
            float finalYawRateRadps = 0.0f;
            float finalYawRad = 0.0f;
            float finalLeftCommand = 0.0f;
            float finalRightCommand = 0.0f;
            float finalRequestedYawRateRadps = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;

            YawRateLongRunScenario()
            {
                DriveBasePlantHarness harness;
                initialYawRateRadps = harness.runtimeState.GetYawRate();

                for (int tick = 0; tick < kYawRateTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kNaN, kTargetYawRateRadps, kNaN, 0.0f, kNaN);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();

                    leftCommandsFinite = leftCommandsFinite && std::isfinite(command.LeftCommand());
                    rightCommandsFinite = rightCommandsFinite && std::isfinite(command.RightCommand());
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    requestedYawRatePreserved =
                        requestedYawRatePreserved &&
                        std::fabs(telemetry.requestedYawRateRadps - kTargetYawRateRadps) <= 1.0e-6f;
                    forwardVelocityStayedBounded =
                        forwardVelocityStayedBounded &&
                        std::fabs(harness.runtimeState.GetForwardVelocity()) < 0.10f;
                    maxAbsForwardVelocityMps =
                        (std::max)(maxAbsForwardVelocityMps, std::fabs(harness.runtimeState.GetForwardVelocity()));
                    maximumYawRateRadps =
                        (std::max)(maximumYawRateRadps, harness.runtimeState.GetYawRate());
                    finalLeftCommand = command.LeftCommand();
                    finalRightCommand = command.RightCommand();
                    finalRequestedYawRateRadps = telemetry.requestedYawRateRadps;
                    finalTelemetryValidFlags = telemetry.telemetryValidFlags;
                }

                finalYawRateRadps = harness.runtimeState.GetYawRate();
                finalYawRad = harness.runtimeState.GetHeading();
            }
        };

        struct HeadingHoldLongRunScenario final
        {
            static constexpr int kHeadingTicks = 2000;
            static constexpr float kTargetYawRad = 0.0f;
            static constexpr float kInitialYawRad = 0.35f;

            bool leftCommandsFinite = true;
            bool rightCommandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedHeadingPreserved = true;
            bool headingErrorStayedBounded = true;
            float initialHeadingErrorRad = 0.0f;
            float finalHeadingErrorRad = 0.0f;
            float finalYawRateRadps = 0.0f;
            float maxAbsHeadingErrorRad = 0.0f;
            float finalRequestedYawRad = 0.0f;
            float totalHeadingDelta = 0.0f;
            float finalLeftCommand = 0.0f;
            float finalRightCommand = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;

            HeadingHoldLongRunScenario()
            {
                DriveBasePlantHarness harness;
                harness.runtimeState.SetHeading(kInitialYawRad);
                initialHeadingErrorRad = AbsYawErrorRad(kTargetYawRad, harness.runtimeState.GetHeading());
                maxAbsHeadingErrorRad = initialHeadingErrorRad;

                for (int tick = 0; tick < kHeadingTicks; ++tick)
                {
                    const CommandVector command =
                        harness.ProposeAndIntegrate(kNaN, 0.0f, kNaN, 0.0f, kTargetYawRad);
                    const DriveTelemetry telemetry = harness.drive.LastTelemetry();
                    const float headingErrorRad =
                        AbsYawErrorRad(kTargetYawRad, harness.runtimeState.GetHeading());

                    leftCommandsFinite = leftCommandsFinite && std::isfinite(command.LeftCommand());
                    rightCommandsFinite = rightCommandsFinite && std::isfinite(command.RightCommand());
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
                    finalLeftCommand = command.LeftCommand();
                    finalRightCommand = command.RightCommand();
                    finalRequestedYawRad = telemetry.requestedYawRad;
                    finalTelemetryValidFlags = telemetry.telemetryValidFlags;
					totalHeadingDelta += (NormalizeAngle(harness.runtimeState.GetYawRate() * 0.001f));
                }

                finalHeadingErrorRad = AbsYawErrorRad(kTargetYawRad, harness.runtimeState.GetHeading());
                finalYawRateRadps = harness.runtimeState.GetYawRate();
            }
        };
    }

    TEST_CLASS(DriveStack_DriveBaseBoundarySnapshotTest)
    {
    public:
        TEST_METHOD(LeftCommandIsFinite)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsFinite)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(LeftCommandIsClamped)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.LeftCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsClamped)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.RightCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=command_kind_flags"
                << L"\nactual=" << scenario.telemetry.commandKindFlags
                << L"\nrequired_mask=" << DriveTelemetry::kCommandKindBodyProposal;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.commandKindFlags, DriveTelemetry::kCommandKindBodyProposal),
                message.str().c_str());
        }

        TEST_METHOD(ProposalSequenceEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.telemetry.telemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryProposalSequenceValid;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryProposalSequenceValid),
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.telemetry.telemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid),
                message.str().c_str());
        }

        TEST_METHOD(LeftPlantCommandMatchesAccelerationFeedforward)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_plant_command"
                << L"\nexpected=" << scenario.expectedPlantCommand.LeftCommand()
                << L"\nactual=" << scenario.telemetry.leftPlantCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.expectedPlantCommand.LeftCommand(),
                scenario.telemetry.leftPlantCommand,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightPlantCommandMatchesAccelerationFeedforward)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_plant_command"
                << L"\nexpected=" << scenario.expectedPlantCommand.RightCommand()
                << L"\nactual=" << scenario.telemetry.rightPlantCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.expectedPlantCommand.RightCommand(),
                scenario.telemetry.rightPlantCommand,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(LeftDriveCommandMatchesReturnedCommand)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_drive_command"
                << L"\nexpected=" << scenario.command.LeftCommand()
                << L"\nactual=" << scenario.telemetry.leftDriveCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.command.LeftCommand(),
                scenario.telemetry.leftDriveCommand,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightDriveCommandMatchesReturnedCommand)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_drive_command"
                << L"\nexpected=" << scenario.command.RightCommand()
                << L"\nactual=" << scenario.telemetry.rightDriveCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.command.RightCommand(),
                scenario.telemetry.rightDriveCommand,
                1.0e-6f,
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseVelocityFeedbackTest)
    {
    public:
        TEST_METHOD(RequestedForwardMpsIsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_forward_mps"
                << L"\nexpected=0.8"
                << L"\nactual="
                << scenario.telemetry.requestedForwardMps
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.80f,
                scenario.telemetry.requestedForwardMps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRateRadpsIsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_yaw_rate_radps"
                << L"\nexpected=0.25"
                << L"\nactual="
                << scenario.telemetry.requestedYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.25f,
                scenario.telemetry.requestedYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedForwardAccelMps2IsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_forward_accel_mps2"
                << L"\nexpected=0.3"
                << L"\nactual="
                << scenario.telemetry.requestedForwardAccelMps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.30f,
                scenario.telemetry.requestedForwardAccelMps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawAccelRadps2IsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_yaw_accel_radps2"
                << L"\nexpected=0.4"
                << L"\nactual="
                << scenario.telemetry.requestedYawAccelRadps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.40f,
                scenario.telemetry.requestedYawAccelRadps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRadIsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_yaw_rad"
                << L"\nexpected=0.18"
                << L"\nactual="
                << scenario.telemetry.requestedYawRad
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.18f,
                scenario.telemetry.requestedYawRad,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ComposedForwardAccelerationUsesProductionPDOnce)
        {
            const FeedbackScenario scenario;
            const float expected =
                0.30f +
                Config::kDriveBasePDCluster.VelocityStatePD.Compute(0.80f - 0.20f, 0.0f);
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=composed_forward_accel_mps2"
                << L"\nexpected=" << expected
                << L"\nactual=" << scenario.telemetry.composedForwardAccelMps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                scenario.telemetry.composedForwardAccelMps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ComposedYawAccelerationUsesProductionPDOnce)
        {
            const FeedbackScenario scenario;
            const float expected =
                0.40f +
                Config::kDriveBasePDCluster.YawRateStatePD.Compute(0.25f - -0.15f, 0.0f) +
                Config::kDriveBasePDCluster.HeadingStatePD.Compute(0.18f - 0.10f, 0.25f - -0.15f);
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=composed_yaw_accel_radps2"
                << L"\nexpected=" << expected
                << L"\nactual=" << scenario.telemetry.composedYawAccelRadps2
                << L"\ntolerance=1e-3";

            Assert::AreEqual(
                expected,
                scenario.telemetry.composedYawAccelRadps2,
                1.0e-3f,
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseClampEvidenceTest)
    {
    public:
        TEST_METHOD(LeftCommandIsFinite)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=left_clamp_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsFinite)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=right_clamp_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(LeftCommandIsClamped)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=left_clamp_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.LeftCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsClamped)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=right_clamp_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.RightCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(PlantVsDriveClampEvidenceIsVisible)
        {
            const ClampScenario scenario;
            const float leftDelta =
                std::fabs(scenario.telemetry.leftPlantCommand - scenario.telemetry.leftDriveCommand);
            const float rightDelta =
                std::fabs(scenario.telemetry.rightPlantCommand - scenario.telemetry.rightDriveCommand);
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=plant_vs_drive_clamp_delta"
                << L"\nleft_delta=" << leftDelta
                << L"\nright_delta=" << rightDelta
                << L"\ncriterion=left_delta>1e-5||right_delta>1e-5"
                << L"\nleft_plant=" << scenario.telemetry.leftPlantCommand
                << L"\nleft_drive=" << scenario.telemetry.leftDriveCommand
                << L"\nright_plant=" << scenario.telemetry.rightPlantCommand
                << L"\nright_drive=" << scenario.telemetry.rightDriveCommand;

            Assert::IsTrue(
                (leftDelta > 1.0e-5f) ||
                (rightDelta > 1.0e-5f),
                message.str().c_str());
        }

        TEST_METHOD(PlantCommandTelemetryFlagIsSet)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.telemetry.telemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryPlantCommandValid;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryPlantCommandValid),
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseSolverFailureEvidenceTest)
    {
    public:
        TEST_METHOD(LeftCommandFallsBackToZero)
        {
            const SolverFailureScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=solver_failure_left_command"
                << L"\nexpected=0"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                scenario.command.LeftCommand(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandFallsBackToZero)
        {
            const SolverFailureScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=solver_failure_right_command"
                << L"\nexpected=0"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                scenario.command.RightCommand(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(CommandKindFlagIsSet)
        {
            const SolverFailureScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=command_kind_flags"
                << L"\nactual=" << scenario.telemetry.commandKindFlags
                << L"\nrequired_mask=" << DriveTelemetry::kCommandKindSolverFailureEvidence;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.commandKindFlags, DriveTelemetry::kCommandKindSolverFailureEvidence),
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseAccelerationTargetLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nmax_abs_left=" << scenario.maxAbsLeftCommand
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nmax_abs_right=" << scenario.maxAbsRightCommand
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(LeftCommandsRemainClamped)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=left_command_clamp"
                << L"\nactual_max_abs=" << scenario.maxAbsLeftCommand
                << L"\ncriterion=max_abs<=1";

            Assert::IsTrue(
                scenario.leftCommandsClamped,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainClamped)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=right_command_clamp"
                << L"\nactual_max_abs=" << scenario.maxAbsRightCommand
                << L"\ncriterion=max_abs<=1";

            Assert::IsTrue(
                scenario.rightCommandsClamped,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationObjectiveIsPreserved)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=composed_forward_accel_mps2"
                << L"\nexpected=2"
                << L"\nactual_final=" << scenario.finalComposedForwardAccelMps2
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.accelerationObjectiveUnchanged,
                message.str().c_str());
        }

        TEST_METHOD(TwentyTickResponseTrendsForward)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=velocity_at_20_ticks_mps"
                << L"\ninitial=" << scenario.initialVelocityMps
                << L"\nactual=" << scenario.velocityAtMinimumHorizonMps
                << L"\ncriterion=actual>initial+0.005";

            Assert::IsTrue(
                scenario.velocityAtMinimumHorizonMps > scenario.initialVelocityMps + 0.005f,
                message.str().c_str());
        }

        TEST_METHOD(FinalResponseAccumulatesForwardVelocity)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=final_velocity_mps"
                << L"\ninitial=" << scenario.initialVelocityMps
                << L"\nactual=" << scenario.finalVelocityMps
                << L"\ncriterion=actual>initial+0.05";

            Assert::IsTrue(
                scenario.finalVelocityMps > scenario.initialVelocityMps + 0.05f,
                message.str().c_str());
        }

        TEST_METHOD(ResponseDoesNotDivergeOppositeRequest)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=minimum_velocity_mps"
                << L"\nactual=" << scenario.minimumVelocityMps
                << L"\ncriterion=actual>-0.01";

            Assert::IsTrue(
                scenario.minimumVelocityMps > -0.01f,
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseVelocityTargetLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nfinal_velocity_mps=" << scenario.finalVelocityMps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nfinal_velocity_mps=" << scenario.finalVelocityMps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(RequestedForwardTargetIsPreserved)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=requested_forward_mps"
                << L"\nexpected=" << VelocityLongRunScenario::kTargetForwardMps
                << L"\nactual_final=" << scenario.finalRequestedVelMps
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedForwardPreserved,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRateTargetIsPreserved)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=requested_yaw_rate_radps"
                << L"\nexpected=0"
                << L"\nactual_final=" << scenario.finalRequestedYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedYawRatePreserved,
                message.str().c_str());
        }

        TEST_METHOD(ResponseApproachesPositiveTarget)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=final_velocity_mps"
                << L"\nactual=" << scenario.finalVelocityMps
                << L"\ncriterion=actual>0.2"
                << L"\ntarget=" << VelocityLongRunScenario::kTargetForwardMps
                << L"\nfinal_requested_velocity=" << scenario.finalRequestedVelMps
                << L"\nfinal_requested_accel=" << scenario.finalRequestedAccelMps2;
            Assert::IsTrue(
                scenario.finalVelocityMps > 0.20f,
                message.str().c_str());
        }

        TEST_METHOD(FinalVelocitySettlesNearTarget)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=final_velocity_mps"
                << L"\nexpected=" << VelocityLongRunScenario::kTargetForwardMps
                << L"\nactual=" << scenario.finalVelocityMps
                << L"\ntolerance=0.15";

            Assert::AreEqual(
                VelocityLongRunScenario::kTargetForwardMps, scenario.finalVelocityMps, 0.15f,
                message.str().c_str());
        }

        TEST_METHOD(ResponseDoesNotDivergeWithWrongSign)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=minimum_velocity_mps"
                << L"\nactual=" << scenario.minimumVelocityMps
                << L"\ncriterion=actual>-0.02";

            Assert::IsTrue(
                scenario.minimumVelocityMps > -0.02f,
                message.str().c_str());
        }

        TEST_METHOD(ResponseDoesNotOvershootBeyondDiagnosticTolerance)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=maximum_velocity_mps"
                << L"\nactual=" << scenario.maximumVelocityMps
                << L"\ncriterion=actual<target+0.3"
                << L"\ntarget=" << VelocityLongRunScenario::kTargetForwardMps;

            Assert::IsTrue(
                scenario.maximumVelocityMps < VelocityLongRunScenario::kTargetForwardMps + 0.30f,
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseYawRateTargetLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nfinal_yaw_rate_radps=" << scenario.finalYawRateRadps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nfinal_yaw_rate_radps=" << scenario.finalYawRateRadps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRateTargetIsPreserved)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=requested_yaw_rate_radps"
                << L"\nexpected=" << YawRateLongRunScenario::kTargetYawRateRadps
                << L"\nactual_final=" << scenario.finalRequestedYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedYawRatePreserved,
                message.str().c_str());
        }

        TEST_METHOD(ResponseBuildsClockwiseYawRate)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=maximum_yaw_rate_radps"
                << L"\ninitial=" << scenario.initialYawRateRadps
                << L"\nactual=" << scenario.maximumYawRateRadps
                << L"\ncriterion=actual>initial+0.35";

            Assert::IsTrue(
                scenario.maximumYawRateRadps > scenario.initialYawRateRadps + 0.35f,
                message.str().c_str());
        }

        TEST_METHOD(FinalYawRateSettlesNearTarget)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=final_yaw_rate_radps"
                << L"\nexpected=" << YawRateLongRunScenario::kTargetYawRateRadps
                << L"\nactual=" << scenario.finalYawRateRadps
                << L"\ntolerance=0.45";

            Assert::AreEqual(
                YawRateLongRunScenario::kTargetYawRateRadps,
                scenario.finalYawRateRadps,
                0.45f,
                message.str().c_str());
        }

        TEST_METHOD(DoesNotCreateLargeForwardDrift)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=max_abs_forward_velocity_mps"
                << L"\nactual=" << scenario.maxAbsForwardVelocityMps
                << L"\ncriterion=actual<0.1";

            Assert::IsTrue(
                scenario.forwardVelocityStayedBounded,
                message.str().c_str());
        }

    };

    TEST_CLASS(DriveStack_DriveBaseHeadingHoldLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nfinal_heading_error_rad=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nfinal_heading_error_rad=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(RequestedHeadingTargetIsPreserved)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=requested_yaw_rad"
                << L"\nexpected=" << HeadingHoldLongRunScenario::kTargetYawRad
                << L"\nactual_final=" << scenario.finalRequestedYawRad
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedHeadingPreserved,
                message.str().c_str());
        }

        TEST_METHOD(PhysicalPlantReducesHeadingError)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=final_heading_error_rad"
                << L"\ninitial=" << scenario.initialHeadingErrorRad
                << L"\nactual=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=actual<initial*0.45";

            Assert::IsTrue(
                scenario.finalHeadingErrorRad < scenario.initialHeadingErrorRad * 0.45f,
                message.str().c_str());
        }

        TEST_METHOD(HeadingErrorStaysBoundedDuringCorrection)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=max_abs_heading_error_rad"
                << L"\ninitial=" << scenario.initialHeadingErrorRad
                << L"\nactual_max=" << scenario.maxAbsHeadingErrorRad
                << L"\ncriterion=error<=initial+0.08";

            Assert::IsTrue(
                scenario.headingErrorStayedBounded,
                message.str().c_str());
        }

        TEST_METHOD(FinalYawRateIsDamped)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=final_yaw_rate_radps"
                << L"\nactual=" << scenario.finalYawRateRadps
                << L"\ncriterion=abs(actual)<0.8";

            Assert::IsTrue(
                std::fabs(scenario.finalYawRateRadps) < 0.80f,
                message.str().c_str());
        }

        TEST_METHOD(SingleTurn)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=total heading traversal"
                << L"\nactual=" << scenario.totalHeadingDelta
				<< L"\ncriterion=" << (scenario.kTargetYawRad - scenario.kInitialYawRad)
                << L"\ntolerance=1e-3";

			Assert::AreEqual(
				scenario.kTargetYawRad - scenario.kInitialYawRad,
				scenario.totalHeadingDelta,
				1.0e-3f,
				message.str().c_str());
        }
    };
}
