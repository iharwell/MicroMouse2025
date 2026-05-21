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
            float maxAbsLeftCommand = 0.0f;
            float maxAbsRightCommand = 0.0f;
            float finalComposedForwardAccelMps2 = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;

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
                    maxAbsLeftCommand = (std::max)(maxAbsLeftCommand, std::fabs(command.LeftCommand()));
                    maxAbsRightCommand = (std::max)(maxAbsRightCommand, std::fabs(command.RightCommand()));
                    commandEvidenceValid =
                        commandEvidenceValid &&
                        IsFlagSet(telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid);
                    accelerationObjectiveUnchanged =
                        accelerationObjectiveUnchanged &&
                        std::fabs(telemetry.composedForwardAccelMps2 - 2.0f) <= 1.0e-6f;
                    finalComposedForwardAccelMps2 = telemetry.composedForwardAccelMps2;
                    finalTelemetryValidFlags = telemetry.telemetryValidFlags;

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
            float finalRequestedYawRateRadps = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;
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
                finalRequestedYawRateRadps = harness.drive.LastTelemetry().requestedYawRateRadps;
                finalTelemetryValidFlags = harness.drive.LastTelemetry().telemetryValidFlags;
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
            static constexpr float kTargetYawRateRadps = 2.20f;

            bool commandsFinite = true;
            bool commandEvidenceValid = true;
            bool requestedYawRatePreserved = true;
            bool forwardVelocityStayedBounded = true;
            float initialYawRateRadps = 0.0f;
            float maxAbsForwardVelocityMps = 0.0f;
            float maximumYawRateRadps = 0.0f;
            float finalYawRateRadps = 0.0f;
            float finalYawRad = 0.0f;
            float finalRequestedYawRateRadps = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;

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
                    maxAbsForwardVelocityMps =
                        (std::max)(maxAbsForwardVelocityMps, std::fabs(harness.runtimeState.GetVelocity()));
                    maximumYawRateRadps =
                        (std::max)(maximumYawRateRadps, harness.runtimeState.GetRotationalVelocity());
                    finalRequestedYawRateRadps = telemetry.requestedYawRateRadps;
                    finalTelemetryValidFlags = telemetry.telemetryValidFlags;
                }

                finalYawRateRadps = harness.runtimeState.GetRotationalVelocity();
                finalYawRad = harness.runtimeState.GetOrientation();
            }
        };

        struct HeadingHoldLongRunScenario final
        {
            static constexpr int kHeadingTicks = 2000;
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
            float finalRequestedYawRad = 0.0f;
            float totalHeadingDelta = 0.0f;
            std::uint16_t finalTelemetryValidFlags = 0U;

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
                    finalRequestedYawRad = telemetry.requestedYawRad;
                    finalTelemetryValidFlags = telemetry.telemetryValidFlags;
					totalHeadingDelta += (VehicleState::NormalizeAngle(harness.runtimeState.GetRotationalVelocity() * 0.001f));
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
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=command_finite"
                << L"\nactual_left=" << scenario.command.LeftCommand()
                << L"\nactual_right=" << scenario.command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                scenario.command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_LeftCommandIsClamped)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_RightCommandIsClamped)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_BodyProposalEvidenceIsSet)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_ProposalSequenceEvidenceIsSet)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_CommandEvidenceIsSet)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_LeftPlantCommandMatchesAccelerationFeedforward)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_RightPlantCommandMatchesAccelerationFeedforward)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_LeftDriveCommandMatchesReturnedCommand)
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

        TEST_METHOD(ProposeBodyTick_BoundarySnapshot_RightDriveCommandMatchesReturnedCommand)
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

        TEST_METHOD(VelocityFeedback_RequestedForwardMpsIsPreserved)
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

        TEST_METHOD(VelocityFeedback_RequestedYawRateRadpsIsPreserved)
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

        TEST_METHOD(VelocityFeedback_RequestedForwardAccelMps2IsPreserved)
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

        TEST_METHOD(VelocityFeedback_RequestedYawAccelRadps2IsPreserved)
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

        TEST_METHOD(VelocityFeedback_RequestedYawRadIsPreserved)
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

        TEST_METHOD(VelocityFeedback_ComposedForwardAccelerationUsesProductionPDOnce)
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

        TEST_METHOD(VelocityFeedback_ComposedYawAccelerationUsesProductionPDOnce)
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

        TEST_METHOD(ClampEvidence_CommandIsFinite)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=clamp_command_finite"
                << L"\nactual_left=" << scenario.command.LeftCommand()
                << L"\nactual_right=" << scenario.command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                scenario.command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(ClampEvidence_LeftCommandIsClamped)
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

        TEST_METHOD(ClampEvidence_RightCommandIsClamped)
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

        TEST_METHOD(ClampEvidence_PlantVsDriveClampEvidenceIsVisible)
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

        TEST_METHOD(ClampEvidence_PlantCommandTelemetryFlagIsSet)
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

        TEST_METHOD(SolverFailureEvidence_LeftCommandFallsBackToZero)
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

        TEST_METHOD(SolverFailureEvidence_RightCommandFallsBackToZero)
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

        TEST_METHOD(SolverFailureEvidence_CommandKindFlagIsSet)
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

        TEST_METHOD(AccelerationTargetLongRun_CommandsRemainFinite)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=command_finite"
                << L"\nactual=" << scenario.commandsFinite
                << L"\nmax_abs_left=" << scenario.maxAbsLeftCommand
                << L"\nmax_abs_right=" << scenario.maxAbsRightCommand
                << L"\ncriterion=all commands finite";

            Assert::IsTrue(
                scenario.commandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationTargetLongRun_LeftCommandsRemainClamped)
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

        TEST_METHOD(AccelerationTargetLongRun_RightCommandsRemainClamped)
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

        TEST_METHOD(AccelerationTargetLongRun_CommandEvidenceRemainsVisible)
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

        TEST_METHOD(AccelerationTargetLongRun_AccelerationObjectiveIsPreserved)
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

        TEST_METHOD(AccelerationTargetLongRun_TwentyTickResponseTrendsForward)
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

        TEST_METHOD(AccelerationTargetLongRun_FinalResponseAccumulatesForwardVelocity)
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

        TEST_METHOD(AccelerationTargetLongRun_ResponseDoesNotDivergeOppositeRequest)
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

        TEST_METHOD(VelocityTargetLongRun_CommandsRemainFinite)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=command_finite"
                << L"\nactual=" << scenario.commandsFinite
                << L"\nfinal_velocity_mps=" << scenario.finalVelocityMps
                << L"\ncriterion=all commands finite";

            Assert::IsTrue(
                scenario.commandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(VelocityTargetLongRun_CommandEvidenceRemainsVisible)
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

        TEST_METHOD(VelocityTargetLongRun_RequestedForwardTargetIsPreserved)
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

        TEST_METHOD(VelocityTargetLongRun_RequestedYawRateTargetIsPreserved)
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

        TEST_METHOD(VelocityTargetLongRun_ResponseApproachesPositiveTarget)
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

        TEST_METHOD(VelocityTargetLongRun_FinalVelocitySettlesNearTarget)
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

        TEST_METHOD(VelocityTargetLongRun_ResponseDoesNotDivergeWithWrongSign)
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

        TEST_METHOD(VelocityTargetLongRun_ResponseDoesNotOvershootBeyondDiagnosticTolerance)
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

        TEST_METHOD(YawRateTargetLongRun_CommandsRemainFinite)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=command_finite"
                << L"\nactual=" << scenario.commandsFinite
                << L"\nfinal_yaw_rate_radps=" << scenario.finalYawRateRadps
                << L"\ncriterion=all commands finite";

            Assert::IsTrue(
                scenario.commandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(YawRateTargetLongRun_CommandEvidenceRemainsVisible)
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

        TEST_METHOD(YawRateTargetLongRun_RequestedYawRateTargetIsPreserved)
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

        TEST_METHOD(YawRateTargetLongRun_ResponseBuildsClockwiseYawRate)
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

        TEST_METHOD(YawRateTargetLongRun_FinalYawRateSettlesNearTarget)
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

        TEST_METHOD(YawRateTargetLongRun_DoesNotCreateLargeForwardDrift)
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

        TEST_METHOD(HeadingHoldLongRun_CommandsRemainFinite)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=command_finite"
                << L"\nactual=" << scenario.commandsFinite
                << L"\nfinal_heading_error_rad=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=all commands finite";

            Assert::IsTrue(
                scenario.commandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(HeadingHoldLongRun_CommandEvidenceRemainsVisible)
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

        TEST_METHOD(HeadingHoldLongRun_RequestedHeadingTargetIsPreserved)
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

        TEST_METHOD(HeadingHoldLongRun_PhysicalPlantReducesHeadingError)
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

        TEST_METHOD(HeadingHoldLongRun_HeadingErrorStaysBoundedDuringCorrection)
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

        TEST_METHOD(HeadingHoldLongRun_FinalYawRateIsDamped)
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

        TEST_METHOD(HeadingHoldLongRun_SingleTurn)
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
