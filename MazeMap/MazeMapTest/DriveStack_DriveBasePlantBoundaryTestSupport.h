#pragma once

#include "..\MazeMap\CoreConfig.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\EigenCompat.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace MazeMap
{
    namespace DriveStackDriveBasePlantBoundaryTestSupport
    {
        using CommandVector = MazeMap::App::Internal::CommandVector;

        inline constexpr float kNaN = (std::numeric_limits<float>::quiet_NaN)();
        inline constexpr float kInf = (std::numeric_limits<float>::infinity)();
        inline constexpr float kDtSeconds = 0.001f;

        inline void SetRollingWheelState(Vehicle& vehicle, VehicleState& runtimeState)
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

        inline bool IsFlagSet(const std::uint16_t flags, const std::uint16_t flag) noexcept
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

        inline float AbsYawErrorRad(const float targetYawRad, const float actualYawRad) noexcept
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
}
