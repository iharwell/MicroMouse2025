#pragma once

#include "..\MazeMap\CommandVector.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\EigenCompat.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>
#include <cstdint>

namespace MazeMap
{
    namespace DriveStackPlantModelPhysicsTestSupport
    {
        inline constexpr float kDirectDtSeconds = 0.001f;
        inline constexpr float kSeedEncoderCountsPerRadps = 10000.0f;

        struct TestRuntime final
        {
            Vehicle vehicle{};
            VehicleState runtimeState{};
            PlantModel plant;

            explicit TestRuntime(float fanDuty = 0.80f) noexcept
                : plant(vehicle, runtimeState)
            {
                vehicle.SetFanDuty(fanDuty);
            }
        };

        inline std::int32_t EncoderDeltaCountsFromWheelSpeedRadps(
            const float wheelSpeedRadps,
            const float dtSeconds) noexcept
        {
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            if (!std::isfinite(wheelSpeedRadps) ||
                !std::isfinite(dtSeconds) ||
                !(dtSeconds > 0.0f) ||
                !(distancePerCountM > 0.0f))
            {
                return 0;
            }

            return static_cast<std::int32_t>(
                std::lround(
                    (Vehicle::WheelLinearVelocityFromWheelSpeed(wheelSpeedRadps) * dtSeconds) /
                    distancePerCountM));
        }

        inline float ResolveSeedEncoderObservationDtSeconds(const float dtSeconds) noexcept
        {
            if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
            {
                return dtSeconds;
            }

            const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            if (!(wheelRadiusM > 0.0f) ||
                !std::isfinite(wheelRadiusM) ||
                !(distancePerCountM > 0.0f) ||
                !std::isfinite(distancePerCountM))
            {
                return kDirectDtSeconds;
            }

            return (kSeedEncoderCountsPerRadps * distancePerCountM) / wheelRadiusM;
        }

        inline void PublishEncoderObservationForWheelSpeedsRadps(
            VehicleState& state,
            const float leftWheelSpeedRadps,
            const float rightWheelSpeedRadps,
            const float dtSeconds = 0.0f,
            const bool valid = true) noexcept
        {
            const float resolvedDtSeconds = ResolveSeedEncoderObservationDtSeconds(dtSeconds);
            const std::int32_t leftDeltaCounts =
                EncoderDeltaCountsFromWheelSpeedRadps(leftWheelSpeedRadps, resolvedDtSeconds);
            const std::int32_t rightDeltaCounts =
                EncoderDeltaCountsFromWheelSpeedRadps(rightWheelSpeedRadps, resolvedDtSeconds);
            const float leftDistanceDeltaM =
                Vehicle::DriveEncoderDistanceFromCounts(leftDeltaCounts);
            const float rightDistanceDeltaM =
                Vehicle::DriveEncoderDistanceFromCounts(rightDeltaCounts);
            const float invDtSeconds =
                (std::isfinite(resolvedDtSeconds) && (resolvedDtSeconds > 0.0f)) ?
                (1.0f / resolvedDtSeconds) :
                0.0f;

            SensorSnapshot::EncoderObs encoderObservation = SensorSnapshot{}.EncoderObservation();
            encoderObservation.SetTotalLeftCounts(leftDeltaCounts);
            encoderObservation.SetTotalRightCounts(rightDeltaCounts);
            encoderObservation.SetLeftDistanceDeltaM(leftDistanceDeltaM);
            encoderObservation.SetRightDistanceDeltaM(rightDistanceDeltaM);
            encoderObservation.SetLeftVelocityMps(leftDistanceDeltaM * invDtSeconds);
            encoderObservation.SetRightVelocityMps(rightDistanceDeltaM * invDtSeconds);
            encoderObservation.SetLeftWheelSpeedRadps(
                Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.LeftVelocityMps()));
            encoderObservation.SetRightWheelSpeedRadps(
                Vehicle::WheelSpeedFromLinearVelocity(encoderObservation.RightVelocityMps()));

            const SensorSnapshot& previousSnapshot = state.GetSensorSnapshot();
            const std::int64_t leftTotalCounts =
                previousSnapshot.LeftEncoderTotalCounts() +
                static_cast<std::int64_t>(leftDeltaCounts);
            const std::int64_t rightTotalCounts =
                previousSnapshot.RightEncoderTotalCounts() +
                static_cast<std::int64_t>(rightDeltaCounts);
            SensorSnapshot snapshot = previousSnapshot;
            snapshot.PublishEncoderObservation(
                encoderObservation,
                valid,
                leftTotalCounts,
                rightTotalCounts,
                Vehicle::DriveEncoderDistanceFromCounts(leftTotalCounts),
                Vehicle::DriveEncoderDistanceFromCounts(rightTotalCounts));
            state.SetSensorSnapshot(snapshot);
        }

        inline VehicleState MakeState(
            float xM,
            float yM,
            float yawRad,
            float forwardVelocityMps,
            float lateralVelocityMps,
            float yawRateRadps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps) noexcept
        {
            VehicleState state;
            state.SetPosition(Eigen::Vector2f(xM, yM));
            state.SetHeading(yawRad);
            state.SetForwardVelocity(forwardVelocityMps);
            state.SetRightwardVelocity(lateralVelocityMps);
            state.SetYawRate(yawRateRadps);
            PublishEncoderObservationForWheelSpeedsRadps(
                state,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            return state;
        }

        inline VehicleState MakeRollingState(
            float forwardVelocityMps,
            float yawRateRadps,
            float lateralVelocityMps = 0.0f,
            float yawRad = 0.0f) noexcept
        {
            return MakeState(
                0.0f,
                0.0f,
                yawRad,
                forwardVelocityMps,
                lateralVelocityMps,
                yawRateRadps,
                Vehicle::WheelSpeedFromLinearVelocity(
                    Vehicle::LeftWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps)),
                Vehicle::WheelSpeedFromLinearVelocity(
                    Vehicle::RightWheelLinearVelocityFromBody(forwardVelocityMps, yawRateRadps)));
        }

        inline App::Internal::CommandVector MakeCommand(float left, float right) noexcept
        {
            App::Internal::CommandVector command{};
            command.SetLeftCommand(left);
            command.SetRightCommand(right);
            return command;
        }

        inline App::Internal::CommandVector SolveAccelerationFeedforwardAt(
            TestRuntime& runtime,
            const VehicleState& state,
            float forwardAccelMps2,
            float yawAccelRadps2) noexcept
        {
            runtime.runtimeState = state;
            return runtime.plant.ComputeFeedforward(forwardAccelMps2, yawAccelRadps2);
        }

        inline float HighCommandFirstBelowMinimumOrFinalHeading()
        {
            TestRuntime runtime;
            VehicleState state =
                MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);

            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (state.GetHeading() < -PI_F)
                {
                    return state.GetHeading();
                }
            }

            return state.GetHeading();
        }

        inline float HighCommandFirstAboveMaximumOrFinalHeading()
        {
            TestRuntime runtime;
            VehicleState state =
                MakeRollingState(1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);

            for (int tick = 0; tick < 250; ++tick)
            {
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
                if (state.GetHeading() > PI_F)
                {
                    return state.GetHeading();
                }
            }

            return state.GetHeading();
        }

        inline App::Internal::CommandVector LongRunForwardFirstNonFiniteLeftOrFinalSolveCommand()
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            App::Internal::CommandVector command{};

            for (int tick = 0; tick < 500; ++tick)
            {
                command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                if (!std::isfinite(command.LeftCommand()))
                {
                    return command;
                }
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }

            return command;
        }

        inline App::Internal::CommandVector LongRunForwardFirstNonFiniteRightOrFinalSolveCommand()
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            App::Internal::CommandVector command{};

            for (int tick = 0; tick < 500; ++tick)
            {
                command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                if (!std::isfinite(command.RightCommand()))
                {
                    return command;
                }
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }

            return command;
        }

        inline float LongRunForwardVelocityDelta()
        {
            TestRuntime runtime;
            VehicleState state = MakeRollingState(0.30f, 0.0f);
            const float initialForwardMps = state.GetForwardVelocity();

            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                runtime.runtimeState = state;
                runtime.plant.integrate(command, kDirectDtSeconds);
                state = runtime.runtimeState;
            }

            return state.GetForwardVelocity() - initialForwardMps;
        }

        struct PositiveYawAccelerationStep final
        {
            VehicleState initial;
            VehicleState integrated;
            App::Internal::CommandVector command;
        };

        inline PositiveYawAccelerationStep IntegratePositiveYawAcceleration()
        {
            TestRuntime runtime;
            const VehicleState state =
                MakeRollingState(0.40f, 2.0f);
            const App::Internal::CommandVector command =
                runtime.plant.ComputeFeedforward(0.0f, 5.0f);
            runtime.runtimeState = state;
            runtime.plant.integrate(command, kDirectDtSeconds);

            return PositiveYawAccelerationStep{
                state,
                runtime.runtimeState,
                command };
        }
    }
}
