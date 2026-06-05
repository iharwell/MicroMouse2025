#pragma once

#include "..\MazeMap\CommandVector.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\EigenCompat.h"
#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>

namespace MazeMap
{
    namespace DriveStackPlantModelPhysicsTestSupport
    {
        inline constexpr float kDirectDtSeconds = 0.001f;

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
            state.SetWheelSpeedLeft(leftWheelSpeedRadps);
            state.SetWheelSpeedRight(rightWheelSpeedRadps);
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
