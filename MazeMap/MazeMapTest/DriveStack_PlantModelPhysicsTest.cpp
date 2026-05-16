#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\Defines.h"
#include "..\MazeMap\EncoderObs.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kDirectDtSeconds = 0.001f;

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

        VehicleState::StateVector MakeState(
            float xM,
            float yM,
            float yawRad,
            float forwardVelocityMps,
            float lateralVelocityMps,
            float yawRateRadps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps) noexcept
        {
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kPx) = xM;
            state(VehicleState::kPy) = yM;
            state(VehicleState::kPsi) = yawRad;
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kV) = lateralVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = rightWheelSpeedRadps;
            VehicleState::NormalizeStateVector(state);
            return state;
        }

        VehicleState::StateVector MakeRollingState(
            const PlantParams& params,
            float forwardVelocityMps,
            float yawRateRadps,
            float lateralVelocityMps = 0.0f,
            float yawRad = 0.0f) noexcept
        {
            const float leftWheelMps =
                forwardVelocityMps + (0.5f * params.trackWidthM * yawRateRadps);
            const float rightWheelMps =
                forwardVelocityMps - (0.5f * params.trackWidthM * yawRateRadps);
            return MakeState(
                0.0f,
                0.0f,
                yawRad,
                forwardVelocityMps,
                lateralVelocityMps,
                yawRateRadps,
                leftWheelMps / params.wheelRadiusM,
                rightWheelMps / params.wheelRadiusM);
        }

        App::Internal::CommandVector MakeCommand(float left, float right) noexcept
        {
            App::Internal::CommandVector command{};
            command.SetLeftCommand(left);
            command.SetRightCommand(right);
            return command;
        }

        void ApplyStateVectorToRuntime(
            VehicleState& runtimeState,
            const VehicleState::StateVector& state) noexcept
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

        App::Internal::CommandVector SolveAccelerationFeedforwardAt(
            TestRuntime& runtime,
            const VehicleState::StateVector& state,
            float forwardAccelMps2,
            float yawAccelRadps2) noexcept
        {
            ApplyStateVectorToRuntime(runtime.runtimeState, state);
            return runtime.plant.ComputeFeedforward(forwardAccelMps2, yawAccelRadps2);
        }

        float SumNormalLoadN(const ContactForces& forces) noexcept
        {
            float sum = 0.0f;
            for (const ContactForce& contact : forces.contacts)
            {
                sum += contact.normalForceN;
            }
            return sum;
        }

        float NoOpDeltaForDt(float dtSeconds, int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.70f, 1.50f, 0.04f, 0.20f);
            const App::Internal::CommandVector command = MakeCommand(0.42f, 0.31f);
            const VehicleState::StateVector actual =
                runtime.plant.integrate(state, command, dtSeconds, params);
            return actual(component) - state(component);
        }

        float StateComponentAfterSingleStep(int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.70f, 1.50f, 0.04f, 0.20f);
            const App::Internal::CommandVector command = MakeCommand(0.42f, 0.31f);
            const VehicleState::StateVector actual =
                runtime.plant.integrate(state, command, 0.004f, params);
            return actual(component);
        }

        float StateComponentAfterSubsteps(int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeRollingState(params, 0.70f, 1.50f, 0.04f, 0.20f);
            const App::Internal::CommandVector command = MakeCommand(0.42f, 0.31f);
            for (int step = 0; step < 4; ++step)
            {
                state = runtime.plant.integrate(state, command, 0.001f, params);
            }
            return state(component);
        }

        float SingleStepMinusSubsteps(int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                MakeRollingState(params, 0.70f, 1.50f, 0.04f, 0.20f);
            const App::Internal::CommandVector command = MakeCommand(0.42f, 0.31f);
            const VehicleState::StateVector singleStep =
                runtime.plant.integrate(initialState, command, 0.004f, params);

            VehicleState::StateVector substeps = initialState;
            for (int step = 0; step < 4; ++step)
            {
                substeps = runtime.plant.integrate(substeps, command, 0.001f, params);
            }

            return singleStep(component) - substeps(component);
        }

        float HighCommandFirstNonFiniteOrFinalComponent(int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeRollingState(params, 1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);

            for (int tick = 0; tick < 250; ++tick)
            {
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
                if (!std::isfinite(state(component)))
                {
                    return state(component);
                }
            }

            return state(component);
        }

        float HighCommandFirstOutOfRangeOrFinalHeading()
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeRollingState(params, 1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);

            for (int tick = 0; tick < 250; ++tick)
            {
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
                if (state(VehicleState::kPsi) < -PI_F || state(VehicleState::kPsi) > PI_F)
                {
                    return state(VehicleState::kPsi);
                }
            }

            return state(VehicleState::kPsi);
        }

        VehicleState::StateVector FinalHighCommandState()
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeRollingState(params, 1.25f, 4.0f, 0.15f, 0.30f);
            const App::Internal::CommandVector command = MakeCommand(0.85f, 0.20f);

            for (int tick = 0; tick < 250; ++tick)
            {
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
            }

            return state;
        }

        App::Internal::CommandVector LongRunForwardFirstNonFiniteOrFinalSolveCommand()
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = MakeRollingState(params, 0.30f, 0.0f);
            App::Internal::CommandVector command{};

            for (int tick = 0; tick < 500; ++tick)
            {
                command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                if (!command.IsFinite())
                {
                    return command;
                }
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
            }

            return command;
        }

        float LongRunForwardFirstNonFiniteOrFinalStateComponent(int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = MakeRollingState(params, 0.30f, 0.0f);

            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
                if (!std::isfinite(state(component)))
                {
                    return state(component);
                }
            }

            return state(component);
        }

        float LongRunForwardVelocityDelta()
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = MakeRollingState(params, 0.30f, 0.0f);
            const float initialForwardMps = state(VehicleState::kU);

            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
            }

            return state(VehicleState::kU) - initialForwardMps;
        }
    }

    TEST_CLASS(DriveStack_PlantModelPhysicsTest)
    {
    public:
        TEST_METHOD(AxisConvention_HeadingZeroForwardMovesWorldPositiveY)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector coast{};
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.50f, 0.0f, 0.0f, 0.0f);
            const VehicleState::StateVector integrated =
                runtime.plant.integrate(state, coast, 0.004f, params);
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_y_m"
                << L"\ninitial=" << state(VehicleState::kPy)
                << L"\nactual=" << integrated(VehicleState::kPy)
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated(VehicleState::kPy) > state(VehicleState::kPy),
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_HeadingZeroForwardDoesNotDriftWorldX)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector coast{};
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.50f, 0.0f, 0.0f, 0.0f);
            const VehicleState::StateVector integrated =
                runtime.plant.integrate(state, coast, 0.004f, params);
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_x_m"
                << L"\nexpected=" << state(VehicleState::kPx)
                << L"\nactual=" << integrated(VehicleState::kPx)
                << L"\ntolerance=2e-4";

            Assert::AreEqual(
                state(VehicleState::kPx),
                integrated(VehicleState::kPx),
                2.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_HeadingRightForwardMovesWorldPositiveX)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector coast{};
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.50f, 0.0f, 0.0f, 0.5f * PI_F);
            const VehicleState::StateVector integrated =
                runtime.plant.integrate(state, coast, 0.004f, params);
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_x_m"
                << L"\ninitial=" << state(VehicleState::kPx)
                << L"\nactual=" << integrated(VehicleState::kPx)
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated(VehicleState::kPx) > state(VehicleState::kPx),
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_HeadingRightForwardDoesNotDriftWorldY)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector coast{};
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.50f, 0.0f, 0.0f, 0.5f * PI_F);
            const VehicleState::StateVector integrated =
                runtime.plant.integrate(state, coast, 0.004f, params);
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_y_m"
                << L"\nexpected=" << state(VehicleState::kPy)
                << L"\nactual=" << integrated(VehicleState::kPy)
                << L"\ntolerance=2e-4";

            Assert::AreEqual(
                state(VehicleState::kPy),
                integrated(VehicleState::kPy),
                2.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_BodyPositiveLateralMovesWorldPositiveX)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector coast{};
            const VehicleState::StateVector state =
                MakeState(0.0f, 0.0f, 0.0f, 0.0f, 0.30f, 0.0f, 0.0f, 0.0f);
            const VehicleState::StateVector integrated =
                runtime.plant.integrate(state, coast, 0.004f, params);
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=world_x_m"
                << L"\ninitial=" << state(VehicleState::kPx)
                << L"\nactual=" << integrated(VehicleState::kPx)
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated(VehicleState::kPx) > state(VehicleState::kPx),
                message.str().c_str());
        }

        TEST_METHOD(AxisConvention_PositiveYawRateIncreasesClockwiseYaw)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector coast{};
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.25f, 1.20f);
            const VehicleState::StateVector integrated =
                runtime.plant.integrate(state, coast, 0.004f, params);
            std::wstringstream message;
            message << L"PM20_AXIS_CONVENTION"
                << L"\nfield=yaw_rad"
                << L"\ninitial=" << state(VehicleState::kPsi)
                << L"\nactual=" << integrated(VehicleState::kPsi)
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                integrated(VehicleState::kPsi) > state(VehicleState::kPsi),
                message.str().c_str());
        }

        TEST_METHOD(WheelYawSign_PositiveYawMakesLeftWheelLinearVelocityFaster)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.40f, 2.0f);
            const Eigen::Vector2f wheelLinearMps =
                runtime.plant.wheelLinearVelocityFromBodyState(state);
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=wheel_linear_velocity_mps"
                << L"\nleft=" << wheelLinearMps.x()
                << L"\nright=" << wheelLinearMps.y()
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                wheelLinearMps.x() > wheelLinearMps.y(),
                message.str().c_str());
        }

        TEST_METHOD(WheelYawSign_EncoderYawMeasurementPreservesClockwisePositiveSign)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.40f, 2.0f);
            EncoderObs observation{};
            observation.omegaLeftRadps = state(VehicleState::kOmegaL);
            observation.omegaRightRadps = state(VehicleState::kOmegaR);
            const float actualYawRateRadps = runtime.plant.measuredYawRateRadps(observation);
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=measured_yaw_rate_radps"
                << L"\nactual=" << actualYawRateRadps
                << L"\ncriterion=actual>0"
                << L"\nleft_omega_radps=" << observation.omegaLeftRadps
                << L"\nright_omega_radps=" << observation.omegaRightRadps;

            Assert::IsTrue(
                actualYawRateRadps > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(WheelYawSign_PositiveTargetYawRateRequestsFasterLeftWheel)
        {
            TestRuntime runtime;
            PlantModel::StateVector state = PlantModel::StateVector::Zero();
            state(VehicleState::kU) = 0.40f;
            state(VehicleState::kR) = 2.0f;
            const Eigen::Vector2f wheelLinearMps =
                runtime.plant.wheelLinearVelocityFromBodyState(state);
            std::wstringstream message;
            message << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=target_wheel_linear_velocity_mps"
                << L"\nleft=" << wheelLinearMps.x()
                << L"\nright=" << wheelLinearMps.y()
                << L"\ncriterion=left>right";

            Assert::IsTrue(
                wheelLinearMps.x() > wheelLinearMps.y(),
                message.str().c_str());
        }

        TEST_METHOD(WheelYawSign_PositiveTargetYawAccelerationRequestsMoreLeftAcceleration)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.40f, 2.0f);
            const App::Internal::CommandVector command =
                runtime.plant.ComputeFeedforward(0.0f, 5.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, command, params);
            std::wstringstream commandMessage;
            commandMessage << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=command_differential"
                << L"\nactual=" << command.Differential()
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand();
            std::wstringstream accelMessage;
            accelMessage << L"PM20_WHEEL_YAW_SIGN"
                << L"\nfield=yaw_accel_radps2"
                << L"\nactual=" << derivatives.yawAccelRadps2
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                command.Differential() > 0.0f,
                commandMessage.str().c_str());
            Assert::IsTrue(
                derivatives.yawAccelRadps2 > 0.0f,
                accelMessage.str().c_str());
        }

        TEST_METHOD(SymmetricDrive_LeftRightForwardForceSymmetry)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);
            const float leftForceN = derivatives.contactForces.LeftBankForwardForceN();
            const float rightForceN = derivatives.contactForces.RightBankForwardForceN();
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=bank_forward_force_n"
                << L"\nexpected_left=" << leftForceN
                << L"\nactual_right=" << rightForceN
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                leftForceN,
                rightForceN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricDrive_WheelAccelerationSymmetry)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);
            const float leftWheelAccelRadps2 = derivatives.stateDot(VehicleState::kOmegaL);
            const float rightWheelAccelRadps2 = derivatives.stateDot(VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=wheel_accel_radps2"
                << L"\nexpected_left=" << leftWheelAccelRadps2
                << L"\nactual_right=" << rightWheelAccelRadps2
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                leftWheelAccelRadps2,
                rightWheelAccelRadps2,
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricDrive_NoYawAccelerationBias)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=yaw_accel_radps2"
                << L"\nexpected=0"
                << L"\nactual=" << derivatives.stateDot(VehicleState::kR)
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                0.0f,
                derivatives.stateDot(VehicleState::kR),
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricDrive_NoLateralAccelerationBias)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);
            std::wstringstream message;
            message << L"PM21_FORCE_SYMMETRY"
                << L"\nfield=lateral_accel_mps2"
                << L"\nexpected=0"
                << L"\nactual=" << derivatives.lateralAccelMps2
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                0.0f,
                derivatives.lateralAccelMps2,
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_SubthresholdDriveDoesNotAccelerateLeftWheelAtRest)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.25f, 0.25f), params);
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=left_wheel_accel_radps2"
                << L"\nexpected=0"
                << L"\nactual=" << derivatives.stateDot(VehicleState::kOmegaL)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                derivatives.stateDot(VehicleState::kOmegaL),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_SubthresholdDriveDoesNotAccelerateRightWheelAtRest)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.25f, 0.25f), params);
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=right_wheel_accel_radps2"
                << L"\nexpected=0"
                << L"\nactual=" << derivatives.stateDot(VehicleState::kOmegaR)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                derivatives.stateDot(VehicleState::kOmegaR),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftPositionX)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kPx);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=position_x_m"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kPx)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kPx),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftPositionY)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kPy);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=position_y_m"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kPy)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kPy),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftYaw)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kPsi);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=yaw_rad"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kPsi)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kPsi),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftForwardVelocity)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kU);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=forward_velocity_mps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kU)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kU),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftLateralVelocity)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kV);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=lateral_velocity_mps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kV)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kV),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftYawRate)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kR);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=yaw_rate_radps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kR)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kR),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftLeftWheelSpeed)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kOmegaL);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kOmegaL)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kOmegaL),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(Stiction_DirectIntegrationDoesNotDriftRightWheelSpeed)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const float initial = state(VehicleState::kOmegaR);
            for (int tick = 0; tick < 100; ++tick)
            {
                state = runtime.plant.integrate(state, MakeCommand(0.25f, 0.25f), kDirectDtSeconds, params);
            }
            std::wstringstream message;
            message << L"PM21_STICTION"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nexpected=" << initial
                << L"\nactual=" << state(VehicleState::kOmegaR)
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                initial,
                state(VehicleState::kOmegaR),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_NoFanContactNormalSumMatchesConfiguredLoad)
        {
            TestRuntime runtime(0.0f);
            const PlantParams params = PlantParams::Default();
            const ContactForces forces =
                runtime.plant.tireForces(MakeRollingState(params, 0.75f, 0.0f), params);
            const float expectedLoadN = params.TotalNormalLoadN(0.0f);
            const float actualLoadN = SumNormalLoadN(forces);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_normal_sum_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << actualLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=0";

            Assert::AreEqual(
                expectedLoadN,
                actualLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_FanOnContactNormalSumMatchesConfiguredLoad)
        {
            TestRuntime runtime(0.80f);
            const PlantParams params = PlantParams::Default();
            const ContactForces forces =
                runtime.plant.tireForces(MakeRollingState(params, 0.75f, 0.0f), params);
            const float expectedLoadN = params.TotalNormalLoadN(0.80f);
            const float actualLoadN = SumNormalLoadN(forces);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_normal_sum_n"
                << L"\nexpected=" << expectedLoadN
                << L"\nactual=" << actualLoadN
                << L"\ntolerance=1e-5"
                << L"\nfan_duty=0.8";

            Assert::AreEqual(
                expectedLoadN,
                actualLoadN,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_FanDutyIncreasesTotalContactNormalLoad)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);
            const float fanOffLoadN = SumNormalLoadN(fanOffForces);
            const float fanOnLoadN = SumNormalLoadN(fanOnForces);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=total_contact_normal_load_n"
                << L"\nfan_off=" << fanOffLoadN
                << L"\nfan_on=" << fanOnLoadN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnLoadN > fanOffLoadN,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_Contact0NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_0_normal_force_n"
                << L"\nfan_off=" << fanOffForces.contacts[0].normalForceN
                << L"\nfan_on=" << fanOnForces.contacts[0].normalForceN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnForces.contacts[0].normalForceN > fanOffForces.contacts[0].normalForceN,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_Contact1NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_1_normal_force_n"
                << L"\nfan_off=" << fanOffForces.contacts[1].normalForceN
                << L"\nfan_on=" << fanOnForces.contacts[1].normalForceN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnForces.contacts[1].normalForceN > fanOffForces.contacts[1].normalForceN,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_Contact2NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_2_normal_force_n"
                << L"\nfan_off=" << fanOffForces.contacts[2].normalForceN
                << L"\nfan_on=" << fanOnForces.contacts[2].normalForceN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnForces.contacts[2].normalForceN > fanOffForces.contacts[2].normalForceN,
                message.str().c_str());
        }

        TEST_METHOD(FanLoad_Contact3NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);
            std::wstringstream message;
            message << L"PM21_FAN_LOAD"
                << L"\nfield=contact_3_normal_force_n"
                << L"\nfan_off=" << fanOffForces.contacts[3].normalForceN
                << L"\nfan_on=" << fanOnForces.contacts[3].normalForceN
                << L"\ncriterion=fan_on>fan_off";

            Assert::IsTrue(
                fanOnForces.contacts[3].normalForceN > fanOffForces.contacts[3].normalForceN,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangePositionX)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangePositionY)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kPy);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeYaw)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kPsi);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_delta_rad"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeForwardVelocity)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kU);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_delta_mps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeLateralVelocity)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kV);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=lateral_velocity_delta_mps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeYawRate)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_delta_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeLeftWheelSpeed)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kOmegaL);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_wheel_speed_delta_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeRightWheelSpeed)
        {
            const float actualDelta = NoOpDeltaForDt(0.0f, VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_wheel_speed_delta_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=0";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_NegativeDtDoesNotChangePositionX)
        {
            const float actualDelta = NoOpDeltaForDt(-0.001f, VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=-0.001";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_NonFiniteDtDoesNotChangePositionX)
        {
            const float actualDelta =
                NoOpDeltaForDt((std::numeric_limits<float>::quiet_NaN)(), VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_delta_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0"
                << L"\ndt_seconds=nan";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                0.0f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepPositionXStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepPositionYStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kPy);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepYawStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kPsi);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepForwardVelocityStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kU);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepLateralVelocityStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kV);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepYawRateStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepLeftWheelSpeedStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kOmegaL);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SingleStepRightWheelSpeedStaysFinite)
        {
            const float actual = StateComponentAfterSingleStep(VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\ndt_seconds=0.004";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepPositionXStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepPositionYStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kPy);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepYawStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kPsi);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepForwardVelocityStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kU);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepLateralVelocityStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kV);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepYawRateStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepLeftWheelSpeedStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kOmegaL);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_SubstepRightWheelSpeedStaysFinite)
        {
            const float actual = StateComponentAfterSubsteps(VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)"
                << L"\nsubsteps=4"
                << L"\ndt_seconds=0.001";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_PositionXRemainsCloseBetweenSingleStepAndSubsteps)
        {
            const float actualDelta = SingleStepMinusSubsteps(VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_x_single_minus_substeps_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.003";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                3.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_PositionYRemainsCloseBetweenSingleStepAndSubsteps)
        {
            const float actualDelta = SingleStepMinusSubsteps(VehicleState::kPy);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=position_y_single_minus_substeps_m"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.003";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                3.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_ForwardVelocityRemainsCloseBetweenSingleStepAndSubsteps)
        {
            const float actualDelta = SingleStepMinusSubsteps(VehicleState::kU);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=forward_velocity_single_minus_substeps_mps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.08";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                8.0e-2f,
                message.str().c_str());
        }

        TEST_METHOD(IntegrateDirect_YawRateRemainsCloseBetweenSingleStepAndSubsteps)
        {
            const float actualDelta = SingleStepMinusSubsteps(VehicleState::kR);
            std::wstringstream message;
            message << L"PM22_INTEGRATE_DIRECT"
                << L"\nfield=yaw_rate_single_minus_substeps_radps"
                << L"\nexpected=0"
                << L"\nactual=" << actualDelta
                << L"\ntolerance=0.8";

            Assert::AreEqual(
                0.0f,
                actualDelta,
                8.0e-1f,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_PositionXStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kPx);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=position_x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_PositionYStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kPy);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=position_y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_YawStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kPsi);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_ForwardVelocityStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kU);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_LateralVelocityStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kV);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_YawRateStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kR);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_LeftWheelSpeedStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kOmegaL);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_RightWheelSpeedStaysFiniteUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstNonFiniteOrFinalComponent(VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_HeadingStaysNormalizedUnderPlausibleHighCommand)
        {
            const float actual = HighCommandFirstOutOfRangeOrFinalHeading();
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=yaw_rad"
                << L"\nactual=" << actual
                << L"\nminimum=" << -PI_F
                << L"\nmaximum=" << PI_F
                << L"\ncriterion=minimum<=actual<=maximum";

            Assert::IsTrue(
                actual >= -PI_F && actual <= PI_F,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_FinalForwardVelocityStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();
            const float actual = state(VehicleState::kU);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<20";

            Assert::IsTrue(
                std::fabs(actual) < 20.0f,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_FinalYawRateStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();
            const float actual = state(VehicleState::kR);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<200";

            Assert::IsTrue(
                std::fabs(actual) < 200.0f,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_FinalLeftWheelSpeedStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();
            const float actual = state(VehicleState::kOmegaL);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<3000";

            Assert::IsTrue(
                std::fabs(actual) < 3000.0f,
                message.str().c_str());
        }

        TEST_METHOD(NumericStability_FinalRightWheelSpeedStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();
            const float actual = state(VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM22_NUMERIC_STABILITY"
                << L"\nfield=final_right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=abs(actual)<3000";

            Assert::IsTrue(
                std::fabs(actual) < 3000.0f,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ForwardAccelerationCommandIsFinite)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(params, 0.60f, 0.0f),
                    0.80f,
                    0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=forward_accel_feedforward_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ReverseAccelerationCommandIsFinite)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(params, 0.60f, 0.0f),
                    -0.80f,
                    0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=reverse_accel_feedforward_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_ClockwiseYawCommandIsFinite)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector command =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(params, 0.60f, 0.0f),
                    0.0f,
                    8.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=yaw_accel_feedforward_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_PositiveForwardAccelerationCommandsMoreAverageThanReverse)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.60f, 0.0f);
            const App::Internal::CommandVector forward =
                SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
            const App::Internal::CommandVector reverse =
                SolveAccelerationFeedforwardAt(runtime, state, -0.80f, 0.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=average_command"
                << L"\nforward_average=" << forward.Average()
                << L"\nreverse_average=" << reverse.Average()
                << L"\ncriterion=forward_average>reverse_average";

            Assert::IsTrue(
                forward.Average() > reverse.Average(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_PositiveClockwiseYawCommandsLeftGreaterThanRight)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const App::Internal::CommandVector clockwise =
                SolveAccelerationFeedforwardAt(
                    runtime,
                    MakeRollingState(params, 0.60f, 0.0f),
                    0.0f,
                    8.0f);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=clockwise_command_differential"
                << L"\nactual=" << clockwise.Differential()
                << L"\ncriterion=actual>0"
                << L"\nleft_command=" << clockwise.LeftCommand()
                << L"\nright_command=" << clockwise.RightCommand();

            Assert::IsTrue(
                clockwise.Differential() > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunForwardSolveCommandStaysFinite)
        {
            const App::Internal::CommandVector command =
                LongRunForwardFirstNonFiniteOrFinalSolveCommand();
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_solve_command"
                << L"\nleft_command=" << command.LeftCommand()
                << L"\nright_command=" << command.RightCommand()
                << L"\ncriterion=isfinite(left)&&isfinite(right)";

            Assert::IsTrue(
                command.IsFinite(),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositionXStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kPx);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_position_x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositionYStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kPy);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_position_y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunYawStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kPsi);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_yaw_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunForwardVelocityStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kU);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunLateralVelocityStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kV);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunYawRateStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kR);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunLeftWheelSpeedStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kOmegaL);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunRightWheelSpeedStaysFinite)
        {
            const float actual = LongRunForwardFirstNonFiniteOrFinalStateComponent(VehicleState::kOmegaR);
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(actual),
                message.str().c_str());
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositiveAccelerationIncreasesForwardVelocity)
        {
            const float actualDelta = LongRunForwardVelocityDelta();
            std::wstringstream message;
            message << L"PM23_INVERSE_SIGN"
                << L"\nfield=long_run_forward_velocity_delta_mps"
                << L"\nactual=" << actualDelta
                << L"\ncriterion=actual>0.05";

            Assert::IsTrue(
                actualDelta > 0.05f,
                message.str().c_str());
        }
    };
}
