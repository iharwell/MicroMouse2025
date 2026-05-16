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

        bool StateComponentIsFiniteAfterSingleStep(int component)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.70f, 1.50f, 0.04f, 0.20f);
            const App::Internal::CommandVector command = MakeCommand(0.42f, 0.31f);
            const VehicleState::StateVector actual =
                runtime.plant.integrate(state, command, 0.004f, params);
            return std::isfinite(actual(component));
        }

        bool StateComponentIsFiniteAfterSubsteps(int component)
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
            return std::isfinite(state(component));
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

        bool HighCommandComponentStayedFinite(int component)
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
                    return false;
                }
            }

            return true;
        }

        bool HighCommandHeadingStayedNormalized()
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
                    return false;
                }
            }

            return true;
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

        bool LongRunForwardSolveCommandStayedFinite()
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = MakeRollingState(params, 0.30f, 0.0f);

            for (int tick = 0; tick < 500; ++tick)
            {
                const App::Internal::CommandVector command =
                    SolveAccelerationFeedforwardAt(runtime, state, 0.80f, 0.0f);
                if (!command.IsFinite())
                {
                    return false;
                }
                state = runtime.plant.integrate(state, command, kDirectDtSeconds, params);
            }

            return true;
        }

        bool LongRunForwardStateComponentStayedFinite(int component)
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
                    return false;
                }
            }

            return true;
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

            Assert::IsTrue(integrated(VehicleState::kPy) > state(VehicleState::kPy),
                L"PM20_AXIS_CONVENTION heading=0 forward velocity must move world +Y.");
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

            Assert::AreEqual(state(VehicleState::kPx), integrated(VehicleState::kPx), 2.0e-4f,
                L"PM20_AXIS_CONVENTION heading=0 forward velocity must not drift in +X.");
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

            Assert::IsTrue(integrated(VehicleState::kPx) > state(VehicleState::kPx),
                L"PM20_AXIS_CONVENTION heading=+90deg forward velocity must move world +X.");
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

            Assert::AreEqual(state(VehicleState::kPy), integrated(VehicleState::kPy), 2.0e-4f,
                L"PM20_AXIS_CONVENTION heading=+90deg forward velocity must not drift in +Y.");
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

            Assert::IsTrue(integrated(VehicleState::kPx) > state(VehicleState::kPx),
                L"PM20_AXIS_CONVENTION body +V right velocity must move world +X at heading 0.");
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

            Assert::IsTrue(integrated(VehicleState::kPsi) > state(VehicleState::kPsi),
                L"PM20_AXIS_CONVENTION positive yaw rate must increase clockwise yaw.");
        }

        TEST_METHOD(WheelYawSign_PositiveYawMakesLeftWheelLinearVelocityFaster)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeRollingState(params, 0.40f, 2.0f);
            const Eigen::Vector2f wheelLinearMps =
                runtime.plant.wheelLinearVelocityFromBodyState(state);

            Assert::IsTrue(wheelLinearMps.x() > wheelLinearMps.y(),
                L"PM20_WHEEL_YAW_SIGN positive yaw must map to left wheel faster than right.");
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

            Assert::IsTrue(runtime.plant.measuredYawRateRadps(observation) > 0.0f,
                L"PM20_WHEEL_YAW_SIGN encoder yaw measurement must preserve clockwise-positive sign.");
        }

        TEST_METHOD(WheelYawSign_PositiveTargetYawRateRequestsFasterLeftWheel)
        {
            TestRuntime runtime;
            PlantModel::StateVector state = PlantModel::StateVector::Zero();
            state(VehicleState::kU) = 0.40f;
            state(VehicleState::kR) = 2.0f;
            const Eigen::Vector2f wheelLinearMps =
                runtime.plant.wheelLinearVelocityFromBodyState(state);

            Assert::IsTrue(wheelLinearMps.x() > wheelLinearMps.y(),
                L"PM20_WHEEL_YAW_SIGN positive target yaw rate must request faster left wheel.");
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

            Assert::IsTrue(command.Differential() > 0.0f,
                L"PM20_WHEEL_YAW_SIGN positive target yaw acceleration must command more left drive.");
            Assert::IsTrue(derivatives.yawAccelRadps2 > 0.0f,
                L"PM20_WHEEL_YAW_SIGN positive target yaw acceleration must produce clockwise yaw acceleration.");
        }

        TEST_METHOD(SymmetricDrive_LeftRightForwardForceSymmetry)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);

            Assert::AreEqual(
                derivatives.contactForces.LeftBankForwardForceN(),
                derivatives.contactForces.RightBankForwardForceN(),
                1.0e-5f,
                L"PM21_FORCE_SYMMETRY symmetric drive must preserve left/right forward force symmetry.");
        }

        TEST_METHOD(SymmetricDrive_WheelAccelerationSymmetry)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);

            Assert::AreEqual(
                derivatives.stateDot(VehicleState::kOmegaL),
                derivatives.stateDot(VehicleState::kOmegaR),
                1.0e-4f,
                L"PM21_FORCE_SYMMETRY symmetric drive must preserve wheel acceleration symmetry.");
        }

        TEST_METHOD(SymmetricDrive_NoYawAccelerationBias)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);

            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kR), 1.0e-4f,
                L"PM21_FORCE_SYMMETRY symmetric drive must not create yaw acceleration bias.");
        }

        TEST_METHOD(SymmetricDrive_NoLateralAccelerationBias)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.80f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.45f, 0.45f), params);

            Assert::AreEqual(0.0f, derivatives.lateralAccelMps2, 1.0e-4f,
                L"PM21_FORCE_SYMMETRY symmetric drive must not create lateral acceleration bias.");
        }

        TEST_METHOD(Stiction_SubthresholdDriveDoesNotAccelerateLeftWheelAtRest)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.25f, 0.25f), params);

            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kOmegaL), 1.0e-6f,
                L"PM21_STICTION subthreshold command at rest must not accelerate left wheel.");
        }

        TEST_METHOD(Stiction_SubthresholdDriveDoesNotAccelerateRightWheelAtRest)
        {
            TestRuntime runtime;
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state =
                MakeState(0.02f, 0.03f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            const PlantDerivatives derivatives =
                runtime.plant.forwardStep(state, MakeCommand(0.25f, 0.25f), params);

            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kOmegaR), 1.0e-6f,
                L"PM21_STICTION subthreshold command at rest must not accelerate right wheel.");
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

            Assert::AreEqual(initial, state(VehicleState::kPx), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift position X under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kPy), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift position Y under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kPsi), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift yaw under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kU), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift forward velocity under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kV), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift lateral velocity under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kR), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift yaw rate under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kOmegaL), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift left wheel speed under subthreshold stationary command.");
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

            Assert::AreEqual(initial, state(VehicleState::kOmegaR), 1.0e-6f,
                L"PM21_STICTION direct integration must not drift right wheel speed under subthreshold stationary command.");
        }

        TEST_METHOD(FanLoad_NoFanContactNormalSumMatchesConfiguredLoad)
        {
            TestRuntime runtime(0.0f);
            const PlantParams params = PlantParams::Default();
            const ContactForces forces =
                runtime.plant.tireForces(MakeRollingState(params, 0.75f, 0.0f), params);

            Assert::AreEqual(params.TotalNormalLoadN(0.0f), SumNormalLoadN(forces), 1.0e-5f,
                L"PM21_FAN_LOAD contact normal sum must match no-fan configured normal load.");
        }

        TEST_METHOD(FanLoad_FanOnContactNormalSumMatchesConfiguredLoad)
        {
            TestRuntime runtime(0.80f);
            const PlantParams params = PlantParams::Default();
            const ContactForces forces =
                runtime.plant.tireForces(MakeRollingState(params, 0.75f, 0.0f), params);

            Assert::AreEqual(params.TotalNormalLoadN(0.80f), SumNormalLoadN(forces), 1.0e-5f,
                L"PM21_FAN_LOAD contact normal sum must match fan-on configured normal load.");
        }

        TEST_METHOD(FanLoad_FanDutyIncreasesTotalContactNormalLoad)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);

            Assert::IsTrue(SumNormalLoadN(fanOnForces) > SumNormalLoadN(fanOffForces),
                L"PM21_FAN_LOAD fan duty must increase total contact normal load.");
        }

        TEST_METHOD(FanLoad_Contact0NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);

            Assert::IsTrue(fanOnForces.contacts[0].normalForceN > fanOffForces.contacts[0].normalForceN,
                L"PM21_FAN_LOAD contact 0 normal must increase with fan duty.");
        }

        TEST_METHOD(FanLoad_Contact1NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);

            Assert::IsTrue(fanOnForces.contacts[1].normalForceN > fanOffForces.contacts[1].normalForceN,
                L"PM21_FAN_LOAD contact 1 normal must increase with fan duty.");
        }

        TEST_METHOD(FanLoad_Contact2NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);

            Assert::IsTrue(fanOnForces.contacts[2].normalForceN > fanOffForces.contacts[2].normalForceN,
                L"PM21_FAN_LOAD contact 2 normal must increase with fan duty.");
        }

        TEST_METHOD(FanLoad_Contact3NormalIncreasesWithFanDuty)
        {
            TestRuntime fanOffRuntime(0.0f);
            TestRuntime fanOnRuntime(0.80f);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector state = MakeRollingState(params, 0.75f, 0.0f);
            const ContactForces fanOffForces = fanOffRuntime.plant.tireForces(state, params);
            const ContactForces fanOnForces = fanOnRuntime.plant.tireForces(state, params);

            Assert::IsTrue(fanOnForces.contacts[3].normalForceN > fanOffForces.contacts[3].normalForceN,
                L"PM21_FAN_LOAD contact 3 normal must increase with fan duty.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangePositionX)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kPx), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change position X.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangePositionY)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kPy), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change position Y.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeYaw)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kPsi), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change yaw.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeForwardVelocity)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kU), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change forward velocity.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeLateralVelocity)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kV), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change lateral velocity.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeYawRate)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kR), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change yaw rate.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeLeftWheelSpeed)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kOmegaL), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change left wheel speed.");
        }

        TEST_METHOD(IntegrateDirect_ZeroDtDoesNotChangeRightWheelSpeed)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(0.0f, VehicleState::kOmegaR), 0.0f,
                L"PM22_INTEGRATE_DIRECT zero dt must not change right wheel speed.");
        }

        TEST_METHOD(IntegrateDirect_NegativeDtDoesNotChangePositionX)
        {
            Assert::AreEqual(0.0f, NoOpDeltaForDt(-0.001f, VehicleState::kPx), 0.0f,
                L"PM22_INTEGRATE_DIRECT negative dt must not change position X.");
        }

        TEST_METHOD(IntegrateDirect_NonFiniteDtDoesNotChangePositionX)
        {
            Assert::AreEqual(
                0.0f,
                NoOpDeltaForDt((std::numeric_limits<float>::quiet_NaN)(), VehicleState::kPx),
                0.0f,
                L"PM22_INTEGRATE_DIRECT non-finite dt must not change position X.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepPositionXStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kPx),
                L"PM22_INTEGRATE_DIRECT single 4ms step position X must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepPositionYStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kPy),
                L"PM22_INTEGRATE_DIRECT single 4ms step position Y must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepYawStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kPsi),
                L"PM22_INTEGRATE_DIRECT single 4ms step yaw must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepForwardVelocityStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kU),
                L"PM22_INTEGRATE_DIRECT single 4ms step forward velocity must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepLateralVelocityStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kV),
                L"PM22_INTEGRATE_DIRECT single 4ms step lateral velocity must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepYawRateStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kR),
                L"PM22_INTEGRATE_DIRECT single 4ms step yaw rate must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepLeftWheelSpeedStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kOmegaL),
                L"PM22_INTEGRATE_DIRECT single 4ms step left wheel speed must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SingleStepRightWheelSpeedStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSingleStep(VehicleState::kOmegaR),
                L"PM22_INTEGRATE_DIRECT single 4ms step right wheel speed must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepPositionXStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kPx),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps position X must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepPositionYStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kPy),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps position Y must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepYawStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kPsi),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps yaw must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepForwardVelocityStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kU),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps forward velocity must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepLateralVelocityStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kV),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps lateral velocity must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepYawRateStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kR),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps yaw rate must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepLeftWheelSpeedStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kOmegaL),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps left wheel speed must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_SubstepRightWheelSpeedStaysFinite)
        {
            Assert::IsTrue(StateComponentIsFiniteAfterSubsteps(VehicleState::kOmegaR),
                L"PM22_INTEGRATE_DIRECT four 1ms substeps right wheel speed must stay finite.");
        }

        TEST_METHOD(IntegrateDirect_PositionXRemainsCloseBetweenSingleStepAndSubsteps)
        {
            Assert::AreEqual(0.0f, SingleStepMinusSubsteps(VehicleState::kPx), 3.0e-3f,
                L"PM22_INTEGRATE_DIRECT position X must remain close between 4ms direct step and substeps.");
        }

        TEST_METHOD(IntegrateDirect_PositionYRemainsCloseBetweenSingleStepAndSubsteps)
        {
            Assert::AreEqual(0.0f, SingleStepMinusSubsteps(VehicleState::kPy), 3.0e-3f,
                L"PM22_INTEGRATE_DIRECT position Y must remain close between 4ms direct step and substeps.");
        }

        TEST_METHOD(IntegrateDirect_ForwardVelocityRemainsCloseBetweenSingleStepAndSubsteps)
        {
            Assert::AreEqual(0.0f, SingleStepMinusSubsteps(VehicleState::kU), 8.0e-2f,
                L"PM22_INTEGRATE_DIRECT forward velocity must remain close between 4ms direct step and substeps.");
        }

        TEST_METHOD(IntegrateDirect_YawRateRemainsCloseBetweenSingleStepAndSubsteps)
        {
            Assert::AreEqual(0.0f, SingleStepMinusSubsteps(VehicleState::kR), 8.0e-1f,
                L"PM22_INTEGRATE_DIRECT yaw rate must remain close between 4ms direct step and substeps.");
        }

        TEST_METHOD(NumericStability_PositionXStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kPx),
                L"PM22_NUMERIC_STABILITY position X stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_PositionYStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kPy),
                L"PM22_NUMERIC_STABILITY position Y stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_YawStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kPsi),
                L"PM22_NUMERIC_STABILITY yaw stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_ForwardVelocityStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kU),
                L"PM22_NUMERIC_STABILITY forward velocity stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_LateralVelocityStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kV),
                L"PM22_NUMERIC_STABILITY lateral velocity stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_YawRateStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kR),
                L"PM22_NUMERIC_STABILITY yaw rate stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_LeftWheelSpeedStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kOmegaL),
                L"PM22_NUMERIC_STABILITY left wheel speed stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_RightWheelSpeedStaysFiniteUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandComponentStayedFinite(VehicleState::kOmegaR),
                L"PM22_NUMERIC_STABILITY right wheel speed stayed finite under plausible high command.");
        }

        TEST_METHOD(NumericStability_HeadingStaysNormalizedUnderPlausibleHighCommand)
        {
            Assert::IsTrue(HighCommandHeadingStayedNormalized(),
                L"PM22_NUMERIC_STABILITY direct integration must keep heading normalized.");
        }

        TEST_METHOD(NumericStability_FinalForwardVelocityStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();

            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 20.0f,
                L"PM22_NUMERIC_STABILITY forward velocity escaped plausible bounds.");
        }

        TEST_METHOD(NumericStability_FinalYawRateStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();

            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 200.0f,
                L"PM22_NUMERIC_STABILITY yaw rate escaped plausible bounds.");
        }

        TEST_METHOD(NumericStability_FinalLeftWheelSpeedStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();

            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 3000.0f,
                L"PM22_NUMERIC_STABILITY left wheel speed escaped plausible bounds.");
        }

        TEST_METHOD(NumericStability_FinalRightWheelSpeedStaysWithinPlausibleBounds)
        {
            const VehicleState::StateVector state = FinalHighCommandState();

            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 3000.0f,
                L"PM22_NUMERIC_STABILITY right wheel speed escaped plausible bounds.");
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

            Assert::IsTrue(command.IsFinite(),
                L"PM23_INVERSE_SIGN forward acceleration feedforward command must be finite.");
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

            Assert::IsTrue(command.IsFinite(),
                L"PM23_INVERSE_SIGN reverse acceleration feedforward command must be finite.");
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

            Assert::IsTrue(command.IsFinite(),
                L"PM23_INVERSE_SIGN yaw acceleration feedforward command must be finite.");
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

            Assert::IsTrue(forward.Average() > reverse.Average(),
                L"PM23_INVERSE_SIGN positive forward acceleration must command more average drive than negative acceleration.");
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

            Assert::IsTrue(clockwise.Differential() > 0.0f,
                L"PM23_INVERSE_SIGN positive clockwise yaw acceleration must command left greater than right.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunForwardSolveCommandStaysFinite)
        {
            Assert::IsTrue(LongRunForwardSolveCommandStayedFinite(),
                L"PM23_INVERSE_SIGN long-run acceleration feedforward command must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositionXStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kPx),
                L"PM23_INVERSE_SIGN long-run direct prediction position X must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositionYStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kPy),
                L"PM23_INVERSE_SIGN long-run direct prediction position Y must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunYawStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kPsi),
                L"PM23_INVERSE_SIGN long-run direct prediction yaw must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunForwardVelocityStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kU),
                L"PM23_INVERSE_SIGN long-run direct prediction forward velocity must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunLateralVelocityStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kV),
                L"PM23_INVERSE_SIGN long-run direct prediction lateral velocity must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunYawRateStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kR),
                L"PM23_INVERSE_SIGN long-run direct prediction yaw rate must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunLeftWheelSpeedStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kOmegaL),
                L"PM23_INVERSE_SIGN long-run direct prediction left wheel speed must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunRightWheelSpeedStaysFinite)
        {
            Assert::IsTrue(LongRunForwardStateComponentStayedFinite(VehicleState::kOmegaR),
                L"PM23_INVERSE_SIGN long-run direct prediction right wheel speed must stay finite.");
        }

        TEST_METHOD(AccelerationFeedforward_LongRunPositiveAccelerationIncreasesForwardVelocity)
        {
            Assert::IsTrue(LongRunForwardVelocityDelta() > 0.05f,
                L"PM23_INVERSE_SIGN long-run positive acceleration request must increase forward velocity.");
        }
    };
}
