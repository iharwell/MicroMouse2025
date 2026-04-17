#include "pch.h"
#include "CppUnitTest.h"

#include "SrUkfCoreTestSupport.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    TEST_CLASS(SrUkfCoreMotionUpdateTest)
    {
    public:
        TEST_METHOD(SrUkfCoreZeroVelocityEncoderUpdateKeepsYawRateVarianceBoundedAtRest)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core(params);

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const ContactForces lowForceContacts = plant.tireForces(initialState, control, params);
            Assert::IsTrue(std::fabs(lowForceContacts.SumForwardForceN()) < 1.0e-4f);
            Assert::IsTrue(std::fabs(lowForceContacts.SumRightForceN()) < 1.0e-4f);

            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(VehicleState::kR, VehicleState::kR);
            const float initialLeftWheelVarianceRadps2 =
                beforeCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL);
            const float initialRightWheelVarianceRadps2 =
                beforeCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR);

            Assert::IsTrue(initialLeftWheelVarianceRadps2 < (0.01f * initialYawRateVarianceRadps2));
            Assert::IsTrue(initialRightWheelVarianceRadps2 < (0.01f * initialYawRateVarianceRadps2));

            constexpr float dt = 0.001f;
            Assert::IsTrue(core.predict(dt, control));

            const VehicleState::StateMatrix predictedCovariance = core.covariance();
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kR, VehicleState::kR)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));

            EncoderObs encoder{};
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(
                afterCovariance(VehicleState::kR, VehicleState::kR) <
                (0.1f * initialYawRateVarianceRadps2));
            Assert::IsTrue(
                afterCovariance(VehicleState::kPsi, VehicleState::kPsi) <
                (0.1f * initialYawRateVarianceRadps2));
        }
        TEST_METHOD(FeedforwardAgreesWithPredict)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core(params);

            constexpr float targetForwardVelocityMps = 0.2f;

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.00f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.01f, 0.001f, 0.005f, 0, 0, 0, 0.02f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            auto control = plant.solveDriveCommandsForVelocityTarget(
                0.0f,
                targetForwardVelocityMps,
                0.0f,
                0.0f,
                params,
                0.8f,
                8.4f);
            Assert::IsTrue(control.control.leftMotorCommand > 0.1f, L"Left motor command should overcome friction."); // Should overcome friction.
            Assert::IsTrue(control.control.rightMotorCommand > 0.1f, L"Right motor command should overcome friction."); // Should overcome friction.

            for (size_t i = 0; i < 1000; i++)
            {
                control = plant.solveDriveCommandsForVelocityTarget(
                    core.state()(VehicleState::kU),
                    targetForwardVelocityMps,
                    core.state()(VehicleState::kR),
                    0.0f,
                    params,
                    0.8f,
                    8.4f);
                core.predict(0.001f, control.control);
            }
            Assert::IsTrue(core.state()(VehicleState::kU) > targetForwardVelocityMps * 0.9f, L"Vehicle speed shouldn't fall too low.");
            Assert::IsTrue(core.state()(VehicleState::kU) < targetForwardVelocityMps * 1.1f, L"Vehicle speed shouldn't be too high.");
        }

        TEST_METHOD(SrUkfCoreMovingEncoderUpdateKeepsYawRateVarianceLowAfterPredictAndUpdate)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core(params);
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const float measuredWheelOmegaRadps = distancePerCountM / (params.wheelRadiusM * 0.001f);
            const float measuredLinearSpeedMps = params.wheelRadiusM * measuredWheelOmegaRadps;

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    measuredLinearSpeedMps,
                    0.0f,
                    0.0f,
                    measuredWheelOmegaRadps,
                    measuredWheelOmegaRadps,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const ContactForces lowForceContacts = plant.tireForces(initialState, control, params);
            Assert::IsTrue(std::fabs(lowForceContacts.SumForwardForceN()) < 1.0e-4f);
            Assert::IsTrue(std::fabs(lowForceContacts.SumRightForceN()) < 1.0e-4f);

            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(VehicleState::kR, VehicleState::kR);
            Assert::IsTrue(
                beforeCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <
                (0.01f * initialYawRateVarianceRadps2));
            Assert::IsTrue(
                beforeCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <
                (0.01f * initialYawRateVarianceRadps2));

            constexpr float dt = 0.001f;
            Assert::IsTrue(core.predict(dt, control));

            const VehicleState::StateMatrix predictedCovariance = core.covariance();
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kR, VehicleState::kR)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(predictedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 1;
            encoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            encoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(
                afterCovariance(VehicleState::kR, VehicleState::kR) <=
                (predictedCovariance(VehicleState::kR, VehicleState::kR) + 1.0e-9f));
            Assert::IsTrue(
                afterCovariance(VehicleState::kR, VehicleState::kR) <
                (0.1f * initialYawRateVarianceRadps2));
            Assert::IsTrue(std::isfinite(afterCovariance(VehicleState::kR, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(afterCovariance(VehicleState::kR, VehicleState::kOmegaR)));
        }
        TEST_METHOD(SrUkfCoreLatestEncoderObservationPullsWheelRatesTowardLatestMeasurement)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const auto omegaFromCounts = [distancePerCountM, &params](int32_t counts, float dtSeconds) noexcept
            {
                return (static_cast<float>(counts) * distancePerCountM) / (params.wheelRadiusM * dtSeconds);
            };
            SrUkfCore core(params);
            ControlInput control{};
            constexpr float dt = 0.010f;

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs first{};
            first.totalLeftCounts = 2;
            first.totalRightCounts = -1;
            first.omegaLeftRadps = omegaFromCounts(first.totalLeftCounts, dt);
            first.omegaRightRadps = omegaFromCounts(first.totalRightCounts, dt);
            const MeasurementUpdateResult firstResult = core.updateEncoderPair(first, dt);
            Assert::IsTrue(firstResult.attempted);
            Assert::IsTrue(firstResult.accepted);

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs second{};
            second.totalLeftCounts = 5;
            second.totalRightCounts = -3;
            second.omegaLeftRadps = omegaFromCounts(second.totalLeftCounts, dt);
            second.omegaRightRadps = omegaFromCounts(second.totalRightCounts, dt);
            const MeasurementUpdateResult secondResult = core.updateEncoderPair(second, dt);
            Assert::IsTrue(secondResult.attempted);
            Assert::IsTrue(secondResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::isfinite(state(VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(state(VehicleState::kOmegaR)));
            Assert::IsTrue(state(VehicleState::kOmegaL) > 0.0f);
            Assert::IsTrue(state(VehicleState::kOmegaR) < 0.0f);
            Assert::IsTrue(
                std::fabs(state(VehicleState::kOmegaL) - second.omegaLeftRadps) <
                std::fabs(first.omegaLeftRadps - second.omegaLeftRadps));
            Assert::IsTrue(
                std::fabs(state(VehicleState::kOmegaR) - second.omegaRightRadps) <
                std::fabs(first.omegaRightRadps - second.omegaRightRadps));
        }

        TEST_METHOD(SrUkfCoreDoesNotLetControlInputCreateUnboundedForwardMotionWithEncoderOpposition)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.18f;
            control.rightMotorCommand = 0.18f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 1.0e-3f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) < 1.0e-3f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);
        }

        TEST_METHOD(SrUkfCoreMustLetControlInputCreateForwardMotionWithNoEncoder)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.5f;
            control.rightMotorCommand = 0.5f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) > 1.0e-2f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) > 1.0e-2f);
        }

        TEST_METHOD(SrUkfCoreAcceptsLaunchEncoderDeltasWhenOpenLoopPredictionDisagrees)
        {
            struct LaunchEncoderSample
            {
                float dtSeconds;
                float leftOmegaRadps;
                float rightOmegaRadps;
                float gyroRawRadps;
            };

            // D:\open_floor_main.csv, April 10, 2026, repeat 11 launch pulse.
            // The raw launch command makes the open-loop plant prediction disagree
            // with measured wheel rates; encoder deltas must still remain authoritative.
            constexpr LaunchEncoderSample samples[] = {
                { 0.001011f, 3.99f, 3.99f, -0.019f },
                { 0.001015f, 9.67f, 15.20f, 0.002f },
                { 0.001005f, 8.72f, 13.76f, 0.092f },
                { 0.001020f, 6.49f, 12.05f, 0.203f },
                { 0.001004f, 5.02f, 9.59f, 0.152f },
                { 0.001019f, 4.64f, 8.35f, 0.149f },
                { 0.001005f, 5.48f, 6.40f, 0.116f },
                { 0.001000f, 6.49f, 6.49f, -0.006f },
                { 0.001005f, 8.38f, 5.59f, -0.108f },
                { 0.001000f, 10.19f, 6.95f, -0.156f },
            };

            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const auto countsFromOmega = [distancePerCountM, &params](float omegaRadps, float dtSeconds) noexcept
            {
                const float counts =
                    (omegaRadps * params.wheelRadiusM * dtSeconds) / distancePerCountM;
                return static_cast<int32_t>((counts >= 0.0f) ? (counts + 0.5f) : (counts - 0.5f));
            };

            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.225f,
                    0.225f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            ControlInput control{};
            control.leftMotorCommand = 0.08f;
            control.rightMotorCommand = 0.08f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            for (int index = 0; index < static_cast<int>(sizeof(samples) / sizeof(samples[0])); ++index)
            {
                const LaunchEncoderSample& sample = samples[index];
                Assert::IsTrue(core.predict(sample.dtSeconds, control));

                EncoderObs encoder{};
                encoder.totalLeftCounts = countsFromOmega(sample.leftOmegaRadps, sample.dtSeconds);
                encoder.totalRightCounts = countsFromOmega(sample.rightOmegaRadps, sample.dtSeconds);
                encoder.omegaLeftRadps = sample.leftOmegaRadps;
                encoder.omegaRightRadps = sample.rightOmegaRadps;
                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, sample.dtSeconds);

                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(
                    encoderResult.accepted,
                    (std::wstring(L"Launch encoder update rejected at sample ") +
                        std::to_wstring(index)).c_str());

                const VehicleState::StateVector& encoderConstrainedState = core.state();
                Assert::IsTrue(std::isfinite(encoderConstrainedState(VehicleState::kU)));
                Assert::AreEqual(sample.leftOmegaRadps, encoderConstrainedState(VehicleState::kOmegaL), 1.0e-5f);
                Assert::AreEqual(sample.rightOmegaRadps, encoderConstrainedState(VehicleState::kOmegaR), 1.0e-5f);

                const MeasurementUpdateResult yawResult = core.updateYawRate(sample.gyroRawRadps);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }
        }

        TEST_METHOD(SrUkfCoreEncoderArcUpdateUsesProjectTurnSignConventions)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.005f, 0.01f, 0.01f, 0.01f, 0.02f, 0.05f, 0.01f)));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.010f;
            Assert::IsTrue(core.predict(dt, control));

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            EncoderObs encoder{};
            encoder.totalLeftCounts = 4;
            encoder.totalRightCounts = 2;
            encoder.omegaLeftRadps =
                (static_cast<float>(encoder.totalLeftCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);
            encoder.omegaRightRadps =
                (static_cast<float>(encoder.totalRightCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);

            const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const float expectedForwardDistanceM =
                0.5f * static_cast<float>(encoder.totalLeftCounts + encoder.totalRightCounts) * distancePerCountM;
            const float expectedYawRad =
                static_cast<float>(encoder.totalLeftCounts - encoder.totalRightCounts) * distancePerCountM / params.trackWidthM;
            const VehicleState::StateVector& state = core.state();

            Assert::IsTrue(std::isfinite(state.sum()));
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kPsi) > 0.0f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < expectedForwardDistanceM);
            Assert::IsTrue(std::fabs(state(VehicleState::kPsi)) <= (2.0f * expectedYawRad));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
        }

        TEST_METHOD(SrUkfCoreEncoderUpdateCanPreserveYawStatesWhenRequested)
        {
            const PlantParams params = PlantParams::Default();
            const float distancePerCountM = DistancePerEncoderCountMeters(params);
            const auto formatState = [](const VehicleState::StateVector& state) -> std::wstring
            {
                return
                    L"px=" + std::to_wstring(state(VehicleState::kPx)) +
                    L", py=" + std::to_wstring(state(VehicleState::kPy)) +
                    L", psi=" + std::to_wstring(state(VehicleState::kPsi)) +
                    L", u=" + std::to_wstring(state(VehicleState::kU)) +
                    L", v=" + std::to_wstring(state(VehicleState::kV)) +
                    L", r=" + std::to_wstring(state(VehicleState::kR)) +
                    L", omegaL=" + std::to_wstring(state(VehicleState::kOmegaL)) +
                    L", omegaR=" + std::to_wstring(state(VehicleState::kOmegaR)) +
                    L", bgz=" + std::to_wstring(state(VehicleState::kBgz));
            };
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.01f,
                    0.09f,
                    0.18f,
                    0.15f,
                    0.02f,
                    0.25f,
                    1.8f,
                    1.7f,
                    0.01f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.04f, 0.12f, 0.20f, 0.02f);

            SrUkfCore coreWithEncoderYaw(params);
            SrUkfCore coreWithoutEncoderYaw(params);
            Assert::IsTrue(coreWithEncoderYaw.reset(initialState, initialCovariance));
            Assert::IsTrue(coreWithoutEncoderYaw.reset(initialState, initialCovariance));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.010f;
            Assert::IsTrue(coreWithEncoderYaw.predict(dt, control));
            Assert::IsTrue(coreWithoutEncoderYaw.predict(dt, control));

            const VehicleState::StateVector predictedState = coreWithoutEncoderYaw.state();
            const VehicleState::StateMatrix predictedCovariance = coreWithoutEncoderYaw.covariance();

            EncoderObs encoder{};
            encoder.totalLeftCounts = 4;
            encoder.totalRightCounts = 2;
            encoder.omegaLeftRadps =
                (static_cast<float>(encoder.totalLeftCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);
            encoder.omegaRightRadps =
                (static_cast<float>(encoder.totalRightCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);

            const MeasurementUpdateResult withYawResult =
                coreWithEncoderYaw.updateEncoderPair(encoder, dt);
            const MeasurementUpdateResult withoutYawResult =
                coreWithoutEncoderYaw.updateEncoderPair(encoder, dt, false);
            Assert::IsTrue(withYawResult.attempted);
            Assert::IsTrue(withYawResult.accepted);
            Assert::IsTrue(withoutYawResult.attempted);
            Assert::IsTrue(withoutYawResult.accepted);

            const VehicleState::StateVector& stateWithEncoderYaw = coreWithEncoderYaw.state();
            const VehicleState::StateVector& stateWithoutEncoderYaw = coreWithoutEncoderYaw.state();
            const VehicleState::StateMatrix covarianceWithoutEncoderYaw = coreWithoutEncoderYaw.covariance();
            const float leftDistanceM = static_cast<float>(encoder.totalLeftCounts) * distancePerCountM;
            const float rightDistanceM = static_cast<float>(encoder.totalRightCounts) * distancePerCountM;
            const float forwardDistanceM = 0.5f * (leftDistanceM + rightDistanceM);
            const float deltaYawRad = (leftDistanceM - rightDistanceM) / params.trackWidthM;
            const float translationYawRad =
                VehicleState::NormalizeAngle(initialState(VehicleState::kPsi) + (0.5f * deltaYawRad));
            const float expectedPx =
                initialState(VehicleState::kPx) + (forwardDistanceM * std::sin(translationYawRad));
            const float expectedPy =
                initialState(VehicleState::kPy) + (forwardDistanceM * std::cos(translationYawRad));
            const float expectedLinearSpeedMps =
                0.5f * params.wheelRadiusM * (encoder.omegaLeftRadps + encoder.omegaRightRadps);

            Assert::IsTrue(
                std::fabs(stateWithEncoderYaw(VehicleState::kPsi) - predictedState(VehicleState::kPsi)) > 1.0e-5f,
                (L"predicted=" + formatState(predictedState) +
                    L"; withEncoderYaw=" + formatState(stateWithEncoderYaw)).c_str());
            Assert::IsTrue(
                std::fabs(stateWithEncoderYaw(VehicleState::kR) - predictedState(VehicleState::kR)) > 1.0e-5f,
                (L"predicted=" + formatState(predictedState) +
                    L"; withEncoderYaw=" + formatState(stateWithEncoderYaw)).c_str());
            Assert::AreEqual(
                predictedState(VehicleState::kPsi),
                stateWithoutEncoderYaw(VehicleState::kPsi),
                1.0e-6f);
            Assert::AreEqual(
                predictedState(VehicleState::kR),
                stateWithoutEncoderYaw(VehicleState::kR),
                1.0e-6f);
            Assert::AreEqual(
                predictedCovariance(VehicleState::kPsi, VehicleState::kPsi),
                covarianceWithoutEncoderYaw(VehicleState::kPsi, VehicleState::kPsi),
                1.0e-6f);
            Assert::AreEqual(
                predictedCovariance(VehicleState::kR, VehicleState::kR),
                covarianceWithoutEncoderYaw(VehicleState::kR, VehicleState::kR),
                1.0e-6f);
            Assert::AreEqual(expectedPx, stateWithEncoderYaw(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(expectedPy, stateWithEncoderYaw(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(expectedPx, stateWithoutEncoderYaw(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(expectedPy, stateWithoutEncoderYaw(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(expectedLinearSpeedMps, stateWithEncoderYaw(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(expectedLinearSpeedMps, stateWithoutEncoderYaw(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateWithEncoderYaw(VehicleState::kOmegaL), 1.0e-5f);
            Assert::AreEqual(encoder.omegaRightRadps, stateWithEncoderYaw(VehicleState::kOmegaR), 1.0e-5f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateWithoutEncoderYaw(VehicleState::kOmegaL), 1.0e-5f);
            Assert::AreEqual(encoder.omegaRightRadps, stateWithoutEncoderYaw(VehicleState::kOmegaR), 1.0e-5f);
        }

        TEST_METHOD(SrUkfCoreSplitDrivePredictBuildsTurnRateWhileKeepingForwardProgress)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            constexpr float initialForwardVelocityMps = 1.0f;
            const float initialWheelSpeedRadps = initialForwardVelocityMps / params.wheelRadiusM;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    initialForwardVelocityMps,
                    0.0f,
                    0.0f,
                    initialWheelSpeedRadps,
                    initialWheelSpeedRadps);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            ControlInput control{};
            control.leftMotorCommand = 0.30f;
            control.rightMotorCommand = 0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 75;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPsi) < 0.0f);
            Assert::IsTrue(state(VehicleState::kPx) < 0.0f);
        }

        TEST_METHOD(SrUkfCoreRepeatedForwardEncoderUpdatesStayMostlyStraightAndBoundAcceleration)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.12f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.05f, 0.08f, 0.20f, 0.03f)));

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            constexpr float dt = 0.005f;
            const float expectedSpeedMps = distancePerCountM / dt;
            const float measuredOmegaRadps = expectedSpeedMps / params.wheelRadiusM;
            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            float previousPositionM = initialState(VehicleState::kPy);
            float previousSpeedMps = expectedSpeedMps;
            constexpr int kSteps = 5;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                EncoderObs encoder{};
                encoder.totalLeftCounts = 1;
                encoder.totalRightCounts = 1;
                encoder.omegaLeftRadps = measuredOmegaRadps;
                encoder.omegaRightRadps = measuredOmegaRadps;
                const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(result.attempted);
                Assert::IsTrue(result.accepted);

                const VehicleState::StateVector& state = core.state();
                const float intervalDistanceM = state(VehicleState::kPy) - previousPositionM;
                const float intervalSpeedMps = intervalDistanceM / dt;
                const float intervalAccelMps2 = (intervalSpeedMps - previousSpeedMps) / dt;
                const float expectedPositionM =
                    initialState(VehicleState::kPy) + ((step + 1.0f) * distancePerCountM);

                Assert::IsTrue(intervalDistanceM > 0.0f);
                Assert::IsTrue(
                    std::fabs(state(VehicleState::kPx)) <
                    (0.25f * (state(VehicleState::kPy) - initialState(VehicleState::kPy))));
                Assert::AreEqual(expectedPositionM, state(VehicleState::kPy), distancePerCountM * 0.25f);
                Assert::AreEqual(expectedSpeedMps, intervalSpeedMps, 0.15f);
                Assert::IsTrue(std::fabs(intervalAccelMps2) < 60.0f);
                Assert::IsTrue(state(VehicleState::kU) > 0.0f);
                Assert::IsTrue(state(VehicleState::kU) < 3.0f);

                previousPositionM = state(VehicleState::kPy);
                previousSpeedMps = intervalSpeedMps;
            }
        }

        TEST_METHOD(SrUkfCoreSingleSymmetricEncoderCountAdvancesForwardByAboutOneCount)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.18f;
            control.rightMotorCommand = 0.18f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            const float measuredOmegaRadps = distancePerCountM / (params.wheelRadiusM * dt);

            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 1;
            encoder.omegaLeftRadps = measuredOmegaRadps;
            encoder.omegaRightRadps = measuredOmegaRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < distancePerCountM);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy) - distancePerCountM) < (0.5f * distancePerCountM));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kU) < (2.0f * (distancePerCountM / dt)));
        }

        TEST_METHOD(SrUkfCoreZeroEncoderObservationCollapsesMotionStateWithoutTeleportingPose)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            constexpr float initialForwardVelocityMps = 1.2f;
            const float wheelSpeedRadps = initialForwardVelocityMps / params.wheelRadiusM;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.03f,
                    0.11f,
                    0.08f,
                    initialForwardVelocityMps,
                    0.02f,
                    0.15f,
                    wheelSpeedRadps,
                    wheelSpeedRadps);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.05f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f)));

            ControlInput control{};
            control.leftMotorCommand = 0.40f;
            control.rightMotorCommand = 0.40f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.01f;
            Assert::IsTrue(core.predict(dt, control));
            const VehicleState::StateMatrix predictedCovariance = core.covariance();

            EncoderObs encoder{};
            const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            const float maxOpenLoopTravelM = initialForwardVelocityMps * dt;
            Assert::IsTrue(std::fabs(state(VehicleState::kPx) - initialState(VehicleState::kPx)) < maxOpenLoopTravelM);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy) - initialState(VehicleState::kPy)) < maxOpenLoopTravelM);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-6f);
            Assert::IsTrue(
                covariance(VehicleState::kU, VehicleState::kU) <
                (0.1f * predictedCovariance(VehicleState::kU, VehicleState::kU)));
            Assert::IsTrue(
                covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) <
                (0.1f * predictedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(
                covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) <
                (0.1f * predictedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPy, VehicleState::kPy)));
            Assert::IsTrue(covariance(VehicleState::kPy, VehicleState::kPy) > 1.0e-12f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateImprovesYawMeasurementFitWithoutMovingPosition)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.02f,
                    0.14f,
                    0.10f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.01f, 0.04f, 0.02f, 0.02f, 0.30f, 0.10f, 0.30f)));

            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            constexpr float observedYawRateRadps = 0.35f;
            const float beforeError =
                std::fabs((before(VehicleState::kR) + before(VehicleState::kBgz)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            const float afterError =
                std::fabs((after(VehicleState::kR) + after(VehicleState::kBgz)) - observedYawRateRadps);

            Assert::AreEqual(before(VehicleState::kPx), after(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(before(VehicleState::kPy), after(VehicleState::kPy), 1.0e-6f);
            Assert::IsTrue(afterError < beforeError);
            Assert::AreEqual(before(VehicleState::kBgz), after(VehicleState::kBgz), 1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz),
                afterCovariance(VehicleState::kBgz, VehicleState::kBgz),
                1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateDoesNotPullWheelRatesThroughYawCrossCovariance)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.02f,
                    0.14f,
                    0.10f,
                    1.40f,
                    0.05f,
                    0.0f,
                    12.0f,
                    11.5f,
                    0.0f);
            VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.01f, 0.04f, 0.02f, 0.02f, 0.30f, 0.30f, 0.03f);
            initialCovariance(VehicleState::kR, VehicleState::kOmegaL) = 0.010f;
            initialCovariance(VehicleState::kOmegaL, VehicleState::kR) = 0.010f;
            initialCovariance(VehicleState::kR, VehicleState::kOmegaR) = -0.012f;
            initialCovariance(VehicleState::kOmegaR, VehicleState::kR) = -0.012f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            constexpr float observedYawRateRadps = 0.45f;
            const float beforeError =
                std::fabs((before(VehicleState::kR) + before(VehicleState::kBgz)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            const float afterError =
                std::fabs((after(VehicleState::kR) + after(VehicleState::kBgz)) - observedYawRateRadps);

            Assert::IsTrue(afterError < beforeError);
            Assert::AreEqual(before(VehicleState::kOmegaL), after(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(before(VehicleState::kOmegaR), after(VehicleState::kOmegaR), 1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL),
                afterCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL),
                1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR),
                afterCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR),
                1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(VehicleState::kR, VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(VehicleState::kOmegaL, VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(VehicleState::kR, VehicleState::kOmegaR), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(VehicleState::kOmegaR, VehicleState::kR), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreFrontWallUpdateMovesForwardForCloserSymmetricObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);
            const LocalMapView map = BuildLocalMapView(maze);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.09f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);

            WallGeometryModel geometry;
            const GeometryPrediction leftPrediction = geometry.predictRay(initialState, params.frontLeftSensor, map);
            const GeometryPrediction rightPrediction = geometry.predictRay(initialState, params.frontRightSensor, map);
            Assert::IsTrue(leftPrediction.hit);
            Assert::IsTrue(rightPrediction.hit);

            SrUkfCore core(params);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs leftObservation{};
            leftObservation.valid = true;
            leftObservation.confidence = 1.0f;
            leftObservation.cls = ObsClass::WallLike;
            leftObservation.rho = leftPrediction.rangeM - 0.012f;

            WallObs rightObservation = leftObservation;
            rightObservation.rho = rightPrediction.rangeM - 0.012f;

            const FrontPairUpdateResult result = core.updateFrontPair(leftObservation, rightObservation, map);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(after(VehicleState::kPy) > (before(VehicleState::kPy) + 0.002f));
            Assert::IsTrue(std::fabs(after(VehicleState::kPx) - before(VehicleState::kPx)) < 0.004f);
            Assert::IsTrue(afterCovariance(VehicleState::kPy, VehicleState::kPy) <
                beforeCovariance(VehicleState::kPy, VehicleState::kPy));
        }

        TEST_METHOD(SrUkfCoreLeftWallUpdateMovesLeftForCloserObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Left, WallState::Wall);
            const LocalMapView map = BuildLocalMapView(maze);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.09f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);

            WallGeometryModel geometry;
            const GeometryPrediction baseline = geometry.predictRay(initialState, params.sideLeftSensor, map);
            Assert::IsTrue(baseline.hit);

            SrUkfCore core(params);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs observation{};
            observation.valid = true;
            observation.confidence = 1.0f;
            observation.cls = ObsClass::WallLike;
            observation.rho = baseline.rangeM - 0.012f;

            const WallUpdateResult result = core.updateSideSensor(Side::Left, observation, map);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(after(VehicleState::kPx) < (before(VehicleState::kPx) - 0.002f));
            Assert::IsTrue(std::fabs(after(VehicleState::kPy) - before(VehicleState::kPy)) < 0.004f);
            Assert::IsTrue(afterCovariance(VehicleState::kPx, VehicleState::kPx) <
                beforeCovariance(VehicleState::kPx, VehicleState::kPx));
        }
        TEST_METHOD(SrUkfCoreControlDirectionsCorrect)
        {
			PlantModel model = PlantModel();
            const PlantParams& params = PlantParams::Default();
            constexpr float forwardVelocityTargetMps = 1.0f;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 0.05f, 0.05f, 0.02f);

            SrUkfCore core;
            core.reset(initialState, initialCovariance);
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 3000; ++step)
            {
                auto control =
                    model.solveDriveCommandsForVelocityTarget(
                        core.state()(VehicleState::kU),
                        forwardVelocityTargetMps,
                        core.state()(VehicleState::kR),
                        0.0f,
                        params);

                Assert::IsTrue(core.predict(dt, control.control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

			auto state = core.state();
            Assert::IsTrue(state(VehicleState::kU) > 0.8f,
                (std::wstring(L"Forward velocity was too low: ") +
                    std::to_wstring(state(VehicleState::kU))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kV)) < 0.01f,
                (std::wstring(L"Lateral velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kV))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kR)) < 0.5f,
                (std::wstring(L"Angular velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kR))).c_str());
        }
        TEST_METHOD(SrUkfCoreControlDirectionsCorrectAfterStationary)
        {
            PlantModel model = PlantModel();
            const PlantParams& params = PlantParams::Default();
            constexpr float forwardVelocityTargetMps = 1.0f;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            auto core = RunUKFCycles(2000, ControlInput{});

            for (int step = 0; step < 3000; ++step)
            {
                auto control =
                    model.solveDriveCommandsForVelocityTarget(
                        core.state()(VehicleState::kU),
                        forwardVelocityTargetMps,
                        core.state()(VehicleState::kR),
                        0.0f,
                        params);

                Assert::IsTrue(core.predict(dt, control.control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            auto state = core.state();
            Assert::IsTrue(state(VehicleState::kU) > 0.8f,
                (std::wstring(L"Forward velocity was too low: ") +
                    std::to_wstring(state(VehicleState::kU))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kV)) < 0.01f,
                (std::wstring(L"Lateral velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kV))).c_str());

            Assert::IsTrue(fabs(state(VehicleState::kR)) < 0.5f,
                (std::wstring(L"Angular velocity was too high: ") +
                    std::to_wstring(state(VehicleState::kR))).c_str());
        }
    };
}
