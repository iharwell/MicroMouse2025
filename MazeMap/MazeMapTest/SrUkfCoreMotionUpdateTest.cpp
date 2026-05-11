#include "pch.h"
#include "CppUnitTest.h"

#include "SrUkfCoreTestSupport.h"
#include "TimeStepPropagationTestSupport.h"
#include <cmath>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kZeroVelocityToleranceMps = 0.008f;
        using CommandVector = App::Internal::CommandVector;

        VehicleState::StateVector PredictStationarySplitCommandStateAfterPivotPredictSequence()
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();
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
            if (!core.reset(initialState, BuildUkfCovariance()))
            {
                return VehicleState::StateVector::Constant(std::numeric_limits<float>::quiet_NaN());
            }

            const CommandVector control = CommandVector(0.60f, -0.60f);

            constexpr float dtSeconds = 0.001f;
            constexpr int kPredictSteps = 500;
            const float pivotScrubCommandAngularRadps =
                kUkfTestPivotScrubMinCommandAngularRadps;
            SyntheticEncoderRemainderState syntheticEncoderState{};
            for (int step = 0; step < kPredictSteps; ++step)
            {
                RunPredictionMatchingCycle(
                    core,
                    control,
                    params,
                    dtSeconds,
                    syntheticEncoderState,
                    0.0f,
                    pivotScrubCommandAngularRadps);
            }

            return core.state();
        }

        WallGeometryModel::GeometryStateFrame BuildGeometryFrame(
            const WallGeometryModel& geometry,
            const VehicleState::StateVector& state)
        {
            return geometry.buildStateFrame(
                Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)),
                state(VehicleState::kPsi));
        }
    }

    TEST_CLASS(SrUkfCoreMotionUpdateTest)
    {
    public:
        TEST_METHOD(SrUkfCoreZeroVelocityEncoderUpdateKeepsYawRateVarianceBoundedAtRest)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core = MakeDefaultSrUkfCore();

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

            App::Internal::CommandVector control{};
            const float controlFanDutyCycle = 0.80f;
            const float controlBatteryVoltageV = params.supplyVoltageV;

            const ContactForces lowForceContacts =
                plant.tireForces(initialState, control, controlFanDutyCycle, params);
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
            Assert::IsTrue(core.predict(dt, control, controlFanDutyCycle, controlBatteryVoltageV));

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
                initialYawRateVarianceRadps2);
            Assert::IsTrue(
                afterCovariance(VehicleState::kPsi, VehicleState::kPsi) <
                initialYawRateVarianceRadps2);
        }
        TEST_METHOD(SrUkfCoreMovingEncoderUpdateKeepsYawRateVarianceLowAfterPredictAndUpdate)
        {
            const PlantParams params = PlantParams::Default();
            PlantModel plant;
            SrUkfCore core = MakeDefaultSrUkfCore();
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

            App::Internal::CommandVector control{};
            const float controlFanDutyCycle = 0.80f;
            const float controlBatteryVoltageV = params.supplyVoltageV;

            const ContactForces lowForceContacts =
                plant.tireForces(initialState, control, controlFanDutyCycle, params);
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
            Assert::IsTrue(core.predict(dt, control, controlFanDutyCycle, controlBatteryVoltageV));

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
                initialYawRateVarianceRadps2);
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
            SrUkfCore core = MakeDefaultSrUkfCore();
            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;

            Assert::IsTrue(core.predict(dt, control, fanDutyCycle, batteryVoltageV));
            EncoderObs first{};
            first.totalLeftCounts = 2;
            first.totalRightCounts = -1;
            first.omegaLeftRadps = omegaFromCounts(first.totalLeftCounts, dt);
            first.omegaRightRadps = omegaFromCounts(first.totalRightCounts, dt);
            const MeasurementUpdateResult firstResult = core.updateEncoderPair(first, dt);
            Assert::IsTrue(firstResult.attempted);
            Assert::IsTrue(firstResult.accepted);

            Assert::IsTrue(core.predict(dt, control, fanDutyCycle, batteryVoltageV));
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

        TEST_METHOD(SrUkfCoreEncoderPairDirectUpdateKeepsBodyStateInvariantWhileUpdatingWheelStates)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.07f,
                    0.19f,
                    0.11f,
                    0.42f,
                    0.03f,
                    0.18f,
                    7.4f,
                    6.8f,
                    0.02f);
            VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.02f, 0.05f, 0.20f, 0.12f, 0.18f, 0.45f, 0.04f);
            initialCovariance(VehicleState::kU, VehicleState::kOmegaL) = 0.020f;
            initialCovariance(VehicleState::kOmegaL, VehicleState::kU) = 0.020f;
            initialCovariance(VehicleState::kU, VehicleState::kOmegaR) = -0.018f;
            initialCovariance(VehicleState::kOmegaR, VehicleState::kU) = -0.018f;
            initialCovariance(VehicleState::kR, VehicleState::kOmegaL) = 0.012f;
            initialCovariance(VehicleState::kOmegaL, VehicleState::kR) = 0.012f;
            initialCovariance(VehicleState::kR, VehicleState::kOmegaR) = -0.010f;
            initialCovariance(VehicleState::kOmegaR, VehicleState::kR) = -0.010f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector stateBeforeEncoder = core.state();

            EncoderObs encoder{};
            encoder.totalLeftCounts = 8;
            encoder.totalRightCounts = -6;
            encoder.omegaLeftRadps = 10.6f;
            encoder.omegaRightRadps = 2.4f;

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateVector& stateAfterEncoder = core.state();
            Assert::IsTrue(FindDebugDumpBool(core, "ukf_dump_filter_diagnostics", "direct_wheel_body_invariant"));
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPx), stateAfterEncoder(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPy), stateAfterEncoder(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPsi), stateAfterEncoder(VehicleState::kPsi), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kU), stateAfterEncoder(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kV), stateAfterEncoder(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kR), stateAfterEncoder(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kBgz), stateAfterEncoder(VehicleState::kBgz), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateAfterEncoder(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, stateAfterEncoder(VehicleState::kOmegaR), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreRejectedEncoderPairUpdateStillKeepsBodyStateInvariant)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();

            const VehicleState::StateVector initialState =
                BuildUkfState(
                    -0.04f,
                    0.16f,
                    -0.07f,
                    0.31f,
                    -0.02f,
                    0.09f,
                    1.6f,
                    1.4f,
                    -0.01f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.02f, 0.04f, 0.10f, 0.08f, 0.10f, 0.08f, 0.03f);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector stateBeforeEncoder = core.state();

            EncoderObs encoder{};
            encoder.totalLeftCounts = 40;
            encoder.totalRightCounts = -36;
            encoder.omegaLeftRadps = 120.0f;
            encoder.omegaRightRadps = -115.0f;

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);
            Assert::IsTrue(encoderResult.nis > kUkfTestEncoderPairNisThreshold);

            const VehicleState::StateVector& stateAfterEncoder = core.state();
            Assert::IsTrue(FindDebugDumpBool(core, "ukf_dump_filter_diagnostics", "direct_wheel_body_invariant"));
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPx), stateAfterEncoder(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPy), stateAfterEncoder(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPsi), stateAfterEncoder(VehicleState::kPsi), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kU), stateAfterEncoder(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kV), stateAfterEncoder(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kR), stateAfterEncoder(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kBgz), stateAfterEncoder(VehicleState::kBgz), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateAfterEncoder(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, stateAfterEncoder(VehicleState::kOmegaR), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreDoesNotLetControlVectorCreateUnboundedForwardMotionWithEncoderOpposition)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const PlantParams params = PlantParams::Default();
            const CommandVector control = CommandVector(0.18f, 0.18f);
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, fanDutyCycle, batteryVoltageV));

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

        TEST_METHOD(SrUkfCoreMustLetControlVectorCreateForwardMotionWithNoEncoder)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const PlantParams params = PlantParams::Default();
            const CommandVector control = CommandVector(0.5f, 0.5f);
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, fanDutyCycle, batteryVoltageV));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) > 1.0e-2f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) > 1.0e-2f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandKeepsZeroForwardVelocity)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            Assert::AreEqual(0.0f, predictedState(VehicleState::kU), kZeroVelocityToleranceMps);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandProducesPositiveLeftWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::IsTrue(predictedState(VehicleState::kOmegaL) > 0.0f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandProducesNegativeRightWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::IsTrue(predictedState(VehicleState::kOmegaR) < 0.0f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandKeepsZeroAverageWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            const float averageWheelLinearSpeedMps =
                0.5f *
                (predictedState(VehicleState::kOmegaL) + predictedState(VehicleState::kOmegaR)) *
                PlantParams::Default().wheelRadiusM;

            Assert::AreEqual(0.0f, averageWheelLinearSpeedMps, kZeroVelocityToleranceMps);
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

            SrUkfCore core = MakeDefaultSrUkfCore();
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

            const CommandVector control = CommandVector(0.08f, 0.08f);
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;

            for (int index = 0; index < static_cast<int>(sizeof(samples) / sizeof(samples[0])); ++index)
            {
                const LaunchEncoderSample& sample = samples[index];
                Assert::IsTrue(core.predict(sample.dtSeconds, control, fanDutyCycle, batteryVoltageV));

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

        TEST_METHOD(SrUkfCoreSplitDrivePredictBuildsTurnRateWhileKeepingForwardProgress)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();
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

            const CommandVector control = CommandVector(0.30f, 0.60f);
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 75;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, fanDutyCycle, batteryVoltageV));
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPsi) < 0.0f);
            Assert::IsTrue(state(VehicleState::kPx) < 0.0f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateDoesNotPullWheelRatesThroughYawCrossCovariance)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
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
            const WallGeometryModel::GeometryStateFrame frame = BuildGeometryFrame(geometry, initialState);
            const GeometryPrediction leftPrediction = geometry.predictRay(frame, params.frontLeftSensor, maze);
            const GeometryPrediction rightPrediction = geometry.predictRay(frame, params.frontRightSensor, maze);
            Assert::IsTrue(leftPrediction.hit);
            Assert::IsTrue(rightPrediction.hit);

            SrUkfCore core = MakeDefaultSrUkfCore();
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

            const FrontPairUpdateResult result = core.updateFrontPair(leftObservation, rightObservation, maze);
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
            const GeometryPrediction baseline =
                geometry.predictRay(BuildGeometryFrame(geometry, initialState), params.sideLeftSensor, maze);
            Assert::IsTrue(baseline.hit);

            SrUkfCore core = MakeDefaultSrUkfCore();
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs observation{};
            observation.valid = true;
            observation.confidence = 1.0f;
            observation.cls = ObsClass::WallLike;
            observation.rho = baseline.rangeM - 0.012f;

            const WallUpdateResult result = core.updateSideSensor(RelativeDirection::Left90, observation, maze);
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

            SrUkfCore core = MakeDefaultSrUkfCore();
            core.reset(initialState, initialCovariance);
            constexpr float dt = 0.001f;
            SyntheticEncoderRemainderState syntheticEncoderState{};

            for (int step = 0; step < 3000; ++step)
            {
                auto control =
                    model.solveSteadyStateFeedforward(
                        forwardVelocityTargetMps,
                        0.0f,
                        0.80f,
                        params.supplyVoltageV);

                RunPredictionMatchingCycle(
                    core,
                    control.control,
                    params,
                    dt,
                    syntheticEncoderState,
                    forwardVelocityTargetMps,
                    0.0f);
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
            constexpr float dt = 0.001f;
            auto core = RunUKFCycles(2000, App::Internal::CommandVector{});
            SyntheticEncoderRemainderState syntheticEncoderState{};

            for (int step = 0; step < 3000; ++step)
            {
                auto control =
                    model.solveSteadyStateFeedforward(
                        forwardVelocityTargetMps,
                        0.0f,
                        0.80f,
                        params.supplyVoltageV);

                RunPredictionMatchingCycle(
                    core,
                    control.control,
                    params,
                    dt,
                    syntheticEncoderState,
                    forwardVelocityTargetMps,
                    0.0f);
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

        TEST_METHOD(SrUkfCoreYawRateUpdateUsesGyroBiasStateInMeasurementEquation)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.02f,
                    -0.03f,
                    0.05f,
                    0.15f,
                    -0.04f,
                    0.11f,
                    0.0f,
                    0.0f,
                    0.08f);
            VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.010f, 0.010f, 0.020f, 0.015f, 0.015f, 0.020f, 0.030f);
            initialCovariance(VehicleState::kR, VehicleState::kBgz) = 0.0f;
            initialCovariance(VehicleState::kBgz, VehicleState::kR) = 0.0f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector before = core.state();
            constexpr float observedYawRateRadps = 0.27f;
            const float beforeError =
                std::fabs((before(VehicleState::kR) + before(VehicleState::kBgz)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.state();
            const float afterError =
                std::fabs((after(VehicleState::kR) + after(VehicleState::kBgz)) - observedYawRateRadps);

            Assert::IsTrue(afterError < beforeError);
            Assert::IsTrue(std::fabs(after(VehicleState::kBgz) - before(VehicleState::kBgz)) > 1.0e-5f);
            Assert::IsTrue(std::isfinite(after(VehicleState::kBgz)));
        }

        TEST_METHOD(SrUkfCoreMovingPredictDoesNotInjectGyroBiasProcessVariance)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.02f,
                    0.11f,
                    0.01f,
                    0.35f,
                    0.0f,
                    0.02f,
                    4.6f,
                    4.5f,
                    0.01f);
            Assert::IsTrue(core.reset(initialState, SrUkfCore::BuildDefaultInitialCovariance()));

            const float beforeVarianceRadps2 = core.covariance()(VehicleState::kBgz, VehicleState::kBgz);

            const CommandVector control = CommandVector(0.16f, 0.16f);
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;

            Assert::IsTrue(core.predict(0.002f, control, fanDutyCycle, batteryVoltageV));

            const float afterVarianceRadps2 = core.covariance()(VehicleState::kBgz, VehicleState::kBgz);
            Assert::AreEqual(2, FindDebugDumpModeId(core));
            Assert::AreEqual(beforeVarianceRadps2, afterVarianceRadps2, 1.0e-9f);
        }

        TEST_METHOD(SrUkfCoreExposesFrozenUkfPolicyStateForRuntimeFeedforward)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.05f,
                    0.07f,
                    0.04f,
                    0.30f,
                    0.02f,
                    -0.05f,
                    3.10f,
                    3.05f,
                    0.01f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.002f, 0.002f, 0.003f, 0.003f, 0.003f, 0.003f, 0.002f)));

            const CommandVector control = CommandVector(0.19f, 0.17f);
            const float fanDutyCycle = 0.80f;
            const float batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.002f;
            Assert::IsTrue(core.predict(dt, control, fanDutyCycle, batteryVoltageV));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 0;
            encoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            encoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);

            Assert::IsTrue(std::isfinite(FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "left_applied_bank_torque_nm")));
            Assert::IsTrue(std::isfinite(FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "right_applied_bank_torque_nm")));
            Assert::IsFalse(FindDebugDumpBool(core, "ukf_dump_grip_recovery", "exact_stationary_lock"));
            Assert::IsTrue(std::isfinite(FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "closure_residual_left_mps")));
            Assert::IsTrue(std::isfinite(FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "closure_residual_right_mps")));
        }

        TEST_METHOD(SrUkfCorePredictRefreshesAppliedTorqueFromCurrentControl)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel plant;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.01f,
                    0.02f,
                    0.03f,
                    0.45f,
                    -0.01f,
                    0.06f,
                    4.00f,
                    3.85f,
                    0.01f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.003f, 0.003f, 0.004f, 0.004f, 0.004f, 0.004f, 0.002f)));

            App::Internal::CommandVector firstControl{};
            firstControl.SetLeftMotorPwm(0.08f);
            firstControl.SetRightMotorPwm(0.06f);
            const float firstControlFanDutyCycle = 0.80f;
            const float firstControlBatteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            Assert::IsTrue(core.predict(dt, firstControl, firstControlFanDutyCycle, firstControlBatteryVoltageV));

            EncoderObs firstEncoder{};
            firstEncoder.totalLeftCounts = 0;
            firstEncoder.totalRightCounts = 0;
            firstEncoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            firstEncoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(firstEncoder, dt).accepted);

            (void)plant;
            (void)prepared;
            (void)initialState;
            (void)firstControlBatteryVoltageV;
            const float firstLeftAppliedBankTorqueNm =
                FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "left_applied_bank_torque_nm");
            const float firstRightAppliedBankTorqueNm =
                FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "right_applied_bank_torque_nm");
            Assert::IsTrue(std::isfinite(firstLeftAppliedBankTorqueNm));
            Assert::IsTrue(std::isfinite(firstRightAppliedBankTorqueNm));

            const VehicleState::StateVector stateBeforeSecondPredict = core.state();
            App::Internal::CommandVector secondControl{};
            secondControl.SetLeftMotorPwm(0.31f);
            secondControl.SetRightMotorPwm(0.27f);
            const float secondControlFanDutyCycle = 0.80f;
            const float secondControlBatteryVoltageV = params.supplyVoltageV;

            Assert::IsTrue(core.predict(dt, secondControl, secondControlFanDutyCycle, secondControlBatteryVoltageV));

            EncoderObs secondEncoder{};
            secondEncoder.totalLeftCounts = 0;
            secondEncoder.totalRightCounts = 0;
            secondEncoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            secondEncoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(secondEncoder, dt).accepted);

            (void)stateBeforeSecondPredict;
            (void)secondControlBatteryVoltageV;
            const float secondLeftAppliedBankTorqueNm =
                FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "left_applied_bank_torque_nm");
            const float secondRightAppliedBankTorqueNm =
                FindDebugDumpFloat(core, "ukf_dump_grip_recovery", "right_applied_bank_torque_nm");
            Assert::IsTrue(std::isfinite(secondLeftAppliedBankTorqueNm));
            Assert::IsTrue(std::isfinite(secondRightAppliedBankTorqueNm));
            Assert::IsTrue(
                (std::fabs(firstLeftAppliedBankTorqueNm - secondLeftAppliedBankTorqueNm) > 1.0e-6f) ||
                (std::fabs(firstRightAppliedBankTorqueNm - secondRightAppliedBankTorqueNm) > 1.0e-6f));
        }
    };
}
