#include "pch.h"
#include "CppUnitTest.h"

#include "SrUkfCoreTestSupport.h"
#include "TimeStepPropagationTestSupport.h"
#include "..\MazeMap\EstimatorPredictModel.h"
#include "..\MazeMap\TorqueEstimateAdapter.h"

#include <cmath>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kZeroVelocityToleranceMps = 0.008f;

        VehicleState::StateVector PredictStationarySplitCommandStateAfterFifteenPredicts()
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
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

            ControlInput control{};
            control.leftMotorCommand = 0.60f;
            control.rightMotorCommand = -0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dtSeconds = 0.01f;
            constexpr int kPredictSteps = 15;
            const float pivotScrubCommandAngularRadps =
                SrUkfCore::GetRuntimeTuning().pivotScrubMinCommandAngularRadps;
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
    }

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
                initialYawRateVarianceRadps2);
            Assert::IsTrue(
                afterCovariance(VehicleState::kPsi, VehicleState::kPsi) <
                initialYawRateVarianceRadps2);
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

        TEST_METHOD(SrUkfCoreEncoderPairDirectUpdateKeepsBodyStateInvariantWhileUpdatingWheelStates)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);

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

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.010f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateVector& stateAfterEncoder = core.state();
            Assert::IsTrue(core.directWheelUpdateBodyStateInvariant());
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
            SrUkfCore core(params);

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

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.010f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);
            Assert::IsTrue(encoderResult.nis > SrUkfCore::GetRuntimeTuning().encoderPairNisThreshold);

            const VehicleState::StateVector& stateAfterEncoder = core.state();
            Assert::IsTrue(core.directWheelUpdateBodyStateInvariant());
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

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandKeepsZeroForwardVelocity)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterFifteenPredicts();

            Assert::AreEqual(0.0f, predictedState(VehicleState::kU), kZeroVelocityToleranceMps);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandProducesPositiveLeftWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterFifteenPredicts();

            Assert::IsTrue(predictedState(VehicleState::kOmegaL) > 0.0f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandProducesNegativeRightWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterFifteenPredicts();

            Assert::IsTrue(predictedState(VehicleState::kOmegaR) < 0.0f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandKeepsZeroAverageWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterFifteenPredicts();
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
            const GeometryPrediction leftPrediction = geometry.predictRay(initialState, params.frontLeftSensor, maze);
            const GeometryPrediction rightPrediction = geometry.predictRay(initialState, params.frontRightSensor, maze);
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
            const GeometryPrediction baseline = geometry.predictRay(initialState, params.sideLeftSensor, maze);
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

            const WallUpdateResult result = core.updateSideSensor(Side::Left, observation, maze);
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
            constexpr float dt = 0.001f;
            SyntheticEncoderRemainderState syntheticEncoderState{};

            for (int step = 0; step < 3000; ++step)
            {
                auto control =
                    model.solveDriveCommandsForVelocityTarget(
                        core.state()(VehicleState::kU),
                        forwardVelocityTargetMps,
                        core.state()(VehicleState::kR),
                        0.0f,
                        params);

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
            auto core = RunUKFCycles(2000, ControlInput{});
            SyntheticEncoderRemainderState syntheticEncoderState{};

            for (int step = 0; step < 3000; ++step)
            {
                auto control =
                    model.solveDriveCommandsForVelocityTarget(
                        core.state()(VehicleState::kU),
                        forwardVelocityTargetMps,
                        core.state()(VehicleState::kR),
                        0.0f,
                        params);

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
            SrUkfCore core;
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

        TEST_METHOD(SrUkfCoreExposesFrozenCycleContextForRuntimeFeedforward)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
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

            ControlInput control{};
            control.leftMotorCommand = 0.19f;
            control.rightMotorCommand = 0.17f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.002f;
            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 0;
            encoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            encoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);

            const ModelCycleContext& cycleContext = core.modelCycleContext();
            Assert::AreEqual(dt, cycleContext.dtS, 1.0e-7f);
            Assert::IsTrue(std::isfinite(cycleContext.appliedTorque.leftAppliedBankTorqueNm));
            Assert::IsTrue(std::isfinite(cycleContext.appliedTorque.rightAppliedBankTorqueNm));
            Assert::IsFalse(cycleContext.schedule.exactStationaryLock);
            Assert::IsTrue(std::isfinite(cycleContext.geometry.effectiveWheelRadiusM));
            Assert::IsTrue(std::isfinite(core.closureResidualLeftMps()));
            Assert::IsTrue(std::isfinite(core.closureResidualRightMps()));
        }

        TEST_METHOD(EstimatorPredictModelExactStationaryLockKeepsPoseFixedAndContractsDynamics)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            EstimatorPredictModel predictModel;

            EstimatorPredictModel::PredictInput input{};
            input.currentState = BuildUkfState(
                0.12f,
                -0.08f,
                0.35f,
                0.40f,
                -0.30f,
                0.25f,
                6.0f,
                -5.0f,
                0.02f);
            input.leftAppliedBankTorqueNm = 0.10f;
            input.rightAppliedBankTorqueNm = -0.08f;
            input.dtS = 0.010f;

            ModelCycleContext cycleContext{};
            cycleContext.schedule.exactStationaryLock = true;
            input.cycleContext = &cycleContext;

            const EstimatorPredictModel::PredictOutput output = predictModel.Integrate(input, prepared);

            Assert::AreEqual(input.currentState(VehicleState::kPx), output.nextState(VehicleState::kPx), 1.0e-9f);
            Assert::AreEqual(input.currentState(VehicleState::kPy), output.nextState(VehicleState::kPy), 1.0e-9f);
            Assert::AreEqual(
                VehicleState::NormalizeAngle(input.currentState(VehicleState::kPsi)),
                output.nextState(VehicleState::kPsi),
                1.0e-9f);
            Assert::IsTrue(std::fabs(output.nextState(VehicleState::kU)) < std::fabs(input.currentState(VehicleState::kU)));
            Assert::IsTrue(std::fabs(output.nextState(VehicleState::kV)) < std::fabs(input.currentState(VehicleState::kV)));
            Assert::IsTrue(std::fabs(output.nextState(VehicleState::kR)) < std::fabs(input.currentState(VehicleState::kR)));
            Assert::IsTrue(std::fabs(output.nextState(VehicleState::kOmegaL)) < std::fabs(input.currentState(VehicleState::kOmegaL)));
            Assert::IsTrue(std::fabs(output.nextState(VehicleState::kOmegaR)) < std::fabs(input.currentState(VehicleState::kOmegaR)));
            Assert::AreEqual(input.currentState(VehicleState::kBgz), output.nextState(VehicleState::kBgz), 1.0e-9f);
        }

        TEST_METHOD(SrUkfCorePredictRefreshesFrozenCycleContextFromCurrentControl)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel plant;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            SrUkfCore core(params);
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

            ControlInput firstControl{};
            firstControl.leftMotorCommand = 0.08f;
            firstControl.rightMotorCommand = 0.06f;
            firstControl.fanDutyCycle = 0.80f;
            firstControl.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            Assert::IsTrue(core.predict(dt, firstControl));

            EncoderObs firstEncoder{};
            firstEncoder.totalLeftCounts = 0;
            firstEncoder.totalRightCounts = 0;
            firstEncoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            firstEncoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(firstEncoder, dt).accepted);

            const ModelCycleContext firstContext = core.modelCycleContext();
            const AppliedTorqueEstimate expectedFirstTorque =
                TorqueEstimateAdapter::Estimate(
                    plant,
                    initialState,
                    firstControl,
                    prepared,
                    firstControl.batteryVoltageV);
            Assert::AreEqual(
                expectedFirstTorque.leftAppliedBankTorqueNm,
                firstContext.appliedTorque.leftAppliedBankTorqueNm,
                1.0e-6f);
            Assert::AreEqual(
                expectedFirstTorque.rightAppliedBankTorqueNm,
                firstContext.appliedTorque.rightAppliedBankTorqueNm,
                1.0e-6f);

            const VehicleState::StateVector stateBeforeSecondPredict = core.state();
            ControlInput secondControl{};
            secondControl.leftMotorCommand = 0.31f;
            secondControl.rightMotorCommand = 0.27f;
            secondControl.fanDutyCycle = 0.80f;
            secondControl.batteryVoltageV = params.supplyVoltageV;

            Assert::IsTrue(core.predict(dt, secondControl));

            EncoderObs secondEncoder{};
            secondEncoder.totalLeftCounts = 0;
            secondEncoder.totalRightCounts = 0;
            secondEncoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            secondEncoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(secondEncoder, dt).accepted);

            const ModelCycleContext secondContext = core.modelCycleContext();
            const AppliedTorqueEstimate expectedSecondTorque =
                TorqueEstimateAdapter::Estimate(
                    plant,
                    stateBeforeSecondPredict,
                    secondControl,
                    prepared,
                    secondControl.batteryVoltageV);
            Assert::AreEqual(
                expectedSecondTorque.leftAppliedBankTorqueNm,
                secondContext.appliedTorque.leftAppliedBankTorqueNm,
                1.0e-6f);
            Assert::AreEqual(
                expectedSecondTorque.rightAppliedBankTorqueNm,
                secondContext.appliedTorque.rightAppliedBankTorqueNm,
                1.0e-6f);
        }

        TEST_METHOD(EstimatorPredictModelUsesRuntimeFanDutyCycleForAppliedTorquePropagation)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector state =
                BuildUkfState(
                    0.01f,
                    0.02f,
                    0.03f,
                    0.40f,
                    0.06f,
                    2.4f,
                    13.5f,
                    11.8f,
                    0.0f);

            EstimatorPredictModel model;
            EstimatorPredictModel::PredictInput lowFanInput{};
            lowFanInput.currentState = state;
            lowFanInput.leftAppliedBankTorqueNm = 0.055f;
            lowFanInput.rightAppliedBankTorqueNm = 0.049f;
            lowFanInput.fanDutyCycle = 0.15f;
            lowFanInput.dtS = 0.002f;

            EstimatorPredictModel::PredictInput highFanInput = lowFanInput;
            highFanInput.fanDutyCycle = 0.95f;

            const auto lowFanOutput = model.Integrate(lowFanInput, prepared);
            const auto highFanOutput = model.Integrate(highFanInput, prepared);

            const VehicleState::StateVector expectedLowFanState =
                EstimatorPredictModel::SemiImplicitAdvance(
                    state,
                    model.EvaluateStep(
                        lowFanInput,
                        prepared),
                    lowFanInput.dtS);
            const VehicleState::StateVector expectedHighFanState =
                EstimatorPredictModel::SemiImplicitAdvance(
                    state,
                    model.EvaluateStep(
                        highFanInput,
                        prepared),
                    highFanInput.dtS);

            for (int index = 0; index < lowFanOutput.nextState.size(); ++index)
            {
                Assert::AreEqual(expectedLowFanState(index), lowFanOutput.nextState(index), 1.0e-6f);
                Assert::AreEqual(expectedHighFanState(index), highFanOutput.nextState(index), 1.0e-6f);
            }

        }
    };
}
