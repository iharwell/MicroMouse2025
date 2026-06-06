#include "pch.h"
#include "EstimatorYawMotionUpdateTestSupport.h"

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        MovingPredictResidualScenario RunMovingPredictResidualScenario()
        {
            MovingPredictResidualScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            scenario.initialState(0) = 0.02f;
            scenario.initialState(1) = 0.11f;
            scenario.initialState(2) = (0.01f);
            scenario.initialState(3) = 0.35f;
            scenario.initialState(4) = 0.0f;
            scenario.initialState(5) = 0.02f;
            scenario.initialState(6) = 0.0f;
            scenario.initialState(7) = 0.0f;
            scenario.initialState(8) = 0.0f;
            scenario.resetAccepted =
                EstimatorMotionUpdateTest::Reset(core, scenario.initialState, EstimatorMotionUpdateTest::BuildDefaultInitialCovariance());

            const CommandVector control = CommandVector(0.16f, 0.16f);
            scenario.predictAccepted = core.predict(0.002f, control);

            scenario.predictedState = EstimatorMotionUpdateTest::WorkingState(core);
            return scenario;
        }

        LaunchEncoderPredictionScenario RunLaunchEncoderPredictionScenario()
        {
            LaunchEncoderPredictionScenario scenario;
            constexpr LaunchEncoderSample samples[kLaunchEncoderSampleCount] = {
                { 0.001011f, 3.99f, 3.99f },
                { 0.001015f, 9.67f, 15.20f },
                { 0.001005f, 8.72f, 13.76f },
                { 0.001020f, 6.49f, 12.05f },
                { 0.001004f, 5.02f, 9.59f },
                { 0.001019f, 4.64f, 8.35f },
                { 0.001005f, 5.48f, 6.40f },
                { 0.001000f, 6.49f, 6.49f },
                { 0.001005f, 8.38f, 5.59f },
                { 0.001000f, 10.19f, 6.95f },
            };
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.225f;
            initialState(1) = 0.225f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.05f * 0.05f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            if (!scenario.resetAccepted)
            {
                scenario.firstIncompleteOperation = L"reset";
                return scenario;
            }

            const CommandVector control = CommandVector(0.08f, 0.08f);
            for (int index = 0; index < kLaunchEncoderSampleCount; ++index)
            {
                const LaunchEncoderSample& sample = samples[index];
                SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
                SetEncoderCountDeltasForWheelSpeedsOverTick(
                    encoder,
                    sample.leftWheelSpeedRadps,
                    sample.rightWheelSpeedRadps,
                    sample.dtSeconds);
                const SensorSnapshot::EncoderObs publishedEncoder =
                    PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, sample.dtSeconds);
                const float expectedForwardSpeedMps =
                    0.5f * (publishedEncoder.LeftVelocityMps() + publishedEncoder.RightVelocityMps());
                const float expectedYawRateRadps =
                    Vehicle::BodyYawRateFromWheelLinear(
                        publishedEncoder.LeftVelocityMps(),
                        publishedEncoder.RightVelocityMps());
                RecordDifference(
                    scenario.measuredForwardSpeedDifference,
                    Vehicle::BodyForwardVelocityFromWheelLinear(
                        Vehicle::WheelLinearVelocityFromWheelSpeed(publishedEncoder.LeftWheelSpeedRadps()),
                        Vehicle::WheelLinearVelocityFromWheelSpeed(publishedEncoder.RightWheelSpeedRadps())) -
                        expectedForwardSpeedMps,
                    3,
                    -1,
                    index);
                RecordDifference(
                    scenario.measuredYawRateDifference,
                    Vehicle::BodyYawRateFromWheelLinear(
                        Vehicle::WheelLinearVelocityFromWheelSpeed(publishedEncoder.LeftWheelSpeedRadps()),
                        Vehicle::WheelLinearVelocityFromWheelSpeed(publishedEncoder.RightWheelSpeedRadps())) -
                        expectedYawRateRadps,
                    5,
                    -1,
                    index);

                if (!core.predict(sample.dtSeconds, control))
                {
                    scenario.firstIncompleteOperation = L"predict";
                    break;
                }
                scenario.completedPredictSamples = index + 1;
                scenario.predictionEncoderInputObserved =
                    FindDebugDumpBool(core, "estimator_dump_update_metrics", "prediction_encoder_input");
                scenario.finalEncoder = publishedEncoder;
                scenario.dumpLeftWheelSpeedRadps =
                    FindDebugDumpFloat(core, "estimator_dump_prediction_encoder_input", "left_wheel_speed_radps");
                scenario.dumpRightWheelSpeedRadps =
                    FindDebugDumpFloat(core, "estimator_dump_prediction_encoder_input", "right_wheel_speed_radps");
                if (!scenario.predictionEncoderInputObserved)
                {
                    scenario.firstIncompleteOperation = L"prediction encoder input not recorded";
                    break;
                }
            }
            return scenario;
        }
    }
}
