#include "pch.h"
#include "EstimatorYawMotionUpdateTestSupport.h"
#include "EstimatorEncoderMotionUpdateTestSupport.h"

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        YawRateExpectation ComputeYawRateExpectation(
            EstimatorTestRuntime& runtime,
            const StateVector& before,
            const CovarianceMatrix& beforeCovariance,
            const float observedYawRateRadps)
        {
            YawRateExpectation expectation;
            const float correctedYawRateRadps =
                observedYawRateRadps - runtime.runtimeState.GetGyroBiasZ();
            const float gyroScaleRadpsPerLsb =
                runtime.vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() *
                0.001f *
                DEG_TO_RAD_F;
            const float gyroScaleToleranceSigmaRadps =
                std::fabs(correctedYawRateRadps) *
                kEstimatorTestImuGyroSensitivityToleranceFraction /
                std::sqrt(3.0f);
            const float yawInnovation = correctedYawRateRadps - before(5);
            const float yawInnovationVariance =
                beforeCovariance(5, 5) +
                kEstimatorTestImuYawRateVarianceRadps2 +
                ((gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f) +
                (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps) +
                runtime.runtimeState.GetGyroBiasZVar();
            const float yawGain =
                beforeCovariance(5, 5) / yawInnovationVariance;
            expectation.yawRateRadps =
                before(5) + (yawGain * yawInnovation);
            expectation.yawVarianceRadps2 =
                beforeCovariance(5, 5) -
                (yawGain * yawInnovationVariance * yawGain);
            expectation.nis =
                (yawInnovation * yawInnovation) / yawInnovationVariance;
            return expectation;
        }

        MovingPredictGyroBiasScenario RunMovingPredictGyroBiasScenario()
        {
            MovingPredictGyroBiasScenario scenario;
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

            scenario.beforeGyroBiasVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();

            const CommandVector control = CommandVector(0.16f, 0.16f);
            scenario.predictAccepted = core.predict(0.002f, control);

            scenario.afterGyroBiasVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();
            scenario.predictedState = EstimatorMotionUpdateTest::WorkingState(core);
            return scenario;
        }

        LaunchEncoderScenario RunLaunchEncoderScenario()
        {
            LaunchEncoderScenario scenario;
            constexpr LaunchEncoderSample samples[kLaunchEncoderSampleCount] = {
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
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
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

            const CommandVector control = CommandVector(0.08f, 0.08f);
            for (int index = 0; index < kLaunchEncoderSampleCount; ++index)
            {
                const LaunchEncoderSample& sample = samples[index];
                if (!core.predict(sample.dtSeconds, control))
                {
                    scenario.firstIncompleteOperation = L"predict";
                    break;
                }

                const StateVector stateBeforeEncoder = EstimatorMotionUpdateTest::WorkingState(core);
                const CovarianceMatrix covarianceBeforeEncoder = EstimatorMotionUpdateTest::WorkingCovariance(core);

                EncoderObs encoder{};
                const float leftCounts =
                    (Vehicle::WheelLinearVelocityFromWheelSpeed(sample.leftWheelSpeedRadps) *
                        sample.dtSeconds) /
                    distancePerCountM;
                const float rightCounts =
                    (Vehicle::WheelLinearVelocityFromWheelSpeed(sample.rightWheelSpeedRadps) *
                        sample.dtSeconds) /
                    distancePerCountM;
                encoder.SetTotalLeftCounts(static_cast<int32_t>(
                        (leftCounts >= 0.0f) ? (leftCounts + 0.5f) : (leftCounts - 0.5f)));
                encoder.SetTotalRightCounts(static_cast<int32_t>(
                        (rightCounts >= 0.0f) ? (rightCounts + 0.5f) : (rightCounts - 0.5f)));
                encoder.SetLeftWheelSpeedRadps(sample.leftWheelSpeedRadps);
                encoder.SetRightWheelSpeedRadps(sample.rightWheelSpeedRadps);

                const EncoderPairExpectation encoderExpectation =
                    ComputeEncoderPairExpectation(
                        runtime.plantModel,
                        encoder,
                        stateBeforeEncoder,
                        covarianceBeforeEncoder,
                        kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                        kEstimatorTestGeneralEncoderYawRateSigmaRadps);
                const bool encoderAccepted =
                    core.updateEncoderPair(encoder, sample.dtSeconds, true);
                if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
                {
                    scenario.firstIncompleteOperation = L"encoder update not attempted";
                    break;
                }
                if (!encoderAccepted)
                {
                    scenario.firstIncompleteOperation = L"encoder update rejected";
                    break;
                }
                if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
                {
                    scenario.firstIncompleteOperation = L"encoder update not recorded accepted";
                    break;
                }
                RecordDifference(
                    scenario.yawNisDifference,
                    EstimatorMotionUpdateTest::LastUpdateNis(core) - encoderExpectation.nis,
                    0,
                    -1,
                    index);

                const StateVector stateAfterEncoder = EstimatorMotionUpdateTest::WorkingState(core);
                const CovarianceMatrix covarianceAfterEncoder = EstimatorMotionUpdateTest::WorkingCovariance(core);
                RecordDifference(
                    scenario.encoderBodyStateDifference,
                    MaxBodyStateDifference(encoderExpectation.bodyState, stateAfterEncoder),
                    index);
                RecordDifference(
                    scenario.encoderUnmeasuredStateDifference,
                    MaxUnmeasuredStateDifference(stateBeforeEncoder, stateAfterEncoder),
                    index);
                RecordDifference(
                    scenario.encoderBodyCovarianceDifference,
                    MaxBodyCovarianceDifference(
                        encoderExpectation.bodyCovariance,
                        covarianceAfterEncoder),
                    index);
                RecordDifference(
                    scenario.encoderCrossCovarianceDifference,
                    MaxBodyResidualCrossCovariance(covarianceAfterEncoder),
                    index);
                RecordDifference(
                    scenario.encoderResidualVarianceDifference,
                    MaxResidualVarianceDifference(
                        covarianceBeforeEncoder,
                        covarianceAfterEncoder),
                    index);
                scenario.completedEncoderSamples = index + 1;

                const StateVector stateBeforeYaw = EstimatorMotionUpdateTest::WorkingState(core);
                const CovarianceMatrix covarianceBeforeYaw = EstimatorMotionUpdateTest::WorkingCovariance(core);
                const YawRateExpectation yawExpectation =
                    ComputeYawRateExpectation(
                        runtime,
                        stateBeforeYaw,
                        covarianceBeforeYaw,
                        sample.gyroRawRadps);
                const bool yawAccepted = core.updateYawRate(sample.gyroRawRadps);
                if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
                {
                    scenario.firstIncompleteOperation = L"yaw update not attempted";
                    break;
                }
                if (!yawAccepted)
                {
                    scenario.firstIncompleteOperation = L"yaw update rejected";
                    break;
                }
                if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
                {
                    scenario.firstIncompleteOperation = L"yaw update not recorded accepted";
                    break;
                }
                RecordDifference(
                    scenario.yawNisDifference,
                    EstimatorMotionUpdateTest::LastUpdateNis(core) - yawExpectation.nis,
                    1,
                    -1,
                    index);

                const StateVector stateAfterYaw = EstimatorMotionUpdateTest::WorkingState(core);
                const CovarianceMatrix covarianceAfterYaw = EstimatorMotionUpdateTest::WorkingCovariance(core);
                RecordDifference(
                    scenario.yawRateDifference,
                    stateAfterYaw(5) - yawExpectation.yawRateRadps,
                    5,
                    -1,
                    index);
                constexpr int kYawUnmeasuredIndices[] = { 3, 6, 7, 8 };
                RecordDifference(
                    scenario.yawUnmeasuredStateDifference,
                    MaxStateDifference(
                        stateBeforeYaw,
                        stateAfterYaw,
                        kYawUnmeasuredIndices),
                    index);
                RecordDifference(
                    scenario.yawVarianceDifference,
                    covarianceAfterYaw(5, 5) - yawExpectation.yawVarianceRadps2,
                    5,
                    5,
                    index);
                RecordDifference(
                    scenario.yawCrossCovarianceDifference,
                    MaxYawResidualRowCovariance(covarianceAfterYaw),
                    index);
                scenario.completedYawSamples = index + 1;
            }
            return scenario;
        }

        YawRateUpdateScenario RunYawResidualCrossScenario()
        {
            YawRateUpdateScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.02f;
            initialState(1) = 0.14f;
            initialState(2) = 0.10f;
            initialState(3) = 1.40f;
            initialState(4) = 0.05f;
            initialState(6) = 0.12f;
            initialState(7) = -0.08f;
            initialState(8) = 0.03f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.02f * 0.02f;
            initialCovariance(4, 4) = 0.02f * 0.02f;
            initialCovariance(5, 5) = 0.30f * 0.30f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            initialCovariance(5, 6) = 0.010f;
            initialCovariance(6, 5) = 0.010f;
            initialCovariance(5, 7) = -0.012f;
            initialCovariance(7, 5) = -0.012f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            scenario.beforeState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.observedYawRateRadps = 0.45f;
            scenario.expectation = ComputeYawRateExpectation(
                runtime,
                scenario.beforeState,
                scenario.beforeCovariance,
                scenario.observedYawRateRadps);
            scenario.updateReturnedAccepted =
                core.updateYawRate(scenario.observedYawRateRadps);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.actualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }

        YawRateUpdateScenario RunBiasedYawRateScenario()
        {
            YawRateUpdateScenario scenario;
            EstimatorTestRuntime runtime;
            runtime.runtimeState.SetGyroBiasZ(0.08f);
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.02f;
            initialState(1) = -0.03f;
            initialState(2) = 0.05f;
            initialState(3) = 0.15f;
            initialState(4) = -0.04f;
            initialState(5) = 0.11f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.010f * 0.010f;
            initialCovariance(1, 1) = 0.010f * 0.010f;
            initialCovariance(2, 2) = 0.010f * 0.010f;
            initialCovariance(3, 3) = 0.020f * 0.020f;
            initialCovariance(4, 4) = 0.015f * 0.015f;
            initialCovariance(5, 5) = 0.015f * 0.015f;
            initialCovariance(6, 6) = 0.020f * 0.020f;
            initialCovariance(7, 7) = 0.020f * 0.020f;
            initialCovariance(8, 8) = 0.030f * 0.030f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            scenario.beforeState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.beforeGyroBiasRadps = runtime.runtimeState.GetGyroBiasZ();
            scenario.observedYawRateRadps = 0.27f;
            scenario.expectation = ComputeYawRateExpectation(
                runtime,
                scenario.beforeState,
                scenario.beforeCovariance,
                scenario.observedYawRateRadps);
            scenario.updateReturnedAccepted =
                core.updateYawRate(scenario.observedYawRateRadps);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.actualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.afterGyroBiasRadps = runtime.runtimeState.GetGyroBiasZ();
            return scenario;
        }
    }
}
