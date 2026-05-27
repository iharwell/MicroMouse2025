#include "pch.h"
#include "EstimatorEncoderMotionUpdateTestSupport.h"

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        EncoderPairExpectation ComputeEncoderPairExpectation(
            PlantModel& plant,
            const EncoderObs& encoder,
            const StateVector& priorState,
            const CovarianceMatrix& priorCovariance,
            const float linearSpeedSigmaMps,
            const float yawRateSigmaRadps)
        {
            EncoderPairExpectation expectation;
            Eigen::Vector2f encoderMeasurement;
            encoderMeasurement <<
                plant.measuredLinearSpeedMps(encoder),
                plant.measuredYawRateRadps(encoder);
            Eigen::Vector2f priorBodyState;
            priorBodyState << priorState(3), priorState(5);
            Eigen::Matrix2f priorBodyCovariance;
            priorBodyCovariance <<
                priorCovariance(3, 3), priorCovariance(3, 5),
                priorCovariance(5, 3), priorCovariance(5, 5);
            Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
            encoderNoise(0, 0) = linearSpeedSigmaMps * linearSpeedSigmaMps;
            encoderNoise(1, 1) = yawRateSigmaRadps * yawRateSigmaRadps;

            const Eigen::Matrix2f innovationCovariance =
                priorBodyCovariance + encoderNoise;
            const float innovationDeterminant =
                (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                (innovationCovariance(0, 1) * innovationCovariance(1, 0));
            Eigen::Matrix2f innovationCovarianceInverse;
            innovationCovarianceInverse <<
                innovationCovariance(1, 1) / innovationDeterminant,
                -innovationCovariance(0, 1) / innovationDeterminant,
                -innovationCovariance(1, 0) / innovationDeterminant,
                innovationCovariance(0, 0) / innovationDeterminant;
            const Eigen::Matrix2f bodyGain =
                priorBodyCovariance * innovationCovarianceInverse;
            expectation.bodyState =
                priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
            expectation.bodyCovariance =
                priorBodyCovariance -
                (bodyGain * innovationCovariance * bodyGain.transpose());
            expectation.nis =
                ((encoderMeasurement - priorBodyState).transpose() *
                 innovationCovarianceInverse *
                 (encoderMeasurement - priorBodyState))(0);
            return expectation;
        }

        EncoderPairUpdateScenario RunZeroVelocityEncoderPairScenario()
        {
            EncoderPairUpdateScenario scenario;
            EstimatorTestRuntime runtime;
            PlantModel& plant = runtime.plantModel;
            Estimator core = MakeDefaultEstimator();
            constexpr float dt = kDefaultEstimatorDtSeconds;

            scenario.initialState = BuildRestingState();
            const CovarianceMatrix initialCovariance = BuildTightInitialCovariance();
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, scenario.initialState, initialCovariance);

            const CommandVector control{};
            PublishStateToRuntime(runtime.runtimeState, scenario.initialState);
            scenario.forwardContactForceN = plant.totalForwardContactForceN(control);
            scenario.rightContactForceN = plant.totalRightContactForceN(control);
            plant.integrate(control, dt);
            scenario.runtimeForwardAccelerationMps2 =
                runtime.runtimeState.GetForwardAcceleration();
            scenario.runtimeRightAccelerationMps2 =
                runtime.runtimeState.GetRightAcceleration();
            scenario.runtimeForwardVelocityMps =
                runtime.runtimeState.GetForwardVelocity();
            scenario.runtimeRightVelocityMps =
                runtime.runtimeState.GetRightwardVelocity();
            scenario.runtimeYawRateRadps = runtime.runtimeState.GetYawRate();

            scenario.beforePredictCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.initialYawRateVarianceRadps2 =
                scenario.beforePredictCovariance(5, 5);
            scenario.predictAccepted = core.predict(dt, control);
            scenario.priorState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.priorCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);

            EncoderObs encoder{};
            const float stationaryYawSigmaRadps =
                kEstimatorTestStationaryEncoderVelocitySigmaMps /
                Vehicle::GetPhysicalTrackWidthM();
            scenario.expectation = ComputeEncoderPairExpectation(
                plant,
                encoder,
                scenario.priorState,
                scenario.priorCovariance,
                kEstimatorTestStationaryEncoderVelocitySigmaMps,
                stationaryYawSigmaRadps);

            scenario.updateReturnedAccepted =
                core.updateEncoderPair(encoder, dt, true);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.actualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }

        EncoderPairUpdateScenario RunMovingEncoderPairScenario()
        {
            EncoderPairUpdateScenario scenario;
            EstimatorTestRuntime runtime;
            PlantModel& plant = runtime.plantModel;
            Estimator core = MakeDefaultEstimator();
            constexpr float dt = kDefaultEstimatorDtSeconds;
            constexpr int32_t measuredCounts = 1;
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            const float measuredLinearSpeedMps =
                (static_cast<float>(measuredCounts) * distancePerCountM) / dt;
            const float measuredWheelSpeedRadps =
                Vehicle::WheelSpeedFromLinearVelocity(measuredLinearSpeedMps);

            scenario.initialState = BuildRestingState(measuredLinearSpeedMps);
            scenario.expectedMeasuredLinearSpeedMps = measuredLinearSpeedMps;
            const CovarianceMatrix initialCovariance = BuildTightInitialCovariance();
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, scenario.initialState, initialCovariance);

            const CommandVector control{};
            PublishStateToRuntime(runtime.runtimeState, scenario.initialState);
            scenario.forwardContactForceN = plant.totalForwardContactForceN(control);
            scenario.rightContactForceN = plant.totalRightContactForceN(control);

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(measuredCounts);
            encoder.SetTotalRightCounts(measuredCounts);
            encoder.SetLeftWheelSpeedRadps(measuredWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(measuredWheelSpeedRadps);
            scenario.measuredLinearSpeedMps = plant.measuredLinearSpeedMps(encoder);
            scenario.measuredYawRateRadps = plant.measuredYawRateRadps(encoder);
            scenario.measuredYawRateVarianceRadps2 =
                plant.measuredYawRateVarianceRadps2(
                    encoder,
                    kEstimatorTestStationaryEncoderVelocitySigmaMps,
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps);
            scenario.measuredWheelVarianceRadps2 =
                plant.measuredWheelVarianceRadps2(
                    encoder,
                    kEstimatorTestStationaryEncoderVelocitySigmaMps,
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps);

            scenario.beforePredictCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.initialYawRateVarianceRadps2 =
                scenario.beforePredictCovariance(5, 5);
            scenario.predictAccepted = core.predict(dt, control);
            scenario.priorState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.priorCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.expectation = ComputeEncoderPairExpectation(
                plant,
                encoder,
                scenario.priorState,
                scenario.priorCovariance,
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                kEstimatorTestGeneralEncoderYawRateSigmaRadps);

            scenario.updateReturnedAccepted =
                core.updateEncoderPair(encoder, dt, true);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.actualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }

        LatestEncoderScenario RunLatestEncoderScenario()
        {
            LatestEncoderScenario scenario;
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const CommandVector control{};
            constexpr float dt = kDefaultEstimatorDtSeconds;

            scenario.firstPredictAccepted = core.predict(dt, control);
            EncoderObs first{};
            first.SetTotalLeftCounts(2);
            first.SetTotalRightCounts(-1);
            first.SetLeftWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(first.TotalLeftCounts()) * distancePerCountM) / dt));
            first.SetRightWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(first.TotalRightCounts()) * distancePerCountM) / dt));
            scenario.firstUpdateReturnedAccepted =
                core.updateEncoderPair(first, dt, true);
            scenario.firstUpdateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.firstUpdateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);

            scenario.secondPredictAccepted = core.predict(dt, control);
            scenario.beforeSecondState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeSecondCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);

            EncoderObs second{};
            second.SetTotalLeftCounts(5);
            second.SetTotalRightCounts(-3);
            second.SetLeftWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(second.TotalLeftCounts()) * distancePerCountM) / dt));
            second.SetRightWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(second.TotalRightCounts()) * distancePerCountM) / dt));
            scenario.expectation = ComputeEncoderPairExpectation(
                runtime.plantModel,
                second,
                scenario.beforeSecondState,
                scenario.beforeSecondCovariance,
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                kEstimatorTestGeneralEncoderYawRateSigmaRadps);

            scenario.secondUpdateReturnedAccepted =
                core.updateEncoderPair(second, dt, true);
            scenario.secondUpdateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.secondUpdateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.secondActualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterSecondState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterSecondCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }

        DirectEncoderScenario RunDirectEncoderScenario()
        {
            DirectEncoderScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.07f;
            initialState(1) = 0.19f;
            initialState(2) = 0.11f;
            initialState(3) = 0.42f;
            initialState(4) = 0.03f;
            initialState(5) = 0.18f;
            initialState(6) = 0.12f;
            initialState(7) = -0.08f;
            initialState(8) = 0.02f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.05f * 0.05f;
            initialCovariance(3, 3) = 0.20f * 0.20f;
            initialCovariance(4, 4) = 0.12f * 0.12f;
            initialCovariance(5, 5) = 0.18f * 0.18f;
            initialCovariance(6, 6) = 0.45f * 0.45f;
            initialCovariance(7, 7) = 0.45f * 0.45f;
            initialCovariance(8, 8) = 0.04f * 0.04f;
            initialCovariance(3, 6) = 0.020f;
            initialCovariance(6, 3) = 0.020f;
            initialCovariance(3, 7) = -0.018f;
            initialCovariance(7, 3) = -0.018f;
            initialCovariance(5, 6) = 0.012f;
            initialCovariance(6, 5) = 0.012f;
            initialCovariance(5, 7) = -0.010f;
            initialCovariance(7, 5) = -0.010f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            scenario.beforeState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(8);
            encoder.SetTotalRightCounts(-6);
            encoder.SetLeftWheelSpeedRadps(10.6f);
            encoder.SetRightWheelSpeedRadps(2.4f);
            scenario.expectation = ComputeEncoderPairExpectation(
                runtime.plantModel,
                encoder,
                scenario.beforeState,
                scenario.beforeCovariance,
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                kEstimatorTestGeneralEncoderYawRateSigmaRadps);

            scenario.updateReturnedAccepted =
                core.updateEncoderPair(encoder, kDefaultEstimatorDtSeconds, true);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.actualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }

        RejectedEncoderScenario RunRejectedEncoderScenario()
        {
            RejectedEncoderScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            StateVector initialState = StateVector::Zero();
            initialState(0) = -0.04f;
            initialState(1) = 0.16f;
            initialState(2) = -0.07f;
            initialState(3) = 0.31f;
            initialState(4) = -0.02f;
            initialState(5) = 0.09f;
            initialState(6) = 0.16f;
            initialState(7) = 0.14f;
            initialState(8) = -0.01f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.10f * 0.10f;
            initialCovariance(4, 4) = 0.08f * 0.08f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.08f * 0.08f;
            initialCovariance(7, 7) = 0.08f * 0.08f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            scenario.beforeState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(40);
            encoder.SetTotalRightCounts(-36);
            encoder.SetLeftWheelSpeedRadps(120.0f);
            encoder.SetRightWheelSpeedRadps(-115.0f);
            scenario.updateReturnedAccepted =
                core.updateEncoderPair(encoder, kDefaultEstimatorDtSeconds, true);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.nis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }
    }
}
