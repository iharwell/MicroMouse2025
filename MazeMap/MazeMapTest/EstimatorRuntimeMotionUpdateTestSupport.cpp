#include "pch.h"
#include "EstimatorRuntimeMotionUpdateTestSupport.h"

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        FeedforwardRuntimeScenario RunFeedforwardRuntimeScenario()
        {
            FeedforwardRuntimeScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.05f;
            initialState(1) = 0.07f;
            initialState(2) = 0.04f;
            initialState(3) = 0.30f;
            initialState(4) = 0.02f;
            initialState(5) = -0.05f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.002f * 0.002f;
            initialCovariance(1, 1) = 0.002f * 0.002f;
            initialCovariance(2, 2) = 0.002f * 0.002f;
            initialCovariance(3, 3) = 0.003f * 0.003f;
            initialCovariance(4, 4) = 0.003f * 0.003f;
            initialCovariance(5, 5) = 0.003f * 0.003f;
            initialCovariance(6, 6) = 0.003f * 0.003f;
            initialCovariance(7, 7) = 0.003f * 0.003f;
            initialCovariance(8, 8) = 0.002f * 0.002f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);

            scenario.beforePredictState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforePredictCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            const CommandVector control = CommandVector(0.19f, 0.17f);
            constexpr float dt = 0.002f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                scenario.beforePredictState(3),
                scenario.beforePredictState(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            SetEncoderCountDeltasForWheelSpeedsOverTick(
                scenario.encoder,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps,
                dt);
            scenario.encoder =
                PublishEncoderObservationToRuntime(runtime.runtimeState, scenario.encoder, dt);
            scenario.runtimeLeftWheelSpeedBeforePredictRadps =
                runtime.runtimeState.GetWheelSpeedLeft();
            scenario.runtimeRightWheelSpeedBeforePredictRadps =
                runtime.runtimeState.GetWheelSpeedRight();

            const float measuredLeftVelocityMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(scenario.encoder.LeftWheelSpeedRadps());
            const float measuredRightVelocityMps =
                Vehicle::WheelLinearVelocityFromWheelSpeed(scenario.encoder.RightWheelSpeedRadps());
            scenario.measuredForwardVelocityMps =
                Vehicle::BodyForwardVelocityFromWheelLinear(
                    measuredLeftVelocityMps,
                    measuredRightVelocityMps);
            scenario.measuredYawRateRadps =
                Vehicle::BodyYawRateFromWheelLinear(
                    measuredLeftVelocityMps,
                    measuredRightVelocityMps);
            scenario.travelForwardVelocityMps =
                Vehicle::BodyForwardVelocityFromWheelLinear(
                    scenario.encoder.LeftDistanceDeltaM() / dt,
                    scenario.encoder.RightDistanceDeltaM() / dt);
            scenario.travelYawRateRadps =
                Vehicle::BodyYawRateFromWheelLinear(
                    scenario.encoder.LeftDistanceDeltaM() / dt,
                    scenario.encoder.RightDistanceDeltaM() / dt);
            scenario.predictAccepted = core.predict(dt, control);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.runtimeLeftWheelSpeedAfterPredictRadps =
                runtime.runtimeState.GetWheelSpeedLeft();
            scenario.runtimeRightWheelSpeedAfterPredictRadps =
                runtime.runtimeState.GetWheelSpeedRight();
            return scenario;
        }

        TorqueRefreshScenario RunTorqueRefreshScenario()
        {
            TorqueRefreshScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.01f;
            initialState(1) = 0.02f;
            initialState(2) = 0.03f;
            initialState(3) = 0.45f;
            initialState(4) = -0.01f;
            initialState(5) = 0.06f;
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.003f * 0.003f;
            initialCovariance(1, 1) = 0.003f * 0.003f;
            initialCovariance(2, 2) = 0.003f * 0.003f;
            initialCovariance(3, 3) = 0.004f * 0.004f;
            initialCovariance(4, 4) = 0.004f * 0.004f;
            initialCovariance(5, 5) = 0.004f * 0.004f;
            initialCovariance(6, 6) = 0.004f * 0.004f;
            initialCovariance(7, 7) = 0.004f * 0.004f;
            initialCovariance(8, 8) = 0.002f * 0.002f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);

            SensorSnapshot::EncoderObs firstEncoder = SensorSnapshot{}.EncoderObservation();
            constexpr float dt = 0.002f;
            float firstLeftWheelSpeedRadps = 0.0f;
            float firstRightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                EstimatorMotionUpdateTest::WorkingState(core)(3),
                EstimatorMotionUpdateTest::WorkingState(core)(5),
                firstLeftWheelSpeedRadps,
                firstRightWheelSpeedRadps);
            SetEncoderCountDeltasForWheelSpeedsOverTick(
                firstEncoder,
                firstLeftWheelSpeedRadps,
                firstRightWheelSpeedRadps,
                dt);
            (void)PublishEncoderObservationToRuntime(runtime.runtimeState, firstEncoder, dt);

            CommandVector firstControl{};
            firstControl.SetLeftCommand(0.08f);
            firstControl.SetRightCommand(0.06f);
            scenario.firstPredictAccepted = core.predict(dt, firstControl);

            const StateVector stateBeforeSecondPredict = EstimatorMotionUpdateTest::WorkingState(core);
            const CovarianceMatrix covarianceBeforeSecondPredict =
                EstimatorMotionUpdateTest::WorkingCovariance(core);
            CommandVector secondControl{};
            secondControl.SetLeftCommand(0.31f);
            secondControl.SetRightCommand(0.27f);

            Estimator firstControlCore = MakeDefaultEstimator();
            Estimator secondControlCore = MakeDefaultEstimator();
            scenario.firstControlResetAccepted =
                EstimatorMotionUpdateTest::Reset(firstControlCore,
                    stateBeforeSecondPredict,
                    covarianceBeforeSecondPredict);
            scenario.secondControlResetAccepted =
                EstimatorMotionUpdateTest::Reset(secondControlCore,
                    stateBeforeSecondPredict,
                    covarianceBeforeSecondPredict);
            scenario.firstControlPredictAccepted =
                firstControlCore.predict(dt, firstControl);
            scenario.secondControlPredictAccepted =
                secondControlCore.predict(dt, secondControl);

            scenario.responseDelta =
                (EstimatorMotionUpdateTest::WorkingState(firstControlCore) - EstimatorMotionUpdateTest::WorkingState(secondControlCore))
                    .cwiseAbs()
                    .maxCoeff();
            return scenario;
        }
    }
}
