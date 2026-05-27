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

            const CommandVector control = CommandVector(0.19f, 0.17f);
            constexpr float dt = 0.002f;
            scenario.predictAccepted = core.predict(dt, control);

            scenario.encoder.SetTotalLeftCounts(1);
            scenario.encoder.SetTotalRightCounts(0);
            scenario.beforeEncoderState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeEncoderCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                scenario.beforeEncoderState(3),
                scenario.beforeEncoderState(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            scenario.encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            scenario.encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            scenario.expectation = ComputeEncoderPairExpectation(
                runtime.plantModel,
                scenario.encoder,
                scenario.beforeEncoderState,
                scenario.beforeEncoderCovariance,
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                kEstimatorTestGeneralEncoderYawRateSigmaRadps);
            runtime.runtimeState.SetWheelSpeedLeft(scenario.encoder.LeftWheelSpeedRadps());
            runtime.runtimeState.SetWheelSpeedRight(scenario.encoder.RightWheelSpeedRadps());
            scenario.runtimeLeftWheelSpeedBeforeUpdateRadps =
                runtime.runtimeState.GetWheelSpeedLeft();
            scenario.runtimeRightWheelSpeedBeforeUpdateRadps =
                runtime.runtimeState.GetWheelSpeedRight();

            scenario.updateReturnedAccepted =
                core.updateEncoderPair(scenario.encoder, dt, true);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.actualNis = EstimatorMotionUpdateTest::LastUpdateNis(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            scenario.runtimeLeftWheelSpeedAfterUpdateRadps =
                runtime.runtimeState.GetWheelSpeedLeft();
            scenario.runtimeRightWheelSpeedAfterUpdateRadps =
                runtime.runtimeState.GetWheelSpeedRight();
            return scenario;
        }

        TorqueRefreshScenario RunTorqueRefreshScenario()
        {
            TorqueRefreshScenario scenario;
            Estimator core = MakeDefaultEstimator();
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

            CommandVector firstControl{};
            firstControl.SetLeftCommand(0.08f);
            firstControl.SetRightCommand(0.06f);
            constexpr float dt = 0.002f;
            scenario.firstPredictAccepted = core.predict(dt, firstControl);

            EncoderObs firstEncoder{};
            firstEncoder.SetTotalLeftCounts(0);
            firstEncoder.SetTotalRightCounts(0);
            float firstLeftWheelSpeedRadps = 0.0f;
            float firstRightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                EstimatorMotionUpdateTest::WorkingState(core)(3),
                EstimatorMotionUpdateTest::WorkingState(core)(5),
                firstLeftWheelSpeedRadps,
                firstRightWheelSpeedRadps);
            firstEncoder.SetLeftWheelSpeedRadps(firstLeftWheelSpeedRadps);
            firstEncoder.SetRightWheelSpeedRadps(firstRightWheelSpeedRadps);
            scenario.firstEncoderAccepted =
                core.updateEncoderPair(firstEncoder, dt, true);

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
