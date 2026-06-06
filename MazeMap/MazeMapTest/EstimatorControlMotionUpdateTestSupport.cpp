#include "pch.h"
#include "EstimatorControlMotionUpdateTestSupport.h"

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        static CommandVector ComputeForwardVelocityCorrection(
            PlantModel& model,
            VehicleState& runtimeState,
            const StateVector& state,
            const float targetForwardMps) noexcept
        {
            PublishStateToRuntime(runtimeState, state);
            const float accelRequestMps2 = 4.0f * (targetForwardMps - state(3));
            return model.ComputeFeedforward(accelRequestMps2, 0.0f);
        }

        ActiveCommandStorageScenario RunActiveCommandStorageScenario()
        {
            ActiveCommandStorageScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            runtime.runtimeState.SetCurrentCommand(App::Internal::CommandVector(0.42f, -0.31f));
            scenario.resetPoseAccepted = core.ResetPose(0.0f, 0.0f, 0.0f);
            scenario.commandAfterReset = runtime.runtimeState.GetCurrentCommand();

            scenario.activeCommand = App::Internal::CommandVector(0.53f, 0.47f);
            scenario.firstPredictAccepted = core.predict(0.002f, scenario.activeCommand);
            scenario.commandAfterFirstPredict = runtime.runtimeState.GetCurrentCommand();

            scenario.nextCommand = App::Internal::CommandVector(-0.18f, 0.24f);
            scenario.secondPredictAccepted = core.predict(0.002f, scenario.nextCommand);
            scenario.commandAfterSecondPredict = runtime.runtimeState.GetCurrentCommand();
            return scenario;
        }

        IterativeMotionScenario RunOpposedControlScenario()
        {
            IterativeMotionScenario scenario;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            const CommandVector control = CommandVector(0.18f, 0.18f);
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            constexpr int kSteps = 200;
            constexpr float dt = kDefaultEstimatorDtSeconds;

            for (int step = 0; step < kSteps; ++step)
            {
                (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, dt);
                if (!core.predict(dt, control))
                {
                    scenario.firstIncompleteOperation = L"predict";
                    break;
                }

                const bool yawAccepted = core.updateYawRate(0.0f);
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

                const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
                const bool accelAccepted =
                    core.updatePlanarAccel(noPlanarAccelObservation);
                if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
                {
                    scenario.firstIncompleteOperation = L"accel update not attempted";
                    break;
                }
                if (!accelAccepted)
                {
                    scenario.firstIncompleteOperation = L"accel update rejected";
                    break;
                }
                if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
                {
                    scenario.firstIncompleteOperation = L"accel update not recorded accepted";
                    break;
                }

                scenario.completedSteps = step + 1;
            }

            scenario.state = EstimatorMotionUpdateTest::WorkingState(core);
            return scenario;
        }

        IterativeMotionScenario RunUnopposedControlScenario()
        {
            IterativeMotionScenario scenario;
            Estimator core = MakeDefaultEstimator();
            const CommandVector control = CommandVector(0.5f, 0.5f);
            constexpr int kSteps = 200;
            constexpr float dt = kDefaultEstimatorDtSeconds;

            for (int step = 0; step < kSteps; ++step)
            {
                if (!core.predict(dt, control))
                {
                    scenario.firstIncompleteOperation = L"predict";
                    break;
                }

                const bool yawAccepted = core.updateYawRate(0.0f);
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

                scenario.completedSteps = step + 1;
            }

            scenario.state = EstimatorMotionUpdateTest::WorkingState(core);
            return scenario;
        }

        StationarySplitCommandPrediction PredictStationarySplitCommandStateAfterPivotPredictSequence()
        {
            StationarySplitCommandPrediction prediction;
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.0f;
            initialState(2) = 0.0f;
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
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
            if (!EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance))
            {
                prediction.firstIncompleteOperation = L"reset";
                return prediction;
            }

            const CommandVector control = CommandVector(0.60f, -0.60f);
            constexpr float dtSeconds = 0.001f;
            const float pivotScrubCommandAngularRadps =
                kEstimatorTestPivotScrubMinCommandAngularRadps;
            const float pivotWheelSpeedRadps =
                Vehicle::WheelSpeedFromLinearVelocity(
                    0.5f * Vehicle::GetPhysicalTrackWidthM() * pivotScrubCommandAngularRadps);
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            SyntheticEncoderRemainderState syntheticEncoderState{};
            for (int step = 0; step < kStationarySplitCommandPredictSteps; ++step)
            {
                SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
                encoder.SetTotalLeftCounts(ConsumeWholeEncoderCounts(
                    (Vehicle::WheelLinearVelocityFromWheelSpeed(pivotWheelSpeedRadps) * dtSeconds) /
                        distancePerCountM,
                    syntheticEncoderState.leftRemainderCounts));
                encoder.SetTotalRightCounts(ConsumeWholeEncoderCounts(
                    (-Vehicle::WheelLinearVelocityFromWheelSpeed(pivotWheelSpeedRadps) * dtSeconds) /
                        distancePerCountM,
                    syntheticEncoderState.rightRemainderCounts));
                (void)PublishEncoderObservationToRuntime(runtime.runtimeState, encoder, dtSeconds);

                if (!core.predict(dtSeconds, control))
                {
                    prediction.firstIncompleteOperation = L"predict";
                    break;
                }

                const bool yawAccepted = core.updateYawRate(pivotScrubCommandAngularRadps);
                if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
                {
                    prediction.firstIncompleteOperation = L"yaw update not attempted";
                    break;
                }
                if (!yawAccepted)
                {
                    prediction.firstIncompleteOperation = L"yaw update rejected";
                    break;
                }
                if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
                {
                    prediction.firstIncompleteOperation = L"yaw update not recorded accepted";
                    break;
                }

                const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
                const bool accelAccepted = core.updatePlanarAccel(noPlanarAccelObservation);
                if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
                {
                    prediction.firstIncompleteOperation = L"accel update not attempted";
                    break;
                }
                if (!accelAccepted)
                {
                    prediction.firstIncompleteOperation = L"accel update rejected";
                    break;
                }
                if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
                {
                    prediction.firstIncompleteOperation = L"accel update not recorded accepted";
                    break;
                }

                prediction.completedSteps = step + 1;
            }

            prediction.state = EstimatorMotionUpdateTest::WorkingState(core);
            return prediction;
        }

        SplitDrivePredictScenario RunSplitDrivePredictScenario()
        {
            SplitDrivePredictScenario scenario;
            Estimator core = MakeDefaultEstimator();
            constexpr float initialForwardVelocityMps = 1.0f;
            scenario.initialState = BuildRestingState(initialForwardVelocityMps);
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
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, scenario.initialState, initialCovariance);

            const CommandVector control = CommandVector(0.30f, 0.60f);
            constexpr float dt = 0.002f;
            constexpr int kSteps = 75;
            for (int step = 0; step < kSteps; ++step)
            {
                if (!core.predict(dt, control))
                {
                    scenario.firstIncompleteOperation = L"predict";
                    break;
                }
                scenario.completedSteps = step + 1;
            }

            scenario.state = EstimatorMotionUpdateTest::WorkingState(core);
            return scenario;
        }

        static bool ApplyPredictionMatchingCycleNoAssert(
            Estimator& core,
            VehicleState& runtimeState,
            const CommandVector& control,
            const float dtSeconds,
            SyntheticEncoderRemainderState& remainderState,
            const wchar_t*& firstIncompleteOperation)
        {
            const StateVector stateBeforePredict = EstimatorMotionUpdateTest::WorkingState(core);
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            if (std::isfinite(distancePerCountM) && (distancePerCountM > 0.0f))
            {
                const float leftDistanceDeltaM =
                    Vehicle::LeftWheelLinearVelocityFromBody(stateBeforePredict(3), stateBeforePredict(5)) *
                    dtSeconds;
                const float rightDistanceDeltaM =
                    Vehicle::RightWheelLinearVelocityFromBody(stateBeforePredict(3), stateBeforePredict(5)) *
                    dtSeconds;
                encoder.SetTotalLeftCounts(ConsumeWholeEncoderCounts(
                    leftDistanceDeltaM / distancePerCountM,
                    remainderState.leftRemainderCounts));
                encoder.SetTotalRightCounts(ConsumeWholeEncoderCounts(
                    rightDistanceDeltaM / distancePerCountM,
                    remainderState.rightRemainderCounts));
            }
            (void)PublishEncoderObservationToRuntime(runtimeState, encoder, dtSeconds);
            if (!core.predict(dtSeconds, control))
            {
                firstIncompleteOperation = L"predict";
                return false;
            }

            const StateVector stateAfterPredict = EstimatorMotionUpdateTest::WorkingState(core);
            (void)stateBeforePredict;
            (void)runtimeState;
            (void)remainderState;

            const bool yawAccepted = core.updateYawRate(stateAfterPredict(5));
            if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
            {
                firstIncompleteOperation = L"yaw update not attempted";
                return false;
            }
            if (!yawAccepted)
            {
                firstIncompleteOperation = L"yaw update rejected";
                return false;
            }
            if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
            {
                firstIncompleteOperation = L"yaw update not recorded accepted";
                return false;
            }
            return true;
        }

        static bool ApplyStationaryCycleNoAssert(
            Estimator& core,
            VehicleState& runtimeState,
            const CommandVector& control,
            const float dtSeconds,
            const wchar_t*& firstIncompleteOperation)
        {
            SensorSnapshot::EncoderObs encoder = SensorSnapshot{}.EncoderObservation();
            (void)PublishEncoderObservationToRuntime(runtimeState, encoder, dtSeconds);
            if (!core.predict(dtSeconds, control))
            {
                firstIncompleteOperation = L"predict";
                return false;
            }

            const bool yawAccepted = core.updateYawRate(0.0f);
            if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
            {
                firstIncompleteOperation = L"yaw update not attempted";
                return false;
            }
            if (!yawAccepted)
            {
                firstIncompleteOperation = L"yaw update rejected";
                return false;
            }
            if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
            {
                firstIncompleteOperation = L"yaw update not recorded accepted";
                return false;
            }

            const ImuAccelObs accel(true, 0.0f, 0.0f);
            const bool accelAccepted = core.updatePlanarAccel(accel);
            if (!EstimatorMotionUpdateTest::LastUpdateAttempted(core))
            {
                firstIncompleteOperation = L"accel update not attempted";
                return false;
            }
            if (!accelAccepted)
            {
                firstIncompleteOperation = L"accel update rejected";
                return false;
            }
            if (!EstimatorMotionUpdateTest::LastUpdateAccepted(core))
            {
                firstIncompleteOperation = L"accel update not recorded accepted";
                return false;
            }
            return true;
        }

        ControlDirectionScenario RunControlDirectionScenario(
            const bool stationaryFirst)
        {
            ControlDirectionScenario scenario;
            EstimatorTestRuntime runtime;
            PlantModel& model = runtime.plantModel;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            StateVector initialState = StateVector::Zero();
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.001f * 0.001f;
            initialCovariance(1, 1) = 0.001f * 0.001f;
            initialCovariance(2, 2) = 0.01f * 0.01f;
            initialCovariance(3, 3) = 0.005f * 0.005f;
            initialCovariance(4, 4) = 0.005f * 0.005f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            if (!EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance))
            {
                scenario.firstIncompleteOperation = L"reset";
                return scenario;
            }

            constexpr float dt = kDefaultEstimatorDtSeconds;
            if (stationaryFirst)
            {
                const CommandVector zeroControl{};
                constexpr int kStationarySteps = 3000;
                for (int step = 0; step < kStationarySteps; ++step)
                {
                    if (!ApplyStationaryCycleNoAssert(
                            core,
                            runtime.runtimeState,
                            zeroControl,
                            dt,
                            scenario.firstIncompleteOperation))
                    {
                        scenario.state = EstimatorMotionUpdateTest::WorkingState(core);
                        return scenario;
                    }
                    scenario.completedStationarySteps = step + 1;
                }
            }

            constexpr float forwardVelocityTargetMps = 3.0f;
            SyntheticEncoderRemainderState syntheticEncoderState{};
            constexpr int kTrackingSteps = 3000;
            for (int step = 0; step < kTrackingSteps; ++step)
            {
                const CommandVector control =
                    ComputeForwardVelocityCorrection(
                        model,
                        runtime.runtimeState,
                        EstimatorMotionUpdateTest::WorkingState(core),
                        forwardVelocityTargetMps);
                if (!ApplyPredictionMatchingCycleNoAssert(
                        core,
                        runtime.runtimeState,
                        control,
                        dt,
                        syntheticEncoderState,
                        scenario.firstIncompleteOperation))
                {
                    break;
                }
                scenario.completedTrackingSteps = step + 1;
            }

            scenario.state = EstimatorMotionUpdateTest::WorkingState(core);
            return scenario;
        }
    }
}
