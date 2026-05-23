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
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.0f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.05f * 0.05f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            if (!core.reset(initialState, initialCovariance))
            {
                return VehicleState::StateVector::Constant(std::numeric_limits<float>::quiet_NaN());
            }

            const CommandVector control = CommandVector(0.60f, -0.60f);

            constexpr float dtSeconds = 0.001f;
            constexpr int kPredictSteps = 500;
            const float pivotScrubCommandAngularRadps =
                kUkfTestPivotScrubMinCommandAngularRadps;
            const float pivotWheelOmegaRadps =
                Vehicle::WheelOmegaFromLinearVelocity(
                    0.5f * Vehicle::GetPhysicalModel().trackWidthM * pivotScrubCommandAngularRadps);
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            SyntheticEncoderRemainderState syntheticEncoderState{};
            for (int step = 0; step < kPredictSteps; ++step)
            {
                Assert::IsTrue(core.predict(dtSeconds, control));

                EncoderObs encoder{};
                encoder.omegaLeftRadps = pivotWheelOmegaRadps;
                encoder.omegaRightRadps = -pivotWheelOmegaRadps;
                encoder.totalLeftCounts =
                    ConsumeWholeEncoderCounts(
                        (Vehicle::WheelLinearVelocityFromOmega(pivotWheelOmegaRadps) * dtSeconds) / distancePerCountM,
                        syntheticEncoderState.leftRemainderCounts);
                encoder.totalRightCounts =
                    ConsumeWholeEncoderCounts(
                        (-Vehicle::WheelLinearVelocityFromOmega(pivotWheelOmegaRadps) * dtSeconds) / distancePerCountM,
                        syntheticEncoderState.rightRemainderCounts);
                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dtSeconds);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(pivotScrubCommandAngularRadps);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);

                ImuAccelObs noPlanarAccelObservation{};
                const MeasurementUpdateResult accelResult = core.updatePlanarAccel(noPlanarAccelObservation);
                Assert::IsTrue(accelResult.attempted);
                Assert::IsTrue(accelResult.accepted);
            }

            return core.workingState();
        }

        WallGeometryModel::GeometryStateFrame BuildGeometryFrame(
            const WallGeometryModel& geometry,
            const VehicleState::StateVector& state)
        {
            return geometry.buildStateFrame(
                Eigen::Vector2f(state(0), state(1)),
                state(2));
        }

        void PublishStateToRuntime(
            VehicleState& runtimeState,
            const VehicleState::StateVector& state) noexcept
        {
            runtimeState.SetPosition(Eigen::Vector2f(state(0), state(1)));
            runtimeState.SetOrientation(state(2));
            runtimeState.SetVelocity(state(3));
            runtimeState.SetLateralVelocity(state(4));
            runtimeState.SetRotationalVelocity(state(5));
            runtimeState.SetWheelSpeedLeft(state(6));
            runtimeState.SetWheelSpeedRight(state(7));
            runtimeState.SetGyroBiasZ(state(8));
        }

        App::Internal::CommandVector ComputeForwardVelocityCorrection(
            PlantModel& model,
            VehicleState& runtimeState,
            const VehicleState::StateVector& state,
            const float targetForwardMps) noexcept
        {
            PublishStateToRuntime(runtimeState, state);
            const float accelRequestMps2 = 4.0f * (targetForwardMps - state(3));
            return model.ComputeFeedforward(accelRequestMps2, 0.0f);
        }
    }

    TEST_CLASS(SrUkfCoreMotionUpdateTest)
    {
    public:
        TEST_METHOD(SrUkfCoreZeroVelocityEncoderUpdateKeepsYawRateVarianceBoundedAtRest)
        {
            SrUkfCoreTestRuntime runtime;
            PlantModel& plant = runtime.plantModel;
            SrUkfCore core = MakeDefaultSrUkfCore();
            constexpr float dt = 0.001f;

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.001f * 0.001f;
            initialCovariance(1, 1) = 0.001f * 0.001f;
            initialCovariance(2, 2) = 0.01f * 0.01f;
            initialCovariance(3, 3) = 0.005f * 0.005f;
            initialCovariance(4, 4) = 0.005f * 0.005f;
            initialCovariance(5, 5) = 1.0f * 1.0f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};

            PublishStateToRuntime(runtime.runtimeState, initialState);
            Assert::IsTrue(std::fabs(plant.totalForwardContactForceN(control)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(plant.totalRightContactForceN(control)) < 1.0e-4f);
            plant.integrate(control, dt);
            Assert::IsTrue(std::fabs(runtime.runtimeState.GetLongitudinalAcceleration()) < 1.0e-4f);
            Assert::IsTrue(std::fabs(runtime.runtimeState.GetLateralAcceleration()) < 1.0e-4f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetVelocity(), 1.0e-6f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetLateralVelocity(), 1.0e-6f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetRotationalVelocity(), 1.0e-6f);

            const VehicleState::StateMatrix beforeCovariance = core.workingCovariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(5, 5);
            const float initialLeftWheelVarianceRadps2 =
                beforeCovariance(6, 6);
            const float initialRightWheelVarianceRadps2 =
                beforeCovariance(7, 7);

            Assert::IsTrue(initialLeftWheelVarianceRadps2 < (0.01f * initialYawRateVarianceRadps2));
            Assert::IsTrue(initialRightWheelVarianceRadps2 < (0.01f * initialYawRateVarianceRadps2));

            Assert::IsTrue(core.predict(dt, control));

            const VehicleState::StateMatrix predictedCovariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(predictedCovariance(5, 5)));
            Assert::IsTrue(std::isfinite(predictedCovariance(6, 6)));
            Assert::IsTrue(std::isfinite(predictedCovariance(7, 7)));

            EncoderObs encoder{};
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateMatrix afterCovariance = core.workingCovariance();
            Assert::IsTrue(
                afterCovariance(5, 5) <
                initialYawRateVarianceRadps2);
            Assert::IsTrue(
                afterCovariance(2, 2) <
                initialYawRateVarianceRadps2);
        }
        TEST_METHOD(SrUkfCoreMovingEncoderUpdateKeepsYawRateVarianceLowAfterPredictAndUpdate)
        {
            SrUkfCoreTestRuntime runtime;
            PlantModel& plant = runtime.plantModel;
            SrUkfCore core = MakeDefaultSrUkfCore();
            constexpr float dt = 0.001f;
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            constexpr int32_t measuredCounts = 1;
            const float measuredLinearSpeedMps =
                (static_cast<float>(measuredCounts) * distancePerCountM) / dt;
            const float measuredWheelOmegaRadps = Vehicle::WheelOmegaFromLinearVelocity(measuredLinearSpeedMps);

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = measuredLinearSpeedMps;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = measuredWheelOmegaRadps;
            initialState(7) = measuredWheelOmegaRadps;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.001f * 0.001f;
            initialCovariance(1, 1) = 0.001f * 0.001f;
            initialCovariance(2, 2) = 0.01f * 0.01f;
            initialCovariance(3, 3) = 0.005f * 0.005f;
            initialCovariance(4, 4) = 0.005f * 0.005f;
            initialCovariance(5, 5) = 1.0f * 1.0f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};

            PublishStateToRuntime(runtime.runtimeState, initialState);
            Assert::IsTrue(std::fabs(plant.totalForwardContactForceN(control)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(plant.totalRightContactForceN(control)) < 1.0e-4f);

            EncoderObs encoder{};
            encoder.totalLeftCounts = measuredCounts;
            encoder.totalRightCounts = measuredCounts;
            encoder.omegaLeftRadps = measuredWheelOmegaRadps;
            encoder.omegaRightRadps = measuredWheelOmegaRadps;
            Assert::AreEqual(measuredLinearSpeedMps, plant.measuredLinearSpeedMps(encoder), 1.0e-6f);
            Assert::AreEqual(0.0f, plant.measuredYawRateRadps(encoder), 1.0e-6f);
            Assert::IsTrue(
                plant.measuredYawRateVarianceRadps2(
                    encoder,
                    kUkfTestStationaryEncoderVelocitySigmaMps,
                    kUkfTestGeneralEncoderLinearSpeedSigmaMps,
                    kUkfTestGeneralEncoderYawRateSigmaRadps) > 0.0f);
            Assert::IsTrue(
                plant.measuredWheelVarianceRadps2(
                    encoder,
                    kUkfTestStationaryEncoderVelocitySigmaMps,
                    kUkfTestGeneralEncoderLinearSpeedSigmaMps,
                    kUkfTestGeneralEncoderYawRateSigmaRadps) > 0.0f);

            const VehicleState::StateMatrix beforeCovariance = core.workingCovariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(5, 5);
            Assert::IsTrue(
                beforeCovariance(6, 6) <
                (0.01f * initialYawRateVarianceRadps2));
            Assert::IsTrue(
                beforeCovariance(7, 7) <
                (0.01f * initialYawRateVarianceRadps2));

            Assert::IsTrue(core.predict(dt, control));

            const VehicleState::StateMatrix predictedCovariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(predictedCovariance(5, 5)));
            Assert::IsTrue(std::isfinite(predictedCovariance(6, 6)));
            Assert::IsTrue(std::isfinite(predictedCovariance(7, 7)));

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateMatrix afterCovariance = core.workingCovariance();
            Assert::IsTrue(
                afterCovariance(5, 5) <=
                (predictedCovariance(5, 5) + 1.0e-9f));
            Assert::IsTrue(
                afterCovariance(5, 5) <
                initialYawRateVarianceRadps2);
            Assert::IsTrue(std::isfinite(afterCovariance(5, 6)));
            Assert::IsTrue(std::isfinite(afterCovariance(5, 7)));
        }
        TEST_METHOD(SrUkfCoreLatestEncoderObservationPullsWheelRatesTowardLatestMeasurement)
        {
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            const auto omegaFromCounts = [distancePerCountM](int32_t counts, float dtSeconds) noexcept
            {
                return Vehicle::WheelOmegaFromLinearVelocity(
                    (static_cast<float>(counts) * distancePerCountM) / dtSeconds);
            };
            SrUkfCore core = MakeDefaultSrUkfCore();
            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;

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

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::isfinite(state(6)));
            Assert::IsTrue(std::isfinite(state(7)));
            Assert::IsTrue(state(6) > 0.0f);
            Assert::IsTrue(state(7) < 0.0f);
            Assert::IsTrue(
                std::fabs(state(6) - second.omegaLeftRadps) <
                std::fabs(first.omegaLeftRadps - second.omegaLeftRadps));
            Assert::IsTrue(
                std::fabs(state(7) - second.omegaRightRadps) <
                std::fabs(first.omegaRightRadps - second.omegaRightRadps));
        }

        TEST_METHOD(SrUkfCoreEncoderPairDirectUpdateKeepsBodyStateInvariantWhileUpdatingWheelStates)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.07f;
            initialState(1) = 0.19f;
            initialState(2) = (0.11f);
            initialState(3) = 0.42f;
            initialState(4) = 0.03f;
            initialState(5) = 0.18f;
            initialState(6) = 7.4f;
            initialState(7) = 6.8f;
            initialState(8) = 0.02f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
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
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector stateBeforeEncoder = core.workingState();

            EncoderObs encoder{};
            encoder.totalLeftCounts = 8;
            encoder.totalRightCounts = -6;
            encoder.omegaLeftRadps = 10.6f;
            encoder.omegaRightRadps = 2.4f;

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateVector& stateAfterEncoder = core.workingState();
            Assert::AreEqual(stateBeforeEncoder(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(3), stateAfterEncoder(3), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(5), stateAfterEncoder(5), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(8), stateAfterEncoder(8), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, stateAfterEncoder(7), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreRejectedEncoderPairUpdateStillKeepsBodyStateInvariant)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = -0.04f;
            initialState(1) = 0.16f;
            initialState(2) = (-0.07f);
            initialState(3) = 0.31f;
            initialState(4) = -0.02f;
            initialState(5) = 0.09f;
            initialState(6) = 1.6f;
            initialState(7) = 1.4f;
            initialState(8) = -0.01f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.10f * 0.10f;
            initialCovariance(4, 4) = 0.08f * 0.08f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.08f * 0.08f;
            initialCovariance(7, 7) = 0.08f * 0.08f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector stateBeforeEncoder = core.workingState();

            EncoderObs encoder{};
            encoder.totalLeftCounts = 40;
            encoder.totalRightCounts = -36;
            encoder.omegaLeftRadps = 120.0f;
            encoder.omegaRightRadps = -115.0f;

            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);
            Assert::IsTrue(encoderResult.nis > kUkfTestEncoderPairNisThreshold);

            const VehicleState::StateVector& stateAfterEncoder = core.workingState();
            Assert::AreEqual(stateBeforeEncoder(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(3), stateAfterEncoder(3), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(5), stateAfterEncoder(5), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(8), stateAfterEncoder(8), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, stateAfterEncoder(7), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreDoesNotLetControlVectorCreateUnboundedForwardMotionWithEncoderOpposition)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const CommandVector control = CommandVector(0.18f, 0.18f);
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

                ImuAccelObs noPlanarAccelObservation{};
                const MeasurementUpdateResult accelResult = core.updatePlanarAccel(noPlanarAccelObservation);
                Assert::IsTrue(accelResult.attempted);
                Assert::IsTrue(accelResult.accepted);
            }

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(3)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(6)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(7)) < 1.0e-4f);
        }

        TEST_METHOD(SrUkfCoreMustLetControlVectorCreateForwardMotionWithNoEncoder)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const CommandVector control = CommandVector(0.5f, 0.5f);
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(1)) > 1.0e-2f);
            Assert::IsTrue(std::fabs(state(3)) > 1.0e-2f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandKeepsZeroForwardVelocity)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            Assert::AreEqual(
                0.0f,
                predictedState(3),
                kZeroVelocityToleranceMps);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandProducesPositiveLeftWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::IsTrue(predictedState(6) > 0.0f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandProducesNegativeRightWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::IsTrue(predictedState(7) < 0.0f);
        }

        TEST_METHOD(SrUkfCorePredictRepeatedSplitCommandKeepsZeroAverageWheelSpeed)
        {
            const VehicleState::StateVector predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            const float averageWheelLinearSpeedMps =
                Vehicle::WheelLinearVelocityFromOmega(
                    0.5f *
                    (predictedState(6) + predictedState(7)));

            Assert::AreEqual(
                0.0f,
                averageWheelLinearSpeedMps,
                kZeroVelocityToleranceMps);
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

            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            const auto countsFromOmega = [distancePerCountM](float omegaRadps, float dtSeconds) noexcept
            {
                const float counts =
                    (Vehicle::WheelLinearVelocityFromOmega(omegaRadps) * dtSeconds) / distancePerCountM;
                return static_cast<int32_t>((counts >= 0.0f) ? (counts + 0.5f) : (counts - 0.5f));
            };

            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.225f;
            initialState(1) = 0.225f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.05f * 0.05f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const CommandVector control = CommandVector(0.08f, 0.08f);

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

                const VehicleState::StateVector& encoderConstrainedState = core.workingState();
                Assert::IsTrue(std::isfinite(encoderConstrainedState(3)));
                Assert::AreEqual(sample.leftOmegaRadps, encoderConstrainedState(6), 1.0e-5f);
                Assert::AreEqual(sample.rightOmegaRadps, encoderConstrainedState(7), 1.0e-5f);

                const MeasurementUpdateResult yawResult = core.updateYawRate(sample.gyroRawRadps);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }
        }

        TEST_METHOD(SrUkfCoreSplitDrivePredictBuildsTurnRateWhileKeepingForwardProgress)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            constexpr float initialForwardVelocityMps = 1.0f;
            const float initialWheelSpeedRadps = Vehicle::WheelOmegaFromLinearVelocity(initialForwardVelocityMps);
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = initialForwardVelocityMps;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = initialWheelSpeedRadps;
            initialState(7) = initialWheelSpeedRadps;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.05f * 0.05f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const CommandVector control = CommandVector(0.30f, 0.60f);

            constexpr float dt = 0.002f;
            constexpr int kSteps = 75;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
            }

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(state(1) > initialState(1));
            Assert::IsTrue(state(3) > 0.0f);
            Assert::IsTrue(state(2) < 0.0f);
            Assert::IsTrue(state(0) < 0.0f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateDoesNotPullWheelRatesThroughYawCrossCovariance)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.02f;
            initialState(1) = 0.14f;
            initialState(2) = (0.10f);
            initialState(3) = 1.40f;
            initialState(4) = 0.05f;
            initialState(5) = 0.0f;
            initialState(6) = 12.0f;
            initialState(7) = 11.5f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
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
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector before = core.workingState();
            const VehicleState::StateMatrix beforeCovariance = core.workingCovariance();
            constexpr float observedYawRateRadps = 0.45f;
            const float beforeError =
                std::fabs((before(5) + before(8)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.workingState();
            const VehicleState::StateMatrix afterCovariance = core.workingCovariance();
            const float afterError =
                std::fabs((after(5) + after(8)) - observedYawRateRadps);

            Assert::IsTrue(afterError < beforeError);
            Assert::AreEqual(before(6), after(6), 1.0e-6f);
            Assert::AreEqual(before(7), after(7), 1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(6, 6),
                afterCovariance(6, 6),
                1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(7, 7),
                afterCovariance(7, 7),
                1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(5, 6), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(6, 5), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(5, 7), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(7, 5), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreFrontWallUpdateMovesForwardForCloserSymmetricObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.09f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;

            WallGeometryModel geometry;
            const WallGeometryModel::GeometryStateFrame frame = BuildGeometryFrame(geometry, initialState);
            const GeometryPrediction leftPrediction =
                geometry.predictRay(frame, Vehicle::GetFrontLeftSensorMount(), maze);
            const GeometryPrediction rightPrediction =
                geometry.predictRay(frame, Vehicle::GetFrontRightSensorMount(), maze);
            Assert::IsTrue(leftPrediction.hit);
            Assert::IsTrue(rightPrediction.hit);

            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.02f * 0.02f;
            initialCovariance(4, 4) = 0.02f * 0.02f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));
            const VehicleState::StateVector before = core.workingState();
            const VehicleState::StateMatrix beforeCovariance = core.workingCovariance();

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

            const VehicleState::StateVector& after = core.workingState();
            const VehicleState::StateMatrix afterCovariance = core.workingCovariance();
            Assert::IsTrue(after(1) > (before(1) + 0.002f));
            Assert::IsTrue(std::fabs(after(0) - before(0)) < 0.004f);
            Assert::IsTrue(afterCovariance(1, 1) <
                beforeCovariance(1, 1));
        }

        TEST_METHOD(SrUkfCoreLeftWallUpdateMovesLeftForCloserObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Left, WallState::Wall);
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.09f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;

            WallGeometryModel geometry;
            const GeometryPrediction baseline =
                geometry.predictRay(
                    BuildGeometryFrame(geometry, initialState),
                    Vehicle::GetSideLeftSensorMount(),
                    maze);
            Assert::IsTrue(baseline.hit);

            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.02f * 0.02f;
            initialCovariance(4, 4) = 0.02f * 0.02f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));
            const VehicleState::StateVector before = core.workingState();
            const VehicleState::StateMatrix beforeCovariance = core.workingCovariance();

            WallObs observation{};
            observation.valid = true;
            observation.confidence = 1.0f;
            observation.cls = ObsClass::WallLike;
            observation.rho = baseline.rangeM - 0.012f;

            const WallUpdateResult result = core.updateSideSensor(RelativeDirection::Left90, observation, maze);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.workingState();
            const VehicleState::StateMatrix afterCovariance = core.workingCovariance();
            Assert::IsTrue(after(0) < (before(0) - 0.002f));
            Assert::IsTrue(std::fabs(after(1) - before(1)) < 0.004f);
            Assert::IsTrue(afterCovariance(0, 0) <
                beforeCovariance(0, 0));
        }
        TEST_METHOD(SrUkfCoreControlDirectionsCorrect)
        {
			SrUkfCoreTestRuntime runtime;
			PlantModel& model = runtime.plantModel;
            constexpr float forwardVelocityTargetMps = 1.0f;
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.0f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.001f * 0.001f;
            initialCovariance(1, 1) = 0.001f * 0.001f;
            initialCovariance(2, 2) = 0.01f * 0.01f;
            initialCovariance(3, 3) = 0.005f * 0.005f;
            initialCovariance(4, 4) = 0.005f * 0.005f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;

            SrUkfCore core = MakeDefaultSrUkfCore();
            core.reset(initialState, initialCovariance);
            constexpr float dt = 0.001f;
            SyntheticEncoderRemainderState syntheticEncoderState{};

            for (int step = 0; step < 3000; ++step)
            {
                const App::Internal::CommandVector control =
                    ComputeForwardVelocityCorrection(
                        model,
                        runtime.runtimeState,
                        core.workingState(),
                        forwardVelocityTargetMps);

                RunPredictionMatchingCycle(
                    core,
                    control,
                    dt,
                    syntheticEncoderState,
                    forwardVelocityTargetMps,
                    0.0f);
            }

			auto state = core.workingState();
            Assert::IsTrue(state(3) > 0.8f,
                (std::wstring(L"Forward velocity was too low: ") +
                    std::to_wstring(state(3))).c_str());

            Assert::IsTrue(fabs(state(4)) < 0.01f,
                (std::wstring(L"Lateral velocity was too high: ") +
                    std::to_wstring(state(4))).c_str());

            Assert::IsTrue(fabs(state(5)) < 0.5f,
                (std::wstring(L"Angular velocity was too high: ") +
                    std::to_wstring(state(5))).c_str());
        }
        TEST_METHOD(SrUkfCoreControlDirectionsCorrectAfterStationary)
        {
            SrUkfCoreTestRuntime runtime;
            PlantModel& model = runtime.plantModel;
            constexpr float forwardVelocityTargetMps = 1.0f;
            constexpr float dt = 0.001f;
            auto core = RunUKFCycles(2000, App::Internal::CommandVector{});
            SyntheticEncoderRemainderState syntheticEncoderState{};

            for (int step = 0; step < 3000; ++step)
            {
                const App::Internal::CommandVector control =
                    ComputeForwardVelocityCorrection(
                        model,
                        runtime.runtimeState,
                        core.workingState(),
                        forwardVelocityTargetMps);

                RunPredictionMatchingCycle(
                    core,
                    control,
                    dt,
                    syntheticEncoderState,
                    forwardVelocityTargetMps,
                    0.0f);
            }

            auto state = core.workingState();
            Assert::IsTrue(state(3) > 0.8f,
                (std::wstring(L"Forward velocity was too low: ") +
                    std::to_wstring(state(3))).c_str());

            Assert::IsTrue(fabs(state(4)) < 0.01f,
                (std::wstring(L"Lateral velocity was too high: ") +
                    std::to_wstring(state(4))).c_str());

            Assert::IsTrue(fabs(state(5)) < 0.5f,
                (std::wstring(L"Angular velocity was too high: ") +
                    std::to_wstring(state(5))).c_str());
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateUsesGyroBiasStateInMeasurementEquation)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.02f;
            initialState(1) = -0.03f;
            initialState(2) = (0.05f);
            initialState(3) = 0.15f;
            initialState(4) = -0.04f;
            initialState(5) = 0.11f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.08f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.010f * 0.010f;
            initialCovariance(1, 1) = 0.010f * 0.010f;
            initialCovariance(2, 2) = 0.010f * 0.010f;
            initialCovariance(3, 3) = 0.020f * 0.020f;
            initialCovariance(4, 4) = 0.015f * 0.015f;
            initialCovariance(5, 5) = 0.015f * 0.015f;
            initialCovariance(6, 6) = 0.020f * 0.020f;
            initialCovariance(7, 7) = 0.020f * 0.020f;
            initialCovariance(8, 8) = 0.030f * 0.030f;
            initialCovariance(5, 8) = 0.0f;
            initialCovariance(8, 5) = 0.0f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const VehicleState::StateVector before = core.workingState();
            constexpr float observedYawRateRadps = 0.27f;
            const float beforeError =
                std::fabs((before(5) + before(8)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.workingState();
            const float afterError =
                std::fabs((after(5) + after(8)) - observedYawRateRadps);

            Assert::IsTrue(afterError < beforeError);
            Assert::IsTrue(std::fabs(after(8) - before(8)) > 1.0e-5f);
            Assert::IsTrue(std::isfinite(after(8)));
        }

        TEST_METHOD(SrUkfCoreMovingPredictDoesNotInjectGyroBiasProcessVariance)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.02f;
            initialState(1) = 0.11f;
            initialState(2) = (0.01f);
            initialState(3) = 0.35f;
            initialState(4) = 0.0f;
            initialState(5) = 0.02f;
            initialState(6) = 4.6f;
            initialState(7) = 4.5f;
            initialState(8) = 0.01f;
            Assert::IsTrue(core.reset(initialState, SrUkfCore::BuildDefaultInitialCovariance()));

            const float beforeVarianceRadps2 = core.workingCovariance()(8, 8);

            const CommandVector control = CommandVector(0.16f, 0.16f);

            Assert::IsTrue(core.predict(0.002f, control));

            const float afterVarianceRadps2 = core.workingCovariance()(8, 8);
            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(3)) > 0.05f);
            Assert::IsTrue(std::fabs(state(6)) > 0.05f);
            Assert::IsTrue(std::fabs(state(7)) > 0.05f);
            Assert::AreEqual(beforeVarianceRadps2, afterVarianceRadps2, 1.0e-9f);
        }

        TEST_METHOD(SrUkfCorePredictAndEncoderUpdateKeepRuntimeFeedforwardStateFinite)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.05f;
            initialState(1) = 0.07f;
            initialState(2) = (0.04f);
            initialState(3) = 0.30f;
            initialState(4) = 0.02f;
            initialState(5) = -0.05f;
            initialState(6) = 3.10f;
            initialState(7) = 3.05f;
            initialState(8) = 0.01f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.002f * 0.002f;
            initialCovariance(1, 1) = 0.002f * 0.002f;
            initialCovariance(2, 2) = 0.002f * 0.002f;
            initialCovariance(3, 3) = 0.003f * 0.003f;
            initialCovariance(4, 4) = 0.003f * 0.003f;
            initialCovariance(5, 5) = 0.003f * 0.003f;
            initialCovariance(6, 6) = 0.003f * 0.003f;
            initialCovariance(7, 7) = 0.003f * 0.003f;
            initialCovariance(8, 8) = 0.002f * 0.002f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const CommandVector control = CommandVector(0.19f, 0.17f);
            constexpr float dt = 0.002f;
            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 0;
            encoder.omegaLeftRadps = core.workingState()(6);
            encoder.omegaRightRadps = core.workingState()(7);
            Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);

            const VehicleState::StateVector& state = core.workingState();
            const VehicleState::StateMatrix covariance = core.workingCovariance();
            for (int row = 0; row < 9; ++row)
            {
                Assert::IsTrue(std::isfinite(state(row)));
                Assert::IsTrue(std::isfinite(covariance(row, row)));
                Assert::IsTrue(covariance(row, row) >= 0.0f);
            }
            Assert::AreEqual(encoder.omegaLeftRadps, state(6), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, state(7), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCorePredictRefreshesAppliedTorqueFromCurrentControl)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.01f;
            initialState(1) = 0.02f;
            initialState(2) = (0.03f);
            initialState(3) = 0.45f;
            initialState(4) = -0.01f;
            initialState(5) = 0.06f;
            initialState(6) = 4.00f;
            initialState(7) = 3.85f;
            initialState(8) = 0.01f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
            initialCovariance(0, 0) = 0.003f * 0.003f;
            initialCovariance(1, 1) = 0.003f * 0.003f;
            initialCovariance(2, 2) = 0.003f * 0.003f;
            initialCovariance(3, 3) = 0.004f * 0.004f;
            initialCovariance(4, 4) = 0.004f * 0.004f;
            initialCovariance(5, 5) = 0.004f * 0.004f;
            initialCovariance(6, 6) = 0.004f * 0.004f;
            initialCovariance(7, 7) = 0.004f * 0.004f;
            initialCovariance(8, 8) = 0.002f * 0.002f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector firstControl{};
            firstControl.SetLeftCommand(0.08f);
            firstControl.SetRightCommand(0.06f);

            constexpr float dt = 0.002f;
            Assert::IsTrue(core.predict(dt, firstControl));

            EncoderObs firstEncoder{};
            firstEncoder.totalLeftCounts = 0;
            firstEncoder.totalRightCounts = 0;
            firstEncoder.omegaLeftRadps = core.workingState()(6);
            firstEncoder.omegaRightRadps = core.workingState()(7);
            Assert::IsTrue(core.updateEncoderPair(firstEncoder, dt).accepted);

            const VehicleState::StateVector stateBeforeSecondPredict = core.workingState();
            const VehicleState::StateMatrix covarianceBeforeSecondPredict = core.workingCovariance();
            App::Internal::CommandVector secondControl{};
            secondControl.SetLeftCommand(0.31f);
            secondControl.SetRightCommand(0.27f);

            SrUkfCore firstControlCore = MakeDefaultSrUkfCore();
            SrUkfCore secondControlCore = MakeDefaultSrUkfCore();
            Assert::IsTrue(firstControlCore.reset(stateBeforeSecondPredict, covarianceBeforeSecondPredict));
            Assert::IsTrue(secondControlCore.reset(stateBeforeSecondPredict, covarianceBeforeSecondPredict));
            Assert::IsTrue(firstControlCore.predict(dt, firstControl));
            Assert::IsTrue(secondControlCore.predict(dt, secondControl));

            const VehicleState::StateVector& firstControlState = firstControlCore.workingState();
            const VehicleState::StateVector& secondControlState = secondControlCore.workingState();

            const float controlResponseDelta =
                (firstControlState - secondControlState).cwiseAbs().maxCoeff();
            Assert::IsTrue(controlResponseDelta > 1.0e-6f);
        }
    };
}

