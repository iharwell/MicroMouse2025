#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorFilterTestSupport.h"
#include "TimeStepPropagationTestSupport.h"
#include <Eigen/LU>
#include <cmath>
#include <limits>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kZeroVelocityToleranceMps = 0.008f;
        using CommandVector = App::Internal::CommandVector;

        WallGeometryModel::GeometryStateFrame BuildGeometryFrame(
            const WallGeometryModel& geometry,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state)
        {
            return geometry.buildStateFrame(
                Eigen::Vector2f(state(0), state(1)),
                state(2));
        }

        void PublishStateToRuntime(
            VehicleState& runtimeState,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept
        {
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                state(3),
                state(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            runtimeState.SetPosition(Eigen::Vector2f(state(0), state(1)));
            runtimeState.SetHeading(state(2));
            runtimeState.SetForwardVelocity(state(3));
            runtimeState.SetRightwardVelocity(state(4));
            runtimeState.SetYawRate(state(5));
            runtimeState.SetForwardAccelerationResidual(state(6));
            runtimeState.SetRightwardAccelerationResidual(state(7));
            runtimeState.SetYawAccelResidual(state(8));
            runtimeState.SetWheelSpeedLeft(leftWheelSpeedRadps);
            runtimeState.SetWheelSpeedRight(rightWheelSpeedRadps);
        }

        App::Internal::CommandVector ComputeForwardVelocityCorrection(
            PlantModel& model,
            VehicleState& runtimeState,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const float targetForwardMps) noexcept
        {
            PublishStateToRuntime(runtimeState, state);
            const float accelRequestMps2 = 4.0f * (targetForwardMps - state(3));
            return model.ComputeFeedforward(accelRequestMps2, 0.0f);
        }
    }

    TEST_CLASS(EstimatorMotionUpdateTest)
    {
    private:
        static Eigen::Matrix<float, VehicleState::kDimension, 1> PredictStationarySplitCommandStateAfterPivotPredictSequence()
        {
            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.0f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
                return Eigen::Matrix<float, VehicleState::kDimension, 1>::Constant(std::numeric_limits<float>::quiet_NaN());
            }

            const CommandVector control = CommandVector(0.60f, -0.60f);

            constexpr float dtSeconds = 0.001f;
            constexpr int kPredictSteps = 500;
            const float pivotScrubCommandAngularRadps =
                kEstimatorTestPivotScrubMinCommandAngularRadps;
            const float pivotWheelSpeedRadps =
                Vehicle::WheelSpeedFromLinearVelocity(
                    0.5f * Vehicle::GetPhysicalTrackWidthM() * pivotScrubCommandAngularRadps);
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            SyntheticEncoderRemainderState syntheticEncoderState{};
            for (int step = 0; step < kPredictSteps; ++step)
            {
                Assert::IsTrue(core.predict(dtSeconds, control));

                EncoderObs encoder{};
                encoder.SetLeftWheelSpeedRadps(pivotWheelSpeedRadps);
                encoder.SetRightWheelSpeedRadps(-pivotWheelSpeedRadps);
                encoder.SetTotalLeftCounts(ConsumeWholeEncoderCounts(
                        (Vehicle::WheelLinearVelocityFromWheelSpeed(pivotWheelSpeedRadps) * dtSeconds) / distancePerCountM,
                        syntheticEncoderState.leftRemainderCounts));
                encoder.SetTotalRightCounts(ConsumeWholeEncoderCounts(
                        (-Vehicle::WheelLinearVelocityFromWheelSpeed(pivotWheelSpeedRadps) * dtSeconds) / distancePerCountM,
                        syntheticEncoderState.rightRemainderCounts));
                const bool encoderAccepted = core.updateEncoderPair(encoder, dtSeconds, true);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(encoderAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());

                const bool yawAccepted = core.updateYawRate(pivotScrubCommandAngularRadps);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(yawAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());

                const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
                const bool accelAccepted = core.updatePlanarAccel(noPlanarAccelObservation);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(accelAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
            }

            return core.workingState();
        }

    public:
        TEST_METHOD(EstimatorZeroVelocityEncoderUpdateKeepsYawRateVarianceBoundedAtRest)
        {
            EstimatorTestRuntime runtime;
            PlantModel& plant = runtime.plantModel;
            Estimator core = MakeDefaultEstimator();
            constexpr float dt = 0.001f;

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
            Assert::IsTrue(std::fabs(runtime.runtimeState.GetForwardAcceleration()) < 1.0e-4f);
            Assert::IsTrue(std::fabs(runtime.runtimeState.GetRightAcceleration()) < 1.0e-4f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetForwardVelocity(), 1.0e-6f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetRightwardVelocity(), 1.0e-6f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetYawRate(), 1.0e-6f);

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> beforeCovariance = core.workingCovariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(5, 5);

            Assert::IsTrue(core.predict(dt, control));

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> predictedCovariance = core.workingCovariance();
            const Eigen::Matrix<float, VehicleState::kDimension, 1> predictedState = core.workingState();
            Assert::AreEqual(initialState(6), predictedState(6), 1.0e-6f);
            Assert::AreEqual(initialState(7), predictedState(7), 1.0e-6f);
            Assert::AreEqual(initialState(8), predictedState(8), 1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(6, 6) + (0.050f * 0.050f * dt),
                predictedCovariance(6, 6),
                1.0e-8f);
            Assert::AreEqual(
                beforeCovariance(7, 7) + (0.050f * 0.050f * dt),
                predictedCovariance(7, 7),
                1.0e-8f);
            Assert::AreEqual(
                beforeCovariance(8, 8) + (0.050f * 0.050f * dt),
                predictedCovariance(8, 8),
                1.0e-8f);

            EncoderObs encoder{};
            Eigen::Vector2f encoderMeasurement;
            encoderMeasurement << 0.0f, 0.0f;
            Eigen::Vector2f priorBodyState;
            priorBodyState << predictedState(3), predictedState(5);
            Eigen::Matrix2f priorBodyCovariance;
            priorBodyCovariance <<
                predictedCovariance(3, 3), predictedCovariance(3, 5),
                predictedCovariance(5, 3), predictedCovariance(5, 5);
            Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
            encoderNoise(0, 0) =
                kEstimatorTestStationaryEncoderVelocitySigmaMps *
                kEstimatorTestStationaryEncoderVelocitySigmaMps;
            encoderNoise(1, 1) =
                (kEstimatorTestStationaryEncoderVelocitySigmaMps / Vehicle::GetPhysicalTrackWidthM()) *
                (kEstimatorTestStationaryEncoderVelocitySigmaMps / Vehicle::GetPhysicalTrackWidthM());
            const Eigen::Matrix2f innovationCovariance = priorBodyCovariance + encoderNoise;
            const float innovationDeterminant =
                (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                (innovationCovariance(0, 1) * innovationCovariance(1, 0));
            Eigen::Matrix2f innovationCovarianceInverse;
            innovationCovarianceInverse <<
                innovationCovariance(1, 1) / innovationDeterminant,
                -innovationCovariance(0, 1) / innovationDeterminant,
                -innovationCovariance(1, 0) / innovationDeterminant,
                innovationCovariance(0, 0) / innovationDeterminant;
            const Eigen::Matrix2f bodyGain = priorBodyCovariance * innovationCovarianceInverse;
            const Eigen::Vector2f expectedBodyState =
                priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
            const Eigen::Matrix2f expectedBodyCovariance =
                priorBodyCovariance - (bodyGain * innovationCovariance * bodyGain.transpose());
            const float expectedEncoderNis =
                ((encoderMeasurement - priorBodyState).transpose() *
                 innovationCovarianceInverse *
                 (encoderMeasurement - priorBodyState))(0);
            const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedEncoderNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& afterState = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> afterCovariance = core.workingCovariance();
            Assert::AreEqual(expectedBodyState(0), afterState(3), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(1), afterState(5), 1.0e-6f);
            Assert::AreEqual(predictedState(0), afterState(0), 1.0e-6f);
            Assert::AreEqual(predictedState(1), afterState(1), 1.0e-6f);
            Assert::AreEqual(predictedState(2), afterState(2), 1.0e-6f);
            Assert::AreEqual(predictedState(4), afterState(4), 1.0e-6f);
            Assert::AreEqual(predictedState(6), afterState(6), 1.0e-6f);
            Assert::AreEqual(predictedState(7), afterState(7), 1.0e-6f);
            Assert::AreEqual(predictedState(8), afterState(8), 1.0e-6f);
            Assert::AreEqual(expectedBodyCovariance(0, 0), afterCovariance(3, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(0, 1), afterCovariance(3, 5), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 0), afterCovariance(5, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 1), afterCovariance(5, 5), 1.0e-7f);
            Assert::AreEqual(0.0f, afterCovariance(3, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(3, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(3, 8), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 8), 1.0e-8f);
            Assert::AreEqual(predictedCovariance(6, 6), afterCovariance(6, 6), 1.0e-8f);
            Assert::AreEqual(predictedCovariance(7, 7), afterCovariance(7, 7), 1.0e-8f);
            Assert::AreEqual(predictedCovariance(8, 8), afterCovariance(8, 8), 1.0e-8f);
            Assert::IsTrue(
                afterCovariance(5, 5) <
                initialYawRateVarianceRadps2);
            Assert::IsTrue(
                afterCovariance(2, 2) <
                initialYawRateVarianceRadps2);
        }
        TEST_METHOD(EstimatorMovingEncoderUpdateKeepsYawRateVarianceLowAfterPredictAndUpdate)
        {
            EstimatorTestRuntime runtime;
            PlantModel& plant = runtime.plantModel;
            Estimator core = MakeDefaultEstimator();
            constexpr float dt = 0.001f;
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            constexpr int32_t measuredCounts = 1;
            const float measuredLinearSpeedMps =
                (static_cast<float>(measuredCounts) * distancePerCountM) / dt;
            const float measuredWheelSpeedRadps = Vehicle::WheelSpeedFromLinearVelocity(measuredLinearSpeedMps);

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = measuredLinearSpeedMps;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
            encoder.SetTotalLeftCounts(measuredCounts);
            encoder.SetTotalRightCounts(measuredCounts);
            encoder.SetLeftWheelSpeedRadps(measuredWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(measuredWheelSpeedRadps);
            Assert::AreEqual(measuredLinearSpeedMps, plant.measuredLinearSpeedMps(encoder), 1.0e-6f);
            Assert::AreEqual(0.0f, plant.measuredYawRateRadps(encoder), 1.0e-6f);
            Assert::IsTrue(
                plant.measuredYawRateVarianceRadps2(
                    encoder,
                    kEstimatorTestStationaryEncoderVelocitySigmaMps,
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps) > 0.0f);
            Assert::IsTrue(
                plant.measuredWheelVarianceRadps2(
                    encoder,
                    kEstimatorTestStationaryEncoderVelocitySigmaMps,
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps) > 0.0f);

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> beforeCovariance = core.workingCovariance();
            const float initialYawRateVarianceRadps2 =
                beforeCovariance(5, 5);

            Assert::IsTrue(core.predict(dt, control));

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> predictedCovariance = core.workingCovariance();
            const Eigen::Matrix<float, VehicleState::kDimension, 1> predictedState = core.workingState();
            Assert::AreEqual(initialState(6), predictedState(6), 1.0e-6f);
            Assert::AreEqual(initialState(7), predictedState(7), 1.0e-6f);
            Assert::AreEqual(initialState(8), predictedState(8), 1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(6, 6) + (0.050f * 0.050f * dt),
                predictedCovariance(6, 6),
                1.0e-8f);
            Assert::AreEqual(
                beforeCovariance(7, 7) + (0.050f * 0.050f * dt),
                predictedCovariance(7, 7),
                1.0e-8f);
            Assert::AreEqual(
                beforeCovariance(8, 8) + (0.050f * 0.050f * dt),
                predictedCovariance(8, 8),
                1.0e-8f);

            Eigen::Vector2f encoderMeasurement;
            encoderMeasurement << measuredLinearSpeedMps, 0.0f;
            Eigen::Vector2f priorBodyState;
            priorBodyState << predictedState(3), predictedState(5);
            Eigen::Matrix2f priorBodyCovariance;
            priorBodyCovariance <<
                predictedCovariance(3, 3), predictedCovariance(3, 5),
                predictedCovariance(5, 3), predictedCovariance(5, 5);
            Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
            encoderNoise(0, 0) =
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps *
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
            encoderNoise(1, 1) =
                kEstimatorTestGeneralEncoderYawRateSigmaRadps *
                kEstimatorTestGeneralEncoderYawRateSigmaRadps;
            const Eigen::Matrix2f innovationCovariance = priorBodyCovariance + encoderNoise;
            const float innovationDeterminant =
                (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                (innovationCovariance(0, 1) * innovationCovariance(1, 0));
            Eigen::Matrix2f innovationCovarianceInverse;
            innovationCovarianceInverse <<
                innovationCovariance(1, 1) / innovationDeterminant,
                -innovationCovariance(0, 1) / innovationDeterminant,
                -innovationCovariance(1, 0) / innovationDeterminant,
                innovationCovariance(0, 0) / innovationDeterminant;
            const Eigen::Matrix2f bodyGain = priorBodyCovariance * innovationCovarianceInverse;
            const Eigen::Vector2f expectedBodyState =
                priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
            const Eigen::Matrix2f expectedBodyCovariance =
                priorBodyCovariance - (bodyGain * innovationCovariance * bodyGain.transpose());
            const float expectedEncoderNis =
                ((encoderMeasurement - priorBodyState).transpose() *
                 innovationCovarianceInverse *
                 (encoderMeasurement - priorBodyState))(0);
            const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedEncoderNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& afterState = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> afterCovariance = core.workingCovariance();
            Assert::AreEqual(expectedBodyState(0), afterState(3), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(1), afterState(5), 1.0e-6f);
            Assert::AreEqual(predictedState(0), afterState(0), 1.0e-6f);
            Assert::AreEqual(predictedState(1), afterState(1), 1.0e-6f);
            Assert::AreEqual(predictedState(2), afterState(2), 1.0e-6f);
            Assert::AreEqual(predictedState(4), afterState(4), 1.0e-6f);
            Assert::AreEqual(predictedState(6), afterState(6), 1.0e-6f);
            Assert::AreEqual(predictedState(7), afterState(7), 1.0e-6f);
            Assert::AreEqual(predictedState(8), afterState(8), 1.0e-6f);
            Assert::AreEqual(expectedBodyCovariance(0, 0), afterCovariance(3, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(0, 1), afterCovariance(3, 5), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 0), afterCovariance(5, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 1), afterCovariance(5, 5), 1.0e-7f);
            Assert::IsTrue(
                afterCovariance(5, 5) <=
                (predictedCovariance(5, 5) + 1.0e-9f));
            Assert::IsTrue(
                afterCovariance(5, 5) <
                initialYawRateVarianceRadps2);
            Assert::AreEqual(0.0f, afterCovariance(3, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(3, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(3, 8), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 8), 1.0e-8f);
            Assert::AreEqual(predictedCovariance(6, 6), afterCovariance(6, 6), 1.0e-8f);
            Assert::AreEqual(predictedCovariance(7, 7), afterCovariance(7, 7), 1.0e-8f);
            Assert::AreEqual(predictedCovariance(8, 8), afterCovariance(8, 8), 1.0e-8f);
        }
        TEST_METHOD(EstimatorLatestEncoderObservationPullsBodyRatesTowardLatestMeasurement)
        {
            const float distancePerCountM = Vehicle::DriveEncoderDistanceFromCounts(1);
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            App::Internal::CommandVector control{};
            constexpr float dt = 0.001f;

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs first{};
            first.SetTotalLeftCounts(2);
            first.SetTotalRightCounts(-1);
            first.SetLeftWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(first.TotalLeftCounts()) * distancePerCountM) / dt));
            first.SetRightWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(first.TotalRightCounts()) * distancePerCountM) / dt));
            const bool firstAccepted = core.updateEncoderPair(first, dt, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(firstAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            Assert::IsTrue(core.predict(dt, control));
            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeSecondEncoder = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeSecondEncoder = core.workingCovariance();
            EncoderObs second{};
            second.SetTotalLeftCounts(5);
            second.SetTotalRightCounts(-3);
            second.SetLeftWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(second.TotalLeftCounts()) * distancePerCountM) / dt));
            second.SetRightWheelSpeedRadps(Vehicle::WheelSpeedFromLinearVelocity(
                    (static_cast<float>(second.TotalRightCounts()) * distancePerCountM) / dt));
            Eigen::Vector2f encoderMeasurement;
            encoderMeasurement <<
                runtime.plantModel.measuredLinearSpeedMps(second),
                runtime.plantModel.measuredYawRateRadps(second);
            Eigen::Vector2f priorBodyState;
            priorBodyState << stateBeforeSecondEncoder(3), stateBeforeSecondEncoder(5);
            Eigen::Matrix2f priorBodyCovariance;
            priorBodyCovariance <<
                covarianceBeforeSecondEncoder(3, 3), covarianceBeforeSecondEncoder(3, 5),
                covarianceBeforeSecondEncoder(5, 3), covarianceBeforeSecondEncoder(5, 5);
            Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
            encoderNoise(0, 0) =
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps *
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
            encoderNoise(1, 1) =
                kEstimatorTestGeneralEncoderYawRateSigmaRadps *
                kEstimatorTestGeneralEncoderYawRateSigmaRadps;
            const Eigen::Matrix2f innovationCovariance = priorBodyCovariance + encoderNoise;
            const float innovationDeterminant =
                (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                (innovationCovariance(0, 1) * innovationCovariance(1, 0));
            Eigen::Matrix2f innovationCovarianceInverse;
            innovationCovarianceInverse <<
                innovationCovariance(1, 1) / innovationDeterminant,
                -innovationCovariance(0, 1) / innovationDeterminant,
                -innovationCovariance(1, 0) / innovationDeterminant,
                innovationCovariance(0, 0) / innovationDeterminant;
            const Eigen::Matrix2f bodyGain = priorBodyCovariance * innovationCovarianceInverse;
            const Eigen::Vector2f expectedBodyState =
                priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
            const Eigen::Matrix2f expectedBodyCovariance =
                priorBodyCovariance - (bodyGain * innovationCovariance * bodyGain.transpose());
            const float expectedEncoderNis =
                ((encoderMeasurement - priorBodyState).transpose() *
                 innovationCovarianceInverse *
                 (encoderMeasurement - priorBodyState))(0);
            const bool secondAccepted = core.updateEncoderPair(second, dt, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(secondAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedEncoderNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            Assert::AreEqual(expectedBodyState(0), state(3), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(1), state(5), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(0), state(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(1), state(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(2), state(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(4), state(4), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(6), state(6), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(7), state(7), 1.0e-6f);
            Assert::AreEqual(stateBeforeSecondEncoder(8), state(8), 1.0e-6f);
            Assert::AreEqual(expectedBodyCovariance(0, 0), covariance(3, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(0, 1), covariance(3, 5), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 0), covariance(5, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 1), covariance(5, 5), 1.0e-7f);
            Assert::AreEqual(0.0f, covariance(3, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(3, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(3, 8), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(5, 8), 1.0e-8f);
            Assert::AreEqual(covarianceBeforeSecondEncoder(6, 6), covariance(6, 6), 1.0e-8f);
            Assert::AreEqual(covarianceBeforeSecondEncoder(7, 7), covariance(7, 7), 1.0e-8f);
            Assert::AreEqual(covarianceBeforeSecondEncoder(8, 8), covariance(8, 8), 1.0e-8f);
        }

        TEST_METHOD(EstimatorEncoderPairDirectUpdateKeepsUnmeasuredStateInvariantWhileUpdatingBodyRates)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.07f;
            initialState(1) = 0.19f;
            initialState(2) = (0.11f);
            initialState(3) = 0.42f;
            initialState(4) = 0.03f;
            initialState(5) = 0.18f;
            initialState(6) = 0.12f;
            initialState(7) = -0.08f;
            initialState(8) = 0.02f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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

            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeEncoder = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeEncoder = core.workingCovariance();

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(8);
            encoder.SetTotalRightCounts(-6);
            encoder.SetLeftWheelSpeedRadps(10.6f);
            encoder.SetRightWheelSpeedRadps(2.4f);
            Eigen::Vector2f encoderMeasurement;
            encoderMeasurement <<
                runtime.plantModel.measuredLinearSpeedMps(encoder),
                runtime.plantModel.measuredYawRateRadps(encoder);
            Eigen::Vector2f priorBodyState;
            priorBodyState << stateBeforeEncoder(3), stateBeforeEncoder(5);
            Eigen::Matrix2f priorBodyCovariance;
            priorBodyCovariance <<
                covarianceBeforeEncoder(3, 3), covarianceBeforeEncoder(3, 5),
                covarianceBeforeEncoder(5, 3), covarianceBeforeEncoder(5, 5);
            Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
            encoderNoise(0, 0) =
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps *
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
            encoderNoise(1, 1) =
                kEstimatorTestGeneralEncoderYawRateSigmaRadps *
                kEstimatorTestGeneralEncoderYawRateSigmaRadps;
            const Eigen::Matrix2f innovationCovariance = priorBodyCovariance + encoderNoise;
            const float innovationDeterminant =
                (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                (innovationCovariance(0, 1) * innovationCovariance(1, 0));
            Eigen::Matrix2f innovationCovarianceInverse;
            innovationCovarianceInverse <<
                innovationCovariance(1, 1) / innovationDeterminant,
                -innovationCovariance(0, 1) / innovationDeterminant,
                -innovationCovariance(1, 0) / innovationDeterminant,
                innovationCovariance(0, 0) / innovationDeterminant;
            const Eigen::Matrix2f bodyGain = priorBodyCovariance * innovationCovarianceInverse;
            const Eigen::Vector2f expectedBodyState =
                priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
            const Eigen::Matrix2f expectedBodyCovariance =
                priorBodyCovariance - (bodyGain * innovationCovariance * bodyGain.transpose());
            const float expectedEncoderNis =
                ((encoderMeasurement - priorBodyState).transpose() *
                 innovationCovarianceInverse *
                 (encoderMeasurement - priorBodyState))(0);

            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedEncoderNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& stateAfterEncoder = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterEncoder = core.workingCovariance();
            Assert::AreEqual(stateBeforeEncoder(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(0), stateAfterEncoder(3), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(1), stateAfterEncoder(5), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(6), stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(7), stateAfterEncoder(7), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(8), stateAfterEncoder(8), 1.0e-6f);
            Assert::AreEqual(expectedBodyCovariance(0, 0), covarianceAfterEncoder(3, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(0, 1), covarianceAfterEncoder(3, 5), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 0), covarianceAfterEncoder(5, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 1), covarianceAfterEncoder(5, 5), 1.0e-7f);
            Assert::AreEqual(0.0f, covarianceAfterEncoder(3, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterEncoder(3, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterEncoder(3, 8), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterEncoder(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterEncoder(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterEncoder(5, 8), 1.0e-8f);
            Assert::AreEqual(covarianceBeforeEncoder(6, 6), covarianceAfterEncoder(6, 6), 1.0e-8f);
            Assert::AreEqual(covarianceBeforeEncoder(7, 7), covarianceAfterEncoder(7, 7), 1.0e-8f);
            Assert::AreEqual(covarianceBeforeEncoder(8, 8), covarianceAfterEncoder(8, 8), 1.0e-8f);
        }

        TEST_METHOD(EstimatorRejectedEncoderPairUpdateStillKeepsBodyStateInvariant)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = -0.04f;
            initialState(1) = 0.16f;
            initialState(2) = (-0.07f);
            initialState(3) = 0.31f;
            initialState(4) = -0.02f;
            initialState(5) = 0.09f;
            initialState(6) = 0.16f;
            initialState(7) = 0.14f;
            initialState(8) = -0.01f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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

            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeEncoder = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeEncoder = core.workingCovariance();

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(40);
            encoder.SetTotalRightCounts(-36);
            encoder.SetLeftWheelSpeedRadps(120.0f);
            encoder.SetRightWheelSpeedRadps(-115.0f);

            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsFalse(encoderAccepted);
            Assert::IsFalse(core.LastUpdateAccepted());
            Assert::IsTrue(core.LastUpdateNis() > kEstimatorTestEncoderPairNisThreshold);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& stateAfterEncoder = core.workingState();
            Assert::AreEqual(stateBeforeEncoder(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(3), stateAfterEncoder(3), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(5), stateAfterEncoder(5), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(6), stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(7), stateAfterEncoder(7), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(8), stateAfterEncoder(8), 1.0e-6f);
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterEncoder = core.workingCovariance();
            for (int row = 0; row < 9; ++row)
            {
                for (int col = 0; col < 9; ++col)
                {
                    Assert::AreEqual(
                        covarianceBeforeEncoder(row, col),
                        covarianceAfterEncoder(row, col),
                        1.0e-7f);
                }
            }
        }

        TEST_METHOD(EstimatorDoesNotLetControlVectorCreateUnboundedForwardMotionWithEncoderOpposition)
        {
            Estimator core = MakeDefaultEstimator();
            const CommandVector control = CommandVector(0.18f, 0.18f);
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(encoderAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());

                const bool yawAccepted = core.updateYawRate(0.0f);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(yawAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());

                const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
                const bool accelAccepted = core.updatePlanarAccel(noPlanarAccelObservation);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(accelAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(3)) < 1.0e-4f);
            Assert::AreEqual(0.0f, state(6), 1.0e-6f);
            Assert::AreEqual(0.0f, state(7), 1.0e-6f);
            Assert::AreEqual(0.0f, state(8), 1.0e-6f);
        }

        TEST_METHOD(EstimatorMustLetControlVectorCreateForwardMotionWithNoEncoder)
        {
            Estimator core = MakeDefaultEstimator();
            const CommandVector control = CommandVector(0.5f, 0.5f);
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const bool yawAccepted = core.updateYawRate(0.0f);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(yawAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(1)) > 1.0e-2f);
            Assert::IsTrue(std::fabs(state(3)) > 1.0e-2f);
        }

        TEST_METHOD(EstimatorPredictRepeatedSplitCommandKeepsZeroForwardVelocity)
        {
            const Eigen::Matrix<float, VehicleState::kDimension, 1> predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();
            Assert::AreEqual(
                0.0f,
                predictedState(3),
                kZeroVelocityToleranceMps);
        }

        TEST_METHOD(EstimatorPredictRepeatedSplitCommandProducesPositiveYawRate)
        {
            const Eigen::Matrix<float, VehicleState::kDimension, 1> predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::IsTrue(predictedState(5) > 0.0f);
        }

        TEST_METHOD(EstimatorPredictRepeatedSplitCommandTurnsClockwise)
        {
            const Eigen::Matrix<float, VehicleState::kDimension, 1> predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::IsTrue(predictedState(2) > 0.0f);
        }

        TEST_METHOD(EstimatorPredictRepeatedSplitCommandKeepsZeroForwardPoseDrift)
        {
            const Eigen::Matrix<float, VehicleState::kDimension, 1> predictedState =
                PredictStationarySplitCommandStateAfterPivotPredictSequence();

            Assert::AreEqual(
                0.0f,
                predictedState(1),
                0.01f);
        }

        TEST_METHOD(EstimatorAcceptsLaunchEncoderDeltasWhenOpenLoopPredictionDisagrees)
        {
            struct LaunchEncoderSample
            {
                float dtSeconds;
                float leftWheelSpeedRadps;
                float rightWheelSpeedRadps;
                float gyroRawRadps;
            };

            // D:\open_floor_main.csv, April 10, 2026, repeat 11 launch pulse.
            // The raw launch command makes the open-loop plant prediction disagree
            // with measured wheel speeds; encoder deltas must still remain authoritative.
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
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.225f;
            initialState(1) = 0.225f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
                const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeEncoder = core.workingState();
                const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeEncoder = core.workingCovariance();

                EncoderObs encoder{};
                const float leftCounts =
                    (Vehicle::WheelLinearVelocityFromWheelSpeed(sample.leftWheelSpeedRadps) * sample.dtSeconds) /
                    distancePerCountM;
                const float rightCounts =
                    (Vehicle::WheelLinearVelocityFromWheelSpeed(sample.rightWheelSpeedRadps) * sample.dtSeconds) /
                    distancePerCountM;
                encoder.SetTotalLeftCounts(static_cast<int32_t>((leftCounts >= 0.0f) ? (leftCounts + 0.5f) : (leftCounts - 0.5f)));
                encoder.SetTotalRightCounts(static_cast<int32_t>((rightCounts >= 0.0f) ? (rightCounts + 0.5f) : (rightCounts - 0.5f)));
                encoder.SetLeftWheelSpeedRadps(sample.leftWheelSpeedRadps);
                encoder.SetRightWheelSpeedRadps(sample.rightWheelSpeedRadps);
                Eigen::Vector2f encoderMeasurement;
                encoderMeasurement <<
                    runtime.plantModel.measuredLinearSpeedMps(encoder),
                    runtime.plantModel.measuredYawRateRadps(encoder);
                Eigen::Vector2f priorBodyState;
                priorBodyState << stateBeforeEncoder(3), stateBeforeEncoder(5);
                Eigen::Matrix2f priorBodyCovariance;
                priorBodyCovariance <<
                    covarianceBeforeEncoder(3, 3), covarianceBeforeEncoder(3, 5),
                    covarianceBeforeEncoder(5, 3), covarianceBeforeEncoder(5, 5);
                Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
                encoderNoise(0, 0) =
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps *
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
                encoderNoise(1, 1) =
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps *
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps;
                const Eigen::Matrix2f innovationCovariance = priorBodyCovariance + encoderNoise;
                const float innovationDeterminant =
                    (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                    (innovationCovariance(0, 1) * innovationCovariance(1, 0));
                Eigen::Matrix2f innovationCovarianceInverse;
                innovationCovarianceInverse <<
                    innovationCovariance(1, 1) / innovationDeterminant,
                    -innovationCovariance(0, 1) / innovationDeterminant,
                    -innovationCovariance(1, 0) / innovationDeterminant,
                    innovationCovariance(0, 0) / innovationDeterminant;
                const Eigen::Matrix2f bodyGain = priorBodyCovariance * innovationCovarianceInverse;
                const Eigen::Vector2f expectedBodyState =
                    priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
                const Eigen::Matrix2f expectedBodyCovariance =
                    priorBodyCovariance - (bodyGain * innovationCovariance * bodyGain.transpose());
                const float expectedEncoderNis =
                    ((encoderMeasurement - priorBodyState).transpose() *
                     innovationCovarianceInverse *
                     (encoderMeasurement - priorBodyState))(0);
                const bool encoderAccepted = core.updateEncoderPair(encoder, sample.dtSeconds, true);

                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(
                    encoderAccepted,
                    (std::wstring(L"Launch encoder update rejected at sample ") +
                        std::to_wstring(index)).c_str());
                Assert::IsTrue(core.LastUpdateAccepted());
                Assert::AreEqual(expectedEncoderNis, core.LastUpdateNis(), 1.0e-4f);

                const Eigen::Matrix<float, VehicleState::kDimension, 1>& encoderConstrainedState = core.workingState();
                const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterEncoder = core.workingCovariance();
                Assert::AreEqual(expectedBodyState(0), encoderConstrainedState(3), 1.0e-5f);
                Assert::AreEqual(expectedBodyState(1), encoderConstrainedState(5), 1.0e-5f);
                Assert::AreEqual(stateBeforeEncoder(0), encoderConstrainedState(0), 1.0e-6f);
                Assert::AreEqual(stateBeforeEncoder(1), encoderConstrainedState(1), 1.0e-6f);
                Assert::AreEqual(stateBeforeEncoder(2), encoderConstrainedState(2), 1.0e-6f);
                Assert::AreEqual(stateBeforeEncoder(4), encoderConstrainedState(4), 1.0e-6f);
                Assert::AreEqual(stateBeforeEncoder(6), encoderConstrainedState(6), 1.0e-6f);
                Assert::AreEqual(stateBeforeEncoder(7), encoderConstrainedState(7), 1.0e-6f);
                Assert::AreEqual(stateBeforeEncoder(8), encoderConstrainedState(8), 1.0e-6f);
                Assert::AreEqual(expectedBodyCovariance(0, 0), covarianceAfterEncoder(3, 3), 1.0e-7f);
                Assert::AreEqual(expectedBodyCovariance(0, 1), covarianceAfterEncoder(3, 5), 1.0e-7f);
                Assert::AreEqual(expectedBodyCovariance(1, 0), covarianceAfterEncoder(5, 3), 1.0e-7f);
                Assert::AreEqual(expectedBodyCovariance(1, 1), covarianceAfterEncoder(5, 5), 1.0e-7f);
                Assert::AreEqual(0.0f, covarianceAfterEncoder(3, 6), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterEncoder(3, 7), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterEncoder(3, 8), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterEncoder(5, 6), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterEncoder(5, 7), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterEncoder(5, 8), 1.0e-8f);
                Assert::AreEqual(covarianceBeforeEncoder(6, 6), covarianceAfterEncoder(6, 6), 1.0e-8f);
                Assert::AreEqual(covarianceBeforeEncoder(7, 7), covarianceAfterEncoder(7, 7), 1.0e-8f);
                Assert::AreEqual(covarianceBeforeEncoder(8, 8), covarianceAfterEncoder(8, 8), 1.0e-8f);

                const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeYaw = core.workingState();
                const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeYaw = core.workingCovariance();
                const float correctedYawRateRadps =
                    sample.gyroRawRadps - runtime.runtimeState.GetGyroBiasZ();
                const float gyroScaleRadpsPerLsb =
                    runtime.vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() * 0.001f * DEG_TO_RAD_F;
                const float gyroScaleToleranceSigmaRadps =
                    std::fabs(correctedYawRateRadps) *
                    kEstimatorTestImuGyroSensitivityToleranceFraction /
                    std::sqrt(3.0f);
                const float yawInnovation =
                    correctedYawRateRadps - stateBeforeYaw(5);
                const float yawInnovationVariance =
                    covarianceBeforeYaw(5, 5) +
                    kEstimatorTestImuYawRateVarianceRadps2 +
                    ((gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f) +
                    (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps) +
                    runtime.runtimeState.GetGyroBiasZVar();
                const float yawGain = covarianceBeforeYaw(5, 5) / yawInnovationVariance;
                const float expectedYawRate = stateBeforeYaw(5) + (yawGain * yawInnovation);
                const float expectedYawVariance =
                    covarianceBeforeYaw(5, 5) -
                    (yawGain * yawInnovationVariance * yawGain);
                const float expectedYawNis = (yawInnovation * yawInnovation) / yawInnovationVariance;
                const bool yawAccepted = core.updateYawRate(sample.gyroRawRadps);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(yawAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
                Assert::AreEqual(expectedYawNis, core.LastUpdateNis(), 1.0e-4f);
                const Eigen::Matrix<float, VehicleState::kDimension, 1>& stateAfterYaw = core.workingState();
                const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterYaw = core.workingCovariance();
                Assert::AreEqual(stateBeforeYaw(3), stateAfterYaw(3), 1.0e-6f);
                Assert::AreEqual(expectedYawRate, stateAfterYaw(5), 1.0e-5f);
                Assert::AreEqual(stateBeforeYaw(6), stateAfterYaw(6), 1.0e-6f);
                Assert::AreEqual(stateBeforeYaw(7), stateAfterYaw(7), 1.0e-6f);
                Assert::AreEqual(stateBeforeYaw(8), stateAfterYaw(8), 1.0e-6f);
                Assert::AreEqual(expectedYawVariance, covarianceAfterYaw(5, 5), 1.0e-7f);
                Assert::AreEqual(0.0f, covarianceAfterYaw(5, 6), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterYaw(5, 7), 1.0e-8f);
                Assert::AreEqual(0.0f, covarianceAfterYaw(5, 8), 1.0e-8f);
            }
        }

        TEST_METHOD(EstimatorSplitDrivePredictBuildsTurnRateWhileKeepingForwardProgress)
        {
            Estimator core = MakeDefaultEstimator();
            constexpr float initialForwardVelocityMps = 1.0f;
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = initialForwardVelocityMps;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(state(1) > initialState(1));
            Assert::IsTrue(state(3) > 0.0f);
            Assert::IsTrue(state(2) < 0.0f);
            Assert::IsTrue(state(0) < 0.0f);
        }

        TEST_METHOD(EstimatorYawRateUpdateDoesNotPullResidualsThroughYawCrossCovariance)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.02f;
            initialState(1) = 0.14f;
            initialState(2) = (0.10f);
            initialState(3) = 1.40f;
            initialState(4) = 0.05f;
            initialState(5) = 0.0f;
            initialState(6) = 0.12f;
            initialState(7) = -0.08f;
            initialState(8) = 0.03f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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

            const Eigen::Matrix<float, VehicleState::kDimension, 1> before = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> beforeCovariance = core.workingCovariance();
            constexpr float observedYawRateRadps = 0.45f;
            const float correctedYawRateRadps =
                observedYawRateRadps - runtime.runtimeState.GetGyroBiasZ();
            const float gyroScaleRadpsPerLsb =
                runtime.vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() * 0.001f * DEG_TO_RAD_F;
            const float gyroScaleToleranceSigmaRadps =
                std::fabs(correctedYawRateRadps) *
                kEstimatorTestImuGyroSensitivityToleranceFraction /
                std::sqrt(3.0f);
            const float yawInnovation =
                correctedYawRateRadps - before(5);
            const float yawInnovationVariance =
                beforeCovariance(5, 5) +
                kEstimatorTestImuYawRateVarianceRadps2 +
                ((gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f) +
                (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps) +
                runtime.runtimeState.GetGyroBiasZVar();
            const float yawGain = beforeCovariance(5, 5) / yawInnovationVariance;
            const float expectedYawRate = before(5) + (yawGain * yawInnovation);
            const float expectedYawVariance =
                beforeCovariance(5, 5) -
                (yawGain * yawInnovationVariance * yawGain);
            const float expectedYawNis =
                (yawInnovation * yawInnovation) / yawInnovationVariance;

            const bool yawAccepted = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(yawAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedYawNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& after = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> afterCovariance = core.workingCovariance();

            Assert::AreEqual(expectedYawRate, after(5), 1.0e-6f);
            Assert::AreEqual(expectedYawVariance, afterCovariance(5, 5), 1.0e-7f);
            Assert::AreEqual(before(6), after(6), 1.0e-6f);
            Assert::AreEqual(before(7), after(7), 1.0e-6f);
            Assert::AreEqual(before(8), after(8), 1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(6, 6),
                afterCovariance(6, 6),
                1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(7, 7),
                afterCovariance(7, 7),
                1.0e-6f);
            Assert::AreEqual(
                beforeCovariance(8, 8),
                afterCovariance(8, 8),
                1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(5, 6), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(6, 5), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(5, 7), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(7, 5), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(5, 8), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(8, 5), 1.0e-6f);
        }

        TEST_METHOD(EstimatorFrontWallUpdateMovesForwardForCloserSymmetricObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
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

            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
            const Eigen::Matrix<float, VehicleState::kDimension, 1> before = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> beforeCovariance = core.workingCovariance();

            const WallObs leftObservation =
                WallObs(leftPrediction.rangeM - 0.012f, 1.0f, ObsClass::WallLike);
            const WallObs rightObservation =
                WallObs(rightPrediction.rangeM - 0.012f, 1.0f, ObsClass::WallLike);

            const bool frontPairAccepted = core.updateFrontPair(leftObservation, rightObservation, maze);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(frontPairAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& after = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> afterCovariance = core.workingCovariance();
            Assert::IsTrue(after(1) > (before(1) + 0.002f));
            Assert::IsTrue(std::fabs(after(0) - before(0)) < 0.004f);
            Assert::IsTrue(afterCovariance(1, 1) <
                beforeCovariance(1, 1));
        }

        TEST_METHOD(EstimatorLeftWallUpdateMovesLeftForCloserObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Left, WallState::Wall);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
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

            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
            const Eigen::Matrix<float, VehicleState::kDimension, 1> before = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> beforeCovariance = core.workingCovariance();

            const WallObs observation =
                WallObs(baseline.rangeM - 0.012f, 1.0f, ObsClass::WallLike);

            const bool sideAccepted = core.updateSideSensor(RelativeDirection::Left90, observation, maze);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(sideAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& after = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> afterCovariance = core.workingCovariance();
            Assert::IsTrue(after(0) < (before(0) - 0.002f));
            Assert::IsTrue(std::fabs(after(1) - before(1)) < 0.004f);
            Assert::IsTrue(afterCovariance(0, 0) <
                beforeCovariance(0, 0));
        }
        TEST_METHOD(EstimatorControlDirectionsCorrect)
        {
			EstimatorTestRuntime runtime;
			PlantModel& model = runtime.plantModel;
            constexpr float forwardVelocityTargetMps = 1.0f;
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.0f;
            initialState(2) = (0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.001f * 0.001f;
            initialCovariance(1, 1) = 0.001f * 0.001f;
            initialCovariance(2, 2) = 0.01f * 0.01f;
            initialCovariance(3, 3) = 0.005f * 0.005f;
            initialCovariance(4, 4) = 0.005f * 0.005f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;

            Estimator core = MakeDefaultEstimator();
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
        TEST_METHOD(EstimatorControlDirectionsCorrectAfterStationary)
        {
            EstimatorTestRuntime runtime;
            PlantModel& model = runtime.plantModel;
            constexpr float forwardVelocityTargetMps = 1.0f;
            constexpr float dt = 0.001f;
            auto core = RunEstimatorCycles(2000, App::Internal::CommandVector{});
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

        TEST_METHOD(EstimatorYawRateUpdateSubtractsExternalGyroBiasAndIncludesBiasVarianceInMeasurementCovariance)
        {
            EstimatorTestRuntime runtime;
            runtime.runtimeState.SetGyroBiasZ(0.08f);
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.02f;
            initialState(1) = -0.03f;
            initialState(2) = (0.05f);
            initialState(3) = 0.15f;
            initialState(4) = -0.04f;
            initialState(5) = 0.11f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.010f * 0.010f;
            initialCovariance(1, 1) = 0.010f * 0.010f;
            initialCovariance(2, 2) = 0.010f * 0.010f;
            initialCovariance(3, 3) = 0.020f * 0.020f;
            initialCovariance(4, 4) = 0.015f * 0.015f;
            initialCovariance(5, 5) = 0.015f * 0.015f;
            initialCovariance(6, 6) = 0.020f * 0.020f;
            initialCovariance(7, 7) = 0.020f * 0.020f;
            initialCovariance(8, 8) = 0.030f * 0.030f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const Eigen::Matrix<float, VehicleState::kDimension, 1> before = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> beforeCovariance = core.workingCovariance();
            const float beforeGyroBiasRadps = runtime.runtimeState.GetGyroBiasZ();
            constexpr float observedYawRateRadps = 0.27f;
            const float correctedYawRateRadps = observedYawRateRadps - beforeGyroBiasRadps;
            const float gyroScaleRadpsPerLsb =
                runtime.vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() * 0.001f * DEG_TO_RAD_F;
            const float gyroScaleToleranceSigmaRadps =
                std::fabs(correctedYawRateRadps) *
                kEstimatorTestImuGyroSensitivityToleranceFraction /
                std::sqrt(3.0f);
            const float yawInnovation =
                correctedYawRateRadps - before(5);
            const float yawInnovationVariance =
                beforeCovariance(5, 5) +
                kEstimatorTestImuYawRateVarianceRadps2 +
                ((gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f) +
                (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps) +
                runtime.runtimeState.GetGyroBiasZVar();
            const float yawGain = beforeCovariance(5, 5) / yawInnovationVariance;
            const float expectedYawRate = before(5) + (yawGain * yawInnovation);
            const float expectedYawVariance =
                beforeCovariance(5, 5) -
                (yawGain * yawInnovationVariance * yawGain);
            const float expectedYawNis =
                (yawInnovation * yawInnovation) / yawInnovationVariance;

            const bool yawAccepted = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(yawAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedYawNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& after = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> afterCovariance = core.workingCovariance();
            const float afterGyroBiasRadps = runtime.runtimeState.GetGyroBiasZ();

            Assert::AreEqual(expectedYawRate, after(5), 1.0e-6f);
            Assert::AreEqual(expectedYawVariance, afterCovariance(5, 5), 1.0e-7f);
            Assert::AreEqual(beforeGyroBiasRadps, afterGyroBiasRadps, 1.0e-6f);
            Assert::AreEqual(before(0), after(0), 1.0e-6f);
            Assert::AreEqual(before(1), after(1), 1.0e-6f);
            Assert::AreEqual(before(2), after(2), 1.0e-6f);
            Assert::AreEqual(before(3), after(3), 1.0e-6f);
            Assert::AreEqual(before(4), after(4), 1.0e-6f);
            Assert::AreEqual(before(6), after(6), 1.0e-6f);
            Assert::AreEqual(before(7), after(7), 1.0e-6f);
            Assert::AreEqual(before(8), after(8), 1.0e-6f);
            Assert::AreEqual(0.0f, afterCovariance(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, afterCovariance(5, 8), 1.0e-8f);
        }

        TEST_METHOD(EstimatorMovingPredictDoesNotInjectGyroBiasProcessVariance)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.02f;
            initialState(1) = 0.11f;
            initialState(2) = (0.01f);
            initialState(3) = 0.35f;
            initialState(4) = 0.0f;
            initialState(5) = 0.02f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Assert::IsTrue(core.reset(initialState, Estimator::BuildDefaultInitialCovariance()));

            const float beforeVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();

            const CommandVector control = CommandVector(0.16f, 0.16f);

            Assert::IsTrue(core.predict(0.002f, control));

            const float afterVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::AreEqual(initialState(6), state(6), 1.0e-6f);
            Assert::AreEqual(initialState(7), state(7), 1.0e-6f);
            Assert::AreEqual(initialState(8), state(8), 1.0e-6f);
            Assert::AreEqual(beforeVarianceRadps2, afterVarianceRadps2, 1.0e-9f);
        }

        TEST_METHOD(EstimatorPredictAndEncoderUpdateKeepRuntimeFeedforwardStateFinite)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.05f;
            initialState(1) = 0.07f;
            initialState(2) = (0.04f);
            initialState(3) = 0.30f;
            initialState(4) = 0.02f;
            initialState(5) = -0.05f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
            encoder.SetTotalLeftCounts(1);
            encoder.SetTotalRightCounts(0);
            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeEncoder = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeEncoder = core.workingCovariance();
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                stateBeforeEncoder(3),
                stateBeforeEncoder(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            Eigen::Vector2f encoderMeasurement;
            encoderMeasurement <<
                runtime.plantModel.measuredLinearSpeedMps(encoder),
                runtime.plantModel.measuredYawRateRadps(encoder);
            Eigen::Vector2f priorBodyState;
            priorBodyState << stateBeforeEncoder(3), stateBeforeEncoder(5);
            Eigen::Matrix2f priorBodyCovariance;
            priorBodyCovariance <<
                covarianceBeforeEncoder(3, 3), covarianceBeforeEncoder(3, 5),
                covarianceBeforeEncoder(5, 3), covarianceBeforeEncoder(5, 5);
            Eigen::Matrix2f encoderNoise = Eigen::Matrix2f::Zero();
            encoderNoise(0, 0) =
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps *
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
            encoderNoise(1, 1) =
                kEstimatorTestGeneralEncoderYawRateSigmaRadps *
                kEstimatorTestGeneralEncoderYawRateSigmaRadps;
            const Eigen::Matrix2f innovationCovariance = priorBodyCovariance + encoderNoise;
            const float innovationDeterminant =
                (innovationCovariance(0, 0) * innovationCovariance(1, 1)) -
                (innovationCovariance(0, 1) * innovationCovariance(1, 0));
            Eigen::Matrix2f innovationCovarianceInverse;
            innovationCovarianceInverse <<
                innovationCovariance(1, 1) / innovationDeterminant,
                -innovationCovariance(0, 1) / innovationDeterminant,
                -innovationCovariance(1, 0) / innovationDeterminant,
                innovationCovariance(0, 0) / innovationDeterminant;
            const Eigen::Matrix2f bodyGain = priorBodyCovariance * innovationCovarianceInverse;
            const Eigen::Vector2f expectedBodyState =
                priorBodyState + (bodyGain * (encoderMeasurement - priorBodyState));
            const Eigen::Matrix2f expectedBodyCovariance =
                priorBodyCovariance - (bodyGain * innovationCovariance * bodyGain.transpose());
            const float expectedEncoderNis =
                ((encoderMeasurement - priorBodyState).transpose() *
                 innovationCovarianceInverse *
                 (encoderMeasurement - priorBodyState))(0);
            runtime.runtimeState.SetWheelSpeedLeft(encoder.LeftWheelSpeedRadps());
            runtime.runtimeState.SetWheelSpeedRight(encoder.RightWheelSpeedRadps());
            Assert::AreEqual(
                encoder.LeftWheelSpeedRadps(),
                runtime.runtimeState.GetWheelSpeedLeft(),
                1.0e-6f);
            Assert::AreEqual(
                encoder.RightWheelSpeedRadps(),
                runtime.runtimeState.GetWheelSpeedRight(),
                1.0e-6f);
            const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());
            Assert::AreEqual(expectedEncoderNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            Assert::AreEqual(stateBeforeEncoder(0), state(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), state(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), state(2), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(0), state(3), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), state(4), 1.0e-6f);
            Assert::AreEqual(expectedBodyState(1), state(5), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(6), state(6), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(7), state(7), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(8), state(8), 1.0e-6f);
            Assert::AreEqual(expectedBodyCovariance(0, 0), covariance(3, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(0, 1), covariance(3, 5), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 0), covariance(5, 3), 1.0e-7f);
            Assert::AreEqual(expectedBodyCovariance(1, 1), covariance(5, 5), 1.0e-7f);
            Assert::AreEqual(0.0f, covariance(3, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(3, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(3, 8), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covariance(5, 8), 1.0e-8f);
            Assert::AreEqual(
                encoder.LeftWheelSpeedRadps(),
                runtime.runtimeState.GetWheelSpeedLeft(),
                1.0e-6f);
            Assert::AreEqual(
                encoder.RightWheelSpeedRadps(),
                runtime.runtimeState.GetWheelSpeedRight(),
                1.0e-6f);
        }

        TEST_METHOD(EstimatorPredictRefreshesAppliedTorqueFromCurrentControl)
        {
            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.01f;
            initialState(1) = 0.02f;
            initialState(2) = (0.03f);
            initialState(3) = 0.45f;
            initialState(4) = -0.01f;
            initialState(5) = 0.06f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
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
            firstEncoder.SetTotalLeftCounts(0);
            firstEncoder.SetTotalRightCounts(0);
            float firstLeftWheelSpeedRadps = 0.0f;
            float firstRightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                core.workingState()(3),
                core.workingState()(5),
                firstLeftWheelSpeedRadps,
                firstRightWheelSpeedRadps);
            firstEncoder.SetLeftWheelSpeedRadps(firstLeftWheelSpeedRadps);
            firstEncoder.SetRightWheelSpeedRadps(firstRightWheelSpeedRadps);
            Assert::IsTrue(core.updateEncoderPair(firstEncoder, dt, true));

            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeSecondPredict = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceBeforeSecondPredict = core.workingCovariance();
            App::Internal::CommandVector secondControl{};
            secondControl.SetLeftCommand(0.31f);
            secondControl.SetRightCommand(0.27f);

            Estimator firstControlCore = MakeDefaultEstimator();
            Estimator secondControlCore = MakeDefaultEstimator();
            Assert::IsTrue(firstControlCore.reset(stateBeforeSecondPredict, covarianceBeforeSecondPredict));
            Assert::IsTrue(secondControlCore.reset(stateBeforeSecondPredict, covarianceBeforeSecondPredict));
            Assert::IsTrue(firstControlCore.predict(dt, firstControl));
            Assert::IsTrue(secondControlCore.predict(dt, secondControl));

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& firstControlState = firstControlCore.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& secondControlState = secondControlCore.workingState();

            const float controlResponseDelta =
                (firstControlState - secondControlState).cwiseAbs().maxCoeff();
            Assert::IsTrue(controlResponseDelta > 1.0e-6f);
        }

        TEST_METHOD(EstimatorPredictStoresActiveCommandForCalculatedState)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            runtime.runtimeState.SetCurrentCommand(App::Internal::CommandVector(0.42f, -0.31f));
            Assert::IsTrue(core.ResetPose(0.0f, 0.0f, 0.0f));
            Assert::AreEqual(0.0f, runtime.runtimeState.GetCurrentCommand().LeftCommand(), 0.0f);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetCurrentCommand().RightCommand(), 0.0f);

            const App::Internal::CommandVector activeCommand(0.53f, 0.47f);
            Assert::IsTrue(core.predict(0.002f, activeCommand));
            Assert::AreEqual(
                activeCommand.LeftCommand(),
                runtime.runtimeState.GetCurrentCommand().LeftCommand(),
                0.0f);
            Assert::AreEqual(
                activeCommand.RightCommand(),
                runtime.runtimeState.GetCurrentCommand().RightCommand(),
                0.0f);

            const App::Internal::CommandVector nextCommand(-0.18f, 0.24f);
            Assert::IsTrue(core.predict(0.002f, nextCommand));
            Assert::AreEqual(
                nextCommand.LeftCommand(),
                runtime.runtimeState.GetCurrentCommand().LeftCommand(),
                0.0f);
            Assert::AreEqual(
                nextCommand.RightCommand(),
                runtime.runtimeState.GetCurrentCommand().RightCommand(),
                0.0f);
        }
    };
}

