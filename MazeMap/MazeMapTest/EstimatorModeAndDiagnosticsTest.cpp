#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorFilterTestSupport.h"
#include "..\MazeMap\PlantModel.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kPlanarAccelUpdateTestDtSeconds = 0.001f;

        Eigen::Matrix<float, VehicleState::kDimension, 1> BuildPlanarAccelUpdateTestState() noexcept
        {
            Eigen::Matrix<float, VehicleState::kDimension, 1> state = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            state(0) = 0.03f;
            state(1) = 0.11f;
            state(2) = NormalizeAngle(0.08f);
            state(3) = 1.2f;
            state(4) = 0.02f;
            state(5) = 0.15f;
            state(6) = 0.0f;
            state(7) = 0.0f;
            state(8) = 0.0f;
            return state;
        }

        Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> BuildPlanarAccelUpdateTestCovariance() noexcept
        {
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            covariance(0, 0) = 0.02f * 0.02f;
            covariance(1, 1) = 0.02f * 0.02f;
            covariance(2, 2) = 0.05f * 0.05f;
            covariance(3, 3) = 0.20f * 0.20f;
            covariance(4, 4) = 0.15f * 0.15f;
            covariance(5, 5) = 0.25f * 0.25f;
            covariance(6, 6) = 0.50f * 0.50f;
            covariance(7, 7) = 0.50f * 0.50f;
            covariance(8, 8) = 0.50f * 0.50f;
            return covariance;
        }

        App::Internal::CommandVector BuildPlanarAccelUpdateTestControl() noexcept
        {
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.26f);
            control.SetRightCommand(0.23f);
            return control;
        }

        ImuAccelObs BuildPlanarAccelObservation(
            EstimatorTestRuntime& runtime,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
            const App::Internal::CommandVector& control,
            float rightAccelDeltaMps2,
            float forwardAccelDeltaMps2) noexcept
        {
            runtime.runtimeState.SetPosition(Eigen::Vector2f(state(0), state(1)));
            runtime.runtimeState.SetHeading(state(2));
            runtime.runtimeState.SetForwardVelocity(state(3));
            runtime.runtimeState.SetRightwardVelocity(state(4));
            runtime.runtimeState.SetYawRate(state(5));
            runtime.runtimeState.SetForwardAccelerationResidual(state(6));
            runtime.runtimeState.SetRightwardAccelerationResidual(state(7));
            runtime.runtimeState.SetYawAccelResidual(state(8));
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                state(3),
                state(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            runtime.runtimeState.SetWheelSpeedLeft(leftWheelSpeedRadps);
            runtime.runtimeState.SetWheelSpeedRight(rightWheelSpeedRadps);
            runtime.plantModel.integrate(control, kPlanarAccelUpdateTestDtSeconds);

            const Eigen::Vector2f imuLeverArmBodyM = Vehicle::GetBackLeftImuMount().positionBodyM();
            const float yawRateRadps = state(5);
            const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
            const Eigen::Vector2f predicted(
                runtime.runtimeState.GetRightAcceleration() -
                    (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                    (runtime.runtimeState.GetYawAccel() * imuLeverArmBodyM.y()),
                runtime.runtimeState.GetForwardAcceleration() -
                    (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                    (runtime.runtimeState.GetYawAccel() * imuLeverArmBodyM.x()));
            return ImuAccelObs(
                true,
                predicted.y() + forwardAccelDeltaMps2,
                predicted.x() + rightAccelDeltaMps2);
        }
    }

    TEST_CLASS(EstimatorModeAndDiagnosticsTest)
    {
    private:
        static void PrimeCoreForPlanarAccelUpdate(
            Estimator& core,
            const Eigen::Matrix<float, VehicleState::kDimension, 1>& initialState,
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>& initialCovariance,
            const App::Internal::CommandVector& control)
        {
            Assert::IsTrue(core.reset(initialState, initialCovariance));
            Assert::IsTrue(core.predict(kPlanarAccelUpdateTestDtSeconds, control));

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(0);
            encoder.SetTotalRightCounts(0);
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                core.workingState()(3),
                core.workingState()(5),
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);
            encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            Assert::IsTrue(core.updateEncoderPair(encoder, kPlanarAccelUpdateTestDtSeconds, true));
        }

    public:
        TEST_METHOD(PlantModelEncoderPairSqrtNoise_UsesGeneralSigmaMappingForNonZeroReadings)
        {
            EstimatorTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            EncoderObs observation{};
            observation.SetLeftWheelSpeedRadps(1.0f);
            observation.SetRightWheelSpeedRadps(1.0f);

            const Eigen::Matrix<float, 2, 2> sqrtNoise =
                plantModel.encoderPairSqrtNoise(
                    observation,
                    kEstimatorTestStationaryEncoderVelocitySigmaMps,
                    kEstimatorTestGeneralEncoderLinearSpeedSigmaMps,
                    kEstimatorTestGeneralEncoderYawRateSigmaRadps);
            const Eigen::Matrix<float, 2, 2> covariance = sqrtNoise * sqrtNoise.transpose();
            const float varianceUMps2 =
                kEstimatorTestGeneralEncoderLinearSpeedSigmaMps * kEstimatorTestGeneralEncoderLinearSpeedSigmaMps;
            const float varianceYawRateRadps2 =
                kEstimatorTestGeneralEncoderYawRateSigmaRadps * kEstimatorTestGeneralEncoderYawRateSigmaRadps;
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalTrackWidthM();
            const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
            const float invWheelRadius2 = 1.0f / (wheelRadiusM * wheelRadiusM);
            const float expectedVarianceRadps2 =
                (varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2)) * invWheelRadius2;
            const float expectedCovarianceRadps2 =
                (varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2)) * invWheelRadius2;

            Assert::AreEqual(expectedVarianceRadps2, covariance(0, 0), 1.0e-5f);
            Assert::AreEqual(expectedVarianceRadps2, covariance(1, 1), 1.0e-5f);
            Assert::AreEqual(expectedCovarianceRadps2, covariance(0, 1), 1.0e-5f);
            Assert::AreEqual(expectedCovarianceRadps2, covariance(1, 0), 1.0e-5f);
        }

        TEST_METHOD(PlantModelStationaryEncoderWheelSpeedSigmaRadps_UsesRequestedZeroSpeedSigma)
        {
            EstimatorTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            const float expectedWheelSpeedSigmaRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kEstimatorTestStationaryEncoderVelocitySigmaMps);
            Assert::AreEqual(
                expectedWheelSpeedSigmaRadps,
                plantModel.stationaryEncoderWheelSpeedSigmaRadps(kEstimatorTestStationaryEncoderVelocitySigmaMps),
                1.0e-6f);
        }

        TEST_METHOD(BuildDefaultInitialCovariance_ReturnsCanonicalResetCovariance)
        {
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = Estimator::BuildDefaultInitialCovariance();

            Assert::AreEqual(1.0e-5f, covariance(0, 0), 1.0e-9f);
            Assert::AreEqual(1.0e-5f, covariance(1, 1), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(2, 2), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(3, 3), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(4, 4), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(5, 5), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(6, 6), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(7, 7), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(8, 8), 1.0e-9f);
        }

        TEST_METHOD(EstimatorPivotConflictKeepsEncoderYawOutOfBodyStateAndUsesGyroYaw)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.20f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.001f * 0.001f;
            initialCovariance(1, 1) = 0.001f * 0.001f;
            initialCovariance(2, 2) = 0.01f * 0.01f;
            initialCovariance(3, 3) = 0.04f * 0.04f;
            initialCovariance(4, 4) = 0.04f * 0.04f;
            initialCovariance(5, 5) = 0.20f * 0.20f;
            initialCovariance(6, 6) = 10.0f * 10.0f;
            initialCovariance(7, 7) = 10.0f * 10.0f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.60f);
            control.SetRightCommand(-0.60f);

            constexpr float kPivotDtSeconds = 0.001f;
            constexpr int kPivotLegacyStepScale = 10;
            int seedLeftCounts = 0;
            int seedRightCounts = 0;
            for (int seedIndex = 0; seedIndex < (8 * kPivotLegacyStepScale); ++seedIndex)
            {
                seedLeftCounts += (100 / kPivotLegacyStepScale);
                seedRightCounts -= (100 / kPivotLegacyStepScale);
                Assert::IsTrue(core.predict(kPivotDtSeconds, control));

                EncoderObs seedEncoder{};
                seedEncoder.SetTotalLeftCounts(seedLeftCounts);
                seedEncoder.SetTotalRightCounts(seedRightCounts);
                seedEncoder.SetLeftWheelSpeedRadps(12.0f);
                seedEncoder.SetRightWheelSpeedRadps(-12.0f);
                runtime.runtimeState.SetWheelSpeedLeft(seedEncoder.LeftWheelSpeedRadps());
                runtime.runtimeState.SetWheelSpeedRight(seedEncoder.RightWheelSpeedRadps());
                const bool seedEncoderAccepted =
                    core.updateEncoderPair(seedEncoder, kPivotDtSeconds, false);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(seedEncoderAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
                Assert::AreEqual(0.0f, core.LastUpdateNis(), 1.0e-6f);

                const bool seedYawAccepted = core.updateYawRate(0.0f);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(seedYawAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforePivot = core.workingState();
            constexpr float pivotGyroRawYawRateRadps = 1.80f;
            EncoderObs pivotEncoderObservation{};
            pivotEncoderObservation.SetLeftWheelSpeedRadps(18.0f);
            pivotEncoderObservation.SetRightWheelSpeedRadps(-18.0f);
            const float encoderDerivedYawRateRadps =
                runtime.plantModel.measuredYawRateRadps(pivotEncoderObservation);

            Assert::IsTrue(core.predict(kPivotDtSeconds, control), L"Pivot scrub predict step was rejected.");
            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateAfterPredict = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterPredict = core.workingCovariance();

            pivotEncoderObservation.SetTotalLeftCounts(seedLeftCounts + (120 / kPivotLegacyStepScale));
            pivotEncoderObservation.SetTotalRightCounts(seedRightCounts - (120 / kPivotLegacyStepScale));
            runtime.runtimeState.SetWheelSpeedLeft(pivotEncoderObservation.LeftWheelSpeedRadps());
            runtime.runtimeState.SetWheelSpeedRight(pivotEncoderObservation.RightWheelSpeedRadps());
            Assert::AreEqual(
                pivotEncoderObservation.LeftWheelSpeedRadps(),
                runtime.runtimeState.GetWheelSpeedLeft(),
                1.0e-6f);
            Assert::AreEqual(
                pivotEncoderObservation.RightWheelSpeedRadps(),
                runtime.runtimeState.GetWheelSpeedRight(),
                1.0e-6f);
            const bool pivotEncoderAccepted =
                core.updateEncoderPair(pivotEncoderObservation, kPivotDtSeconds, false);
            Assert::IsTrue(core.LastUpdateAttempted(), L"Pivot scrub encoder update was not attempted.");
            Assert::IsTrue(pivotEncoderAccepted, L"Pivot scrub encoder update was rejected.");
            Assert::IsTrue(core.LastUpdateAccepted(), L"Pivot scrub encoder update was rejected.");
            Assert::AreEqual(0.0f, core.LastUpdateNis(), 1.0e-6f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateAfterEncoder = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterEncoder = core.workingCovariance();
            Assert::AreEqual(stateAfterPredict(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(3), stateAfterEncoder(3), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(5), stateAfterEncoder(5), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(6), stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(7), stateAfterEncoder(7), 1.0e-6f);
            Assert::AreEqual(stateAfterPredict(8), stateAfterEncoder(8), 1.0e-6f);
            for (int row = 0; row < 9; ++row)
            {
                for (int col = 0; col < 9; ++col)
                {
                    Assert::AreEqual(
                        covarianceAfterPredict(row, col),
                        covarianceAfterEncoder(row, col),
                        1.0e-7f);
                }
            }

            const float gyroCorrectedYawRateRadps =
                pivotGyroRawYawRateRadps - runtime.runtimeState.GetGyroBiasZ();
            const float gyroScaleRadpsPerLsb =
                runtime.vehicle.BackLeftImu().GyroSensitivityMdpsPerLsb() * 0.001f * DEG_TO_RAD_F;
            const float gyroScaleToleranceSigmaRadps =
                std::fabs(gyroCorrectedYawRateRadps) *
                kEstimatorTestImuGyroSensitivityToleranceFraction /
                std::sqrt(3.0f);
            const float yawInnovation =
                gyroCorrectedYawRateRadps - stateAfterEncoder(5);
            const float yawInnovationVariance =
                covarianceAfterEncoder(5, 5) +
                kEstimatorTestImuYawRateVarianceRadps2 +
                ((gyroScaleRadpsPerLsb * gyroScaleRadpsPerLsb) / 12.0f) +
                (gyroScaleToleranceSigmaRadps * gyroScaleToleranceSigmaRadps) +
                runtime.runtimeState.GetGyroBiasZVar();
            const float yawGain = covarianceAfterEncoder(5, 5) / yawInnovationVariance;
            const float expectedYawRate = stateAfterEncoder(5) + (yawGain * yawInnovation);
            const float expectedYawVariance =
                covarianceAfterEncoder(5, 5) -
                (yawGain * yawInnovationVariance * yawGain);
            const float expectedYawNis =
                (yawInnovation * yawInnovation) / yawInnovationVariance;
            const bool pivotYawAccepted =
                core.updateYawRate(pivotGyroRawYawRateRadps);
            Assert::IsTrue(core.LastUpdateAttempted(), L"Pivot scrub yaw update was not attempted.");
            Assert::IsTrue(pivotYawAccepted, L"Pivot scrub yaw update was rejected.");
            Assert::IsTrue(core.LastUpdateAccepted(), L"Pivot scrub yaw update was rejected.");
            Assert::AreEqual(expectedYawNis, core.LastUpdateNis(), 1.0e-4f);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& stateAfterPivot = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covarianceAfterPivot = core.workingCovariance();
            Assert::AreEqual(stateAfterEncoder(0), stateAfterPivot(0), 1.0e-6f);
            Assert::AreEqual(stateAfterEncoder(1), stateAfterPivot(1), 1.0e-6f);
            Assert::AreEqual(stateAfterEncoder(2), stateAfterPivot(2), 1.0e-6f);
            Assert::AreEqual(stateAfterEncoder(3), stateAfterPivot(3), 1.0e-6f);
            Assert::AreEqual(stateAfterEncoder(4), stateAfterPivot(4), 1.0e-6f);
            Assert::AreEqual(expectedYawRate, stateAfterPivot(5), 1.0e-5f);
            Assert::AreEqual(stateAfterEncoder(6), stateAfterPivot(6), 1.0e-6f);
            Assert::AreEqual(stateAfterEncoder(7), stateAfterPivot(7), 1.0e-6f);
            Assert::AreEqual(stateAfterEncoder(8), stateAfterPivot(8), 1.0e-6f);
            Assert::AreEqual(expectedYawVariance, covarianceAfterPivot(5, 5), 1.0e-7f);
            Assert::AreEqual(0.0f, covarianceAfterPivot(5, 6), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterPivot(5, 7), 1.0e-8f);
            Assert::AreEqual(0.0f, covarianceAfterPivot(5, 8), 1.0e-8f);
            Assert::IsTrue(
                std::fabs(stateAfterPivot(5) - gyroCorrectedYawRateRadps) <
                std::fabs(stateAfterPivot(5) - encoderDerivedYawRateRadps));
            Assert::AreEqual(stateBeforePivot(6), stateAfterPivot(6), 1.0e-6f);
            Assert::AreEqual(stateBeforePivot(7), stateAfterPivot(7), 1.0e-6f);
            Assert::AreEqual(stateBeforePivot(8), stateAfterPivot(8), 1.0e-6f);
        }

        TEST_METHOD(EstimatorPivotCommandWithoutYawConflictUpdatesBodyRatesWithoutPoseJump)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.10f;
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

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.60f);
            control.SetRightCommand(-0.60f);

            Assert::IsTrue(core.predict(0.001f, control));
            const Eigen::Matrix<float, VehicleState::kDimension, 1> stateBeforeEncoder = core.workingState();

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(10);
            encoder.SetTotalRightCounts(-10);
            encoder.SetLeftWheelSpeedRadps(0.60f);
            encoder.SetRightWheelSpeedRadps(-0.60f);
            const float measuredForwardSpeedMps = runtime.plantModel.measuredLinearSpeedMps(encoder);
            const float measuredYawRateRadps = runtime.plantModel.measuredYawRateRadps(encoder);
            Assert::IsTrue(core.updateEncoderPair(encoder, 0.001f, true));

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& stateAfterEncoder = core.workingState();
            Assert::AreEqual(stateBeforeEncoder(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::IsTrue(
                std::fabs(stateAfterEncoder(3) - measuredForwardSpeedMps) <
                std::fabs(stateBeforeEncoder(3) - measuredForwardSpeedMps));
            Assert::IsTrue(
                std::fabs(stateAfterEncoder(5) - measuredYawRateRadps) <
                std::fabs(stateBeforeEncoder(5) - measuredYawRateRadps));
            Assert::AreEqual(stateBeforeEncoder(6), stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(7), stateAfterEncoder(7), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(8), stateAfterEncoder(8), 1.0e-6f);
        }

        TEST_METHOD(EstimatorDiagnosticDebugDumpReportsPivotScrubInactiveForNonPivotMotion)
        {
            Estimator core = MakeDefaultEstimator();

            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.10f;
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

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.25f);

            Assert::IsTrue(core.predict(0.001f, control));

            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(6);
            encoder.SetTotalRightCounts(6);
            encoder.SetLeftWheelSpeedRadps(0.80f);
            encoder.SetRightWheelSpeedRadps(0.80f);
            Assert::IsTrue(core.updateEncoderPair(encoder, 0.001f, true));
            Assert::IsTrue(core.updateYawRate(0.02f));

            Assert::IsFalse(FindDebugDumpBool(core, "estimator_dump_pivot_scrub", "pivot_scrub_mode"));
            Assert::IsFalse(FindDebugDumpBool(core, "estimator_dump_pivot_scrub", "encoder_body_update_skipped"));
            Assert::IsFalse(FindDebugDumpBool(core, "estimator_dump_pivot_scrub", "zero_u_soft_applied"));
            Assert::AreEqual(0.0f, FindDebugDumpFloat(core, "estimator_dump_pivot_scrub_encoder", "masked_delta_norm"), 1.0e-6f);
            Assert::AreEqual(0.0f, FindDebugDumpFloat(core, "estimator_dump_pivot_scrub_zero_u", "innovation_mps"), 1.0e-6f);
            Assert::AreEqual(0.0f, FindDebugDumpFloat(core, "estimator_dump_pivot_scrub_gyro", "masked_delta_norm"), 1.0e-6f);

            const std::vector<std::pair<std::string, std::string>> dumpLines = CollectDebugDumpLines(core);
            const std::size_t pivotModeIndex = FindFirstDumpLineIndexContaining(dumpLines, "estimator_dump_pivot_scrub");
            Assert::IsTrue(pivotModeIndex < dumpLines.size());
            Assert::IsTrue(dumpLines[pivotModeIndex].second.find("pivot_scrub_mode=false") != std::string::npos);
        }
        TEST_METHOD(EstimatorLaunchTransientDoesNotForceLateralVelocityToZero)
        {
            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.20f;
            initialState(4) = 0.40f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.30f * 0.30f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.20f);
            control.SetRightCommand(0.20f);
            Assert::IsTrue(core.predict(0.001f, control));

            control.SetLeftCommand(-0.20f);
            control.SetRightCommand(-0.20f);
            Assert::IsTrue(core.predict(0.001f, control));

            EncoderObs encoder{};
            encoder.SetLeftWheelSpeedRadps(2.0f);
            encoder.SetRightWheelSpeedRadps(2.0f);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const bool yawAccepted = core.updateYawRate(0.0f);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(yawAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(4)) > 0.10f);
        }

        TEST_METHOD(EstimatorPlanarAccelUpdateDampsLateralVelocityForCredibleRollingGrip)
        {
            const float forwardVelocityMps = 1.0f;
            const float yawRateRadps = 1.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                forwardVelocityMps,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);

            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = forwardVelocityMps;
            initialState(4) = 0.20f;
            initialState(5) = yawRateRadps;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.30f * 0.30f;
            initialCovariance(5, 5) = 0.10f * 0.10f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            EncoderObs encoder{};
            encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const float initialLateralVelocityMps = initialState(4);
            const float initialLateralVarianceMps2 = core.workingCovariance()(4, 4);

            const bool yawAccepted = core.updateYawRate(yawRateRadps);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(yawAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
            const bool lateralAidAccepted =
                core.updatePlanarAccel(noPlanarAccelObservation);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(lateralAidAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(state(4)));
            Assert::IsTrue(std::fabs(state(4)) < 0.05f);
            Assert::IsTrue(std::fabs(state(4)) < std::fabs(initialLateralVelocityMps));
            Assert::IsTrue(std::isfinite(covariance(4, 4)));
            Assert::IsTrue(covariance(4, 4) < initialLateralVarianceMps2);
        }

        TEST_METHOD(EstimatorPlanarAccelUpdateDoesNotOverConstrainLateralVelocityWhenRollingGripIsNotCredible)
        {
            const float forwardVelocityMps = 2.0f;
            const float yawRateRadps = 8.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelSpeedsFromBodyVelocity(
                forwardVelocityMps,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);

            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = forwardVelocityMps;
            initialState(4) = 0.20f;
            initialState(5) = yawRateRadps;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.01f * 0.01f;
            initialCovariance(1, 1) = 0.01f * 0.01f;
            initialCovariance(2, 2) = 0.03f * 0.03f;
            initialCovariance(3, 3) = 0.05f * 0.05f;
            initialCovariance(4, 4) = 0.001f * 0.001f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.30f * 0.30f;
            initialCovariance(7, 7) = 0.30f * 0.30f;
            initialCovariance(8, 8) = 0.03f * 0.03f;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            EncoderObs encoder{};
            encoder.SetLeftWheelSpeedRadps(leftWheelSpeedRadps);
            encoder.SetRightWheelSpeedRadps(rightWheelSpeedRadps);
            const bool encoderAccepted = core.updateEncoderPair(encoder, 0.001f, true);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(encoderAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const bool yawAccepted = core.updateYawRate(yawRateRadps);
            Assert::IsTrue(core.LastUpdateAttempted());
            Assert::IsTrue(yawAccepted);
            Assert::IsTrue(core.LastUpdateAccepted());

            const ImuAccelObs noPlanarAccelObservation(true, 0.0f, 0.0f);
            const bool lateralAidAccepted =
                core.updatePlanarAccel(noPlanarAccelObservation);
            Assert::IsTrue(core.LastUpdateAttempted());

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            Assert::IsTrue(std::fabs(state(4)) > 0.10f);
            Assert::IsTrue(std::isfinite(covariance(4, 4)));
            Assert::IsTrue(covariance(4, 4) >= (0.020f * 0.020f));
        }

        TEST_METHOD(EstimatorYawAndPlanarAccelUpdatesRemainCallableSeparately)
        {
            EstimatorTestRuntime runtime;
            const Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = BuildPlanarAccelUpdateTestState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            Estimator mergedCore = MakeDefaultEstimator();
            Estimator sequentialCore = MakeDefaultEstimator();
            PrimeCoreForPlanarAccelUpdate(
                mergedCore,
                initialState,
                initialCovariance,
                control);
            PrimeCoreForPlanarAccelUpdate(
                sequentialCore,
                initialState,
                initialCovariance,
                control);

            const ImuAccelObs accelObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    mergedCore.workingState(),
                    control,
                    0.35f,
                    -0.55f);
            const float gyroObservation =
                mergedCore.workingState()(5) +
                0.08f;

            const bool yawAccepted = sequentialCore.updateYawRate(gyroObservation);
            const bool yawAttempted = sequentialCore.LastUpdateAttempted();
            const bool accelAccepted = sequentialCore.updatePlanarAccel(accelObservation);
            const bool accelAttempted = sequentialCore.LastUpdateAttempted();
            const bool mergedYawAccepted = mergedCore.updateYawRate(gyroObservation);
            const bool mergedYawAttempted = mergedCore.LastUpdateAttempted();
            const bool mergedAccelAccepted = mergedCore.updatePlanarAccel(accelObservation);
            const bool mergedAccelAttempted = mergedCore.LastUpdateAttempted();

            Assert::IsTrue(yawAttempted);
            Assert::IsTrue(yawAccepted);
            Assert::IsTrue(accelAttempted);
            Assert::IsTrue(accelAccepted);
            Assert::IsTrue(mergedYawAttempted);
            Assert::IsTrue(mergedYawAccepted);
            Assert::IsTrue(mergedAccelAttempted);
            Assert::IsTrue(mergedAccelAccepted);
            Assert::IsTrue(
                (mergedCore.workingState() - sequentialCore.workingState()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (mergedCore.workingCovariance() - sequentialCore.workingCovariance()).cwiseAbs().maxCoeff() <= 1.0e-6f);
        }

        TEST_METHOD(EstimatorPlanarAccelUpdateUsesForwardAndRightChannels)
        {
            EstimatorTestRuntime runtime;
            const Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = BuildPlanarAccelUpdateTestState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            Estimator baselineCore = MakeDefaultEstimator();
            Estimator rightPerturbedCore = MakeDefaultEstimator();
            Estimator forwardPerturbedCore = MakeDefaultEstimator();
            PrimeCoreForPlanarAccelUpdate(
                baselineCore,
                initialState,
                initialCovariance,
                control);
            PrimeCoreForPlanarAccelUpdate(
                rightPerturbedCore,
                initialState,
                initialCovariance,
                control);
            PrimeCoreForPlanarAccelUpdate(
                forwardPerturbedCore,
                initialState,
                initialCovariance,
                control);

            const ImuAccelObs baselineObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    baselineCore.workingState(),
                    control,
                    0.0f,
                    0.35f);
            const ImuAccelObs rightPerturbedObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    rightPerturbedCore.workingState(),
                    control,
                    25.0f,
                    0.35f);
            const ImuAccelObs forwardPerturbedObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    forwardPerturbedCore.workingState(),
                    control,
                    0.0f,
                    0.55f);
            const bool baselineAccepted = baselineCore.updatePlanarAccel(baselineObservation);
            const bool baselineAttempted = baselineCore.LastUpdateAttempted();
            const bool rightAccepted =
                rightPerturbedCore.updatePlanarAccel(rightPerturbedObservation);
            const bool rightAttempted = rightPerturbedCore.LastUpdateAttempted();
            const bool forwardAccepted =
                forwardPerturbedCore.updatePlanarAccel(forwardPerturbedObservation);
            const bool forwardAttempted = forwardPerturbedCore.LastUpdateAttempted();

            Assert::IsTrue(baselineAttempted);
            Assert::IsTrue(baselineAccepted);
            Assert::IsTrue(rightAttempted);
            Assert::IsTrue(rightAccepted);
            Assert::IsTrue(forwardAttempted);
            Assert::IsTrue(forwardAccepted);

            Assert::IsTrue(
                (baselineCore.workingState() - rightPerturbedCore.workingState()).cwiseAbs().maxCoeff() > 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.workingCovariance() - rightPerturbedCore.workingCovariance()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.workingState() - forwardPerturbedCore.workingState()).cwiseAbs().maxCoeff() > 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.workingCovariance() - forwardPerturbedCore.workingCovariance()).cwiseAbs().maxCoeff() <=
                1.0e-6f);
        }
    };
}








