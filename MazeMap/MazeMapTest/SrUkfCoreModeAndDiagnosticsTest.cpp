#include "pch.h"
#include "CppUnitTest.h"

#include "SrUkfCoreTestSupport.h"
#include "..\MazeMap\PlantModel.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kPlanarAccelUpdateTestDtSeconds = 0.001f;

        VehicleState::StateVector BuildPlanarAccelUpdateTestState() noexcept
        {
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(0) = 0.03f;
            state(1) = 0.11f;
            state(2) = NormalizeAngle(0.08f);
            state(3) = 1.2f;
            state(4) = 0.02f;
            state(5) = 0.15f;
            state(6) = 12.0f;
            state(7) = 12.0f;
            state(8) = 0.01f;
            return state;
        }

        VehicleState::StateMatrix BuildPlanarAccelUpdateTestCovariance() noexcept
        {
            VehicleState::StateMatrix covariance = VehicleState::StateMatrix::Zero();
            covariance(0, 0) = 0.02f * 0.02f;
            covariance(1, 1) = 0.02f * 0.02f;
            covariance(2, 2) = 0.05f * 0.05f;
            covariance(3, 3) = 0.20f * 0.20f;
            covariance(4, 4) = 0.15f * 0.15f;
            covariance(5, 5) = 0.25f * 0.25f;
            covariance(6, 6) = 0.50f * 0.50f;
            covariance(7, 7) = 0.50f * 0.50f;
            covariance(8, 8) = 0.05f * 0.05f;
            return covariance;
        }

        App::Internal::CommandVector BuildPlanarAccelUpdateTestControl() noexcept
        {
            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.26f);
            control.SetRightCommand(0.23f);
            return control;
        }

        void PrimeCoreForPlanarAccelUpdate(
            SrUkfCore& core,
            const VehicleState::StateVector& initialState,
            const VehicleState::StateMatrix& initialCovariance,
            const App::Internal::CommandVector& control)
        {
            Assert::IsTrue(core.reset(initialState, initialCovariance));
            Assert::IsTrue(core.predict(kPlanarAccelUpdateTestDtSeconds, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 0;
            encoder.totalRightCounts = 0;
            encoder.omegaLeftRadps = core.workingState()(6);
            encoder.omegaRightRadps = core.workingState()(7);
            Assert::IsTrue(core.updateEncoderPair(encoder, kPlanarAccelUpdateTestDtSeconds).accepted);
        }

        ImuAccelObs BuildPlanarAccelObservation(
            SrUkfCoreTestRuntime& runtime,
            const VehicleState::StateVector& state,
            const App::Internal::CommandVector& control,
            float lateralAccelDeltaMps2,
            float forwardAccelDeltaMps2) noexcept
        {
            runtime.runtimeState.SetPosition(Eigen::Vector2f(state(0), state(1)));
            runtime.runtimeState.SetOrientation(state(2));
            runtime.runtimeState.SetVelocity(state(3));
            runtime.runtimeState.SetLateralVelocity(state(4));
            runtime.runtimeState.SetRotationalVelocity(state(5));
            runtime.runtimeState.SetWheelSpeedLeft(state(6));
            runtime.runtimeState.SetWheelSpeedRight(state(7));
            runtime.runtimeState.SetGyroBiasZ(state(8));
            runtime.plantModel.integrate(control, kPlanarAccelUpdateTestDtSeconds);

            const Eigen::Vector2f imuLeverArmBodyM = Vehicle::GetBackLeftImuMount().positionBodyM();
            const float yawRateRadps = state(5);
            const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
            const Eigen::Vector2f predicted(
                runtime.runtimeState.GetLateralAcceleration() -
                    (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                    (runtime.runtimeState.GetYawAcceleration() * imuLeverArmBodyM.y()),
                runtime.runtimeState.GetLongitudinalAcceleration() -
                    (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                    (runtime.runtimeState.GetYawAcceleration() * imuLeverArmBodyM.x()));
            ImuAccelObs observation{};
            observation.valid = true;
            observation.accelBodyXMps2 = predicted.x() + lateralAccelDeltaMps2;
            observation.accelBodyYMps2 = predicted.y() + forwardAccelDeltaMps2;
            return observation;
        }
    }

    TEST_CLASS(SrUkfCoreModeAndDiagnosticsTest)
    {
    public:
        TEST_METHOD(PlantModelEncoderPairSqrtNoise_UsesGeneralSigmaMappingForNonZeroReadings)
        {
            SrUkfCoreTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            EncoderObs observation{};
            observation.omegaLeftRadps = 1.0f;
            observation.omegaRightRadps = 1.0f;

            const Eigen::Matrix<float, 2, 2> sqrtNoise =
                plantModel.encoderPairSqrtNoise(
                    observation,
                    kUkfTestStationaryEncoderVelocitySigmaMps,
                    kUkfTestGeneralEncoderLinearSpeedSigmaMps,
                    kUkfTestGeneralEncoderYawRateSigmaRadps);
            const Eigen::Matrix<float, 2, 2> covariance = sqrtNoise * sqrtNoise.transpose();
            const float varianceUMps2 =
                kUkfTestGeneralEncoderLinearSpeedSigmaMps * kUkfTestGeneralEncoderLinearSpeedSigmaMps;
            const float varianceYawRateRadps2 =
                kUkfTestGeneralEncoderYawRateSigmaRadps * kUkfTestGeneralEncoderYawRateSigmaRadps;
            const float halfTrackWidthM = 0.5f * Vehicle::GetPhysicalModel().trackWidthM;
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

        TEST_METHOD(PlantModelStationaryEncoderOmegaSigmaRadps_UsesRequestedZeroSpeedSigma)
        {
            SrUkfCoreTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            const float expectedSigmaRadps =
                Vehicle::WheelOmegaFromLinearVelocity(kUkfTestStationaryEncoderVelocitySigmaMps);
            Assert::AreEqual(
                expectedSigmaRadps,
                plantModel.stationaryEncoderOmegaSigmaRadps(kUkfTestStationaryEncoderVelocitySigmaMps),
                1.0e-6f);
        }

        TEST_METHOD(BuildDefaultInitialCovariance_ReturnsCanonicalResetCovariance)
        {
            const VehicleState::StateMatrix covariance = SrUkfCore::BuildDefaultInitialCovariance();

            Assert::AreEqual(1.0e-5f, covariance(0, 0), 1.0e-9f);
            Assert::AreEqual(1.0e-5f, covariance(1, 1), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(2, 2), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(3, 3), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(4, 4), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(5, 5), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(6, 6), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(7, 7), 1.0e-9f);
            Assert::AreEqual(3.05e-4f, covariance(8, 8), 1.0e-12f);
        }

        TEST_METHOD(SrUkfCorePivotConflictKeepsEncoderYawOutOfBodyStateAndUsesGyroYaw)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.20f;
            initialState(4) = 0.0f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
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

            struct PivotTickResults
            {
                bool predictAccepted;
                MeasurementUpdateResult encoderResult;
                MeasurementUpdateResult yawResult;
            };

            constexpr float kPivotDtSeconds = 0.001f;
            constexpr int kPivotLegacyStepScale = 10;
            const auto runPivotTick =
                [&](int totalLeftCounts,
                    int totalRightCounts,
                    float omegaLeftRadps,
                    float omegaRightRadps,
                    float yawRateRawRadps) -> PivotTickResults
            {
                PivotTickResults results{};
                results.predictAccepted = core.predict(kPivotDtSeconds, control);

                EncoderObs encoder{};
                encoder.totalLeftCounts = totalLeftCounts;
                encoder.totalRightCounts = totalRightCounts;
                encoder.omegaLeftRadps = omegaLeftRadps;
                encoder.omegaRightRadps = omegaRightRadps;
                results.encoderResult = core.updateEncoderPair(encoder, kPivotDtSeconds);
                results.yawResult = core.updateYawRate(yawRateRawRadps);
                return results;
            };

            int seedLeftCounts = 0;
            int seedRightCounts = 0;
            for (int seedIndex = 0; seedIndex < (8 * kPivotLegacyStepScale); ++seedIndex)
            {
                seedLeftCounts += (100 / kPivotLegacyStepScale);
                seedRightCounts -= (100 / kPivotLegacyStepScale);
                const PivotTickResults seedResults = runPivotTick(seedLeftCounts, seedRightCounts, 12.0f, -12.0f, 0.0f);
                Assert::IsTrue(seedResults.predictAccepted);
                Assert::IsTrue(seedResults.encoderResult.attempted);
                Assert::IsTrue(seedResults.encoderResult.accepted);
                Assert::IsTrue(seedResults.yawResult.attempted);
                Assert::IsTrue(seedResults.yawResult.accepted);
            }

            const VehicleState::StateVector stateBeforePivot = core.workingState();
            const float prePivotGyroFitError =
                std::fabs((stateBeforePivot(5) + stateBeforePivot(8)) - 1.80f);

            const PivotTickResults pivotResults =
                runPivotTick(
                    seedLeftCounts + (120 / kPivotLegacyStepScale),
                    seedRightCounts - (120 / kPivotLegacyStepScale),
                    18.0f,
                    -18.0f,
                    1.80f);
            Assert::IsTrue(pivotResults.predictAccepted, L"Pivot scrub predict step was rejected.");
            Assert::IsTrue(pivotResults.encoderResult.attempted, L"Pivot scrub encoder update was not attempted.");
            Assert::IsTrue(pivotResults.encoderResult.accepted, L"Pivot scrub encoder update was rejected.");
            Assert::IsTrue(pivotResults.yawResult.attempted, L"Pivot scrub yaw update was not attempted.");
            Assert::IsTrue(pivotResults.yawResult.accepted, L"Pivot scrub yaw update was rejected.");

            const VehicleState::StateVector& stateAfterPivot = core.workingState();
            const float postPivotGyroFitError =
                std::fabs((stateAfterPivot(5) + stateAfterPivot(8)) - 1.80f);
            Assert::AreEqual(18.0f, stateAfterPivot(6), 1.0e-5f);
            Assert::AreEqual(-18.0f, stateAfterPivot(7), 1.0e-5f);
            Assert::IsTrue(postPivotGyroFitError < prePivotGyroFitError);
            Assert::IsTrue(std::fabs(stateAfterPivot(3)) < 0.25f);
            Assert::IsTrue(std::isfinite(stateAfterPivot(2)));
            Assert::IsTrue(std::isfinite(stateAfterPivot(5)));
        }

        TEST_METHOD(SrUkfCorePivotCommandWithoutYawConflictUpdatesWheelStateWithoutBodyJump)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.10f;
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

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.60f);
            control.SetRightCommand(-0.60f);

            Assert::IsTrue(core.predict(0.001f, control));
            const VehicleState::StateVector stateBeforeEncoder = core.workingState();

            EncoderObs encoder{};
            encoder.totalLeftCounts = 10;
            encoder.totalRightCounts = -10;
            encoder.omegaLeftRadps = 0.60f;
            encoder.omegaRightRadps = -0.60f;
            Assert::IsTrue(core.updateEncoderPair(encoder, 0.001f).accepted);

            const VehicleState::StateVector& stateAfterEncoder = core.workingState();
            Assert::AreEqual(stateBeforeEncoder(0), stateAfterEncoder(0), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(1), stateAfterEncoder(1), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(2), stateAfterEncoder(2), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(3), stateAfterEncoder(3), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(4), stateAfterEncoder(4), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(5), stateAfterEncoder(5), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateAfterEncoder(6), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, stateAfterEncoder(7), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreDiagnosticDebugDumpReportsPivotScrubInactiveForNonPivotMotion)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();

            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.10f;
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

            App::Internal::CommandVector control{};
            control.SetLeftCommand(0.25f);
            control.SetRightCommand(0.25f);

            Assert::IsTrue(core.predict(0.001f, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 6;
            encoder.totalRightCounts = 6;
            encoder.omegaLeftRadps = 0.80f;
            encoder.omegaRightRadps = 0.80f;
            Assert::IsTrue(core.updateEncoderPair(encoder, 0.001f).accepted);
            Assert::IsTrue(core.updateYawRate(0.02f).accepted);

            Assert::IsFalse(FindDebugDumpBool(core, "ukf_dump_pivot_scrub", "pivot_scrub_mode"));
            Assert::IsFalse(FindDebugDumpBool(core, "ukf_dump_pivot_scrub", "encoder_body_update_skipped"));
            Assert::IsFalse(FindDebugDumpBool(core, "ukf_dump_pivot_scrub", "zero_u_soft_applied"));
            Assert::AreEqual(0.0f, FindDebugDumpFloat(core, "ukf_dump_pivot_scrub_encoder", "masked_delta_norm"), 1.0e-6f);
            Assert::AreEqual(0.0f, FindDebugDumpFloat(core, "ukf_dump_pivot_scrub_zero_u", "innovation_mps"), 1.0e-6f);
            Assert::AreEqual(0.0f, FindDebugDumpFloat(core, "ukf_dump_pivot_scrub_gyro", "masked_delta_norm"), 1.0e-6f);

            const std::vector<std::pair<std::string, std::string>> dumpLines = CollectDebugDumpLines(core);
            const std::size_t pivotModeIndex = FindFirstDumpLineIndexContaining(dumpLines, "ukf_dump_pivot_scrub");
            Assert::IsTrue(pivotModeIndex < dumpLines.size());
            Assert::IsTrue(dumpLines[pivotModeIndex].second.find("pivot_scrub_mode=false") != std::string::npos);
        }
        TEST_METHOD(SrUkfCoreLaunchTransientDoesNotForceLateralVelocityToZero)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.20f;
            initialState(4) = 0.40f;
            initialState(5) = 0.0f;
            initialState(6) = 0.0f;
            initialState(7) = 0.0f;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
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
            encoder.omegaLeftRadps = 2.0f;
            encoder.omegaRightRadps = 2.0f;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(4)) > 0.10f);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateDampsLateralVelocityForCredibleRollingGrip)
        {
            const float forwardVelocityMps = 1.0f;
            const float yawRateRadps = 1.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelOmegasFromBodyVelocity(
                forwardVelocityMps,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);

            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = forwardVelocityMps;
            initialState(4) = 0.20f;
            initialState(5) = yawRateRadps;
            initialState(6) = leftWheelSpeedRadps;
            initialState(7) = rightWheelSpeedRadps;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
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
            encoder.omegaLeftRadps = leftWheelSpeedRadps;
            encoder.omegaRightRadps = rightWheelSpeedRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const float initialLateralVelocityMps = initialState(4);
            const float initialLateralVarianceMps2 = core.workingCovariance()(4, 4);

            const MeasurementUpdateResult yawResult = core.updateYawRate(yawRateRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            ImuAccelObs noPlanarAccelObservation{};
            const MeasurementUpdateResult lateralAidResult =
                core.updatePlanarAccel(noPlanarAccelObservation);
            Assert::IsTrue(lateralAidResult.attempted);
            Assert::IsTrue(lateralAidResult.accepted);

            const VehicleState::StateVector& state = core.workingState();
            const VehicleState::StateMatrix covariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(state(4)));
            Assert::IsTrue(std::fabs(state(4)) < 0.05f);
            Assert::IsTrue(std::fabs(state(4)) < std::fabs(initialLateralVelocityMps));
            Assert::IsTrue(std::isfinite(covariance(4, 4)));
            Assert::IsTrue(covariance(4, 4) < initialLateralVarianceMps2);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateDoesNotOverConstrainLateralVelocityWhenRollingGripIsNotCredible)
        {
            const float forwardVelocityMps = 2.0f;
            const float yawRateRadps = 8.0f;
            float leftWheelSpeedRadps = 0.0f;
            float rightWheelSpeedRadps = 0.0f;
            Vehicle::WheelOmegasFromBodyVelocity(
                forwardVelocityMps,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps);

            SrUkfCore core = MakeDefaultSrUkfCore();
            VehicleState::StateVector initialState = VehicleState::StateVector::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = (0.0f);
            initialState(3) = forwardVelocityMps;
            initialState(4) = 0.20f;
            initialState(5) = yawRateRadps;
            initialState(6) = leftWheelSpeedRadps;
            initialState(7) = rightWheelSpeedRadps;
            initialState(8) = 0.0f;
            VehicleState::StateMatrix initialCovariance = VehicleState::StateMatrix::Zero();
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
            encoder.omegaLeftRadps = leftWheelSpeedRadps;
            encoder.omegaRightRadps = rightWheelSpeedRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(yawRateRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            ImuAccelObs noPlanarAccelObservation{};
            const MeasurementUpdateResult lateralAidResult =
                core.updatePlanarAccel(noPlanarAccelObservation);
            Assert::IsTrue(lateralAidResult.attempted);

            const VehicleState::StateVector& state = core.workingState();
            const VehicleState::StateMatrix covariance = core.workingCovariance();
            Assert::IsTrue(std::fabs(state(4)) > 0.10f);
            Assert::IsTrue(std::isfinite(covariance(4, 4)));
            Assert::IsTrue(covariance(4, 4) >= (0.020f * 0.020f));
        }

        TEST_METHOD(SrUkfCoreYawAndPlanarAccelUpdatesRemainCallableSeparately)
        {
            SrUkfCoreTestRuntime runtime;
            const VehicleState::StateVector initialState = BuildPlanarAccelUpdateTestState();
            const VehicleState::StateMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            SrUkfCore mergedCore = MakeDefaultSrUkfCore();
            SrUkfCore sequentialCore = MakeDefaultSrUkfCore();
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
                mergedCore.workingState()(8) +
                0.08f;

            const MeasurementUpdateResult yawResult = sequentialCore.updateYawRate(gyroObservation);
            const MeasurementUpdateResult accelResult = sequentialCore.updatePlanarAccel(accelObservation);
            const MeasurementUpdateResult mergedYawResult = mergedCore.updateYawRate(gyroObservation);
            const MeasurementUpdateResult mergedAccelResult = mergedCore.updatePlanarAccel(accelObservation);

            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);
            Assert::IsTrue(accelResult.attempted);
            Assert::IsTrue(accelResult.accepted);
            Assert::IsTrue(mergedYawResult.attempted);
            Assert::IsTrue(mergedYawResult.accepted);
            Assert::IsTrue(mergedAccelResult.attempted);
            Assert::IsTrue(mergedAccelResult.accepted);
            Assert::IsTrue(
                (mergedCore.workingState() - sequentialCore.workingState()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (mergedCore.workingCovariance() - sequentialCore.workingCovariance()).cwiseAbs().maxCoeff() <= 1.0e-6f);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateUsesForwardChannelAndIgnoresLateralOnlyPerturbation)
        {
            SrUkfCoreTestRuntime runtime;
            const VehicleState::StateVector initialState = BuildPlanarAccelUpdateTestState();
            const VehicleState::StateMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl();

            SrUkfCore baselineCore = MakeDefaultSrUkfCore();
            SrUkfCore lateralPerturbedCore = MakeDefaultSrUkfCore();
            SrUkfCore forwardPerturbedCore = MakeDefaultSrUkfCore();
            PrimeCoreForPlanarAccelUpdate(
                baselineCore,
                initialState,
                initialCovariance,
                control);
            PrimeCoreForPlanarAccelUpdate(
                lateralPerturbedCore,
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
            const ImuAccelObs lateralPerturbedObservation =
                BuildPlanarAccelObservation(
                    runtime,
                    lateralPerturbedCore.workingState(),
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
            const MeasurementUpdateResult baselineResult = baselineCore.updatePlanarAccel(baselineObservation);
            const MeasurementUpdateResult lateralResult =
                lateralPerturbedCore.updatePlanarAccel(lateralPerturbedObservation);
            const MeasurementUpdateResult forwardResult =
                forwardPerturbedCore.updatePlanarAccel(forwardPerturbedObservation);

            Assert::IsTrue(baselineResult.attempted);
            Assert::IsTrue(baselineResult.accepted);
            Assert::IsTrue(lateralResult.attempted);
            Assert::IsTrue(lateralResult.accepted);
            Assert::IsTrue(forwardResult.attempted);
            Assert::IsTrue(forwardResult.accepted);

            Assert::IsTrue(
                (baselineCore.workingState() - lateralPerturbedCore.workingState()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.workingCovariance() - lateralPerturbedCore.workingCovariance()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.workingState() - forwardPerturbedCore.workingState()).cwiseAbs().maxCoeff() > 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.workingCovariance() - forwardPerturbedCore.workingCovariance()).cwiseAbs().maxCoeff() <=
                1.0e-6f);
        }
    };
}








