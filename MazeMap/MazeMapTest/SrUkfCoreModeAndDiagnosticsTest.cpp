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
            return BuildUkfState(
                0.03f,
                0.11f,
                0.08f,
                1.2f,
                0.02f,
                0.15f,
                12.0f,
                12.0f,
                0.01f);
        }

        VehicleState::StateMatrix BuildPlanarAccelUpdateTestCovariance() noexcept
        {
            return BuildUkfCovariance(0.02f, 0.05f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f);
        }

        App::Internal::CommandVector BuildPlanarAccelUpdateTestControl(const PlantParams& params) noexcept
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
            encoder.omegaLeftRadps = core.workingState()(VehicleState::kOmegaL);
            encoder.omegaRightRadps = core.workingState()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(encoder, kPlanarAccelUpdateTestDtSeconds).accepted);
        }

        ImuAccelObs BuildPlanarAccelObservation(
            const PlantModel& plant,
            const VehicleState::StateVector& state,
            const App::Internal::CommandVector& control,
            const PlantModel::PreparedParams& prepared,
            float lateralAccelDeltaMps2,
            float forwardAccelDeltaMps2) noexcept
        {
            const Eigen::Vector2f predicted =
                plant.imuPlanarAcceleration(state, control, prepared);
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
            const PlantParams params = PlantParams::Default();
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
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float varianceUMps2 =
                kUkfTestGeneralEncoderLinearSpeedSigmaMps * kUkfTestGeneralEncoderLinearSpeedSigmaMps;
            const float varianceYawRateRadps2 =
                kUkfTestGeneralEncoderYawRateSigmaRadps * kUkfTestGeneralEncoderYawRateSigmaRadps;
            const float invWheelRadius2 = 1.0f / (params.wheelRadiusM * params.wheelRadiusM);
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
            const PlantParams params = PlantParams::Default();
            SrUkfCoreTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            const float expectedSigmaRadps = kUkfTestStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;
            Assert::AreEqual(
                expectedSigmaRadps,
                plantModel.stationaryEncoderOmegaSigmaRadps(kUkfTestStationaryEncoderVelocitySigmaMps),
                1.0e-6f);
        }

        TEST_METHOD(BuildDefaultInitialCovariance_ReturnsCanonicalResetCovariance)
        {
            const VehicleState::StateMatrix covariance = SrUkfCore::BuildDefaultInitialCovariance();

            Assert::AreEqual(1.0e-5f, covariance(VehicleState::kPx, VehicleState::kPx), 1.0e-9f);
            Assert::AreEqual(1.0e-5f, covariance(VehicleState::kPy, VehicleState::kPy), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kPsi, VehicleState::kPsi), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kU, VehicleState::kU), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kV, VehicleState::kV), 1.0e-9f);
            Assert::AreEqual(1.0e-3f, covariance(VehicleState::kR, VehicleState::kR), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(VehicleState::kOmegaL, VehicleState::kOmegaL), 1.0e-9f);
            Assert::AreEqual(0.25f, covariance(VehicleState::kOmegaR, VehicleState::kOmegaR), 1.0e-9f);
            Assert::AreEqual(3.05e-4f, covariance(VehicleState::kBgz, VehicleState::kBgz), 1.0e-12f);
        }

        TEST_METHOD(SrUkfCorePivotConflictKeepsEncoderYawOutOfBodyStateAndUsesGyroYaw)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();

            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.20f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.04f, 0.04f, 0.20f, 10.0f, 0.02f);
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
                std::fabs((stateBeforePivot(VehicleState::kR) + stateBeforePivot(VehicleState::kBgz)) - 1.80f);

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
                std::fabs((stateAfterPivot(VehicleState::kR) + stateAfterPivot(VehicleState::kBgz)) - 1.80f);
            Assert::AreEqual(18.0f, stateAfterPivot(VehicleState::kOmegaL), 1.0e-5f);
            Assert::AreEqual(-18.0f, stateAfterPivot(VehicleState::kOmegaR), 1.0e-5f);
            Assert::IsTrue(postPivotGyroFitError < prePivotGyroFitError);
            Assert::IsTrue(std::fabs(stateAfterPivot(VehicleState::kU)) < 0.25f);
            Assert::IsTrue(std::isfinite(stateAfterPivot(VehicleState::kPsi)));
            Assert::IsTrue(std::isfinite(stateAfterPivot(VehicleState::kR)));
        }

        TEST_METHOD(SrUkfCorePivotCommandWithoutYawConflictUpdatesWheelStateWithoutBodyJump)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();

            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.10f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

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
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPx), stateAfterEncoder(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPy), stateAfterEncoder(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kPsi), stateAfterEncoder(VehicleState::kPsi), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kU), stateAfterEncoder(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kV), stateAfterEncoder(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(stateBeforeEncoder(VehicleState::kR), stateAfterEncoder(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(encoder.omegaLeftRadps, stateAfterEncoder(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(encoder.omegaRightRadps, stateAfterEncoder(VehicleState::kOmegaR), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreDiagnosticDebugDumpReportsPivotScrubInactiveForNonPivotMotion)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();

            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.10f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

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
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.20f,
                0.40f,
                0.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.30f, 0.05f, 0.30f, 0.03f)));

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
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) > 0.10f);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateDampsLateralVelocityForCredibleRollingGrip)
        {
            const PlantParams params = PlantParams::Default();
            const float forwardVelocityMps = 1.0f;
            const float yawRateRadps = 1.0f;
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float leftWheelSpeedRadps =
                (forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;
            const float rightWheelSpeedRadps =
                (forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;

            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                forwardVelocityMps,
                0.20f,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps,
                0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.30f, 0.10f, 0.30f, 0.03f)));

            EncoderObs encoder{};
            encoder.omegaLeftRadps = leftWheelSpeedRadps;
            encoder.omegaRightRadps = rightWheelSpeedRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const float initialLateralVelocityMps = core.workingState()(VehicleState::kV);
            const float initialLateralVarianceMps2 = core.workingCovariance()(VehicleState::kV, VehicleState::kV);

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
            Assert::IsTrue(std::isfinite(state(VehicleState::kV)));
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 0.02f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < std::fabs(initialLateralVelocityMps));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kV, VehicleState::kV)));
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) < initialLateralVarianceMps2);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateDoesNotOverConstrainLateralVelocityWhenRollingGripIsNotCredible)
        {
            const PlantParams params = PlantParams::Default();
            const float forwardVelocityMps = 2.0f;
            const float yawRateRadps = 8.0f;
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float leftWheelSpeedRadps =
                (forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;
            const float rightWheelSpeedRadps =
                (forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;

            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                forwardVelocityMps,
                0.20f,
                yawRateRadps,
                leftWheelSpeedRadps,
                rightWheelSpeedRadps,
                0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.001f, 0.05f, 0.30f, 0.03f)));

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
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) > 0.10f);
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kV, VehicleState::kV)));
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) >= (0.020f * 0.020f));
        }

        TEST_METHOD(SrUkfCoreYawAndPlanarAccelUpdatesRemainCallableSeparately)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCoreTestRuntime runtime;
            const PlantModel& plant = runtime.plantModel;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector initialState = BuildPlanarAccelUpdateTestState();
            const VehicleState::StateMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl(params);

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
                    plant,
                    mergedCore.workingState(),
                    control,
                    prepared,
                    0.35f,
                    -0.55f);
            const float gyroObservation =
                mergedCore.workingState()(VehicleState::kR) +
                mergedCore.workingState()(VehicleState::kBgz) +
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
            const PlantParams params = PlantParams::Default();
            SrUkfCoreTestRuntime runtime;
            const PlantModel& plant = runtime.plantModel;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector initialState = BuildPlanarAccelUpdateTestState();
            const VehicleState::StateMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const App::Internal::CommandVector control = BuildPlanarAccelUpdateTestControl(params);

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
                    plant,
                    baselineCore.workingState(),
                    control,
                    prepared,
                    0.0f,
                    0.35f);
            const ImuAccelObs lateralPerturbedObservation =
                BuildPlanarAccelObservation(
                    plant,
                    lateralPerturbedCore.workingState(),
                    control,
                    prepared,
                    25.0f,
                    0.35f);
            const ImuAccelObs forwardPerturbedObservation =
                BuildPlanarAccelObservation(
                    plant,
                    forwardPerturbedCore.workingState(),
                    control,
                    prepared,
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
                (baselineCore.workingCovariance() - forwardPerturbedCore.workingCovariance()).cwiseAbs().maxCoeff() >
                1.0e-6f);
        }
    };
}








