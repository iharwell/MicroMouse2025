#include "pch.h"
#include "CppUnitTest.h"

#include "SrUkfCoreTestSupport.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\UkfRobustUpdatePolicy.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        struct ScopedUkfRuntimeTuningRestore
        {
            SrUkfCore::RuntimeTuning saved = SrUkfCore::GetRuntimeTuning();

            ~ScopedUkfRuntimeTuningRestore()
            {
                SrUkfCore::SetRuntimeTuning(saved);
            }
        };

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

        ControlInput BuildPlanarAccelUpdateTestControl(const PlantParams& params) noexcept
        {
            ControlInput control{};
            control.leftMotorCommand = 0.26f;
            control.rightMotorCommand = 0.23f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            return control;
        }

        void PrimeCoreForPlanarAccelUpdate(
            SrUkfCore& core,
            const VehicleState::StateVector& initialState,
            const VehicleState::StateMatrix& initialCovariance,
            const ControlInput& control)
        {
            Assert::IsTrue(core.reset(initialState, initialCovariance));
            Assert::IsTrue(core.predict(kPlanarAccelUpdateTestDtSeconds, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 0;
            encoder.totalRightCounts = 0;
            encoder.omegaLeftRadps = core.state()(VehicleState::kOmegaL);
            encoder.omegaRightRadps = core.state()(VehicleState::kOmegaR);
            Assert::IsTrue(core.updateEncoderPair(encoder, kPlanarAccelUpdateTestDtSeconds).accepted);
        }

        ImuAccelObs BuildPlanarAccelObservation(
            const PlantModel& plant,
            const VehicleState::StateVector& state,
            const ControlInput& control,
            const PlantModel::PreparedParams& prepared,
            float lateralAccelDeltaMps2,
            float forwardAccelDeltaMps2) noexcept
        {
            const Eigen::Vector2f predicted = plant.imuPlanarAcceleration(state, control, prepared);
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
        TEST_METHOD(ComputeEncoderPairSqrtNoise_UsesGeneralSigmaMappingForNonZeroReadings)
        {
            const PlantParams params = PlantParams::Default();
            EncoderObs observation{};
            observation.omegaLeftRadps = 1.0f;
            observation.omegaRightRadps = 1.0f;

            const Eigen::Matrix<float, 2, 2> sqrtNoise = SrUkfCore::ComputeEncoderPairSqrtNoise(observation, params);
            const Eigen::Matrix<float, 2, 2> covariance = sqrtNoise * sqrtNoise.transpose();
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float varianceUMps2 =
                SrUkfCore::kGeneralEncoderLinearSpeedSigmaMps * SrUkfCore::kGeneralEncoderLinearSpeedSigmaMps;
            const float varianceYawRateRadps2 =
                SrUkfCore::kGeneralEncoderYawRateSigmaRadps * SrUkfCore::kGeneralEncoderYawRateSigmaRadps;
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

        TEST_METHOD(ComputeStationaryEncoderOmegaSigmaRadps_UsesRequestedZeroSpeedSigma)
        {
            const PlantParams params = PlantParams::Default();
            const float expectedSigmaRadps = SrUkfCore::kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;
            Assert::AreEqual(expectedSigmaRadps, SrUkfCore::ComputeStationaryEncoderOmegaSigmaRadps(params), 1.0e-9f);
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

        TEST_METHOD(SrUkfCoreRuntimeTuningOverridesRoundTripAndDriveStationaryThresholds)
        {
            ScopedUkfRuntimeTuningRestore restore;

            const SrUkfCore::RuntimeTuning defaults = SrUkfCore::BuildDefaultRuntimeTuning();
            SrUkfCore::RuntimeTuning tuning = defaults;
            tuning.stationaryEncoderVelocitySigmaMps = 0.02f;
            tuning.imuYawRateSigmaRadps = 0.05f;
            tuning.generalEncoderLinearSpeedSigmaMps = 0.01f;
            tuning.gripLinearProcessNoise.sigmaUSqrtQ = 0.123f;
            SrUkfCore::SetRuntimeTuning(tuning);

            const SrUkfCore::RuntimeTuning applied = SrUkfCore::GetRuntimeTuning();
            Assert::AreEqual(tuning.stationaryEncoderVelocitySigmaMps, applied.stationaryEncoderVelocitySigmaMps, 1.0e-9f);
            Assert::AreEqual(tuning.imuYawRateSigmaRadps, applied.imuYawRateSigmaRadps, 1.0e-9f);
            Assert::AreEqual(tuning.generalEncoderLinearSpeedSigmaMps, applied.generalEncoderLinearSpeedSigmaMps, 1.0e-9f);
            Assert::AreEqual(tuning.gripLinearProcessNoise.sigmaUSqrtQ, applied.gripLinearProcessNoise.sigmaUSqrtQ, 1.0e-9f);

            VehicleState state;
            VehicleState::StateVector stateVector = VehicleState::StateVector::Zero();
            stateVector(VehicleState::kU) = 0.015f;
            stateVector(VehicleState::kR) = 0.12f;
            const float wheelOmegaRadps = 0.5f * (tuning.stationaryEncoderVelocitySigmaMps / PlantParams::Default().wheelRadiusM);
            stateVector(VehicleState::kOmegaL) = wheelOmegaRadps;
            stateVector(VehicleState::kOmegaR) = wheelOmegaRadps;
            state.SetVelocity(stateVector(VehicleState::kU));
            state.SetRotationalVelocity(stateVector(VehicleState::kR));
            state.SetWheelSpeedLeft(stateVector(VehicleState::kOmegaL));
            state.SetWheelSpeedRight(stateVector(VehicleState::kOmegaR));
            Assert::IsTrue(state.IsStationary());

            SrUkfCore::ResetRuntimeTuning();
            const SrUkfCore::RuntimeTuning reset = SrUkfCore::GetRuntimeTuning();
            Assert::AreEqual(defaults.stationaryEncoderVelocitySigmaMps, reset.stationaryEncoderVelocitySigmaMps, 1.0e-9f);
            Assert::AreEqual(defaults.imuYawRateSigmaRadps, reset.imuYawRateSigmaRadps, 1.0e-9f);
            Assert::AreEqual(defaults.generalEncoderLinearSpeedSigmaMps, reset.generalEncoderLinearSpeedSigmaMps, 1.0e-9f);
            Assert::AreEqual(defaults.gripLinearProcessNoise.sigmaUSqrtQ, reset.gripLinearProcessNoise.sigmaUSqrtQ, 1.0e-9f);
            Assert::IsFalse(state.IsStationary());
        }

        TEST_METHOD(SrUkfCoreRegimeHelpersExposeSpecThresholds)
        {
            ControlInput control{};
            EncoderObs encoder{};
            Assert::IsTrue(SrUkfCore::IsStationaryCandidate(
                control,
                0.0f,
                0.0f,
                encoder,
                0.01f,
                0.0f,
                0.0f,
                0.0f,
                0U));
            Assert::IsFalse(SrUkfCore::IsStationaryCandidate(
                control,
                0.0f,
                0.0f,
                encoder,
                0.13f,
                0.0f,
                0.0f,
                0.0f,
                0U));

            Assert::AreEqual(0.005f, SrUkfCore::ComputeNonholonomicSigmaMps(0.0f), 1.0e-6f);
            Assert::AreEqual(0.040f, SrUkfCore::ComputeNonholonomicSigmaMps(1.0f), 1.0e-6f);
            Assert::IsTrue(SrUkfCore::HasLaunchOrReversalTrigger(0.10f, 0.0f, 0.0f, 0.0f, 0.0f, true, false));
            Assert::IsTrue(SrUkfCore::HasInconsistentOrSaturatedTrigger(0x1U, 0.0f, 0.0f, false, 0.0f));
            Assert::AreEqual(
                static_cast<int>(SrUkfCore::OperatingMode::InconsistentOrSaturated),
                static_cast<int>(SrUkfCore::ClassifyOperatingMode(false, true, true)));
            Assert::IsTrue(SrUkfCore::IsYawValidForFeedforward(
                SrUkfCore::OperatingMode::GripLinear,
                0.01f,
                0.0f,
                0.02f,
                true,
                0.01f,
                0.01f));
            Assert::IsFalse(SrUkfCore::IsYawValidForFeedforward(
                SrUkfCore::OperatingMode::InconsistentOrSaturated,
                0.0f,
                0.0f,
                0.0f,
                false,
                0.0f,
                0.01f));
        }

        TEST_METHOD(UkfRobustUpdatePolicyAppliesLaunchAndSevereEdgeScheduling)
        {
            const FrozenCycleSchedule nominal =
                UkfRobustUpdatePolicy::BuildFrozenCycleSchedule(
                    GripUtilizationSnapshot{},
                    TransientContactMemoryState{},
                    RegripRecoveryState{},
                    false,
                    true,
                    false,
                    false,
                    false,
                    false);

            const FrozenCycleSchedule launch =
                UkfRobustUpdatePolicy::BuildFrozenCycleSchedule(
                    GripUtilizationSnapshot{},
                    TransientContactMemoryState{},
                    RegripRecoveryState{},
                    false,
                    true,
                    false,
                    false,
                    true,
                    false);
            Assert::IsTrue(launch.lateralPseudoMeasurementCovarianceScale < nominal.lateralPseudoMeasurementCovarianceScale);

            GripUtilizationSnapshot oneBank{};
            oneBank.leftBankAnomalySeverity = 1.0f;
            const FrozenCycleSchedule oneBankSchedule =
                UkfRobustUpdatePolicy::BuildFrozenCycleSchedule(
                    oneBank,
                    TransientContactMemoryState{},
                    RegripRecoveryState{},
                    false,
                    true,
                    false,
                    false,
                    false,
                    false);
            Assert::IsTrue(oneBankSchedule.closureCovarianceScaleLeft > oneBankSchedule.closureCovarianceScaleRight);

            GripUtilizationSnapshot severe{};
            severe.longitudinalClosureSeverity = 1.0f;
            severe.differentialClosureSeverity = 1.0f;
            severe.lateralAccelerationSeverity = 1.0f;
            severe.yawConsistencySeverity = 1.0f;
            severe.leftBankAnomalySeverity = 1.0f;
            severe.rightBankAnomalySeverity = 1.0f;
            severe.leftBankPreProjectionUtilization = 1.20f;
            severe.rightBankPreProjectionUtilization = 1.15f;

            TransientContactMemoryState memory{};
            memory.leftBankMemory = 1.0f;
            memory.rightBankMemory = 1.0f;

            RegripRecoveryState regrip{};
            regrip.leftBankInRecovery = true;
            regrip.rightBankInRecovery = true;
            regrip.leftBankRecoveryScore = 1.0f;
            regrip.rightBankRecoveryScore = 1.0f;
            regrip.leftBankRecoveryTimeRemainingS = 0.05f;
            regrip.rightBankRecoveryTimeRemainingS = 0.05f;

            const FrozenCycleSchedule severeEdge =
                UkfRobustUpdatePolicy::BuildFrozenCycleSchedule(
                    severe,
                    memory,
                    regrip,
                    false,
                    true,
                    false,
                    false,
                    false,
                    true);
            Assert::IsTrue(severeEdge.leftBankHoldoffActive);
            Assert::IsTrue(severeEdge.rightBankHoldoffActive);
            Assert::IsTrue(severeEdge.lateralPseudoMeasurementCovarianceScale >= 64.0f);
        }

        TEST_METHOD(SrUkfCorePivotScrubModeMasksEncoderYawAndAppliesSoftZeroU)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);

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

            ControlInput control{};
            control.leftMotorCommand = 0.60f;
            control.rightMotorCommand = -0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

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
                core.setRuntimeContext(0.0f, 2.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
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
            bool yawConflictSeeded = false;
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
                Assert::IsFalse(core.pivotScrubMode());

                yawConflictSeeded =
                    (std::fabs(core.yawConsistencyLowPassRadps()) > SrUkfCore::kPivotScrubYawConsistencyThresholdRadps) ||
                    (std::fabs(core.yawWindowMismatchRad()) > SrUkfCore::kPivotScrubYawWindowMismatchThresholdRad);
                if (yawConflictSeeded)
                {
                    break;
                }
            }

            Assert::IsTrue(yawConflictSeeded, L"Pivot scrub seed phase never produced a yaw conflict.");

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

            Assert::IsTrue(core.pivotScrubMode(), L"Pivot scrub mode did not engage after the conflict tick.");
            Assert::IsTrue(core.pivotScrubEncoderBodyUpdateSkipped());
            Assert::IsTrue(core.pivotScrubZeroUSoftApplied());
            Assert::IsTrue(std::isfinite(core.pivotScrubEncoderWheelMaskedDeltaNorm()));
            Assert::IsTrue(core.pivotScrubEncoderWheelMaskedDeltaNorm() > 0.0f);
            Assert::AreEqual(0.0f, core.pivotScrubEncoderWheelDeltaPsiRad(), 1.0e-4f);
            Assert::AreEqual(0.0f, core.pivotScrubEncoderWheelDeltaRRadps(), 1.0e-4f);
            Assert::IsTrue(std::fabs(core.pivotScrubEncoderWheelDeltaOmegaLRadps()) > 0.0f);
            Assert::IsTrue(std::fabs(core.pivotScrubEncoderWheelDeltaOmegaRRadps()) > 0.0f);
            Assert::IsTrue(std::isfinite(core.pivotScrubGyroDeltaPsiRad()));
            Assert::IsTrue(std::fabs(core.pivotScrubGyroDeltaRRadps()) > 0.0f);
            Assert::IsTrue(std::isfinite(core.pivotScrubGyroDeltaBgzRadps()));
            Assert::AreEqual(0.0f, core.pivotScrubGyroDeltaOmegaLRadps(), 1.0e-4f);
            Assert::AreEqual(0.0f, core.pivotScrubGyroDeltaOmegaRRadps(), 1.0e-4f);
            Assert::IsTrue(core.pivotScrubGyroMaskedDeltaNorm() > 0.0f);
            Assert::IsTrue(std::fabs(core.pivotScrubZeroUInnovationMps()) > 0.0f);
            Assert::IsTrue(std::fabs(core.pivotScrubZeroUDeltaMps()) > 0.0f);

            const std::vector<std::pair<std::string, std::string>> dumpLines = CollectDebugDumpLines(core);
            const std::size_t pivotModeIndex = FindFirstDumpLineIndexContaining(dumpLines, "ukf_dump_pivot_scrub");
            const std::size_t encoderIndex = FindFirstDumpLineIndexContaining(dumpLines, "ukf_dump_pivot_scrub_encoder");
            const std::size_t zeroUIndex = FindFirstDumpLineIndexContaining(dumpLines, "ukf_dump_pivot_scrub_zero_u");
            const std::size_t gyroIndex = FindFirstDumpLineIndexContaining(dumpLines, "ukf_dump_pivot_scrub_gyro");

            Assert::IsTrue(pivotModeIndex < dumpLines.size());
            Assert::IsTrue(encoderIndex < dumpLines.size());
            Assert::IsTrue(zeroUIndex < dumpLines.size());
            Assert::IsTrue(gyroIndex < dumpLines.size());

            const std::string& pivotModeLine = dumpLines[pivotModeIndex].second;
            const std::string& encoderLine = dumpLines[encoderIndex].second;
            const std::string& zeroULine = dumpLines[zeroUIndex].second;
            const std::string& gyroLine = dumpLines[gyroIndex].second;
            Assert::IsTrue(pivotModeLine.find("pivot_scrub_mode=true") != std::string::npos);
            Assert::IsTrue(encoderLine.find("masked_delta_norm=") != std::string::npos);
            Assert::IsTrue(zeroULine.find("innovation_mps=") != std::string::npos);
            Assert::IsTrue(gyroLine.find("delta_omega_l_radps=") != std::string::npos);
            Assert::IsTrue(gyroLine.find("masked_delta_norm=") != std::string::npos);
        }

        TEST_METHOD(SrUkfCorePivotCommandWithoutYawConflictDoesNotEnterPivotScrubMode)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);

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

            ControlInput control{};
            control.leftMotorCommand = 0.60f;
            control.rightMotorCommand = -0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            core.setRuntimeContext(0.0f, 2.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
            Assert::IsTrue(core.predict(0.001f, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 10;
            encoder.totalRightCounts = -10;
            encoder.omegaLeftRadps = 0.60f;
            encoder.omegaRightRadps = -0.60f;
            Assert::IsTrue(core.updateEncoderPair(encoder, 0.001f).accepted);

            Assert::IsFalse(core.pivotScrubMode());
            Assert::IsFalse(core.pivotScrubEncoderBodyUpdateSkipped());
            Assert::IsFalse(core.pivotScrubZeroUSoftApplied());
        }

        TEST_METHOD(SrUkfCoreNonPivotMotionLeavesPivotScrubTelemetryCleared)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);

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

            ControlInput control{};
            control.leftMotorCommand = 0.25f;
            control.rightMotorCommand = 0.25f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            core.setRuntimeContext(0.12f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
            Assert::IsTrue(core.predict(0.001f, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 6;
            encoder.totalRightCounts = 6;
            encoder.omegaLeftRadps = 0.80f;
            encoder.omegaRightRadps = 0.80f;
            Assert::IsTrue(core.updateEncoderPair(encoder, 0.001f).accepted);
            Assert::IsTrue(core.updateYawRate(0.02f).accepted);

            Assert::IsFalse(core.pivotScrubMode());
            Assert::IsFalse(core.pivotScrubEncoderBodyUpdateSkipped());
            Assert::IsFalse(core.pivotScrubZeroUSoftApplied());
            Assert::AreEqual(0.0f, core.pivotScrubEncoderWheelMaskedDeltaNorm(), 1.0e-6f);
            Assert::AreEqual(0.0f, core.pivotScrubZeroUInnovationMps(), 1.0e-6f);
            Assert::AreEqual(0.0f, core.pivotScrubGyroMaskedDeltaNorm(), 1.0e-6f);

            const std::vector<std::pair<std::string, std::string>> dumpLines = CollectDebugDumpLines(core);
            const std::size_t pivotModeIndex = FindFirstDumpLineIndexContaining(dumpLines, "ukf_dump_pivot_scrub");
            Assert::IsTrue(pivotModeIndex < dumpLines.size());
            Assert::IsTrue(dumpLines[pivotModeIndex].second.find("pivot_scrub_mode=false") != std::string::npos);
        }
        TEST_METHOD(SrUkfCoreLaunchModeDisablesGripPseudoMeasurement)
        {
            SrUkfCore core;
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

            ControlInput control{};
            control.leftMotorCommand = 0.20f;
            control.rightMotorCommand = 0.20f;
            core.setRuntimeContext(0.20f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
            Assert::IsTrue(core.predict(0.001f, control));

            control.leftMotorCommand = -0.20f;
            control.rightMotorCommand = -0.20f;
            core.setRuntimeContext(-0.20f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
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

            const VehicleState::StateVector& state = core.state();
            Assert::AreEqual(
                static_cast<int>(SrUkfCore::OperatingMode::LaunchOrReversalTransient),
                static_cast<int>(core.operatingMode()));
            Assert::IsFalse(core.nonholonomicConstraintEnabled());
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) > 0.10f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateAppliesGripPseudoMeasurementForCredibleRollingGrip)
        {
            const PlantParams params = PlantParams::Default();
            const float forwardVelocityMps = 1.0f;
            const float yawRateRadps = 1.0f;
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float leftWheelSpeedRadps =
                (forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;
            const float rightWheelSpeedRadps =
                (forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;

            SrUkfCore core;
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
            core.setRuntimeContext(1.0f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);

            EncoderObs encoder{};
            encoder.omegaLeftRadps = leftWheelSpeedRadps;
            encoder.omegaRightRadps = rightWheelSpeedRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const float initialLateralVelocityMps = core.state()(VehicleState::kV);
            const float initialLateralVarianceMps2 = core.covariance()(VehicleState::kV, VehicleState::kV);
            const float expectedSigmaMps = SrUkfCore::ComputeNonholonomicSigmaMps(forwardVelocityMps);

            const MeasurementUpdateResult yawResult = core.updateYawRate(yawRateRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::IsTrue(core.nonholonomicConstraintEnabled());
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 0.02f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < std::fabs(initialLateralVelocityMps));
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) < initialLateralVarianceMps2);
            Assert::AreEqual(expectedSigmaMps, core.nhcSigmaMps(), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateInflatesLateralVarianceWhenRollingGripIsNotCredible)
        {
            const PlantParams params = PlantParams::Default();
            const float forwardVelocityMps = 2.0f;
            const float yawRateRadps = 8.0f;
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float leftWheelSpeedRadps =
                (forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;
            const float rightWheelSpeedRadps =
                (forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) / params.wheelRadiusM;

            SrUkfCore core;
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
            core.setRuntimeContext(2.0f, 0.0f, 0x1U, 0.0f, 0.0f, true, 0.0f, 0.0f);

            EncoderObs encoder{};
            encoder.omegaLeftRadps = leftWheelSpeedRadps;
            encoder.omegaRightRadps = rightWheelSpeedRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, 0.001f);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const MeasurementUpdateResult yawResult = core.updateYawRate(yawRateRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const float expectedSigmaMps = SrUkfCore::ComputeNonholonomicSigmaMps(forwardVelocityMps);
            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::IsFalse(core.nonholonomicConstraintEnabled());
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) > 0.10f);
            Assert::AreEqual(expectedSigmaMps, core.nhcSigmaMps(), 1.0e-6f);
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) >= (0.020f * 0.020f));
        }

        TEST_METHOD(SrUkfCoreRejectsInvalidMergedImuUpdate)
        {
            SrUkfCore core;
            ImuMergedObs observation{};
            observation.valid = false;
            observation.gyroZRadps = 0.5f;
            observation.accelBodyXMps2 = 1.0f;
            observation.accelBodyYMps2 = 0.1f;

            const MeasurementUpdateResult result = core.updateImuMerged(observation);
            Assert::IsFalse(result.attempted);
            Assert::IsFalse(result.accepted);
        }

        TEST_METHOD(SrUkfCoreMergedImuUpdateMatchesSequentialYawAndPlanarAccelPath)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel plant;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector initialState = BuildPlanarAccelUpdateTestState();
            const VehicleState::StateMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const ControlInput control = BuildPlanarAccelUpdateTestControl(params);

            SrUkfCore mergedCore(params);
            SrUkfCore sequentialCore(params);
            PrimeCoreForPlanarAccelUpdate(mergedCore, initialState, initialCovariance, control);
            PrimeCoreForPlanarAccelUpdate(sequentialCore, initialState, initialCovariance, control);

            const ImuAccelObs accelObservation =
                BuildPlanarAccelObservation(
                    plant,
                    mergedCore.state(),
                    control,
                    prepared,
                    0.35f,
                    -0.55f);
            ImuMergedObs mergedObservation{};
            mergedObservation.valid = true;
            mergedObservation.gyroZRadps =
                mergedCore.state()(VehicleState::kR) +
                mergedCore.state()(VehicleState::kBgz) +
                0.08f;
            mergedObservation.accelBodyXMps2 = accelObservation.accelBodyXMps2;
            mergedObservation.accelBodyYMps2 = accelObservation.accelBodyYMps2;

            const MeasurementUpdateResult mergedResult = mergedCore.updateImuMerged(mergedObservation);
            const MeasurementUpdateResult yawResult = sequentialCore.updateYawRate(mergedObservation.gyroZRadps);
            const MeasurementUpdateResult accelResult = sequentialCore.updatePlanarAccel(accelObservation);

            Assert::IsTrue(mergedResult.attempted);
            Assert::IsTrue(mergedResult.accepted);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);
            Assert::IsTrue(accelResult.attempted);
            Assert::IsTrue(accelResult.accepted);
            Assert::IsTrue(
                (mergedCore.state() - sequentialCore.state()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (mergedCore.covariance() - sequentialCore.covariance()).cwiseAbs().maxCoeff() <= 1.0e-6f);
        }

        TEST_METHOD(SrUkfCorePlanarAccelUpdateUsesForwardChannelAndIgnoresLateralOnlyPerturbation)
        {
            const PlantParams params = PlantParams::Default();
            const PlantModel plant;
            const PlantModel::PreparedParams prepared = PlantModel::Prepare(params);
            const VehicleState::StateVector initialState = BuildPlanarAccelUpdateTestState();
            const VehicleState::StateMatrix initialCovariance = BuildPlanarAccelUpdateTestCovariance();
            const ControlInput control = BuildPlanarAccelUpdateTestControl(params);

            SrUkfCore baselineCore(params);
            SrUkfCore lateralPerturbedCore(params);
            SrUkfCore forwardPerturbedCore(params);
            PrimeCoreForPlanarAccelUpdate(baselineCore, initialState, initialCovariance, control);
            PrimeCoreForPlanarAccelUpdate(lateralPerturbedCore, initialState, initialCovariance, control);
            PrimeCoreForPlanarAccelUpdate(forwardPerturbedCore, initialState, initialCovariance, control);

            const ImuAccelObs baselineObservation =
                BuildPlanarAccelObservation(
                    plant,
                    baselineCore.state(),
                    control,
                    prepared,
                    0.0f,
                    0.35f);
            const ImuAccelObs lateralPerturbedObservation =
                BuildPlanarAccelObservation(
                    plant,
                    lateralPerturbedCore.state(),
                    control,
                    prepared,
                    25.0f,
                    0.35f);
            const ImuAccelObs forwardPerturbedObservation =
                BuildPlanarAccelObservation(
                    plant,
                    forwardPerturbedCore.state(),
                    control,
                    prepared,
                    0.0f,
                    -0.35f);

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
                (baselineCore.state() - lateralPerturbedCore.state()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::IsTrue(
                (baselineCore.covariance() - lateralPerturbedCore.covariance()).cwiseAbs().maxCoeff() <= 1.0e-6f);
            Assert::AreEqual(
                baselineCore.forwardAccelInnovationMps2(),
                lateralPerturbedCore.forwardAccelInnovationMps2(),
                1.0e-6f);

            Assert::IsTrue(
                std::fabs(
                    baselineCore.forwardAccelInnovationMps2() -
                    forwardPerturbedCore.forwardAccelInnovationMps2()) > 0.5f);
            Assert::IsTrue(
                (baselineCore.state() - forwardPerturbedCore.state()).cwiseAbs().maxCoeff() > 1.0e-6f);
        }
    };
}
