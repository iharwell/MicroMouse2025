#include "pch.h"
#include "CppUnitTest.h"

#include "SrUkfCoreTestSupport.h"

#include <algorithm>
#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        constexpr float kStationaryMotionTolerance = 5.0e-4f;
        constexpr float kStationaryPoseDriftToleranceM = 1.0e-3f;
        constexpr float kStationaryYawVarianceToleranceRadps2 = 1.1e-4f;
        constexpr float kStationaryBiasSeedYawRateTolerance = 5.0e-3f;
        constexpr float kStationaryBiasWalkTolerance = 1.0e-4f;

    }

    TEST_CLASS(SrUkfCoreBiasAndStationaryTest)
    {
    public:
        TEST_METHOD(SrUkfCoreStationaryYawConstraintCollapsesMotionStateWithoutSnappingBiasBackToAnchor)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.35f,
                0.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.002f;
            constexpr float rawStationaryGyroRadps = 0.04f;
            const int kSteps =
                static_cast<int>(std::ceil(kUkfTestStationaryCertificationDwellS / dt)) + 10;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps).accepted);
            }

            const VehicleState::StateVector& state = core.workingState();
            const VehicleState::StateMatrix covariance = core.workingCovariance();
            Assert::AreEqual(0, FindDebugDumpModeId(core));
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < params.stopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < params.stopEnterSpeedMps);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < params.stopEnterYawRateRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < params.stopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < params.stopEnterWheelSpeedRadps);
            Assert::IsTrue(std::fabs(state(VehicleState::kBgz)) > 1.0e-3f);
            Assert::IsTrue(
                std::fabs((state(VehicleState::kR) + state(VehicleState::kBgz)) - rawStationaryGyroRadps) <
                rawStationaryGyroRadps);
            Assert::IsFalse(FindDebugDumpBool(core, "ukf_dump_mode", "bias_update_enabled"));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kU, VehicleState::kU)));
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) < 1.0e-4f);
            Assert::IsTrue(
                covariance(VehicleState::kV, VehicleState::kV) < (0.005f * 0.005f),
                (std::wstring(L"Stationary lateral variance was ") +
                    std::to_wstring(covariance(VehicleState::kV, VehicleState::kV))).c_str());
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) >= (0.010f * 0.010f));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) < 1.0e-4f);
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kBgz, VehicleState::kBgz)));
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) > 0.0f);
        }

        TEST_METHOD(SrUkfCoreExactStationaryLockKeepsPoseFixedWhenEncoderCountsRemainNonZero)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            const VehicleState::StateVector initialState = BuildUkfState(
                0.42f,
                -0.17f,
                0.28f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            encoder.totalLeftCounts = 12;
            encoder.totalRightCounts = 8;
            constexpr float dt = 0.002f;
            const int kSteps =
                static_cast<int>(std::ceil(kUkfTestStationaryCertificationDwellS / dt)) + 10;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(0.0f).accepted);
            }

            const VehicleState::StateVector& state = core.workingState();
            Assert::AreEqual(0, FindDebugDumpModeId(core));
            Assert::IsTrue(FindDebugDumpBool(core, "ukf_dump_grip_recovery", "exact_stationary_lock"));
            Assert::AreEqual(initialState(VehicleState::kPx), state(VehicleState::kPx), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(VehicleState::kPy), state(VehicleState::kPy), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(VehicleState::kPsi), state(VehicleState::kPsi), 1.0e-4f);
        }

        TEST_METHOD(SrUkfCoreInitialStationaryGyroBiasSeedsFromSamplesFiftyToOneHundredFiftyAndSlowWalks)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            const int kSteps =
                (std::max)(
                    200,
                    static_cast<int>(std::ceil(kUkfTestStationaryCertificationDwellS / dt)) + 20);
            InitialStationaryGyroBiasExpectation expected{};

            for (int step = 0; step < kSteps; ++step)
            {
                const float rawStationaryGyroRadps =
                    (step < 49) ? 0.01f :
                    ((step < 150) ? 0.04f : 0.07f);
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps).accepted);
                AdvanceInitialStationaryGyroBiasExpectation(expected, rawStationaryGyroRadps, dt);
            }

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(expected.seedApplied);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < kStationaryBiasSeedYawRateTolerance);
            Assert::IsTrue(state(VehicleState::kBgz) > 0.04f);
            Assert::IsTrue(state(VehicleState::kBgz) < 0.08f);
            Assert::IsTrue(FindDebugDumpBool(core, "ukf_dump_mode", "bias_update_enabled"));
        }

        TEST_METHOD(SrUkfCoreStationaryGyroMeasurementDoesNotCollapseBiasVarianceToZero)
        {
            SrUkfCore core = MakeDefaultSrUkfCore();
            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            constexpr float rawStationaryGyroRadps = 0.04f;

            const VehicleState::StateMatrix beforeCovariance = core.workingCovariance();
            Assert::IsTrue(
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz) >
                0.0f);

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps).accepted);
            }

            const VehicleState::StateMatrix covariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kBgz, VehicleState::kBgz)));
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) > 0.0f);
            Assert::IsTrue(
                covariance(VehicleState::kBgz, VehicleState::kBgz) <
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz));
        }

        TEST_METHOD(SrUkfCoreGyroBiasDefaultsMatchExplicitAuthor62Tuning)
        {
            Assert::AreEqual(1.2e-6f, kUkfTestImuYawRateVarianceRadps2, 1.0e-12f);
            Assert::AreEqual(
                kUkfTestImuYawRateVarianceRadps2,
                kUkfTestImuYawRateSigmaRadps * kUkfTestImuYawRateSigmaRadps,
                1.0e-12f);
            Assert::AreEqual(0.569900f, kUkfTestImuAccelSigmaMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, kUkfTestGyroBiasProcessVarianceMovingRadps2PerSample, 0.0f);
            Assert::AreEqual(3.0e-16f, kUkfTestGyroBiasProcessVarianceStationaryRadps2PerSample, 1.0e-20f);
            Assert::AreEqual(3.05e-4f, kUkfTestGyroBiasInitialVarianceUnseededRadps2, 1.0e-12f);
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseX)
        {
			SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < kStationaryPoseDriftToleranceM);


			// If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
			const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(VehicleState::kPx, VehicleState::kPx) < 10.0f, L"Final x position variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseY)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) < kStationaryPoseDriftToleranceM);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPy, VehicleState::kPy)));
            Assert::IsTrue(covariance(VehicleState::kPy, VehicleState::kPy) < 100.0f, L"Final y position variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsForwardVelocity)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) < 0.0001f, L"Final forward velocity variance was too high");
        }

        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsLateralVelocity)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 1.0e-5f,
                (L"Final lateral velocity was too high:\n" +
                    std::to_wstring((state(VehicleState::kV)))).c_str());


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) < 0.0001f,
                (L"Final lateral velocity variance was too high:\n" +
                std::to_wstring((covariance(VehicleState::kV,VehicleState::kV)))).c_str());
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsYawRate)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 1.0e-4f,
                (L"Final yaw rate was too high:\n" +
                    std::to_wstring((state(VehicleState::kR)))).c_str());


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) <= kStationaryYawVarianceToleranceRadps2,
                (L"Final yaw rate variance was too high:\n" +
                    std::to_wstring((covariance(VehicleState::kR, VehicleState::kR)))).c_str());
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsOmegaL)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) < 0.0001f, L"Final left wheel speed variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsOmegaR)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.workingState();
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) < 0.0001f, L"Final left wheel speed variance was too high");
        }
        TEST_METHOD(SrUkfCoreRepeatedZeroEncoderUpdatesDriveYawRateVarianceExtremelyLow)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 0.005f, 1.0f, 0.05f, 0.02f);

            SrUkfCore core = MakeDefaultSrUkfCore();
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);
            }

            const VehicleState::StateMatrix covariance = core.workingCovariance();
            const float finalYawRateVarianceRadps2 = covariance(VehicleState::kR, VehicleState::kR);
            const PlantParams params = PlantParams::Default();
            SrUkfCoreTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            const float stationaryEncoderOmegaSigmaRadps =
                plantModel.stationaryEncoderOmegaSigmaRadps(kUkfTestStationaryEncoderVelocitySigmaMps);
            const float stationaryYawRateSigmaRadps =
                std::sqrt(2.0f) * params.wheelRadiusM * stationaryEncoderOmegaSigmaRadps / params.trackWidthM;
            const float stationaryYawMeasurementVarianceRadps2 =
                stationaryYawRateSigmaRadps * stationaryYawRateSigmaRadps;
            const float acceptableYawRateVarianceRadps2 =
                (std::max)(1.01f * stationaryYawMeasurementVarianceRadps2, 1.0e-8f);
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kR, VehicleState::kR)));
            Assert::IsTrue(
                finalYawRateVarianceRadps2 <= acceptableYawRateVarianceRadps2,
                (std::wstring(L"Final yaw-rate variance was ") + std::to_wstring(finalYawRateVarianceRadps2)).c_str());
        }

        TEST_METHOD(SrUkfCoreRepeatedZeroEncoderUpdatesKeepLateralVelocityVarianceBounded)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.001f, 0.01f, 0.005f, 1.0f, 1.0f, 0.05f, 0.02f);

            SrUkfCore core = MakeDefaultSrUkfCore();
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const float initialLateralVelocityVarianceMps2 = core.workingCovariance()(VehicleState::kV, VehicleState::kV);
            Assert::AreEqual(1.0f, initialLateralVelocityVarianceMps2, 1.0e-6f);

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);
            }

            const VehicleState::StateMatrix covariance = core.workingCovariance();
            const float finalLateralVelocityVarianceMps2 = covariance(VehicleState::kV, VehicleState::kV);
            Assert::IsTrue(std::isfinite(finalLateralVelocityVarianceMps2));
            Assert::IsTrue(
                finalLateralVelocityVarianceMps2 < initialLateralVelocityVarianceMps2,
                (std::wstring(L"Final lateral-velocity variance was ") +
                    std::to_wstring(finalLateralVelocityVarianceMps2)).c_str());
        }
        TEST_METHOD(SrUkfCoreRepeatedStationaryCyclesKeepMotionZeroAndReseedInitialGyroBiasFromStationaryWindow)
        {
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.18f,
                    0.27f,
                    0.11f,
                    0.35f,
                    -0.22f,
                    0.18f,
                    8.0f,
                    -7.5f,
                    0.04f);
            const VehicleState::StateMatrix initialCovariance =
                BuildUkfCovariance(0.02f, 0.04f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f);

            SrUkfCore core = MakeDefaultSrUkfCore();
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;
            VehicleState::StateMatrix firstCertifiedCovariance = VehicleState::StateMatrix::Zero();
            bool capturedFirstCertifiedCovariance = false;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control, 0.80f, PlantParams::Default().supplyVoltageV));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
                if (!capturedFirstCertifiedCovariance &&
                    (FindDebugDumpModeId(core) == 0))
                {
                    firstCertifiedCovariance = core.workingCovariance();
                    capturedFirstCertifiedCovariance = true;
                }
            }

            const VehicleState::StateVector& state = core.workingState();
            const VehicleState::StateMatrix covariance = core.workingCovariance();

            Assert::IsTrue(capturedFirstCertifiedCovariance);
            Assert::AreEqual(0, FindDebugDumpModeId(core));
            Assert::AreEqual(initialState(VehicleState::kPx), state(VehicleState::kPx), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(VehicleState::kPy), state(VehicleState::kPy), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(VehicleState::kPsi), state(VehicleState::kPsi), 1.0e-4f);
            Assert::AreEqual(0.0f, state(VehicleState::kU), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(VehicleState::kV), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(VehicleState::kR), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(VehicleState::kBgz), kStationaryBiasWalkTolerance);
            Assert::IsTrue(FindDebugDumpBool(core, "ukf_dump_mode", "bias_update_enabled"));

            Assert::IsTrue(std::isfinite(covariance(VehicleState::kU, VehicleState::kU)));
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) < 1.0e-4f);
            Assert::IsTrue(
                covariance(VehicleState::kV, VehicleState::kV) < (0.005f * 0.005f),
                (std::wstring(L"Repeated stationary lateral variance was ") +
                    std::to_wstring(covariance(VehicleState::kV, VehicleState::kV))).c_str());
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) >= (0.010f * 0.010f));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR)));
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) < 1.0e-4f);
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kBgz, VehicleState::kBgz)));
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) > 0.0f);

            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPx, VehicleState::kPx)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPy, VehicleState::kPy)));
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPsi, VehicleState::kPsi)));
            Assert::IsTrue(
                covariance(VehicleState::kPx, VehicleState::kPx) <=
                (firstCertifiedCovariance(VehicleState::kPx, VehicleState::kPx) + 1.0e-9f));
            Assert::IsTrue(
                covariance(VehicleState::kPy, VehicleState::kPy) <=
                (firstCertifiedCovariance(VehicleState::kPy, VehicleState::kPy) + 1.0e-9f));
            Assert::IsTrue(
                covariance(VehicleState::kPsi, VehicleState::kPsi) <=
                (firstCertifiedCovariance(VehicleState::kPsi, VehicleState::kPsi) + 1.0e-9f));
        }

        TEST_METHOD(SrUkfCoreStationaryReleaseInflatesMotionAndWheelCovariance)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core = MakeDefaultSrUkfCore();
            App::Internal::CommandVector stationaryControl{};
            EncoderObs stationaryEncoder{};
            constexpr float dt = 0.002f;
            const int stationarySteps =
                static_cast<int>(std::ceil(kUkfTestStationaryCertificationDwellS / dt)) + 10;

            for (int step = 0; step < stationarySteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, stationaryControl));
                Assert::IsTrue(core.updateEncoderPair(stationaryEncoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(0.0f).accepted);
            }

            Assert::IsTrue(FindDebugDumpModeId(core) != 3);
            const VehicleState::StateMatrix stationaryCovariance = core.workingCovariance();

            App::Internal::CommandVector launchControl{};
            launchControl.SetLeftMotorPwm(0.30f);
            launchControl.SetRightMotorPwm(0.30f);
            const float launchControlFanDutyCycle = 0.80f;
            const float launchControlBatteryVoltageV = params.supplyVoltageV;
            Assert::IsTrue(core.predict(dt, launchControl, launchControlFanDutyCycle, launchControlBatteryVoltageV));

            EncoderObs launchEncoder{};
            launchEncoder.totalLeftCounts = 2;
            launchEncoder.totalRightCounts = 2;
            launchEncoder.omegaLeftRadps = 8.0f;
            launchEncoder.omegaRightRadps = 8.0f;
            Assert::IsTrue(core.updateEncoderPair(launchEncoder, dt).accepted);

            const VehicleState::StateMatrix releasedCovariance = core.workingCovariance();
            Assert::IsTrue(FindDebugDumpBool(core, "ukf_dump_filter_diagnostics", "release_inflation_applied"));
            Assert::IsTrue(FindDebugDumpModeId(core) != 0);
            Assert::IsTrue(
                releasedCovariance(VehicleState::kU, VehicleState::kU) >
                stationaryCovariance(VehicleState::kU, VehicleState::kU));
            Assert::IsTrue(
                releasedCovariance(VehicleState::kV, VehicleState::kV) >
                stationaryCovariance(VehicleState::kV, VehicleState::kV));
            Assert::IsTrue(
                releasedCovariance(VehicleState::kR, VehicleState::kR) >
                stationaryCovariance(VehicleState::kR, VehicleState::kR));
            Assert::IsTrue(
                releasedCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) >
                stationaryCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL));
            Assert::IsTrue(
                releasedCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) >
                stationaryCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR));
        }

    };
}








