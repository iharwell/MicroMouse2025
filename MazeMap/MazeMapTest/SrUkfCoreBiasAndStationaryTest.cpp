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
        struct ScopedUkfRuntimeTuningRestore
        {
            SrUkfCore::RuntimeTuning saved = SrUkfCore::GetRuntimeTuning();

            ~ScopedUkfRuntimeTuningRestore()
            {
                SrUkfCore::SetRuntimeTuning(saved);
            }
        };
    }

    TEST_CLASS(SrUkfCoreBiasAndStationaryTest)
    {
    public:
        TEST_METHOD(SrUkfCoreStationaryYawConstraintCollapsesMotionStateOncePostSwapStationaryDwellCompletes)
        {
            SrUkfCore core;
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

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.002f;
            constexpr float rawStationaryGyroRadps = 0.04f;
            const int kSteps =
                static_cast<int>(std::ceil(SrUkfCore::GetRuntimeTuning().stationaryCertificationDwellS / dt)) + 10;
            for (int step = 0; step < kSteps; ++step)
            {
                core.setRuntimeContext(0.0f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps).accepted);
            }

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::AreEqual(
                static_cast<int>(SrUkfCore::OperatingMode::StationaryCertified),
                static_cast<int>(core.operatingMode()));
            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kBgz), 1.0e-6f);
            Assert::IsFalse(core.biasUpdateEnabled());
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
        TEST_METHOD(SrUkfCoreInitialStationaryGyroBiasSeedsFromSamplesFiftyToOneHundredFiftyAndSlowWalks)
        {
            SrUkfCore core;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            const int kSteps =
                (std::max)(
                    200,
                    static_cast<int>(std::ceil(SrUkfCore::GetRuntimeTuning().stationaryCertificationDwellS / dt)) + 20);
            InitialStationaryGyroBiasExpectation expected{};

            for (int step = 0; step < kSteps; ++step)
            {
                const float rawStationaryGyroRadps =
                    (step < 49) ? 0.01f :
                    ((step < 150) ? 0.04f : 0.07f);
                core.setRuntimeContext(0.0f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps).accepted);
                AdvanceInitialStationaryGyroBiasExpectation(expected, rawStationaryGyroRadps, dt);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(expected.seedApplied);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::IsTrue(state(VehicleState::kBgz) > 0.04f);
            Assert::AreEqual(expected.biasRadps, state(VehicleState::kBgz), 1.0e-6f);
            Assert::IsTrue(core.biasUpdateEnabled());
        }

        TEST_METHOD(SrUkfCoreStationaryGyroMeasurementDoesNotCollapseBiasVarianceToZero)
        {
            SrUkfCore core;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            constexpr float rawStationaryGyroRadps = 0.04f;

            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            Assert::IsTrue(
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz) >
                0.0f);

            for (int step = 0; step < 200; ++step)
            {
                core.setRuntimeContext(0.0f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt).accepted);
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps).accepted);
            }

            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kBgz, VehicleState::kBgz)));
            Assert::IsTrue(covariance(VehicleState::kBgz, VehicleState::kBgz) > 0.0f);
            Assert::IsTrue(
                covariance(VehicleState::kBgz, VehicleState::kBgz) <
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz));
        }

        TEST_METHOD(SrUkfCoreStationaryGyroBiasWalkProcessVarianceMatchesThirtySecondTimeConstant)
        {
            constexpr float dt = 0.001f;
            constexpr float measurementVarianceRadps2 = 1.0f;
            const float processVarianceRadps2 =
                SrUkfCore::ComputeStationaryGyroBiasWalkProcessVarianceRadps2(
                    dt,
                    measurementVarianceRadps2);
            const float posteriorVarianceRadps2 =
                SrUkfCore::ComputeStationaryGyroBiasWalkPosteriorVarianceRadps2(
                    dt,
                    measurementVarianceRadps2);

            Assert::AreEqual(1.1111111e-9f, processVarianceRadps2, 1.0e-12f);
            Assert::AreEqual(
                StationaryGyroBiasBlendFactor(dt),
                posteriorVarianceRadps2,
                1.0e-9f);
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseX)
        {
			SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 1.0e-4f);


			// If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
			const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kPx, VehicleState::kPx) < 10.0f, L"Final x position variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseY)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kPy, VehicleState::kPy) < 10.0f, L"Final y position variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsForwardVelocity)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) < 0.0001f, L"Final forward velocity variance was too high");
        }

        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsLateralVelocity)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 1.0e-5f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kV, VehicleState::kV) < 0.0001f, (L"Final lateral velocity variance was too high:\n" +
                std::to_wstring((covariance(VehicleState::kV,VehicleState::kV)))).c_str());
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsYawRate)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kR, VehicleState::kR) <= 0.0001f, L"Final yaw rate variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsOmegaL)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) < 0.0001f, L"Final left wheel speed variance was too high");
        }
        TEST_METHOD(SrUkfCoreDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsOmegaR)
        {
            SrUkfCore core = RunUKFCycles(2000);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and wheel speeds are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.covariance();
            Assert::IsTrue(covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) < 0.0001f, L"Final left wheel speed variance was too high");
        }
        TEST_METHOD(SrUkfCoreRepeatedZeroEncoderUpdatesDriveYawRateVarianceExtremelyLow)
        {
            const ScopedUkfRuntimeTuningRestore restore{};
            SrUkfCore::ResetRuntimeTuning();
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

            SrUkfCore core;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);
            }

            const VehicleState::StateMatrix covariance = core.covariance();
            const float finalYawRateVarianceRadps2 = covariance(VehicleState::kR, VehicleState::kR);
            const PlantParams params = PlantParams::Default();
            const float stationaryEncoderOmegaSigmaRadps =
                SrUkfCore::ComputeStationaryEncoderOmegaSigmaRadps(params);
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

            SrUkfCore core;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const float initialLateralVelocityVarianceMps2 = core.covariance()(VehicleState::kV, VehicleState::kV);
            Assert::AreEqual(1.0f, initialLateralVelocityVarianceMps2, 1.0e-6f);

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);
            }

            const VehicleState::StateMatrix covariance = core.covariance();
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

            SrUkfCore core;
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;
            VehicleState::StateMatrix firstCertifiedCovariance = VehicleState::StateMatrix::Zero();
            bool capturedFirstCertifiedCovariance = false;

            for (int step = 0; step < kSteps; ++step)
            {
                core.setRuntimeContext(0.0f, 0.0f, 0U, 0.0f, 0.0f, true, 0.0f, 0.0f);
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
                if (!capturedFirstCertifiedCovariance &&
                    (core.operatingMode() == SrUkfCore::OperatingMode::StationaryCertified))
                {
                    firstCertifiedCovariance = core.covariance();
                    capturedFirstCertifiedCovariance = true;
                }
            }

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();

            Assert::IsTrue(capturedFirstCertifiedCovariance);
            Assert::AreEqual(
                static_cast<int>(SrUkfCore::OperatingMode::StationaryCertified),
                static_cast<int>(core.operatingMode()));
            Assert::AreEqual(initialState(VehicleState::kPx), state(VehicleState::kPx), 1.0e-5f);
            Assert::AreEqual(initialState(VehicleState::kPy), state(VehicleState::kPy), 1.0e-5f);
            Assert::AreEqual(initialState(VehicleState::kPsi), state(VehicleState::kPsi), 1.0e-5f);
            Assert::AreEqual(0.0f, state(VehicleState::kU), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kV), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kOmegaR), 1.0e-6f);
            Assert::AreEqual(0.0f, state(VehicleState::kBgz), 1.0e-6f);
            Assert::IsTrue(core.biasUpdateEnabled());

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
    };
}
