#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\Estimator.h"

#include "EstimatorFilterTestSupport.h"

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

    TEST_CLASS(EstimatorBiasAndStationaryTest)
    {
    public:
        TEST_METHOD(EstimatorStationaryYawConstraintCollapsesMotionStateWithoutSnappingBiasBackToAnchor)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
            initialState(3) = 0.0f;
            initialState(4) = 0.35f;
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
            EncoderObs encoder{};
            constexpr float dt = 0.002f;
            constexpr float rawStationaryGyroRadps = 0.04f;
            const int kSteps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt, true));
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps));
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            Assert::IsTrue(std::fabs(state(3)) < kStationaryMotionTolerance);
            Assert::IsTrue(std::fabs(state(4)) < kStationaryMotionTolerance);
            Assert::IsTrue(std::fabs(state(5)) < kStationaryMotionTolerance);
            Assert::IsTrue(std::fabs(state(6)) < kStationaryMotionTolerance);
            Assert::IsTrue(std::fabs(state(7)) < kStationaryMotionTolerance);
            Assert::IsTrue(std::fabs(state(8)) < kStationaryMotionTolerance);
            const float gyroBiasRadps = runtime.runtimeState.GetGyroBiasZ();
            Assert::IsTrue(std::fabs(gyroBiasRadps) > 1.0e-3f);
            Assert::IsTrue(
                std::fabs((state(5) + gyroBiasRadps) - rawStationaryGyroRadps) <
                rawStationaryGyroRadps);
            Assert::IsTrue(std::isfinite(covariance(3, 3)));
            Assert::IsTrue(covariance(3, 3) < 1.0e-4f);
            Assert::IsTrue(
                covariance(4, 4) < (0.005f * 0.005f),
                (std::wstring(L"Stationary lateral variance was ") +
                    std::to_wstring(covariance(4, 4))).c_str());
            Assert::IsTrue(covariance(5, 5) >= (0.010f * 0.010f));
            Assert::IsTrue(std::isfinite(covariance(6, 6)));
            Assert::IsTrue(std::isfinite(covariance(7, 7)));
            Assert::IsTrue(covariance(6, 6) < 1.0e-4f);
            Assert::IsTrue(covariance(7, 7) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(covariance(8, 8)));
            Assert::IsTrue(covariance(8, 8) > 0.0f);
            Assert::IsTrue(covariance(8, 8) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(runtime.runtimeState.GetGyroBiasZVar()));
            Assert::IsTrue(runtime.runtimeState.GetGyroBiasZVar() > 0.0f);
        }

        TEST_METHOD(EstimatorExactStationaryLockKeepsPoseFixedWhenEncoderCountsRemainNonZero)
        {
            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.42f;
            initialState(1) = -0.17f;
            initialState(2) = NormalizeAngle(0.28f);
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

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            encoder.SetTotalLeftCounts(12);
            encoder.SetTotalRightCounts(8);
            constexpr float dt = 0.002f;
            const int kSteps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt, true));
                Assert::IsTrue(core.updateYawRate(0.0f));
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::AreEqual(initialState(0), state(0), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(1), state(1), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(2), state(2), 1.0e-4f);
        }

        TEST_METHOD(EstimatorInitialStationaryGyroBiasSeedsFromSamplesFiftyToOneHundredFiftyAndSlowWalks)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            const int kSteps =
                (std::max)(
                    200,
                    static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 20);
            InitialStationaryGyroBiasExpectation expected{};

            for (int step = 0; step < kSteps; ++step)
            {
                const float rawStationaryGyroRadps =
                    (step < 49) ? 0.01f :
                    ((step < 150) ? 0.04f : 0.07f);
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt, true));
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps));
                AdvanceInitialStationaryGyroBiasExpectation(expected, rawStationaryGyroRadps, dt);
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const float gyroBiasRadps = runtime.runtimeState.GetGyroBiasZ();
            Assert::IsTrue(expected.seedApplied);
            Assert::IsTrue(std::fabs(state(5)) < kStationaryBiasSeedYawRateTolerance);
            Assert::IsTrue(gyroBiasRadps > 0.04f);
            Assert::IsTrue(gyroBiasRadps < 0.08f);
            Assert::IsTrue(std::isfinite(runtime.runtimeState.GetGyroBiasZVar()));
        }

        TEST_METHOD(EstimatorStationaryGyroMeasurementDoesNotCollapseBiasVarianceToZero)
        {
            EstimatorTestRuntime runtime;
            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;
            constexpr float rawStationaryGyroRadps = 0.04f;

            const float beforeBiasVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();
            Assert::IsTrue(
                beforeBiasVarianceRadps2 >
                0.0f);

            for (int step = 0; step < 200; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt, true));
                Assert::IsTrue(core.updateYawRate(rawStationaryGyroRadps));
            }

            const float afterBiasVarianceRadps2 = runtime.runtimeState.GetGyroBiasZVar();
            Assert::IsTrue(std::isfinite(afterBiasVarianceRadps2));
            Assert::IsTrue(afterBiasVarianceRadps2 > 0.0f);
            Assert::IsTrue(
                afterBiasVarianceRadps2 <
                beforeBiasVarianceRadps2);
        }

        TEST_METHOD(EstimatorExactStationaryLockKeepsPoseReferenceAndCollapsesStationaryStates)
        {
            Estimator core = MakeDefaultEstimator();
            Eigen::Matrix<float, VehicleState::kDimension, 1> poseReferenceState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            poseReferenceState(0) = 1.20f;
            poseReferenceState(1) = 0.70f;
            poseReferenceState(2) = NormalizeAngle(-0.20f);
            poseReferenceState(3) = 0.020f;
            poseReferenceState(4) = -0.015f;
            poseReferenceState(5) = 0.040f;
            poseReferenceState(6) = 0.60f;
            poseReferenceState(7) = -0.50f;
            poseReferenceState(8) = 0.0f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> poseReferenceCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            poseReferenceCovariance(0, 0) = 0.012f * 0.012f;
            poseReferenceCovariance(1, 1) = 0.012f * 0.012f;
            poseReferenceCovariance(2, 2) = 0.02f * 0.02f;
            poseReferenceCovariance(3, 3) = 0.15f * 0.15f;
            poseReferenceCovariance(4, 4) = 0.11f * 0.11f;
            poseReferenceCovariance(5, 5) = 0.09f * 0.09f;
            poseReferenceCovariance(6, 6) = 0.40f * 0.40f;
            poseReferenceCovariance(7, 7) = 0.40f * 0.40f;
            poseReferenceCovariance(8, 8) = 0.03f * 0.03f;
            poseReferenceCovariance(0, 1) = 2.5e-5f;
            poseReferenceCovariance(1, 0) = 2.5e-5f;
            poseReferenceCovariance(0, 2) = -1.5e-5f;
            poseReferenceCovariance(2, 0) = -1.5e-5f;
            poseReferenceCovariance(1, 2) = 1.2e-5f;
            poseReferenceCovariance(2, 1) = 1.2e-5f;
            Assert::IsTrue(core.reset(poseReferenceState, poseReferenceCovariance));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.002f;
            const int kSteps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> firstCertifiedCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            bool capturedFirstCertifiedCovariance = false;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
                Assert::IsTrue(core.updateEncoderPair(encoder, dt, true));
                Assert::IsTrue(core.updateYawRate(0.0f));
                if (!capturedFirstCertifiedCovariance &&
                    (step >= static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt))))
                {
                    firstCertifiedCovariance = core.workingCovariance();
                    capturedFirstCertifiedCovariance = true;
                }
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> constrainedCovariance = core.workingCovariance();

            Assert::AreEqual(poseReferenceState(0), state(0), 1.0e-6f);
            Assert::AreEqual(poseReferenceState(1), state(1), 1.0e-6f);
            Assert::AreEqual(poseReferenceState(2), state(2), 1.0e-6f);
            Assert::AreEqual(0.0f, state(3), 1.0e-7f);
            Assert::AreEqual(0.0f, state(4), 1.0e-7f);
            Assert::AreEqual(0.0f, state(5), 1.0e-7f);
            Assert::AreEqual(0.0f, state(6), 1.0e-7f);
            Assert::AreEqual(0.0f, state(7), 1.0e-7f);
            Assert::AreEqual(poseReferenceState(8), state(8), 1.0e-7f);

            Assert::IsTrue(capturedFirstCertifiedCovariance);
            for (int row = 0; row <= 2; ++row)
            {
                for (int col = 0; col <= 2; ++col)
                {
                    Assert::IsTrue(std::isfinite(constrainedCovariance(row, col)));
                }
                Assert::IsTrue(
                    constrainedCovariance(row, row) <=
                    (firstCertifiedCovariance(row, row) + 1.0e-9f));
            }

            Assert::IsTrue(constrainedCovariance(3, 3) < 1.0e-4f);
            Assert::IsTrue(
                constrainedCovariance(4, 4) < 1.0e-4f,
                (std::wstring(L"Stationary lateral variance was ") +
                    std::to_wstring(constrainedCovariance(4, 4))).c_str());
            Assert::IsTrue(constrainedCovariance(5, 5) >= (0.010f * 0.010f));
            Assert::IsTrue(constrainedCovariance(6, 6) < 1.0e-4f);
            Assert::IsTrue(constrainedCovariance(7, 7) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(constrainedCovariance(8, 8)));
            Assert::IsTrue(constrainedCovariance(8, 8) > 0.0f);
        }

        TEST_METHOD(EstimatorGyroBiasDefaultsMatchExplicitAuthor62Tuning)
        {
            Assert::AreEqual(1.2e-6f, kEstimatorTestImuYawRateVarianceRadps2, 1.0e-12f);
            Assert::AreEqual(
                kEstimatorTestImuYawRateVarianceRadps2,
                kEstimatorTestImuYawRateSigmaRadps * kEstimatorTestImuYawRateSigmaRadps,
                1.0e-12f);
            Assert::AreEqual(0.569900f, kEstimatorTestImuAccelSigmaMps2, 1.0e-6f);
            Assert::AreEqual(0.0f, kEstimatorTestGyroBiasProcessVarianceMovingRadps2PerSample, 0.0f);
            Assert::AreEqual(3.0e-16f, kEstimatorTestGyroBiasProcessVarianceStationaryRadps2PerSample, 1.0e-20f);
            Assert::AreEqual(3.05e-4f, kEstimatorTestGyroBiasInitialVarianceUnseededRadps2, 1.0e-12f);
        }
        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseX)
        {
			Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(0)) < kStationaryPoseDriftToleranceM);


			// If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
			const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(0, 0) < 10.0f, L"Final x position variance was too high");
        }
        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsPoseY)
        {
            Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(1)) < kStationaryPoseDriftToleranceM);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(std::isfinite(covariance(1, 1)));
            Assert::IsTrue(covariance(1, 1) < 100.0f, L"Final y position variance was too high");
        }
        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsForwardVelocity)
        {
            Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(3)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(3, 3) < 0.0001f, L"Final forward velocity variance was too high");
        }

        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsLateralVelocity)
        {
            Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(4)) < 1.0e-5f,
                (L"Final lateral velocity was too high:\n" +
                    std::to_wstring((state(4)))).c_str());


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(4, 4) < 0.0001f,
                (L"Final lateral velocity variance was too high:\n" +
                std::to_wstring((covariance(4,4)))).c_str());
        }
        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsYawRate)
        {
            Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(5)) < 1.0e-4f,
                (L"Final yaw rate was too high:\n" +
                    std::to_wstring((state(5)))).c_str());


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(5, 5) <= kStationaryYawVarianceToleranceRadps2,
                (L"Final yaw rate variance was too high:\n" +
                    std::to_wstring((covariance(5, 5)))).c_str());
        }
        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsForwardAccelerationResidual)
        {
            Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(6)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(6, 6) < 0.0001f, L"Final forward acceleration residual variance was too high");
        }
        TEST_METHOD(EstimatorDoesNotDriftOrLoseCertaintyUnderRepeatedZeroMotionMeasurementsRightwardAccelerationResidual)
        {
            Estimator core = RunEstimatorCycles(2000);

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            Assert::IsTrue(std::fabs(state(7)) < 1.0e-4f);


            // If the robot is stationary, we grow increasingly sure that the velocity, yaw rate, and acceleration residuals are all near zero.
            // We should still have some uncertainty about the exact position and heading, but it shouldn't grow without bound.
            // The runtime gyro bias should be allowed to absorb the stationary measurements, as this is when it's most appropriate to update that value.
            const auto covariance = core.workingCovariance();
            Assert::IsTrue(covariance(7, 7) < 0.0001f, L"Final rightward acceleration residual variance was too high");
        }
        TEST_METHOD(EstimatorRepeatedZeroEncoderUpdatesDriveYawRateVarianceExtremelyLow)
        {
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
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

            Estimator core = MakeDefaultEstimator();
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(encoderAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
            }

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            const float finalYawRateVarianceRadps2 = covariance(5, 5);
            EstimatorTestRuntime runtime;
            const PlantModel& plantModel = runtime.plantModel;
            const float stationaryEncoderWheelSpeedSigmaRadps =
                plantModel.stationaryEncoderWheelSpeedSigmaRadps(kEstimatorTestStationaryEncoderVelocitySigmaMps);
            const float stationaryYawRateSigmaRadps =
                std::sqrt(2.0f) *
                Vehicle::WheelLinearVelocityFromWheelSpeed(stationaryEncoderWheelSpeedSigmaRadps) /
                Vehicle::GetPhysicalTrackWidthM();
            const float stationaryYawMeasurementVarianceRadps2 =
                stationaryYawRateSigmaRadps * stationaryYawRateSigmaRadps;
            const float acceptableYawRateVarianceRadps2 =
                (std::max)(1.01f * stationaryYawMeasurementVarianceRadps2, 1.0e-8f);
            Assert::IsTrue(std::isfinite(covariance(5, 5)));
            Assert::IsTrue(
                finalYawRateVarianceRadps2 <= acceptableYawRateVarianceRadps2,
                (std::wstring(L"Final yaw-rate variance was ") + std::to_wstring(finalYawRateVarianceRadps2)).c_str());
        }

        TEST_METHOD(EstimatorRepeatedZeroEncoderUpdatesKeepLateralVelocityVarianceBounded)
        {
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.0f;
            initialState(1) = 0.09f;
            initialState(2) = NormalizeAngle(0.0f);
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
            initialCovariance(4, 4) = 1.0f * 1.0f;
            initialCovariance(5, 5) = 1.0f * 1.0f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;

            Estimator core = MakeDefaultEstimator();
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            const float initialLateralVelocityVarianceMps2 = core.workingCovariance()(4, 4);
            Assert::AreEqual(1.0f, initialLateralVelocityVarianceMps2, 1.0e-6f);

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;

            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const bool encoderAccepted = core.updateEncoderPair(encoder, dt, true);
                Assert::IsTrue(core.LastUpdateAttempted());
                Assert::IsTrue(encoderAccepted);
                Assert::IsTrue(core.LastUpdateAccepted());
            }

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();
            const float finalLateralVelocityVarianceMps2 = covariance(4, 4);
            Assert::IsTrue(std::isfinite(finalLateralVelocityVarianceMps2));
            Assert::IsTrue(
                finalLateralVelocityVarianceMps2 < initialLateralVelocityVarianceMps2,
                (std::wstring(L"Final lateral-velocity variance was ") +
                    std::to_wstring(finalLateralVelocityVarianceMps2)).c_str());
        }
        TEST_METHOD(EstimatorRepeatedStationaryCyclesKeepMotionZeroAndReseedInitialGyroBiasFromStationaryWindow)
        {
            EstimatorTestRuntime runtime;
            runtime.runtimeState.SetGyroBiasZ(0.04f);
            Eigen::Matrix<float, VehicleState::kDimension, 1> initialState = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
            initialState(0) = 0.18f;
            initialState(1) = 0.27f;
            initialState(2) = NormalizeAngle(0.11f);
            initialState(3) = 0.35f;
            initialState(4) = -0.22f;
            initialState(5) = 0.18f;
            initialState(6) = 8.0f;
            initialState(7) = -7.5f;
            initialState(8) = 0.04f;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> initialCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.20f * 0.20f;
            initialCovariance(4, 4) = 0.15f * 0.15f;
            initialCovariance(5, 5) = 0.25f * 0.25f;
            initialCovariance(6, 6) = 0.50f * 0.50f;
            initialCovariance(7, 7) = 0.50f * 0.50f;
            initialCovariance(8, 8) = 0.05f * 0.05f;

            Estimator core(runtime.vehicle, runtime.plantModel, runtime.runtimeState);
            Assert::IsTrue(core.reset(initialState, initialCovariance));

            App::Internal::CommandVector control{};
            EncoderObs encoder{};
            constexpr float dt = 0.001f;
            constexpr int kSteps = 1000;
            Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> firstCertifiedCovariance = Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension>::Zero();
            bool capturedFirstCertifiedCovariance = false;

            for (int step = 0; step < kSteps; ++step)
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
                if (!capturedFirstCertifiedCovariance &&
                    (step >= static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt))))
                {
                    firstCertifiedCovariance = core.workingCovariance();
                    capturedFirstCertifiedCovariance = true;
                }
            }

            const Eigen::Matrix<float, VehicleState::kDimension, 1>& state = core.workingState();
            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> covariance = core.workingCovariance();

            Assert::IsTrue(capturedFirstCertifiedCovariance);
            Assert::AreEqual(initialState(0), state(0), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(1), state(1), kStationaryPoseDriftToleranceM);
            Assert::AreEqual(initialState(2), state(2), 1.0e-4f);
            Assert::AreEqual(0.0f, state(3), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(4), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(5), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(6), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(7), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, state(8), kStationaryMotionTolerance);
            Assert::AreEqual(0.0f, runtime.runtimeState.GetGyroBiasZ(), kStationaryBiasWalkTolerance);

            Assert::IsTrue(std::isfinite(covariance(3, 3)));
            Assert::IsTrue(covariance(3, 3) < 1.0e-4f);
            Assert::IsTrue(
                covariance(4, 4) < (0.005f * 0.005f),
                (std::wstring(L"Repeated stationary lateral variance was ") +
                    std::to_wstring(covariance(4, 4))).c_str());
            Assert::IsTrue(covariance(5, 5) >= (0.010f * 0.010f));
            Assert::IsTrue(std::isfinite(covariance(6, 6)));
            Assert::IsTrue(std::isfinite(covariance(7, 7)));
            Assert::IsTrue(covariance(6, 6) < 1.0e-4f);
            Assert::IsTrue(covariance(7, 7) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(covariance(8, 8)));
            Assert::IsTrue(covariance(8, 8) > 0.0f);
            Assert::IsTrue(covariance(8, 8) < 1.0e-4f);
            Assert::IsTrue(std::isfinite(runtime.runtimeState.GetGyroBiasZVar()));
            Assert::IsTrue(runtime.runtimeState.GetGyroBiasZVar() > 0.0f);

            Assert::IsTrue(std::isfinite(covariance(0, 0)));
            Assert::IsTrue(std::isfinite(covariance(1, 1)));
            Assert::IsTrue(std::isfinite(covariance(2, 2)));
            Assert::IsTrue(
                covariance(0, 0) <=
                (firstCertifiedCovariance(0, 0) + 1.0e-9f));
            Assert::IsTrue(
                covariance(1, 1) <=
                (firstCertifiedCovariance(1, 1) + 1.0e-9f));
            Assert::IsTrue(
                covariance(2, 2) <=
                (firstCertifiedCovariance(2, 2) + 1.0e-9f));
        }

        TEST_METHOD(EstimatorStationaryReleaseInflatesMotionAndResidualCovariance)
        {
            Estimator core = MakeDefaultEstimator();
            App::Internal::CommandVector stationaryControl{};
            EncoderObs stationaryEncoder{};
            constexpr float dt = 0.002f;
            const int stationarySteps =
                static_cast<int>(std::ceil(kEstimatorTestStationaryCertificationDwellS / dt)) + 10;

            for (int step = 0; step < stationarySteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, stationaryControl));
                Assert::IsTrue(core.updateEncoderPair(stationaryEncoder, dt, true));
                Assert::IsTrue(core.updateYawRate(0.0f));
            }

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> stationaryCovariance = core.workingCovariance();

            App::Internal::CommandVector launchControl{};
            launchControl.SetLeftCommand(0.30f);
            launchControl.SetRightCommand(0.30f);
            Assert::IsTrue(core.predict(dt, launchControl));

            EncoderObs launchEncoder{};
            launchEncoder.SetTotalLeftCounts(2);
            launchEncoder.SetTotalRightCounts(2);
            launchEncoder.SetLeftWheelSpeedRadps(8.0f);
            launchEncoder.SetRightWheelSpeedRadps(8.0f);
            Assert::IsTrue(core.updateEncoderPair(launchEncoder, dt, true));

            const Eigen::Matrix<float, VehicleState::kDimension, VehicleState::kDimension> releasedCovariance = core.workingCovariance();
            Assert::IsTrue(
                releasedCovariance(3, 3) >
                stationaryCovariance(3, 3));
            Assert::IsTrue(
                releasedCovariance(4, 4) >
                stationaryCovariance(4, 4));
            Assert::IsTrue(
                releasedCovariance(5, 5) >
                stationaryCovariance(5, 5));
            Assert::IsTrue(
                releasedCovariance(6, 6) >
                stationaryCovariance(6, 6));
            Assert::IsTrue(
                releasedCovariance(7, 7) >
                stationaryCovariance(7, 7));
        }

    };
}








