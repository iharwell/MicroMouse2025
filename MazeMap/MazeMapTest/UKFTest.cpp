#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"

#include "..\MazeMap\MouseUkf.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    TEST_CLASS(UKFTest)
    {
    public:
        using FilterState = Eigen::Matrix<float, 4, 1>;
        using FilterCovariance = Eigen::Matrix<float, 4, 4>;
        using FilterControl = Eigen::Matrix<float, 2, 1>;

        TEST_METHOD(SquareRootUkfMaintainsPositiveDefiniteCovariance)
        {
            UKF<4, 2, 2> filter;

            FilterState initialState;
            initialState << 0.0f, 0.0f, 0.2f, -0.1f;
            FilterCovariance initialCovariance = FilterCovariance::Identity() * 0.15f;
            filter.setState(initialState, initialCovariance);
            filter.setProcessNoise(FilterCovariance::Identity() * 1.0e-4f);

            const Eigen::Matrix<float, 2, 2> sqrtMeasurementNoise =
                Eigen::Matrix<float, 2, 2>::Identity() * 0.05f;

            FilterState truth = initialState;
            FilterControl control;
            control << 0.15f, -0.05f;

            constexpr float dt = 0.001f;
            for (int step = 0; step < 120; ++step)
            {
                truth(0) += dt * truth(2);
                truth(1) += dt * truth(3);
                truth(2) += dt * control(0);
                truth(3) += dt * control(1);

                const bool predictOk = filter.Predict(
                    dt,
                    control,
                    [](const FilterState& sigmaPoint, const FilterControl& sigmaControl, float sigmaDt) noexcept
                    {
                        FilterState predicted = sigmaPoint;
                        predicted(0) += sigmaDt * sigmaPoint(2);
                        predicted(1) += sigmaDt * sigmaPoint(3);
                        predicted(2) += sigmaDt * sigmaControl(0);
                        predicted(3) += sigmaDt * sigmaControl(1);
                        return predicted;
                    });
                Assert::IsTrue(predictOk);

                Eigen::Matrix<float, 2, 1> measurement;
                measurement << truth(0), truth(1);
                const bool updateOk = filter.Update<2>(
                    measurement,
                    sqrtMeasurementNoise,
                    15.0f,
                    [](const FilterState& sigmaPoint) noexcept
                    {
                        Eigen::Matrix<float, 2, 1> projection;
                        projection << sigmaPoint(0), sigmaPoint(1);
                        return projection;
                    });
                Assert::IsTrue(updateOk);

                const FilterCovariance covariance = filter.covariance();
                const Eigen::LLT<FilterCovariance> llt(covariance);
                Assert::IsTrue(llt.info() == Eigen::Success);
                Assert::IsTrue(std::isfinite(covariance.trace()));
                Assert::IsTrue(std::isfinite(filter.state().sum()));
            }
        }

        TEST_METHOD(PlantModelSymmetricDriveDoesNotCreateYawBias)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.5f;
            state(VehicleState::kOmegaL) = state(VehicleState::kU) / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = state(VehicleState::kU) / params.wheelRadiusM;

            ControlInput control;
            control.leftMotorCommand = 0.55f;
            control.rightMotorCommand = 0.55f;
            control.fanDutyCycle = 0.80f;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kR)));
            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kR), 1.0e-4f);
        }

        TEST_METHOD(PlantModelUsesTransientSlipRatherThanInstantaneousTarget)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kOmegaL) = state(VehicleState::kU) / params.wheelRadiusM;
            state(VehicleState::kOmegaR) = state(VehicleState::kU) / params.wheelRadiusM;

            ControlInput control;
            control.leftMotorCommand = 0.85f;
            control.rightMotorCommand = 0.85f;
            control.fanDutyCycle = 0.80f;

            const VehicleState::StateVector propagated =
                plant.integrateMidpoint(state, control, 0.0005f, params);
            const SlipTargets propagatedTargets = plant.slipTargets(propagated, params);

            Assert::IsTrue(std::isfinite(propagated(VehicleState::kKappaL)));
            Assert::IsTrue(std::isfinite(propagatedTargets.kappaLeft));
            Assert::IsTrue(std::fabs(propagated(VehicleState::kKappaL)) > 0.0f);
            Assert::IsTrue(std::fabs(propagated(VehicleState::kKappaL)) < std::fabs(propagatedTargets.kappaLeft));
            Assert::IsTrue(std::fabs(propagated(VehicleState::kKappaR)) < std::fabs(propagatedTargets.kappaRight));
        }

        TEST_METHOD(ComputeEncoderPairSqrtNoise_UsesGeneralSigmaMappingForNonZeroReadings)
        {
            const PlantParams params = PlantParams::Default();
            EncoderObs observation{};
            observation.omegaLeftRadps = 1.0f;
            observation.omegaRightRadps = 1.0f;

            const Eigen::Matrix<float, 2, 2> sqrtNoise = ComputeEncoderPairSqrtNoise(observation, params);
            const Eigen::Matrix<float, 2, 2> covariance = sqrtNoise * sqrtNoise.transpose();
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            const float varianceUMps2 = 0.0018f * 0.0018f;
            const float varianceYawRateRadps2 = 0.051f * 0.051f;
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
            const float expectedSigmaRadps = 1.76e-6f / params.wheelRadiusM;
            Assert::AreEqual(expectedSigmaRadps, ComputeStationaryEncoderOmegaSigmaRadps(params), 1.0e-9f);
        }

        TEST_METHOD(ConfiguredGeneralImuSigmasMatchInitialEstimates)
        {
            Assert::AreEqual(0.0013f, kImuYawRateSigmaRadps, 1.0e-9f);
            Assert::AreEqual(0.014f, kImuAccelSigmaMps2, 1.0e-9f);
        }

        TEST_METHOD(SrUkfCoreRejectsInvalidMergedImuUpdate)
        {
            SrUkfCore core;
            ImuMergedObs observation{};
            observation.valid = false;
            observation.yawRateRadps = 0.5f;
            observation.accelXComMps2 = 1.0f;
            observation.accelYComMps2 = 0.1f;

            const MeasurementUpdateResult result = core.updateImuMerged(observation);
            Assert::IsFalse(result.attempted);
            Assert::IsFalse(result.accepted);
        }

        TEST_METHOD(SquareRootUkfSymmetrizesAsymmetricCovarianceInputs)
        {
            UKF<4, 2, 2> filter;

            FilterState initialState;
            initialState << 0.2f, -0.1f, 0.4f, -0.3f;
            FilterCovariance initialCovariance = FilterCovariance::Identity() * 0.1f;
            initialCovariance(0, 1) = 0.03f;
            initialCovariance(1, 0) = -0.01f;
            initialCovariance(2, 3) = 0.02f;
            initialCovariance(3, 2) = -0.015f;
            filter.setState(initialState, initialCovariance);

            FilterCovariance processNoise = FilterCovariance::Zero();
            processNoise(0, 0) = 1.0e-4f;
            processNoise(1, 1) = 2.0e-4f;
            processNoise(2, 2) = 3.0e-4f;
            processNoise(3, 3) = 4.0e-4f;
            processNoise(0, 2) = 5.0e-5f;
            processNoise(2, 0) = -2.0e-5f;
            filter.setProcessNoise(processNoise);

            FilterControl control;
            control << 0.1f, -0.08f;

            const bool predictOk = filter.Predict(
                0.002f,
                control,
                [](const FilterState& sigmaPoint, const FilterControl& sigmaControl, float sigmaDt) noexcept
                {
                    FilterState predicted = sigmaPoint;
                    predicted(0) += sigmaDt * sigmaPoint(2);
                    predicted(1) += sigmaDt * sigmaPoint(3);
                    predicted(2) += sigmaDt * sigmaControl(0);
                    predicted(3) += sigmaDt * sigmaControl(1);
                    return predicted;
                });

            Assert::IsTrue(predictOk);
            const FilterCovariance covariance = filter.covariance();
            const FilterCovariance asymmetry = covariance - covariance.transpose();
            Assert::IsTrue(asymmetry.cwiseAbs().maxCoeff() <= 1.0e-6f);

            const Eigen::LLT<FilterCovariance> llt(covariance);
            Assert::IsTrue(llt.info() == Eigen::Success);
        }

        TEST_METHOD(SrUkfCoreDoesNotDriftUnderRepeatedZeroMotionMeasurements)
        {
            SrUkfCore core;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;

            for (int step = 0; step < 2000; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(encoderResult.attempted);
                Assert::IsTrue(encoderResult.accepted);

                const MeasurementUpdateResult yawResult = core.updateYawRate(0.0f);
                Assert::IsTrue(yawResult.attempted);
                Assert::IsTrue(yawResult.accepted);
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);
        }
    };
}
