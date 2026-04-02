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
        using AngleState = Eigen::Matrix<float, 1, 1>;
        using AngleCovariance = Eigen::Matrix<float, 1, 1>;
        using AngleControl = Eigen::Matrix<float, 1, 1>;

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

        TEST_METHOD(SquareRootUkfUsesReferenceRelativeWrappedMeanForAngles)
        {
            UKF<1, 1, 1> filter;
            filter.setStateNormalizer([](AngleState& state) noexcept
                {
                    state(0) = VehicleState::NormalizeAngle(state(0));
                });

            AngleState initialState;
            initialState << (PI_F - 0.01f);
            AngleCovariance covariance = AngleCovariance::Zero();
            covariance(0, 0) = 0.04f;
            filter.setState(initialState, covariance);
            filter.setProcessNoise(AngleCovariance::Zero());

            AngleControl control = AngleControl::Zero();
            const bool predictOk = filter.Predict(
                0.001f,
                control,
                [](const AngleState& sigmaPoint, const AngleControl&, float) noexcept
                {
                    return sigmaPoint;
                });

            Assert::IsTrue(predictOk);
            const float wrappedError =
                VehicleState::NormalizeAngle(filter.state()(0) - initialState(0));
            Assert::IsTrue(std::fabs(wrappedError) < 0.05f);
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
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            Assert::IsTrue(std::isfinite(derivatives.stateDot(VehicleState::kR)));
            Assert::AreEqual(0.0f, derivatives.stateDot(VehicleState::kR), 1.0e-4f);
        }

        TEST_METHOD(PlantModelComputesFiniteAlgebraicSlipAndForces)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 2.0f;
            state(VehicleState::kV) = 0.15f;
            state(VehicleState::kR) = 1.8f;
            state(VehicleState::kOmegaL) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.65f;
            control.rightMotorCommand = 0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const WheelKinematics kinematics = plant.wheelKinematics(state, params);
            const SlipTargets slip = plant.slipTargets(state, kinematics, params);
            const ContactForces forces = plant.tireForces(state, control, params);

            Assert::IsTrue(std::isfinite(slip.kappaLeft));
            Assert::IsTrue(std::isfinite(slip.kappaRight));
            for (float ratio : slip.lateralRatio)
            {
                Assert::IsTrue(std::isfinite(ratio));
            }

            for (const ContactForce& force : forces.contacts)
            {
                Assert::IsTrue(std::isfinite(force.fx));
                Assert::IsTrue(std::isfinite(force.fy));
                Assert::IsTrue(force.saturation >= 0.0f);
                Assert::IsTrue(force.saturation <= 1.0f);
            }
        }

        TEST_METHOD(PlantModelImuAccelerationIncludesLeverArmTerms)
        {
            PlantModel plant;
            PlantParams zeroLeverParams = PlantParams::Default();
            zeroLeverParams.imu.positionBodyM = Eigen::Vector2f::Zero();
            PlantParams leverParams = zeroLeverParams;
            leverParams.imu.positionBodyM = Eigen::Vector2f(0.020f, -0.010f);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.4f;
            state(VehicleState::kV) = 0.2f;
            state(VehicleState::kR) = 5.0f;
            state(VehicleState::kOmegaL) = 0.9f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.1f * (state(VehicleState::kU) / zeroLeverParams.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.30f;
            control.rightMotorCommand = 0.55f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = zeroLeverParams.supplyVoltageV;

            const PlantDerivatives zeroLever = plant.forwardStep(state, control, zeroLeverParams);
            const PlantDerivatives withLever = plant.forwardStep(state, control, leverParams);

            const float expectedDeltaX =
                (-withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.y()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.x());
            const float expectedDeltaY =
                (withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.x()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.y());

            Assert::AreEqual(
                zeroLever.imuAccelBodyMps2.x() + expectedDeltaX,
                withLever.imuAccelBodyMps2.x(),
                1.0e-5f);
            Assert::AreEqual(
                zeroLever.imuAccelBodyMps2.y() + expectedDeltaY,
                withLever.imuAccelBodyMps2.y(),
                1.0e-5f);
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
            observation.gyroZRadps = 0.5f;
            observation.accelBodyXMps2 = 1.0f;
            observation.accelBodyYMps2 = 0.1f;

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

        TEST_METHOD(SrUkfCoreUsesStationaryGyroMeasurementToConstrainBiasState)
        {
            SrUkfCore core;
            ControlInput control{};
            EncoderObs encoder{};
            constexpr float dt = 0.0005f;

            Assert::IsTrue(core.predict(dt, control));
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.accepted);

            const float rawStationaryGyroRadps = 0.12f;
            const MeasurementUpdateResult yawResult = core.updateYawRate(rawStationaryGyroRadps);
            Assert::IsTrue(yawResult.attempted);
            Assert::IsTrue(yawResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::AreEqual(0.0f, state(VehicleState::kR), 1.0e-6f);
            Assert::AreEqual(rawStationaryGyroRadps, state(VehicleState::kBgz), 1.0e-6f);
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
