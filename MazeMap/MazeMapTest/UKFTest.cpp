#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\SrUkfCore.h"
#include "..\MazeMap\UKF.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using ScalarState = Eigen::Matrix<float, 2, 1>;
        using ScalarCovariance = Eigen::Matrix<float, 2, 2>;
        using ScalarControl = Eigen::Matrix<float, 1, 1>;

        VehicleState::StateVector BuildUkfState(
            float xM,
            float yM,
            float yawRad,
            float forwardVelocityMps,
            float lateralVelocityMps,
            float yawRateRadps,
            float leftWheelSpeedRadps,
            float rightWheelSpeedRadps,
            float gyroBiasRadps = 0.0f)
        {
            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kPx) = xM;
            state(VehicleState::kPy) = yM;
            state(VehicleState::kPsi) = yawRad;
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kV) = lateralVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = rightWheelSpeedRadps;
            state(VehicleState::kBgz) = gyroBiasRadps;
            VehicleState::NormalizeStateVector(state);
            return state;
        }

        VehicleState::StateMatrix BuildUkfCovariance(
            float positionSigmaM = 0.01f,
            float headingSigmaRad = 0.03f,
            float forwardVelocitySigmaMps = 0.05f,
            float lateralVelocitySigmaMps = 0.05f,
            float yawRateSigmaRadps = 0.10f,
            float wheelSigmaRadps = 0.30f,
            float gyroBiasSigmaRadps = 0.03f)
        {
            VehicleState::StateMatrix covariance = VehicleState::StateMatrix::Zero();
            covariance(VehicleState::kPx, VehicleState::kPx) = positionSigmaM * positionSigmaM;
            covariance(VehicleState::kPy, VehicleState::kPy) = positionSigmaM * positionSigmaM;
            covariance(VehicleState::kPsi, VehicleState::kPsi) = headingSigmaRad * headingSigmaRad;
            covariance(VehicleState::kU, VehicleState::kU) = forwardVelocitySigmaMps * forwardVelocitySigmaMps;
            covariance(VehicleState::kV, VehicleState::kV) = lateralVelocitySigmaMps * lateralVelocitySigmaMps;
            covariance(VehicleState::kR, VehicleState::kR) = yawRateSigmaRadps * yawRateSigmaRadps;
            covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelSigmaRadps * wheelSigmaRadps;
            covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelSigmaRadps * wheelSigmaRadps;
            covariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasSigmaRadps * gyroBiasSigmaRadps;
            return covariance;
        }

        LocalMapView BuildLocalMapView(const Maze& maze) noexcept
        {
            LocalMapView map{};
            map.maze = &maze;
            map.radiusCells = 1U;
            return map;
        }
    }

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
            UKF<4, 2> filter;

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
            UKF<1, 1> filter;
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

        TEST_METHOD(SquareRootUkfVelocitySigmaChangesPositionVarianceButNotPositionMean)
        {
            UKF<2, 1> lowVarianceFilter;
            UKF<2, 1> highVarianceFilter;

            ScalarState initialState{};
            initialState << 1.25f, 2.0f;
            ScalarCovariance lowCovariance = ScalarCovariance::Zero();
            lowCovariance(0, 0) = 1.0e-6f;
            lowCovariance(1, 1) = 0.01f * 0.01f;
            ScalarCovariance highCovariance = ScalarCovariance::Zero();
            highCovariance(0, 0) = 1.0e-6f;
            highCovariance(1, 1) = 0.10f * 0.10f;

            lowVarianceFilter.setState(initialState, lowCovariance);
            highVarianceFilter.setState(initialState, highCovariance);
            lowVarianceFilter.setProcessNoise(ScalarCovariance::Zero());
            highVarianceFilter.setProcessNoise(ScalarCovariance::Zero());

            const ScalarControl control = ScalarControl::Zero();
            constexpr float dt = 0.40f;
            const auto constantVelocityModel =
                [](const ScalarState& sigmaPoint, const ScalarControl&, float sigmaDt) noexcept
                {
                    ScalarState predicted = sigmaPoint;
                    predicted(0) += sigmaPoint(1) * sigmaDt;
                    return predicted;
                };

            Assert::IsTrue(lowVarianceFilter.Predict(dt, control, constantVelocityModel));
            Assert::IsTrue(highVarianceFilter.Predict(dt, control, constantVelocityModel));

            const float expectedPositionM = initialState(0) + (initialState(1) * dt);
            Assert::AreEqual(expectedPositionM, lowVarianceFilter.state()(0), 1.0e-4f);
            Assert::AreEqual(expectedPositionM, highVarianceFilter.state()(0), 1.0e-4f);
            Assert::AreEqual(lowVarianceFilter.state()(0), highVarianceFilter.state()(0), 1.0e-4f);
            Assert::AreEqual(initialState(1), lowVarianceFilter.state()(1), 1.0e-6f);
            Assert::AreEqual(initialState(1), highVarianceFilter.state()(1), 1.0e-6f);

            const ScalarCovariance lowPosterior = lowVarianceFilter.covariance();
            const ScalarCovariance highPosterior = highVarianceFilter.covariance();
            Assert::IsTrue(highPosterior(0, 0) > (lowPosterior(0, 0) * 10.0f));
            Assert::IsTrue(highPosterior(1, 1) > (lowPosterior(1, 1) * 10.0f));
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
                Assert::IsTrue(std::isfinite(force.rightForceN));
                Assert::IsTrue(std::isfinite(force.forwardForceN));
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
                (withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.y()) -
                ((state(VehicleState::kR) * state(VehicleState::kR)) * leverParams.imu.positionBodyM.x());
            const float expectedDeltaY =
                (-withLever.stateDot(VehicleState::kR) * leverParams.imu.positionBodyM.x()) -
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

        TEST_METHOD(PlantModelPredictsImuAccelerationInProjectBodyAxes)
        {
            PlantModel plant;
            PlantParams params = PlantParams::Default();

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = 1.1f;
            state(VehicleState::kV) = -0.3f;
            state(VehicleState::kR) = 4.0f;
            state(VehicleState::kOmegaL) = 0.95f * (state(VehicleState::kU) / params.wheelRadiusM);
            state(VehicleState::kOmegaR) = 1.05f * (state(VehicleState::kU) / params.wheelRadiusM);

            ControlInput control;
            control.leftMotorCommand = 0.25f;
            control.rightMotorCommand = 0.45f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            const PlantDerivatives derivatives = plant.forwardStep(state, control, params);
            const Eigen::Vector2f predictedMeasurement = plant.imuPlanarAcceleration(state, control, params);

            Assert::AreEqual(derivatives.imuAccelBodyMps2.x(), predictedMeasurement.x(), 1.0e-5f);
            Assert::AreEqual(derivatives.imuAccelBodyMps2.y(), predictedMeasurement.y(), 1.0e-5f);
        }

        TEST_METHOD(PlantModelSymmetricPositiveDriveFromRestMovesForward)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            VehicleState::StateVector state = BuildUkfState(
                0.0f,
                0.09f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f);

            ControlInput control{};
            control.leftMotorCommand = 0.50f;
            control.rightMotorCommand = 0.50f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 25;
            for (int step = 0; step < kSteps; ++step)
            {
                state = plant.integrate(state, control, dt, params);
            }

            const float totalTimeS = dt * static_cast<float>(kSteps);
            const float averageAccelMps2 = state(VehicleState::kU) / totalTimeS;

            Assert::IsTrue(std::isfinite(state.sum()));
            Assert::IsTrue(state(VehicleState::kU) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPy) > 0.09f);
            Assert::IsTrue(averageAccelMps2 > 0.0f);
            Assert::IsTrue(averageAccelMps2 < 60.0f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 0.002f);
            Assert::IsTrue(std::fabs(state(VehicleState::kV)) < 0.02f);
            Assert::IsTrue(std::fabs(state(VehicleState::kR)) < 0.10f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPsi)) < 0.01f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsMatchesStraightTargetMotion)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float forwardVelocityMps = 2.4f;
            constexpr float desiredLongitudinalAccelMps2 = 3.2f;
            constexpr float yawRateRadps = 0.0f;
            constexpr float desiredYawAccelRadps2 = 0.0f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    desiredYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;

            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);
            Assert::IsTrue(solution.converged);
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) <= 1.0f);
            Assert::AreEqual(desiredLongitudinalAccelMps2, achieved.longitudinalAccelMps2, 5.0e-4f);
            Assert::AreEqual(desiredYawAccelRadps2, achieved.yawAccelRadps2, 5.0e-4f);
            Assert::AreEqual(solution.leftWheelAccelRadps2, achieved.stateDot(VehicleState::kOmegaL), 1.0e-2f);
            Assert::AreEqual(solution.rightWheelAccelRadps2, achieved.stateDot(VehicleState::kOmegaR), 1.0e-2f);
        }

        TEST_METHOD(PlantModelSolveDriveCommandsMatchesCombinedMotionTarget)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            constexpr float forwardVelocityMps = 2.1f;
            constexpr float desiredLongitudinalAccelMps2 = 1.4f;
            constexpr float yawRateRadps = 5.5f;
            constexpr float desiredYawAccelRadps2 = 9.0f;

            const DriveCommandSolution solution =
                plant.solveDriveCommands(
                    forwardVelocityMps,
                    desiredLongitudinalAccelMps2,
                    yawRateRadps,
                    desiredYawAccelRadps2,
                    params,
                    0.80f,
                    params.supplyVoltageV);

            VehicleState::StateVector state = VehicleState::StateVector::Zero();
            state(VehicleState::kU) = forwardVelocityMps;
            state(VehicleState::kR) = yawRateRadps;
            state(VehicleState::kOmegaL) = solution.leftWheelSpeedRadps;
            state(VehicleState::kOmegaR) = solution.rightWheelSpeedRadps;

            const PlantDerivatives achieved = plant.forwardStep(state, solution.control, params);
            Assert::IsTrue(solution.converged);
            Assert::IsTrue(std::isfinite(solution.control.leftMotorCommand));
            Assert::IsTrue(std::isfinite(solution.control.rightMotorCommand));
            Assert::IsTrue(std::fabs(solution.control.leftMotorCommand) <= 1.0f);
            Assert::IsTrue(std::fabs(solution.control.rightMotorCommand) <= 1.0f);
            Assert::AreEqual(desiredLongitudinalAccelMps2, achieved.longitudinalAccelMps2, 5.0e-4f);
            Assert::AreEqual(desiredYawAccelRadps2, achieved.yawAccelRadps2, 5.0e-4f);
            Assert::AreEqual(solution.leftWheelAccelRadps2, achieved.stateDot(VehicleState::kOmegaL), 1.0e-2f);
            Assert::AreEqual(solution.rightWheelAccelRadps2, achieved.stateDot(VehicleState::kOmegaR), 1.0e-2f);
        }

        TEST_METHOD(PlantModelDriveCommandInverseMatchesForwardMotorModel)
        {
            PlantModel plant;
            const PlantParams params = PlantParams::Default();
            const float wheelSpeedRadps = 145.0f;
            const float desiredWheelTorqueNm = 0.0035f;

            const float command =
                plant.driveCommandFromTorque(
                    desiredWheelTorqueNm,
                    wheelSpeedRadps,
                    params.supplyVoltageV,
                    params);
            const float reconstructedTorqueNm =
                plant.driveTorqueFromCommand(
                    command,
                    wheelSpeedRadps,
                    params.supplyVoltageV,
                    params);

            Assert::IsTrue(std::isfinite(command));
            Assert::IsTrue(std::fabs(command) <= 1.0f);
            Assert::AreEqual(desiredWheelTorqueNm, reconstructedTorqueNm, 1.0e-4f);
        }

        TEST_METHOD(ComputeEncoderPairSqrtNoise_UsesGeneralSigmaMappingForNonZeroReadings)
        {
            const PlantParams params = PlantParams::Default();
            EncoderObs observation{};
            observation.omegaLeftRadps = 1.0f;
            observation.omegaRightRadps = 1.0f;

            const Eigen::Matrix<float, 2, 2> sqrtNoise = SrUkfCore::ComputeEncoderPairSqrtNoise(observation, params);
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
            Assert::AreEqual(expectedSigmaRadps, SrUkfCore::ComputeStationaryEncoderOmegaSigmaRadps(params), 1.0e-9f);
        }

        TEST_METHOD(ConfiguredGeneralImuSigmasMatchInitialEstimates)
        {
            Assert::AreEqual(0.0013f, SrUkfCore::kImuYawRateSigmaRadps, 1.0e-9f);
            Assert::AreEqual(0.014f, SrUkfCore::kImuAccelSigmaMps2, 1.0e-9f);
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
            UKF<4, 2> filter;

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

        TEST_METHOD(SrUkfCoreUsesProvidedWheelRatesAfterPriorEncoderObservation)
        {
            SrUkfCore core;
            ControlInput control{};
            constexpr float dt = 0.001f;

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs first{};
            first.totalLeftCounts = 120;
            first.totalRightCounts = -96;
            first.omegaLeftRadps = 0.35f;
            first.omegaRightRadps = -0.28f;
            const MeasurementUpdateResult firstResult = core.updateEncoderPair(first, dt);
            Assert::IsTrue(firstResult.attempted);
            Assert::IsTrue(firstResult.accepted);

            Assert::IsTrue(core.predict(dt, control));
            EncoderObs second{};
            second.totalLeftCounts = -7;
            second.totalRightCounts = 11;
            second.omegaLeftRadps = 1.25f;
            second.omegaRightRadps = -0.75f;
            const MeasurementUpdateResult secondResult = core.updateEncoderPair(second, dt);
            Assert::IsTrue(secondResult.attempted);
            Assert::IsTrue(secondResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::AreEqual(second.omegaLeftRadps, state(VehicleState::kOmegaL), 1.0e-6f);
            Assert::AreEqual(second.omegaRightRadps, state(VehicleState::kOmegaR), 1.0e-6f);
        }

        TEST_METHOD(SrUkfCoreDoesNotLetControlInputCreateForwardMotionWithoutEncoderSupport)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.18f;
            control.rightMotorCommand = 0.18f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            EncoderObs encoder{};
            constexpr float dt = 0.001f;

            for (int step = 0; step < 200; ++step)
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
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-4f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-4f);
        }

        TEST_METHOD(SrUkfCoreEncoderArcUpdateUsesProjectTurnSignConventions)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(
                initialState,
                BuildUkfCovariance(0.005f, 0.01f, 0.01f, 0.01f, 0.02f, 0.05f, 0.01f)));

            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.010f;
            Assert::IsTrue(core.predict(dt, control));

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            EncoderObs encoder{};
            encoder.totalLeftCounts = 4;
            encoder.totalRightCounts = 2;
            encoder.omegaLeftRadps =
                (static_cast<float>(encoder.totalLeftCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);
            encoder.omegaRightRadps =
                (static_cast<float>(encoder.totalRightCounts) * distancePerCountM) /
                (params.wheelRadiusM * dt);

            const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const float expectedForwardDistanceM =
                0.5f * static_cast<float>(encoder.totalLeftCounts + encoder.totalRightCounts) * distancePerCountM;
            const float expectedYawRad =
                static_cast<float>(encoder.totalLeftCounts - encoder.totalRightCounts) * distancePerCountM / params.trackWidthM;
            const VehicleState::StateVector& state = core.state();

            Assert::IsTrue(std::isfinite(state.sum()));
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kPx) > 0.0f);
            Assert::IsTrue(state(VehicleState::kPsi) > 0.0f);
            Assert::AreEqual(expectedYawRad, state(VehicleState::kPsi), 1.0e-6f);
            Assert::AreEqual(
                expectedForwardDistanceM / dt,
                state(VehicleState::kU),
                1.0e-5f);
        }

        TEST_METHOD(SrUkfCoreSplitDrivePredictBuildsTurnRateWhileKeepingForwardProgress)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            constexpr float initialForwardVelocityMps = 1.0f;
            const float initialWheelSpeedRadps = initialForwardVelocityMps / params.wheelRadiusM;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.09f,
                    0.0f,
                    initialForwardVelocityMps,
                    0.0f,
                    0.0f,
                    initialWheelSpeedRadps,
                    initialWheelSpeedRadps);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance()));

            ControlInput control{};
            control.leftMotorCommand = 0.30f;
            control.rightMotorCommand = 0.60f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            constexpr float dt = 0.002f;
            constexpr int kSteps = 75;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));
            }

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(state(VehicleState::kPy) > initialState(VehicleState::kPy));
            Assert::IsTrue(state(VehicleState::kU) > 0.5f);
            Assert::IsTrue(state(VehicleState::kR) < -0.05f);
            Assert::IsTrue(state(VehicleState::kPsi) < -0.005f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) > 1.0e-3f);
        }

        TEST_METHOD(SrUkfCoreRepeatedForwardEncoderUpdatesStayLinearAndBoundAcceleration)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.0f,
                    0.12f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.01f, 0.03f, 0.05f, 0.05f, 0.08f, 0.20f, 0.03f)));

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            constexpr float dt = 0.005f;
            const float expectedSpeedMps = distancePerCountM / dt;
            const float measuredOmegaRadps = expectedSpeedMps / params.wheelRadiusM;
            ControlInput control{};
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;

            float previousPositionM = initialState(VehicleState::kPy);
            float previousSpeedMps = expectedSpeedMps;
            constexpr int kSteps = 5;
            for (int step = 0; step < kSteps; ++step)
            {
                Assert::IsTrue(core.predict(dt, control));

                EncoderObs encoder{};
                encoder.totalLeftCounts = 1;
                encoder.totalRightCounts = 1;
                encoder.omegaLeftRadps = measuredOmegaRadps;
                encoder.omegaRightRadps = measuredOmegaRadps;
                const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
                Assert::IsTrue(result.attempted);
                Assert::IsTrue(result.accepted);

                const VehicleState::StateVector& state = core.state();
                const float intervalDistanceM = state(VehicleState::kPy) - previousPositionM;
                const float intervalSpeedMps = intervalDistanceM / dt;
                const float intervalAccelMps2 = (intervalSpeedMps - previousSpeedMps) / dt;
                const float expectedPositionM =
                    initialState(VehicleState::kPy) + ((step + 1.0f) * distancePerCountM);

                Assert::IsTrue(intervalDistanceM > 0.0f);
                Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 0.002f);
                Assert::AreEqual(expectedPositionM, state(VehicleState::kPy), distancePerCountM * 0.25f);
                Assert::AreEqual(expectedSpeedMps, intervalSpeedMps, 0.15f);
                Assert::IsTrue(std::fabs(intervalAccelMps2) < 60.0f);
                Assert::IsTrue(state(VehicleState::kU) > 0.0f);
                Assert::IsTrue(state(VehicleState::kU) < 3.0f);

                previousPositionM = state(VehicleState::kPy);
                previousSpeedMps = intervalSpeedMps;
            }
        }

        TEST_METHOD(SrUkfCoreAnchorsPoseIncrementToEncoderCountsInsteadOfControlPrediction)
        {
            SrUkfCore core;
            const PlantParams params = PlantParams::Default();
            ControlInput control{};
            control.leftMotorCommand = 0.18f;
            control.rightMotorCommand = 0.18f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.001f;

            const float distancePerCountM =
                (2.0f * PI_F * params.wheelRadiusM) /
                (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
            const float measuredOmegaRadps = distancePerCountM / (params.wheelRadiusM * dt);

            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            encoder.totalLeftCounts = 1;
            encoder.totalRightCounts = 1;
            encoder.omegaLeftRadps = measuredOmegaRadps;
            encoder.omegaRightRadps = measuredOmegaRadps;
            const MeasurementUpdateResult encoderResult = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(encoderResult.attempted);
            Assert::IsTrue(encoderResult.accepted);

            const VehicleState::StateVector& state = core.state();
            Assert::IsTrue(std::fabs(state(VehicleState::kPx)) < 1.0e-7f);
            Assert::IsTrue(std::fabs(state(VehicleState::kPy) - distancePerCountM) < 1.0e-7f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU) - (distancePerCountM / dt)) < 1.0e-5f);
        }

        TEST_METHOD(SrUkfCoreZeroEncoderObservationConstrainsSpeedVarianceWithoutMovingPose)
        {
            const PlantParams params = PlantParams::Default();
            SrUkfCore core(params);
            constexpr float initialForwardVelocityMps = 1.2f;
            const float wheelSpeedRadps = initialForwardVelocityMps / params.wheelRadiusM;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.03f,
                    0.11f,
                    0.08f,
                    initialForwardVelocityMps,
                    0.02f,
                    0.15f,
                    wheelSpeedRadps,
                    wheelSpeedRadps);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.05f, 0.20f, 0.15f, 0.25f, 0.50f, 0.05f)));

            ControlInput control{};
            control.leftMotorCommand = 0.40f;
            control.rightMotorCommand = 0.40f;
            control.fanDutyCycle = 0.80f;
            control.batteryVoltageV = params.supplyVoltageV;
            constexpr float dt = 0.01f;
            Assert::IsTrue(core.predict(dt, control));

            EncoderObs encoder{};
            const MeasurementUpdateResult result = core.updateEncoderPair(encoder, dt);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& state = core.state();
            const VehicleState::StateMatrix covariance = core.covariance();
            Assert::AreEqual(initialState(VehicleState::kPx), state(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(initialState(VehicleState::kPy), state(VehicleState::kPy), 1.0e-6f);
            Assert::AreEqual(initialState(VehicleState::kPsi), state(VehicleState::kPsi), 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kU)) < 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaL)) < 1.0e-6f);
            Assert::IsTrue(std::fabs(state(VehicleState::kOmegaR)) < 1.0e-6f);
            Assert::IsTrue(covariance(VehicleState::kU, VehicleState::kU) < 1.0e-8f);
            Assert::IsTrue(covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) < 1.0e-7f);
            Assert::IsTrue(std::isfinite(covariance(VehicleState::kPy, VehicleState::kPy)));
            Assert::IsTrue(covariance(VehicleState::kPy, VehicleState::kPy) > 1.0e-12f);
        }

        TEST_METHOD(SrUkfCoreYawRateUpdateImprovesYawMeasurementFitWithoutMovingPosition)
        {
            SrUkfCore core;
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.02f,
                    0.14f,
                    0.10f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.01f, 0.04f, 0.02f, 0.02f, 0.30f, 0.10f, 0.30f)));

            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();
            constexpr float observedYawRateRadps = 0.35f;
            const float beforeError =
                std::fabs((before(VehicleState::kR) + before(VehicleState::kBgz)) - observedYawRateRadps);

            const MeasurementUpdateResult result = core.updateYawRate(observedYawRateRadps);
            Assert::IsTrue(result.attempted);
            Assert::IsTrue(result.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            const float afterError =
                std::fabs((after(VehicleState::kR) + after(VehicleState::kBgz)) - observedYawRateRadps);

            Assert::AreEqual(before(VehicleState::kPx), after(VehicleState::kPx), 1.0e-6f);
            Assert::AreEqual(before(VehicleState::kPy), after(VehicleState::kPy), 1.0e-6f);
            Assert::IsTrue(afterError < beforeError);
            Assert::IsTrue(afterCovariance(VehicleState::kBgz, VehicleState::kBgz) <
                beforeCovariance(VehicleState::kBgz, VehicleState::kBgz));
        }

        TEST_METHOD(SrUkfCoreFrontWallUpdateMovesForwardForCloserSymmetricObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);
            const LocalMapView map = BuildLocalMapView(maze);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.09f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);

            WallGeometryModel geometry;
            const GeometryPrediction leftPrediction = geometry.predictRay(initialState, params.frontLeftSensor, map);
            const GeometryPrediction rightPrediction = geometry.predictRay(initialState, params.frontRightSensor, map);
            Assert::IsTrue(leftPrediction.hit);
            Assert::IsTrue(rightPrediction.hit);

            SrUkfCore core(params);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs leftObservation{};
            leftObservation.valid = true;
            leftObservation.confidence = 1.0f;
            leftObservation.cls = ObsClass::WallLike;
            leftObservation.rho = leftPrediction.rangeM - 0.012f;

            WallObs rightObservation = leftObservation;
            rightObservation.rho = rightPrediction.rangeM - 0.012f;

            const FrontPairUpdateResult result = core.updateFrontPair(leftObservation, rightObservation, map);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(after(VehicleState::kPy) > (before(VehicleState::kPy) + 0.002f));
            Assert::IsTrue(std::fabs(after(VehicleState::kPx) - before(VehicleState::kPx)) < 0.004f);
            Assert::IsTrue(afterCovariance(VehicleState::kPy, VehicleState::kPy) <
                beforeCovariance(VehicleState::kPy, VehicleState::kPy));
        }

        TEST_METHOD(SrUkfCoreLeftWallUpdateMovesLeftForCloserObservation)
        {
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Left, WallState::Wall);
            const LocalMapView map = BuildLocalMapView(maze);
            const PlantParams params = PlantParams::Default();
            const VehicleState::StateVector initialState =
                BuildUkfState(
                    0.09f,
                    0.09f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f);

            WallGeometryModel geometry;
            const GeometryPrediction baseline = geometry.predictRay(initialState, params.sideLeftSensor, map);
            Assert::IsTrue(baseline.hit);

            SrUkfCore core(params);
            Assert::IsTrue(core.reset(initialState, BuildUkfCovariance(0.02f, 0.04f, 0.02f, 0.02f, 0.05f, 0.05f, 0.02f)));
            const VehicleState::StateVector before = core.state();
            const VehicleState::StateMatrix beforeCovariance = core.covariance();

            WallObs observation{};
            observation.valid = true;
            observation.confidence = 1.0f;
            observation.cls = ObsClass::WallLike;
            observation.rho = baseline.rangeM - 0.012f;

            const WallUpdateResult result = core.updateSideSensor(Side::Left, observation, map);
            Assert::IsTrue(result.filter.attempted);
            Assert::IsTrue(result.filter.accepted);

            const VehicleState::StateVector& after = core.state();
            const VehicleState::StateMatrix afterCovariance = core.covariance();
            Assert::IsTrue(after(VehicleState::kPx) < (before(VehicleState::kPx) - 0.002f));
            Assert::IsTrue(std::fabs(after(VehicleState::kPy) - before(VehicleState::kPy)) < 0.004f);
            Assert::IsTrue(afterCovariance(VehicleState::kPx, VehicleState::kPx) <
                beforeCovariance(VehicleState::kPx, VehicleState::kPx));
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
