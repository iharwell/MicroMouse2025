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

        TEST_METHOD(SrUkfCoreAcceptsMandatoryZeroDeltaEncoderUpdates)
        {
            SrUkfCore core;
            ControlInput control;
            control.leftMotorCommand = 0.0f;
            control.rightMotorCommand = 0.0f;
            control.fanDutyCycle = 0.80f;

            Assert::IsTrue(core.predict(0.0005f, control));

            EncoderObs firstObservation{};
            firstObservation.totalLeftCounts = 1000;
            firstObservation.totalRightCounts = 1000;
            firstObservation.omegaLeftRadps = 0.0f;
            firstObservation.omegaRightRadps = 0.0f;
            const MeasurementUpdateResult firstUpdate = core.updateEncoderPair(firstObservation, 0.0005f);
            Assert::IsTrue(firstUpdate.attempted);
            Assert::IsTrue(firstUpdate.accepted);

            EncoderObs secondObservation = firstObservation;
            const MeasurementUpdateResult secondUpdate = core.updateEncoderPair(secondObservation, 0.0005f);
            Assert::IsTrue(secondUpdate.attempted);
            Assert::IsTrue(secondUpdate.accepted);
            Assert::AreEqual(0.0f, core.state()(VehicleState::kOmegaL), 2.0f);
            Assert::AreEqual(0.0f, core.state()(VehicleState::kOmegaR), 2.0f);
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
    };
}
