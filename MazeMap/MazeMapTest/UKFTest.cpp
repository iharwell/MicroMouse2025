#include "pch.h"
#include "CppUnitTest.h"

#include "..\MazeMap\UKF.h"
#include "..\MazeMap\VehicleState.h"

#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    namespace
    {
        using ScalarState = Eigen::Matrix<float, 2, 1>;
        using ScalarCovariance = Eigen::Matrix<float, 2, 2>;
        using ScalarControl = Eigen::Matrix<float, 1, 1>;
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
    };
}
