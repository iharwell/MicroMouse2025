#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorModeAndDiagnosticsTestSupport.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorModeAndDiagnosticsTestSupport;

    TEST_CLASS(EstimatorDefaultInitialCovarianceTest)
    {
    public:
        TEST_METHOD(PoseXMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-5f, covariance(0, 0), 1.0e-9f);
        }

        TEST_METHOD(PoseYMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-5f, covariance(1, 1), 1.0e-9f);
        }

        TEST_METHOD(HeadingMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(2, 2), 1.0e-9f);
        }

        TEST_METHOD(ForwardVelocityMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(3, 3), 1.0e-9f);
        }

        TEST_METHOD(LateralVelocityMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(4, 4), 1.0e-9f);
        }

        TEST_METHOD(YawRateMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(1.0e-3f, covariance(5, 5), 1.0e-9f);
        }

        TEST_METHOD(ForwardAccelResidualMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(0.25f, covariance(6, 6), 1.0e-9f);
        }

        TEST_METHOD(RightwardAccelResidualMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(0.25f, covariance(7, 7), 1.0e-9f);
        }

        TEST_METHOD(YawAccelResidualMatchesReset)
        {
            const CovarianceMatrix covariance =
                EstimatorModeAndDiagnosticsTest::BuildDefaultInitialCovariance();
            Assert::AreEqual(0.25f, covariance(8, 8), 1.0e-9f);
        }

    };
}
