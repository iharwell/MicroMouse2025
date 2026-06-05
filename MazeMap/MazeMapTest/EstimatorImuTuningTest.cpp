#include "pch.h"
#include "CppUnitTest.h"

#include "EstimatorBiasAndStationaryTestSupport.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace EstimatorBiasAndStationaryTestSupport;

    TEST_CLASS(EstimatorImuTuningTest)
    {
    public:
        TEST_METHOD(YawRateVarianceMatchesTuning)
        {
            Assert::AreEqual(1.2e-6f, kEstimatorTestImuYawRateVarianceRadps2, 1.0e-12f);
        }

        TEST_METHOD(YawRateSigmaSquaresToVariance)
        {
            Assert::AreEqual(kEstimatorTestImuYawRateVarianceRadps2, kEstimatorTestImuYawRateSigmaRadps * kEstimatorTestImuYawRateSigmaRadps, 1.0e-12f);
        }

        TEST_METHOD(AccelSigmaMatchesTuning)
        {
            Assert::AreEqual(0.569900f, kEstimatorTestImuAccelSigmaMps2, 1.0e-6f);
        }

    };
}
