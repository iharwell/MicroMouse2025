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

    TEST_CLASS(EstimatorEncoderNoiseTuningTest)
    {
    public:
        TEST_METHOD(GeneralSigmaLeftVarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedVarianceRadps2, result.covariance(0, 0), 1.0e-5f);
        }

        TEST_METHOD(GeneralSigmaRightVarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedVarianceRadps2, result.covariance(1, 1), 1.0e-5f);
        }

        TEST_METHOD(GeneralSigmaLeftRightCovarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedCovarianceRadps2, result.covariance(0, 1), 1.0e-5f);
        }

        TEST_METHOD(GeneralSigmaRightLeftCovarianceMatches)
        {
            const EncoderPairNoiseResult result = RunEncoderPairNoiseScenario();
            Assert::AreEqual(result.expectedCovarianceRadps2, result.covariance(1, 0), 1.0e-5f);
        }

        TEST_METHOD(StationaryWheelSpeedSigmaUsesRequestedZeroSpeedSigma)
        {
            EstimatorTestRuntime runtime;
            const float expectedWheelSpeedSigmaRadps =
                Vehicle::WheelSpeedFromLinearVelocity(kEstimatorTestStationaryEncoderVelocitySigmaMps);
            Assert::AreEqual(
                expectedWheelSpeedSigmaRadps,
                runtime.plantModel.stationaryEncoderWheelSpeedSigmaRadps(kEstimatorTestStationaryEncoderVelocitySigmaMps),
                1.0e-6f);
        }

    };
}
