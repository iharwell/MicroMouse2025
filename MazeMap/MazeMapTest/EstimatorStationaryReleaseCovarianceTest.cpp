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

    TEST_CLASS(EstimatorStationaryReleaseCovarianceTest)
    {
    public:
        TEST_METHOD(InflatesForwardVelocityVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[3,3]", result.releasedCovariance(3, 3), L"> stationary", result.stationaryCovariance(3, 3), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(3, 3) > result.stationaryCovariance(3, 3), message.c_str());
        }

        TEST_METHOD(InflatesLateralVelocityVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[4,4]", result.releasedCovariance(4, 4), L"> stationary", result.stationaryCovariance(4, 4), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(4, 4) > result.stationaryCovariance(4, 4), message.c_str());
        }

        TEST_METHOD(InflatesYawRateVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[5,5]", result.releasedCovariance(5, 5), L"> stationary", result.stationaryCovariance(5, 5), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(5, 5) > result.stationaryCovariance(5, 5), message.c_str());
        }

        TEST_METHOD(InflatesForwardAccelVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[6,6]", result.releasedCovariance(6, 6), L"> stationary", result.stationaryCovariance(6, 6), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(6, 6) > result.stationaryCovariance(6, 6), message.c_str());
        }

        TEST_METHOD(InflatesRightwardAccelVariance)
        {
            const ReleaseCovarianceResult result = RunStationaryRelease();
            const std::wstring message = LimitMessage(L"released_covariance[7,7]", result.releasedCovariance(7, 7), L"> stationary", result.stationaryCovariance(7, 7), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.releasedCovariance(7, 7) > result.stationaryCovariance(7, 7), message.c_str());
        }
    };
}
