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

    TEST_CLASS(EstimatorZeroEncoderStationaryVarianceTest)
    {
    public:
        TEST_METHOD(YawRateVarianceIsFinite)
        {
            const FilterSnapshot result = RunZeroEncoderYawVariance();
            const std::wstring message = ValueMessage(L"covariance[5,5]", result.covariance(5, 5), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.covariance(5, 5)), message.c_str());
        }

        TEST_METHOD(YawRateVarianceDropsToMeasurementLimit)
        {
            const FilterSnapshot result = RunZeroEncoderYawVariance();
            const float limit = ExpectedZeroEncoderYawVarianceLimit();
            const std::wstring message = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) <= limit, message.c_str());
        }

        TEST_METHOD(InitialLateralVarianceMatchesProfile)
        {
            const LateralVarianceResult result = RunZeroEncoderLateralVariance();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(1.0f, result.initialVarianceMps2, 1.0e-6f, message.c_str());
        }

        TEST_METHOD(FinalLateralVarianceIsFinite)
        {
            const LateralVarianceResult result = RunZeroEncoderLateralVariance();
            const std::wstring message = ValueMessage(L"final_lateral_variance_mps2", result.finalVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.finalVarianceMps2), message.c_str());
        }

        TEST_METHOD(FinalLateralVarianceShrinks)
        {
            const LateralVarianceResult result = RunZeroEncoderLateralVariance();
            const std::wstring message = LimitMessage(L"final_lateral_variance_mps2", result.finalVarianceMps2, L"< initial", result.initialVarianceMps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.finalVarianceMps2 < result.initialVarianceMps2, message.c_str());
        }

    };
}
