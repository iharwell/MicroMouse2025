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

    TEST_CLASS(EstimatorStationaryYawConstraintTest)
    {
    public:
        TEST_METHOD(ForwardVelocityCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(3), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(LateralVelocityCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(4), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(YawRateCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(5), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ForwardAccelResidualCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(6), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(RightwardAccelResidualCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(7), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(YawAccelResidualCollapses)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(8), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ForwardVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[3,3]", result.covariance(3, 3), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(3, 3) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(LateralVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const float limit = 0.005f * 0.005f;
            const std::wstring message = LimitMessage(L"covariance[4,4]", result.covariance(4, 4), L"<", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(4, 4) < limit, message.c_str());
        }

        TEST_METHOD(YawRateVarianceTracksCorrectedYawObservation)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring finiteMessage = ValueMessage(L"covariance[5,5]", result.covariance(5, 5), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.covariance(5, 5)), finiteMessage.c_str());
            const std::wstring positiveMessage = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) > 0.0f, positiveMessage.c_str());
            const float correctedYawVarianceLimitRadps2 = 1.5f * kEstimatorTestImuYawRateVarianceRadps2;
            const std::wstring boundedMessage = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L"<=", correctedYawVarianceLimitRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) <= correctedYawVarianceLimitRadps2, boundedMessage.c_str());
        }

        TEST_METHOD(ForwardAccelVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[6,6]", result.covariance(6, 6), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(6, 6) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(RightwardAccelVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[7,7]", result.covariance(7, 7), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(7, 7) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(YawAccelVariancePositive)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[8,8]", result.covariance(8, 8), L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(8, 8) > 0.0f, message.c_str());
        }

        TEST_METHOD(YawAccelVarianceBounded)
        {
            const FilterSnapshot result = RunStationaryYawConstraint();
            const std::wstring message = LimitMessage(L"covariance[8,8]", result.covariance(8, 8), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(8, 8) < 1.0e-4f, message.c_str());
        }

    };
}
