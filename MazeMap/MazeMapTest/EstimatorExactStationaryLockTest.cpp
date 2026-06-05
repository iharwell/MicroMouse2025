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

    TEST_CLASS(EstimatorExactStationaryLockTest)
    {
    public:
        TEST_METHOD(KeepsReferencePoseX)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(0), result.state(0), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsReferencePoseY)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(1), result.state(1), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(KeepsReferenceHeading)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(2), result.state(2), 1.0e-6f, message.c_str());
        }

        TEST_METHOD(CollapsesForwardVelocity)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(3), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesLateralVelocity)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(4), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesYawRate)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(5), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesForwardAccelResidual)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(6), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CollapsesRightwardAccelResidual)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(7), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(PreservesYawAccelResidual)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(8), result.state(8), 1.0e-7f, message.c_str());
        }

        TEST_METHOD(CapturesFirstCertifiedCovariance)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = std::wstring(L"captured=") + BoolText(result.capturedFirstCertifiedCovariance) + L" " + ScenarioMessage(result.status);
            Assert::IsTrue(result.capturedFirstCertifiedCovariance, message.c_str());
        }

        TEST_METHOD(PoseXVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = result.firstCertifiedCovariance(0, 0) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[0,0]", result.covariance(0, 0), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(0, 0) <= limit, message.c_str());
        }

        TEST_METHOD(PoseYVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = result.firstCertifiedCovariance(1, 1) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[1,1]", result.covariance(1, 1), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(1, 1) <= limit, message.c_str());
        }

        TEST_METHOD(HeadingVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const float limit = result.firstCertifiedCovariance(2, 2) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[2,2]", result.covariance(2, 2), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(2, 2) <= limit, message.c_str());
        }

        TEST_METHOD(ForwardVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = LimitMessage(L"covariance[3,3]", result.covariance(3, 3), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(3, 3) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(LateralVelocityVarianceBounded)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring message = LimitMessage(L"covariance[4,4]", result.covariance(4, 4), L"<", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(4, 4) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(YawRateVarianceTracksCorrectedYawObservation)
        {
            const FilterSnapshot result = RunExactStationaryReference();
            const std::wstring finiteMessage = ValueMessage(L"covariance[5,5]", result.covariance(5, 5), ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::isfinite(result.covariance(5, 5)), finiteMessage.c_str());
            const std::wstring positiveMessage = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L">", 0.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) > 0.0f, positiveMessage.c_str());
            const float correctedYawVarianceLimitRadps2 = 1.5f * kEstimatorTestImuYawRateVarianceRadps2;
            const std::wstring boundedMessage = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L"<=", correctedYawVarianceLimitRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) <= correctedYawVarianceLimitRadps2, boundedMessage.c_str());
        }

    };
}
