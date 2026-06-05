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

    TEST_CLASS(EstimatorRepeatedStationaryCycleTest)
    {
    public:
        TEST_METHOD(CapturesCertifiedCovariance)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = std::wstring(L"captured=") + BoolText(result.capturedFirstCertifiedCovariance) + L" " + ScenarioMessage(result.status);
            Assert::IsTrue(result.capturedFirstCertifiedCovariance, message.c_str());
        }

        TEST_METHOD(KeepsPoseX)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(0), result.state(0), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepsPoseY)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(1), result.state(1), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepsHeading)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(2), result.state(2), 1.0e-4f, message.c_str());
        }

        TEST_METHOD(ZeroesForwardVelocity)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(3), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ZeroesLateralVelocity)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(4), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(ZeroesYawRate)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(0.0f, result.state(5), kStationaryMotionTolerance, message.c_str());
        }

        TEST_METHOD(PoseXVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const float limit = result.firstCertifiedCovariance(0, 0) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[0,0]", result.covariance(0, 0), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(0, 0) <= limit, message.c_str());
        }

        TEST_METHOD(PoseYVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const float limit = result.firstCertifiedCovariance(1, 1) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[1,1]", result.covariance(1, 1), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(1, 1) <= limit, message.c_str());
        }

        TEST_METHOD(HeadingVarianceDoesNotGrow)
        {
            const FilterSnapshot result = RunRepeatedStationaryCycles();
            const float limit = result.firstCertifiedCovariance(2, 2) + 1.0e-9f;
            const std::wstring message = LimitMessage(L"covariance[2,2]", result.covariance(2, 2), L"<=", limit, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(2, 2) <= limit, message.c_str());
        }

    };
}
