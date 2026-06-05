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

    TEST_CLASS(EstimatorRepeatedZeroMotionStabilityTest)
    {
    public:
        TEST_METHOD(KeepsPoseXStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"position_x_m", result.state(0), L"magnitude <", kStationaryPoseDriftToleranceM, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(0)) < kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(BoundsPoseXVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[0,0]", result.covariance(0, 0), L"<", 10.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(0, 0) < 10.0f, message.c_str());
        }

        TEST_METHOD(KeepsPoseYStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"position_y_m", result.state(1), L"magnitude <", kStationaryPoseDriftToleranceM, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(1)) < kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(BoundsPoseYVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[1,1]", result.covariance(1, 1), L"<", 100.0f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(1, 1) < 100.0f, message.c_str());
        }

        TEST_METHOD(KeepsForwardVelocityStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"forward_velocity_mps", result.state(3), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(3)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsForwardVelocityVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[3,3]", result.covariance(3, 3), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(3, 3) < 0.0001f, message.c_str());
        }

        TEST_METHOD(KeepsLateralVelocityStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"lateral_velocity_mps", result.state(4), L"magnitude <", 1.0e-5f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(4)) < 1.0e-5f, message.c_str());
        }

        TEST_METHOD(BoundsLateralVelocityVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[4,4]", result.covariance(4, 4), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(4, 4) < 0.0001f, message.c_str());
        }

        TEST_METHOD(KeepsYawRateStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"yaw_rate_radps", result.state(5), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(5)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsYawRateVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[5,5]", result.covariance(5, 5), L"<=", kStationaryYawVarianceToleranceRadps2, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(5, 5) <= kStationaryYawVarianceToleranceRadps2, message.c_str());
        }

        TEST_METHOD(KeepsForwardAccelResidualStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"forward_accel_residual_mps2", result.state(6), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(6)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsForwardAccelVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[6,6]", result.covariance(6, 6), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(6, 6) < 0.0001f, message.c_str());
        }

        TEST_METHOD(KeepsRightwardAccelResidualStable)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"rightward_accel_residual_mps2", result.state(7), L"magnitude <", 1.0e-4f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(std::fabs(result.state(7)) < 1.0e-4f, message.c_str());
        }

        TEST_METHOD(BoundsRightwardAccelVariance)
        {
            const FilterSnapshot result = RunZeroMotionCycles(2000);
            const std::wstring message = LimitMessage(L"covariance[7,7]", result.covariance(7, 7), L"<", 0.0001f, ScenarioMessage(result.status).c_str());
            Assert::IsTrue(result.covariance(7, 7) < 0.0001f, message.c_str());
        }

    };
}
