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

    TEST_CLASS(EstimatorNonzeroStationaryEncoderCountsTest)
    {
    public:
        TEST_METHOD(KeepPoseX)
        {
            const FilterSnapshot result = RunExactStationaryLockWithNonzeroCounts();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(0), result.state(0), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepPoseY)
        {
            const FilterSnapshot result = RunExactStationaryLockWithNonzeroCounts();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(1), result.state(1), kStationaryPoseDriftToleranceM, message.c_str());
        }

        TEST_METHOD(KeepHeading)
        {
            const FilterSnapshot result = RunExactStationaryLockWithNonzeroCounts();
            const std::wstring message = ScenarioMessage(result.status);
            Assert::AreEqual(result.initialState(2), result.state(2), 1.0e-4f, message.c_str());
        }

    };
}
