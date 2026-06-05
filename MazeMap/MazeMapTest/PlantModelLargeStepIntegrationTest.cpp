#include "pch.h"
#include "CppUnitTest.h"

#include "PlantModelDynamicsTestSupport.h"

#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace PlantModelDynamicsTestSupport;
    TEST_CLASS(PlantModelLargeStepIntegrationTest)
    {
    public:
        TEST_METHOD(SingleLargeStepWheelSpeedsRemainSymmetric)
        {
            const LargeStepMeasurement measurement = IntegrateSingleLargeStep();
            const float actual = std::fabs(measurement.wheelSpeedDeltaRadps);
            std::wstringstream message;
            message << L"SingleLargeStepWheelSpeedsRemainSymmetric"
                << L"\nactual_abs=" << actual
                << L"\nlimit=1"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 1.0f, message.str().c_str());
        }

        TEST_METHOD(SingleLargeStepPositionXDriftIsBounded)
        {
            const LargeStepMeasurement measurement = IntegrateSingleLargeStep();
            const float actual = std::fabs(measurement.state.GetPositionX());
            std::wstringstream message;
            message << L"SingleLargeStepPositionXDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.005"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.005f, message.str().c_str());
        }

        TEST_METHOD(SingleLargeStepYawRateDriftIsBounded)
        {
            const LargeStepMeasurement measurement = IntegrateSingleLargeStep();
            const float actual = std::fabs(measurement.state.GetYawRate());
            std::wstringstream message;
            message << L"SingleLargeStepYawRateDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.10"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.10f, message.str().c_str());
        }

    };
}
