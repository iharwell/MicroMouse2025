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
    TEST_CLASS(PlantModelInPlaceSlipDecayTest)
    {
    public:
        TEST_METHOD(InPlaceSlipYawRateDoesNotIncreaseDuringDecay)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            std::wstringstream message;
            message << L"InPlaceSlipYawRateDoesNotIncreaseDuringDecay"
                << L"\nexpected_first_failure_step=-1"
                << L"\nactual_first_failure_step="
                << measurement.firstMonotonicFailureStep
                << L"\nprevious_abs_radps="
                << measurement.firstMonotonicFailurePreviousRadps
                << L"\nactual_abs_radps="
                << measurement.firstMonotonicFailureActualRadps;

            Assert::AreEqual(
                -1,
                measurement.firstMonotonicFailureStep,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawRateNeverExceedsInitialMagnitude)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            const float limit =
                measurement.initialYawRateRadps +
                measurement.initialMagnitudeToleranceRadps;
            std::wstringstream message;
            message << L"InPlaceSlipYawRateNeverExceedsInitialMagnitude"
                << L"\nactual=" << measurement.maxYawRateAbsRadps
                << L"\nlimit=" << limit
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.maxYawRateAbsRadps <= limit,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawRateReboundStaysWithinStopBand)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            std::wstringstream message;
            message << L"InPlaceSlipYawRateReboundStaysWithinStopBand"
                << L"\nactual=" << measurement.maxReboundRadps
                << L"\nlimit=" << measurement.maxAllowedReboundRadps
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.maxReboundRadps <= measurement.maxAllowedReboundRadps,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipSpinDownReachesStopBand)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            std::wstringstream message;
            message << L"InPlaceSlipSpinDownReachesStopBand"
                << L"\nactual_stop_step=" << measurement.stopStep
                << L"\ncriterion=actual_stop_step>0";

            Assert::IsTrue(
                measurement.stopStep > 0,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipStopTimeStaysWithinPhysicalWindow)
        {
            const InPlaceSlipSpinDownMeasurement measurement =
                MeasureInPlaceSlipSpinDown();
            const float limit =
                measurement.maxAllowedStopTimeS + measurement.dtSeconds;
            std::wstringstream message;
            message << L"InPlaceSlipStopTimeStaysWithinPhysicalWindow"
                << L"\nactual=" << measurement.stopTimeS
                << L"\nlimit=" << limit
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.stopTimeS <= limit,
                message.str().c_str());
        }

    };
}
