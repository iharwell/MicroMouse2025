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
    TEST_CLASS(PlantModelFeedforwardTimingTest)
    {
    public:
        TEST_METHOD(FeedforwardAccumulatorStaysFinite)
        {
            const FeedforwardTimingMeasurement measurement = MeasureFeedforwardTiming();
            std::wstringstream message;
            message << L"FeedforwardAccumulatorStaysFinite"
                << L"\nfeedforward_accumulator=" << measurement.accumulator
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.accumulator),
                message.str().c_str());
        }

        TEST_METHOD(FeedforwardRunsFasterThanIntegrate)
        {
            const FeedforwardTimingMeasurement measurement = MeasureFeedforwardTiming();
            std::wstringstream message;
            message << L"FeedforwardRunsFasterThanIntegrate"
                << L"\nfeedforward_ticks=" << measurement.feedforwardDuration.count()
                << L"\nintegrate_ticks=" << measurement.integrateDuration.count()
                << L"\ncriterion=feedforward_ticks<integrate_ticks";

            Assert::IsTrue(
                measurement.feedforwardDuration < measurement.integrateDuration,
                message.str().c_str());
        }

    };
}
