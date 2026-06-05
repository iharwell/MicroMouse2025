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
    TEST_CLASS(PlantModelLowSpeedYawTest)
    {
    public:
        TEST_METHOD(BelowLowSpeedYawAccelerationIsFinite)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"BelowLowSpeedYawAccelerationIsFinite"
                << L"\nactual=" << measurement.belowYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.belowYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(CenterLowSpeedYawAccelerationIsFinite)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"CenterLowSpeedYawAccelerationIsFinite"
                << L"\nactual=" << measurement.centerYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.centerYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(AboveLowSpeedYawAccelerationIsFinite)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"AboveLowSpeedYawAccelerationIsFinite"
                << L"\nactual=" << measurement.aboveYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.aboveYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(BelowLowSpeedYawAccelerationDecelerates)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"BelowLowSpeedYawAccelerationDecelerates"
                << L"\nactual=" << measurement.belowYawAccelRadps2
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                measurement.belowYawAccelRadps2 < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(CenterLowSpeedYawAccelerationDecelerates)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"CenterLowSpeedYawAccelerationDecelerates"
                << L"\nactual=" << measurement.centerYawAccelRadps2
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                measurement.centerYawAccelRadps2 < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(AboveLowSpeedYawAccelerationDecelerates)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"AboveLowSpeedYawAccelerationDecelerates"
                << L"\nactual=" << measurement.aboveYawAccelRadps2
                << L"\ncriterion=actual<0";

            Assert::IsTrue(
                measurement.aboveYawAccelRadps2 < 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(LowSpeedYawAccelerationNeighborDeltaIsBounded)
        {
            const LowSpeedYawAccelerationMeasurement measurement =
                MeasureLowSpeedYawAcceleration();
            std::wstringstream message;
            message << L"LowSpeedYawAccelerationNeighborDeltaIsBounded"
                << L"\nactual=" << measurement.maxNeighborDeltaRadps2
                << L"\nlimit=" << measurement.maxAllowedNeighborDeltaRadps2
                << L"\ncriterion=actual<=limit";

            Assert::IsTrue(
                measurement.maxNeighborDeltaRadps2 <=
                    measurement.maxAllowedNeighborDeltaRadps2,
                message.str().c_str());
        }

    };
}
