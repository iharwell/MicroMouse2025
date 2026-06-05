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
    TEST_CLASS(PlantModelContactContinuumYawTest)
    {
    public:
        TEST_METHOD(ContactContinuumBelowZeroYawAccelerationIsFinite)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumBelowZeroYawAccelerationIsFinite"
                << L"\nactual=" << measurement.yawAccelerationsRadps2[0]
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.yawAccelerationsRadps2[0]),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroYawAccelerationIsFinite)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumZeroYawAccelerationIsFinite"
                << L"\nactual=" << measurement.yawAccelerationsRadps2[1]
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.yawAccelerationsRadps2[1]),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveZeroYawAccelerationIsFinite)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumAboveZeroYawAccelerationIsFinite"
                << L"\nactual=" << measurement.yawAccelerationsRadps2[2]
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.yawAccelerationsRadps2[2]),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumYawAccelerationNeighborDeltaIsBounded)
        {
            const ContactContinuumYawAccelerationMeasurement measurement =
                MeasureContactContinuumYawAcceleration();
            std::wstringstream message;
            message << L"ContactContinuumYawAccelerationNeighborDeltaIsBounded"
                << L"\nactual=" << measurement.maxNeighborDeltaRadps2
                << L"\nlimit=1e-3"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.maxNeighborDeltaRadps2 < 1.0e-3f,
                message.str().c_str());
        }

    };
}
