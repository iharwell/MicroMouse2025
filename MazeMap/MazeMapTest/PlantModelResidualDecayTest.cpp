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
    TEST_CLASS(PlantModelResidualDecayTest)
    {
    public:
        TEST_METHOD(ForwardAccelerationResidualUsesDeterministicDecay)
        {
            const ResidualDecayMeasurement measurement = MeasureResidualDecay();
            std::wstringstream message;
            message << L"ForwardAccelerationResidualUsesDeterministicDecay"
                << L"\nexpected=" << measurement.expectedForwardResidual
                << L"\nactual=" << measurement.actualForwardResidual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedForwardResidual,
                measurement.actualForwardResidual,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightwardAccelerationResidualUsesDeterministicDecay)
        {
            const ResidualDecayMeasurement measurement = MeasureResidualDecay();
            std::wstringstream message;
            message << L"RightwardAccelerationResidualUsesDeterministicDecay"
                << L"\nexpected=" << measurement.expectedRightResidual
                << L"\nactual=" << measurement.actualRightResidual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedRightResidual,
                measurement.actualRightResidual,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(YawAccelerationResidualUsesDeterministicDecay)
        {
            const ResidualDecayMeasurement measurement = MeasureResidualDecay();
            std::wstringstream message;
            message << L"YawAccelerationResidualUsesDeterministicDecay"
                << L"\nexpected=" << measurement.expectedYawResidual
                << L"\nactual=" << measurement.actualYawResidual
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                measurement.expectedYawResidual,
                measurement.actualYawResidual,
                1.0e-6f,
                message.str().c_str());
        }

    };
}
