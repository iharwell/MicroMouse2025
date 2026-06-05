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
    TEST_CLASS(PlantModelPositiveDriveTest)
    {
    public:
        TEST_METHOD(SymmetricPositiveDriveForwardVelocityIncreases)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDriveForwardVelocityIncreases"
                << L"\nactual=" << measurement.state.GetForwardVelocity()
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                measurement.state.GetForwardVelocity() > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDrivePositionYIncreases)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDrivePositionYIncreases"
                << L"\ninitial_y_m=0.09"
                << L"\nactual=" << measurement.state.GetPositionY()
                << L"\ncriterion=actual>initial";

            Assert::IsTrue(
                measurement.state.GetPositionY() > 0.09f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveAverageAccelerationIsPositive)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDriveAverageAccelerationIsPositive"
                << L"\nactual=" << measurement.averageAccelMps2
                << L"\ncriterion=actual>0";

            Assert::IsTrue(
                measurement.averageAccelMps2 > 0.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveAverageAccelerationIsPlausible)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            std::wstringstream message;
            message << L"SymmetricPositiveDriveAverageAccelerationIsPlausible"
                << L"\nactual=" << measurement.averageAccelMps2
                << L"\nlimit=60"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.averageAccelMps2 < 60.0f,
                message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDrivePositionXDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetPositionX());
            std::wstringstream message;
            message << L"SymmetricPositiveDrivePositionXDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.002"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.002f, message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveLateralVelocityDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetRightwardVelocity());
            std::wstringstream message;
            message << L"SymmetricPositiveDriveLateralVelocityDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.02"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.02f, message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveYawRateDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetYawRate());
            std::wstringstream message;
            message << L"SymmetricPositiveDriveYawRateDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.10"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.10f, message.str().c_str());
        }

        TEST_METHOD(SymmetricPositiveDriveHeadingDriftIsBounded)
        {
            const PositiveDriveFromRestMeasurement measurement =
                IntegratePositiveDriveFromRest();
            const float actual = std::fabs(measurement.state.GetHeading());
            std::wstringstream message;
            message << L"SymmetricPositiveDriveHeadingDriftIsBounded"
                << L"\nactual_abs=" << actual
                << L"\nlimit=0.01"
                << L"\ncriterion=actual_abs<limit";

            Assert::IsTrue(actual < 0.01f, message.str().c_str());
        }

    };
}
