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
    TEST_CLASS(PlantModelImuAccelerationTest)
    {
    public:
        TEST_METHOD(ImuRightLeverContributionIsPresent)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(5.0f, 1.4f, 0.2f, 0.9f, 1.1f, 0.30f, 0.55f);
            std::wstringstream message;
            message << L"ImuRightLeverContributionIsPresent"
                << L"\nactual_abs=" << std::fabs(measurement.rightLeverContributionMps2)
                << L"\nlimit=1e-3"
                << L"\ncriterion=actual_abs>limit";

            Assert::IsTrue(
                std::fabs(measurement.rightLeverContributionMps2) > 1.0e-3f,
                message.str().c_str());
        }

        TEST_METHOD(ImuRightAccelerationMatchesLeverArmEquation)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(5.0f, 1.4f, 0.2f, 0.9f, 1.1f, 0.30f, 0.55f);
            std::wstringstream message;
            message << L"ImuRightAccelerationMatchesLeverArmEquation"
                << L"\nexpected=" << measurement.expectedRightAccelerationMps2
                << L"\nactual=" << measurement.predictedRightAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedRightAccelerationMps2,
                measurement.predictedRightAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ImuForwardAccelerationMatchesLeverArmEquation)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(5.0f, 1.4f, 0.2f, 0.9f, 1.1f, 0.30f, 0.55f);
            std::wstringstream message;
            message << L"ImuForwardAccelerationMatchesLeverArmEquation"
                << L"\nexpected=" << measurement.expectedForwardAccelerationMps2
                << L"\nactual=" << measurement.predictedForwardAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedForwardAccelerationMps2,
                measurement.predictedForwardAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ImuRightAccelerationUsesProjectBodyAxes)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(4.0f, 1.1f, -0.3f, 0.95f, 1.05f, 0.25f, 0.45f);
            std::wstringstream message;
            message << L"ImuRightAccelerationUsesProjectBodyAxes"
                << L"\nexpected=" << measurement.expectedRightAccelerationMps2
                << L"\nactual=" << measurement.predictedRightAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedRightAccelerationMps2,
                measurement.predictedRightAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

        TEST_METHOD(ImuForwardAccelerationUsesProjectBodyAxes)
        {
            const ImuAccelerationMeasurement measurement =
                MeasureImuAcceleration(4.0f, 1.1f, -0.3f, 0.95f, 1.05f, 0.25f, 0.45f);
            std::wstringstream message;
            message << L"ImuForwardAccelerationUsesProjectBodyAxes"
                << L"\nexpected=" << measurement.expectedForwardAccelerationMps2
                << L"\nactual=" << measurement.predictedForwardAccelerationMps2
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                measurement.expectedForwardAccelerationMps2,
                measurement.predictedForwardAccelerationMps2,
                1.0e-5f,
                message.str().c_str());
        }

    };
}
