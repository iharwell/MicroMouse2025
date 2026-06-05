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
    TEST_CLASS(PlantModelInPlaceSlipContactTest)
    {
    public:
        TEST_METHOD(InPlaceSlipContact0ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact0ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[0]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[0],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact1ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact1ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[1]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[1],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact2ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact2ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[2]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[2],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact3ForwardRelativeVelocityIsZero)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipContact3ForwardRelativeVelocityIsZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.forwardRelativeVelocityMps[3]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                measurement.forwardRelativeVelocityMps[3],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact0RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 0U);
            std::wstringstream message;
            message << L"InPlaceSlipContact0RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[0]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[0],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact1RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 1U);
            std::wstringstream message;
            message << L"InPlaceSlipContact1RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[1]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[1],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact2RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 2U);
            std::wstringstream message;
            message << L"InPlaceSlipContact2RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[2]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[2],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact3RightRelativeVelocityMatchesGeometry)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected =
                InPlaceSlipExpectedRightRelativeVelocity(measurement, 3U);
            std::wstringstream message;
            message << L"InPlaceSlipContact3RightRelativeVelocityMatchesGeometry"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightRelativeVelocityMps[3]
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                measurement.rightRelativeVelocityMps[3],
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact0RightForceMatchesFrontLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 0U);
            std::wstringstream message;
            message << L"InPlaceSlipContact0RightForceMatchesFrontLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[0]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[0],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact1RightForceMatchesFrontLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 1U);
            std::wstringstream message;
            message << L"InPlaceSlipContact1RightForceMatchesFrontLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[1]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[1],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact2RightForceMatchesRearLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 2U);
            std::wstringstream message;
            message << L"InPlaceSlipContact2RightForceMatchesRearLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[2]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[2],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipContact3RightForceMatchesRearLimit)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            const float expected = InPlaceSlipExpectedRightForce(measurement, 3U);
            std::wstringstream message;
            message << L"InPlaceSlipContact3RightForceMatchesRearLimit"
                << L"\nexpected=" << expected
                << L"\nactual=" << measurement.rightForceN[3]
                << L"\ntolerance=1e-4";

            Assert::AreEqual(
                expected,
                measurement.rightForceN[3],
                1.0e-4f,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawAccelerationIsFinite)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipYawAccelerationIsFinite"
                << L"\nactual=" << measurement.observedYawAccelRadps2
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.observedYawAccelRadps2),
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawRateDecreases)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipYawRateDecreases"
                << L"\ninitial=" << measurement.initialYawRateRadps
                << L"\nactual=" << measurement.actualYawRateRadps
                << L"\ncriterion=actual<initial";

            Assert::IsTrue(
                measurement.actualYawRateRadps < measurement.initialYawRateRadps,
                message.str().c_str());
        }

        TEST_METHOD(InPlaceSlipYawAccelerationMatchesSustainedWindow)
        {
            const InPlaceSlipYawDecelMeasurement measurement =
                MeasureInPlaceSlipYawDecel();
            std::wstringstream message;
            message << L"InPlaceSlipYawAccelerationMatchesSustainedWindow"
                << L"\nexpected=" << measurement.expectedYawAccelRadps2
                << L"\nactual=" << measurement.observedYawAccelRadps2
                << L"\ntolerance=1e-3";

            Assert::AreEqual(
                measurement.expectedYawAccelRadps2,
                measurement.observedYawAccelRadps2,
                1.0e-3f,
                message.str().c_str());
        }

    };
}
