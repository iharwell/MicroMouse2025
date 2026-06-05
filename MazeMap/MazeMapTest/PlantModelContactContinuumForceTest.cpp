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
    TEST_CLASS(PlantModelContactContinuumForceTest)
    {
    public:
        TEST_METHOD(ContactContinuumFrontLeftForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontLeftForwardForceIsFinite"
                << L"\nactual=" << measurement.frontLeftForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.frontLeftForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumFrontRightForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontRightForwardForceIsFinite"
                << L"\nactual=" << measurement.frontRightForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.frontRightForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearLeftForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearLeftForwardForceIsFinite"
                << L"\nactual=" << measurement.rearLeftForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.rearLeftForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearRightForwardForceIsFinite)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearRightForwardForceIsFinite"
                << L"\nactual=" << measurement.rearRightForwardForceN
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(measurement.rearRightForwardForceN),
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumFrontLeftForwardForceIsNegative)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontLeftForwardForceIsNegative"
                << L"\nactual=" << measurement.frontLeftForwardForceN
                << L"\nlimit=-1e-6"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.frontLeftForwardForceN < -1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearLeftForwardForceIsNegative)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearLeftForwardForceIsNegative"
                << L"\nactual=" << measurement.rearLeftForwardForceN
                << L"\nlimit=-1e-6"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(
                measurement.rearLeftForwardForceN < -1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumFrontRightForwardForceIsPositive)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumFrontRightForwardForceIsPositive"
                << L"\nactual=" << measurement.frontRightForwardForceN
                << L"\nlimit=1e-6"
                << L"\ncriterion=actual>limit";

            Assert::IsTrue(
                measurement.frontRightForwardForceN > 1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumRearRightForwardForceIsPositive)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumRearRightForwardForceIsPositive"
                << L"\nactual=" << measurement.rearRightForwardForceN
                << L"\nlimit=1e-6"
                << L"\ncriterion=actual>limit";

            Assert::IsTrue(
                measurement.rearRightForwardForceN > 1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ContactContinuumNetForwardForceIsNearZero)
        {
            const ContactForwardForceCoupleMeasurement measurement =
                MeasureContactForwardForceCouple();
            std::wstringstream message;
            message << L"ContactContinuumNetForwardForceIsNearZero"
                << L"\nexpected=0"
                << L"\nactual=" << measurement.totalForwardForceN
                << L"\ntolerance=1e-5";

            Assert::AreEqual(
                0.0f,
                measurement.totalForwardForceN,
                1.0e-5f,
                message.str().c_str());
        }

    };
}
