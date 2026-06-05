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
    TEST_CLASS(PlantModelMixedSlipFiniteTest)
    {
    public:
        TEST_METHOD(MixedSlipLeftLongitudinalSlipIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.leftLongitudinalSlipMps;
            std::wstringstream message;
            message << L"MixedSlipLeftLongitudinalSlipIsFinite"
                << L"\nfield=left_longitudinal_slip_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipRightLongitudinalSlipIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightLongitudinalSlipMps;
            std::wstringstream message;
            message << L"MixedSlipRightLongitudinalSlipIsFinite"
                << L"\nfield=right_longitudinal_slip_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"MixedSlipContact0ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"MixedSlipContact1ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"MixedSlipContact2ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3ForwardRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"MixedSlipContact3ForwardRelativeVelocityIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"MixedSlipContact0RightRelativeVelocityIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"MixedSlipContact1RightRelativeVelocityIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"MixedSlipContact2RightRelativeVelocityIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3RightRelativeVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"MixedSlipContact3RightRelativeVelocityIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[0];
            std::wstringstream message;
            message << L"MixedSlipContact0RightForceIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[1];
            std::wstringstream message;
            message << L"MixedSlipContact1RightForceIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[2];
            std::wstringstream message;
            message << L"MixedSlipContact2RightForceIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3RightForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.rightForceN[3];
            std::wstringstream message;
            message << L"MixedSlipContact3RightForceIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[0];
            std::wstringstream message;
            message << L"MixedSlipContact0ForwardForceIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[1];
            std::wstringstream message;
            message << L"MixedSlipContact1ForwardForceIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[2];
            std::wstringstream message;
            message << L"MixedSlipContact2ForwardForceIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3ForwardForceIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.forwardForceN[3];
            std::wstringstream message;
            message << L"MixedSlipContact3ForwardForceIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[0];
            std::wstringstream message;
            message << L"MixedSlipContact0SaturationIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[1];
            std::wstringstream message;
            message << L"MixedSlipContact1SaturationIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[2];
            std::wstringstream message;
            message << L"MixedSlipContact2SaturationIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3SaturationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[3];
            std::wstringstream message;
            message << L"MixedSlipContact3SaturationIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"MixedSlipContact0PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"MixedSlipContact1PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"MixedSlipContact2PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3PreProjectionUtilizationIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"MixedSlipContact3PreProjectionUtilizationIsFinite"
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

    };
}
