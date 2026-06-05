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
    TEST_CLASS(PlantModelMixedSlipBoundsTest)
    {
    public:
        TEST_METHOD(MixedSlipContact0SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[0];
            std::wstringstream message;
            message << L"MixedSlipContact0SaturationIsNonNegative"
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[0] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[0];
            std::wstringstream message;
            message << L"MixedSlipContact0SaturationDoesNotExceedUnity"
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[0] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"MixedSlipContact0PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[0] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact0PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"MixedSlipContact0PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[0]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[0] >= measurement.saturation[0], message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[1];
            std::wstringstream message;
            message << L"MixedSlipContact1SaturationIsNonNegative"
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[1] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[1];
            std::wstringstream message;
            message << L"MixedSlipContact1SaturationDoesNotExceedUnity"
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[1] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"MixedSlipContact1PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[1] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact1PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"MixedSlipContact1PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[1]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[1] >= measurement.saturation[1], message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[2];
            std::wstringstream message;
            message << L"MixedSlipContact2SaturationIsNonNegative"
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[2] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[2];
            std::wstringstream message;
            message << L"MixedSlipContact2SaturationDoesNotExceedUnity"
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[2] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"MixedSlipContact2PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[2] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact2PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"MixedSlipContact2PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[2]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[2] >= measurement.saturation[2], message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3SaturationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[3];
            std::wstringstream message;
            message << L"MixedSlipContact3SaturationIsNonNegative"
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.saturation[3] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3SaturationDoesNotExceedUnity)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.saturation[3];
            std::wstringstream message;
            message << L"MixedSlipContact3SaturationDoesNotExceedUnity"
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual<=1";

            Assert::IsTrue(measurement.saturation[3] <= 1.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3PreProjectionUtilizationIsNonNegative)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"MixedSlipContact3PreProjectionUtilizationIsNonNegative"
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual>=0";

            Assert::IsTrue(measurement.preProjectionUtilization[3] >= 0.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipContact3PreProjectionUtilizationIsAtLeastSaturation)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"MixedSlipContact3PreProjectionUtilizationIsAtLeastSaturation"
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\nsaturation=" << measurement.saturation[3]
                << L"\ncriterion=actual>=saturation";

            Assert::IsTrue(measurement.preProjectionUtilization[3] >= measurement.saturation[3], message.str().c_str());
        }

    };
}
