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
    TEST_CLASS(PlantModelMixedSlipStateTest)
    {
    public:
        TEST_METHOD(MixedSlipStatePositionXIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetPositionX();
            std::wstringstream message;
            message << L"MixedSlipStatePositionXIsFinite"
                << L"\nfield=x_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStatePositionYIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetPositionY();
            std::wstringstream message;
            message << L"MixedSlipStatePositionYIsFinite"
                << L"\nfield=y_m"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateHeadingIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetHeading();
            std::wstringstream message;
            message << L"MixedSlipStateHeadingIsFinite"
                << L"\nfield=heading_rad"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateForwardVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetForwardVelocity();
            std::wstringstream message;
            message << L"MixedSlipStateForwardVelocityIsFinite"
                << L"\nfield=forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLateralVelocityIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetRightwardVelocity();
            std::wstringstream message;
            message << L"MixedSlipStateLateralVelocityIsFinite"
                << L"\nfield=lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateYawRateIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetYawRate();
            std::wstringstream message;
            message << L"MixedSlipStateYawRateIsFinite"
                << L"\nfield=yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLeftWheelSpeedIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetWheelSpeedLeft();
            std::wstringstream message;
            message << L"MixedSlipStateLeftWheelSpeedIsFinite"
                << L"\nfield=left_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateRightWheelSpeedIsFinite)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetWheelSpeedRight();
            std::wstringstream message;
            message << L"MixedSlipStateRightWheelSpeedIsFinite"
                << L"\nfield=right_wheel_speed_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateHeadingDoesNotExceedPi)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetHeading();
            std::wstringstream message;
            message << L"MixedSlipStateHeadingDoesNotExceedPi"
                << L"\nfield=heading_rad"
                << L"\nactual=" << actual
                << L"\nlimit=PI_F"
                << L"\ncriterion=actual<=PI_F";

            Assert::IsTrue(measurement.state.GetHeading() <= PI_F, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateHeadingDoesNotGoBelowNegativePi)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = measurement.state.GetHeading();
            std::wstringstream message;
            message << L"MixedSlipStateHeadingDoesNotGoBelowNegativePi"
                << L"\nfield=heading_rad"
                << L"\nactual=" << actual
                << L"\nlimit=-PI_F"
                << L"\ncriterion=actual>=-PI_F";

            Assert::IsTrue(measurement.state.GetHeading() >= -PI_F, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateForwardVelocityMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetForwardVelocity());
            std::wstringstream message;
            message << L"MixedSlipStateForwardVelocityMagnitudeIsBounded"
                << L"\nfield=forward_velocity_abs_mps"
                << L"\nactual=" << actual
                << L"\nlimit=10"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetForwardVelocity()) < 10.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLateralVelocityMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetRightwardVelocity());
            std::wstringstream message;
            message << L"MixedSlipStateLateralVelocityMagnitudeIsBounded"
                << L"\nfield=lateral_velocity_abs_mps"
                << L"\nactual=" << actual
                << L"\nlimit=10"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetRightwardVelocity()) < 10.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateYawRateMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetYawRate());
            std::wstringstream message;
            message << L"MixedSlipStateYawRateMagnitudeIsBounded"
                << L"\nfield=yaw_rate_abs_radps"
                << L"\nactual=" << actual
                << L"\nlimit=50"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetYawRate()) < 50.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateLeftWheelSpeedMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetWheelSpeedLeft());
            std::wstringstream message;
            message << L"MixedSlipStateLeftWheelSpeedMagnitudeIsBounded"
                << L"\nfield=left_wheel_speed_abs_radps"
                << L"\nactual=" << actual
                << L"\nlimit=1000"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetWheelSpeedLeft()) < 1000.0f, message.str().c_str());
        }

        TEST_METHOD(MixedSlipStateRightWheelSpeedMagnitudeIsBounded)
        {
            const MixedSlipMeasurement measurement = MeasureMixedSlipCommand();
            const float actual = std::fabs(measurement.state.GetWheelSpeedRight());
            std::wstringstream message;
            message << L"MixedSlipStateRightWheelSpeedMagnitudeIsBounded"
                << L"\nfield=right_wheel_speed_abs_radps"
                << L"\nactual=" << actual
                << L"\nlimit=1000"
                << L"\ncriterion=actual<limit";

            Assert::IsTrue(std::fabs(measurement.state.GetWheelSpeedRight()) < 1000.0f, message.str().c_str());
        }
    };
}
