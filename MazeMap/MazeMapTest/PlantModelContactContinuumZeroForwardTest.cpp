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
    TEST_CLASS(PlantModelContactContinuumZeroForwardTest)
    {
    public:
        TEST_METHOD(ContactContinuumZeroForwardContact0ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact0SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[0];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact0SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact1SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[1];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact1SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact2SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[2];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact2SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardContact3SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.saturation[3];
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardContact3SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateForwardVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateForwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateForwardVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateLateralVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateRightwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateLateralVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateYawRateIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateYawRateRadps;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateYawRateIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumZeroForwardStateYawAccelerationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.0f, 0.0f, 3.0f);
            const float actual = sample.stateYawAccelRadps2;
            std::wstringstream message;
            message << L"ContactContinuumZeroForwardStateYawAccelerationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_accel_radps2"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

    };
}
