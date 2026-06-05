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
    TEST_CLASS(PlantModelContactContinuumBelowForwardTest)
    {
    public:
        TEST_METHOD(ContactContinuumBelowForwardContact0ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact0SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[0];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact0SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact1SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[1];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact1SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact2SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[2];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact2SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardContact3SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[3];
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardContact3SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateForwardVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateForwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateForwardVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateLateralVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateRightwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateLateralVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateYawRateIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawRateRadps;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateYawRateIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumBelowForwardStateYawAccelerationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(-0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawAccelRadps2;
            std::wstringstream message;
            message << L"ContactContinuumBelowForwardStateYawAccelerationIsFinite"
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
