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
    TEST_CLASS(PlantModelContactContinuumAboveForwardTest)
    {
    public:
        TEST_METHOD(ContactContinuumAboveForwardContact0ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3ForwardRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3ForwardRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3RightRelativeVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightRelativeVelocityMps[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3RightRelativeVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_relative_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3ForwardForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.forwardForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3ForwardForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=forward_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3RightForceIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.rightForceN[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3RightForceIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=right_force_n"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3PreProjectionUtilizationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.preProjectionUtilization[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3PreProjectionUtilizationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=pre_projection_utilization"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact0SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[0];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact0SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=0"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact1SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[1];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact1SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=1"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact2SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[2];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact2SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=2"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardContact3SaturationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.saturation[3];
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardContact3SaturationIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\ncontact_index=3"
                << L"\nfield=saturation"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateForwardVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateForwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateForwardVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_forward_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateLateralVelocityIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateRightwardVelocityMps;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateLateralVelocityIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_lateral_velocity_mps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateYawRateIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawRateRadps;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateYawRateIsFinite"
                << L"\nforward_speed_mps=" << sample.forwardSpeedMps
                << L"\nright_speed_mps=" << sample.rightSpeedMps
                << L"\nyaw_rate_radps=" << sample.yawRateRadps
                << L"\nfield=state_yaw_rate_radps"
                << L"\nactual=" << actual
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(std::isfinite(actual), message.str().c_str());
        }

        TEST_METHOD(ContactContinuumAboveForwardStateYawAccelerationIsFinite)
        {
            const ContactContinuumFiniteSample sample =
                MeasureContactContinuumFiniteSample(0.002f, 0.0f, 3.0f);
            const float actual = sample.stateYawAccelRadps2;
            std::wstringstream message;
            message << L"ContactContinuumAboveForwardStateYawAccelerationIsFinite"
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
