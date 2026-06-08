#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseVelocityFeedbackTest)
    {
    public:
        TEST_METHOD(RequestedForwardMpsIsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_forward_mps"
                << L"\nexpected=0.8"
                << L"\nactual="
                << scenario.telemetry.requestedForwardMps
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.80f,
                scenario.telemetry.requestedForwardMps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRateRadpsIsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_yaw_rate_radps"
                << L"\nexpected=0.25"
                << L"\nactual="
                << scenario.telemetry.requestedYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.25f,
                scenario.telemetry.requestedYawRateRadps,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedForwardAccelMps2IsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_forward_accel_mps2"
                << L"\nexpected=0.3"
                << L"\nactual="
                << scenario.telemetry.requestedForwardAccelMps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.30f,
                scenario.telemetry.requestedForwardAccelMps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawAccelRadps2IsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_yaw_accel_radps2"
                << L"\nexpected=0.4"
                << L"\nactual="
                << scenario.telemetry.requestedYawAccelRadps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.40f,
                scenario.telemetry.requestedYawAccelRadps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRadIsPreserved)
        {
            const FeedbackScenario scenario;
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=requested_yaw_rad"
                << L"\nexpected=0.18"
                << L"\nactual="
                << scenario.telemetry.requestedYawRad
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.18f,
                scenario.telemetry.requestedYawRad,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ComposedForwardAccelerationUsesProductionTrackingOnce)
        {
            const FeedbackScenario scenario;
            const float expected =
                Config::kDriveBaseTrackingTuning.ComposeForwardAccelerationMps2(
                    0.30f,
                    kNaN,
                    0.80f - 0.20f,
                    0.30f - 0.0f);
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=composed_forward_accel_mps2"
                << L"\nexpected=" << expected
                << L"\nactual=" << scenario.telemetry.composedForwardAccelMps2
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                expected,
                scenario.telemetry.composedForwardAccelMps2,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(ComposedYawAccelerationUsesProductionTrackingOnce)
        {
            const FeedbackScenario scenario;
            const float expected =
                Config::kDriveBaseTrackingTuning.ComposeYawAccelerationRadps2(
                    0.40f,
                    AngleDifference(0.10f, 0.18f),
                    0.25f - -0.15f,
                    0.40f - 0.0f);
            std::wstringstream message;
            message << L"DRV30_FEEDBACK_DOUBLE_APPLY"
                << L"\nfield=composed_yaw_accel_radps2"
                << L"\nexpected=" << expected
                << L"\nactual=" << scenario.telemetry.composedYawAccelRadps2
                << L"\ntolerance=1e-3";

            Assert::AreEqual(
                expected,
                scenario.telemetry.composedYawAccelRadps2,
                1.0e-3f,
                message.str().c_str());
        }

    };
}
