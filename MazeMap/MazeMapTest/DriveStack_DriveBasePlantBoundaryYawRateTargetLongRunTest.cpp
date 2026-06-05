#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseYawRateTargetLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nfinal_yaw_rate_radps=" << scenario.finalYawRateRadps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nfinal_yaw_rate_radps=" << scenario.finalYawRateRadps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRateTargetIsPreserved)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=requested_yaw_rate_radps"
                << L"\nexpected=" << YawRateLongRunScenario::kTargetYawRateRadps
                << L"\nactual_final=" << scenario.finalRequestedYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedYawRatePreserved,
                message.str().c_str());
        }

        TEST_METHOD(ResponseBuildsClockwiseYawRate)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=maximum_yaw_rate_radps"
                << L"\ninitial=" << scenario.initialYawRateRadps
                << L"\nactual=" << scenario.maximumYawRateRadps
                << L"\ncriterion=actual>initial+0.35";

            Assert::IsTrue(
                scenario.maximumYawRateRadps > scenario.initialYawRateRadps + 0.35f,
                message.str().c_str());
        }

        TEST_METHOD(FinalYawRateSettlesNearTarget)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=final_yaw_rate_radps"
                << L"\nexpected=" << YawRateLongRunScenario::kTargetYawRateRadps
                << L"\nactual=" << scenario.finalYawRateRadps
                << L"\ntolerance=0.45";

            Assert::AreEqual(
                YawRateLongRunScenario::kTargetYawRateRadps,
                scenario.finalYawRateRadps,
                0.45f,
                message.str().c_str());
        }

        TEST_METHOD(DoesNotCreateLargeForwardDrift)
        {
            const YawRateLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_YAW_RATE_CLOSED_LOOP"
                << L"\nfield=max_abs_forward_velocity_mps"
                << L"\nactual=" << scenario.maxAbsForwardVelocityMps
                << L"\ncriterion=actual<0.1";

            Assert::IsTrue(
                scenario.forwardVelocityStayedBounded,
                message.str().c_str());
        }

    };
}
