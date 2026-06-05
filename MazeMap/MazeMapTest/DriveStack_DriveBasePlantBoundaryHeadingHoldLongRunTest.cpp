#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseHeadingHoldLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nfinal_heading_error_rad=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nfinal_heading_error_rad=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(RequestedHeadingTargetIsPreserved)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=requested_yaw_rad"
                << L"\nexpected=" << HeadingHoldLongRunScenario::kTargetYawRad
                << L"\nactual_final=" << scenario.finalRequestedYawRad
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedHeadingPreserved,
                message.str().c_str());
        }

        TEST_METHOD(PhysicalPlantReducesHeadingError)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=final_heading_error_rad"
                << L"\ninitial=" << scenario.initialHeadingErrorRad
                << L"\nactual=" << scenario.finalHeadingErrorRad
                << L"\ncriterion=actual<initial*0.45";

            Assert::IsTrue(
                scenario.finalHeadingErrorRad < scenario.initialHeadingErrorRad * 0.45f,
                message.str().c_str());
        }

        TEST_METHOD(HeadingErrorStaysBoundedDuringCorrection)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=max_abs_heading_error_rad"
                << L"\ninitial=" << scenario.initialHeadingErrorRad
                << L"\nactual_max=" << scenario.maxAbsHeadingErrorRad
                << L"\ncriterion=error<=initial+0.08";

            Assert::IsTrue(
                scenario.headingErrorStayedBounded,
                message.str().c_str());
        }

        TEST_METHOD(FinalYawRateIsDamped)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=final_yaw_rate_radps"
                << L"\nactual=" << scenario.finalYawRateRadps
                << L"\ncriterion=abs(actual)<0.8";

            Assert::IsTrue(
                std::fabs(scenario.finalYawRateRadps) < 0.80f,
                message.str().c_str());
        }

        TEST_METHOD(SingleTurn)
        {
            const HeadingHoldLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_HEADING_CLOSED_LOOP"
                << L"\nfield=total heading traversal"
                << L"\nactual=" << scenario.totalHeadingDelta
				<< L"\ncriterion=" << (scenario.kTargetYawRad - scenario.kInitialYawRad)
                << L"\ntolerance=1e-3";

			Assert::AreEqual(
				scenario.kTargetYawRad - scenario.kInitialYawRad,
				scenario.totalHeadingDelta,
				1.0e-3f,
				message.str().c_str());
        }
    };
}
