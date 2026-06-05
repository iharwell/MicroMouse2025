#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseVelocityTargetLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nfinal_velocity_mps=" << scenario.finalVelocityMps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nfinal_velocity_mps=" << scenario.finalVelocityMps
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(RequestedForwardTargetIsPreserved)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=requested_forward_mps"
                << L"\nexpected=" << VelocityLongRunScenario::kTargetForwardMps
                << L"\nactual_final=" << scenario.finalRequestedVelMps
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedForwardPreserved,
                message.str().c_str());
        }

        TEST_METHOD(RequestedYawRateTargetIsPreserved)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=requested_yaw_rate_radps"
                << L"\nexpected=0"
                << L"\nactual_final=" << scenario.finalRequestedYawRateRadps
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.requestedYawRatePreserved,
                message.str().c_str());
        }

        TEST_METHOD(ResponseApproachesPositiveTarget)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=final_velocity_mps"
                << L"\nactual=" << scenario.finalVelocityMps
                << L"\ncriterion=actual>0.2"
                << L"\ntarget=" << VelocityLongRunScenario::kTargetForwardMps
                << L"\nfinal_requested_velocity=" << scenario.finalRequestedVelMps
                << L"\nfinal_requested_accel=" << scenario.finalRequestedAccelMps2;
            Assert::IsTrue(
                scenario.finalVelocityMps > 0.20f,
                message.str().c_str());
        }

        TEST_METHOD(FinalVelocitySettlesNearTarget)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=final_velocity_mps"
                << L"\nexpected=" << VelocityLongRunScenario::kTargetForwardMps
                << L"\nactual=" << scenario.finalVelocityMps
                << L"\ntolerance=0.15";

            Assert::AreEqual(
                VelocityLongRunScenario::kTargetForwardMps, scenario.finalVelocityMps, 0.15f,
                message.str().c_str());
        }

        TEST_METHOD(ResponseDoesNotDivergeWithWrongSign)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=minimum_velocity_mps"
                << L"\nactual=" << scenario.minimumVelocityMps
                << L"\ncriterion=actual>-0.02";

            Assert::IsTrue(
                scenario.minimumVelocityMps > -0.02f,
                message.str().c_str());
        }

        TEST_METHOD(ResponseDoesNotOvershootBeyondDiagnosticTolerance)
        {
            const VelocityLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_VELOCITY_TARGET_LONG_RUN"
                << L"\nfield=maximum_velocity_mps"
                << L"\nactual=" << scenario.maximumVelocityMps
                << L"\ncriterion=actual<target+0.3"
                << L"\ntarget=" << VelocityLongRunScenario::kTargetForwardMps;

            Assert::IsTrue(
                scenario.maximumVelocityMps < VelocityLongRunScenario::kTargetForwardMps + 0.30f,
                message.str().c_str());
        }

    };
}
