#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseAccelerationTargetLongRunTest)
    {
    public:
        TEST_METHOD(LeftCommandsRemainFinite)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=left_command"
                << L"\nactual_final=" << scenario.finalLeftCommand
                << L"\nmax_abs_left=" << scenario.maxAbsLeftCommand
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.leftCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainFinite)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=right_command"
                << L"\nactual_final=" << scenario.finalRightCommand
                << L"\nmax_abs_right=" << scenario.maxAbsRightCommand
                << L"\ncriterion=all samples finite";

            Assert::IsTrue(
                scenario.rightCommandsFinite,
                message.str().c_str());
        }

        TEST_METHOD(LeftCommandsRemainClamped)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=left_command_clamp"
                << L"\nactual_max_abs=" << scenario.maxAbsLeftCommand
                << L"\ncriterion=max_abs<=1";

            Assert::IsTrue(
                scenario.leftCommandsClamped,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandsRemainClamped)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=right_command_clamp"
                << L"\nactual_max_abs=" << scenario.maxAbsRightCommand
                << L"\ncriterion=max_abs<=1";

            Assert::IsTrue(
                scenario.rightCommandsClamped,
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceRemainsVisible)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.finalTelemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                scenario.commandEvidenceValid,
                message.str().c_str());
        }

        TEST_METHOD(AccelerationObjectiveIsPreserved)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=composed_forward_accel_mps2"
                << L"\nexpected=2"
                << L"\nactual_final=" << scenario.finalComposedForwardAccelMps2
                << L"\ntolerance=1e-6";

            Assert::IsTrue(
                scenario.accelerationObjectiveUnchanged,
                message.str().c_str());
        }

        TEST_METHOD(TwentyTickResponseTrendsForward)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=velocity_at_20_ticks_mps"
                << L"\ninitial=" << scenario.initialVelocityMps
                << L"\nactual=" << scenario.velocityAtMinimumHorizonMps
                << L"\ncriterion=actual>initial+0.005";

            Assert::IsTrue(
                scenario.velocityAtMinimumHorizonMps > scenario.initialVelocityMps + 0.005f,
                message.str().c_str());
        }

        TEST_METHOD(FinalResponseAccumulatesForwardVelocity)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=final_velocity_mps"
                << L"\ninitial=" << scenario.initialVelocityMps
                << L"\nactual=" << scenario.finalVelocityMps
                << L"\ncriterion=actual>initial+0.05";

            Assert::IsTrue(
                scenario.finalVelocityMps > scenario.initialVelocityMps + 0.05f,
                message.str().c_str());
        }

        TEST_METHOD(ResponseDoesNotDivergeOppositeRequest)
        {
            const AccelerationLongRunScenario scenario;
            std::wstringstream message;
            message << L"DRV30_ACCEL_TARGET_LONG_RUN"
                << L"\nfield=minimum_velocity_mps"
                << L"\nactual=" << scenario.minimumVelocityMps
                << L"\ncriterion=actual>-0.01";

            Assert::IsTrue(
                scenario.minimumVelocityMps > -0.01f,
                message.str().c_str());
        }

    };
}
