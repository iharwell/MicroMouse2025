#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseSolverFailureEvidenceTest)
    {
    public:
        TEST_METHOD(LeftCommandFallsBackToZero)
        {
            const SolverFailureScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=solver_failure_left_command"
                << L"\nexpected=0"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                scenario.command.LeftCommand(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandFallsBackToZero)
        {
            const SolverFailureScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=solver_failure_right_command"
                << L"\nexpected=0"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                0.0f,
                scenario.command.RightCommand(),
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(CommandKindFlagIsSet)
        {
            const SolverFailureScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=command_kind_flags"
                << L"\nactual=" << scenario.telemetry.commandKindFlags
                << L"\nrequired_mask=" << DriveTelemetry::kCommandKindSolverFailureEvidence;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.commandKindFlags, DriveTelemetry::kCommandKindSolverFailureEvidence),
                message.str().c_str());
        }

    };
}
