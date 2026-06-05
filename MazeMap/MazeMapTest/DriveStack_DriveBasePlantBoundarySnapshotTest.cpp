#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseBoundarySnapshotTest)
    {
    public:
        TEST_METHOD(LeftCommandIsFinite)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsFinite)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(LeftCommandIsClamped)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.LeftCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsClamped)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.RightCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=command_kind_flags"
                << L"\nactual=" << scenario.telemetry.commandKindFlags
                << L"\nrequired_mask=" << DriveTelemetry::kCommandKindBodyProposal;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.commandKindFlags, DriveTelemetry::kCommandKindBodyProposal),
                message.str().c_str());
        }

        TEST_METHOD(ProposalSequenceEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.telemetry.telemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryProposalSequenceValid;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryProposalSequenceValid),
                message.str().c_str());
        }

        TEST_METHOD(CommandEvidenceIsSet)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.telemetry.telemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryCommandEvidenceValid;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryCommandEvidenceValid),
                message.str().c_str());
        }

        TEST_METHOD(LeftPlantCommandMatchesAccelerationFeedforward)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_plant_command"
                << L"\nexpected=" << scenario.expectedPlantCommand.LeftCommand()
                << L"\nactual=" << scenario.telemetry.leftPlantCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.expectedPlantCommand.LeftCommand(),
                scenario.telemetry.leftPlantCommand,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightPlantCommandMatchesAccelerationFeedforward)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_plant_command"
                << L"\nexpected=" << scenario.expectedPlantCommand.RightCommand()
                << L"\nactual=" << scenario.telemetry.rightPlantCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.expectedPlantCommand.RightCommand(),
                scenario.telemetry.rightPlantCommand,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(LeftDriveCommandMatchesReturnedCommand)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=left_drive_command"
                << L"\nexpected=" << scenario.command.LeftCommand()
                << L"\nactual=" << scenario.telemetry.leftDriveCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.command.LeftCommand(),
                scenario.telemetry.leftDriveCommand,
                1.0e-6f,
                message.str().c_str());
        }

        TEST_METHOD(RightDriveCommandMatchesReturnedCommand)
        {
            const BoundarySnapshotScenario scenario;
            std::wstringstream message;
            message << L"DRV30_BOUNDARY_SNAPSHOT"
                << L"\nfield=right_drive_command"
                << L"\nexpected=" << scenario.command.RightCommand()
                << L"\nactual=" << scenario.telemetry.rightDriveCommand
                << L"\ntolerance=1e-6";

            Assert::AreEqual(
                scenario.command.RightCommand(),
                scenario.telemetry.rightDriveCommand,
                1.0e-6f,
                message.str().c_str());
        }

    };
}
