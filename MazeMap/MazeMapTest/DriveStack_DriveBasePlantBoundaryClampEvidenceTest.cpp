#include "pch.h"
#include "CppUnitTest.h"

#include "DriveStack_DriveBasePlantBoundaryTestSupport.h"

#include <cmath>
#include <sstream>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
    using namespace DriveStackDriveBasePlantBoundaryTestSupport;

    TEST_CLASS(DriveStack_DriveBaseClampEvidenceTest)
    {
    public:
        TEST_METHOD(LeftCommandIsFinite)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=left_clamp_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.LeftCommand()),
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsFinite)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=right_clamp_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=isfinite(actual)";

            Assert::IsTrue(
                std::isfinite(scenario.command.RightCommand()),
                message.str().c_str());
        }

        TEST_METHOD(LeftCommandIsClamped)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=left_clamp_command"
                << L"\nactual=" << scenario.command.LeftCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.LeftCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(RightCommandIsClamped)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=right_clamp_command"
                << L"\nactual=" << scenario.command.RightCommand()
                << L"\ncriterion=abs(actual)<=1";

            Assert::IsTrue(
                std::fabs(scenario.command.RightCommand()) <= 1.0f,
                message.str().c_str());
        }

        TEST_METHOD(PlantVsDriveClampEvidenceIsVisible)
        {
            const ClampScenario scenario;
            const float leftDelta =
                std::fabs(scenario.telemetry.leftPlantCommand - scenario.telemetry.leftDriveCommand);
            const float rightDelta =
                std::fabs(scenario.telemetry.rightPlantCommand - scenario.telemetry.rightDriveCommand);
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=plant_vs_drive_clamp_delta"
                << L"\nleft_delta=" << leftDelta
                << L"\nright_delta=" << rightDelta
                << L"\ncriterion=left_delta>1e-5||right_delta>1e-5"
                << L"\nleft_plant=" << scenario.telemetry.leftPlantCommand
                << L"\nleft_drive=" << scenario.telemetry.leftDriveCommand
                << L"\nright_plant=" << scenario.telemetry.rightPlantCommand
                << L"\nright_drive=" << scenario.telemetry.rightDriveCommand;

            Assert::IsTrue(
                (leftDelta > 1.0e-5f) ||
                (rightDelta > 1.0e-5f),
                message.str().c_str());
        }

        TEST_METHOD(PlantCommandTelemetryFlagIsSet)
        {
            const ClampScenario scenario;
            std::wstringstream message;
            message << L"DRV30_TELEMETRY_EVIDENCE"
                << L"\nfield=telemetry_valid_flags"
                << L"\nactual=" << scenario.telemetry.telemetryValidFlags
                << L"\nrequired_mask=" << DriveTelemetry::kTelemetryPlantCommandValid;

            Assert::IsTrue(
                IsFlagSet(scenario.telemetry.telemetryValidFlags, DriveTelemetry::kTelemetryPlantCommandValid),
                message.str().c_str());
        }

    };
}
