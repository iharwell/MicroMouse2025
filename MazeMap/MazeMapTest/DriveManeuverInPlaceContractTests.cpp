#include "pch.h"
#include "DriveManeuverContractTestSupport.h"

#include "CppUnitTest.h"
#include "..\MazeMap\Defines.h"
#include "..\MazeMap\DriveTelemetry.h"
#include "..\MazeMap\Maneuver.h"

#include <cmath>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap::App
{
    using namespace DriveManeuverContractTestSupport;

    TEST_CLASS(DriveManeuverIP45ContractTest)
    {
        static constexpr ManeuverCode kCode = IP45;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverIP90ContractTest)
    {
        static constexpr ManeuverCode kCode = IP90;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverIP135ContractTest)
    {
        static constexpr ManeuverCode kCode = IP135;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverIP180ContractTest)
    {
        static constexpr ManeuverCode kCode = IP180;
        static constexpr bool kSmoothTurn = false;

    public:
        TEST_METHOD(Completes)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"completion", kCode, trace);
            Assert::IsTrue(trace.completed, message.c_str());
        }

        TEST_METHOD(CommandSamplesCaptured)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_samples", kCode, trace);
            Assert::IsTrue(!trace.samples.empty(), message.c_str());
        }

        TEST_METHOD(LeftReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.leftReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(RightReturnedCommandIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_returned_command", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.rightReturnedCommandFinite, message.c_str());
        }

        TEST_METHOD(BodyProposalEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"body_proposal_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.commandKindFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kCommandKindBodyProposal);
            Assert::IsTrue(trace.bodyProposalEvidenceSet, message.c_str());
        }

        TEST_METHOD(CommandTelemetryEvidenceIsSet)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"command_telemetry_evidence", kCode, trace) +
                L" actual_flags=" + std::to_wstring(trace.lastTelemetry.telemetryValidFlags) +
                L" required_mask=" + std::to_wstring(DriveTelemetry::kTelemetryCommandEvidenceValid);
            Assert::IsTrue(trace.commandTelemetryEvidenceSet, message.c_str());
        }

        TEST_METHOD(LeftDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"left_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.LeftCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.leftDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.leftCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RightDriveEvidenceMatchesCommand)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"right_drive_evidence", kCode, trace) +
                L" expected=" + std::to_wstring(trace.lastReturnedCommand.RightCommand()) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.rightDriveCommand) +
                L" tolerance=1e-6";
            Assert::IsTrue(trace.rightCommandEvidenceMatchesReturnedCommand, message.c_str());
        }

        TEST_METHOD(RequestedForwardIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_mps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardMps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardMpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rate_radps", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRateRadps) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRateRadpsFinite, message.c_str());
        }

        TEST_METHOD(RequestedForwardAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_forward_accel_mps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedForwardAccelMps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedForwardAccelMps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawAccelIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_accel_radps2", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawAccelRadps2) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawAccelRadps2Finite, message.c_str());
        }

        TEST_METHOD(RequestedYawIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const std::wstring message = BuildManeuverMessage(L"requested_yaw_rad", kCode, trace) +
                L" actual=" + std::to_wstring(trace.lastTelemetry.requestedYawRad) +
                L" criterion=isfinite(actual)";
            Assert::IsTrue(trace.requestedYawRadFinite, message.c_str());
        }

        TEST_METHOD(TruthPositionXIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionX();
            const std::wstring message = BuildManeuverMessage(L"truth_position_x", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthPositionYIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetPositionY();
            const std::wstring message = BuildManeuverMessage(L"truth_position_y", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthHeadingIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetHeading();
            const std::wstring message = BuildManeuverMessage(L"truth_heading", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthForwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetForwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_forward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightwardVelocityIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetRightwardVelocity();
            const std::wstring message = BuildManeuverMessage(L"truth_rightward_velocity", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthYawRateIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetYawRate();
            const std::wstring message = BuildManeuverMessage(L"truth_yaw_rate", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthLeftWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedLeft();
            const std::wstring message = BuildManeuverMessage(L"truth_left_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }

        TEST_METHOD(TruthRightWheelSpeedIsFinite)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float actual = trace.truthState.GetWheelSpeedRight();
            const std::wstring message = BuildManeuverMessage(L"truth_right_wheel_speed", kCode, trace) +
                L" actual=" + std::to_wstring(actual) + L" criterion=isfinite(actual)";
            Assert::IsTrue(std::isfinite(actual), message.c_str());
        }


        TEST_METHOD(ShiftWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float shiftMeters = ComputeInPlaceShiftMeters(trace);
            const std::wstring message = BuildManeuverMessage(L"shift", kCode, trace) +
                L" actual_m=" + std::to_wstring(shiftMeters) +
                L" limit_m=" + std::to_wstring(kInPlacePositionToleranceM);
            Assert::IsTrue(shiftMeters < kInPlacePositionToleranceM, message.c_str());
        }

        TEST_METHOD(HeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeInPlaceHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

        TEST_METHOD(DurationWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float expectedTimeSeconds = ComputeInPlaceExpectedTimeSeconds(kCode);
            const float relativeError = ComputeInPlaceRelativeTimeError(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"time", kCode, trace) +
                L" elapsed_s=" + std::to_wstring(trace.elapsedSeconds) +
                L" expected_s=" + std::to_wstring(expectedTimeSeconds) +
                L" rel_err=" + std::to_wstring(relativeError) +
                L" limit=" + std::to_wstring(kTimeToleranceFraction);
            Assert::IsTrue(relativeError <= kTimeToleranceFraction, message.c_str());
        }

    };
}
