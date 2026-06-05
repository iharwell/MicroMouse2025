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

    TEST_CLASS(DriveManeuverS180LSContractTest)
    {
        static constexpr ManeuverCode kCode = S180LS;
        static constexpr bool kSmoothTurn = true;

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


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };

    TEST_CLASS(DriveManeuverS180SSContractTest)
    {
        static constexpr ManeuverCode kCode = S180SS;
        static constexpr bool kSmoothTurn = true;

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


        TEST_METHOD(VelocityStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothVelocityNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"velocity", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kVelocityVariationLimit);
            Assert::IsTrue(normalizedSpan < kVelocityVariationLimit, message.c_str());
        }

        TEST_METHOD(YawAccelerationStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawAccelerationNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_accel", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawAccelerationVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawAccelerationVariationLimit, message.c_str());
        }

        TEST_METHOD(YawRateStable)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float normalizedSpan = ComputeSmoothYawRateNormalizedSpan(trace);
            const std::wstring message = BuildManeuverMessage(L"yaw_rate", kCode, trace) +
                L" span=" + std::to_wstring(normalizedSpan) +
                L" limit=" + std::to_wstring(kYawRateVariationLimit);
            Assert::IsTrue(normalizedSpan < kYawRateVariationLimit, message.c_str());
        }

        TEST_METHOD(FinalPositionWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float positionErrorMeters = ComputeSmoothFinalPositionErrorMeters(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"position", kCode, trace) +
                L" error_m=" + std::to_wstring(positionErrorMeters) +
                L" limit_m=" + std::to_wstring(kSmoothPositionToleranceM);
            Assert::IsTrue(positionErrorMeters <= kSmoothPositionToleranceM, message.c_str());
        }

        TEST_METHOD(FinalHeadingWithinTolerance)
        {
            const ManeuverExecutionTrace trace = SimulateDriveManeuver(kCode, kSmoothTurn);
            const float headingErrorRad = ComputeSmoothFinalHeadingErrorRad(kCode, trace);
            const std::wstring message = BuildManeuverMessage(L"heading", kCode, trace) +
                L" error_deg=" + std::to_wstring(headingErrorRad * RAD_TO_DEG_F) +
                L" limit_deg=" + std::to_wstring(kHeadingToleranceRad * RAD_TO_DEG_F);
            Assert::IsTrue(headingErrorRad <= kHeadingToleranceRad, message.c_str());
        }

    };
}
