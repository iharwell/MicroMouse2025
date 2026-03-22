#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\DiagnosticCoverage.h"
#include "..\MazeMap\DiagnosticLogBudget.h"
#include "..\MazeMap\DiagnosticMotionPlan.h"
#include "..\MazeMap\EncoderStallPolicy.h"
#include "..\MazeMap\FanRampProfile.h"
#include "..\MazeMap\GyroBiasUpdatePolicy.h"
#include "..\MazeMap\ImuCalibrationPolicy.h"
#include "..\MazeMap\ImuSamplingProfile.h"
#include "..\MazeMap\InPlaceTurnProfile.h"
#include "..\MazeMap\LaunchAssistProfile.h"
#include "..\MazeMap\Maze.h"
#include "..\MazeMap\MissionMazeExport.h"
#include "..\MazeMap\MissionStartPolicy.h"
#include "..\MazeMap\MotorEncoderDrive.h"
#include "..\MazeMap\MotorModelUnits.h"
#include "..\MazeMap\MotionTargetProjection.h"
#include "..\MazeMap\CruiseSpeedFloor.h"
#include "..\MazeMap\OpenLoopDriveCommand.h"
#include "..\MazeMap\RollingAverageWindow.h"
#include "..\MazeMap\StartupWaitProfile.h"
#include "..\MazeMap\TrackWidthEstimate.h"
#include "..\MazeMap\TractionLimitSweep.h"
#include "..\MazeMap\TurnCommandGeometry.h"
#include "..\MazeMap\TurnWallEdgeTracker.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\DiagonalWallCentering.h"
#include "..\MazeMap\WallDetectionThresholds.h"
#include "..\MazeMap\WheelControlProfile.h"
#include "..\MazeMap\ManeuverSet.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	namespace
	{
		bool SummaryContains(const char* token)
		{
			for (size_t index = 0U; index < GetDiagnosticSummaryInstructionCount(); ++index)
			{
				const DiagnosticSummaryInstruction& instruction = GetDiagnosticSummaryInstruction(index);
				if (std::strstr(instruction.message, token) != nullptr)
				{
					return true;
				}
			}

			return false;
		}
	}

	TEST_CLASS(DiagnosticCoverageTest)
	{
	public:
		TEST_METHOD(SummaryInstructionsStayWithinEventBudget)
		{
			for (size_t index = 0U; index < GetDiagnosticSummaryInstructionCount(); ++index)
			{
				const DiagnosticSummaryInstruction& instruction = GetDiagnosticSummaryInstruction(index);
				Assert::IsTrue(std::strlen(instruction.message) <= 150U);
			}
		}

		TEST_METHOD(DiagnosticEventBudgetFitsCircleResultPayload)
		{
			static constexpr char kEventType[] = "circle_result";
			static constexpr char kMessage[] =
				"circle_cw_fast;cruise_mps=0.450;l_cnt=-123456;r_cnt=123456;enc_yaw_deg=-360.0;"
				"avg_speed_mps=0.450;avg_omega_radps=-5.123;est_lat_mps2=2.305;avg_lat_mps2=2.101;peak_lat_mps2=3.250";

			Assert::IsTrue(DiagnosticEventLineFits(std::strlen(kEventType), std::strlen(kMessage)));
		}

		TEST_METHOD(DiagnosticEventBudgetFitsTurningTractionResultPayload)
		{
			static constexpr char kEventType[] = "traction_limit_result";
			static constexpr char kMessage[] =
				"reason=traction_loss;slip=1;elapsed_ms=18450;cmd_v_mps=0.742;cmd_w_radps=-4.122;"
				"enc_v_mps=0.705;enc_w_radps=-3.001;pred_lat_mps2=2.116;planar_accel_mps2=1.221;"
				"yaw_ratio=0.642;planar_ratio=0.577";

			Assert::IsTrue(DiagnosticEventLineFits(std::strlen(kEventType), std::strlen(kMessage)));
		}

		TEST_METHOD(DiagnosticEventBudgetFitsCorridorRepeatabilityResultPayload)
		{
			static constexpr char kEventType[] = "corridor_repeatability_result";
			static constexpr char kMessage[] =
				"speed_index=3;cruise_mps=0.750;dx_m=-0.012345;dy_m=0.023456;closure_m=0.026507;"
				"yaw_err_deg=-3.210;left_delta_m=1.523456;right_delta_m=1.512345;"
				"left_delta_cnt=123456;right_delta_cnt=123012";

			Assert::IsTrue(DiagnosticEventLineFits(std::strlen(kEventType), std::strlen(kMessage)));
		}

		TEST_METHOD(DiagnosticEventBudgetFitsPositionAuditSmoothTurnPayload)
		{
			static constexpr char kEventType[] = "position_smooth_turn_result";
			static constexpr char kMessage[] =
				"code=S90LS;speed_idx=2;v=0.800;nominal_radius_m=0.153000;measured_radius_m=0.148200;"
				"effective_track_width_m=0.078900;yaw_err_deg=-2.340;corridor_err_m=0.006500;east_touch_correction_m=0.018000";

			Assert::IsTrue(DiagnosticEventLineFits(std::strlen(kEventType), std::strlen(kMessage)));
		}

		TEST_METHOD(TurnWallEdgeTrackerOnlyReportsRisingEdgesAfterInitialization)
		{
			TurnWallEdgeTracker tracker{};

			ObserveTurnWallStates(tracker, true, false);
			Assert::IsTrue(tracker.initialized);
			Assert::IsFalse(tracker.leftWallRose);
			Assert::IsFalse(tracker.rightWallRose);

			ObserveTurnWallStates(tracker, true, true);
			Assert::IsFalse(tracker.leftWallRose);
			Assert::IsTrue(tracker.rightWallRose);

			ObserveTurnWallStates(tracker, false, true);
			ObserveTurnWallStates(tracker, true, true);
			Assert::IsTrue(tracker.leftWallRose);
			Assert::IsTrue(tracker.rightWallRose);
		}

		TEST_METHOD(SummaryInstructionsCoverResultFamilies)
		{
			Assert::IsTrue(SummaryContains("kickoff_*"));
			Assert::IsTrue(SummaryContains("forward_*"));
			Assert::IsTrue(SummaryContains("turn_result"));
			Assert::IsTrue(SummaryContains("straight_result"));
			Assert::IsTrue(SummaryContains("arc_result"));
			Assert::IsTrue(SummaryContains("arc_circle_result"));
			Assert::IsTrue(SummaryContains("circle_result"));
			Assert::IsTrue(SummaryContains("square_result"));
		}

		TEST_METHOD(SummaryInstructionsCoverTunableGroups)
		{
			static const char* requiredTokens[] = {
				"kGyroBiasSamples",
				"kGyroBiasUpdateMaxAbsRateRadps",
				"kWheelStaticFeedforward",
				"kWheelRestLaunchDriveCommand",
				"kWheelRestLaunchMaxDriveCommand",
				"kWheelRestLaunchRampMs",
				"kWheelRestLaunchSpeedThresholdMps",
				"kWheelRestLaunchDriveThreshold",
				"kWheelVelocityFeedforward",
				"kWheelVelocityKp",
				"kWheelVelocityKi",
				"kWheelIntegralLimit",
				"kDiagnosticWheelVelocityKpScale",
				"kDiagnosticWheelVelocityKiScale",
				"kDiagnosticWheelIntegralLimitScale",
				"kTurnHeadingKp",
				"kTurnYawD",
				"kAngleToleranceRad",
				"kAngularSpeedToleranceRadps",
				"kStraightHeadingKp",
				"kStraightYawD",
				"kDistanceToleranceM",
				"kSpeedToleranceMps",
				"kEncoderProgressEpsilonM",
				"kEncoderStallCommandThresholdMps",
				"kEncoderStallTimeoutMs",
				"kEncoderStallStartupGraceMs",
				"kArcHeadingKp",
				"kArcYawD",
				"kTrackWidthM",
			};

			for (const char* token : requiredTokens)
			{
				Assert::IsTrue(SummaryContains(token));
			}
		}

		TEST_METHOD(SelectDiagnosticReturnDistancePrefersMeasuredEncoderTravel)
		{
			const float selectedDistanceM = SelectDiagnosticReturnDistanceM(0.180f, 0.193f);
			Assert::IsTrue(std::fabs(selectedDistanceM - 0.193f) < 1.0e-6f);
		}

		TEST_METHOD(SelectDiagnosticReturnDistanceFallsBackToNominalDistance)
		{
			const float fromZeroM = SelectDiagnosticReturnDistanceM(0.180f, 0.0f);
			const float fromNanM = SelectDiagnosticReturnDistanceM(0.180f, std::numeric_limits<float>::quiet_NaN());
			Assert::IsTrue(std::fabs(fromZeroM - 0.180f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(fromNanM - 0.180f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeDiagnosticCharacterizationTravelLimitUsesBoundaryReserve)
		{
			const float limitM = ComputeDiagnosticCharacterizationTravelLimitM(0.340f, 0.140f);
			Assert::IsTrue(std::fabs(limitM - 0.200f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeDiagnosticCharacterizationTravelLimitClampsInvalidInputs)
		{
			const float zeroLimitM = ComputeDiagnosticCharacterizationTravelLimitM(0.100f, 0.140f);
			const float noReserveLimitM = ComputeDiagnosticCharacterizationTravelLimitM(0.340f, std::numeric_limits<float>::quiet_NaN());
			Assert::IsTrue(std::fabs(zeroLimitM) < 1.0e-6f);
			Assert::IsTrue(std::fabs(noReserveLimitM - 0.340f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeFanRampDutyCycleStartsAtZeroAndReachesTargetAtDeadline)
		{
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(0.80f, 0UL, 2000UL)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(0.80f, 2000UL, 2000UL) - 0.80f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(0.80f, 3000UL, 2000UL) - 0.80f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeLaunchAssistDriveFloorRampsFromBaseToMaxCommand)
		{
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(0.30f, 0.55f, 0UL, 250UL) - 0.30f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(0.30f, 0.55f, 125UL, 250UL) - 0.425f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(0.30f, 0.55f, 250UL, 250UL) - 0.55f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(0.30f, 0.55f, 500UL, 250UL) - 0.55f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeLaunchAssistDriveFloorClampsInvalidInputs)
		{
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(std::numeric_limits<float>::quiet_NaN(), 0.55f, 125UL, 250UL)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(0.60f, 0.55f, 125UL, 250UL) - 0.60f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeLaunchAssistDriveFloor(0.30f, 1.20f, 0UL, 0UL) - 1.0f) < 1.0e-6f);
		}

		TEST_METHOD(VehiclePhysicalModelMatchesSharedGeometryAndKinematicFit)
		{
			const VehiclePhysicalModel& model = Vehicle::GetPhysicalModel();
			Assert::IsTrue(std::fabs(model.massKg - 0.14f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.lengthM - 0.1085f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.frontWallContactOffsetM - 0.056f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.trackWidthM - 0.08203f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.trackWidthPhysicalMinM - 0.07004f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.trackWidthPhysicalMaxM - 0.07868f) < 1.0e-6f);
		}

		TEST_METHOD(MotorEncoderDriveSharedModelMatchesMeasuredDrivetrain)
		{
			const MotorEncoderDrivePhysicalModel& model = MotorEncoderDrive::GetSharedPhysicalModel();
			Assert::IsTrue(std::fabs(model.nominalVoltageV - 6.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.nominalNoLoadSpeedRpm - 14100.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.supplyVoltageV - 8.4f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.resistanceOhms - 4.31f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.torqueConstantNmPerA - MilliNewtonMetersToNewtonMeters(3.96f)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.noLoadCurrentA - MilliAmpsToAmps(45.9f)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.speedConstantRadpsPerVolt - ComputeMotorSpeedConstantRadpsPerVolt(14100.0f, 6.0f, MilliAmpsToAmps(45.9f), 4.31f)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.gearRatio - (56.0f / 17.0f)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.wheelDiameterM - 0.025345f) < 1.0e-6f);
			Assert::AreEqual(4096U, static_cast<unsigned>(model.pulsesPerRev));
		}

		TEST_METHOD(MotorEncoderDriveDefaultFactoriesUseSharedModelAndHardwareMap)
		{
			const MotorEncoderDrivePhysicalModel& model = MotorEncoderDrive::GetSharedPhysicalModel();
			MotorEncoderDrive left = MotorEncoderDrive::CreateDefaultLeftDrive();
			MotorEncoderDrive right = MotorEncoderDrive::CreateDefaultRightDrive();

			Assert::IsTrue(std::fabs(left.getWheelDiameter() - model.wheelDiameterM) < 1.0e-6f);
			Assert::IsTrue(std::fabs(right.getWheelDiameter() - model.wheelDiameterM) < 1.0e-6f);
			Assert::IsTrue(std::fabs(left.getGearRatio() - model.gearRatio) < 1.0e-6f);
			Assert::IsTrue(std::fabs(right.getGearRatio() - model.gearRatio) < 1.0e-6f);
			Assert::AreEqual(2U, static_cast<unsigned>(left.getEncoderChannel()));
			Assert::AreEqual(1U, static_cast<unsigned>(right.getEncoderChannel()));
			Assert::IsTrue(left.getInvertMotorDirection());
			Assert::IsTrue(right.getInvertMotorDirection());
			Assert::IsFalse(left.getInvertEncoderDirection());
			Assert::IsFalse(right.getInvertEncoderDirection());
		}

		TEST_METHOD(VehicleDefaultModelUsesMeasuredTurnEnvelope)
		{
			Vehicle vehicle;
			Assert::IsTrue(std::fabs(vehicle.GetMaxLateralAcceleration() - 16.5f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(vehicle.GetMaxRotationalVelocity() - 9.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(vehicle.GetMaxAngularAcceleration() - 45.0f) < 1.0e-6f);
		}

		TEST_METHOD(TryComputeEffectiveTrackWidthMUsesEncoderDifferentialOverYaw)
		{
			float effectiveTrackWidthM = 0.0f;
			Assert::IsTrue(TryComputeEffectiveTrackWidthM(-0.125f, 0.125f, PI_F, effectiveTrackWidthM));
			Assert::AreEqual(0.079577f, effectiveTrackWidthM, 0.00001f);
		}

		TEST_METHOD(ManeuverNominalTurnRadiusExposureMatchesSmoothTurnDefinitions)
		{
			Assert::AreEqual(63.0f / 180.0f, ManeuverSet::GetSet()[S90SS].GetNominalTurnRadiusInCells(), 1.0e-6f);
			Assert::AreEqual(153.0f / 180.0f, ManeuverSet::GetSet()[S90LS].GetNominalTurnRadiusInCells(), 1.0e-6f);
		}

		TEST_METHOD(CodeDegreesUsesRightTurnSignForUnmirroredSmoothTurns)
		{
			Assert::AreEqual(static_cast<int>(-90), static_cast<int>(CodeDegrees(S90SS)));
			Assert::AreEqual(static_cast<int>(90), static_cast<int>(CodeDegrees(S90SS_M)));
			Assert::AreEqual(static_cast<int>(-90), static_cast<int>(CodeDegrees(S90LS)));
			Assert::AreEqual(static_cast<int>(90), static_cast<int>(CodeDegrees(S90LS_M)));
		}

		TEST_METHOD(VehicleTurnSpeedRespectsLateralAndMaxSpeedLimits)
		{
			Vehicle vehicle;
			vehicle.SetMaxLateralAcceleration(4.5f);
			vehicle.SetMaxSpeed(0.80f);

			Assert::IsTrue(std::fabs(vehicle.GetTurnSpeed(0.04f) - std::sqrt(0.18f)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(vehicle.GetTurnSpeed(1.00f) - 0.80f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(vehicle.GetSpeedFromCurvature(25.0f) - std::sqrt(0.18f)) < 1.0e-6f);
		}

		TEST_METHOD(VehicleInPlaceTurnTimeUsesAngularAccelerationProfile)
		{
			Vehicle vehicle;
			vehicle.SetMaxAngularAcceleration(45.0f);

			const float quarterTurn = vehicle.GetInPlaceTurnTime(0.25f * PI_F);
			const float halfTurn = vehicle.GetInPlaceTurnTime(PI_F);
			const float expectedQuarterTurn = 2.0f * std::sqrt((0.25f * PI_F) / 45.0f);
			const float expectedHalfTurn = 2.0f * std::sqrt(PI_F / 45.0f);

			Assert::IsTrue(std::fabs(quarterTurn - expectedQuarterTurn) < 1.0e-6f);
			Assert::IsTrue(std::fabs(halfTurn - expectedHalfTurn) < 1.0e-6f);
			Assert::IsTrue(halfTurn > quarterTurn);
		}

		TEST_METHOD(TryComputeProjectedDistanceToTargetMProjectsForwardTravel)
		{
			float projectedDistanceM = 0.0f;
			Assert::IsTrue(TryComputeProjectedDistanceToTargetM(0.0900f, 0.0585f, 0.0900f, 0.0900f, 0.0f, 1.0f, projectedDistanceM));
			Assert::IsTrue(std::fabs(projectedDistanceM - 0.0315f) < 1.0e-6f);
		}

		TEST_METHOD(TryComputeProjectedDistanceToTargetMProjectsReverseTravel)
		{
			float projectedDistanceM = 0.0f;
			Assert::IsTrue(TryComputeProjectedDistanceToTargetM(0.1180f, 0.0927f, 0.0900f, 0.0927f, -1.0f, 0.0f, projectedDistanceM));
			Assert::IsTrue(std::fabs(projectedDistanceM - 0.0280f) < 1.0e-6f);
		}

		TEST_METHOD(TryComputeProjectedDistanceToTargetMRejectsInvalidInputs)
		{
			float projectedDistanceM = 1.0f;
			Assert::IsFalse(TryComputeProjectedDistanceToTargetM(
				std::numeric_limits<float>::quiet_NaN(),
				0.0f,
				0.1f,
				0.2f,
				0.0f,
				1.0f,
				projectedDistanceM));
			Assert::IsFalse(TryComputeProjectedDistanceToTargetM(
				0.0f,
				0.0f,
				0.1f,
				0.2f,
				0.0f,
				0.0f,
				projectedDistanceM));
		}

		TEST_METHOD(OpenLoopDriveCommandHelpersBuildSymmetricAndDifferentialCommands)
		{
			const OpenLoopDriveCommand symmetric = MakeSymmetricOpenLoopDriveCommand(0.42f);
			Assert::AreEqual(0.42f, symmetric.leftDriveCommand, 1.0e-6f);
			Assert::AreEqual(0.42f, symmetric.rightDriveCommand, 1.0e-6f);

			const OpenLoopDriveCommand differential = MakeDifferentialOpenLoopDriveCommand(0.40f, 0.15f);
			Assert::AreEqual(0.25f, differential.leftDriveCommand, 1.0e-6f);
			Assert::AreEqual(0.55f, differential.rightDriveCommand, 1.0e-6f);
		}

		TEST_METHOD(ClampOpenLoopDriveCommandLimitsAndRejectsInvalidInputs)
		{
			const OpenLoopDriveCommand clamped = ClampOpenLoopDriveCommand(MakeOpenLoopDriveCommand(-1.25f, 1.40f));
			Assert::AreEqual(-1.0f, clamped.leftDriveCommand, 1.0e-6f);
			Assert::AreEqual(1.0f, clamped.rightDriveCommand, 1.0e-6f);

			const OpenLoopDriveCommand invalid = ClampOpenLoopDriveCommand(
				MakeOpenLoopDriveCommand(std::numeric_limits<float>::quiet_NaN(), 0.25f));
			Assert::AreEqual(0.0f, invalid.leftDriveCommand, 1.0e-6f);
			Assert::AreEqual(0.0f, invalid.rightDriveCommand, 1.0e-6f);
		}

		TEST_METHOD(TryComputeLinearWallSignalDistanceThresholdMUsesInverseSquareScaling)
		{
			float thresholdDistanceM = 0.0f;
			Assert::IsTrue(TryComputeLinearWallSignalDistanceThresholdM(0.0550f, 1.0f / 6.0f, thresholdDistanceM));
			Assert::AreEqual(0.1347219f, thresholdDistanceM, 0.0001f);
		}

		TEST_METHOD(TryComputeInverseSquareDistanceFromReferenceSignalReconstructsDistance)
		{
			float distanceM = 0.0f;
			Assert::IsTrue(TryComputeInverseSquareDistanceFromReferenceSignal(0.060f, 0.240f, 0.0550f, distanceM));
			Assert::AreEqual(0.1100f, distanceM, 0.0001f);
		}

		TEST_METHOD(TryComputeSignalHighThresholdsUsesCalibrationSignalFractions)
		{
			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			Assert::IsTrue(TryComputeSignalHighThresholds(0.240f, 1.0f / 6.0f, 1.0f / 8.0f, onMeasuredThreshold, offMeasuredThreshold));
			Assert::AreEqual(0.040f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.030f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(RollingAverageWindowUsesLastControlLoopSamples)
		{
			RollingAverageWindow<4U> window{};
			Assert::AreEqual(1.0f, window.Push(1.0f), 1.0e-6f);
			Assert::AreEqual(1.5f, window.Push(2.0f), 1.0e-6f);
			Assert::AreEqual(2.0f, window.Push(3.0f), 1.0e-6f);
			Assert::AreEqual(2.5f, window.Push(4.0f), 1.0e-6f);
			Assert::AreEqual(4.75f, window.Push(10.0f), 1.0e-6f);
		}

		TEST_METHOD(WallSensorCalibrationCurveSupportsDenseMovingFrontSweep)
		{
			WallSensorCalibrationCurve curve{};
			for (uint8_t index = 0U; index < 16U; ++index)
			{
				const float measuredValue = 0.050f + (0.010f * static_cast<float>(index));
				const float actualDistanceM = 0.040f + (0.002f * static_cast<float>(index));
				Assert::IsTrue(curve.AddPoint(measuredValue, actualDistanceM, 100.0f + static_cast<float>(index)));
			}

			Assert::AreEqual(static_cast<uint8_t>(16U), curve.GetCount());
		}

		TEST_METHOD(TryComputeNormalizedWallSignalBalanceErrorUsesReferenceNormalizedSignals)
		{
			float balanceError = 0.0f;
			Assert::IsTrue(TryComputeNormalizedWallSignalBalanceError(0.120f, 0.240f, 0.120f, 0.240f, 1.0f / 12.0f, balanceError));
			Assert::AreEqual(0.0f, balanceError, 1.0e-6f);

			Assert::IsTrue(TryComputeNormalizedWallSignalBalanceError(0.180f, 0.240f, 0.060f, 0.240f, 1.0f / 12.0f, balanceError));
			Assert::AreEqual(0.5f, balanceError, 1.0e-6f);

			Assert::IsFalse(TryComputeNormalizedWallSignalBalanceError(0.010f, 0.240f, 0.120f, 0.240f, 1.0f / 12.0f, balanceError));
		}

		TEST_METHOD(TryComputeSignedTurnAngleRadUsesShortestSignedTurn)
		{
			float angleRad = 0.0f;
			Assert::IsTrue(TryComputeSignedTurnAngleRad(0.5f * PI_F, 0.0f, angleRad));
			Assert::AreEqual(-0.5f * PI_F, angleRad, 1.0e-6f);

			Assert::IsTrue(TryComputeSignedTurnAngleRad(179.0f * DEG_TO_RAD, -179.0f * DEG_TO_RAD, angleRad));
			Assert::AreEqual(static_cast<float>(2.0f * DEG_TO_RAD), angleRad, 1.0e-6f);
		}

		TEST_METHOD(InPlaceTurnProfileUsesSharedCompletionAndCommandLaw)
		{
			InPlaceTurnProfile profile{};
			profile.maxAngularSpeedRadps = 9.0f;
			profile.angularAccelRadps2 = 45.0f;
			profile.headingKp = 7.5f;
			profile.yawD = 0.12f;
			profile.angleToleranceRad = 0.75f * DEG_TO_RAD;
			profile.angularSpeedToleranceRadps = 0.10f;

			Assert::IsTrue(IsInPlaceTurnComplete(0.50f * DEG_TO_RAD, 0.08f, profile));
			Assert::IsFalse(IsInPlaceTurnComplete(1.00f * DEG_TO_RAD, 0.08f, profile));
			Assert::IsFalse(IsInPlaceTurnComplete(0.50f * DEG_TO_RAD, 0.12f, profile));

			float commandedOmegaRadps = 0.0f;
			float angularCommandRadps = 0.0f;
			Assert::IsTrue(TryComputeInPlaceTurnCommandRadps(
				0.50f * PI_F,
				0.0f,
				0.010f,
				profile,
				commandedOmegaRadps,
				angularCommandRadps));
			Assert::IsTrue(commandedOmegaRadps > 0.0f);
			Assert::IsTrue(angularCommandRadps > 0.0f);
		}

		TEST_METHOD(ApplyMinimumCruiseSpeedFloorRaisesOnlyNonzeroRequests)
		{
			Assert::AreEqual(0.0f, ApplyMinimumCruiseSpeedFloor(0.0f, 0.0676f, 0.40f), 1.0e-6f);
			Assert::AreEqual(0.0676f, ApplyMinimumCruiseSpeedFloor(0.040f, 0.0676f, 0.40f), 1.0e-6f);
			Assert::AreEqual(0.1200f, ApplyMinimumCruiseSpeedFloor(0.1200f, 0.0676f, 0.40f), 1.0e-6f);
			Assert::AreEqual(0.4000f, ApplyMinimumCruiseSpeedFloor(0.5000f, 0.0676f, 0.40f), 1.0e-6f);
		}

		TEST_METHOD(TryComputeFrontWallHalfwayIntoAdjacentDistanceMUsesCellGeometry)
		{
			float thresholdDistanceM = 0.0f;
			Assert::IsTrue(TryComputeFrontWallHalfwayIntoAdjacentDistanceM(0.1800f, 0.0120f, 0.04223f, thresholdDistanceM));
			Assert::AreEqual(0.13177f, thresholdDistanceM, 0.00001f);
		}

		TEST_METHOD(TryClampWallThresholdDistanceRangeMLimitsAggressiveFrontThresholds)
		{
			float onThresholdM = 0.0f;
			float offThresholdM = 0.0f;
			Assert::IsTrue(TryClampWallThresholdDistanceRangeM(0.13177f, 0.14677f, 0.0850f, 0.1000f, onThresholdM, offThresholdM));
			Assert::AreEqual(0.0850f, onThresholdM, 0.00001f);
			Assert::AreEqual(0.1000f, offThresholdM, 0.00001f);
		}

		TEST_METHOD(TryFitAmbientAwareLogDifferentialSignalModelFitsFrontSensorResponse)
		{
			WallSensorCalibrationCurve curve{};
			Assert::IsTrue(curve.AddPoint(0.0957005f, 0.0550f, 0.0900f));
			Assert::IsTrue(curve.AddPoint(0.1082337f, 0.0480f, 0.1100f));
			Assert::IsTrue(curve.AddPoint(0.1302973f, 0.0400f, 0.1300f));
			Assert::IsTrue(curve.AddPoint(0.1748982f, 0.0280f, 0.1800f));

			float gain = 0.0f;
			float lightScale = 0.0f;
			Assert::IsTrue(TryFitAmbientAwareLogDifferentialSignalModel(curve, gain, lightScale));
			Assert::AreEqual(0.18f, gain, 0.005f);
			Assert::AreEqual(0.0035f, lightScale, 0.0003f);
		}

		TEST_METHOD(TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModelRespondsToAmbientLevel)
		{
			float measuredValue = 0.0f;
			Assert::IsTrue(TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(0.18f, 0.0035f, 0.15f, 0.0450f, measuredValue));
			Assert::AreEqual(0.1008498f, measuredValue, 0.0005f);
			const float nominalAmbientMeasuredValue = measuredValue;

			Assert::IsTrue(TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(0.18f, 0.0035f, 0.15f, 0.0850f, measuredValue));
			Assert::AreEqual(0.0343908f, measuredValue, 0.0005f);

			float higherAmbientMeasuredValue = 0.0f;
			Assert::IsTrue(TryComputeMeasuredValueForActualDistanceUsingAmbientAwareLogSignalModel(0.18f, 0.0035f, 0.30f, 0.0450f, higherAmbientMeasuredValue));
			Assert::AreEqual(0.0508513f, higherAmbientMeasuredValue, 0.0005f);
			Assert::IsTrue(higherAmbientMeasuredValue < nominalAmbientMeasuredValue);
		}

		TEST_METHOD(ShouldReleaseWallTouchSeatRequiresMinimumSkidDuration)
		{
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 99UL, 100UL, true));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(0.79f, 0.80f, 150UL, 100UL, true));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 150UL, 100UL, false));
			Assert::IsTrue(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 100UL, 100UL, true));
		}

		TEST_METHOD(ComputeFanRampDutyCycleInterpolatesAndClampsInputs)
		{
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(0.80f, 500UL, 2000UL) - 0.20f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(1.20f, 1000UL, 2000UL) - 0.50f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(std::numeric_limits<float>::quiet_NaN(), 1000UL, 2000UL)) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeFanRampDutyCycle(0.80f, 0UL, 0UL) - 0.80f) < 1.0e-6f);
		}

		TEST_METHOD(ShouldUpdateGyroBiasFromStationarySampleUsesAbsoluteRateThreshold)
		{
			Assert::IsTrue(ShouldUpdateGyroBiasFromStationarySample(0.018f, 0.020f));
			Assert::IsTrue(ShouldUpdateGyroBiasFromStationarySample(-0.018f, 0.020f));
			Assert::IsFalse(ShouldUpdateGyroBiasFromStationarySample(0.021f, 0.020f));
			Assert::IsFalse(ShouldUpdateGyroBiasFromStationarySample(-0.021f, 0.020f));
		}

		TEST_METHOD(ShouldUpdateGyroBiasFromStationarySampleRejectsInvalidInputs)
		{
			Assert::IsFalse(ShouldUpdateGyroBiasFromStationarySample(std::numeric_limits<float>::quiet_NaN(), 0.020f));
			Assert::IsFalse(ShouldUpdateGyroBiasFromStationarySample(0.005f, 0.0f));
			Assert::IsFalse(ShouldUpdateGyroBiasFromStationarySample(0.005f, std::numeric_limits<float>::quiet_NaN()));
		}

		TEST_METHOD(ComputeGyroBiasSampleCountRespectsMinimumAveragingWindow)
		{
			Assert::AreEqual(300UL, ComputeGyroBiasSampleCount(300UL, 2UL, 500UL));
			Assert::AreEqual(250UL, ComputeGyroBiasSampleCount(100UL, 2UL, 500UL));
			Assert::AreEqual(125UL, ComputeGyroBiasSampleCount(100UL, 4UL, 500UL));
		}

		TEST_METHOD(HaveEncoderCountsChangedFlagsEitherWheelMotion)
		{
			const EncoderCountPair start{ 100, -50 };

			Assert::IsFalse(HaveEncoderCountsChanged(start, EncoderCountPair{ 100, -50 }));
			Assert::IsTrue(HaveEncoderCountsChanged(start, EncoderCountPair{ 101, -50 }));
			Assert::IsTrue(HaveEncoderCountsChanged(start, EncoderCountPair{ 100, -49 }));
		}

		TEST_METHOD(AccelSelfTestValidationMatchesDatasheetBand)
		{
			Assert::IsTrue(IsAccelSelfTestDeltaValidMg(50.0f));
			Assert::IsTrue(IsAccelSelfTestDeltaValidMg(1700.0f));
			Assert::IsFalse(IsAccelSelfTestDeltaValidMg(49.9f));
			Assert::IsFalse(IsAccelSelfTestDeltaValidMg(1700.1f));
		}

		TEST_METHOD(GyroSelfTestValidationMatchesSupportedDatasheetBands)
		{
			Assert::IsTrue(IsGyroSelfTestDeltaValidDps(20.0f, 250.0f));
			Assert::IsTrue(IsGyroSelfTestDeltaValidDps(80.0f, 250.0f));
			Assert::IsFalse(IsGyroSelfTestDeltaValidDps(19.9f, 250.0f));
			Assert::IsFalse(IsGyroSelfTestDeltaValidDps(80.1f, 250.0f));
			Assert::IsTrue(IsGyroSelfTestDeltaValidDps(150.0f, 2000.0f));
			Assert::IsTrue(IsGyroSelfTestDeltaValidDps(700.0f, 2000.0f));
			Assert::IsFalse(IsGyroSelfTestDeltaValidDps(149.9f, 2000.0f));
			Assert::IsFalse(IsGyroSelfTestDeltaValidDps(700.1f, 2000.0f));
			Assert::IsFalse(IsGyroSelfTestDeltaValidDps(150.0f, 500.0f));
		}

		TEST_METHOD(UiImuSamplingProfileMatchesSupportedControlPeriods)
		{
			Assert::AreEqual(1000UL, GetUiImuSampleRateHzForControlPeriodUs(1000UL));
			Assert::AreEqual(2000UL, GetUiImuSampleRateHzForControlPeriodUs(500UL));
			Assert::AreEqual(0UL, GetUiImuSampleRateHzForControlPeriodUs(750UL));
		}

		TEST_METHOD(UiImuAccelLpf2CutoffMatchesDatasheetOdrDiv400Rule)
		{
			Assert::IsTrue(std::fabs(GetUiAccelLpf2CutoffHzForControlPeriodUs(1000UL) - 2.5f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(GetUiAccelLpf2CutoffHzForControlPeriodUs(500UL) - 5.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(GetUiAccelLpf2CutoffHzForControlPeriodUs(750UL)) < 1.0e-6f);
		}

		TEST_METHOD(UiImuGyroCut213ReferenceMatchesDatasheetTables)
		{
			Assert::IsTrue(std::fabs(GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(1000UL) - 195.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(500UL) - 210.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(GetUiGyroCut213DatasheetReferenceHzForControlPeriodUs(750UL)) < 1.0e-6f);
		}

		TEST_METHOD(StartupWaitIndicatorBlinksAtOneHertzWithHalfSecondDuty)
		{
			Assert::IsTrue(IsStartupWaitIndicatorEnabled(0UL, 1000UL));
			Assert::IsTrue(IsStartupWaitIndicatorEnabled(499UL, 1000UL));
			Assert::IsFalse(IsStartupWaitIndicatorEnabled(500UL, 1000UL));
			Assert::IsFalse(IsStartupWaitIndicatorEnabled(999UL, 1000UL));
			Assert::IsTrue(IsStartupWaitIndicatorEnabled(1000UL, 1000UL));
		}

		TEST_METHOD(StartupWaitIndicatorRejectsDegenerateBlinkPeriods)
		{
			Assert::IsFalse(IsStartupWaitIndicatorEnabled(0UL, 0UL));
			Assert::IsFalse(IsStartupWaitIndicatorEnabled(250UL, 1UL));
		}

		TEST_METHOD(ComputeMissionStartTurnClearanceMatchesStartCellGeometry)
		{
			Assert::IsTrue(std::fabs(ComputeMissionStartTurnClearanceM(0.180f, 0.0585f) - 0.0315f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeMissionStartCenterAdvanceMatchesStartCellGeometry)
		{
			Assert::IsTrue(std::fabs(ComputeMissionStartCenterAdvanceM(0.180f, 0.0585f) - 0.0315f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeCellInnerGeometryAccountsForTwelveMillimeterWalls)
		{
			Assert::IsTrue(std::fabs(ComputeCellWallFaceInsetM(0.012f) - 0.006f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeCellInnerSpanM(0.180f, 0.012f) - 0.168f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeWallTouchPoseCentersAccountForInnerWallFaces)
		{
			Assert::IsTrue(std::fabs(ComputeWallTouchPoseFromWestWallM(0.012f, 0.056f) - 0.062f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchPoseFromEastWallM(0.180f, 0.012f, 0.056f) - 0.118f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchPoseFromSouthWallM(0.012f, 0.056f) - 0.062f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchPoseFromNorthWallM(0.180f, 0.012f, 0.056f) - 0.118f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeWallTouchPoseSupportsMultiCellCorridorNorthWall)
		{
			Assert::IsTrue(std::fabs(ComputeWallTouchPoseFromNorthWallM(0.900f, 0.012f, 0.056f) - 0.838f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeCalibrationSafeMaxCenterXLeavesThreeMillimetersFromEastWall)
		{
			Assert::IsTrue(std::fabs(ComputeCalibrationSafeMaxCenterXFromEastWallM(0.180f, 0.012f, 0.0525f, 0.003f) - 0.1185f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeCalibrationSafeMaxCenterXLeavesThreeMillimetersAfterPositionTolerance)
		{
			Assert::IsTrue(std::fabs(ComputeCalibrationSafeMaxCenterXFromEastWallM(0.180f, 0.012f, 0.0525f, 0.006f) - 0.1155f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeCalibrationSafeMaxCenterXUsesRearCornerSweepRadius)
		{
			const float safeMaxCenterXM = ComputeCalibrationSafeMaxCenterXFromEastWallForRearCornerM(0.180f, 0.012f, 0.0525f, 0.0421f, 0.006f);
			Assert::IsTrue(std::fabs(safeMaxCenterXM - 0.10070479f) < 1.0e-5f);
		}

		TEST_METHOD(ComputeStartupWallCalibrationFrontSampleCentersUseFixedWestWallReference)
		{
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterXM(0.062f, 0.000f, 0.020f, 0U) - 0.062f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterXM(0.062f, 0.000f, 0.020f, 1U) - 0.082f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterXM(0.062f, 0.005f, 0.020f, 3U) - 0.127f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeStartupWallCalibrationFarthestFrontSampleCenterUsesLastSweepPoint)
		{
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFarthestFrontSampleCenterXM(0.062f, 0.000f, 0.020f, 4U) - 0.122f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFarthestFrontSampleCenterXM(0.062f, 0.005f, 0.020f, 0U) - 0.067f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeStartupWallCalibrationFrontSampleCentersRedistributeAcrossSafeSpan)
		{
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.10070479f, 4U, 0U) - 0.062f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.10070479f, 4U, 1U) - 0.074901596f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.10070479f, 4U, 3U) - 0.10070479f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.050f, 4U, 3U) - 0.062f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeMissionStartTurnClearanceRejectsInvalidInputs)
		{
			Assert::IsTrue(std::fabs(ComputeMissionStartTurnClearanceM(0.180f, std::numeric_limits<float>::quiet_NaN())) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeMissionStartTurnClearanceM(0.090f, 0.060f)) < 1.0e-6f);
		}

		TEST_METHOD(IsEncoderProgressWatchdogArmedWaitsOutStartupGrace)
		{
			Assert::IsFalse(IsEncoderProgressWatchdogArmed(0.10f, 0.050f, 249UL, 0.06f, 0.003f, 250UL));
			Assert::IsTrue(IsEncoderProgressWatchdogArmed(0.10f, 0.050f, 250UL, 0.06f, 0.003f, 250UL));
		}

		TEST_METHOD(IsEncoderProgressWatchdogArmedRejectsInactiveOrInvalidInputs)
		{
			Assert::IsFalse(IsEncoderProgressWatchdogArmed(0.05f, 0.050f, 500UL, 0.06f, 0.003f, 250UL));
			Assert::IsFalse(IsEncoderProgressWatchdogArmed(0.10f, 0.002f, 500UL, 0.06f, 0.003f, 250UL));
			Assert::IsFalse(IsEncoderProgressWatchdogArmed(std::numeric_limits<float>::quiet_NaN(), 0.050f, 500UL, 0.06f, 0.003f, 250UL));
		}

		TEST_METHOD(ExportMazeSnapshotWritesMazeRowsToFile)
		{
			const char* fileName = "mission_maze_export_test.txt";
			std::remove(fileName);

			Maze maze;
			Assert::IsTrue(ExportMazeSnapshot(maze, fileName));

			std::ifstream file(fileName);
			Assert::IsTrue(file.is_open());

			std::string line;
			int lineCount = 0;
			while (std::getline(file, line))
			{
				Assert::IsFalse(line.empty());
				Assert::AreEqual('"', line.front());
				Assert::AreEqual('"', line.back());
				++lineCount;
			}

			Assert::AreEqual(16, lineCount);
			file.close();
			std::remove(fileName);
		}

		TEST_METHOD(TryGetKnownMissionStartWallStateReturnsFixedStartCellTopology)
		{
			WallState wallState = WallState::Unknown;
			Assert::IsTrue(TryGetKnownMissionStartWallState(CellCoordinates(0, 0), Direction::Up, wallState));
			Assert::AreEqual(static_cast<int>(WallState::NoWall), static_cast<int>(wallState));

			Assert::IsTrue(TryGetKnownMissionStartWallState(CellCoordinates(0, 0), Direction::Down, wallState));
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(wallState));

			Assert::IsTrue(TryGetKnownMissionStartWallState(CellCoordinates(0, 0), Direction::Left, wallState));
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(wallState));

			Assert::IsTrue(TryGetKnownMissionStartWallState(CellCoordinates(0, 0), Direction::Right, wallState));
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(wallState));
		}

		TEST_METHOD(TryGetKnownMissionStartWallStateRejectsNonStartCellsAndDiagonals)
		{
			WallState wallState = WallState::Unknown;
			Assert::IsFalse(TryGetKnownMissionStartWallState(CellCoordinates(1, 0), Direction::Up, wallState));
			Assert::IsFalse(TryGetKnownMissionStartWallState(CellCoordinates(0, 1), Direction::Right, wallState));
			Assert::IsFalse(TryGetKnownMissionStartWallState(CellCoordinates(0, 0), Direction::UpRight, wallState));
		}

		TEST_METHOD(IsWallTouchContactSampleAllowsShortHardContactsAfterARealBump)
		{
			Assert::IsTrue(IsWallTouchContactSample(0.004f, 0.006f, 0.030f, 0.003f, 0.012f, 120UL, 120UL));
			Assert::IsFalse(IsWallTouchContactSample(0.001f, 0.006f, 0.030f, 0.003f, 0.012f, 120UL, 120UL));
			Assert::IsFalse(IsWallTouchContactSample(0.004f, 0.020f, 0.030f, 0.003f, 0.012f, 120UL, 120UL));
		}

		TEST_METHOD(ComputeWallTouchMinimumLatchTravelCapsToNearbyWallGeometry)
		{
			Assert::IsTrue(std::fabs(ComputeWallTouchMinimumLatchTravelM(0.0255f, 0.030f, 0.008f) - 0.0255f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchMinimumLatchTravelM(0.028f, 0.030f, 0.008f) - 0.028f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchMinimumLatchTravelM(0.033f, 0.030f, 0.008f) - 0.030f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchMinimumLatchTravelM(0.045f, 0.030f, 0.008f) - 0.037f) < 1.0e-6f);
		}

		TEST_METHOD(IsWallTouchSeatedSampleRejectsAngledOrSingleWheelSpinContact)
		{
			Assert::IsTrue(IsWallTouchSeatedSample(0.030f, 0.030f, 0.004f, 0.08f, 0.010f, -0.012f, 0.012f, 0.20f, 0.020f));
			Assert::IsFalse(IsWallTouchSeatedSample(0.030f, 0.030f, 0.004f, 0.24f, 0.010f, -0.012f, 0.012f, 0.20f, 0.020f));
			Assert::IsFalse(IsWallTouchSeatedSample(0.030f, 0.030f, 0.004f, 0.08f, 0.028f, -0.004f, 0.012f, 0.20f, 0.020f));
			Assert::IsFalse(IsWallTouchSeatedSample(0.020f, 0.030f, 0.004f, 0.08f, 0.010f, -0.012f, 0.012f, 0.20f, 0.020f));
		}

		TEST_METHOD(IsMissionStartupStationarySampleUsesWheelAndYawThresholds)
		{
			Assert::IsTrue(IsMissionStartupStationarySample(0.001f, 0.010f, 0.003f, -0.004f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationarySample(0.001f, 0.021f, 0.003f, -0.004f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationarySample(0.001f, 0.010f, 0.009f, -0.004f, 0.008f, 0.020f));
		}

		TEST_METHOD(IsMissionStartupStationarySampleRejectsInvalidInputs)
		{
			Assert::IsFalse(IsMissionStartupStationarySample(
				std::numeric_limits<float>::quiet_NaN(),
				0.010f,
				0.003f,
				-0.004f,
				0.008f,
				0.020f));
			Assert::IsFalse(IsMissionStartupStationarySample(0.001f, 0.010f, 0.003f, -0.004f, 0.0f, 0.020f));
		}

		TEST_METHOD(ComputeTurningTractionMetricsMatchesEncoderCircleKinematics)
		{
			const TurningTractionMetrics metrics = ComputeTurningTractionMetrics(0.450f, 0.750f, 0.080f, 3.750f, 2.150f);
			Assert::IsTrue(std::fabs(metrics.encoderLinearSpeedMps - 0.600f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(metrics.encoderOmegaRadps - 3.750f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(metrics.predictedLateralAccelMps2 - 2.250f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(metrics.yawCoherence - 1.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(metrics.planarCoherence - (2.150f / 2.250f)) < 1.0e-6f);
		}

		TEST_METHOD(ComputeTurningLaunchCommandsPreservesWheelRatioAtLaunch)
		{
			const TurningLaunchCommands commands = ComputeTurningLaunchCommands(0.120f, -0.6666667f, 0.0795f, 0.30f);
			Assert::IsTrue(std::fabs(commands.leftCommand - 0.30f) < 1.0e-5f);
			Assert::IsTrue(std::fabs(commands.rightCommand - 0.19149f) < 1.0e-4f);
		}

		TEST_METHOD(ComputeTurningTractionAngularCommandUsesArcFeedbackAndOptionalClamp)
		{
			const float commanded = ComputeTurningTractionAngularCommand(-0.5f, -0.4f, -0.1f, -0.25f, 10.0f, 0.4f, 9.0f);
			Assert::IsTrue(std::fabs(commanded + 3.4f) < 1.0e-6f);

			const float clamped = ComputeTurningTractionAngularCommand(-0.5f, -1.5f, 0.0f, 0.0f, 10.0f, 0.4f, 5.0f);
			Assert::IsTrue(std::fabs(clamped + 5.0f) < 1.0e-6f);

			const float unbounded = ComputeTurningTractionAngularCommand(-0.5f, -1.5f, 0.0f, 0.0f, 10.0f, 0.4f, 0.0f);
			Assert::IsTrue(std::fabs(unbounded + 15.5f) < 1.0e-6f);
		}

		TEST_METHOD(IsTurningTractionLossDetectedRequiresSustainedMismatchAtSpeed)
		{
			TurningTractionMetrics slipping{};
			slipping.encoderLinearSpeedMps = 0.600f;
			slipping.predictedLateralAccelMps2 = 2.250f;
			slipping.yawCoherence = 0.62f;
			slipping.planarCoherence = 0.58f;
			Assert::IsTrue(IsTurningTractionLossDetected(slipping, 0.25f, 1.00f, 0.70f, 0.65f));

			TurningTractionMetrics clean = slipping;
			clean.planarCoherence = 0.82f;
			Assert::IsFalse(IsTurningTractionLossDetected(clean, 0.25f, 1.00f, 0.70f, 0.65f));
			Assert::IsFalse(IsTurningTractionLossDetected(slipping, 0.25f, 3.00f, 0.70f, 0.65f));
		}

		TEST_METHOD(NormalizeWheelControlProfileFallsBackToNominalScales)
		{
			WheelControlProfile requested{};
			requested.velocityKpScale = 0.0f;
			requested.velocityKiScale = std::numeric_limits<float>::quiet_NaN();
			requested.integralLimitScale = -2.0f;

			const WheelControlProfile normalized = NormalizeWheelControlProfile(requested);
			Assert::IsTrue(std::fabs(normalized.velocityKpScale - 1.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(normalized.velocityKiScale - 1.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(normalized.integralLimitScale - 1.0f) < 1.0e-6f);
		}

		TEST_METHOD(ScaleWheelControlValueUsesPositiveFiniteScales)
		{
			Assert::IsTrue(std::fabs(ScaleWheelControlValue(1.10f, 2.0f) - 2.20f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ScaleWheelControlValue(1.50f, 0.0f) - 1.50f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ScaleWheelControlValue(0.25f, -1.0f) - 0.25f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ScaleWheelControlValue(std::numeric_limits<float>::quiet_NaN(), 2.0f)) < 1.0e-6f);
		}

		TEST_METHOD(MotorModelUnitConversionsMatch1717T006SRDatasheet)
		{
			const float noLoadCurrentA = MilliAmpsToAmps(45.9f);
			Assert::IsTrue(std::fabs(MilliAmpsToAmps(45.9f) - 0.0459f) < 1.0e-7f);
			Assert::IsTrue(std::fabs(MilliNewtonMetersToNewtonMeters(3.96f) - 0.00396f) < 1.0e-7f);
			Assert::IsTrue(std::fabs(RpmPerVoltToRadPerSecondPerVolt(2410.0f) - 252.374f) < 1.0e-3f);
			Assert::IsTrue(std::fabs(ComputeMotorSpeedConstantRadpsPerVolt(14100.0f, 6.0f, noLoadCurrentA, 4.31f) - 254.482f) < 1.0e-3f);
		}

		TEST_METHOD(MotorEncoderDriveMatches1717T006SRDatasheetAtNominalVoltage)
		{
			const float resistanceOhms = 4.31f;
			const float voltageV = 6.0f;
			const float torqueConstantNmPerA = MilliNewtonMetersToNewtonMeters(3.96f);
			const float noLoadCurrentA = MilliAmpsToAmps(45.9f);
			const float speedConstantRadpsPerVolt = ComputeMotorSpeedConstantRadpsPerVolt(14100.0f, voltageV, noLoadCurrentA, resistanceOhms);
			const float wheelDiameterM = 0.010f;
			const float wheelRadiusM = wheelDiameterM * 0.5f;
			const float noLoadSpeedMps = wheelRadiusM * RpmToRadPerSecond(14100.0f);
			const float stallForceN = MilliNewtonMetersToNewtonMeters(5.33f) / wheelRadiusM;

			MotorEncoderDrive drive(
				resistanceOhms,
				voltageV,
				torqueConstantNmPerA,
				speedConstantRadpsPerVolt,
				noLoadCurrentA,
				1.0f,
				wheelDiameterM,
				1U,
				0U,
				1U,
				2U,
				3U);

			MotorEncoderDrive frictionlessDrive(
				resistanceOhms,
				voltageV,
				torqueConstantNmPerA,
				speedConstantRadpsPerVolt,
				0.0f,
				1.0f,
				wheelDiameterM,
				1U,
				0U,
				1U,
				2U,
				3U);

			Assert::IsTrue(std::fabs(drive.getMaxForceAtVelocity(0.0f) - stallForceN) < 1.0e-3f);
			Assert::IsTrue(std::fabs(drive.getSpeedAtForceLimit(0.0f) - noLoadSpeedMps) < 1.0e-3f);
			Assert::IsTrue(drive.getMaxForceAtVelocity(0.0f) < frictionlessDrive.getMaxForceAtVelocity(0.0f));
		}

		TEST_METHOD(MotorEncoderDriveProjectsHigherNoLoadSpeedAt2SOvervoltage)
		{
			const float nominalVoltageV = 6.0f;
			const float overvoltedBusV = 8.4f;
			const float resistanceOhms = 4.31f;
			const float noLoadCurrentA = MilliAmpsToAmps(45.9f);
			const float speedConstantRadpsPerVolt = ComputeMotorSpeedConstantRadpsPerVolt(14100.0f, nominalVoltageV, noLoadCurrentA, resistanceOhms);
			const float wheelDiameterM = 0.010f;
			const float wheelRadiusM = wheelDiameterM * 0.5f;
			const float nominalNoLoadSpeedMps = wheelRadiusM * RpmToRadPerSecond(14100.0f);
			const float expectedOvervoltedNoLoadSpeedMps =
				nominalNoLoadSpeedMps * ((overvoltedBusV - (noLoadCurrentA * resistanceOhms)) / (nominalVoltageV - (noLoadCurrentA * resistanceOhms)));

			MotorEncoderDrive drive(
				resistanceOhms,
				overvoltedBusV,
				MilliNewtonMetersToNewtonMeters(3.96f),
				speedConstantRadpsPerVolt,
				noLoadCurrentA,
				1.0f,
				wheelDiameterM,
				1U,
				0U,
				1U,
				2U,
				3U);

			Assert::IsTrue(std::fabs(drive.getSpeedAtForceLimit(0.0f) - expectedOvervoltedNoLoadSpeedMps) < 1.0e-3f);
			Assert::IsTrue(drive.getSpeedAtForceLimit(0.0f) > nominalNoLoadSpeedMps);
		}
	};
}
