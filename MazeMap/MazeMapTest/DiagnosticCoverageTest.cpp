#include "pch.h"
#include "CppUnitTest.h"
#include "..\MazeMap\DiagnosticCoverage.h"
#include "..\MazeMap\DiagnosticLogBudget.h"
#include "..\MazeMap\DiagnosticMotionPlan.h"
#include "..\MazeMap\EncoderStallPolicy.h"
#include "..\MazeMap\FanRampProfile.h"
#include "..\MazeMap\FrontWallCharacterizationStorage.h"
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
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\RollingAverageWindow.h"
#include "..\MazeMap\SearchRunPlanner.h"
#include "..\MazeMap\SmoothTurnYawRateController.h"
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

#include "..\MazeMap\CoreConfig.h"
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
				"kArcTrackWidthTightRadiusM",
				"kArcTrackWidthTightM",
				"kArcTrackWidthWideRadiusM",
				"kArcTrackWidthWideM",
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
			Assert::IsTrue(std::fabs(model.yawInertiaKgM2 - 0.000220f) < 1.0e-9f);
			Assert::IsTrue(std::fabs(model.frontWallContactOffsetM - 0.056f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.trackWidthM - 0.084635f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.trackWidthPhysicalMinM - 0.07004f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.trackWidthPhysicalMaxM - 0.07868f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.arcTrackWidthInterpolation.tightRadiusM - 0.063f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.arcTrackWidthInterpolation.tightTrackWidthM - 0.096491f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.arcTrackWidthInterpolation.wideRadiusM - 0.153f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(model.arcTrackWidthInterpolation.wideTrackWidthM - 0.097348f) < 1.0e-6f);
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
			Assert::IsTrue(std::fabs(model.wheelDiameterM - 0.025220f) < 1.0e-6f);
			Assert::AreEqual(4096U, static_cast<unsigned>(model.pulsesPerRev));
		}

		TEST_METHOD(PlantParamsDefaultUsesLaunchFitWheelBankInertia)
		{
			const PlantParams params = PlantParams::Default();
			Assert::IsTrue(std::fabs(params.equivalentWheelInertiaKgM2 - 1.6e-5f) < 1.0e-10f);
		}

		TEST_METHOD(PlantParamsDefaultMatchesLatestLaunchThresholdFit)
		{
			const PlantParams params = PlantParams::Default();
			Assert::IsTrue(std::fabs(params.rollingFrictionTorqueNm - 0.00372f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(params.staticFrictionTorqueNm - 0.007028315f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(params.staticFrictionMaxSpeedMps - 0.005f) < 1.0e-9f);
			Assert::IsTrue(std::fabs(params.viscousFrictionNmPerRadps - 0.0f) < 1.0e-9f);
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
			const VehiclePhysicalModel& model = Vehicle::GetPhysicalModel();
			Assert::IsTrue(std::fabs(model.trackWidthM - 0.084635f) < 1.0e-6f);
			Assert::IsTrue(model.trackWidthPhysicalMinM < model.trackWidthPhysicalMaxM);
			Assert::IsTrue(std::fabs(vehicle.GetMaxLateralAcceleration() - 16.5f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(vehicle.GetMaxRotationalVelocity() - 9.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(vehicle.GetMaxAngularAcceleration() - 45.0f) < 1.0e-6f);
		}

		TEST_METHOD(ArcTrackWidthInterpolationClampsAndBlendsByRadius)
		{
			Assert::AreEqual(0.096491f, Vehicle::GetArcEffectiveTrackWidth(0.040f), 1.0e-6f);
			Assert::AreEqual(0.097348f, Vehicle::GetArcEffectiveTrackWidth(0.200f), 1.0e-6f);
			Assert::AreEqual(0.096920f, Vehicle::GetArcEffectiveTrackWidth(0.108f), 1.0e-5f);
		}

		TEST_METHOD(ArcTrackWidthInterpolationFallsBackToBaseWidthForStraightAndInPlaceMotion)
		{
			Assert::AreEqual(0.084635f, Vehicle::GetEffectiveTrackWidthForMotion(0.0f, 4.0f), 1.0e-6f);
			Assert::AreEqual(0.084635f, Vehicle::GetEffectiveTrackWidthForMotion(0.3f, 0.0f), 1.0e-6f);
			Assert::AreEqual(0.096491f, Vehicle::GetEffectiveTrackWidthForMotion(0.3f, (0.3f / 0.063f)), 1.0e-5f);
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

		TEST_METHOD(ManeuverTrackedDistanceExposureMatchesSmoothTurnDefinitions)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			Assert::IsTrue(set.SupportsPointTracking(S90SS));
			Assert::IsTrue(set.SupportsPointTracking(S90LS));
			Assert::AreEqual(
				(10.0f / 180.0f) + (33.0f / 180.0f) + ((PI_F / 2.0f) * (63.0f / 180.0f)) + (10.0f / 180.0f),
				set.GetTravelDistanceMeters(S90SS, 1.0f),
				1.0e-6f);
			Assert::AreEqual(
				(9.0f / 180.0f) + (36.0f / 180.0f) + ((PI_F / 2.0f) * (153.0f / 180.0f)) + (9.0f / 180.0f),
				set.GetTravelDistanceMeters(S90LS, 1.0f),
				1.0e-6f);
		}

		TEST_METHOD(ManeuverTrackedDistanceExceedsHalfStepShortcutForSmoothTurns)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float shortDistanceCells = set.GetTravelDistanceMeters(S90SS, 1.0f);
			const float longDistanceCells = set.GetTravelDistanceMeters(S90LS, 1.0f);

			const float shortShortcutCells = 0.5f * static_cast<float>(set.DistanceTravelled(S90SS));
			const float longShortcutCells = 0.5f * static_cast<float>(set.DistanceTravelled(S90LS));

			Assert::IsTrue(shortDistanceCells > shortShortcutCells);
			Assert::IsTrue(longDistanceCells > longShortcutCells);
			Assert::IsTrue(shortDistanceCells < longDistanceCells);
		}

		TEST_METHOD(ManeuverPointSamplingMirrorsYawAndOmegaForMirroredCodes)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float sampleDistanceM = 0.5f * set.GetTravelDistanceMeters(S90SS, 1.0f);
			ManeuverPoint rightPoint{};
			ManeuverPoint leftPoint{};
			Assert::IsTrue(set.TryGetManeuverPoint(S90SS, sampleDistanceM, 0.4f, rightPoint, 1.0f));
			Assert::IsTrue(set.TryGetManeuverPoint(S90SS_M, sampleDistanceM, 0.4f, leftPoint, 1.0f));
			Assert::IsTrue(rightPoint.Theta > 0.0f);
			Assert::IsTrue(rightPoint.Omega > 0.0f);
			Assert::AreEqual(-rightPoint.Theta, leftPoint.Theta, 1.0e-6f);
			Assert::AreEqual(-rightPoint.Omega, leftPoint.Omega, 1.0e-6f);
			Assert::AreEqual(rightPoint.Velocity, leftPoint.Velocity, 1.0e-6f);
		}

		TEST_METHOD(PositionAuditSmoothTurnValidityAndRunoutDifferentiatesShortAndLongSmooth90)
		{
			Maze maze;
			for (uint8_t y = 0U; y < 5U; ++y)
			{
				Cell& cell = maze[CellCoordinates(0U, y)];
				maze.SetWall(cell, Up, (y < 4U) ? NoWall : Wall);
				maze.SetWall(cell, Down, (y > 0U) ? NoWall : Wall);
				maze.SetWall(cell, Left, Wall);
				maze.SetWall(cell, Right, (y == 4U) ? NoWall : Wall);
			}
			for (uint8_t x = 1U; x <= 4U; ++x)
			{
				Cell& cell = maze[CellCoordinates(x, 4U)];
				maze.SetWall(cell, Up, Wall);
				maze.SetWall(cell, Down, Wall);
				maze.SetWall(cell, Left, NoWall);
				maze.SetWall(cell, Right, (x < 4U) ? NoWall : Wall);
			}

			const DirectionalLocation shortStart(MazeLocation(1U, 8U), Up);
			const DirectionalLocation longStart(MazeLocation::CellCenter(CellCoordinates(0U, 3U)), Up);
			const auto countClearHalfSteps = [&maze](DirectionalLocation start)
			{
				uint8_t clearHalfSteps = 0U;
				while (clearHalfSteps < 31U)
				{
					start = start.MoveForward(1U);
					if (!maze.IsAccessibleLocation(start.GetLocation()))
					{
						break;
					}

					++clearHalfSteps;
				}

				return clearHalfSteps;
			};

			Assert::IsTrue(maze.IsAccessibleLocation(shortStart.GetLocation()));
			Assert::IsTrue(maze.IsAccessibleLocation(longStart.GetLocation()));
			Assert::IsTrue(ManeuverSet::GetSet().IsValidMove(S90SS, shortStart, maze));
			Assert::IsFalse(ManeuverSet::GetSet().IsValidMove(S90SS, longStart, maze));
			Assert::IsTrue(ManeuverSet::GetSet().IsValidMove(S90LS, longStart, maze));
			Assert::IsFalse(ManeuverSet::GetSet().IsValidMove(S90LS, shortStart, maze));
			const DirectionalLocation shortEnd = ManeuverSet::GetSet().Move(S90SS, shortStart);
			const DirectionalLocation longEnd = ManeuverSet::GetSet().Move(S90LS, longStart);
			DirectionalLocation shortLastClear = shortEnd;
			DirectionalLocation shortBlocked = shortEnd;
			DirectionalLocation longLastClear = longEnd;
			DirectionalLocation longBlocked = longEnd;
			shortLastClear = shortLastClear.MoveForward(7U);
			shortBlocked = shortBlocked.MoveForward(8U);
			longLastClear = longLastClear.MoveForward(6U);
			longBlocked = longBlocked.MoveForward(7U);
			Assert::IsTrue(shortEnd == DirectionalLocation(MazeLocation(2U, 9U), Right));
			Assert::IsTrue(longEnd == DirectionalLocation(MazeLocation(3U, 9U), Right));
			Assert::AreEqual(static_cast<uint8_t>(7U), countClearHalfSteps(shortEnd));
			Assert::AreEqual(static_cast<uint8_t>(6U), countClearHalfSteps(longEnd));
			Assert::IsTrue(maze.IsAccessibleLocation(shortLastClear.GetLocation()));
			Assert::IsFalse(maze.IsAccessibleLocation(shortBlocked.GetLocation()));
			Assert::IsTrue(maze.IsAccessibleLocation(longLastClear.GetLocation()));
			Assert::IsFalse(maze.IsAccessibleLocation(longBlocked.GetLocation()));
		}

		TEST_METHOD(PositionAuditConfiguredFixedFixtureRoutesReturnToStartWhenReversed)
		{
			Maze maze;
			for (uint8_t y = 0U; y < 5U; ++y)
			{
				Cell& cell = maze[CellCoordinates(0U, y)];
				maze.SetWall(cell, Up, (y < 4U) ? NoWall : Wall);
				maze.SetWall(cell, Down, (y > 0U) ? NoWall : Wall);
				maze.SetWall(cell, Left, Wall);
				maze.SetWall(cell, Right, (y == 4U) ? NoWall : Wall);
			}
			for (uint8_t x = 1U; x <= 4U; ++x)
			{
				Cell& cell = maze[CellCoordinates(x, 4U)];
				maze.SetWall(cell, Up, Wall);
				maze.SetWall(cell, Down, Wall);
				maze.SetWall(cell, Left, NoWall);
				maze.SetWall(cell, Right, (x < 4U) ? NoWall : Wall);
			}

			const ManeuverSet& set = ManeuverSet::GetSet();
			const auto executePath = [&maze, &set](DirectionalLocation current, const ManeuverCode* codes, const size_t count)
			{
				for (size_t index = 0U; index < count; ++index)
				{
					Assert::IsTrue(set.IsValidMove(codes[index], current, maze));
					current = set.Move(codes[index], current);
					Assert::IsTrue(maze.IsAccessibleLocation(current.GetLocation()));
				}

				return current;
			};

			const DirectionalLocation start(MazeLocation::CellCenter(CellCoordinates(0U, 0U)), Up);
			const ManeuverCode straightPhase[] = { S8, IP180, S8 };
			const DirectionalLocation straightEnd = executePath(start, straightPhase, _countof(straightPhase));
			Assert::IsTrue(straightEnd == DirectionalLocation(MazeLocation::CellCenter(CellCoordinates(0U, 0U)), Down));

			const ManeuverCode shortPhase[] = { S7, S90SS, S7 };
			const DirectionalLocation shortEnd = executePath(start, shortPhase, _countof(shortPhase));
			Assert::IsTrue(shortEnd == DirectionalLocation(MazeLocation::CellCenter(CellCoordinates(4U, 4U)), Right));
			const ManeuverCode shortReverse[] = {
				set.GetReverseCode(shortPhase[2]),
				set.GetReverseCode(shortPhase[1]),
				set.GetReverseCode(shortPhase[0]),
			};
			const DirectionalLocation shortReturnStart(shortEnd.GetLocation(), Left);
			const DirectionalLocation shortReturnEnd = executePath(shortReturnStart, shortReverse, _countof(shortReverse));
			Assert::IsTrue(shortReturnEnd == DirectionalLocation(MazeLocation::CellCenter(CellCoordinates(0U, 0U)), Down));

			const ManeuverCode longPhase[] = { S6, S90LS, S6 };
			const DirectionalLocation longEnd = executePath(start, longPhase, _countof(longPhase));
			Assert::IsTrue(longEnd == DirectionalLocation(MazeLocation::CellCenter(CellCoordinates(4U, 4U)), Right));
			const ManeuverCode longReverse[] = {
				set.GetReverseCode(longPhase[2]),
				set.GetReverseCode(longPhase[1]),
				set.GetReverseCode(longPhase[0]),
			};
			const DirectionalLocation longReturnStart(longEnd.GetLocation(), Left);
			const DirectionalLocation longReturnEnd = executePath(longReturnStart, longReverse, _countof(longReverse));
			Assert::IsTrue(longReturnEnd == DirectionalLocation(MazeLocation::CellCenter(CellCoordinates(0U, 0U)), Down));
		}

		TEST_METHOD(CodeDegreesUsesRightTurnSignForUnmirroredSmoothTurns)
		{
			Assert::AreEqual(static_cast<int>(180), static_cast<int>(CodeDegrees(IP180)));
			Assert::AreEqual(static_cast<int>(-180), static_cast<int>(CodeDegrees(IP180_M)));
			Assert::AreEqual(static_cast<int>(90), static_cast<int>(CodeDegrees(S90SS)));
			Assert::AreEqual(static_cast<int>(-90), static_cast<int>(CodeDegrees(S90SS_M)));
			Assert::AreEqual(static_cast<int>(90), static_cast<int>(CodeDegrees(S90LS)));
			Assert::AreEqual(static_cast<int>(-90), static_cast<int>(CodeDegrees(S90LS_M)));
		}

		TEST_METHOD(ManeuverPointSamplingUsesStraightRampCurveAndRampOutPhases)
		{
			class TestSmoothTurn final : public SmoothTurnManeuver<1, true, false>
			{
			public:
				TestSmoothTurn()
					: SmoothTurnManeuver<1, true, false>(0.5f, 1.0f, 0.2f, 0.1f, 0.1f)
				{
				}

				ManeuverCode GetManeuverID() const override { return S90SS; }
				ManeuverCode GetBackwardsManeuverID() const override { return S90SS; }
			};

			TestSmoothTurn maneuver{};
			ManeuverPoint point{};
			Assert::IsTrue(maneuver.TryGetManeuverPoint(0.05f, 0.4f, 1.0f, point));
			Assert::AreEqual(0.0f, point.Theta, 1.0e-6f);
			Assert::AreEqual(0.0f, point.Omega, 1.0e-6f);

			Assert::IsTrue(maneuver.TryGetManeuverPoint(0.2f, 0.4f, 1.0f, point));
			Assert::AreEqual(0.05f, point.Theta, 1.0e-6f);
			Assert::AreEqual(0.4f, point.Omega, 1.0e-6f);

			Assert::IsTrue(maneuver.TryGetManeuverPoint(0.45f, 0.4f, 1.0f, point));
			Assert::AreEqual(0.5f, point.Theta, 1.0e-6f);
			Assert::AreEqual(0.8f, point.Omega, 1.0e-6f);

			Assert::IsTrue(maneuver.TryGetManeuverPoint(0.75f, 0.4f, 1.0f, point));
			Assert::AreEqual(0.9875f, point.Theta, 1.0e-6f);
			Assert::AreEqual(0.2f, point.Omega, 1.0e-6f);

			Assert::IsTrue(maneuver.TryGetManeuverPoint(0.85f, 0.4f, 1.0f, point));
			Assert::AreEqual(1.0f, point.Theta, 1.0e-6f);
			Assert::AreEqual(0.0f, point.Omega, 1.0e-6f);
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

		TEST_METHOD(SmoothTurnYawRatePdControllerUsesProportionalTermAndDelayedDerivative)
		{
			SmoothTurnYawRateControllerState state{};
			const float firstCorrection = ComputeSmoothTurnYawRatePdCorrection(4.0f, 3.0f, 0.002f, 0.70f, 0.006f, state);
			Assert::AreEqual(0.70f, firstCorrection, 1.0e-6f);

			const float secondCorrection = ComputeSmoothTurnYawRatePdCorrection(4.4f, 3.2f, 0.002f, 0.70f, 0.006f, state);
			Assert::AreEqual(1.44f, secondCorrection, 1.0e-5f);
		}

		TEST_METHOD(SmoothTurnYawRatePdControllerRejectsInvalidInputAndResetsState)
		{
			SmoothTurnYawRateControllerState state{};
			(void)ComputeSmoothTurnYawRatePdCorrection(3.0f, 2.0f, 0.002f, 0.70f, 0.006f, state);

			const float invalidCorrection = ComputeSmoothTurnYawRatePdCorrection(
				std::numeric_limits<float>::quiet_NaN(),
				2.0f,
				0.002f,
				0.70f,
				0.006f,
				state);
			Assert::AreEqual(0.0f, invalidCorrection, 1.0e-6f);
			Assert::IsFalse(state.hasPreviousError);

			const float recoveredCorrection = ComputeSmoothTurnYawRatePdCorrection(2.5f, 2.0f, 0.002f, 0.70f, 0.006f, state);
			Assert::AreEqual(0.35f, recoveredCorrection, 1.0e-6f);
		}

		TEST_METHOD(TryComputeLinearWallSignalDistanceThresholdMUsesInverseSquareScaling)
		{
			float thresholdDistanceM = 0.0f;
			Assert::IsTrue(TryComputeLinearWallSignalDistanceThresholdM(0.0550f, 1.0f / 6.0f, thresholdDistanceM));
			Assert::AreEqual(0.1347219f, thresholdDistanceM, 0.0001f);
		}

		TEST_METHOD(TryComputeInverseSquareSignalAtDistanceFromReferenceKeepsThresholdGeometryConsistent)
		{
			float normalizedReferenceSignal = 0.0f;
			Assert::IsTrue(TryComputeInverseSquareSignalAtDistanceFromReference(0.054f, 0.049633f, 0.055589f, normalizedReferenceSignal));

			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			Assert::IsTrue(TryComputeSignalHighThresholds(
				normalizedReferenceSignal,
				1.0f / 6.0f,
				1.0f / 8.0f,
				onMeasuredThreshold,
				offMeasuredThreshold));

			float onDistanceM = 0.0f;
			float offDistanceM = 0.0f;
			Assert::IsTrue(TryComputeInverseSquareDistanceFromReferenceSignal(onMeasuredThreshold, 0.054f, 0.049633f, onDistanceM));
			Assert::IsTrue(TryComputeInverseSquareDistanceFromReferenceSignal(offMeasuredThreshold, 0.054f, 0.049633f, offDistanceM));
			Assert::AreEqual(0.136165f, onDistanceM, 0.0001f);
			Assert::AreEqual(0.157230f, offDistanceM, 0.0001f);
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

		TEST_METHOD(TryComputeSignalRiseThresholdsUsesBaselineToWeakestSpan)
		{
			float onRiseThreshold = 0.0f;
			float offRiseThreshold = 0.0f;
			Assert::IsTrue(TryComputeSignalRiseThresholds(0.052160f, 0.083306f, 0.50f, 0.35f, onRiseThreshold, offRiseThreshold));
			Assert::AreEqual(0.015573f, onRiseThreshold, 1.0e-6f);
			Assert::AreEqual(0.010901f, offRiseThreshold, 1.0e-6f);
		}

		TEST_METHOD(TryComputeSignalBandThresholdsInterpolateBetweenBaselineAndWallReference)
		{
			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			Assert::IsTrue(TryComputeSignalBandThresholds(0.006000f, 0.054000f, 1.0f / 6.0f, 1.0f / 8.0f, onMeasuredThreshold, offMeasuredThreshold));
			Assert::AreEqual(0.014000f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.012000f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(TryComputeSignalRiseThresholdsUseOpenDeltaSubtractedSideSignal)
		{
			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			Assert::IsTrue(TryComputeSignalRiseThresholds(0.002000f, 0.032000f, 1.0f / 6.0f, 1.0f / 8.0f, onMeasuredThreshold, offMeasuredThreshold));
			Assert::AreEqual(0.005000f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.003750f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(TryComputeWallSegmentCenterWindowKepsOnlyMiddleThird)
		{
			float minCoordinateM = 0.0f;
			float maxCoordinateM = 0.0f;
			Assert::IsTrue(TryComputeWallSegmentCenterWindowM(0.090f, 0.180f, 0.012f, 1.0f / 3.0f, minCoordinateM, maxCoordinateM));
			Assert::AreEqual(0.062f, minCoordinateM, 1.0e-6f);
			Assert::AreEqual(0.118f, maxCoordinateM, 1.0e-6f);
			Assert::IsTrue(IsWithinWallSegmentCenterWindowM(0.090f, 0.180f, 0.012f, 1.0f / 3.0f));
			Assert::IsFalse(IsWithinWallSegmentCenterWindowM(0.050f, 0.180f, 0.012f, 1.0f / 3.0f));
			Assert::IsFalse(IsWithinWallSegmentCenterWindowM(0.130f, 0.180f, 0.012f, 1.0f / 3.0f));
		}

		TEST_METHOD(IsWithinWallSegmentCenterWindowRejectsBoundaryPostCoordinates)
		{
			Assert::IsFalse(IsWithinWallSegmentCenterWindowM(0.723477f, 0.180f, 0.012f, 1.0f / 3.0f));
			Assert::IsFalse(IsWithinWallSegmentCenterWindowM(0.730234f, 0.180f, 0.012f, 1.0f / 3.0f));
			Assert::IsTrue(IsWithinWallSegmentCenterWindowM(0.810000f, 0.180f, 0.012f, 1.0f / 3.0f));
		}

		TEST_METHOD(ClassifyFrontCalibrationSpinHeadingFromNorthUsesOpenAndEastKnownWallBuckets)
		{
			Assert::AreEqual(
				static_cast<int>(FrontCalibrationSpinHeadingClass::OpenNorth),
				static_cast<int>(ClassifyFrontCalibrationSpinHeadingFromNorth(
					0.0f * DEG_TO_RAD_F,
					25.0f * DEG_TO_RAD_F,
					30.0f * DEG_TO_RAD_F,
					90.0f * DEG_TO_RAD_F)));
			Assert::AreEqual(
				static_cast<int>(FrontCalibrationSpinHeadingClass::Ignore),
				static_cast<int>(ClassifyFrontCalibrationSpinHeadingFromNorth(
					27.0f * DEG_TO_RAD_F,
					25.0f * DEG_TO_RAD_F,
					30.0f * DEG_TO_RAD_F,
					90.0f * DEG_TO_RAD_F)));
			Assert::AreEqual(
				static_cast<int>(FrontCalibrationSpinHeadingClass::Wall),
				static_cast<int>(ClassifyFrontCalibrationSpinHeadingFromNorth(
					31.0f * DEG_TO_RAD_F,
					25.0f * DEG_TO_RAD_F,
					30.0f * DEG_TO_RAD_F,
					90.0f * DEG_TO_RAD_F)));
			Assert::AreEqual(
				static_cast<int>(FrontCalibrationSpinHeadingClass::Wall),
				static_cast<int>(ClassifyFrontCalibrationSpinHeadingFromNorth(
					90.0f * DEG_TO_RAD_F,
					25.0f * DEG_TO_RAD_F,
					30.0f * DEG_TO_RAD_F,
					90.0f * DEG_TO_RAD_F)));
			Assert::AreEqual(
				static_cast<int>(FrontCalibrationSpinHeadingClass::Ignore),
				static_cast<int>(ClassifyFrontCalibrationSpinHeadingFromNorth(
					120.0f * DEG_TO_RAD_F,
					25.0f * DEG_TO_RAD_F,
					30.0f * DEG_TO_RAD_F,
					90.0f * DEG_TO_RAD_F)));
			Assert::AreEqual(
				static_cast<int>(FrontCalibrationSpinHeadingClass::Ignore),
				static_cast<int>(ClassifyFrontCalibrationSpinHeadingFromNorth(
					225.0f * DEG_TO_RAD_F,
					25.0f * DEG_TO_RAD_F,
					30.0f * DEG_TO_RAD_F,
					90.0f * DEG_TO_RAD_F)));
		}

		TEST_METHOD(TryComputeSignedTravelToCellCenterAlongHeadingProjectsObservationRecentering)
		{
			float signedTravelM = 0.0f;
			Assert::IsTrue(TryComputeSignedTravelToCellCenterAlongHeadingM(
				CellCoordinates(0, 1),
				0.180f,
				0.0879f,
				0.3406f,
				0.0f,
				1.0f,
				signedTravelM));
			Assert::AreEqual(-0.0706f, signedTravelM, 0.0005f);

			Assert::IsTrue(TryComputeSignedTravelToCellCenterAlongHeadingM(
				CellCoordinates(0, 1),
				0.180f,
				0.0900f,
				0.2500f,
				0.0f,
				1.0f,
				signedTravelM));
			Assert::AreEqual(0.0200f, signedTravelM, 0.0005f);

			Assert::IsFalse(TryComputeSignedTravelToCellCenterAlongHeadingM(
				CellCoordinates(0, 1),
				0.180f,
				0.0900f,
				0.2500f,
				0.0f,
				0.0f,
				signedTravelM));
		}

		TEST_METHOD(TryComputeSideWallObservationPoseTargetsSensorAtCellCenter)
		{
			float targetXM = 0.0f;
			float targetYM = 0.0f;
			Assert::IsTrue(TryComputeSideWallObservationPoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.05026f,
				targetXM,
				targetYM));
			Assert::AreEqual(0.09000f, targetXM, 1.0e-6f);
			Assert::AreEqual(0.21974f, targetYM, 1.0e-5f);

			Assert::IsTrue(TryComputeSideWallObservationPoseM(
				CellCoordinates(2, 3),
				Right,
				0.180f,
				0.05026f,
				targetXM,
				targetYM));
			Assert::AreEqual(0.39974f, targetXM, 1.0e-5f);
			Assert::AreEqual(0.63000f, targetYM, 1.0e-6f);
		}

		TEST_METHOD(TryComputeSideWallObservationPoseRejectsInvalidInputs)
		{
			float targetXM = 0.0f;
			float targetYM = 0.0f;
			Assert::IsFalse(TryComputeSideWallObservationPoseM(
				CellCoordinates(0, 1),
				None,
				0.180f,
				0.05026f,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallObservationPoseM(
				CellCoordinates(0, 1),
				Up,
				0.0f,
				0.05026f,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallObservationPoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				-0.001f,
				targetXM,
				targetYM));
		}

		TEST_METHOD(TryComputeSideWallObservationSamplePoseSpansTargetRegion)
		{
			float targetXM = 0.0f;
			float targetYM = 0.0f;
			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				0U,
				9U,
				targetXM,
				targetYM));
			Assert::AreEqual(0.09000f, targetXM, 1.0e-6f);
			Assert::AreEqual(0.19174f, targetYM, 1.0e-5f);

			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				4U,
				9U,
				targetXM,
				targetYM));
			Assert::AreEqual(0.09000f, targetXM, 1.0e-6f);
			Assert::AreEqual(0.21974f, targetYM, 1.0e-5f);

			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				8U,
				9U,
				targetXM,
				targetYM));
			Assert::AreEqual(0.09000f, targetXM, 1.0e-6f);
			Assert::AreEqual(0.24774f, targetYM, 1.0e-5f);

			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(2, 3),
				Right,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				0U,
				9U,
				targetXM,
				targetYM));
			Assert::AreEqual(0.37174f, targetXM, 1.0e-5f);
			Assert::AreEqual(0.63000f, targetYM, 1.0e-6f);
		}

		TEST_METHOD(FrontWallObservationGeometryPutsLatchCloserThanLegacyElevenCentimeters)
		{
			float targetXM = 0.0f;
			float targetYM = 0.0f;
			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 0),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				4U,
				9U,
				targetXM,
				targetYM));

			const float northWallYM = 0.174f;
			const float frontSensorForwardOffsetM = 0.04223f;
			const float frontSensorForwardY = 0.99452f;
			const float frontSensorYM = targetYM + frontSensorForwardOffsetM;
			const float onDistanceM = (northWallYM - frontSensorYM) / frontSensorForwardY;
			Assert::AreEqual(0.092537f, onDistanceM, 1.0e-4f);
			Assert::IsTrue(onDistanceM < 0.110f);

			const float offDistanceM = onDistanceM + 0.020f;
			Assert::AreEqual(0.112537f, offDistanceM, 1.0e-4f);
		}

		TEST_METHOD(TryComputeSideWallObservationSamplePoseRejectsInvalidInputs)
		{
			float targetXM = 0.0f;
			float targetYM = 0.0f;
			Assert::IsFalse(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				None,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				0U,
				9U,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				0.0f,
				0U,
				9U,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				9U,
				9U,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				0U,
				0U,
				targetXM,
				targetYM));
		}

		TEST_METHOD(TryComputeSideWallTravelFractionPosePlacesResetBeforeObservationWindow)
		{
			float resetXM = 0.0f;
			float resetYM = 0.0f;
			Assert::IsTrue(TryComputeSideWallTravelFractionPoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.05026f,
				0.25f,
				resetXM,
				resetYM));
			Assert::AreEqual(0.09000f, resetXM, 1.0e-6f);
			Assert::AreEqual(0.17474f, resetYM, 1.0e-5f);

			float sampleXM = 0.0f;
			float sampleYM = 0.0f;
			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				0U,
				9U,
				sampleXM,
				sampleYM));
			Assert::IsTrue(resetYM < sampleYM);

			Assert::IsTrue(TryComputeSideWallTravelFractionPoseM(
				CellCoordinates(2, 3),
				Right,
				0.180f,
				0.05026f,
				0.25f,
				resetXM,
				resetYM));
			Assert::AreEqual(0.35474f, resetXM, 1.0e-5f);
			Assert::AreEqual(0.63000f, resetYM, 1.0e-6f);

			Assert::IsTrue(TryComputeSideWallObservationSamplePoseM(
				CellCoordinates(2, 3),
				Right,
				0.180f,
				0.012f,
				0.05026f,
				1.0f / 3.0f,
				0U,
				9U,
				sampleXM,
				sampleYM));
			Assert::IsTrue(resetXM < sampleXM);
		}

		TEST_METHOD(TryComputeSideWallTravelFractionPoseRejectsInvalidInputs)
		{
			float targetXM = 0.0f;
			float targetYM = 0.0f;
			Assert::IsFalse(TryComputeSideWallTravelFractionPoseM(
				CellCoordinates(0, 1),
				None,
				0.180f,
				0.05026f,
				0.25f,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallTravelFractionPoseM(
				CellCoordinates(0, 1),
				Up,
				0.0f,
				0.05026f,
				0.25f,
				targetXM,
				targetYM));
			Assert::IsFalse(TryComputeSideWallTravelFractionPoseM(
				CellCoordinates(0, 1),
				Up,
				0.180f,
				0.05026f,
				-0.01f,
				targetXM,
				targetYM));
		}

		TEST_METHOD(TryComputeRobustSignalBandFromSamplesRejectsOutlier)
		{
			const std::array<float, 5U> samples = { 1.0f, 1.0f, 1.0f, 1.0f, 10.0f };
			float median = 0.0f;
			float low = 0.0f;
			float high = 0.0f;
			Assert::IsTrue(TryComputeRobustSignalBandFromSamples(samples, 5U, 3.0f, median, low, high));
			Assert::AreEqual(1.0f, median, 1.0e-6f);
			Assert::AreEqual(1.0f, low, 1.0e-6f);
			Assert::AreEqual(1.0f, high, 1.0e-6f);
		}

		TEST_METHOD(TryComputeRobustDistanceMatchedSignalBandFromSamplesUsesNearestDistanceSubset)
		{
			const std::array<float, 12U> signalSamples = {
				0.002f, 0.003f, 0.004f, 0.020f, 0.021f, 0.022f,
				0.023f, 0.024f, 0.070f, 0.080f, 0.090f, 0.100f
			};
			const std::array<float, 12U> distanceSamples = {
				0.040f, 0.050f, 0.060f, 0.106f, 0.109f, 0.111f,
				0.113f, 0.116f, 0.150f, 0.160f, 0.170f, 0.180f
			};
			float median = 0.0f;
			float low = 0.0f;
			float high = 0.0f;
			Assert::IsTrue(TryComputeRobustDistanceMatchedSignalBandFromSamples(
				signalSamples,
				distanceSamples,
				12U,
				0.110f,
				4U,
				4U,
				0.020f,
				3.0f,
				median,
				low,
				high));
			Assert::AreEqual(0.0215f, median, 1.0e-6f);
			Assert::IsTrue(low > 0.0f);
			Assert::IsTrue(low < median);
			Assert::IsTrue(high > median);
		}

		TEST_METHOD(TryComputeSignalRiseThresholdsSupportNormalizedSideWallReferenceThresholds)
		{
			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			Assert::IsTrue(TryComputeSignalRiseThresholds(0.0f, 0.043248f, 0.10f, 0.07f, onMeasuredThreshold, offMeasuredThreshold));
			Assert::AreEqual(0.0043248f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.00302736f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(TryComputeConservativeSignalRiseThresholdsFromBandsUsesConservativeFrontSpan)
		{
			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			float signalBaseline = 0.0f;
			Assert::IsTrue(TryComputeConservativeSignalRiseThresholdsFromBands(
				0.050000f,
				0.056000f,
				0.080000f,
				0.086000f,
				0.50f,
				0.35f,
				onMeasuredThreshold,
				offMeasuredThreshold,
				signalBaseline));
			Assert::AreEqual(0.056000f, signalBaseline, 1.0e-6f);
			Assert::AreEqual(0.012000f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.008400f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(TryComputeConservativeSignalRiseThresholdsFromCollapsedOpenBandSupportsInPlaceFrontThresholds)
		{
			const std::array<float, 9> openSamples{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
			const std::array<float, 9> wallSamples{ 0.030f, 0.030f, 0.030f, 0.030f, 0.030f, 0.030f, 0.030f, 0.030f, 0.030f };
			float openMedian = 0.0f;
			float openLow = 0.0f;
			float openHigh = 0.0f;
			Assert::IsTrue(TryComputeRobustSignalBandFromSamples(
				openSamples,
				static_cast<uint16_t>(openSamples.size()),
				3.0f,
				openMedian,
				openLow,
				openHigh));
			float wallMedian = 0.0f;
			float wallLow = 0.0f;
			float wallHigh = 0.0f;
			Assert::IsTrue(TryComputeRobustSignalBandFromSamples(
				wallSamples,
				static_cast<uint16_t>(wallSamples.size()),
				3.0f,
				wallMedian,
				wallLow,
				wallHigh));

			float onMeasuredThreshold = 0.0f;
			float offMeasuredThreshold = 0.0f;
			float signalBaseline = 0.0f;
			Assert::IsTrue(TryComputeConservativeSignalRiseThresholdsFromBands(
				openLow,
				openHigh,
				wallLow,
				wallHigh,
				0.22f,
				0.15f,
				onMeasuredThreshold,
				offMeasuredThreshold,
				signalBaseline));
			Assert::AreEqual(0.0f, signalBaseline, 1.0e-6f);
			Assert::AreEqual(0.0066f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.0045f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(FinalizeFrontWallCharacterizationStorageProducesValidChecksum)
		{
			FrontWallCharacterizationStorage storage{};
			storage.sampleCount = 3U;
			storage.distanceStepM = 0.001f;
			storage.commandedReverseSpeedMps = 0.03f;
			storage.zeroThresholdDifferentialLight = 0.0005f;
			storage.terminalDistanceM = 0.015f;
			storage.distanceM[0] = 0.000f;
			storage.distanceM[1] = 0.008f;
			storage.distanceM[2] = 0.015f;
			storage.frontLeftLitLight[0] = 0.120f;
			storage.frontLeftLitLight[1] = 0.090f;
			storage.frontLeftLitLight[2] = 0.000f;
			storage.frontLeftDifferentialLight[0] = 0.120f;
			storage.frontLeftDifferentialLight[1] = 0.090f;
			storage.frontLeftDifferentialLight[2] = 0.000f;
			storage.frontRightLitLight[0] = 0.140f;
			storage.frontRightLitLight[1] = 0.110f;
			storage.frontRightLitLight[2] = 0.000f;
			storage.frontRightDifferentialLight[0] = 0.140f;
			storage.frontRightDifferentialLight[1] = 0.110f;
			storage.frontRightDifferentialLight[2] = 0.000f;

			FinalizeFrontWallCharacterizationStorage(storage);
			Assert::IsTrue(IsValidFrontWallCharacterizationStorage(storage));
		}

		TEST_METHOD(IsValidFrontWallCharacterizationStorageRejectsNonMonotonicDistance)
		{
			FrontWallCharacterizationStorage storage{};
			storage.sampleCount = 3U;
			storage.distanceStepM = 0.001f;
			storage.commandedReverseSpeedMps = 0.03f;
			storage.zeroThresholdDifferentialLight = 0.0005f;
			storage.terminalDistanceM = 0.020f;
			storage.distanceM[0] = 0.000f;
			storage.distanceM[1] = 0.010f;
			storage.distanceM[2] = 0.009f;
			storage.frontLeftDifferentialLight[0] = 0.120f;
			storage.frontLeftDifferentialLight[1] = 0.090f;
			storage.frontLeftDifferentialLight[2] = 0.070f;
			storage.frontRightDifferentialLight[0] = 0.140f;
			storage.frontRightDifferentialLight[1] = 0.100f;
			storage.frontRightDifferentialLight[2] = 0.080f;

			FinalizeFrontWallCharacterizationStorage(storage);
			Assert::IsFalse(IsValidFrontWallCharacterizationStorage(storage));
		}

		TEST_METHOD(TrySampleFrontWallCharacterizationDifferentialLightInterpolatesWithinStoredCurve)
		{
			FrontWallCharacterizationStorage storage{};
			storage.sampleCount = 4U;
			storage.distanceStepM = 0.005f;
			storage.commandedReverseSpeedMps = 0.03f;
			storage.zeroThresholdDifferentialLight = 0.0005f;
			storage.terminalDistanceM = 0.015f;
			storage.distanceM[0] = 0.000f;
			storage.distanceM[1] = 0.005f;
			storage.distanceM[2] = 0.010f;
			storage.distanceM[3] = 0.015f;
			storage.frontLeftDifferentialLight[0] = 0.120f;
			storage.frontLeftDifferentialLight[1] = 0.090f;
			storage.frontLeftDifferentialLight[2] = 0.040f;
			storage.frontLeftDifferentialLight[3] = 0.000f;
			storage.frontRightDifferentialLight[0] = 0.130f;
			storage.frontRightDifferentialLight[1] = 0.100f;
			storage.frontRightDifferentialLight[2] = 0.050f;
			storage.frontRightDifferentialLight[3] = 0.000f;
			FinalizeFrontWallCharacterizationStorage(storage);

			float differentialLight = 0.0f;
			Assert::IsTrue(TrySampleFrontWallCharacterizationDifferentialLight(
				storage,
				false,
				0.0075f,
				differentialLight));
			Assert::AreEqual(0.065f, differentialLight, 1.0e-6f);
		}

		TEST_METHOD(TryMatchFrontWallCharacterizationChannelFindsScaledWallTemplate)
		{
			FrontWallCharacterizationStorage storage{};
			storage.sampleCount = 10U;
			storage.distanceStepM = 0.005f;
			storage.commandedReverseSpeedMps = 0.03f;
			storage.zeroThresholdDifferentialLight = 0.0005f;
			storage.terminalDistanceM = 0.045f;
			for (uint16_t index = 0U; index < storage.sampleCount; ++index)
			{
				storage.distanceM[index] = 0.005f * static_cast<float>(index);
			}
			storage.frontLeftDifferentialLight[0] = 0.120f;
			storage.frontLeftDifferentialLight[1] = 0.105f;
			storage.frontLeftDifferentialLight[2] = 0.090f;
			storage.frontLeftDifferentialLight[3] = 0.070f;
			storage.frontLeftDifferentialLight[4] = 0.050f;
			storage.frontLeftDifferentialLight[5] = 0.030f;
			storage.frontLeftDifferentialLight[6] = 0.015f;
			storage.frontLeftDifferentialLight[7] = 0.004f;
			storage.frontLeftDifferentialLight[8] = 0.000f;
			storage.frontLeftDifferentialLight[9] = 0.000f;
			for (uint16_t index = 0U; index < storage.sampleCount; ++index)
			{
				storage.frontRightDifferentialLight[index] = storage.frontLeftDifferentialLight[index];
			}
			FinalizeFrontWallCharacterizationStorage(storage);

			const float floorDifferentialLight =
				EstimateFrontWallCharacterizationChannelFloor(storage, false);
			const float signalBaseline = 0.010f;
			const float expectedDistanceM[] = { 0.000f, 0.010f, 0.020f, 0.030f, 0.040f };
			float measuredDifferentialLight[_countof(expectedDistanceM)] = {};
			for (size_t index = 0U; index < _countof(expectedDistanceM); ++index)
			{
				float templateDifferentialLight = 0.0f;
				Assert::IsTrue(TrySampleFrontWallCharacterizationDifferentialLight(
					storage,
					false,
					expectedDistanceM[index],
					templateDifferentialLight));
				const float templateRise = (std::max)(0.0f, templateDifferentialLight - floorDifferentialLight);
				measuredDifferentialLight[index] = signalBaseline + (0.50f * templateRise);
			}

			FrontWallCharacterizationMatch match{};
			Assert::IsTrue(TryMatchFrontWallCharacterizationChannel(
				storage,
				false,
				measuredDifferentialLight,
				expectedDistanceM,
				static_cast<uint16_t>(_countof(expectedDistanceM)),
				signalBaseline,
				match));
			Assert::IsTrue(match.valid);
			Assert::AreEqual(0.50f, match.scale, 1.0e-3f);
			Assert::IsTrue(match.normalizedCorrelation > 0.999f);
			Assert::IsTrue(match.relativeResidual < 1.0e-5f);
		}

		TEST_METHOD(TryMatchFrontWallCharacterizationChannelKeepsFlatOpenSignalBelowWallThreshold)
		{
			FrontWallCharacterizationStorage storage{};
			storage.sampleCount = 10U;
			storage.distanceStepM = 0.005f;
			storage.commandedReverseSpeedMps = 0.03f;
			storage.zeroThresholdDifferentialLight = 0.0005f;
			storage.terminalDistanceM = 0.045f;
			for (uint16_t index = 0U; index < storage.sampleCount; ++index)
			{
				storage.distanceM[index] = 0.005f * static_cast<float>(index);
			}
			storage.frontLeftDifferentialLight[0] = 0.120f;
			storage.frontLeftDifferentialLight[1] = 0.105f;
			storage.frontLeftDifferentialLight[2] = 0.090f;
			storage.frontLeftDifferentialLight[3] = 0.070f;
			storage.frontLeftDifferentialLight[4] = 0.050f;
			storage.frontLeftDifferentialLight[5] = 0.030f;
			storage.frontLeftDifferentialLight[6] = 0.015f;
			storage.frontLeftDifferentialLight[7] = 0.004f;
			storage.frontLeftDifferentialLight[8] = 0.000f;
			storage.frontLeftDifferentialLight[9] = 0.000f;
			for (uint16_t index = 0U; index < storage.sampleCount; ++index)
			{
				storage.frontRightDifferentialLight[index] = storage.frontLeftDifferentialLight[index];
			}
			FinalizeFrontWallCharacterizationStorage(storage);

			const float signalBaseline = 0.010f;
			const float expectedDistanceM[] = { 0.000f, 0.010f, 0.020f, 0.030f, 0.040f };
			const float measuredDifferentialLight[] = { signalBaseline, signalBaseline, signalBaseline, signalBaseline, signalBaseline };

			FrontWallCharacterizationMatch match{};
			Assert::IsTrue(TryMatchFrontWallCharacterizationChannel(
				storage,
				false,
				measuredDifferentialLight,
				expectedDistanceM,
				static_cast<uint16_t>(_countof(expectedDistanceM)),
				signalBaseline,
				match));
			Assert::IsTrue(match.valid);
			Assert::AreEqual(0.0f, match.scale, 1.0e-6f);
			Assert::AreEqual(0.0f, match.normalizedCorrelation, 1.0e-6f);
			Assert::AreEqual(0.0f, match.relativeResidual, 1.0e-6f);
		}

		TEST_METHOD(TryScaleSignalHighThresholdsPreservesThresholdOrdering)
		{
			float onMeasuredThreshold = 0.095013f;
			float offMeasuredThreshold = 0.078842f;
			Assert::IsTrue(TryScaleSignalHighThresholds(0.70f, onMeasuredThreshold, offMeasuredThreshold));
			Assert::AreEqual(0.0665091f, onMeasuredThreshold, 1.0e-6f);
			Assert::AreEqual(0.0551894f, offMeasuredThreshold, 1.0e-6f);
		}

		TEST_METHOD(HysteresisSignalHighUsesReleaseThresholdWhileLatched)
		{
			Assert::IsFalse(HysteresisSignalHigh(false, 0.0650f, 0.0700f, 0.0550f));
			Assert::IsTrue(HysteresisSignalHigh(false, 0.0710f, 0.0700f, 0.0550f));
			Assert::IsTrue(HysteresisSignalHigh(true, 0.0600f, 0.0700f, 0.0550f));
			Assert::IsFalse(HysteresisSignalHigh(true, 0.0540f, 0.0700f, 0.0550f));
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

		TEST_METHOD(RollingAverageWindowUsesTrimmedMeanForFiveSampleWindow)
		{
			RollingAverageWindow<5U> window{};
			Assert::AreEqual(1.0f, window.Push(1.0f), 1.0e-6f);
			Assert::AreEqual(1.0f, window.Push(1.0f), 1.0e-6f);
			Assert::AreEqual(1.0f, window.Push(1.0f), 1.0e-6f);
			Assert::AreEqual(1.0f, window.Push(1.0f), 1.0e-6f);
			Assert::AreEqual(1.0f, window.Push(10.0f), 1.0e-6f);
			Assert::AreEqual(1.0f, window.Average(), 1.0e-6f);
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

			Assert::IsTrue(TryComputeSignedTurnAngleRad(179.0f * DEG_TO_RAD_F, -179.0f * DEG_TO_RAD_F, angleRad));
			Assert::AreEqual(2.0f * DEG_TO_RAD_F, angleRad, 1.0e-6f);
		}

		TEST_METHOD(InPlaceTurnProfileUsesSharedCompletionAndCommandLaw)
		{
			InPlaceTurnProfile profile{};
			profile.maxAngularSpeedRadps = 9.0f;
			profile.angularAccelRadps2 = 45.0f;
			profile.headingKp = 7.5f;
			profile.yawD = 0.12f;
			profile.angleToleranceRad = 0.75f * DEG_TO_RAD_F;
			profile.angularSpeedToleranceRadps = 0.10f;

			Assert::IsTrue(IsInPlaceTurnComplete(0.50f * DEG_TO_RAD_F, 0.08f, profile));
			Assert::IsFalse(IsInPlaceTurnComplete(1.00f * DEG_TO_RAD_F, 0.08f, profile));
			Assert::IsFalse(IsInPlaceTurnComplete(0.50f * DEG_TO_RAD_F, 0.12f, profile));

			float angularCommandRadps = 0.0f;
			Assert::IsTrue(TryComputeInPlaceTurnCommandRadps(
				0.50f * PI_F,
				0.0f,
				profile,
				angularCommandRadps));
			Assert::IsTrue(angularCommandRadps > 0.0f);

			Assert::IsTrue(TryComputeInPlaceTurnCommandRadps(
				0.01f,
				0.0f,
				profile,
				angularCommandRadps));
			const float expectedSmallAngleCommandRadps =
				sqrtf(2.0f * profile.angularAccelRadps2 * 0.01f) +
				(profile.headingKp * 0.01f);
			Assert::AreEqual(expectedSmallAngleCommandRadps, angularCommandRadps, 1.0e-6f);
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
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 99UL, 100UL, true, true, true, true));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(0.79f, 0.80f, 150UL, 100UL, true, true, true, true));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 150UL, 100UL, false, true, true, true));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 150UL, 100UL, true, false, true, true));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 150UL, 100UL, true, true, true, false));
			Assert::IsFalse(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 150UL, 100UL, true, true, false, true));
			Assert::IsTrue(ShouldReleaseWallTouchSeat(1.0f, 0.80f, 100UL, 100UL, true, true, true, true));
		}

		TEST_METHOD(IsWallTouchSeatAsymmetricReleaseCueRequiresOppositeBiasAndSmallAdvance)
		{
			Assert::IsFalse(IsWallTouchSeatAsymmetricReleaseCue(false, false, true, true, 0.002f, 0.010f));
			Assert::IsFalse(IsWallTouchSeatAsymmetricReleaseCue(true, true, true, true, 0.002f, 0.010f));
			Assert::IsFalse(IsWallTouchSeatAsymmetricReleaseCue(true, true, false, false, 0.002f, 0.010f));
			Assert::IsFalse(IsWallTouchSeatAsymmetricReleaseCue(true, true, false, true, 0.012f, 0.010f));
			Assert::IsTrue(IsWallTouchSeatAsymmetricReleaseCue(true, true, false, true, 0.002f, 0.010f));
		}

		TEST_METHOD(HasWallTouchSeatQualifiedBiasPhaseRequiresFullBiasDuration)
		{
			Assert::IsFalse(HasWallTouchSeatQualifiedBiasPhase(99UL, 100UL));
			Assert::IsFalse(HasWallTouchSeatQualifiedBiasPhase(150UL, 0UL));
			Assert::IsTrue(HasWallTouchSeatQualifiedBiasPhase(100UL, 100UL));
			Assert::IsTrue(HasWallTouchSeatQualifiedBiasPhase(180UL, 100UL));
		}

		TEST_METHOD(ComputeAverageEncoderAbsSpeedMpsUsesWheelMagnitudes)
		{
			Assert::AreEqual(0.030f, ComputeAverageEncoderAbsSpeedMps(0.020f, -0.040f), 1.0e-6f);
			Assert::AreEqual(0.0f, ComputeAverageEncoderAbsSpeedMps(NAN, 0.040f), 1.0e-6f);
		}

		TEST_METHOD(PlanSearchReplanResponseRequiresValidNextStep)
		{
			Path<PATH_SIZE> path;
			Assert::IsTrue(path.push_back(CellCoordinates(2, 3)));

			const SearchReplanResponse singlePoint = PlanSearchReplanResponse(path, Direction::Up);
			Assert::IsFalse(singlePoint.hasPath);
			Assert::IsFalse(singlePoint.requiresTurn);
			Assert::AreEqual(static_cast<int>(Direction::None), static_cast<int>(singlePoint.nextDirection));

			Assert::IsTrue(path.push_back(CellCoordinates(2, 3)));
			const SearchReplanResponse duplicatePoint = PlanSearchReplanResponse(path, Direction::Up);
			Assert::IsFalse(duplicatePoint.hasPath);
			Assert::IsFalse(duplicatePoint.requiresTurn);
			Assert::AreEqual(static_cast<int>(Direction::None), static_cast<int>(duplicatePoint.nextDirection));
		}

		TEST_METHOD(PlanSearchReplanResponseReportsTurnRequirement)
		{
			Path<PATH_SIZE> path;
			Assert::IsTrue(path.push_back(CellCoordinates(4, 5)));
			Assert::IsTrue(path.push_back(CellCoordinates(5, 5)));

			const SearchReplanResponse response = PlanSearchReplanResponse(path, Direction::Up);
			Assert::IsTrue(response.hasPath);
			Assert::IsTrue(response.requiresTurn);
			Assert::AreEqual(static_cast<int>(Direction::Right), static_cast<int>(response.nextDirection));
		}

		TEST_METHOD(PlanSearchReplanResponseRecognizesAlignedContinuation)
		{
			Path<PATH_SIZE> path;
			Assert::IsTrue(path.push_back(CellCoordinates(7, 7)));
			Assert::IsTrue(path.push_back(CellCoordinates(7, 8)));

			const SearchReplanResponse response = PlanSearchReplanResponse(path, Direction::Up);
			Assert::IsTrue(response.hasPath);
			Assert::IsFalse(response.requiresTurn);
			Assert::AreEqual(static_cast<int>(Direction::Up), static_cast<int>(response.nextDirection));
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

		TEST_METHOD(UiImuAccelLpf2CutoffMatchesConfiguredFraction)
		{
			using AccelFilterFreq = MazeMap::Vehicle::ImuBackLeft::ACCEL_FILTER_FREQ;
			Assert::IsTrue(std::fabs(GetUiAccelLpf2CutoffHzForControlPeriodUs(500UL, AccelFilterFreq::FRAC_1_020) - 100.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(GetUiAccelLpf2CutoffHzForControlPeriodUs(500UL, AccelFilterFreq::FRAC_1_400) - 5.0f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(GetUiAccelLpf2CutoffHzForControlPeriodUs(500UL, AccelFilterFreq::FRAC_1_002)) < 1.0e-6f);
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

		TEST_METHOD(CalibrationCenterCoordinateValidationSupportsMultiCellFixtureTargets)
		{
			Assert::IsTrue(IsValidCalibrationCenterCoordinateM(0.090f));
			Assert::IsTrue(IsValidCalibrationCenterCoordinateM(0.838f));
			Assert::IsFalse(IsValidCalibrationCenterCoordinateM(-0.010f));
			Assert::IsFalse(IsValidCalibrationCenterCoordinateM(std::numeric_limits<float>::quiet_NaN()));
		}

		TEST_METHOD(CalibrationDirectionTowardTargetSelectsExpectedSignedAxisMotion)
		{
			Direction selectedDirection = Left;
			Assert::IsTrue(TrySelectCalibrationDirectionTowardTarget(0.7765f, 0.8100f, 0.0010f, Down, Up, selectedDirection));
			Assert::AreEqual(static_cast<int>(Up), static_cast<int>(selectedDirection));
			Assert::IsTrue(TrySelectCalibrationDirectionTowardTarget(0.8400f, 0.8100f, 0.0010f, Down, Up, selectedDirection));
			Assert::AreEqual(static_cast<int>(Down), static_cast<int>(selectedDirection));
			Assert::IsFalse(TrySelectCalibrationDirectionTowardTarget(0.8105f, 0.8100f, 0.0010f, Down, Up, selectedDirection));
		}

		TEST_METHOD(CalibrationDirectionTowardTargetRejectsInvalidInputs)
		{
			Direction selectedDirection = Up;
			Assert::IsFalse(TrySelectCalibrationDirectionTowardTarget(
				std::numeric_limits<float>::quiet_NaN(),
				0.8100f,
				0.0010f,
				Down,
				Up,
				selectedDirection));
			Assert::IsFalse(TrySelectCalibrationDirectionTowardTarget(
				0.7765f,
				0.8100f,
				-0.0010f,
				Down,
				Up,
				selectedDirection));
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

		TEST_METHOD(ComputeCalibrationSafeMinCenterXUsesRearCornerSweepRadius)
		{
			const float safeMinCenterXM = ComputeCalibrationSafeMinCenterXFromWestWallForRearCornerM(0.012f, 0.0525f, 0.0421f, 0.006f);
			Assert::IsTrue(std::fabs(safeMinCenterXM - 0.07929521f) < 1.0e-5f);
		}

		TEST_METHOD(ComputeCalibrationSafeMinAndMaxCenterXMirrorAcrossCellCenter)
		{
			const float safeMinCenterXM = ComputeCalibrationSafeMinCenterXFromWestWallForRearCornerM(0.012f, 0.0525f, 0.0421f, 0.006f);
			const float safeMaxCenterXM = ComputeCalibrationSafeMaxCenterXFromEastWallForRearCornerM(0.180f, 0.012f, 0.0525f, 0.0421f, 0.006f);
			Assert::IsTrue(std::fabs((safeMinCenterXM + safeMaxCenterXM) - 0.180f) < 1.0e-5f);
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

		TEST_METHOD(ComputeStartupWallCalibrationFrontSampleCentersUseFixedEastWallReference)
		{
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterFromEastWallXM(0.118f, 0.000f, 0.020f, 0U) - 0.118f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterFromEastWallXM(0.118f, 0.000f, 0.020f, 1U) - 0.098f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterFromEastWallXM(0.118f, 0.005f, 0.020f, 3U) - 0.053f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeStartupWallCalibrationFarthestFrontSampleCenterFromEastWallUsesLastSweepPoint)
		{
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFarthestFrontSampleCenterFromEastWallXM(0.118f, 0.000f, 0.020f, 4U) - 0.058f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFarthestFrontSampleCenterFromEastWallXM(0.118f, 0.005f, 0.020f, 0U) - 0.113f) < 1.0e-6f);
		}

		TEST_METHOD(ComputeStartupWallCalibrationFrontSampleCentersRedistributeAcrossSafeSpan)
		{
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.10070479f, 4U, 0U) - 0.062f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.10070479f, 4U, 1U) - 0.074901596f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.062f, 0.10070479f, 4U, 3U) - 0.10070479f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeStartupWallCalibrationFrontSampleCenterWithinSpanXM(0.118f, 0.07929521f, 4U, 3U) - 0.07929521f) < 1.0e-6f);
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

		TEST_METHOD(EncoderProgressWatchdogPolicyUsesAtLeastNinetySecondTimeout)
		{
			Assert::IsTrue(Config::kEncoderStallTimeoutMs >= 90000UL);
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

		TEST_METHOD(ComputeWallTouchMaximumApproachDistanceExpandsForKnownLongCorridors)
		{
			Assert::IsTrue(std::fabs(ComputeWallTouchMaximumApproachDistanceM(0.045f, 0.168f, 0.008f) - 0.168f) < 1.0e-6f);
			Assert::IsTrue(std::fabs(ComputeWallTouchMaximumApproachDistanceM(0.3984f, 0.168f, 0.008f) - 0.4064f) < 1.0e-6f);
		}

		TEST_METHOD(IsWallTouchSeatedSampleRejectsAngledOrSingleWheelSpinContact)
		{
			Assert::IsTrue(IsWallTouchSeatedSample(0.030f, 0.030f, 0.004f, 0.08f, 0.010f, -0.012f, 0.012f, 0.20f, 0.020f));
			Assert::IsFalse(IsWallTouchSeatedSample(0.030f, 0.030f, 0.004f, 0.24f, 0.010f, -0.012f, 0.012f, 0.20f, 0.020f));
			Assert::IsFalse(IsWallTouchSeatedSample(0.030f, 0.030f, 0.004f, 0.08f, 0.028f, -0.004f, 0.012f, 0.20f, 0.020f));
			Assert::IsFalse(IsWallTouchSeatedSample(0.020f, 0.030f, 0.004f, 0.08f, 0.010f, -0.012f, 0.012f, 0.20f, 0.020f));
		}

		TEST_METHOD(CountWallTouchContactIndicatorsRequiresTwoIndependentCues)
		{
			Assert::AreEqual(0, static_cast<int>(CountWallTouchContactIndicators(false, false, false)));
			Assert::AreEqual(1, static_cast<int>(CountWallTouchContactIndicators(true, false, false)));
			Assert::AreEqual(2, static_cast<int>(CountWallTouchContactIndicators(true, true, false)));
			Assert::AreEqual(3, static_cast<int>(CountWallTouchContactIndicators(true, true, true)));
		}

		TEST_METHOD(HasWallTouchConfirmedContactRequiresPersistenceAndTwoIndicators)
		{
			Assert::IsFalse(HasWallTouchConfirmedContact(11UL, 12UL, 2U));
			Assert::IsFalse(HasWallTouchConfirmedContact(12UL, 12UL, 1U));
			Assert::IsFalse(HasWallTouchConfirmedContact(12UL, 0UL, 2U));
			Assert::IsTrue(HasWallTouchConfirmedContact(12UL, 12UL, 2U));
			Assert::IsTrue(HasWallTouchConfirmedContact(20UL, 12UL, 3U));
		}

		TEST_METHOD(ComputeWallTouchSeatWiggleTurnFractionEscalatesAndClamps)
		{
			Assert::AreEqual(0.16f, ComputeWallTouchSeatWiggleTurnFraction(0U, 0.16f, 0.04f, 0.28f), 1.0e-6f);
			Assert::AreEqual(0.20f, ComputeWallTouchSeatWiggleTurnFraction(1U, 0.16f, 0.04f, 0.28f), 1.0e-6f);
			Assert::AreEqual(0.28f, ComputeWallTouchSeatWiggleTurnFraction(4U, 0.16f, 0.04f, 0.28f), 1.0e-6f);
			Assert::AreEqual(0.0f, ComputeWallTouchSeatWiggleTurnFraction(1U, std::numeric_limits<float>::quiet_NaN(), 0.04f, 0.28f), 1.0e-6f);
		}

		TEST_METHOD(IsWallTouchSquareCycleGoodRequiresFrontSkewYawAndSignalAgreement)
		{
			Assert::IsTrue(IsWallTouchSquareCycleGood(0.0015f, 0.0025f, 0.040f, 0.050f, 0.010f, 0.020f, true));
			Assert::IsFalse(IsWallTouchSquareCycleGood(0.0030f, 0.0025f, 0.040f, 0.050f, 0.010f, 0.020f, true));
			Assert::IsFalse(IsWallTouchSquareCycleGood(0.0015f, 0.0025f, 0.060f, 0.050f, 0.010f, 0.020f, true));
			Assert::IsFalse(IsWallTouchSquareCycleGood(0.0015f, 0.0025f, 0.040f, 0.050f, 0.030f, 0.020f, true));
			Assert::IsFalse(IsWallTouchSquareCycleGood(0.0015f, 0.0025f, 0.040f, 0.050f, 0.010f, 0.020f, false));
		}

		TEST_METHOD(HasWallTouchSquareUpSaturatedOnlyFlagsSmallImprovements)
		{
			Assert::IsTrue(HasWallTouchSquareUpSaturated(0.0040f, 0.0037f, 0.0005f));
			Assert::IsFalse(HasWallTouchSquareUpSaturated(0.0040f, 0.0030f, 0.0005f));
			Assert::IsFalse(HasWallTouchSquareUpSaturated(0.0030f, 0.0040f, 0.0005f));
		}

		TEST_METHOD(IsWallTouchSquareSuccessEligibleEnforcesContactTimeAndCyclePersistence)
		{
			Assert::IsFalse(IsWallTouchSquareSuccessEligible(449UL, 450UL, 3U, 3U, 2U, 2U));
			Assert::IsFalse(IsWallTouchSquareSuccessEligible(450UL, 450UL, 2U, 3U, 2U, 2U));
			Assert::IsFalse(IsWallTouchSquareSuccessEligible(450UL, 450UL, 3U, 3U, 1U, 2U));
			Assert::IsTrue(IsWallTouchSquareSuccessEligible(450UL, 450UL, 3U, 3U, 2U, 2U));
			Assert::IsTrue(IsWallTouchSquareSuccessEligible(700UL, 450UL, 5U, 3U, 3U, 2U));
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

		TEST_METHOD(IsMissionStartupStationaryFromSensorsUsesEncoderAverageInsteadOfPose)
		{
			Assert::IsTrue(IsMissionStartupStationaryFromSensors(0.003f, -0.004f, 0.010f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationaryFromSensors(0.012f, 0.010f, 0.010f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationaryFromSensors(0.003f, -0.004f, 0.021f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationaryFromSensors(
				std::numeric_limits<float>::quiet_NaN(),
				-0.004f,
				0.010f,
				0.008f,
				0.020f));
		}

		TEST_METHOD(IsMissionStartupStationaryFromEncoderWindowUsesEncoderTravelInsteadOfVelocityNoise)
		{
			Assert::IsTrue(IsMissionStartupStationaryFromEncoderWindow(0.0f, 0.0f, 0.250f, 0.010f, 0.008f, 0.020f));
			Assert::IsTrue(IsMissionStartupStationaryFromEncoderWindow(0.0005f, -0.0005f, 0.250f, 0.010f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationaryFromEncoderWindow(0.0035f, 0.0035f, 0.250f, 0.010f, 0.008f, 0.020f));
			Assert::IsFalse(IsMissionStartupStationaryFromEncoderWindow(0.0f, 0.0f, 0.250f, 0.021f, 0.008f, 0.020f));
		}

		TEST_METHOD(IsMissionStartupStationaryFromEncoderWindowRejectsInvalidInputs)
		{
			Assert::IsFalse(IsMissionStartupStationaryFromEncoderWindow(
				std::numeric_limits<float>::quiet_NaN(),
				0.0f,
				0.250f,
				0.010f,
				0.008f,
				0.020f));
			Assert::IsFalse(IsMissionStartupStationaryFromEncoderWindow(0.0f, 0.0f, 0.0f, 0.010f, 0.008f, 0.020f));
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

		TEST_METHOD(ClampWheelDriveCommandUsesFiniteUnitRange)
		{
			Assert::AreEqual(0.40f, ClampWheelDriveCommand(0.40f), 1.0e-6f);
			Assert::AreEqual(1.0f, ClampWheelDriveCommand(1.40f), 1.0e-6f);
			Assert::AreEqual(-1.0f, ClampWheelDriveCommand(-1.40f), 1.0e-6f);
			Assert::AreEqual(0.0f, ClampWheelDriveCommand(std::numeric_limits<float>::quiet_NaN()), 1.0e-6f);
		}

		TEST_METHOD(ShouldAccumulateWheelVelocityIntegralOnlyAllowsUnwindingAtSaturation)
		{
			Assert::IsTrue(ShouldAccumulateWheelVelocityIntegral(0.80f, 0.80f, 0.10f));
			Assert::IsFalse(ShouldAccumulateWheelVelocityIntegral(1.20f, 1.0f, 0.10f));
			Assert::IsTrue(ShouldAccumulateWheelVelocityIntegral(1.20f, 1.0f, -0.10f));
			Assert::IsFalse(ShouldAccumulateWheelVelocityIntegral(-1.20f, -1.0f, -0.10f));
			Assert::IsTrue(ShouldAccumulateWheelVelocityIntegral(-1.20f, -1.0f, 0.10f));
			Assert::IsFalse(ShouldAccumulateWheelVelocityIntegral(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.10f));
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
