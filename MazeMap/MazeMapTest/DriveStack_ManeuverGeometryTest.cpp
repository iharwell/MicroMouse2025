#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\Maneuver.h"
#include "..\MazeMap\ManeuverInstance.h"
#include "..\MazeMap\ManeuverSet.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	namespace
	{
		constexpr float kDistanceToleranceM = 1.0e-5f;
		constexpr float kAngleToleranceRad = 2.0e-4f;
		constexpr float kOmegaToleranceRadps = 2.0e-4f;
		constexpr float kDerivativeToleranceRadPerM = 4.0e-2f;
		constexpr float kSampleSpeedMps = 0.7f;

		const ManeuverCode kCatalogTurnCodes[] =
		{
			IP45,
			IP90,
			IP135,
			IP180,
			S45SS,
			S45SD,
			S45LS,
			S45LD,
			S90SS,
			S90SD,
			S90LS,
			S135SS,
			S135SD,
			S135LS,
			S135LD,
			S180SS,
			S180LS,
		};

		const ManeuverCode kRepresentativeStraightCodes[] =
		{
			S1,
			S6,
			S31,
		};

		const float kProfileFractions[] =
		{
			0.0f,
			0.10f,
			0.25f,
			0.50f,
			0.75f,
			0.90f,
			1.0f,
		};

		const float kMirrorProfileFractions[] =
		{
			0.25f,
			0.50f,
			0.75f,
		};

		const float kDerivativeFractions[] =
		{
			0.10f,
			0.25f,
			0.50f,
			0.75f,
			0.90f,
		};

		std::wstring CodeLabel(ManeuverCode code)
		{
			std::wstringstream message;
			message << L"code=" << static_cast<unsigned>(static_cast<uint8_t>(code));
			if ((code & MIRRORED_MANEUVER_FLAG) == MIRRORED_MANEUVER_FLAG)
			{
				message << L"_M";
			}
			return message.str();
		}

		void AppendFailure(std::wstring& failures, const std::wstring& message)
		{
			if (!failures.empty())
			{
				failures += L"\n";
			}
			failures += message;
		}

		void AssertNoFailures(const std::wstring& failures)
		{
			Assert::IsTrue(failures.empty(), failures.c_str());
		}

		bool NearlyEqual(float expected, float actual, float tolerance)
		{
			return std::fabs(expected - actual) <= tolerance;
		}

		ManeuverCode Mirrored(ManeuverCode code)
		{
			return code | MIRRORED_MANEUVER_FLAG;
		}

		DirectionalLocation CompatibleStart(const Maneuver& maneuver)
		{
			if (maneuver.SupportsStraightEntry())
			{
				return DirectionalLocation(MazeLocation(15U, 15U), Up);
			}
			return DirectionalLocation(MazeLocation(15U, 15U), UpRight);
		}

		bool TryProfilePoint(
			const ManeuverSet& set,
			ManeuverCode code,
			float distanceM,
			ManeuverPoint& point,
			float cellSizeM)
		{
			return set.TryGetManeuverPoint(code, distanceM, kSampleSpeedMps, point, cellSizeM);
		}

		float Abs(float value)
		{
			return std::fabs(value);
		}
	}

	TEST_CLASS(DriveStack_ManeuverGeometryTest)
	{
	public:
		TEST_METHOD(MM00_RepresentativeStraight_SetMoveEndpointMatchesForwardCells)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const DirectionalLocation start(MazeLocation(15U, 15U), Up);
			std::wstring failures;

			for (ManeuverCode code : kRepresentativeStraightCodes)
			{
				const DirectionalLocation expectedEnd = start.MoveForward(static_cast<uint8_t>(code));
				const DirectionalLocation actualEnd = set.Move(code, start);
				if (!(expectedEnd == actualEnd))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_ENDPOINT"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected_x=" << static_cast<unsigned>(expectedEnd.GetLocation().GetX())
						<< L"\nexpected_y=" << static_cast<unsigned>(expectedEnd.GetLocation().GetY())
						<< L"\nexpected_dir=" << static_cast<unsigned>(expectedEnd.GetDirection())
						<< L"\nactual_x=" << static_cast<unsigned>(actualEnd.GetLocation().GetX())
						<< L"\nactual_y=" << static_cast<unsigned>(actualEnd.GetLocation().GetY())
						<< L"\nactual_dir=" << static_cast<unsigned>(actualEnd.GetDirection())
						<< L"\ncriterion=actual==expected";
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_RepresentativeStraight_InstanceEndpointMatchesForwardCells)
		{
			const DirectionalLocation start(MazeLocation(15U, 15U), Up);
			std::wstring failures;

			for (ManeuverCode code : kRepresentativeStraightCodes)
			{
				const DirectionalLocation expectedEnd = start.MoveForward(static_cast<uint8_t>(code));
				const ManeuverInstance instance(code, start);
				if (!(expectedEnd == instance.getEnd()))
				{
					const DirectionalLocation actualEnd = instance.getEnd();
					std::wstringstream message;
					message << L"MM00_GEOM_INSTANCE_ENDPOINT"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected_x=" << static_cast<unsigned>(expectedEnd.GetLocation().GetX())
						<< L"\nexpected_y=" << static_cast<unsigned>(expectedEnd.GetLocation().GetY())
						<< L"\nexpected_dir=" << static_cast<unsigned>(expectedEnd.GetDirection())
						<< L"\nactual_x=" << static_cast<unsigned>(actualEnd.GetLocation().GetX())
						<< L"\nactual_y=" << static_cast<unsigned>(actualEnd.GetLocation().GetY())
						<< L"\nactual_dir=" << static_cast<unsigned>(actualEnd.GetDirection())
						<< L"\ncriterion=actual==expected";
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_RepresentativeStraight_SetTravelDistanceMatchesHalfCellCount)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kRepresentativeStraightCodes)
			{
				const float expectedDistanceM = 0.5f * cellSizeM * static_cast<float>(static_cast<uint8_t>(code));
				const float actualDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				if (!NearlyEqual(expectedDistanceM, actualDistanceM, kDistanceToleranceM))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_DISTANCE"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected=" << expectedDistanceM
						<< L"\nactual=" << actualDistanceM
						<< L"\ntolerance=" << kDistanceToleranceM;
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_RepresentativeStraight_InstanceTravelDistanceMatchesHalfCellCount)
		{
			const float cellSizeM = Maze::GetCellDimension();
			const DirectionalLocation start(MazeLocation(15U, 15U), Up);
			std::wstring failures;

			for (ManeuverCode code : kRepresentativeStraightCodes)
			{
				const float expectedDistanceM = 0.5f * cellSizeM * static_cast<float>(static_cast<uint8_t>(code));
				const ManeuverInstance instance(code, start);
				const float actualDistanceM = instance.GetTravelDistanceMeters(cellSizeM);
				if (!NearlyEqual(expectedDistanceM, actualDistanceM, kDistanceToleranceM))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_INSTANCE_DISTANCE"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected=" << expectedDistanceM
						<< L"\nactual=" << actualDistanceM
						<< L"\ntolerance=" << kDistanceToleranceM;
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_RepresentativeStraight_DoesNotSupportPointTracking)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			std::wstring failures;

			for (ManeuverCode code : kRepresentativeStraightCodes)
			{
				if (set.SupportsPointTracking(code))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_STRAIGHT_POINT_TRACKING"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected=false"
						<< L"\nactual=true";
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurns_InstanceEndpointMatchesSetMove)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				const Maneuver& maneuver = set[baseCode];
				const DirectionalLocation start = CompatibleStart(maneuver);

				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const ManeuverInstance instance(code, start);
					const DirectionalLocation actualEnd = set.Move(code, start);
					if (!(actualEnd == instance.getEnd()))
					{
						const DirectionalLocation expectedEnd = instance.getEnd();
						std::wstringstream message;
						message << L"MM00_GEOM_INSTANCE_ENDPOINT"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nexpected_x=" << static_cast<unsigned>(expectedEnd.GetLocation().GetX())
							<< L"\nexpected_y=" << static_cast<unsigned>(expectedEnd.GetLocation().GetY())
							<< L"\nexpected_dir=" << static_cast<unsigned>(expectedEnd.GetDirection())
							<< L"\nactual_x=" << static_cast<unsigned>(actualEnd.GetLocation().GetX())
							<< L"\nactual_y=" << static_cast<unsigned>(actualEnd.GetLocation().GetY())
							<< L"\nactual_dir=" << static_cast<unsigned>(actualEnd.GetDirection())
							<< L"\ncriterion=actual==expected";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurns_SetTravelDistanceMatchesCatalogCells)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				const Maneuver& maneuver = set[baseCode];
				const float expectedDistanceM = maneuver.GetTravelDistanceInCells() * cellSizeM;

				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const float actualDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
					if (!NearlyEqual(expectedDistanceM, actualDistanceM, kDistanceToleranceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_DISTANCE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nexpected=" << expectedDistanceM
							<< L"\nactual=" << actualDistanceM
							<< L"\ntolerance=" << kDistanceToleranceM;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurns_InstanceTravelDistanceMatchesSetDistance)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				const Maneuver& maneuver = set[baseCode];
				const DirectionalLocation start = CompatibleStart(maneuver);

				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const ManeuverInstance instance(code, start);
					const float expectedDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
					const float actualDistanceM = instance.GetTravelDistanceMeters(cellSizeM);
					if (!NearlyEqual(expectedDistanceM, actualDistanceM, kDistanceToleranceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_INSTANCE_DISTANCE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nexpected=" << expectedDistanceM
							<< L"\nactual=" << actualDistanceM
							<< L"\ntolerance=" << kDistanceToleranceM;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurns_ReverseCodeReturnsToStart)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				const Maneuver& maneuver = set[baseCode];
				const DirectionalLocation start = CompatibleStart(maneuver);

				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const DirectionalLocation actualEnd = set.Move(code, start);
					const ManeuverCode reverseCode = set.GetReverseCode(code);
					DirectionalLocation reverseStart = actualEnd;
					const DirectionalLocation returnedToStart = set.Move(reverseCode, reverseStart.Turn(Reverse)).Turn(Reverse);
					if (!(start == returnedToStart))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_REVERSE_ENDPOINT"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nreverse_code=" << CodeLabel(reverseCode)
							<< L"\nexpected_x=" << static_cast<unsigned>(start.GetLocation().GetX())
							<< L"\nexpected_y=" << static_cast<unsigned>(start.GetLocation().GetY())
							<< L"\nexpected_dir=" << static_cast<unsigned>(start.GetDirection())
							<< L"\nactual_x=" << static_cast<unsigned>(returnedToStart.GetLocation().GetX())
							<< L"\nactual_y=" << static_cast<unsigned>(returnedToStart.GetLocation().GetY())
							<< L"\nactual_dir=" << static_cast<unsigned>(returnedToStart.GetDirection())
							<< L"\ncriterion=returned_to_start==start";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurns_TravelDistanceIsFinite)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const float actualDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
					if (!std::isfinite(actualDistanceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_DISTANCE_FINITE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nactual=" << actualDistanceM
							<< L"\ncriterion=isfinite(actual)";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurns_TravelDistanceIsNonnegative)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const float actualDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
					if (!(actualDistanceM >= 0.0f))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_DISTANCE_NONNEGATIVE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nactual=" << actualDistanceM
							<< L"\ncriterion=actual>=0";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CatalogTurnBaseCodes_DeclaredYawDegreesArePositive)
		{
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				const int16_t actualDegrees = CodeDegrees(baseCode);
				if (!(actualDegrees > 0))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_CODE_DEGREES_POSITIVE"
						<< L"\ncode=" << CodeLabel(baseCode)
						<< L"\nactual=" << actualDegrees
						<< L"\ncriterion=actual>0";
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_MirroredCodes_InvertDeclaredYawDegrees)
		{
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				const int16_t baseDegrees = CodeDegrees(baseCode);
				const int16_t mirroredDegrees = CodeDegrees(Mirrored(baseCode));
				if (mirroredDegrees != -baseDegrees)
				{
					std::wstringstream message;
					message << L"MM00_GEOM_MIRROR_CODE_DEGREES"
						<< L"\ncode=" << CodeLabel(baseCode)
						<< L"\nexpected=" << -baseDegrees
						<< L"\nactual=" << mirroredDegrees
						<< L"\nbase_degrees=" << baseDegrees;
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CodeDegrees_NoneIsZero)
		{
			std::wstring failures;

			const int16_t noneDegrees = CodeDegrees(MC_NONE);
			if (noneDegrees != 0)
			{
				std::wstringstream message;
				message << L"MM00_GEOM_CODE_DEGREES_NONE"
					<< L"\ncode=" << CodeLabel(MC_NONE)
					<< L"\nexpected=0"
					<< L"\nactual=" << noneDegrees;
				AppendFailure(failures, message.str());
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CodeDegrees_StraightIsZero)
		{
			std::wstring failures;

			const int16_t straightDegrees = CodeDegrees(S6);
			if (straightDegrees != 0)
			{
				std::wstringstream message;
				message << L"MM00_GEOM_CODE_DEGREES_STRAIGHT"
					<< L"\ncode=" << CodeLabel(S6)
					<< L"\nexpected=0"
					<< L"\nactual=" << straightDegrees;
				AppendFailure(failures, message.str());
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_InPlaceMirrors_DoNotSupportPointTracking)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				const bool baseSupportsPointTracking = set.SupportsPointTracking(code);
				const bool mirroredSupportsPointTracking = set.SupportsPointTracking(Mirrored(code));
				if (!baseSupportsPointTracking && mirroredSupportsPointTracking)
				{
					std::wstringstream message;
					message << L"MM00_GEOM_IP_POINT_TRACKING"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nbase_supports=" << baseSupportsPointTracking
						<< L"\nmirrored_supports=" << mirroredSupportsPointTracking
						<< L"\ncriterion=mirrored_supports==base_supports_when_base_false";
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_TravelDistanceIsPositive)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float actualDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				if (!(actualDistanceM > 0.0f))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_PROFILE_DISTANCE"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nactual=" << actualDistanceM
						<< L"\ncriterion=actual>0";
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_SamplesAreAvailable)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				if (!(totalDistanceM > 0.0f))
				{
					continue;
				}

				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (!TryProfilePoint(set, code, distanceM, point, cellSizeM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_PROFILE_SAMPLE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\ntotal_distance_m=" << totalDistanceM
							<< L"\nexpected_available=true"
							<< L"\nactual_available=false";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_PointXIsFinite)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) && !std::isfinite(point.X))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_POINT_X_FINITE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.X
							<< L"\ncriterion=isfinite(actual)";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_PointYIsFinite)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) && !std::isfinite(point.Y))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_POINT_Y_FINITE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.Y
							<< L"\ncriterion=isfinite(actual)";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_PointThetaIsFinite)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) && !std::isfinite(point.Theta))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_POINT_THETA_FINITE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.Theta
							<< L"\ncriterion=isfinite(actual)";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_PointOmegaIsFinite)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) && !std::isfinite(point.Omega))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_POINT_OMEGA_FINITE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.Omega
							<< L"\ncriterion=isfinite(actual)";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_PointVelocityIsFinite)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) && !std::isfinite(point.Velocity))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_POINT_VELOCITY_FINITE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.Velocity
							<< L"\ncriterion=isfinite(actual)";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_PreserveRequestedVelocity)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) &&
						!NearlyEqual(kSampleSpeedMps, point.Velocity, kDistanceToleranceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_PROFILE_VELOCITY"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nexpected=" << kSampleSpeedMps
							<< L"\nactual=" << point.Velocity
							<< L"\ntolerance=" << kDistanceToleranceM;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_ThetaIsMonotonic)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				float previousThetaRad = -kAngleToleranceRad;
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					if (!TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM))
					{
						continue;
					}

					if (!(point.Theta + kAngleToleranceRad >= previousThetaRad))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_THETA_MONOTONIC"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\nprevious_theta_rad=" << previousThetaRad
							<< L"\nactual_theta_rad=" << point.Theta
							<< L"\ntolerance=" << kAngleToleranceRad
							<< L"\ncriterion=actual+tolerance>=previous";
						AppendFailure(failures, message.str());
					}
					previousThetaRad = point.Theta;
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_ThetaIsNonnegative)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) &&
						!(point.Theta >= -kAngleToleranceRad))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_THETA_NONNEGATIVE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.Theta
							<< L"\nminimum=" << -kAngleToleranceRad
							<< L"\ncriterion=actual>=minimum";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_ThetaStaysWithinFinalHeading)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				const float expectedFinalThetaRad = static_cast<float>(CodeDegrees(code)) * PI_F / 180.0f;
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) &&
						!(point.Theta <= expectedFinalThetaRad + kAngleToleranceRad))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_THETA_BOUND"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nexpected_final_theta_rad=" << expectedFinalThetaRad
							<< L"\nactual=" << point.Theta
							<< L"\ntolerance=" << kAngleToleranceRad
							<< L"\ncriterion=actual<=expected_final+tolerance";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_OmegaIsNonnegative)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				for (float fraction : kProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (TryProfilePoint(set, code, distanceM, point, cellSizeM) &&
						!(point.Omega >= -kOmegaToleranceRadps))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_OMEGA_NONNEGATIVE"
							<< L"\ncode=" << CodeLabel(code)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\nactual=" << point.Omega
							<< L"\nminimum=" << -kOmegaToleranceRadps
							<< L"\ncriterion=actual>=minimum";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_FinalThetaMatchesDeclaredYaw)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				const float expectedFinalThetaRad = static_cast<float>(CodeDegrees(code)) * PI_F / 180.0f;
				ManeuverPoint finalPoint{};
				if (TryProfilePoint(set, code, totalDistanceM, finalPoint, cellSizeM) &&
					!NearlyEqual(expectedFinalThetaRad, finalPoint.Theta, kAngleToleranceRad))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_FINAL_THETA"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected=" << expectedFinalThetaRad
						<< L"\nactual=" << finalPoint.Theta
						<< L"\ntolerance=" << kAngleToleranceRad
						<< L"\ntotal_distance_m=" << totalDistanceM;
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothProfiles_FinalOmegaIsZero)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
				ManeuverPoint finalPoint{};
				if (TryProfilePoint(set, code, totalDistanceM, finalPoint, cellSizeM) &&
					!NearlyEqual(0.0f, finalPoint.Omega, kOmegaToleranceRadps))
				{
					std::wstringstream message;
					message << L"MM00_GEOM_FINAL_OMEGA"
						<< L"\ncode=" << CodeLabel(code)
						<< L"\nexpected=0"
						<< L"\nactual=" << finalPoint.Omega
						<< L"\ntolerance=" << kOmegaToleranceRadps
						<< L"\ntotal_distance_m=" << totalDistanceM;
					AppendFailure(failures, message.str());
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_BaseSamplesAreAvailable)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (!TryProfilePoint(set, baseCode, distanceM, point, cellSizeM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_BASE_SAMPLE"
							<< L"\ncode=" << CodeLabel(baseCode)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\ntotal_distance_m=" << totalDistanceM
							<< L"\nexpected_available=true"
							<< L"\nactual_available=false";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_MirroredSamplesAreAvailable)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const ManeuverCode mirroredCode = Mirrored(baseCode);
				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint point{};
					const float distanceM = totalDistanceM * fraction;
					if (!TryProfilePoint(set, mirroredCode, distanceM, point, cellSizeM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_SAMPLE"
							<< L"\ncode=" << CodeLabel(mirroredCode)
							<< L"\nfraction=" << fraction
							<< L"\ndistance_m=" << distanceM
							<< L"\ntotal_distance_m=" << totalDistanceM
							<< L"\nexpected_available=true"
							<< L"\nactual_available=false";
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_XIsInverted)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const ManeuverCode mirroredCode = Mirrored(baseCode);
				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint rightPoint{};
					ManeuverPoint leftPoint{};
					if (TryProfilePoint(set, baseCode, totalDistanceM * fraction, rightPoint, cellSizeM) &&
						TryProfilePoint(set, mirroredCode, totalDistanceM * fraction, leftPoint, cellSizeM) &&
						!NearlyEqual(-rightPoint.X, leftPoint.X, kDistanceToleranceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_X"
							<< L"\nbase_code=" << CodeLabel(baseCode)
							<< L"\nmirrored_code=" << CodeLabel(mirroredCode)
							<< L"\nfraction=" << fraction
							<< L"\nexpected=" << -rightPoint.X
							<< L"\nactual=" << leftPoint.X
							<< L"\ntolerance=" << kDistanceToleranceM
							<< L"\nright_x=" << rightPoint.X;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_YPreserved)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const ManeuverCode mirroredCode = Mirrored(baseCode);
				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint rightPoint{};
					ManeuverPoint leftPoint{};
					if (TryProfilePoint(set, baseCode, totalDistanceM * fraction, rightPoint, cellSizeM) &&
						TryProfilePoint(set, mirroredCode, totalDistanceM * fraction, leftPoint, cellSizeM) &&
						!NearlyEqual(rightPoint.Y, leftPoint.Y, kDistanceToleranceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_Y"
							<< L"\nbase_code=" << CodeLabel(baseCode)
							<< L"\nmirrored_code=" << CodeLabel(mirroredCode)
							<< L"\nfraction=" << fraction
							<< L"\nexpected=" << rightPoint.Y
							<< L"\nactual=" << leftPoint.Y
							<< L"\ntolerance=" << kDistanceToleranceM;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_ThetaIsInverted)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const ManeuverCode mirroredCode = Mirrored(baseCode);
				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint rightPoint{};
					ManeuverPoint leftPoint{};
					if (TryProfilePoint(set, baseCode, totalDistanceM * fraction, rightPoint, cellSizeM) &&
						TryProfilePoint(set, mirroredCode, totalDistanceM * fraction, leftPoint, cellSizeM) &&
						!NearlyEqual(-rightPoint.Theta, leftPoint.Theta, kAngleToleranceRad))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_THETA"
							<< L"\nbase_code=" << CodeLabel(baseCode)
							<< L"\nmirrored_code=" << CodeLabel(mirroredCode)
							<< L"\nfraction=" << fraction
							<< L"\nexpected=" << -rightPoint.Theta
							<< L"\nactual=" << leftPoint.Theta
							<< L"\ntolerance=" << kAngleToleranceRad
							<< L"\nright_theta=" << rightPoint.Theta;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_OmegaIsInverted)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const ManeuverCode mirroredCode = Mirrored(baseCode);
				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint rightPoint{};
					ManeuverPoint leftPoint{};
					if (TryProfilePoint(set, baseCode, totalDistanceM * fraction, rightPoint, cellSizeM) &&
						TryProfilePoint(set, mirroredCode, totalDistanceM * fraction, leftPoint, cellSizeM) &&
						!NearlyEqual(-rightPoint.Omega, leftPoint.Omega, kOmegaToleranceRadps))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_OMEGA"
							<< L"\nbase_code=" << CodeLabel(baseCode)
							<< L"\nmirrored_code=" << CodeLabel(mirroredCode)
							<< L"\nfraction=" << fraction
							<< L"\nexpected=" << -rightPoint.Omega
							<< L"\nactual=" << leftPoint.Omega
							<< L"\ntolerance=" << kOmegaToleranceRadps
							<< L"\nright_omega=" << rightPoint.Omega;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothMirrorProfiles_VelocityIsPreserved)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				const ManeuverCode mirroredCode = Mirrored(baseCode);
				const float totalDistanceM = set.GetTravelDistanceMeters(baseCode, cellSizeM);
				for (float fraction : kMirrorProfileFractions)
				{
					ManeuverPoint rightPoint{};
					ManeuverPoint leftPoint{};
					if (TryProfilePoint(set, baseCode, totalDistanceM * fraction, rightPoint, cellSizeM) &&
						TryProfilePoint(set, mirroredCode, totalDistanceM * fraction, leftPoint, cellSizeM) &&
						!NearlyEqual(rightPoint.Velocity, leftPoint.Velocity, kDistanceToleranceM))
					{
						std::wstringstream message;
						message << L"MM00_GEOM_MIRROR_VELOCITY"
							<< L"\nbase_code=" << CodeLabel(baseCode)
							<< L"\nmirrored_code=" << CodeLabel(mirroredCode)
							<< L"\nfraction=" << fraction
							<< L"\nexpected=" << rightPoint.Velocity
							<< L"\nactual=" << leftPoint.Velocity
							<< L"\ntolerance=" << kDistanceToleranceM;
						AppendFailure(failures, message.str());
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothPointDerivative_SamplesAreAvailable)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
					const float epsilonM = (std::max)(1.0e-4f, totalDistanceM * 1.0e-4f);

					for (float fraction : kDerivativeFractions)
					{
						const float distanceM = totalDistanceM * fraction;
						ManeuverPoint point{};
						if (!TryProfilePoint(set, code, distanceM - epsilonM, point, cellSizeM))
						{
							std::wstringstream message;
							message << L"MM00_GEOM_DTHETA_DS_PREVIOUS_SAMPLE"
								<< L"\ncode=" << CodeLabel(code)
								<< L"\nfraction=" << fraction
								<< L"\ndistance_m=" << (distanceM - epsilonM)
								<< L"\nexpected_available=true"
								<< L"\nactual_available=false";
							AppendFailure(failures, message.str());
						}
						if (!TryProfilePoint(set, code, distanceM, point, cellSizeM))
						{
							std::wstringstream message;
							message << L"MM00_GEOM_DTHETA_DS_CURRENT_SAMPLE"
								<< L"\ncode=" << CodeLabel(code)
								<< L"\nfraction=" << fraction
								<< L"\ndistance_m=" << distanceM
								<< L"\nexpected_available=true"
								<< L"\nactual_available=false";
							AppendFailure(failures, message.str());
						}
						if (!TryProfilePoint(set, code, distanceM + epsilonM, point, cellSizeM))
						{
							std::wstringstream message;
							message << L"MM00_GEOM_DTHETA_DS_NEXT_SAMPLE"
								<< L"\ncode=" << CodeLabel(code)
								<< L"\nfraction=" << fraction
								<< L"\ndistance_m=" << (distanceM + epsilonM)
								<< L"\nexpected_available=true"
								<< L"\nactual_available=false";
							AppendFailure(failures, message.str());
						}
					}
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_SmoothPointDerivative_MatchesYawRateOverVelocity)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			const float cellSizeM = Maze::GetCellDimension();
			std::wstring failures;

			for (ManeuverCode baseCode : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(baseCode))
				{
					continue;
				}

				for (ManeuverCode code : { baseCode, Mirrored(baseCode) })
				{
					const float totalDistanceM = set.GetTravelDistanceMeters(code, cellSizeM);
					const float epsilonM = (std::max)(1.0e-4f, totalDistanceM * 1.0e-4f);

					for (float fraction : kDerivativeFractions)
					{
						const float distanceM = totalDistanceM * fraction;
						ManeuverPoint previous{};
						ManeuverPoint current{};
						ManeuverPoint next{};

						if (!TryProfilePoint(set, code, distanceM - epsilonM, previous, cellSizeM) ||
							!TryProfilePoint(set, code, distanceM, current, cellSizeM) ||
							!TryProfilePoint(set, code, distanceM + epsilonM, next, cellSizeM))
						{
							continue;
						}

						const float numericDThetaDs = (next.Theta - previous.Theta) / (2.0f * epsilonM);
						const float kinematicDThetaDs = current.Omega / current.Velocity;
						const float error = Abs(numericDThetaDs - kinematicDThetaDs);
						if (!(error <= kDerivativeToleranceRadPerM))
						{
							std::wstringstream message;
							message << L"MM00_GEOM_DTHETA_DS"
								<< L"\ncode=" << CodeLabel(code)
								<< L"\nfraction=" << fraction
								<< L"\nnumeric_dtheta_ds=" << numericDThetaDs
								<< L"\nkinematic_dtheta_ds=" << kinematicDThetaDs
								<< L"\nactual_error=" << error
								<< L"\ntolerance=" << kDerivativeToleranceRadPerM;
							AppendFailure(failures, message.str());
						}
					}
				}
			}

			AssertNoFailures(failures);
		}
	};
}
