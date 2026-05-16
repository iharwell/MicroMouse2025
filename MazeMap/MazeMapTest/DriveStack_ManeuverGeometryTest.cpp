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

		std::wstring Message(const wchar_t* label, ManeuverCode code)
		{
			std::wstringstream message;
			message << label << L" " << CodeLabel(code);
			return message.str();
		}

		std::wstring Message(const wchar_t* label, ManeuverCode code, const wchar_t* detail, float value)
		{
			std::wstringstream message;
			message << label << L" " << CodeLabel(code) << L" " << detail << L"=" << value;
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
					AppendFailure(failures, Message(L"MM00_GEOM_ENDPOINT", code));
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
					AppendFailure(failures, Message(L"MM00_GEOM_INSTANCE_ENDPOINT", code));
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
					AppendFailure(failures, Message(L"MM00_GEOM_DISTANCE", code));
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
					AppendFailure(failures, Message(L"MM00_GEOM_INSTANCE_DISTANCE", code));
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
					AppendFailure(failures, Message(L"MM00_GEOM_STRAIGHT_POINT_TRACKING", code));
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
						AppendFailure(failures, Message(L"MM00_GEOM_INSTANCE_ENDPOINT", code));
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
						AppendFailure(failures, Message(L"MM00_GEOM_DISTANCE", code));
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
						AppendFailure(failures, Message(L"MM00_GEOM_INSTANCE_DISTANCE", code));
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
						AppendFailure(failures, Message(L"MM00_GEOM_REVERSE_ENDPOINT", code));
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
						AppendFailure(failures, Message(L"MM00_GEOM_DISTANCE_FINITE", code));
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
						AppendFailure(failures, Message(L"MM00_GEOM_DISTANCE_NONNEGATIVE", code));
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
				if (!(CodeDegrees(baseCode) > 0))
				{
					AppendFailure(failures, Message(L"MM00_GEOM_CODE_DEGREES_POSITIVE", baseCode));
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
					AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_CODE_DEGREES", baseCode));
				}
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CodeDegrees_NoneIsZero)
		{
			std::wstring failures;

			if (CodeDegrees(MC_NONE) != 0)
			{
				AppendFailure(failures, L"MM00_GEOM_CODE_DEGREES_NONE");
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_CodeDegrees_StraightIsZero)
		{
			std::wstring failures;

			if (CodeDegrees(S6) != 0)
			{
				AppendFailure(failures, L"MM00_GEOM_CODE_DEGREES_STRAIGHT");
			}

			AssertNoFailures(failures);
		}

		TEST_METHOD(MM00_InPlaceMirrors_DoNotSupportPointTracking)
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
			std::wstring failures;

			for (ManeuverCode code : kCatalogTurnCodes)
			{
				if (!set.SupportsPointTracking(code) && set.SupportsPointTracking(Mirrored(code)))
				{
					AppendFailure(failures, Message(L"MM00_GEOM_IP_POINT_TRACKING", code));
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

				if (!(set.GetTravelDistanceMeters(code, cellSizeM) > 0.0f))
				{
					AppendFailure(failures, Message(L"MM00_GEOM_PROFILE_DISTANCE", code));
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
					if (!TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_PROFILE_SAMPLE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) && !std::isfinite(point.X))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_POINT_X_FINITE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) && !std::isfinite(point.Y))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_POINT_Y_FINITE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) && !std::isfinite(point.Theta))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_POINT_THETA_FINITE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) && !std::isfinite(point.Omega))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_POINT_OMEGA_FINITE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) && !std::isfinite(point.Velocity))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_POINT_VELOCITY_FINITE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) &&
						!NearlyEqual(kSampleSpeedMps, point.Velocity, kDistanceToleranceM))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_PROFILE_VELOCITY", code, L"fraction", fraction));
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
						AppendFailure(failures, Message(L"MM00_GEOM_THETA_MONOTONIC", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) &&
						!(point.Theta >= -kAngleToleranceRad))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_THETA_NONNEGATIVE", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) &&
						!(point.Theta <= expectedFinalThetaRad + kAngleToleranceRad))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_THETA_BOUND", code, L"fraction", fraction));
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
					if (TryProfilePoint(set, code, totalDistanceM * fraction, point, cellSizeM) &&
						!(point.Omega >= -kOmegaToleranceRadps))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_OMEGA_NONNEGATIVE", code, L"fraction", fraction));
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
					AppendFailure(failures, Message(L"MM00_GEOM_FINAL_THETA", code));
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
					AppendFailure(failures, Message(L"MM00_GEOM_FINAL_OMEGA", code));
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
					if (!TryProfilePoint(set, baseCode, totalDistanceM * fraction, point, cellSizeM))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_BASE_SAMPLE", baseCode, L"fraction", fraction));
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
					if (!TryProfilePoint(set, mirroredCode, totalDistanceM * fraction, point, cellSizeM))
					{
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_SAMPLE", mirroredCode, L"fraction", fraction));
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
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_X", baseCode, L"fraction", fraction));
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
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_Y", baseCode, L"fraction", fraction));
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
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_THETA", baseCode, L"fraction", fraction));
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
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_OMEGA", baseCode, L"fraction", fraction));
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
						AppendFailure(failures, Message(L"MM00_GEOM_MIRROR_VELOCITY", baseCode, L"fraction", fraction));
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
							AppendFailure(failures, Message(L"MM00_GEOM_DTHETA_DS_PREVIOUS_SAMPLE", code, L"fraction", fraction));
						}
						if (!TryProfilePoint(set, code, distanceM, point, cellSizeM))
						{
							AppendFailure(failures, Message(L"MM00_GEOM_DTHETA_DS_CURRENT_SAMPLE", code, L"fraction", fraction));
						}
						if (!TryProfilePoint(set, code, distanceM + epsilonM, point, cellSizeM))
						{
							AppendFailure(failures, Message(L"MM00_GEOM_DTHETA_DS_NEXT_SAMPLE", code, L"fraction", fraction));
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
							AppendFailure(failures, Message(L"MM00_GEOM_DTHETA_DS", code, L"error", error));
						}
					}
				}
			}

			AssertNoFailures(failures);
		}
	};
}
