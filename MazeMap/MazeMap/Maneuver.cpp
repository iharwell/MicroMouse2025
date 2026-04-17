#include "pch.h"
#include "Maneuver.h"
#include <algorithm>
#include <cmath>
namespace MazeMap
{
	namespace Detail
	{
		bool TryComputeSmoothTurnManeuverPoint(
			float radiusInCells,
			float radians,
			float turnInDistanceCells,
			float preTurnDistanceCells,
			float postTurnDistanceCells,
			float traveledDistanceCells,
			float forwardSpeedMps,
			float cellSizeM,
			ManeuverPoint& point) noexcept
		{
			point = ManeuverPoint(0.0f, 0.0f, 0.0f, 0.0f, forwardSpeedMps);
			if (!std::isfinite(traveledDistanceCells) ||
				!std::isfinite(forwardSpeedMps) ||
				!std::isfinite(cellSizeM) ||
				!(cellSizeM > 0.0f) ||
				!std::isfinite(radiusInCells) ||
				!(radiusInCells > 0.0f) ||
				!std::isfinite(radians) ||
				!(MazeMap::Math::Absf(radians) > 0.0f) ||
				!std::isfinite(turnInDistanceCells) ||
				!(turnInDistanceCells >= 0.0f) ||
				!std::isfinite(preTurnDistanceCells) ||
				!(preTurnDistanceCells >= 0.0f) ||
				!std::isfinite(postTurnDistanceCells) ||
				!(postTurnDistanceCells >= 0.0f))
			{
				return false;
			}

			const float constantTurnDistanceCells = (radians * radiusInCells) - turnInDistanceCells;
			const float totalDistanceCells =
				preTurnDistanceCells +
				turnInDistanceCells +
				(radians * radiusInCells) +
				postTurnDistanceCells;
			if (!(totalDistanceCells > 0.0f) || !std::isfinite(totalDistanceCells))
			{
				return false;
			}

			const float clampedDistanceCells = (std::clamp)(traveledDistanceCells, 0.0f, totalDistanceCells);
			const float turnSign = (radians < 0.0f) ? -1.0f : 1.0f;
			const float rampDistanceCells = turnInDistanceCells;
			const float preTurnEndCells = preTurnDistanceCells;
			const float rampInEndCells = preTurnEndCells + rampDistanceCells;
			const float constantEndCells = rampInEndCells + constantTurnDistanceCells;
			const float rampOutEndCells = constantEndCells + rampDistanceCells;
			const float radiusMeters = radiusInCells * cellSizeM;

			if (clampedDistanceCells <= preTurnEndCells)
			{
				return point.IsFinite();
			}

			if (rampDistanceCells <= 0.0f)
			{
				if (clampedDistanceCells <= constantEndCells)
				{
					const float curveDistanceCells = clampedDistanceCells - preTurnEndCells;
					point.Theta = turnSign * (curveDistanceCells / radiusInCells);
					point.Omega = turnSign * (forwardSpeedMps / radiusMeters);
				}
				else
				{
					point.Theta = radians;
				}
				return point.IsFinite();
			}

			if (clampedDistanceCells <= rampInEndCells)
			{
				const float rampDistanceTravelledCells = clampedDistanceCells - preTurnEndCells;
				point.Theta =
					turnSign *
					((rampDistanceTravelledCells * rampDistanceTravelledCells) /
						(2.0f * rampDistanceCells * radiusInCells));
				point.Omega =
					turnSign *
					forwardSpeedMps *
					(rampDistanceTravelledCells / (rampDistanceCells * radiusMeters));
				return point.IsFinite();
			}

			if (clampedDistanceCells <= constantEndCells)
			{
				const float curveDistanceTravelledCells = clampedDistanceCells - rampInEndCells;
				point.Theta =
					turnSign *
					((0.5f * rampDistanceCells) + curveDistanceTravelledCells) /
					radiusInCells;
				point.Omega = turnSign * (forwardSpeedMps / radiusMeters);
				return point.IsFinite();
			}

			if (clampedDistanceCells <= rampOutEndCells)
			{
				const float rampDistanceTravelledCells = clampedDistanceCells - constantEndCells;
				point.Theta =
					turnSign *
					((0.5f * rampDistanceCells) +
						constantTurnDistanceCells +
						rampDistanceTravelledCells -
						((rampDistanceTravelledCells * rampDistanceTravelledCells) /
							(2.0f * rampDistanceCells))) /
					radiusInCells;
				point.Omega =
					turnSign *
					forwardSpeedMps *
					(1.0f - (rampDistanceTravelledCells / rampDistanceCells)) /
					radiusMeters;
				return point.IsFinite();
			}

			point.Theta = radians;
			return point.IsFinite();
		}
	}
}
