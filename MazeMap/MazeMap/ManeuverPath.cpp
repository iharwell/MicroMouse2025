#include "pch.h"
#include "ManeuverPath.h"
#include "ManeuverSet.h"

namespace MazeMap
{
	namespace
	{
		constexpr uint16_t kHalfStepPathCapacity = PATH_SIZE * 2;

		struct CandidateMatch
		{
			ManeuverCode Code;
			uint16_t ConsumedPoints;
			DirectionalLocation End;
			uint8_t Category;
		};

		inline uint8_t DirectionIndex(Direction direction)
		{
			for (uint8_t i = 0; i < 8; ++i)
			{
				if (OrdinalDirections[i] == direction)
				{
					return i;
				}
			}
			return 0;
		}

		inline bool IsInPlaceTurnCode(ManeuverCode code)
		{
			const ManeuverCode baseCode = static_cast<ManeuverCode>(code & INVERTED_MIRRORED_MANEUVER_FLAG);
			return baseCode == IP45 || baseCode == IP90 || baseCode == IP135 || baseCode == IP180;
		}

		inline uint8_t MatchCategory(ManeuverCode code)
		{
			if (code <= S31)
			{
				return 1;
			}
			if (IsInPlaceTurnCode(code))
			{
				return 2;
			}
			return 0;
		}

		bool ExpandCodeToHalfStepLocations(
			ManeuverCode code,
			DirectionalLocation start,
			MazeLocation* points,
			uint16_t& pointCount,
			DirectionalLocation& end)
		{
			if (code == MC_NONE || points == nullptr)
			{
				return false;
			}

			pointCount = 1;
			points[0] = start.GetLocation();
			end = start;

			if (code <= S31)
			{
				for (uint8_t i = 0; i < static_cast<uint8_t>(code); ++i)
				{
					end = end.MoveForward(1);
					if (pointCount >= kHalfStepPathCapacity)
					{
						return false;
					}
					points[pointCount] = end.GetLocation();
					++pointCount;
				}
				return true;
			}

			const ManeuverSet& maneuvers = ManeuverSet::GetSet();
			const uint8_t stepCount = maneuvers.GetStepCount(code);
			for (uint8_t i = 0; i < stepCount; ++i)
			{
				RelativeDirectionalDistance step = maneuvers.GetStep(code, i);
				end = end.Turn(step.GetDirection());
				for (uint8_t j = 0; j < step.GetDistance(); ++j)
				{
					end = end.MoveForward(1);
					if (pointCount >= kHalfStepPathCapacity)
					{
						return false;
					}
					points[pointCount] = end.GetLocation();
					++pointCount;
				}
			}
			return true;
		}

		bool CandidateHasHigherPriority(const CandidateMatch& left, const CandidateMatch& right)
		{
			if (left.Category != right.Category)
			{
				return left.Category < right.Category;
			}
			if (left.ConsumedPoints != right.ConsumedPoints)
			{
				return left.ConsumedPoints > right.ConsumedPoints;
			}
			return static_cast<uint8_t>(left.Code & INVERTED_MIRRORED_MANEUVER_FLAG)
				> static_cast<uint8_t>(right.Code & INVERTED_MIRRORED_MANEUVER_FLAG);
		}

		void SortCandidateMatches(CandidateMatch* matches, uint16_t count)
		{
			for (uint16_t i = 0; i < count; ++i)
			{
				for (uint16_t j = i + 1; j < count; ++j)
				{
					if (CandidateHasHigherPriority(matches[j], matches[i]))
					{
						CandidateMatch tmp = matches[i];
						matches[i] = matches[j];
						matches[j] = tmp;
					}
				}
			}
		}

		uint16_t CollectCandidateMatches(
			const HalfStepPath<PATH_SIZE * 2>& hsp,
			uint16_t startIndex,
			DirectionalLocation current,
			CandidateMatch* matches,
			uint16_t maxMatches)
		{
			if (matches == nullptr || startIndex >= hsp.GetSize())
			{
				return 0;
			}

			const uint16_t remainingPointCount = hsp.GetSize() - startIndex;
			uint16_t matchCount = 0;
			MazeLocation expandedPoints[kHalfStepPathCapacity] = {};

			auto tryAddCode = [&](ManeuverCode code)
			{
				if (matchCount >= maxMatches)
				{
					return;
				}

				uint16_t expandedCount = 0;
				DirectionalLocation end;
				if (!ExpandCodeToHalfStepLocations(code, current, expandedPoints, expandedCount, end))
				{
					return;
				}
				if (expandedCount > remainingPointCount)
				{
					return;
				}
				for (uint16_t i = 0; i < expandedCount; ++i)
				{
					if (!(expandedPoints[i] == hsp[startIndex + i]))
					{
						return;
					}
				}
				if (expandedCount == 1 && end.GetDirection() == current.GetDirection())
				{
					return;
				}

				matches[matchCount].Code = code;
				matches[matchCount].ConsumedPoints = expandedCount;
				matches[matchCount].End = end;
				matches[matchCount].Category = MatchCategory(code);
				++matchCount;
			};

			const uint8_t maxStraightDistance = static_cast<uint8_t>((remainingPointCount > 0U) ? (remainingPointCount - 1U) : 0U);
			for (uint8_t distance = maxStraightDistance; distance >= 1; --distance)
			{
				tryAddCode(static_cast<ManeuverCode>(distance));
				if (distance == 1)
				{
					break;
				}
			}

			const ManeuverSet& maneuvers = ManeuverSet::GetSet();
			for (uint8_t i = 0; i < maneuvers.size(); ++i)
			{
				const ManeuverCode code = maneuvers[i].GetManeuverID();
				tryAddCode(code);
				tryAddCode(code | MIRRORED_MANEUVER_FLAG);
			}

			SortCandidateMatches(matches, matchCount);
			return matchCount;
		}

		bool BuildManeuverPathFromHalfStepRecursive(
			const HalfStepPath<PATH_SIZE * 2>& hsp,
			uint16_t currentIndex,
			DirectionalLocation current,
			ManeuverPath& result,
			bool visited[PATH_SIZE * 2][8])
		{
			if (currentIndex >= (hsp.GetSize() - 1U))
			{
				return true;
			}
			if (!(current.GetLocation() == hsp[currentIndex]))
			{
				return false;
			}

			const uint8_t directionIndex = DirectionIndex(current.GetDirection());
			if (visited[currentIndex][directionIndex])
			{
				return false;
			}
			visited[currentIndex][directionIndex] = true;

			CandidateMatch matches[kHalfStepPathCapacity] = {};
			const uint16_t matchCount = CollectCandidateMatches(hsp, currentIndex, current, matches, kHalfStepPathCapacity);
			for (uint16_t i = 0; i < matchCount; ++i)
			{
				const CandidateMatch& match = matches[i];
				if (!result.push_back(match.Code))
				{
					return false;
				}

				const uint16_t nextIndex = currentIndex + match.ConsumedPoints - 1U;
				if (BuildManeuverPathFromHalfStepRecursive(hsp, nextIndex, match.End, result, visited))
				{
					return true;
				}
				result.pop_back();
			}

			return false;
		}
	}

	bool ManeuverPath::FromHalfStep(const HalfStepPath<PATH_SIZE * 2>& hsp, DirectionalLocation start, ManeuverPath& mp)
	{
		mp.clear();
		if (hsp.GetSize() == 0)
		{
			return true;
		}
		if (!(hsp[0] == start.GetLocation()))
		{
			return false;
		}
		if (hsp.GetSize() == 1)
		{
			return true;
		}

		bool visited[PATH_SIZE * 2][8] = {};
		return BuildManeuverPathFromHalfStepRecursive(hsp, 0U, start, mp, visited);

	}
	float ManeuverPath::Cost(const Vehicle& v, float cellSize)
	{
		return const_cast<const ManeuverPath*>(this)->Cost(v, cellSize);
	}
	float ManeuverPath::Cost(const Vehicle& v, float cellSize) const
	{
		float cost = 0.0f;
		ManeuverSet& ms = ManeuverSet::GetSet();
		ManeuverCode prevCode = MC_NONE;
		const size_t stepCount = static_cast<size_t>(GetSize());

		for (size_t i = 0; i < stepCount; i++)
		{
			ManeuverCode code = _steps[i];
			ManeuverCode nextCode = MC_NONE;
			if ((i + 1U) < stepCount)
			{
				nextCode = _steps[i + 1];
			}
			if (code <= S31)
			{
				float c = v.GetStraightLineCost(code * 0.5f * cellSize, ms.GetExitSpeed(prevCode, v, cellSize), ms.GetEntrySpeed(nextCode, v, cellSize));
				cost += c;
			}
			else
			{
				float c = ms.GetCost(code, v, cellSize);
				cost += c;
			}
		}
		return cost;
	}
	DirectionalLocation ManeuverPath::ExecutePath(DirectionalLocation start) const
	{
		ManeuverSet& ms = ManeuverSet::GetSet();
		DirectionalLocation current = start;
		for (int i = 0; i < _size; i++)
		{
			current = ms.Move(_steps[i], current);
		}
		return current;
	}
	DirectionalLocation ManeuverPath::ExecuteReverse(DirectionalLocation end) const
	{
		ManeuverSet& ms = ManeuverSet::GetSet();
		DirectionalLocation current(end.GetLocation(), -end.GetDirection());
		for (int i = _size - 1; i >= 0; i--)
		{
			ManeuverCode reverseCode = ms.GetReverseCode(_steps[i]);
			current = ms.Move(reverseCode, current);
		}
		return DirectionalLocation(current.GetLocation(), -current.GetDirection());
	}
	void ManeuverPath::ToHalfStepPath(DirectionalLocation start, HalfStepPath<PATH_SIZE * 2>& path)
	{
		DirectionalLocation current = start;
		DirectionalLocation tmp = start;
		MazeMap::ManeuverSet& ms = MazeMap::ManeuverSet::GetSet();
		path.clear();
		path.push_back(current.GetLocation());

		for (int i = 0; i < GetSize(); ++i)
		{
			tmp = ms.Move(_steps[i], current);
			for (int j = 0; j < ms.GetStepCount(_steps[i]); j++)
			{
				RelativeDirectionalDistance rdd = ms.GetStep(_steps[i], static_cast<uint8_t>(j));
				RelativeDirection rd = rdd.GetDirection();
				uint8_t distance = rdd.GetDistance();
				current = current >> rd;
				for (uint8_t k = 0; k < distance; k++)
				{
					current = current.MoveForward(1);
					path.push_back(current.GetLocation());
				}
			}
#ifdef _WINDOWS
			if (current.GetLocation() != tmp.GetLocation() || current.GetDirection() != tmp.GetDirection())
			{
				throw std::errc::invalid_seek;
			}
#endif
		}
	}
}
