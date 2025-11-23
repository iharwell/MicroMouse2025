#include "pch.h"
#include "ManeuverPath.h"
#include "ManeuverSet.h"

namespace MazeMap
{

	void ManeuverPath::FromHalfStep(const HalfStepPath<PATH_SIZE * 2>& hsp, const ManeuverPath mp)
	{
		ManeuverSet& ms = ManeuverSet::GetSet();
		DirectionalLocation marker(hsp[0], hsp[0].DirectionTo(hsp[1]));


		for (int i = 0; i < hsp.GetSize(); ++i)
		{
		}

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

		for (size_t i = 0; i < GetSize(); i++)
		{
			ManeuverCode code = _steps[i];
			ManeuverCode nextCode = MC_NONE;
			if (i < (GetSize() - 1))
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
				RelativeDirectionalDistance rdd = ms.GetStep(_steps[i], j);
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