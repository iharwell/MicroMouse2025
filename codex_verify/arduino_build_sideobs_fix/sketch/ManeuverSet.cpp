#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ManeuverSet.cpp"
#include "pch.h"
#include "ManeuverSet.h"

namespace MazeMap
{
	ManeuverSet* ManeuverSet::singleton = NULL;

	ManeuverSet& ManeuverSet::GetSet()
	{
		if (singleton == NULL)
		{
			singleton = new ManeuverSet();
		}
		return *singleton;
	}

	ManeuverSet::ManeuverSet()
	{
		int i = 0;
		_maneuvers[i++] = new TurnInPlace45();
		_maneuvers[i++] = new TurnInPlace90();
		_maneuvers[i++] = new TurnInPlace135();
		_maneuvers[i++] = new TurnInPlace180();

		_maneuvers[i++] = new Smooth45LongStraight();
		_maneuvers[i++] = new Smooth45LongDiagonal();
		_maneuvers[i++] = new Smooth45ShortStraight();
		_maneuvers[i++] = new Smooth45ShortDiagonal();

		_maneuvers[i++] = new Smooth90LongStraight();
		//_maneuvers[i++] = new Smooth90LongDiagonal();
		_maneuvers[i++] = new Smooth90ShortStraight();
		_maneuvers[i++] = new Smooth90ShortDiagonal();

		_maneuvers[i++] = new Smooth135LongStraight();
		_maneuvers[i++] = new Smooth135LongDiagonal();
		_maneuvers[i++] = new Smooth135ShortStraight();
		_maneuvers[i++] = new Smooth135ShortDiagonal();

		_maneuvers[i++] = new Smooth180LongStraight();
		_maneuvers[i++] = new Smooth180ShortStraight();

		//_maneuvers[i++] = new Smooth90ExtraLongStraight();
		//_maneuvers[i++] = new Smooth180ExtraLongStraight();
	}

	void ManeuverSet::SortByCost(const Vehicle& vehicle, float cellSize)
	{
		float costs[SETSIZE];
#ifdef _WINDOWS
		ManeuverCode codes[SETSIZE];
#endif

		for (int i = 0; i < SETSIZE; ++i)
		{
			costs[i] = _maneuvers[i]->GetCost(vehicle) / (0.1f+_maneuvers[i]->DistanceTravelled());

#ifdef _WINDOWS
			codes[i] = _maneuvers[i]->GetManeuverID();
#endif
		}

		for (int i = 0; i < SETSIZE; ++i)
		{
			for (int j = i + 1; j < SETSIZE; ++j)
			{
				if(costs[i] < costs[j])
				{
					const Maneuver* tmp = _maneuvers[i];
					float ctmp = costs[i];
#ifdef _WINDOWS
					ManeuverCode codeTmp = _maneuvers[i]->GetManeuverID();
#endif

					_maneuvers[i] = _maneuvers[j];
					costs[i] = costs[j];
#ifdef _WINDOWS
					codes[i] = codes[j];
					codes[j] = codeTmp;
#endif

					_maneuvers[j] = tmp;
					costs[j] = ctmp;
				}
			}
		}

		//delete costs;
	}

	uint8_t ManeuverSet::GetStepCount(ManeuverCode code) const
	{
		if (code == MC_NONE)
		{
			return 0;
		}
		if (code <= S31)
		{
			return 1;
		}
		return (*this)[code].GetStepCount();
	}
	RelativeDirectionalDistance ManeuverSet::GetStep(ManeuverCode code, uint8_t index) const
	{
		if (code <= S31)
		{
			if (index == 0)
			{
				return RelativeDirectionalDistance(Forward, code);
			}
			return RelativeDirectionalDistance(Forward, 0);
		}
		if (code & MIRRORED_MANEUVER_FLAG)
		{
			RelativeDirectionalDistance rdd((*this)[code].GetStep(index));
			rdd = RelativeDirectionalDistance(-rdd.GetDirection(), rdd.GetDistance());

			return rdd;
		}
		return (*this)[code].GetStep(index);
	}
}