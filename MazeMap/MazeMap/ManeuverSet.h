#pragma once

#include "Defines.h"
#include "MazeLocation.h"
#include "Vehicle.h"
#include "Maneuver.h"

namespace MazeMap
{
	/*	enum TurnCode : uint8_t
	{
		IP45 = 1,
		IP90 = 2,
		IP135 = 3,
		IP180 = 4,
		S45S = 5,
		S45LS = 6,
		S45LD = 7,
		S90SS = 8,
		S90D = 9,
		S90LS = 10,
		S135S = 11,
		S135D = 12
	};*/
	
	constexpr uint8_t SETSIZE = 17;
	class EXPORT ManeuverSet
	{
	private:
		static ManeuverSet* singleton;

		const Maneuver* _maneuvers[SETSIZE];

		ManeuverSet();
	public:
		static ManeuverSet& GetSet();
		virtual ~ManeuverSet()
		{
			for (size_t i = 0; i < SETSIZE; i++)
			{
				delete _maneuvers[i];
				_maneuvers[i] = NULL;
			}
		}

		const Maneuver& operator[](uint8_t index) { return *(_maneuvers[index]); }
		const Maneuver& operator[](uint8_t index) const { return *(_maneuvers[index]); }

		const Maneuver& operator[](ManeuverCode index)
		{
			return (*const_cast<const ManeuverSet*>(this))[index];
		}
		const Maneuver& operator[](ManeuverCode index) const
		{
			index = static_cast<ManeuverCode>(index & (~MIRRORED_MANEUVER_FLAG));
			for (size_t i = 0; i < SETSIZE; i++)
			{
				if (_maneuvers[i]->GetManeuverID() == index)
				{
					return (*_maneuvers[i]);
				}
			}
			return (*_maneuvers[0]);
		}
		uint8_t size() { return SETSIZE; }
		uint8_t size() const { return SETSIZE; }


		bool SupportsDiagonalEntry() const;
		bool SupportsStraightEntry() const;

		uint8_t GetStepCount(ManeuverCode code) const;
		RelativeDirectionalDistance GetStep(ManeuverCode code, uint8_t index) const;

		float GetCost(ManeuverCode code, const Vehicle& vehicle, float cellSize) const
		{
			if (code == MC_NONE)
			{
				return 0.0f;
			}
			if (code <= S31)
			{
				return -1.0f;
			}
			return (*this)[code].GetCost(vehicle);
		}
		float GetEntrySpeed(ManeuverCode code, const Vehicle& vehicle, float cellSize) const
		{
			if (code == MC_NONE)
			{
				return 0.0f;
			}
			if (code <= S31)
			{
				return -1.0f;
			}
			return (*this)[code].GetEntrySpeed(vehicle);
		}
		float GetExitSpeed(ManeuverCode code, const Vehicle& vehicle, float cellSize) const
		{
			if (code == MC_NONE)
			{
				return 0.0f;
			}
			if (code <= S31)
			{
				return -1.0f;
			}
			return (*this)[code].GetExitSpeed(vehicle);
		}

		bool IsValidMove(ManeuverCode code, DirectionalLocation start, const Maze& maze) const
		{
			if (code <= S31)
			{
				for (uint8_t i = 0; i < code; i++)
				{
					if (!maze.IsAccessibleLocation(start.MoveForward(i).GetLocation()))
					{
						return false;
					}
				}
				return true;
			}
			else
			{

			}
		}

		ManeuverCode GetReverseCode(ManeuverCode code) const
		{
			if (code <= S31)
			{
				return code;
			}
			return (*this)[code].GetBackwardsManeuverID() ^ (code&MIRRORED_MANEUVER_FLAG);
		}

		DirectionalLocation Move(ManeuverCode code, DirectionalLocation start) const
		{
			if (code <= S31)
			{
				return start.MoveForward(code);
			}

			const Maneuver& man = (*this)[code];
			return man.Move(start, code & MIRRORED_MANEUVER_FLAG);
		}

		uint8_t DistanceTravelled(ManeuverCode code) const
		{
			if (code <= S31)
			{
				return static_cast<uint8_t>(code);
			}

			const Maneuver& man = (*this)[code];
			return man.DistanceTravelled();
		}

		void SortByCost(const Vehicle& vehicle, float cellSize);
	};
}


