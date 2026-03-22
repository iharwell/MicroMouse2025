#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ManeuverSet.h"
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
	class ManeuverSet
	{
	private:
		static ManeuverSet* singleton;

		const Maneuver* _maneuvers[SETSIZE];

		EXPORT ManeuverSet();
	public:
		static EXPORT ManeuverSet& GetSet();
		MAZEMAP_INLINE virtual ~ManeuverSet()
		{
			for (size_t i = 0; i < SETSIZE; i++)
			{
				delete _maneuvers[i];
				_maneuvers[i] = NULL;
			}
		}

		MAZEMAP_INLINE const Maneuver& operator[](uint8_t index) { return *(_maneuvers[index]); }
		MAZEMAP_INLINE const Maneuver& operator[](uint8_t index) const { return *(_maneuvers[index]); }

		MAZEMAP_INLINE const Maneuver& operator[](ManeuverCode index)
		{
			return (*const_cast<const ManeuverSet*>(this))[index];
		}
		MAZEMAP_INLINE const Maneuver& operator[](ManeuverCode index) const
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
		MAZEMAP_INLINE uint8_t size() { return SETSIZE; }
		MAZEMAP_INLINE uint8_t size() const { return SETSIZE; }


		EXPORT bool SupportsDiagonalEntry() const;
		EXPORT bool SupportsStraightEntry() const;

		EXPORT uint8_t GetStepCount(ManeuverCode code) const;
		EXPORT RelativeDirectionalDistance GetStep(ManeuverCode code, uint8_t index) const;

		MAZEMAP_INLINE float GetCost(ManeuverCode code, const Vehicle& vehicle, float cellSize) const
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
		MAZEMAP_INLINE float GetEntrySpeed(ManeuverCode code, const Vehicle& vehicle, float cellSize) const
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
		MAZEMAP_INLINE float GetExitSpeed(ManeuverCode code, const Vehicle& vehicle, float cellSize) const
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

		MAZEMAP_INLINE bool IsValidMove(ManeuverCode code, DirectionalLocation start, const Maze& maze) const
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
				return (*this)[code].IsValidMove(start, maze, (code & MIRRORED_MANEUVER_FLAG) == MIRRORED_MANEUVER_FLAG);
			}
		}

		MAZEMAP_INLINE ManeuverCode GetReverseCode(ManeuverCode code) const
		{
			if (code <= S31)
			{
				return code;
			}
			return (*this)[code].GetBackwardsManeuverID() ^ (code&MIRRORED_MANEUVER_FLAG);
		}

		MAZEMAP_INLINE DirectionalLocation Move(ManeuverCode code, DirectionalLocation start) const
		{
			if (code <= S31)
			{
				return start.MoveForward(code);
			}

			const Maneuver& man = (*this)[code];
			return man.Move(start, code & MIRRORED_MANEUVER_FLAG);
		}

		MAZEMAP_INLINE uint8_t DistanceTravelled(ManeuverCode code) const
		{
			if (code <= S31)
			{
				return static_cast<uint8_t>(code);
			}

			const Maneuver& man = (*this)[code];
			return man.DistanceTravelled();
		}

		EXPORT void SortByCost(const Vehicle& vehicle, float cellSize);
	};
}












