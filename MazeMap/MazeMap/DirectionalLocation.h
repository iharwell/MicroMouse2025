#pragma once
#include "Defines.h"
#include "Direction.h"
#include "CellCoordinates.h"
#include "MazeLocation.h"

namespace MazeMap
{
	class DirectionalLocation
	{
	private:
		MazeLocation _loc;
		Direction _dir;
	public:
		EXPORT DirectionalLocation();
		EXPORT DirectionalLocation(MazeLocation loc, Direction dir);
		EXPORT DirectionalLocation(uint8_t halfX, uint8_t halfY, Direction dir);

		EXPORT MazeLocation GetLocation();
		EXPORT MazeLocation GetLocation() const;

		EXPORT Direction GetDirection();
		EXPORT Direction GetDirection() const;

		EXPORT CellCoordinates GetFollowingCell();

		EXPORT DirectionalLocation Turn(RelativeDirection relDir);
		MAZEMAP_INLINE DirectionalLocation MoveForward(uint8_t halfSteps)
		{
			int8_t dx = 0;
			int8_t dy = 0;
			if ((GetDirection() & Direction::Up))
			{
				dy = halfSteps;
			}
			else if ((GetDirection() & Direction::Down))
			{
				dy = -halfSteps;
			}
			if ((GetDirection() & Direction::Left))
			{
				dx = -halfSteps;
			}
			else if ((GetDirection() & Direction::Right))
			{
				dx = halfSteps;
			}
			return DirectionalLocation(_loc.GetX() + dx, _loc.GetY() + dy, _dir);
		}
		MAZEMAP_INLINE DirectionalLocation MoveForward(uint8_t halfSteps) const
		{
			return const_cast<DirectionalLocation*>(this)->MoveForward(halfSteps);
		}

		EXPORT DirectionalLocation operator>>(uint8_t halfSteps);
		EXPORT DirectionalLocation operator>>(RelativeDirection relDir);
		MAZEMAP_INLINE DirectionalLocation operator>>(RelativeDirectionalDistance instruction)
		{
			DirectionalLocation loc = (*this);
			loc = loc.Turn(instruction.GetDirection());
			loc = loc.MoveForward(instruction.GetDistance());
			return loc;
		}
		EXPORT bool operator==(const DirectionalLocation& other);
		EXPORT bool operator==(const DirectionalLocation& other) const;
	};
}
















