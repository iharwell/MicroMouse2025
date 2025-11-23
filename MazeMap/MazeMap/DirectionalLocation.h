#pragma once
#include "Defines.h"
#include "Direction.h"
#include "CellCoordinates.h"
#include "MazeLocation.h"

namespace MazeMap
{
	class EXPORT DirectionalLocation
	{
	private:
		MazeLocation _loc;
		Direction _dir;
	public:
		DirectionalLocation();
		DirectionalLocation(MazeLocation loc, Direction dir);
		DirectionalLocation(uint8_t halfX, uint8_t halfY, Direction dir);

		MazeLocation GetLocation();
		MazeLocation GetLocation() const;

		Direction GetDirection();
		Direction GetDirection() const;

		CellCoordinates GetFollowingCell();

		DirectionalLocation Turn(RelativeDirection relDir);
		DirectionalLocation MoveForward(uint8_t halfSteps)
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

		DirectionalLocation operator>>(uint8_t halfSteps);
		DirectionalLocation operator>>(RelativeDirection relDir);
		inline DirectionalLocation operator>>(RelativeDirectionalDistance instruction)
		{
			DirectionalLocation loc = (*this);
			loc = loc.Turn(instruction.GetDirection());
			loc = loc.MoveForward(instruction.GetDistance());
			return loc;
		}
		bool operator==(const DirectionalLocation& other);
		bool operator==(const DirectionalLocation& other) const;
	};
}

