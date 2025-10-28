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

		MazeLocation GetLocation();
		MazeLocation GetLocation() const;

		Direction GetDirection();
		Direction GetDirection() const;

		CellCoordinates GetFollowingCell();

		DirectionalLocation Turn(RelativeDirection relDir);
		DirectionalLocation MoveForward(uint8_t halfSteps);

		DirectionalLocation operator>>(uint8_t halfSteps);
		DirectionalLocation operator>>(RelativeDirection relDir);
	};
}

