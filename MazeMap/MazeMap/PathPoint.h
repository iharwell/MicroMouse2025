#pragma once
#include "Defines.h"
#include "Cell.h"
#include "Maze.h"

namespace MazeMap
{
	class EXPORT PathPoint
	{
	private:
		CellCoordinates _start;
		Direction _startLocation;

		Direction _direction;

		CellCoordinates _end;
		Direction _endLocation;
	};
}

