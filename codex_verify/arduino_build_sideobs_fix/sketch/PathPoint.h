#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\PathPoint.h"
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

