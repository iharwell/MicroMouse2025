#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\CompactPath.h"
#pragma once
#include "defines.h"
#include "MazeMask.h"
#include "Maze.h"
#include "Cell.h"
#include "Path.h"

namespace MazeMap
{
	class EXPORT CompactPath
	{
	private:
		MazeMask _memberCells;
		CellCoordinates _start;
	public:
		//CompactPath();
		CompactPath(const Path<PATH_SIZE>& path);

		void Unpack(const Maze& maze, Path<PATH_SIZE>& destination);
	};
}
