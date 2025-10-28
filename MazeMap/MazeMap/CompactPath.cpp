#include "pch.h"
#include "CompactPath.h"

namespace MazeMap
{
	CompactPath::CompactPath(const Path<PATH_SIZE>& path)
		: _memberCells(false)
		, _start(path[0])
	{
		for (uint8_t i = 0; i < path.GetSize(); i++)
		{
			_memberCells.SetFlag(path[i], true);
		}
	}

	void CompactPath::Unpack(const Maze& maze, Path<PATH_SIZE>& destination)
	{
		destination.clear();
		MazeMask tmp = MazeMask(_memberCells);

		CellCoordinates current = _start;

		destination.push_back(current);
		tmp.SetFlag(current, false);

		bool stepMade;
		do
		{
			stepMade = false;
			for (Direction d = Direction::Up; d <= Direction::Right; d <<= 1)
			{
				if (   current.IsValidMove(d)
					&& tmp[current >> d]
					&& (maze[current].GetWall(d) == WallState::NoWall))
				{
					current = current >> d;
					tmp.SetFlag(current, false);
					destination.push_back(current);
					stepMade = true;
				}
			}
		} while (stepMade)
			;
	}
}