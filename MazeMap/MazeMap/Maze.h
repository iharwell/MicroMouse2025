#ifndef MAZE_H
#define MAZE_H

#include "Defines.h"
#include "Cell.h"

namespace MazeMap
{
	class EXPORT Maze
	{
	private:
		Cell _cells[16][16];
		float _cellDimension;
	public:
		Maze();
		Maze(float cellDimension);

		int GetXSize();
		int GetXSize() const;
		int GetYSize();
		int GetYSize() const;

		Cell& operator()(int x, int y);
		const Cell&  operator()(int x, int y) const;

		void SetWall(Cell& cell, Direction direction, WallState state);

		Cell& Index(int x, int y);
		const Cell& Index(int x, int y) const;

		float GetCellDimension();
		float GetCellDimension() const;
	};
}

#endif