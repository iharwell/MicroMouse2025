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
		CellCoordinates _goal;
		bool _goalFound;
	public:
		Maze();
		Maze(float cellDimension);

		uint8_t GetXSize();
		uint8_t GetXSize() const;
		uint8_t GetYSize();
		uint8_t GetYSize() const;

		Cell& operator()(uint8_t x, uint8_t y);
		const Cell&  operator()(uint8_t x, uint8_t y) const;

		void SetWall(Cell& cell, Direction direction, WallState state);

		Cell& Index(int x, int y);
		const Cell& Index(int x, int y) const;

		float GetCellDimension();
		float GetCellDimension() const;

		bool HasFoundGoal();

		CellCoordinates GetGoalLowerLeft();
		CellCoordinates GetGoalLowerLeft() const;

		bool IsComplete();


	};
}

#endif