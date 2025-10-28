#ifndef MAZE_H
#define MAZE_H

#include "Defines.h"
#include "CellCoordinates.h"
#include "MazeLocation.h"
#include "Cell.h"
#include "MazeMask.h"

namespace MazeMap
{
	class EXPORT Maze
	{
	private:
		Cell _cells[16][16];
		float _cellDimension;
		CellCoordinates _goal;
		MazeMask _reachable;
		bool _complete;
		bool _reachableCalculated;
		bool _goalFound;

		void FindReachables(MazeMask& mask, CellCoordinates cell) const;
	public:
		Maze();
		Maze(float cellDimension);

		uint8_t GetXSize();
		uint8_t GetXSize() const;
		uint8_t GetYSize();
		uint8_t GetYSize() const;

		Cell& operator()(uint8_t x, uint8_t y);
		const Cell&  operator()(uint8_t x, uint8_t y) const;
		Cell& operator[](CellCoordinates coords);
		const Cell& operator[](CellCoordinates coords) const;

		void SetWall(Cell& cell, Direction direction, WallState state);

		Cell& Index(int x, int y);
		const Cell& Index(int x, int y) const;
		Cell& Index(CellCoordinates coords);
		const Cell& Index(CellCoordinates coords) const;

		float GetCellDimension();
		float GetCellDimension() const;

		bool HasFoundGoal();

		MazeMask GetReachableMask() const;
		MazeMask GetReachableMask();

		CellCoordinates GetGoalLowerLeft();
		CellCoordinates GetGoalLowerLeft() const;

		bool IsComplete();

		void PreCalculate();

		bool IsAccessibleLocation(MazeLocation location);
		bool IsAccessibleLocation(MazeLocation location) const;
	};
}

#endif