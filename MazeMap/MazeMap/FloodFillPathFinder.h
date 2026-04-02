#pragma once
// Declares the flood-fill path finder that turns maze reachability into shortest-cell exploration paths.

#include "MaskQueue.h"
#include "PathFinder.h"

namespace MazeMap
{
	// Implements the classic flood-fill planner used by search-mode exploration and goal routing.
	class EXPORT FloodFillPathFinder : public PathFinder
	{
	private:
		MaskQueue _queue;
		MazeMask _alpha;
		MazeMask _beta;
		uint16_t _distances[16][16];

	public:
		FloodFillPathFinder(const Maze& maze, const Vehicle& vehicle);

		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void PathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, Path<PATH_SIZE>& result) override;
		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE * 2>& result) override;

		// Builds a path from the designated start cell to the nearest unknown cell, and stores the result in the provided Path object.
		virtual void PathToNearestUnknown(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result) override;
		// Builds a path from the designated start cell to the nearest unknown cell, and stores the result in the provided Path object.
		virtual void HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result) override;

		// Builds a path from the designated start cell to the goal of the maze, and stores the result in the provided Path object.
		virtual void PathToGoal(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result) override;
		// Builds a path from the designated start cell to the goal of the maze, and stores the result in the provided Path object.
		virtual void HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result) override;

	protected:
		// Builds the distance grid starting from the cells flagged in the queue.
		// To use, add the desired cells to the queue, but do not swap pages on it, and then call this function.
		virtual void SetupDistances();
	};
}
