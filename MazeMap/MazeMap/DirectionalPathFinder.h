#pragma once
#include "Defines.h"
#include "PathFinder.h"
#include "DirectionalLocation.h"
#include "HalfStepPath.h"

namespace MazeMap
{
	class EXPORT DirectionalPathFinder : public PathFinder
	{
	private:
		MaskQueue _queue;
		float _data[32][32][8];
		float _lastEstimatedTime;
	public:
		DirectionalPathFinder(const Maze& maze, const Vehicle& vehicle);

		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE*2>& result) override;

		// Builds a path from the designated start cell to the nearest unknown cell, and stores the result in the provided Path object.
		virtual void HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result) override;

		// Builds a path from the designated start cell to the goal of the maze, and stores the result in the provided Path object.
		virtual void HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result) override;

		float GetLastEstimatedTime();
	protected:
		void EvaluateCosts();
		void UpdateCell(uint8_t row, uint8_t col);

		// Assigns costs to all cells reachable from the given location, 
		// with the points inline recieving costs as follows:
		// For the direction of arrival, assign the cost to arrive at the highest turning speed.
		// For the directions at 45 or 90 degrees off the direction of arrival, assign the costs
		// associated with arriving at the speed appropriate for turns of that angle.
		void RadiateLocation(MazeLocation loc);
		void Reset();
		float Cost(DirectionalLocation dirLoc);
		void UpdateCost(DirectionalLocation dirLoc, float cost);
		float Cost(MazeLocation loc, Direction d);
		void UpdateCost(MazeLocation loc, Direction d, float cost);
		Direction GetAscendDirection(MazeLocation loc, Direction d);
		Direction GetDescendDirection(MazeLocation loc, Direction d);
		void DescendGradient(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result);
		//uint8_t GetPriorStraightLength(MazeLocation currentLoc, Direction currentDirection);
		//Direction GetPriorDirection(MazeLocation currentLoc, Direction currentDirection);
	};
}
