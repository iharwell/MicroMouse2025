#pragma once
#include "defines.h"
#include "Path.h"
#include "Maze.h"
#include "Vehicle.h"
#include "MaskQueue.h"
#include "HalfStepPath.h"
namespace MazeMap
{
	class EXPORT PathFinder
	{
	private:
		const Maze& _maze;
		const Vehicle& _vehicle;
	public:
		PathFinder(const Maze& maze, const Vehicle& vehicle);

		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void PathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, Path<PATH_SIZE>& result);
		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE*2>& result) = 0;

		// Builds a path from the designated start cell to the nearest unknown cell, and stores the result in the provided Path object.
		virtual void PathToNearestUnknown(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result);
		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result) = 0;

		// Builds a path from the designated start cell to the goal of the maze, and stores the result in the provided Path object.
		virtual void PathToGoal(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result);
		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result) = 0;

	protected:

		// Constructs a path by iteratively stepping to the adjacent cell with the lowest cost.
		template <typename T>
		void DescendGradient(CellCoordinates start, Direction startDirection, const T (&cost)[16][16], Path<PATH_SIZE>& result);

		// Returns a reference to the maze that this PathFinder is linked to.
		const Maze& GetMaze() const;

		// Returns a reference to the maze that this PathFinder is linked to.
		const Maze& GetMaze();

		// Returns a reference to the vehicle that this PathFinder is linked to. This may be used to evaluate the time needed to turn, accelerate, etc.
		const Vehicle& GetVehicle() const;

		// Returns a reference to the vehicle that this PathFinder is linked to. This may be used to evaluate the time needed to turn, accelerate, etc.
		const Vehicle& GetVehicle();
	};

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
		virtual void HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE*2>& result) override;

		// Builds a path from the designated start cell to the nearest unknown cell, and stores the result in the provided Path object.
		virtual void PathToNearestUnknown(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result)override;
		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result) override;

		// Builds a path from the designated start cell to the goal of the maze, and stores the result in the provided Path object.
		virtual void PathToGoal(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result)override;
		// Builds a path from the designated start cell to the designated end cell, and stores the result in the provided Path object.
		virtual void HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE*2>& result) override;

	protected:
		// Builds the distance grid starting from the cells flagged in the queue.
		// To use, add the desired cells to the queue, but do not swap pages on it, and then call this function.
		virtual void SetupDistances();
	};

	template<typename T>
	void PathFinder::DescendGradient(CellCoordinates start, Direction startDirection, const T (&cost)[16][16], Path<PATH_SIZE>& result)
	{
		result.clear();
		// Create path by moving from start to end.
		T distance = cost[start.GetX()][start.GetY()];
		Direction dir = startDirection;
		CellCoordinates current = start;
		result.push_back(start);

		while (distance > 0)
		{
			Direction next = dir;
			uint16_t minDist = 255;
			CellCoordinates target;
			Direction preferredCardinal = next;
			if (IsDiagonal(dir))
			{
				CellCoordinates prev = result.last(1);
				Direction lastEffectiveDirection = prev.DirectionTo(current);

				preferredCardinal = static_cast<Direction>(preferredCardinal ^ lastEffectiveDirection);
			}

			Direction minDir = Direction::None;

			for (Direction d = Direction::Up; d <= Direction::Right; d <<= 1)
			{
				if (GetMaze()[current].GetWall(d) != WallState::NoWall)
				{
					continue;
				}
				target = current >> d;
				if (cost[target.GetX()][target.GetY()] < minDist)
				{
					minDist = cost[target.GetX()][target.GetY()];
					minDir = d;
				}
				else if (cost[target.GetX()][target.GetY()] == minDist)
				{
					minDir = minDir | d;
				}
			}

			if ((minDir & preferredCardinal) == preferredCardinal )
			{
				target = current >> preferredCardinal;
				if (cost[target.GetX()][target.GetY()] == minDist)
				{
					result.push_back(target);
					dir = next;
					distance = minDist;
					current = target;
					continue;
				}
			}
			if ((minDir & (preferredCardinal+RelativeDirection::R90)) == (preferredCardinal + RelativeDirection::R90))
			{
				target = current >> (preferredCardinal + RelativeDirection::R90);
				if (cost[target.GetX()][target.GetY()] == minDist)
				{
					//dir = result.last(1).DirectionTo(result.last()) + RelativeDirection::R45;
					dir = result.last().DirectionTo(target) + RelativeDirection::R45;
					result.push_back(target);
					distance = minDist;
					current = target;
					continue;
				}
			}
			if ((minDir & (preferredCardinal + RelativeDirection::L90)) == (preferredCardinal + RelativeDirection::L90))
			{
				target = current >> (preferredCardinal + RelativeDirection::L90);
				if (cost[target.GetX()][target.GetY()] == minDist)
				{
					//dir = result.last(1).DirectionTo(result.last()) + RelativeDirection::L45;
					dir = result.last().DirectionTo(target) + RelativeDirection::L45;
					result.push_back(target);
					distance = minDist;
					current = target;
					continue;
				}
			}
			if ((minDir & (preferredCardinal + RelativeDirection::Reverse)) == (preferredCardinal + RelativeDirection::Reverse))
			{
				target = current >> (preferredCardinal + RelativeDirection::Reverse);
				if (cost[target.GetX()][target.GetY()] == minDist)
				{
					if (result.last(1).DirectionTo(current) == (preferredCardinal + RelativeDirection::R90))
					{
						dir = dir + RelativeDirection::R90;
					}
					else if (result.last(1).DirectionTo(current) == (preferredCardinal + RelativeDirection::L90))
					{
						dir = dir + RelativeDirection::L90;
					}
					else
					{
						dir = dir + RelativeDirection::Reverse;
					}
					result.push_back(target);
					distance = minDist;
					current = target;
				}
			}
		}
	}
}

