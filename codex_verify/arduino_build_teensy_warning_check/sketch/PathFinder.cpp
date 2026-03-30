#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\PathFinder.cpp"
#include "pch.h"
#include "PathFinder.h"
#include "DirectionalPathFinder.h"

namespace MazeMap
{


	void ResetDistances(uint16_t(&distances)[16][16], uint16_t initialValue)
	{
		for (uint8_t i = 0; i < 16; i++)
		{
			for (uint8_t j = 0; j < 16; j++)
			{
				distances[i][j] = initialValue;
			}
		}
	}

	PathFinder::PathFinder(const Maze& maze, const Vehicle& vehicle)
		: _maze(maze)
		, _vehicle(vehicle)
	{
	}

	const Maze& PathFinder::GetMaze() const { return _maze; }
	const Maze& PathFinder::GetMaze() { return const_cast<const PathFinder*>(this)->GetMaze(); }

	const Vehicle& PathFinder::GetVehicle() const { return _vehicle; }
	const Vehicle& PathFinder::GetVehicle() { return const_cast<const PathFinder*>(this)->GetVehicle(); }


	void PathFinder::PathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, Path<PATH_SIZE>& result)
	{
		HalfStepPath<PATH_SIZE * 2>* hsp = new HalfStepPath<PATH_SIZE * 2>();
		HalfStepPathFromTo(start, startDirection, end, *hsp);
		hsp->ConvertToPath(result);
		delete hsp;
	}
	void PathFinder::PathToNearestUnknown(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result)
	{
		HalfStepPath<PATH_SIZE * 2>* hsp = new HalfStepPath<PATH_SIZE * 2>();
		HalfStepPathToNearestUnknown(start, startDirection, *hsp);
		hsp->ConvertToPath(result);
		delete hsp;
	}
	void PathFinder::PathToGoal(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result)
	{
		HalfStepPath<PATH_SIZE * 2>* hsp = new HalfStepPath<PATH_SIZE * 2>();
		HalfStepPathToGoal(start, startDirection, *hsp);
		hsp->ConvertToPath(result);
		delete hsp;
	}


	FloodFillPathFinder::FloodFillPathFinder(const Maze& maze, const Vehicle& vehicle)
		: PathFinder(maze, vehicle)
		, _queue()
		, _alpha(false)
		, _beta(false)
		, _distances()
	{ }

	void FloodFillPathFinder::PathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, Path<PATH_SIZE>& result)
	{
		_alpha.Clear(false);
		_beta.Clear(false);
		_queue.Clear();

		_queue.Enqueue(end);

		SetupDistances();

		DescendGradient(start, startDirection, _distances, result);
	}

	void FloodFillPathFinder::HalfStepPathFromTo(CellCoordinates start, Direction startDirection, CellCoordinates end, HalfStepPath<PATH_SIZE*2>& result)
	{
		Path<PATH_SIZE>* tmp = new Path<PATH_SIZE>();
		PathFromTo(start, startDirection, end, *tmp);
		HalfStepPath<PATH_SIZE * 2>::HalfStepPathFromPath<PATH_SIZE>(*tmp, result);
		delete tmp;
	}

	void FloodFillPathFinder::PathToNearestUnknown(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result)
	{
		_alpha.Clear(false);
		_beta.Clear(false);
		_queue.Clear();

		_queue.Enqueue(start);

		CellCoordinates end = start;

		ResetDistances(_distances, 256);

		_queue.SwapQueues();
		uint16_t distance = 0;
		bool moreWork;
		do {
			moreWork = false;
			// Assign distance to the end to each cell
			for (uint8_t i = 0; i < 16; i++)
			{
				if (_queue.GetCurrent().GetRow(i) == 0)
				{
					continue;
				}

				for (uint8_t j = 0; j < 16; j++)
				{
					if (_queue.GetCurrent(i, j))
					{
						if (_distances[i][j] > distance)
						{
							_distances[i][j] = distance;
							const Cell& c = GetMaze()(i, j);
							if (!c.IsFullyKnown())
							{
								end = c.GetCoords();
								moreWork = false;
								break;
							}
							for (Direction d = Direction::Up; d <= Direction::Right; d <<= 1)
							{
								if (!c.GetCoords().IsValidMove(d) || _alpha[c.GetCoords() >> d])
								{
									continue;
								}
								if (c.GetWall(d) == WallState::NoWall)
								{
									_alpha.SetFlag(c.GetCoords() >> d, true);
									_queue.Enqueue(c.GetCoords() >> d);
									moreWork = true;
								}
							}
						}
					}
				}
				if (end != start)
				{
					break;
				}
			}
			++distance;
			_queue.SwapQueues();
		} while (moreWork)
			;

		PathFromTo(start, startDirection, end, result);
	}
	void FloodFillPathFinder::HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
		Path<PATH_SIZE>* tmp = new Path<PATH_SIZE>();
		PathToNearestUnknown(start, startDirection, *tmp);
		HalfStepPath<PATH_SIZE * 2>::HalfStepPathFromPath<PATH_SIZE>(*tmp, result);
		delete tmp;
	}
	void FloodFillPathFinder::PathToGoal(CellCoordinates start, Direction startDirection, Path<PATH_SIZE>& result)
	{
		_alpha.Clear(false);
		_beta.Clear(false);
		_queue.Clear();

		CellCoordinates goalLL = GetMaze().GetGoalLowerLeft();

		_queue.Enqueue(goalLL);
		_queue.Enqueue(goalLL >> Direction::Up);
		_queue.Enqueue(goalLL >> Direction::Up >> Direction::Right);
		_queue.Enqueue(goalLL >> Direction::Right);

		SetupDistances();

		DescendGradient(start, startDirection, _distances, result);
	}
	void FloodFillPathFinder::HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
		Path<PATH_SIZE>* tmp = new Path<PATH_SIZE>();
		PathToGoal(start, startDirection, *tmp);
		HalfStepPath<PATH_SIZE * 2>::HalfStepPathFromPath<PATH_SIZE>(*tmp, result);
		delete tmp;
	}

	void FloodFillPathFinder::SetupDistances()
	{
		ResetDistances(_distances, 256);

		_queue.SwapQueues();
		uint16_t distance = 0;
		bool moreWork;
		do {
			moreWork = false;
			// Assign distance to the end to each cell
			for (uint8_t i = 0; i < 16; i++)
			{
				if (_queue.GetCurrent().GetRow(i) == 0)
				{
					continue;
				}

				for (uint8_t j = 0; j < 16; j++)
				{
					if (_queue.GetCurrent(i, j))
					{
						if (_distances[i][j] > distance)
						{
							_distances[i][j] = distance;
							const Cell& c = GetMaze()(i, j);
							for (Direction d = Direction::Up; d <= Direction::Right; d <<= 1)
							{
								if (!c.GetCoords().IsValidMove(d) || _alpha[c.GetCoords() >> d])
								{
									continue;
								}
								if (c.GetWall(d) == WallState::NoWall)
								{
									_alpha.SetFlag(c.GetCoords() >> d, true);
									_queue.Enqueue(c.GetCoords() >> d);
									moreWork = true;
								}
							}
						}
					}
				}
			}
			++distance;
			_queue.SwapQueues();
		} while (moreWork)
			;
	}
}