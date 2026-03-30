#include "pch.h"
#include "DirectionalPathFinder.h"
#include "ManeuverPathFinder.h"
#include <limits>

namespace MazeMap
{
	HalfStepPath<PATH_SIZE * 4> _hsp = HalfStepPath<PATH_SIZE * 4>();
	static uint8_t DirectionToIndex(Direction d)
	{
		return static_cast<uint8_t>(d - Direction::Up);
	}

	DirectionalPathFinder::DirectionalPathFinder(const Maze& maze, const Vehicle& vehicle)
		: PathFinder(maze, vehicle)
		, _data()
		, _lastEstimatedTime(0.0f)
	{
		for (size_t i = 0; i < 32; i++)
		{
			for (size_t j = 0; j < 32; j++)
			{
				for (size_t k = 0; k < 8; k++)
				{
					_data[i][j][k] = (std::numeric_limits<float>::infinity)();
				}
			}
		}
	}
	float DirectionalPathFinder::GetLastEstimatedTime()
	{
		return _lastEstimatedTime;
	}
	void DirectionalPathFinder::HalfStepPathFromTo(
		CellCoordinates start,
		Direction startDirection,
		CellCoordinates end,
		HalfStepPath<PATH_SIZE * 2>& result)
	{
		Reset();
		for (uint8_t i = 0; i < 8; i++)
		{
			UpdateCost(MazeLocation::CellCenter(end), OrdinalDirections[i], 0);
		}
		EvaluateCosts();

		DescendGradient(start, startDirection, result);
	}
	void DirectionalPathFinder::HalfStepPathToNearestUnknown(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
		(void)start;
		(void)startDirection;
		(void)result;
	}
	void DirectionalPathFinder::HalfStepPathToGoal(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
		Reset();
		MazeLocation goalLLC = MazeLocation::CellCenter(GetMaze().GetGoalLowerLeft());
		for (uint8_t i = 0; i < 2; i++)
		{
			for (uint8_t j = 0; j < 2; j++)
			{
				for (uint8_t k = 0; k < 8; k++)
				{
					UpdateCost(MazeLocation(goalLLC.GetX() + 2 * i, goalLLC.GetY() + 2 * j), OrdinalDirections[k], 0);

				}

			}
		}
		EvaluateCosts();

		DescendGradient(start, startDirection, result);
	}
	void DirectionalPathFinder::EvaluateCosts()
	{
		_queue.SwapQueues();
		while (_queue.GetCurrent().AnyFlags())
		{
			for (uint8_t i = 0; i < 16; i++)
			{
				if (!_queue.GetCurrent().GetRow(i))
				{
					continue;
				}
				for (uint8_t j = 0; j < 16; j++)
				{
					if (_queue.GetCurrent(i, j))
					{
						UpdateCell(i, j);
					}
				}
			}
			_queue.SwapQueues();
		}
	}
	void DirectionalPathFinder::Reset()
	{
		for (size_t i = 0; i < 32; i++)
		{
			for (size_t j = 0; j < 32; j++)
			{
				for (size_t k = 0; k < 8; k++)
				{
					_data[i][j][k] = (std::numeric_limits<float>::infinity)();
				}
			}
		}
		_queue.Clear();
	}

	float DirectionalPathFinder::Cost(DirectionalLocation dirLoc)
	{
		return Cost(dirLoc.GetLocation(), dirLoc.GetDirection());
	}

	void DirectionalPathFinder::UpdateCost(DirectionalLocation dirLoc, float cost)
	{
		UpdateCost(dirLoc.GetLocation(), dirLoc.GetDirection(), cost);
	}

	float DirectionalPathFinder::Cost(MazeLocation loc, Direction d)
	{
		return _data[loc.GetX()][loc.GetY()][DirectionToIndex(d)];
	}

	void DirectionalPathFinder::UpdateCost(MazeLocation loc, Direction d, float cost)
	{
		float prev = _data[loc.GetX()][loc.GetY()][DirectionToIndex(d)];
		if (prev > cost)
		{
			_queue.Enqueue(static_cast<MazeMap::CellCoordinates>(loc));
			_data[loc.GetX()][loc.GetY()][DirectionToIndex(d)] = cost;
		}
	}

	void DirectionalPathFinder::DescendGradient(CellCoordinates start, Direction startDirection, HalfStepPath<PATH_SIZE * 2>& result)
	{
		_hsp.clear();
		DirectionalLocation dirLoc(MazeLocation::CellCenter(start), startDirection);
		result.clear();
		result.push_back(MazeLocation::CellCenter(start));
		bool first = true;
		while (Cost(dirLoc.GetLocation(), -dirLoc.GetDirection()) > 0.001)
		{
			float minCost = (std::numeric_limits<float>::infinity)();
			Direction minDir = dirLoc.GetDirection();
			for (uint8_t i = 0; i < 8; ++i)
			{
				Direction possibleDirection = OrdinalDirections[i];
				if (GetMaze().IsAccessibleLocation(dirLoc.GetLocation() >> possibleDirection))
				{
					float possibleCost = Cost(dirLoc.GetLocation() >> possibleDirection, -possibleDirection);
					if (possibleCost < minCost)
					{
						minCost = possibleCost;
						minDir = possibleDirection;
					}
				}
			}
			if (first)
			{
				_lastEstimatedTime = minCost;
				first = false;
			}

			if (GetMaze().IsAccessibleLocation(dirLoc.GetLocation() >> minDir))
			{
				dirLoc = DirectionalLocation(dirLoc.GetLocation() >> minDir, minDir);
				result.push_back(dirLoc.GetLocation());
				/*CellCoordinates first = dirLoc.GetLocation().GetFirstConnectedCell();
				CellCoordinates second = dirLoc.GetLocation().GetSecondConnectedCell();
				CellCoordinates lastCell = result.last();
				CellCoordinates lastCell2 = result.last(1);
				if (first != lastCell && first != lastCell2)
				{
					result.push_back(first);
				}
				if (first != second && second != lastCell && second != lastCell2)
				{
					result.push_back(second);
				}*/
			}
			else
			{
				return;
			}
		}


	}

	Direction DirectionalPathFinder::GetAscendDirection(MazeLocation loc, Direction d)
	{
		float minCost = (std::numeric_limits<float>::infinity)();
		Direction minDir = d;
		for (uint8_t i = 0; i < 8; ++i)
		{
			Direction possibleDirection = OrdinalDirections[i];
			if (GetMaze().IsAccessibleLocation(loc >> possibleDirection))
			{
				float possibleCost = Cost(loc >> possibleDirection, -possibleDirection);
				if (possibleCost < minCost)
				{
					minCost = possibleCost;
					minDir = possibleDirection;
				}
			}
		}
		return minDir;
	}
	Direction DirectionalPathFinder::GetDescendDirection(MazeLocation loc, Direction d)
	{
		float minCost = (std::numeric_limits<float>::infinity)();
		Direction minDir = d;
		for (uint8_t i = 0; i < 8; ++i)
		{
			Direction possibleDirection = OrdinalDirections[i];
			if (GetMaze().IsAccessibleLocation(loc >> possibleDirection))
			{
				float possibleCost = Cost(loc >> possibleDirection, -possibleDirection);
				if (possibleCost < minCost)
				{
					minCost = possibleCost;
					minDir = possibleDirection;
				}
			}
		}
		return minDir;
	}
	void DirectionalPathFinder::UpdateCell(uint8_t row, uint8_t col)
	{
		// Iterate through all contained half steps.
		for (uint8_t i = row<<1; i <= (row<<1)+2; i++)
		{
			for (uint8_t j = col << 1; j <= (col << 1) + 2; j++)
			{
				MazeLocation loc(i, j);
				if (!GetMaze().IsAccessibleLocation(loc))
				{
					continue;
				}
				RadiateLocation(loc);
			}
		}
	}
	void DirectionalPathFinder::RadiateLocation(MazeLocation loc)
	{
		float cellDim = GetMaze().GetCellDimension() / 100;
		float fastestTurnSpeed = GetVehicle().GetFastestTurnSpeed(cellDim);
		for (uint8_t i = 0; i < 8; i++)
		{
			Direction fromDir = OrdinalDirections[i];
			float fromVal = Cost(loc, fromDir);
			if (!isfinite(fromVal))
			{
				continue;
			}
			MazeLocation currentLoc = loc;
			float distance = 0.0f;

			while (GetMaze().IsAccessibleLocation(currentLoc >> fromDir))
			{
				currentLoc = currentLoc >> (fromDir);
				if (IsDiagonal(fromDir))
				{
					distance += GetMaze().GetCellDimension() / 100 * sqrtf(2.0f);
				}
				else
				{
					distance += GetMaze().GetCellDimension() / 100;
				}
				bool cost45Ready = false;
				bool cost90Ready = false;
				float cost45 = 0.0f;
				float cost90 = 0.0f;

				if (GetMaze().IsAccessibleLocation(currentLoc >> fromDir))
				{
					float direct = fromVal + GetVehicle().GetStraightLineCost(distance, fastestTurnSpeed, fastestTurnSpeed);
					if (Cost(currentLoc, fromDir) > direct)
					{
						UpdateCost(currentLoc, fromDir, direct);
					}
				}
				if (GetMaze().IsAccessibleLocation(currentLoc >> (fromDir + RelativeDirection::R45)))
				{
					if (!cost45Ready)
					{
						cost45 = fromVal
							+ GetVehicle().GetStraightLineCost(distance, fastestTurnSpeed, GetVehicle().GetTurnSpeed(RelativeDirection::R45, cellDim))
							+ GetVehicle().GetTurnCost(RelativeDirection::R45, cellDim);
						cost45Ready = true;
					}
					if (Cost(currentLoc, fromDir + RelativeDirection::R45) > cost45)
					{
						UpdateCost(currentLoc, fromDir + RelativeDirection::R45, cost45);
					}
				}
				if (GetMaze().IsAccessibleLocation(currentLoc >> (fromDir + RelativeDirection::L45)))
				{
					if (!cost45Ready)
					{
						cost45 = fromVal
							+ GetVehicle().GetStraightLineCost(distance, fastestTurnSpeed, GetVehicle().GetTurnSpeed(RelativeDirection::R45, cellDim))
							+ GetVehicle().GetTurnCost(RelativeDirection::R45, cellDim);
						cost45Ready = true;
					}
					if (Cost(currentLoc, fromDir + RelativeDirection::L45) > cost45)
					{
						UpdateCost(currentLoc, fromDir + RelativeDirection::L45, cost45);
					}
				}
				if (GetMaze().IsAccessibleLocation(currentLoc >> (fromDir + RelativeDirection::R90)))
				{
					if (!cost90Ready)
					{
						cost90 = fromVal
							+ GetVehicle().GetStraightLineCost(distance, fastestTurnSpeed, GetVehicle().GetTurnSpeed(RelativeDirection::R90, cellDim))
							+ GetVehicle().GetTurnCost(RelativeDirection::R90, cellDim);
						cost90Ready = true;
					}
					if (Cost(currentLoc, fromDir + RelativeDirection::R90) > cost90)
					{
						UpdateCost(currentLoc, fromDir + RelativeDirection::R90, cost90);
					}
				}
				if (GetMaze().IsAccessibleLocation(currentLoc >> (fromDir + RelativeDirection::L90)))
				{
					if (!cost90Ready)
					{
						cost90 = fromVal
							+ GetVehicle().GetStraightLineCost(distance, fastestTurnSpeed, GetVehicle().GetTurnSpeed(RelativeDirection::R90, cellDim))
							+ GetVehicle().GetTurnCost(RelativeDirection::R90, cellDim);
						cost90Ready = true;
					}
					if (Cost(currentLoc, fromDir + RelativeDirection::L90) > cost90)
					{
						UpdateCost(currentLoc, fromDir + RelativeDirection::L90, cost90);
					}
				}
			}
		}
	}

	/*
	uint8_t DirectionalPathFinder::GetPriorStraightLength(MazeLocation currentLoc, Direction currentDirection)
	{
		uint8_t result = 0;
		float currentCost = Cost(currentLoc, currentDirection);
		while (GetMaze().IsAccessibleLocation(currentLoc >> (-currentDirection))
			&& Cost(currentLoc >> (-currentDirection), currentDirection) < Cost(currentLoc, currentDirection))
		{
			++result;
			currentLoc = currentLoc >> (-currentDirection);
			currentCost = Cost(currentLoc, currentDirection);
		}
		return result;
	}
	Direction DirectionalPathFinder::GetPriorDirection(MazeLocation currentLoc, Direction currentDirection)
	{
		return Direction::None;
	}
	*/
}
