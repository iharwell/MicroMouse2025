#include "pch.h"
#include "Maze.h"

namespace MazeMap
{
	Maze::Maze()
		: _cellDimension(18.0)
		, _goalFound(false)
		, _goal(0, 0)
	{
		for (uint16_t i = 0; i < 16; ++i)
		{
			for (uint16_t j = 0; j < 16; j++)
			{
				_cells[i][j] = Cell(i, j);
			}
		}
	}

	Maze::Maze(float cellDimension)
		: _cellDimension(cellDimension)
		, _goalFound(false)
		, _goal(0,0)
	{
		for (uint16_t i = 0; i < 16; ++i)
		{
			for (uint16_t j = 0; j < 16; j++)
			{
				_cells[i][j] = Cell(i, j);
			}
		}
	}

	uint8_t MazeMap::Maze::GetXSize() const { return static_cast<uint8_t>(sizeof(_cells)/sizeof(Cell)/GetYSize()); }
	uint8_t MazeMap::Maze::GetYSize() const { return static_cast<uint8_t>(sizeof(_cells[0])/sizeof(Cell)); }
	Cell& Maze::operator()(uint8_t x, uint8_t y)
	{
		return _cells[x][y];
	}
	const Cell& Maze::operator()(uint8_t x, uint8_t y) const
	{
		return _cells[x][y];
	}
	void Maze::SetWall(Cell& cell, Direction direction, WallState state)
	{
		int x = cell.GetX();
		int y = cell.GetY();
		switch (direction)
		{
		case Up:
			Index(x, y).SetUp(state);
			if (y < 15)
			{
				Index(x, y + 1).SetDown(state);
			}
			break;
		case Down:
			Index(x, y).SetDown(state);
			if (y > 0)
			{
				Index(x, y - 1).SetUp(state);
			}
			break;
		case Left:
			Index(x, y).SetLeft(state);
			if (x > 0)
			{
				Index(x-1, y).SetRight(state);
			}
			break;
		case Right:
		default:
			Index(x, y).SetRight(state);
			if (x < 15)
			{
				Index(x+1, y).SetLeft(state);
			}
			break;
		}
	}
	float Maze::GetCellDimension() const
	{
		return _cellDimension;
	}

	bool Maze::HasFoundGoal()
	{
		if (!_goalFound)
		{
			for (int i = 0; i < 15; ++i)
			{
				for (int j = 0; j < 15; ++j)
				{
					Cell currentBL = Index(i, j);
					if (currentBL.GetUp() == NoWall && currentBL.GetRight() == NoWall)
					{
						Cell currentUR = Index(i+1, j+1);
						if (currentUR.GetDown() == NoWall && currentUR.GetLeft() == NoWall)
						{
							_goal = currentBL.GetCoords();
							_goalFound = true;
						}
					}
				}
			}
		}
		return _goalFound;
	}
	/*bool Maze::HasFoundGoal()
	{
		if (!_goalFound)
		{
			for (int i = 0; i < 15; ++i)
			{
				for (int j = 0; j < 15; ++j)
				{
					Cell currentBL = Index(i, j);
					if (currentBL.GetUp() == NoWall && currentBL.GetRight() == NoWall)
					{
						Cell currentUR = Index(i + 1, j + 1);
						if (currentUR.GetDown() == NoWall && currentUR.GetLeft() == NoWall)
						{
							_goal = currentBL.GetCoords();
							_goalFound = true;
						}
					}
				}
			}
		}
		return _goalFound;
	}*/

	bool CheckComplete(bool visited[], CellCoordinates current, const Maze& maze);

	bool Maze::IsComplete()
	{
		CellCoordinates coords = CellCoordinates(0, 0);
		bool Visited[16*16];

		return CheckComplete(Visited, coords, (*this));
	}
	bool IsVisited(bool visited[], CellCoordinates coords)
	{
		return visited[coords.GetX() + 16 * coords.GetY()];
	}
	bool SetVisited(bool visited[], CellCoordinates coords)
	{
		return visited[coords.GetX() + 16 * coords.GetY()] = true;
	}

	bool CheckComplete(bool visited[], CellCoordinates current, const Maze& maze)
	{
		const Cell& c = maze(current.GetX(), current.GetY());


		if (!c.IsVisited())
		{
			return false;
		}
		SetVisited(visited, current);
		if (c.GetUp() == NoWall && !IsVisited(visited, current.Up()))
		{
			if (!CheckComplete(visited, current.Up(), maze))
			{
				return false;
			}
		}
		if (c.GetDown() == NoWall && !IsVisited(visited, current.Down()))
		{
			if (!CheckComplete(visited, current.Down(), maze))
			{
				return false;
			}
		}
		if (c.GetLeft() == NoWall && !IsVisited(visited, current.Left()))
		{
			if (!CheckComplete(visited, current.Left(), maze))
			{
				return false;
			}
		}
		if (c.GetRight() == NoWall && !IsVisited(visited, current.Right()))
		{
			if (!CheckComplete(visited, current.Right(), maze))
			{
				return false;
			}
		}

		return true;
	}

	Cell& Maze::Index(int x, int y)
	{
		return _cells[x][y];
	}
	const Cell& Maze::Index(int x, int y) const
	{
		return _cells[x][y];
	}
	float Maze::GetCellDimension() { return const_cast<Maze const*>(this)->GetCellDimension(); }
	uint8_t MazeMap::Maze::GetXSize() { return const_cast<Maze const*>(this)->GetXSize(); }
	uint8_t MazeMap::Maze::GetYSize() { return const_cast<Maze const*>(this)->GetYSize(); }
}
