#include "pch.h"
#include "Maze.h"

namespace MazeMap
{
	Maze::Maze()
		: _cellDimension(18.0)
	{
		for (int i = 0; i < 16; ++i)
		{
			for (int j = 0; j < 16; j++)
			{
				_cells[i][j] = Cell(i, j);
			}
		}
	}

	Maze::Maze(float cellDimension)
		: _cellDimension(cellDimension)
	{
		for (int i = 0; i < 16; ++i)
		{
			for (int j = 0; j < 16; j++)
			{
				_cells[i][j] = Cell(i, j);
			}
		}
	}

	int MazeMap::Maze::GetXSize() const { return sizeof(_cells)/sizeof(Cell)/GetYSize(); }
	int MazeMap::Maze::GetYSize() const { return sizeof(_cells[0])/sizeof(Cell); }
	Cell& Maze::operator()(int x, int y)
	{
		return _cells[x][y];
	}
	const Cell& Maze::operator()(int x, int y) const
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

	Cell& Maze::Index(int x, int y)
	{
		return _cells[x][y];
	}
	const Cell& Maze::Index(int x, int y) const
	{
		return _cells[x][y];
	}
	float Maze::GetCellDimension() { return const_cast<Maze const*>(this)->GetCellDimension(); }
	int MazeMap::Maze::GetXSize() { return const_cast<Maze const*>(this)->GetXSize(); }
	int MazeMap::Maze::GetYSize() { return const_cast<Maze const*>(this)->GetYSize(); }
}
