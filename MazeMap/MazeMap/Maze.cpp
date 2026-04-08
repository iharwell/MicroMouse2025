#include "pch.h"
#include "Maze.h"
#include "MissionMazeExport.h"
#include "MazeMask.h"
#include <cassert>
#include "MaskQueue.h"

namespace MazeMap
{
	Maze::Maze()
		: _goal(0, 0)
		, _reachable()
		, _complete(false)
		, _reachableCalculated(false)
		, _goalFound(false)
		, _accessible(false)
	{
		for (uint8_t i = 0; i < 16; ++i)
		{
			for (uint8_t j = 0; j < 16; j++)
			{
				_cells[i][j] = Cell(i, j);
				if (j == 15)
				{
					_cells[i][j].SetUp(WallState::Wall);
				}
				if (j == 0)
				{
					_cells[i][j].SetDown(WallState::Wall);
				}
				if (i == 0)
				{
					_cells[i][j].SetLeft(WallState::Wall);
				}
				if (i == 15)
				{
					_cells[i][j].SetRight(WallState::Wall);
				}
			}
		}
	}

	Maze::Maze(float cellDimension)
		: _goal(0, 0)
		, _reachable()
		, _complete(false)
		, _reachableCalculated(false)
		, _goalFound(false)
		, _accessible(false)
	{
		(void)cellDimension;
		for (uint8_t i = 0; i < 16; ++i)
		{
			for (uint8_t j = 0; j < 16; j++)
			{
				_cells[i][j] = Cell(i, j);
				if (j == 15)
				{
					_cells[i][j].SetUp(WallState::Wall);
				}
				if (j == 0)
				{
					_cells[i][j].SetDown(WallState::Wall);
				}
				if (i == 0)
				{
					_cells[i][j].SetLeft(WallState::Wall);
				}
				if (i == 15)
				{
					_cells[i][j].SetRight(WallState::Wall);
				}
			}
		}
		_cells[0][0].SetRight(WallState::Wall);
		_cells[1][0].SetLeft(WallState::Wall);
		_cells[0][0].SetUp(WallState::NoWall);
		_cells[0][1].SetDown(WallState::NoWall);
	}

	uint8_t MazeMap::Maze::GetXSize() const { return static_cast<uint8_t>(sizeof(_cells) / sizeof(Cell) / GetYSize()); }
	uint8_t MazeMap::Maze::GetYSize() const { return static_cast<uint8_t>(sizeof(_cells[0]) / sizeof(Cell)); }
	Cell& Maze::operator()(uint8_t x, uint8_t y)
	{
		return _cells[x][y];
	}
	const Cell& Maze::operator()(uint8_t x, uint8_t y) const
	{
		return _cells[x][y];
	}
	Cell& Maze::operator[](CellCoordinates coords)
	{
		return _cells[coords.GetX()][coords.GetY()];
	}
	const Cell& Maze::operator[](CellCoordinates coords) const
	{
		return _cells[coords.GetX()][coords.GetY()];
	}
	void Maze::SetWall(uint8_t x, uint8_t y, Direction direction, WallState state)
	{
		CellCoordinates coords = CellCoordinates(x, y);
		SetWall(coords, direction, state);
	}
	void Maze::SetWall(CellCoordinates coords, Direction direction, WallState state)
	{
		// Don't set the thing if we're not changing it.
		if (Index(coords).GetWall(direction) == state)
		{
			// If we're at the edge of the map, then we don't have to check other cells.
			if (!coords.IsValidMove(direction))
			{
				return;
			}
			// Check the cell in designated direction to see if it has been correctly set.
			// If it's correct, then we don't need to do anything.
			// If it's not correct, we have to set it.
			if (Index(coords >> direction).GetWall(-direction) == state)
			{
				return;
			}
		}

		Index(coords).SetWall(direction, state);
		if (coords.IsValidMove(direction))
		{
			Index(coords >> direction).SetWall(-direction, state);
		}
		_reachableCalculated = false;
		if (state == Unknown)
		{
			_complete = false;
			// Unset the goal if we're close enough to affect it.
			if (_goalFound)
			{
				int dx = static_cast<int32_t>(_goal.GetX()) - coords.GetX();
				int dy = static_cast<int32_t>(_goal.GetY()) - coords.GetY();
				if (dx >= -2 && dx <= 1 && dy >= -2 && dy <= 1)
				{
					_goalFound = false;
					_goal = CellCoordinates(0, 0);
				}
			}
		}
	}
	void Maze::SetWall(Cell& cell, Direction direction, WallState state)
	{
		SetWall(cell.GetCoords(), direction, state);
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
						Cell currentUR = Index(i + 1, j + 1);
						if (currentUR.GetDown() == NoWall && currentUR.GetLeft() == NoWall)
						{
							_goal = currentBL.GetCoords();
							_goalFound = true;
							break;
						}
					}
				}
				if (_goalFound)
				{
					break;
				}
			}
		}
		return _goalFound;
	}

	void Maze::FindReachables(MazeMask& reachable, CellCoordinates cell) const
	{
		reachable.Clear(false);

		reachable.SetFlag(cell, true);

		MazeMask queue = MazeMask();
		queue.SetFlag(cell, true);

		bool workRemaining;
		do
		{
			workRemaining = false;
			for (uint8_t i = 0; i < 16; ++i)
			{
				if (queue.GetRow(i) == 0)
				{
					continue;
				}
				for (uint8_t j = 0; j < 16; ++j)
				{
					bool cellFlag = queue.GetFlag(i, j);
					if (!cellFlag)
					{
						continue;
					}
					CellCoordinates current = CellCoordinates(i, j);
					queue.SetFlag(i, j, false);
					reachable.SetFlag(i, j, true);
					for (Direction d = Direction::Up; d <= Direction::Right; d <<= 1)
					{
						if (!current.IsValidMove(d))
						{
							continue;
						}
						if (Index(i, j).GetWall(d) == WallState::NoWall)
						{
							CellCoordinates target = current >> d;
							if (reachable[target])
							{
								continue;
							}
							reachable.SetFlag(target, true);
							queue.SetFlag(target, true);
							workRemaining = true;
						}
					}
				}
			}
		} while (workRemaining);
	}

	MazeMask Maze::GetReachableMask() const
	{
		if (_reachableCalculated)
		{
			return _reachable;
		}

		MazeMask reachable = MazeMask();
		CellCoordinates current = CellCoordinates(0, 0);
		FindReachables(reachable, current);
		return reachable;
	}

	MazeMask Maze::GetReachableMask()
	{
		return const_cast<const Maze*>(this)->GetReachableMask();
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

	bool CheckComplete(MazeMask visited, CellCoordinates current, const Maze& maze);

	CellCoordinates Maze::GetGoalLowerLeft() const
	{
		return _goal;
	}
	CellCoordinates Maze::GetGoalLowerLeft()
	{
		if (HasFoundGoal())
		{
			return _goal;
		}
		else
		{
			return CellCoordinates(0, 0);
		}
	}

	bool Maze::IsComplete()
	{
		if (_complete)
		{
			return true;
		}
		CellCoordinates coords = CellCoordinates(0, 0);
		MazeMask Visited = MazeMask();

		return CheckComplete(Visited, coords, (*this));
	}

	void Maze::PreCalculate()
	{
		if (IsComplete())
		{

			for (uint8_t i = 0; i < 32; i++)
			{
				for (uint8_t j = 0; j < 32; j++)
				{
					MazeLocation loc(i, j);
					_accessible.SetFlag(loc, IsAccessibleLocation(loc));

				}
			}
			_complete = true;
			_reachable = GetReachableMask();
			_reachableCalculated = true;
			HasFoundGoal();
		}
	}

	/*bool SetVisited(MazeMask visited, CellCoordinates coords)
	{
		return visited.SetFlag(coords.GetX(), coords.GetY(), true);
	}*/

	bool CheckComplete(MazeMask visited, CellCoordinates cell, const Maze& maze)
	{
		const Cell& c = maze(cell.GetX(), cell.GetY());


		if (!c.IsFullyKnown())
		{
			return false;
		}
		visited.Clear(false);

		visited.SetFlag(cell, true);

		MazeMask queue = MazeMask();
		queue.SetFlag(cell, true);

		bool workRemaining;
		do
		{
			workRemaining = false;
			for (uint8_t i = 0; i < 16; ++i)
			{
				if (queue.GetRow(i) == 0)
				{
					continue;
				}
				for (uint8_t j = 0; j < 16; ++j)
				{
					bool cellFlag = queue.GetFlag(i, j);
					if (!cellFlag)
					{
						continue;
					}
					CellCoordinates current = CellCoordinates(i, j);
					queue.SetFlag(i, j, false);
					visited.SetFlag(i, j, true);

					if (!maze.Index(current).IsFullyKnown())
					{
						return false;
					}

					for (Direction d = Direction::Up; d <= Direction::Right; d <<= 1)
					{
						if (!current.IsValidMove(d))
						{
							continue;
						}
						if (maze.Index(i, j).GetWall(d) == WallState::NoWall)
						{
							CellCoordinates target = current >> d;
							if (!visited[target])
							{
								queue.SetFlag(target, true);
								workRemaining = true;
							}
						}
					}
				}
			}
		} while (workRemaining);
		return true;
	}

	const float Maze::_cellDimension = 18.0f;

float Maze::GetCellDimension()
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
	Cell& Maze::Index(CellCoordinates coords)
	{
		return _cells[coords.GetX()][coords.GetY()];
	}
	const Cell& Maze::Index(CellCoordinates coords) const
	{
		return _cells[coords.GetX()][coords.GetY()];
	}
	//float Maze::GetCellDimension() { return const_cast<Maze const*>(this)->GetCellDimension(); }
	uint8_t Maze::GetXSize() { return const_cast<Maze const*>(this)->GetXSize(); }
	uint8_t Maze::GetYSize() { return const_cast<Maze const*>(this)->GetYSize(); }

	bool Maze::IsAccessibleLocation(MazeLocation location) { return const_cast<const Maze*>(this)->IsAccessibleLocation(location); }
	bool Maze::IsAccessibleLocation(MazeLocation location) const
	{
		if (_reachableCalculated)
		{
			if (location.GetX() >= 32 || location.GetY() >= 32 || location.GetX() == 0 || location.GetY() == 0)
			{
				return false;
			}
			return _accessible[location];
		}

		Direction d = location.DirectionFromCellCenter();

		/*if (location.GetX() == 0 || location.GetY() == 0)
		{
			return false;
		}
		if (location.GetX() >= 32 || location.GetY() >= 32)
		{
			return false;
		}*/
		if (d == Direction::None)
		{
			return true;
		}
		if (((location.GetX()|location.GetY())&1)==0)
		{
			return false;
		}
		CellCoordinates coords = static_cast<CellCoordinates>(location);
		Cell c = Index(coords);
		WallState s = c.GetWall(d);
		return s == WallState::NoWall;
	}
	bool Maze::IsIntersection(CellCoordinates location) const
	{
		int openings = (Index(location).GetUp() == WallState::NoWall);
		openings += (Index(location).GetDown() == WallState::NoWall);
		openings += (Index(location).GetLeft() == WallState::NoWall);
		openings += (Index(location).GetRight() == WallState::NoWall);
		return openings >= 2;
	}
	bool Maze::IsIntersection(CellCoordinates location) { return const_cast<const Maze*>(this)->IsIntersection(location); }
	bool Maze::IsIntersection(MazeLocation location) const { return IsIntersection(static_cast<CellCoordinates>(location)); }
	MazeMask Maze::DeadEndMask(CellCoordinates startLocation) const
	{
		MazeMask mask = MazeMask();

		bool cont = true;
		while (cont)
		{
			cont = false;
			for (uint8_t i = 0; i < 16; i++)
			{
				for (uint8_t j = 0; j < 16; j++)
				{
					if (i == startLocation.GetX() && j == startLocation.GetY())
					{
						continue;
					}

					if (mask(i, j))
					{
						continue;
					}
					Cell c = Index(i, j);


					int openings = 0;
					if (j < 15 && !mask(i, j + 1))
					{
						openings += (c.GetUp() == WallState::NoWall);
					}
					if (j > 0 && !mask(i, j - 1))
					{
						openings += (c.GetDown() == WallState::NoWall);
					}
					if (i > 0 && !mask(i - 1, j))
					{
						openings += (c.GetLeft() == WallState::NoWall);
					}
					if (i < 15 && !mask(i+1, j))
					{
						openings += (c.GetRight() == WallState::NoWall);
					}

					if (openings < 2)
					{
						mask.SetFlag(i, j, true);
						cont = true;
					}
				}
			}
		}


		return MazeMask();
	}
	bool Maze::IsIntersection(MazeLocation location) { return const_cast<const Maze*>(this)->IsIntersection(location); }
	bool Maze::IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction)
	{
		return const_cast<const Maze*>(this)->IsValidMove(location, instruction);
	}
	bool Maze::IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction) const
	{
		MazeLocation loc = location.GetLocation();
		Direction d = location.GetDirection() + instruction.GetDirection();
		for (uint8_t i = 0; i < instruction.GetDistance(); ++i)
		{
			if (!IsAccessibleLocation(loc))
			{
				return false;
			}
			loc = loc >> d;
		}
		return true;
	}
	void Maze::ExportToFile(const char* fileName)
	{
		ExportToFileC(fileName);
	}

	void Maze::ExportToFileC(const char* fileName) const
	{
		(void)ExportMazeSnapshot(*this, fileName);
	}

	void Maze::DumpMaze() const
	{
		ExportToFileC("mazeDump.txt");
	}
}


