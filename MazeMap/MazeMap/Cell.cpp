#include "pch.h"
#include "Cell.h"

namespace MazeMap
{
	Cell::Cell()
		: _x(0)
		, _y(0)
		, _up(Unknown)
		, _down(Unknown)
		, _left(Unknown)
		, _right(Unknown)
	{
	}

	Cell::Cell(int x, int y)
		: _x(x)
		, _y(y)
		, _up(Unknown)
		, _down(Unknown)
		, _left(Unknown)
		, _right(Unknown)
	{
	}

	Cell::Cell(int x, int y, WallState up, WallState down, WallState left, WallState right)
		: _x(x)
		, _y(y)
		, _up(up)
		, _down(down)
		, _left(left)
		, _right(right)
	{
	}

	WallState Cell::GetUp() const { return _up; }
	WallState Cell::GetUp() { return const_cast<Cell const*>(this)->GetUp(); }

	WallState Cell::GetDown() const { return _down; }
	WallState Cell::GetDown() { return const_cast<Cell const*>(this)->GetDown(); }

	WallState Cell::GetLeft() const { return _left; }
	WallState Cell::GetLeft() { return const_cast<Cell const*>(this)->GetLeft(); }

	WallState Cell::GetRight() const { return _right; }
	WallState Cell::GetRight() { return const_cast<Cell const*>(this)->GetRight(); }

	int Cell::GetX() const { return _x; }
	int Cell::GetX() { return const_cast<Cell const*>(this)->GetX(); }
	int Cell::GetY() const { return _y; }
	int Cell::GetY() { return const_cast<Cell const*>(this)->GetY(); }


	void Cell::SetUp(WallState up)
	{
		_up = up;
	}
	void Cell::SetDown(WallState down)
	{
		_down = down;
	}
	void Cell::SetLeft(WallState left)
	{
		_left = left;
	}
	void Cell::SetRight(WallState right)
	{
		_right = right;
	}
	bool Cell::IsVisited()
	{
		return (_up)&&(_down)&&(_left)&&(_right);
	}
}