#include "pch.h"
#include "MazeLocation.h"

namespace MazeMap
{

	MazeLocation::MazeLocation()
		: _halfX(1)
		, _halfY(1)
	{
	}
	MazeLocation::MazeLocation(uint8_t halfX, uint8_t halfY)
		: _halfX(halfX)
		, _halfY(halfY)
	{
	}


	// Returns the x component of the coordinate.
	uint8_t MazeLocation::GetX() const { return _halfX; }
	// Returns the x component of the coordinate.
	uint8_t MazeLocation::GetX() { return const_cast<const MazeLocation*>(this)->GetX(); }

	// Returns the y component of the coordinate.
	uint8_t MazeLocation::GetY() const { return _halfY; }
	// Returns the y component of the coordinate.
	uint8_t MazeLocation::GetY() { return const_cast<const MazeLocation*>(this)->GetY(); }

	// Returns the coordinates after moving one half cell in the provided direction.
	MazeLocation MazeLocation::operator>>(Direction direction)
	{
		uint8_t x = GetX();
		uint8_t y = GetY();

		if (direction & Direction::Up)
		{
			++y;
		}
		if (direction & Direction::Down)
		{
			--y;
		}
		if (direction & Direction::Left)
		{
			--x;
		}
		if (direction & Direction::Right)
		{
			++x;
		}
		return MazeLocation(x, y);
	}

	// Returns the coordinates after moving one half cell up.
	MazeLocation MazeLocation::Up() const { return MazeLocation(GetX(), GetY() + 1); }
	// Returns the coordinates after moving one half cell down.
	MazeLocation MazeLocation::Down() const { return MazeLocation(GetX(), GetY() - 1); }
	// Returns the coordinates after moving one half cell left.
	MazeLocation MazeLocation::Left() const { return MazeLocation(GetX() - 1, GetY()); }
	// Returns the coordinates after moving one half cell right.
	MazeLocation MazeLocation::Right() const { return MazeLocation(GetX() + 1, GetY()); }

	// Returns the coordinates after moving one half cell up.
	MazeLocation MazeLocation::Up() { return const_cast<const MazeLocation*>(this)->Up(); }
	// Returns the coordinates after moving one half cell down.
	MazeLocation MazeLocation::Down() { return const_cast<const MazeLocation*>(this)->Down(); }
	// Returns the coordinates after moving one half cell left.
	MazeLocation MazeLocation::Left() { return const_cast<const MazeLocation*>(this)->Left(); }
	// Returns the coordinates after moving one half cell right.
	MazeLocation MazeLocation::Right() { return const_cast<const MazeLocation*>(this)->Right(); }

	// Returns true if both the x- and y-components are equal between the two coordinates.
	bool MazeLocation::operator==(const MazeLocation& other) { return other == (*this); }
	// Returns true if both the x- and y-components are equal between the two coordinates.
	bool MazeLocation::operator==(const MazeLocation& other) const
	{
		return (GetX() == other.GetX()) && (GetY() == other.GetY());
	}

	// Returns true if either the x- or y-components are different between the two coordinates.
	bool MazeLocation::operator!=(const MazeLocation& other) { return other != (*this); }
	// Returns true if either the x- or y-components are different between the two coordinates.
	bool MazeLocation::operator!=(const MazeLocation& other) const
	{
		return (GetX() != other.GetX()) || (GetY() != other.GetY());
	}

	// Returns true if the provided coordinates are one cell up from these.
	bool MazeLocation::IsUp(MazeLocation other) const { return (GetX() == other.GetX()) && (GetY() + 1 == other.GetY()); }
	// Returns true if the provided coordinates are one cell down from these.
	bool MazeLocation::IsDown(MazeLocation other) const { return (GetX() == other.GetX()) && (GetY() - 1 == other.GetY()); }
	// Returns true if the provided coordinates are one cell left from these.
	bool MazeLocation::IsLeft(MazeLocation other) const { return (GetX() - 1 == other.GetX()) && (GetY() == other.GetY()); }
	// Returns true if the provided coordinates are one cell right from these.
	bool MazeLocation::IsRight(MazeLocation other) const { return (GetX() + 1 == other.GetX()) && (GetY() == other.GetY()); }

	// Returns true if the provided coordinates are one cell up from these.
	bool MazeLocation::IsUp(MazeLocation other) { return const_cast<const MazeLocation*>(this)->IsUp(other); }
	// Returns true if the provided coordinates are one cell down from these.
	bool MazeLocation::IsDown(MazeLocation other) { return const_cast<const MazeLocation*>(this)->IsDown(other); }
	// Returns true if the provided coordinates are one cell left from these.
	bool MazeLocation::IsLeft(MazeLocation other) { return const_cast<const MazeLocation*>(this)->IsLeft(other); }
	// Returns true if the provided coordinates are one cell right from these.
	bool MazeLocation::IsRight(MazeLocation other) { return const_cast<const MazeLocation*>(this)->IsRight(other); }

	// Returns the direction of the designated cell, with diagonal directions allowed.
	Direction MazeLocation::DirectionTo(MazeLocation other) const
	{
		Direction d = Direction::None;
		if (other.GetX() > GetX())
		{
			d |= Direction::Right;
		}
		else if (other.GetX() < GetX())
		{
			d |= Direction::Left;
		}
		if (other.GetY() > GetY())
		{
			d |= Direction::Up;
		}
		else if (other.GetY() < GetY())
		{
			d |= Direction::Down;
		}
		return d;
	}
	// Returns the direction of the designated cell, with diagonal directions allowed.
	Direction MazeLocation::DirectionTo(MazeLocation other) { return const_cast<const MazeLocation*>(this)->DirectionTo(other); }

	// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
	bool MazeLocation::IsValidMove(Direction direction) { return const_cast<const MazeLocation*>(this)->IsValidMove(direction); }
	// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
	bool MazeLocation::IsValidMove(Direction direction) const
	{
		// Bit logic to identify conflicting direction bits.
		bool conflicts = ((direction & (direction >> 1)) | Direction::Left) != Direction::Left;
		if (conflicts)
		{
			return false;
		}

		if ((direction & Direction::Up) && (GetY() == 32))
		{
			return false;
		}
		if ((direction & Direction::Down) && (GetY() == 0))
		{
			return false;
		}
		if ((direction & Direction::Left) && (GetX() == 0))
		{
			return false;
		}
		if ((direction & Direction::Right) && (GetX() == 32))
		{
			return false;
		}
		return true;
	}

	CellCoordinates MazeLocation::GetFirstConnectedCell()
	{
		uint8_t x1;
		uint8_t x2;
		uint8_t y1;
		uint8_t y2;
		if ((GetX() & 1) == 1)
		{
			x1 = x2 = (GetX() >> 1);
		}
		else
		{
			x1 = (GetX() >> 1) - 1;
			x2 = x1 + 1;
		}
		if ((GetY() & 1) == 1)
		{
			y1 = y2 = (GetY() >> 1);
		}
		else
		{
			y1 = (GetY() >> 1) - 1;
			y2 = y1 + 1;
		}

		return CellCoordinates(x1, y1);
	}
	CellCoordinates MazeLocation::GetSecondConnectedCell()
	{
		uint8_t x1;
		uint8_t x2;
		uint8_t y1;
		uint8_t y2;
		if ((GetX() & 1) == 1)
		{
			x1 = x2 = (GetX() >> 1);
		}
		else
		{
			x1 = (GetX() >> 1) - 1;
			x2 = x1 + 1;
		}
		if ((GetY() & 1) == 1)
		{
			y1 = y2 = (GetY() >> 1);
		}
		else
		{
			y1 = (GetY() >> 1) - 1;
			y2 = y1 + 1;
		}

		return CellCoordinates(x2, y2);
	}

	Direction MazeLocation::DirectionFromCellCenter() { return const_cast<const MazeLocation*>(this)->DirectionFromCellCenter(); }
	Direction MazeLocation::DirectionFromCellCenter() const
	{
		Direction d = Direction::None;
		if ((GetX() & 1) == 0)
		{
			d |= Direction::Left << ((GetX()>>5) & 1);
		}
		if ((GetY() & 1) == 0)
		{
			d |= Direction::Up << (((~GetY()) >> 5) & 1);
		}
		return d;
	}
}