#include "pch.h"
#include "CellCoordinates.h"
namespace MazeMap
{


	CellCoordinates::CellCoordinates()
		: _x(0)
		, _y(0)
	{
	}
	CellCoordinates::CellCoordinates(uint8_t x, uint8_t y)
		: _x(x)
		, _y(y)
	{
	}


	uint8_t CellCoordinates::GetX() const { return _x; }
	uint8_t CellCoordinates::GetY() const { return _y; }

	CellCoordinates CellCoordinates::Up() const { return CellCoordinates(GetX(), GetY() + 1); }
	CellCoordinates CellCoordinates::Down() const { return CellCoordinates(GetX(), GetY() - 1); }
	CellCoordinates CellCoordinates::Left() const { return CellCoordinates(GetX() - 1, GetY()); }
	CellCoordinates CellCoordinates::Right() const { return CellCoordinates(GetX() + 1, GetY()); }

	bool CellCoordinates::operator==(const CellCoordinates& other) const
	{
		return (GetX() == other.GetX()) && (GetY() == other.GetY());
	}
	bool CellCoordinates::operator!=(const CellCoordinates& other) const
	{
		return (GetX() != other.GetX()) || (GetY() != other.GetY());
	}
	bool CellCoordinates::operator!=(const CellCoordinates& other)
	{
		return other != (*this);
	}
	bool CellCoordinates::IsValidMove(Direction direction) { return const_cast<const CellCoordinates*>(this)->IsValidMove(direction); }
	bool CellCoordinates::IsValidMove(Direction direction) const
	{
		// Bit logic to identify conflicting direction bits.
		bool conflicts = ((direction & (direction >> 1)) | Direction::Left) != Direction::Left;
		if (conflicts)
		{
			return false;
		}

		if ((direction & Direction::Up) && (GetY() == 15))
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
		if ((direction & Direction::Right) && (GetX() == 15))
		{
			return false;
		}
		return true;
	}

	bool CellCoordinates::IsUp(CellCoordinates other) const { return (GetX() == other.GetX()) && (GetY() + 1 == other.GetY()); }
	bool CellCoordinates::IsDown(CellCoordinates other) const { return (GetX() == other.GetX()) && (GetY() - 1 == other.GetY()); }
	bool CellCoordinates::IsLeft(CellCoordinates other) const { return (GetX() - 1 == other.GetX()) && (GetY() == other.GetY()); }
	bool CellCoordinates::IsRight(CellCoordinates other) const { return (GetX() + 1 == other.GetX()) && (GetY() == other.GetY()); }

	Direction CellCoordinates::DirectionTo(CellCoordinates other) const
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

	uint8_t CellCoordinates::GetX() { return const_cast<CellCoordinates const*>(this)->GetX(); }
	uint8_t CellCoordinates::GetY() { return const_cast<CellCoordinates const*>(this)->GetY(); }
	CellCoordinates CellCoordinates::operator>>(Direction direction) { return (*const_cast<CellCoordinates const*>(this)) >> direction; }
	CellCoordinates CellCoordinates::operator>>(Direction direction) const
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
		return CellCoordinates(x, y);
	}
	CellCoordinates CellCoordinates::Up() { return const_cast<CellCoordinates const*>(this)->Up(); }
	CellCoordinates CellCoordinates::Down() { return const_cast<CellCoordinates const*>(this)->Down(); }
	CellCoordinates CellCoordinates::Left() { return const_cast<CellCoordinates const*>(this)->Left(); }
	CellCoordinates CellCoordinates::Right() { return const_cast<CellCoordinates const*>(this)->Right(); }
	bool CellCoordinates::IsUp(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsUp(other); }
	bool CellCoordinates::IsDown(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsDown(other); }
	bool CellCoordinates::IsLeft(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsLeft(other); }
	bool CellCoordinates::IsRight(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsRight(other); }
	Direction CellCoordinates::DirectionTo(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->DirectionTo(other); }
	bool CellCoordinates::operator==(const CellCoordinates& other) { return other == (*this); }
	CellCoordinates CellCoordinates::GetNearest(float x, float y, float cellDim)
	{
		x = std::clamp(x, 0.0f, cellDim * 16.0f);
		y = std::clamp(y, 0.0f, cellDim * 16.0f);
		return CellCoordinates(static_cast<uint8_t>((x / cellDim) + 0.5f), static_cast<uint8_t>((y / cellDim) + 0.5f));
	}
}