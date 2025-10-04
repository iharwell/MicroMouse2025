#include "pch.h"
#include "Cell.h"

namespace MazeMap
{
	Cell::Cell()
		: _coords(0, 0)
		, _data(0)
	{
	}

	Cell::Cell(uint8_t x, uint8_t y)
		: _coords(x, y)
		, _data(0)
	{
	}

	Cell::Cell(uint8_t x, uint8_t y, WallState up, WallState down, WallState left, WallState right)
		: _coords(x, y)
		, _data((up)|(down<<2)|(left<<4)|(right<<6))
	{
	}

	WallState Cell::GetUp() const { return static_cast<WallState>(_data & 0x03 ); }
	WallState Cell::GetDown() const { return static_cast<WallState>((_data>>2) & 0x03); }
	WallState Cell::GetLeft() const { return static_cast<WallState>((_data >> 4) & 0x03); }
	WallState Cell::GetRight() const { return static_cast<WallState>((_data >> 6) & 0x03); }

	CellCoordinates Cell::GetCoords() const { return  _coords; }
	CellCoordinates Cell::GetCoords() { return  const_cast<Cell const*>(this)->GetCoords(); }

	uint8_t Cell::GetX() const { return  _coords.GetX(); }
	uint8_t Cell::GetY() const { return _coords.GetY(); }

	WallState Cell::GetUp() { return static_cast<WallState>(_data & 0x03); }
	WallState Cell::GetDown() { return static_cast<WallState>((_data >> 2) & 0x03); }
	WallState Cell::GetLeft() { return static_cast<WallState>((_data >> 4) & 0x03); }
	WallState Cell::GetRight() { return static_cast<WallState>((_data >> 6) & 0x03); }

	uint8_t Cell::GetX() { return const_cast<Cell const*>(this)->GetX(); }
	uint8_t Cell::GetY() { return const_cast<Cell const*>(this)->GetY(); }


	void Cell::SetUp(WallState up)
	{
		_data = (_data&0xFC) | (static_cast<char>(up) & 0x03);
	}
	void Cell::SetDown(WallState down)
	{
		_data = (_data & 0xF3) | ((static_cast<char>(down) & 0x03) << 2);
	}
	void Cell::SetLeft(WallState left)
	{
		_data = (_data & 0xCF) | (static_cast<char>(left) << 4);
	}
	void Cell::SetRight(WallState right)
	{
		_data = (_data & 0x3F) | (static_cast<char>(right) << 6);
	}
	bool Cell::IsVisited() { return const_cast<const Cell*>(this)->IsVisited(); }
	bool Cell::IsVisited() const { return (_data & 0x55) == 0x55; }

	CharBlock Cell::Serialize() const
	{
		char table[] {'?', 'O', '?', 'W'};
		CharBlock cb;
		cb.data = 0;
		cb.chars[0] = table[static_cast<int>(GetUp())];
		cb.chars[1] = table[static_cast<int>(GetDown())];
		cb.chars[2] = table[static_cast<int>(GetLeft())];
		cb.chars[3] = table[static_cast<int>(GetRight())];
		return cb;
	}

	CharBlock Cell::Serialize() { return const_cast<Cell const*>(this)->Serialize(); }

	CellCoordinates::CellCoordinates()
		: _x(0)
		, _y(0)
	{}
	CellCoordinates::CellCoordinates(uint8_t x, uint8_t y)
		: _x(x)
		, _y(y)
	{}
	uint8_t CellCoordinates::GetX() const { return _x; }
	uint8_t CellCoordinates::GetY() const { return _y; }
	CellCoordinates CellCoordinates::Up() const
	{
		return CellCoordinates(GetX(), GetY() + 1);
	}
	CellCoordinates CellCoordinates::Down() const
	{
		return CellCoordinates(GetX(), GetY() - 1);
	}
	CellCoordinates CellCoordinates::Left() const
	{
		return CellCoordinates(GetX()-1, GetY());
	}
	CellCoordinates CellCoordinates::Right() const
	{
		return CellCoordinates(GetX()+1, GetY());
	}
	bool CellCoordinates::operator==(const CellCoordinates& other) const
	{
		return (GetX() == other.GetX()) && (GetY() == other.GetY());
	}

	bool CellCoordinates::IsUp(CellCoordinates other) const { return (GetX() == other.GetX()) && (GetY() + 1 == other.GetY()); }
	bool CellCoordinates::IsDown(CellCoordinates other) const { return (GetX() == other.GetX()) && (GetY() - 1 == other.GetY()); }
	bool CellCoordinates::IsLeft(CellCoordinates other) const { return (GetX() - 1 == other.GetX()) && (GetY() == other.GetY()); }
	bool CellCoordinates::IsRight(CellCoordinates other) const { return (GetX() + 1 == other.GetX()) && (GetY() == other.GetY()); }

	Direction CellCoordinates::DirectionTo(CellCoordinates other) const
	{
		Direction d = None;
		if (other.GetX() > GetX())
		{
			d = static_cast<Direction>(d | Direction::Right);
		}
		else if (other.GetX() < GetX())
		{
			d = static_cast<Direction>(d | Direction::Left);
		}
		if (other.GetY() > GetY())
		{
			d = static_cast<Direction>(d | Direction::Up);
		}
		else if (other.GetY() < GetY())
		{
			d = static_cast<Direction>(d | Direction::Down);
		}
		return d;
	}

	uint8_t CellCoordinates::GetX() { return const_cast<CellCoordinates const*>(this)->GetX(); }
	uint8_t CellCoordinates::GetY() { return const_cast<CellCoordinates const*>(this)->GetY(); }
	CellCoordinates CellCoordinates::Up() { return const_cast<CellCoordinates const*>(this)->Up(); }
	CellCoordinates CellCoordinates::Down() { return const_cast<CellCoordinates const*>(this)->Down(); }
	CellCoordinates CellCoordinates::Left() { return const_cast<CellCoordinates const*>(this)->Left(); }
	CellCoordinates CellCoordinates::Right() { return const_cast<CellCoordinates const*>(this)->Right(); }
	bool CellCoordinates::IsUp(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsUp(other); }
	bool CellCoordinates::IsDown(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsDown(other); }
	bool CellCoordinates::IsLeft(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsLeft(other); }
	bool CellCoordinates::IsRight(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->IsRight(other); }
	Direction CellCoordinates::DirectionTo(CellCoordinates other) { return const_cast<CellCoordinates const*>(this)->DirectionTo(other); }
	bool CellCoordinates::operator==(const CellCoordinates& other){ return other==(*this);}
}