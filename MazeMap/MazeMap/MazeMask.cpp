#include "pch.h"
#include "MazeMask.h"

namespace MazeMap
{
	MazeMask::MazeMask()
		: _data()
	{}

	MazeMask::MazeMask(bool initialValue)
		: _data()
	{
		if (initialValue)
		{
			for (size_t i = 0; i < 4; ++i)
			{
				_data.Longs[i] = (0ull) - (1ull);
			}
		}
	}

	/*MazeMask::MazeMask(const MazeMask& maskToClone)
		: _data()
	{
		for (size_t i = 0; i < 4; ++i)
		{
			_data.Longs[i] = maskToClone._data.Longs[i];
		}
	}*/

	MazeMask::MazeMask(uint16_t initialValues[])
		: _data()
	{
		for (size_t i = 0; i < 16; ++i)
		{
			_data.Shorts[i] = initialValues[i];
		}
		/*if (initialValue)
		{
			for (size_t i = 0; i < 4; ++i)
			{
				_data.Longs[i] = (0ull) - (1ull);
			}
		}*/
	}

	bool MazeMask::SetFlag(uint8_t x, uint8_t y, bool value)
	{
		if (x >= 16)
		{
			return false;
		}
		if (y >= 16)
		{
			return false;
		}
		if (value)
		{
			_data.Shorts[x] |= (1 << y);
			return 1;
		}
		else
		{
			_data.Shorts[x] &= ~(1 << y);
			return 0;
		}
	}
	bool MazeMask::SetFlag(CellCoordinates coords, bool value)
	{
#ifdef _DEBUG
		if (coords.GetX() >= 16)
		{
			return false;
		}
		if (coords.GetY() >= 16)
		{
			return false;
		}
#endif
		if (value)
		{
			_data.Shorts[coords.GetX()] |= (1 << coords.GetY());
			return 1;
		}
		else
		{
			_data.Shorts[coords.GetX()] &= ~(1 << coords.GetY());
			return 0;
		}
	}
	bool MazeMask::GetFlag(uint8_t x, uint8_t y) { return const_cast<const MazeMask*>(this)->GetFlag(x, y); }
	bool MazeMask::GetFlag(uint8_t x, uint8_t y) const
	{
		return static_cast<bool>((_data.Shorts[x] >> y) & 1);
	}

	void MazeMask::SetRow(uint8_t row, uint16_t data) { _data.Shorts[row] = data; }
	uint16_t MazeMask::GetRow(uint8_t row) const { return _data.Shorts[row]; }
	uint16_t MazeMask::GetRow(uint8_t row) { return const_cast<const MazeMask*>(this)->GetRow(row); }

	bool MazeMask::operator()(uint8_t x, uint8_t y) { return const_cast<const MazeMask*>(this)->GetFlag(x, y); }
	bool MazeMask::operator()(uint8_t x, uint8_t y) const { return GetFlag(x, y); }

	bool MazeMask::operator[](CellCoordinates coords) { return const_cast<const MazeMask*>(this)->GetFlag(coords.GetX(), coords.GetY()); }
	bool MazeMask::operator[](CellCoordinates coords) const { return GetFlag(coords.GetX(), coords.GetY()); }

	MazeMask MazeMask::GetPerimeter(PerimeterType perimeterType) { return const_cast<const MazeMask*>(this)->GetPerimeter(perimeterType); }
	MazeMask MazeMask::GetPerimeter(PerimeterType perimeterType) const
	{
		MazeMask perimeter = MazeMask();
		bool incBoundary = (perimeterType & PerimeterType::IncludeMazeBoundary) == PerimeterType::IncludeMazeBoundary;

		for (uint8_t i = 0; i < 16; ++i)
		{
			uint16_t row = _data.Shorts[i];
			if ((perimeterType & PerimeterType::Outer) == PerimeterType::Outer)
			{
				row = ~row;
			}
			uint16_t rowDown = 0xFFFF;
			uint16_t rowUp = 0xFFFF;

			if (i == 0)
			{
				if (incBoundary)
				{
					rowDown = 0;
				}
			}
			else
			{
				rowDown = _data.Shorts[i - 1];
				if ((perimeterType & PerimeterType::Outer) == PerimeterType::Outer)
				{
					rowDown = ~rowDown;
				}
			}
			if (i == 15)
			{
				if (incBoundary)
				{
					rowUp = 0;
				}
			}
			else
			{
				rowUp = _data.Shorts[i + 1];
				if ((perimeterType & PerimeterType::Outer) == PerimeterType::Outer)
				{
					rowUp = ~rowUp;
				}
			}

			// 16 bits indicating if there's an unset cell above or below a given cell
			uint16_t vertBorder = (~rowDown) | (~rowUp);
			uint16_t horBorder = ((~row) >> 1) | ((~row) << 1);
			if (!incBoundary)
			{
				horBorder &= ~0x8001;
			}
			perimeter.SetRow(i, row & (vertBorder | horBorder));
		}
		return perimeter;
	}

	void MazeMask::Clear(bool value)
	{
		uint64_t val = 0ull - value;
		for (size_t i = 0; i < 4; ++i)
		{
			_data.Longs[i] = val;
		}
	}
	bool MazeMask::AnyFlags() const
	{
		return (_data.Longs[0] | _data.Longs[1] | _data.Longs[2] | _data.Longs[3]) > 0;
	}
	bool MazeMask::AnyFlags() { return const_cast<const MazeMask*>(this)->AnyFlags(); }
}