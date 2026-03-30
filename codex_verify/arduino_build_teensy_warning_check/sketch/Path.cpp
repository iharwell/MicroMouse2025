#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Path.cpp"
#include "pch.h"
#include "Path.h"

namespace MazeMap
{/*
	Path::Path()
	{
	}
	uint16_t Path::GetSize() { return const_cast<const Path*>(this)->GetSize(); }
	uint16_t Path::GetSize() const { return _size; }

	CellCoordinates& Path::Index(uint16_t index) { return _steps[index]; }

	const CellCoordinates& Path::Index(uint16_t index) const { return _steps[index]; }

	CellCoordinates& Path::operator[](uint16_t index) { return _steps[index]; }
	const CellCoordinates& Path::operator[](uint16_t index) const { return _steps[index]; }

	bool Path::push_back(CellCoordinates nextStep)
	{
		if (_size < 255)
		{
			_steps[_size] = nextStep;
			++_size;
			return true;
		}
		return false;
	}

	bool Path::pop_back()
	{
		if (_size > 0)
		{
			--_size;
			return true;
		}
		return false;
	}

	bool Path::pop_back(CellCoordinates& valueRemoved)
	{
		if (_size > 0)
		{
			--_size;
			valueRemoved = _steps[_size];
			return true;
		}
		return false;
	}

	bool Path::contains(CellCoordinates coords) const { return indexOf(coords) != -1; }
	int Path::indexOf(CellCoordinates coords) const
	{
		for (int i = 0; i < _size; i++)
		{
			if (_steps[i] == coords)
			{
				return i;
			}
		}
		return -1;
	}

	bool Path::HasLoop() const
	{
		for (int i = 0; i < _size; ++i)
		{
			// The previous step is adjacent to this one, but doesn't form a loop.
			Direction d = None;
			CellCoordinates current = _steps[i];
			// Don't check the previous or next step.
			if (i > 0)
			{
				d = current.DirectionTo(_steps[i - 1]);
			}
			if ((i + 1) < _size)
			{
				d = static_cast<Direction>(d | current.DirectionTo(_steps[i + 1]));
			}


			if (!(d | Direction::Up))
			{
				bool b = contains(current.Up());
			}
			if (!(d | Direction::Down))
			{
				bool b = contains(current.Down());
			}
			if (!(d | Direction::Left))
			{
				bool b = contains(current.Left());
			}
			if (!(d | Direction::Right))
			{
				bool b = contains(current.Right());
			}
		}
		return false;
	}

	bool Path::HasLoop() { return const_cast<const Path*>(this)->HasLoop(); }

	int Path::indexOf(CellCoordinates coords) { return const_cast<const Path*>(this)->indexOf(coords); }

	bool Path::contains(CellCoordinates coords) { return const_cast<const Path*>(this)->contains(coords); }*/

}