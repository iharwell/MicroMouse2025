#ifndef PATH_H
#define PATH_H

#include "Cell.h"
namespace MazeMap
{
	template<int _SIZE>
	class Path
	{
	private:
		uint16_t _size;
		CellCoordinates _steps[_SIZE];
	public:
		Path()
			: _size(0)
		{}

		uint16_t GetSize() { return const_cast<const Path*>(this)->GetSize(); }
		uint16_t GetSize() const { return _size; }

		CellCoordinates& Index(uint16_t index) { return _steps[index]; }
		const CellCoordinates& Index(uint16_t index) const { return _steps[index]; }

		CellCoordinates& operator[](uint16_t index) { return _steps[index]; }
		const CellCoordinates& operator[](uint16_t index) const { return _steps[index]; }

		bool push_back(CellCoordinates nextStep)
		{
			if (_size < _SIZE)
			{
				_steps[_size] = nextStep;
				++_size;
				return true;
			}
			return false;
		}

		bool pop_back()
		{
			if (_size > 0)
			{
				--_size;
				return true;
			}
			return false;
		}
		bool pop_back(CellCoordinates& valueRemoved)
		{
			if (_size > 0)
			{
				--_size;
				valueRemoved = _steps[_size];
				return true;
			}
			return false;
		}
		CellCoordinates first()
		{
			if (_size > 0)
			{
				return _steps[0];
			}
			return CellCoordinates(0, 0);
		}
		CellCoordinates last() const
		{
			if (_size > 0)
			{
				return _steps[_size-1];
			}
			return CellCoordinates(0, 0);
		}
		CellCoordinates last() { return const_cast<const Path<_SIZE>*>(this)->last(); }
		CellCoordinates last(uint16_t index) const
		{
			if (_size > index)
			{
				return _steps[_size - 1 - index];
			}
			return CellCoordinates(0, 0);
		}
		CellCoordinates last(uint16_t index) { return const_cast<const Path<_SIZE>*>(this)->last(index); }
		void clear()
		{
			_size = 0;
		}

		bool contains(CellCoordinates coords) { return const_cast<const Path*>(this)->contains(coords); }
		bool contains(CellCoordinates coords) const { return indexOf(coords) != -1; }

		int indexOf(CellCoordinates coords) { return const_cast<const Path*>(this)->indexOf(coords); }
		int indexOf(CellCoordinates coords) const
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


	};
}
#endif


