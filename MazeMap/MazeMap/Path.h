#ifndef PATH_H
#define PATH_H

#include "Cell.h"
namespace MazeMap
{
	template<int _SIZE>
	class EXPORT Path
	{
	private:
		uint16_t _size;
		CellCoordinates _steps[_SIZE];
	public:
		Path() {}

		uint16_t GetSize() { return const_cast<const Path*>(this)->GetSize(); }
		uint16_t GetSize() const { return _size; }

		CellCoordinates& Index(uint16_t index) { return _steps[index]; }
		const CellCoordinates& Index(uint16_t index) const { return _steps[index]; }

		CellCoordinates& operator[](uint16_t index) { return _steps[index]; }
		const CellCoordinates& operator[](uint16_t index) const { return _steps[index]; }

		bool push_back(CellCoordinates nextStep)
		{
			if (_size < 255)
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

