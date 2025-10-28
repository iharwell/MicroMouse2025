#pragma once
#include "Defines.h"
#include "MazeLocation.h"

namespace MazeMap
{
	template <int SIZE>
	class EXPORT HalfStepPath
	{
	private:
		uint16_t _size;
		MazeLocation _steps[SIZE];
	public:
		HalfStepPath()
			: _size(0)
		{
		}
		template<int _PSIZE>
		static void HalfStepPathFromPath(const Path<_PSIZE>& path, HalfStepPath<SIZE> hpath)
		{
			hpath.clear();
			if (path.GetSize() == 0)
			{
				return;
			}
			hpath.push_back(MazeLocation::CellCenter(path[0]));
			for (size_t i = 1; i < path.GetSize(); i++)
			{
				MazeLocation loc = hpath.last();
				MazeLocation between = MazeLocation::Between(path[i - 1], path[i]);
				int8_t dx = between.GetX() - loc.GetX();
				int8_t dy = between.GetY() - loc.GetY();
				if (dx == 2)
				{
					hpath.push_back(MazeLocation(loc.GetX() + 1, loc.GetY()));
				}
				if (dx == -2)
				{
					hpath.push_back(MazeLocation(loc.GetX() - 1, loc.GetY()));
				}
				if (dy == 2)
				{
					hpath.push_back(MazeLocation(loc.GetX(), loc.GetY() + 1));
				}
				if (dy == -2)
				{
					hpath.push_back(MazeLocation(loc.GetX(), loc.GetY() - 1));
				}
				hpath.push_back(between);
			}
			if (path.GetSize() > 1)
			{
				hpath.push_back(MazeLocation::CellCenter(path.last()));
			}
		}

		uint16_t GetSize() { return const_cast<const HalfStepPath*>(this)->GetSize(); }
		uint16_t GetSize() const { return _size; }

		MazeLocation& Index(uint16_t index) { return _steps[index]; }
		const MazeLocation& Index(uint16_t index) const { return _steps[index]; }

		MazeLocation& operator[](uint16_t index) { return _steps[index]; }
		const MazeLocation& operator[](uint16_t index) const { return _steps[index]; }

		bool push_back(MazeLocation nextStep)
		{
			if (_size < SIZE)
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
		bool pop_back(MazeLocation& valueRemoved)
		{
			if (_size > 0)
			{
				--_size;
				valueRemoved = _steps[_size];
				return true;
			}
			return false;
		}
		MazeLocation first()
		{
			if (_size > 0)
			{
				return _steps[0];
			}
			return MazeLocation(0, 0);
		}
		MazeLocation last()
		{
			if (_size > 0)
			{
				return _steps[_size - 1];
			}
			return MazeLocation(0, 0);
		}
		MazeLocation last(uint16_t index)
		{
			if (_size > index)
			{
				return _steps[_size - 1 - index];
			}
			return MazeLocation(0, 0);
		}
		void clear()
		{
			_size = 0;
		}

		bool contains(MazeLocation coords) { return const_cast<const HalfStepPath*>(this)->contains(coords); }
		bool contains(MazeLocation coords) const { return indexOf(coords) != -1; }

		int indexOf(MazeLocation coords) { return const_cast<const HalfStepPath*>(this)->indexOf(coords); }
		int indexOf(MazeLocation coords) const
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

		template<int _PSIZE>
		void ConvertToPath(Path<_PSIZE>& path)
		{
			path.clear();
			if (_size == 0)
			{
				return;
			}
			if (_size == 1)
			{
				MazeLocation hstep = _steps[0];
				CellCoordinates f = hstep.GetFirstConnectedCell();
				CellCoordinates s = hstep.GetSecondConnectedCell();
				path.push_back(f);
				if (f != s)
				{
					path.push_back(s);
				}
				return;
			}

			MazeLocation hstep = _steps[0];
			MazeLocation nextStep = _steps[1];
			CellCoordinates intersect;
			if (hstep.GetFirstConnectedCell() != hstep.GetSecondConnectedCell())
			{
				if (hstep.IntersectsAt(nextStep, intersect))
				{
					if (hstep.GetFirstConnectedCell() == intersect)
					{
						path.push_back(hstep.GetSecondConnectedCell());
					}
					else
					{
						path.push_back(hstep.GetFirstConnectedCell());
					}
				}
			}
			else
			{
				path.push_back(hstep.GetFirstConnectedCell());
				if (nextStep.GetFirstConnectedCell() == path.last())
				{
					path.push_back(nextStep.GetSecondConnectedCell());
				}
				else
				{
					path.push_back(nextStep.GetFirstConnectedCell());
				}
			}
			for (size_t i = 1; i < _size; i++)
			{
				MazeLocation hstep = _steps[i];
				CellCoordinates f = hstep.GetFirstConnectedCell();
				CellCoordinates s = hstep.GetSecondConnectedCell();
				CellCoordinates l0 = path.last(0);
				CellCoordinates l1 = path.last(1);
				if (f != l0 && f!=l1)
				{
					path.push_back(f);
				}
				if (f != s && s != l0 && s != l1)
				{
					path.push_back(s);
				}
			}
		}
	};
}
