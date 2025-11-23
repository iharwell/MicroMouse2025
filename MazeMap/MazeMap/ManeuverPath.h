#pragma once
#include "Defines.h"
#include "PathFinder.h"
#include "DirectionalLocation.h"
#include "HalfStepPath.h"
#include "Maneuver.h"

namespace MazeMap
{
	class EXPORT ManeuverPath
	{
	private:
		ManeuverCode _steps[PATH_SIZE * 2];
		uint16_t _size;
	public:

		ManeuverPath()
			: _size(0)
			, _steps()
		{
		}

		static void FromHalfStep(const HalfStepPath<PATH_SIZE * 2>& hsp, const ManeuverPath mp);
		float Cost(const Vehicle& v, float cellSize);
		float Cost(const Vehicle& v, float cellSize) const;

		uint16_t GetSize() { return const_cast<const ManeuverPath*>(this)->GetSize(); }
		uint16_t GetSize() const { return _size; }

		ManeuverCode& Index(uint16_t index) { return _steps[index]; }
		const ManeuverCode& Index(uint16_t index) const { return _steps[index]; }

		ManeuverCode& operator[](uint16_t index) { return _steps[index]; }
		const ManeuverCode& operator[](uint16_t index) const { return _steps[index]; }

		bool push_back(ManeuverCode nextStep)
		{
			if (_size < PATH_SIZE * 2)
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
		bool pop_back(ManeuverCode& valueRemoved)
		{
			if (_size > 0)
			{
				--_size;
				valueRemoved = _steps[_size];
				return true;
			}
			return false;
		}
		ManeuverCode first()
		{
			if (_size > 0)
			{
				return _steps[0];
			}
			return ManeuverCode::MC_NONE;
		}
		ManeuverCode last() const
		{
			if (_size > 0)
			{
				return _steps[_size - 1];
			}
			return ManeuverCode::MC_NONE;
		}
		ManeuverCode last() { return const_cast<const ManeuverPath*>(this)->last(); }
		ManeuverCode last(uint16_t index) const
		{
			if (_size > index)
			{
				return _steps[_size - 1 - index];
			}
			return ManeuverCode::MC_NONE;
		}
		ManeuverCode last(uint16_t index) { return const_cast<const ManeuverPath*>(this)->last(index); }
		void clear()
		{
			_size = 0;
		}

		bool contains(ManeuverCode coords) { return const_cast<const ManeuverPath*>(this)->contains(coords); }
		bool contains(ManeuverCode coords) const { return indexOf(coords) != -1; }

		int indexOf(ManeuverCode coords) { return const_cast<const ManeuverPath*>(this)->indexOf(coords); }
		int indexOf(ManeuverCode coords) const
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

		DirectionalLocation ExecutePath(DirectionalLocation start) const;
		DirectionalLocation ExecuteReverse(DirectionalLocation end) const;

		void ToHalfStepPath(DirectionalLocation start, HalfStepPath<PATH_SIZE * 2>& path);

	};
}
