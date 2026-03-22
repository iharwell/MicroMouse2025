#pragma once
#include "Defines.h"
#include "PathFinder.h"
#include "DirectionalLocation.h"
#include "HalfStepPath.h"
#include "Maneuver.h"

namespace MazeMap
{
	class ManeuverPath
	{
	private:
		ManeuverCode _steps[PATH_SIZE * 2];
		uint16_t _size;
	public:

		MAZEMAP_INLINE ManeuverPath()
			: _steps()
			, _size(0)
		{
		}

		static EXPORT bool FromHalfStep(const HalfStepPath<PATH_SIZE * 2>& hsp, DirectionalLocation start, ManeuverPath& mp);
		EXPORT float Cost(const Vehicle& v, float cellSize);
		EXPORT float Cost(const Vehicle& v, float cellSize) const;

		MAZEMAP_INLINE uint16_t GetSize() { return const_cast<const ManeuverPath*>(this)->GetSize(); }
		MAZEMAP_INLINE uint16_t GetSize() const { return _size; }

		MAZEMAP_INLINE ManeuverCode& Index(uint16_t index) { return _steps[index]; }
		MAZEMAP_INLINE const ManeuverCode& Index(uint16_t index) const { return _steps[index]; }

		MAZEMAP_INLINE ManeuverCode& operator[](uint16_t index) { return _steps[index]; }
		MAZEMAP_INLINE const ManeuverCode& operator[](uint16_t index) const { return _steps[index]; }

		MAZEMAP_INLINE bool push_back(ManeuverCode nextStep)
		{
			if (_size < PATH_SIZE * 2)
			{
				_steps[_size] = nextStep;
				++_size;
				return true;
			}
			return false;
		}

		MAZEMAP_INLINE bool pop_back()
		{
			if (_size > 0)
			{
				--_size;
				return true;
			}
			return false;
		}
		MAZEMAP_INLINE bool pop_back(ManeuverCode& valueRemoved)
		{
			if (_size > 0)
			{
				--_size;
				valueRemoved = _steps[_size];
				return true;
			}
			return false;
		}
		MAZEMAP_INLINE ManeuverCode first()
		{
			if (_size > 0)
			{
				return _steps[0];
			}
			return ManeuverCode::MC_NONE;
		}
		MAZEMAP_INLINE ManeuverCode last() const
		{
			if (_size > 0)
			{
				return _steps[_size - 1];
			}
			return ManeuverCode::MC_NONE;
		}
		MAZEMAP_INLINE ManeuverCode last() { return const_cast<const ManeuverPath*>(this)->last(); }
		MAZEMAP_INLINE ManeuverCode last(uint16_t index) const
		{
			if (_size > index)
			{
				return _steps[_size - 1 - index];
			}
			return ManeuverCode::MC_NONE;
		}
		MAZEMAP_INLINE ManeuverCode last(uint16_t index) { return const_cast<const ManeuverPath*>(this)->last(index); }
		MAZEMAP_INLINE void clear()
		{
			_size = 0;
		}

		MAZEMAP_INLINE bool contains(ManeuverCode coords) { return const_cast<const ManeuverPath*>(this)->contains(coords); }
		MAZEMAP_INLINE bool contains(ManeuverCode coords) const { return indexOf(coords) != -1; }

		MAZEMAP_INLINE int indexOf(ManeuverCode coords) { return const_cast<const ManeuverPath*>(this)->indexOf(coords); }
		MAZEMAP_INLINE int indexOf(ManeuverCode coords) const
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

		EXPORT DirectionalLocation ExecutePath(DirectionalLocation start) const;
		EXPORT DirectionalLocation ExecuteReverse(DirectionalLocation end) const;

		EXPORT void ToHalfStepPath(DirectionalLocation start, HalfStepPath<PATH_SIZE * 2>& path);

	};
}








