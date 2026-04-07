#pragma once

#include "Defines.h"
#include "DirectionalLocation.h"
#include "ManeuverSet.h"

namespace MazeMap
{
	class ManeuverPoint
	{
	public:
		ManeuverPoint(float x, float y, float theta, float omega, float velocity)
			: X(x), Y(y), Theta(theta), Omega(omega), Velocity(velocity)
		{
		}

		float X;
		float Y;
		float Theta;
		float Omega;
		float Velocity;
	};

	class ManeuverInstance
	{
	private:
		ManeuverCode _code;
		DirectionalLocation _start;
		float _entrySpeed;
		float _exitSpeed;
	public:
		MAZEMAP_INLINE ManeuverInstance()
			: _code(MC_NONE)
			, _start()
			, _entrySpeed(0.0f)
			, _exitSpeed(0.0f)
		{
		}

		MAZEMAP_INLINE ManeuverInstance(
			ManeuverCode code,
			DirectionalLocation start,
			float entrySpeed = 0.0f,
			float exitSpeed = 0.0f)
			: _code(code)
			, _start(start)
			, _entrySpeed(entrySpeed)
			, _exitSpeed(exitSpeed)
		{
		}

		MAZEMAP_INLINE ManeuverCode getCode() { return const_cast<const ManeuverInstance*>(this)->getCode(); }
		MAZEMAP_INLINE ManeuverCode getCode() const { return _code; }
		MAZEMAP_INLINE void setCode(ManeuverCode code) { _code = code; }

		MAZEMAP_INLINE DirectionalLocation getStart() { return const_cast<const ManeuverInstance*>(this)->getStart(); }
		MAZEMAP_INLINE DirectionalLocation getStart() const { return _start; }
		MAZEMAP_INLINE void setStart(DirectionalLocation start) { _start = start; }

		MAZEMAP_INLINE DirectionalLocation getEnd() { return const_cast<const ManeuverInstance*>(this)->getEnd(); }
		MAZEMAP_INLINE DirectionalLocation getEnd() const
		{
			return ManeuverSet::GetSet().Move(_code, _start);
		}

		inline float getEntrySpeed() { return const_cast<const ManeuverInstance*>(this)->getEntrySpeed(); }
		inline float getEntrySpeed() const { return _entrySpeed; }
		inline void setEntrySpeed(float entrySpeed) { _entrySpeed = entrySpeed; }

		inline float getExitSpeed() { return const_cast<const ManeuverInstance*>(this)->getExitSpeed(); }
		inline float getExitSpeed() const { return _exitSpeed; }
		inline void setExitSpeed(float exitSpeed) { _exitSpeed = exitSpeed; }

		inline bool IsStraight() const
		{
			return _code != MC_NONE && _code <= S31;
		}
	};
}

