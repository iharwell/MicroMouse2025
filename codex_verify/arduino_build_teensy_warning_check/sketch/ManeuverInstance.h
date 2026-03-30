#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\ManeuverInstance.h"
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

		MAZEMAP_INLINE ManeuverCode GetCode() { return const_cast<const ManeuverInstance*>(this)->GetCode(); }
		MAZEMAP_INLINE ManeuverCode GetCode() const { return _code; }
		MAZEMAP_INLINE ManeuverCode getCode() { return GetCode(); }
		MAZEMAP_INLINE ManeuverCode getCode() const { return GetCode(); }
		MAZEMAP_INLINE void SetCode(ManeuverCode code) { _code = code; }
		MAZEMAP_INLINE void setCode(ManeuverCode code) { SetCode(code); }

		MAZEMAP_INLINE DirectionalLocation GetStart() { return const_cast<const ManeuverInstance*>(this)->GetStart(); }
		MAZEMAP_INLINE DirectionalLocation GetStart() const { return _start; }
		MAZEMAP_INLINE DirectionalLocation getStart() { return GetStart(); }
		MAZEMAP_INLINE DirectionalLocation getStart() const { return GetStart(); }
		MAZEMAP_INLINE void SetStart(DirectionalLocation start) { _start = start; }
		MAZEMAP_INLINE void setStart(DirectionalLocation start) { SetStart(start); }

		MAZEMAP_INLINE DirectionalLocation GetEnd() { return const_cast<const ManeuverInstance*>(this)->GetEnd(); }
		MAZEMAP_INLINE DirectionalLocation GetEnd() const
		{
			return ManeuverSet::GetSet().Move(_code, _start);
		}
		MAZEMAP_INLINE DirectionalLocation getEnd() { return GetEnd(); }
		MAZEMAP_INLINE DirectionalLocation getEnd() const { return GetEnd(); }

		MAZEMAP_INLINE float GetEntrySpeed() { return const_cast<const ManeuverInstance*>(this)->GetEntrySpeed(); }
		MAZEMAP_INLINE float GetEntrySpeed() const { return _entrySpeed; }
		MAZEMAP_INLINE float getEntrySpeed() { return GetEntrySpeed(); }
		MAZEMAP_INLINE float getEntrySpeed() const { return GetEntrySpeed(); }
		MAZEMAP_INLINE void SetEntrySpeed(float entrySpeed) { _entrySpeed = entrySpeed; }
		MAZEMAP_INLINE void setEntrySpeed(float entrySpeed) { SetEntrySpeed(entrySpeed); }

		MAZEMAP_INLINE float GetExitSpeed() { return const_cast<const ManeuverInstance*>(this)->GetExitSpeed(); }
		MAZEMAP_INLINE float GetExitSpeed() const { return _exitSpeed; }
		MAZEMAP_INLINE float getExitSpeed() { return GetExitSpeed(); }
		MAZEMAP_INLINE float getExitSpeed() const { return GetExitSpeed(); }
		MAZEMAP_INLINE void SetExitSpeed(float exitSpeed) { _exitSpeed = exitSpeed; }
		MAZEMAP_INLINE void setExitSpeed(float exitSpeed) { SetExitSpeed(exitSpeed); }

		MAZEMAP_INLINE bool IsStraight() const
		{
			return _code != MC_NONE && _code <= S31;
		}
	};
}

