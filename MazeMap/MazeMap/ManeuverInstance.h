#pragma once

#include "Defines.h"
#include "ManeuverSet.h"

namespace MazeMap
{
	class EXPORT ManeuverPoint
	{
	public:
		ManeuverPoint(float x, float y, float theta, float omega, float velocity)
			: X(x), Y(y), Theta(theta), Omega(omega), Velocity(velocity)
		{ }
		float X;
		float Y;
		float Theta;
		float Omega;
		float Velocity;
	};

	class EXPORT ManeuverInstance
	{
	private:
		ManeuverCode _code;
		DirectionalLocation _start;
		float _entrySpeed;
		float _exitSpeed;
	public:
		ManeuverInstance(ManeuverCode code, DirectionalLocation start, float entrySpeed, float exitSpeed, float turnInDist, float turnCurvature)
			: _code(code), _start(start), _entrySpeed(entrySpeed), _exitSpeed(exitSpeed), _turnInDistance(turnInDist), _turnCurvature(turnCurvature)
		{
		}

		ManeuverCode getCode() { return const_cast<const ManeuverInstance*>(this)->getCode(); }
		ManeuverCode getCode() const { return _code; }
		void setCode(ManeuverCode code) { _code = code; }

		DirectionalLocation getStart() { return const_cast<const ManeuverInstance*>(this)->getStart(); }
		DirectionalLocation getStart() const { return _start; }
		void setCode(DirectionalLocation start) { _start = start; }

		DirectionalLocation getEnd() { return const_cast<const ManeuverInstance*>(this)->getEnd(); }
		DirectionalLocation getEnd() const
		{
			auto set = ManeuverSet::GetSet();
			return set.Move(getCode(), getStart());
		}

		float getEntrySpeed() { return const_cast<const ManeuverInstance*>(this)->getEntrySpeed(); }
		float getEntrySpeed() const { return _entrySpeed; }
		void setEntrySpeed(float entrySpeed) { _entrySpeed = entrySpeed; }

		float getExitSpeed() { return const_cast<const ManeuverInstance*>(this)->getExitSpeed(); }
		float getExitSpeed() const { return _exitSpeed; }
		void setExitSpeed(float exitSpeed) { _exitSpeed = exitSpeed; }

		float getTurnInDistance() { return const_cast<const ManeuverInstance*>(this)->getTurnInDistance(); }
		float getTurnInDistance() const
		{
			const ManeuverSet& set = ManeuverSet::GetSet();
		}

		float getTurnCurvature() { return const_cast<const ManeuverInstance*>(this)->getTurnCurvature(); }
		float getTurnCurvature() const
		{
			if (getCode() <= ManeuverCode::S31)
			{
				return 0.0f;
			}
		}

		void EvaluateAtDistance(float distance, ManeuverPoint& result, const Vehicle& vehicle, const Maze& maze)
		{
			float initialForwardX, initialForwardY;
			GetHeading(getStart().GetDirection(), initialForwardX, initialForwardY);
			float initialX, initialY;
			getStart().GetLocation().GetPhysicalLocation(maze.GetCellDimension(), initialX, initialY);

			if (getCode() < 32)
			{

			}
		}
	private:
		void EvaluateTurnInOutAtDistance(float distanceIntoTurnIn, float omega0, float omega1, ManeuverPoint& result, const Vehicle& vehicle, const Maze& maze)
		{
			float curvature = getTurnCurvature();
			float k = 1.0f / (curvature * 2 * vehicle.GetSpeedFromCurvature(curvature));
			result.Omega = omega0 + (k * distanceIntoTurnIn * distanceIntoTurnIn);
			
		}
		void EvaluateTurnInAtDistance(float distanceIntoTurnIn, ManeuverPoint& result, const Vehicle& vehicle, const Maze& maze)
		{
			float curvature = getTurnCurvature();
			float k = 1.0f / (curvature * 2 * vehicle.GetSpeedFromCurvature(curvature));
			result.Omega =
		}
	};
}
