#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\VehicleState.h"
#pragma once
#include "Defines.h"
#include "Vector2f.h"

namespace MazeMap
{
	class EXPORT VehicleState
	{
	private:

		Vectorf<2> _position;
		float _velocity;
		float _orientation;
		float _rotationalVelocity;

		float _motorDriveLeft;
		float _motorDriveRight;

		float _time;

		Vectorf<2> _positionVar;
		float _velocityVar;
		float _orientationVar;
		float _rotationalVelocityVar;

		float _motorDriveLeftVar;
		float _motorDriveRightVar;
	public:

		void SetPosition(const Vectorf<2>& position);
		Vectorf<2> GetPosition();
		Vectorf<2> GetPosition() const;

		void SetVelocity(float velocity);
		float GetVelocity();
		float GetVelocity() const;

		void SetOrientation(const float& orientation);
		float GetOrientation();
		float GetOrientation() const;

		void SetRotationalVelocity(float rotationalVelocity);
		float GetRotationalVelocity();
		float GetRotationalVelocity() const;

		void SetTime(float time);
		float GetTime();
		float GetTime() const;

		void SetMotorDriveL(float motorDriveL);
		float GetMotorDriveL();
		float GetMotorDriveL() const;

		void SetMotorDriveR(float motorDriveR);
		float GetMotorDriveR();
		float GetMotorDriveR() const;

		void SetPositionVar(const Vectorf<2>& positionVariance);
		Vectorf<2> GetPositionVar();
		Vectorf<2> GetPositionVar() const;

		void SetVelocityVar(float velocityVariance);
		float GetVelocityVar();
		float GetVelocityVar() const;

		void SetOrientationVar(const float& orientationVariance);
		float GetOrientationVar();
		float GetOrientationVar() const;

		void SetRotationalVelocityVar(float rotationalVelocityVariance);
		float GetRotationalVelocityVar();
		float GetRotationalVelocityVar() const;

		void SetMotorDriveLVar(float motorDriveLVariance);
		float GetMotorDriveLVar();
		float GetMotorDriveLVar() const;

		void SetMotorDriveRVar(float motorDriveRVariance);
		float GetMotorDriveRVar();
		float GetMotorDriveRVar() const;
	};
}
