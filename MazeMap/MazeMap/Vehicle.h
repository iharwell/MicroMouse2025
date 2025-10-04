#pragma once

#include "Defines.h"
#include "Vector2f.h"
#include "CircularBuffer.h"

namespace MazeMap
{
	class EXPORT VehicleState
	{
	private:
		Vector2f _position;
		Vector2f _velocity;
		Vector2f _acceleration;
		Vector2f _orientation;
		float _rotationalVelocity;

		float _motorDriveLeft;
		float _motorDriveRight;

		float _time;

		Vector2f _positionVar;
		Vector2f _velocityVar;
		Vector2f _accelerationVar;
		Vector2f _orientationVar;
		float _rotationalVelocityVar;

		float _motorDriveLeftVar;
		float _motorDriveRightVar;
	public:

		Vector2f GetPosition();
		Vector2f GetPosition() const;

		Vector2f GetVelocity();
		Vector2f GetVelocity() const;

		Vector2f GetAcceleration();
		Vector2f GetAcceleration() const;

		Vector2f GetOrientation();
		Vector2f GetOrientation() const;

		float GetRotationalVelocity();
		float GetRotationalVelocity() const;

		float GetTime();
		float GetTime() const;

		float GetMotorDriveL();
		float GetMotorDriveL() const;

		float GetMotorDriveR();
		float GetMotorDriveR() const;

		Vector2f GetPositionVar();
		Vector2f GetPositionVar() const;

		Vector2f GetVelocityVar();
		Vector2f GetVelocityVar() const;

		Vector2f GetAccelerationVar();
		Vector2f GetAccelerationVar() const;

		Vector2f GetOrientationVar();
		Vector2f GetOrientationVar() const;

		float GetRotationalVelocityVar();
		float GetRotationalVelocityVar() const;

		float GetMotorDriveLVar();
		float GetMotorDriveLVar() const;

		float GetMotorDriveRVar();
		float GetMotorDriveRVar() const;
	};

	class EXPORT Vehicle
	{
	private:
		CircularBuffer<VehicleState, 15> _stateHistory;

		float _peakAcceleration;
		float _peakRotationalVelocity;

	public:
		Vehicle();
		Vehicle(float peakAcceleration, float peakRotationalVelocity);
		const VehicleState& GetVehicleState();
		const VehicleState& GetVehicleState() const;

	};
}
