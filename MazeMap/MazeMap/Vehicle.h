#pragma once

#include "Defines.h"
#include "Vector2f.h"
#include "Direction.h"
#include "CircularBuffer.h"

namespace MazeMap
{
	class EXPORT VehicleState
	{
	private:
		Vector2f _position;
		Vector2f _velocity;
		float _orientation;
		float _rotationalVelocity;

		float _motorDriveLeft;
		float _motorDriveRight;

		float _time;

		Vector2f _positionVar;
		Vector2f _velocityVar;
		float _orientationVar;
		float _rotationalVelocityVar;

		float _motorDriveLeftVar;
		float _motorDriveRightVar;
	public:

		void SetPosition(const Vector2f& position);
		Vector2f GetPosition();
		Vector2f GetPosition() const;

		void SetVelocity(const Vector2f& velocity);
		Vector2f GetVelocity();
		Vector2f GetVelocity() const;

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

		void SetPositionVar(const Vector2f& positionVariance);
		Vector2f GetPositionVar();
		Vector2f GetPositionVar() const;

		void SetVelocityVar(const Vector2f& velocityVariance);
		Vector2f GetVelocityVar();
		Vector2f GetVelocityVar() const;

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

	class EXPORT Vehicle
	{
	private:
		CircularBuffer<VehicleState, 15> _stateHistory;
		float _peakForwardAcceleration;
		float _peakLateralAcceleration;
		float _peakRotationalVelocity;
		float _maxSpeed;
	public:
		Vehicle();
		Vehicle(float peakForwardAcceleration, float peakLateralAcceleration, float peakRotationalVelocity, float maxSpeed);
		const VehicleState& GetVehicleState();
		const VehicleState& GetVehicleState() const;

		void ProgressVehicleState(const VehicleState& previousState, VehicleState& projectedState, float timeDelta);

		float GetStraightLineCost(float distance, float initialVelocity, float finalVelocity);
		float GetStraightLineCost(float distance, float initialVelocity, float finalVelocity) const;
		float GetTurnCost(RelativeDirection relDir, float cellDimensions);
		float GetTurnCost(RelativeDirection relDir, float cellDimensions) const;
		float GetTurnSpeed(RelativeDirection relDir, float cellDimensions);
		float GetTurnSpeed(RelativeDirection relDir, float cellDimensions) const;
		float GetFastestTurnSpeed(float cellDimensions);
		float GetFastestTurnSpeed(float cellDimensions) const;

		float GetMaxForwardAcceleration();
		float GetMaxForwardAcceleration() const;
		float GetMaxLateralAcceleration();
		float GetMaxLateralAcceleration() const;
		float GetMaxRotationalVelocity();
		float GetMaxRotationalVelocity() const;
		float GetMaxSpeed();
		float GetMaxSpeed() const;
	};
}
