#pragma once

#include "Defines.h"
#include "Vector2f.h"
#include "Direction.h"
#include "CircularBuffer.h"
#include "VehicleState.h"
#include "MotorEncoderDrive.h"

namespace MazeMap
{

	class EXPORT Vehicle
	{
	private:
		CircularBuffer<VehicleState, 15> _stateHistory;
		float _peakForwardAcceleration;
		float _peakLateralAcceleration;
		float _peakRotationalVelocity;
		float _peakAngularAcceleration;

		const float _mass;
		const float _centerOfMassHeight;
		const float _turningMomentOfInertia;
		const float _trackWidth;
		const MotorEncoderDrive _leftDriver;
		const MotorEncoderDrive _rightDriver;

		float _downforce;

		float _maxSpeed;
		float _width;
	public:
		Vehicle();
		Vehicle(
			float peakForwardAcceleration,
			float peakLateralAcceleration,
			float peakRotationalVelocity,
			float maxSpeed,
			float peakAngularAcceleration);
		Vehicle(
			float mass,
			float centerOfMassHeight,
			float turningMomentOfInertia,
			float maxSpeed,
			float trackWidth,
			float stallTorqueAtWheels);

		const VehicleState& GetVehicleState();
		const VehicleState& GetVehicleState() const;

		void ProgressVehicleState(const VehicleState& previousState, VehicleState& projectedState, float timeDelta);

		float GetStraightLineCost(float distance, float initialVelocity, float finalVelocity);
		float GetStraightLineCost(float distance, float initialVelocity, float finalVelocity) const;

		float GetTurnCost(RelativeDirection relDir, float cellDimensions);
		float GetTurnCost(RelativeDirection relDir, float cellDimensions) const;

		float GetTurnSpeed(RelativeDirection relDir, float cellDimensions);
		float GetTurnSpeed(RelativeDirection relDir, float cellDimensions) const;

		float GetTurnSpeed(float turningRadius) const;
		float GetSpeedFromCurvature(float curvature) const;

		float GetFastestTurnSpeed(float cellDimensions);
		float GetFastestTurnSpeed(float cellDimensions) const;

		float GetPeakRotationalVelocity() const { return _peakRotationalVelocity; }

		float GetMaxForwardAcceleration();
		float GetMaxForwardAcceleration() const;

		float GetCompensatedMaxForwardAcceleration(float velocity);
		float GetCompensatedMaxForwardAcceleration(float velocity) const;

		float GetMaxLateralAcceleration();
		float GetMaxLateralAcceleration() const;

		float GetMaxRotationalVelocity();
		float GetMaxRotationalVelocity() const;

		float GetMaxAngularAcceleration();
		float GetMaxAngularAcceleration() const;

		void SetMaxSpeed(float maxSpeed);
		float GetMaxSpeed();
		float GetMaxSpeed() const;

		float GetWidth();
		float GetWidth() const;
	};

	// Vehicle(
	//		float peakForwardAcceleration,
	//		float peakLateralAcceleration,
	//		float peakRotationalVelocity,
	//		float maxSpeed,
	//		float peakAngularAcceleration);

}
