#include "pch.h"
#include "Vehicle.h"
#include "math.h"
namespace MazeMap
{


	void VehicleState::SetPosition( const Vector2f& position ) { _position = position; }
	Vector2f VehicleState::GetPosition() const { return _position; }
	Vector2f VehicleState::GetPosition() { return const_cast<const VehicleState*>(this)->GetPosition(); }

	void VehicleState::SetVelocity(const Vector2f& velocity) { _velocity = velocity; }
	Vector2f VehicleState::GetVelocity() const { return _velocity; }
	Vector2f VehicleState::GetVelocity() { return const_cast<const VehicleState*>(this)->GetVelocity(); }

	/*void VehicleState::SetAcceleration(const Vector2f& acceleration) { _acceleration = acceleration; }
	Vector2f VehicleState::GetAcceleration() const { return _acceleration; }
	Vector2f VehicleState::GetAcceleration() { return const_cast<const VehicleState*>(this)->GetAcceleration(); }*/

	void VehicleState::SetOrientation(const float& orientation) { _orientation = orientation; }
	float VehicleState::GetOrientation() const { return _orientation; }
	float VehicleState::GetOrientation() { return const_cast<const VehicleState*>(this)->GetOrientation(); }

	void VehicleState::SetRotationalVelocity(float rotationalVelocity) { _rotationalVelocity = rotationalVelocity; }
	float VehicleState::GetRotationalVelocity() const { return _rotationalVelocity; }
	float VehicleState::GetRotationalVelocity() { return const_cast<const VehicleState*>(this)->GetRotationalVelocity(); }

	void VehicleState::SetTime(float time) { _time = time; }
	float VehicleState::GetTime() const { return _time; }
	float VehicleState::GetTime() { return const_cast<const VehicleState*>(this)->GetTime(); }

	void VehicleState::SetMotorDriveL(float motorDriveL) { _motorDriveLeft = motorDriveL; }
	float VehicleState::GetMotorDriveL() const { return _motorDriveLeft; }
	float VehicleState::GetMotorDriveL() { return const_cast<const VehicleState*>(this)->GetMotorDriveL(); }

	void VehicleState::SetMotorDriveR(float motorDriveR) { _motorDriveRight = motorDriveR; }
	float VehicleState::GetMotorDriveR() const { return _motorDriveRight; }
	float VehicleState::GetMotorDriveR() { return const_cast<const VehicleState*>(this)->GetMotorDriveR(); }

	void VehicleState::SetPositionVar(const Vector2f& positionVariance) { _positionVar = positionVariance; }
	Vector2f VehicleState::GetPositionVar() const { return _position; }
	Vector2f VehicleState::GetPositionVar() { return const_cast<const VehicleState*>(this)->GetPositionVar(); }

	void VehicleState::SetVelocityVar(const Vector2f& velocityVariance) { _velocityVar = velocityVariance; }
	Vector2f VehicleState::GetVelocityVar() const { return _velocity; }
	Vector2f VehicleState::GetVelocityVar() { return const_cast<const VehicleState*>(this)->GetVelocityVar(); }

	/*void VehicleState::SetAccelerationVar(const Vector2f& accelerationVariance) { _accelerationVar = accelerationVariance; }
	Vector2f VehicleState::GetAccelerationVar() const { return _acceleration; }
	Vector2f VehicleState::GetAccelerationVar() { return const_cast<const VehicleState*>(this)->GetAccelerationVar(); }*/

	void VehicleState::SetOrientationVar(const float& orientationVariance) { _orientationVar = orientationVariance; }
	float VehicleState::GetOrientationVar() const { return _orientation; }
	float VehicleState::GetOrientationVar() { return const_cast<const VehicleState*>(this)->GetOrientationVar(); }

	void VehicleState::SetRotationalVelocityVar(float rotationalVelocityVariance) { _rotationalVelocityVar = rotationalVelocityVariance; }
	float VehicleState::GetRotationalVelocityVar() const { return _rotationalVelocity; }
	float VehicleState::GetRotationalVelocityVar() { return const_cast<const VehicleState*>(this)->GetRotationalVelocityVar(); }

	void VehicleState::SetMotorDriveLVar(float motorDriveLVariance) { _motorDriveLeftVar = motorDriveLVariance; }
	float VehicleState::GetMotorDriveLVar() const { return _motorDriveLeft; }
	float VehicleState::GetMotorDriveLVar() { return const_cast<const VehicleState*>(this)->GetMotorDriveLVar(); }

	void VehicleState::SetMotorDriveRVar(float motorDriveRVariance) { _motorDriveRightVar = motorDriveRVariance; }
	float VehicleState::GetMotorDriveRVar() const { return _motorDriveRight; }
	float VehicleState::GetMotorDriveRVar() { return const_cast<const VehicleState*>(this)->GetMotorDriveRVar(); }

	Vehicle::Vehicle()
		: _peakForwardAcceleration(15.0f)
		, _peakLateralAcceleration(15.0f)
		, _peakRotationalVelocity(6.0f)
		, _maxSpeed(4.0f)
	{}

	Vehicle::Vehicle(float peakForwardAcceleration, float peakLateralAcceleration, float peakRotationalVelocity, float maxSpeed)
		: _peakForwardAcceleration(peakForwardAcceleration)
		, _peakLateralAcceleration(peakLateralAcceleration)
		, _peakRotationalVelocity(peakRotationalVelocity)
		, _maxSpeed(maxSpeed)
	{
	}

	const VehicleState& Vehicle::GetVehicleState() const { return _stateHistory.GetLatest(); }
	void Vehicle::ProgressVehicleState(const VehicleState& previousState, VehicleState& projectedState, float timeDelta)
	{
		projectedState.SetTime(previousState.GetTime() + timeDelta);

		
	}

	float Vehicle::GetStraightLineCost(float distance, float initialVelocity, float finalVelocity)
	{
		return const_cast<const Vehicle*>(this)->GetStraightLineCost(distance, initialVelocity, finalVelocity);
	}
	float Vehicle::GetStraightLineCost(float distance, float initialVelocity, float finalVelocity) const
	{
		float distToV1 = (finalVelocity * finalVelocity - initialVelocity * initialVelocity) / (2 * GetMaxForwardAcceleration());
		// Accelerating without enough space to reach target
		if (distToV1 >= distance)
		{
			return (sqrtf(initialVelocity * initialVelocity + 2 * GetMaxForwardAcceleration() * distance) - initialVelocity) / GetMaxForwardAcceleration();
		}
		// Decelerating without enough space to reach target
		else if (-distToV1 >= distance)
		{
			return (sqrtf(finalVelocity * finalVelocity + 2 * GetMaxForwardAcceleration() * distance) - finalVelocity) / GetMaxForwardAcceleration();
		}

		// We have space, so see if we can hit Max V
		float distToMaxV = (GetMaxSpeed() * GetMaxSpeed() - initialVelocity * initialVelocity) / (2.0f * GetMaxForwardAcceleration());
		float distFromMaxV = (GetMaxSpeed() * GetMaxSpeed() - finalVelocity * finalVelocity) / (2.0f * GetMaxForwardAcceleration());

		if (distToMaxV + distFromMaxV < distance)
		{
			float maxVelTime = (distance - distToMaxV - distFromMaxV) / GetMaxSpeed();
			float accelTime = (GetMaxSpeed() - initialVelocity + GetMaxSpeed() - finalVelocity) / GetMaxForwardAcceleration();
			return maxVelTime + accelTime;
		}

		// We have space to hit V1, but not Vmax

		// We need to know the time to switch from accelerating to deccelerating
		// Start with time to hit V1, and then use symmetry on the remaining time
		float tToV1 = (finalVelocity - initialVelocity) / GetMaxForwardAcceleration();
		float vGreater = fmaxf(initialVelocity, finalVelocity);

		float dSymmetric = (distance - fabsf(distToV1)) / 2.0f;
		float tSymmetric = (sqrtf(vGreater * vGreater + 2.0f * GetMaxForwardAcceleration() * dSymmetric) - vGreater) / GetMaxForwardAcceleration();
		return tToV1 + 2.0f * tSymmetric;
	}
	float Vehicle::GetTurnCost(RelativeDirection relDir, float cellDimensions) const
	{
		float speed = GetTurnSpeed(relDir, cellDimensions);
		float quarterAngles = 0.0f;
		if (relDir == RelativeDirection::L45 || relDir == RelativeDirection::R45)
		{
			quarterAngles = 1.0f;
		}
		else if (relDir == RelativeDirection::L90 || relDir == RelativeDirection::R90)
		{
			quarterAngles = 2.0f;
		}
		else if (relDir == RelativeDirection::L135 || relDir == RelativeDirection::R135)
		{
			quarterAngles = 3.0f;
		}
		else if (relDir == RelativeDirection::Reverse)
		{
			quarterAngles = 4.0f;
		}
		return quarterAngles * PI_F * cellDimensions / 2.0f / speed;
	}
	float Vehicle::GetTurnSpeed(RelativeDirection relDir, float cellDimensions) const
	{
		float r = cellDimensions;
		if (relDir == RelativeDirection::R45 || relDir == RelativeDirection::L45)
		{
			r *= (1.0f + sqrtf(2.0f) / 2.0f);
		}

		return sqrtf(GetMaxLateralAcceleration() * r);
	}
	float Vehicle::GetFastestTurnSpeed(float cellDimensions) const
	{
		float r = cellDimensions * (2.0f);
		return sqrtf(GetMaxLateralAcceleration() * r);
	}
	float Vehicle::GetMaxForwardAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxForwardAcceleration(); }
	float Vehicle::GetMaxForwardAcceleration() const { return _peakForwardAcceleration; }
	float Vehicle::GetMaxLateralAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxLateralAcceleration(); }
	float Vehicle::GetMaxLateralAcceleration() const { return _peakLateralAcceleration; }
	float Vehicle::GetMaxRotationalVelocity() { return const_cast<const Vehicle*>(this)->GetMaxRotationalVelocity(); }
	float Vehicle::GetMaxRotationalVelocity() const { return _peakRotationalVelocity; }
	float Vehicle::GetMaxSpeed() { return const_cast<const Vehicle*>(this)->GetMaxSpeed(); }
	float Vehicle::GetMaxSpeed() const { return _maxSpeed; }

	void DirectStateUpdate(const VehicleState& previousState, VehicleState& result, float timeDelta)
	{
		result.SetTime(previousState.GetTime() + timeDelta);

		//result.SetAcceleration(previousState.GetAcceleration() + )
	}
	const VehicleState& Vehicle::GetVehicleState() { return const_cast<const Vehicle*>(this)->GetVehicleState(); }
}