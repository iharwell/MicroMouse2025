#include "pch.h"
#include "Vehicle.h"
#include "math.h"
namespace MazeMap
{



	Vehicle::Vehicle()
		: _peakForwardAcceleration(15.0f)
		, _peakLateralAcceleration(15.0f)
		, _peakRotationalVelocity(6.0f)
		, _maxSpeed(4.0f)
		, _peakAngularAcceleration(5000.0f)
		, _width(0.08f)
		, _mass(0.1f)
		, _centerOfMassHeight(0.005f)
		, _turningMomentOfInertia(0.00005f)
	{}

	Vehicle::Vehicle(float peakForwardAcceleration, float peakLateralAcceleration, float peakRotationalVelocity, float maxSpeed, float peakAngularAcceleration)
		: _peakForwardAcceleration(peakForwardAcceleration)
		, _peakLateralAcceleration(peakLateralAcceleration)
		, _peakRotationalVelocity(peakRotationalVelocity)
		, _maxSpeed(maxSpeed)
		, _peakAngularAcceleration(peakAngularAcceleration)
		, _width(0.08f)
		, _mass(0.1f)
		, _centerOfMassHeight(0.005f)
		, _turningMomentOfInertia(0.00005f)
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
		float tToV1 = fabsf(finalVelocity - initialVelocity) / GetMaxForwardAcceleration();
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
		float r = cellDimensions/2.0f;
		if (relDir == RelativeDirection::R45 || relDir == RelativeDirection::L45)
		{
			r *= (2.0f + sqrtf(2.0f));
		}

		return sqrtf(GetMaxLateralAcceleration() * r);
	}
	float Vehicle::GetFastestTurnSpeed(float cellDimensions) const
	{
		float r = cellDimensions * (2.0f);
		return sqrtf(GetMaxLateralAcceleration() * r);
	}

	float Vehicle::GetTurnSpeed(float turningRadius) const { return sqrtf(GetMaxLateralAcceleration() * turningRadius); }
	float Vehicle::GetMaxForwardAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxForwardAcceleration(); }
	float Vehicle::GetMaxForwardAcceleration() const { return _peakForwardAcceleration; }
	float Vehicle::GetMaxLateralAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxLateralAcceleration(); }
	float Vehicle::GetMaxLateralAcceleration() const { return _peakLateralAcceleration; }
	float Vehicle::GetMaxRotationalVelocity() { return const_cast<const Vehicle*>(this)->GetMaxRotationalVelocity(); }
	float Vehicle::GetMaxRotationalVelocity() const { return _peakRotationalVelocity; }
	float Vehicle::GetMaxAngularAcceleration() { return const_cast<const Vehicle*>(this)->GetMaxAngularAcceleration(); }
	float Vehicle::GetMaxAngularAcceleration() const { return _peakAngularAcceleration; }
	float Vehicle::GetMaxSpeed() { return const_cast<const Vehicle*>(this)->GetMaxSpeed(); }
	float Vehicle::GetMaxSpeed() const { return _maxSpeed; }
	float Vehicle::GetWidth() { return const_cast<const Vehicle*>(this)->GetWidth(); }
	float Vehicle::GetWidth() const { return _width; }

	void DirectStateUpdate(const VehicleState& previousState, VehicleState& result, float timeDelta)
	{
		result.SetTime(previousState.GetTime() + timeDelta);

		//result.SetAcceleration(previousState.GetAcceleration() + )
	}
	const VehicleState& Vehicle::GetVehicleState() { return const_cast<const Vehicle*>(this)->GetVehicleState(); }
}