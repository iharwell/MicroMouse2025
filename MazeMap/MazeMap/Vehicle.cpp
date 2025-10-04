#include "pch.h"
#include "Vehicle.h"

namespace MazeMap
{
	Vector2f VehicleState::GetPosition() const { return _position; }
	Vector2f VehicleState::GetPosition() { return const_cast<const VehicleState*>(this)->GetPosition(); }

	Vector2f VehicleState::GetVelocity() const { return _velocity; }
	Vector2f VehicleState::GetVelocity() { return const_cast<const VehicleState*>(this)->GetVelocity(); }

	Vector2f VehicleState::GetAcceleration() const { return _acceleration; }
	Vector2f VehicleState::GetAcceleration() { return const_cast<const VehicleState*>(this)->GetAcceleration(); }

	Vector2f VehicleState::GetOrientation() const { return _orientation; }
	Vector2f VehicleState::GetOrientation() { return const_cast<const VehicleState*>(this)->GetOrientation(); }

	float VehicleState::GetRotationalVelocity() const { return _rotationalVelocity; }
	float VehicleState::GetRotationalVelocity() { return const_cast<const VehicleState*>(this)->GetRotationalVelocity(); }

	float VehicleState::GetTime() const { return _time; }
	float VehicleState::GetTime() { return const_cast<const VehicleState*>(this)->GetTime(); }

	float VehicleState::GetMotorDriveL() const { return _motorDriveLeft; }
	float VehicleState::GetMotorDriveL() { return const_cast<const VehicleState*>(this)->GetMotorDriveL(); }

	float VehicleState::GetMotorDriveR() const { return _motorDriveRight; }
	float VehicleState::GetMotorDriveR() { return const_cast<const VehicleState*>(this)->GetMotorDriveR(); }

	Vector2f VehicleState::GetPositionVar() const { return _position; }
	Vector2f VehicleState::GetPositionVar() { return const_cast<const VehicleState*>(this)->GetPositionVar(); }

	Vector2f VehicleState::GetVelocityVar() const { return _velocity; }
	Vector2f VehicleState::GetVelocityVar() { return const_cast<const VehicleState*>(this)->GetVelocityVar(); }

	Vector2f VehicleState::GetAccelerationVar() const { return _acceleration; }
	Vector2f VehicleState::GetAccelerationVar() { return const_cast<const VehicleState*>(this)->GetAccelerationVar(); }

	Vector2f VehicleState::GetOrientationVar() const { return _orientation; }
	Vector2f VehicleState::GetOrientationVar() { return const_cast<const VehicleState*>(this)->GetOrientationVar(); }

	float VehicleState::GetRotationalVelocityVar() const { return _rotationalVelocity; }
	float VehicleState::GetRotationalVelocityVar() { return const_cast<const VehicleState*>(this)->GetRotationalVelocityVar(); }

	float VehicleState::GetMotorDriveLVar() const { return _motorDriveLeft; }
	float VehicleState::GetMotorDriveLVar() { return const_cast<const VehicleState*>(this)->GetMotorDriveLVar(); }

	float VehicleState::GetMotorDriveRVar() const { return _motorDriveRight; }
	float VehicleState::GetMotorDriveRVar() { return const_cast<const VehicleState*>(this)->GetMotorDriveRVar(); }

	const VehicleState& Vehicle::GetVehicleState() const { return _stateHistory.GetLatest(); }
	const VehicleState& Vehicle::GetVehicleState() { return const_cast<const Vehicle*>(this)->GetVehicleState(); }
}