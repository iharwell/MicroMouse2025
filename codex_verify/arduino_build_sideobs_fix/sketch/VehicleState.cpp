#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\VehicleState.cpp"
#include "pch.h"
#include "VehicleState.h"

namespace MazeMap
{
	void VehicleState::SetPosition(const Vectorf<2>& position) { _position = position; }
	Vectorf<2> VehicleState::GetPosition() const { return _position; }
	Vectorf<2> VehicleState::GetPosition() { return const_cast<const VehicleState*>(this)->GetPosition(); }

	void VehicleState::SetVelocity(float velocity) { _velocity = velocity; }
	float VehicleState::GetVelocity() const { return _velocity; }
	float VehicleState::GetVelocity() { return const_cast<const VehicleState*>(this)->GetVelocity(); }

	/*void VehicleState::SetAcceleration(const Vector2f& acceleration) { _acceleration = acceleration; }
	float VehicleState::GetAcceleration() const { return _acceleration; }
	float VehicleState::GetAcceleration() { return const_cast<const VehicleState*>(this)->GetAcceleration(); }*/

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

	void VehicleState::SetPositionVar(const Vectorf<2>& positionVariance) { _positionVar = positionVariance; }
	Vectorf<2> VehicleState::GetPositionVar() const { return _position; }
	Vectorf<2> VehicleState::GetPositionVar() { return const_cast<const VehicleState*>(this)->GetPositionVar(); }

	void VehicleState::SetVelocityVar(float velocityVariance) { _velocityVar = velocityVariance; }
	float VehicleState::GetVelocityVar() const { return _velocityVar; }
	float VehicleState::GetVelocityVar() { return const_cast<const VehicleState*>(this)->GetVelocityVar(); }

	/*void VehicleState::SetAccelerationVar(const Vector2f& accelerationVariance) { _accelerationVar = accelerationVariance; }
	float VehicleState::GetAccelerationVar() const { return _acceleration; }
	float VehicleState::GetAccelerationVar() { return const_cast<const VehicleState*>(this)->GetAccelerationVar(); }*/

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
}