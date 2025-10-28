//#include "DirectionalLocation.h"
#include "SimVehicle.h"

SimVehicle::SimVehicle()
	: _location()
	, _orientation()
{}


void SimVehicle::Move(MazeMap::RelativeDirection relDir)
{
	if (relDir != MazeMap::RelativeDirection::Forward)
	{
		Turn(relDir);
	}

	_location = _location >> _orientation;

	API::moveForward(1);
}
void SimVehicle::Move(MazeMap::Direction dir)
{
	Move(dir - _orientation);
}
void SimVehicle::Turn(MazeMap::RelativeDirection relDir)
{
	_orientation = _orientation + relDir;
	switch (relDir)
	{
	case MazeMap::RelativeDirection::Forward:
		return;
	case MazeMap::RelativeDirection::L45:
		API::turnLeft45();
		return;
	case MazeMap::RelativeDirection::R45:
		API::turnRight45();
		return;
	case MazeMap::RelativeDirection::L90:
		API::turnLeft();
		return;
	case MazeMap::RelativeDirection::R90:
		API::turnRight();
		return;
	case MazeMap::RelativeDirection::L135:
		API::turnLeft();
		API::turnLeft45();
		return;
	case MazeMap::RelativeDirection::R135:
		API::turnRight();
		API::turnRight45();
		return;
	case MazeMap::RelativeDirection::Reverse:
		API::turnRight();
		API::turnRight();
		return;
	}
}

void SimVehicle::MoveTo(MazeMap::MazeLocation location)
{
	Move(GetPosition().DirectionTo(location));
}

bool SimVehicle::ReadWall(MazeMap::RelativeDirection relDir)
{
	bool b = false;
	switch (relDir)
	{
	case MazeMap::RelativeDirection::Forward:
		return API::wallFront();
	case MazeMap::RelativeDirection::L90:
		return API::wallLeft();
	case MazeMap::RelativeDirection::R90:
		return API::wallRight();
	case MazeMap::RelativeDirection::Reverse:
		API::turnRight();
		b = API::wallRight();
		API::turnLeft();
		return b;
	default:
		return false;
	}
}

MazeMap::MazeLocation SimVehicle::GetPosition()
{
	return _location;
}
MazeMap::Direction SimVehicle::GetOrientation()
{
	return _orientation;
}
