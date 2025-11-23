//#include "DirectionalLocation.h"
#include "SimVehicle.h"

SimVehicle::SimVehicle()
	: _location()
	, _orientation(MazeMap::Direction::Up)
{}

char directionLetters[9] = { ' ', 'n', 's', ' ', 'w', ' ', ' ' , ' ' , 'e' };

void SimVehicle::Move(MazeMap::RelativeDirection relDir)
{
	if (relDir != MazeMap::RelativeDirection::Forward)
	{
		Turn(relDir);
	}

	_location = _location >> _orientation;

	API::moveForwardHalf(1);
}
void SimVehicle::Move(MazeMap::RelativeDirection relDir, int halfSteps)
{
	if (relDir != MazeMap::RelativeDirection::Forward)
	{
		Turn(relDir);
	}


	for (size_t i = 0; i < halfSteps; i++)
	{
		_location = _location >> _orientation;
		API::moveForwardHalf(1);

	}
}
void SimVehicle::Move(MazeMap::Direction dir)
{
	Move(dir - _orientation);
}
void SimVehicle::Move(MazeMap::Direction dir, int halfSteps)
{
	Move(dir - _orientation, halfSteps);
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
	MazeMap::CellCoordinates c = GetPosition().GetFirstConnectedCell();
	switch (relDir)
	{
	case MazeMap::RelativeDirection::Forward:
		b = API::wallFront();
		if (b)
		{
			API::setWall(c.GetX(), c.GetY(), directionLetters[GetOrientation()]);
		}
		return b;
	case MazeMap::RelativeDirection::L90:
		b = API::wallLeft();
		if (b)
		{
			API::setWall(c.GetX(), c.GetY(), directionLetters[GetOrientation() + MazeMap::RelativeDirection::L90]);
		}
		return b;
	case MazeMap::RelativeDirection::R90:
		b = API::wallRight();
		if (b)
		{
			API::setWall(c.GetX(), c.GetY(), directionLetters[GetOrientation() + MazeMap::RelativeDirection::R90]);
		}
		return b;
	case MazeMap::RelativeDirection::Reverse:
		API::turnRight();
		b = API::wallRight();
		API::turnLeft();
		if (b)
		{
			API::setWall(c.GetX(), c.GetY(), directionLetters[GetOrientation() + MazeMap::RelativeDirection::R]);
		}
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
