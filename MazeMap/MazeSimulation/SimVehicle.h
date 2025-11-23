#pragma once
#include "..\MazeMap\Direction.h"
#include "..\MazeMap\Maze.h"
#include "mmsAPI.h"
class SimVehicle
{
private:
	MazeMap::MazeLocation _location;
	MazeMap::Direction _orientation;
public:
	SimVehicle();


	void Move(MazeMap::RelativeDirection relDir);

	void Move(MazeMap::RelativeDirection relDir, int halfSteps);
	void Move(MazeMap::Direction dir);
	void Move(MazeMap::Direction dir, int halfSteps);
	void Turn(MazeMap::RelativeDirection relDir);

	void MoveTo(MazeMap::MazeLocation location);

	bool ReadWall(MazeMap::RelativeDirection relDir);

	MazeMap::MazeLocation GetPosition();
	MazeMap::Direction GetOrientation();
};

