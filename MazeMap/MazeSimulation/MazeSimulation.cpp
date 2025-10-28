// MazeSimulation.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <stdint.h>
#include <iostream>
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\Maze.h"
#include "..\MazeMap\PathFinder.h"
#include "..\MazeMap\DirectionalPathFinder.h"
#include "SimVehicle.h"

MazeMap::Vehicle vehicle = MazeMap::Vehicle();
MazeMap::Maze maze = MazeMap::Maze();

SimVehicle simVehicle = SimVehicle();

void MovePath(const MazeMap::HalfStepPath<PATH_SIZE*2>& path)
{
    MazeMap::Direction d;

    MazeMap::MazeLocation current;
    for (uint8_t i = 1; i < path.GetSize(); i++)
    {
        MazeMap::MazeLocation next = path[i];

        simVehicle.MoveTo(next);
    }
}

void ReadWalls()
{
    MazeMap::Cell& c = maze[simVehicle.GetPosition().GetFirstConnectedCell()];
    for (MazeMap::Direction d = MazeMap::Direction::Up; d <= MazeMap::Direction::Right; d = static_cast<MazeMap::Direction>(d<<1))
    {
        if (c.GetWall(d) == MazeMap::WallState::Unknown)
        {
            if (simVehicle.ReadWall(d-simVehicle.GetOrientation()))
            {
                c.SetWall(d, MazeMap::WallState::Wall);
            }
            else
            {
                c.SetWall(d, MazeMap::WallState::NoWall);
            }
        }
    }
}

int main()
{
    MazeMap::FloodFillPathFinder pathfinder = MazeMap::FloodFillPathFinder(maze, vehicle);
    MazeMap::HalfStepPath<PATH_SIZE*2> path = MazeMap::HalfStepPath<PATH_SIZE * 2>();
    MazeMap::Direction d;
    while (!maze.IsComplete())
    {
        pathfinder.HalfStepPathToNearestUnknown(simVehicle.GetPosition().GetFirstConnectedCell(), simVehicle.GetOrientation(), path);
        MovePath(path);
        ReadWalls();
    }
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
