#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Maze.h"
#ifndef MAZE_H
#define MAZE_H
#define MAZE_EXPORT
#include "Defines.h"
#include "CellCoordinates.h"
#include "MazeLocation.h"
#include "DirectionalLocation.h"
#include "Cell.h"
#include "MazeMask.h"

namespace MazeMap
{
    class EXPORT Maze
    {
    private:
        Cell _cells[16][16];
        static const float _cellDimension;
        CellCoordinates _goal;
        MazeMask _reachable;
        bool _complete;
        bool _reachableCalculated;
        bool _goalFound;
        DoubleMazeMask _accessible;
        void FindReachables(MazeMask& mask, CellCoordinates cell) const;
    public:
        Maze();
        Maze(float cellDimension);

        uint8_t GetXSize();
        uint8_t GetXSize() const;
        uint8_t GetYSize();
        uint8_t GetYSize() const;

        Cell& operator()(uint8_t x, uint8_t y);
        const Cell& operator()(uint8_t x, uint8_t y) const;
        Cell& operator[](CellCoordinates coords);
        const Cell& operator[](CellCoordinates coords) const;

        void SetWall(Cell& cell, Direction direction, WallState state);

        Cell& Index(int x, int y);
        const Cell& Index(int x, int y) const;
        Cell& Index(CellCoordinates coords);
        const Cell& Index(CellCoordinates coords) const;

        static float GetCellDimension();

        bool HasFoundGoal();

        MazeMask GetReachableMask() const;
        MazeMask GetReachableMask();

        CellCoordinates GetGoalLowerLeft();
        CellCoordinates GetGoalLowerLeft() const;

        bool IsComplete();

        void PreCalculate();

        bool IsAccessibleLocation(MazeLocation location);
        bool IsAccessibleLocation(MazeLocation location) const;

        bool IsIntersection(CellCoordinates location);
        bool IsIntersection(CellCoordinates location) const;

        bool IsIntersection(MazeLocation location);
        bool IsIntersection(MazeLocation location) const;

        MazeMask DeadEndMask(CellCoordinates startLocation) const;

        bool IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction);
        bool IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction) const;

        void ExportToFile(const char* fileName);
        void ExportToFileC(const char* fileName) const;
        void DumpMaze() const;
    };
}

#endif
