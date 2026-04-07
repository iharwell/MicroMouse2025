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
        // Initializes the maze with the guaranteed starting wall arrangement.
        Maze();
		// Initializes the maze with the provided cell dimension and guaranteed starting wall arrangement.
        Maze(float cellDimension);

        // The left-right size of the maze in cells.
        uint8_t GetXSize();
        // The left-right size of the maze in cells.
        uint8_t GetXSize() const;
        // The up-down size of the maze in cells.
        uint8_t GetYSize();
        // The up-down size of the maze in cells.
        uint8_t GetYSize() const;

        // Returns the cell of the maze at the specified location. To set walls, use SetWall instead.
        Cell& operator()(uint8_t x, uint8_t y);
        // Returns the cell of the maze at the specified location.
        const Cell& operator()(uint8_t x, uint8_t y) const;
        // Returns the cell of the maze at the specified location. To set walls, use SetWall instead.
        Cell& operator[](CellCoordinates coords);
        // Returns the cell of the maze at the specified location.
        const Cell& operator[](CellCoordinates coords) const;


        // Sets the state of a wall representation in the maze and guarantees a valid maze state.
        void SetWall(uint8_t x, uint8_t y, Direction direction, WallState state);
        // Sets the state of a wall representation in the maze and guarantees a valid maze state.
        void SetWall(CellCoordinates coords, Direction direction, WallState state);
        // Sets the state of a wall representation in the maze and guarantees a valid maze state.
        void SetWall(Cell& cell, Direction direction, WallState state);

        // Returns the cell of the maze at the specified location. To set walls, use SetWall instead.
        Cell& Index(int x, int y);
        // Returns the cell of the maze at the specified location.
        const Cell& Index(int x, int y) const;
        // Returns the cell of the maze at the specified location. To set walls, use SetWall instead.
        Cell& Index(CellCoordinates coords);
        // Returns the cell of the maze at the specified location.
        const Cell& Index(CellCoordinates coords) const;

        // Returns the size of a cell in meters.
        static float GetCellDimension();

        // Indicates whether the goal of the maze has been located.
        bool HasFoundGoal();

        // Returns a bit mask indicating all cells that can be reached from the start
        // of the maze, with true indicating a reachable cell and false indicating an unreachable cell.
        MazeMask GetReachableMask() const;
        // Returns a bit mask indicating all cells that can be reached from the start
        // of the maze, with true indicating a reachable cell and false indicating an unreachable cell.
        MazeMask GetReachableMask();

        // Returns the lower left cell of the goal of the maze, so the goal includes the returned address,
        // as well as the (x,y+1), (x+1,y), and (x+1, y+1) relative cells.
        CellCoordinates GetGoalLowerLeft();
        // Returns the lower left cell of the goal of the maze, so the goal includes the returned address,
        // as well as the (x,y+1), (x+1,y), and (x+1, y+1) relative cells.
        CellCoordinates GetGoalLowerLeft() const;

        // Indicates whether any accessible unknowns remain in the maze.
        bool IsComplete();

        // For a complete maze, this precalculates the goal location and accessibility masks for later use.
        void PreCalculate();

        // Returns whether a given location is accessible (true) or not (false).
        bool IsAccessibleLocation(MazeLocation location);
        // Returns whether a given location is accessible (true) or not (false).
        bool IsAccessibleLocation(MazeLocation location) const;

        // Indicates whether a given cell is branching (has 3+ open sides) or not.
        bool IsIntersection(CellCoordinates location);
        // Indicates whether a given cell is branching (has 3+ open sides) or not.
        bool IsIntersection(CellCoordinates location) const;

        // Indicates whether a given cell is branching (has 3+ open sides) or not.
        bool IsIntersection(MazeLocation location);
        // Indicates whether a given cell is branching (has 3+ open sides) or not.
        bool IsIntersection(MazeLocation location) const;

        // Calculates a mask indicating whether a cell is a dead end (true, 3 walls) or not (false).
        MazeMask DeadEndMask(CellCoordinates startLocation) const;

        // Determines whether a given traversal move is valid in the maze.
        bool IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction);
        // Determines whether a given traversal move is valid in the maze.
        bool IsValidMove(DirectionalLocation location, RelativeDirectionalDistance instruction) const;

        // Serializes the maze to a file.
        void ExportToFile(const char* fileName);
        // Serializes the maze to a file.
        void ExportToFileC(const char* fileName) const;

        // Serializes the maze to mazeDump.txt.
        void DumpMaze() const;
    };
}

#endif
