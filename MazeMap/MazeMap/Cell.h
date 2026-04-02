#ifndef CELL_H
#define CELL_H

#include "defines.h"
#include "Direction.h"
#include "CellCoordinates.h"
#include <stdint.h>
namespace MazeMap
{
	// A simple union to enable serialization of the Cell class more efficiently.
	union EXPORT CharBlock
	{
		char chars[4];
		int data;
	};

	// The state of a wall in the map of the maze.
	enum EXPORT WallState : uint8_t
	{
		// Indicates that we don't know if there's a wall present or not.
		Unknown = 0,
		// Indicates that we know that there isn't a wall present.
		NoWall = 1,
		// Indicates that we know that there is a wall present.
		Wall = 3
	};

	// Represents a single cell in the maze, and used with the Maze class for navigation and mapping.
	class EXPORT Cell
	{
	private:
		CellCoordinates _coords;
		unsigned char _data;
	public:
		Cell();
		Cell(uint8_t x, uint8_t y);
		Cell(uint8_t x, uint8_t y, WallState up, WallState down, WallState left, WallState right);

		// Sets the WallState for the indicated side of this Cell.
		void SetWall(Direction direction, WallState state);
		// Gets the WallState for the indicated side of this Cell.
		WallState GetWall(Direction direction);
		// Gets the WallState for the indicated side of this Cell.
		WallState GetWall(Direction direction) const;

		// Gets the WallState for the upper side of this Cell.
		WallState GetUp();
		// Gets the WallState for the upper side of this Cell.
		WallState GetUp() const;

		// Gets the WallState for the lower side of this Cell.
		WallState GetDown();
		// Gets the WallState for the lower side of this Cell.
		WallState GetDown() const;

		// Gets the WallState for the left side of this Cell.
		WallState GetLeft();
		// Gets the WallState for the left side of this Cell.
		WallState GetLeft() const;

		// Gets the WallState for the right side of this Cell.
		WallState GetRight();
		// Gets the WallState for the right side of this Cell.
		WallState GetRight() const;

		// Gets the CellCoordinates of this cell.
		CellCoordinates GetCoords();
		// Gets the CellCoordinates of this cell.
		CellCoordinates GetCoords() const;

		// Gets the x-coordinate of this cell.
		// Shortcut for GetCoords().GetX()
		uint8_t GetX();
		// Gets the x-coordinate of this cell.
		// Shortcut for GetCoords().GetX()
		uint8_t GetX() const;

		// Gets the y-coordinate of this cell.
		// Shortcut for GetCoords().GetY()
		uint8_t GetY();
		// Gets the y-coordinate of this cell.
		// Shortcut for GetCoords().GetY()
		uint8_t GetY() const;


		// Sets the WallState for the upper side of this Cell.
		void SetUp(WallState up);
		// Sets the WallState for the lower side of this Cell.
		void SetDown(WallState down);
		// Sets the WallState for the left side of this Cell.
		void SetLeft(WallState left);
		// Sets the WallState for the right side of this Cell.
		void SetRight(WallState right);

		// Returns true if none of the sides of this Cell are set to WallState::Unknown
		bool IsFullyKnown();
		// Returns true if none of the sides of this Cell are set to WallState::Unknown
		bool IsFullyKnown() const;

		CharBlock Serialize();
		CharBlock Serialize() const;
	};

}
#endif
