#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Cell.h"
#ifndef CELL_H
#define CELL_H

#include "defines.h"
#include "Direction.h"
#include "CellCoordinates.h"
#include <stdint.h>
namespace MazeMap
{
	union EXPORT CharBlock
	{
		char chars[4];
		int data;
	};

	enum EXPORT WallState : uint8_t
	{
		// Indicates that we don't know if there's a wall present or not.
		Unknown = 0,
		// Indicates that we know that there isn't a wall present.
		NoWall = 1,
		// Indicates that we know that there is a wall present.
		Wall = 3
	};
	/*
	class EXPORT CellCoordinates
	{
	private:
		uint8_t _x;
		uint8_t _y;

	public:
		CellCoordinates();
		CellCoordinates(uint8_t _x, uint8_t _y);

		// Returns the x component of the coordinate.
		uint8_t GetX() const;
		// Returns the x component of the coordinate.
		uint8_t GetX();

		// Returns the y component of the coordinate.
		uint8_t GetY() const;
		// Returns the y component of the coordinate.
		uint8_t GetY();

		// Returns the coordinates after moving one cell in the provided direction.
		CellCoordinates operator>>(Direction direction);

		// Returns the coordinates after moving one cell up.
		CellCoordinates Up() const;
		// Returns the coordinates after moving one cell down.
		CellCoordinates Down() const;
		// Returns the coordinates after moving one cell left.
		CellCoordinates Left() const;
		// Returns the coordinates after moving one cell right.
		CellCoordinates Right() const;

		// Returns the coordinates after moving one cell up.
		CellCoordinates Up();
		// Returns the coordinates after moving one cell down.
		CellCoordinates Down();
		// Returns the coordinates after moving one cell left.
		CellCoordinates Left();
		// Returns the coordinates after moving one cell right.
		CellCoordinates Right();

		// Returns true if both the x- and y-components are equal between the two coordinates.
		bool operator==(const CellCoordinates& other);
		// Returns true if both the x- and y-components are equal between the two coordinates.
		bool operator==(const CellCoordinates& other) const;

		// Returns true if either the x- or y-components are different between the two coordinates.
		bool operator!=(const CellCoordinates& other);
		// Returns true if either the x- or y-components are different between the two coordinates.
		bool operator!=(const CellCoordinates& other) const;

		// Returns true if the provided coordinates are one cell up from these.
		bool IsUp(CellCoordinates other) const;
		// Returns true if the provided coordinates are one cell down from these.
		bool IsDown(CellCoordinates other) const;
		// Returns true if the provided coordinates are one cell left from these.
		bool IsLeft(CellCoordinates other) const;
		// Returns true if the provided coordinates are one cell right from these.
		bool IsRight(CellCoordinates other) const;

		// Returns true if the provided coordinates are one cell up from these.
		bool IsUp(CellCoordinates other) ;
		// Returns true if the provided coordinates are one cell down from these.
		bool IsDown(CellCoordinates other) ;
		// Returns true if the provided coordinates are one cell left from these.
		bool IsLeft(CellCoordinates other) ;
		// Returns true if the provided coordinates are one cell right from these.
		bool IsRight(CellCoordinates other) ;

		// Returns the direction of the designated cell, with diagonal directions allowed.
		Direction DirectionTo(CellCoordinates other) const;
		// Returns the direction of the designated cell, with diagonal directions allowed.
		Direction DirectionTo(CellCoordinates other);

		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		bool IsValidMove(Direction direction);
		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		bool IsValidMove(Direction direction) const;
	};*/

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
