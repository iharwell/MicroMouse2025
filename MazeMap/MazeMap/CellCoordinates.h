#pragma once
#include "Defines.h"
#include "Direction.h"

namespace MazeMap
{

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
		// Returns the coordinates after moving one cell in the provided direction.
		CellCoordinates operator>>(Direction direction) const;

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
		bool IsUp(CellCoordinates other);
		// Returns true if the provided coordinates are one cell down from these.
		bool IsDown(CellCoordinates other);
		// Returns true if the provided coordinates are one cell left from these.
		bool IsLeft(CellCoordinates other);
		// Returns true if the provided coordinates are one cell right from these.
		bool IsRight(CellCoordinates other);

		// Returns the direction of the designated cell, with diagonal directions allowed.
		Direction DirectionTo(CellCoordinates other) const;
		// Returns the direction of the designated cell, with diagonal directions allowed.
		Direction DirectionTo(CellCoordinates other);

		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		bool IsValidMove(Direction direction);
		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		bool IsValidMove(Direction direction) const;
	};
}
