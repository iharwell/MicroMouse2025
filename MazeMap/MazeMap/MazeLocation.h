#pragma once

#include "Defines.h"
#include "Direction.h"
#include "CellCoordinates.h"
namespace MazeMap
{
	class EXPORT MazeLocation
	{
	private:
		uint8_t _halfX;
		uint8_t _halfY;
	public:
		MazeLocation();
		MazeLocation(uint8_t halfX, uint8_t halfY);

		static MazeLocation CellCenter(CellCoordinates coords) { return MazeLocation((coords.GetX() << 1) + 1, (coords.GetY() << 1) + 1); }
		static MazeLocation Between(CellCoordinates a, CellCoordinates b) { return MazeLocation((a.GetX() + b.GetX() + 1), (a.GetY() + b.GetY() + 1)); }

		// Returns the x component of the coordinate.
		uint8_t GetX() const;
		// Returns the x component of the coordinate.
		uint8_t GetX();

		// Returns the y component of the coordinate.
		uint8_t GetY() const;
		// Returns the y component of the coordinate.
		uint8_t GetY();

		// Returns the coordinates after moving one half cell in the provided direction.
		MazeLocation operator>>(Direction direction);

		// Returns the coordinates after moving one half cell up.
		MazeLocation Up() const;
		// Returns the coordinates after moving one half cell down.
		MazeLocation Down() const;
		// Returns the coordinates after moving one half cell left.
		MazeLocation Left() const;
		// Returns the coordinates after moving one half cell right.
		MazeLocation Right() const;

		// Returns the coordinates after moving one half cell up.
		MazeLocation Up();
		// Returns the coordinates after moving one half cell down.
		MazeLocation Down();
		// Returns the coordinates after moving one half cell left.
		MazeLocation Left();
		// Returns the coordinates after moving one half cell right.
		MazeLocation Right();

		// Returns true if both the x- and y-components are equal between the two coordinates.
		bool operator==(const MazeLocation& other);
		// Returns true if both the x- and y-components are equal between the two coordinates.
		bool operator==(const MazeLocation& other) const;

		// Returns true if either the x- or y-components are different between the two coordinates.
		bool operator!=(const MazeLocation& other);
		// Returns true if either the x- or y-components are different between the two coordinates.
		bool operator!=(const MazeLocation& other) const;

		// Returns true if the provided coordinates are one cell up from these.
		bool IsUp(MazeLocation other) const;
		// Returns true if the provided coordinates are one cell down from these.
		bool IsDown(MazeLocation other) const;
		// Returns true if the provided coordinates are one cell left from these.
		bool IsLeft(MazeLocation other) const;
		// Returns true if the provided coordinates are one cell right from these.
		bool IsRight(MazeLocation other) const;

		// Returns true if the provided coordinates are one cell up from these.
		bool IsUp(MazeLocation other);
		// Returns true if the provided coordinates are one cell down from these.
		bool IsDown(MazeLocation other);
		// Returns true if the provided coordinates are one cell left from these.
		bool IsLeft(MazeLocation other);
		// Returns true if the provided coordinates are one cell right from these.
		bool IsRight(MazeLocation other);

		// Returns the direction of the designated cell, with diagonal directions allowed.
		Direction DirectionTo(MazeLocation other) const;
		// Returns the direction of the designated cell, with diagonal directions allowed.
		Direction DirectionTo(MazeLocation other);

		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		bool IsValidMove(Direction direction);
		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		bool IsValidMove(Direction direction) const;

		Direction DirectionFromCellCenter();
		Direction DirectionFromCellCenter() const;

		CellCoordinates GetFirstConnectedCell();
		CellCoordinates GetSecondConnectedCell();
		explicit operator CellCoordinates()
		{
			uint8_t x = GetX();
			if (x >= 32)
			{
				--x;
			}

			uint8_t y = GetY();
			if (y >= 32)
			{
				--y;
			}
			return CellCoordinates(x >> 1, y >> 1);
		}

		bool IntersectsAt(MazeLocation otherLoc, CellCoordinates& intersection)
		{
			CellCoordinates f0 = GetFirstConnectedCell();
			CellCoordinates s0 = GetSecondConnectedCell();
			CellCoordinates f1 = otherLoc.GetFirstConnectedCell();
			CellCoordinates s1 = otherLoc.GetSecondConnectedCell();

			if (f0 == f1)
			{
				intersection = f0;
				return true;
			}
			if (f0 == s1)
			{
				intersection = f0;
				return true;
			}
			if (s0 == f1)
			{
				intersection = s0;
				return true;
			}
			if (s0 == s1)
			{
				intersection = s0;
				return true;
			}
			return false;
		}
	};
}
