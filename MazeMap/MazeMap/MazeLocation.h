#pragma once

#include "Defines.h"
#include "Direction.h"
#include "CellCoordinates.h"
namespace MazeMap
{
	class MazeLocation
	{
	private:
		uint8_t _halfX;
		uint8_t _halfY;
	public:
		EXPORT MazeLocation();
		EXPORT MazeLocation(uint8_t halfX, uint8_t halfY);

		static MAZEMAP_INLINE MazeLocation CellCenter(CellCoordinates coords)
		{
			return MazeLocation(
				static_cast<uint8_t>((coords.GetX() << 1) + 1),
				static_cast<uint8_t>((coords.GetY() << 1) + 1)
			);
		}
		static MAZEMAP_INLINE MazeLocation Between(CellCoordinates a, CellCoordinates b)
		{
			return MazeLocation(
				static_cast<uint8_t>(a.GetX() + b.GetX() + 1),
				static_cast<uint8_t>(a.GetY() + b.GetY() + 1)
			);
		}

		// Returns the x component of the coordinate.
		EXPORT uint8_t GetX() const;
		// Returns the x component of the coordinate.
		EXPORT uint8_t GetX();

		// Returns the y component of the coordinate.
		EXPORT uint8_t GetY() const;
		// Returns the y component of the coordinate.
		EXPORT uint8_t GetY();

		// Returns the coordinates after moving one half cell in the provided direction.
		EXPORT MazeLocation operator>>(Direction direction);

		// Returns the coordinates after moving one half cell up.
		EXPORT MazeLocation Up() const;
		// Returns the coordinates after moving one half cell down.
		EXPORT MazeLocation Down() const;
		// Returns the coordinates after moving one half cell left.
		EXPORT MazeLocation Left() const;
		// Returns the coordinates after moving one half cell right.
		EXPORT MazeLocation Right() const;

		// Returns the coordinates after moving one half cell up.
		EXPORT MazeLocation Up();
		// Returns the coordinates after moving one half cell down.
		EXPORT MazeLocation Down();
		// Returns the coordinates after moving one half cell left.
		EXPORT MazeLocation Left();
		// Returns the coordinates after moving one half cell right.
		EXPORT MazeLocation Right();

		// Returns true if both the x- and y-components are equal between the two coordinates.
		EXPORT bool operator==(const MazeLocation& other);
		// Returns true if both the x- and y-components are equal between the two coordinates.
		EXPORT bool operator==(const MazeLocation& other) const;

		// Returns true if either the x- or y-components are different between the two coordinates.
		EXPORT bool operator!=(const MazeLocation& other);
		// Returns true if either the x- or y-components are different between the two coordinates.
		EXPORT bool operator!=(const MazeLocation& other) const;

		// Returns true if the provided coordinates are one cell up from these.
		EXPORT bool IsUp(MazeLocation other) const;
		// Returns true if the provided coordinates are one cell down from these.
		EXPORT bool IsDown(MazeLocation other) const;
		// Returns true if the provided coordinates are one cell left from these.
		EXPORT bool IsLeft(MazeLocation other) const;
		// Returns true if the provided coordinates are one cell right from these.
		EXPORT bool IsRight(MazeLocation other) const;

		// Returns true if the provided coordinates are one cell up from these.
		EXPORT bool IsUp(MazeLocation other);
		// Returns true if the provided coordinates are one cell down from these.
		EXPORT bool IsDown(MazeLocation other);
		// Returns true if the provided coordinates are one cell left from these.
		EXPORT bool IsLeft(MazeLocation other);
		// Returns true if the provided coordinates are one cell right from these.
		EXPORT bool IsRight(MazeLocation other);

		// Returns the direction of the designated cell, with diagonal directions allowed.
		EXPORT Direction DirectionTo(MazeLocation other) const;
		// Returns the direction of the designated cell, with diagonal directions allowed.
		EXPORT Direction DirectionTo(MazeLocation other);

		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		EXPORT bool IsValidMove(Direction direction);
		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		EXPORT bool IsValidMove(Direction direction) const;

		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		EXPORT bool IsValidMove(RelativeDirectionalDistance direction);
		// Returns true if moving one cell in the provided direction results in a coordinate that is in the 16x16 bounds of the maze.
		EXPORT bool IsValidMove(RelativeDirectionalDistance direction) const;

		EXPORT Direction DirectionFromCellCenter();
		EXPORT Direction DirectionFromCellCenter() const;

		EXPORT CellCoordinates GetFirstConnectedCell();
		EXPORT CellCoordinates GetSecondConnectedCell();
		MAZEMAP_INLINE explicit operator CellCoordinates()
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
			return CellCoordinates(static_cast<uint8_t>(x >> 1), static_cast<uint8_t>(y >> 1));
		}
		MAZEMAP_INLINE explicit operator CellCoordinates() const
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
			return CellCoordinates(static_cast<uint8_t>(x >> 1), static_cast<uint8_t>(y >> 1));
		}

		MAZEMAP_INLINE void GetPhysicalLocation(float cellDim, float& xOut, float& yOut)
		{
			xOut = cellDim * 0.5f * GetX();
			yOut = cellDim * 0.5f * GetY();
		}

		MAZEMAP_INLINE bool IntersectsAt(MazeLocation otherLoc, CellCoordinates& intersection)
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









































