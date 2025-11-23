#include "pch.h"
#include "Direction.h"
namespace MazeMap
{
	bool IsDiagonal(Direction dir)
	{
		return ((dir & 0b1100) > 0) && ((dir & 0b0011) > 0);
	}
	Direction operator<<(Direction dir, int n)
	{
		return static_cast<Direction>(static_cast<uint8_t>(dir) << n);
	}
	Direction& operator<<=(Direction& dir, int n)
	{
		return dir = static_cast<Direction>(static_cast<uint8_t>(dir) << n);
	}
	Direction operator|(Direction dir, Direction other)
	{
		return dir = static_cast<Direction>(static_cast<uint8_t>(dir) | static_cast<uint8_t>(other));
	}
	Direction& operator|=(Direction& dir, Direction other)
	{
		return dir = dir | other;
	}
	Direction operator&(Direction dir, Direction other)
	{
		return dir = static_cast<Direction>(static_cast<uint8_t>(dir) & static_cast<uint8_t>(other));
	}
	Direction operator-(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] + 4) & 0b0111];
	}

	RelativeDirection operator-(RelativeDirection dir)
	{
		return static_cast<RelativeDirection>((-static_cast<int8_t>(dir)) & 0b0111);
	}

	RelativeDirection operator-(Direction final, Direction initial)
	{
		if (final == initial)
		{
			return RelativeDirection::Forward;
		}
		RelativeDirection fRel = RelativeDirections[final];
		RelativeDirection iRel = RelativeDirections[initial];
		return (fRel - iRel);
	}
	RelativeDirection operator-(RelativeDirection final, RelativeDirection initial)
	{
		return static_cast<RelativeDirection>((static_cast<uint8_t>(final) - static_cast<uint8_t>(initial)) & 0b0111);
	}

	Direction operator+(Direction dir, RelativeDirection rel)
	{
		return OrdinalDirections[(RelativeDirections[dir] + rel) & 0b0111];
	}

	Direction operator+(RelativeDirection rel, Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] + rel) & 0b0111];
	}

	RelativeDirection operator+(RelativeDirection a, RelativeDirection b)
	{
		return static_cast<RelativeDirection>((static_cast<uint8_t>(a) + static_cast<uint8_t>(b)) & 0b0111);
	}

	Direction TurnRight45(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] + 1) & 0b0111];
	}
	Direction TurnRight90(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] + 2) & 0b0111];
	}
	Direction TurnRight135(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] + 3) & 0b0111];
	}
	Direction TurnLeft45(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] - 1) & 0b0111];
	}
	Direction TurnLeft90(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] - 2) & 0b0111];
	}
	Direction TurnLeft135(Direction dir)
	{
		return OrdinalDirections[(RelativeDirections[dir] - 3) & 0b0111];
	}
	Direction Turn180(Direction dir) { return -dir; }

#ifdef SMALL_RDD
	RelativeDirectionalDistance::RelativeDirectionalDistance()
		: _data(0)
	{ }
	RelativeDirectionalDistance::RelativeDirectionalDistance(RelativeDirection direction, uint8_t distance)
		: _data((distance<<3)|(direction&7))
	{ }
#else
	RelativeDirectionalDistance::RelativeDirectionalDistance()
		: _distance(0)
		, _direction(Forward)
	{
	}
	RelativeDirectionalDistance::RelativeDirectionalDistance(RelativeDirection direction, uint8_t distance)
		: _distance(distance)
		, _direction(direction)
	{
	}
#endif
}