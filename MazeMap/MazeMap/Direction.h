#pragma once
#include "Defines.h"
#include <cstdint>
namespace MazeMap
{


	enum EXPORT Direction : uint8_t
	{
		None = 0x00,
		Up = 0x01,
		Down = 0x02,
		Left = 0x04,
		Right = 0x08,
		UpLeft = Up | Left,
		UpRight = Up | Right,
		DownLeft = Down | Left,
		DownRight = Down | Right
	};

	enum EXPORT RelativeDirection : uint8_t
	{
		L135 = 5,
		L90 =  6,
		L45 =  7,
		F =    0,
		R45  = 1,
		R90  = 2,
		R135 = 3,
		R =    4,

		Left135 = L135,
		Left90  = L90,
		Left45  = L45,
		Right135 = R135,
		Right90  = R90,
		Right45  = R45,
		Forward  = F,
		Reverse  = R,

		NoRelativeDirection = 0
	};
	const Direction OrdinalDirections[] =
	{
		Direction::Up,
		Direction::UpRight,
		Direction::Right,
		Direction::DownRight,
		Direction::Down,
		Direction::DownLeft,
		Direction::Left,
		Direction::UpLeft
	};
	const RelativeDirection RelativeDirections[] =
	{
		RelativeDirection::NoRelativeDirection,
		RelativeDirection::Forward,  // 0b0001
		RelativeDirection::Reverse,  // 0b0010
		RelativeDirection::NoRelativeDirection,     // 0b0011
		RelativeDirection::Left90,   // 0b0100
		RelativeDirection::Left45,   // 0b0101
		RelativeDirection::Left135,  // 0b0110
		RelativeDirection::NoRelativeDirection,     // 0b0111
		RelativeDirection::Right90,  // 0b1000
		RelativeDirection::Right45,  // 0b1001
		RelativeDirection::Right135, // 0b1010
	};

	bool IsDiagonal(Direction dir);

	EXPORT Direction operator<<(Direction dir, int n);
	 EXPORT Direction& operator<<=(Direction& dir, int n);
	 EXPORT Direction operator|(Direction dir, Direction other);
	 EXPORT Direction& operator|=(Direction& dir, Direction other);
	 EXPORT Direction operator&(Direction dir, Direction other);

	 EXPORT Direction operator-(Direction dir);
	 EXPORT RelativeDirection operator-(RelativeDirection dir);

	 EXPORT RelativeDirection operator-(Direction final, Direction initial);
	 EXPORT RelativeDirection operator-(RelativeDirection final, RelativeDirection initial);

	 EXPORT Direction operator+(Direction dir, RelativeDirection rel);
	 EXPORT Direction operator+(RelativeDirection rel, Direction dir);
	 EXPORT RelativeDirection operator+(RelativeDirection a, RelativeDirection b);

	 EXPORT Direction TurnRight45(Direction dir);
	 EXPORT Direction TurnRight90(Direction dir);
	 EXPORT Direction TurnRight135(Direction dir);
	 EXPORT Direction TurnLeft45(Direction dir);
	 EXPORT Direction TurnLeft90(Direction dir);
	 EXPORT Direction TurnLeft135(Direction dir);
	 EXPORT Direction Turn180(Direction dir);
}
