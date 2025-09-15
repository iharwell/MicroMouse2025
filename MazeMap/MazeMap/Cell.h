#ifndef CELL_H
#define CELL_H

#include "defines.h"

namespace MazeMap
{
	enum EXPORT WallState : char
	{
		Unknown = 0,
		NoWall = 1,
		Wall = 2
	};
	enum EXPORT Direction : char
	{
		Up,
		Down,
		Left,
		Right
	};

	class EXPORT Cell
	{
	private:
		int _x, _y;
		WallState _up, _down, _left, _right;

	public:
		Cell();
		Cell(int x, int y);
		Cell(int x, int y, WallState up, WallState down, WallState left, WallState right);

		WallState GetUp();
		WallState GetUp() const;

		WallState GetDown();
		WallState GetDown() const;

		WallState GetLeft();
		WallState GetLeft() const;

		WallState GetRight();
		WallState GetRight() const;

		int GetX();
		int GetX() const;
		int GetY();
		int GetY() const;

		void SetUp(WallState up);
		void SetDown(WallState down);
		void SetLeft(WallState left);
		void SetRight(WallState right);

		bool IsVisited();
	};

}
#endif
