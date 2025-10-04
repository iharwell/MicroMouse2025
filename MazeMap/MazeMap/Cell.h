#ifndef CELL_H
#define CELL_H

#include "defines.h"
#include <stdint.h>
namespace MazeMap
{
	union EXPORT CharBlock
	{
		char chars[4];
		int data;
	};

	enum EXPORT WallState : char
	{
		Unknown = 0,
		NoWall = 1,
		Wall = 3
	};
	enum EXPORT Direction : char
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

	class EXPORT CellCoordinates
	{
	private:
		uint8_t _x;
		uint8_t _y;

	public:
		CellCoordinates();
		CellCoordinates(uint8_t _x, uint8_t _y);

		uint8_t GetX() const;
		uint8_t GetY() const;
		uint8_t GetX();
		uint8_t GetY();

		CellCoordinates Up() const;
		CellCoordinates Down() const;
		CellCoordinates Left() const;
		CellCoordinates Right() const;

		CellCoordinates Up();
		CellCoordinates Down();
		CellCoordinates Left();
		CellCoordinates Right();

		bool operator==(const CellCoordinates& other);
		bool operator==(const CellCoordinates& other) const;

		bool IsUp(CellCoordinates other) const;
		bool IsDown(CellCoordinates other) const;
		bool IsLeft(CellCoordinates other) const;
		bool IsRight(CellCoordinates other) const;

		bool IsUp(CellCoordinates other) ;
		bool IsDown(CellCoordinates other) ;
		bool IsLeft(CellCoordinates other) ;
		bool IsRight(CellCoordinates other) ;

		Direction DirectionTo(CellCoordinates other) const;
		Direction DirectionTo(CellCoordinates other);
	};

	class EXPORT Cell
	{
	private:
		CellCoordinates _coords;
		unsigned char _data;
	public:
		Cell();
		Cell(uint8_t x, uint8_t y);
		Cell(uint8_t x, uint8_t y, WallState up, WallState down, WallState left, WallState right);

		WallState GetUp();
		WallState GetUp() const;

		WallState GetDown();
		WallState GetDown() const;

		WallState GetLeft();
		WallState GetLeft() const;

		WallState GetRight();
		WallState GetRight() const;

		CellCoordinates GetCoords();
		CellCoordinates GetCoords() const;

		uint8_t GetX();
		uint8_t GetX() const;

		uint8_t GetY();
		uint8_t GetY() const;

		void SetUp(WallState up);
		void SetDown(WallState down);
		void SetLeft(WallState left);
		void SetRight(WallState right);

		bool IsVisited();
		bool IsVisited() const;

		CharBlock Serialize();
		CharBlock Serialize() const;
	};

}
#endif
