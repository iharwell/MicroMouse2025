#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\MazeMask.h"
#pragma once

#include "defines.h"
#include "MazeLocation.h"
#include "Cell.h"
namespace MazeMap
{
	enum PerimeterType
	{
		Inner=0x00,
		Outer=0x01,
		IncludeMazeBoundary=0x02,
	};

	union EXPORT MazeBits
	{
		uint16_t Shorts[16];
		uint32_t Ints[8];
		uint64_t Longs[4];
	};

	/// <summary>
	/// Provides a very compact data structure for holding boolean masks of the maze. Inspired by std::bitvector.
	/// </summary>
	class EXPORT MazeMask
	{
	private:
		MazeBits _data;
	public:
		MazeMask();
		MazeMask(bool initialValue);
		MazeMask(uint16_t initialValues[]);

		bool SetFlag(CellCoordinates coords, bool value);
		bool SetFlag(uint8_t x, uint8_t y, bool value);
		bool GetFlag(uint8_t x, uint8_t y);
		bool GetFlag(uint8_t x, uint8_t y) const;

		bool operator()(uint8_t x, uint8_t y);
		bool operator()(uint8_t x, uint8_t y) const;

		bool operator[](CellCoordinates coords);
		bool operator[](CellCoordinates coords) const;

		MazeMask GetPerimeter(PerimeterType perimeterType);
		MazeMask GetPerimeter(PerimeterType perimeterType) const;

		uint16_t GetRow(uint8_t row);
		uint16_t GetRow(uint8_t row) const;
		void SetRow(uint8_t row, uint16_t data);

		void Clear(bool value);
		bool AnyFlags() const;
		bool AnyFlags();
	};

	union EXPORT DoubleMazeBits
	{
		uint32_t Ints[32];
		uint64_t Longs[16];
	};

	/// <summary>
	/// Provides a very compact data structure for holding boolean masks of the maze. Inspired by std::bitvector.
	/// </summary>
	class EXPORT DoubleMazeMask
	{
	private:
		DoubleMazeBits _data;
	public:
		DoubleMazeMask();
		DoubleMazeMask(bool initialValue);
		DoubleMazeMask(uint32_t initialValues[]);

		bool SetFlag(MazeLocation coords, bool value);
		bool SetFlag(uint8_t x, uint8_t y, bool value);
		bool GetFlag(uint8_t x, uint8_t y);
		bool GetFlag(uint8_t x, uint8_t y) const;

		bool operator()(uint8_t x, uint8_t y);
		bool operator()(uint8_t x, uint8_t y) const;

		bool operator[](MazeLocation coords);
		bool operator[](MazeLocation coords) const;

		DoubleMazeMask GetPerimeter(PerimeterType perimeterType);
		DoubleMazeMask GetPerimeter(PerimeterType perimeterType) const;

		uint32_t GetRow(uint8_t row);
		uint32_t GetRow(uint8_t row) const;
		void SetRow(uint8_t row, uint32_t data);

		void Clear(bool value);
		bool AnyFlags() const;
		bool AnyFlags();
	};
}
