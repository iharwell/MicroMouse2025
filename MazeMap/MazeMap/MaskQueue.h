#pragma once
#include "MazeMask.h"
namespace MazeMap
{
	class EXPORT MaskQueue
	{
	private:
		MazeMask _data[2];
		uint8_t _current;
	public:
		MaskQueue();

		const MazeMask& GetCurrent();
		const MazeMask& GetCurrent() const;
		bool GetCurrent(CellCoordinates coords);
		bool GetCurrent(CellCoordinates coords) const;
		bool GetCurrent(uint8_t x, uint8_t y);
		bool GetCurrent(uint8_t x, uint8_t y) const;

		void Enqueue(CellCoordinates coords);
		void Enqueue(uint8_t x, uint8_t y);

		void Clear();

		void SwapQueues();
	};
	class EXPORT DoubleMaskQueue
	{
	private:
		DoubleMazeMask _data[2];
		uint8_t _current;
	public:
		DoubleMaskQueue();

		const DoubleMazeMask& GetCurrent();
		const DoubleMazeMask& GetCurrent() const;
		bool GetCurrent(MazeLocation coords);
		bool GetCurrent(MazeLocation coords) const;
		bool GetCurrent(uint8_t x, uint8_t y);
		bool GetCurrent(uint8_t x, uint8_t y) const;

		void Enqueue(MazeLocation coords);
		void Enqueue(uint8_t x, uint8_t y);

		void Clear();

		void SwapQueues();
	};
}
