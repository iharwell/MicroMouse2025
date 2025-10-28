#include "pch.h"
#include "MaskQueue.h"

namespace MazeMap
{
	MaskQueue::MaskQueue()
		: _data{MazeMask(), MazeMask()}
		, _current(0)
	{ }

	const MazeMask& MaskQueue::GetCurrent() const { return _data[_current]; }
	const MazeMask& MaskQueue::GetCurrent() { return const_cast<const MaskQueue*>(this)->GetCurrent(); }

	bool MaskQueue::GetCurrent(CellCoordinates coords) const { return _data[_current][coords]; }
	bool MaskQueue::GetCurrent(CellCoordinates coords) { return const_cast<const MaskQueue*>(this)->GetCurrent(coords); }
	bool MaskQueue::GetCurrent(uint8_t x, uint8_t y) const { return _data[_current](x,y); }
	bool MaskQueue::GetCurrent(uint8_t x, uint8_t y) { return const_cast<const MaskQueue*>(this)->GetCurrent(x,y); }

	void MaskQueue::Enqueue(CellCoordinates coords) { _data[_current ^ 1].SetFlag(coords, true); }
	void MaskQueue::Enqueue(uint8_t x, uint8_t y) { _data[_current ^ 1].SetFlag(x, y, true); }

	void MaskQueue::Clear()
	{
		_data[0].Clear(false);
		_data[1].Clear(false);
		_current = 0;
	}

	void MaskQueue::SwapQueues()
	{
		_data[_current].Clear(false);
		_current ^= 1;
	}
}
