#ifndef SENSOR_H
#define SENSOR_H

#include "Defines.h"
#include "CircularBuffer.h"

namespace MazeMap
{
	template<typename T>
	class EXPORT Sensor
	{
	private:
		TimedCircularBuffer<T, 50> _buffer;
	public:


	};
}

#endif