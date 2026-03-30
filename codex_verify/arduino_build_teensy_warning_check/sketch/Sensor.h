#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Sensor.h"
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