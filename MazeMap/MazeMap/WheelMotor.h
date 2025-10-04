#pragma once

#include "Defines.h"
#include "Vector2f.h"

namespace MazeMap
{
	class EXPORT WheelMotor
	{
	private:
		float _currentOutput;
		float _desiredOutput;
	public:
		bool SetOutput(float desiredValue);
		float GetCurrentOutput() const;
	};
}

