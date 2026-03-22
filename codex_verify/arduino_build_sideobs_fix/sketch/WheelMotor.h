#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\WheelMotor.h"
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
		const float _gearRatio;
		const float _wheelRadius;
		const float _maxCurrent;
		const float _torqueConstant;
		const float _speedConstant;
		const float _motorMomentOfInertia;
		const float _wheelMomentOfInertia;

		const float _lateralOffset;
		const float _coefficientOfFriction;

		float _downforce;
	public:
		bool SetOutput(float desiredValue);
		float GetCurrentOutput() const;
	};
}

