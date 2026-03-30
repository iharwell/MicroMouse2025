#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\WheelMotor.h"
#pragma once

#include "Defines.h"
#include "Vector2f.h"

namespace MazeMap
{
	class EXPORT WheelMotor
	{
	private:
		float _currentOutput = 0.0f;
		float _desiredOutput = 0.0f;
		const float _gearRatio = 0.0f;
		const float _wheelRadius = 0.0f;
		const float _maxCurrent = 0.0f;
		const float _torqueConstant = 0.0f;
		const float _speedConstant = 0.0f;
		const float _motorMomentOfInertia = 0.0f;
		const float _wheelMomentOfInertia = 0.0f;

		const float _lateralOffset = 0.0f;
		const float _coefficientOfFriction = 0.0f;

		float _downforce = 0.0f;
	public:
		bool SetOutput(float desiredValue);
		float GetCurrentOutput() const;
	};
}

