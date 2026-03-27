#pragma once

#include "Defines.h"

#include <cmath>

namespace MazeMap
{
	struct SmoothTurnYawRateControllerState
	{
		float previousErrorRadps = 0.0f;
		bool hasPreviousError = false;

		MAZEMAP_INLINE void Reset() noexcept
		{
			previousErrorRadps = 0.0f;
			hasPreviousError = false;
		}
	};

	MAZEMAP_INLINE float ComputeSmoothTurnYawRatePdCorrection(
		float desiredYawRateRadps,
		float measuredYawRateRadps,
		float dtSeconds,
		float kp,
		float kd,
		SmoothTurnYawRateControllerState& state) noexcept
	{
		if (!std::isfinite(desiredYawRateRadps) ||
			!std::isfinite(measuredYawRateRadps) ||
			!std::isfinite(dtSeconds) ||
			!std::isfinite(kp) ||
			!std::isfinite(kd))
		{
			state.Reset();
			return 0.0f;
		}

		const float errorRadps = desiredYawRateRadps - measuredYawRateRadps;
		float errorDerivativeRadps2 = 0.0f;
		if (state.hasPreviousError && (dtSeconds > 1.0e-6f))
		{
			errorDerivativeRadps2 = (errorRadps - state.previousErrorRadps) / dtSeconds;
		}

		state.previousErrorRadps = errorRadps;
		state.hasPreviousError = true;
		return (kp * errorRadps) + (kd * errorDerivativeRadps2);
	}
}
