#line 1 "C:\\Users\\thene\\source\\repos\\MicroMouse2025\\MazeMap\\MazeMap\\Kinematics.h"
#pragma once
#include "defines.h"
#include <math.h>
namespace MazeMap
{
	class EXPORT LinearKinematics
	{
	public:
		static inline float AccelIgnoringDist(float v0, float v1, float t){return (v1 - v0) / t;}
		static inline float AccelIgnoringV0(float dist, float v1, float t){return 2.0f * (v1 / t - dist / (t * t));}
		static inline float AccelIgnoringV1(float dist, float v0, float t){return 2.0f * (dist / (t * t) - v0/t);}
		static inline float AccelIgnoringT(float dist, float v0, float v1){return (v1*v1-v0*v0)/(2*dist);}

		static inline float V0IgnoringDist(float v1, float t, float a){return v1 - a * t;}
		static inline float V0IgnoringV1(float dist, float t, float a){return dist/t - 0.5f * a * t;}
		static inline float V0IgnoringT(float dist, float v1, float a){return sqrtf(v1 * v1 - 2 * a * dist);}
		static inline float V0IgnoringAccel(float dist, float v1, float t){return 2.0f * dist / t - v1;}

		static inline float V1IgnoringDist(float v0, float t, float a){return v0 + a * t;}
		static inline float V1IgnoringV0(float dist, float t, float a){return dist / t + 0.5f * a * t;}
		static inline float V1IgnoringT(float dist, float v0, float a){return sqrtf(v0 * v0 + 2 * a * dist);}
		static inline float V1IgnoringAccel(float dist, float v0, float t){return 2.0f * dist / t - v0;}

		static inline float TIgnoringDist(float v0, float v1, float a){return (v1 - v0) / a;}
		static inline float TIgnoringV0(float dist, float v1, float a){return (v1 - sqrtf(v1 * v1 - 2.0f * a * dist)) / a;}
		static inline float TIgnoringV1(float dist, float v0,float a)
		{
			return (-v0 + sqrtf(v0 * v0 + 2.0f * a * dist)) / a;
		}
		static inline float TIgnoringAccel(float dist, float v0, float v1){return 2.0f * dist / (v0 + v1);}

		static inline float DistIgnoringV0(float v1, float t, float a){return t * (v1 + 0.5f * a * t);}
		static inline float DistIgnoringV1(float v0, float t, float a){return t * (v0 + 0.5f * a * t);}
		static inline float DistIgnoringTime(float v0, float v1, float a){return 0.5f * (v1 * v1 - v0 * v0) / a;}
		static inline float DistIgnoringAccel(float v0, float v1, float t){return 0.5f * (v0 + v1) * t;}
	};

	class EXPORT RotationalKinematics
	{
	public:
		static inline float CentGivenOmegaR(float omega, float r) { return omega*omega*r; }
		static inline float CentGivenVtR(float vt, float r) { return vt * vt / r; }
		static inline float CentGivenThetaTR(float theta, float t, float r) { return (theta*theta) / (t*t*r); }
		static inline float CentGivenOmegaVt(float omega, float vt) { return vt*omega; }

		static inline float RGivenCentVt(float ac, float vt) { return vt*vt/ac; }


		static inline float TGivenOmegaTheta(float omega, float theta) { return theta/omega; }
		static inline float TGivenCentVtTheta(float ac, float vt, float theta) { return vt * theta / ac; }
		static inline float TGivenCentRTheta(float ac, float r, float theta) { return sqrtf(theta*theta*r/ac); }

	};
}
