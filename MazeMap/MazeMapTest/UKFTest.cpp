#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"

#include "..\MazeMap\UKF.h"
#include "..\eigen-5.0.0\Eigen\Eigen"
#include "..\eigen-5.0.0\Eigen\Dense"
#include "..\eigen-5.0.0\Eigen\Cholesky"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(UKFTest)
	{
	public:

		// Px, Py, Vx, Vy
		using StateVec = Eigen::Vector<float, 4>;
		// Px, Py
		using MeasVec = Eigen::Vector<float, 2>;
		// Cx, Cy
		using ControlVec = Eigen::Vector<float, 2>;
		static constexpr float dt = 0.001f;
		static MeasVec MeasModel(StateVec state)
		{
			return MeasVec(state[0], state[1]);
		}
		static StateVec ProcessModel(StateVec state, ControlVec control)
		{
			StateVec next(
				state[0] + dt * state[2] + dt * dt * control[0],
				state[1] + dt * state[3] + dt * dt * control[1],
				state[2] + dt * control[0],
				state[3] + dt * control[1]);
			return next;
		}

		ControlVec Controller(StateVec state, float targetX, float targetY, float& integralX, float& integralY)
		{
			const float p = 0.01;
			const float i = 0.0001;
			const float d = 0.01;

			float ex = targetX - state[0];
			float ey = targetY - state[1];

			float dx = ex / state[2];
			float dy = ex / state[3];

			integralX += ex * i;
			integralY += ey * i;

			return ControlVec(ex * p + dx * d + integralX * i, ey * p + dy * d + integralY * i);
		}

		TEST_METHOD(TestMethod1)
		{
			UKF<4, 2, 2> filter;
			auto x0 = StateVec(0.0f,0.0f,0.0f,0.0f);
			auto P0 = Eigen::Matrix<float, 4, 4>::Identity() * 1.0f;
			auto noiseQ = Eigen::Matrix<float, 4, 4>::Identity() * 0.0001f;
			auto noiseR = Eigen::Matrix<float, 2, 2>::Identity() * 0.001f;

			filter.setNoise(noiseQ, noiseR);
			filter.setState(x0, P0);
			filter.MeasFunction = (&MeasModel);
			filter.ProcessFunction = (&ProcessModel);

			auto control = ControlVec(0, 0);
			auto measurements = MeasVec(0, 0);
			for (size_t i = 0; i < 100; i++)
			{
				filter.Predict(control);
				filter.Update(control);
			}
			int k = sizeof(filter);
			return;
		}
	};
}