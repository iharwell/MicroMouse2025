#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\WallSensor.h"
#include "..\MazeMap\WallSensorCalibration.h"

#include <array>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	TEST_CLASS(MazeMapTest)
	{
	public:

		static WallSensor MakeTestWallSensor()
		{
			const std::array<float, 8> adcToLightTable = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f };
			const WallSensor::DistanceModel distanceModel = { 1.0f, 1.0f, 0.05f, 10.0f };
			return WallSensor(
				0U,
				1U,
				Vectorf<2>(0.0f, 0.0f),
				Vectorf<2>(1.0f, 0.0f),
				adcToLightTable,
				distanceModel);
		}

		TEST_METHOD(WallSensorDifferentialLightClampsAtZero)
		{
			Assert::AreEqual(0.0f, WallSensor::DifferentialLightLevel(1.00f, 0.80f), 0.0001f);
		}

		TEST_METHOD(WallSensorDistanceFromLightLevelsUsesExplicitOnOffPair)
		{
			WallSensor sensor = MakeTestWallSensor();
			Assert::AreEqual(2.0f, sensor.DistanceFromLightLevels(0.25f, 0.75f), 0.0001f);
		}

		TEST_METHOD(WallSensorCalibrationCurveOffsetModeUsesSinglePointCorrection)
		{
			WallSensorCalibrationCurve curve;
			Assert::IsTrue(curve.AddPoint(0.080f, 0.050f));
			Assert::AreEqual(0.040f, curve.Apply(0.070f, WallSensorCalibrationMode::DistanceOffset), 0.0001f);
		}

		TEST_METHOD(WallSensorCalibrationCurveInterpolatesFrontDeltaCalibration)
		{
			WallSensorCalibrationCurve curve;
			Assert::IsTrue(curve.AddPoint(0.120f, 0.090f));
			Assert::IsTrue(curve.AddPoint(0.200f, 0.060f));
			Assert::IsTrue(curve.AddPoint(0.320f, 0.030f));
			Assert::AreEqual(0.045f, curve.Apply(0.260f, WallSensorCalibrationMode::DirectInterpolation), 0.0001f);
		}

	};
}
