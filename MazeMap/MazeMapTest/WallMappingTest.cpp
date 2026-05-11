#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\MapEvidenceUpdater.h"
#include "..\MazeMap\WallGeometryModel.h"
#include "..\MazeMap\WallSensorPreprocessor.h"
#include "..\MazeMap\WallBeliefMap.h"
#include "..\MazeMap\WallObservationPipeline.h"
#include "..\MazeMap\WallSensor.h"
#include "..\MazeMap\WallSensorCalibration.h"

#include <array>
#include <cmath>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace MazeMap
{
	static Eigen::Matrix2f RotationBodyFromYaw(const float yawOffsetRad)
	{
		const float s = std::sin(yawOffsetRad);
		const float c = std::cos(yawOffsetRad);

		Eigen::Matrix2f rotation = Eigen::Matrix2f::Identity();
		rotation(0, 0) = c;
		rotation(0, 1) = s;
		rotation(1, 0) = -s;
		rotation(1, 1) = c;
		return rotation;
	}

	static SensorMount RotateMountYaw(const SensorMount& mount, const float yawOffsetRad)
	{
		return SensorMount(
			mount.positionBodyM(),
			mount.bodyFromSensor() * RotationBodyFromYaw(yawOffsetRad),
			mount.clockwiseYawSign());
	}

	static WallGeometryModel::GeometryStateFrame BuildGeometryFrame(
		const WallGeometryModel& geometry,
		const VehicleState::StateVector& state)
	{
		return geometry.buildStateFrame(
			Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)),
			state(VehicleState::kPsi));
	}

	TEST_CLASS(WallMappingTest)
	{
	public:

		static WallSensor MakeTestWallSensor()
		{
			const std::array<float, 8> adcToLightTable = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f };
			const WallSensor::DistanceModel distanceModel = { 1.0f, 1.0f, 0.05f, 10.0f };
			return WallSensor(
				0U,
				1U,
				Eigen::Vector2f(0.0f, 0.0f),
				Eigen::Vector2f(1.0f, 0.0f),
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

		TEST_METHOD(WallDecisionAccumulatorRequiresCommittedEvidence)
		{
			WallDecisionAccumulator accumulator;
			accumulator.Update(WallSampleClassification::WallHit, 0.55f, 0.55f, 0.08f);
			accumulator.Update(WallSampleClassification::Unknown, 0.55f, 0.55f, 0.08f);
			Assert::AreEqual(
				static_cast<int>(WallSampleClassification::Unknown),
				static_cast<int>(accumulator.FinalClassification(1.0f)));

			accumulator.Update(WallSampleClassification::WallHit, 0.55f, 0.55f, 0.08f);
			Assert::AreEqual(
				static_cast<int>(WallSampleClassification::WallHit),
				static_cast<int>(accumulator.FinalClassification(1.0f)));
		}

		TEST_METHOD(WallDecisionAccumulatorTransitionImpulseSupportsOpeningMiss)
		{
			WallDecisionAccumulator accumulator;
			accumulator.Update(WallSampleClassification::Unknown, 0.55f, 0.55f, 0.08f);
			accumulator.InjectMissImpulse(0.35f);
			accumulator.Update(WallSampleClassification::WallMiss, 0.55f, 0.55f, 0.08f);
			accumulator.Update(WallSampleClassification::WallMiss, 0.55f, 0.55f, 0.08f);

			Assert::AreEqual(
				static_cast<int>(WallSampleClassification::WallMiss),
				static_cast<int>(accumulator.FinalClassification(1.0f)));
		}

		TEST_METHOD(WallBeliefMapConfirmsUnknownWallAndMirrorsNeighbor)
		{
			WallBeliefMap beliefs;
			WallBeliefMap::Config config{};
			const CellCoordinates cell(4U, 4U);
			const WallBeliefMap::Update update = beliefs.ApplyObservation(
				cell,
				Direction::Up,
				WallSampleClassification::WallHit,
				config,
				12U);

			Assert::IsTrue(update.valid);
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(update.hardState));
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(beliefs.Get(cell, Direction::Up).hardState));
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(beliefs.Get(cell >> Direction::Up, Direction::Down).hardState));
		}

		TEST_METHOD(WallBeliefMapSingleContradictoryMissDoesNotClearConfirmedWall)
		{
			WallBeliefMap beliefs;
			WallBeliefMap::Config config{};
			const CellCoordinates cell(2U, 2U);
			beliefs.SeedKnownState(cell, Direction::Right, WallState::Wall, config, 1U);

			const WallBeliefMap::Update update = beliefs.ApplyObservation(
				cell,
				Direction::Right,
				WallSampleClassification::WallMiss,
				config,
				2U);

			Assert::IsTrue(update.valid);
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(update.hardState));
			Assert::IsTrue(update.contradictionCount >= 1U);
			Assert::IsTrue(update.logOdds < config.setThreshold);
		}

		TEST_METHOD(WallSensorPreprocessorClassifiesWallPostAndOpenObservations)
		{
			WallSensor sensor = MakeTestWallSensor();
			WallSensorPreprocessor preprocessor;

			WallSensorPreprocessor::Input wallInput{};
			wallInput.ledOffLevel = 0.1f;
			wallInput.ledOnLevel = 10.1f;
			wallInput.supportSpanM = 0.06f;
			const WallObs wallObservation = preprocessor.process(sensor, wallInput);
			Assert::IsTrue(wallObservation.valid);
			Assert::AreEqual(static_cast<int>(ObsClass::WallLike), static_cast<int>(wallObservation.cls));

			WallSensorPreprocessor::Input postInput = wallInput;
			postInput.supportSpanM = 0.01f;
			const WallObs postObservation = preprocessor.process(sensor, postInput);
			Assert::IsTrue(postObservation.valid);
			Assert::AreEqual(static_cast<int>(ObsClass::PostLike), static_cast<int>(postObservation.cls));

			WallSensorPreprocessor::Input openInput{};
			openInput.ledOffLevel = 0.1f;
			openInput.ledOnLevel = 6.1f;
			openInput.supportSpanM = 0.06f;
			const WallObs openObservation = preprocessor.process(sensor, openInput);
			Assert::IsTrue(openObservation.valid);
			Assert::AreEqual(static_cast<int>(ObsClass::OpenLike), static_cast<int>(openObservation.cls));
		}

		TEST_METHOD(WallObservationPipelineBuildsCanonicalWallObservations)
		{
			WallObs frontLeft{};
			WallObs frontRight{};
			BuildFrontWallObservations(
				true,
				true,
				false,
				true,
				0.18f,
				0.21f,
				0.25f,
				frontLeft,
				frontRight);

			Assert::IsTrue(frontLeft.valid);
			Assert::IsTrue(frontRight.valid);
			Assert::AreEqual(0.18f, frontLeft.rho, 1.0e-6f);
			Assert::AreEqual(0.21f, frontRight.rho, 1.0e-6f);
			Assert::AreEqual(0.90f, frontLeft.confidence, 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::WallLike), static_cast<int>(frontLeft.cls));

			const WallObs sideObservation = BuildSideWallObservation(true, false, true, 0.14f, 0.25f);
			Assert::IsTrue(sideObservation.valid);
			Assert::AreEqual(0.14f, sideObservation.rho, 1.0e-6f);
			Assert::AreEqual(0.80f, sideObservation.confidence, 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::WallLike), static_cast<int>(sideObservation.cls));
		}

		TEST_METHOD(WallGeometryModelRespectsSensorMountForFrontWallPrediction)
		{
			Maze maze;
			maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);

			WallGeometryModel geometry;
			PlantParams params = PlantParams::Default();

			VehicleState::StateVector state = VehicleState::StateVector::Zero();
			state(VehicleState::kPx) = 0.09f;
			state(VehicleState::kPy) = 0.09f;
			state(VehicleState::kPsi) = 0.0f;

			const WallGeometryModel::GeometryStateFrame frame = BuildGeometryFrame(geometry, state);
			GeometryPrediction baseline = geometry.predictRay(frame, params.frontLeftSensor, maze);
			Assert::IsTrue(baseline.hit);
			Assert::AreEqual(static_cast<int>(GeometryHitType::WallFace), static_cast<int>(baseline.type));

			const SensorMount rotatedSensor = RotateMountYaw(params.frontLeftSensor, 0.35f);
			GeometryPrediction rotated = geometry.predictRay(frame, rotatedSensor, maze);
			Assert::IsTrue(rotated.rangeM > baseline.rangeM);
		}

		TEST_METHOD(WallGeometryModelDefaultFrameUsesProjectForwardHeading)
		{
			const WallGeometryModel::GeometryStateFrame frame{};
			Assert::AreEqual(0.0f, frame.heading.x(), 1.0e-6f);
			Assert::AreEqual(1.0f, frame.heading.y(), 1.0e-6f);
		}

		TEST_METHOD(WallGeometryModelCanIdentifyPostHits)
		{
			Maze maze;

			WallGeometryModel geometry;
			const SensorMount sensor = RotateMountYaw(
				SensorMount::FromForwardDirectionBody(
					Eigen::Vector2f(0.0f, 0.0f),
					Eigen::Vector2f(0.0f, 1.0f)),
				PI_F / 4.0f);

			VehicleState::StateVector state = VehicleState::StateVector::Zero();
			state(VehicleState::kPx) = 0.09f;
			state(VehicleState::kPy) = 0.09f;
			state(VehicleState::kPsi) = 0.0f;

			const GeometryPrediction prediction = geometry.predictRay(BuildGeometryFrame(geometry, state), sensor, maze);
			Assert::IsTrue(prediction.hit);
			Assert::AreEqual(static_cast<int>(GeometryHitType::Post), static_cast<int>(prediction.type));
		}

		TEST_METHOD(MapEvidenceUpdaterPostLikeObservationDoesNotForceWall)
		{
			MapEvidenceUpdater updater;
			MapEvidenceUpdater::Config config{};

			WallObs observation{};
			observation.valid = true;
			observation.confidence = 0.9f;
			observation.cls = ObsClass::PostLike;
			observation.rho = 0.05f;

			GeometryPrediction bestFit{};
			bestFit.hit = true;
			bestFit.type = GeometryHitType::Post;

			const bool changed = updater.Apply(
				CellCoordinates(1U, 1U),
				Direction::Right,
				observation,
				bestFit,
				config,
				false);

			Assert::IsFalse(changed);
			Assert::AreEqual(static_cast<int>(WallState::Unknown), static_cast<int>(updater.Get(CellCoordinates(1U, 1U), Direction::Right).state));
		}

		TEST_METHOD(MapEvidenceUpdaterCommitsWallAndOpenEvidence)
		{
			MapEvidenceUpdater updater;
			MapEvidenceUpdater::Config config{};

			WallObs wallObservation{};
			wallObservation.valid = true;
			wallObservation.confidence = 0.9f;
			wallObservation.cls = ObsClass::WallLike;

			GeometryPrediction wallFit{};
			wallFit.hit = true;
			wallFit.type = GeometryHitType::WallFace;
			wallFit.cell = CellCoordinates(3U, 3U);
			wallFit.edge = Direction::Up;

			for (int i = 0; i < 3; ++i)
			{
				Assert::IsTrue(updater.Apply(CellCoordinates(3U, 3U), Direction::Up, wallObservation, wallFit, config, false));
			}
			Assert::AreEqual(static_cast<int>(WallState::Wall), static_cast<int>(updater.Get(CellCoordinates(3U, 3U), Direction::Up).state));
			Assert::AreEqual(3, static_cast<int>(updater.Get(CellCoordinates(3U, 3U), Direction::Up).score));
			Assert::AreEqual(
				static_cast<int>(WallState::Wall),
				static_cast<int>(updater.Get(CellCoordinates(3U, 3U) >> Direction::Up, Direction::Down).state));
			Assert::AreEqual(3, static_cast<int>(updater.Get(CellCoordinates(3U, 3U) >> Direction::Up, Direction::Down).score));

			MapEvidenceUpdater openUpdater;
			WallObs openObservation{};
			openObservation.valid = true;
			openObservation.confidence = 0.9f;
			openObservation.cls = ObsClass::OpenLike;

			GeometryPrediction openFit{};
			openFit.hit = false;
			openFit.type = GeometryHitType::None;

			for (int i = 0; i < 3; ++i)
			{
				Assert::IsTrue(openUpdater.Apply(CellCoordinates(5U, 5U), Direction::Right, openObservation, openFit, config, false));
			}
			Assert::AreEqual(static_cast<int>(WallState::NoWall), static_cast<int>(openUpdater.Get(CellCoordinates(5U, 5U), Direction::Right).state));
			Assert::AreEqual(-3, static_cast<int>(openUpdater.Get(CellCoordinates(5U, 5U), Direction::Right).score));
			Assert::AreEqual(
				static_cast<int>(WallState::NoWall),
				static_cast<int>(openUpdater.Get(CellCoordinates(5U, 5U) >> Direction::Right, Direction::Left).state));
			Assert::AreEqual(-3, static_cast<int>(openUpdater.Get(CellCoordinates(5U, 5U) >> Direction::Right, Direction::Left).score));
		}

	};
}


