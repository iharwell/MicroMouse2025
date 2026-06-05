#include "pch.h"
#include "CppUnitTest.h"
#include "Templates.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\MapEvidenceUpdater.h"
#include "..\MazeMap\SensorSnapshot.h"
#include "..\MazeMap\WallGeometryModel.h"
#include "..\MazeMap\WallSensorPreprocessor.h"
#include "..\MazeMap\WallBeliefMap.h"
#include "..\MazeMap\WallObservationPipeline.h"
#include "..\MazeMap\WallSensor.h"
#include "..\MazeMap\WallSensorCalibration.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"

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
		const VehicleState& state)
	{
		return geometry.buildStateFrame(
			state.GetPosition(),
			state.GetHeading());
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

		TEST_METHOD(BuildEvidenceObservationSnapshotRequiresCommittedEvidence)
		{
			SensorSnapshot firstPass[2]{};
			firstPass[0].SetLeftWallObservationWindowValid(true);
			firstPass[0].SetLeftWallObservation(true);
			SensorSnapshot combined{};
			Assert::IsTrue(combined.BuildEvidenceObservationSnapshot(firstPass, 2U));
			Assert::IsFalse(combined.LeftWallObservationWindowValid());

			SensorSnapshot secondPass[3]{};
			secondPass[0] = firstPass[0];
			secondPass[2].SetLeftWallObservationWindowValid(true);
			secondPass[2].SetLeftWallObservation(true);
			Assert::IsTrue(combined.BuildEvidenceObservationSnapshot(secondPass, 3U));
			Assert::IsTrue(combined.LeftWallObservationWindowValid());
			Assert::IsTrue(combined.HasLeftWallObservation());
		}

		TEST_METHOD(BuildEvidenceObservationSnapshotTransitionImpulseSupportsOpeningMiss)
		{
			SensorSnapshot samples[3]{};
			samples[0].SetLeftTransitionDetected(true);
			samples[1].SetLeftWallObservationWindowValid(true);
			samples[1].SetLeftWallObservation(false);
			samples[2].SetLeftWallObservationWindowValid(true);
			samples[2].SetLeftWallObservation(false);

			SensorSnapshot combined{};
			Assert::IsTrue(combined.BuildEvidenceObservationSnapshot(samples, 3U));
			Assert::IsTrue(combined.LeftWallObservationWindowValid());
			Assert::IsFalse(combined.HasLeftWallObservation());
			Assert::IsTrue(combined.LeftTransitionDetected());
		}

		TEST_METHOD(BuildEvidenceObservationSnapshotSkipsInvalidFrontSamples)
		{
			SensorSnapshot samples[3]{};
			samples[0].SetFrontWall(false);
			samples[1].SetFrontWall(false);
			samples[2].SetFrontWallObservationValid(true);
			samples[2].SetFrontWall(true);
			samples[2].SetFrontLeftWall(true);
			samples[2].SetFrontRightWall(true);

			SensorSnapshot combined{};
			Assert::IsTrue(combined.BuildEvidenceObservationSnapshot(samples, 3U));
			Assert::IsFalse(combined.FrontWallObservationValid());

			samples[1].SetFrontWallObservationValid(true);
			samples[1].SetFrontWall(true);
			samples[1].SetFrontLeftWall(true);
			samples[1].SetFrontRightWall(true);
			Assert::IsTrue(combined.BuildEvidenceObservationSnapshot(samples, 3U));
			Assert::IsTrue(combined.FrontWallObservationValid());
			Assert::IsTrue(combined.HasFrontWall());
			Assert::IsTrue(combined.HasFrontLeftWall());
			Assert::IsTrue(combined.HasFrontRightWall());
		}

		TEST_METHOD(BuildEvidenceObservationSnapshotAveragesRawBiasAndCorrectedYawRates)
		{
			SensorSnapshot samples[3]{};
			samples[0].SetRawYawRateRadps(0.30f);
			samples[0].SetYawRateBiasRadps(0.10f);
			samples[0].SetYawRateRadps(0.20f);
			samples[1].SetRawYawRateRadps(0.36f);
			samples[1].SetYawRateBiasRadps(0.12f);
			samples[1].SetYawRateRadps(0.24f);
			samples[2].SetRawYawRateRadps(0.42f);
			samples[2].SetYawRateBiasRadps(0.14f);
			samples[2].SetYawRateRadps(0.28f);

			SensorSnapshot combined{};
			Assert::IsTrue(combined.BuildEvidenceObservationSnapshot(samples, 3U));

			Assert::AreEqual(0.36f, combined.RawYawRateRadps(), 1.0e-6f);
			Assert::AreEqual(0.12f, combined.YawRateBiasRadps(), 1.0e-6f);
			Assert::AreEqual(0.24f, combined.YawRateRadps(), 1.0e-6f);
			Assert::AreEqual(
				combined.RawYawRateRadps() - combined.YawRateBiasRadps(),
				combined.YawRateRadps(),
				1.0e-6f);
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

			const WallObs wallObservation = preprocessor.process(
				sensor,
				0.1f,
				10.1f,
				sensor.DistanceFromLightLevels(0.1f, 10.1f),
				0.06f);
			Assert::IsTrue(wallObservation.IsValid());
			Assert::AreEqual(static_cast<int>(ObsClass::WallLike), static_cast<int>(wallObservation.Class()));
			Assert::IsTrue(wallObservation.MeasurementNoiseSigmaM() > 0.0f);

			const WallObs postObservation = preprocessor.process(
				sensor,
				0.1f,
				10.1f,
				sensor.DistanceFromLightLevels(0.1f, 10.1f),
				0.01f);
			Assert::IsTrue(postObservation.IsValid());
			Assert::AreEqual(static_cast<int>(ObsClass::PostLike), static_cast<int>(postObservation.Class()));

			const WallObs openObservation = preprocessor.process(
				sensor,
				0.1f,
				6.1f,
				sensor.DistanceFromLightLevels(0.1f, 6.1f),
				0.06f);
			Assert::IsTrue(openObservation.IsValid());
			Assert::AreEqual(static_cast<int>(ObsClass::OpenLike), static_cast<int>(openObservation.Class()));
		}

		TEST_METHOD(WallSensorPreprocessorPreservesFarReturnConfidenceClassAndNoise)
		{
			WallSensor sensor = MakeTestWallSensor();
			WallSensorPreprocessor preprocessor;

			const WallObs farObservation = preprocessor.process(
				sensor,
				0.1f,
				2.1f,
				sensor.DistanceFromLightLevels(0.1f, 2.1f),
				0.06f);
			Assert::IsTrue(farObservation.IsValid());
			Assert::IsTrue(farObservation.Rho() > 0.25f);
			Assert::AreEqual(static_cast<int>(ObsClass::OpenLike), static_cast<int>(farObservation.Class()));
			Assert::IsTrue(farObservation.MeasurementNoiseSigmaM() > 0.0f);

			const WallObs sideObservation = WallObs::BuildSideWallObservation(
				true,
				false,
				false,
				farObservation.Rho(),
				kDefaultWallObservationMaxRangeM,
				farObservation.MeasurementNoiseSigmaM(),
				farObservation.Confidence(),
				farObservation.Class());
			Assert::IsTrue(sideObservation.IsValid());
			Assert::AreEqual(farObservation.Rho(), sideObservation.Rho(), 1.0e-6f);
			Assert::AreEqual(farObservation.Confidence(), sideObservation.Confidence(), 1.0e-6f);
			Assert::AreEqual(farObservation.MeasurementNoiseSigmaM(), sideObservation.MeasurementNoiseSigmaM(), 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::OpenLike), static_cast<int>(sideObservation.Class()));
		}

		TEST_METHOD(WallObservationPipelineBuildsCanonicalWallObservations)
		{
			WallObs frontLeft{};
			WallObs frontRight{};
			WallObs::BuildFrontWallObservations(
				true,
				true,
				false,
				true,
				0.18f,
				0.21f,
				0.25f,
				frontLeft,
				frontRight);

			Assert::IsTrue(frontLeft.IsValid());
			Assert::IsTrue(frontRight.IsValid());
			Assert::AreEqual(0.18f, frontLeft.Rho(), 1.0e-6f);
			Assert::AreEqual(0.21f, frontRight.Rho(), 1.0e-6f);
			Assert::AreEqual(0.90f, frontLeft.Confidence(), 1.0e-6f);
			Assert::AreEqual(0.0f, frontLeft.MeasurementNoiseSigmaM(), 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::WallLike), static_cast<int>(frontLeft.Class()));

			const WallObs sideObservation = WallObs::BuildSideWallObservation(true, false, true, 0.14f, 0.25f);
			Assert::IsTrue(sideObservation.IsValid());
			Assert::AreEqual(0.14f, sideObservation.Rho(), 1.0e-6f);
			Assert::AreEqual(0.80f, sideObservation.Confidence(), 1.0e-6f);
			Assert::AreEqual(0.0f, sideObservation.MeasurementNoiseSigmaM(), 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::WallLike), static_cast<int>(sideObservation.Class()));

			WallObs::BuildFrontWallObservations(
				true,
				true,
				false,
				true,
				0.18f,
				0.21f,
				0.25f,
				frontLeft,
				frontRight,
				0.003f,
				0.004f);
			Assert::AreEqual(0.003f, frontLeft.MeasurementNoiseSigmaM(), 1.0e-6f);
			Assert::AreEqual(0.004f, frontRight.MeasurementNoiseSigmaM(), 1.0e-6f);
			WallObs::BuildFrontWallObservations(
				true,
				true,
				false,
				true,
				0.18f,
				0.21f,
				0.25f,
				frontLeft,
				frontRight,
				0.003f,
				0.004f,
				0.52f,
				0.61f,
				ObsClass::PostLike,
				ObsClass::WallLike);
			Assert::AreEqual(0.52f, frontLeft.Confidence(), 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::PostLike), static_cast<int>(frontLeft.Class()));

			const WallObs noisySideObservation = WallObs::BuildSideWallObservation(true, false, true, 0.14f, 0.25f, 0.005f);
			Assert::AreEqual(0.005f, noisySideObservation.MeasurementNoiseSigmaM(), 1.0e-6f);
			const WallObs openSideObservation =
				WallObs::BuildSideWallObservation(true, false, false, 0.22f, 0.25f, 0.006f, 0.37f, ObsClass::OpenLike);
			Assert::IsTrue(openSideObservation.IsValid());
			Assert::AreEqual(0.22f, openSideObservation.Rho(), 1.0e-6f);
			Assert::AreEqual(0.37f, openSideObservation.Confidence(), 1.0e-6f);
			Assert::AreEqual(0.006f, openSideObservation.MeasurementNoiseSigmaM(), 1.0e-6f);
			Assert::AreEqual(static_cast<int>(ObsClass::OpenLike), static_cast<int>(openSideObservation.Class()));
		}

		TEST_METHOD(WallGeometryModelRespectsSensorMountForFrontWallPrediction)
		{
			Maze maze;
			maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);

			WallGeometryModel geometry;

			VehicleState state;
			state.SetPosition(Eigen::Vector2f(0.09f, 0.09f));
			state.SetHeading(0.0f);

			const WallGeometryModel::GeometryStateFrame frame = BuildGeometryFrame(geometry, state);
			const SensorMount frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
			GeometryPrediction baseline = geometry.predictRay(frame, frontLeftSensor, maze);
			Assert::IsTrue(baseline.hit);
			Assert::AreEqual(static_cast<int>(GeometryHitType::WallFace), static_cast<int>(baseline.type));

			const SensorMount rotatedSensor = RotateMountYaw(frontLeftSensor, 0.35f);
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

			VehicleState state;
			state.SetPosition(Eigen::Vector2f(0.09f, 0.09f));
			state.SetHeading(0.0f);

			const GeometryPrediction prediction = geometry.predictRay(BuildGeometryFrame(geometry, state), sensor, maze);
			Assert::IsTrue(prediction.hit);
			Assert::AreEqual(static_cast<int>(GeometryHitType::Post), static_cast<int>(prediction.type));
		}

		TEST_METHOD(MapEvidenceUpdaterPostLikeObservationDoesNotForceWall)
		{
			MapEvidenceUpdater updater;
			MapEvidenceUpdater::Config config{};

			const WallObs observation(0.05f, 0.9f, ObsClass::PostLike);

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

			const WallObs wallObservation(0.05f, 0.9f, ObsClass::WallLike);

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
			const WallObs openObservation(0.20f, 0.9f, ObsClass::OpenLike);

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


