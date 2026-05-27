#include "pch.h"
#include "EstimatorWallMotionUpdateTestSupport.h"

namespace MazeMap
{
    namespace EstimatorMotionUpdateSupport
    {
        static WallGeometryModel::GeometryStateFrame BuildGeometryFrame(
            const WallGeometryModel& geometry,
            const StateVector& state)
        {
            return geometry.buildStateFrame(
                Eigen::Vector2f(state(0), state(1)),
                state(2));
        }

        WallUpdateScenario RunFrontWallScenario()
        {
            WallUpdateScenario scenario;
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Up, WallState::Wall);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.09f;
            initialState(1) = 0.09f;

            WallGeometryModel geometry;
            const WallGeometryModel::GeometryStateFrame frame =
                BuildGeometryFrame(geometry, initialState);
            const GeometryPrediction leftPrediction =
                geometry.predictRay(frame, Vehicle::GetFrontLeftSensorMount(), maze);
            const GeometryPrediction rightPrediction =
                geometry.predictRay(frame, Vehicle::GetFrontRightSensorMount(), maze);
            scenario.primaryPredictionHit = leftPrediction.hit;
            scenario.secondaryPredictionHit = rightPrediction.hit;

            Estimator core = MakeDefaultEstimator();
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.02f * 0.02f;
            initialCovariance(4, 4) = 0.02f * 0.02f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            scenario.beforeState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);

            const WallObs leftObservation =
                WallObs(leftPrediction.rangeM - 0.012f, 1.0f, ObsClass::WallLike);
            const WallObs rightObservation =
                WallObs(rightPrediction.rangeM - 0.012f, 1.0f, ObsClass::WallLike);
            scenario.updateReturnedAccepted =
                core.updateFrontPair(leftObservation, rightObservation, maze);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }

        WallUpdateScenario RunLeftWallScenario()
        {
            WallUpdateScenario scenario;
            Maze maze;
            maze.SetWall(maze(0, 0), Direction::Left, WallState::Wall);
            StateVector initialState = StateVector::Zero();
            initialState(0) = 0.09f;
            initialState(1) = 0.09f;

            WallGeometryModel geometry;
            const GeometryPrediction baseline =
                geometry.predictRay(
                    BuildGeometryFrame(geometry, initialState),
                    Vehicle::GetSideLeftSensorMount(),
                    maze);
            scenario.primaryPredictionHit = baseline.hit;

            Estimator core = MakeDefaultEstimator();
            CovarianceMatrix initialCovariance = CovarianceMatrix::Zero();
            initialCovariance(0, 0) = 0.02f * 0.02f;
            initialCovariance(1, 1) = 0.02f * 0.02f;
            initialCovariance(2, 2) = 0.04f * 0.04f;
            initialCovariance(3, 3) = 0.02f * 0.02f;
            initialCovariance(4, 4) = 0.02f * 0.02f;
            initialCovariance(5, 5) = 0.05f * 0.05f;
            initialCovariance(6, 6) = 0.05f * 0.05f;
            initialCovariance(7, 7) = 0.05f * 0.05f;
            initialCovariance(8, 8) = 0.02f * 0.02f;
            scenario.resetAccepted = EstimatorMotionUpdateTest::Reset(core, initialState, initialCovariance);
            scenario.beforeState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.beforeCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);

            const WallObs observation =
                WallObs(baseline.rangeM - 0.012f, 1.0f, ObsClass::WallLike);
            scenario.updateReturnedAccepted =
                core.updateSideSensor(RelativeDirection::Left90, observation, maze);
            scenario.updateAttempted = EstimatorMotionUpdateTest::LastUpdateAttempted(core);
            scenario.updateRecordedAccepted = EstimatorMotionUpdateTest::LastUpdateAccepted(core);
            scenario.afterState = EstimatorMotionUpdateTest::WorkingState(core);
            scenario.afterCovariance = EstimatorMotionUpdateTest::WorkingCovariance(core);
            return scenario;
        }
    }
}
