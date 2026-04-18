#include "pch.h"
#include "MouseUkfFacade.h"

#include <cmath>

namespace MazeMap
{
    MouseUkfFacade::MouseUkfFacade(const PlantParams& params) noexcept
        : _core(params)
        , _mapEvidence()
    {
    }

    bool MouseUkfFacade::predict(float dt, const ControlInput& control) noexcept
    {
        return _core.predict(dt, control);
    }

    MeasurementUpdateResult MouseUkfFacade::updateEncoderPair(
        const EncoderObs& observation,
        float dt,
        bool updateYaw) noexcept
    {
        return _core.updateEncoderPair(observation, dt, updateYaw);
    }

    MeasurementUpdateResult MouseUkfFacade::updateYawRate(float yawRateRadps) noexcept
    {
        return _core.updateYawRate(yawRateRadps);
    }

    MeasurementUpdateResult MouseUkfFacade::updatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        return _core.updatePlanarAccel(observation);
    }

    MeasurementUpdateResult MouseUkfFacade::updateImuMerged(const ImuMergedObs& observation) noexcept
    {
        return _core.updateImuMerged(observation);
    }

    bool MouseUkfFacade::reset(
        const SrUkfCore::StateVector& state,
        const SrUkfCore::StateMatrix& covariance) noexcept
    {
        _mapEvidence.Reset();
        return _core.reset(state, covariance);
    }

    Direction MouseUkfFacade::dominantDirectionForSensor(
        const SensorExtrinsics& sensor,
        const VehicleState::StateVector& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const Eigen::Vector2f directionWorld = geometryModel.sensorDirectionWorld(state, sensor);
        const float x = directionWorld.x();
        const float y = directionWorld.y();
        if (std::fabs(x) >= std::fabs(y))
        {
            return (x >= 0.0f) ? Direction::Right : Direction::Left;
        }
        return (y >= 0.0f) ? Direction::Up : Direction::Down;
    }

    CellCoordinates MouseUkfFacade::estimateSensorCell(
        const SensorExtrinsics& sensor,
        const VehicleState::StateVector& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const Eigen::Vector2f sensorPositionWorld = geometryModel.sensorOriginWorld(state, sensor);
        return WallGeometryModel::WorldToCell(sensorPositionWorld.x(), sensorPositionWorld.y());
    }

    FrontPairUpdateResult MouseUkfFacade::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const Maze& maze,
        bool freezeMapMutation,
        const MapEvidenceUpdater::Config& evidenceConfig) noexcept
    {
        FrontPairUpdateResult result = _core.updateFrontPair(left, right, maze);
        if (result.filter.accepted && !freezeMapMutation)
        {
            const PlantParams& params = _core.params();
            const Direction leftDirection = dominantDirectionForSensor(params.frontLeftSensor, _core.state());
            const Direction rightDirection = dominantDirectionForSensor(params.frontRightSensor, _core.state());
            const CellCoordinates leftCell = estimateSensorCell(params.frontLeftSensor, _core.state());
            const CellCoordinates rightCell = estimateSensorCell(params.frontRightSensor, _core.state());
            _mapEvidence.Apply(leftCell, leftDirection, left, result.leftPrediction, evidenceConfig, freezeMapMutation);
            _mapEvidence.Apply(
                rightCell,
                rightDirection,
                right,
                result.rightPrediction,
                evidenceConfig,
                freezeMapMutation);
        }
        return result;
    }

    WallUpdateResult MouseUkfFacade::updateSideSensor(
        Side which,
        const WallObs& observation,
        const Maze& maze,
        bool freezeMapMutation,
        const MapEvidenceUpdater::Config& evidenceConfig) noexcept
    {
        WallUpdateResult result = _core.updateSideSensor(which, observation, maze);
        if (result.filter.accepted && !freezeMapMutation)
        {
            const PlantParams& params = _core.params();
            const SensorExtrinsics& sensor =
                (which == Side::Left) ? params.sideLeftSensor : params.sideRightSensor;
            const Direction direction = dominantDirectionForSensor(sensor, _core.state());
            const CellCoordinates cell = estimateSensorCell(sensor, _core.state());
            _mapEvidence.Apply(cell, direction, observation, result.prediction, evidenceConfig, freezeMapMutation);
        }
        return result;
    }
}
