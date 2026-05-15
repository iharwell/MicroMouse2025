#include "pch.h"
#include "Estimator.h"

#include "BootUtilityModeFramework.h"
#include "Vehicle.h"
#include "WallGeometryModel.h"

#include <cmath>
#include <cstdio>

namespace MazeMap
{
    namespace
    {
        struct MeasuredKinematics final
        {
            float leftVelocityMps = 0.0f;
            float rightVelocityMps = 0.0f;
            float linearSpeedMps = 0.0f;
            float angularSpeedRadps = 0.0f;
        };

        MeasuredKinematics ResolveMeasuredKinematics(
            const EncoderObs& encoderObservation,
            float measuredYawRateRadps) noexcept
        {
            MeasuredKinematics measured{};
            measured.leftVelocityMps = Vehicle::WheelLinearVelocityFromOmega(encoderObservation.omegaLeftRadps);
            measured.rightVelocityMps = Vehicle::WheelLinearVelocityFromOmega(encoderObservation.omegaRightRadps);
            measured.linearSpeedMps =
                Vehicle::BodyForwardVelocityFromWheelLinear(measured.leftVelocityMps, measured.rightVelocityMps);
            const float fallbackYawRateRadps =
                Vehicle::BodyYawRateFromWheelLinear(measured.leftVelocityMps, measured.rightVelocityMps);
            measured.angularSpeedRadps =
                std::isfinite(measuredYawRateRadps) ?
                measuredYawRateRadps :
                fallbackYawRateRadps;
            return measured;
        }

        WallGeometryModel::GeometryStateFrame BuildWallGeometryFrame(
            const WallGeometryModel& geometryModel,
            const VehicleState::StateVector& state) noexcept
        {
            return geometryModel.buildStateFrame(
                Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)),
                state(VehicleState::kPsi));
        }
    }

    Estimator::Estimator(const PlantModel& plantModel, VehicleState& runtimeState) noexcept
        : _core(plantModel, runtimeState)
        , _mapEvidence()
        , _runtimeState(runtimeState)
    {
    }

    bool Estimator::predict(
        float dt,
        const App::Internal::CommandVector& control) noexcept
    {
        return _core.predict(dt, control);
    }

    MeasurementUpdateResult Estimator::updateEncoderPair(
        const EncoderObs& observation,
        float dt,
        bool updateYaw) noexcept
    {
        return _core.updateEncoderPair(observation, dt, updateYaw);
    }

    MeasurementUpdateResult Estimator::updateYawRate(float yawRateRadps) noexcept
    {
        return _core.updateYawRate(yawRateRadps);
    }

    MeasurementUpdateResult Estimator::updatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        return _core.updatePlanarAccel(observation);
    }

    bool Estimator::reset(
        const SrUkfCore::StateVector& state,
        const SrUkfCore::StateMatrix& covariance) noexcept
    {
        _mapEvidence.Reset();
        const bool ok = _core.reset(state, covariance);
        if (!ok)
        {
            TriggerFault("reset_failed");
            return false;
        }
        return true;
    }

    Direction Estimator::dominantDirectionForSensor(
        const SensorMount& sensor,
        const VehicleState::StateVector& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const Eigen::Vector2f directionWorld =
            geometryModel.sensorDirectionWorld(BuildWallGeometryFrame(geometryModel, state), sensor);
        const float x = directionWorld.x();
        const float y = directionWorld.y();
        if (std::fabs(x) >= std::fabs(y))
        {
            return (x >= 0.0f) ? Direction::Right : Direction::Left;
        }
        return (y >= 0.0f) ? Direction::Up : Direction::Down;
    }

    CellCoordinates Estimator::estimateSensorCell(
        const SensorMount& sensor,
        const VehicleState::StateVector& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const Eigen::Vector2f sensorPositionWorld =
            geometryModel.sensorOriginWorld(BuildWallGeometryFrame(geometryModel, state), sensor);
        return WallGeometryModel::WorldToCell(sensorPositionWorld.x(), sensorPositionWorld.y());
    }

    FrontPairUpdateResult Estimator::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const Maze& maze,
        bool freezeMapMutation,
        const MapEvidenceUpdater::Config& evidenceConfig) noexcept
    {
        FrontPairUpdateResult result = _core.updateFrontPair(left, right, maze);
        if (result.filter.accepted && !freezeMapMutation)
        {
            const SensorMount frontLeftSensor = Vehicle::GetFrontLeftSensorMount();
            const SensorMount frontRightSensor = Vehicle::GetFrontRightSensorMount();
            const Direction leftDirection = dominantDirectionForSensor(frontLeftSensor, _core.workingState());
            const Direction rightDirection = dominantDirectionForSensor(frontRightSensor, _core.workingState());
            const CellCoordinates leftCell = estimateSensorCell(frontLeftSensor, _core.workingState());
            const CellCoordinates rightCell = estimateSensorCell(frontRightSensor, _core.workingState());
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

    WallUpdateResult Estimator::updateSideSensor(
        RelativeDirection which,
        const WallObs& observation,
        const Maze& maze,
        bool freezeMapMutation,
        const MapEvidenceUpdater::Config& evidenceConfig) noexcept
    {
        WallUpdateResult result = _core.updateSideSensor(which, observation, maze);
        if (result.filter.accepted && !freezeMapMutation)
        {
            const SensorMount& sensor =
                (which == RelativeDirection::Left90) ?
                Vehicle::GetSideLeftSensorMount() :
                Vehicle::GetSideRightSensorMount();
            const Direction direction = dominantDirectionForSensor(sensor, _core.workingState());
            const CellCoordinates cell = estimateSensorCell(sensor, _core.workingState());
            _mapEvidence.Apply(cell, direction, observation, result.prediction, evidenceConfig, freezeMapMutation);
        }
        return result;
    }

    void Estimator::ClearFault() noexcept
    {
        _faulted = false;
        _faultReason[0] = '\0';
    }

    bool Estimator::ResetPose(float xMeters, float yMeters, float yawRad) noexcept
    {
        VehicleState::StateVector state = VehicleState::StateVector::Zero();
        state(VehicleState::kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(VehicleState::kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(VehicleState::kPsi) = WrapAngleRad(yawRad);
        ClearFault();
        return reset(state, BuildInitialCovariance());
    }

    bool Estimator::ResetForSessionTransition(float xMeters, float yMeters, float yawRad) noexcept
    {
        VehicleState::StateVector state = VehicleState::StateVector::Zero();
        state(VehicleState::kPx) = std::isfinite(xMeters) ? xMeters : 0.0f;
        state(VehicleState::kPy) = std::isfinite(yMeters) ? yMeters : 0.0f;
        state(VehicleState::kPsi) = WrapAngleRad(yawRad);
        float preservedGyroBiasRadps = _runtimeState.GetGyroBiasZ();
        if (!std::isfinite(preservedGyroBiasRadps))
        {
            preservedGyroBiasRadps = 0.0f;
        }
        state(VehicleState::kBgz) = std::isfinite(preservedGyroBiasRadps) ? preservedGyroBiasRadps : 0.0f;

        ClearFault();
        const bool ok = _core.reset(state, BuildInitialCovariance());
        if (!ok)
        {
            TriggerFault("session_transition_reset_failed");
            return false;
        }

        ResetRuntimeMetadata();
        return true;
    }

    bool Estimator::RestoreSessionStartPhysicalState(float xMeters, float yMeters, float yawRad) noexcept
    {
        // Sorting note: this owns only the estimator state restore. Encoder consumption,
        // controller reset, and braking remain DriveBase concerns and are not translated here.
        return ResetForSessionTransition(xMeters, yMeters, yawRad);
    }

    bool Estimator::SetGyroBiasZ(float gyroBiasRadps) noexcept
    {
        _runtimeState.SetGyroBiasZ(std::isfinite(gyroBiasRadps) ? gyroBiasRadps : 0.0f);
        return true;
    }

    void Estimator::ProjectMeasuredKinematics(
        float dtSeconds,
        const EncoderObs& encoderObservation,
        float measuredYawRateRadps) noexcept
    {
        if (_faulted)
        {
            return;
        }

        const MeasuredKinematics measured =
            ResolveMeasuredKinematics(encoderObservation, measuredYawRateRadps);
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float midYawRad =
                WrapAngleRad(_runtimeState.GetOrientation() + (0.5f * measured.angularSpeedRadps * dtSeconds));
            const Eigen::Vector2f midHeading = HeadingUnitFromYawRad(midYawRad);
            _runtimeState.SetPosition(Eigen::Vector2f(
                _runtimeState.GetPositionX() + (measured.linearSpeedMps * midHeading.x() * dtSeconds),
                _runtimeState.GetPositionY() + (measured.linearSpeedMps * midHeading.y() * dtSeconds)));
            _runtimeState.SetOrientation(_runtimeState.GetOrientation() + (measured.angularSpeedRadps * dtSeconds));
        }

        _runtimeState.SetVelocity(measured.linearSpeedMps);
        _runtimeState.SetLateralVelocity(0.0f);
        _runtimeState.SetRotationalVelocity(measured.angularSpeedRadps);
        _runtimeState.SetLongitudinalAcceleration(0.0f);
        _runtimeState.SetLateralAcceleration(0.0f);
        _runtimeState.SetYawAcceleration(0.0f);
        _runtimeState.SetWheelSpeedLeft(encoderObservation.omegaLeftRadps);
        _runtimeState.SetWheelSpeedRight(encoderObservation.omegaRightRadps);
    }

    void Estimator::ResetRuntimeMetadata() noexcept
    {
        _runtimeState.SetTime(0.0f);
        _runtimeState.SetTimestampUs(0U);
        _runtimeState.SetLongitudinalAcceleration(0.0f);
        _runtimeState.SetLateralAcceleration(0.0f);
        _runtimeState.SetYawAcceleration(0.0f);
        _runtimeState.SetSensorSnapshot(SensorSnapshot{});
    }

    void Estimator::TriggerFault(const char* reason) noexcept
    {
        if (_faulted)
        {
            return;
        }

        _faulted = true;
        std::snprintf(
            _faultReason,
            sizeof(_faultReason),
            "%s",
            (reason != nullptr && reason[0] != '\0') ? reason : "ukf_failure");

        char traceLine[96] = {};
        std::snprintf(traceLine, sizeof(traceLine), "ukf_fault:%s", _faultReason);
        MazeMap::App::Internal::BootUtilityModeFramework::AppendStartupTrace(traceLine);
    }
}

