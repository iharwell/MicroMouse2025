#include "pch.h"
#include "Estimator.h"

#include "BootUtilityModeFramework.h"

#include <cmath>

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
            const SrUkfCore& core,
            const EncoderObs& encoderObservation,
            float measuredYawRateRadps) noexcept
        {
            MeasuredKinematics measured{};
            const PlantPreparedParams& prepared = core.preparedParams();
            measured.leftVelocityMps = encoderObservation.omegaLeftRadps * prepared.wheelRadiusM;
            measured.rightVelocityMps = encoderObservation.omegaRightRadps * prepared.wheelRadiusM;
            measured.linearSpeedMps = 0.5f * (measured.leftVelocityMps + measured.rightVelocityMps);
            const float fallbackYawRateRadps =
                (prepared.trackWidthM > 0.0f) ?
                ((measured.leftVelocityMps - measured.rightVelocityMps) / prepared.trackWidthM) :
                0.0f;
            measured.angularSpeedRadps =
                std::isfinite(measuredYawRateRadps) ?
                measuredYawRateRadps :
                fallbackYawRateRadps;
            return measured;
        }
    }

    Estimator::Estimator(const PlantParams& params, VehicleState* runtimeState) noexcept
        : _core(params)
        , _mapEvidence()
        , _localRuntimeState()
        , _runtimeState((runtimeState != nullptr) ? runtimeState : &_localRuntimeState)
    {
        SyncRuntimeState();
    }

    bool Estimator::predict(
        float dt,
        const App::Internal::LoopController::ControlVector& control,
        float fanDutyCycle,
        float batteryVoltageV) noexcept
    {
        return _core.predict(dt, control, fanDutyCycle, batteryVoltageV);
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
        SyncRuntimeState();
        return true;
    }

    Direction Estimator::dominantDirectionForSensor(
        const SensorMount& sensor,
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

    CellCoordinates Estimator::estimateSensorCell(
        const SensorMount& sensor,
        const VehicleState::StateVector& state) noexcept
    {
        const WallGeometryModel geometryModel{};
        const Eigen::Vector2f sensorPositionWorld = geometryModel.sensorOriginWorld(state, sensor);
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
            const PlantParams& params = _core.params();
            const SensorMount& sensor =
                (which == RelativeDirection::Left90) ? params.sideLeftSensor : params.sideRightSensor;
            const Direction direction = dominantDirectionForSensor(sensor, _core.state());
            const CellCoordinates cell = estimateSensorCell(sensor, _core.state());
            _mapEvidence.Apply(cell, direction, observation, result.prediction, evidenceConfig, freezeMapMutation);
        }
        return result;
    }

    void Estimator::AttachRuntimeState(VehicleState& runtimeState) noexcept
    {
        runtimeState = *_runtimeState;
        _runtimeState = &runtimeState;
        SyncRuntimeState();
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
        const VehicleState::StateVector currentState = _core.state();
        float preservedGyroBiasRadps = currentState(VehicleState::kBgz);
        if (!std::isfinite(preservedGyroBiasRadps))
        {
            preservedGyroBiasRadps = _core.gyroBiasAnchorRadps();
        }
        if (!std::isfinite(preservedGyroBiasRadps))
        {
            preservedGyroBiasRadps = _runtimeState->GetGyroBiasZ();
        }
        state(VehicleState::kBgz) = std::isfinite(preservedGyroBiasRadps) ? preservedGyroBiasRadps : 0.0f;

        ClearFault();
        const bool ok = _core.reset(state, BuildInitialCovariance());
        if (!ok)
        {
            TriggerFault("session_transition_reset_failed");
            return false;
        }

        SyncRuntimeState();
        ResetRuntimeMetadata();
        return true;
    }

    bool Estimator::SetStateCoordinate(int stateIndex, float coordinateM) noexcept
    {
        VehicleState::StateVector state = _core.state();
        const VehicleState::StateMatrix covariance = _core.covariance();
        state(stateIndex) = coordinateM;
        VehicleState::NormalizeStateVector(state);
        const bool ok = _core.setState(state, covariance);
        if (!ok)
        {
            TriggerFault("set_state_failed");
            return false;
        }
        SyncRuntimeState();
        return true;
    }

    bool Estimator::SetGyroBiasZ(float gyroBiasRadps) noexcept
    {
        VehicleState::StateVector state = _core.state();
        const VehicleState::StateMatrix covariance = _core.covariance();
        state(VehicleState::kBgz) = std::isfinite(gyroBiasRadps) ? gyroBiasRadps : 0.0f;
        VehicleState::NormalizeStateVector(state);
        const bool ok = _core.setState(state, covariance);
        if (!ok)
        {
            TriggerFault("set_gyro_bias_failed");
            return false;
        }
        SyncRuntimeState();
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

        VehicleState::StateVector state = _core.state();
        const VehicleState::StateMatrix covariance = _core.covariance();
        const PlantPreparedParams& prepared = _core.preparedParams();
        if (!(prepared.wheelRadiusM > 0.0f) || !std::isfinite(prepared.wheelRadiusM))
        {
            SyncRuntimeState();
            return;
        }

        const MeasuredKinematics measured = ResolveMeasuredKinematics(_core, encoderObservation, measuredYawRateRadps);
        if ((dtSeconds > 0.0f) && std::isfinite(dtSeconds))
        {
            const float midYawRad =
                WrapAngleRad(state(VehicleState::kPsi) + (0.5f * measured.angularSpeedRadps * dtSeconds));
            const Eigen::Vector2f midHeading = HeadingUnitFromYawRad(midYawRad);
            state(VehicleState::kPx) += measured.linearSpeedMps * midHeading.x() * dtSeconds;
            state(VehicleState::kPy) += measured.linearSpeedMps * midHeading.y() * dtSeconds;
            state(VehicleState::kPsi) =
                WrapAngleRad(state(VehicleState::kPsi) + (measured.angularSpeedRadps * dtSeconds));
        }

        state(VehicleState::kU) = measured.linearSpeedMps;
        state(VehicleState::kV) = 0.0f;
        state(VehicleState::kR) = measured.angularSpeedRadps;
        state(VehicleState::kOmegaL) = encoderObservation.omegaLeftRadps;
        state(VehicleState::kOmegaR) = encoderObservation.omegaRightRadps;
        VehicleState::NormalizeStateVector(state);
        if (!_core.setState(state, covariance))
        {
            TriggerFault("project_measured_kinematics_failed");
            return;
        }
        SyncRuntimeState();
    }

    void Estimator::ResetRuntimeMetadata() noexcept
    {
        _runtimeState->SetTime(0.0f);
        _runtimeState->SetTimestampUs(0U);
        _runtimeState->SetSensorSnapshot(SensorSnapshot{});
    }

    void Estimator::SyncRuntimeState() noexcept
    {
        _runtimeState->SetStateVector(_core.state());
        _runtimeState->SetCovariance(_core.covariance());
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

