#pragma once
// Declares the authoritative micromouse estimator owner that combines the SR-UKF core with maze-evidence updates.

#include "MazeMapRuntimeCore.h"
#include "EncoderObs.h"
#include "ImuAccelObs.h"
#include "CommandVector.h"
#include "Maze.h"
#include "MapEvidenceUpdater.h"
#include "SensorMount.h"
#include "WallObservationPipeline.h"
#include "Direction.h"
#include "SensorSnapshot.h"
#include "SrUkfCore.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace MazeMap
{
    // High-level estimator entry point used by runtime code, simulations, and tests.
    class EXPORT Estimator
    {
    public:
        explicit Estimator(
            const PlantParams& params = PlantParams::Default(),
            VehicleState* runtimeState = nullptr) noexcept;

        void AttachRuntimeState(VehicleState& runtimeState) noexcept;

        SrUkfCore& ukf() noexcept { return _core; }
        const SrUkfCore& ukf() const noexcept { return _core; }
        MapEvidenceUpdater& mapEvidence() noexcept { return _mapEvidence; }
        const MapEvidenceUpdater& mapEvidence() const noexcept { return _mapEvidence; }
        VehicleState& RuntimeState() noexcept { return *_runtimeState; }
        const VehicleState& RuntimeState() const noexcept { return *_runtimeState; }
        void SyncRuntimeState() noexcept;
        bool HasFault() const noexcept { return _faulted; }
        const char* FaultReason() const noexcept
        {
            return (_faultReason[0] != '\0') ? _faultReason : "ukf_failure";
        }
        void ClearFault() noexcept;

        bool ResetPose(float xMeters, float yMeters, float yawRad) noexcept;
        bool ResetForSessionTransition(float xMeters, float yMeters, float yawRad) noexcept;
        bool SetStateCoordinate(int stateIndex, float coordinateM) noexcept;
        bool SetGyroBiasZ(float gyroBiasRadps) noexcept;
        void ProjectMeasuredKinematics(
            float dtSeconds,
            const EncoderObs& encoderObservation,
            float measuredYawRateRadps = std::numeric_limits<float>::quiet_NaN()) noexcept;

        bool predict(
            float dt,
            const App::Internal::CommandVector& control,
            float fanDutyCycle = 0.80f,
            float batteryVoltageV = 0.0f) noexcept;

        template <typename LoopHook>
        bool predict(
            float dt,
            const App::Internal::CommandVector& control,
            float fanDutyCycle,
            float batteryVoltageV,
            LoopHook&& loopHook) noexcept
        {
            return _core.predict(dt, control, fanDutyCycle, batteryVoltageV, static_cast<LoopHook&&>(loopHook));
        }

        MeasurementUpdateResult updateEncoderPair(
            const EncoderObs& observation,
            float dt,
            bool updateYaw = true) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updateEncoderPair(
            const EncoderObs& observation,
            float dt,
            bool updateYaw,
            LoopHook&& loopHook) noexcept
        {
            return _core.updateEncoderPair(observation, dt, updateYaw, static_cast<LoopHook&&>(loopHook));
        }

        MeasurementUpdateResult updateYawRate(float yawRateRadps) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updateYawRate(float yawRateRadps, LoopHook&& loopHook) noexcept
        {
            return _core.updateYawRate(yawRateRadps, static_cast<LoopHook&&>(loopHook));
        }

        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation) noexcept;

        template <typename LoopHook>
        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation, LoopHook&& loopHook) noexcept
        {
            return _core.updatePlanarAccel(observation, static_cast<LoopHook&&>(loopHook));
        }

        bool reset(
            const SrUkfCore::StateVector& state,
            const SrUkfCore::StateMatrix& covariance) noexcept;

        FrontPairUpdateResult updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

        WallUpdateResult updateSideSensor(
            RelativeDirection which,
            const WallObs& observation,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

    private:
        static Direction dominantDirectionForSensor(
            const SensorMount& sensor,
            const VehicleState::StateVector& state) noexcept;
        static CellCoordinates estimateSensorCell(
            const SensorMount& sensor,
            const VehicleState::StateVector& state) noexcept;
        static VehicleState::StateMatrix BuildInitialCovariance() noexcept
        {
            return SrUkfCore::BuildDefaultInitialCovariance();
        }
        void ResetRuntimeMetadata() noexcept;
        void TriggerFault(const char* reason) noexcept;

        SrUkfCore _core;
        MapEvidenceUpdater _mapEvidence;
        VehicleState _localRuntimeState{};
        VehicleState* _runtimeState{};
        bool _faulted = false;
        char _faultReason[64] = {};
    };
}

