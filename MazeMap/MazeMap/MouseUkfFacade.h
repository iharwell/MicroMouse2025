#pragma once
// Declares the high-level micromouse estimator facade that combines the SR-UKF core with maze-evidence updates.

#include "Maze.h"
#include "MapEvidenceUpdater.h"
#include "SrUkfCore.h"

namespace MazeMap
{
    // High-level UKF entry point used by runtime code, simulations, and tests.
    class EXPORT MouseUkfFacade
    {
    public:
        explicit MouseUkfFacade(const PlantParams& params = PlantParams::Default()) noexcept;

        SrUkfCore& ukf() noexcept { return _core; }
        const SrUkfCore& ukf() const noexcept { return _core; }
        MapEvidenceUpdater& mapEvidence() noexcept { return _mapEvidence; }
        const MapEvidenceUpdater& mapEvidence() const noexcept { return _mapEvidence; }

        bool predict(float dt, const ControlInput& control) noexcept;

        template <typename LoopHook>
        bool predict(float dt, const ControlInput& control, LoopHook&& loopHook) noexcept
        {
            return _core.predict(dt, control, static_cast<LoopHook&&>(loopHook));
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

        MeasurementUpdateResult updateImuMerged(const ImuMergedObs& observation) noexcept;
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
            Side which,
            const WallObs& observation,
            const Maze& maze,
            bool freezeMapMutation = false,
            const MapEvidenceUpdater::Config& evidenceConfig = MapEvidenceUpdater::Config{}) noexcept;

    private:
        static Direction dominantDirectionForSensor(
            const SensorExtrinsics& sensor,
            const VehicleState::StateVector& state) noexcept;
        static CellCoordinates estimateSensorCell(
            const SensorExtrinsics& sensor,
            const VehicleState::StateVector& state) noexcept;

        SrUkfCore _core;
        MapEvidenceUpdater _mapEvidence;
    };
}
