#pragma once

#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\Vehicle.h"
#include "..\MazeMap\VehicleState.h"
#include "..\MazeMap\WallGeometryModel.h"
#include "..\MazeMap\WallObservationPipeline.h"

#include <cmath>

namespace MazeMap
{
    inline App::Internal::CommandVector MakeControlVector(
        float leftCommand = 0.0f,
        float rightCommand = 0.0f) noexcept
    {
        return App::Internal::CommandVector(leftCommand, rightCommand);
    }

    inline float DefaultFanDutyCycle() noexcept
    {
        return 0.80f;
    }

    inline void UpdateDriveEstimator(
        Estimator& estimator,
        VehicleState& runtimeState,
        float dtSeconds,
        const SensorSnapshot& snapshot,
        const App::Internal::CommandVector& appliedControl = MakeControlVector(),
        const Maze* map = nullptr)
    {
        runtimeState.SetSensorSnapshot(snapshot);
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
        }

        if (estimator.HasFault())
        {
            return;
        }

        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            if (!estimator.predict(dtSeconds, appliedControl))
            {
                return;
            }
        }

        const SensorSnapshot& publishedSnapshot = runtimeState.GetSensorSnapshot();
        if (std::isfinite(publishedSnapshot.RawYawRateRadps()))
        {
            if (!estimator.updateYawRate(publishedSnapshot.YawRateRadps()))
            {
                return;
            }
        }

        const bool accelObservationValid =
            publishedSnapshot.AccelerationBiasValid() &&
            std::isfinite(publishedSnapshot.BodyRightAccelerationMps2()) &&
            std::isfinite(publishedSnapshot.BodyForwardAccelerationMps2());
        const ImuAccelObs accelObservation(
            accelObservationValid,
            publishedSnapshot.BodyForwardAccelerationMps2(),
            publishedSnapshot.BodyRightAccelerationMps2());
        (void)estimator.updatePlanarAccel(accelObservation);

        if (map != nullptr)
        {
            const Vehicle wallSensorOwner;
            const float frontLeftMaxRangeM = wallSensorOwner.FrontLeftWallSensor().DistanceFromDifferentialLight(0.0f);
            const float frontRightMaxRangeM = wallSensorOwner.FrontRightWallSensor().DistanceFromDifferentialLight(0.0f);
            const float sideLeftMaxRangeM = wallSensorOwner.SideLeftWallSensor().DistanceFromDifferentialLight(0.0f);
            const float sideRightMaxRangeM = wallSensorOwner.SideRightWallSensor().DistanceFromDifferentialLight(0.0f);
            WallObs frontLeftObs{};
            WallObs frontRightObs{};
            WallObs unusedFrontLeftObs{};
            WallObs unusedFrontRightObs{};
            WallObs::BuildFrontWallObservations(
                publishedSnapshot.FrontWallObservationValid(),
                publishedSnapshot.HasFrontWall(),
                publishedSnapshot.FrontWallUsesFallbackDetection(),
                publishedSnapshot.FrontWallUsesCharacterizationDetection(),
                publishedSnapshot.FrontLeftDistanceM(),
                publishedSnapshot.FrontRightDistanceM(),
                frontLeftMaxRangeM,
                frontLeftObs,
                unusedFrontRightObs);
            WallObs::BuildFrontWallObservations(
                publishedSnapshot.FrontWallObservationValid(),
                publishedSnapshot.HasFrontWall(),
                publishedSnapshot.FrontWallUsesFallbackDetection(),
                publishedSnapshot.FrontWallUsesCharacterizationDetection(),
                publishedSnapshot.FrontLeftDistanceM(),
                publishedSnapshot.FrontRightDistanceM(),
                frontRightMaxRangeM,
                unusedFrontLeftObs,
                frontRightObs);
            if (frontLeftObs.IsValid() && frontRightObs.IsValid())
            {
                (void)estimator.updateFrontPair(frontLeftObs, frontRightObs, *map, true);
            }

            const WallObs leftSideObs = WallObs::BuildSideWallObservation(
                publishedSnapshot.LeftDistanceValidForControl(),
                publishedSnapshot.LeftTransitionDetected(),
                publishedSnapshot.HasLeftWallObservation(),
                publishedSnapshot.SideLeftDistanceM(),
                sideLeftMaxRangeM);
            if (leftSideObs.IsValid())
            {
                (void)estimator.updateSideSensor(RelativeDirection::Left90, leftSideObs, *map, true);
            }

            const WallObs rightSideObs = WallObs::BuildSideWallObservation(
                publishedSnapshot.RightDistanceValidForControl(),
                publishedSnapshot.RightTransitionDetected(),
                publishedSnapshot.HasRightWallObservation(),
                publishedSnapshot.SideRightDistanceM(),
                sideRightMaxRangeM);
            if (rightSideObs.IsValid())
            {
                (void)estimator.updateSideSensor(RelativeDirection::Right90, rightSideObs, *map, true);
            }
        }

    }
}


