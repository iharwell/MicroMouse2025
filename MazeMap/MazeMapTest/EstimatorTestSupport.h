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

        const bool updateYawFromEncoder = !std::isfinite(snapshot.gyroRawRadps);
        if (snapshot.encoderObservationValid)
        {
            (void)estimator.updateEncoderPair(snapshot.encoderObservation, dtSeconds, updateYawFromEncoder);
        }

        if (std::isfinite(snapshot.gyroRawRadps))
        {
            const MeasurementUpdateResult yawUpdate = estimator.updateYawRate(snapshot.gyroRawRadps);
            if (!yawUpdate.accepted)
            {
                return;
            }
        }

        ImuAccelObs accelObservation{};
        accelObservation.valid =
            snapshot.accelBiasValid &&
            std::isfinite(snapshot.accelBodyXMps2) &&
            std::isfinite(snapshot.accelBodyYMps2);
        accelObservation.accelBodyXMps2 = snapshot.accelBodyXMps2;
        accelObservation.accelBodyYMps2 = snapshot.accelBodyYMps2;
        (void)estimator.updatePlanarAccel(accelObservation);

        if (map != nullptr)
        {
            const Vehicle wallSensorOwner;
            const float frontLeftMaxRangeM = wallSensorOwner.FrontLeft.DistanceFromDifferentialLight(0.0f);
            const float frontRightMaxRangeM = wallSensorOwner.FrontRight.DistanceFromDifferentialLight(0.0f);
            const float sideLeftMaxRangeM = wallSensorOwner.SideLeft.DistanceFromDifferentialLight(0.0f);
            const float sideRightMaxRangeM = wallSensorOwner.SideRight.DistanceFromDifferentialLight(0.0f);
            WallObs frontLeftObs{};
            WallObs frontRightObs{};
            WallObs unusedFrontLeftObs{};
            WallObs unusedFrontRightObs{};
            BuildFrontWallObservations(
                snapshot.frontWallObservationValid,
                snapshot.frontWall,
                snapshot.frontWallUsesFallbackDetection,
                snapshot.frontWallUsesCharacterizationDetection,
                snapshot.frontLeftDistanceM,
                snapshot.frontRightDistanceM,
                frontLeftMaxRangeM,
                frontLeftObs,
                unusedFrontRightObs);
            BuildFrontWallObservations(
                snapshot.frontWallObservationValid,
                snapshot.frontWall,
                snapshot.frontWallUsesFallbackDetection,
                snapshot.frontWallUsesCharacterizationDetection,
                snapshot.frontLeftDistanceM,
                snapshot.frontRightDistanceM,
                frontRightMaxRangeM,
                unusedFrontLeftObs,
                frontRightObs);
            if (frontLeftObs.valid && frontRightObs.valid)
            {
                (void)estimator.updateFrontPair(frontLeftObs, frontRightObs, *map, true);
            }

            const WallObs leftSideObs = BuildSideWallObservation(
                snapshot.leftDistanceValidForControl,
                snapshot.leftTransitionDetected,
                snapshot.leftWallObservation,
                snapshot.sideLeftDistanceM,
                sideLeftMaxRangeM);
            if (leftSideObs.valid)
            {
                (void)estimator.updateSideSensor(RelativeDirection::Left90, leftSideObs, *map, true);
            }

            const WallObs rightSideObs = BuildSideWallObservation(
                snapshot.rightDistanceValidForControl,
                snapshot.rightTransitionDetected,
                snapshot.rightWallObservation,
                snapshot.sideRightDistanceM,
                sideRightMaxRangeM);
            if (rightSideObs.valid)
            {
                (void)estimator.updateSideSensor(RelativeDirection::Right90, rightSideObs, *map, true);
            }
        }

    }
}


