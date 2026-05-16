#pragma once

#include "..\MazeMap\DriveBase.h"
#include "..\MazeMap\Estimator.h"
#include "..\MazeMap\PlantModel.h"
#include "..\MazeMap\VehicleState.h"
#include "..\MazeMap\WallGeometryModel.h"

#include <cmath>

namespace MazeMap
{
    inline VehicleState::StateVector BuildUkfState(
        float xM,
        float yM,
        float yawRad,
        float forwardVelocityMps,
        float lateralVelocityMps,
        float yawRateRadps,
        float leftWheelSpeedRadps,
        float rightWheelSpeedRadps,
        float gyroBiasRadps = 0.0f)
    {
        VehicleState::StateVector state = VehicleState::StateVector::Zero();
        state(VehicleState::kPx) = xM;
        state(VehicleState::kPy) = yM;
        state(VehicleState::kPsi) = yawRad;
        state(VehicleState::kU) = forwardVelocityMps;
        state(VehicleState::kV) = lateralVelocityMps;
        state(VehicleState::kR) = yawRateRadps;
        state(VehicleState::kOmegaL) = leftWheelSpeedRadps;
        state(VehicleState::kOmegaR) = rightWheelSpeedRadps;
        state(VehicleState::kBgz) = gyroBiasRadps;
        VehicleState::NormalizeStateVector(state);
        return state;
    }

    inline void SetVehicleStateFromUkfStateVector(
        VehicleState& vehicleState,
        const VehicleState::StateVector& state)
    {
        vehicleState.SetPosition(Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)));
        vehicleState.SetOrientation(state(VehicleState::kPsi));
        vehicleState.SetVelocity(state(VehicleState::kU));
        vehicleState.SetLateralVelocity(state(VehicleState::kV));
        vehicleState.SetRotationalVelocity(state(VehicleState::kR));
        vehicleState.SetWheelSpeedLeft(state(VehicleState::kOmegaL));
        vehicleState.SetWheelSpeedRight(state(VehicleState::kOmegaR));
        vehicleState.SetGyroBiasZ(state(VehicleState::kBgz));
    }

    inline VehicleState::StateVector BuildUkfStateVector(const VehicleState& state)
    {
        return BuildUkfState(
            state.GetPositionX(),
            state.GetPositionY(),
            state.GetOrientation(),
            state.GetVelocity(),
            state.GetLateralVelocity(),
            state.GetRotationalVelocity(),
            state.GetWheelSpeedLeft(),
            state.GetWheelSpeedRight(),
            state.GetGyroBiasZ());
    }

    inline VehicleState::StateMatrix BuildUkfCovariance(
        float positionSigmaM = 0.01f,
        float headingSigmaRad = 0.03f,
        float forwardVelocitySigmaMps = 0.05f,
        float lateralVelocitySigmaMps = 0.05f,
        float yawRateSigmaRadps = 0.10f,
        float wheelSigmaRadps = 0.30f,
        float gyroBiasSigmaRadps = 0.03f)
    {
        VehicleState::StateMatrix covariance = VehicleState::StateMatrix::Zero();
        covariance(VehicleState::kPx, VehicleState::kPx) = positionSigmaM * positionSigmaM;
        covariance(VehicleState::kPy, VehicleState::kPy) = positionSigmaM * positionSigmaM;
        covariance(VehicleState::kPsi, VehicleState::kPsi) = headingSigmaRad * headingSigmaRad;
        covariance(VehicleState::kU, VehicleState::kU) = forwardVelocitySigmaMps * forwardVelocitySigmaMps;
        covariance(VehicleState::kV, VehicleState::kV) = lateralVelocitySigmaMps * lateralVelocitySigmaMps;
        covariance(VehicleState::kR, VehicleState::kR) = yawRateSigmaRadps * yawRateSigmaRadps;
        covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelSigmaRadps * wheelSigmaRadps;
        covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelSigmaRadps * wheelSigmaRadps;
        covariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasSigmaRadps * gyroBiasSigmaRadps;
        return covariance;
    }

    inline float DistancePerEncoderCountMeters(const PlantParams& params) noexcept
    {
        return
            (2.0f * PI_F * params.wheelRadiusM) /
            (params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev));
    }

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
            const PlantParams params = PlantParams::Default();
            WallObs frontLeftObs{};
            WallObs frontRightObs{};
            BuildFrontWallObservations(
                snapshot.frontWallObservationValid,
                snapshot.frontWall,
                snapshot.frontWallUsesFallbackDetection,
                snapshot.frontWallUsesCharacterizationDetection,
                snapshot.frontLeftDistanceM,
                snapshot.frontRightDistanceM,
                params.noHitRangeM,
                frontLeftObs,
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
                params.noHitRangeM);
            if (leftSideObs.valid)
            {
                (void)estimator.updateSideSensor(RelativeDirection::Left90, leftSideObs, *map, true);
            }

            const WallObs rightSideObs = BuildSideWallObservation(
                snapshot.rightDistanceValidForControl,
                snapshot.rightTransitionDetected,
                snapshot.rightWallObservation,
                snapshot.sideRightDistanceM,
                params.noHitRangeM);
            if (rightSideObs.valid)
            {
                (void)estimator.updateSideSensor(RelativeDirection::Right90, rightSideObs, *map, true);
            }
        }

    }
}


