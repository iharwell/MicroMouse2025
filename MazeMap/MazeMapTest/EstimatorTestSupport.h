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
        float leftMotorPwm = 0.0f,
        float rightMotorPwm = 0.0f) noexcept
    {
        return App::Internal::CommandVector(leftMotorPwm, rightMotorPwm);
    }

    inline float DefaultFanDutyCycle() noexcept
    {
        return 0.80f;
    }

    inline float DefaultBatteryVoltageV(const PlantParams& params) noexcept
    {
        return params.supplyVoltageV;
    }

    inline void UpdateDriveEstimator(
        DriveBase& drive,
        Estimator& estimator,
        float dtSeconds,
        const SensorSnapshot& snapshot,
        const Maze* map = nullptr)
    {
        drive.RecordMeasurementInputs(snapshot);
        const auto control = drive.CurrentControlVector();
        const float fanDutyCycle = GetMissionFanDutyCycle();
        const float batteryVoltageV = drive.CurrentBatteryVoltageV();
        const EncoderObs encoderObservation = drive.ConsumeEncoderObservation(dtSeconds);
        VehicleState& runtimeState = estimator.RuntimeState();
        runtimeState.SetSensorSnapshot(snapshot);
        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            runtimeState.SetTime(runtimeState.GetTime() + dtSeconds);
        }

        if (estimator.HasFault())
        {
            estimator.SyncRuntimeState();
            return;
        }

        estimator.ukf().setRuntimeContext(
            drive.GetLastLinearCommandMps(),
            drive.GetLastAngularCommandRadps(),
            drive.GetLastSaturationFlags(),
            drive.GetLastLeftLaunchAssistFloor(),
            drive.GetLastRightLaunchAssistFloor(),
            snapshot.accelBiasValid,
            snapshot.accelBodyXMps2,
            snapshot.accelBodyYMps2);

        if (std::isfinite(dtSeconds) && (dtSeconds > 0.0f))
        {
            if (!estimator.predict(dtSeconds, control, fanDutyCycle, batteryVoltageV))
            {
                estimator.SyncRuntimeState();
                return;
            }
        }

        const bool updateYawFromEncoder = !std::isfinite(snapshot.gyroRawRadps);
        (void)estimator.updateEncoderPair(encoderObservation, dtSeconds, updateYawFromEncoder);

        if (std::isfinite(snapshot.gyroRawRadps))
        {
            const MeasurementUpdateResult yawUpdate = estimator.updateYawRate(snapshot.gyroRawRadps);
            if (!yawUpdate.accepted)
            {
                estimator.SyncRuntimeState();
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
            const PlantParams& params = estimator.ukf().params();
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

        estimator.SyncRuntimeState();
    }
}


