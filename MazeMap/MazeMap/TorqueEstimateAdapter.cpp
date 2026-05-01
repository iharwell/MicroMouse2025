#include "pch.h"
#include "TorqueEstimateAdapter.h"

#include "PlantModel.h"

#include <cmath>

namespace MazeMap
{
    EstimatorGeometry BuildEstimatorGeometry(const PlantPreparedParams& params) noexcept
    {
        EstimatorGeometry geometry{};
        geometry.effectiveWheelRadiusM = (std::isfinite(params.wheelRadiusM) && (params.wheelRadiusM > 0.0f)) ? params.wheelRadiusM : 0.0f;
        geometry.inverseEffectiveWheelRadiusPerM =
            (geometry.effectiveWheelRadiusM > 0.0f) ? (1.0f / geometry.effectiveWheelRadiusM) : 0.0f;
        geometry.effectiveTrackWidthM = (std::isfinite(params.trackWidthM) && (params.trackWidthM > 0.0f)) ? params.trackWidthM : 0.0f;
        geometry.halfEffectiveTrackWidthM = 0.5f * geometry.effectiveTrackWidthM;
        geometry.inverseEffectiveTrackWidthPerM =
            (geometry.effectiveTrackWidthM > 0.0f) ? (1.0f / geometry.effectiveTrackWidthM) : 0.0f;
        geometry.effectiveYawLeverArmM = (std::isfinite(params.yawLeverArmM) && (params.yawLeverArmM > 0.0f)) ? params.yawLeverArmM : 0.0f;
        geometry.imuPositionBodyM = params.imuPositionBodyM;
        return geometry;
    }

    EstimatorGeometry BuildEstimatorGeometry(const PlantParams& params) noexcept
    {
        return BuildEstimatorGeometry(PlantModel::Prepare(params));
    }

    AppliedTorqueEstimate TorqueEstimateAdapter::Estimate(
        const PlantModel& plant,
        const VehicleState::StateVector& currentState,
        const ControlInput& control,
        const PlantPreparedParams& params,
        float batteryVoltageV) noexcept
    {
        AppliedTorqueEstimate estimate{};
        estimate.batteryVoltageAvailable = std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f);
        const float resolvedBatteryVoltageV = estimate.batteryVoltageAvailable ? batteryVoltageV : params.supplyVoltageV;

        const float leftWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaL)) ? currentState(VehicleState::kOmegaL) : 0.0f;
        const float rightWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaR)) ? currentState(VehicleState::kOmegaR) : 0.0f;
        const float leftMotorCommand = std::isfinite(control.leftMotorCommand) ? control.leftMotorCommand : 0.0f;
        const float rightMotorCommand = std::isfinite(control.rightMotorCommand) ? control.rightMotorCommand : 0.0f;

        estimate.leftAppliedBankTorqueNm =
            plant.driveTorqueFromCommand(leftMotorCommand, leftWheelSpeedRadps, resolvedBatteryVoltageV, params);
        estimate.rightAppliedBankTorqueNm =
            plant.driveTorqueFromCommand(rightMotorCommand, rightWheelSpeedRadps, resolvedBatteryVoltageV, params);

        estimate.leftCurrentLimited =
            estimate.batteryVoltageAvailable && (std::fabs(leftMotorCommand) >= 0.999f);
        estimate.rightCurrentLimited =
            estimate.batteryVoltageAvailable && (std::fabs(rightMotorCommand) >= 0.999f);
        return estimate;
    }
}
