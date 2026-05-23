
#include "pch.h"
#include "PlantModel.h"

#include "MotorEncoderDrive.h"
#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#ifndef MAZEMAP_PLANTMODEL_VALIDATE_INVERSE
#define MAZEMAP_PLANTMODEL_VALIDATE_INVERSE 0
#endif

namespace
{
    using MazeMap::MotorEncoderDrive;
    using MazeMap::PlantModel;
    using MazeMap::Vehicle;
    using MazeMap::VehicleState;
    using CommandVector = MazeMap::App::Internal::CommandVector;

    constexpr uint8_t kFrontLeft = 0U;
    constexpr uint8_t kFrontRight = 1U;
    constexpr uint8_t kRearLeft = 2U;
    constexpr uint8_t kRearRight = 3U;
    constexpr float kSignEpsilon = 1.0e-6f;
    constexpr float kForceEpsilonN = 1.0e-4f;
    constexpr float kRollingSpeedRegularizationMps = 0.05f;
    constexpr float kRollingFrictionTorqueNm = 0.00372f;
    constexpr float kReliableLaunchDriveCommand = 0.30f;
    constexpr float kStaticFrictionMaxSpeedMps = 0.005f;
    constexpr float kViscousFrictionNmPerRadps = 0.0f;
    constexpr float kLateralVelocityDampingNsPerM = 0.0f;
    constexpr float kYawRateDampingNmsPerRad = 0.0f;
    constexpr float kFrontLoadFraction = 0.5f;
    constexpr float kFrontLongitudinalForceSplit = 0.5f;
    constexpr float kMuFront = 1.65f;
    constexpr float kMuRear = 1.65f;
    constexpr float kStopEnterSpeedMps = 0.02f;
    constexpr float kStopExitSpeedMps = 0.05f;
    constexpr float kStopEnterYawRateRadps = 0.20f;
    constexpr float kStopExitYawRateRadps = 0.50f;
    constexpr float kStopEnterWheelSpeedRadps = 2.0f;
    constexpr float kStopExitWheelSpeedRadps = 5.0f;
    constexpr float kStopEnterCommand = 0.03f;
    constexpr float kStopExitCommand = 0.06f;

    bool EmitPlantDebugTextLine(
        void* context,
        const PlantModel::DebugTextSink sink,
        const char* type,
        const char* format,
        ...) noexcept
    {
        if (sink == nullptr || type == nullptr || type[0] == '\0' || format == nullptr)
        {
            return false;
        }

        std::va_list args;
        va_start(args, format);
        const bool ok = sink(context, type, format, args);
        va_end(args);
        return ok;
    }

    inline float ResolvePositiveCalibration(float value) noexcept
    {
        return (std::isfinite(value) && (value > 0.0f)) ? value : 0.0f;
    }

    inline float ClampUnit(float value) noexcept
    {
        return (std::clamp)(value, -1.0f, 1.0f);
    }

    inline float Clamp01(float value) noexcept
    {
        return (std::clamp)(value, 0.0f, 1.0f);
    }

    inline CommandVector ZeroControlVector() noexcept
    {
        return CommandVector(0.0f, 0.0f);
    }

    inline float SignedDirectionFast(float preferredValue, float fallbackValue) noexcept
    {
        const int preferredSign = (preferredValue > kSignEpsilon) - (preferredValue < -kSignEpsilon);
        if (preferredSign != 0)
        {
            return static_cast<float>(preferredSign);
        }

        const int fallbackSign = (fallbackValue > kSignEpsilon) - (fallbackValue < -kSignEpsilon);
        return static_cast<float>(fallbackSign);
    }

    inline float ResolvePhysicalTrackWidthM(const Vehicle& vehicle) noexcept
    {
        const float physicalTrackWidthM = vehicle.GetTrackWidth();
        return
            (std::isfinite(physicalTrackWidthM) && (physicalTrackWidthM > 0.0f)) ?
            physicalTrackWidthM :
            0.0f;
    }

    inline float ResolveTractionLimitedReserveScale(float tractionReserveScale) noexcept
    {
        return
            (std::isfinite(tractionReserveScale) &&
             (tractionReserveScale > 0.0f) &&
             (tractionReserveScale <= 1.0f)) ?
            tractionReserveScale :
            PlantModel::kTractionLimitedReserveScale;
    }

    inline float SmoothStep(float edge0, float edge1, float value) noexcept
    {
        if (!(std::isfinite(edge0) && std::isfinite(edge1) && std::isfinite(value)))
        {
            return 0.0f;
        }
        if (edge1 <= edge0)
        {
            return (value >= edge1) ? 1.0f : 0.0f;
        }

        const float t = Clamp01((value - edge0) / (edge1 - edge0));
        return t * t * (3.0f - (2.0f * t));
    }

    inline float ComputeRegularizedLongitudinalSpeedMps(
        float longitudinalSpeedMps,
        float slipSpeedFloorMps) noexcept
    {
        const float resolvedLongitudinalSpeedMps =
            std::isfinite(longitudinalSpeedMps) ? longitudinalSpeedMps : 0.0f;
        const float resolvedSlipSpeedFloorMps =
            (std::isfinite(slipSpeedFloorMps) && (slipSpeedFloorMps > 0.0f)) ?
            slipSpeedFloorMps :
            0.0f;
        return MazeMap::Math::Sqrtf(
            (resolvedLongitudinalSpeedMps * resolvedLongitudinalSpeedMps) +
            (resolvedSlipSpeedFloorMps * resolvedSlipSpeedFloorMps));
    }

    inline float ComputeLateralScrubWeight(
        float rightVelocityMps,
        float yawRateRadps) noexcept
    {
        const float longitudinalOffsetM =
            std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
        const float frontLateralSpeedMps = rightVelocityMps + (longitudinalOffsetM * yawRateRadps);
        const float rearLateralSpeedMps = rightVelocityMps - (longitudinalOffsetM * yawRateRadps);
        const float lateralScrubSpeedMps =
            (std::max)(std::fabs(frontLateralSpeedMps), std::fabs(rearLateralSpeedMps));
        return
            std::isfinite(lateralScrubSpeedMps) ?
            SmoothStep(
                kStopEnterSpeedMps,
                kStopExitSpeedMps,
                lateralScrubSpeedMps) :
            0.0f;
    }

    inline float ComputeSpeedNormMps(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps) noexcept
    {
        const float halfTrackWidthM = 0.5f * std::fabs(Vehicle::GetPhysicalModel().trackWidthM);
        const float longitudinalOffsetM =
            std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
        const float yawLeverArmM = (std::max)(longitudinalOffsetM, halfTrackWidthM);
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float yawSpeedMps = std::fabs(yawRateRadps * yawLeverArmM);
        const float leftWheelSpeedMps = std::fabs(wheelRadiusM * omegaLeftRadps);
        const float rightWheelSpeedMps = std::fabs(wheelRadiusM * omegaRightRadps);
        return
            (std::max)(
                (std::max)(
                    (std::max)(std::fabs(forwardVelocityMps), std::fabs(rightVelocityMps)),
                    yawSpeedMps),
                (std::max)(leftWheelSpeedMps, rightWheelSpeedMps));
    }

    inline bool IsStoppedFast(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm) noexcept
    {
        return
            (ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps) < kStopEnterSpeedMps) &&
            (std::fabs(yawRateRadps) < kStopEnterYawRateRadps) &&
            (std::fabs(omegaLeftRadps) < kStopEnterWheelSpeedRadps) &&
            (std::fabs(omegaRightRadps) < kStopEnterWheelSpeedRadps) &&
            (commandNorm < kStopEnterCommand) &&
            (ComputeLateralScrubWeight(rightVelocityMps, yawRateRadps) <= 0.0f);
    }

    inline bool ShouldSnapToZeroFast(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm) noexcept
    {
        return
            (ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps) < kStopEnterSpeedMps) &&
            (std::fabs(yawRateRadps) < kStopEnterYawRateRadps) &&
            (std::fabs(omegaLeftRadps) < kStopEnterWheelSpeedRadps) &&
            (std::fabs(omegaRightRadps) < kStopEnterWheelSpeedRadps) &&
            (commandNorm < kStopEnterCommand);
    }

    inline bool ShouldReportStoppedDiagnosticsFast(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps) noexcept
    {
        return
            (ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps) < kStopEnterSpeedMps) &&
            (std::fabs(yawRateRadps) < kStopEnterYawRateRadps) &&
            (std::fabs(omegaLeftRadps) < kStopEnterWheelSpeedRadps) &&
            (std::fabs(omegaRightRadps) < kStopEnterWheelSpeedRadps);
    }

    inline float ResolveRollingMotionWeight(
        float forwardVelocityMps,
        float rightVelocityMps,
        float yawRateRadps,
        float omegaLeftRadps,
        float omegaRightRadps,
        float commandNorm) noexcept
    {
        const float speedNormMps =
            ComputeSpeedNormMps(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps);
        const float speedWeight =
            SmoothStep(kStopEnterSpeedMps, kStopExitSpeedMps, speedNormMps);
        const float yawRateWeight =
            SmoothStep(kStopEnterYawRateRadps, kStopExitYawRateRadps, std::fabs(yawRateRadps));
        const float commandWeight =
            SmoothStep(kStopEnterCommand, kStopExitCommand, commandNorm);
        const float rollWeight = (std::max)((std::max)(speedWeight, yawRateWeight), commandWeight);
        const float lateralScrubWeight = ComputeLateralScrubWeight(rightVelocityMps, yawRateRadps);
        return (std::max)(rollWeight, lateralScrubWeight);
    }

    inline float ReserveScaledDemandWithRequestedSign(
        float requestedDemand,
        float achievedDemand,
        float reserveScale) noexcept
    {
        if (!std::isfinite(requestedDemand) ||
            !std::isfinite(achievedDemand) ||
            !std::isfinite(reserveScale) ||
            !(reserveScale > 0.0f))
        {
            return 0.0f;
        }

        const float requestedSign = SignedDirectionFast(requestedDemand, 0.0f);
        if (requestedSign == 0.0f)
        {
            return 0.0f;
        }

        const float reservedMagnitude =
            reserveScale *
            (std::min)(
                std::fabs(requestedDemand),
                std::fabs(achievedDemand));
        return requestedSign * reservedMagnitude;
    }

    inline float ClampMagnitudeFast(float value, float limit) noexcept
    {
        const float magnitudeLimit =
            (std::isfinite(limit) && (limit > 0.0f)) ? limit : 0.0f;
        return (magnitudeLimit > 0.0f) ?
            (std::clamp)(value, -magnitudeLimit, magnitudeLimit) :
            value;
    }

} // namespace

namespace
{
    inline float DriveFrictionTorqueFast(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        float staticFrictionTorqueNm) noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float staticFrictionSpeedThresholdRadps =
            (wheelRadiusM > 0.0f) ? (kStaticFrictionMaxSpeedMps / wheelRadiusM) : 0.0f;
        const float viscousFrictionTorqueNm = kViscousFrictionNmPerRadps * wheelBankSpeedRadps;
        if (std::fabs(wheelBankSpeedRadps) <= staticFrictionSpeedThresholdRadps)
        {
            const float sign = SignedDirectionFast(wheelTorqueRequestNm, wheelBankSpeedRadps);
            return (staticFrictionTorqueNm * sign) + viscousFrictionTorqueNm;
        }

        const float sign = SignedDirectionFast(wheelBankSpeedRadps, wheelTorqueRequestNm);
        return (kRollingFrictionTorqueNm * sign) + viscousFrictionTorqueNm;
    }

    inline float SmoothDirection(float value, float epsilon) noexcept
    {
        const float resolvedEpsilon =
            (std::isfinite(epsilon) && (epsilon > 0.0f)) ?
            epsilon :
            1.0e-4f;
        const float denominator = MazeMap::Math::Sqrtf((value * value) + (resolvedEpsilon * resolvedEpsilon));
        return (denominator > 0.0f) ? (value / denominator) : 0.0f;
    }

    inline float DriveFrictionTorqueSmooth(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm,
        float staticFrictionTorqueNm) noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float invWheelRadiusM = (wheelRadiusM > 0.0f) ? (1.0f / wheelRadiusM) : 0.0f;
        const float staticFrictionSpeedThresholdRadps =
            (wheelRadiusM > 0.0f) ? (kStaticFrictionMaxSpeedMps * invWheelRadiusM) : 0.0f;
        const float viscousFrictionTorqueNm = kViscousFrictionNmPerRadps * wheelBankSpeedRadps;
        const float speedScaleRadps =
            (std::max)(
                1.0e-4f,
                (std::max)(
                    staticFrictionSpeedThresholdRadps,
                    kRollingSpeedRegularizationMps * invWheelRadiusM));
        const float rollingBlend =
            SmoothStep(speedScaleRadps, 2.0f * speedScaleRadps, std::fabs(wheelBankSpeedRadps));
        const float staticBlend = 1.0f - rollingBlend;
        const float staticDirection = SmoothDirection(wheelTorqueRequestNm, speedScaleRadps);
        const float rollingDirection = SmoothDirection(wheelBankSpeedRadps, speedScaleRadps);
        const float coulombTorqueNm =
            (staticBlend * staticFrictionTorqueNm * staticDirection) +
            (rollingBlend * kRollingFrictionTorqueNm * rollingDirection);
        return coulombTorqueNm + viscousFrictionTorqueNm;
    }

} // namespace

namespace MazeMap
{
    PlantModel::PlantModel(const Vehicle& vehicle, VehicleState& runtimeState) noexcept
        : _vehicle(vehicle)
        , _runtimeState(runtimeState)
        , _leftDrive(vehicle.GetLeftMotorEncoderDrive())
        , _rightDrive(vehicle.GetRightMotorEncoderDrive())
    {
    }

    bool PlantModel::WriteUkfPlantDebugTextDump(void* context, DebugTextSink sink) const noexcept
    {
        if (sink == nullptr)
        {
            return false;
        }

        const float massKg = _vehicle.GetMass();
        const float trackWidthM = _vehicle.GetTrackWidth();
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float contactPatchLongitudinalOffsetM =
            Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM;
        const float equivalentWheelInertiaKgM2 =
            0.5f * (_leftDrive.getEquivalentWheelInertiaKgM2() + _rightDrive.getEquivalentWheelInertiaKgM2());
        const float motorCurrentLimitA =
            (_leftDrive.getResistance() > 0.0f) ?
            (_leftDrive.getVoltage() / _leftDrive.getResistance()) :
            2.4f;
        const float staticFrictionTorqueNm =
            (std::max)(
                0.0f,
                _leftDrive.getTorqueFromCommand(
                    kReliableLaunchDriveCommand,
                    0.0f,
                    _vehicle.GetBatteryVoltage()));

        if (!EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_mass_geometry",
                "mass_kg=%.9g;effective_longitudinal_mass_kg=%.9g;yaw_inertia_kg_m2=%.9g;track_width_m=%.9g;contact_patch_longitudinal_offset_m=%.9g;wheel_radius_m=%.9g;equivalent_wheel_inertia_kg_m2=%.9g",
                static_cast<double>(massKg),
                static_cast<double>(massKg),
                static_cast<double>(_vehicle.GetYawInertia()),
                static_cast<double>(trackWidthM),
                static_cast<double>(contactPatchLongitudinalOffsetM),
                static_cast<double>(Vehicle::GetDriveWheelRadiusM()),
                static_cast<double>(equivalentWheelInertiaKgM2)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_drive_electrical",
                "supply_voltage_v=%.9g;drive_resistance_ohms=%.9g;torque_constant_nm_per_a=%.9g;speed_constant_radps_per_volt=%.9g;no_load_current_a=%.9g;motor_current_limit_a=%.9g;gear_ratio=%.9g;encoder_counts_per_motor_rev=%u",
                static_cast<double>(_vehicle.GetBatteryVoltage()),
                static_cast<double>(_leftDrive.getResistance()),
                static_cast<double>(_leftDrive.getTorqueConstant()),
                static_cast<double>(_leftDrive.getSpeedConstant()),
                static_cast<double>(_leftDrive.getNoLoadCurrent()),
                static_cast<double>(motorCurrentLimitA),
                static_cast<double>(_leftDrive.getGearRatio()),
                static_cast<unsigned>(_leftDrive.getPulsesPerRev())) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_tire_friction",
                "drivetrain_efficiency=%.9g;rolling_friction_torque_nm=%.9g;viscous_friction_nm_per_radps=%.9g;longitudinal_tire_stiffness_n=%.9g;mu_front=%.9g;mu_rear=%.9g;front_load_fraction=%.9g",
                1.0,
                static_cast<double>(kRollingFrictionTorqueNm),
                static_cast<double>(kViscousFrictionNmPerRadps),
                static_cast<double>(
                    0.5f * (_leftDrive.getLongitudinalTireStiffnessN() + _rightDrive.getLongitudinalTireStiffnessN())),
                static_cast<double>(kMuFront),
                static_cast<double>(kMuRear),
                static_cast<double>(kFrontLoadFraction)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_static_friction",
                "static_friction_torque_nm=%.9g;static_friction_max_speed_mps=%.9g",
                static_cast<double>(staticFrictionTorqueNm),
                static_cast<double>(kStaticFrictionMaxSpeedMps)) ||
            !EmitPlantDebugTextLine(
                context,
                sink,
                "ukf_dump_params_misc",
                "velocity_epsilon_mps=%.9g;force_epsilon_n=%.9g;fan_downforce_at_full_duty_n=%.9g;no_hit_range_m=%.9g",
                static_cast<double>(kRollingSpeedRegularizationMps),
                static_cast<double>(kForceEpsilonN),
                static_cast<double>(_vehicle.GetFanDownforceAtFullDuty()),
                static_cast<double>(_vehicle.FrontLeft.GetNoHitRangeM())))
        {
            return false;
        }

        const std::array<Eigen::Vector2f, 4> contactPositionsBodyM = {
            Eigen::Vector2f(-halfTrackWidthM, std::fabs(contactPatchLongitudinalOffsetM)),
            Eigen::Vector2f(halfTrackWidthM, std::fabs(contactPatchLongitudinalOffsetM)),
            Eigen::Vector2f(-halfTrackWidthM, -std::fabs(contactPatchLongitudinalOffsetM)),
            Eigen::Vector2f(halfTrackWidthM, -std::fabs(contactPatchLongitudinalOffsetM))
        };
        for (std::size_t index = 0; index < contactPositionsBodyM.size(); ++index)
        {
            const Eigen::Vector2f& position = contactPositionsBodyM[index];
            if (!EmitPlantDebugTextLine(
                    context,
                    sink,
                    "ukf_dump_contact_position",
                    "index=%u;x_m=%.9g;y_m=%.9g",
                    static_cast<unsigned>(index),
                    static_cast<double>(position.x()),
                    static_cast<double>(position.y())))
            {
                return false;
            }
        }

        const auto emitSensorMount =
            [&](const char* type, const SensorMount& sensor) noexcept
        {
            const Eigen::Matrix2f& bodyFromSensor = sensor.bodyFromSensor();
            return EmitPlantDebugTextLine(
                context,
                sink,
                type,
                "position_x_m=%.9g;position_y_m=%.9g;body_from_sensor_00=%.9g;body_from_sensor_01=%.9g;body_from_sensor_10=%.9g;body_from_sensor_11=%.9g;clockwise_yaw_sign=%.9g",
                static_cast<double>(sensor.positionBodyM().x()),
                static_cast<double>(sensor.positionBodyM().y()),
                static_cast<double>(bodyFromSensor(0, 0)),
                static_cast<double>(bodyFromSensor(0, 1)),
                static_cast<double>(bodyFromSensor(1, 0)),
                static_cast<double>(bodyFromSensor(1, 1)),
                static_cast<double>(sensor.clockwiseYawSign()));
        };
        return
            emitSensorMount("ukf_dump_sensor_front_left", Vehicle::GetFrontLeftSensorMount()) &&
            emitSensorMount("ukf_dump_sensor_front_right", Vehicle::GetFrontRightSensorMount()) &&
            emitSensorMount("ukf_dump_sensor_side_left", Vehicle::GetSideLeftSensorMount()) &&
            emitSensorMount("ukf_dump_sensor_side_right", Vehicle::GetSideRightSensorMount()) &&
            emitSensorMount("ukf_dump_imu_mount", Vehicle::GetBackLeftImuMount());
    }

    PlantModel::StateVector PlantModel::BuildBoundStateVector() const noexcept
    {
        StateVector state = StateVector::Zero();
        state(VehicleState::kPx) = _runtimeState.GetPositionX();
        state(VehicleState::kPy) = _runtimeState.GetPositionY();
        state(VehicleState::kPsi) = _runtimeState.GetOrientation();
        state(VehicleState::kU) = _runtimeState.GetVelocity();
        state(VehicleState::kV) = _runtimeState.GetLateralVelocity();
        state(VehicleState::kR) = _runtimeState.GetRotationalVelocity();
        state(VehicleState::kOmegaL) = _runtimeState.GetWheelSpeedLeft();
        state(VehicleState::kOmegaR) = _runtimeState.GetWheelSpeedRight();
        state(VehicleState::kBgz) = _runtimeState.GetGyroBiasZ();
        state(VehicleState::kPsi) = NormalizeAngle(state(VehicleState::kPsi));
        return state;
    }

    void PlantModel::ApplyStateVectorToBoundState(const StateVector& state) noexcept
    {
        _runtimeState.SetPosition(Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)));
        _runtimeState.SetOrientation(state(VehicleState::kPsi));
        _runtimeState.SetVelocity(state(VehicleState::kU));
        _runtimeState.SetLateralVelocity(state(VehicleState::kV));
        _runtimeState.SetRotationalVelocity(state(VehicleState::kR));
        _runtimeState.SetWheelSpeedLeft(state(VehicleState::kOmegaL));
        _runtimeState.SetWheelSpeedRight(state(VehicleState::kOmegaR));
        _runtimeState.SetGyroBiasZ(state(VehicleState::kBgz));
    }

    void PlantModel::resolveAppliedBankTorques(
        const StateVector& currentState,
        const App::Internal::CommandVector& control,
        float& leftAppliedBankTorqueNm,
        float& rightAppliedBankTorqueNm) const noexcept
    {
        const float leftWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaL)) ? currentState(VehicleState::kOmegaL) : 0.0f;
        const float rightWheelSpeedRadps =
            std::isfinite(currentState(VehicleState::kOmegaR)) ? currentState(VehicleState::kOmegaR) : 0.0f;
        const float leftMotorCommand = std::isfinite(control.LeftCommand()) ? control.LeftCommand() : 0.0f;
        const float rightMotorCommand = std::isfinite(control.RightCommand()) ? control.RightCommand() : 0.0f;
        const float batteryVoltageV = _vehicle.GetBatteryVoltage();

        leftAppliedBankTorqueNm =
            _leftDrive.getTorqueFromCommand(
                leftMotorCommand,
                leftWheelSpeedRadps,
                batteryVoltageV);
        rightAppliedBankTorqueNm =
            _rightDrive.getTorqueFromCommand(
                rightMotorCommand,
                rightWheelSpeedRadps,
                batteryVoltageV);
    }

    PlantModel::PlantDerivatives PlantModel::forwardStep(
        const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(BuildBoundStateVector(), control);
    }

    PlantModel::PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const App::Internal::CommandVector& control) const noexcept
    {
        float leftDriveTorqueNm = 0.0f;
        float rightDriveTorqueNm = 0.0f;
        resolveAppliedBankTorques(
            state,
            control,
            leftDriveTorqueNm,
            rightDriveTorqueNm);
        const float activityNorm =
            (std::max)(std::fabs(control.LeftCommand()), std::fabs(control.RightCommand()));
        return evaluateAppliedBankTorqueStep(
            state,
            leftDriveTorqueNm,
            rightDriveTorqueNm,
            activityNorm);
    }

    PlantModel::WheelOnlyMeasurementPrediction PlantModel::predictWheelOnlyMeasurement(
        const StateVector& state) const noexcept
    {
        WheelOnlyMeasurementPrediction prediction{};
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float trackWidthM =
            (std::isfinite(_vehicle.GetTrackWidth()) && (_vehicle.GetTrackWidth() > 0.0f)) ?
            _vehicle.GetTrackWidth() :
            0.0f;
        prediction.forwardSpeedMps = std::isfinite(state(VehicleState::kU)) ? state(VehicleState::kU) : 0.0f;
        prediction.yawRateRadps = std::isfinite(state(VehicleState::kR)) ? state(VehicleState::kR) : 0.0f;
        prediction.leftWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((prediction.forwardSpeedMps + (0.5f * trackWidthM * prediction.yawRateRadps)) / wheelRadiusM) :
            0.0f;
        prediction.rightWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((prediction.forwardSpeedMps - (0.5f * trackWidthM * prediction.yawRateRadps)) / wheelRadiusM) :
            0.0f;
        return prediction;
    }

    PlantModel::PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm) const noexcept
    {
        return evaluateAppliedBankTorqueStep(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            (std::max)(std::fabs(leftAppliedBankTorqueNm), std::fabs(rightAppliedBankTorqueNm)));
    }

    PlantModel::PlantDerivatives PlantModel::evaluateAppliedBankTorqueStep(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float activityNorm) const noexcept
    {
        const float fanDutyCycle = _vehicle.GetFanDuty();
        PlantDerivatives derivatives{};
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float invWheelRadiusM = (wheelRadiusM > 0.0f) ? (1.0f / wheelRadiusM) : 0.0f;
        const float staticFrictionTorqueNm =
            (std::max)(
                0.0f,
                _leftDrive.getTorqueFromCommand(
                    kReliableLaunchDriveCommand,
                    0.0f,
                    _vehicle.GetBatteryVoltage()));
        const float staticFrictionSpeedThresholdRadps =
            (wheelRadiusM > 0.0f) ? (kStaticFrictionMaxSpeedMps * invWheelRadiusM) : 0.0f;
        const float massKg =
            (std::isfinite(_vehicle.GetMass()) && (_vehicle.GetMass() > 0.0f)) ?
            _vehicle.GetMass() :
            1.0f;
        const float invMassKg = 1.0f / massKg;
        const float yawInertiaKgM2 =
            (std::isfinite(_vehicle.GetYawInertia()) && (_vehicle.GetYawInertia() > 0.0f)) ?
            _vehicle.GetYawInertia() :
            1.0f;
        const float invYawInertiaKgM2 = 1.0f / yawInertiaKgM2;
        const float wheelInertiaKgM2 =
            0.5f * (_leftDrive.getEquivalentWheelInertiaKgM2() + _rightDrive.getEquivalentWheelInertiaKgM2());
        const float resolvedWheelInertiaKgM2 =
            (std::isfinite(wheelInertiaKgM2) && (wheelInertiaKgM2 > 0.0f)) ?
            wheelInertiaKgM2 :
            1.0f;
        const float invWheelInertiaKgM2 = 1.0f / resolvedWheelInertiaKgM2;
        const float longitudinalTireStiffnessN =
            0.5f * (_leftDrive.getLongitudinalTireStiffnessN() + _rightDrive.getLongitudinalTireStiffnessN());
        const float lateralDampingOverMass = kLateralVelocityDampingNsPerM * invMassKg;
        const float yawDampingOverInertia = kYawRateDampingNmsPerRad * invYawInertiaKgM2;

        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float psi = state(VehicleState::kPsi);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);
        const float commandNorm =
            std::isfinite(activityNorm) ? std::fabs(activityNorm) : 0.0f;

        derivatives.wheelKinematics = wheelKinematics(state);
        const float frontLeftLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_leftDrive.getFrontLateralTireStiffnessNPerRad());
        const float frontRightLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_rightDrive.getFrontLateralTireStiffnessNPerRad());
        const float rearLeftLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_leftDrive.getRearLateralTireStiffnessNPerRad());
        const float rearRightLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_rightDrive.getRearLateralTireStiffnessNPerRad());
        const float contactMassKg = _vehicle.GetMass();
        const float totalNormalLoadN =
            (contactMassKg * GRAVITY_MPS2) + (Clamp01(fanDutyCycle) * _vehicle.GetFanDownforceAtFullDuty());
        const float frontWheelNormalLoadN = 0.5f * kFrontLoadFraction * totalNormalLoadN;
        const float rearWheelNormalLoadN = 0.5f * (1.0f - kFrontLoadFraction) * totalNormalLoadN;
        const float contactEnvelopeMu =
            (std::isfinite(_vehicle.GetPeakCombinedAcceleration()) &&
             (_vehicle.GetPeakCombinedAcceleration() > 0.0f) &&
             std::isfinite(contactMassKg) &&
             (contactMassKg > 0.0f)) ?
            ((_vehicle.GetPeakCombinedAcceleration() * contactMassKg) / (std::max)(totalNormalLoadN, kForceEpsilonN)) :
            0.0f;
        const float peakFrontMu = (contactEnvelopeMu > 0.0f) ? contactEnvelopeMu : kMuFront;
        const float peakRearMu = (contactEnvelopeMu > 0.0f) ? contactEnvelopeMu : kMuRear;
        const float sustainedAccelMps2 = _vehicle.GetMaxLateralAcceleration();
        const float lateralForceSustainedLimitN =
            (std::isfinite(sustainedAccelMps2) &&
             (sustainedAccelMps2 > 0.0f) &&
             std::isfinite(contactMassKg) &&
             (contactMassKg > 0.0f)) ?
            (sustainedAccelMps2 * contactMassKg) :
            0.0f;

        if (IsStoppedFast(
            forwardVelocityMps,
            rightVelocityMps,
            yawRateRadps,
            omegaLeftRadps,
            omegaRightRadps,
            commandNorm))
        {
            derivatives.contactForces.contacts[kFrontLeft].normalForceN = frontWheelNormalLoadN;
            derivatives.contactForces.contacts[kFrontRight].normalForceN = frontWheelNormalLoadN;
            derivatives.contactForces.contacts[kRearLeft].normalForceN = rearWheelNormalLoadN;
            derivatives.contactForces.contacts[kRearRight].normalForceN = rearWheelNormalLoadN;
            derivatives.regime = MotionRegime::StoppedHold;
            return derivatives;
        }

        const float motionWeight =
            ResolveRollingMotionWeight(
                forwardVelocityMps,
                rightVelocityMps,
                yawRateRadps,
                omegaLeftRadps,
                omegaRightRadps,
                commandNorm);

        ContactForces rollingContactForces{};
        float rollingMaxUtilization = 0.0f;
        float leftBankForwardForceN = 0.0f;
        float rightBankForwardForceN = 0.0f;
        float frontRightForceN = 0.0f;
        float rearRightForceN = 0.0f;
        float sumForwardForceN = 0.0f;
        float sumRightForceN = 0.0f;
        float leftBankForwardForceRawN = 0.0f;
        float rightBankForwardForceRawN = 0.0f;
        float frontLeftRightForceRawN = 0.0f;
        float frontRightRightForceRawN = 0.0f;
        float rearLeftRightForceRawN = 0.0f;
        float rearRightRightForceRawN = 0.0f;
        if (motionWeight > 0.0f)
        {
            const SlipTargets targets = slipTargets(state, derivatives.wheelKinematics);
            frontLeftRightForceRawN =
                -frontLeftLateralStiffnessNPerRad *
                std::atan(targets.lateralRatio[kFrontLeft]);
            frontRightRightForceRawN =
                -frontRightLateralStiffnessNPerRad *
                std::atan(targets.lateralRatio[kFrontRight]);
            rearLeftRightForceRawN =
                -rearLeftLateralStiffnessNPerRad *
                std::atan(targets.lateralRatio[kRearLeft]);
            rearRightRightForceRawN =
                -rearRightLateralStiffnessNPerRad *
                std::atan(targets.lateralRatio[kRearRight]);
            leftBankForwardForceRawN = longitudinalTireStiffnessN * targets.kappaLeft;
            rightBankForwardForceRawN = longitudinalTireStiffnessN * targets.kappaRight;
        }

        const float lambdaRear = 1.0f - kFrontLongitudinalForceSplit;
        const float fxFlRaw = kFrontLongitudinalForceSplit * leftBankForwardForceRawN;
        const float fxRlRaw = lambdaRear * leftBankForwardForceRawN;
        const float fxFrRaw = kFrontLongitudinalForceSplit * rightBankForwardForceRawN;
        const float fxRrRaw = lambdaRear * rightBankForwardForceRawN;
        const float fyFlRaw =
            ClampMagnitudeFast(frontLeftRightForceRawN, 0.5f * kFrontLoadFraction * lateralForceSustainedLimitN);
        const float fyFrRaw =
            ClampMagnitudeFast(frontRightRightForceRawN, 0.5f * kFrontLoadFraction * lateralForceSustainedLimitN);
        const float fyRlRaw =
            ClampMagnitudeFast(rearLeftRightForceRawN, 0.5f * (1.0f - kFrontLoadFraction) * lateralForceSustainedLimitN);
        const float fyRrRaw =
            ClampMagnitudeFast(rearRightRightForceRawN, 0.5f * (1.0f - kFrontLoadFraction) * lateralForceSustainedLimitN);

        ContactForce& fl = rollingContactForces.contacts[kFrontLeft];
        const float flRawMagnitudeN = MazeMap::Math::Sqrtf((fxFlRaw * fxFlRaw) + (fyFlRaw * fyFlRaw));
        const float flMaxForceN = (std::max)(0.0f, peakFrontMu * frontWheelNormalLoadN);
        const float flScale = (std::min)(flMaxForceN / (std::max)(flRawMagnitudeN, kForceEpsilonN), 1.0f);
        const float flSaturationDenominatorN =
            (flMaxForceN > kForceEpsilonN) ? flMaxForceN : kForceEpsilonN;
        fl.forwardForceN = flScale * fxFlRaw;
        fl.rightForceN = flScale * fyFlRaw;
        fl.normalForceN = frontWheelNormalLoadN;
        fl.preProjectionUtilization = flRawMagnitudeN / flSaturationDenominatorN;
        fl.saturation = (fl.preProjectionUtilization < 1.0f) ? fl.preProjectionUtilization : 1.0f;

        ContactForce& fr = rollingContactForces.contacts[kFrontRight];
        const float frRawMagnitudeN = MazeMap::Math::Sqrtf((fxFrRaw * fxFrRaw) + (fyFrRaw * fyFrRaw));
        const float frMaxForceN = (std::max)(0.0f, peakFrontMu * frontWheelNormalLoadN);
        const float frScale = (std::min)(frMaxForceN / (std::max)(frRawMagnitudeN, kForceEpsilonN), 1.0f);
        const float frSaturationDenominatorN =
            (frMaxForceN > kForceEpsilonN) ? frMaxForceN : kForceEpsilonN;
        fr.forwardForceN = frScale * fxFrRaw;
        fr.rightForceN = frScale * fyFrRaw;
        fr.normalForceN = frontWheelNormalLoadN;
        fr.preProjectionUtilization = frRawMagnitudeN / frSaturationDenominatorN;
        fr.saturation = (fr.preProjectionUtilization < 1.0f) ? fr.preProjectionUtilization : 1.0f;

        ContactForce& rl = rollingContactForces.contacts[kRearLeft];
        const float rlRawMagnitudeN = MazeMap::Math::Sqrtf((fxRlRaw * fxRlRaw) + (fyRlRaw * fyRlRaw));
        const float rlMaxForceN = (std::max)(0.0f, peakRearMu * rearWheelNormalLoadN);
        const float rlScale = (std::min)(rlMaxForceN / (std::max)(rlRawMagnitudeN, kForceEpsilonN), 1.0f);
        const float rlSaturationDenominatorN =
            (rlMaxForceN > kForceEpsilonN) ? rlMaxForceN : kForceEpsilonN;
        rl.forwardForceN = rlScale * fxRlRaw;
        rl.rightForceN = rlScale * fyRlRaw;
        rl.normalForceN = rearWheelNormalLoadN;
        rl.preProjectionUtilization = rlRawMagnitudeN / rlSaturationDenominatorN;
        rl.saturation = (rl.preProjectionUtilization < 1.0f) ? rl.preProjectionUtilization : 1.0f;

        ContactForce& rr = rollingContactForces.contacts[kRearRight];
        const float rrRawMagnitudeN = MazeMap::Math::Sqrtf((fxRrRaw * fxRrRaw) + (fyRrRaw * fyRrRaw));
        const float rrMaxForceN = (std::max)(0.0f, peakRearMu * rearWheelNormalLoadN);
        const float rrScale = (std::min)(rrMaxForceN / (std::max)(rrRawMagnitudeN, kForceEpsilonN), 1.0f);
        const float rrSaturationDenominatorN =
            (rrMaxForceN > kForceEpsilonN) ? rrMaxForceN : kForceEpsilonN;
        rr.forwardForceN = rrScale * fxRrRaw;
        rr.rightForceN = rrScale * fyRrRaw;
        rr.normalForceN = rearWheelNormalLoadN;
        rr.preProjectionUtilization = rrRawMagnitudeN / rrSaturationDenominatorN;
        rr.saturation = (rr.preProjectionUtilization < 1.0f) ? rr.preProjectionUtilization : 1.0f;

        leftBankForwardForceN = fl.forwardForceN + rl.forwardForceN;
        rightBankForwardForceN = fr.forwardForceN + rr.forwardForceN;
        frontRightForceN = fl.rightForceN + fr.rightForceN;
        rearRightForceN = rl.rightForceN + rr.rightForceN;
        sumForwardForceN = leftBankForwardForceN + rightBankForwardForceN;
        sumRightForceN = frontRightForceN + rearRightForceN;
        rollingMaxUtilization =
            (std::max)(
                (std::max)(fl.preProjectionUtilization, fr.preProjectionUtilization),
                (std::max)(rl.preProjectionUtilization, rr.preProjectionUtilization));

        const float contactYawHalfTrackWidthM = 0.5f * std::fabs(_vehicle.GetTrackWidth());
        const float contactYawLongitudinalOffsetM =
            std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
        float yawMomentNm =
            (contactYawHalfTrackWidthM *
                (leftBankForwardForceN - rightBankForwardForceN)) +
            (contactYawLongitudinalOffsetM *
                (frontRightForceN - rearRightForceN));
        float leftPreFrictionWheelTorqueNm =
            leftAppliedBankTorqueNm - (wheelRadiusM * leftBankForwardForceN);
        float rightPreFrictionWheelTorqueNm =
            rightAppliedBankTorqueNm - (wheelRadiusM * rightBankForwardForceN);
        const float leftViscousFrictionTorqueNm =
            kViscousFrictionNmPerRadps * omegaLeftRadps;
        const float rightViscousFrictionTorqueNm =
            kViscousFrictionNmPerRadps * omegaRightRadps;
        float leftFrictionTorqueNm = leftViscousFrictionTorqueNm;
        float rightFrictionTorqueNm = rightViscousFrictionTorqueNm;
        if ((std::fabs(omegaLeftRadps) <= staticFrictionSpeedThresholdRadps) &&
            (std::fabs(omegaRightRadps) <= staticFrictionSpeedThresholdRadps))
        {
            const int leftTorqueSign =
                (leftPreFrictionWheelTorqueNm > kSignEpsilon) -
                (leftPreFrictionWheelTorqueNm < -kSignEpsilon);
            const int leftSpeedSign =
                (omegaLeftRadps > kSignEpsilon) -
                (omegaLeftRadps < -kSignEpsilon);
            const int rightTorqueSign =
                (rightPreFrictionWheelTorqueNm > kSignEpsilon) -
                (rightPreFrictionWheelTorqueNm < -kSignEpsilon);
            const int rightSpeedSign =
                (omegaRightRadps > kSignEpsilon) -
                (omegaRightRadps < -kSignEpsilon);
            leftFrictionTorqueNm +=
                staticFrictionTorqueNm *
                static_cast<float>((leftTorqueSign != 0) ? leftTorqueSign : leftSpeedSign);
            rightFrictionTorqueNm +=
                staticFrictionTorqueNm *
                static_cast<float>((rightTorqueSign != 0) ? rightTorqueSign : rightSpeedSign);
        }
        else
        {
            const int leftSpeedSign =
                (omegaLeftRadps > kSignEpsilon) -
                (omegaLeftRadps < -kSignEpsilon);
            const int leftTorqueSign =
                (leftPreFrictionWheelTorqueNm > kSignEpsilon) -
                (leftPreFrictionWheelTorqueNm < -kSignEpsilon);
            leftFrictionTorqueNm +=
                kRollingFrictionTorqueNm *
                static_cast<float>((leftSpeedSign != 0) ? leftSpeedSign : leftTorqueSign);

            const int rightSpeedSign =
                (omegaRightRadps > kSignEpsilon) -
                (omegaRightRadps < -kSignEpsilon);
            const int rightTorqueSign =
                (rightPreFrictionWheelTorqueNm > kSignEpsilon) -
                (rightPreFrictionWheelTorqueNm < -kSignEpsilon);
            rightFrictionTorqueNm +=
                kRollingFrictionTorqueNm *
                static_cast<float>((rightSpeedSign != 0) ? rightSpeedSign : rightTorqueSign);
        }

        if ((commandNorm >= kStopEnterCommand) && (wheelRadiusM > kForceEpsilonN))
        {
            float leftDriveTorqueNm = leftAppliedBankTorqueNm - leftFrictionTorqueNm;
            float rightDriveTorqueNm = rightAppliedBankTorqueNm - rightFrictionTorqueNm;
            if ((std::fabs(omegaLeftRadps) <= staticFrictionSpeedThresholdRadps) &&
                (std::fabs(leftAppliedBankTorqueNm) <= staticFrictionTorqueNm))
            {
                leftDriveTorqueNm = 0.0f;
            }
            if ((std::fabs(omegaRightRadps) <= staticFrictionSpeedThresholdRadps) &&
                (std::fabs(rightAppliedBankTorqueNm) <= staticFrictionTorqueNm))
            {
                rightDriveTorqueNm = 0.0f;
            }

            const float halfTrackWidthM = 0.5f * std::fabs(_vehicle.GetTrackWidth());
            if (halfTrackWidthM > 0.0f)
            {
                const float lowSpeedWeight =
                    1.0f -
                    SmoothStep(
                        kStopEnterSpeedMps,
                        kStopExitSpeedMps,
                        ComputeSpeedNormMps(
                            forwardVelocityMps,
                            rightVelocityMps,
                            yawRateRadps,
                            omegaLeftRadps,
                            omegaRightRadps));
                const float driveYawMomentNm =
                    halfTrackWidthM * ((leftDriveTorqueNm - rightDriveTorqueNm) * invWheelRadiusM);
                const int driveYawSign =
                    (driveYawMomentNm > kSignEpsilon) - (driveYawMomentNm < -kSignEpsilon);
                const float yawScrubMomentNm =
                    lowSpeedWeight *
                    std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM) *
                    _vehicle.GetMass() *
                    Vehicle::GetSustainedLateralAccelerationReferenceMps2();
                const float appliedYawScrubMomentNm =
                    static_cast<float>(driveYawSign) *
                    (std::min)(std::fabs(driveYawMomentNm), yawScrubMomentNm);
                const float bankScrubTorqueNm =
                    (0.5f * wheelRadiusM * appliedYawScrubMomentNm) / halfTrackWidthM;
                leftDriveTorqueNm -= bankScrubTorqueNm;
                rightDriveTorqueNm += bankScrubTorqueNm;
            }

            frontLeftRightForceRawN = rollingContactForces.contacts[kFrontLeft].rightForceN;
            frontRightRightForceRawN = rollingContactForces.contacts[kFrontRight].rightForceN;
            rearLeftRightForceRawN = rollingContactForces.contacts[kRearLeft].rightForceN;
            rearRightRightForceRawN = rollingContactForces.contacts[kRearRight].rightForceN;

            leftBankForwardForceRawN = leftDriveTorqueNm * invWheelRadiusM;
            rightBankForwardForceRawN = rightDriveTorqueNm * invWheelRadiusM;
            if (halfTrackWidthM > 0.0f)
            {
                const float wheelMassKg = resolvedWheelInertiaKgM2 * invWheelRadiusM * invWheelRadiusM;
                const float yawAccelerationForceScale = (halfTrackWidthM * halfTrackWidthM) * invYawInertiaKgM2;
                const float diagonal =
                    1.0f + (wheelMassKg * (invMassKg + yawAccelerationForceScale));
                const float offDiagonal =
                    wheelMassKg * (invMassKg - yawAccelerationForceScale);
                const float determinant = (diagonal * diagonal) - (offDiagonal * offDiagonal);
                if (std::isfinite(determinant) && (std::fabs(determinant) > kForceEpsilonN))
                {
                    const float unsprungLeftBankForwardForceRawN =
                        ((diagonal * leftBankForwardForceRawN) -
                         (offDiagonal * rightBankForwardForceRawN)) /
                        determinant;
                    const float unsprungRightBankForwardForceRawN =
                        ((diagonal * rightBankForwardForceRawN) -
                         (offDiagonal * leftBankForwardForceRawN)) /
                        determinant;
                    leftBankForwardForceRawN = unsprungLeftBankForwardForceRawN;
                    rightBankForwardForceRawN = unsprungRightBankForwardForceRawN;
                }
            }

            const float updatedFxFlRaw = kFrontLongitudinalForceSplit * leftBankForwardForceRawN;
            const float updatedFxRlRaw = lambdaRear * leftBankForwardForceRawN;
            const float updatedFxFrRaw = kFrontLongitudinalForceSplit * rightBankForwardForceRawN;
            const float updatedFxRrRaw = lambdaRear * rightBankForwardForceRawN;
            const float updatedFyFlRaw =
                ClampMagnitudeFast(frontLeftRightForceRawN, 0.5f * kFrontLoadFraction * lateralForceSustainedLimitN);
            const float updatedFyFrRaw =
                ClampMagnitudeFast(frontRightRightForceRawN, 0.5f * kFrontLoadFraction * lateralForceSustainedLimitN);
            const float updatedFyRlRaw =
                ClampMagnitudeFast(rearLeftRightForceRawN, 0.5f * (1.0f - kFrontLoadFraction) * lateralForceSustainedLimitN);
            const float updatedFyRrRaw =
                ClampMagnitudeFast(rearRightRightForceRawN, 0.5f * (1.0f - kFrontLoadFraction) * lateralForceSustainedLimitN);

            const float updatedFlRawMagnitudeN =
                MazeMap::Math::Sqrtf((updatedFxFlRaw * updatedFxFlRaw) + (updatedFyFlRaw * updatedFyFlRaw));
            const float updatedFlMaxForceN = (std::max)(0.0f, peakFrontMu * frontWheelNormalLoadN);
            const float updatedFlScale =
                (std::min)(updatedFlMaxForceN / (std::max)(updatedFlRawMagnitudeN, kForceEpsilonN), 1.0f);
            const float updatedFlSaturationDenominatorN =
                (updatedFlMaxForceN > kForceEpsilonN) ? updatedFlMaxForceN : kForceEpsilonN;
            fl.forwardForceN = updatedFlScale * updatedFxFlRaw;
            fl.rightForceN = updatedFlScale * updatedFyFlRaw;
            fl.normalForceN = frontWheelNormalLoadN;
            fl.preProjectionUtilization = updatedFlRawMagnitudeN / updatedFlSaturationDenominatorN;
            fl.saturation = (fl.preProjectionUtilization < 1.0f) ? fl.preProjectionUtilization : 1.0f;

            const float updatedFrRawMagnitudeN =
                MazeMap::Math::Sqrtf((updatedFxFrRaw * updatedFxFrRaw) + (updatedFyFrRaw * updatedFyFrRaw));
            const float updatedFrMaxForceN = (std::max)(0.0f, peakFrontMu * frontWheelNormalLoadN);
            const float updatedFrScale =
                (std::min)(updatedFrMaxForceN / (std::max)(updatedFrRawMagnitudeN, kForceEpsilonN), 1.0f);
            const float updatedFrSaturationDenominatorN =
                (updatedFrMaxForceN > kForceEpsilonN) ? updatedFrMaxForceN : kForceEpsilonN;
            fr.forwardForceN = updatedFrScale * updatedFxFrRaw;
            fr.rightForceN = updatedFrScale * updatedFyFrRaw;
            fr.normalForceN = frontWheelNormalLoadN;
            fr.preProjectionUtilization = updatedFrRawMagnitudeN / updatedFrSaturationDenominatorN;
            fr.saturation = (fr.preProjectionUtilization < 1.0f) ? fr.preProjectionUtilization : 1.0f;

            const float updatedRlRawMagnitudeN =
                MazeMap::Math::Sqrtf((updatedFxRlRaw * updatedFxRlRaw) + (updatedFyRlRaw * updatedFyRlRaw));
            const float updatedRlMaxForceN = (std::max)(0.0f, peakRearMu * rearWheelNormalLoadN);
            const float updatedRlScale =
                (std::min)(updatedRlMaxForceN / (std::max)(updatedRlRawMagnitudeN, kForceEpsilonN), 1.0f);
            const float updatedRlSaturationDenominatorN =
                (updatedRlMaxForceN > kForceEpsilonN) ? updatedRlMaxForceN : kForceEpsilonN;
            rl.forwardForceN = updatedRlScale * updatedFxRlRaw;
            rl.rightForceN = updatedRlScale * updatedFyRlRaw;
            rl.normalForceN = rearWheelNormalLoadN;
            rl.preProjectionUtilization = updatedRlRawMagnitudeN / updatedRlSaturationDenominatorN;
            rl.saturation = (rl.preProjectionUtilization < 1.0f) ? rl.preProjectionUtilization : 1.0f;

            const float updatedRrRawMagnitudeN =
                MazeMap::Math::Sqrtf((updatedFxRrRaw * updatedFxRrRaw) + (updatedFyRrRaw * updatedFyRrRaw));
            const float updatedRrMaxForceN = (std::max)(0.0f, peakRearMu * rearWheelNormalLoadN);
            const float updatedRrScale =
                (std::min)(updatedRrMaxForceN / (std::max)(updatedRrRawMagnitudeN, kForceEpsilonN), 1.0f);
            const float updatedRrSaturationDenominatorN =
                (updatedRrMaxForceN > kForceEpsilonN) ? updatedRrMaxForceN : kForceEpsilonN;
            rr.forwardForceN = updatedRrScale * updatedFxRrRaw;
            rr.rightForceN = updatedRrScale * updatedFyRrRaw;
            rr.normalForceN = rearWheelNormalLoadN;
            rr.preProjectionUtilization = updatedRrRawMagnitudeN / updatedRrSaturationDenominatorN;
            rr.saturation = (rr.preProjectionUtilization < 1.0f) ? rr.preProjectionUtilization : 1.0f;

            leftBankForwardForceN = fl.forwardForceN + rl.forwardForceN;
            rightBankForwardForceN = fr.forwardForceN + rr.forwardForceN;
            frontRightForceN = fl.rightForceN + fr.rightForceN;
            rearRightForceN = rl.rightForceN + rr.rightForceN;
            sumForwardForceN = leftBankForwardForceN + rightBankForwardForceN;
            sumRightForceN = frontRightForceN + rearRightForceN;
            rollingMaxUtilization =
                (std::max)(
                    (std::max)(fl.preProjectionUtilization, fr.preProjectionUtilization),
                    (std::max)(rl.preProjectionUtilization, rr.preProjectionUtilization));
            yawMomentNm =
                (contactYawHalfTrackWidthM *
                    (leftBankForwardForceN - rightBankForwardForceN)) +
                (contactYawLongitudinalOffsetM *
                    (frontRightForceN - rearRightForceN));
            leftPreFrictionWheelTorqueNm =
                leftAppliedBankTorqueNm - (wheelRadiusM * leftBankForwardForceN);
            rightPreFrictionWheelTorqueNm =
                rightAppliedBankTorqueNm - (wheelRadiusM * rightBankForwardForceN);
        }

        float leftNetWheelTorqueNm = leftPreFrictionWheelTorqueNm - leftFrictionTorqueNm;
        float rightNetWheelTorqueNm = rightPreFrictionWheelTorqueNm - rightFrictionTorqueNm;
        if ((std::fabs(omegaLeftRadps) <= staticFrictionSpeedThresholdRadps) &&
            (std::fabs(leftPreFrictionWheelTorqueNm) <= staticFrictionTorqueNm) &&
            (SignedDirectionFast(leftPreFrictionWheelTorqueNm, 0.0f) != 0.0f))
        {
            leftNetWheelTorqueNm = 0.0f;
        }
        if ((std::fabs(omegaRightRadps) <= staticFrictionSpeedThresholdRadps) &&
            (std::fabs(rightPreFrictionWheelTorqueNm) <= staticFrictionTorqueNm) &&
            (SignedDirectionFast(rightPreFrictionWheelTorqueNm, 0.0f) != 0.0f))
        {
            rightNetWheelTorqueNm = 0.0f;
        }

        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(psi, s, c);

        const float pxDot =
            ((rightVelocityMps * c) + (forwardVelocityMps * s)) * motionWeight;
        const float pyDot =
            ((-rightVelocityMps * s) + (forwardVelocityMps * c)) * motionWeight;
        const float psiDot = yawRateRadps * motionWeight;
        const float uDot =
            ((yawRateRadps * rightVelocityMps) +
             (sumForwardForceN * invMassKg)) *
            motionWeight;
        const float vDot =
            ((-yawRateRadps * forwardVelocityMps) +
             (sumRightForceN * invMassKg) -
             (lateralDampingOverMass * rightVelocityMps)) *
            motionWeight;
        const float rDot =
            ((yawMomentNm * invYawInertiaKgM2) -
             (yawDampingOverInertia * yawRateRadps)) *
            motionWeight;
        const float omegaLDot = (leftNetWheelTorqueNm * invWheelInertiaKgM2) * motionWeight;
        const float omegaRDot = (rightNetWheelTorqueNm * invWheelInertiaKgM2) * motionWeight;

        derivatives.stateDot(VehicleState::kPx) = pxDot;
        derivatives.stateDot(VehicleState::kPy) = pyDot;
        derivatives.stateDot(VehicleState::kPsi) = psiDot;
        derivatives.stateDot(VehicleState::kU) = uDot;
        derivatives.stateDot(VehicleState::kV) = vDot;
        derivatives.stateDot(VehicleState::kR) = rDot;
        derivatives.stateDot(VehicleState::kOmegaL) = omegaLDot;
        derivatives.stateDot(VehicleState::kOmegaR) = omegaRDot;
        derivatives.stateDot(VehicleState::kBgz) = 0.0f;

        derivatives.contactForces = rollingContactForces;
        if (motionWeight <= 0.0f)
        {
            for (ContactForce& contact : derivatives.contactForces.contacts)
            {
                contact.forwardForceN = 0.0f;
                contact.rightForceN = 0.0f;
                contact.saturation = 0.0f;
                contact.preProjectionUtilization = 0.0f;
            }
        }
        else if (motionWeight < 1.0f)
        {
            for (ContactForce& contact : derivatives.contactForces.contacts)
            {
                contact.forwardForceN *= motionWeight;
                contact.rightForceN *= motionWeight;
                contact.saturation *= motionWeight;
                contact.preProjectionUtilization *= motionWeight;
            }
        }
        derivatives.maxContactUtilization =
            (motionWeight >= 1.0f) ? rollingMaxUtilization : (motionWeight * rollingMaxUtilization);

        if (motionWeight < 0.5f)
        {
            derivatives.regime = MotionRegime::StoppedHold;
            derivatives.slipTargets = SlipTargets{};
        }
        else
        {
            derivatives.slipTargets = slipTargets(state, derivatives.wheelKinematics);
            if (rollingMaxUtilization >= (1.0f - 1.0e-4f))
            {
                derivatives.regime = MotionRegime::RollingSaturated;
            }
            else
            {
                derivatives.regime = MotionRegime::RollingAdherent;
            }
        }

        const float originAccelRightMps2 = derivatives.stateDot(VehicleState::kV) + (yawRateRadps * forwardVelocityMps);
        const float originAccelForwardMps2 = derivatives.stateDot(VehicleState::kU) - (yawRateRadps * rightVelocityMps);
        derivatives.originAccelBodyMps2 = Eigen::Vector2f(originAccelRightMps2, originAccelForwardMps2);

        const Eigen::Vector2f imuLeverArmBodyM = Vehicle::GetBackLeftImuMount().positionBodyM();
        const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
        derivatives.imuAccelBodyMps2 = Eigen::Vector2f(
            originAccelRightMps2 -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (rDot * imuLeverArmBodyM.y()),
            originAccelForwardMps2 -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (rDot * imuLeverArmBodyM.x()));
        derivatives.longitudinalAccelMps2 = originAccelForwardMps2;
        derivatives.lateralAccelMps2 = originAccelRightMps2;
        derivatives.yawAccelRadps2 = rDot;
        return derivatives;
    }

    PlantModel::StateVector PlantModel::integrateAppliedBankTorques(
        const StateVector& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float dtS) const noexcept
    {
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return state;
        }

        const PlantDerivatives evaluatedStep =
            evaluateAppliedBankTorqueStep(
                state,
                leftAppliedBankTorqueNm,
                rightAppliedBankTorqueNm,
                (std::max)(std::fabs(leftAppliedBankTorqueNm), std::fabs(rightAppliedBankTorqueNm)));
        return advanceStateFromDerivatives(state, evaluatedStep, dtS);
    }

    PlantModel::StateVector PlantModel::advanceStateFromDerivatives(
        const StateVector& currentState,
        const PlantDerivatives& evaluatedStep,
        float dtS) noexcept
    {
        StateVector nextState = currentState;
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return nextState;
        }

        nextState(VehicleState::kOmegaL) += dtS * evaluatedStep.stateDot(VehicleState::kOmegaL);
        nextState(VehicleState::kOmegaR) += dtS * evaluatedStep.stateDot(VehicleState::kOmegaR);

        nextState(VehicleState::kU) += dtS * evaluatedStep.stateDot(VehicleState::kU);
        nextState(VehicleState::kV) += dtS * evaluatedStep.stateDot(VehicleState::kV);
        nextState(VehicleState::kR) += dtS * evaluatedStep.stateDot(VehicleState::kR);

        nextState(VehicleState::kPsi) =
            NormalizeAngle(
                currentState(VehicleState::kPsi) + (dtS * nextState(VehicleState::kR)));

        float sineHeading = 0.0f;
        float cosineHeading = 0.0f;
        sin_cosf(nextState(VehicleState::kPsi), sineHeading, cosineHeading);
        const float worldRightVelocityMps =
            (nextState(VehicleState::kV) * cosineHeading) +
            (nextState(VehicleState::kU) * sineHeading);
        const float worldForwardVelocityMps =
            (-nextState(VehicleState::kV) * sineHeading) +
            (nextState(VehicleState::kU) * cosineHeading);
        nextState(VehicleState::kPx) += dtS * worldRightVelocityMps;
        nextState(VehicleState::kPy) += dtS * worldForwardVelocityMps;

        nextState(VehicleState::kBgz) += dtS * evaluatedStep.stateDot(VehicleState::kBgz);
        nextState(VehicleState::kPsi) = NormalizeAngle(nextState(VehicleState::kPsi));
        return nextState;
    }

    PlantModel::WheelKinematics PlantModel::wheelKinematics(const StateVector& state) const noexcept
    {
        WheelKinematics kinematics{};
        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float yawRateRadps = state(VehicleState::kR);
        const float halfTrackWidthM = 0.5f * std::fabs(_vehicle.GetTrackWidth());
        const float longitudinalOffsetM =
            std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
        const float leftBankVelocityMps = forwardVelocityMps + (halfTrackWidthM * yawRateRadps);
        const float rightBankVelocityMps = forwardVelocityMps - (halfTrackWidthM * yawRateRadps);
        const float frontLateralVelocityMps = rightVelocityMps + (longitudinalOffsetM * yawRateRadps);
        const float rearLateralVelocityMps = rightVelocityMps - (longitudinalOffsetM * yawRateRadps);

        kinematics.leftBankForwardVelocityMps = leftBankVelocityMps;
        kinematics.rightBankForwardVelocityMps = rightBankVelocityMps;

        kinematics.contacts[kFrontLeft].forwardVelocityMps = leftBankVelocityMps;
        kinematics.contacts[kFrontRight].forwardVelocityMps = rightBankVelocityMps;
        kinematics.contacts[kRearLeft].forwardVelocityMps = leftBankVelocityMps;
        kinematics.contacts[kRearRight].forwardVelocityMps = rightBankVelocityMps;

        kinematics.contacts[kFrontLeft].rightVelocityMps = frontLateralVelocityMps;
        kinematics.contacts[kFrontRight].rightVelocityMps = frontLateralVelocityMps;
        kinematics.contacts[kRearLeft].rightVelocityMps = rearLateralVelocityMps;
        kinematics.contacts[kRearRight].rightVelocityMps = rearLateralVelocityMps;
        return kinematics;
    }

    PlantModel::SlipTargets PlantModel::slipTargets(const StateVector& state) const noexcept
    {
        return slipTargets(state, wheelKinematics(state));
    }

    PlantModel::SlipTargets PlantModel::slipTargets(
        const StateVector& state,
        const WheelKinematics& kinematics) const noexcept
    {
        const float forwardVelocityMps = state(VehicleState::kU);
        const float rightVelocityMps = state(VehicleState::kV);
        const float yawRateRadps = state(VehicleState::kR);
        const float omegaLeftRadps = state(VehicleState::kOmegaL);
        const float omegaRightRadps = state(VehicleState::kOmegaR);

        if (ShouldReportStoppedDiagnosticsFast(
            forwardVelocityMps,
            rightVelocityMps,
            yawRateRadps,
            omegaLeftRadps,
            omegaRightRadps))
        {
            return SlipTargets{};
        }

        SlipTargets targets{};
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float leftCircumferentialVelocityMps = wheelRadiusM * omegaLeftRadps;
        const float rightCircumferentialVelocityMps = wheelRadiusM * omegaRightRadps;
        const float uRefLeft =
            ComputeRegularizedLongitudinalSpeedMps(
                kinematics.leftBankForwardVelocityMps,
                kRollingSpeedRegularizationMps);
        const float uRefRight =
            ComputeRegularizedLongitudinalSpeedMps(
                kinematics.rightBankForwardVelocityMps,
                kRollingSpeedRegularizationMps);

        targets.kappaLeft =
            (leftCircumferentialVelocityMps - kinematics.leftBankForwardVelocityMps) *
            (1.0f / uRefLeft);
        targets.kappaRight =
            (rightCircumferentialVelocityMps - kinematics.rightBankForwardVelocityMps) *
            (1.0f / uRefRight);
        for (std::size_t index = 0; index < targets.lateralRatio.size(); ++index)
        {
            const float contactReferenceSpeedMps =
                ComputeRegularizedLongitudinalSpeedMps(
                    kinematics.contacts[index].forwardVelocityMps,
                    kRollingSpeedRegularizationMps);
            targets.lateralRatio[index] =
                kinematics.contacts[index].rightVelocityMps / contactReferenceSpeedMps;
        }
        return targets;
    }

    PlantModel::ContactForces PlantModel::tireForces(const StateVector& state) const noexcept
    {
        return tireForces(
            state,
            App::Internal::CommandVector(0.0f, 0.0f));
    }

    PlantModel::ContactForces PlantModel::tireForces(
        const StateVector& state,
        const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(state, control).contactForces;
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(state, control).imuAccelBodyMps2;
    }

    float PlantModel::contactSaturation(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = tireForces(BuildBoundStateVector(), control);
        return (contactIndex < forces.contacts.size()) ?
            forces.contacts[contactIndex].saturation :
            0.0f;
    }

    float PlantModel::contactPreProjectionUtilization(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = tireForces(BuildBoundStateVector(), control);
        return (contactIndex < forces.contacts.size()) ?
            forces.contacts[contactIndex].preProjectionUtilization :
            0.0f;
    }

    float PlantModel::contactLateralSlipAngleRad(uint8_t contactIndex) const noexcept
    {
        const SlipTargets targets = slipTargets(BuildBoundStateVector());
        return (contactIndex < targets.lateralRatio.size()) ?
            std::atan(targets.lateralRatio[contactIndex]) :
            0.0f;
    }

    float PlantModel::backLeftImuRightAccelerationMps2(
        const App::Internal::CommandVector& control) const noexcept
    {
        return imuPlanarAcceleration(BuildBoundStateVector(), control).x();
    }

    float PlantModel::backLeftImuForwardAccelerationMps2(
        const App::Internal::CommandVector& control) const noexcept
    {
        return imuPlanarAcceleration(BuildBoundStateVector(), control).y();
    }

    void PlantModel::integrate(const App::Internal::CommandVector& control, float dt) noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return;
        }

        const StateVector currentState = BuildBoundStateVector();
        const float commandNorm =
            (std::max)(std::fabs(control.LeftCommand()), std::fabs(control.RightCommand()));
        const PlantDerivatives derivatives = forwardStep(currentState, control);
        StateVector nextState = currentState + (dt * derivatives.stateDot);
        nextState(VehicleState::kPsi) = NormalizeAngle(nextState(VehicleState::kPsi));

        if (ShouldSnapToZeroFast(
            nextState(VehicleState::kU),
            nextState(VehicleState::kV),
            nextState(VehicleState::kR),
            nextState(VehicleState::kOmegaL),
            nextState(VehicleState::kOmegaR),
            commandNorm))
        {
            nextState(VehicleState::kU) = 0.0f;
            nextState(VehicleState::kV) = 0.0f;
            nextState(VehicleState::kR) = 0.0f;
            nextState(VehicleState::kOmegaL) = 0.0f;
            nextState(VehicleState::kOmegaR) = 0.0f;
        }

        ApplyStateVectorToBoundState(nextState);
        _runtimeState.SetTime(_runtimeState.GetTime() + dt);
        _runtimeState.SetCurrentCommand(control);
        _runtimeState.SetLongitudinalAcceleration(derivatives.longitudinalAccelMps2);
        _runtimeState.SetLateralAcceleration(derivatives.lateralAccelMps2);
        _runtimeState.SetYawAcceleration(derivatives.yawAccelRadps2);
    }

    App::Internal::CommandVector PlantModel::ComputeFeedforward(
        float desiredAccelMps2,
        float desiredYawAccelRadps2) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float invWheelRadiusM = (wheelRadiusM > 0.0f) ? (1.0f / wheelRadiusM) : 0.0f;
        const float trackWidthM = std::fabs(_vehicle.GetTrackWidth());
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float invTrackWidthM = (trackWidthM > 0.0f) ? (1.0f / trackWidthM) : 0.0f;
        const float longitudinalOffsetM =
            std::fabs(Vehicle::GetPhysicalModel().driveWheelLongitudinalOffsetM);
        const float massKg =
            (std::isfinite(_vehicle.GetMass()) && (_vehicle.GetMass() > 0.0f)) ?
            _vehicle.GetMass() :
            1.0f;
        const float yawInertiaKgM2 =
            (std::isfinite(_vehicle.GetYawInertia()) && (_vehicle.GetYawInertia() > 0.0f)) ?
            _vehicle.GetYawInertia() :
            1.0f;
        const float wheelInertiaKgM2 =
            0.5f * (_leftDrive.getEquivalentWheelInertiaKgM2() + _rightDrive.getEquivalentWheelInertiaKgM2());
        const float resolvedWheelInertiaKgM2 =
            (std::isfinite(wheelInertiaKgM2) && (wheelInertiaKgM2 > 0.0f)) ?
            wheelInertiaKgM2 :
            1.0f;
        const float sustainedAccelMps2 = _vehicle.GetMaxLateralAcceleration();
        const float lateralForceSustainedLimitN =
            (std::isfinite(sustainedAccelMps2) && (sustainedAccelMps2 > 0.0f)) ?
            (sustainedAccelMps2 * massKg) :
            0.0f;
        const float staticFrictionTorqueNm =
            (std::max)(
                0.0f,
                _leftDrive.getTorqueFromCommand(
                    kReliableLaunchDriveCommand,
                    0.0f,
                    _vehicle.GetBatteryVoltage()));
        const float staticFrictionSpeedThresholdRadps =
            (wheelRadiusM > 0.0f) ? (kStaticFrictionMaxSpeedMps * invWheelRadiusM) : 0.0f;
        const float runtimeForwardVelocityMps = _runtimeState.GetVelocity();
        const float runtimeRightVelocityMps = _runtimeState.GetLateralVelocity();
        const float runtimeYawRateRadps = _runtimeState.GetRotationalVelocity();
        const float forwardVelocityMps =
            std::isfinite(runtimeForwardVelocityMps) ? runtimeForwardVelocityMps : 0.0f;
        const float rightVelocityMps =
            std::isfinite(runtimeRightVelocityMps) ? runtimeRightVelocityMps : 0.0f;
        const float yawRateRadps =
            std::isfinite(runtimeYawRateRadps) ? runtimeYawRateRadps : 0.0f;
        const float uRefBody =
            MazeMap::Math::Sqrtf(
                (forwardVelocityMps * forwardVelocityMps) +
                (kRollingSpeedRegularizationMps * kRollingSpeedRegularizationMps));

        const float wheelInertiaOverRadiusSqKg =
            resolvedWheelInertiaKgM2 * invWheelRadiusM * invWheelRadiusM;
        const float leftBankAccelMps2 =
            desiredAccelMps2 + (halfTrackWidthM * desiredYawAccelRadps2);
        const float rightBankAccelMps2 =
            desiredAccelMps2 - (halfTrackWidthM * desiredYawAccelRadps2);
        const float commonForceRequestN =
            0.5f *
            massKg *
            (desiredAccelMps2 - (yawRateRadps * rightVelocityMps));
        const float frontLateralVelocityMps =
            rightVelocityMps + (longitudinalOffsetM * yawRateRadps);
        const float rearLateralVelocityMps =
            rightVelocityMps - (longitudinalOffsetM * yawRateRadps);
        const float alphaFront = std::atan(frontLateralVelocityMps / uRefBody);
        const float alphaRear = std::atan(rearLateralVelocityMps / uRefBody);
        const float frontLeftLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_leftDrive.getFrontLateralTireStiffnessNPerRad());
        const float frontRightLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_rightDrive.getFrontLateralTireStiffnessNPerRad());
        const float rearLeftLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_leftDrive.getRearLateralTireStiffnessNPerRad());
        const float rearRightLateralStiffnessNPerRad =
            ResolvePositiveCalibration(_rightDrive.getRearLateralTireStiffnessNPerRad());
        const float frontAxleRightForceRawN =
            -(frontLeftLateralStiffnessNPerRad + frontRightLateralStiffnessNPerRad) * alphaFront;
        const float rearAxleRightForceRawN =
            -(rearLeftLateralStiffnessNPerRad + rearRightLateralStiffnessNPerRad) * alphaRear;
        const float frontAxleRightForceLimitN =
            kFrontLoadFraction * lateralForceSustainedLimitN;
        const float rearAxleRightForceLimitN =
            (1.0f - kFrontLoadFraction) * lateralForceSustainedLimitN;
        const float frontAxleRightForceN =
            (frontAxleRightForceLimitN > kForceEpsilonN) ?
            (std::clamp)(
                frontAxleRightForceRawN,
                -frontAxleRightForceLimitN,
                frontAxleRightForceLimitN) :
            frontAxleRightForceRawN;
        const float rearAxleRightForceN =
            (rearAxleRightForceLimitN > kForceEpsilonN) ?
            (std::clamp)(
                rearAxleRightForceRawN,
                -rearAxleRightForceLimitN,
                rearAxleRightForceLimitN) :
            rearAxleRightForceRawN;
        const float lateralYawMomentNm =
            longitudinalOffsetM * (frontAxleRightForceN - rearAxleRightForceN);
        const int requestedYawSign =
            (desiredYawAccelRadps2 > kSignEpsilon) -
            (desiredYawAccelRadps2 < -kSignEpsilon);
        const float lowYawSpeedWeight =
            1.0f -
            SmoothStep(
                kStopEnterYawRateRadps,
                kStopExitYawRateRadps,
                std::fabs(yawRateRadps));
        const float yawScrubBreakawayMomentNm =
            lowYawSpeedWeight *
            static_cast<float>(requestedYawSign) *
            longitudinalOffsetM *
            massKg *
            Vehicle::GetSustainedLateralAccelerationReferenceMps2();
        const float requestedYawMomentNm =
            (yawInertiaKgM2 * desiredYawAccelRadps2) +
            (kYawRateDampingNmsPerRad * yawRateRadps) -
            lateralYawMomentNm +
            yawScrubBreakawayMomentNm;
        const float differentialForceRequestN = -requestedYawMomentNm * invTrackWidthM;
        const float leftForceCommandN =
            commonForceRequestN - differentialForceRequestN;
        const float rightForceCommandN =
            commonForceRequestN + differentialForceRequestN;

        const float leftWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) * invWheelRadiusM) :
            0.0f;
        const float rightWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) * invWheelRadiusM) :
            0.0f;
        const float leftWheelInertiaForceN =
            wheelInertiaOverRadiusSqKg * leftBankAccelMps2;
        const float rightWheelInertiaForceN =
            wheelInertiaOverRadiusSqKg * rightBankAccelMps2;
        const float leftWheelTorqueRequestNm =
            wheelRadiusM * (leftForceCommandN + leftWheelInertiaForceN);
        const float rightWheelTorqueRequestNm =
            wheelRadiusM * (rightForceCommandN + rightWheelInertiaForceN);

        const float leftViscousFrictionTorqueNm =
            kViscousFrictionNmPerRadps * leftWheelSpeedRadps;
        const float rightViscousFrictionTorqueNm =
            kViscousFrictionNmPerRadps * rightWheelSpeedRadps;
        float leftFrictionTorqueNm = leftViscousFrictionTorqueNm;
        float rightFrictionTorqueNm = rightViscousFrictionTorqueNm;
        if ((std::fabs(leftWheelSpeedRadps) <= staticFrictionSpeedThresholdRadps) &&
            (std::fabs(rightWheelSpeedRadps) <= staticFrictionSpeedThresholdRadps))
        {
            const int leftTorqueSign =
                (leftWheelTorqueRequestNm > kSignEpsilon) -
                (leftWheelTorqueRequestNm < -kSignEpsilon);
            const int leftSpeedSign =
                (leftWheelSpeedRadps > kSignEpsilon) -
                (leftWheelSpeedRadps < -kSignEpsilon);
            const int rightTorqueSign =
                (rightWheelTorqueRequestNm > kSignEpsilon) -
                (rightWheelTorqueRequestNm < -kSignEpsilon);
            const int rightSpeedSign =
                (rightWheelSpeedRadps > kSignEpsilon) -
                (rightWheelSpeedRadps < -kSignEpsilon);
            leftFrictionTorqueNm +=
                staticFrictionTorqueNm *
                static_cast<float>((leftTorqueSign != 0) ? leftTorqueSign : leftSpeedSign);
            rightFrictionTorqueNm +=
                staticFrictionTorqueNm *
                static_cast<float>((rightTorqueSign != 0) ? rightTorqueSign : rightSpeedSign);
        }
        else
        {
            const int leftSpeedSign =
                (leftWheelSpeedRadps > kSignEpsilon) -
                (leftWheelSpeedRadps < -kSignEpsilon);
            const int leftTorqueSign =
                (leftWheelTorqueRequestNm > kSignEpsilon) -
                (leftWheelTorqueRequestNm < -kSignEpsilon);
            leftFrictionTorqueNm +=
                kRollingFrictionTorqueNm *
                static_cast<float>((leftSpeedSign != 0) ? leftSpeedSign : leftTorqueSign);

            const int rightSpeedSign =
                (rightWheelSpeedRadps > kSignEpsilon) -
                (rightWheelSpeedRadps < -kSignEpsilon);
            const int rightTorqueSign =
                (rightWheelTorqueRequestNm > kSignEpsilon) -
                (rightWheelTorqueRequestNm < -kSignEpsilon);
            rightFrictionTorqueNm +=
                kRollingFrictionTorqueNm *
                static_cast<float>((rightSpeedSign != 0) ? rightSpeedSign : rightTorqueSign);
        }

        return CommandVector(
            _leftDrive.getCommandFromTorque(
                leftWheelTorqueRequestNm + leftFrictionTorqueNm,
                leftWheelSpeedRadps,
                _vehicle.GetBatteryVoltage()),
            _rightDrive.getCommandFromTorque(
                rightWheelTorqueRequestNm + rightFrictionTorqueNm,
                rightWheelSpeedRadps,
                _vehicle.GetBatteryVoltage()));
    }

    Eigen::Matrix<float, 2, 2> PlantModel::encoderPairCovarianceRadps(
        float linearSpeedSigmaMps,
        float yawRateSigmaRadps) const noexcept
    {
        Eigen::Matrix<float, 2, 2> covariance = Eigen::Matrix<float, 2, 2>::Zero();
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        if (!(wheelRadiusM > 0.0f) || !std::isfinite(wheelRadiusM))
        {
            covariance(0, 0) = 1.0f;
            covariance(1, 1) = 1.0f;
            return covariance;
        }

        const float resolvedLinearSigmaMps =
            (std::isfinite(linearSpeedSigmaMps) && (linearSpeedSigmaMps > 0.0f)) ?
            linearSpeedSigmaMps :
            1.0f;
        const float resolvedYawSigmaRadps =
            (std::isfinite(yawRateSigmaRadps) && (yawRateSigmaRadps > 0.0f)) ?
            yawRateSigmaRadps :
            1.0f;
        const float halfTrackWidthM = 0.5f * ResolvePhysicalTrackWidthM(_vehicle);
        const float varianceUMps2 = resolvedLinearSigmaMps * resolvedLinearSigmaMps;
        const float varianceYawRateRadps2 = resolvedYawSigmaRadps * resolvedYawSigmaRadps;
        const float varianceWheelLinearMps2 =
            varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float covarianceWheelLinearMps2 =
            varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float invWheelRadiusM = 1.0f / wheelRadiusM;
        const float invWheelRadius2 = invWheelRadiusM * invWheelRadiusM;
        covariance(0, 0) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 1) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(0, 1) = covarianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 0) = covariance(0, 1);
        return covariance;
    }

    Eigen::Matrix<float, 2, 2> PlantModel::encoderPairSqrtNoise(
        const EncoderObs& observation,
        float stationaryLinearSpeedSigmaMps,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            Eigen::Matrix<float, 2, 2> sqrtNoise = Eigen::Matrix<float, 2, 2>::Zero();
            const float sigmaRadps = stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps);
            sqrtNoise(0, 0) = sigmaRadps;
            sqrtNoise(1, 1) = sigmaRadps;
            return sqrtNoise;
        }

        const Eigen::Matrix<float, 2, 2> covariance =
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const Eigen::LLT<Eigen::Matrix<float, 2, 2>> llt(covariance);
        if (llt.info() == Eigen::Success)
        {
            return llt.matrixL();
        }

        Eigen::Matrix<float, 2, 2> fallback = Eigen::Matrix<float, 2, 2>::Zero();
        fallback(0, 0) = 1.0f;
        fallback(1, 1) = 1.0f;
        return fallback;
    }

    float PlantModel::stationaryEncoderOmegaSigmaRadps(float stationaryLinearSpeedSigmaMps) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        if (!(wheelRadiusM > 0.0f) || !std::isfinite(wheelRadiusM))
        {
            return 1.0f;
        }

        const float resolvedStationarySigmaMps =
            (std::isfinite(stationaryLinearSpeedSigmaMps) && (stationaryLinearSpeedSigmaMps > 0.0f)) ?
            stationaryLinearSpeedSigmaMps :
            1.0f;
        return resolvedStationarySigmaMps / wheelRadiusM;
    }

    float PlantModel::measuredLinearSpeedMps(const EncoderObs& observation) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        if (!(wheelRadiusM > 0.0f) || !std::isfinite(wheelRadiusM))
        {
            return 0.0f;
        }

        return 0.5f * wheelRadiusM * (observation.omegaLeftRadps + observation.omegaRightRadps);
    }

    float PlantModel::measuredYawRateRadps(const EncoderObs& observation) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float trackWidthM = ResolvePhysicalTrackWidthM(_vehicle);
        if (!(wheelRadiusM > 0.0f) ||
            !std::isfinite(wheelRadiusM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return 0.0f;
        }

        return wheelRadiusM * (observation.omegaLeftRadps - observation.omegaRightRadps) / trackWidthM;
    }

    float PlantModel::measuredYawRateVarianceRadps2(
        const EncoderObs& observation,
        float stationaryLinearSpeedSigmaMps,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float trackWidthM = ResolvePhysicalTrackWidthM(_vehicle);
        if (!(wheelRadiusM > 0.0f) ||
            !std::isfinite(wheelRadiusM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return 1.0f;
        }

        const bool zeroWheelObservation =
            (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
        const Eigen::Matrix<float, 2, 2> wheelCovarianceRadps2 =
            zeroWheelObservation ?
            (Eigen::Matrix<float, 2, 2>::Identity() *
                (stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps) *
                  stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps))) :
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const float yawScale = wheelRadiusM / trackWidthM;
        const float variance =
            (yawScale * yawScale) *
            (wheelCovarianceRadps2(0, 0) +
             wheelCovarianceRadps2(1, 1) -
             (2.0f * wheelCovarianceRadps2(0, 1)));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    float PlantModel::measuredWheelVarianceRadps2(
        const EncoderObs& observation,
        float stationaryLinearSpeedSigmaMps,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        const bool zeroWheelObservation =
            (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
        if (zeroWheelObservation)
        {
            const float stationarySigmaRadps = stationaryEncoderOmegaSigmaRadps(stationaryLinearSpeedSigmaMps);
            return stationarySigmaRadps * stationarySigmaRadps;
        }

        const Eigen::Matrix<float, 2, 2> covariance =
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const float variance = (std::max)(covariance(0, 0), covariance(1, 1));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    Eigen::Vector2f PlantModel::wheelLinearVelocityFromBodyState(const StateVector& state) const noexcept
    {
        const float trackWidthM =
            (std::isfinite(_vehicle.GetTrackWidth()) && (_vehicle.GetTrackWidth() > 0.0f)) ?
            _vehicle.GetTrackWidth() :
            0.0f;
        const float forwardSpeedMps =
            std::isfinite(state(VehicleState::kU)) ? state(VehicleState::kU) : 0.0f;
        const float yawRateRadps =
            std::isfinite(state(VehicleState::kR)) ? state(VehicleState::kR) : 0.0f;
        Eigen::Vector2f velocity = Eigen::Vector2f::Zero();
        velocity(0) = forwardSpeedMps + (0.5f * trackWidthM * yawRateRadps);
        velocity(1) = forwardSpeedMps - (0.5f * trackWidthM * yawRateRadps);
        return velocity;
    }

    float PlantModel::sustainedCombinedAccelerationUsage(float accelerationMps2) const noexcept
    {
        const float limit =
            (std::isfinite(Vehicle::GetSustainedLateralAccelerationReferenceMps2()) &&
             (Vehicle::GetSustainedLateralAccelerationReferenceMps2() > 0.0f)) ?
            Vehicle::GetSustainedLateralAccelerationReferenceMps2() :
            1.0f;
        return std::fabs(accelerationMps2) / limit;
    }

    float PlantModel::nominalCombinedAccelerationUsage(float accelerationMps2) const noexcept
    {
        const float limit =
            (std::isfinite(_vehicle.GetNominalCombinedAcceleration()) &&
             (_vehicle.GetNominalCombinedAcceleration() > 0.0f)) ?
            _vehicle.GetNominalCombinedAcceleration() :
            1.0f;
        return std::fabs(accelerationMps2) / limit;
    }

    float PlantModel::peakCombinedAccelerationUsage(float accelerationMps2) const noexcept
    {
        const float limit =
            (std::isfinite(_vehicle.GetPeakCombinedAcceleration()) &&
             (_vehicle.GetPeakCombinedAcceleration() > 0.0f)) ?
            _vehicle.GetPeakCombinedAcceleration() :
            1.0f;
        return std::fabs(accelerationMps2) / limit;
    }

    float PlantModel::stopExitYawRateUsage(float yawRateRadps) const noexcept
    {
        const float limit =
            (std::isfinite(kStopExitYawRateRadps) &&
             (kStopExitYawRateRadps > 0.0f)) ?
            kStopExitYawRateRadps :
            1.0f;
        return std::fabs(yawRateRadps) / limit;
    }

    float PlantModel::totalForwardContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control).contactForces.SumForwardForceN();
    }

    float PlantModel::totalRightContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control).contactForces.SumRightForceN();
    }

    float PlantModel::leftBankForwardContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control).contactForces.LeftBankForwardForceN();
    }

    float PlantModel::rightBankForwardContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control).contactForces.RightBankForwardForceN();
    }

    float PlantModel::contactRightForceN(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = forwardStep(control).contactForces;
        return (contactIndex < forces.contacts.size()) ?
            forces.contacts[contactIndex].rightForceN :
            0.0f;
    }

    float PlantModel::contactForwardForceN(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = forwardStep(control).contactForces;
        return (contactIndex < forces.contacts.size()) ?
            forces.contacts[contactIndex].forwardForceN :
            0.0f;
    }

    float PlantModel::contactNormalLoadN(uint8_t contactIndex) const noexcept
    {
        if (contactIndex < 4U)
        {
            return forwardStep(ZeroControlVector()).contactForces.contacts[contactIndex].normalForceN;
        }
        return 0.0f;
    }

    float PlantModel::totalContactNormalLoadN() const noexcept
    {
        const ContactForces forces = forwardStep(ZeroControlVector()).contactForces;
        return
            forces.contacts[0].normalForceN +
            forces.contacts[1].normalForceN +
            forces.contacts[2].normalForceN +
            forces.contacts[3].normalForceN;
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        velocityTargetTechnicalLimits(
            _runtimeState.GetVelocity(),
            _runtimeState.GetRotationalVelocity(),
            maxLongitudinalAccelMps2,
            maxYawAccelRadps2);
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float forwardVelocityMps,
        float yawRateRadps,
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        maxLongitudinalAccelMps2 = 0.0f;
        maxYawAccelRadps2 = 0.0f;

        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float trackWidthM = std::fabs(_vehicle.GetTrackWidth());
        if (!(std::isfinite(wheelRadiusM) &&
            std::isfinite(trackWidthM) &&
            (wheelRadiusM > kForceEpsilonN)))
        {
            return;
        }

        const float invWheelRadiusM = 1.0f / wheelRadiusM;
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float massKg =
            (std::isfinite(_vehicle.GetMass()) && (_vehicle.GetMass() > 0.0f)) ?
            _vehicle.GetMass() :
            1.0f;
        const float yawInertiaKgM2 =
            (std::isfinite(_vehicle.GetYawInertia()) && (_vehicle.GetYawInertia() > 0.0f)) ?
            _vehicle.GetYawInertia() :
            1.0f;
        const float resolvedForwardVelocityMps =
            std::isfinite(forwardVelocityMps) ? forwardVelocityMps : 0.0f;
        const float resolvedYawRateRadps =
            std::isfinite(yawRateRadps) ? yawRateRadps : 0.0f;
        const float leftBankSpeedRadps =
            (resolvedForwardVelocityMps + (halfTrackWidthM * resolvedYawRateRadps)) *
            invWheelRadiusM;
        const float rightBankSpeedRadps =
            (resolvedForwardVelocityMps - (halfTrackWidthM * resolvedYawRateRadps)) *
            invWheelRadiusM;
        const float batteryVoltageV = _vehicle.GetBatteryVoltage();
        const float leftPositiveTorqueNm =
            _leftDrive.getTorqueFromCommand(1.0f, leftBankSpeedRadps, batteryVoltageV);
        const float leftNegativeTorqueNm =
            _leftDrive.getTorqueFromCommand(-1.0f, leftBankSpeedRadps, batteryVoltageV);
        const float rightPositiveTorqueNm =
            _rightDrive.getTorqueFromCommand(1.0f, rightBankSpeedRadps, batteryVoltageV);
        const float rightNegativeTorqueNm =
            _rightDrive.getTorqueFromCommand(-1.0f, rightBankSpeedRadps, batteryVoltageV);
        const float leftSymmetricTorqueNm =
            (std::min)(std::fabs(leftPositiveTorqueNm), std::fabs(leftNegativeTorqueNm));
        const float rightSymmetricTorqueNm =
            (std::min)(std::fabs(rightPositiveTorqueNm), std::fabs(rightNegativeTorqueNm));
        const float leftBankForceN =
            (std::isfinite(leftSymmetricTorqueNm) && (wheelRadiusM > kForceEpsilonN)) ?
            (leftSymmetricTorqueNm / wheelRadiusM) :
            0.0f;
        const float rightBankForceN =
            (std::isfinite(rightSymmetricTorqueNm) && (wheelRadiusM > kForceEpsilonN)) ?
            (rightSymmetricTorqueNm / wheelRadiusM) :
            0.0f;
        const float maxBankForceN =
            (std::min)(
                (std::max)(0.0f, leftBankForceN),
                (std::max)(0.0f, rightBankForceN));
        maxLongitudinalAccelMps2 =
            (2.0f * maxBankForceN) / massKg;
        maxYawAccelRadps2 =
            (trackWidthM * maxBankForceN) / yawInertiaKgM2;
    }

    float PlantModel::driveFrictionTorque(
        float wheelBankSpeedRadps,
        float wheelTorqueRequestNm) const noexcept
    {
        return DriveFrictionTorqueFast(wheelBankSpeedRadps, wheelTorqueRequestNm, staticFrictionTorqueNm());
    }

    float PlantModel::staticFrictionTorqueNm() const noexcept
    {
        return
            (std::max)(
                0.0f,
                _leftDrive.getTorqueFromCommand(
                    kReliableLaunchDriveCommand,
                    0.0f,
                    _vehicle.GetBatteryVoltage()));
    }

    float PlantModel::rollingFrictionTorqueNm() const noexcept
    {
        return kRollingFrictionTorqueNm;
    }

    float PlantModel::staticFrictionSpeedThresholdRadps() const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        return (wheelRadiusM > 0.0f) ? (kStaticFrictionMaxSpeedMps / wheelRadiusM) : 0.0f;
    }

    float PlantModel::leftDriveEquivalentWheelInertiaKgM2() const noexcept
    {
        return _leftDrive.getEquivalentWheelInertiaKgM2();
    }

    float PlantModel::rightDriveEquivalentWheelInertiaKgM2() const noexcept
    {
        return _rightDrive.getEquivalentWheelInertiaKgM2();
    }

    float PlantModel::leftDriveLongitudinalTireStiffnessN() const noexcept
    {
        return _leftDrive.getLongitudinalTireStiffnessN();
    }

    float PlantModel::rightDriveLongitudinalTireStiffnessN() const noexcept
    {
        return _rightDrive.getLongitudinalTireStiffnessN();
    }
}


