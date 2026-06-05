
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

namespace MazeMap
{
    bool PlantModel::WriteDebugTextLine(
        void* context,
        bool (* const sink)(void* context, const char* type, const char* format, std::va_list args) noexcept,
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

    float PlantModel::SignedDirection(float preferredValue, float fallbackValue) noexcept
    {
        const int preferredSign = (preferredValue > kSignEpsilon) - (preferredValue < -kSignEpsilon);
        if (preferredSign != 0)
        {
            return static_cast<float>(preferredSign);
        }

        const int fallbackSign = (fallbackValue > kSignEpsilon) - (fallbackValue < -kSignEpsilon);
        return static_cast<float>(fallbackSign);
    }

    float PlantModel::PositivePart(const float value) noexcept
    {
        return (std::isfinite(value) && (value > 0.0f)) ? value : 0.0f;
    }

    float PlantModel::PositiveFiniteOrDefault(const float value, const float fallback) noexcept
    {
        return (std::isfinite(value) && (value > 0.0f)) ? value : fallback;
    }

    float PlantModel::RationalSquareGate(const float value, const float knee) noexcept
    {
        const float resolvedValue = std::isfinite(value) ? value : 0.0f;
        const float resolvedKnee = PositiveFiniteOrDefault(knee, 1.0e-6f);
        const float value2 = resolvedValue * resolvedValue;
        const float knee2 = resolvedKnee * resolvedKnee;
        const float denominator = value2 + knee2;
        return (denominator > 0.0f) ? (value2 / denominator) : 0.0f;
    }

    float PlantModel::loadWeightedContactRelativeSpeedMps(
        const WheelKinematics& kinematics,
        const std::array<float, 4>& contactNormalLoadN,
        const float totalNormalLoadN) noexcept
    {
        const float normalSum = (totalNormalLoadN > kForceEpsilonN) ? totalNormalLoadN : kForceEpsilonN;
        float weightedSpeed2 = 0.0f;
        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const ContactKinematics& contactKinematics = kinematics._contacts[contactIndex];
            const float rightSpeed = contactKinematics._rightRelativeVelocityMps;
            const float forwardSpeed = contactKinematics._forwardRelativeVelocityMps;
            weightedSpeed2 +=
                (std::max)(0.0f, contactNormalLoadN[contactIndex]) *
                ((rightSpeed * rightSpeed) + (forwardSpeed * forwardSpeed));
        }
        return MazeMap::Math::Sqrtf((std::max)(0.0f, weightedSpeed2 / normalSum));
    }

    float PlantModel::aggregateContactYawMomentCorrectionAlongYawNm(
        const float forwardVelocityMps,
        const float contactRelativeSpeedMps,
        const float projectedYawMomentAlongYawNm,
        const float yawMomentYieldNm,
        const float variantCContactYawMomentCorrectionAlongYawNm) const noexcept
    {
        const float relWeight = PositiveFiniteOrDefault(
            _aggregateContactYawMomentCorrectionForceRelWeight,
            kDefaultAggregateContactYawMomentCorrectionForceRelWeight);
        const float speedV2 =
            (forwardVelocityMps * forwardVelocityMps) +
            ((relWeight * contactRelativeSpeedMps) * (relWeight * contactRelativeSpeedMps));
        const float speedKnee = PositiveFiniteOrDefault(
            _aggregateContactYawMomentCorrectionBlendSpeedKneeMps,
            kDefaultAggregateContactYawMomentCorrectionBlendSpeedKneeMps);
        const float speedKnee2 = speedKnee * speedKnee;
        const float speedLow =
            (speedKnee2 > 0.0f) ? (speedKnee2 / (speedKnee2 + (std::max)(0.0f, speedV2))) : 0.0f;
        const float resolvedYieldNm = PositiveFiniteOrDefault(yawMomentYieldNm, kForceEpsilonN);
        const float utilization = PositivePart(projectedYawMomentAlongYawNm) / resolvedYieldNm;
        const float forceGate =
            RationalSquareGate(utilization, _aggregateContactYawMomentCorrectionBlendForceKnee);
        const float speedFade = PositiveFiniteOrDefault(
            _aggregateContactYawMomentCorrectionForceSpeedFadeMps,
            kDefaultAggregateContactYawMomentCorrectionForceSpeedFadeMps);
        const float speedFade2 = speedFade * speedFade;
        const float speedRelief =
            (speedFade2 > 0.0f) ? (speedFade2 / (speedFade2 + (std::max)(0.0f, speedV2))) : 0.0f;
        const float forceBranchContactYawMomentCorrectionAlongYawNm =
            speedRelief *
            PositiveFiniteOrDefault(
                _aggregateContactYawMomentCorrectionForceSlidingMomentNm,
                kDefaultAggregateContactYawMomentCorrectionForceSlidingMomentNm);
        const float blend = (std::clamp)(speedLow * forceGate, 0.0f, 1.0f);
        const float contactYawMomentCorrectionAlongYawNm =
            variantCContactYawMomentCorrectionAlongYawNm +
            (blend *
                (forceBranchContactYawMomentCorrectionAlongYawNm -
                    variantCContactYawMomentCorrectionAlongYawNm));
        return std::isfinite(contactYawMomentCorrectionAlongYawNm) ?
            contactYawMomentCorrectionAlongYawNm :
            0.0f;
    }

    float PlantModel::variantCAggregateContactYawMomentCorrectionAlongYawNm(
        const float yawDirection,
        const float forwardVelocityMps,
        const WheelKinematics& kinematics,
        const std::array<float, 4>& contactRightPositionsM,
        const std::array<float, 4>& contactForwardPositionsM,
        const std::array<float, 4>& contactNormalLoadN,
        const std::array<float, 4>& projectedForwardForceN,
        const std::array<float, 4>& projectedRightForceN,
        const float totalNormalLoadN,
        const float maxProjectedContactUtilization) const noexcept
    {
        if (yawDirection == 0.0f)
        {
            return 0.0f;
        }

        float gainFrontRightBasis = 0.0f;
        float gainRearRightBasis = 0.0f;
        float gainLeftLongBasis = 0.0f;
        float gainRightLongBasis = 0.0f;
        float projectedMomentNm = 0.0f;

        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const float rightPositionM = contactRightPositionsM[contactIndex];
            const float forwardPositionM = contactForwardPositionsM[contactIndex];
            const ContactKinematics& contactKinematics = kinematics._contacts[contactIndex];
            const float rightBasis =
                -yawDirection * forwardPositionM * contactKinematics._rightRelativeVelocityMps;
            const float longitudinalBasis =
                yawDirection * rightPositionM * contactKinematics._forwardRelativeVelocityMps;
            if (forwardPositionM > 0.0f)
            {
                gainFrontRightBasis += rightBasis;
            }
            else
            {
                gainRearRightBasis += rightBasis;
            }

            if (rightPositionM < 0.0f)
            {
                gainLeftLongBasis += longitudinalBasis;
            }
            else
            {
                gainRightLongBasis += longitudinalBasis;
            }

            const float projectedLocalMomentNm =
                (forwardPositionM * projectedRightForceN[contactIndex]) -
                (rightPositionM * projectedForwardForceN[contactIndex]);
            projectedMomentNm += projectedLocalMomentNm;
        }

        const float contactRelativeSpeedMps =
            loadWeightedContactRelativeSpeedMps(
                kinematics,
                contactNormalLoadN,
                totalNormalLoadN);
        const float relKnee = PositiveFiniteOrDefault(
            _aggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps,
            kDefaultAggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps);
        const float forwardKnee = PositiveFiniteOrDefault(
            _aggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps,
            kDefaultAggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps);
        const float lowRel =
            1.0f /
            (1.0f + ((contactRelativeSpeedMps * contactRelativeSpeedMps) / (relKnee * relKnee)));
        const float lowForward =
            1.0f /
            (1.0f + ((forwardVelocityMps * forwardVelocityMps) / (forwardKnee * forwardKnee)));
        const float highForward = 1.0f - lowForward;
        const float utilization =
            (std::clamp)(
                std::isfinite(maxProjectedContactUtilization) ? maxProjectedContactUtilization : 0.0f,
                0.0f,
                5.0f);
        const float utilSmooth = utilization / (1.0f + utilization);
        const float projectedYawMomentAlongCorrectionDirectionNm = -yawDirection * projectedMomentNm;

        const float contactYawMomentCorrectionAlongYawNm =
            (kAggregateContactYawMomentCorrectionVariantCLongLowRelCoeff *
                lowRel * (gainLeftLongBasis + gainRightLongBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCRightBaseCoeff *
                (gainFrontRightBasis + gainRearRightBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCForceMomentHighForwardCoeff *
                projectedYawMomentAlongCorrectionDirectionNm * highForward) +
            (kAggregateContactYawMomentCorrectionVariantCRightUtilCoeff *
                utilSmooth * (gainFrontRightBasis + gainRearRightBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCLongBaseCoeff *
                (gainLeftLongBasis + gainRightLongBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCLongUtilCoeff *
                utilSmooth * (gainLeftLongBasis + gainRightLongBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCRightLowRelCoeff *
                lowRel * (gainFrontRightBasis + gainRearRightBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCLongHighForwardCoeff *
                highForward * (gainLeftLongBasis + gainRightLongBasis)) +
            (kAggregateContactYawMomentCorrectionVariantCRightHighForwardCoeff *
                highForward * (gainFrontRightBasis + gainRearRightBasis));
        return std::isfinite(contactYawMomentCorrectionAlongYawNm) ?
            contactYawMomentCorrectionAlongYawNm :
            0.0f;
    }

    float PlantModel::residualDecayAlpha(const float dtS, const float tauS) noexcept
    {
        if (!std::isfinite(dtS) || !(dtS > 0.0f) || !std::isfinite(tauS) || !(tauS > 0.0f))
        {
            return 1.0f;
        }

        return std::exp(-dtS / tauS);
    }

    float PlantModel::forwardAccelerationResidualDecayAlpha(const float dtS) noexcept
    {
        return residualDecayAlpha(dtS, kForwardAccelerationResidualDecayTauS);
    }

    float PlantModel::rightAccelerationResidualDecayAlpha(const float dtS) noexcept
    {
        return residualDecayAlpha(dtS, kRightAccelerationResidualDecayTauS);
    }

    float PlantModel::yawAccelerationResidualDecayAlpha(const float dtS) noexcept
    {
        return residualDecayAlpha(dtS, kYawAccelResidualDecayTauS);
    }

    PlantModel::PlantModel(const Vehicle& vehicle, VehicleState& runtimeState) noexcept
        : _vehicle(vehicle)
        , _runtimeState(runtimeState)
        , _leftDrive(vehicle.GetLeftMotorEncoderDrive())
        , _rightDrive(vehicle.GetRightMotorEncoderDrive())
    {
    }

    bool PlantModel::WritePlantDebugTextDump(
        void* context,
        bool (*sink)(void* context, const char* type, const char* format, std::va_list args) noexcept)
        const noexcept
    {
        if (sink == nullptr)
        {
            return false;
        }

        const float massKg = _vehicle.GetMass();
        const float trackWidthM = _vehicle.GetTrackWidth();
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float contactPatchLongitudinalOffsetM =
            Vehicle::GetDriveWheelLongitudinalOffsetM();
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

        if (!WriteDebugTextLine(
                context,
                sink,
                "plant_dump_params_mass_geometry",
                "mass_kg=%.9g;effective_longitudinal_mass_kg=%.9g;yaw_inertia_kg_m2=%.9g;track_width_m=%.9g;contact_patch_longitudinal_offset_m=%.9g;wheel_radius_m=%.9g;equivalent_wheel_inertia_kg_m2=%.9g",
                static_cast<double>(massKg),
                static_cast<double>(massKg),
                static_cast<double>(_vehicle.GetYawInertia()),
                static_cast<double>(trackWidthM),
                static_cast<double>(contactPatchLongitudinalOffsetM),
                static_cast<double>(Vehicle::GetDriveWheelRadiusM()),
                static_cast<double>(equivalentWheelInertiaKgM2)) ||
            !WriteDebugTextLine(
                context,
                sink,
                "plant_dump_params_drive_electrical",
                "supply_voltage_v=%.9g;drive_resistance_ohms=%.9g;torque_constant_nm_per_a=%.9g;speed_constant_radps_per_volt=%.9g;no_load_current_a=%.9g;motor_current_limit_a=%.9g;gear_ratio=%.9g;encoder_counts_per_motor_rev=%u",
                static_cast<double>(_vehicle.GetBatteryVoltage()),
                static_cast<double>(_leftDrive.getResistance()),
                static_cast<double>(_leftDrive.getTorqueConstant()),
                static_cast<double>(_leftDrive.getSpeedConstant()),
                static_cast<double>(_leftDrive.getNoLoadCurrent()),
                static_cast<double>(motorCurrentLimitA),
                static_cast<double>(_leftDrive.getGearRatio()),
                static_cast<unsigned>(_leftDrive.getPulsesPerRev())) ||
            !WriteDebugTextLine(
                context,
                sink,
                "plant_dump_params_tire_friction",
                "drivetrain_efficiency=%.9g;rolling_friction_torque_nm=%.9g;viscous_friction_nm_per_radps=%.9g;longitudinal_tire_stiffness_n=%.9g;mu_front=%.9g;mu_rear=%.9g;front_load_fraction=%.9g",
                1.0,
                static_cast<double>(kRollingFrictionTorqueNm),
                static_cast<double>(kViscousFrictionNmPerRadps),
                static_cast<double>(
                    0.5f * (_leftDrive.getLongitudinalTireStiffnessN() + _rightDrive.getLongitudinalTireStiffnessN())),
                static_cast<double>(kMuFront),
                static_cast<double>(kMuRear),
                static_cast<double>(kFrontLoadFraction)) ||
            !WriteDebugTextLine(
                context,
                sink,
                "plant_dump_params_static_friction",
                "static_friction_torque_nm=%.9g;static_friction_max_speed_mps=%.9g",
                static_cast<double>(staticFrictionTorqueNm),
                static_cast<double>(kStaticFrictionMaxSpeedMps)) ||
            !WriteDebugTextLine(
                context,
                sink,
                "plant_dump_params_misc",
                "force_epsilon_n=%.9g;contact_yaw_patch_force_gain_ns_per_m=%.9g;fan_downforce_at_full_duty_n=%.9g;no_hit_range_m=%.9g",
                static_cast<double>(kForceEpsilonN),
                static_cast<double>(kContactYawPatchForceGainNsPerM),
                static_cast<double>(_vehicle.GetFanDownforceAtFullDuty()),
                static_cast<double>(_vehicle.FrontLeftWallSensor().GetNoHitRangeM())) ||
            !WriteDebugTextLine(
                context,
                sink,
                "plant_dump_params_aggregate_contact_yaw_correction",
                "blend_speed_knee_mps=%.9g;blend_force_knee=%.9g;force_rel_weight=%.9g;force_speed_fade_mps=%.9g;force_sliding_nm=%.9g;variant_c_rel_knee_mps=%.9g;variant_c_fwd_knee_mps=%.9g",
                static_cast<double>(_aggregateContactYawMomentCorrectionBlendSpeedKneeMps),
                static_cast<double>(_aggregateContactYawMomentCorrectionBlendForceKnee),
                static_cast<double>(_aggregateContactYawMomentCorrectionForceRelWeight),
                static_cast<double>(_aggregateContactYawMomentCorrectionForceSpeedFadeMps),
                static_cast<double>(_aggregateContactYawMomentCorrectionForceSlidingMomentNm),
                static_cast<double>(_aggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps),
                static_cast<double>(_aggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps)))
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
            if (!WriteDebugTextLine(
                    context,
                    sink,
                    "plant_dump_contact_position",
                    "index=%u;x_m=%.9g;y_m=%.9g",
                    static_cast<unsigned>(index),
                    static_cast<double>(position.x()),
                    static_cast<double>(position.y())))
            {
                return false;
            }
        }

        const char* const sensorTypes[] = {
            "plant_dump_sensor_front_left",
            "plant_dump_sensor_front_right",
            "plant_dump_sensor_side_left",
            "plant_dump_sensor_side_right",
            "plant_dump_imu_mount"
        };
        const SensorMount sensorMounts[] = {
            Vehicle::GetFrontLeftSensorMount(),
            Vehicle::GetFrontRightSensorMount(),
            Vehicle::GetSideLeftSensorMount(),
            Vehicle::GetSideRightSensorMount(),
            Vehicle::GetBackLeftImuMount()
        };
        for (std::size_t index = 0; index < 5U; ++index)
        {
            const SensorMount& sensor = sensorMounts[index];
            const Eigen::Matrix2f& bodyFromSensor = sensor.bodyFromSensor();
            if (!WriteDebugTextLine(
                    context,
                    sink,
                    sensorTypes[index],
                    "position_x_m=%.9g;position_y_m=%.9g;body_from_sensor_00=%.9g;body_from_sensor_01=%.9g;body_from_sensor_10=%.9g;body_from_sensor_11=%.9g;clockwise_yaw_sign=%.9g",
                    static_cast<double>(sensor.positionBodyM().x()),
                    static_cast<double>(sensor.positionBodyM().y()),
                    static_cast<double>(bodyFromSensor(0, 0)),
                    static_cast<double>(bodyFromSensor(0, 1)),
                    static_cast<double>(bodyFromSensor(1, 0)),
                    static_cast<double>(bodyFromSensor(1, 1)),
                    static_cast<double>(sensor.clockwiseYawSign())))
            {
                return false;
            }
        }
        return true;
    }

    Eigen::Matrix<float, VehicleState::kDimension, 1> PlantModel::BuildBoundStateVector() const noexcept
    {
        Eigen::Matrix<float, VehicleState::kDimension, 1> state = Eigen::Matrix<float, VehicleState::kDimension, 1>::Zero();
        state(VehicleState::kPx) = _runtimeState.GetPositionX();
        state(VehicleState::kPy) = _runtimeState.GetPositionY();
        state(VehicleState::kHeading) = _runtimeState.GetHeading();
        state(VehicleState::kVf) = _runtimeState.GetForwardVelocity();
        state(VehicleState::kVr) = _runtimeState.GetRightwardVelocity();
        state(VehicleState::kYawRate) = _runtimeState.GetYawRate();
        state(VehicleState::kDeltaAf) = _runtimeState.GetForwardAccelerationResidual();
        state(VehicleState::kDeltaAr) = _runtimeState.GetRightwardAccelerationResidual();
        state(VehicleState::kDeltaYawAccel) = _runtimeState.GetYawAccelResidual();
        state(VehicleState::kHeading) = NormalizeAngle(state(VehicleState::kHeading));
        return state;
    }

    void PlantModel::ApplyStateVectorToBoundState(const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) noexcept
    {
        _runtimeState.SetPosition(Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)));
        _runtimeState.SetHeading(state(VehicleState::kHeading));
        _runtimeState.SetForwardVelocity(state(VehicleState::kVf));
        _runtimeState.SetRightwardVelocity(state(VehicleState::kVr));
        _runtimeState.SetYawRate(state(VehicleState::kYawRate));
        _runtimeState.SetForwardAccelerationResidual(state(VehicleState::kDeltaAf));
        _runtimeState.SetRightwardAccelerationResidual(state(VehicleState::kDeltaAr));
        _runtimeState.SetYawAccelResidual(state(VehicleState::kDeltaYawAccel));

        const Eigen::Vector2f wheelLinearVelocityMps = wheelLinearVelocityFromBodyState(state);
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        if (std::isfinite(wheelRadiusM) && (wheelRadiusM > 0.0f))
        {
            _runtimeState.SetWheelSpeedLeft(wheelLinearVelocityMps.x() / wheelRadiusM);
            _runtimeState.SetWheelSpeedRight(wheelLinearVelocityMps.y() / wheelRadiusM);
        }
    }

    void PlantModel::resolveAppliedBankTorques(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& currentState,
        const App::Internal::CommandVector& control,
        float& leftAppliedBankTorqueNm,
        float& rightAppliedBankTorqueNm,
        const EncoderObs* const encoderInput) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const Eigen::Vector2f wheelLinearVelocityMps = wheelLinearVelocityFromBodyState(currentState);
        const bool useEncoderInput =
            (encoderInput != nullptr) &&
            std::isfinite(encoderInput->LeftWheelSpeedRadps()) &&
            std::isfinite(encoderInput->RightWheelSpeedRadps());
        const float leftWheelSpeedRadps =
            useEncoderInput ?
            encoderInput->LeftWheelSpeedRadps() :
            ((std::isfinite(wheelRadiusM) && (wheelRadiusM > 0.0f)) ?
            (wheelLinearVelocityMps.x() / wheelRadiusM) :
            0.0f);
        const float rightWheelSpeedRadps =
            useEncoderInput ?
            encoderInput->RightWheelSpeedRadps() :
            ((std::isfinite(wheelRadiusM) && (wheelRadiusM > 0.0f)) ?
            (wheelLinearVelocityMps.y() / wheelRadiusM) :
            0.0f);
        const bool activeBrakeCommand = !control.IsFinite();
        const float leftMotorCommand =
            (!activeBrakeCommand && std::isfinite(control.LeftCommand())) ? control.LeftCommand() : 0.0f;
        const float rightMotorCommand =
            (!activeBrakeCommand && std::isfinite(control.RightCommand())) ? control.RightCommand() : 0.0f;
        const float batteryVoltageV = _vehicle.GetBatteryVoltage();
        const float staticLaunchTorqueNm = staticFrictionTorqueNm();

        const float leftDirectTorqueNm =
            activeBrakeCommand ?
            _leftDrive.getTorqueFromBrake(leftWheelSpeedRadps) :
            _leftDrive.getTorqueFromCommand(
                leftMotorCommand,
                leftWheelSpeedRadps,
                batteryVoltageV);
        float leftCurrentLimitedTorqueNm = leftDirectTorqueNm;
        if (!activeBrakeCommand)
        {
            const float leftPositiveLimitNm =
                (std::max)(
                    0.0f,
                    _leftDrive.getTorqueFromCommand(
                        1.0f,
                        leftWheelSpeedRadps,
                        batteryVoltageV));
            const float leftNegativeLimitNm =
                (std::min)(
                    0.0f,
                    _leftDrive.getTorqueFromCommand(
                        -1.0f,
                        leftWheelSpeedRadps,
                        batteryVoltageV));
            leftCurrentLimitedTorqueNm =
                (leftPositiveLimitNm > leftNegativeLimitNm) ?
                (std::clamp)(leftDirectTorqueNm, leftNegativeLimitNm, leftPositiveLimitNm) :
                leftDirectTorqueNm;
        }
        const float leftTorqueCorrectionDeltaNm = 0.0f;
        leftCurrentLimitedTorqueNm += leftTorqueCorrectionDeltaNm;
        const float leftWheelSurfaceSpeedMps = wheelRadiusM * leftWheelSpeedRadps;
        const float leftSlowLaunchRatio =
            (kStaticFrictionMaxSpeedMps > 0.0f) ?
            (std::fabs(leftWheelSurfaceSpeedMps) / kStaticFrictionMaxSpeedMps) :
            0.0f;
        const float leftLaunchTorqueNm =
            staticLaunchTorqueNm *
            std::exp(-(leftSlowLaunchRatio * leftSlowLaunchRatio));
        const float leftLaunchDirection =
            SignedDirection(leftCurrentLimitedTorqueNm, leftWheelSpeedRadps);
        leftAppliedBankTorqueNm = 0.0f;
        if (std::fabs(leftCurrentLimitedTorqueNm) > leftLaunchTorqueNm)
        {
            leftAppliedBankTorqueNm =
                leftCurrentLimitedTorqueNm -
                (leftLaunchDirection * leftLaunchTorqueNm);
        }
        const float leftLossDirection = SignedDirection(leftWheelSpeedRadps, leftAppliedBankTorqueNm);
        const float leftRollingLossTorqueNm =
            (kRollingFrictionTorqueNm * leftLossDirection) +
            (kViscousFrictionNmPerRadps * leftWheelSpeedRadps);
        leftAppliedBankTorqueNm -= leftRollingLossTorqueNm;
        if (!std::isfinite(leftAppliedBankTorqueNm))
        {
            leftAppliedBankTorqueNm = 0.0f;
        }

        const float rightDirectTorqueNm =
            activeBrakeCommand ?
            _rightDrive.getTorqueFromBrake(rightWheelSpeedRadps) :
            _rightDrive.getTorqueFromCommand(
                rightMotorCommand,
                rightWheelSpeedRadps,
                batteryVoltageV);
        float rightCurrentLimitedTorqueNm = rightDirectTorqueNm;
        if (!activeBrakeCommand)
        {
            const float rightPositiveLimitNm =
                (std::max)(
                    0.0f,
                    _rightDrive.getTorqueFromCommand(
                        1.0f,
                        rightWheelSpeedRadps,
                        batteryVoltageV));
            const float rightNegativeLimitNm =
                (std::min)(
                    0.0f,
                    _rightDrive.getTorqueFromCommand(
                        -1.0f,
                        rightWheelSpeedRadps,
                        batteryVoltageV));
            rightCurrentLimitedTorqueNm =
                (rightPositiveLimitNm > rightNegativeLimitNm) ?
                (std::clamp)(rightDirectTorqueNm, rightNegativeLimitNm, rightPositiveLimitNm) :
                rightDirectTorqueNm;
        }
        const float rightTorqueCorrectionDeltaNm = 0.0f;
        rightCurrentLimitedTorqueNm += rightTorqueCorrectionDeltaNm;
        const float rightWheelSurfaceSpeedMps = wheelRadiusM * rightWheelSpeedRadps;
        const float rightSlowLaunchRatio =
            (kStaticFrictionMaxSpeedMps > 0.0f) ?
            (std::fabs(rightWheelSurfaceSpeedMps) / kStaticFrictionMaxSpeedMps) :
            0.0f;
        const float rightLaunchTorqueNm =
            staticLaunchTorqueNm *
            std::exp(-(rightSlowLaunchRatio * rightSlowLaunchRatio));
        const float rightLaunchDirection =
            SignedDirection(rightCurrentLimitedTorqueNm, rightWheelSpeedRadps);
        rightAppliedBankTorqueNm = 0.0f;
        if (std::fabs(rightCurrentLimitedTorqueNm) > rightLaunchTorqueNm)
        {
            rightAppliedBankTorqueNm =
                rightCurrentLimitedTorqueNm -
                (rightLaunchDirection * rightLaunchTorqueNm);
        }
        const float rightLossDirection = SignedDirection(rightWheelSpeedRadps, rightAppliedBankTorqueNm);
        const float rightRollingLossTorqueNm =
            (kRollingFrictionTorqueNm * rightLossDirection) +
            (kViscousFrictionNmPerRadps * rightWheelSpeedRadps);
        rightAppliedBankTorqueNm -= rightRollingLossTorqueNm;
        if (!std::isfinite(rightAppliedBankTorqueNm))
        {
            rightAppliedBankTorqueNm = 0.0f;
        }
    }

    PlantModel::PlantDerivatives PlantModel::forwardStep(
        const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(BuildBoundStateVector(), control, nullptr);
    }

    PlantModel::PlantDerivatives PlantModel::forwardStep(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const App::Internal::CommandVector& control,
        const EncoderObs* const encoderInput) const noexcept
    {
        float leftDriveTorqueNm = 0.0f;
        float rightDriveTorqueNm = 0.0f;
        resolveAppliedBankTorques(
            state,
            control,
            leftDriveTorqueNm,
            rightDriveTorqueNm,
            encoderInput);
        return evaluateAppliedBankTorqueStep(
            state,
            leftDriveTorqueNm,
            rightDriveTorqueNm,
            encoderInput);
    }

    PlantModel::PlantDerivatives PlantModel::forwardStepFromAppliedBankTorques(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm) const noexcept
    {
        return evaluateAppliedBankTorqueStep(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            nullptr);
    }

    PlantModel::PlantDerivatives PlantModel::evaluateAppliedBankTorqueStep(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        const EncoderObs* const encoderInput) const noexcept
    {
        PlantDerivatives derivatives{};
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float massKg =
            (std::isfinite(_vehicle.GetMass()) && (_vehicle.GetMass() > 0.0f)) ?
            _vehicle.GetMass() :
            1.0f;
        const float invMassKg = 1.0f / massKg;
        const float yawInertiaKgM2 =
            (std::isfinite(_vehicle.GetYawInertia()) && (_vehicle.GetYawInertia() > 0.0f)) ?
            _vehicle.GetYawInertia() :
            1.0f;
        const float wheelSpinupMassKg =
            (wheelRadiusM > kForceEpsilonN) ?
            (((std::max)(0.0f, _leftDrive.getEquivalentWheelInertiaKgM2()) +
              (std::max)(0.0f, _rightDrive.getEquivalentWheelInertiaKgM2())) / (wheelRadiusM * wheelRadiusM)) :
            0.0f;

        const float forwardVelocityMps =
            std::isfinite(state(VehicleState::kVf)) ? state(VehicleState::kVf) : 0.0f;
        const float rightVelocityMps =
            std::isfinite(state(VehicleState::kVr)) ? state(VehicleState::kVr) : 0.0f;
        const float heading =
            std::isfinite(state(VehicleState::kHeading)) ? state(VehicleState::kHeading) : 0.0f;
        const float yawRateRadps =
            std::isfinite(state(VehicleState::kYawRate)) ? state(VehicleState::kYawRate) : 0.0f;
        const float forwardAccelResidualMps2 =
            std::isfinite(state(VehicleState::kDeltaAf)) ? state(VehicleState::kDeltaAf) : 0.0f;
        const float rightAccelResidualMps2 =
            std::isfinite(state(VehicleState::kDeltaAr)) ? state(VehicleState::kDeltaAr) : 0.0f;
        const float yawAccelResidualRadps2 =
            std::isfinite(state(VehicleState::kDeltaYawAccel)) ? state(VehicleState::kDeltaYawAccel) : 0.0f;
        derivatives._wheelKinematics = wheelKinematics(state, encoderInput);

        const float totalNormalLoadN =
            (massKg * GRAVITY_MPS2) +
            ((std::clamp)(_vehicle.GetFanDuty(), 0.0f, 1.0f) * _vehicle.GetFanDownforceAtFullDuty());
        const float frontWheelNormalLoadN = 0.5f * kFrontLoadFraction * totalNormalLoadN;
        const float rearWheelNormalLoadN = 0.5f * (1.0f - kFrontLoadFraction) * totalNormalLoadN;
        ContactForces contactForces{};
        ContactForce& frontLeftContact = contactForces._contacts[kFrontLeft];
        ContactForce& frontRightContact = contactForces._contacts[kFrontRight];
        ContactForce& rearLeftContact = contactForces._contacts[kRearLeft];
        ContactForce& rearRightContact = contactForces._contacts[kRearRight];
        frontLeftContact._normalForceN = frontWheelNormalLoadN;
        frontRightContact._normalForceN = frontWheelNormalLoadN;
        rearLeftContact._normalForceN = rearWheelNormalLoadN;
        rearRightContact._normalForceN = rearWheelNormalLoadN;

        const float leftDriveForceN =
            (wheelRadiusM > kForceEpsilonN) ?
            (leftAppliedBankTorqueNm / wheelRadiusM) :
            0.0f;
        const float rightDriveForceN =
            (wheelRadiusM > kForceEpsilonN) ?
            (rightAppliedBankTorqueNm / wheelRadiusM) :
            0.0f;
        const float leftBankNormalLoadN = frontLeftContact._normalForceN + rearLeftContact._normalForceN;
        const float rightBankNormalLoadN = frontRightContact._normalForceN + rearRightContact._normalForceN;
        const float frontLeftDriveRequestN =
            (leftBankNormalLoadN > kForceEpsilonN) ?
            (leftDriveForceN * (frontLeftContact._normalForceN / leftBankNormalLoadN)) :
            (0.5f * leftDriveForceN);
        const float rearLeftDriveRequestN =
            (leftBankNormalLoadN > kForceEpsilonN) ?
            (leftDriveForceN * (rearLeftContact._normalForceN / leftBankNormalLoadN)) :
            (0.5f * leftDriveForceN);
        const float frontRightDriveRequestN =
            (rightBankNormalLoadN > kForceEpsilonN) ?
            (rightDriveForceN * (frontRightContact._normalForceN / rightBankNormalLoadN)) :
            (0.5f * rightDriveForceN);
        const float rearRightDriveRequestN =
            (rightBankNormalLoadN > kForceEpsilonN) ?
            (rightDriveForceN * (rearRightContact._normalForceN / rightBankNormalLoadN)) :
            (0.5f * rightDriveForceN);

        const float leftLongitudinalStiffnessN = _leftDrive.getLongitudinalTireStiffnessN();
        const float rightLongitudinalStiffnessN = _rightDrive.getLongitudinalTireStiffnessN();
        const float frontLeftRightContactForceGainNPerMps =
            _leftDrive.getFrontRightContactForceGainNPerMps();
        const float frontRightRightContactForceGainNPerMps =
            _rightDrive.getFrontRightContactForceGainNPerMps();
        const float rearLeftRightContactForceGainNPerMps =
            _leftDrive.getRearRightContactForceGainNPerMps();
        const float rearRightRightContactForceGainNPerMps =
            _rightDrive.getRearRightContactForceGainNPerMps();
        const float frontLeftLongitudinalStiffnessNPerMps =
            (std::isfinite(leftLongitudinalStiffnessN) && (leftLongitudinalStiffnessN > 0.0f)) ?
            (0.5f * leftLongitudinalStiffnessN) :
            0.0f;
        const float frontRightLongitudinalStiffnessNPerMps =
            (std::isfinite(rightLongitudinalStiffnessN) && (rightLongitudinalStiffnessN > 0.0f)) ?
            (0.5f * rightLongitudinalStiffnessN) :
            0.0f;
        const float rearLeftLongitudinalStiffnessNPerMps = frontLeftLongitudinalStiffnessNPerMps;
        const float rearRightLongitudinalStiffnessNPerMps = frontRightLongitudinalStiffnessNPerMps;
        const float frontLeftRightContactForceGainNPerMpsResolved =
            (std::isfinite(frontLeftRightContactForceGainNPerMps) &&
             (frontLeftRightContactForceGainNPerMps > 0.0f)) ?
            frontLeftRightContactForceGainNPerMps :
            0.0f;
        const float frontRightRightContactForceGainNPerMpsResolved =
            (std::isfinite(frontRightRightContactForceGainNPerMps) &&
             (frontRightRightContactForceGainNPerMps > 0.0f)) ?
            frontRightRightContactForceGainNPerMps :
            0.0f;
        const float rearLeftRightContactForceGainNPerMpsResolved =
            (std::isfinite(rearLeftRightContactForceGainNPerMps) &&
             (rearLeftRightContactForceGainNPerMps > 0.0f)) ?
            rearLeftRightContactForceGainNPerMps :
            0.0f;
        const float rearRightRightContactForceGainNPerMpsResolved =
            (std::isfinite(rearRightRightContactForceGainNPerMps) &&
             (rearRightRightContactForceGainNPerMps > 0.0f)) ?
            rearRightRightContactForceGainNPerMps :
            0.0f;

        const float sustainedContactMu =
            (std::isfinite(Vehicle::GetSustainedLateralAccelerationReferenceMps2()) &&
             (Vehicle::GetSustainedLateralAccelerationReferenceMps2() > 0.0f) &&
             (totalNormalLoadN > kForceEpsilonN)) ?
            ((Vehicle::GetSustainedLateralAccelerationReferenceMps2() * massKg) / totalNormalLoadN) :
            0.0f;
        const float peakFrontMu = (sustainedContactMu > 0.0f) ? sustainedContactMu : kMuFront;
        const float peakRearMu = (sustainedContactMu > 0.0f) ? sustainedContactMu : kMuRear;

        float frontLeftRawForwardForceN =
            frontLeftDriveRequestN +
            (frontLeftLongitudinalStiffnessNPerMps *
                derivatives._wheelKinematics._contacts[kFrontLeft]._forwardRelativeVelocityMps);
        float frontRightRawForwardForceN =
            frontRightDriveRequestN +
            (frontRightLongitudinalStiffnessNPerMps *
                derivatives._wheelKinematics._contacts[kFrontRight]._forwardRelativeVelocityMps);
        float rearLeftRawForwardForceN =
            rearLeftDriveRequestN +
            (rearLeftLongitudinalStiffnessNPerMps *
                derivatives._wheelKinematics._contacts[kRearLeft]._forwardRelativeVelocityMps);
        float rearRightRawForwardForceN =
            rearRightDriveRequestN +
            (rearRightLongitudinalStiffnessNPerMps *
                derivatives._wheelKinematics._contacts[kRearRight]._forwardRelativeVelocityMps);
        float frontLeftRawRightForceN =
            frontLeftRightContactForceGainNPerMpsResolved *
            derivatives._wheelKinematics._contacts[kFrontLeft]._rightRelativeVelocityMps;
        float frontRightRawRightForceN =
            frontRightRightContactForceGainNPerMpsResolved *
            derivatives._wheelKinematics._contacts[kFrontRight]._rightRelativeVelocityMps;
        float rearLeftRawRightForceN =
            rearLeftRightContactForceGainNPerMpsResolved *
            derivatives._wheelKinematics._contacts[kRearLeft]._rightRelativeVelocityMps;
        float rearRightRawRightForceN =
            rearRightRightContactForceGainNPerMpsResolved *
            derivatives._wheelKinematics._contacts[kRearRight]._rightRelativeVelocityMps;

        const float contactCorrectionHalfTrackWidthM = 0.5f * std::fabs(_vehicle.GetTrackWidth());
        const float contactCorrectionLongitudinalOffsetM =
            std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());
        const std::array<float, 4> contactRightPositionsM = {
            -contactCorrectionHalfTrackWidthM,
            contactCorrectionHalfTrackWidthM,
            -contactCorrectionHalfTrackWidthM,
            contactCorrectionHalfTrackWidthM
        };
        const std::array<float, 4> contactForwardPositionsM = {
            contactCorrectionLongitudinalOffsetM,
            contactCorrectionLongitudinalOffsetM,
            -contactCorrectionLongitudinalOffsetM,
            -contactCorrectionLongitudinalOffsetM
        };
        const float resolvedTotalNormalLoadN =
            (totalNormalLoadN > kForceEpsilonN) ? totalNormalLoadN : kForceEpsilonN;
        float loadWeightedPatchYawVelocityM2ps = 0.0f;
        float loadWeightedPatchRadiusSquaredM2 = 0.0f;
        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const ContactForce& contact = contactForces._contacts[contactIndex];
            const ContactKinematics& kinematics = derivatives._wheelKinematics._contacts[contactIndex];
            const float rightPositionM = contactRightPositionsM[contactIndex];
            const float forwardPositionM = contactForwardPositionsM[contactIndex];
            loadWeightedPatchYawVelocityM2ps +=
                contact._normalForceN *
                ((forwardPositionM * kinematics._rightRelativeVelocityMps) -
                 (rightPositionM * kinematics._forwardRelativeVelocityMps));
            loadWeightedPatchRadiusSquaredM2 +=
                contact._normalForceN *
                ((rightPositionM * rightPositionM) + (forwardPositionM * forwardPositionM));
        }
        const float patchYawVelocityM2ps =
            loadWeightedPatchYawVelocityM2ps / resolvedTotalNormalLoadN;
        const float patchRadiusSquaredM2 =
            (std::max)(
                loadWeightedPatchRadiusSquaredM2 / resolvedTotalNormalLoadN,
                kForceEpsilonN * kForceEpsilonN);
        const float contactYawCorrectionScale =
            kContactYawPatchForceGainNsPerM *
            patchYawVelocityM2ps /
            patchRadiusSquaredM2;
        frontLeftRawForwardForceN -=
            (frontLeftContact._normalForceN / resolvedTotalNormalLoadN) *
            contactRightPositionsM[kFrontLeft] *
            contactYawCorrectionScale;
        frontRightRawForwardForceN -=
            (frontRightContact._normalForceN / resolvedTotalNormalLoadN) *
            contactRightPositionsM[kFrontRight] *
            contactYawCorrectionScale;
        rearLeftRawForwardForceN -=
            (rearLeftContact._normalForceN / resolvedTotalNormalLoadN) *
            contactRightPositionsM[kRearLeft] *
            contactYawCorrectionScale;
        rearRightRawForwardForceN -=
            (rearRightContact._normalForceN / resolvedTotalNormalLoadN) *
            contactRightPositionsM[kRearRight] *
            contactYawCorrectionScale;
        frontLeftRawRightForceN +=
            (frontLeftContact._normalForceN / resolvedTotalNormalLoadN) *
            contactForwardPositionsM[kFrontLeft] *
            contactYawCorrectionScale;
        frontRightRawRightForceN +=
            (frontRightContact._normalForceN / resolvedTotalNormalLoadN) *
            contactForwardPositionsM[kFrontRight] *
            contactYawCorrectionScale;
        rearLeftRawRightForceN +=
            (rearLeftContact._normalForceN / resolvedTotalNormalLoadN) *
            contactForwardPositionsM[kRearLeft] *
            contactYawCorrectionScale;
        rearRightRawRightForceN +=
            (rearRightContact._normalForceN / resolvedTotalNormalLoadN) *
            contactForwardPositionsM[kRearRight] *
            contactYawCorrectionScale;

        const float frontLeftRawMagnitudeN =
            MazeMap::Math::Sqrtf(
                (frontLeftRawForwardForceN * frontLeftRawForwardForceN) +
                (frontLeftRawRightForceN * frontLeftRawRightForceN));
        const float frontLeftMaxForceN = (std::max)(0.0f, peakFrontMu * frontLeftContact._normalForceN);
        const float frontLeftScale =
            (frontLeftRawMagnitudeN > frontLeftMaxForceN && frontLeftRawMagnitudeN > kForceEpsilonN) ?
            (frontLeftMaxForceN / frontLeftRawMagnitudeN) :
            1.0f;
        frontLeftContact._forwardForceN = frontLeftScale * frontLeftRawForwardForceN;
        frontLeftContact._rightForceN = frontLeftScale * frontLeftRawRightForceN;
        frontLeftContact._preProjectionUtilization =
            frontLeftRawMagnitudeN / (std::max)(frontLeftMaxForceN, kForceEpsilonN);
        frontLeftContact._saturation = (std::min)(frontLeftContact._preProjectionUtilization, 1.0f);

        const float frontRightRawMagnitudeN =
            MazeMap::Math::Sqrtf(
                (frontRightRawForwardForceN * frontRightRawForwardForceN) +
                (frontRightRawRightForceN * frontRightRawRightForceN));
        const float frontRightMaxForceN = (std::max)(0.0f, peakFrontMu * frontRightContact._normalForceN);
        const float frontRightScale =
            (frontRightRawMagnitudeN > frontRightMaxForceN && frontRightRawMagnitudeN > kForceEpsilonN) ?
            (frontRightMaxForceN / frontRightRawMagnitudeN) :
            1.0f;
        frontRightContact._forwardForceN = frontRightScale * frontRightRawForwardForceN;
        frontRightContact._rightForceN = frontRightScale * frontRightRawRightForceN;
        frontRightContact._preProjectionUtilization =
            frontRightRawMagnitudeN / (std::max)(frontRightMaxForceN, kForceEpsilonN);
        frontRightContact._saturation = (std::min)(frontRightContact._preProjectionUtilization, 1.0f);

        const float rearLeftRawMagnitudeN =
            MazeMap::Math::Sqrtf(
                (rearLeftRawForwardForceN * rearLeftRawForwardForceN) +
                (rearLeftRawRightForceN * rearLeftRawRightForceN));
        const float rearLeftMaxForceN = (std::max)(0.0f, peakRearMu * rearLeftContact._normalForceN);
        const float rearLeftScale =
            (rearLeftRawMagnitudeN > rearLeftMaxForceN && rearLeftRawMagnitudeN > kForceEpsilonN) ?
            (rearLeftMaxForceN / rearLeftRawMagnitudeN) :
            1.0f;
        rearLeftContact._forwardForceN = rearLeftScale * rearLeftRawForwardForceN;
        rearLeftContact._rightForceN = rearLeftScale * rearLeftRawRightForceN;
        rearLeftContact._preProjectionUtilization =
            rearLeftRawMagnitudeN / (std::max)(rearLeftMaxForceN, kForceEpsilonN);
        rearLeftContact._saturation = (std::min)(rearLeftContact._preProjectionUtilization, 1.0f);

        const float rearRightRawMagnitudeN =
            MazeMap::Math::Sqrtf(
                (rearRightRawForwardForceN * rearRightRawForwardForceN) +
                (rearRightRawRightForceN * rearRightRawRightForceN));
        const float rearRightMaxForceN = (std::max)(0.0f, peakRearMu * rearRightContact._normalForceN);
        const float rearRightScale =
            (rearRightRawMagnitudeN > rearRightMaxForceN && rearRightRawMagnitudeN > kForceEpsilonN) ?
            (rearRightMaxForceN / rearRightRawMagnitudeN) :
            1.0f;
        rearRightContact._forwardForceN = rearRightScale * rearRightRawForwardForceN;
        rearRightContact._rightForceN = rearRightScale * rearRightRawRightForceN;
        rearRightContact._preProjectionUtilization =
            rearRightRawMagnitudeN / (std::max)(rearRightMaxForceN, kForceEpsilonN);
        rearRightContact._saturation = (std::min)(rearRightContact._preProjectionUtilization, 1.0f);

        const std::array<float, 4> projectedForwardForceN = {
            frontLeftContact._forwardForceN,
            frontRightContact._forwardForceN,
            rearLeftContact._forwardForceN,
            rearRightContact._forwardForceN
        };
        const std::array<float, 4> projectedRightForceN = {
            frontLeftContact._rightForceN,
            frontRightContact._rightForceN,
            rearLeftContact._rightForceN,
            rearRightContact._rightForceN
        };
        const std::array<float, 4> contactNormalLoadN = {
            frontLeftContact._normalForceN,
            frontRightContact._normalForceN,
            rearLeftContact._normalForceN,
            rearRightContact._normalForceN
        };
        const std::array<float, 4> projectedMaxForceN = {
            frontLeftMaxForceN,
            frontRightMaxForceN,
            rearLeftMaxForceN,
            rearRightMaxForceN
        };
        float maxProjectedContactUtilization = 0.0f;
        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const float projectedMagnitudeN =
                MazeMap::Math::Sqrtf(
                    (projectedForwardForceN[contactIndex] * projectedForwardForceN[contactIndex]) +
                    (projectedRightForceN[contactIndex] * projectedRightForceN[contactIndex]));
            maxProjectedContactUtilization =
                (std::max)(
                    maxProjectedContactUtilization,
                    projectedMagnitudeN / (std::max)(projectedMaxForceN[contactIndex], kForceEpsilonN));
        }
        const float leftBankForwardForceN = frontLeftContact._forwardForceN + rearLeftContact._forwardForceN;
        const float rightBankForwardForceN = frontRightContact._forwardForceN + rearRightContact._forwardForceN;
        const float frontRightForceN = frontLeftContact._rightForceN + frontRightContact._rightForceN;
        const float rearRightForceN = rearLeftContact._rightForceN + rearRightContact._rightForceN;
        const float sumForwardForceN = leftBankForwardForceN + rightBankForwardForceN;
        const float sumRightForceN = frontRightForceN + rearRightForceN;
        derivatives._maxContactUtilization =
            (std::max)(
                (std::max)(frontLeftContact._preProjectionUtilization, frontRightContact._preProjectionUtilization),
                (std::max)(rearLeftContact._preProjectionUtilization, rearRightContact._preProjectionUtilization));

        const float contactYawHalfTrackWidthM = 0.5f * std::fabs(_vehicle.GetTrackWidth());
        const float contactYawLongitudinalOffsetM =
            std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());
        const float projectedYawMomentNm =
            (contactYawHalfTrackWidthM * (leftBankForwardForceN - rightBankForwardForceN)) +
            (contactYawLongitudinalOffsetM * (frontRightForceN - rearRightForceN));
        const float yawDirection = SignedDirection(yawRateRadps, projectedYawMomentNm);
        const float variantCContactYawMomentCorrectionAlongYawNm =
            variantCAggregateContactYawMomentCorrectionAlongYawNm(
                yawDirection,
                forwardVelocityMps,
                derivatives._wheelKinematics,
                contactRightPositionsM,
                contactForwardPositionsM,
                contactNormalLoadN,
                projectedForwardForceN,
                projectedRightForceN,
                totalNormalLoadN,
                maxProjectedContactUtilization);
        const float contactRelativeSpeedMps =
            loadWeightedContactRelativeSpeedMps(
                derivatives._wheelKinematics,
                contactNormalLoadN,
                totalNormalLoadN);
        const float yawMomentYieldNm =
            (std::max)(0.0f, sustainedContactMu * contactYawHalfTrackWidthM * totalNormalLoadN);
        const float contactYawMomentCorrectionAlongYawNm =
            aggregateContactYawMomentCorrectionAlongYawNm(
                forwardVelocityMps,
                contactRelativeSpeedMps,
                PositivePart(yawDirection * projectedYawMomentNm),
                yawMomentYieldNm,
                variantCContactYawMomentCorrectionAlongYawNm);
        const float yawMomentNm =
            projectedYawMomentNm -
            (yawDirection * contactYawMomentCorrectionAlongYawNm);

        float sineHeading = 0.0f;
        float cosineHeading = 0.0f;
        sin_cosf(heading, sineHeading, cosineHeading);

        const float rawForwardAccelMps2 = sumForwardForceN / (massKg + wheelSpinupMassKg);
        const float rawRightAccelMps2 =
            (sumRightForceN * invMassKg) -
            (kRightVelocityDampingNsPerM * rightVelocityMps * invMassKg);
        const float rawYawAccelRadps2 =
            yawMomentNm /
            (yawInertiaKgM2 + (wheelSpinupMassKg * contactYawHalfTrackWidthM * contactYawHalfTrackWidthM));
        const float maxForwardAccelMps2 =
            (std::isfinite(_vehicle.GetMaxForwardAcceleration()) &&
             (_vehicle.GetMaxForwardAcceleration() > 0.0f)) ?
            _vehicle.GetMaxForwardAcceleration() :
            0.0f;
        const float forwardAccelMps2 =
            (maxForwardAccelMps2 > 0.0f) ?
            (std::clamp)(rawForwardAccelMps2, -maxForwardAccelMps2, maxForwardAccelMps2) :
            rawForwardAccelMps2;
        const float rightAccelMps2 = rawRightAccelMps2;
        const float yawAccelRadps2 = rawYawAccelRadps2;
        const float resolvedForwardAccelMps2 = forwardAccelMps2 + forwardAccelResidualMps2;
        const float resolvedRightAccelMps2 = rightAccelMps2 + rightAccelResidualMps2;
        const float resolvedYawAccelRadps2 = yawAccelRadps2 + yawAccelResidualRadps2;

        derivatives._stateDot(VehicleState::kPx) =
            (rightVelocityMps * cosineHeading) + (forwardVelocityMps * sineHeading);
        derivatives._stateDot(VehicleState::kPy) =
            (-rightVelocityMps * sineHeading) + (forwardVelocityMps * cosineHeading);
        derivatives._stateDot(VehicleState::kHeading) = yawRateRadps;
        derivatives._stateDot(VehicleState::kVf) = resolvedForwardAccelMps2 + (yawRateRadps * rightVelocityMps);
        derivatives._stateDot(VehicleState::kVr) = resolvedRightAccelMps2 - (yawRateRadps * forwardVelocityMps);
        derivatives._stateDot(VehicleState::kYawRate) = resolvedYawAccelRadps2;
        derivatives._stateDot(VehicleState::kDeltaAf) =
            -forwardAccelResidualMps2 / kForwardAccelerationResidualDecayTauS;
        derivatives._stateDot(VehicleState::kDeltaAr) =
            -rightAccelResidualMps2 / kRightAccelerationResidualDecayTauS;
        derivatives._stateDot(VehicleState::kDeltaYawAccel) =
            -yawAccelResidualRadps2 / kYawAccelResidualDecayTauS;

        derivatives._contactForces = contactForces;
        derivatives._originAccelBodyMps2 = Eigen::Vector2f(resolvedRightAccelMps2, resolvedForwardAccelMps2);

        const Eigen::Vector2f imuLeverArmBodyM = Vehicle::GetBackLeftImuMount().positionBodyM();
        const float yawRateSquaredRadps2 = yawRateRadps * yawRateRadps;
        derivatives._imuAccelBodyMps2 = Eigen::Vector2f(
            resolvedRightAccelMps2 -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.x()) +
                (resolvedYawAccelRadps2 * imuLeverArmBodyM.y()),
            resolvedForwardAccelMps2 -
                (yawRateSquaredRadps2 * imuLeverArmBodyM.y()) -
                (resolvedYawAccelRadps2 * imuLeverArmBodyM.x()));
        derivatives._forwardAccelMps2 = resolvedForwardAccelMps2;
        derivatives._rightAccelMps2 = resolvedRightAccelMps2;
        derivatives._yawAccelRadps2 = resolvedYawAccelRadps2;
        return derivatives;
    }
    Eigen::Matrix<float, VehicleState::kDimension, 1> PlantModel::integrateAppliedBankTorques(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        float leftAppliedBankTorqueNm,
        float rightAppliedBankTorqueNm,
        float dtS,
        const EncoderObs* const encoderInput) const noexcept
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
                encoderInput);
        return advanceStateFromDerivatives(state, evaluatedStep, dtS);
    }

    Eigen::Matrix<float, VehicleState::kDimension, 1> PlantModel::predictStateFromCommandReference(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& referenceState,
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const App::Internal::CommandVector& control,
        const float dtS,
        const EncoderObs* const encoderInput) const noexcept
    {
        float leftAppliedBankTorqueNm = 0.0f;
        float rightAppliedBankTorqueNm = 0.0f;
        resolveAppliedBankTorques(
            referenceState,
            control,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            encoderInput);
        return predictStateWithAppliedBankTorques(
            state,
            leftAppliedBankTorqueNm,
            rightAppliedBankTorqueNm,
            dtS,
            encoderInput);
    }

    Eigen::Matrix<float, VehicleState::kDimension, 1> PlantModel::predictStateWithAppliedBankTorques(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const float leftAppliedBankTorqueNm,
        const float rightAppliedBankTorqueNm,
        const float dtS,
        const EncoderObs* const encoderInput) const noexcept
    {
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return state;
        }

        const float forwardDecayAlpha = forwardAccelerationResidualDecayAlpha(dtS);
        const float rightDecayAlpha = rightAccelerationResidualDecayAlpha(dtS);
        const float yawAccelDecayAlpha = yawAccelerationResidualDecayAlpha(dtS);
        const float forwardAccelerationResidualMps2 =
            std::isfinite(state(VehicleState::kDeltaAf)) ? state(VehicleState::kDeltaAf) : 0.0f;
        const float rightAccelerationResidualMps2 =
            std::isfinite(state(VehicleState::kDeltaAr)) ? state(VehicleState::kDeltaAr) : 0.0f;
        const float yawAccelerationResidualRadps2 =
            std::isfinite(state(VehicleState::kDeltaYawAccel)) ? state(VehicleState::kDeltaYawAccel) : 0.0f;

        Eigen::Matrix<float, VehicleState::kDimension, 1> propagationState = state;
        propagationState(VehicleState::kDeltaAf) =
            0.5f * (1.0f + forwardDecayAlpha) * forwardAccelerationResidualMps2;
        propagationState(VehicleState::kDeltaAr) =
            0.5f * (1.0f + rightDecayAlpha) * rightAccelerationResidualMps2;
        propagationState(VehicleState::kDeltaYawAccel) =
            0.5f * (1.0f + yawAccelDecayAlpha) * yawAccelerationResidualRadps2;

        Eigen::Matrix<float, VehicleState::kDimension, 1> nextState =
            integrateAppliedBankTorques(
                propagationState,
                leftAppliedBankTorqueNm,
                rightAppliedBankTorqueNm,
                dtS,
                encoderInput);
        nextState(VehicleState::kDeltaAf) = forwardDecayAlpha * forwardAccelerationResidualMps2;
        nextState(VehicleState::kDeltaAr) = rightDecayAlpha * rightAccelerationResidualMps2;
        nextState(VehicleState::kDeltaYawAccel) = yawAccelDecayAlpha * yawAccelerationResidualRadps2;
        nextState(VehicleState::kHeading) = NormalizeAngle(nextState(VehicleState::kHeading));
        return nextState;
    }

    void PlantModel::plantActivityForState(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const App::Internal::CommandVector& control,
        const EncoderObs* const encoderInput,
        float& forwardAccelMps2,
        float& rightAccelMps2,
        float& yawAccelRadps2,
        float& maxContactRelativeSpeedMps,
        float& maxContactUtilization,
        float& maxContactSaturation,
        float& totalNormalLoadN) const noexcept
    {
        const PlantDerivatives derivatives = forwardStep(state, control, encoderInput);
        forwardAccelMps2 = derivatives._forwardAccelMps2;
        rightAccelMps2 = derivatives._rightAccelMps2;
        yawAccelRadps2 = derivatives._yawAccelRadps2;
        maxContactRelativeSpeedMps = 0.0f;
        for (const ContactKinematics& contact : derivatives._wheelKinematics._contacts)
        {
            maxContactRelativeSpeedMps =
                (std::max)(
                    maxContactRelativeSpeedMps,
                    (std::max)(
                        std::fabs(contact._forwardRelativeVelocityMps),
                        std::fabs(contact._rightRelativeVelocityMps)));
        }

        maxContactSaturation = 0.0f;
        totalNormalLoadN = 0.0f;
        for (const ContactForce& contact : derivatives._contactForces._contacts)
        {
            maxContactSaturation = (std::max)(maxContactSaturation, contact._saturation);
            totalNormalLoadN += (std::max)(0.0f, contact._normalForceN);
        }

        maxContactUtilization = derivatives._maxContactUtilization;
    }

    Eigen::Vector2f PlantModel::backLeftImuPlanarAccelerationForState(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const App::Internal::CommandVector& control,
        const EncoderObs* const encoderInput) const noexcept
    {
        return imuPlanarAcceleration(state, control, encoderInput);
    }

    Eigen::Matrix<float, VehicleState::kDimension, 1> PlantModel::advanceStateFromDerivatives(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& currentState,
        const PlantDerivatives& evaluatedStep,
        float dtS) noexcept
    {
        Eigen::Matrix<float, VehicleState::kDimension, 1> nextState = currentState;
        if (!(std::isfinite(dtS) && (dtS > 0.0f)))
        {
            return nextState;
        }

        nextState(VehicleState::kVf) += dtS * evaluatedStep._stateDot(VehicleState::kVf);
        nextState(VehicleState::kVr) += dtS * evaluatedStep._stateDot(VehicleState::kVr);
        nextState(VehicleState::kYawRate) += dtS * evaluatedStep._stateDot(VehicleState::kYawRate);
        nextState(VehicleState::kDeltaAf) =
            currentState(VehicleState::kDeltaAf) *
            std::exp(-dtS / kForwardAccelerationResidualDecayTauS);
        nextState(VehicleState::kDeltaAr) =
            currentState(VehicleState::kDeltaAr) *
            std::exp(-dtS / kRightAccelerationResidualDecayTauS);
        nextState(VehicleState::kDeltaYawAccel) =
            currentState(VehicleState::kDeltaYawAccel) *
            std::exp(-dtS / kYawAccelResidualDecayTauS);

        nextState(VehicleState::kHeading) =
            NormalizeAngle(
                currentState(VehicleState::kHeading) + (dtS * nextState(VehicleState::kYawRate)));

        float sineHeading = 0.0f;
        float cosineHeading = 0.0f;
        sin_cosf(nextState(VehicleState::kHeading), sineHeading, cosineHeading);
        const float worldRightVelocityMps =
            (nextState(VehicleState::kVr) * cosineHeading) +
            (nextState(VehicleState::kVf) * sineHeading);
        const float worldForwardVelocityMps =
            (-nextState(VehicleState::kVr) * sineHeading) +
            (nextState(VehicleState::kVf) * cosineHeading);
        nextState(VehicleState::kPx) += dtS * worldRightVelocityMps;
        nextState(VehicleState::kPy) += dtS * worldForwardVelocityMps;

        nextState(VehicleState::kHeading) = NormalizeAngle(nextState(VehicleState::kHeading));
        return nextState;
    }

    PlantModel::WheelKinematics PlantModel::wheelKinematics(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const EncoderObs* const encoderInput) const noexcept
    {
        WheelKinematics kinematics{};
        const float forwardVelocityMps = state(VehicleState::kVf);
        const float rightVelocityMps = state(VehicleState::kVr);
        const float yawRateRadps = state(VehicleState::kYawRate);
        const Eigen::Vector2f wheelLinearVelocityMps = wheelLinearVelocityFromBodyState(state);
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const bool useEncoderInput =
            (encoderInput != nullptr) &&
            std::isfinite(encoderInput->LeftWheelSpeedRadps()) &&
            std::isfinite(encoderInput->RightWheelSpeedRadps());
        const bool useEncoderLinearVelocity =
            useEncoderInput &&
            std::isfinite(encoderInput->LeftVelocityMps()) &&
            std::isfinite(encoderInput->RightVelocityMps());
        const float halfTrackWidthM = 0.5f * std::fabs(_vehicle.GetTrackWidth());
        const float longitudinalOffsetM =
            std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());
        const float leftBodyForwardVelocityMps = forwardVelocityMps + (halfTrackWidthM * yawRateRadps);
        const float rightBodyForwardVelocityMps = forwardVelocityMps - (halfTrackWidthM * yawRateRadps);
        const float leftSurfaceVelocityMps =
            useEncoderLinearVelocity ?
            encoderInput->LeftVelocityMps() :
            ((useEncoderInput && std::isfinite(wheelRadiusM) && (wheelRadiusM > 0.0f)) ?
                (wheelRadiusM * encoderInput->LeftWheelSpeedRadps()) :
                wheelLinearVelocityMps.x());
        const float rightSurfaceVelocityMps =
            useEncoderLinearVelocity ?
            encoderInput->RightVelocityMps() :
            ((useEncoderInput && std::isfinite(wheelRadiusM) && (wheelRadiusM > 0.0f)) ?
                (wheelRadiusM * encoderInput->RightWheelSpeedRadps()) :
                wheelLinearVelocityMps.y());
        const float frontRightBodyVelocityMps = rightVelocityMps + (longitudinalOffsetM * yawRateRadps);
        const float rearRightBodyVelocityMps = rightVelocityMps - (longitudinalOffsetM * yawRateRadps);

        kinematics._leftBankForwardRelativeVelocityMps =
            leftSurfaceVelocityMps - leftBodyForwardVelocityMps;
        kinematics._rightBankForwardRelativeVelocityMps =
            rightSurfaceVelocityMps - rightBodyForwardVelocityMps;

        kinematics._contacts[kFrontLeft]._forwardRelativeVelocityMps =
            kinematics._leftBankForwardRelativeVelocityMps;
        kinematics._contacts[kFrontRight]._forwardRelativeVelocityMps =
            kinematics._rightBankForwardRelativeVelocityMps;
        kinematics._contacts[kRearLeft]._forwardRelativeVelocityMps =
            kinematics._leftBankForwardRelativeVelocityMps;
        kinematics._contacts[kRearRight]._forwardRelativeVelocityMps =
            kinematics._rightBankForwardRelativeVelocityMps;

        kinematics._contacts[kFrontLeft]._rightRelativeVelocityMps = -frontRightBodyVelocityMps;
        kinematics._contacts[kFrontRight]._rightRelativeVelocityMps = -frontRightBodyVelocityMps;
        kinematics._contacts[kRearLeft]._rightRelativeVelocityMps = -rearRightBodyVelocityMps;
        kinematics._contacts[kRearRight]._rightRelativeVelocityMps = -rearRightBodyVelocityMps;
        return kinematics;
    }

    Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const Eigen::Matrix<float, VehicleState::kDimension, 1>& state,
        const App::Internal::CommandVector& control,
        const EncoderObs* const encoderInput) const noexcept
    {
        return forwardStep(state, control, encoderInput)._imuAccelBodyMps2;
    }

    float PlantModel::contactSaturation(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = forwardStep(BuildBoundStateVector(), control, nullptr)._contactForces;
        return (contactIndex < forces._contacts.size()) ?
            forces._contacts[contactIndex]._saturation :
            0.0f;
    }

    float PlantModel::contactPreProjectionUtilization(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = forwardStep(BuildBoundStateVector(), control, nullptr)._contactForces;
        return (contactIndex < forces._contacts.size()) ?
            forces._contacts[contactIndex]._preProjectionUtilization :
            0.0f;
    }

    float PlantModel::contactForwardRelativeVelocityMps(uint8_t contactIndex) const noexcept
    {
        const WheelKinematics kinematics = wheelKinematics(BuildBoundStateVector(), nullptr);
        return (contactIndex < kinematics._contacts.size()) ?
            kinematics._contacts[contactIndex]._forwardRelativeVelocityMps :
            0.0f;
    }

    float PlantModel::contactRightRelativeVelocityMps(uint8_t contactIndex) const noexcept
    {
        const WheelKinematics kinematics = wheelKinematics(BuildBoundStateVector(), nullptr);
        return (contactIndex < kinematics._contacts.size()) ?
            kinematics._contacts[contactIndex]._rightRelativeVelocityMps :
            0.0f;
    }

    float PlantModel::backLeftImuRightAccelerationMps2(
        const App::Internal::CommandVector& control) const noexcept
    {
        return imuPlanarAcceleration(BuildBoundStateVector(), control, nullptr).x();
    }

    float PlantModel::backLeftImuForwardAccelerationMps2(
        const App::Internal::CommandVector& control) const noexcept
    {
        return imuPlanarAcceleration(BuildBoundStateVector(), control, nullptr).y();
    }

    void PlantModel::integrate(const App::Internal::CommandVector& control, float dt) noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return;
        }

        const Eigen::Matrix<float, VehicleState::kDimension, 1> currentState = BuildBoundStateVector();
        const PlantDerivatives derivatives = forwardStep(currentState, control, nullptr);
        Eigen::Matrix<float, VehicleState::kDimension, 1> nextState = currentState + (dt * derivatives._stateDot);
        nextState(VehicleState::kDeltaAf) =
            currentState(VehicleState::kDeltaAf) *
            std::exp(-dt / kForwardAccelerationResidualDecayTauS);
        nextState(VehicleState::kDeltaAr) =
            currentState(VehicleState::kDeltaAr) *
            std::exp(-dt / kRightAccelerationResidualDecayTauS);
        nextState(VehicleState::kDeltaYawAccel) =
            currentState(VehicleState::kDeltaYawAccel) *
            std::exp(-dt / kYawAccelResidualDecayTauS);
        nextState(VehicleState::kHeading) = NormalizeAngle(nextState(VehicleState::kHeading));

        ApplyStateVectorToBoundState(nextState);
        _runtimeState.SetTime(_runtimeState.GetTime() + dt);
        _runtimeState.SetCurrentCommand(control);
        _runtimeState.SetForwardAcceleration(derivatives._forwardAccelMps2);
        _runtimeState.SetRightAcceleration(derivatives._rightAccelMps2);
        _runtimeState.SetYawAccel(derivatives._yawAccelRadps2);
    }

    App::Internal::CommandVector PlantModel::ComputeFeedforward(
        float desiredAccelMps2,
        float desiredYawAccelRadps2) const noexcept
    {
        const bool hasForwardAccelObjective = !std::isnan(desiredAccelMps2);
        const bool hasYawAccelObjective = !std::isnan(desiredYawAccelRadps2);
        const bool maximizeForwardAccel =
            hasForwardAccelObjective && std::isinf(desiredAccelMps2);
        const bool maximizeYawAccel =
            hasYawAccelObjective && std::isinf(desiredYawAccelRadps2);
        if (maximizeForwardAccel || maximizeYawAccel)
        {
            float maxLongitudinalAccelMps2 = 0.0f;
            float maxYawAccelRadps2 = 0.0f;
            velocityTargetTechnicalLimits(maxLongitudinalAccelMps2, maxYawAccelRadps2);

            if (maximizeForwardAccel)
            {
                const float resolvedLimit =
                    (std::isfinite(maxLongitudinalAccelMps2) && (maxLongitudinalAccelMps2 > 0.0f)) ?
                    maxLongitudinalAccelMps2 :
                    0.0f;
                desiredAccelMps2 = std::signbit(desiredAccelMps2) ? -resolvedLimit : resolvedLimit;
            }

            if (maximizeYawAccel)
            {
                const float resolvedLimit =
                    (std::isfinite(maxYawAccelRadps2) && (maxYawAccelRadps2 > 0.0f)) ?
                    maxYawAccelRadps2 :
                    0.0f;
                desiredYawAccelRadps2 = std::signbit(desiredYawAccelRadps2) ? -resolvedLimit : resolvedLimit;
            }
        }

        // NaN is the caller's "no objective for this axis" token. The neutral
        // feedforward component is the minimum-commitment solution for that unconstrained axis.
        if (!hasForwardAccelObjective)
        {
            desiredAccelMps2 = 0.0f;
        }
        if (!hasYawAccelObjective)
        {
            desiredYawAccelRadps2 = 0.0f;
        }

        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float invWheelRadiusM = (wheelRadiusM > 0.0f) ? (1.0f / wheelRadiusM) : 0.0f;
        const float trackWidthM = std::fabs(_vehicle.GetTrackWidth());
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float invTrackWidthM = (trackWidthM > 0.0f) ? (1.0f / trackWidthM) : 0.0f;
        const float massKg =
            (std::isfinite(_vehicle.GetMass()) && (_vehicle.GetMass() > 0.0f)) ?
            _vehicle.GetMass() :
            1.0f;
        const float yawInertiaKgM2 =
            (std::isfinite(_vehicle.GetYawInertia()) && (_vehicle.GetYawInertia() > 0.0f)) ?
            _vehicle.GetYawInertia() :
            1.0f;
        const float leftWheelInertiaKgM2 =
            (std::isfinite(_leftDrive.getEquivalentWheelInertiaKgM2()) &&
             (_leftDrive.getEquivalentWheelInertiaKgM2() > 0.0f)) ?
            _leftDrive.getEquivalentWheelInertiaKgM2() :
            0.0f;
        const float rightWheelInertiaKgM2 =
            (std::isfinite(_rightDrive.getEquivalentWheelInertiaKgM2()) &&
             (_rightDrive.getEquivalentWheelInertiaKgM2() > 0.0f)) ?
            _rightDrive.getEquivalentWheelInertiaKgM2() :
            0.0f;
        const float forwardVelocityMps =
            std::isfinite(_runtimeState.GetForwardVelocity()) ? _runtimeState.GetForwardVelocity() : 0.0f;
        const float yawRateRadps =
            std::isfinite(_runtimeState.GetYawRate()) ? _runtimeState.GetYawRate() : 0.0f;
        const float leftWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((forwardVelocityMps + (halfTrackWidthM * yawRateRadps)) * invWheelRadiusM) :
            0.0f;
        const float rightWheelSpeedRadps =
            (wheelRadiusM > 0.0f) ?
            ((forwardVelocityMps - (halfTrackWidthM * yawRateRadps)) * invWheelRadiusM) :
            0.0f;

        const Eigen::Matrix<float, VehicleState::kDimension, 1> currentState = BuildBoundStateVector();
        const WheelKinematics currentKinematics = wheelKinematics(currentState, nullptr);
        const float totalNormalLoadN =
            (massKg * GRAVITY_MPS2) +
            ((std::clamp)(_vehicle.GetFanDuty(), 0.0f, 1.0f) * _vehicle.GetFanDownforceAtFullDuty());
        const float frontWheelNormalLoadN = 0.5f * kFrontLoadFraction * totalNormalLoadN;
        const float rearWheelNormalLoadN = 0.5f * (1.0f - kFrontLoadFraction) * totalNormalLoadN;
        const std::array<float, 4> contactNormalLoadN = {
            frontWheelNormalLoadN,
            frontWheelNormalLoadN,
            rearWheelNormalLoadN,
            rearWheelNormalLoadN
        };
        const std::array<float, 4> contactRightPositionsM = {
            -halfTrackWidthM,
            halfTrackWidthM,
            -halfTrackWidthM,
            halfTrackWidthM
        };
        const float contactLongitudinalOffsetM = std::fabs(Vehicle::GetDriveWheelLongitudinalOffsetM());
        const std::array<float, 4> contactForwardPositionsM = {
            contactLongitudinalOffsetM,
            contactLongitudinalOffsetM,
            -contactLongitudinalOffsetM,
            -contactLongitudinalOffsetM
        };
        const float contactRelativeSpeedMps =
            loadWeightedContactRelativeSpeedMps(
                currentKinematics,
                contactNormalLoadN,
                totalNormalLoadN);
        const float sustainedContactMu =
            (std::isfinite(Vehicle::GetSustainedLateralAccelerationReferenceMps2()) &&
             (Vehicle::GetSustainedLateralAccelerationReferenceMps2() > 0.0f) &&
             (totalNormalLoadN > kForceEpsilonN)) ?
            ((Vehicle::GetSustainedLateralAccelerationReferenceMps2() * massKg) / totalNormalLoadN) :
            0.0f;
        const float yawMomentYieldNm =
            (std::max)(0.0f, sustainedContactMu * halfTrackWidthM * totalNormalLoadN);
        const auto positiveGainOrZero = [](const float gain) noexcept -> float
        {
            return (std::isfinite(gain) && (gain > 0.0f)) ? gain : 0.0f;
        };
        const float frontLeftRightForceN =
            positiveGainOrZero(_leftDrive.getFrontRightContactForceGainNPerMps()) *
            currentKinematics._contacts[kFrontLeft]._rightRelativeVelocityMps;
        const float frontRightRightForceN =
            positiveGainOrZero(_rightDrive.getFrontRightContactForceGainNPerMps()) *
            currentKinematics._contacts[kFrontRight]._rightRelativeVelocityMps;
        const float rearLeftRightForceN =
            positiveGainOrZero(_leftDrive.getRearRightContactForceGainNPerMps()) *
            currentKinematics._contacts[kRearLeft]._rightRelativeVelocityMps;
        const float rearRightRightForceN =
            positiveGainOrZero(_rightDrive.getRearRightContactForceGainNPerMps()) *
            currentKinematics._contacts[kRearRight]._rightRelativeVelocityMps;
        const std::array<float, 4> synthesizedRightForceN = {
            frontLeftRightForceN,
            frontRightRightForceN,
            rearLeftRightForceN,
            rearRightRightForceN
        };
        const float synthesizedRightYawMomentNm =
            contactLongitudinalOffsetM *
            ((frontLeftRightForceN + frontRightRightForceN) -
             (rearLeftRightForceN + rearRightRightForceN));

        const float desiredYawMomentNm = yawInertiaKgM2 * desiredYawAccelRadps2;
        const float yawDirection = SignedDirection(yawRateRadps, desiredYawMomentNm);
        const auto variantCAtProjectedMoment = [&](const float projectedMomentAlongYawNm) noexcept -> float
        {
            if (yawDirection == 0.0f || !(trackWidthM > 0.0f))
            {
                return 0.0f;
            }

            const float projectedYawMomentNm = yawDirection * projectedMomentAlongYawNm;
            const float longitudinalYawMomentNm = projectedYawMomentNm - synthesizedRightYawMomentNm;
            const float yawDifferentialForceN = longitudinalYawMomentNm * invTrackWidthM;
            const float leftForceN = yawDifferentialForceN;
            const float rightForceN = -yawDifferentialForceN;
            const float leftBankNormalLoadN = contactNormalLoadN[kFrontLeft] + contactNormalLoadN[kRearLeft];
            const float rightBankNormalLoadN = contactNormalLoadN[kFrontRight] + contactNormalLoadN[kRearRight];
            const std::array<float, 4> synthesizedForwardForceN = {
                (leftBankNormalLoadN > kForceEpsilonN) ?
                    (leftForceN * (contactNormalLoadN[kFrontLeft] / leftBankNormalLoadN)) :
                    (0.5f * leftForceN),
                (rightBankNormalLoadN > kForceEpsilonN) ?
                    (rightForceN * (contactNormalLoadN[kFrontRight] / rightBankNormalLoadN)) :
                    (0.5f * rightForceN),
                (leftBankNormalLoadN > kForceEpsilonN) ?
                    (leftForceN * (contactNormalLoadN[kRearLeft] / leftBankNormalLoadN)) :
                    (0.5f * leftForceN),
                (rightBankNormalLoadN > kForceEpsilonN) ?
                    (rightForceN * (contactNormalLoadN[kRearRight] / rightBankNormalLoadN)) :
                    (0.5f * rightForceN)
            };

            float maxUtilization = 0.0f;
            for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
            {
                const float magnitudeN =
                    MazeMap::Math::Sqrtf(
                        (synthesizedForwardForceN[contactIndex] * synthesizedForwardForceN[contactIndex]) +
                        (synthesizedRightForceN[contactIndex] * synthesizedRightForceN[contactIndex]));
                const float maxForceN =
                    (std::max)(0.0f, sustainedContactMu * contactNormalLoadN[contactIndex]);
                maxUtilization =
                    (std::max)(
                        maxUtilization,
                        magnitudeN / (std::max)(maxForceN, kForceEpsilonN));
            }

            return variantCAggregateContactYawMomentCorrectionAlongYawNm(
                yawDirection,
                forwardVelocityMps,
                currentKinematics,
                contactRightPositionsM,
                contactForwardPositionsM,
                contactNormalLoadN,
                synthesizedForwardForceN,
                synthesizedRightForceN,
                totalNormalLoadN,
                maxUtilization);
        };
        const auto correctionAtProjectedMoment = [&](const float projectedMomentAlongYawNm) noexcept -> float
        {
            return aggregateContactYawMomentCorrectionAlongYawNm(
                forwardVelocityMps,
                contactRelativeSpeedMps,
                PositivePart(projectedMomentAlongYawNm),
                yawMomentYieldNm,
                variantCAtProjectedMoment(projectedMomentAlongYawNm));
        };

        float requestedYawMomentNm = desiredYawMomentNm;
        if (yawDirection != 0.0f)
        {
            const float desiredMomentAlongYawNm = yawDirection * desiredYawMomentNm;
            const float correctionAtZeroNm = correctionAtProjectedMoment(0.0f);
            const float relWeight = PositiveFiniteOrDefault(
                _aggregateContactYawMomentCorrectionForceRelWeight,
                kDefaultAggregateContactYawMomentCorrectionForceRelWeight);
            const float speedV2 =
                (forwardVelocityMps * forwardVelocityMps) +
                ((relWeight * contactRelativeSpeedMps) * (relWeight * contactRelativeSpeedMps));
            const float speedKnee = PositiveFiniteOrDefault(
                _aggregateContactYawMomentCorrectionBlendSpeedKneeMps,
                kDefaultAggregateContactYawMomentCorrectionBlendSpeedKneeMps);
            const float speedKnee2 = speedKnee * speedKnee;
            const float speedLow =
                (speedKnee2 > 0.0f) ? (speedKnee2 / (speedKnee2 + (std::max)(0.0f, speedV2))) : 0.0f;
            if (!(yawMomentYieldNm > kForceEpsilonN) || !(speedLow > kForceEpsilonN))
            {
                requestedYawMomentNm = yawDirection * (desiredMomentAlongYawNm + correctionAtZeroNm);
            }
            else if (desiredMomentAlongYawNm <= -correctionAtZeroNm)
            {
                requestedYawMomentNm = yawDirection * (desiredMomentAlongYawNm + correctionAtZeroNm);
            }
            else
            {
                const float speedFade = PositiveFiniteOrDefault(
                    _aggregateContactYawMomentCorrectionForceSpeedFadeMps,
                    kDefaultAggregateContactYawMomentCorrectionForceSpeedFadeMps);
                const float speedFade2 = speedFade * speedFade;
                const float forceBranchContactYawMomentCorrectionAlongYawNm =
                    ((speedFade2 > 0.0f) ? (speedFade2 / (speedFade2 + (std::max)(0.0f, speedV2))) : 0.0f) *
                    PositiveFiniteOrDefault(
                        _aggregateContactYawMomentCorrectionForceSlidingMomentNm,
                        kDefaultAggregateContactYawMomentCorrectionForceSlidingMomentNm);
                float lo = (std::max)(0.0f, desiredMomentAlongYawNm);
                float hi =
                    lo +
                    (std::max)(
                        (std::max)(forceBranchContactYawMomentCorrectionAlongYawNm, correctionAtZeroNm),
                        0.0f) +
                    (4.0f *
                        PositiveFiniteOrDefault(
                            _aggregateContactYawMomentCorrectionBlendForceKnee,
                            kDefaultAggregateContactYawMomentCorrectionBlendForceKnee) *
                        yawMomentYieldNm);
                if (!(hi > lo))
                {
                    hi = lo + yawMomentYieldNm;
                }

                for (uint8_t grow = 0U; grow < 8U; ++grow)
                {
                    if ((hi - correctionAtProjectedMoment(hi)) >= desiredMomentAlongYawNm)
                    {
                        break;
                    }
                    hi = lo + (2.0f * (hi - lo));
                }

                for (uint8_t iteration = 0U; iteration < 16U; ++iteration)
                {
                    const float mid = 0.5f * (lo + hi);
                    if ((mid - correctionAtProjectedMoment(mid)) < desiredMomentAlongYawNm)
                    {
                        lo = mid;
                    }
                    else
                    {
                        hi = mid;
                    }
                }

                requestedYawMomentNm = yawDirection * hi;
            }
        }
        const float forwardForceRequestN = massKg * desiredAccelMps2;
        const float commonForceRequestN = 0.5f * forwardForceRequestN;
        const float differentialForceRequestN = requestedYawMomentNm * invTrackWidthM;
        const float leftForceCommandN = commonForceRequestN + differentialForceRequestN;
        const float rightForceCommandN = commonForceRequestN - differentialForceRequestN;
        const float leftBankAccelMps2 = desiredAccelMps2 + (halfTrackWidthM * desiredYawAccelRadps2);
        const float rightBankAccelMps2 = desiredAccelMps2 - (halfTrackWidthM * desiredYawAccelRadps2);
        const float leftWheelTorqueRequestNm =
            (wheelRadiusM * leftForceCommandN) +
            ((wheelRadiusM > 0.0f) ? (leftWheelInertiaKgM2 * leftBankAccelMps2 * invWheelRadiusM) : 0.0f);
        const float rightWheelTorqueRequestNm =
            (wheelRadiusM * rightForceCommandN) +
            ((wheelRadiusM > 0.0f) ? (rightWheelInertiaKgM2 * rightBankAccelMps2 * invWheelRadiusM) : 0.0f);
        const float staticLaunchTorqueNm = staticFrictionTorqueNm();
        const float leftSurfaceSpeedMps = wheelRadiusM * leftWheelSpeedRadps;
        const float rightSurfaceSpeedMps = wheelRadiusM * rightWheelSpeedRadps;
        const float leftSlowLaunchRatio =
            (kStaticFrictionMaxSpeedMps > 0.0f) ?
            (std::fabs(leftSurfaceSpeedMps) / kStaticFrictionMaxSpeedMps) :
            0.0f;
        const float rightSlowLaunchRatio =
            (kStaticFrictionMaxSpeedMps > 0.0f) ?
            (std::fabs(rightSurfaceSpeedMps) / kStaticFrictionMaxSpeedMps) :
            0.0f;
        const float leftLaunchTorqueNm =
            staticLaunchTorqueNm * std::exp(-(leftSlowLaunchRatio * leftSlowLaunchRatio));
        const float rightLaunchTorqueNm =
            staticLaunchTorqueNm * std::exp(-(rightSlowLaunchRatio * rightSlowLaunchRatio));
        const float leftLaunchDirection =
            SignedDirection(leftWheelTorqueRequestNm, leftWheelSpeedRadps);
        const float rightLaunchDirection =
            SignedDirection(rightWheelTorqueRequestNm, rightWheelSpeedRadps);
        const float leftLossDirection =
            SignedDirection(leftWheelSpeedRadps, leftWheelTorqueRequestNm);
        const float rightLossDirection =
            SignedDirection(rightWheelSpeedRadps, rightWheelTorqueRequestNm);
        const float leftRollingLossTorqueNm =
            (kRollingFrictionTorqueNm * leftLossDirection) +
            (kViscousFrictionNmPerRadps * leftWheelSpeedRadps);
        const float rightRollingLossTorqueNm =
            (kRollingFrictionTorqueNm * rightLossDirection) +
            (kViscousFrictionNmPerRadps * rightWheelSpeedRadps);
        float leftCommandTorqueNm = leftWheelTorqueRequestNm;
        float rightCommandTorqueNm = rightWheelTorqueRequestNm;
        if (SignedDirection(leftWheelTorqueRequestNm, leftWheelSpeedRadps) != 0.0f)
        {
            leftCommandTorqueNm += (leftLaunchDirection * leftLaunchTorqueNm) + leftRollingLossTorqueNm;
        }
        if (SignedDirection(rightWheelTorqueRequestNm, rightWheelSpeedRadps) != 0.0f)
        {
            rightCommandTorqueNm += (rightLaunchDirection * rightLaunchTorqueNm) + rightRollingLossTorqueNm;
        }

        return App::Internal::CommandVector(
            _leftDrive.getCommandFromTorque(
                leftCommandTorqueNm,
                leftWheelSpeedRadps,
                _vehicle.GetBatteryVoltage()),
            _rightDrive.getCommandFromTorque(
                rightCommandTorqueNm,
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
        const float physicalTrackWidthM = _vehicle.GetTrackWidth();
        const float halfTrackWidthM =
            0.5f *
            ((std::isfinite(physicalTrackWidthM) && (physicalTrackWidthM > 0.0f)) ?
                physicalTrackWidthM :
                0.0f);
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
        const EncoderObs&,
        float,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
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

    float PlantModel::stationaryEncoderWheelSpeedSigmaRadps(float stationaryLinearSpeedSigmaMps) const noexcept
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

        return 0.5f * wheelRadiusM * (observation.LeftWheelSpeedRadps() + observation.RightWheelSpeedRadps());
    }

    float PlantModel::measuredYawRateRadps(const EncoderObs& observation) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float physicalTrackWidthM = _vehicle.GetTrackWidth();
        const float trackWidthM =
            (std::isfinite(physicalTrackWidthM) && (physicalTrackWidthM > 0.0f)) ?
            physicalTrackWidthM :
            0.0f;
        if (!(wheelRadiusM > 0.0f) ||
            !std::isfinite(wheelRadiusM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return 0.0f;
        }

        return wheelRadiusM * (observation.LeftWheelSpeedRadps() - observation.RightWheelSpeedRadps()) / trackWidthM;
    }

    float PlantModel::measuredYawRateVarianceRadps2(
        const EncoderObs&,
        float,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float physicalTrackWidthM = _vehicle.GetTrackWidth();
        const float trackWidthM =
            (std::isfinite(physicalTrackWidthM) && (physicalTrackWidthM > 0.0f)) ?
            physicalTrackWidthM :
            0.0f;
        if (!(wheelRadiusM > 0.0f) ||
            !std::isfinite(wheelRadiusM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return 1.0f;
        }

        const Eigen::Matrix<float, 2, 2> wheelCovarianceRadps2 =
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
        const EncoderObs&,
        float,
        float generalLinearSpeedSigmaMps,
        float generalYawRateSigmaRadps) const noexcept
    {
        const Eigen::Matrix<float, 2, 2> covariance =
            encoderPairCovarianceRadps(generalLinearSpeedSigmaMps, generalYawRateSigmaRadps);
        const float variance = (std::max)(covariance(0, 0), covariance(1, 1));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    Eigen::Vector2f PlantModel::wheelLinearVelocityFromBodyState(const Eigen::Matrix<float, VehicleState::kDimension, 1>& state) const noexcept
    {
        const float trackWidthM =
            (std::isfinite(_vehicle.GetTrackWidth()) && (_vehicle.GetTrackWidth() > 0.0f)) ?
            _vehicle.GetTrackWidth() :
            0.0f;
        const float forwardSpeedMps =
            std::isfinite(state(VehicleState::kVf)) ? state(VehicleState::kVf) : 0.0f;
        const float yawRateRadps =
            std::isfinite(state(VehicleState::kYawRate)) ? state(VehicleState::kYawRate) : 0.0f;
        Eigen::Vector2f wheelLinearVelocityMps = Eigen::Vector2f::Zero();
        wheelLinearVelocityMps(0) = forwardSpeedMps + (0.5f * trackWidthM * yawRateRadps);
        wheelLinearVelocityMps(1) = forwardSpeedMps - (0.5f * trackWidthM * yawRateRadps);
        return wheelLinearVelocityMps;
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
        return forwardStep(control)._contactForces.SumForwardForceN();
    }

    float PlantModel::totalRightContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control)._contactForces.SumRightForceN();
    }

    float PlantModel::leftBankForwardContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control)._contactForces.LeftBankForwardForceN();
    }

    float PlantModel::rightBankForwardContactForceN(const App::Internal::CommandVector& control) const noexcept
    {
        return forwardStep(control)._contactForces.RightBankForwardForceN();
    }

    float PlantModel::contactRightForceN(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = forwardStep(control)._contactForces;
        return (contactIndex < forces._contacts.size()) ?
            forces._contacts[contactIndex]._rightForceN :
            0.0f;
    }

    float PlantModel::contactForwardForceN(
        const App::Internal::CommandVector& control,
        uint8_t contactIndex) const noexcept
    {
        const ContactForces forces = forwardStep(control)._contactForces;
        return (contactIndex < forces._contacts.size()) ?
            forces._contacts[contactIndex]._forwardForceN :
            0.0f;
    }

    float PlantModel::contactNormalLoadN(uint8_t contactIndex) const noexcept
    {
        if (contactIndex < 4U)
        {
            return forwardStep(App::Internal::CommandVector(0.0f, 0.0f))._contactForces._contacts[contactIndex]._normalForceN;
        }
        return 0.0f;
    }

    float PlantModel::totalContactNormalLoadN() const noexcept
    {
        const ContactForces forces = forwardStep(App::Internal::CommandVector(0.0f, 0.0f))._contactForces;
        return
            forces._contacts[0]._normalForceN +
            forces._contacts[1]._normalForceN +
            forces._contacts[2]._normalForceN +
            forces._contacts[3]._normalForceN;
    }

    void PlantModel::velocityTargetTechnicalLimits(
        float& maxLongitudinalAccelMps2,
        float& maxYawAccelRadps2) const noexcept
    {
        velocityTargetTechnicalLimits(
            _runtimeState.GetForwardVelocity(),
            _runtimeState.GetYawRate(),
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
        const float wheelRadiusM = Vehicle::GetDriveWheelRadiusM();
        const float staticFrictionSpeedThresholdRadps =
            (wheelRadiusM > 0.0f) ? (kStaticFrictionMaxSpeedMps / wheelRadiusM) : 0.0f;
        const float viscousFrictionTorqueNm = kViscousFrictionNmPerRadps * wheelBankSpeedRadps;
        if (std::fabs(wheelBankSpeedRadps) <= staticFrictionSpeedThresholdRadps)
        {
            const float sign = SignedDirection(wheelTorqueRequestNm, wheelBankSpeedRadps);
            return (staticFrictionTorqueNm() * sign) + viscousFrictionTorqueNm;
        }

        const float sign = SignedDirection(wheelBankSpeedRadps, wheelTorqueRequestNm);
        return (kRollingFrictionTorqueNm * sign) + viscousFrictionTorqueNm;
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

    float PlantModel::aggregateContactYawMomentCorrectionBlendSpeedKneeMps() const noexcept
    {
        return _aggregateContactYawMomentCorrectionBlendSpeedKneeMps;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionBlendSpeedKneeMps(
        const float speedKneeMps) noexcept
    {
        _aggregateContactYawMomentCorrectionBlendSpeedKneeMps =
            PositiveFiniteOrDefault(
                speedKneeMps,
                kDefaultAggregateContactYawMomentCorrectionBlendSpeedKneeMps);
    }

    float PlantModel::aggregateContactYawMomentCorrectionBlendForceKnee() const noexcept
    {
        return _aggregateContactYawMomentCorrectionBlendForceKnee;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionBlendForceKnee(
        const float forceKnee) noexcept
    {
        _aggregateContactYawMomentCorrectionBlendForceKnee =
            PositiveFiniteOrDefault(
                forceKnee,
                kDefaultAggregateContactYawMomentCorrectionBlendForceKnee);
    }

    float PlantModel::aggregateContactYawMomentCorrectionForceRelWeight() const noexcept
    {
        return _aggregateContactYawMomentCorrectionForceRelWeight;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionForceRelWeight(
        const float relWeight) noexcept
    {
        _aggregateContactYawMomentCorrectionForceRelWeight =
            PositiveFiniteOrDefault(
                relWeight,
                kDefaultAggregateContactYawMomentCorrectionForceRelWeight);
    }

    float PlantModel::aggregateContactYawMomentCorrectionForceSpeedFadeMps() const noexcept
    {
        return _aggregateContactYawMomentCorrectionForceSpeedFadeMps;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionForceSpeedFadeMps(
        const float speedFadeMps) noexcept
    {
        _aggregateContactYawMomentCorrectionForceSpeedFadeMps =
            PositiveFiniteOrDefault(
                speedFadeMps,
                kDefaultAggregateContactYawMomentCorrectionForceSpeedFadeMps);
    }

    float PlantModel::aggregateContactYawMomentCorrectionForceSlidingMomentNm() const noexcept
    {
        return _aggregateContactYawMomentCorrectionForceSlidingMomentNm;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionForceSlidingMomentNm(
        const float slidingMomentNm) noexcept
    {
        _aggregateContactYawMomentCorrectionForceSlidingMomentNm =
            PositiveFiniteOrDefault(
                slidingMomentNm,
                kDefaultAggregateContactYawMomentCorrectionForceSlidingMomentNm);
    }

    float PlantModel::aggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps() const noexcept
    {
        return _aggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps(
        const float relativeSpeedKneeMps) noexcept
    {
        _aggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps =
            PositiveFiniteOrDefault(
                relativeSpeedKneeMps,
                kDefaultAggregateContactYawMomentCorrectionVariantCRelativeSpeedKneeMps);
    }

    float PlantModel::aggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps() const noexcept
    {
        return _aggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps;
    }

    void PlantModel::setAggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps(
        const float forwardSpeedKneeMps) noexcept
    {
        _aggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps =
            PositiveFiniteOrDefault(
                forwardSpeedKneeMps,
                kDefaultAggregateContactYawMomentCorrectionVariantCForwardSpeedKneeMps);
    }
}


