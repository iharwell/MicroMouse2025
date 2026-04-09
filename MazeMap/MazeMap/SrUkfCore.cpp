#include "pch.h"
#include "SrUkfCore.h"

#include "Vehicle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    constexpr std::array<const char*, MazeMap::VehicleState::kDimension> kUkfStateFieldNames = {
        "px_m",
        "py_m",
        "psi_rad",
        "u_mps",
        "v_mps",
        "r_radps",
        "omega_l_radps",
        "omega_r_radps",
        "bgz_radps"
    };

    constexpr std::array<const char*, 3> kUkfImuNoiseFieldNames = {
        "gyro_z_radps",
        "accel_body_x_mps2",
        "accel_body_y_mps2"
    };

    constexpr std::array<const char*, 2> kUkfFrontNoiseFieldNames = {
        "front_left_range_m",
        "front_right_range_m"
    };

    constexpr std::array<const char*, 1> kUkfSideNoiseFieldNames = {
        "side_range_m"
    };

    struct LateralNoiseSchedule
    {
        float rhoSustain = 0.0f;
        float sigmaAyMps2 = 0.12f;
        float lambdaVInvS = 60.0f;
        float vResetVarianceMps2 = 1.2e-4f;
        bool enableVReset = true;
    };

    float ComputeRhoSustain(
        const MazeMap::VehicleState::StateVector& state,
        const MazeMap::PlantParams& params) noexcept
    {
        const float velocityEpsilonMps =
            (std::isfinite(params.velocityEpsilonMps) && (params.velocityEpsilonMps > 0.0f)) ?
            params.velocityEpsilonMps :
            0.05f;
        const float forwardVelocityMps = state(MazeMap::VehicleState::kU);
        if (!std::isfinite(forwardVelocityMps) || (std::fabs(forwardVelocityMps) < velocityEpsilonMps))
        {
            return 0.0f;
        }

        const float yawRateRadps = state(MazeMap::VehicleState::kR);
        if (!std::isfinite(yawRateRadps))
        {
            return 0.0f;
        }

        const float ayRefMps2 = MazeMap::Vehicle::GetSustainedLateralAccelerationReferenceMps2();
        if (!(ayRefMps2 > 0.0f) || !std::isfinite(ayRefMps2))
        {
            return 0.0f;
        }

        return std::fabs(forwardVelocityMps * yawRateRadps) / ayRefMps2;
    }

    float ComputeSigmaAyMps2(const float rhoSustain) noexcept
    {
        if (rhoSustain <= 0.35f)
        {
            return 0.12f;
        }
        if (rhoSustain <= 0.75f)
        {
            const float t = (rhoSustain - 0.35f) / 0.40f;
            return 0.12f + ((1.00f - 0.12f) * t);
        }
        if (rhoSustain <= 1.00f)
        {
            const float t = (rhoSustain - 0.75f) / 0.25f;
            return 1.00f + ((2.50f - 1.00f) * t);
        }

        return (std::min)(2.50f + (4.00f * (rhoSustain - 1.00f)), 5.00f);
    }

    float ComputeLambdaVInvS(const float rhoSustain) noexcept
    {
        if (rhoSustain <= 0.85f)
        {
            return 60.0f;
        }
        if (rhoSustain >= 1.15f)
        {
            return 35.0f;
        }

        const float t = (rhoSustain - 0.85f) / 0.30f;
        return 60.0f + ((35.0f - 60.0f) * t);
    }

    LateralNoiseSchedule BuildLateralNoiseSchedule(
        const MazeMap::VehicleState::StateVector& state,
        const MazeMap::PlantParams& params) noexcept
    {
        LateralNoiseSchedule schedule{};
        schedule.rhoSustain = ComputeRhoSustain(state, params);
        schedule.sigmaAyMps2 = ComputeSigmaAyMps2(schedule.rhoSustain);
        schedule.lambdaVInvS = ComputeLambdaVInvS(schedule.rhoSustain);
        const float sigmaVResetMps =
            schedule.sigmaAyMps2 /
            MazeMap::Math::Sqrtf(2.0f * schedule.lambdaVInvS);
        schedule.vResetVarianceMps2 = sigmaVResetMps * sigmaVResetMps;
        schedule.enableVReset = schedule.rhoSustain < 0.85f;
        return schedule;
    }

    void UpdateLateralProcessNoiseDensityRoot(
        MazeMap::SrUkfCore::StateMatrix& sqrtProcessNoiseDensity,
        const MazeMap::VehicleState::StateVector& state,
        const MazeMap::PlantParams& params) noexcept
    {
        const LateralNoiseSchedule schedule = BuildLateralNoiseSchedule(state, params);
        sqrtProcessNoiseDensity(
            MazeMap::VehicleState::kV,
            MazeMap::VehicleState::kV) = schedule.sigmaAyMps2;
    }

    Eigen::Vector2f HeadingUnitFromYaw(float yaw) noexcept
    {
        float s = 0.0f;
        float c = 0.0f;
        sin_cosf(yaw, s, c);
        return Eigen::Vector2f(s, c);
    }

    template <typename... Args>
    bool AppendFormattedFragment(
        char* buffer,
        const std::size_t bufferSize,
        std::size_t& used,
        const char* format,
        Args... args) noexcept
    {
        if (buffer == nullptr || format == nullptr || used >= bufferSize)
        {
            return false;
        }

        const int length = std::snprintf(buffer + used, bufferSize - used, format, args...);
        if (length <= 0 || static_cast<std::size_t>(length) >= (bufferSize - used))
        {
            return false;
        }

        used += static_cast<std::size_t>(length);
        return true;
    }

    inline bool EmitDebugTextMessage(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* message) noexcept
    {
        return
            sink != nullptr &&
            type != nullptr &&
            type[0] != '\0' &&
            message != nullptr &&
            sink(context, type, message);
    }

    template <typename... Args>
    bool EmitDebugTextLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* format,
        Args... args) noexcept
    {
        char message[300] = {};
        const int length = std::snprintf(message, sizeof(message), format, args...);
        if (length <= 0 || length >= static_cast<int>(sizeof(message)))
        {
            return false;
        }

        return EmitDebugTextMessage(context, sink, type, message);
    }

    template <std::size_t N, typename Reader>
    bool EmitNamedFloatValuesLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const char* prefix,
        const std::array<const char*, N>& fieldNames,
        Reader&& reader) noexcept
    {
        char message[300] = {};
        std::size_t used = 0U;
        if (prefix != nullptr && prefix[0] != '\0' &&
            !AppendFormattedFragment(message, sizeof(message), used, "%s", prefix))
        {
            return false;
        }

        for (std::size_t index = 0; index < N; ++index)
        {
            if (!AppendFormattedFragment(
                    message,
                    sizeof(message),
                    used,
                    "%s%s=%.9g",
                    (used != 0U) ? ";" : "",
                    fieldNames[index],
                    static_cast<double>(reader(index))))
            {
                return false;
            }
        }

        return EmitDebugTextMessage(context, sink, type, message);
    }

    template <std::size_t N, typename MatrixType>
    bool EmitNamedMatrixRowLine(
        void* context,
        const MazeMap::SrUkfCore::DebugTextSink sink,
        const char* type,
        const std::array<const char*, N>& fieldNames,
        const MatrixType& matrix,
        const int row) noexcept
    {
        char prefix[48] = {};
        const int prefixLength = std::snprintf(
            prefix,
            sizeof(prefix),
            "row=%s",
            fieldNames[static_cast<std::size_t>(row)]);
        if (prefixLength <= 0 || prefixLength >= static_cast<int>(sizeof(prefix)))
        {
            return false;
        }

        return EmitNamedFloatValuesLine(
            context,
            sink,
            type,
            prefix,
            fieldNames,
            [&](const std::size_t column) noexcept
            {
                return matrix(row, static_cast<int>(column));
            });
    }
}

namespace MazeMap
{
    SrUkfCore::SrUkfCore(
        const PlantParams& params,
        const PlantModel& plantModel) noexcept
        : _plantModel(plantModel)
        , _geometryModel()
        , _params(params)
        , _filter()
        , _lastControl()
        , _lastEncoderObs()
        , _prePredictState(StateVector::Zero())
        , _havePredictionReference(false)
        , _sqrtProcessNoiseDensity(StateMatrix::Zero())
        , _sqrtImuNoise(Eigen::Matrix<float, 3, 3>::Identity())
        , _sqrtFrontNoise(Eigen::Matrix<float, 2, 2>::Identity())
        , _sqrtSideNoise(Eigen::Matrix<float, 1, 1>::Identity())
    {
        _filter.setStateNormalizer(&VehicleState::NormalizeStateVector);

        StateVector initialState = StateVector::Zero();
        const StateMatrix initialCovariance = BuildDefaultInitialCovariance();
        _filter.setState(initialState, initialCovariance);

        const StateMatrix processNoise = BuildDefaultProcessNoiseDensity();
        _sqrtProcessNoiseDensity = StateMatrix::Zero();
        _sqrtProcessNoiseDensity.diagonal() = processNoise.diagonal().cwiseSqrt();
        UpdateLateralProcessNoiseDensityRoot(_sqrtProcessNoiseDensity, initialState, _params);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _prePredictState = initialState;

        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
    }

    bool SrUkfCore::WriteDebugTextDump(void* context, DebugTextSink sink) const noexcept
    {
        if (sink == nullptr)
        {
            return false;
        }

        if (!EmitNamedFloatValuesLine(
                context,
                sink,
                "ukf_dump_state",
                nullptr,
                kUkfStateFieldNames,
                [&](const std::size_t index) noexcept
                {
                    return _filter.state()(static_cast<int>(index));
                }))
        {
            return false;
        }

        const StateMatrix covariance = _filter.covariance();
        for (int row = 0; row < VehicleState::kDimension; ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_covariance_row",
                    kUkfStateFieldNames,
                    covariance,
                    row))
            {
                return false;
            }
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_prediction_reference",
                "have_prediction_reference=%s",
                _havePredictionReference ? "true" : "false"))
        {
            return false;
        }

        if (!EmitNamedFloatValuesLine(
                context,
                sink,
                "ukf_dump_pre_predict_state",
                nullptr,
                kUkfStateFieldNames,
                [&](const std::size_t index) noexcept
                {
                    return _prePredictState(static_cast<int>(index));
                }))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_last_control",
                "left_motor_command=%.9g;right_motor_command=%.9g;fan_duty_cycle=%.9g;battery_voltage_v=%.9g",
                static_cast<double>(_lastControl.leftMotorCommand),
                static_cast<double>(_lastControl.rightMotorCommand),
                static_cast<double>(_lastControl.fanDutyCycle),
                static_cast<double>(_lastControl.batteryVoltageV)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_last_encoder_obs",
                "total_left_counts=%ld;total_right_counts=%ld;omega_left_radps=%.9g;omega_right_radps=%.9g",
                static_cast<long>(_lastEncoderObs.totalLeftCounts),
                static_cast<long>(_lastEncoderObs.totalRightCounts),
                static_cast<double>(_lastEncoderObs.omegaLeftRadps),
                static_cast<double>(_lastEncoderObs.omegaRightRadps)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_mass_geometry",
                "mass_kg=%.9g;effective_longitudinal_mass_kg=%.9g;yaw_inertia_kg_m2=%.9g;track_width_m=%.9g;contact_patch_longitudinal_offset_m=%.9g;wheel_radius_m=%.9g;equivalent_wheel_inertia_kg_m2=%.9g",
                static_cast<double>(_params.massKg),
                static_cast<double>(_params.effectiveLongitudinalMassKg),
                static_cast<double>(_params.yawInertiaKgM2),
                static_cast<double>(_params.trackWidthM),
                static_cast<double>(_params.contactPatchLongitudinalOffsetM),
                static_cast<double>(_params.wheelRadiusM),
                static_cast<double>(_params.equivalentWheelInertiaKgM2)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_drive_electrical",
                "supply_voltage_v=%.9g;drive_resistance_ohms=%.9g;torque_constant_nm_per_a=%.9g;speed_constant_radps_per_volt=%.9g;no_load_current_a=%.9g;motor_current_limit_a=%.9g;gear_ratio=%.9g;encoder_counts_per_motor_rev=%u",
                static_cast<double>(_params.supplyVoltageV),
                static_cast<double>(_params.driveResistanceOhms),
                static_cast<double>(_params.torqueConstantNmPerA),
                static_cast<double>(_params.speedConstantRadpsPerVolt),
                static_cast<double>(_params.noLoadCurrentA),
                static_cast<double>(_params.motorCurrentLimitA),
                static_cast<double>(_params.gearRatio),
                static_cast<unsigned>(_params.encoderCountsPerMotorRev)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_tire_friction",
                "drivetrain_efficiency=%.9g;rolling_friction_torque_nm=%.9g;viscous_friction_nm_per_radps=%.9g;longitudinal_tire_stiffness_n=%.9g;cornering_stiffness_front_n_per_rad=%.9g;cornering_stiffness_rear_n_per_rad=%.9g;mu_front=%.9g;mu_rear=%.9g;front_load_fraction=%.9g",
                static_cast<double>(_params.drivetrainEfficiency),
                static_cast<double>(_params.rollingFrictionTorqueNm),
                static_cast<double>(_params.viscousFrictionNmPerRadps),
                static_cast<double>(_params.longitudinalTireStiffnessN),
                static_cast<double>(_params.corneringStiffnessFrontNPerRad),
                static_cast<double>(_params.corneringStiffnessRearNPerRad),
                static_cast<double>(_params.muFront),
                static_cast<double>(_params.muRear),
                static_cast<double>(_params.frontLoadFraction)))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_params_misc",
                "velocity_epsilon_mps=%.9g;force_epsilon_n=%.9g;fan_downforce_at_full_duty_n=%.9g;no_hit_range_m=%.9g",
                static_cast<double>(_params.velocityEpsilonMps),
                static_cast<double>(_params.forceEpsilonN),
                static_cast<double>(_params.fanDownforceAtFullDutyN),
                static_cast<double>(_params.noHitRangeM)))
        {
            return false;
        }

        for (std::size_t index = 0; index < _params.contactPositionsBodyM.size(); ++index)
        {
            const Eigen::Vector2f& position = _params.contactPositionsBodyM[index];
            if (!EmitDebugTextLine(
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

        auto emitSensorExtrinsics =
            [&](const char* type, const SensorExtrinsics& sensor) noexcept
            {
                return EmitDebugTextLine(
                    context,
                    sink,
                    type,
                    "position_x_m=%.9g;position_y_m=%.9g;direction_x=%.9g;direction_y=%.9g;yaw_offset_rad=%.9g",
                    static_cast<double>(sensor.positionBodyM.x()),
                    static_cast<double>(sensor.positionBodyM.y()),
                    static_cast<double>(sensor.directionBody.x()),
                    static_cast<double>(sensor.directionBody.y()),
                    static_cast<double>(sensor.yawOffsetRad));
            };
        if (!emitSensorExtrinsics("ukf_dump_sensor_front_left", _params.frontLeftSensor) ||
            !emitSensorExtrinsics("ukf_dump_sensor_front_right", _params.frontRightSensor) ||
            !emitSensorExtrinsics("ukf_dump_sensor_side_left", _params.sideLeftSensor) ||
            !emitSensorExtrinsics("ukf_dump_sensor_side_right", _params.sideRightSensor))
        {
            return false;
        }

        if (!EmitDebugTextLine(
                context,
                sink,
                "ukf_dump_imu_extrinsics",
                "position_x_m=%.9g;position_y_m=%.9g;accel_body_from_imu_00=%.9g;accel_body_from_imu_01=%.9g;accel_body_from_imu_10=%.9g;accel_body_from_imu_11=%.9g;gyro_z_sign=%.9g",
                static_cast<double>(_params.imu.positionBodyM.x()),
                static_cast<double>(_params.imu.positionBodyM.y()),
                static_cast<double>(_params.imu.accelBodyFromImu(0, 0)),
                static_cast<double>(_params.imu.accelBodyFromImu(0, 1)),
                static_cast<double>(_params.imu.accelBodyFromImu(1, 0)),
                static_cast<double>(_params.imu.accelBodyFromImu(1, 1)),
                static_cast<double>(_params.imu.gyroZSign)))
        {
            return false;
        }

        for (int row = 0; row < VehicleState::kDimension; ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_process_noise_sqrt_row",
                    kUkfStateFieldNames,
                    _sqrtProcessNoiseDensity,
                    row))
            {
                return false;
            }
        }

        for (int row = 0; row < static_cast<int>(kUkfImuNoiseFieldNames.size()); ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_imu_noise_sqrt_row",
                    kUkfImuNoiseFieldNames,
                    _sqrtImuNoise,
                    row))
            {
                return false;
            }
        }

        for (int row = 0; row < static_cast<int>(kUkfFrontNoiseFieldNames.size()); ++row)
        {
            if (!EmitNamedMatrixRowLine(
                    context,
                    sink,
                    "ukf_dump_front_noise_sqrt_row",
                    kUkfFrontNoiseFieldNames,
                    _sqrtFrontNoise,
                    row))
            {
                return false;
            }
        }

        return EmitNamedMatrixRowLine(
            context,
            sink,
            "ukf_dump_side_noise_sqrt_row",
            kUkfSideNoiseFieldNames,
            _sqrtSideNoise,
            0);
    }

    SrUkfCore::StateMatrix SrUkfCore::BuildDefaultInitialCovariance() noexcept
    {
        StateMatrix covariance = StateMatrix::Identity() * 1.0e-3f;
        covariance(VehicleState::kPx, VehicleState::kPx) = 1.0e-5f;
        covariance(VehicleState::kPy, VehicleState::kPy) = 1.0e-5f;
        covariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = 0.25f;
        covariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = 0.25f;
        covariance(VehicleState::kBgz, VehicleState::kBgz) = 0.01f;
        return covariance;
    }

    SrUkfCore::StateMatrix SrUkfCore::BuildDefaultProcessNoiseDensity() noexcept
    {
        StateMatrix processNoise = StateMatrix::Zero();
        processNoise.diagonal() <<
            0.0f,
            0.0f,
            0.0f,
            3.0e-4f,
            0.0f,
            6.0e-4f,
            4.0e-2f,
            4.0e-2f,
            2.0e-8f;
        return processNoise;
    }

    bool SrUkfCore::reset(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
        UpdateLateralProcessNoiseDensityRoot(_sqrtProcessNoiseDensity, state, _params);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _lastControl = ControlInput{};
        _lastEncoderObs = EncoderObs{};
        _prePredictState = state;
        _havePredictionReference = false;
        return true;
    }

    bool SrUkfCore::setState(const StateVector& state, const StateMatrix& covariance) noexcept
    {
        _filter.setState(state, covariance);
        UpdateLateralProcessNoiseDensityRoot(_sqrtProcessNoiseDensity, state, _params);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);
        _prePredictState = state;
        _havePredictionReference = false;
        return true;
    }

    void SrUkfCore::InvokeLoopHook(void* context, LoopHookInvoker loopHook) noexcept
    {
        if (context != nullptr && loopHook != nullptr)
        {
            loopHook(context);
        }
    }

    Eigen::Matrix<float, 2, 2> SrUkfCore::ComputeGeneralEncoderPairCovarianceRadps(
        const PlantParams& params) noexcept
    {
        Eigen::Matrix<float, 2, 2> covariance = Eigen::Matrix<float, 2, 2>::Zero();
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            covariance(0, 0) = 1.0f;
            covariance(1, 1) = 1.0f;
            return covariance;
        }

        const float trackWidthM =
            (params.trackWidthM > 0.0f && std::isfinite(params.trackWidthM)) ?
            params.trackWidthM :
            Vehicle::GetPhysicalModel().trackWidthM;
        const float halfTrackWidthM = 0.5f * trackWidthM;
        const float varianceUMps2 = kGeneralEncoderLinearSpeedSigmaMps * kGeneralEncoderLinearSpeedSigmaMps;
        const float varianceYawRateRadps2 = kGeneralEncoderYawRateSigmaRadps * kGeneralEncoderYawRateSigmaRadps;
        const float varianceWheelLinearMps2 =
            varianceUMps2 + ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float covarianceWheelLinearMps2 =
            varianceUMps2 - ((halfTrackWidthM * halfTrackWidthM) * varianceYawRateRadps2);
        const float invWheelRadius2 = 1.0f / (params.wheelRadiusM * params.wheelRadiusM);
        covariance(0, 0) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 1) = varianceWheelLinearMps2 * invWheelRadius2;
        covariance(0, 1) = covarianceWheelLinearMps2 * invWheelRadius2;
        covariance(1, 0) = covariance(0, 1);
        return covariance;
    }

    Eigen::Matrix<float, 2, 2> SrUkfCore::ComputeGeneralEncoderPairSqrtNoise(
        const PlantParams& params) noexcept
    {
        const Eigen::Matrix<float, 2, 2> covariance = ComputeGeneralEncoderPairCovarianceRadps(params);
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

    float SrUkfCore::ComputeStationaryEncoderOmegaSigmaRadps(const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            return 1.0f;
        }

        return kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;
    }

    Eigen::Matrix<float, 2, 2> SrUkfCore::ComputeEncoderPairSqrtNoise(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            Eigen::Matrix<float, 2, 2> sqrtNoise = Eigen::Matrix<float, 2, 2>::Zero();
            const float sigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(params);
            sqrtNoise(0, 0) = sigmaRadps;
            sqrtNoise(1, 1) = sigmaRadps;
            return sqrtNoise;
        }

        return ComputeGeneralEncoderPairSqrtNoise(params);
    }

    float SrUkfCore::ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            return std::numeric_limits<float>::infinity();
        }

        return kEncoderPairNisThreshold;
    }

    bool SrUkfCore::predict(float dt, const ControlInput& control) noexcept
    {
        return predictImpl(dt, control, nullptr, nullptr);
    }

    bool SrUkfCore::predictImpl(
        float dt,
        const ControlInput& control,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        _lastControl = control;
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return true;
        }

        _prePredictState = _filter.state();
        _havePredictionReference = true;
        UpdateLateralProcessNoiseDensityRoot(_sqrtProcessNoiseDensity, _prePredictState, _params);
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dt));
        Eigen::Matrix<float, 3, 1> controlVector;
        controlVector << control.leftMotorCommand, control.rightMotorCommand, control.fanDutyCycle;
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        return _filter.Predict(
            dt,
            controlVector,
            [this, &control](const StateVector& sigmaPoint, const Eigen::Matrix<float, 3, 1>&, float sigmaDt) noexcept
            {
                return _plantModel.integrate(sigmaPoint, control, sigmaDt, _params);
            },
            invokeLoop);
    }

    MeasurementUpdateResult SrUkfCore::updateEncoderPair(const EncoderObs& observation, float dt) noexcept
    {
        return updateEncoderPairImpl(observation, dt, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updateEncoderPairImpl(
        const EncoderObs& observation,
        float dt,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        (void)dt;
        MeasurementUpdateResult result{};
        result.attempted = true;

        const EncoderObs measured = observation;
        _lastEncoderObs = measured;

        Eigen::Matrix<float, 2, 1> z;
        z << measured.omegaLeftRadps, measured.omegaRightRadps;
        if (HasExactZeroWheelObservation(measured))
        {
            applyWheelRateConstraint(measured, ComputeMeasuredWheelVarianceRadps2(measured, _params));
            result.accepted = true;
            result.nis = 0.0f;
            return result;
        }

        const Eigen::Matrix<float, 2, 2> sqrtEncoderNoise = ComputeEncoderPairSqrtNoise(measured, _params);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _filter.Update<2>(
            z,
            sqrtEncoderNoise,
            ComputeEncoderPairNisThreshold(measured),
            [](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 2, 1> prediction;
                prediction << sigmaPoint(VehicleState::kOmegaL), sigmaPoint(VehicleState::kOmegaR);
                return prediction;
            },
            invokeLoop);
        if (result.accepted)
        {
            applyWheelRateConstraint(measured, ComputeMeasuredWheelVarianceRadps2(measured, _params));
        }
        result.nis = _filter.lastNis();
        return result;
    }

    MeasurementUpdateResult SrUkfCore::updateImuMerged(const ImuMergedObs& observation) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = observation.valid;
        if (!observation.valid)
        {
            return result;
        }

        const MeasurementUpdateResult yawResult = updateYawRate(observation.gyroZRadps);
        const ImuAccelObs accelObservation{
            observation.valid,
            observation.accelBodyXMps2,
            observation.accelBodyYMps2
        };
        const MeasurementUpdateResult accelResult = updatePlanarAccel(accelObservation);
        result.accepted = yawResult.accepted && accelResult.accepted;
        result.nis = accelResult.attempted ? accelResult.nis : yawResult.nis;
        return result;
    }

    MeasurementUpdateResult SrUkfCore::updateYawRate(float yawRateRadps) noexcept
    {
        return updateYawRateImpl(yawRateRadps, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updateYawRateImpl(
        float yawRateRadps,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = std::isfinite(yawRateRadps);
        if (!result.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 1, 1> z;
        z << yawRateRadps;
        Eigen::Matrix<float, 1, 1> sqrtNoise;
        sqrtNoise(0, 0) = _sqrtImuNoise(0, 0);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _filter.Update<1>(
            z,
            sqrtNoise,
            std::numeric_limits<float>::infinity(),
            [](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction << sigmaPoint(VehicleState::kR) + sigmaPoint(VehicleState::kBgz);
                return prediction;
            },
            invokeLoop);
        if (result.accepted)
        {
            if (HasExactZeroWheelObservation(_lastEncoderObs) &&
                (controlCommandsAreEffectivelyZero() ||
                 (std::fabs(yawRateRadps) <= (3.0f * kImuYawRateSigmaRadps))))
            {
                applyStationaryZeroMotionConstraint(yawRateRadps);
            }
        }
        result.nis = _filter.lastNis();
        return result;
    }

    MeasurementUpdateResult SrUkfCore::updatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        return updatePlanarAccelImpl(observation, nullptr, nullptr);
    }

    MeasurementUpdateResult SrUkfCore::updatePlanarAccelImpl(
        const ImuAccelObs& observation,
        void* loopHookContext,
        LoopHookInvoker loopHook) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted =
            observation.valid &&
            std::isfinite(observation.accelBodyXMps2) &&
            std::isfinite(observation.accelBodyYMps2);
        if (!result.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 2, 1> z;
        z << observation.accelBodyXMps2, observation.accelBodyYMps2;
        Eigen::Matrix<float, 2, 2> sqrtNoise = Eigen::Matrix<float, 2, 2>::Zero();
        sqrtNoise(0, 0) = _sqrtImuNoise(1, 1);
        sqrtNoise(1, 1) = _sqrtImuNoise(2, 2);
        const auto invokeLoop = [loopHookContext, loopHook]() noexcept
        {
            InvokeLoopHook(loopHookContext, loopHook);
        };
        result.accepted = _filter.Update<2>(
            z,
            sqrtNoise,
            9.21034f,
            [this](const StateVector& sigmaPoint) noexcept
            {
                return accelPredictionForState(sigmaPoint);
            },
            invokeLoop);
        result.nis = _filter.lastNis();
        return result;
    }

    bool SrUkfCore::HasExactZeroWheelObservation(const EncoderObs& observation) noexcept
    {
        return (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
    }

    float SrUkfCore::ComputeDistancePerEncoderCountM(const PlantParams& params) noexcept
    {
        const float encoderCountsPerWheelRev = params.gearRatio * static_cast<float>(params.encoderCountsPerMotorRev);
        if (!(params.wheelRadiusM > 0.0f) ||
            !std::isfinite(params.wheelRadiusM) ||
            !(encoderCountsPerWheelRev > 0.0f) ||
            !std::isfinite(encoderCountsPerWheelRev))
        {
            return 0.0f;
        }

        return (2.0f * PI_F * params.wheelRadiusM) / encoderCountsPerWheelRev;
    }

    float SrUkfCore::ComputeMeasuredLinearSpeedMps(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            return 0.0f;
        }

        return 0.5f * params.wheelRadiusM * (observation.omegaLeftRadps + observation.omegaRightRadps);
    }

    float SrUkfCore::ComputeMeasuredLinearSpeedVarianceMps2(const EncoderObs& observation) noexcept
    {
        const float sigmaMps =
            HasExactZeroWheelObservation(observation) ?
            kStationaryEncoderVelocitySigmaMps :
            kGeneralEncoderLinearSpeedSigmaMps;
        return sigmaMps * sigmaMps;
    }

    float SrUkfCore::ComputeMeasuredWheelVarianceRadps2(
        const EncoderObs& observation,
        const PlantParams& params) noexcept
    {
        if (HasExactZeroWheelObservation(observation))
        {
            const float stationarySigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(params);
            return stationarySigmaRadps * stationarySigmaRadps;
        }

        const Eigen::Matrix<float, 2, 2> covariance = ComputeGeneralEncoderPairCovarianceRadps(params);
        const float variance = (std::max)(covariance(0, 0), covariance(1, 1));
        return (std::isfinite(variance) && (variance > 0.0f)) ? variance : 1.0f;
    }

    bool SrUkfCore::controlCommandsAreEffectivelyZero() const noexcept
    {
        return
            (std::fabs(_lastControl.leftMotorCommand) <= 1.0e-6f) &&
            (std::fabs(_lastControl.rightMotorCommand) <= 1.0e-6f);
    }

    void SrUkfCore::anchorPoseToEncoderDelta(
        StateVector& anchoredState,
        const EncoderObs& measured) const noexcept
    {
        if (!_havePredictionReference)
        {
            return;
        }

        const float distancePerCountM = ComputeDistancePerEncoderCountM(_params);
        const float trackWidthM = _params.trackWidthM;
        if (!(distancePerCountM > 0.0f) ||
            !std::isfinite(distancePerCountM) ||
            !(trackWidthM > 0.0f) ||
            !std::isfinite(trackWidthM))
        {
            return;
        }

        const float leftDistanceM = static_cast<float>(measured.totalLeftCounts) * distancePerCountM;
        const float rightDistanceM = static_cast<float>(measured.totalRightCounts) * distancePerCountM;
        const float forwardDistanceM = 0.5f * (leftDistanceM + rightDistanceM);
        const float deltaYawRad = (leftDistanceM - rightDistanceM) / trackWidthM;
        const float referenceYawRad = _prePredictState(VehicleState::kPsi);
        const float translationYawRad = VehicleState::NormalizeAngle(referenceYawRad + (0.5f * deltaYawRad));
        const Eigen::Vector2f heading = HeadingUnitFromYaw(translationYawRad);

        anchoredState(VehicleState::kPx) = _prePredictState(VehicleState::kPx) + (forwardDistanceM * heading.x());
        anchoredState(VehicleState::kPy) = _prePredictState(VehicleState::kPy) + (forwardDistanceM * heading.y());
        anchoredState(VehicleState::kPsi) = VehicleState::NormalizeAngle(referenceYawRad + deltaYawRad);
    }

    void SrUkfCore::applyWheelRateConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept
    {
        StateVector anchoredState = _filter.state();
        anchorPoseToEncoderDelta(anchoredState, measured);
        anchoredState(VehicleState::kU) = ComputeMeasuredLinearSpeedMps(measured, _params);
        anchoredState(VehicleState::kOmegaL) = measured.omegaLeftRadps;
        anchoredState(VehicleState::kOmegaR) = measured.omegaRightRadps;
        VehicleState::NormalizeStateVector(anchoredState);

        StateMatrix anchoredCovariance = _filter.covariance();
        anchoredCovariance.row(VehicleState::kU).setZero();
        anchoredCovariance.col(VehicleState::kU).setZero();
        anchoredCovariance.row(VehicleState::kOmegaL).setZero();
        anchoredCovariance.col(VehicleState::kOmegaL).setZero();
        anchoredCovariance.row(VehicleState::kOmegaR).setZero();
        anchoredCovariance.col(VehicleState::kOmegaR).setZero();
        anchoredCovariance(VehicleState::kU, VehicleState::kU) =
            (std::max)(ComputeMeasuredLinearSpeedVarianceMps2(measured), 1.0e-12f);
        const float constrainedVariance = (std::max)(wheelVarianceRadps2, 1.0e-12f);
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = constrainedVariance;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = constrainedVariance;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    void SrUkfCore::applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept
    {
        StateVector anchoredState = _filter.state();
        const LateralNoiseSchedule lateralNoiseSchedule = BuildLateralNoiseSchedule(_prePredictState, _params);
        const bool enableVReset = lateralNoiseSchedule.enableVReset;
        anchoredState(VehicleState::kU) = 0.0f;
        if (enableVReset)
        {
            anchoredState(VehicleState::kV) = 0.0f;
        }
        anchoredState(VehicleState::kR) = 0.0f;
        anchoredState(VehicleState::kOmegaL) = 0.0f;
        anchoredState(VehicleState::kOmegaR) = 0.0f;
        anchoredState(VehicleState::kBgz) = yawRateRadps;
        VehicleState::NormalizeStateVector(anchoredState);

        StateMatrix anchoredCovariance = _filter.covariance();
        const float stationarySigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(_params);
        const float wheelVarianceRadps2 = (std::max)(stationarySigmaRadps * stationarySigmaRadps, 1.0e-12f);
        const float linearVarianceMps2 =
            (std::max)(kStationaryEncoderVelocitySigmaMps * kStationaryEncoderVelocitySigmaMps, 1.0e-12f);
        const float yawVarianceRadps2 =
            (std::max)(kImuYawRateSigmaRadps * kImuYawRateSigmaRadps, 1.0e-12f);
        const float gyroBiasVarianceRadps2 = (std::max)(yawVarianceRadps2, 1.0e-8f);

        const std::array<int, 4> constrainedIndices = {
            VehicleState::kU,
            VehicleState::kR,
            VehicleState::kOmegaL,
            VehicleState::kOmegaR
        };
        for (const int index : constrainedIndices)
        {
            anchoredCovariance.row(index).setZero();
            anchoredCovariance.col(index).setZero();
        }
        anchoredCovariance.row(VehicleState::kBgz).setZero();
        anchoredCovariance.col(VehicleState::kBgz).setZero();
        if (enableVReset)
        {
            anchoredCovariance.row(VehicleState::kV).setZero();
            anchoredCovariance.col(VehicleState::kV).setZero();
        }

        anchoredCovariance(VehicleState::kU, VehicleState::kU) = linearVarianceMps2;
        if (enableVReset)
        {
            anchoredCovariance(VehicleState::kV, VehicleState::kV) =
                (std::max)(lateralNoiseSchedule.vResetVarianceMps2, 1.0e-12f);
        }
        anchoredCovariance(VehicleState::kR, VehicleState::kR) = yawVarianceRadps2;
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelVarianceRadps2;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelVarianceRadps2;
        anchoredCovariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasVarianceRadps2;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    float SrUkfCore::wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept
    {
        const float normalizedConfidence = (std::clamp)(confidence, 0.0f, 1.0f);
        return minimumNoise + ((1.0f - normalizedConfidence) * 0.020f);
    }

    Eigen::Matrix<float, 2, 1> SrUkfCore::accelPredictionForState(const StateVector& sigmaPoint) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const Eigen::Vector2f accel = _plantModel.imuPlanarAcceleration(sigmaPoint, _lastControl, _params);
        prediction << accel.x(), accel.y();
        return prediction;
    }

    float SrUkfCore::wallPredictionForSensor(
        const StateVector& sigmaPoint,
        const SensorExtrinsics& sensor,
        const LocalMapView& map) const noexcept
    {
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint, map);
        const GeometryPrediction prediction = _geometryModel.predictRay(frame, sensor, map);
        return prediction.hit ? prediction.rangeM : map.noHitRangeM;
    }

    Eigen::Matrix<float, 2, 1> SrUkfCore::frontPairPredictionForState(
        const StateVector& sigmaPoint,
        const LocalMapView& map) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint, map);
        const GeometryPrediction leftPrediction = _geometryModel.predictRay(frame, _params.frontLeftSensor, map);
        const GeometryPrediction rightPrediction = _geometryModel.predictRay(frame, _params.frontRightSensor, map);
        prediction(0) = leftPrediction.hit ? leftPrediction.rangeM : map.noHitRangeM;
        prediction(1) = rightPrediction.hit ? rightPrediction.rangeM : map.noHitRangeM;
        return prediction;
    }

    FrontPairUpdateResult SrUkfCore::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const LocalMapView& map) noexcept
    {
        FrontPairUpdateResult result{};
        result.leftPrediction = _geometryModel.predictRay(_filter.state(), _params.frontLeftSensor, map);
        result.rightPrediction = _geometryModel.predictRay(_filter.state(), _params.frontRightSensor, map);
        result.filter.attempted = left.valid && right.valid && map.IsValid();
        if (!result.filter.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 2, 2> sqrtNoise = _sqrtFrontNoise;
        sqrtNoise(0, 0) = wallNoiseFromConfidence(left.confidence, _sqrtFrontNoise(0, 0));
        sqrtNoise(1, 1) = wallNoiseFromConfidence(right.confidence, _sqrtFrontNoise(1, 1));

        Eigen::Matrix<float, 2, 1> z;
        z << left.rho, right.rho;
        result.filter.accepted = _filter.Update<2>(
            z,
            sqrtNoise,
            9.21034f,
            [this, &map](const StateVector& sigmaPoint) noexcept
            {
                return frontPairPredictionForState(sigmaPoint, map);
            });
        result.filter.nis = _filter.lastNis();
        return result;
    }

    WallUpdateResult SrUkfCore::updateSideSensor(
        Side which,
        const WallObs& observation,
        const LocalMapView& map) noexcept
    {
        WallUpdateResult result{};
        const SensorExtrinsics& sensor =
            (which == Side::Left) ? _params.sideLeftSensor : _params.sideRightSensor;
        result.prediction = _geometryModel.predictRay(_filter.state(), sensor, map);
        result.filter.attempted = observation.valid && map.IsValid();
        if (!result.filter.attempted)
        {
            return result;
        }

        Eigen::Matrix<float, 1, 1> sqrtNoise = _sqrtSideNoise;
        sqrtNoise(0, 0) = wallNoiseFromConfidence(observation.confidence, _sqrtSideNoise(0, 0));

        Eigen::Matrix<float, 1, 1> z;
        z << observation.rho;
        result.filter.accepted = _filter.Update<1>(
            z,
            sqrtNoise,
            7.87944f,
            [this, &map, &sensor](const StateVector& sigmaPoint) noexcept
            {
                Eigen::Matrix<float, 1, 1> prediction;
                prediction(0) = wallPredictionForSensor(sigmaPoint, sensor, map);
                return prediction;
            });
        result.filter.nis = _filter.lastNis();
        return result;
    }
}
