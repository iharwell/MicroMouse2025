#pragma once

#include "Cell.h"
#include "CellCoordinates.h"
#include "Direction.h"
#include "Maze.h"
#include "MotorEncoderDrive.h"
#include "UKF.h"
#include "Vehicle.h"
#include "VehicleState.h"
#include "WallSensor.h"
#include "WallSensorCalibration.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
namespace MazeMap
{
    enum class GeometryHitType : uint8_t
    {
        None = 0U,
        WallFace = 1U,
        Post = 2U
    };

    struct ContactKinematics
    {
        float vx = 0.0f;
        float vy = 0.0f;
    };

    struct WheelKinematics
    {
        float leftBankLongitudinalVelocityMps = 0.0f;
        float rightBankLongitudinalVelocityMps = 0.0f;
        std::array<ContactKinematics, 4> contacts{};
    };

    struct SlipTargets
    {
        float kappaLeft = 0.0f;
        float kappaRight = 0.0f;
        std::array<float, 4> lateralRatio{};
    };

    struct ContactForce
    {
        float fx = 0.0f;
        float fy = 0.0f;
        float fz = 0.0f;
        float saturation = 1.0f;
    };

    struct ContactForces
    {
        std::array<ContactForce, 4> contacts{};

        float SumFx() const noexcept
        {
            float sum = 0.0f;
            for (const ContactForce& contact : contacts)
            {
                sum += contact.fx;
            }
            return sum;
        }

        float SumFy() const noexcept
        {
            float sum = 0.0f;
            for (const ContactForce& contact : contacts)
            {
                sum += contact.fy;
            }
            return sum;
        }

        float LeftBankFx() const noexcept
        {
            return contacts[0].fx + contacts[2].fx;
        }

        float RightBankFx() const noexcept
        {
            return contacts[1].fx + contacts[3].fx;
        }
    };

    struct PlantDerivatives
    {
        VehicleState::StateVector stateDot = VehicleState::StateVector::Zero();
        ContactForces contactForces{};
        WheelKinematics wheelKinematics{};
        SlipTargets slipTargets{};
        Eigen::Vector2f originAccelBodyMps2 = Eigen::Vector2f::Zero();
        Eigen::Vector2f imuAccelBodyMps2 = Eigen::Vector2f::Zero();
        float longitudinalAccelMps2 = 0.0f;
        float lateralAccelMps2 = 0.0f;
        float yawAccelRadps2 = 0.0f;
    };

    struct PlantParams
    {
        float massKg = 0.14f;
        float effectiveLongitudinalMassKg = 0.14f;
        float yawInertiaKgM2 = 0.000665f;
        float trackWidthM = 0.084635f;
        float contactPatchLongitudinalOffsetM = 0.015f;
        float wheelRadiusM = 0.01261f;
        float equivalentWheelInertiaKgM2 = 4.8e-5f;

        float supplyVoltageV = 8.4f;
        float driveResistanceOhms = 4.31f;
        float torqueConstantNmPerA = 0.00396f;
        float speedConstantRadpsPerVolt = 0.0f;
        float noLoadCurrentA = 0.0459f;
        float motorCurrentLimitA = 0.0f;
        float gearRatio = 56.0f / 17.0f;
        uint16_t encoderCountsPerMotorRev = 4096U;

        float drivetrainEfficiency = 0.88f;
        float rollingFrictionTorqueNm = 0.0035f;
        float viscousFrictionNmPerRadps = 2.5e-4f;

        float longitudinalStiffnessN = 18.0f;
        float corneringStiffnessFrontNPerRad = 18.0f;
        float corneringStiffnessRearNPerRad = 16.0f;
        float muFront = 1.65f;
        float muRear = 1.65f;
        float frontLoadFraction = 0.5f;

        float velocityEpsilonMps = 0.05f;
        float forceEpsilonN = 1.0e-4f;
        float gravityMps2 = 9.80665f;
        float fanDownforceAtFullDutyN = 1.60f;

        float mazeCellSizeM = Maze::GetCellDimension() / 100.0f;
        float mazeWallThicknessM = WALL_THICKNESS;
        float noHitRangeM = 0.30f;
        float postHalfWidthM = 0.5f * WALL_THICKNESS;
        std::array<Eigen::Vector2f, 4> contactPositionsBodyM = {
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero(),
            Eigen::Vector2f::Zero()
        };

        SensorExtrinsics frontLeftSensor{};
        SensorExtrinsics frontRightSensor{};
        SensorExtrinsics sideLeftSensor{};
        SensorExtrinsics sideRightSensor{};
        ImuExtrinsics imu{};

        static PlantParams Default() noexcept
        {
            PlantParams params{};
            const VehiclePhysicalModel& physicalModel = Vehicle::GetPhysicalModel();
            const MotorEncoderDrivePhysicalModel& driveModel = MotorEncoderDrive::GetSharedPhysicalModel();
            params.massKg = physicalModel.massKg;
            params.effectiveLongitudinalMassKg = physicalModel.massKg;
            params.yawInertiaKgM2 = physicalModel.yawInertiaKgM2;
            params.trackWidthM = physicalModel.trackWidthM;
            params.wheelRadiusM = 0.5f * driveModel.wheelDiameterM;
            params.supplyVoltageV = driveModel.supplyVoltageV;
            params.driveResistanceOhms = driveModel.resistanceOhms;
            params.torqueConstantNmPerA = driveModel.torqueConstantNmPerA;
            params.speedConstantRadpsPerVolt = driveModel.speedConstantRadpsPerVolt;
            params.noLoadCurrentA = driveModel.noLoadCurrentA;
            params.gearRatio = driveModel.gearRatio;
            params.encoderCountsPerMotorRev = driveModel.pulsesPerRev;
            params.motorCurrentLimitA =
                (params.driveResistanceOhms > 0.0f) ?
                (params.supplyVoltageV / params.driveResistanceOhms) :
                0.0f;

            params.frontLeftSensor = Vehicle::GetFrontLeftSensorExtrinsics();
            params.frontRightSensor = Vehicle::GetFrontRightSensorExtrinsics();
            params.sideLeftSensor = Vehicle::GetSideLeftSensorExtrinsics();
            params.sideRightSensor = Vehicle::GetSideRightSensorExtrinsics();
            params.imu = Vehicle::GetBackLeftImuExtrinsics();

            const float frontContactX = std::fabs(params.contactPatchLongitudinalOffsetM);
            const float halfTrackWidthM = 0.5f * params.trackWidthM;
            params.contactPositionsBodyM[0] = Eigen::Vector2f(frontContactX, halfTrackWidthM);
            params.contactPositionsBodyM[1] = Eigen::Vector2f(frontContactX, -halfTrackWidthM);
            params.contactPositionsBodyM[2] = Eigen::Vector2f(-frontContactX, halfTrackWidthM);
            params.contactPositionsBodyM[3] = Eigen::Vector2f(-frontContactX, -halfTrackWidthM);
            return params;
        }

        Eigen::Vector2f ContactPosition(uint8_t index) const noexcept
        {
            return contactPositionsBodyM[(index < contactPositionsBodyM.size()) ? index : (contactPositionsBodyM.size() - 1U)];
        }

        float TotalNormalLoadN(float fanDutyCycle) const noexcept
        {
            return
                (massKg * gravityMps2) +
                ((std::max)(0.0f, fanDutyCycle) * fanDownforceAtFullDutyN);
        }

        float FrontWheelLoadN(float fanDutyCycle) const noexcept
        {
            return 0.5f * frontLoadFraction * TotalNormalLoadN(fanDutyCycle);
        }

        float RearWheelLoadN(float fanDutyCycle) const noexcept
        {
            return 0.5f * (1.0f - frontLoadFraction) * TotalNormalLoadN(fanDutyCycle);
        }
    };

    inline constexpr float kGeneralEncoderLinearSpeedSigmaMps = 0.0018f;
    inline constexpr float kGeneralEncoderYawRateSigmaRadps = 0.051f;
    inline constexpr float kStationaryEncoderVelocitySigmaMps = 1.76e-6f;
    inline constexpr float kEncoderPairNisThreshold = 13.81551f;
    inline constexpr float kImuYawRateSigmaRadps = 0.0013f;
    inline constexpr float kImuAccelSigmaMps2 = 0.014f;

    inline Eigen::Matrix<float, 2, 2> ComputeGeneralEncoderPairCovarianceRadps(
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
            MazeMap::Vehicle::GetPhysicalModel().trackWidthM;
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

    inline Eigen::Matrix<float, 2, 2> ComputeGeneralEncoderPairSqrtNoise(
        const PlantParams& params) noexcept
    {
        const Eigen::Matrix<float, 2, 2> covariance = ComputeGeneralEncoderPairCovarianceRadps(params);
        const Eigen::LLT<Eigen::Matrix<float, 2, 2>> llt(covariance);
        if (llt.info() == Eigen::Success)
        {
            return llt.matrixL();
        }

        Eigen::Matrix<float, 2, 2> fallback = Eigen::Matrix<float, 2, 2>::Zero();
        const float fallbackSigmaRadps = 1.0f;
        fallback(0, 0) = fallbackSigmaRadps;
        fallback(1, 1) = fallbackSigmaRadps;
        return fallback;
    }

    inline float ComputeStationaryEncoderOmegaSigmaRadps(const PlantParams& params) noexcept
    {
        if (!(params.wheelRadiusM > 0.0f) || !std::isfinite(params.wheelRadiusM))
        {
            return 1.0f;
        }

        return kStationaryEncoderVelocitySigmaMps / params.wheelRadiusM;
    }

    inline Eigen::Matrix<float, 2, 2> ComputeEncoderPairSqrtNoise(
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

    inline float ComputeEncoderPairNisThreshold(const EncoderObs& observation) noexcept
    {
        if ((observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f))
        {
            return std::numeric_limits<float>::infinity();
        }

        return kEncoderPairNisThreshold;
    }

    struct PrimitiveDefinition
    {
        float pathLengthM = 0.0f;
        float curvatureInvM = 0.0f;
        float durationS = 0.10f;
        uint8_t sampleCount = 16U;
    };

    struct OperatingPoint
    {
        float entrySpeedMps = 0.0f;
        float exitSpeedMps = 0.0f;
        float fanDutyCycle = 0.80f;
    };

    struct PrimitiveSample
    {
        float timeS = 0.0f;
        float arcLengthM = 0.0f;
        float linearSpeedMps = 0.0f;
        float yawRateRadps = 0.0f;
        float leftMotorCommand = 0.0f;
        float rightMotorCommand = 0.0f;
    };

    template <size_t MaxSamples = 64U>
    struct PrimitiveTable
    {
        uint8_t count = 0U;
        std::array<PrimitiveSample, MaxSamples> samples{};
    };

    struct WallPreprocessorInput
    {
        float ledOffLevel = 0.0f;
        float ledOnLevel = 0.0f;
        float supportSpanM = 0.05f;
        float multiSensorCoherence = 1.0f;
        float incidenceCosine = 1.0f;
        float derivativeConsistency = 1.0f;
        bool saturated = false;
    };

    struct WallSensorPreprocessorConfig
    {
        float zeroOffset = 0.0f;
        float gain = 1.0f;
        float minPseudoRangeM = 0.01f;
        float maxPseudoRangeM = 0.25f;
        float wallLikeRangeM = 0.11f;
        float openLikeRangeM = 0.16f;
        float postSupportSpanM = 0.020f;
        float wallSupportSpanM = 0.040f;
        float minConfidence = 0.25f;
        float noiseFloor = 0.02f;
        WallSensorCalibrationMode calibrationMode = WallSensorCalibrationMode::DistanceOffset;
        WallSensorCalibrationCurve calibration{};
    };

    struct LocalMapView
    {
        const Maze* maze = nullptr;
        float cellSizeM = Maze::GetCellDimension() / 100.0f;
        float wallThicknessM = WALL_THICKNESS;
        float postHalfWidthM = 0.5f * WALL_THICKNESS;
        float noHitRangeM = 0.30f;
        uint8_t radiusCells = 2U;
        bool freezeMapMutation = false;

        bool IsValid() const noexcept
        {
            return maze != nullptr;
        }
    };

    struct GeometryPrediction
    {
        bool hit = false;
        GeometryHitType type = GeometryHitType::None;
        float rangeM = 0.0f;
        Eigen::Vector2f pointWorldM = Eigen::Vector2f::Zero();
        CellCoordinates cell{};
        Direction edge = Direction::None;
        uint8_t postGridX = 0U;
        uint8_t postGridY = 0U;
    };

    struct EdgeEvidence
    {
        int8_t score = 0;
        WallState state = WallState::Unknown;
    };

    struct MapEvidenceUpdaterConfig
    {
        int8_t maxScore = 8;
        int8_t wallThreshold = 3;
        int8_t openThreshold = 3;
        int8_t wallHitWeight = 1;
        int8_t openHitWeight = 1;
        float minimumConfidence = 0.25f;
    };

    struct MeasurementUpdateResult
    {
        bool attempted = false;
        bool accepted = false;
        float nis = 0.0f;
    };

    struct WallUpdateResult
    {
        MeasurementUpdateResult filter{};
        GeometryPrediction prediction{};
    };

    struct FrontPairUpdateResult
    {
        MeasurementUpdateResult filter{};
        GeometryPrediction leftPrediction{};
        GeometryPrediction rightPrediction{};
    };

    inline Eigen::Vector2f HeadingUnitFromYaw(float yaw) noexcept
    {
        return Eigen::Vector2f(cosf(yaw), sinf(yaw));
    }

    inline Eigen::Vector2f RotateBodyVectorToWorld(
        const Eigen::Vector2f& vectorBody,
        const Eigen::Vector2f& heading) noexcept
    {
        return Eigen::Vector2f(
            (heading.x() * vectorBody.x()) - (heading.y() * vectorBody.y()),
            (heading.y() * vectorBody.x()) + (heading.x() * vectorBody.y()));
    }

    inline Eigen::Vector2f ResolveSensorDirectionBody(const SensorExtrinsics& sensor) noexcept
    {
        if (std::fabs(sensor.yawOffsetRad) <= 1.0e-6f)
        {
            return sensor.directionBody;
        }

        const Eigen::Vector2f offsetHeading = HeadingUnitFromYaw(sensor.yawOffsetRad);
        return RotateBodyVectorToWorld(sensor.directionBody, offsetHeading);
    }

    inline float FastAngleErrorRad(float targetAngleRad, float currentAngleRad) noexcept
    {
        return VehicleState::NormalizeAngle(targetAngleRad - currentAngleRad);
    }

    class PlantModel
    {
    public:
        using StateVector = VehicleState::StateVector;

        PlantDerivatives forwardStep(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;

        WheelKinematics wheelKinematics(const StateVector& state, const PlantParams& params) const noexcept;
        SlipTargets slipTargets(const StateVector& state, const PlantParams& params) const noexcept;
        SlipTargets slipTargets(
            const StateVector& state,
            const WheelKinematics& kinematics,
            const PlantParams& params) const noexcept;
        ContactForces tireForces(const StateVector& state, const PlantParams& params) const noexcept;
        ContactForces tireForces(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        Eigen::Vector2f imuPlanarAcceleration(
            const StateVector& state,
            const ControlInput& control,
            const PlantParams& params) const noexcept;
        StateVector integrateMidpoint(
            const StateVector& state,
            const ControlInput& control,
            float dt,
            const PlantParams& params) const noexcept;
        float driveTorqueFromCommand(
            float motorCommand,
            float wheelBankSpeedRadps,
            float batteryVoltageV,
            const PlantParams& params) const noexcept;
        float driveFrictionTorque(float wheelBankSpeedRadps, const PlantParams& params) const noexcept;
    };

    class PrimitiveInverseSolver
    {
    public:
        template <size_t MaxSamples = 64U>
        PrimitiveTable<MaxSamples> solvePrimitive(
            const PrimitiveDefinition& primitive,
            const OperatingPoint& operatingPoint,
            const PlantModel& plant,
            const PlantParams& params) const noexcept
        {
            PrimitiveTable<MaxSamples> table{};
            if (!(primitive.durationS > 0.0f) || !(primitive.sampleCount > 0U))
            {
                return table;
            }

            const uint8_t count = (primitive.sampleCount > static_cast<uint8_t>(MaxSamples)) ?
                static_cast<uint8_t>(MaxSamples) :
                primitive.sampleCount;
            table.count = count;
            const float durationS = primitive.durationS;
            const float linearAccelMps2 =
                (operatingPoint.exitSpeedMps - operatingPoint.entrySpeedMps) / durationS;
            const float trackWidthM = params.trackWidthM;
            const float wheelRadiusM = params.wheelRadiusM;

            for (uint8_t index = 0U; index < count; ++index)
            {
                PrimitiveSample& sample = table.samples[index];
                const float alpha = (count > 1U) ?
                    (static_cast<float>(index) / static_cast<float>(count - 1U)) :
                    0.0f;
                sample.timeS = durationS * alpha;
                sample.arcLengthM = primitive.pathLengthM * alpha;
                sample.linearSpeedMps =
                    operatingPoint.entrySpeedMps +
                    (alpha * (operatingPoint.exitSpeedMps - operatingPoint.entrySpeedMps));
                sample.yawRateRadps = primitive.curvatureInvM * sample.linearSpeedMps;

                const float angularAccelRadps2 = primitive.curvatureInvM * linearAccelMps2;
                const float leftSpeedMps = sample.linearSpeedMps - (0.5f * trackWidthM * sample.yawRateRadps);
                const float rightSpeedMps = sample.linearSpeedMps + (0.5f * trackWidthM * sample.yawRateRadps);
                const float leftWheelSpeedRadps = leftSpeedMps / wheelRadiusM;
                const float rightWheelSpeedRadps = rightSpeedMps / wheelRadiusM;
                const float leftWheelAccelRadps2 =
                    (linearAccelMps2 - (0.5f * trackWidthM * angularAccelRadps2)) / wheelRadiusM;
                const float rightWheelAccelRadps2 =
                    (linearAccelMps2 + (0.5f * trackWidthM * angularAccelRadps2)) / wheelRadiusM;

                const float longitudinalForcePerSideN = 0.5f * params.massKg * linearAccelMps2;
                const float yawForcePerSideN =
                    (trackWidthM > 0.0f) ?
                    ((params.yawInertiaKgM2 * angularAccelRadps2) / trackWidthM) :
                    0.0f;
                const float leftGroundForceN = longitudinalForcePerSideN - yawForcePerSideN;
                const float rightGroundForceN = longitudinalForcePerSideN + yawForcePerSideN;

                const float leftWheelTorqueNm =
                    (params.equivalentWheelInertiaKgM2 * leftWheelAccelRadps2) +
                    (params.wheelRadiusM * leftGroundForceN) +
                    plant.driveFrictionTorque(leftWheelSpeedRadps, params);
                const float rightWheelTorqueNm =
                    (params.equivalentWheelInertiaKgM2 * rightWheelAccelRadps2) +
                    (params.wheelRadiusM * rightGroundForceN) +
                    plant.driveFrictionTorque(rightWheelSpeedRadps, params);

                sample.leftMotorCommand = inverseMotorCommand(leftWheelTorqueNm, leftWheelSpeedRadps, params);
                sample.rightMotorCommand = inverseMotorCommand(rightWheelTorqueNm, rightWheelSpeedRadps, params);
            }

            return table;
        }

    private:
        static float inverseMotorCommand(float wheelTorqueNm, float wheelSpeedRadps, const PlantParams& params) noexcept
        {
            const float motorTorqueNm =
                (params.drivetrainEfficiency > 0.0f) ?
                (wheelTorqueNm / ((std::max)(params.gearRatio * params.drivetrainEfficiency, 1.0e-4f))) :
                0.0f;
            float motorCurrentA =
                (params.torqueConstantNmPerA > 0.0f) ?
                (motorTorqueNm / params.torqueConstantNmPerA) :
                0.0f;
            if (motorCurrentA > 0.0f)
            {
                motorCurrentA += params.noLoadCurrentA;
            }
            else if (motorCurrentA < 0.0f)
            {
                motorCurrentA -= params.noLoadCurrentA;
            }

            const float motorSpeedRadps = wheelSpeedRadps * params.gearRatio;
            const float backEmfVoltageV =
                (params.speedConstantRadpsPerVolt > 0.0f) ?
                (motorSpeedRadps / params.speedConstantRadpsPerVolt) :
                0.0f;
            const float commandedVoltageV =
                (motorCurrentA * params.driveResistanceOhms) + backEmfVoltageV;
            if (!(params.supplyVoltageV > 0.0f))
            {
                return 0.0f;
            }
            return (std::clamp)(commandedVoltageV / params.supplyVoltageV, -1.0f, 1.0f);
        }
    };

    class WallSensorPreprocessor
    {
    public:
        explicit WallSensorPreprocessor(
            const WallSensorPreprocessorConfig& config = WallSensorPreprocessorConfig{}) noexcept
            : _config(config)
        {
        }

        const WallSensorPreprocessorConfig& config() const noexcept
        {
            return _config;
        }

        WallObs process(
            const WallSensor& sensor,
            const WallPreprocessorInput& input) const noexcept
        {
            WallObs observation{};
            const float differential =
                (std::max)(0.0f, ((sensor.DifferentialLightLevel(input.ledOffLevel, input.ledOnLevel) - _config.zeroOffset) * _config.gain));
            if (!std::isfinite(differential))
            {
                return observation;
            }

            float pseudoRangeM = sensor.DistanceFromDifferentialLight(differential);
            if (_config.calibration.GetCount() > 0U)
            {
                pseudoRangeM = _config.calibration.Apply(pseudoRangeM, _config.calibrationMode);
            }

            if (!(std::isfinite(pseudoRangeM) &&
                (pseudoRangeM >= _config.minPseudoRangeM) &&
                (pseudoRangeM <= _config.maxPseudoRangeM)))
            {
                return observation;
            }

            const float ambientMagnitude = (std::max)(std::fabs(input.ledOffLevel), _config.noiseFloor);
            const float snr = differential / ambientMagnitude;
            const float snrScore = (std::clamp)((snr - 1.0f) / 7.0f, 0.0f, 1.0f);
            const float supportScore = (std::clamp)(input.supportSpanM / _config.wallSupportSpanM, 0.0f, 1.0f);
            const float coherenceScore = (std::clamp)(input.multiSensorCoherence, 0.0f, 1.0f);
            const float incidenceScore = (std::clamp)(input.incidenceCosine, 0.0f, 1.0f);
            const float derivativeScore = (std::clamp)(input.derivativeConsistency, 0.0f, 1.0f);
            const float saturationScore = input.saturated ? 0.15f : 1.0f;

            observation.valid = true;
            observation.rho = pseudoRangeM;
            observation.confidence =
                (0.30f * snrScore) +
                (0.20f * supportScore) +
                (0.20f * coherenceScore) +
                (0.15f * incidenceScore) +
                (0.10f * derivativeScore) +
                (0.05f * saturationScore);
            observation.confidence = (std::clamp)(observation.confidence, 0.0f, 1.0f);

            if (observation.confidence < _config.minConfidence)
            {
                observation.cls = ObsClass::Ambiguous;
            }
            else if (pseudoRangeM >= _config.openLikeRangeM)
            {
                observation.cls = ObsClass::OpenLike;
            }
            else if (input.supportSpanM <= _config.postSupportSpanM)
            {
                observation.cls = ObsClass::PostLike;
            }
            else if (pseudoRangeM <= _config.wallLikeRangeM)
            {
                observation.cls = ObsClass::WallLike;
            }
            else
            {
                observation.cls = ObsClass::Ambiguous;
            }

            return observation;
        }

    private:
        WallSensorPreprocessorConfig _config;
    };

    class WallGeometryModel
    {
    public:
        struct GeometryStateFrame
        {
            Eigen::Vector2f positionWorldM = Eigen::Vector2f::Zero();
            Eigen::Vector2f heading = Eigen::Vector2f(1.0f, 0.0f);
            CellCoordinates centerCell{};
        };

        GeometryPrediction predictRay(
            const VehicleState::StateVector& state,
            const SensorExtrinsics& sensorExtrinsics,
            const LocalMapView& map) const noexcept;

        GeometryPrediction predictRay(
            const GeometryStateFrame& frame,
            const SensorExtrinsics& sensorExtrinsics,
            const LocalMapView& map) const noexcept;

        Eigen::Vector2f sensorOriginWorld(
            const VehicleState::StateVector& state,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        Eigen::Vector2f sensorOriginWorld(
            const GeometryStateFrame& frame,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        Eigen::Vector2f sensorDirectionWorld(
            const VehicleState::StateVector& state,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        Eigen::Vector2f sensorDirectionWorld(
            const GeometryStateFrame& frame,
            const SensorExtrinsics& sensorExtrinsics) const noexcept;

        GeometryStateFrame buildStateFrame(
            const VehicleState::StateVector& state,
            const LocalMapView& map) const noexcept;

    private:
        static CellCoordinates worldToCell(float xMeters, float yMeters, float cellSizeM) noexcept;
        static bool intersectRayAabb(
            const Eigen::Vector2f& origin,
            const Eigen::Vector2f& direction,
            const Eigen::Vector2f& minCorner,
            const Eigen::Vector2f& maxCorner,
            float& rangeM) noexcept;
        static void testUniqueWall(
            const Eigen::Vector2f& rayOrigin,
            const Eigen::Vector2f& rayDirection,
            const CellCoordinates& cell,
            Direction direction,
            WallState state,
            const LocalMapView& map,
            GeometryPrediction& best) noexcept;
    };

    class MapEvidenceUpdater
    {
    public:
        static constexpr uint8_t kMazeSize = 16U;
        static constexpr uint8_t kDirectionCount = 4U;

        void Reset() noexcept
        {
            for (auto& plane : _edges)
            {
                for (auto& row : plane)
                {
                    for (auto& edge : row)
                    {
                        edge = EdgeEvidence{};
                    }
                }
            }
        }

        const EdgeEvidence& Get(const CellCoordinates& cell, Direction direction) const noexcept
        {
            return _edges[cell.GetX()][cell.GetY()][directionIndex(direction)];
        }

        bool Apply(
            const CellCoordinates& cell,
            Direction direction,
            const WallObs& observation,
            const GeometryPrediction& bestFit,
            const MapEvidenceUpdaterConfig& config = MapEvidenceUpdaterConfig{},
            bool freezeMutation = false) noexcept;

    private:
        using DirectionRow = std::array<EdgeEvidence, kDirectionCount>;
        using MazeRow = std::array<DirectionRow, kMazeSize>;
        using MazePlane = std::array<MazeRow, kMazeSize>;

        MazePlane _edges{};

        static bool isOrdinal(Direction direction) noexcept;
        static size_t directionIndex(Direction direction) noexcept;
        static int8_t saturatingAdd(int8_t current, int8_t delta, int8_t limit) noexcept;
        static WallState stateFromScore(int8_t score, const MapEvidenceUpdaterConfig& config) noexcept;
        void setMirrored(const CellCoordinates& cell, Direction direction, const EdgeEvidence& evidence) noexcept;
    };

    class SrUkfCore
    {
    public:
        using StateVector = VehicleState::StateVector;
        using StateMatrix = VehicleState::StateMatrix;

        SrUkfCore(
            const PlantParams& params = PlantParams::Default(),
            const PlantModel& plantModel = PlantModel()) noexcept;

        const StateVector& state() const noexcept
        {
            return _filter.state();
        }

        StateMatrix covariance() const noexcept
        {
            return _filter.covariance();
        }

        const PlantParams& params() const noexcept
        {
            return _params;
        }

        bool reset(const StateVector& state, const StateMatrix& covariance) noexcept
        {
            _filter.setState(state, covariance);
            _lastControl = ControlInput{};
            _lastEncoderObs = EncoderObs{};
            _haveEncoderReference = false;
            return true;
        }

        bool setState(const StateVector& state, const StateMatrix& covariance) noexcept
        {
            _filter.setState(state, covariance);
            return true;
        }

        bool predict(float dt, const ControlInput& control) noexcept;
        template <typename LoopHook>
        bool predict(float dt, const ControlInput& control, LoopHook&& loopHook) noexcept;

        MeasurementUpdateResult updateEncoderPair(const EncoderObs& observation, float dt) noexcept;
        template <typename LoopHook>
        MeasurementUpdateResult updateEncoderPair(const EncoderObs& observation, float dt, LoopHook&& loopHook) noexcept;

        MeasurementUpdateResult updateYawRate(float yawRateRadps) noexcept;
        template <typename LoopHook>
        MeasurementUpdateResult updateYawRate(float yawRateRadps, LoopHook&& loopHook) noexcept;
        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation) noexcept;
        template <typename LoopHook>
        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation, LoopHook&& loopHook) noexcept;
        MeasurementUpdateResult updateImuMerged(const ImuMergedObs& observation) noexcept;
        FrontPairUpdateResult updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const LocalMapView& map) noexcept;
        WallUpdateResult updateSideSensor(
            Side which,
            const WallObs& observation,
            const LocalMapView& map) noexcept;

    private:
        static bool HasExactZeroWheelObservation(const EncoderObs& observation) noexcept;
        static float wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept;
        bool controlCommandsAreEffectivelyZero() const noexcept;
        void applyWheelRateConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept;
        void applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept;
        Eigen::Matrix<float, 2, 1> accelPredictionForState(const StateVector& sigmaPoint) const noexcept;
        Eigen::Matrix<float, 2, 1> frontPairPredictionForState(
            const StateVector& sigmaPoint,
            const LocalMapView& map) const noexcept;
        float wallPredictionForSensor(
            const StateVector& sigmaPoint,
            const SensorExtrinsics& sensor,
            const LocalMapView& map) const noexcept;

        PlantModel _plantModel;
        WallGeometryModel _geometryModel;
        PlantParams _params;
        UKF<VehicleState::kDimension, 3, 3> _filter;
        ControlInput _lastControl;
        EncoderObs _lastEncoderObs;
        bool _haveEncoderReference;
        StateMatrix _sqrtProcessNoiseDensity;
        Eigen::Matrix<float, 2, 2> _sqrtEncoderNoise;
        Eigen::Matrix<float, 3, 3> _sqrtImuNoise;
        Eigen::Matrix<float, 2, 2> _sqrtFrontNoise;
        Eigen::Matrix<float, 1, 1> _sqrtSideNoise;
    };

    class MouseUkfFacade
    {
    public:
        explicit MouseUkfFacade(const PlantParams& params = PlantParams::Default()) noexcept;

        SrUkfCore& ukf() noexcept { return _core; }
        const SrUkfCore& ukf() const noexcept { return _core; }
        MapEvidenceUpdater& mapEvidence() noexcept { return _mapEvidence; }
        const MapEvidenceUpdater& mapEvidence() const noexcept { return _mapEvidence; }

        bool predict(float dt, const ControlInput& control) noexcept
        {
            return _core.predict(dt, control);
        }

        template <typename LoopHook>
        bool predict(float dt, const ControlInput& control, LoopHook&& loopHook) noexcept
        {
            return _core.predict(dt, control, loopHook);
        }

        MeasurementUpdateResult updateEncoderPair(const EncoderObs& observation, float dt) noexcept
        {
            return _core.updateEncoderPair(observation, dt);
        }

        template <typename LoopHook>
        MeasurementUpdateResult updateEncoderPair(const EncoderObs& observation, float dt, LoopHook&& loopHook) noexcept
        {
            return _core.updateEncoderPair(observation, dt, loopHook);
        }

        MeasurementUpdateResult updateYawRate(float yawRateRadps) noexcept
        {
            return _core.updateYawRate(yawRateRadps);
        }

        template <typename LoopHook>
        MeasurementUpdateResult updateYawRate(float yawRateRadps, LoopHook&& loopHook) noexcept
        {
            return _core.updateYawRate(yawRateRadps, loopHook);
        }

        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation) noexcept
        {
            return _core.updatePlanarAccel(observation);
        }

        template <typename LoopHook>
        MeasurementUpdateResult updatePlanarAccel(const ImuAccelObs& observation, LoopHook&& loopHook) noexcept
        {
            return _core.updatePlanarAccel(observation, loopHook);
        }

        MeasurementUpdateResult updateImuMerged(const ImuMergedObs& observation) noexcept
        {
            return _core.updateImuMerged(observation);
        }

        bool reset(
            const SrUkfCore::StateVector& state,
            const SrUkfCore::StateMatrix& covariance) noexcept
        {
            _mapEvidence.Reset();
            return _core.reset(state, covariance);
        }

        FrontPairUpdateResult updateFrontPair(
            const WallObs& left,
            const WallObs& right,
            const LocalMapView& map,
            const MapEvidenceUpdaterConfig& evidenceConfig = MapEvidenceUpdaterConfig{}) noexcept;

        WallUpdateResult updateSideSensor(
            Side which,
            const WallObs& observation,
            const LocalMapView& map,
            const MapEvidenceUpdaterConfig& evidenceConfig = MapEvidenceUpdaterConfig{}) noexcept;

    private:
        static Direction dominantDirectionForSensor(
            const SensorExtrinsics& sensor,
            const VehicleState::StateVector& state) noexcept;
        static CellCoordinates estimateSensorCell(
            const SensorExtrinsics& sensor,
            const VehicleState::StateVector& state,
            float cellSizeM) noexcept;

        PlantParams _params;
        SrUkfCore _core;
        MapEvidenceUpdater _mapEvidence;
    };

    inline PlantDerivatives PlantModel::forwardStep(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        PlantDerivatives derivatives{};
        if (!(std::isfinite(control.leftMotorCommand) &&
            std::isfinite(control.rightMotorCommand) &&
            std::isfinite(control.fanDutyCycle) &&
            std::isfinite(params.massKg) &&
            std::isfinite(params.effectiveLongitudinalMassKg) &&
            std::isfinite(params.yawInertiaKgM2) &&
            (params.massKg > 0.0f) &&
            (params.effectiveLongitudinalMassKg > 0.0f) &&
            (params.yawInertiaKgM2 > 0.0f) &&
            (params.wheelRadiusM > 0.0f) &&
            (params.trackWidthM > 0.0f)))
        {
            return derivatives;
        }

        const WheelKinematics kinematics = wheelKinematics(state, params);
        const SlipTargets targets = slipTargets(state, kinematics, params);
        const ContactForces forces = tireForces(state, control, params);

        const float u = state(VehicleState::kU);
        const float v = state(VehicleState::kV);
        const float psi = state(VehicleState::kPsi);
        const float r = state(VehicleState::kR);
        const float omegaLeft = state(VehicleState::kOmegaL);
        const float omegaRight = state(VehicleState::kOmegaR);
        const float batteryVoltageV =
            (std::isfinite(control.batteryVoltageV) && (control.batteryVoltageV > 0.0f)) ?
            control.batteryVoltageV :
            params.supplyVoltageV;

        float yawMomentNm = 0.0f;
        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const Eigen::Vector2f contactPosition = params.ContactPosition(contactIndex);
            yawMomentNm +=
                (contactPosition.x() * forces.contacts[contactIndex].fy) -
                (contactPosition.y() * forces.contacts[contactIndex].fx);
        }

        const float tauMotorLeft = driveTorqueFromCommand(control.leftMotorCommand, omegaLeft, batteryVoltageV, params);
        const float tauMotorRight = driveTorqueFromCommand(control.rightMotorCommand, omegaRight, batteryVoltageV, params);
        const float tauFrictionLeft = driveFrictionTorque(omegaLeft, params);
        const float tauFrictionRight = driveFrictionTorque(omegaRight, params);
        const Eigen::Vector2f heading = HeadingUnitFromYaw(psi);

        derivatives.stateDot(VehicleState::kPx) = (u * heading.x()) - (v * heading.y());
        derivatives.stateDot(VehicleState::kPy) = (u * heading.y()) + (v * heading.x());
        derivatives.stateDot(VehicleState::kPsi) = r;
        derivatives.stateDot(VehicleState::kU) = (r * v) + (forces.SumFx() / params.effectiveLongitudinalMassKg);
        derivatives.stateDot(VehicleState::kV) = (-r * u) + (forces.SumFy() / params.massKg);
        derivatives.stateDot(VehicleState::kR) = yawMomentNm / params.yawInertiaKgM2;
        derivatives.stateDot(VehicleState::kOmegaL) =
            (tauMotorLeft - (params.wheelRadiusM * forces.LeftBankFx()) - tauFrictionLeft) /
            params.equivalentWheelInertiaKgM2;
        derivatives.stateDot(VehicleState::kOmegaR) =
            (tauMotorRight - (params.wheelRadiusM * forces.RightBankFx()) - tauFrictionRight) /
            params.equivalentWheelInertiaKgM2;
        derivatives.stateDot(VehicleState::kBgz) = 0.0f;

        derivatives.contactForces = forces;
        derivatives.wheelKinematics = kinematics;
        derivatives.slipTargets = targets;
        derivatives.originAccelBodyMps2 = Eigen::Vector2f(
            derivatives.stateDot(VehicleState::kU) - (r * v),
            derivatives.stateDot(VehicleState::kV) + (r * u));
        derivatives.imuAccelBodyMps2 = Eigen::Vector2f(
            derivatives.originAccelBodyMps2.x() -
                (derivatives.stateDot(VehicleState::kR) * params.imu.positionBodyM.y()) -
                ((r * r) * params.imu.positionBodyM.x()),
            derivatives.originAccelBodyMps2.y() +
                (derivatives.stateDot(VehicleState::kR) * params.imu.positionBodyM.x()) -
                ((r * r) * params.imu.positionBodyM.y()));
        derivatives.longitudinalAccelMps2 = derivatives.originAccelBodyMps2.x();
        derivatives.lateralAccelMps2 = derivatives.originAccelBodyMps2.y();
        derivatives.yawAccelRadps2 = derivatives.stateDot(VehicleState::kR);
        return derivatives;
    }

    inline WheelKinematics PlantModel::wheelKinematics(const StateVector& state, const PlantParams& params) const noexcept
    {
        WheelKinematics kinematics{};
        const float u = state(VehicleState::kU);
        const float v = state(VehicleState::kV);
        const float r = state(VehicleState::kR);

        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const Eigen::Vector2f position = params.ContactPosition(contactIndex);
            ContactKinematics& contact = kinematics.contacts[contactIndex];
            contact.vx = u - (r * position.y());
            contact.vy = v + (r * position.x());
        }

        kinematics.leftBankLongitudinalVelocityMps = kinematics.contacts[0].vx;
        kinematics.rightBankLongitudinalVelocityMps = kinematics.contacts[1].vx;
        return kinematics;
    }

    inline SlipTargets PlantModel::slipTargets(const StateVector& state, const PlantParams& params) const noexcept
    {
        return slipTargets(state, wheelKinematics(state, params), params);
    }

    inline SlipTargets PlantModel::slipTargets(
        const StateVector& state,
        const WheelKinematics& kinematics,
        const PlantParams& params) const noexcept
    {
        SlipTargets targets{};
        const float velocityEpsilonMps = (std::max)(params.velocityEpsilonMps, 1.0e-3f);
        const float wheelCircumferentialLeft = params.wheelRadiusM * state(VehicleState::kOmegaL);
        const float wheelCircumferentialRight = params.wheelRadiusM * state(VehicleState::kOmegaR);

        targets.kappaLeft =
            (wheelCircumferentialLeft - kinematics.leftBankLongitudinalVelocityMps) /
            (std::max)(std::fabs(kinematics.leftBankLongitudinalVelocityMps), velocityEpsilonMps);
        targets.kappaRight =
            (wheelCircumferentialRight - kinematics.rightBankLongitudinalVelocityMps) /
            (std::max)(std::fabs(kinematics.rightBankLongitudinalVelocityMps), velocityEpsilonMps);

        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            const ContactKinematics& contact = kinematics.contacts[contactIndex];
            targets.lateralRatio[contactIndex] =
                -contact.vy / (std::max)(std::fabs(contact.vx), velocityEpsilonMps);
        }
        return targets;
    }

    inline ContactForces PlantModel::tireForces(const StateVector& state, const PlantParams& params) const noexcept
    {
        return tireForces(state, ControlInput{}, params);
    }

    inline ContactForces PlantModel::tireForces(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        ContactForces forces{};
        const WheelKinematics kinematics = wheelKinematics(state, params);
        const SlipTargets targets = slipTargets(state, kinematics, params);
        const float fanDutyCycle = (std::clamp)(control.fanDutyCycle, 0.0f, 1.0f);
        const float frontWheelLoadN = params.FrontWheelLoadN(fanDutyCycle);
        const float rearWheelLoadN = params.RearWheelLoadN(fanDutyCycle);

        for (uint8_t contactIndex = 0U; contactIndex < 4U; ++contactIndex)
        {
            ContactForce& force = forces.contacts[contactIndex];
            const bool isFront = contactIndex < 2U;
            const bool isLeft = (contactIndex == 0U) || (contactIndex == 2U);
            const float kappa = isLeft ? targets.kappaLeft : targets.kappaRight;
            const float lateralRatio = targets.lateralRatio[contactIndex];
            const float mu = isFront ? params.muFront : params.muRear;
            const float corneringStiffness =
                isFront ? params.corneringStiffnessFrontNPerRad : params.corneringStiffnessRearNPerRad;
            const float normalLoadN = isFront ? frontWheelLoadN : rearWheelLoadN;
            const float fx0 = 0.5f * params.longitudinalStiffnessN * kappa;
            const float fy0 = -corneringStiffness * lateralRatio;
            const float requestedForceMagnitudeN = MazeMap::Math::Sqrtf((fx0 * fx0) + (fy0 * fy0));
            const float lambda =
                (mu * normalLoadN) /
                ((2.0f * requestedForceMagnitudeN) + (std::max)(params.forceEpsilonN, 1.0e-5f));
            const float saturation =
                (lambda >= 1.0f) ? 1.0f :
                (lambda <= 0.0f) ? 0.0f :
                (lambda * (2.0f - lambda));
            force.fx = saturation * fx0;
            force.fy = saturation * fy0;
            force.fz = normalLoadN;
            force.saturation = saturation;
        }
        return forces;
    }

    inline Eigen::Vector2f PlantModel::imuPlanarAcceleration(
        const StateVector& state,
        const ControlInput& control,
        const PlantParams& params) const noexcept
    {
        return forwardStep(state, control, params).imuAccelBodyMps2;
    }

    inline PlantModel::StateVector PlantModel::integrateMidpoint(
        const StateVector& state,
        const ControlInput& control,
        float dt,
        const PlantParams& params) const noexcept
    {
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return state;
        }

        const PlantDerivatives first = forwardStep(state, control, params);
        StateVector midpointState = state + (0.5f * dt * first.stateDot);
        VehicleState::NormalizeStateVector(midpointState);
        const PlantDerivatives midpoint = forwardStep(midpointState, control, params);
        StateVector integratedState = state + (dt * midpoint.stateDot);
        VehicleState::NormalizeStateVector(integratedState);
        return integratedState;
    }

    inline float PlantModel::driveTorqueFromCommand(
        float motorCommand,
        float wheelBankSpeedRadps,
        float batteryVoltageV,
        const PlantParams& params) const noexcept
    {
        const float command = (std::clamp)(motorCommand, -1.0f, 1.0f);
        const float motorSpeedRadps = wheelBankSpeedRadps * params.gearRatio;
        const float appliedVoltageV =
            command *
            ((std::isfinite(batteryVoltageV) && (batteryVoltageV > 0.0f)) ? batteryVoltageV : params.supplyVoltageV);
        const float backEmfVoltageV =
            (params.speedConstantRadpsPerVolt > 0.0f) ?
            (motorSpeedRadps / params.speedConstantRadpsPerVolt) :
            0.0f;
        float currentA =
            (params.driveResistanceOhms > 0.0f) ?
            ((appliedVoltageV - backEmfVoltageV) / params.driveResistanceOhms) :
            0.0f;
        if (params.motorCurrentLimitA > 0.0f)
        {
            currentA = (std::clamp)(currentA, -params.motorCurrentLimitA, params.motorCurrentLimitA);
        }

        const float motorTorqueNm = params.torqueConstantNmPerA * currentA;
        return motorTorqueNm * params.gearRatio * params.drivetrainEfficiency;
    }

    inline float PlantModel::driveFrictionTorque(float wheelBankSpeedRadps, const PlantParams& params) const noexcept
    {
        const float sign =
            (wheelBankSpeedRadps > 0.0f) ? 1.0f :
            (wheelBankSpeedRadps < 0.0f) ? -1.0f :
            0.0f;
        return (params.rollingFrictionTorqueNm * sign) +
            (params.viscousFrictionNmPerRadps * wheelBankSpeedRadps);
    }

    inline Eigen::Vector2f WallGeometryModel::sensorOriginWorld(
        const VehicleState::StateVector& state,
        const SensorExtrinsics& sensorExtrinsics) const noexcept
    {
        return sensorOriginWorld(buildStateFrame(state, LocalMapView{}), sensorExtrinsics);
    }

    inline Eigen::Vector2f WallGeometryModel::sensorOriginWorld(
        const GeometryStateFrame& frame,
        const SensorExtrinsics& sensorExtrinsics) const noexcept
    {
        return frame.positionWorldM + RotateBodyVectorToWorld(sensorExtrinsics.positionBodyM, frame.heading);
    }

    inline Eigen::Vector2f WallGeometryModel::sensorDirectionWorld(
        const VehicleState::StateVector& state,
        const SensorExtrinsics& sensorExtrinsics) const noexcept
    {
        return sensorDirectionWorld(buildStateFrame(state, LocalMapView{}), sensorExtrinsics);
    }

    inline Eigen::Vector2f WallGeometryModel::sensorDirectionWorld(
        const GeometryStateFrame& frame,
        const SensorExtrinsics& sensorExtrinsics) const noexcept
    {
        return RotateBodyVectorToWorld(ResolveSensorDirectionBody(sensorExtrinsics), frame.heading);
    }

    inline WallGeometryModel::GeometryStateFrame WallGeometryModel::buildStateFrame(
        const VehicleState::StateVector& state,
        const LocalMapView& map) const noexcept
    {
        GeometryStateFrame frame{};
        frame.positionWorldM = Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy));
        frame.heading = HeadingUnitFromYaw(state(VehicleState::kPsi));
        frame.centerCell = worldToCell(frame.positionWorldM.x(), frame.positionWorldM.y(), map.cellSizeM);
        return frame;
    }

    inline CellCoordinates WallGeometryModel::worldToCell(float xMeters, float yMeters, float cellSizeM) noexcept
    {
        const float safeCellSize = (cellSizeM > 0.0f) ? cellSizeM : 0.18f;
        int x = static_cast<int>(std::floor(xMeters / safeCellSize));
        int y = static_cast<int>(std::floor(yMeters / safeCellSize));
        x = (std::clamp)(x, 0, 15);
        y = (std::clamp)(y, 0, 15);
        return CellCoordinates(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
    }

    inline bool WallGeometryModel::intersectRayAabb(
        const Eigen::Vector2f& origin,
        const Eigen::Vector2f& direction,
        const Eigen::Vector2f& minCorner,
        const Eigen::Vector2f& maxCorner,
        float& rangeM) noexcept
    {
        rangeM = 0.0f;
        float tMin = 0.0f;
        float tMax = (std::numeric_limits<float>::max)();

        for (int axis = 0; axis < 2; ++axis)
        {
            const float originComponent = origin[axis];
            const float directionComponent = direction[axis];
            const float minComponent = minCorner[axis];
            const float maxComponent = maxCorner[axis];

            if (std::fabs(directionComponent) <= 1.0e-6f)
            {
                if ((originComponent < minComponent) || (originComponent > maxComponent))
                {
                    return false;
                }
                continue;
            }

            float t1 = (minComponent - originComponent) / directionComponent;
            float t2 = (maxComponent - originComponent) / directionComponent;
            if (t1 > t2)
            {
                const float swap = t1;
                t1 = t2;
                t2 = swap;
            }
            tMin = (std::max)(tMin, t1);
            tMax = (std::min)(tMax, t2);
            if (tMax < tMin)
            {
                return false;
            }
        }

        if (tMax <= 0.0f)
        {
            return false;
        }

        rangeM = (tMin > 0.0f) ? tMin : tMax;
        return std::isfinite(rangeM) && (rangeM > 0.0f);
    }

    inline void WallGeometryModel::testUniqueWall(
        const Eigen::Vector2f& rayOrigin,
        const Eigen::Vector2f& rayDirection,
        const CellCoordinates& cell,
        Direction direction,
        WallState state,
        const LocalMapView& map,
        GeometryPrediction& best) noexcept
    {
        if (state != WallState::Wall)
        {
            return;
        }

        const float cellX = static_cast<float>(cell.GetX()) * map.cellSizeM;
        const float cellY = static_cast<float>(cell.GetY()) * map.cellSizeM;

        Eigen::Vector2f minCorner = Eigen::Vector2f::Zero();
        Eigen::Vector2f maxCorner = Eigen::Vector2f::Zero();
        switch (direction)
        {
        case Direction::Up:
            minCorner = Eigen::Vector2f(cellX, cellY + map.cellSizeM - (0.5f * map.wallThicknessM));
            maxCorner = Eigen::Vector2f(cellX + map.cellSizeM, cellY + map.cellSizeM + (0.5f * map.wallThicknessM));
            break;
        case Direction::Down:
            minCorner = Eigen::Vector2f(cellX, cellY - (0.5f * map.wallThicknessM));
            maxCorner = Eigen::Vector2f(cellX + map.cellSizeM, cellY + (0.5f * map.wallThicknessM));
            break;
        case Direction::Left:
            minCorner = Eigen::Vector2f(cellX - (0.5f * map.wallThicknessM), cellY);
            maxCorner = Eigen::Vector2f(cellX + (0.5f * map.wallThicknessM), cellY + map.cellSizeM);
            break;
        case Direction::Right:
        default:
            minCorner = Eigen::Vector2f(cellX + map.cellSizeM - (0.5f * map.wallThicknessM), cellY);
            maxCorner = Eigen::Vector2f(cellX + map.cellSizeM + (0.5f * map.wallThicknessM), cellY + map.cellSizeM);
            break;
        }

        float rangeM = 0.0f;
        if (!intersectRayAabb(rayOrigin, rayDirection, minCorner, maxCorner, rangeM))
        {
            return;
        }
        if (!(rangeM > 0.0f) || !(rangeM < best.rangeM))
        {
            return;
        }

        best.hit = true;
        best.type = GeometryHitType::WallFace;
        best.rangeM = rangeM;
        best.pointWorldM = rayOrigin + (rayDirection * rangeM);
        best.cell = cell;
        best.edge = direction;
    }

    inline GeometryPrediction WallGeometryModel::predictRay(
        const VehicleState::StateVector& state,
        const SensorExtrinsics& sensorExtrinsics,
        const LocalMapView& map) const noexcept
    {
        return predictRay(buildStateFrame(state, map), sensorExtrinsics, map);
    }

    inline GeometryPrediction WallGeometryModel::predictRay(
        const GeometryStateFrame& frame,
        const SensorExtrinsics& sensorExtrinsics,
        const LocalMapView& map) const noexcept
    {
        GeometryPrediction best{};
        best.rangeM = map.noHitRangeM;
        if (!map.IsValid())
        {
            return best;
        }

        const Eigen::Vector2f rayOrigin = sensorOriginWorld(frame, sensorExtrinsics);
        const Eigen::Vector2f rayDirection = sensorDirectionWorld(frame, sensorExtrinsics);
        const CellCoordinates centerCell = frame.centerCell;

        const int minX = (std::max)(0, static_cast<int>(centerCell.GetX()) - static_cast<int>(map.radiusCells));
        const int maxX = (std::min)(15, static_cast<int>(centerCell.GetX()) + static_cast<int>(map.radiusCells));
        const int minY = (std::max)(0, static_cast<int>(centerCell.GetY()) - static_cast<int>(map.radiusCells));
        const int maxY = (std::min)(15, static_cast<int>(centerCell.GetY()) + static_cast<int>(map.radiusCells));

        for (int x = minX; x <= maxX; ++x)
        {
            for (int y = minY; y <= maxY; ++y)
            {
                const CellCoordinates cell(static_cast<uint8_t>(x), static_cast<uint8_t>(y));
                const Cell& mazeCell = (*map.maze)[cell];
                testUniqueWall(rayOrigin, rayDirection, cell, Direction::Up, mazeCell.GetUp(), map, best);
                testUniqueWall(rayOrigin, rayDirection, cell, Direction::Right, mazeCell.GetRight(), map, best);
                if (y == 0)
                {
                    testUniqueWall(rayOrigin, rayDirection, cell, Direction::Down, mazeCell.GetDown(), map, best);
                }
                if (x == 0)
                {
                    testUniqueWall(rayOrigin, rayDirection, cell, Direction::Left, mazeCell.GetLeft(), map, best);
                }
            }
        }

        for (int gridX = minX; gridX <= maxX + 1; ++gridX)
        {
            for (int gridY = minY; gridY <= maxY + 1; ++gridY)
            {
                const float gridXM = static_cast<float>(gridX) * map.cellSizeM;
                const float gridYM = static_cast<float>(gridY) * map.cellSizeM;
                const Eigen::Vector2f minCorner(gridXM - map.postHalfWidthM, gridYM - map.postHalfWidthM);
                const Eigen::Vector2f maxCorner(gridXM + map.postHalfWidthM, gridYM + map.postHalfWidthM);
                float rangeM = 0.0f;
                if (!intersectRayAabb(rayOrigin, rayDirection, minCorner, maxCorner, rangeM))
                {
                    continue;
                }
                if (!(rangeM > 0.0f) || !(rangeM < best.rangeM))
                {
                    continue;
                }

                best.hit = true;
                best.type = GeometryHitType::Post;
                best.rangeM = rangeM;
                best.pointWorldM = rayOrigin + (rayDirection * rangeM);
                best.postGridX = static_cast<uint8_t>(gridX);
                best.postGridY = static_cast<uint8_t>(gridY);
                best.edge = Direction::None;
            }
        }

        return best;
    }

    inline bool MapEvidenceUpdater::isOrdinal(Direction direction) noexcept
    {
        return
            direction == Direction::Up ||
            direction == Direction::Down ||
            direction == Direction::Left ||
            direction == Direction::Right;
    }

    inline size_t MapEvidenceUpdater::directionIndex(Direction direction) noexcept
    {
        switch (direction)
        {
        case Direction::Up:
            return 0U;
        case Direction::Down:
            return 1U;
        case Direction::Left:
            return 2U;
        case Direction::Right:
        default:
            return 3U;
        }
    }

    inline int8_t MapEvidenceUpdater::saturatingAdd(int8_t current, int8_t delta, int8_t limit) noexcept
    {
        const int candidate = static_cast<int>(current) + static_cast<int>(delta);
        if (candidate > limit)
        {
            return limit;
        }
        if (candidate < -limit)
        {
            return static_cast<int8_t>(-limit);
        }
        return static_cast<int8_t>(candidate);
    }

    inline WallState MapEvidenceUpdater::stateFromScore(int8_t score, const MapEvidenceUpdaterConfig& config) noexcept
    {
        if (score >= config.wallThreshold)
        {
            return WallState::Wall;
        }
        if (score <= -config.openThreshold)
        {
            return WallState::NoWall;
        }
        return WallState::Unknown;
    }

    inline void MapEvidenceUpdater::setMirrored(
        const CellCoordinates& cell,
        Direction direction,
        const EdgeEvidence& evidence) noexcept
    {
        _edges[cell.GetX()][cell.GetY()][directionIndex(direction)] = evidence;
        if (cell.IsValidMove(direction))
        {
            const CellCoordinates neighbor = cell >> direction;
            _edges[neighbor.GetX()][neighbor.GetY()][directionIndex(-direction)] = evidence;
        }
    }

    inline bool MapEvidenceUpdater::Apply(
        const CellCoordinates& cell,
        Direction direction,
        const WallObs& observation,
        const GeometryPrediction& bestFit,
        const MapEvidenceUpdaterConfig& config,
        bool freezeMutation) noexcept
    {
        if (freezeMutation ||
            !observation.valid ||
            (observation.confidence < config.minimumConfidence) ||
            !isOrdinal(direction))
        {
            return false;
        }

        if ((observation.cls == ObsClass::Ambiguous) || (observation.cls == ObsClass::PostLike))
        {
            return false;
        }

        CellCoordinates targetCell = cell;
        Direction targetDirection = direction;
        if ((bestFit.type == GeometryHitType::WallFace) && isOrdinal(bestFit.edge))
        {
            targetCell = bestFit.cell;
            targetDirection = bestFit.edge;
        }

        EdgeEvidence updated = Get(targetCell, targetDirection);
        if (observation.cls == ObsClass::WallLike)
        {
            updated.score = saturatingAdd(updated.score, config.wallHitWeight, config.maxScore);
        }
        else if (observation.cls == ObsClass::OpenLike && bestFit.type != GeometryHitType::Post)
        {
            updated.score = saturatingAdd(updated.score, static_cast<int8_t>(-config.openHitWeight), config.maxScore);
        }
        else
        {
            return false;
        }

        updated.state = stateFromScore(updated.score, config);
        setMirrored(targetCell, targetDirection, updated);
        return true;
    }

    inline SrUkfCore::SrUkfCore(
        const PlantParams& params,
        const PlantModel& plantModel) noexcept
        : _plantModel(plantModel)
        , _geometryModel()
        , _params(params)
        , _filter()
        , _lastControl()
        , _lastEncoderObs()
        , _haveEncoderReference(false)
        , _sqrtProcessNoiseDensity(StateMatrix::Zero())
        , _sqrtEncoderNoise(Eigen::Matrix<float, 2, 2>::Identity())
        , _sqrtImuNoise(Eigen::Matrix<float, 3, 3>::Identity())
        , _sqrtFrontNoise(Eigen::Matrix<float, 2, 2>::Identity())
        , _sqrtSideNoise(Eigen::Matrix<float, 1, 1>::Identity())
    {
        _filter.setStateNormalizer(&VehicleState::NormalizeStateVector);

        StateVector initialState = StateVector::Zero();
        StateMatrix initialCovariance = StateMatrix::Identity() * 1.0e-3f;
        initialCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = 0.25f;
        initialCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = 0.25f;
        initialCovariance(VehicleState::kBgz, VehicleState::kBgz) = 0.01f;
        _filter.setState(initialState, initialCovariance);

        StateMatrix processNoise = StateMatrix::Zero();
        processNoise.diagonal() <<
            0.0f,
            0.0f,
            0.0f,
            3.0e-4f,
            3.0e-4f,
            6.0e-4f,
            4.0e-2f,
            4.0e-2f,
            2.0e-5f;
        _sqrtProcessNoiseDensity = StateMatrix::Zero();
        _sqrtProcessNoiseDensity.diagonal() = processNoise.diagonal().cwiseSqrt();
        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity);

        _sqrtEncoderNoise = ComputeGeneralEncoderPairSqrtNoise(_params);
        _sqrtImuNoise(0, 0) = kImuYawRateSigmaRadps;
        _sqrtImuNoise(1, 1) = kImuAccelSigmaMps2;
        _sqrtImuNoise(2, 2) = kImuAccelSigmaMps2;
        _sqrtFrontNoise(0, 0) = 0.010f;
        _sqrtFrontNoise(1, 1) = 0.010f;
        _sqrtSideNoise(0, 0) = 0.012f;
    }

    inline bool SrUkfCore::predict(float dt, const ControlInput& control) noexcept
    {
        return predict(dt, control, MazeMap::NoopUkfLoopHook{});
    }

    template <typename LoopHook>
    inline bool SrUkfCore::predict(float dt, const ControlInput& control, LoopHook&& loopHook) noexcept
    {
        _lastControl = control;
        if (!(std::isfinite(dt) && (dt > 0.0f)))
        {
            return true;
        }

        _filter.setProcessNoiseSquareRoot(_sqrtProcessNoiseDensity * MazeMap::Math::Sqrtf(dt));
        Eigen::Matrix<float, 3, 1> controlVector;
        controlVector << control.leftMotorCommand, control.rightMotorCommand, control.fanDutyCycle;
        return _filter.Predict(
            dt,
            controlVector,
            [this, &control](const StateVector& sigmaPoint, const Eigen::Matrix<float, 3, 1>&, float sigmaDt) noexcept
            {
                return _plantModel.integrateMidpoint(sigmaPoint, control, sigmaDt, _params);
            },
            loopHook);
    }

    inline MeasurementUpdateResult SrUkfCore::updateEncoderPair(const EncoderObs& observation, float dt) noexcept
    {
        return updateEncoderPair(observation, dt, MazeMap::NoopUkfLoopHook{});
    }

    template <typename LoopHook>
    inline MeasurementUpdateResult SrUkfCore::updateEncoderPair(
        const EncoderObs& observation,
        float dt,
        LoopHook&& loopHook) noexcept
    {
        MeasurementUpdateResult result{};
        result.attempted = true;

        EncoderObs measured = observation;
        if (_haveEncoderReference && (dt > 0.0f))
        {
            const int32_t deltaLeftCounts = observation.totalLeftCounts - _lastEncoderObs.totalLeftCounts;
            const int32_t deltaRightCounts = observation.totalRightCounts - _lastEncoderObs.totalRightCounts;
            const float motorToWheel = 1.0f / _params.gearRatio;
            const float deltaThetaMotorLeft =
                (2.0f * PI_F * static_cast<float>(deltaLeftCounts)) /
                static_cast<float>(_params.encoderCountsPerMotorRev);
            const float deltaThetaMotorRight =
                (2.0f * PI_F * static_cast<float>(deltaRightCounts)) /
                static_cast<float>(_params.encoderCountsPerMotorRev);
            measured.omegaLeftRadps = (deltaThetaMotorLeft * motorToWheel) / dt;
            measured.omegaRightRadps = (deltaThetaMotorRight * motorToWheel) / dt;
        }

        _lastEncoderObs = observation;
        _haveEncoderReference = true;

        Eigen::Matrix<float, 2, 1> z;
        z << measured.omegaLeftRadps, measured.omegaRightRadps;
        if (HasExactZeroWheelObservation(measured))
        {
            const float stationarySigmaRadps = ComputeStationaryEncoderOmegaSigmaRadps(_params);
            applyWheelRateConstraint(measured, stationarySigmaRadps * stationarySigmaRadps);
            result.accepted = true;
            result.nis = 0.0f;
            return result;
        }

        const Eigen::Matrix<float, 2, 2> sqrtEncoderNoise = ComputeEncoderPairSqrtNoise(measured, _params);
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
            loopHook);
        if (result.accepted)
        {
            StateVector anchoredState = _filter.state();
            anchoredState(VehicleState::kOmegaL) = measured.omegaLeftRadps;
            anchoredState(VehicleState::kOmegaR) = measured.omegaRightRadps;
            VehicleState::NormalizeStateVector(anchoredState);
            _filter.setState(anchoredState, _filter.covariance());
        }
        result.nis = _filter.lastNis();
        return result;
    }

    inline MeasurementUpdateResult SrUkfCore::updateImuMerged(const ImuMergedObs& observation) noexcept
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

    inline MeasurementUpdateResult SrUkfCore::updateYawRate(float yawRateRadps) noexcept
    {
        return updateYawRate(yawRateRadps, MazeMap::NoopUkfLoopHook{});
    }

    template <typename LoopHook>
    inline MeasurementUpdateResult SrUkfCore::updateYawRate(float yawRateRadps, LoopHook&& loopHook) noexcept
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
            loopHook);
        if (result.accepted)
        {
            if (controlCommandsAreEffectivelyZero() &&
                HasExactZeroWheelObservation(_lastEncoderObs))
            {
                applyStationaryZeroMotionConstraint(yawRateRadps);
            }
        }
        result.nis = _filter.lastNis();
        return result;
    }

    inline MeasurementUpdateResult SrUkfCore::updatePlanarAccel(const ImuAccelObs& observation) noexcept
    {
        return updatePlanarAccel(observation, MazeMap::NoopUkfLoopHook{});
    }

    template <typename LoopHook>
    inline MeasurementUpdateResult SrUkfCore::updatePlanarAccel(
        const ImuAccelObs& observation,
        LoopHook&& loopHook) noexcept
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
        result.accepted = _filter.Update<2>(
            z,
            sqrtNoise,
            9.21034f,
            [this](const StateVector& sigmaPoint) noexcept
            {
                return accelPredictionForState(sigmaPoint);
            },
            loopHook);
        result.nis = _filter.lastNis();
        return result;
    }

    inline bool SrUkfCore::HasExactZeroWheelObservation(const EncoderObs& observation) noexcept
    {
        return (observation.omegaLeftRadps == 0.0f) && (observation.omegaRightRadps == 0.0f);
    }

    inline bool SrUkfCore::controlCommandsAreEffectivelyZero() const noexcept
    {
        return
            (std::fabs(_lastControl.leftMotorCommand) <= 1.0e-6f) &&
            (std::fabs(_lastControl.rightMotorCommand) <= 1.0e-6f);
    }

    inline void SrUkfCore::applyWheelRateConstraint(const EncoderObs& measured, float wheelVarianceRadps2) noexcept
    {
        StateVector anchoredState = _filter.state();
        anchoredState(VehicleState::kOmegaL) = measured.omegaLeftRadps;
        anchoredState(VehicleState::kOmegaR) = measured.omegaRightRadps;
        VehicleState::NormalizeStateVector(anchoredState);

        StateMatrix anchoredCovariance = _filter.covariance();
        anchoredCovariance.row(VehicleState::kOmegaL).setZero();
        anchoredCovariance.col(VehicleState::kOmegaL).setZero();
        anchoredCovariance.row(VehicleState::kOmegaR).setZero();
        anchoredCovariance.col(VehicleState::kOmegaR).setZero();
        const float constrainedVariance = (std::max)(wheelVarianceRadps2, 1.0e-12f);
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = constrainedVariance;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = constrainedVariance;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    inline void SrUkfCore::applyStationaryZeroMotionConstraint(float yawRateRadps) noexcept
    {
        StateVector anchoredState = _filter.state();
        anchoredState(VehicleState::kU) = 0.0f;
        anchoredState(VehicleState::kV) = 0.0f;
        anchoredState(VehicleState::kR) = 0.0f;
        anchoredState(VehicleState::kOmegaL) = 0.0f;
        anchoredState(VehicleState::kOmegaR) = 0.0f;
        anchoredState(VehicleState::kBgz) = yawRateRadps;
        VehicleState::NormalizeStateVector(anchoredState);

        StateMatrix anchoredCovariance = _filter.covariance();
        const float wheelVarianceRadps2 =
            (std::max)(ComputeStationaryEncoderOmegaSigmaRadps(_params) * ComputeStationaryEncoderOmegaSigmaRadps(_params), 1.0e-12f);
        const float linearVarianceMps2 =
            (std::max)(kStationaryEncoderVelocitySigmaMps * kStationaryEncoderVelocitySigmaMps, 1.0e-12f);
        const float yawVarianceRadps2 =
            (std::max)(kImuYawRateSigmaRadps * kImuYawRateSigmaRadps, 1.0e-12f);
        const float gyroBiasVarianceRadps2 = (std::max)(yawVarianceRadps2, 1.0e-8f);

        const std::array<int, 5> constrainedIndices = {
            VehicleState::kU,
            VehicleState::kV,
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

        anchoredCovariance(VehicleState::kU, VehicleState::kU) = linearVarianceMps2;
        anchoredCovariance(VehicleState::kV, VehicleState::kV) = linearVarianceMps2;
        anchoredCovariance(VehicleState::kR, VehicleState::kR) = yawVarianceRadps2;
        anchoredCovariance(VehicleState::kOmegaL, VehicleState::kOmegaL) = wheelVarianceRadps2;
        anchoredCovariance(VehicleState::kOmegaR, VehicleState::kOmegaR) = wheelVarianceRadps2;
        anchoredCovariance(VehicleState::kBgz, VehicleState::kBgz) = gyroBiasVarianceRadps2;
        _filter.setState(anchoredState, anchoredCovariance);
    }

    inline float SrUkfCore::wallNoiseFromConfidence(float confidence, float minimumNoise) noexcept
    {
        const float normalizedConfidence = (std::clamp)(confidence, 0.0f, 1.0f);
        return minimumNoise + ((1.0f - normalizedConfidence) * 0.020f);
    }

    inline Eigen::Matrix<float, 2, 1> SrUkfCore::accelPredictionForState(const StateVector& sigmaPoint) const noexcept
    {
        Eigen::Matrix<float, 2, 1> prediction{};
        const Eigen::Vector2f accel = _plantModel.imuPlanarAcceleration(sigmaPoint, _lastControl, _params);
        prediction << accel.x(), accel.y();
        return prediction;
    }

    inline float SrUkfCore::wallPredictionForSensor(
        const StateVector& sigmaPoint,
        const SensorExtrinsics& sensor,
        const LocalMapView& map) const noexcept
    {
        const WallGeometryModel::GeometryStateFrame frame = _geometryModel.buildStateFrame(sigmaPoint, map);
        const GeometryPrediction prediction = _geometryModel.predictRay(frame, sensor, map);
        return prediction.hit ? prediction.rangeM : map.noHitRangeM;
    }

    inline Eigen::Matrix<float, 2, 1> SrUkfCore::frontPairPredictionForState(
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

    inline FrontPairUpdateResult SrUkfCore::updateFrontPair(
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

    inline WallUpdateResult SrUkfCore::updateSideSensor(
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

    inline MouseUkfFacade::MouseUkfFacade(const PlantParams& params) noexcept
        : _params(params)
        , _core(params)
        , _mapEvidence()
    {
    }

    inline Direction MouseUkfFacade::dominantDirectionForSensor(
        const SensorExtrinsics& sensor,
        const VehicleState::StateVector& state) noexcept
    {
        const Eigen::Vector2f heading = HeadingUnitFromYaw(state(VehicleState::kPsi));
        const Eigen::Vector2f directionWorld = RotateBodyVectorToWorld(ResolveSensorDirectionBody(sensor), heading);
        const float x = directionWorld.x();
        const float y = directionWorld.y();
        if (std::fabs(x) >= std::fabs(y))
        {
            return (x >= 0.0f) ? Direction::Right : Direction::Left;
        }
        return (y >= 0.0f) ? Direction::Up : Direction::Down;
    }

    inline CellCoordinates MouseUkfFacade::estimateSensorCell(
        const SensorExtrinsics& sensor,
        const VehicleState::StateVector& state,
        float cellSizeM) noexcept
    {
        const Eigen::Vector2f heading = HeadingUnitFromYaw(state(VehicleState::kPsi));
        const Eigen::Vector2f sensorPositionWorld =
            Eigen::Vector2f(state(VehicleState::kPx), state(VehicleState::kPy)) +
            RotateBodyVectorToWorld(sensor.positionBodyM, heading);
        const float x = sensorPositionWorld.x();
        const float y = sensorPositionWorld.y();
        const float safeCellSize = (cellSizeM > 0.0f) ? cellSizeM : 0.18f;
        int cellX = static_cast<int>(std::floor(x / safeCellSize));
        int cellY = static_cast<int>(std::floor(y / safeCellSize));
        cellX = (std::clamp)(cellX, 0, 15);
        cellY = (std::clamp)(cellY, 0, 15);
        return CellCoordinates(static_cast<uint8_t>(cellX), static_cast<uint8_t>(cellY));
    }

    inline FrontPairUpdateResult MouseUkfFacade::updateFrontPair(
        const WallObs& left,
        const WallObs& right,
        const LocalMapView& map,
        const MapEvidenceUpdaterConfig& evidenceConfig) noexcept
    {
        FrontPairUpdateResult result = _core.updateFrontPair(left, right, map);
        if (result.filter.accepted && !map.freezeMapMutation)
        {
            const Direction leftDirection = dominantDirectionForSensor(_params.frontLeftSensor, _core.state());
            const Direction rightDirection = dominantDirectionForSensor(_params.frontRightSensor, _core.state());
            const CellCoordinates leftCell = estimateSensorCell(_params.frontLeftSensor, _core.state(), map.cellSizeM);
            const CellCoordinates rightCell = estimateSensorCell(_params.frontRightSensor, _core.state(), map.cellSizeM);
            _mapEvidence.Apply(leftCell, leftDirection, left, result.leftPrediction, evidenceConfig, map.freezeMapMutation);
            _mapEvidence.Apply(rightCell, rightDirection, right, result.rightPrediction, evidenceConfig, map.freezeMapMutation);
        }
        return result;
    }

    inline WallUpdateResult MouseUkfFacade::updateSideSensor(
        Side which,
        const WallObs& observation,
        const LocalMapView& map,
        const MapEvidenceUpdaterConfig& evidenceConfig) noexcept
    {
        WallUpdateResult result = _core.updateSideSensor(which, observation, map);
        if (result.filter.accepted && !map.freezeMapMutation)
        {
            const SensorExtrinsics& sensor =
                (which == Side::Left) ? _params.sideLeftSensor : _params.sideRightSensor;
            const Direction direction = dominantDirectionForSensor(sensor, _core.state());
            const CellCoordinates cell = estimateSensorCell(sensor, _core.state(), map.cellSizeM);
            _mapEvidence.Apply(cell, direction, observation, result.prediction, evidenceConfig, map.freezeMapMutation);
        }
        return result;
    }
}
